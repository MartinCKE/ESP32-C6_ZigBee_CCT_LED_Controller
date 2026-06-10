#include "status_led.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "STATUS_LED";

#define STATUS_LED_TASK_STACK_DEFAULT     (2048)
#define STATUS_LED_TASK_PRIORITY_DEFAULT  (5)
#define STATUS_LED_TASK_NAME_DEFAULT      "status_led"

#define LOOP_MS                          (20)

// Effects timing
#define BREATHE_PERIOD_JOINING_MS        (2000)  // 1 Hz breathe
#define BREATHE_PERIOD_NORMAL_MS         (5000)  // 1 Hz breathe
#define BOOT_BLINK_ON_MS                 (150)
#define BOOT_BLINK_OFF_MS                (150)

#define JOINED_FLASH_ON_MS               (150)
#define JOINED_FLASH_OFF_MS              (150)
#define JOINED_FLASH_COUNT               (3)

#define BUTTON_PULSE_MS                  (120)

#define BOOT_OK_TOTAL_MS          (3000)
#define BOOT_OK_CYCLES            (10)

#define BOOT_OK_STEP_COUNT        (BOOT_OK_CYCLES * 4) // R->Y->G->Y = 4 steps
#define BOOT_OK_STEP_MS           (BOOT_OK_TOTAL_MS / BOOT_OK_STEP_COUNT)

#define OTA_SUCCESS_FLASH_ON_MS   (120)
#define OTA_SUCCESS_FLASH_OFF_MS  (120)
#define OTA_SUCCESS_FLASH_COUNT   (6)

typedef struct {
    status_led_config_t cfg;
    SemaphoreHandle_t lock;

    status_led_state_t state;

    // Overlay pulse (green)
    TickType_t button_pulse_until;

    // Sequences
    bool seq_active;
    uint32_t seq_step;
    TickType_t seq_next_change;

    // Breathe phase
    uint32_t breathe_phase_ms;

    bool boot_anim_active;
    uint32_t boot_anim_step;
    TickType_t boot_anim_next_change;

    bool ota_success_active;
    uint32_t ota_success_step;
    TickType_t ota_success_next_change;

    // LEDC
    bool ledc_ready;
    ledc_channel_t ch_r;
    ledc_channel_t ch_y;
    ledc_channel_t ch_g;

    TaskHandle_t task;
} ctx_t;

static ctx_t s;

static uint8_t breathe_next(uint32_t BREATHE_PERIOD_MS)
{
    s.breathe_phase_ms += LOOP_MS;
    if (s.breathe_phase_ms >= BREATHE_PERIOD_MS) {
        s.breathe_phase_ms = 0;
    }

    const uint32_t half = BREATHE_PERIOD_MS / 2;
    uint32_t x = s.breathe_phase_ms;
    uint32_t v;

    if (x < half) v = (x * 255U) / half;
    else          v = ((BREATHE_PERIOD_MS - x) * 255U) / half;

    if (v > 255U) v = 255U;
    return (uint8_t)v;
}

void status_led_boot_ok_start(void)
{
    if (!s.lock) return;

    xSemaphoreTake(s.lock, portMAX_DELAY);

    s.boot_anim_active = true;
    s.boot_anim_step = 0;
    s.boot_anim_next_change = xTaskGetTickCount(); // start immediately

    xSemaphoreGive(s.lock);
}

void status_led_ota_success_start(void)
{
    if (!s.lock) return;

    xSemaphoreTake(s.lock, portMAX_DELAY);

    s.ota_success_active = true;
    s.ota_success_step = 0;
    s.ota_success_next_change = xTaskGetTickCount();

    xSemaphoreGive(s.lock);
}


static void seq_reset(void)
{
    s.seq_active = false;
    s.seq_step = 0;
    s.seq_next_change = 0;
}

static void seq_start_boot_ok(void)
{
    s.seq_active = true;
    s.seq_step = 0;
    s.seq_next_change = xTaskGetTickCount();
    s.breathe_phase_ms = 0; // reuse as global time accumulator
}
static void seq_start_joined_ok(void)
{
    s.seq_active = true;
    s.seq_step = 0;
    s.seq_next_change = xTaskGetTickCount(); // start now
}

static void ledc_set_u8(ledc_channel_t ch, uint8_t v)
{
    if (s.cfg.active_low) {
        v = (uint8_t)(255 - v);
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, v);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

static void leds_set_u8(uint8_t g, uint8_t y, uint8_t r)
{
    if (!s.ledc_ready) return;
    ledc_set_u8(s.ch_g, g);
    ledc_set_u8(s.ch_y, y);
    ledc_set_u8(s.ch_r, r);
}

static esp_err_t ledc_init(void)
{
    // 8-bit resolution => duty 0..255
    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .freq_hz          = 5000, 
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(err));
        return err;
    }

    s.ch_g = LEDC_CHANNEL_0;
    s.ch_y = LEDC_CHANNEL_1;
    s.ch_r = LEDC_CHANNEL_2;

    ledc_channel_config_t c = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
        .intr_type  = LEDC_INTR_DISABLE,
    };

    ledc_channel_config_t cg = c;
    cg.channel  = s.ch_g;
    cg.gpio_num = s.cfg.gpio_green;

    ledc_channel_config_t cy = c;
    cy.channel  = s.ch_y;
    cy.gpio_num = s.cfg.gpio_yellow;

    ledc_channel_config_t cr = c;
    cr.channel  = s.ch_r;
    cr.gpio_num = s.cfg.gpio_red;

    err = ledc_channel_config(&cg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config green failed: %s", esp_err_to_name(err));
        return err;
    }

    err = ledc_channel_config(&cy);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config yellow failed: %s", esp_err_to_name(err));
        return err;
    }

    err = ledc_channel_config(&cr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config red failed: %s", esp_err_to_name(err));
        return err;
    }

    // All off initially
    leds_set_u8(0, 0, 0);

    return ESP_OK;
}

static bool render_boot_ok_anim(uint8_t *g, uint8_t *y, uint8_t *r)
{
    if (!s.boot_anim_active) return false;

    TickType_t now = xTaskGetTickCount();

    if (now >= s.boot_anim_next_change) {
        s.boot_anim_step++;
        s.boot_anim_next_change = now + pdMS_TO_TICKS(BOOT_OK_STEP_MS);

        if (s.boot_anim_step >= BOOT_OK_STEP_COUNT) {
            s.boot_anim_active = false;
            *g = *y = *r = 0;
            return true; // consumed rendering, animation just ended
        }
    }

    float progress = (float)s.boot_anim_step / (float)BOOT_OK_STEP_COUNT;
    if (progress < 0.0f) progress = 0.0f;
    if (progress > 1.0f) progress = 1.0f;

    uint8_t brightness = (uint8_t)(progress * 255.0f);

    uint32_t phase = s.boot_anim_step % 4; // R,Y,G,Y

    *g = *y = *r = 0;
    switch (phase) {
    case 0: *r = brightness; break;
    case 1: *y = brightness; break;
    case 2: *g = brightness; break;
    case 3: *y = brightness; break;
    }

    return true; // animation is active and rendered
}

static bool render_ota_success_anim(uint8_t *g, uint8_t *y, uint8_t *r)
{
    if (!s.ota_success_active) return false;

    TickType_t now = xTaskGetTickCount();
    const uint32_t total_steps = OTA_SUCCESS_FLASH_COUNT * 2;

    if (now >= s.ota_success_next_change) {
        bool on_phase = (s.ota_success_step % 2 == 0);
        s.ota_success_next_change = now + pdMS_TO_TICKS(on_phase ? OTA_SUCCESS_FLASH_ON_MS : OTA_SUCCESS_FLASH_OFF_MS);
        s.ota_success_step++;

        if (s.ota_success_step > total_steps) {
            s.ota_success_active = false;
            *g = *y = *r = 0;
            return true;
        }
    }

    *g = *y = *r = 0;
    if (s.ota_success_step > 0 && ((s.ota_success_step - 1) % 2 == 0)) {
        *g = 255;
        *y = 255;
    }

    return true;
}


static void render_u8(uint8_t *g, uint8_t *y, uint8_t *r)
{
    if (render_ota_success_anim(g, y, r)) {
        return;
    }
    if (render_boot_ok_anim(g, y, r)) {
        return;
    }
    *g = 0; *y = 0; *r = 0;

    TickType_t now = xTaskGetTickCount();
    //ESP_LOGI(TAG, "Render state %d (seq_active=%d step=%d)", (int)s.state, (int)s.seq_active, (int)s.seq_step);
    // Sequences first
    if (s.seq_active) {
        if (s.state == STATUS_LED_STATE_BOOT_OK) {
            // One green blink: ON -> OFF -> done
            if (now >= s.seq_next_change) {
                if (s.seq_step == 0) {
                    s.seq_step = 1;
                    s.seq_next_change = now + pdMS_TO_TICKS(BOOT_BLINK_ON_MS);
                } else if (s.seq_step == 1) {
                    s.seq_step = 2;
                    s.seq_next_change = now + pdMS_TO_TICKS(BOOT_BLINK_OFF_MS);
                } else {
                    seq_reset();
                }
            }
            if (s.seq_step == 1) *g = 255;
            return;
        }

        if (s.state == STATUS_LED_STATE_JOINED_SUCCESSFULLY) {
            // Flash yellow 3 times then done
            uint32_t total_transitions = JOINED_FLASH_COUNT * 2; // ON,OFF per flash
            ESP_LOGD(TAG, "Joined sequence step %d/%d", (int)s.seq_step, (int)total_transitions);

            if (now >= s.seq_next_change) {
                if (s.seq_step < total_transitions) {
                    bool on_phase = (s.seq_step % 2 == 0);
                    s.seq_next_change = now + pdMS_TO_TICKS(on_phase ? JOINED_FLASH_ON_MS : JOINED_FLASH_OFF_MS);
                    s.seq_step++;
                } else {
                    seq_reset();
                }
            }

            if (s.seq_step > 0) {
                bool last_phase_was_on = ((s.seq_step - 1) % 2 == 0);
                if (last_phase_was_on) *y = 255;
            }
            return;
        }
    }

    // Normal states (your table)
    switch (s.state) {
    case STATUS_LED_STATE_JOINING_NETWORK:
        *y = breathe_next(BREATHE_PERIOD_JOINING_MS); // yellow breathe 1 Hz
        break;

    case STATUS_LED_STATE_NORMAL_OPERATION:
        *g = breathe_next(BREATHE_PERIOD_NORMAL_MS); // green breathe 1 Hz
        break;

    case STATUS_LED_STATE_WAKEUP_SEQUENCE:
        // all off
        break;

    case STATUS_LED_STATE_SENSOR_ERROR:
        *r = 255; // solid red
        break;

    case STATUS_LED_STATE_ZIGBEE_LOST:
        *r = breathe_next(BREATHE_PERIOD_NORMAL_MS); // red breathe
        break;

    case STATUS_LED_STATE_BOOT_OK:
    case STATUS_LED_STATE_JOINED_SUCCESSFULLY:
    default:
        // if sequence finished, keep off
        break;
    }

    // Overlay: button press -> green short blink
    if (s.button_pulse_until != 0 && now < s.button_pulse_until) {
        *y = 255;
    }
}

static void status_led_task(void *arg)
{
    (void)arg;

    while (1) {
        uint8_t g, y, r;

        xSemaphoreTake(s.lock, portMAX_DELAY);
        render_u8(&g, &y, &r);
        xSemaphoreGive(s.lock);

        leds_set_u8(g, y, r);
        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}

void status_led_start(const status_led_config_t *cfg)
{
    if (!cfg) {
        ESP_LOGE(TAG, "cfg is NULL");
        return;
    }
    if (s.task) {
        ESP_LOGD(TAG, "Already started");
        return;
    }

    s.cfg = *cfg;
    s.lock = xSemaphoreCreateMutex();
    s.state = STATUS_LED_STATE_BOOT_OK;

    s.button_pulse_until = 0;
    s.breathe_phase_ms = 0;
    seq_reset();

    esp_err_t err = ledc_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LEDC init failed, status LEDs disabled");
        return;
    }
    s.ledc_ready = true;

    uint32_t stack = (cfg->task_stack_bytes != 0) ? cfg->task_stack_bytes : STATUS_LED_TASK_STACK_DEFAULT;
    UBaseType_t prio = (cfg->task_priority != 0) ? cfg->task_priority : STATUS_LED_TASK_PRIORITY_DEFAULT;
    const char *name = (cfg->task_name != NULL) ? cfg->task_name : STATUS_LED_TASK_NAME_DEFAULT;

    xTaskCreate(status_led_task, name, stack / sizeof(StackType_t), NULL, prio, &s.task);
}

void status_led_set_state(status_led_state_t state)
{
    if (!s.lock) return;

    xSemaphoreTake(s.lock, portMAX_DELAY);

    if (s.state != state) {
        s.state = state;

        // Keep breathe phase continuous (prevents visible "restart" artifacts)
        // s.breathe_phase_ms = 0;

        // Start sequences for these states
        if (state == STATUS_LED_STATE_BOOT_OK) {
            seq_start_boot_ok();
        } else if (state == STATUS_LED_STATE_JOINED_SUCCESSFULLY) {
            seq_start_joined_ok();
        } else {
            seq_reset();
        }

        ESP_LOGD(TAG, "Set status LED state: %d", (int)state);
    }

    xSemaphoreGive(s.lock);
}

void status_led_notify_button_press(void)
{
    if (!s.lock) return;

    xSemaphoreTake(s.lock, portMAX_DELAY);
    s.button_pulse_until = xTaskGetTickCount() + pdMS_TO_TICKS(BUTTON_PULSE_MS);
    xSemaphoreGive(s.lock);
}
