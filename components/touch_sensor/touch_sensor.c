#include "touch_sensor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "TOUCH_SENSOR";

#define TOUCH_TASK_STACK_DEFAULT   (3072)
#define TOUCH_TASK_PRIO_DEFAULT    (6)
#define TOUCH_TASK_NAME_DEFAULT    "touch_sensor"

#define LOOP_TICK_MS               (10)  
#define QUEUE_LEN                  (16)

uint8_t  last_brightness;
uint16_t last_ct_mired;

typedef struct {
    touch_sensor_config_t cfg;

    SemaphoreHandle_t lock;
    QueueHandle_t q;
    TaskHandle_t task;

    bool started;

    // debounced stable state
    bool touched;

    // gesture state
    bool pressed;
    TickType_t press_tick;

    uint8_t tap_count;
    TickType_t first_tap_tick;

    // hold state
    bool hold_active;
    TickType_t last_hold_step_tick;
    int8_t hold_dir;            // +1 or -1 (current direction while holding)
    int8_t next_hold_dir;       // direction to use at start of NEXT hold (flips on release)

    // mode
    touch_hold_mode_t mode;

    // local cached values (used if getters not provided)
    bool     cached_power;
    uint8_t  cached_brightness;
    uint16_t cached_ct_mired;

    bool suppress_tap_once;
} ctx_t;

static ctx_t s;

static inline TickType_t now_tick(void) { return xTaskGetTickCount(); }
static inline uint32_t ticks_to_ms(TickType_t t) { return (uint32_t)(t * portTICK_PERIOD_MS); }
static inline TickType_t ms_to_ticks(uint32_t ms) { return pdMS_TO_TICKS(ms); }

static inline bool level_to_touched(int level)
{
    if (s.cfg.active_low) return (level == 0);
    return (level != 0);
}

static uint8_t clamp_u8(int v, uint8_t mn, uint8_t mx)
{
    if (v < (int)mn) return mn;
    if (v > (int)mx) return mx;
    return (uint8_t)v;
}

static uint16_t clamp_u16(int v, uint16_t mn, uint16_t mx)
{
    if (v < (int)mn) return mn;
    if (v > (int)mx) return mx;
    return (uint16_t)v;
}

static uint8_t get_brightness(void)
{
    if (s.cfg.actions.get_brightness) return s.cfg.actions.get_brightness(s.cfg.user_ctx);
    return s.cached_brightness;
}

static uint16_t get_ct(void)
{
    if (s.cfg.actions.get_ct_mired) return s.cfg.actions.get_ct_mired(s.cfg.user_ctx);
    return s.cached_ct_mired;
}

static void apply_toggle_power(void)
{
    if (s.cfg.actions.toggle_power) {
        s.cfg.actions.toggle_power(s.cfg.user_ctx);
    } else {
        ESP_LOGW(TAG, "toggle_power callback not set");
    }
}

static void apply_brightness(uint8_t v)
{
    s.cached_brightness = v;  
    if (s.cfg.actions.apply_brightness) {
        s.cfg.actions.apply_brightness(v, s.cfg.user_ctx);
    }
}


static void apply_ct(uint16_t m)
{
    s.cached_ct_mired = m;  
    if (s.cfg.actions.apply_ct_mired) {
        s.cfg.actions.apply_ct_mired(m, s.cfg.user_ctx);
    }
}



static void set_mode(touch_hold_mode_t mode)
{
    if (s.mode == mode) return;
    s.mode = mode;
    ESP_LOGI(TAG, "Hold mode: %s", (mode == TOUCH_HOLD_MODE_BRIGHTNESS) ? "BRIGHTNESS" : "CT");
    if (s.cfg.actions.mode_changed) {
        s.cfg.actions.mode_changed(mode, s.cfg.user_ctx);
    }
}

// ---------- ISR ----------
static void IRAM_ATTR gpio_isr(void *arg)
{
    (void)arg;
    int level = gpio_get_level((gpio_num_t)s.cfg.gpio_num);

    BaseType_t hp = pdFALSE;
    if (s.q) xQueueSendFromISR(s.q, &level, &hp);
    if (hp) portYIELD_FROM_ISR();
}

// ---------- Gesture actions ----------
static void on_press_old(void)
{
    s.pressed = true;
    s.press_tick = now_tick();
    s.hold_active = false;
    if (s.cfg.actions.user_interaction) {
        s.cfg.actions.user_interaction(s.cfg.user_ctx);
    }
    ESP_LOGI(TAG, "Touch PRESS");
}

static void on_press(void)
{
    s.pressed = true;
    s.press_tick = now_tick();
    s.hold_active = false;

    // default
    s.suppress_tap_once = false;

    // call optional user hook
    if (s.cfg.actions.user_interaction) {
        s.cfg.actions.user_interaction(s.cfg.user_ctx);

        // ✅ if user hook requested consuming this press, suppress tap
        if (s.cfg.user_ctx) {
            // match the struct in main.c
            typedef struct {
                volatile bool consume_next_tap;
            } touch_user_ctx_t;

            touch_user_ctx_t *t = (touch_user_ctx_t *)s.cfg.user_ctx;
            if (t->consume_next_tap) {
                t->consume_next_tap = false;   // clear it
                s.suppress_tap_once = true;
                s.tap_count = 0;               // cancel any pending single/double tap logic
            }
        }
    }

    ESP_LOGI(TAG, "Touch PRESS");
}
static void start_hold_if_needed(void)
{
    if (!s.pressed || s.hold_active) return;

    TickType_t elapsed = now_tick() - s.press_tick;
    if (elapsed >= ms_to_ticks(s.cfg.hold_threshold_ms)) {
        s.hold_active = true;
        s.last_hold_step_tick = now_tick();
        s.hold_dir = s.next_hold_dir;
        ESP_LOGI(TAG, "Hold START (mode=%s, dir=%s)",
                 (s.mode == TOUCH_HOLD_MODE_BRIGHTNESS) ? "BRIGHTNESS" : "CT",
                 (s.hold_dir > 0) ? "UP" : "DOWN");
    }
}

static void hold_step(void)
{
    if (!s.hold_active || !s.pressed) return;

    TickType_t now = now_tick();

    if (s.mode == TOUCH_HOLD_MODE_BRIGHTNESS) {
        if ((now - s.last_hold_step_tick) < ms_to_ticks(s.cfg.brightness_step_ms)) return;
        s.last_hold_step_tick = now;

        int v = (int)get_brightness() + (int)s.hold_dir * (int)s.cfg.brightness_step;
        uint8_t clamped = clamp_u8(v, s.cfg.brightness_min, s.cfg.brightness_max);

        // bounce at ends
        if (clamped == s.cfg.brightness_max && s.hold_dir > 0) s.hold_dir = -1;
        if (clamped == s.cfg.brightness_min && s.hold_dir < 0) s.hold_dir = +1;

        apply_brightness(clamped);
        return;
    }

    // CT mode
    if ((now - s.last_hold_step_tick) < ms_to_ticks(s.cfg.ct_step_ms)) return;
    s.last_hold_step_tick = now;

    int v = (int)get_ct() + (int)s.hold_dir * (int)s.cfg.ct_step;
    uint16_t clamped = clamp_u16(v, s.cfg.ct_min_mired, s.cfg.ct_max_mired);

    // bounce at ends
    if (clamped == s.cfg.ct_max_mired && s.hold_dir > 0) s.hold_dir = -1;
    if (clamped == s.cfg.ct_min_mired && s.hold_dir < 0) s.hold_dir = +1;

    apply_ct(clamped);
}

static void on_release(void)
{
    bool was_hold = s.hold_active;

    s.pressed = false;
    s.hold_active = false;

    if (was_hold) {
        // flip direction for next hold (your rule)
        s.next_hold_dir = (s.next_hold_dir > 0) ? -1 : +1;

        // Commit once
        if (s.mode == TOUCH_HOLD_MODE_BRIGHTNESS) {
            if (s.cfg.actions.commit_brightness)
                s.cfg.actions.commit_brightness(s.cached_brightness, s.cfg.user_ctx);
        } else {
            if (s.cfg.actions.commit_ct_mired)
                s.cfg.actions.commit_ct_mired(s.cached_ct_mired, s.cfg.user_ctx);
        }
    return;
    }

    if (s.suppress_tap_once) {
        s.suppress_tap_once = false;
        s.tap_count = 0;
        ESP_LOGI(TAG, "Tap suppressed");
        return;
    }

    // tap candidate
    TickType_t now = now_tick();

    if (s.tap_count == 0) {
        s.tap_count = 1;
        s.first_tap_tick = now;
        // do NOT toggle immediately; wait to see if a second tap arrives
        return;
    }

    // second tap within window -> double tap
    if (s.tap_count == 1) {
        TickType_t dt = now - s.first_tap_tick;
        if (dt <= ms_to_ticks(s.cfg.double_tap_window_ms)) {
            s.tap_count = 0;
            // double tap action: toggle hold mode
            set_mode((s.mode == TOUCH_HOLD_MODE_BRIGHTNESS) ? TOUCH_HOLD_MODE_CT : TOUCH_HOLD_MODE_BRIGHTNESS);
            ESP_LOGI(TAG, "Double TAP");
            return;
        } else {
            // too late: treat previous as single tap, start new window for this one
            s.tap_count = 1;
            s.first_tap_tick = now;
            // single tap will be processed by timeout logic
            return;
        }
    }
}

static void process_single_tap_timeout(void)
{
    if (s.tap_count != 1) return;

    TickType_t now = now_tick();
    TickType_t dt = now - s.first_tap_tick;

    if (dt >= ms_to_ticks(s.cfg.double_tap_window_ms)) {
        // no 2nd tap came -> single tap action
        s.tap_count = 0;
        ESP_LOGI(TAG, "Single TAP");
        apply_toggle_power();
    }
}

// ---------- Task ----------
static void touch_task(void *arg)
{
    (void)arg;

    // baseline state
    int level = gpio_get_level((gpio_num_t)s.cfg.gpio_num);
    s.touched = level_to_touched(level);
    s.pressed = false;
    s.tap_count = 0;
    s.hold_active = false;

    // start direction: up
    s.next_hold_dir = +1;

    // cached defaults if no getters
    s.cached_power = false;
    s.cached_brightness = 128;
    s.cached_ct_mired = (s.cfg.ct_min_mired + s.cfg.ct_max_mired) / 2;

    ESP_LOGI(TAG, "Touch task started. GPIO%d active_%s",
             s.cfg.gpio_num, s.cfg.active_low ? "LOW" : "HIGH");

    while (1) {
        int lvl;
        // wake periodically to handle timeouts/hold stepping
        if (xQueueReceive(s.q, &lvl, ms_to_ticks(LOOP_TICK_MS)) == pdTRUE) {

            // Debounce: wait, then re-sample
            if (s.cfg.debounce_ms > 0) {
                vTaskDelay(ms_to_ticks(s.cfg.debounce_ms));
            }

            int stable_lvl = gpio_get_level((gpio_num_t)s.cfg.gpio_num);
            bool new_touched = level_to_touched(stable_lvl);

            if (new_touched != s.touched) {
                s.touched = new_touched;
                if (new_touched) on_press();
                else on_release();
            }

            // drain bursty edges
            while (xQueueReceive(s.q, &lvl, 0) == pdTRUE) { }
        }

        // periodic logic
        start_hold_if_needed();
        hold_step();
        process_single_tap_timeout();
    }
}

esp_err_t touch_sensor_start(const touch_sensor_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (s.started) return ESP_OK;

    s.cfg = *cfg;

    if (s.cfg.task_stack_bytes == 0) s.cfg.task_stack_bytes = TOUCH_TASK_STACK_DEFAULT;
    if (s.cfg.task_priority == 0)    s.cfg.task_priority = TOUCH_TASK_PRIO_DEFAULT;
    if (s.cfg.task_name == NULL)     s.cfg.task_name = TOUCH_TASK_NAME_DEFAULT;

    if (s.cfg.debounce_ms == 0) s.cfg.debounce_ms = 40;
    if (s.cfg.hold_threshold_ms == 0) s.cfg.hold_threshold_ms = 450;
    if (s.cfg.double_tap_window_ms == 0) s.cfg.double_tap_window_ms = 300;

    if (s.cfg.brightness_max == 0) s.cfg.brightness_max = 255;
    if (s.cfg.brightness_step == 0) s.cfg.brightness_step = 2;
    if (s.cfg.brightness_step_ms == 0) s.cfg.brightness_step_ms = 20;

    if (s.cfg.ct_step == 0) s.cfg.ct_step = 5;
    if (s.cfg.ct_step_ms == 0) s.cfg.ct_step_ms = 40;

    s.mode = s.cfg.initial_hold_mode;


    // CT defaults (mired). Your lamp range is 200..455
    if (s.cfg.ct_min_mired == 0) s.cfg.ct_min_mired = 200;
    if (s.cfg.ct_max_mired == 0) s.cfg.ct_max_mired = 455;

    // Safety: if user swapped them or they’re equal
    if (s.cfg.ct_max_mired <= s.cfg.ct_min_mired) {
        s.cfg.ct_min_mired = 200;
        s.cfg.ct_max_mired = 455;
    }

    if (s.cfg.ct_step == 0) s.cfg.ct_step = 1;
    if (s.cfg.ct_step_ms == 0) s.cfg.ct_step_ms = 20;


    s.lock = xSemaphoreCreateMutex();
    if (!s.lock) return ESP_ERR_NO_MEM;

    s.q = xQueueCreate(QUEUE_LEN, sizeof(int));
    if (!s.q) return ESP_ERR_NO_MEM;

    // GPIO input
    gpio_config_t io = {0};
    io.pin_bit_mask = (1ULL << s.cfg.gpio_num);
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = s.cfg.enable_pullup ? 1 : 0;
    io.pull_down_en = s.cfg.enable_pulldown ? 1 : 0;
    io.intr_type = GPIO_INTR_ANYEDGE;

    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) return err;
    
    // Install ISR service once; if already installed, ESP_ERR_INVALID_STATE may occur.
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = gpio_isr_handler_add((gpio_num_t)s.cfg.gpio_num, gpio_isr, NULL);
    if (err != ESP_OK) return err;

    // Task
    BaseType_t ok = xTaskCreate(
        touch_task,
        s.cfg.task_name,
        s.cfg.task_stack_bytes / sizeof(StackType_t),
        NULL,
        s.cfg.task_priority,
        &s.task
    );
    if (ok != pdPASS) return ESP_ERR_NO_MEM;

    s.started = true;
    ESP_LOGI(TAG, "Touch sensor started. Hold mode initial=%s",
             (s.mode == TOUCH_HOLD_MODE_BRIGHTNESS) ? "BRIGHTNESS" : "CT");
    return ESP_OK;
}

esp_err_t touch_sensor_stop(void)
{
    if (!s.started) return ESP_OK;

    gpio_isr_handler_remove((gpio_num_t)s.cfg.gpio_num);
    gpio_set_intr_type((gpio_num_t)s.cfg.gpio_num, GPIO_INTR_DISABLE);

    if (s.task) {
        vTaskDelete(s.task);
        s.task = NULL;
    }
    if (s.q) {
        vQueueDelete(s.q);
        s.q = NULL;
    }
    if (s.lock) {
        vSemaphoreDelete(s.lock);
        s.lock = NULL;
    }

    s.started = false;
    return ESP_OK;
}

touch_hold_mode_t touch_sensor_get_hold_mode(void)
{
    return s.mode;
}
