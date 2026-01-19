// tlc59108.c
#include "esp_check.h"
#include "esp_log.h"
#include "tlc59108.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include "zigbee_app.h"   // for current_brightness, mired, zigbee_set_onoff_and_report(), etc.

/*
 * Notes / fixes:
 * - Fixed out-of-bounds bugs:
 *   - amber_channels/white_channels length is 3 -> loops use i<3 and group count 3
 *   - all_channels length is 6 -> group count 6 everywhere
 * - Added smooth fade engine (2ms steps) in a task.
 * - Normal “lamp control” should use:
 *     tlc_set_logical_brightness_smooth(target, mired)
 *     tlc_set_ct_mired(new_mired)
 *   and light_toggle_handler() now fades on/off.
 * - Logging in led_apply_brightness_and_ct() is reduced (DEBUG only) to avoid ruining smooth fades.
 */

// Breathing animation state
static bool  breathing_enabled   = false;
static float breathing_speed_hz  = 0.5f;
static float breathing_time      = 0.0f;

#define TLC_POWER_GPIO  GPIO_NUM_10
static bool power_gpio_initialized = false;

#define TLC_RESET_GPIO  GPIO_NUM_15
static bool reset_gpio_initialized = false;

// NOTE: You also have current_brightness in zigbee_app.c; keep one “logical” source of truth there.
uint16_t brightness = 128;

#define TLC_ADDR 0x41
#define REG_MODE1   0x00
#define REG_MODE2   0x01
#define REG_PWM0    0x02
#define REG_LEDOUT0 0x0C
#define REG_LEDOUT1 0x0D

static const uint8_t amber_channels[] = {0, 1, 2};
static const uint8_t white_channels[] = {3, 4, 5};
static const uint8_t all_channels[]   = {0, 1, 2, 3, 4, 5};

static i2c_master_dev_handle_t tlc_dev;

static const char *TAG = "TLC9108";

static bool light_on = true;
static uint8_t s_last_nonzero_bri = 128;   // fallback if NVS/level ever goes 0

// Software-tracked current output (post-mix)
static uint8_t s_out_amber = 0;
static uint8_t s_out_white = 0;

// Fade engine state (logical brightness 0..255)
typedef struct {
    uint8_t  cur;        // current logical brightness
    uint8_t  target;     // target logical brightness
    uint16_t mired;      // mired used for mixing during fade
    bool     running;
} fade_t;

static fade_t s_fade = {
    .cur = 0, 
    .target = 0, 
    .mired = 300, 
    .running = false
};
static TaskHandle_t s_fade_task = NULL;

typedef struct {
    uint16_t cur;
    uint16_t target;
    uint32_t transition_ms;
    bool running;
} ct_fade_t;

static ct_fade_t s_ct = {
    .cur = 300,
    .target = 300,
    .transition_ms = 300,
    .running = false
};


// CT mixing ratios
static float ct_white_ratio = 0.5f;
static float ct_amber_ratio = 0.5f;

bool light_is_on(void)
{
    return light_on;
}

void light_remember_brightness(uint8_t bri)
{
    if (bri > 0) s_last_nonzero_bri = bri;
}

void light_set_on(bool on, bool report_zigbee)
{
    // If Zigbee repeats "On" or "Off", still enforce the output target.
    light_on = on;
    ESP_LOGI(TAG, "light_set_on: on=%d report_zigbee=%d", (int)on, (int)report_zigbee);

    if (!on) {
        // Fade down to 0, do NOT change current_brightness
        tlc_set_logical_brightness_smooth(0, (uint16_t)mired);
    } else {
        // Pick a non-zero brightness target
        uint8_t bri = (current_brightness > 0) ? (uint8_t)current_brightness : s_last_nonzero_bri;
        if (bri == 0) bri = 128;

        // Remember it for next time
        s_last_nonzero_bri = bri;

        // Fade up
        tlc_set_logical_brightness_smooth(bri, (uint16_t)mired);
    }

    if (report_zigbee) {
        zigbee_set_onoff_and_report(light_on);
    }
}


static esp_err_t tlc_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = { reg, value };
    esp_err_t err = i2c_master_transmit(tlc_dev, data, 2, -1);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TLC write failed: reg=0x%02X val=0x%02X err=%s",
                 reg, value, esp_err_to_name(err));
    }
    return err;
}

esp_err_t tlc_read_reg(uint8_t reg, uint8_t *out_value)
{
    return i2c_master_transmit_receive(tlc_dev, &reg, 1, out_value, 1, -1);
}

uint8_t percentage_to_8bit(uint8_t percentage)
{
    return (percentage * 255 + 50) / 100;
}

void tlc_reset_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << TLC_RESET_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&io_conf);
    gpio_set_level(TLC_RESET_GPIO, 1);
}

void tlc_power_init(void)
{
    if (power_gpio_initialized) return;

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << TLC_POWER_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io_conf);
    power_gpio_initialized = true;
}

void tlc_power_set(bool on)
{
    if (!power_gpio_initialized) {
        tlc_power_init();
    }
    gpio_set_level(TLC_POWER_GPIO, on ? 1 : 0);
}

void tlc_reset_pulse(void)
{
    if (!reset_gpio_initialized) {
        tlc_reset_init();
        reset_gpio_initialized = true;
    }

    gpio_set_level(TLC_RESET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(TLC_RESET_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
}

// ---------- PWM primitives ----------

esp_err_t tlc59108_set_pwm(uint8_t channel, uint8_t value)
{
    if (channel > 7) return ESP_ERR_INVALID_ARG;
    return tlc_write_reg(REG_PWM0 + channel, value);
}

esp_err_t tlc59108_set_group_pwm(const uint8_t *channels, uint8_t count, uint8_t value)
{
    for (int i = 0; i < count; i++) {
        tlc59108_set_pwm(channels[i], value);
    }
    return ESP_OK;
}

void tlc_set_channel_brightness(uint8_t channel, uint8_t value)
{
    if (channel > 7) return;
    tlc_write_reg(REG_PWM0 + channel, value);
}

void tlc_set_group_brightness(uint8_t *channels, int count, uint8_t value)
{
    for (int i = 0; i < count; i++) {
        tlc_set_channel_brightness(channels[i], value);
    }
}

// ---------- brightness getters (fixed bounds) ----------

void tlc_get_group_brightness(uint8_t *amber, uint8_t *white)
{
    if (amber) *amber = s_out_amber;
    if (white) *white = s_out_white;
}

static uint8_t get_current_logical_brightness_from_outputs(void)
{
    // logical brightness is roughly amber+white (since each is brightness * ratio)
    // choose max() to be safe (prevents weird values if ratios change)
    uint8_t a = s_out_amber;
    uint8_t w = s_out_white;
    return (a > w) ? a : w;
}


uint8_t tlc_get_amber_brightness(void)
{
    uint8_t sum = 0, v = 0;
    for (int i = 0; i < 3; i++) {
        tlc_read_reg(REG_PWM0 + amber_channels[i], &v);
        sum += v;
    }
    return sum / 3;
}

uint8_t tlc_get_white_brightness(void)
{
    uint8_t sum = 0, v = 0;
    for (int i = 0; i < 3; i++) {
        tlc_read_reg(REG_PWM0 + white_channels[i], &v);
        sum += v;
    }
    return sum / 3;
}

// ---------- CT + apply ----------

void led_color_temperature_control(uint16_t brightness_in, uint16_t mired_in)
{
    // Protect against divide-by-zero / nonsense input
    if (mired_in < 1) mired_in = 1;

    float kelvin = 1000000.0f / (float)mired_in;

    if (kelvin < 2200.0f) kelvin = 2200.0f;
    if (kelvin > 5000.0f) kelvin = 5000.0f;

    ct_white_ratio = (kelvin - 2200.0f) / (5000.0f - 2200.0f);
    if (ct_white_ratio < 0) ct_white_ratio = 0;
    if (ct_white_ratio > 1) ct_white_ratio = 1;

    ct_amber_ratio = 1.0f - ct_white_ratio;

    led_apply_brightness_and_ct(brightness_in, mired_in);
    ESP_LOGI(TAG, "led_color_temperature_control: bri=%u mired=%u", (unsigned)brightness_in, (unsigned)mired_in);
}

void led_apply_brightness_and_ct(uint16_t brightness_in, uint16_t mired_in)
{
    (void)mired_in;

    uint8_t amber_pwm = (uint16_t)(brightness_in * ct_amber_ratio);
    uint8_t white_pwm = (uint16_t)(brightness_in * ct_white_ratio);

    tlc_set_group_brightness((uint8_t *)amber_channels, 3, amber_pwm);
    tlc_set_group_brightness((uint8_t *)white_channels, 3, white_pwm);

    s_out_amber = amber_pwm;
    s_out_white = white_pwm;

    // Avoid spamming logs during fade (2ms steps)
    ESP_LOGD(TAG, "Apply: bri=%u amber=%u white=%u", (unsigned)brightness_in, amber_pwm, white_pwm);
}

// ---------- fade engine ----------

static void fade_task(void *arg)
{
    (void)arg;
    const TickType_t step_delay = pdMS_TO_TICKS(2);

    while (1) {
        bool did_work = false;

        // ---- Brightness step ----
        if (s_fade.running && s_fade.cur != s_fade.target) {
            if (s_fade.cur < s_fade.target) s_fade.cur++;
            else s_fade.cur--;
            did_work = true;
        } else {
            s_fade.running = false;
        }

        // ---- CT step ----
        if (s_ct.running && s_ct.cur != s_ct.target) {
            uint32_t steps = s_ct.transition_ms / 2;
            if (steps < 1) steps = 1;

            int32_t diff = (int32_t)s_ct.target - (int32_t)s_ct.cur;
            int32_t step = diff / (int32_t)steps;
            if (step == 0) step = (diff > 0) ? 1 : -1;

            int32_t next = (int32_t)s_ct.cur + step;
            if ((diff > 0 && next > (int32_t)s_ct.target) ||
                (diff < 0 && next < (int32_t)s_ct.target)) {
                next = s_ct.target;
            }

            s_ct.cur = (uint16_t)next;
            did_work = true;
        } else {
            s_ct.running = false;
        }

        // ---- Apply output (only once) ----
        if (did_work) {
            // This must apply using the CURRENT CT value (s_ct.cur)
            led_color_temperature_control((uint16_t)s_fade.cur, (uint16_t)s_ct.cur);
            s_fade.mired = s_ct.cur;
            vTaskDelay(step_delay);
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}



void tlc_set_logical_brightness_smooth(uint8_t target, uint16_t mired_now)
{
    // If we are currently at 0 output (or not running), start cur from real output
    uint8_t cur_hw = get_current_logical_brightness_from_outputs();

    // If fade isn't running, sync cur to actual output so ramp always starts correctly
    if (!s_fade.running) {
        s_fade.cur = cur_hw;
    }

    s_fade.target = target;
    s_fade.mired  = mired_now;
    s_fade.running = true;

    ESP_LOGI(TAG, "tlc_set_logical_brightness: cur=%u target=%u mired=%u",
             (unsigned)s_fade.cur, (unsigned)s_fade.target, (unsigned)s_fade.mired);
}

void tlc_set_ct_mired_smooth(uint16_t target_mired, uint32_t transition_ms)
{
    if (target_mired < 200) target_mired = 200;
    if (target_mired > 455) target_mired = 455;
    if (transition_ms < 1) transition_ms = 1;

    // Always start from the currently active CT, NOT only when cur==0
    // The "currently active" CT for the fade engine is s_ct.cur.
    // If you don't track that properly yet, set it from global mired
    // only when not running.
    if (!s_ct.running) {
        s_ct.cur = (uint16_t)s_fade.mired;   // <- start from what's being rendered
        if (s_ct.cur == 0) s_ct.cur = (uint16_t)mired; // fallback
    }

    s_ct.target = target_mired;
    s_ct.transition_ms = transition_ms;
    s_ct.running = true;

    // Ensure fade engine uses CT state (important if your fade task reads s_fade.mired)
    s_fade.mired = s_ct.cur;
}



void tlc_set_ct_mired(uint16_t new_mired)
{
    if (new_mired < 200) new_mired = 200;
    if (new_mired > 455) new_mired = 455;

    s_ct.cur = new_mired;
    s_ct.target = new_mired;
    s_ct.running = false;

    s_fade.mired = new_mired;

    led_color_temperature_control((uint16_t)s_fade.cur, new_mired);
}

// ---------- init ----------

esp_err_t tlc59108_init(i2c_master_bus_handle_t bus)
{
    tlc_power_set(true);
    vTaskDelay(pdMS_TO_TICKS(20));

    i2c_device_config_t devcfg = {
        .device_address = TLC_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &devcfg, &tlc_dev));

    // MODE1 = normal
    tlc_write_reg(REG_MODE1, 0x00);
    // MODE2 = totem-pole, non-inverted
    tlc_write_reg(REG_MODE2, 0x20);
    // LEDOUT = PWM mode
    tlc_write_reg(REG_LEDOUT0, 0xAA);
    tlc_write_reg(REG_LEDOUT1, 0xAA);

    // init all PWM to 0 (only channels 0..5 used, but safe to clear 0..7)
    for (int i = 0; i < 8; i++) {
        tlc_write_reg(REG_PWM0 + i, 0);
    }

    if (!s_fade_task) {
        xTaskCreate(fade_task, "tlc_fade", 2048, NULL, 6, &s_fade_task);
    }

    return ESP_OK;
}

// ---------- convenience / misc ----------

esp_err_t tlc_set_all_brightness(uint8_t value)
{
    ESP_LOGI(TAG, "Setting all brightness to %d", value);
    return tlc59108_set_group_pwm(all_channels, 6, value);
}

esp_err_t tlc_set_all_brightness_percentage(uint8_t percentage)
{
    uint8_t value = percentage_to_8bit(percentage);
    return tlc59108_set_group_pwm(all_channels, 6, value);
}

esp_err_t tlc_set_white_brightness(uint8_t value)
{
    return tlc59108_set_group_pwm(white_channels, 3, value);
}

esp_err_t tlc_set_amber_brightness(uint8_t value)
{
    return tlc59108_set_group_pwm(amber_channels, 3, value);
}

void tlc_breathe_update(float dt_seconds)
{
    if (!breathing_enabled) return;

    const float two_pi = 6.28318530718f;
    float wave = 0.5f * (1.0f + sinf(two_pi * breathing_speed_hz * breathing_time));

    uint8_t bri = (uint8_t)(wave * 100.0f);
    tlc_set_all_brightness(bri);

    breathing_time += dt_seconds;
}

void tlc_breathe_init(float speed_hz)
{
    breathing_speed_hz = speed_hz;
    breathing_time = 0.0f;
    breathing_enabled = true;
}

void tlc_set_breathing_enabled(bool enabled)
{
    breathing_enabled = enabled;
}

void tlc_dump_registers(void)
{
    uint8_t val;
    printf("----- TLC59108 Register Dump -----\n");
    for (int reg = 0x00; reg <= 0x0D; reg++) {
        if (tlc_read_reg(reg, &val) == ESP_OK) {
            printf("Reg 0x%02X = 0x%02X\n", reg, val);
        } else {
            printf("Reg 0x%02X = <ERROR>\n", reg);
        }
    }
    printf("----------------------------------\n");
}

// ---------- animations / sequences ----------

void tlc_boot_led_sequence(void)
{
    const int step = 20;
    const int max_val = 255;
    const int num_channels = 6;

    for (int ch = 0; ch < num_channels; ch++) {
        tlc59108_set_pwm(ch, 0);
    }

    int val = 0;

    while (1) {
        val += step;
        if (val > max_val) val = max_val;

        for (int ch = 0; ch < num_channels; ch++) {
            tlc59108_set_pwm(ch, (uint8_t)val);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        if (val == max_val) break;
    }

    while (1) {
        val -= step;
        if (val < 0) val = 0;

        for (int ch = 0; ch < num_channels; ch++) {
            tlc59108_set_pwm(ch, (uint8_t)val);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        if (val == 0) break;
    }

    for (int ch = 0; ch < num_channels; ch++) {
        tlc59108_set_pwm(ch, 0);
    }
}

void led_boot_trail_spin_animation(void)
{
    const uint8_t num_leds = 6;
    const uint8_t head_brightness = 255;
    const uint8_t trail_step = 60;
    const int spin_delay_ms = 120;
    const int rotations = 2;

    for (int r = 0; r < rotations; r++) {
        for (int head = 0; head < num_leds; head++) {
            for (int i = 0; i < num_leds; i++) {
                int dist = (head - i + num_leds) % num_leds;
                int value = head_brightness - dist * trail_step;
                if (value < 0) value = 0;
                tlc_set_channel_brightness(all_channels[i], (uint8_t)value);
            }
            vTaskDelay(pdMS_TO_TICKS(spin_delay_ms));
        }
    }

    for (int b = 0; b <= 255; b += 5) {
        for (int i = 0; i < num_leds; i++) tlc_set_channel_brightness(all_channels[i], (uint8_t)b);
        vTaskDelay(pdMS_TO_TICKS(15));
    }

    for (int b = 255; b >= 0; b -= 5) {
        for (int i = 0; i < num_leds; i++) tlc_set_channel_brightness(all_channels[i], (uint8_t)b);
        vTaskDelay(pdMS_TO_TICKS(15));
    }

    for (int i = 0; i < num_leds; i++) tlc_set_channel_brightness(all_channels[i], 0);
}

void zigbee_connection_confirmed_sequence(void)
{
    const TickType_t on_time  = pdMS_TO_TICKS(200);
    const TickType_t off_time = pdMS_TO_TICKS(120);

    tlc_set_breathing_enabled(false);
    tlc_set_all_brightness(0);

    for (int i = 0; i < 3; i++) {
        tlc_set_all_brightness(128);
        vTaskDelay(on_time);
        tlc_set_all_brightness(0);
        vTaskDelay(off_time);
    }

    // After blink, return to “normal” output (smooth engine will take over as you call it)
    light_on = true;
    zigbee_set_onoff_and_report(light_on);
}


void light_toggle_handler(void)
{
    light_set_on(!light_is_on(), true);
}


// Optional: keep this around if other code needs state
bool tlc_light_is_on(void)
{
    return light_on;
}
