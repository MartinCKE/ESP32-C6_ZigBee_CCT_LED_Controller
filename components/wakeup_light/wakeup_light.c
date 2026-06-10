#include "wakeup_light.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tlc59108.h"
#include "zigbee_app.h"
#include <math.h>

static const char *TAG = "WAKEUP";



#define NVS_NS "wakeup"

// NVS keys
#define KEY_START_BRI  "s_bri"
#define KEY_END_BRI    "e_bri"
#define KEY_START_CT   "s_ct"
#define KEY_END_CT     "e_ct"
#define KEY_FADE_MS    "fade_ms"

// Wakeup cycle parameters
#ifndef WAKEUP_STEP_MS
#define WAKEUP_STEP_MS 2000U
#endif
#ifndef WAKEUP_INNER_FADE_MS
#define WAKEUP_INNER_FADE_MS 800U
#endif
static TaskHandle_t s_wakeup_task = NULL;
static volatile bool s_wakeup_task_stop = false;

/* If your file already declares these static clamp_* implementations later,
 * provide prototypes so this code can call them safely. */
static uint8_t  clamp_u8(uint8_t v, uint8_t lo, uint8_t hi);
static uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi);
static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi);

static volatile bool s_wakeup_running = false;
static wakeup_apply_cb_t s_apply_cb = NULL;

static volatile uint8_t  s_last_applied_bri = 0;
static volatile uint16_t s_last_applied_ct  = 0;
static volatile bool s_freeze_stop = false;

uint8_t wakeup_get_last_bri(void) { return s_last_applied_bri; }
uint16_t wakeup_get_last_ct(void) { return s_last_applied_ct; }

esp_err_t wakeup_stop_cycle_freeze(void)
{
    return wakeup_stop_cycle();
}

#define WAKEUP_DEFAULT_CFG  {                 \
    .start_bri      = 1,                      \
    .end_bri        = 128,                    \
    .start_ct_mired = 455,                    \
    .end_ct_mired   = 200,                    \
    .fade_time_ms   = 15 * 60 * 1000UL,       \
    .enabled        = false,                  \
}

static wakeup_cfg_t s_cfg = WAKEUP_DEFAULT_CFG;

wakeup_cfg_t wakeup_get_defaults(void)
{
    return (wakeup_cfg_t)WAKEUP_DEFAULT_CFG;
}

bool wakeup_is_running(void)
{
    return s_wakeup_running;
}


void wakeup_register_apply_cb(wakeup_apply_cb_t cb)
{
    s_apply_cb = cb;
    ESP_LOGD(TAG, "wakeup apply callback registered: %p", (void*)cb);
}

static void wakeup_runner_task(void *arg)
{
    (void)arg;

    wakeup_cfg_t cfg = wakeup_get();

    cfg.start_bri      = clamp_u8(cfg.start_bri, 0, 255);
    cfg.end_bri        = clamp_u8(cfg.end_bri, 0, 255);
    cfg.start_ct_mired = clamp_u16(cfg.start_ct_mired, 200, 455);
    cfg.end_ct_mired   = clamp_u16(cfg.end_ct_mired, 200, 455);
    cfg.fade_time_ms   = clamp_u32(cfg.fade_time_ms, 1, 900000UL);

    ESP_LOGD(TAG, "Wakeup runner start: bri %u->%u, ct %u->%u, %lu ms",
             (unsigned)cfg.start_bri, (unsigned)cfg.end_bri,
             (unsigned)cfg.start_ct_mired, (unsigned)cfg.end_ct_mired,
             (unsigned long)cfg.fade_time_ms);

    /* Apply start state quickly */
    light_set_on_state(true);
    tlc_set_ct_mired_smooth(cfg.start_ct_mired, WAKEUP_INNER_FADE_MS);
    tlc_set_logical_brightness_smooth_ms(cfg.start_bri, tlc_get_current_ct_mired(), WAKEUP_INNER_FADE_MS);

    s_last_applied_bri = cfg.start_bri;        // ✅ NEW
    s_last_applied_ct  = cfg.start_ct_mired;   // ✅ NEW

    TickType_t start_ticks = xTaskGetTickCount();
    const TickType_t step_ticks = pdMS_TO_TICKS(WAKEUP_STEP_MS);

    while (!s_wakeup_task_stop) {
        ESP_LOGD(TAG, "Wakeup runner step");
        TickType_t now = xTaskGetTickCount();
        uint32_t elapsed_ms = (uint32_t)((uint64_t)(now - start_ticks) * portTICK_PERIOD_MS);

        if (elapsed_ms >= cfg.fade_time_ms) break;

        double t = (double)elapsed_ms / (double)cfg.fade_time_ms;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;

        /* smoothstep easing (optional but nice) */
        t = t * t * (3.0 - 2.0 * t);

        double bri_d = (double)cfg.start_bri + ((double)cfg.end_bri - (double)cfg.start_bri) * t;
        double ct_d  = (double)cfg.start_ct_mired + ((double)cfg.end_ct_mired - (double)cfg.start_ct_mired) * t;

        uint8_t bri = (uint8_t)bri_d;
        uint16_t ct = (uint16_t)ct_d;

        bri = clamp_u8(bri, 0, 255);
        ct  = clamp_u16(ct, 200, 455);
        ESP_LOGD(TAG, "Wakeup step: t=%.3f elapsed=%lu ms -> bri=%u ct=%u",
                 t, (unsigned long)elapsed_ms, (unsigned)bri, (unsigned)ct);

        s_last_applied_bri = bri;
        s_last_applied_ct  = ct;

        /* cooperative short fades */
        uint16_t ct_prev = (uint16_t)mired;   // current system CT before step
        tlc_set_ct_mired_smooth(ct, WAKEUP_INNER_FADE_MS);
        tlc_set_logical_brightness_smooth_ms(bri, ct_prev, WAKEUP_INNER_FADE_MS);

        mired = ct;
        current_brightness = bri;

        zigbee_set_ct_and_report(ct);
        zigbee_set_level_and_report(bri);

        vTaskDelay(step_ticks);
    }


    if (!s_freeze_stop) {
        s_last_applied_bri = cfg.end_bri;
        s_last_applied_ct = cfg.end_ct_mired;
        current_brightness = cfg.end_bri;
        mired = cfg.end_ct_mired;
        light_set_on_state(cfg.end_bri > 0);
        tlc_set_ct_mired_smooth(cfg.end_ct_mired, WAKEUP_INNER_FADE_MS);
        tlc_set_logical_brightness_smooth_ms(cfg.end_bri, tlc_get_current_ct_mired(), WAKEUP_INNER_FADE_MS);
        zigbee_set_ct_and_report(cfg.end_ct_mired);
        zigbee_set_level_and_report(cfg.end_bri);
    } else {
        ESP_LOGD(TAG, "Wakeup stopped (freeze) at bri=%u ct=%u",
                (unsigned)s_last_applied_bri, (unsigned)s_last_applied_ct);
    }

    s_freeze_stop = false;

    s_wakeup_task = NULL;
    s_wakeup_task_stop = false;
    s_wakeup_running = false;

    ESP_LOGD(TAG, "Wakeup runner done");
    vTaskDelete(NULL);
}

/* Start: create the runner task that issues short fades periodically */
esp_err_t wakeup_start_cycle(void)
{
    if (s_wakeup_task) {
        ESP_LOGD(TAG, "Wakeup already running");
        return ESP_FAIL;
    }

    wakeup_cfg_t cfg = wakeup_get();
    if (!cfg.enabled) {
        ESP_LOGD(TAG, "Wakeup disabled; not starting");
        return ESP_FAIL;
    }

    s_wakeup_task_stop = false;
    s_wakeup_running = true;

    BaseType_t ok = xTaskCreate(wakeup_runner_task, "wakeup_runner", 3072, NULL, tskIDLE_PRIORITY + 1, &s_wakeup_task);
    if (ok != pdPASS) {
        s_wakeup_task = NULL;
        s_wakeup_running = false;
        ESP_LOGE(TAG, "Failed to create wakeup_runner task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t wakeup_stop_cycle(void)
{
    if (!s_wakeup_task) {
        ESP_LOGD(TAG, "Wakeup not running");
        return ESP_FAIL;
    }

    // Freeze semantics: stop at the currently rendered output.
    s_freeze_stop = true;
    s_wakeup_task_stop = true;

    uint16_t hold_ct = tlc_get_current_ct_mired();
    uint8_t hold_bri = get_current_logical_brightness_from_outputs();

    tlc_set_output_immediate(hold_bri, hold_ct);

    mired = hold_ct;
    current_brightness = hold_bri;
    s_last_applied_bri = hold_bri;
    s_last_applied_ct = hold_ct;
    light_set_on_state(hold_bri > 0);

    return ESP_OK;
}

static uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static uint8_t clamp_u8(uint8_t v, uint8_t lo, uint8_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

esp_err_t wakeup_load_from_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "No wakeup NVS yet (%s), using defaults", esp_err_to_name(err));
        s_cfg = (wakeup_cfg_t)WAKEUP_DEFAULT_CFG;
        return err;
    }

    int32_t v32;
    uint32_t vu32;

    if (nvs_get_i32(h, KEY_START_BRI, &v32) == ESP_OK) s_cfg.start_bri = (uint8_t)v32;
    if (nvs_get_i32(h, KEY_END_BRI, &v32) == ESP_OK)   s_cfg.end_bri   = (uint8_t)v32;
    if (nvs_get_i32(h, KEY_START_CT, &v32) == ESP_OK)  s_cfg.start_ct_mired = (uint16_t)v32;
    if (nvs_get_i32(h, KEY_END_CT, &v32) == ESP_OK)    s_cfg.end_ct_mired   = (uint16_t)v32;
    if (nvs_get_u32(h, KEY_FADE_MS, &vu32) == ESP_OK)  s_cfg.fade_time_ms   = (uint32_t)vu32;

    nvs_close(h);

    // sanitize
    s_cfg.start_bri      = clamp_u8(s_cfg.start_bri, 0, 255);
    s_cfg.end_bri        = clamp_u8(s_cfg.end_bri, 0, 255);
    s_cfg.start_ct_mired = clamp_u16(s_cfg.start_ct_mired, 200, 455);
    s_cfg.end_ct_mired   = clamp_u16(s_cfg.end_ct_mired, 200, 455);
    s_cfg.fade_time_ms   = clamp_u32(s_cfg.fade_time_ms, 1, 900000UL);

    ESP_LOGD(TAG, "Loaded cfg: start_bri=%u end_bri=%u start_ct=%u end_ct=%u fade_ms=%lu",
             (unsigned)s_cfg.start_bri, (unsigned)s_cfg.end_bri,
             (unsigned)s_cfg.start_ct_mired, (unsigned)s_cfg.end_ct_mired,
             (unsigned long)s_cfg.fade_time_ms);

    return ESP_OK;
}

esp_err_t wakeup_save_to_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return err;
    }

    // store as i32/u32 (simple & robust)
    nvs_set_i32(h, KEY_START_BRI, (int32_t)s_cfg.start_bri);
    nvs_set_i32(h, KEY_END_BRI,   (int32_t)s_cfg.end_bri);
    nvs_set_i32(h, KEY_START_CT,  (int32_t)s_cfg.start_ct_mired);
    nvs_set_i32(h, KEY_END_CT,    (int32_t)s_cfg.end_ct_mired);
    nvs_set_u32(h, KEY_FADE_MS,   (uint32_t)s_cfg.fade_time_ms);

    err = nvs_commit(h);
    nvs_close(h);

    if (err == ESP_OK) ESP_LOGD(TAG, "Wakeup cfg saved to NVS");
    else ESP_LOGE(TAG, "nvs_commit failed: %s", esp_err_to_name(err));

    return err;
}

esp_err_t wakeup_init(void)
{
    (void)wakeup_load_from_nvs();
    /* Always come up disabled after reboot */
    s_cfg.enabled = false;

    /* Optional: persist that to NVS so next boot is also off */
    wakeup_save_to_nvs();

    /* Ensure no runner is active */
    (void)wakeup_stop_cycle();

    return ESP_OK;
}

wakeup_cfg_t wakeup_get(void)
{
    return s_cfg; // struct copy
}

esp_err_t wakeup_set(const wakeup_cfg_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;

    wakeup_cfg_t c = *cfg;

    // sanitize (and enforce "explicit start/end", as you wanted)
    c.start_bri      = clamp_u8(c.start_bri, 0, 255);
    c.end_bri        = clamp_u8(c.end_bri, 0, 255);
    c.start_ct_mired = clamp_u16(c.start_ct_mired, 200, 455);
    c.end_ct_mired   = clamp_u16(c.end_ct_mired, 200, 455);
    c.fade_time_ms   = clamp_u32(c.fade_time_ms, 1, 900000UL);

    s_cfg = c;

    ESP_LOGD(TAG, "Set cfg: start_bri=%u end_bri=%u start_ct=%u end_ct=%u fade_ms=%lu enabled=%u",
             (unsigned)s_cfg.start_bri, (unsigned)s_cfg.end_bri,
             (unsigned)s_cfg.start_ct_mired, (unsigned)s_cfg.end_ct_mired,
             (unsigned long)s_cfg.fade_time_ms, (unsigned)s_cfg.enabled);

    return ESP_OK;
}
