#include "wakeup_light.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "WAKEUP";

#define NVS_NS "wakeup"

// NVS keys
#define KEY_START_BRI  "s_bri"
#define KEY_END_BRI    "e_bri"
#define KEY_START_CT   "s_ct"
#define KEY_END_CT     "e_ct"
#define KEY_FADE_MS    "fade_ms"

static wakeup_cfg_t s_cfg = {
    .start_bri = 1,
    .end_bri = 128,
    .start_ct_mired = 455,
    .end_ct_mired = 200,
    .fade_time_ms = 15 * 60 * 1000UL, // 15 min default
};

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
        ESP_LOGW(TAG, "No wakeup NVS yet (%s), using defaults", esp_err_to_name(err));
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
    s_cfg.fade_time_ms   = clamp_u32(s_cfg.fade_time_ms, 1, 3600000UL);

    ESP_LOGI(TAG, "Loaded cfg: start_bri=%u end_bri=%u start_ct=%u end_ct=%u fade_ms=%lu",
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

    if (err == ESP_OK) ESP_LOGI(TAG, "Wakeup cfg saved to NVS");
    else ESP_LOGE(TAG, "nvs_commit failed: %s", esp_err_to_name(err));

    return err;
}

esp_err_t wakeup_init(void)
{
    (void)wakeup_load_from_nvs();
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
    c.fade_time_ms   = clamp_u32(c.fade_time_ms, 1, 3600000UL);

    s_cfg = c;

    ESP_LOGI(TAG, "Set cfg: start_bri=%u end_bri=%u start_ct=%u end_ct=%u fade_ms=%lu",
             (unsigned)s_cfg.start_bri, (unsigned)s_cfg.end_bri,
             (unsigned)s_cfg.start_ct_mired, (unsigned)s_cfg.end_ct_mired,
             (unsigned long)s_cfg.fade_time_ms);

    return ESP_OK;
}
