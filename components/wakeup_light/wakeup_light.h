#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  start_bri;
    uint8_t  end_bri;
    uint16_t start_ct_mired;
    uint16_t end_ct_mired;
    uint32_t fade_time_ms;
    bool     enabled;
} wakeup_cfg_t;

// Call once at boot (after nvs_flash_init)
esp_err_t wakeup_init(void);

// Get current config (copy)
wakeup_cfg_t wakeup_get(void);

// Update config (in RAM)
esp_err_t wakeup_set(const wakeup_cfg_t *cfg);

// Persist current config to NVS
esp_err_t wakeup_save_to_nvs(void);

// Optional convenience
esp_err_t wakeup_load_from_nvs(void);

#ifdef __cplusplus
}
#endif
