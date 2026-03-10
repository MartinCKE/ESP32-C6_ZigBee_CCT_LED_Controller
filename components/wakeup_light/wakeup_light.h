#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* callback type to apply brightness & CT to hardware */
typedef void (*wakeup_apply_cb_t)(uint8_t bri, uint16_t ct_mired);

typedef struct {
    uint8_t  start_bri;
    uint8_t  end_bri;
    uint16_t start_ct_mired;
    uint16_t end_ct_mired;
    uint32_t fade_time_ms;
    bool     enabled;
} wakeup_cfg_t;

/* Register a hardware-apply callback (must be called before start) */
void wakeup_register_apply_cb(wakeup_apply_cb_t cb);

/* Start/stop/query wakeup cycle (non-blocking) */
esp_err_t wakeup_start_cycle(void);
esp_err_t wakeup_stop_cycle(void);
bool wakeup_is_running(void);
wakeup_cfg_t wakeup_get_defaults(void);
/* Lifecycle / config */
esp_err_t wakeup_init(void);               /* call once at boot after nvs init */
wakeup_cfg_t wakeup_get(void);             /* returns copy */
esp_err_t wakeup_set(const wakeup_cfg_t *cfg);
esp_err_t wakeup_save_to_nvs(void);
esp_err_t wakeup_load_from_nvs(void);

esp_err_t wakeup_stop_cycle_freeze(void);
uint8_t  wakeup_get_last_bri(void);
uint16_t wakeup_get_last_ct(void);

#ifdef __cplusplus
}
#endif
