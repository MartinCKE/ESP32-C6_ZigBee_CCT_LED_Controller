#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BUTTON_EVENT_NONE = 0,
    BUTTON_EVENT_SINGLE_PRESS,
    BUTTON_EVENT_DOUBLE_PRESS,
    BUTTON_EVENT_LONG_PRESS,
} button_event_t;

typedef struct {
    int gpio_num;                        // e.g. 18
    bool active_low;                     // true if pressed reads as 0
    bool enable_internal_pullup;         // typical for active_low button
    uint32_t debounce_ms;                // e.g. 30
    uint32_t long_press_ms;              // e.g. 2000
    uint32_t double_press_window_ms;     // e.g. 350
} button_config_t;

/**
 * Initialize the button driver. You must provide the task handle that will
 * receive notifications (typically your button handling task).
 */
esp_err_t button_init(const button_config_t *cfg, TaskHandle_t notify_task);

/**
 * Deinit: remove ISR handler and reset internal state.
 */
esp_err_t button_deinit(void);


/**
 * Blocking wait for next button event.
 * Call from the task you passed to button_init().
 *
 * @param out_event event returned here
 * @param timeout_ticks FreeRTOS ticks to wait; use portMAX_DELAY to wait forever
 * @return true if an event was received, false if timed out
 */
bool button_wait_event(button_event_t *out_event, TickType_t timeout_ticks);

#ifdef __cplusplus
}
#endif
