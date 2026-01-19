#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TOUCH_HOLD_MODE_BRIGHTNESS = 0,
    TOUCH_HOLD_MODE_CT         = 1,
} touch_hold_mode_t;

/**
 * Callbacks that the driver uses to control your lamp.
 * These are called from the touch sensor task context (NOT ISR).
 */
typedef struct {
    void (*toggle_power)(void *user_ctx);

    // Called frequently during hold (hardware only, no Zigbee/NVS)
    void (*apply_brightness)(uint8_t level, void *user_ctx);
    void (*apply_ct_mired)(uint16_t mired, void *user_ctx);

    // Called once at end of hold (do Zigbee report + NVS save)
    void (*commit_brightness)(uint8_t level, void *user_ctx);
    void (*commit_ct_mired)(uint16_t mired, void *user_ctx);

    void (*mode_changed)(touch_hold_mode_t mode, void *user_ctx);

    // Optional: read current values so hold starts from the true current state
    // (recommended if Zigbee can change brightness/CT externally)
    //bool     (*get_power)(void *user_ctx);           // optional
    uint8_t  (*get_brightness)(void *user_ctx);      // optional
    uint16_t (*get_ct_mired)(void *user_ctx);        // optional
} touch_sensor_actions_t;

typedef struct {
    int gpio_num;                // AT42QT1011 OUT -> ESP GPIO (your case: 4)
    bool active_low;             // false for active-high (you said IC is active high)
    bool enable_pullup;          // keep false if you truly have no pulls and IC drives it
    bool enable_pulldown;        // false normally
    uint32_t debounce_ms;        // e.g. 20..60

    // Gesture timing
    uint32_t hold_threshold_ms;      // e.g. 450
    uint32_t double_tap_window_ms;   // e.g. 300

    // Hold behavior
    touch_hold_mode_t initial_hold_mode; // BRIGHTNESS or CT
    uint8_t  brightness_min;          // e.g. 0
    uint8_t  brightness_max;          // e.g. 255
    uint8_t  brightness_step;         // e.g. 2
    uint32_t brightness_step_ms;      // e.g. 20

    uint16_t ct_min_mired;            // e.g. 200
    uint16_t ct_max_mired;            // e.g. 455
    uint16_t ct_step;                 // e.g. 1 or 2
    uint32_t ct_step_ms;              // e.g. 20

    // Driver calls these to apply changes
    touch_sensor_actions_t actions;
    void *user_ctx;

    // Task
    uint32_t task_stack_bytes; // 0 = default
    uint32_t task_priority;    // 0 = default
    const char *task_name;     // NULL = default
} touch_sensor_config_t;

/**
 * Start touch sensor driver (ISR + task).
 * Gesture detection + ramping are handled internally.
 */
esp_err_t touch_sensor_start(const touch_sensor_config_t *cfg);

/** Stop driver (optional). */
esp_err_t touch_sensor_stop(void);

/** Current hold mode (brightness vs CT). */
touch_hold_mode_t touch_sensor_get_hold_mode(void);

#ifdef __cplusplus
}
#endif

