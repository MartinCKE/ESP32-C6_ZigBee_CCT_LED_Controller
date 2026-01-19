#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    STATUS_LED_STATE_BOOT_OK = 0,
    STATUS_LED_STATE_JOINING_NETWORK,
    STATUS_LED_STATE_JOINED_SUCCESSFULLY,
    STATUS_LED_STATE_NORMAL_OPERATION,
    STATUS_LED_STATE_WAKEUP_SEQUENCE,
    STATUS_LED_STATE_SENSOR_ERROR,
    STATUS_LED_STATE_ZIGBEE_LOST,
} status_led_state_t;

typedef struct {
    int gpio_red;      // e.g. 19
    int gpio_yellow;   // e.g. 20
    int gpio_green;    // e.g. 21
    bool active_low;   // set true if writing 0 turns LED on
    uint32_t task_stack_bytes; // 0 = default
    uint32_t task_priority;    // 0 = default
    const char *task_name;     // NULL = default
} status_led_config_t;

/** Start status LED driver task (call once). */
void status_led_start(const status_led_config_t *cfg);

/** Set current status state (thread-safe). */
void status_led_set_state(status_led_state_t state);
void status_led_boot_ok_start(void);

/** Trigger button press indication: Green short blink (thread-safe, NOT ISR). */
void status_led_notify_button_press(void);

#ifdef __cplusplus
}
#endif
