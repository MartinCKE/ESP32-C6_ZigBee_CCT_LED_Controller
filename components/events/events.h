#pragma once
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    APP_EVENT_NONE = 0,
    APP_EVENT_ZIGBEE_FACTORY_RESET,
    // add more later (LED patterns, join request, etc.)
} app_event_t;

/**
 * Create the queue (call once from app_main).
 */
bool events_init(void);

/**
 * Send an event (safe from task context).
 */
bool events_post(app_event_t ev);

/**
 * Receive an event (blocking).
 */
bool events_wait(app_event_t *out_ev, TickType_t timeout);

/**
 * Optional: for modules that need direct access.
 */
QueueHandle_t events_get_queue(void);

#ifdef __cplusplus
}
#endif
