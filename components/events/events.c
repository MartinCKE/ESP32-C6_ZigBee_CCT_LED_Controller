#include "events.h"

static QueueHandle_t s_evt_q = NULL;

bool events_init(void)
{
    if (s_evt_q) return true;
    s_evt_q = xQueueCreate(8, sizeof(app_event_t));
    return (s_evt_q != NULL);
}

bool events_post(app_event_t ev)
{
    if (!s_evt_q) return false;
    return (xQueueSend(s_evt_q, &ev, 0) == pdTRUE);
}

bool events_wait(app_event_t *out_ev, TickType_t timeout)
{
    if (!s_evt_q || !out_ev) return false;
    return (xQueueReceive(s_evt_q, out_ev, timeout) == pdTRUE);
}

QueueHandle_t events_get_queue(void)
{
    return s_evt_q;
}
