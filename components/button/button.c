#include "button.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "zigbee_app.h"
#include "tlc59108.h"

static const char *TAG = "button";

typedef struct {
    button_config_t cfg;
    TaskHandle_t notify_task;

    // State machine variables (used by the task only)
    int last_level;
    int stable_level;
    int64_t last_isr_edge_us;

    int64_t press_start_us;
    bool pressed;

    bool long_reported;

    bool waiting_second;
    int64_t first_release_us;

    button_event_t pending_event;
} button_ctx_t;

static button_ctx_t s_btn = {0};

static inline int read_level(void)
{
    return gpio_get_level((gpio_num_t)s_btn.cfg.gpio_num);
}

static inline bool is_pressed_level(int level)
{
    return s_btn.cfg.active_low ? (level == 0) : (level == 1);
}

static inline int64_t now_us(void)
{
    return esp_timer_get_time();
}

static void IRAM_ATTR button_isr_handler(void *arg)
{
    (void)arg;
    int64_t t = esp_timer_get_time();
    // very lightweight edge debounce in ISR to avoid floods
    // (real debounce handled in task)
    if (s_btn.last_isr_edge_us == 0 || (t - s_btn.last_isr_edge_us) > (int64_t)(s_btn.cfg.debounce_ms) * 1000) {
        s_btn.last_isr_edge_us = t;
        BaseType_t hp_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(s_btn.notify_task, &hp_task_woken);
        if (hp_task_woken) {
            portYIELD_FROM_ISR();
        }
    }
}

static void set_pending_event(button_event_t ev)
{
    s_btn.pending_event = ev;
}

static bool pop_pending_event(button_event_t *out_event)
{
    if (s_btn.pending_event == BUTTON_EVENT_NONE) {
        return false;
    }
    *out_event = s_btn.pending_event;
    s_btn.pending_event = BUTTON_EVENT_NONE;
    return true;
}

esp_err_t button_init(const button_config_t *cfg, TaskHandle_t notify_task)
{
    if (!cfg || notify_task == NULL) return ESP_ERR_INVALID_ARG;

    s_btn = (button_ctx_t){0};
    s_btn.cfg = *cfg;
    s_btn.notify_task = notify_task;
    s_btn.pending_event = BUTTON_EVENT_NONE;

    gpio_config_t io = {0};
    io.pin_bit_mask = 1ULL << cfg->gpio_num;
    io.mode = GPIO_MODE_INPUT;
    io.intr_type = GPIO_INTR_ANYEDGE;

    if (cfg->active_low && cfg->enable_internal_pullup) {
        io.pull_up_en = GPIO_PULLUP_ENABLE;
        io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    } else {
        io.pull_up_en = GPIO_PULLUP_DISABLE;
        io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    }

    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) return err;

    // Install ISR service
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = gpio_isr_handler_add((gpio_num_t)cfg->gpio_num, button_isr_handler, NULL);
    if (err != ESP_OK) return err;

    // Initialize state
    s_btn.last_level = read_level();
    s_btn.stable_level = s_btn.last_level;
    s_btn.pressed = is_pressed_level(s_btn.stable_level);
    s_btn.press_start_us = s_btn.pressed ? now_us() : 0;
    s_btn.long_reported = false;
    s_btn.waiting_second = false;
    s_btn.first_release_us = 0;

    ESP_LOGD(TAG, "Button initialized on GPIO%d (active_%s)",
             cfg->gpio_num, cfg->active_low ? "low" : "high");

    return ESP_OK;
}

esp_err_t button_deinit(void)
{
    gpio_isr_handler_remove((gpio_num_t)s_btn.cfg.gpio_num);
    // Don't uninstall the ISR service globally since other code may use it.
    s_btn = (button_ctx_t){0};
    return ESP_OK;
}

/**
 * Internal: run the state machine until:
 * - we produce an event, or
 * - we need to wait for more edges/time.
 */
static void button_process_state_machine(void)
{
    const int64_t t = now_us();
    const int level = read_level();

    // Debounce: accept new stable state only if level stays for debounce_ms.
    // Simple method: when edge occurs, wait debounce_ms before trusting.
    // Here, we just check time since last_isr_edge_us (set by ISR).
    if (s_btn.last_isr_edge_us != 0) {
        int64_t since_edge_us = t - s_btn.last_isr_edge_us;
        if (since_edge_us >= (int64_t)s_btn.cfg.debounce_ms * 1000) {
            // consider stable
            s_btn.stable_level = level;
            s_btn.last_isr_edge_us = 0; // consume this edge window
        } else {
            // not stable yet
            return;
        }
    }

    bool pressed_now = is_pressed_level(s_btn.stable_level);

    // Handle transitions
    if (!s_btn.pressed && pressed_now) {
        // released -> pressed
        s_btn.pressed = true;
        s_btn.press_start_us = t;
        s_btn.long_reported = false;
        return;
    }

    if (s_btn.pressed && !pressed_now) {
        // pressed -> released
        s_btn.pressed = false;

        // If we already reported long press, do nothing else for this press
        if (s_btn.long_reported) {
            s_btn.long_reported = false;
            s_btn.waiting_second = false;
            return;
        }

        // short press released: handle single/double
        if (!s_btn.waiting_second) {
            s_btn.waiting_second = true;
            s_btn.first_release_us = t;
        } else {
            // second press completed within window?
            int64_t dt_ms = (t - s_btn.first_release_us) / 1000;
            if (dt_ms <= (int64_t)s_btn.cfg.double_press_window_ms) {
                set_pending_event(BUTTON_EVENT_DOUBLE_PRESS);
                s_btn.waiting_second = false;
            } else {
                // window expired: previous becomes single, start waiting again
                set_pending_event(BUTTON_EVENT_SINGLE_PRESS);
                s_btn.first_release_us = t;
                s_btn.waiting_second = true;
            }
        }
        return;
    }

    // No transition: check for long press while held
    if (s_btn.pressed && !s_btn.long_reported) {
        int64_t held_ms = (t - s_btn.press_start_us) / 1000;
        if (held_ms >= (int64_t)s_btn.cfg.long_press_ms) {
            s_btn.long_reported = true;
            set_pending_event(BUTTON_EVENT_LONG_PRESS);
            s_btn.waiting_second = false; // long press cancels click counting
            return;
        }
    }

    // While released: if waiting for second press, check timeout -> single press
    if (!s_btn.pressed && s_btn.waiting_second) {
        int64_t dt_ms = (t - s_btn.first_release_us) / 1000;
        if (dt_ms >= (int64_t)s_btn.cfg.double_press_window_ms) {
            set_pending_event(BUTTON_EVENT_SINGLE_PRESS);
            s_btn.waiting_second = false;
            return;
        }
    }
}

bool button_wait_event(button_event_t *out_event, TickType_t timeout_ticks)
{
    if (!out_event) return false;

    // First, if there is already a pending event, return it immediately.
    if (pop_pending_event(out_event)) return true;

    // We need to handle time-based decisions (double-click timeout, long-press),
    // so we cannot wait forever on notifications only.
    // We'll wake on ISR notify OR periodically to evaluate timeouts.
    const TickType_t periodic_ticks = pdMS_TO_TICKS(20);

    TickType_t waited = 0;
    while (true) {
        // Process state machine each loop
        button_process_state_machine();
        if (pop_pending_event(out_event)) return true;

        TickType_t this_wait = periodic_ticks;
        if (timeout_ticks != portMAX_DELAY) {
            if (waited >= timeout_ticks) return false;
            TickType_t remaining = timeout_ticks - waited;
            if (remaining < this_wait) this_wait = remaining;
        }

        // Wait for ISR notification or timeout
        // ulTaskNotifyTake clears the count when pdTRUE.
        uint32_t got = ulTaskNotifyTake(pdTRUE, this_wait);
        (void)got;

        if (timeout_ticks != portMAX_DELAY) {
            waited += this_wait;
        }
    }
}

