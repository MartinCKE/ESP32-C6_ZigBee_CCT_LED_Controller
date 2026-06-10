#include "driver/i2c_master.h"
#include "tlc59108.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "zigbee_app.h"
#include "esp_zigbee_core.h"
#include "nvs_flash.h"
#include "ms8607.h"
#include "button.h"
#include "events.h"
#include "status_led.h"
#include "touch_sensor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wakeup_light.h"

static const char *TAG = "MAIN";

#define TOUCH_CT_TRANSITION_MS 40
#define TOUCH_BRI_TRANSITION_MS 20

static uint8_t get_bri(void *ctx) { (void)ctx; return get_current_logical_brightness_from_outputs(); }
static uint16_t get_ct(void *ctx) { (void)ctx; return tlc_get_current_ct_mired(); }

void scan_i2c(i2c_master_bus_handle_t bus)
{
    ESP_LOGD(TAG, "Starting I2C scan...");

    for (int addr = 1; addr < 127; addr++) {
        esp_err_t ret = i2c_master_probe(bus, addr, 50);  // timeout = 50us
        if (ret == ESP_OK) {
            ESP_LOGD(TAG, "Found I2C device at 0x%02X", addr);
        }
    }
    ESP_LOGD(TAG, "I2C scan finished.");
}

void led_task(void *arg)
{
    const TickType_t loop_delay = pdMS_TO_TICKS(30);
    bool connection_confirmed = false;
    while (1) {
        if (!zigbee_is_connected()) {
            tlc_breathe_update(0.03f);
        }
        else {
            if (!connection_confirmed) {
                connection_confirmed = true;
            }
        }
        vTaskDelay(loop_delay);
    }
}


void temperature_task(void *arg) {
    const TickType_t temp_delay = pdMS_TO_TICKS(5000);
    float t_ms = -1.0f;
    float rh = -1.0f;
    while (1) {
        esp_err_t ret_ms   = ms8607_read_temperature_humidity(&t_ms, &rh);
        if (ret_ms == ESP_OK) {
            ESP_LOGD(TAG, "MS8607: %.2f C, %.1f %%RH", t_ms, rh);
            if (zigbee_is_connected()) {
                zigbee_update_temp_rh(t_ms, rh);
            }

        } else {
            ESP_LOGW(TAG, "MS8607 read failed: %s", esp_err_to_name(ret_ms));
        }
        vTaskDelay(temp_delay);
    }
}

static void button_task(void *arg)
{
    (void)arg;

    button_config_t cfg = {
        .gpio_num = 18,
        .active_low = true,
        .enable_internal_pullup = true,
        .debounce_ms = 50,
        .long_press_ms = 1500,
        .double_press_window_ms = 600,
    };

    ESP_ERROR_CHECK(button_init(&cfg, xTaskGetCurrentTaskHandle()));

    while (1) {
        button_event_t ev;
        if (button_wait_event(&ev, portMAX_DELAY)) {
            switch (ev) {
                case BUTTON_EVENT_SINGLE_PRESS:
                    ESP_LOGD(TAG, "Single press");
                    status_led_notify_button_press();
                    light_toggle_handler();
                    break;
                case BUTTON_EVENT_DOUBLE_PRESS:
                    events_post(APP_EVENT_ZIGBEE_FACTORY_RESET);
                    ESP_LOGD(TAG, "Double press");
                    break;
                case BUTTON_EVENT_LONG_PRESS:
                    ESP_LOGD(TAG, "Long press");
                    break;
                default:
                    break;
            }
        }
    }
}

static bool touch_user_interaction_cb(void *user_ctx)
{
    (void)user_ctx;

    if (wakeup_is_running()) {
        ESP_LOGD(TAG, "Touch press: stopping wakeup (freeze)");
        zigbee_wakeup_cancel_and_report();
        return true;
    }

    return false;
}



static void act_toggle_power(void *ctx)
{
    light_toggle_handler();
}

static void act_apply_brightness(uint8_t level, void *ctx)
{
    (void)ctx;
    if (level == 0) level = 1; // avoid zero brightness 
    current_brightness = level;
    light_set_on_state(true);
    ESP_LOGD(TAG, "Brightness apply: level=%u mired=%u", (unsigned)level, (unsigned)mired);
    tlc_set_logical_brightness_smooth_ms((uint8_t)current_brightness, (uint16_t)mired, TOUCH_BRI_TRANSITION_MS);
}

static void act_commit_brightness(uint8_t level, void *ctx)
{
    (void)ctx;
    current_brightness = level;
    SaveToNVS();
    zigbee_set_level_and_report(level);
    zigbee_set_ct_and_report(mired);
}

static void act_apply_ct(uint16_t mired_new, void *ctx)
{
    (void)ctx;
    mired = mired_new;
    ESP_LOGD(TAG, "CT apply: mired=%u brightness=%u", (unsigned)mired_new, (unsigned)current_brightness);
    tlc_set_ct_mired_smooth((uint16_t)mired, TOUCH_CT_TRANSITION_MS);
}

static void act_commit_ct(uint16_t mired_new, void *ctx)
{
    (void)ctx;
    mired = mired_new;
    ESP_LOGD(TAG, "CT commit: mired=%u", (unsigned)mired_new);
    SaveToNVS();
    zigbee_set_ct_and_report(mired_new);
    zigbee_set_level_and_report(current_brightness);
}


void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);

    status_led_config_t led_cfg = {
        .gpio_red = 19,
        .gpio_yellow = 20,
        .gpio_green = 21,
        .active_low = false,  
        .task_stack_bytes = 0,
        .task_priority = 0,
        .task_name = NULL,
    };
    status_led_start(&led_cfg);
    status_led_boot_ok_start(); 

    touch_sensor_config_t cfg = {
        .gpio_num = 4,
        .active_low = false,
        .enable_pullup = false,
        .enable_pulldown = false,
        .debounce_ms = 40,
        .hold_threshold_ms = 450,
        .double_tap_window_ms = 500,
        .initial_hold_mode = TOUCH_HOLD_MODE_BRIGHTNESS,
        .actions = {
            .toggle_power      = act_toggle_power,
            .apply_brightness  = act_apply_brightness,
            .apply_ct_mired    = act_apply_ct,
            .commit_brightness = act_commit_brightness,
            .commit_ct_mired   = act_commit_ct,
            .get_brightness    = get_bri,
            .get_ct_mired      = get_ct,
            .user_interaction  = touch_user_interaction_cb,
        },
        .user_ctx = NULL,
    };
    ESP_ERROR_CHECK(touch_sensor_start(&cfg));


    // Initiate i2c bus
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = GPIO_NUM_7, 
        .sda_io_num = GPIO_NUM_6,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };

    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    scan_i2c(bus);
    ESP_ERROR_CHECK(ms8607_init(bus));
    vTaskDelay(pdMS_TO_TICKS(300)); // initial delay to allow Zigbee to connect first
    // Initiate LED driver
    tlc_reset_init();
    tlc59108_init(bus);
    tlc_power_set(true);   
    tlc_reset_init();
    led_boot_trail_spin_animation();

    // Start breathing to indicate "not yet joined"
    //tlc_breathe_init(0.2f);  // 0.25 Hz = slow breathing

    events_init();

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_LOGD(TAG, "Starting NVS flash init");
    ESP_ERROR_CHECK(nvs_flash_init());
    esp_err_t ota_mark_ret = esp_ota_mark_app_valid_cancel_rollback();
    if (ota_mark_ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA app marked valid");
        status_led_ota_success_start();
    } else {
        ESP_LOGD(TAG, "OTA app validation mark skipped: %s", esp_err_to_name(ota_mark_ret));
    }

    LoadFromNVS(); // Loading last known brightness / color temperature
    tlc_set_logical_brightness_smooth((uint8_t)current_brightness, (uint16_t)mired);
    light_set_on(true, true);

    ESP_LOGD(TAG, "Starting ESP Zigbee Config");
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    /* Start Zigbee stack task */
    ESP_LOGD(TAG, "Starting ESP Zigbee Task");
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
    
    /* Start LED task */
    xTaskCreate(led_task,"led_task",2048,NULL,5,NULL);

    /* Start Temp Sensor task */
    xTaskCreate(temperature_task,"temperature_task",4096,NULL,4,NULL);

    /* Start Button task */
    xTaskCreate(button_task, "button_task", 4096, NULL, 10, NULL);

    xTaskCreate(zigbee_event_task, "zigbee_event_task", 4096, NULL, 6, NULL);

    vTaskDelay(pdMS_TO_TICKS(400)); // Wait for tasks to settle

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}
