#include "driver/i2c_master.h"
#include "tlc59108.h"
#include "tc74.h"
#include "esp_log.h"
#include "zigbee_app.h"
#include "esp_zigbee_core.h"
#include "esp_check.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "nvs_flash.h"
#include "ms8607.h"
#include "button.h"
#include "events.h"
#include "status_led.h"
#include "touch_sensor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAIN";

static uint8_t get_bri(void *ctx) { (void)ctx; return (uint8_t)current_brightness; }
static uint16_t get_ct(void *ctx) { (void)ctx; return (uint16_t)mired; }


// I2C Scanner function
void scan_i2c(i2c_master_bus_handle_t bus)
{
    ESP_LOGI(TAG, "Starting I2C scan...");

    for (int addr = 1; addr < 127; addr++) {
        esp_err_t ret = i2c_master_probe(bus, addr, 50);  // timeout = 50us
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✔ Found device at address: 0x%02X", addr);
        }
    }
    ESP_LOGI(TAG, "I2C scan finished.");
}

void led_task(void *arg)
{
    const TickType_t loop_delay = pdMS_TO_TICKS(30);
    bool connection_confirmed = false;
    //bool connected = false;
    //int32_t counter = 0;
    while (1) {
        //counter++;
        //if (!connected) {
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
    const TickType_t temp_delay = pdMS_TO_TICKS(2000);
    float t_ms = -1.0f;
    float rh = -1.0f;
    while (1) {
        //float t_ms   = -1.0f;
        //float rh     = -1.0f;

        esp_err_t ret_ms   = ms8607_read_temperature_humidity(&t_ms, &rh);

        // Log all values in one line (with error info if something failed)
        if (ret_ms == ESP_OK) {
            ESP_LOGI(TAG, "MS8607: %.2f °C, %.1f %%RH", t_ms, rh);
            if (zigbee_is_connected()) {
                zigbee_update_temp_rh(t_ms, rh);
            }

        } else {
            ESP_LOGI(TAG, "MS8607: %s (%.2f °C, %.1f %%RH)",
                     esp_err_to_name(ret_ms),   t_ms, rh);

            if (ret_ms != ESP_OK) {
                ESP_LOGE(TAG, "MS8607 read failed: %s", esp_err_to_name(ret_ms));
            }
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
        .debounce_ms = 30,
        .long_press_ms = 2000,
        .double_press_window_ms = 500,
    };

    ESP_ERROR_CHECK(button_init(&cfg, xTaskGetCurrentTaskHandle()));

    while (1) {
        button_event_t ev;
        if (button_wait_event(&ev, portMAX_DELAY)) {
            switch (ev) {
                case BUTTON_EVENT_SINGLE_PRESS:
                    ESP_LOGI(TAG, "Single press");
                    status_led_notify_button_press();
                    light_toggle_handler();
                    break;
                case BUTTON_EVENT_DOUBLE_PRESS:
                    events_post(APP_EVENT_ZIGBEE_FACTORY_RESET);
                    ESP_LOGI(TAG, "Double press");
                    break;
                case BUTTON_EVENT_LONG_PRESS:
                    ESP_LOGI(TAG, "Long press");
                    break;
                default:
                    break;
            }
        }
    }
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
    led_apply_brightness_and_ct(current_brightness, mired);
}

static void act_commit_brightness(uint8_t level, void *ctx)
{
    (void)ctx;
    // current_brightness already set by apply, but keep it explicit:
    current_brightness = level;
    SaveToNVS();
    zigbee_set_level_and_report(level);
}

static void act_apply_ct(uint16_t mired_new, void *ctx)
{
    (void)ctx;
    mired = mired_new;
    //led_apply_brightness_and_ct(current_brightness, mired);
    ESP_LOGI("MAIN", "CT apply: mired_new=%u", (unsigned)mired_new);
    //led_color_temperature_control(current_brightness, mired_new);
    tlc_set_ct_mired((uint16_t)mired); // 400ms feels nice
}

static void act_commit_ct(uint16_t mired_new, void *ctx)
{
    (void)ctx;
    mired = mired_new;
    ESP_LOGI("MAIN", "CT commit: mired_new=%u", (unsigned)mired_new);
    tlc_set_ct_mired((uint16_t)mired); // 400ms feels nice
    //led_color_temperature_control(current_brightness, mired_new);
    SaveToNVS();
    zigbee_set_ct_and_report(mired_new);
}




void app_main(void)
{
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
    status_led_boot_ok_start();  // plays 3s r<->y<->g ramp animation

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
    // Initialize temperature sensor
    tc74_init(bus);
    ESP_ERROR_CHECK(ms8607_init(bus));
    vTaskDelay(pdMS_TO_TICKS(300)); // initial delay to allow Zigbee to connect first
    // Initiate LED driver
    tlc_reset_init();
    tlc59108_init(bus);
    tlc_power_set(true);   
    //tlc_dump_registers();
    tlc_reset_init();
    led_boot_trail_spin_animation();

    // Start breathing to indicate "not yet joined"
    //tlc_breathe_init(0.2f);  // 0.25 Hz = slow breathing

    events_init();
        
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_LOGI("MAIN", "Starting NVS flash init");
    ESP_ERROR_CHECK(nvs_flash_init());

    LoadFromNVS(); // Loading last known brightness / color temperature

    ESP_LOGI("MAIN", "Starting ESP Zigbee Config");
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    /* Start Zigbee stack task */
    ESP_LOGI("MAIN", "Starting ESP Zigbee Task");
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
        //float t;
        //if (tc74_read_temperature(&t) == ESP_OK) {
        //    ESP_LOGI(TAG, "while loop Temp = %.1f °C", t);
        //    }
        vTaskDelay(pdMS_TO_TICKS(100)); // yield so IDLE resets WDT
    }
}
