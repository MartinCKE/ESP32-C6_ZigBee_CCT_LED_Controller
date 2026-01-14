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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAIN";

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
    int32_t counter = 0;
    while (1) {
        counter++;
        //if (!connected) {
        if (!zigbee_is_connected()) {
            tlc_breathe_update(0.03f);
        }
        else {
            if (!connection_confirmed) {
                connection_confirmed = true;
                zigbee_connection_confirmed_sequence();
            }
        }
        //if (counter >= 33*10 && !connected) { // approx every second
        //    ESP_LOGI(TAG, "Fake connection done");
        //    connected = true;
        //}
        vTaskDelay(loop_delay);
    }
}



void temperature_task_old_men_funke(void *arg) {
    const TickType_t temp_delay = pdMS_TO_TICKS(2000);
    float t_ms;
    float rh;
    while (1) {
        float t = -1.0f; 
        esp_err_t ret = tc74_read_temperature(&t);
        

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Temp = %.1f °C", t);
            if (zigbee_is_connected()) {
                zigbee_update_temperature(t);
            }
        } else {
            ESP_LOGE(TAG, "I2C Read Failed: %s", esp_err_to_name(ret));
        }
        vTaskDelay(temp_delay);
    }
}

void temperature_task(void *arg) {
    const TickType_t temp_delay = pdMS_TO_TICKS(2000);

    while (1) {
        float t_tc74 = -1.0f;
        float t_ms   = -1.0f;
        float rh     = -1.0f;

        esp_err_t ret_tc74 = tc74_read_temperature(&t_tc74);
        esp_err_t ret_ms   = ms8607_read_temperature_humidity(&t_ms, &rh);

        // Log all values in one line (with error info if something failed)
        if (ret_tc74 == ESP_OK && ret_ms == ESP_OK) {
            ESP_LOGI(TAG, "TC74: %.1f °C | MS8607: %.2f °C, %.1f %%RH",
                     t_tc74, t_ms, rh);
        } else {
            ESP_LOGI(TAG, "TC74: %s (%.1f °C) | MS8607: %s (%.2f °C, %.1f %%RH)",
                     esp_err_to_name(ret_tc74), t_tc74,
                     esp_err_to_name(ret_ms),   t_ms, rh);

            if (ret_tc74 != ESP_OK) {
                ESP_LOGE(TAG, "TC74 read failed: %s", esp_err_to_name(ret_tc74));
            }
            if (ret_ms != ESP_OK) {
                ESP_LOGE(TAG, "MS8607 read failed: %s", esp_err_to_name(ret_ms));
            }
        }

        // Zigbee updates (only send what is valid)
        if (zigbee_is_connected()) {
            if (ret_tc74 == ESP_OK) {
                zigbee_update_temperature(t_tc74);
            }

            // If you add these Zigbee endpoints/clusters later:
            // if (ret_ms == ESP_OK) zigbee_update_temperature_ms8607(t_ms);
            // if (ret_ms == ESP_OK) zigbee_update_humidity(rh);
        }

        vTaskDelay(temp_delay);
    }
}



void app_main(void)
{
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


    scan_i2c(bus);

    const float dt = 0.01f;  // 10 ms tick
    
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

    vTaskDelay(pdMS_TO_TICKS(400)); // Wait for tasks to settle

    while (1) {
        //float t;
        //if (tc74_read_temperature(&t) == ESP_OK) {
        //    ESP_LOGI(TAG, "while loop Temp = %.1f °C", t);
        //    }
        vTaskDelay(pdMS_TO_TICKS(100)); // yield so IDLE resets WDT
    }
}
