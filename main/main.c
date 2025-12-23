#include "driver/i2c_master.h"
#include "tlc59108.h"
#include "tc74.h"
#include "esp_log.h"
#include "zigbee_app.h"
#include "esp_zigbee_core.h"
#include "esp_check.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//static const char *TAG = "I2C_SCAN";
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

    while (1) {
        if (!zigbee_is_connected()) {
            tlc_breathe_update(0.03f);
        }
        vTaskDelay(loop_delay);
    }
}


void temperature_task(void *arg)
{
    const TickType_t temp_delay = pdMS_TO_TICKS(2000);

    while (1) {
        if (zigbee_is_connected()) {
            int8_t t;
            if (tc74_read_temperature(&t) == ESP_OK) {
                ESP_LOGI("TEMP", "Temp = %d °C", t);
            }
            vTaskDelay(temp_delay);
        } else {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}



void app_main(void)
{

    tlc_reset_init();
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

    ESP_ERROR_CHECK(nvs_flash_init());

    // initialize devices
    tlc59108_init(bus);
    tlc_dump_registers();
    tc74_init(bus);

    // Turn LED driver ON
    tlc_power_set(true);   
    tlc_reset_init();

    //tlc_test_channels();
    //tlc_boot_led_sequence();
    led_boot_trail_spin_animation();

    // Start breathing to indicate "not yet joined"
    tlc_breathe_init(0.2f);  // 0.25 Hz = slow breathing

    const float dt = 0.01f;  // 10 ms tick

    scan_i2c(bus);

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_LOGI("MAIN", "Starting NVS flash init");
    ESP_ERROR_CHECK(nvs_flash_init());

    LoadFromNVS();

    ESP_LOGI("MAIN", "Starting ESP Zigbee Config");
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    /* Start Zigbee stack task */
    ESP_LOGI("MAIN", "Starting ESP Zigbee Task");
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
    


    xTaskCreate(led_task,"led_task",2048,NULL,5,NULL);

    xTaskCreate(temperature_task,"temperature_task",2048,NULL,4,NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100)); // yield so IDLE resets WDT
       
    }
}
