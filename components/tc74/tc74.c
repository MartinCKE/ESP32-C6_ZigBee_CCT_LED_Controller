#include "tc74.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TC74_ADDR 0x4C
#define REG_TEMP  0x00
#define REG_CONF  0x01

static const char *TAG = "TC74";
static i2c_master_dev_handle_t tc74_dev;

esp_err_t tc74_init(i2c_master_bus_handle_t bus)
{
    i2c_device_config_t devcfg = {
        .device_address = TC74_ADDR,
        .scl_speed_hz = 100000, // 100 kHz
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &devcfg, &tc74_dev));

    // Read CONFIG register
    uint8_t reg = REG_CONF;
    uint8_t cfg = 0;

    ESP_ERROR_CHECK(
        i2c_master_transmit_receive(tc74_dev, &reg, 1, &cfg, 1, -1)
    );

    // Wake-up if needed
    if (cfg & 0x80) {
        ESP_LOGW(TAG, "TC74 in standby mode, waking up...");
        uint8_t wake_cmd[2] = { REG_CONF, 0x00 };
        ESP_ERROR_CHECK(i2c_master_transmit(tc74_dev, wake_cmd, 2, -1));
        vTaskDelay(pdMS_TO_TICKS(250)); // TC74 requires ≥200ms to wake
    }

    ESP_LOGI(TAG, "TC74 initialized");
    return ESP_OK;
}

esp_err_t tc74_read_temperature_old(float *out_temp)
{
    uint8_t reg = REG_TEMP;
    uint8_t raw = 0;

    ESP_ERROR_CHECK(
        i2c_master_transmit_receive(tc74_dev, &reg, 1, &raw, 1, -1)
    );

    *out_temp = (int8_t)raw;
    return ESP_OK;
}

esp_err_t tc74_read_temperature(float *out_temp_c)
{
    uint8_t reg = REG_TEMP;
    uint8_t raw = 0;

    esp_err_t err = i2c_master_transmit_receive(tc74_dev, &reg, 1, &raw, 1, -1);
    if (err != ESP_OK) return err;

    *out_temp_c = (float)((int8_t)raw);  // TC74 is signed 8-bit °C
    return ESP_OK;
}

