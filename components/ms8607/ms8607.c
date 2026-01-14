#include "ms8607.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include <string.h>
#define TAG "MS8607"

// I2C addresses (7-bit)
#define MS8607_ADDR_PT  0x76   // pressure + temperature
#define MS8607_ADDR_RH  0x40   // humidity

// PT commands (MS5637-like)
#define PT_CMD_RESET        0x1E
#define PT_CMD_ADC_READ    0x00
#define PT_CMD_CONV_D2     0x58    // temperature, OSR=4096
#define PT_CMD_PROM_BASE   0xA0

// RH commands (HTU21D-like)
#define RH_CMD_SOFT_RESET  0xFE
#define RH_CMD_TRIG_RH     0xF5

static i2c_master_dev_handle_t pt_dev = NULL;
static i2c_master_dev_handle_t rh_dev = NULL;

static uint16_t prom[8];

// ---------------- CRC4 for PROM ----------------
static uint8_t crc4(uint16_t *n_prom)
{
    uint16_t n_rem = 0;
    uint16_t crc_read = n_prom[7] & 0x000F;

    n_prom[7] &= 0xFFF0;

    for (int cnt = 0; cnt < 16; cnt++) {
        n_rem ^= (cnt & 1) ? (n_prom[cnt >> 1] & 0x00FF)
                           : (n_prom[cnt >> 1] >> 8);
        for (int n = 8; n > 0; n--) {
            n_rem = (n_rem & 0x8000) ? (n_rem << 1) ^ 0x3000
                                     : (n_rem << 1);
        }
    }

    n_prom[7] |= crc_read;
    return (n_rem >> 12) & 0xF;
}

// ---------------- PT helpers ----------------
static esp_err_t pt_read_prom_first(void)
{
    for (int i = 0; i < 8; i++) {
        uint8_t cmd = PT_CMD_PROM_BASE + (i * 2);
        uint8_t rx[2];
        ESP_LOGI(TAG, "%i : Reading PROM cmd 0x%02X", i, cmd);
        esp_err_t ret = i2c_master_transmit(pt_dev, &cmd, 1, -1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "PROM cmd 0x%02X write failed: %s", cmd, esp_err_to_name(ret));
            return ret;
        }

        // Some boards/devices need a tiny settle time between STOP and next START
        vTaskDelay(pdMS_TO_TICKS(1));

        ret = i2c_master_receive(pt_dev, rx, 2, -1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "PROM cmd 0x%02X read failed: %s", cmd, esp_err_to_name(ret));
            return ret;
        }

        prom[i] = (rx[0] << 8) | rx[1];
    }
    /*
    uint16_t prom_copy[8];
    memcpy(prom_copy, prom, sizeof(prom));
    if (crc4(prom_copy) != (prom[7] & 0xF)) {
        ESP_LOGE(TAG, "PROM CRC mismatch");
        return ESP_FAIL;
    }
    */

    return ESP_OK;
}

static uint16_t prom[8];

static esp_err_t pt_read_prom(void)
{
    for (int i = 0; i < 7; i++) {                 // <-- ONLY 7 words
        uint8_t cmd = PT_CMD_PROM_BASE + (i * 2); // 0xA0 .. 0xAC
        uint8_t rx[2];

        ESP_LOGI(TAG, "%d : Reading PROM cmd 0x%02X", i, cmd);

        esp_err_t ret = i2c_master_transmit(pt_dev, &cmd, 1, -1);
        if (ret != ESP_OK) return ret;

        vTaskDelay(pdMS_TO_TICKS(1));

        ret = i2c_master_receive(pt_dev, rx, 2, -1);
        if (ret != ESP_OK) return ret;

        prom[i] = (rx[0] << 8) | rx[1];
    }

    prom[7] = 0; // datasheet CRC code expects this as subsidiary value :contentReference[oaicite:3]{index=3}

    // CRC check below...
    return ESP_OK;
}




static esp_err_t pt_read_temperature(float *temp_c)
{
    uint8_t cmd = PT_CMD_CONV_D2;
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(pt_dev, &cmd, 1, -1),
        TAG, "PT conv start failed"
    );

    vTaskDelay(pdMS_TO_TICKS(10));

    cmd = PT_CMD_ADC_READ;
    uint8_t rx[3];
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit_receive(pt_dev, &cmd, 1, rx, 3, -1),
        TAG, "PT ADC read failed"
    );

    uint32_t D2 = (rx[0] << 16) | (rx[1] << 8) | rx[2];

    int32_t dT = D2 - ((int32_t)prom[5] << 8);
    int32_t TEMP = 2000 + ((int64_t)dT * prom[6]) / (1LL << 23);

    *temp_c = TEMP / 100.0f;
    return ESP_OK;
}

// ---------------- RH helpers ----------------
static esp_err_t rh_read(float *rh)
{
    uint8_t cmd = RH_CMD_TRIG_RH;
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(rh_dev, &cmd, 1, -1),
        TAG, "RH trigger failed"
    );

    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t rx[3];
    ESP_RETURN_ON_ERROR(
        i2c_master_receive(rh_dev, rx, 3, -1),
        TAG, "RH read failed"
    );

    uint16_t raw = ((rx[0] << 8) | rx[1]) & ~0x3;
    *rh = -6.0f + 125.0f * ((float)raw / 65536.0f);
    return ESP_OK;
}

// ---------------- Public API ----------------
esp_err_t ms8607_init(i2c_master_bus_handle_t bus)
{
    i2c_device_config_t cfg = {
        .device_address = MS8607_ADDR_PT,
        .scl_speed_hz = 100000,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus, &cfg, &pt_dev);
    
    ESP_LOGI(TAG, "add PT ret=%s handle=%p", esp_err_to_name(ret), (void*)pt_dev);
    if (ret != ESP_OK) return ret;

    cfg.device_address = MS8607_ADDR_RH;
    ret = i2c_master_bus_add_device(bus, &cfg, &rh_dev);
    ESP_LOGI(TAG, "add RH ret=%s handle=%p", esp_err_to_name(ret), (void*)rh_dev);
    if (ret != ESP_OK) return ret;

    // now PT reset
    uint8_t cmd = PT_CMD_RESET;
    ret = i2c_master_transmit(pt_dev, &cmd, 1, -1);
    ESP_LOGI(TAG, "PT reset ret=%s", esp_err_to_name(ret));
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(3));
    ESP_LOGI(TAG, "OK her");
    return pt_read_prom();
}


esp_err_t ms8607_read_temperature_humidity(float *temp_c, float *rh)
{
    ESP_RETURN_ON_ERROR(pt_read_temperature(temp_c), TAG, "Temp read failed");
    ESP_RETURN_ON_ERROR(rh_read(rh), TAG, "RH read failed");
    return ESP_OK;
}
