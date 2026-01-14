#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

esp_err_t ms8607_init(i2c_master_bus_handle_t bus);
esp_err_t ms8607_read_temperature_humidity(float *temp_c, float *rh);