#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

esp_err_t tc74_init(i2c_master_bus_handle_t bus);
esp_err_t tc74_read_temperature(int8_t *temp_out);
