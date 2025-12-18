#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"



esp_err_t tlc59108_init(i2c_master_bus_handle_t bus);
esp_err_t tlc59108_set_pwm(uint8_t channel, uint8_t value);
esp_err_t tlc59108_set_group_pwm(const uint8_t *channels, uint8_t count, uint8_t value);
esp_err_t tlc_read_reg(uint8_t reg, uint8_t *out_value);

esp_err_t tlc_set_all_brightness(uint8_t value);
esp_err_t tlc_set_white_brightness(uint8_t value);
esp_err_t tlc_set_amber_brightness(uint8_t value);

esp_err_t tlc_set_all_brightness_percentage(uint8_t percentage);


void tlc_power_init(void);
void tlc_power_set(bool on);

void tlc_reset_init(void);
void tlc_reset_pulse(void);

void tlc_dump_registers(void);
void tlc_reset_init(void);

void tlc_boot_led_sequence(void);

void tlc_breathe_init(float speed_hz);
void tlc_breathe_update(float dt_seconds);
void tlc_set_breathing_enabled(bool enabled);

void tlc_set_channel_brightness(uint8_t channel, uint8_t value);
void tlc_set_group_brightness(uint8_t *channels, int count, uint8_t value);

uint8_t percentage_to_8bit(uint8_t percentage);

