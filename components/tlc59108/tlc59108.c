#include "esp_check.h"
#include "tlc59108.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

// Breathing animation state
static bool breathing_enabled = false;
static float breathing_speed_hz = 0.5f;   // default
static float breathing_time = 0.0f;

#define TLC_POWER_GPIO  GPIO_NUM_10
static bool power_gpio_initialized = false;

#define TLC_RESET_GPIO  GPIO_NUM_15
static bool reset_gpio_initialized = false;


#define TLC_ADDR 0x40
#define REG_MODE1   0x00
#define REG_MODE2   0x01
#define REG_PWM0    0x02
#define REG_LEDOUT0 0x0C
#define REG_LEDOUT1 0x0D

static const uint8_t amber_channels[] = {0, 1, 2, 3};
static const uint8_t white_channels[] = {4, 5, 6, 7};
static const uint8_t all_channels[]   = {0, 1, 2, 3, 4, 5, 6, 7};

static const char *TAG = "TLC9108";

void tlc_reset_init(void)
{
    // Configure reset pin HIGH immediately
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << TLC_RESET_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,   // <-- enable internal pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&io_conf);

    gpio_set_level(TLC_RESET_GPIO, 1);  // ensure reset HIGH
}

uint8_t percentage_to_8bit(uint8_t percentage)
{
    return (percentage * 255 + 50) / 100;   // +50 for rounding
}

void tlc_power_set(bool on)
{
    if (!power_gpio_initialized) {
        tlc_power_init();
    }

    gpio_set_level(TLC_POWER_GPIO, on ? 1 : 0);
}


void tlc_reset_pulse(void)
{
    if (!reset_gpio_initialized) {
        tlc_reset_init();
    }

    // Active LOW reset
    gpio_set_level(TLC_RESET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(1));     // Minimum 500 µs according to datasheet
    gpio_set_level(TLC_RESET_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(1));     // Allow chip to stabilize
}


void tlc_power_init(void)
{
    if (power_gpio_initialized) return;

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << TLC_POWER_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io_conf);
    power_gpio_initialized = true;
}

void tlc_boot_led_sequence(void)
{
    const int step = 20;
    const int max_val = 255;
    const int num_channels = 6;

    // Al channels are off at the start
    for (int ch = 0; ch < num_channels; ch++) {
        tlc59108_set_pwm(ch, 0);
    }

    int val = 0;

    // Ramp Up Cycle
    while (1) {
        // Increase brightness
        val += step;
        if (val > max_val) {
            val = max_val;
        }

        // Apply this brightness to each channel one by one
        for (int ch = 0; ch < num_channels; ch++) {
            tlc59108_set_pwm(ch, val);
            vTaskDelay(pdMS_TO_TICKS(50));   
        }

        // Stop ramping up once we've reached max
        if (val == max_val) {
            break;
        }
    }

    // Ramp Down Cycle
    while (1) {
        // Decrease brightness
        val -= step;
        if (val < 0) {
            val = 0;
        }

        // Apply this brightness to each channel one by one
        for (int ch = 0; ch < num_channels; ch++) {
            tlc59108_set_pwm(ch, val);
            vTaskDelay(pdMS_TO_TICKS(50));   
        }

        // Stop when fully off
        if (val == 0) {
            break;
        }
    }

    // Ensure all off at the end
    for (int ch = 0; ch < num_channels; ch++) {
        tlc59108_set_pwm(ch, 0);
    }
}



static i2c_master_dev_handle_t tlc_dev;

static esp_err_t tlc_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = { reg, value };
    esp_err_t err = i2c_master_transmit(tlc_dev, data, 2, -1);
    if (err != ESP_OK) {
        printf("TLC write failed: reg=0x%02X val=0x%02X err=%s\n",
               reg, value, esp_err_to_name(err));
    }
    return err;
}

esp_err_t tlc_read_reg(uint8_t reg, uint8_t *out_value)
{
    return i2c_master_transmit_receive(tlc_dev, &reg, 1, out_value, 1, -1);
}

esp_err_t tlc59108_init(i2c_master_bus_handle_t bus)
{
    // Ensure the device is powered before reset
    tlc_power_set(true);
    vTaskDelay(pdMS_TO_TICKS(20));
    // Reset the device
    //tlc_reset_pulse();
    //vTaskDelay(pdMS_TO_TICKS(5));

    i2c_device_config_t devcfg = {
        .device_address = TLC_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &devcfg, &tlc_dev));

    // MODE1 = normal
    tlc_write_reg(REG_MODE1, 0x00);
    // MODE2 = totem-pole, non-inverted
    tlc_write_reg(REG_MODE2, 0x20);
    // LEDOUT = PWM mode
    tlc_write_reg(REG_LEDOUT0, 0xAA);
    tlc_write_reg(REG_LEDOUT1, 0xAA);

    for (int i = 0; i < 8; i++)
        tlc_write_reg(REG_PWM0 + i, 0);

    return ESP_OK;
}

esp_err_t tlc59108_set_pwm(uint8_t channel, uint8_t value)
{
    if (channel > 7) return ESP_ERR_INVALID_ARG;
    return tlc_write_reg(REG_PWM0 + channel, value);
}

esp_err_t tlc59108_set_group_pwm(const uint8_t *channels, uint8_t count, uint8_t value)
{
    for (int i = 0; i < count; i++)
        tlc59108_set_pwm(channels[i], value);
    return ESP_OK;
}

void tlc_dump_registers(void)
{
    uint8_t val;

    printf("----- TLC59108 Register Dump -----\n");

    for (int reg = 0x00; reg <= 0x0D; reg++) {
        if (tlc_read_reg(reg, &val) == ESP_OK) {
            printf("Reg 0x%02X = 0x%02X\n", reg, val);
        } else {
            printf("Reg 0x%02X = <ERROR>\n", reg);
        }
    }

    printf("----------------------------------\n");
}

esp_err_t tlc_set_all_brightness(uint8_t value)
{
    return tlc59108_set_group_pwm(all_channels, 8, value);
}

esp_err_t tlc_set_all_brightness_percentage(uint8_t percentage)
{
    uint8_t value = percentage_to_8bit(percentage);
    return tlc59108_set_group_pwm(all_channels, 8, value);
}

esp_err_t tlc_set_white_brightness(uint8_t value)
{
    return tlc59108_set_group_pwm(white_channels, 4, value);
}

esp_err_t tlc_set_amber_brightness(uint8_t value)
{
    return tlc59108_set_group_pwm(amber_channels, 4, value);
}

void tlc_breathe_update(float dt_seconds)
{
    if (!breathing_enabled)
        return;

    const float two_pi = 6.28318530718f;

    // breathing waveform: 0...1
    float wave = 0.5f * (1.0f + sinf(two_pi * breathing_speed_hz * breathing_time));

    uint8_t brightness = (uint8_t)(wave * 100.0f);

    tlc_set_all_brightness(brightness);

    breathing_time += dt_seconds;
}

void tlc_breathe_init(float speed_hz)
{
    breathing_speed_hz = speed_hz;
    breathing_time = 0.0f;
    breathing_enabled = true;
}

void tlc_set_breathing_enabled(bool enabled)
{
    breathing_enabled = enabled;
}

void tlc_set_channel_brightness(uint8_t channel, uint8_t value)
{
    if (channel > 7) return;
    tlc_write_reg(REG_PWM0 + channel, value);
    ESP_LOGI(TAG, "Set channel %d to %d", channel, value);
}

void tlc_set_group_brightness(uint8_t *channels, int count, uint8_t value)
{
    for(int i = 0; i < count; i++)
        tlc_set_channel_brightness(channels[i], value);
    ESP_LOGI(TAG, "Set group %d to %d", channels[0], value);
}