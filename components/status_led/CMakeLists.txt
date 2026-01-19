idf_component_register(
    SRCS "status_led.c"
    INCLUDE_DIRS "."
    REQUIRES freertos esp_driver_gpio esp_driver_ledc
)
