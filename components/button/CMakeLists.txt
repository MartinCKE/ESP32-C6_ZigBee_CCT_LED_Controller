idf_component_register(
    SRCS "button.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_timer freertos zigbee_app tlc59108
)
