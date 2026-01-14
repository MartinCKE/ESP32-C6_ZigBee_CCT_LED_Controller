idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES nvs_flash
    REQUIRES tlc59108 tc74 esp_driver_i2c driver zigbee_app espressif__esp-zigbee-lib ms8607
)