idf_component_register(
    SRCS "tlc59108.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2c esp_driver_gpio driver
)