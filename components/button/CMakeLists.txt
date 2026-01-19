idf_component_register(
    SRCS "button.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer freertos zigbee_app tlc59108
)
