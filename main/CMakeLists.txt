idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES nvs_flash app_update
    REQUIRES tlc59108 esp_driver_i2c zigbee_app ms8607 button events status_led touch_sensor
)
