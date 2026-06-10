idf_component_register(
    SRCS "zigbee_app.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES tlc59108 nvs_flash app_update
    REQUIRES espressif__esp-zigbee-lib events status_led wakeup_light
    )
