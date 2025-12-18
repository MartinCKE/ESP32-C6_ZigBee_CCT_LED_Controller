idf_component_register(
    SRCS "zigbee_app.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES tlc59108 tc74  
    REQUIRES espressif__esp-zigbee-lib int_temp_sensor_driver 
    )