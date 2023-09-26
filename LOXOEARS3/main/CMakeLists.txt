set(srcs "I2S-Driver/I2S.c"
    "Application/MDF.c"
    "Application/CanTask.c"
    "CAN/can.c"
    "led_strip/led_strip_rmt_ws2812.c"
    "main.c")

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ".")