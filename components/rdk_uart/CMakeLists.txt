idf_component_register(
    SRCS "rdk_uart.c"
    INCLUDE_DIRS "include"
    REQUIRES driver imu freertos control
)
