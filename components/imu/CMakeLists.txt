idf_component_register(
    SRCS "imu.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer freertos
)
