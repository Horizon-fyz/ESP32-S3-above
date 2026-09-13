idf_component_register(
    SRCS "gps.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos driver esp_timer
)
