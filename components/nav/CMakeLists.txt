idf_component_register(
    SRCS "nav.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos driver esp_timer
)
