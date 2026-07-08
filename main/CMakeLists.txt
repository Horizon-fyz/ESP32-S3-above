idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES freertos nvs_flash wiznet status_led tcp_parser motor imu servo control)
# 透传 ioLibrary 配置宏 (main.c 也需要这些定义才能正确处理 socket.h)
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    _WIZCHIP_=5500
    _WIZCHIP_IO_MODE_=_WIZCHIP_IO_MODE_SPI_VDM_
    _PHY_IO_MODE_=_PHY_IO_MODE_PHYCR_
)
