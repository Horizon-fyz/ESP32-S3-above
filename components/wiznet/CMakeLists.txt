idf_component_register(
    SRCS
        "Ethernet/socket.c"
        "Ethernet/wizchip_conf.c"
        "Ethernet/W5500/w5500.c"
        "src/wiznet_spi.c"
        "src/wiznet_manager.c"
        "src/wiznet_socket.c"
    INCLUDE_DIRS
        "include"
        "Ethernet"
        "Ethernet/W5500"
    REQUIRES
        driver
        esp_netif
        esp_timer
)
# 必须在所有 ioLibrary 头文件被处理前定义 _WIZCHIP_ 等宏
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    _WIZCHIP_=5500
    _WIZCHIP_IO_MODE_=_WIZCHIP_IO_MODE_SPI_VDM_
    _PHY_IO_MODE_=_PHY_IO_MODE_PHYCR_
)
# ioLibrary 源码来自第三方, 不要求零警告
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-misleading-indentation
    -Wno-unused-variable
    -Wno-unused-but-set-variable
    -Wno-missing-field-initializers
    -Wno-unused-function
)
