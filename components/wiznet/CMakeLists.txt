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
# ⚠️ 不要在这里定义 `_PHY_IO_MODE_`: ioLibrary 的 wizchip_conf.h 与 wiznet_conf.h 都会定义它,
#    命令行再给一个会触发 "macro redefined" 警告 (GCC 无开关可关)。PHY 访问方式统一由
#    include/wiznet_conf.h 决定 (当前 = MII, 与既有实况一致)。
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    _WIZCHIP_=5500
    _WIZCHIP_IO_MODE_=_WIZCHIP_IO_MODE_SPI_VDM_
)
# ioLibrary 源码来自第三方, 不要求零警告
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-misleading-indentation
    -Wno-unused-variable
    -Wno-unused-but-set-variable
    -Wno-missing-field-initializers
    -Wno-unused-function
)
