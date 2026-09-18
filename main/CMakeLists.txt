idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES freertos nvs_flash wiznet motor imu servo gimbal control console nav driver)
# 透传 ioLibrary 配置宏 (main.c 也需要这些定义才能正确处理 socket.h)
# ⚠️ 这里**不要再定义** `_PHY_IO_MODE_`: wizchip_conf.h 与 wiznet_conf.h 都会定义它,
#    命令行再给一个会触发 "macro redefined" 警告 (GCC 没有任何开关能关掉)。
#    PHY 访问方式统一由 components/wiznet/include/wiznet_conf.h 决定 (当前 = MII)。
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    _WIZCHIP_=5500
    _WIZCHIP_IO_MODE_=_WIZCHIP_IO_MODE_SPI_VDM_
)

# ioLibrary 的 socket.h 里有一批 "static 声明但从不定义" 的 IPv6 原型
# (connect_IO_6 / sendto_IO_6 / recvfrom_IO_6), 任何包含它的 TU 都会得到
# -Wunused-function。该警告在 TU 结束时才报, `#pragma GCC diagnostic` 压不住
# (已实测), 只能按组件关掉 —— 与 components/wiznet 的做法一致。
# ⚠️ 代价: main.c 里"定义但未使用"的函数不再报警告; 发现死代码要靠人工 review。
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-unused-function
)
