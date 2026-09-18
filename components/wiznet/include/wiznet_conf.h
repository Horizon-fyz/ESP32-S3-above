/**
 * @file wiznet_conf.h
 * @brief WIZnet ioLibrary 配置头文件 - ESP32-S3 + W5500 适配
 *
 * 必须在包含任何 ioLibrary 头文件 (wizchip_conf.h, socket.h) 之前定义
 * _WIZCHIP_ 和 _WIZCHIP_IO_MODE_. 本头文件用 #ifndef 保护, 首次定义生效.
 */

#ifndef WIZNET_CONF_H_
#define WIZNET_CONF_H_

/* 目标芯片: W5500 */
#ifndef _WIZCHIP_
#define _WIZCHIP_                      5500
#endif

/* SPI 主机接口模式: 使用 VDM(可变长数据) - 兼容性最好 */
#ifndef _WIZCHIP_IO_MODE_
#define _WIZCHIP_IO_MODE_              _WIZCHIP_IO_MODE_SPI_VDM_
#endif

/* PHY 访问方式: MII 的 MDC/MDIO 软访问
 *
 * ⚠️ 为什么这里是 MII 而不是 PHYCR: ioLibrary 的 `wizchip_conf.h` 里 `_PHY_IO_MODE_`
 *    原本是**无 #ifndef 保护的强制定义** (默认 `_PHY_IO_MODE_MII_`), 会把这里设的值覆盖掉
 *    —— 也就是说本工程此前**实际一直跑在 MII 模式** (链路状态/100M 全双工照样能设, 只是
 *    走 MDC/MDIO 而不是直写 PHYCR 寄存器)。现已给那边补上 `#ifndef` 保护, 让**本文件成为
 *    唯一取值处**, 取值与既有实况**保持一致 (MII)**; 要改 PHYCR 属行为变更, 需单独验证。
 */
#ifndef _PHY_IO_MODE_
#define _PHY_IO_MODE_                  _PHY_IO_MODE_MII_
#endif

/* 消除与 Xtensa 系统头的宏重名:
 *   specreg.h (经 FreeRTOS portmacro.h) 把 `MR` 定义成**特殊寄存器编号**,
 *   W5500 的 w5500.h 把 `MR` 定义成**模式寄存器** —— 两者取值不同, GCC 对"宏重定义"
 *   必报警且**没有任何开关能关闭** (`-Wno-macro-redefined` / `-Wno-builtin-macro-redefined`
 *   实测均无效)。这里在包含 ioLibrary 之前先撤掉 Xtensa 那个定义, 于是 w5500.h 的 `MR`
 *   成为"首次定义", 警告消失。
 * ⚠️ 因此本头文件必须在 **FreeRTOS / xtensa 头之后** 包含 (各 .c 的 include 顺序已按此排列,
 *    见 main.c / wiznet_manager.c / wiznet_spi.c)。 */
#ifdef MR
#undef MR
#endif

#endif /* WIZNET_CONF_H_ */
