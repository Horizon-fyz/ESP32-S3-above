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

/* PHY 访问: 通过寄存器接口 (无需 MDC/MDIO 引脚) */
#ifndef _PHY_IO_MODE_
#define _PHY_IO_MODE_                  _PHY_IO_MODE_PHYCR_
#endif

#endif /* WIZNET_CONF_H_ */
