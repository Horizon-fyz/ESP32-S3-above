/**
 * @file wiznet_manager.h
 * @brief W5500 + ioLibrary 网络管理器
 *
 * 基于 WIZnet 官方 ioLibrary 驱动, 直接使用 W5500 硬件 TCP/IP 协议栈 (TOE),
 * 绕过 ESP-IDF 的 LwIP, 提升吞吐量到 25~35 Mbps, 降低 CPU 占用.
 *
 * 使用示例:
 * @code
 *   wiznet_manager_config_t cfg = WIZNET_MANAGER_DEFAULT_CONFIG();
 *   wiznet_manager_init(&cfg);
 *   if (wiznet_manager_wait_for_link(5000)) {
 *       // 通过 ioLibrary 的 socket/listen/recv/send 进行 TCP 通信
 *   }
 * @endcode
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_netif.h"
#include "driver/spi_master.h"  /* spi_host_device_t */

#ifdef __cplusplus
extern "C" {
#endif

/* Socket 编号常量, 与 ioLibrary 一致 */
#define WIZNET_SOCK_MAX  8

/**
 * @brief W5500 网络管理器配置
 */
typedef struct {
    int spi_sck_gpio;             /**< SPI SCK 引脚 */
    int spi_mosi_gpio;            /**< SPI MOSI 引脚 */
    int spi_miso_gpio;            /**< SPI MISO 引脚 */
    int spi_cs_gpio;              /**< SPI CS 引脚 */
    int spi_rst_gpio;             /**< 复位引脚 (-1=不使用) */
    int spi_int_gpio;             /**< 中断引脚 (-1=不使用) */

    spi_host_device_t spi_host;   /**< SPI 主机 (通常 SPI2_HOST) */
    int               spi_clock_hz; /**< SPI 时钟频率 (Hz), 推荐 30M~40M */

    uint8_t mac[6];               /**< MAC 地址 (全 0 时使用芯片 eFuse / 默认值) */
    uint8_t ip[4];                /**< 静态 IP */
    uint8_t netmask[4];           /**< 子网掩码 */
    uint8_t gateway[4];           /**< 网关 */
    uint8_t dns[4];               /**< DNS 服务器 */

    /* 各 socket 的 TX/RX buffer 大小 (单位 KB, 总和 ≤ 16) */
    uint8_t tx_buf_size_kb[WIZNET_SOCK_MAX];
    uint8_t rx_buf_size_kb[WIZNET_SOCK_MAX];
} wiznet_manager_config_t;

/**
 * @brief 获取默认配置
 *
 * 适用于 ESP32-S3 + W5500 (SPI2) 通用接线:
 *   SCK=12, MOSI=11, MISO=13, CS=10, RST=-1, INT=-1
 *   IP: 192.168.1.100, MASK: 255.255.255.0, GW: 192.168.1.1
 */
wiznet_manager_config_t wiznet_manager_get_default_config(void);

/**
 * @brief 初始化 W5500 + ioLibrary
 *
 * 执行流程:
 *   1. 初始化 SPI 总线
 *   2. 初始化 SPI 桥接层 (CS, RST, 注册回调)
 *   3. 调用 wizchip_init 配置 buffer 大小
 *   4. 调用 wizchip_setnetinfo 配置网络参数
 *
 * @return ESP_OK 成功, 其他失败
 */
esp_err_t wiznet_manager_init(const wiznet_manager_config_t *cfg);

/**
 * @brief 反初始化, 释放资源
 */
void wiznet_manager_deinit(void);

/**
 * @brief 等待 PHY Link 建立
 *
 * @param timeout_ms 超时时间 (ms)
 * @return true 已 Link Up, false 超时
 */
bool wiznet_manager_wait_for_link(uint32_t timeout_ms);

/**
 * @brief 检查 PHY 是否 Link Up
 */
bool wiznet_manager_is_link_up(void);

/**
 * @brief 获取 PHY 当前协商速率 (Mbps), 返回 0/10/100
 */
uint8_t wiznet_manager_get_link_speed(void);

/**
 * @brief 获取当前网络信息 (IP/MASK/GW/MAC)
 *
 * @param ip_info  输出 esp_netif_ip_info_t (仅 ip/netmask/gw 字段有效)
 * @return ESP_OK 成功
 */
esp_err_t wiznet_manager_get_ip_info(esp_netif_ip_info_t *ip_info);

/**
 * @brief 获取已初始化的 ioLibrary 句柄 (用于高级操作)
 */
void *wiznet_manager_get_wizchip(void);

#ifdef __cplusplus
}
#endif
