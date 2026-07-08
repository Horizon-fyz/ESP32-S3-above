/**
 * @file wiznet_spi.h
 * @brief W5500 SPI 桥接层头文件
 */

#ifndef WIZNET_SPI_H_
#define WIZNET_SPI_H_

#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief W5500 SPI 桥接层配置
 */
typedef struct {
    spi_host_device_t spi_host;  /**< SPI 主机 (SPI2_HOST / SPI3_HOST) */
    int               cs_gpio;   /**< 片选 GPIO, -1 表示不使用软件 CS */
    int               rst_gpio;  /**< 复位 GPIO, -1 表示不使用软件复位 */
    int               int_gpio;  /**< 中断 GPIO, -1 表示不使用中断 */
    int               clock_hz;  /**< SPI 时钟频率 (Hz), 最大 80MHz */
} wiznet_spi_config_t;

/**
 * @brief 初始化 W5500 SPI 桥接层
 *
 * 此函数会:
 *   1. 创建互斥锁用于 ioLibrary 临界区保护
 *   2. 配置 CS/RST/INT GPIO
 *   3. 添加 SPI 设备 (CS 由软件控制, 避免与 WIZCHIP_SELECT 冲突)
 *   4. 注册 ioLibrary 要求的全部 SPI 回调
 *
 * @note 调用前必须先调用 spi_bus_initialize() 初始化 SPI 总线
 */
esp_err_t wiznet_spi_init(const wiznet_spi_config_t *cfg);

/**
 * @brief 获取底层 SPI 设备句柄 (高级用法, 一般不需要)
 */
spi_device_handle_t wiznet_spi_get_device(void);

/**
 * @brief 查询 W5500 INT 事件 (供任务在循环中轮询)
 *
 * W5500 拉低 INT 后, ISR 会置位标志. 任务调用本函数将获知有事件待处理.
 * 必须在非 ISR 上下文中调用. 调用后自动清除标志.
 *
 * @retval true  W5500 有事件待处理
 * @retval false 无事件
 */
bool wiznet_spi_check_int(void);

#ifdef __cplusplus
}
#endif

#endif /* WIZNET_SPI_H_ */
