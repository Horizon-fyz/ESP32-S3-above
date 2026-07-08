/**
 * @file status_led.h
 * @brief 网络状态指示灯模块
 *
 * 使用ESP32-S3-DevKitC-1板载RGB LED (WS2812, GPIO48)
 * 显示网络连接状态和数据交换状态。
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 指示灯状态
 */
typedef enum {
    STATUS_LED_DISCONNECTED,   ///< 未连接网络 - 红色
    STATUS_LED_CONNECTED,      ///< 已连接网络，无显著数据交换 - 绿色
    STATUS_LED_DATA_EXCHANGE,  ///< 已连接网络，有数据交换 - 蓝色闪烁
} status_led_state_t;

/**
 * @brief 初始化状态指示灯
 *
 * @param gpio_num RGB LED引脚号，ESP32-S3-DevKitC-1为GPIO48
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t status_led_init(int gpio_num);

/**
 * @brief 设置指示灯状态
 *
 * @param state 指示灯状态
 */
void status_led_set_state(status_led_state_t state);

/**
 * @brief 更新指示灯状态（根据网络状态自动判断）
 *
 * 此函数会检查网络连接状态和数据交换情况，自动设置指示灯。
 * 需要在任务中定期调用（建议100-200ms一次）。
 */
void status_led_update(void);

/**
 * @brief 通知收到了 TCP 数据 (供 TCP 接收任务调用)
 *
 * 调用此函数可让 status_led 检测到"数据交换活跃", 触发蓝色闪烁.
 */
void status_led_notify_rx(uint32_t bytes);

/**
 * @brief 通知发送了 TCP 数据 (供 TCP 发送任务调用)
 *
 * 调用此函数可让 status_led 检测到"数据交换活跃", 触发蓝色闪烁.
 */
void status_led_notify_tx(uint32_t bytes);

/**
 * @brief 反初始化指示灯
 */
void status_led_deinit(void);

#ifdef __cplusplus
}
#endif
