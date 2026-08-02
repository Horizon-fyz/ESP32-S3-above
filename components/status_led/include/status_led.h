/**
 * @file status_led.h
 * @brief 网络状态指示灯模块
 *
 * 使用单颗板载 WS2812B RGB LED (GPIO48), 通过 RMT 单总线驱动。
 * 硬件参数:
 *   - LED 型号: XL-5050RGBC-WS2812B (5V)
 *   - 数据引脚: GPIO48
 *   - 像素数量: 1
 *   - 灯序: GRB
 *   - DO 悬空, 无级联
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 指示灯颜色
 */
typedef enum {
    LED_COLOR_OFF = 0,
    LED_COLOR_RED,
    LED_COLOR_GREEN,
    LED_COLOR_BLUE,          /* 深蓝 */
    LED_COLOR_YELLOW,        /* 深黄 */
    LED_COLOR_YELLOW_DIM,    /* 浅黄 */
    LED_COLOR_PURPLE,
    LED_COLOR_BLUE_DIM,      /* 浅蓝 */
    LED_COLOR_WHITE,
    LED_COLOR_MAX
} led_color_t;

/**
 * @brief 指示灯状态
 */
typedef enum {
    STATUS_LED_DISCONNECTED,   ///< 未连接网络 - 红色
    STATUS_LED_CONNECTED,      ///< 已连接网络，无显著数据交换 - 绿色
    STATUS_LED_DATA_EXCHANGE,  ///< 已连接网络，有数据交换 - 蓝色闪烁
} status_led_state_t;

/**
 * @brief WS2812B 配置
 */
typedef struct {
    int gpio_num;        ///< WS2812B 数据引脚, 默认 GPIO48
    uint8_t brightness;  ///< 全局亮度 0-255, 默认 64
} status_led_config_t;

/**
 * @brief 获取默认配置
 *
 * 默认: GPIO48, 亮度 64。
 */
status_led_config_t status_led_get_default_config(void);

/**
 * @brief 初始化状态指示灯
 *
 * @param cfg WS2812B 配置
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t status_led_init(const status_led_config_t *cfg);

/**
 * @brief 设置指示灯颜色
 *
 * @param color 颜色枚举
 */
void status_led_set_color(led_color_t color);

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
