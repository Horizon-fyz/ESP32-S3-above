/**
 * @file servo.h
 * @brief PCA9685 16 路 PWM 舵机驱动 (I2C1)
 *
 * 硬件连接:
 *   SDA = GPIO4
 *   SCL = GPIO5
 *   默认地址 0x70 (A0~A5 全部接地)
 *
 * 用途: 控制云台舵机, 通道 0/1 已连接自制云台舵机.
 *
 * 标准舵机参数:
 *   频率 50Hz, 周期 20ms
 *   脉宽 1ms (0°) ~ 1.5ms (90°) ~ 2ms (180°)
 *   12 位分辨率: 4096 步 (0~4095)
 *   1ms = 4096 * 1/20 = 205, 1.5ms = 307, 2ms = 410
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_CHANNEL_COUNT  16      /* PCA9685 通道数 */

/**
 * @brief 舵机配置
 */
typedef struct {
    int      sda_gpio;          ///< SDA 引脚, 默认 4
    int      scl_gpio;          ///< SCL 引脚, 默认 5
    uint32_t i2c_freq_hz;       ///< I2C 频率, 默认 400kHz
    uint8_t  i2c_addr;          ///< PCA9685 地址, 默认 0x70
    uint16_t pwm_freq_hz;       ///< PWM 输出频率, 舵机一般 50Hz
} servo_config_t;

/**
 * @brief 获取默认配置 (I2C1, SDA=4, SCL=5, 400kHz, 50Hz PWM)
 */
servo_config_t servo_get_default_config(void);

/**
 * @brief 初始化 I2C1 + PCA9685
 */
esp_err_t servo_init(const servo_config_t *cfg);

/**
 * @brief 设置某通道的脉宽 (微秒)
 *
 * @param channel   通道号 0~15
 * @param pulse_us  脉宽 0~20000 (μs), 标准舵机 1000~2000
 */
esp_err_t servo_set_pulse_us(uint8_t channel, uint16_t pulse_us);

/**
 * @brief 设置某通道角度 (0~180°)
 *
 * 内部自动换算为脉宽, 假设标准舵机 1ms=0°, 2ms=180°.
 */
esp_err_t servo_set_angle(uint8_t channel, float angle);

/**
 * @brief 关闭所有通道输出 (PCA9685 全部 OFF)
 */
esp_err_t servo_sleep_all(void);

/**
 * @brief 检查 PCA9685 是否已成功初始化
 */
bool servo_is_ready(void);

/**
 * @brief 反初始化
 */
void servo_deinit(void);

#ifdef __cplusplus
}
#endif
