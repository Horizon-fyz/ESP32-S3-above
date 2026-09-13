/**
 * @file motor.h
 * @brief 电机控制抽象 (电调 PWM + L298N 方向)
 *
 * 统一管理两类电机:
 *   - 电调 (Electronic Speed Controller): 通过 LEDC 输出 50~400Hz PWM
 *     占空比对应油门 -100%~+100%
 *   - L298N 直流电机驱动: ENA 输出 PWM 调速, IN1/IN2 控制方向
 *
 * 启动时所有电机紧急停止, 防止上电失控.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 电机 ID 枚举
 */
typedef enum {
    MOTOR_ESC_1 = 0,    ///< 推进电调 1 (PWM: GPIO42)
    MOTOR_ESC_2,        ///< 推进电调 2 (PWM: GPIO41)
    MOTOR_MAIN_DC,      ///< 主推进直流电机 (L298N: ENA=38 + IN1=48 + IN2=47)
    MOTOR_THRUST_REV_1, ///< 反推电调 1 (PWM: GPIO40, 与 ESC1/2 同组连续脚)
    MOTOR_THRUST_REV_2, ///< 反推电调 2 (PWM: GPIO39, 与 ESC1/2 同组连续脚)
    MOTOR_MAX,
} motor_id_t;

/**
 * @brief L298N 电机方向
 */
typedef enum {
    MOTOR_DIR_STOP = 0,    ///< 停止 (高阻/刹车, 视具体接线)
    MOTOR_DIR_FORWARD,     ///< 正转
    MOTOR_DIR_REVERSE,     ///< 反转
} motor_dir_t;

/**
 * @brief 电机配置
 */
typedef struct {
    /* 电调 1 (MOTOR_ESC_1) - GPIO42 */
    int      esc1_gpio;         ///< 电调 1 PWM 引脚 (-1=未使用)
    uint32_t esc1_freq_hz;      ///< 电调 1 频率 (典型 50Hz)
    uint8_t  esc1_ledc_timer;
    uint8_t  esc1_ledc_channel;

    /* 电调 2 (MOTOR_ESC_2) - GPIO41 */
    int      esc2_gpio;         ///< 电调 2 PWM 引脚 (-1=未使用)
    uint32_t esc2_freq_hz;      ///< 电调 2 频率 (典型 50Hz)
    uint8_t  esc2_ledc_timer;
    uint8_t  esc2_ledc_channel;

    /* L298N 直流电机 (MOTOR_MAIN_DC) - ENA=38, IN1=48, IN2=47 */
    int      dc_ena_gpio;      ///< ENA 调速 PWM 引脚
    int      dc_in1_gpio;      ///< IN1 方向控制
    int      dc_in2_gpio;      ///< IN2 方向控制
    uint32_t dc_pwm_freq_hz;   ///< 调速频率 (1kHz~25kHz, 常用 5kHz)
    uint8_t  dc_ledc_timer;
    uint8_t  dc_ledc_channel;

    /* 反推电调 1 (MOTOR_THRUST_REV_1) - GPIO40 */
    int      rev1_gpio;         ///< 反推电调 1 PWM 引脚 (-1=未使用)
    uint32_t rev1_freq_hz;      ///< 反推电调 1 频率 (典型 50Hz)
    uint8_t  rev1_ledc_timer;
    uint8_t  rev1_ledc_channel;

    /* 反推电调 2 (MOTOR_THRUST_REV_2) - GPIO39 */
    int      rev2_gpio;         ///< 反推电调 2 PWM 引脚 (-1=未使用)
    uint32_t rev2_freq_hz;      ///< 反推电调 2 频率 (典型 50Hz)
    uint8_t  rev2_ledc_timer;
    uint8_t  rev2_ledc_channel;
} motor_config_t;

/**
 * @brief 获取默认电机配置
 */
motor_config_t motor_get_default_config(void);

/**
 * @brief 初始化所有电机 GPIO / LEDC 通道, 启动时全部停止
 */
esp_err_t motor_init(const motor_config_t *cfg);

/**
 * @brief 设置电调油门
 *
 * @param id       电机 ID (MOTOR_ESC_1 / MOTOR_ESC_2 / MOTOR_THRUST_REV_1 / MOTOR_THRUST_REV_2)
 * @param throttle 油门百分比, -100.0 (反向最大) ~ +100.0 (正向最大), 0=停
 */
esp_err_t motor_set_esc_throttle(motor_id_t id, float throttle);

/**
 * @brief 设置 L298N 直流电机速度与方向
 *
 * @param id    电机 ID (目前仅 MOTOR_MAIN_DC)
 * @param speed 速度百分比 0~100
 * @param dir   方向 (FORWARD/REVERSE/STOP)
 */
esp_err_t motor_set_dc_speed(motor_id_t id, float speed, motor_dir_t dir);

/**
 * @brief 紧急停止 - 切断所有电机输出
 */
void motor_emergency_stop(void);

/**
 * @brief 反初始化 (释放 LEDC 通道和 GPIO)
 */
void motor_deinit(void);

#ifdef __cplusplus
}
#endif
