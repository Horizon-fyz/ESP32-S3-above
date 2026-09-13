/**
 * @file servo.h
 * @brief PCA9685 16 路 PWM 舵机驱动 (I2C1)
 *
 * 硬件连接 (v5.10 换板 ESP32-S3-ETH):
 *   SDA = GPIO21
 *   SCL = GPIO17
 *   默认地址 0x40 (A0~A5 全部悬空)
 *   ⚠️ 该 I2C1 总线与船体 10 轴 IMU (0x50) 共用, 由本组件负责安装。
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
    int      sda_gpio;          ///< SDA 引脚, 默认 21 (I2C1, 与船体 10 轴 IMU 共用)
    int      scl_gpio;          ///< SCL 引脚, 默认 17 (I2C1, 与船体 10 轴 IMU 共用)
    uint32_t i2c_freq_hz;       ///< I2C 频率, 默认 400kHz
    uint8_t  i2c_addr;          ///< PCA9685 地址, 默认 0x40
    uint16_t pwm_freq_hz;       ///< PWM 输出频率, 舵机一般 50Hz
} servo_config_t;

/**
 * @brief 单通道舵机标定参数
 *
 * ============ 两套角度刻度 (重要) ============
 * 本模块内部维护两套角度, 各有用途, 换算只发生在这一处, 不要在其他地方乘系数:
 *
 *   1) 标称角 (range_deg):  报文/控制台/上位机用。它是"看上去的角度",
 *      ch1 就是 0~180°, ch0 就是 0~360° —— 满足可读性要求。
 *   2) 物理角 (phys_range_deg): 云台实际转过的角度, 也是 MPU 直接测到的量。
 *      ch1 实测为 0~192°, ch0 实测为 0~365° (机械行程比标称刻度大一些)。
 *
 *   关系: 物理角 = 标称角 × (phys_range_deg / range_deg)
 *
 * 为什么要分开: 在物理角域里, MPU 的 pitch/yaw 与云台转角严格是 1:1
 * (同一个刚体), 控制律因此不含任何修正系数; 而标称角域只负责"好看好读"。
 * 若把比例系数散进控制律, 容易被误当成 PID 可调参数, 也容易漏乘/重复乘。
 *
 * ============ 其余字段 ============
 *   - min_us/max_us: 脉宽行程端点, 对应**物理角** 0° 与 phys_range_deg
 *   - trim_us:       中位微调偏移, 正数向右偏. 用于对齐舵机测试仪的中位
 *   - limit_min_deg/limit_max_deg: 软件行程限位 (**标称角**域, 安全窗口)
 *
 * 中位脉宽 = (min_us + max_us)/2 + trim_us
 * 角度→脉宽: pulse = min_us + cmd/range_deg * (max_us - min_us) + trim_us
 *
 * 关于行程限位:
 *   端点(min_us/max_us)是**机械硬限位**, 长期顶死会堵转发热甚至烧舵机。
 *   限位是"允许运行的角度窗口", 与标定**互相独立** —— 它不改变角度↔脉宽
 *   的换算关系, 只是把命令角钳位在窗口内。若把端点改掉, 角度对应关系会
 *   整体错位, 所以调整范围只能用限位, 不要改端点。
 */
typedef struct {
    uint16_t min_us;        ///< 最小脉宽 (对应物理角 0°)
    uint16_t max_us;        ///< 最大脉宽 (对应物理角 phys_range_deg)
    int16_t  trim_us;       ///< 中位微调偏移 (µs), 默认 0
    uint16_t range_deg;     ///< **标称**角度满量程 (报文/上位机用): 180 / 360
    uint16_t phys_range_deg;///< **物理**行程 (°), 实测值: ch1=192, ch0=365; 0 = 同 range_deg
    uint16_t limit_min_deg; ///< 软件行程下限 (**标称角**°), 0 = 不限
    uint16_t limit_max_deg; ///< 软件行程上限 (**标称角**°), 0 = 取 range_deg
} servo_cal_t;

/**
 * 获取默认配置 (I2C1, SDA=4, SCL=5, 400kHz, 50Hz PWM, 地址 0x40)
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
 * @brief 设置某通道角度
 *
 * 有效范围 0 ~ 该通道标定的 range_deg (180° 舵机 0~180, 360° 舵机 0~360),
 * 内部按该通道 min_us/max_us/trim_us 换算为脉宽. 超范围会被钳位.
 */
esp_err_t servo_set_angle(uint8_t channel, float angle);

/**
 * @brief 关闭所有通道输出 (PCA9685 全部 OFF)
 */
esp_err_t servo_sleep_all(void);

/* ==================== 标定 API ==================== */

/**
 * @brief 获取通用默认标定 (1000/2000/trim=0)
 *
 * 注: 各通道实际使用的"出厂默认"由 servo.c 内的按通道默认表决定
 *     (ch0 360° 舵机实测量程 530~2730µs 对应 0~360°、中点 1630µs=180°;
 *      ch1 180° 舵机实测量程 600~2700µs 对应 0~180°、中点 1650µs=90°),
 *     复位某通道请用 servo_reset_cal().
 */
servo_cal_t servo_get_default_cal(void);

/**
 * @brief 写入某通道标定参数 (不落盘)
 *
 * @param channel 通道号 0~15
 * @param cal     标定参数, 内部会做合法性钳位
 */
esp_err_t servo_set_cal(uint8_t channel, const servo_cal_t *cal);

/**
 * @brief 读取某通道标定参数
 */
esp_err_t servo_get_cal(uint8_t channel, servo_cal_t *cal);

/**
 * @brief 复位某通道标定为默认值 (不落盘)
 */
esp_err_t servo_reset_cal(uint8_t channel);

/**
 * @brief 让某通道输出到标定中位 (回中)
 *
 * 中位脉宽 = (min_us + max_us)/2 + trim_us
 */
esp_err_t servo_center(uint8_t channel);

/**
 * @brief 让全部通道回中
 */
esp_err_t servo_center_all(void);

/**
 * @brief 读取某通道最近一次设置的脉宽 (µs)
 */
esp_err_t servo_get_pulse_us(uint8_t channel, uint16_t *pulse_us);

/**
 * @brief 由最近一次下发的脉宽反算当前角度 (°)
 *
 * 按该通道标定换算, 结果钳位在 0 ~ range_deg。
 */
esp_err_t servo_get_angle(uint8_t channel, float *angle);

/**
 * @brief 读取某通道的软件行程限位 (°)
 *
 * @param min_deg  输出: 下限 (0 表示不限, 即 0°)
 * @param max_deg  输出: 上限 (若标定里为 0, 则返回 range_deg)
 */
esp_err_t servo_get_limit(uint8_t channel, float *min_deg, float *max_deg);

/* ==================== 标称角 ↔ 物理角 换算 ==================== */
/* 内部/闭环控制请用物理角; 报文与显示请用标称角 (servo_set_angle/servo_get_angle)。
 * 具体见 servo_cal_t 的"两套角度刻度"说明。 */

/**
 * @brief 标称角 → 物理角
 */
float servo_cmd_to_phys_deg(uint8_t channel, float cmd_deg);

/**
 * @brief 物理角 → 标称角
 */
float servo_phys_to_cmd_deg(uint8_t channel, float phys_deg);

/**
 * @brief 以**物理角**设置某通道舵机 (内部换算成标称角后下发)
 *
 * 闭环控制应该用这个: MPU 测到的就是物理角, 两者 1:1, 控制律里不需要系数。
 * 它同样受软件行程限位约束。
 */
esp_err_t servo_set_phys_angle(uint8_t channel, float phys_deg);

/**
 * @brief 将全部通道标定保存到 NVS
 */
esp_err_t servo_save_cal(void);

/**
 * @brief 从 NVS 加载全部通道标定 (无记录时保持默认)
 */
esp_err_t servo_load_cal(void);

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
