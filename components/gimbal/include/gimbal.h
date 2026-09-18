/**
 * @file gimbal.h
 * @brief 二维云台运动规划 (速度 / 加速度受限的轨迹输出)
 *
 * 背景:
 *   云台上载荷重心偏高, 转动惯量大. 若把目标角度一次性写给舵机,
 *   舵机会以自身最大能力冲过去 —— 电流饱和、齿轮/结构受冲击、过冲后振荡.
 *   本模块把"阶跃指令"转成"受速度与加速度约束的轨迹"(梯形速度曲线),
 *   以 GIMBAL_UPDATE_HZ 周期逐点下发, 实现平缓启停.
 *
 * 核心是"制动距离"约束: 剩余行程越短, 允许速度越低,
 *   v_allow = min(max_vel, sqrt(2 * max_acc * 剩余角度))
 * 因此减速段是自然生成的, 不需要预先算时间.
 *
 * 后续可在此模块之上叠加 MPU 姿态闭环.
 *
 * 注意: 本模块是各通道舵机的**唯一**指令来源, 其他任务不要直接调 servo_set_angle,
 *       否则会与规划点打架.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GIMBAL_CH_COUNT   16      /* 与 PCA9685 通道数一致 */

/**
 * @brief 单轴的运功限制
 */
typedef struct {
    float max_vel_dps;    ///< 最大角速度 (°/s), 建议 30~90
    float max_acc_dps2;   ///< 最大角加速度 (°/s²), 建议 60~180
} gimbal_limit_t;

/**
 * @brief 单轴规划状态 (只读)
 */
typedef struct {
    float target_deg;     ///< 目标角度 (°)
    float current_deg;    ///< 当前规划点/已下发给舵机的角度 (°)
    float vel_dps;        ///< 当前规划速度 (°/s)
    bool  moving;         ///< 是否仍在运动
} gimbal_state_t;

/**
 * @brief 初始化: 建立各轴规划状态并启动周期任务
 *
 * 需在 servo_init() 之后调用 (要用舵机当前角度初始化规划点)。
 * 默认限制: 90°/s、180°/s²。
 */
esp_err_t gimbal_init(void);

/**
 * @brief 设置某轴的运功限制
 */
esp_err_t gimbal_set_limit(uint8_t ch, const gimbal_limit_t *limit);

/**
 * @brief 读取某轴的运功限制
 */
esp_err_t gimbal_get_limit(uint8_t ch, gimbal_limit_t *limit);

/**
 * @brief 以受限轨迹移动到目标角度 (°)
 *
 * 运动过程中可随时再次调用以改变目标 (轨迹会自然重规划)。
 * 角度范围按该通道的舵机标定 (servo_cal_t.range_deg)。
 */
esp_err_t gimbal_move_to(uint8_t ch, float target_deg);

/**
 * @brief 立即停止该轴 (停在当前规划点, 速度清零)
 */
esp_err_t gimbal_stop(uint8_t ch);

/**
 * @brief 查询该轴规划状态
 */
esp_err_t gimbal_get_state(uint8_t ch, gimbal_state_t *st);

/* ==================== 姿态闭环 (自稳 / 指向) ====================
 *
 * 本模块分两层:
 *   执行层 gimbal_move_to() —— 速度/加速度受限的运动规划 (开环定位)
 *   控制层 姿态闭环      —— 用 MPU 反馈把云台锁在目标姿态 (自稳/指向)
 *
 * 两者**互斥**: 某通道开启闭环后, 规划器不再写它, 由闭环直接下发。
 *
 * 数据流:
 *   姿态任务 --gimbal_feed_attitude()--> 本模块 --PID--> servo_set_angle()
 *
 * 控制律 (增量式, 工作在**物理角**域, 与 MPU 读数 1:1):
 *   e      = target_phys − meas_phys                       (°)
 *   integ += e·dt, 并按积分限幅
 *   Δcmd   = fb_sign × (kp·e + ki·integ + kd·de/dt) × (range_deg / phys_range_deg)
 *   cmd   += Δcmd      (标称角, 直接下发给舵机, 单周期变化受速率限制)
 *
 *   fb_sign 是**反馈方向**: 目标物理角增大时, 标称角该往哪边走。
 *     ch1 = −1 (实测 pitch 与标称角反向)   ch0 = +1 (实测 yaw 与标称角同向)
 *
 * 无反馈保护 (重要):
 *   闭环依赖 gimbal_feed_attitude() 持续喂入实测角。若云台 MPU 未就绪、陀螺零偏
 *   标定失败或读数中断, 实测角会恒为 0 —— 那不是"真实水平"。照跑 PID 的话误差恒 0,
 *   输出会停在 0° 并被行程限位钳到下限 (ch1 = 30°), 把俯仰顶死。
 *   因此超过 GIMBAL_FB_TIMEOUT_MS 没收到喂入时: **停 PID, 该轴送回标定中位**
 *   (中位脉宽 = (min_us+max_us)/2 + trim), 状态里 `nofb = true`; 反馈恢复后自动接管。
 *
 * 关于外部参考源 (预留):
 *   现在只用云台 MPU 独立闭环即可 —— 重力是绝对参考, 自稳不需要船体信息。
 *   将来要接**惯导模块** (components/nav: 船体姿态 + GPS) 时, 只需把它的值喂进来
 *   参与运算 (例如船体姿态做前馈、GPS 航向替代 yaw 做绝对参考), 不用改这里的结构。
 */

/** 单通道闭环 PID 参数 */
typedef struct {
    float kp;   ///< 比例 (无量纲): 1.0 ≈ 一个周期走完全部误差, 建议 0.5~1.2
    float ki;   ///< 积分 (1/s): 消除重力下垂等稳态差, 建议 0.2~1.0
    float kd;   ///< 微分 (s): 抑制过冲, 读数有噪声时留 0
} gimbal_pid_t;

/** 单通道闭环状态 (只读) */
typedef struct {
    bool  enabled;      ///< 是否开启闭环
    bool  nofb;         ///< 无姿态反馈 (云台 MPU 未就绪/标定失败/读数中断): 已停 PID 并回中位
    float target_phys;  ///< 目标物理角 (°)
    float meas_phys;    ///< 最近一次喂入的实测物理角 (°)
    float err;          ///< 最近一次误差 (°, = target − meas)
    float cmd_deg;      ///< 最近一次下发的标称角 (°)
} gimbal_stab_state_t;

/**
 * @brief 由姿态任务喂入云台实测姿态 (物理角, °)
 *
 * 实测值就是 MPU 的 pitch / yaw —— MPU 与云台是同一刚体, 两者 1:1。
 * 开启闭环后必须持续喂, 否则闭环会用过期数据。
 */
void gimbal_feed_attitude(float pitch_phys_deg, float yaw_phys_deg);

/**
 * @brief 设置某通道的闭环目标物理角 (°)
 *
 * 会被钳位到该通道的行程限位内。ch1 填 0 即"保持水平"。
 */
esp_err_t gimbal_set_target_phys(uint8_t ch, float phys_deg);

/**
 * @brief 开关某通道的姿态闭环
 *
 * 开启时: 目标默认锁定在**当前姿态**(不会突然跳), 积分清零, 规划器放手。
 * 关闭时: 停在当前标称角, 交还给规划器。
 */
esp_err_t gimbal_enable_stabilize(uint8_t ch, bool enable);

/**
 * @brief 查询某通道闭环状态
 */
esp_err_t gimbal_get_stab_state(uint8_t ch, gimbal_stab_state_t *st);

/**
 * @brief 设置 / 读取某通道闭环 PID
 */
esp_err_t gimbal_set_pid(uint8_t ch, const gimbal_pid_t *pid);
esp_err_t gimbal_get_pid(uint8_t ch, gimbal_pid_t *pid);

/**
 * @brief 设置 / 读取某通道的反馈方向 (+1 或 −1)
 */
esp_err_t gimbal_set_fb_sign(uint8_t ch, float sign);
esp_err_t gimbal_get_fb_sign(uint8_t ch, float *sign);

#ifdef __cplusplus
}
#endif
