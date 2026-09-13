/**
 * @file control.h
 * @brief TCP 控制协议 (差速驱动 v4.0)
 *
 * 协议族 (按帧头区分):
 *   ┌─────────────────────────────────────────────────────────────┐
 *   │ 帧头 0xAA 0x55: 上位机 → 主节点 控制帧 (16 字节)            │
 *   │   [3]  speed         int8   整体推进速度 (-100~+100)        │
 *   │   [4]  yaw           int8   偏航/转向 (-100~+100)           │
 *   │   [5]  remote_light  uint8  远端灯开关 (0=关, 1=开)         │
 *   │   [6]  bucket_speed  int8   L298N 铲斗电机速度 (-100~+100, 0=停) │
 *   │   [7]  reserved      uint8  保留                            │
 *   │   [8-9] reserved     uint8  原舵机, 已删除                   │
 *   │   [10] flags         uint8  bit0=本地执行, bit1=转发远端     │
 *   │   [11-14] reserved   uint8  保留                            │
 *   │   [15] CRC8          uint8  前 15 字节异或                  │
 *   ├─────────────────────────────────────────────────────────────┤
 *   │ 帧头 0xAA 0x55: 主 → 远端 转发子帧 (8 字节)                │
 *   │   [3]  speed         int8   速度                            │
 *   │   [4]  yaw           int8   偏航                            │
 *   │   [5]  remote_light  uint8  远端灯开关                      │
 *   │   [6]  bucket_speed  int8   远端 L298N 铲斗电机速度 (-100~+100, 0=停) │
 *   │   [7]  CRC8          uint8  前 7 字节异或                   │
 *   ├─────────────────────────────────────────────────────────────┤
 *   │ 帧头 0xBB 0x66: 状态数据帧 (16 字节) - 双向                 │
 *   │   [2]  type          uint8  0x01=本地 MPU, 0x02=远端 MPU,   │
 *   │                              0x03=远端沉浮状态(v3.0)        │
 *   │   type=0x01/0x02: [3-14] ax ay az gx gy gz (6×int16 LE)    │
 *   │   type=0x03:      见下方 v3.0 沉浮状态帧布局                │
 *   │   [15] CRC8          uint8                                  │
 *   └─────────────────────────────────────────────────────────────┘
 *
 * v3.0 水上水下联合方案 (远端 vertical_ctrl, 取代 v2.x 水舱压载):
 *   ├ 16B 控制帧 cmd=0x11 DEPTH (上位机→主控):
 *   │   [3-4] target_depth int16 LE (cm)  [5] target_pitch int8(°)  [6] target_roll int8(°)
 *   ├ 8B 子帧 cmd=0x11 DEPTH (主控→远端):
 *   │   [3-4] target_depth int16 LE (cm)  [5] target_pitch int8(°)  [6] target_roll int8(°)
 *   └ 16B 状态帧 type=0x03 VCTRL_STATUS (远端→主控→上位机, 主控原样透传):
 *       [3-4] depth_cm int16 (cm)         [5-6] pitch int16 (0.1°)
 *       [7-8] roll int16 (0.1°)           [9]   mode (SURFACE/DESCEND/HOVER/ASCEND/EMERGENCY)
 *       [10] out_f  [11] out_rl  [12] out_rr (int8, 油门%)  [13] flags (CTRL_VCTRL_FLAG_*)  [14] 保留
 *
 *   ⚠️ 主控对 type=0x02 / type=0x03 **原样透传**, 不解析不修改. 现有实现把字节 3-14
 *      按 ax..gz 解出再重建, 由于两者都是 int16 LE 且 CRC 由前 15 字节异或得出,
 *      重建结果与原帧逐字节一致, 因此 type=0x03 也被无损转发, 无需为它单独写解析.
 *
 * 差速混合 (主节点本地, 远端也用同样公式):
 *   left_esc  = clamp(speed + yaw, -100, +100) → ESC1 (主/远)
 *   right_esc = clamp(speed - yaw, -100, +100) → ESC2 (主/远)
 *   dc_speed  = |speed|                            → L298N PWM
 *   bucket_speed → 远端 L298N 铲斗电机 (INT8 有符号速度, 0=制动)
 *
 * 主机 (上位机) 控制律基于本地 MPU 数据:
 *   主机接收 MPU 原始数据 → 主机做姿态解算 + 控制律 → 主机发 speed+yaw
 *   主节点只做差速混合 (open-loop), 不做姿态闭环.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 帧常量 */
#define CTRL_CTRL_HEAD_0       0xAA
#define CTRL_CTRL_HEAD_1       0x55
#define CTRL_CTRL_FRAME_SIZE   16
#define CTRL_FWD_FRAME_SIZE    8

#define CTRL_MPU_HEAD_0        0xBB
#define CTRL_MPU_HEAD_1        0x66
#define CTRL_MPU_FRAME_SIZE    16

/* 命令码 */
typedef enum {
    CTRL_CMD_MOTOR  = 0x10,   ///< 电机控制 (差速混合)
    CTRL_CMD_DEPTH  = 0x11,   ///< 沉浮控制 (v3.0: 目标深度 cm + 目标俯仰° + 目标横滚°)
    CTRL_CMD_STOP   = 0x20,   ///< 紧急停止 (本地 + 远端)
    CTRL_CMD_REBOOT = 0x30,   ///< 系统重启
    CTRL_CMD_SHUTDOWN = 0x40, ///< 深度睡眠关机
} ctrl_cmd_t;

/* MPU / 状态数据类型 */
typedef enum {
    CTRL_MPU_TYPE_LOCAL    = 0x01,   ///< 本地 MPU (原始 6 轴)
    CTRL_MPU_TYPE_REMOTE   = 0x02,   ///< 远端 MPU (原始 6 轴)
    CTRL_MPU_TYPE_DEPTH    = 0x03,   ///< 远端沉浮/姿态状态 (v3.0, vertical_ctrl)
} ctrl_mpu_type_t;

/* v3.0 状态帧 (type=0x03) flags 位 */
#define CTRL_VCTRL_FLAG_DEPTH_VALID  0x01   ///< 深度数据有效
#define CTRL_VCTRL_FLAG_IMU_VALID    0x02   ///< IMU 姿态有效
#define CTRL_VCTRL_FLAG_MANUAL       0x04   ///< 远端沉浮控制已暂停 (手动模式)

/* 控制帧 flags 位 */
#define CTRL_FLAG_ENABLE_LOCAL     0x01
#define CTRL_FLAG_FORWARD_REMOTE   0x02

/* 解析后的控制指令 (主节点收到 16B 帧后) */
typedef struct {
    uint8_t cmd;
    int8_t  speed;            /* -100 ~ +100 */
    int8_t  yaw;              /* -100 ~ +100 */
    uint8_t remote_light;     /* 0=关, 1=开 */
    int8_t  bucket_speed;     /* L298N 铲斗电机速度 -100~+100, 0=停 */
    uint8_t flags;
    /* v3.0 DEPTH (cmd=0x11) 指令 */
    int16_t target_depth_cm;  /* 目标深度 cm (0=回水面) */
    int8_t  target_pitch_deg; /* 目标俯仰角 °, 正=抬头 */
    int8_t  target_roll_deg;  /* 目标横滚角 °, 正=右滚 */
} ctrl_command_t;

/* MPU 数据 */
typedef struct {
    uint8_t  type;
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;
} ctrl_mpu_data_t;

/* 主→远端 转发回调 */
typedef void (*ctrl_forward_cb_t)(const ctrl_command_t *cmd);

/* ========== API ========== */

size_t control_process(const uint8_t *data, size_t len);
void   control_set_forward_callback(ctrl_forward_cb_t cb);
void   control_reset(void);

/* 帧构造器 */
void control_build_ctrl_frame(uint8_t *frame,
    uint8_t cmd, int8_t speed, int8_t yaw,
    uint8_t remote_light, int8_t bucket_speed, uint8_t flags);

void control_build_mpu_frame(uint8_t *frame,
    uint8_t type,
    int16_t ax, int16_t ay, int16_t az,
    int16_t gx, int16_t gy, int16_t gz);

/* v2.0→v3.0 迁出: 远端沉浮状态帧 (type=0x03) 由远端 vertical_ctrl 构造, 主控原样透传,
 * 因此主控侧不提供 control_build_vctrl_status_frame(); 布局见文件头注释. */

/* v3.0 沉浮控制帧 (16B, cmd=0x11, 上位机→主控)
 *   [3-4] target_depth int16 LE (cm)  [5] target_pitch int8(°)  [6] target_roll int8(°) */
void control_build_depth_ctrl_frame(uint8_t *frame,
    int16_t target_depth_cm, int8_t target_pitch, int8_t target_roll, uint8_t flags);

/* v3.0 沉浮控制子帧 (8B, cmd=0x11, 主控→远端)
 *   [3-4] target_depth int16 LE (cm)  [5] target_pitch int8(°)  [6] target_roll int8(°) */
void control_build_depth_fwd_frame(uint8_t *frame,
    int16_t target_depth_cm, int8_t target_pitch, int8_t target_roll);

uint32_t control_get_frame_count(void);

#ifdef __cplusplus
}
#endif
