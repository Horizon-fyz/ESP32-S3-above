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
 *   │   [6]  remote_dir    uint8  远端电机 (0=停, 1=正, 2=反)     │
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
 *   │   [6]  remote_dir    uint8  远端电机方向+停止                │
 *   │   [7]  CRC8          uint8  前 7 字节异或                   │
 *   ├─────────────────────────────────────────────────────────────┤
 *   │ 帧头 0xBB 0x66: MPU 数据帧 (16 字节) - 双向                 │
 *   │   [2]  type          uint8  0x01=本地, 0x02=远端, 0x03=深度状态 │
 *   │   [3-14] ax ay az gx gy gz (6×int16 LE, 原始寄存器)         │
 *   │   [15] CRC8          uint8                                  │
 *   └─────────────────────────────────────────────────────────────┘
 *
 * v2.0 水上水下联合方案 (depth_ctrl):
 *   ├ 16B 控制帧 cmd=0x11 DEPTH (上位机→主控):
 *   │   [3-4] target_depth int16 LE (cm)  [5] mode (0=自动)
 *   ├ 8B 子帧 cmd=0x11 DEPTH (主控→远端):
 *   │   [3-4] target_depth int16 LE (cm)  [5] mode (0=自动)  [6] 保留
 *   └ 16B MPU 帧 type=0x03 DEPTH_STATUS (远端→主控→上位机):
 *       [3-4] depth_a int16 (cm)  [5-6] depth_b int16 (cm)
 *       [7-8] comp_pressure uint16 (mbar)  [9-10] est_water uint16 (0.01L)
 *       [11] mode  [12] pid_out int8 (-100~100)  [13] actuator_bits  [14] 保留
 *
 * 差速混合 (主节点本地, 远端也用同样公式):
 *   left_esc  = clamp(speed + yaw, -100, +100) → ESC1 (主/远)
 *   right_esc = clamp(speed - yaw, -100, +100) → ESC2 (主/远)
 *   dc_speed  = |speed|                            → L298N PWM
 *   dc_dir    = (speed > 0) ? FORWARD :
 *               (speed < 0) ? REVERSE : STOP       → L298N IN1/IN2
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
    CTRL_CMD_DEPTH  = 0x11,   ///< 压载深度控制 (v2.0, 目标深度 cm + 模式)
    CTRL_CMD_STOP   = 0x20,   ///< 紧急停止 (本地 + 远端)
    CTRL_CMD_REBOOT = 0x30,   ///< 系统重启
    CTRL_CMD_SHUTDOWN = 0x40, ///< 深度睡眠关机
} ctrl_cmd_t;

/* MPU 数据类型 */
typedef enum {
    CTRL_MPU_TYPE_LOCAL    = 0x01,   ///< 本地 MPU
    CTRL_MPU_TYPE_REMOTE   = 0x02,   ///< 远端 MPU
    CTRL_MPU_TYPE_DEPTH    = 0x03,   ///< 深度状态 (v2.0, depth_ctrl)
} ctrl_mpu_type_t;

/* flags 位 */
#define CTRL_FLAG_ENABLE_LOCAL     0x01
#define CTRL_FLAG_FORWARD_REMOTE   0x02

/* 解析后的控制指令 (主节点收到 16B 帧后) */
typedef struct {
    uint8_t cmd;
    int8_t  speed;            /* -100 ~ +100 */
    int8_t  yaw;              /* -100 ~ +100 */
    uint8_t remote_light;     /* 0=关, 1=开 */
    uint8_t remote_dir;       /* 0=停, 1=正, 2=反 */
    uint8_t flags;
    /* v2.0 DEPTH 指令 */
    int16_t target_depth_cm;  /* 目标深度 cm */
    uint8_t depth_mode;       /* 0=自动 */
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
    uint8_t remote_light, uint8_t remote_dir, uint8_t flags);

void control_build_mpu_frame(uint8_t *frame,
    uint8_t type,
    int16_t ax, int16_t ay, int16_t az,
    int16_t gx, int16_t gy, int16_t gz);

/* v2.0 深度控制帧 (16B, cmd=0x11, 上位机→主控) */
void control_build_depth_ctrl_frame(uint8_t *frame,
    int16_t target_depth_cm, uint8_t mode, uint8_t flags);

/* v2.0 深度控制子帧 (8B, cmd=0x11, 主控→远端) */
void control_build_depth_fwd_frame(uint8_t *frame,
    int16_t target_depth_cm, uint8_t mode);

uint32_t control_get_frame_count(void);

#ifdef __cplusplus
}
#endif
