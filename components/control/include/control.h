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
 *   │   [2]  type          uint8  0x01=本地, 0x02=远端            │
 *   │   [3-14] ax ay az gx gy gz (6×int16 LE, 原始寄存器)         │
 *   │   [15] CRC8          uint8                                  │
 *   └─────────────────────────────────────────────────────────────┘
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
    CTRL_CMD_STOP   = 0x20,   ///< 紧急停止 (本地 + 远端)
    CTRL_CMD_REBOOT = 0x30,   ///< 系统重启
    CTRL_CMD_SHUTDOWN = 0x40, ///< 深度睡眠关机
} ctrl_cmd_t;

/* MPU 数据类型 */
typedef enum {
    CTRL_MPU_TYPE_LOCAL  = 0x01,   ///< 本地 MPU
    CTRL_MPU_TYPE_REMOTE = 0x02,   ///< 远端 MPU
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

uint32_t control_get_frame_count(void);

#ifdef __cplusplus
}
#endif
