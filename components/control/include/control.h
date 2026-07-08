/**
 * @file control.h
 * @brief TCP 控制协议解析 (双 socket 多帧类型)
 *
 * 协议族 (按帧头区分):
 *   ┌─────────────────────────────────────────────────────────────┐
 *   │ 帧头 0xAA 0x55: 上位机 → 主节点 控制帧 (16 字节)            │
 *   │   [3-4] 本地电调1, 本地电调2                                │
 *   │   [5-6] 远端电调1, 远端电调2 (主节点转发到远端)             │
 *   │   [7]   L298N DC (hi 4 位速度, lo 4 位方向)                 │
 *   │   [8-9] 本地舵机 0/1 角度                                   │
 *   │   [10]  flags (bit0=本地, bit1=转发远端)                    │
 *   │   [15]  CRC8                                               │
 *   ├─────────────────────────────────────────────────────────────┤
 *   │ 帧头 0xAA 0x55: 主 → 远端 转发子帧 (8 字节)                │
 *   │   [3-4] 远端电调1, 远端电调2                                │
 *   │   [7]   CRC8                                               │
 *   ├─────────────────────────────────────────────────────────────┤
 *   │ 帧头 0xBB 0x66: MPU 数据帧 (16 字节) - 双向                 │
 *   │   [2]   type (0x01=本地 MPU, 0x02=远端 MPU)                │
 *   │   [3-14] 6 轴 int16: ax ay az gx gy gz (LE)                │
 *   │   [15]  CRC8                                               │
 *   └─────────────────────────────────────────────────────────────┘
 *
 * 解析策略 (与之前相同):
 *   - 字节流扫描, 残帧保留
 *   - 凑齐一帧立即解析并执行
 *   - 主 → 远端 转发由 tcp_server 任务完成 (因为需要访问 socket 1)
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 控制帧常量 */
#define CTRL_CTRL_HEAD_0       0xAA    /* 控制帧头 1 */
#define CTRL_CTRL_HEAD_1       0x55    /* 控制帧头 2 */
#define CTRL_CTRL_FRAME_SIZE   16      /* 上位机→主 控制帧 */
#define CTRL_FWD_FRAME_SIZE    8       /* 主→远端 转发子帧 */

#define CTRL_MPU_HEAD_0        0xBB    /* MPU 帧头 1 */
#define CTRL_MPU_HEAD_1        0x66    /* MPU 帧头 2 */
#define CTRL_MPU_FRAME_SIZE    16      /* MPU 数据帧 */

/* 命令码 */
typedef enum {
    CTRL_CMD_MOTOR  = 0x10,   ///< 电机控制
    CTRL_CMD_STOP   = 0x20,   ///< 紧急停止
    CTRL_CMD_REBOOT = 0x30,   ///< 系统重启
    CTRL_CMD_SHUTDOWN = 0x40, ///< 深度睡眠关机
} ctrl_cmd_t;

/* MPU 数据类型 */
typedef enum {
    CTRL_MPU_TYPE_LOCAL  = 0x01,   ///< 本地 MPU
    CTRL_MPU_TYPE_REMOTE = 0x02,   ///< 远端 MPU (主节点转发)
} ctrl_mpu_type_t;

/* 解析后的控制指令 (主节点收到上位机 16B 帧后解析的结果) */
typedef struct {
    uint8_t cmd;                 /* CTRL_CMD_* */
    int8_t  local_esc[2];        /* 本地电调油门 (-100~+100) */
    int8_t  remote_esc[2];       /* 远端电调油门 (-100~+100) */
    uint8_t dc_packed;           /* hi 4 位: 速度 0~15, lo 4 位: 方向 */
    uint8_t servo_angle[2];      /* 舵机 0/1 角度 (0~180) */
    uint8_t flags;               /* bit0: 本地执行, bit1: 转发远端 */
} ctrl_command_t;

/* 解析后的 MPU 数据 */
typedef struct {
    uint8_t  type;               /* CTRL_MPU_TYPE_* */
    int16_t  ax, ay, az;         /* 加速度 (原始寄存器, 16384 LSB/g) */
    int16_t  gx, gy, gz;         /* 角速度 (原始寄存器, 131 LSB/°/s) */
} ctrl_mpu_data_t;

/* 控制帧处理回调 (主 → 远端 转发) */
typedef void (*ctrl_forward_cb_t)(const ctrl_command_t *cmd);

/* ========== API ========== */

/**
 * @brief 处理一段接收到的字节流, 提取并执行完整控制帧
 *
 * 内部有状态机, 跨调用保留残帧.
 *
 * @param data 新到达的字节流
 * @param len  字节数
 * @return 解析并执行的完整帧数
 */
size_t control_process(const uint8_t *data, size_t len);

/**
 * @brief 注册主→远端 转发回调 (tcp_server 任务设置)
 *
 * 收到上位机控制帧, 且 flags bit1 置位时, 会调用此回调.
 * 回调内通过 socket 1 发送 8B 子帧到远端.
 */
void control_set_forward_callback(ctrl_forward_cb_t cb);

/**
 * @brief 复位解析器 (连接切换时调用)
 */
void control_reset(void);

/**
 * @brief 构造上位机控制帧 (16B)
 */
void control_build_ctrl_frame(uint8_t *frame,
                              uint8_t cmd,
                              int8_t local_esc1, int8_t local_esc2,
                              int8_t remote_esc1, int8_t remote_esc2,
                              uint8_t dc_packed,
                              uint8_t servo0, uint8_t servo1,
                              uint8_t flags);

/**
 * @brief 构造 MPU 数据帧 (16B)
 */
void control_build_mpu_frame(uint8_t *frame,
                             uint8_t type,
                             int16_t ax, int16_t ay, int16_t az,
                             int16_t gx, int16_t gy, int16_t gz);

/**
 * @brief 获取累计处理帧数
 */
uint32_t control_get_frame_count(void);

#ifdef __cplusplus
}
#endif
