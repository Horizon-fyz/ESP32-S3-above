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
 *   │ 帧头 0xAA 0x55: 上位机 → 主节点 **云台手动帧** (16 字节, 0x12) │
 *   │   [3-4]  ch0_deg10   int16 LE  ch0 目标角 ×0.1° (标称角)    │
 *   │   [5-6]  ch1_deg10   int16 LE  ch1 目标角 ×0.1° (标称角)    │
 *   │   [7]    ch_mask     uint8     bit0=下发 ch0, bit1=下发 ch1  │
 *   │   [8-9]  reserved    uint8     保留                         │
 *   │   [10]   flags       uint8     bit0=本地执行 (同控制帧)      │
 *   │   [11-14] reserved   uint8     保留                         │
 *   │   [15]   CRC8        uint8     前 15 字节异或               │
 *   │   ⚠️ **只有 ch_mask 置位的通道会动作** ⇒ 老上位机 / 未使用该  │
 *   │      命令时 (全 0) 不会有任何舵机动作, 天然向后兼容.         │
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
 *   │                              0x03=远端沉浮状态(v3.0),       │
 *   │                              0x04=GPS 定位, 0x05=GPS 运动   │
 *   │   type=0x01/0x02: [3-14] ax ay az gx gy gz (6×int16 LE)    │
 *   │   type=0x03:      见下方 v3.0 沉浮状态帧布局                │
 *   │   type=0x04/0x05: 见下方 GPS 上报帧布局                     │
 *   │   [15] CRC8          uint8                                  │
 *   └─────────────────────────────────────────────────────────────┘
 *
 * GPS 上报 (主控 → 上位机, 各 2Hz; 数据源 = 惯导模块 components/nav):
 *   ├ 16B 状态帧 type=0x04 (GPS_POS) —— 位置/卫星/有效性:
 *   │   [3-6]   lat   int32 LE  纬度 ×1e7 (°), 负 = 南纬
 *   │   [7-10]  lon   int32 LE  经度 ×1e7 (°), 负 = 西经
 *   │   [11-12] alt   int16 LE  海拔 ×0.1 m
 *   │   [13]    sats  uint8     卫星数
 *   │   [14]    flags uint8     CTRL_GPS_FLAG_*
 *   └ 16B 状态帧 type=0x05 (GPS_NAV) —— 运动/精度/时间:
 *       [3-4]   course uint16 LE 航向 ×0.01 (°), 0~35999
 *       [5-6]   speed  uint16 LE 对地速度 ×0.01 km/h
 *       [7]     pdop   uint8     ×0.1 (上限 25.5, 超出截顶)
 *       [8]     hdop   uint8     ×0.1
 *       [9]     vdop   uint8     ×0.1
 *       [10-12] hour / minute / second  uint8 (模块时间; 时区由模块 TIMEZONE 定, 默认 UTC+8)
 *       [13]    flags  uint8     CTRL_GPS_FLAG_*
 *       [14]    保留 0
 *   ⚠️ 未定位时 lat/lon 已被驱动清零 (nav 不保留旧坐标), 只看 flags 的 FIX 位;
 *      模块没数据时**一帧都不发** (上位机面板显示 --)。
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
 * ⚠️ 差速混合与电机驱动**不在本组件** (v5.12 起):
 *   本组件只做**协议解析**; 解析出的 ctrl_command_t 通过 control_set_local_callback()
 *   交给 main.c, 由 main.c 按当前硬件决定怎么驱动 (水面 4 路电调 + L298N 滚筒).
 *
 *   原因: 原先本组件内写死了旧硬件 (2 路电调 + L298N 由 |speed| 驱动),
 *         与现有"4 路单向电调 (正=推进/负=反推) + L298N 滚筒收放"不符:
 *         旧代码把负油门直接发给单向电调会被钳成 0, 反推两路永远不会动.
 *
 *   本工程 main.c 的映射 (约定, 见参数总览 §2):
 *     left  = clamp(speed + yaw, -100, +100) → 左路 (正=左推进 IO41, 负=左反推 IO39)
 *     right = clamp(speed - yaw, -100, +100) → 右路 (正=右推进 IO42, 负=右反推 IO40)
 *     bucket_speed                           → 主控 L298N 滚筒收放电机 (IO38/48/47)
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
    CTRL_CMD_SERVO  = 0x12,   ///< 云台舵机手动 (v9.2: ch0/ch1 目标标称角 + 通道掩码)
    CTRL_CMD_STOP   = 0x20,   ///< 紧急停止 (本地 + 远端)
    CTRL_CMD_REBOOT = 0x30,   ///< 系统重启
    CTRL_CMD_SHUTDOWN = 0x40, ///< 深度睡眠关机
} ctrl_cmd_t;

/* MPU / 状态数据类型 */
typedef enum {
    CTRL_MPU_TYPE_LOCAL    = 0x01,   ///< 本地 MPU (原始 6 轴)
    CTRL_MPU_TYPE_REMOTE   = 0x02,   ///< 远端 MPU (原始 6 轴)
    CTRL_MPU_TYPE_DEPTH    = 0x03,   ///< 远端沉浮/姿态状态 (v3.0, vertical_ctrl)
    CTRL_MPU_TYPE_GPS_POS  = 0x04,   ///< 主控 GPS 定位 (lat/lon/alt/卫星数)
    CTRL_MPU_TYPE_GPS_NAV  = 0x05,   ///< 主控 GPS 运动+精度+时间 (航向/地速/DOP/UTC)
} ctrl_mpu_type_t;

/* GPS 状态帧 (type=0x04 / 0x05) flags 位 */
#define CTRL_GPS_FLAG_FIX    0x01    ///< 定位有效 (经纬度可信)
#define CTRL_GPS_FLAG_TIME   0x02    ///< 模块时间有效 (TIME 帧已收到)

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
    /* v9.2 SERVO (cmd=0x12) 指令: 云台手动 */
    uint8_t servo_mask;       /* bit0=下发 ch0, bit1=下发 ch1 (0=不下发任何通道) */
    int16_t servo_deg10[2];   /* 目标标称角 ×0.1° (ch0 / ch1) */
} ctrl_command_t;

/* MPU 数据 */
typedef struct {
    uint8_t  type;
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;
} ctrl_mpu_data_t;

/* 主→远端 转发回调 */
typedef void (*ctrl_forward_cb_t)(const ctrl_command_t *cmd);

/* 本地指令回调 (v5.12): control 解析出控制帧后调本回调, 由 main.c 执行本地动作
 * (差速混合 + 4 路电调 + L298N 滚筒 / 急停 / 重启 / 关机).
 * 由 TCP 任务上下文调用, 频率可达 20Hz —— 实现里**不要 printf**, 否则刷屏. */
typedef void (*ctrl_local_cb_t)(const ctrl_command_t *cmd);

/* ========== API ========== */

size_t control_process(const uint8_t *data, size_t len);
void   control_set_forward_callback(ctrl_forward_cb_t cb);
void   control_set_local_callback(ctrl_local_cb_t cb);
void   control_reset(void);

/* 帧构造器 */
void control_build_ctrl_frame(uint8_t *frame,
    uint8_t cmd, int8_t speed, int8_t yaw,
    uint8_t remote_light, int8_t bucket_speed, uint8_t flags);

void control_build_mpu_frame(uint8_t *frame,
    uint8_t type,
    int16_t ax, int16_t ay, int16_t az,
    int16_t gx, int16_t gy, int16_t gz);

/* GPS 状态帧 type=0x04 (定位): [3-6] lat ×1e7  [7-10] lon ×1e7
 * [11-12] alt ×0.1m  [13] 卫星数  [14] flags (CTRL_GPS_FLAG_*) */
void control_build_gps_pos_frame(uint8_t *frame,
    uint8_t flags, int32_t lat_e7, int32_t lon_e7, int16_t alt_dm, uint8_t sats);

/* GPS 状态帧 type=0x05 (运动/精度/时间): [3-4] 航向×0.01°  [5-6] 地速×0.01km/h
 * [7] pdop  [8] hdop  [9] vdop (均 ×0.1, 上限 25.5)  [10-12] 时 分 秒  [13] flags */
void control_build_gps_nav_frame(uint8_t *frame,
    uint8_t flags, uint16_t course_cdeg, uint16_t speed_ckmh,
    uint8_t pdop_d1, uint8_t hdop_d1, uint8_t vdop_d1,
    uint8_t hour, uint8_t minute, uint8_t second);

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
