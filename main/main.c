/**
 * @file main.c
 * @brief ESP32-S3 W5500 AUV 网络控制器 (主控制节点, TCP Server)
 *
 * [当前模式 - 启用: W5500 以太网 + **TCP Server 8080 (上位机内网操控)**
 *   + 水面 4 路电调 (推进 ESC1=IO41 左 / ESC2=IO42 右, 反推 REV1=IO39 左 / REV2=IO40 右;
 *     ⚠️ 反推**暂时关闭** `REV_ESC_ENABLE=0`: 负油门按停处理, 方向线只保持在正向半区,
 *     控制台 `l <v>` / `r <v>` / 裸数字) + L298N 滚筒收放电机 (ENA=38 / IN1=48 / IN2=47)
 *   + 云台舵机 (PCA9685 ch0/ch1): **控制台 `p`/`g`/`n`/`z` 与上位机拖动条 (cmd=0x12) 双路手动**
 *   + GPS 状态回传 (惯导 GPS → 上位机, type=0x04/0x05, 2Hz, v9.1) + 串口控制台;
 *  总开关现状: `W5500/MOTOR/NAV/SERVO = 1`, `REV_ESC_ENABLE = 0` (反推暂时关闭),
 *             `MPU6050_ENABLE = 0` (**云台 MPU 暂时废弃**: 不初始化/不自检, 姿态解算不启动,
 *             上位机「主控 MPU」回传自动停发); `TEST_MODE = 0` = 云台纯开环 (不自动动作)。
 *  未启用: 8081 远端转发 (水下节点未就绪)。]
 *
 * 角色: 主控制节点 (核心控制 + 数据处理 + 指令分发)
 * 硬件: W5500 + 4 ESC (2 推进 + 2 反推) + 1 L298N (滚筒收放) + 惯导模块 (GPS+10 轴 IMU)
 *       (未接: MPU6050 / PCA9685)
 * 网络: TCP Server, **仅 8080 (上位机)**; 8081 (远端) 暂未启用
 *   - 8080: 上位机 (笔记本) - 下发 16B 控制帧, 收 16B 状态帧
 *   - 8081: 远端 ESP32-S3 节点 - 转发控制 + 双向 MPU 推送 (暂未启用, 设计见参数总览 §2)
 *
 * 集成组件:
 *   - wiznet    : 板载 W5500 以太网 (ioLibrary, 20MHz SPI polling, 100M FULL)
 *   - nav       : **惯导模块** (亚博 GPS+10 轴 IMU 一体, WIT 0x55 协议主动上报,
 *                 UART2: ESP32 RX=16 <- 模块 TX, ESP32 TX=18 -> 模块 RX)
 *                 —— 取代了旧的"亚博 10 轴 IMU (7E 23 请求式)"与"旧 GPS (NMEA/UART1 2/1/3)"
 *   - imu       : MPU6050 (I2C1 共用总线 16/17, 0x68) —— 只剩云台这一颗
 *   - servo     : PCA9685 (I2C1: SDA=16, SCL=17)
 *   - motor     : 4 ESC (LEDC, 42/41/40/39) + L298N (ENA=38, IN1=48, IN2=47)
 *   - control   : 16B 控制帧 + 16B 状态帧**协议解析**; 差速混合与电机驱动在本文件
 *                 (control 只解析, 通过 control_set_local_callback 回调上来, 见 §2)
 *
 * ⚠️ 16/18 的用途: v5.11.2 起云台 MPU6050 改与 PCA9685 共用 I2C1 (**v8.0 起 SDA=16/SCL=17**), 曾腾出的 16/18
 *    现给**惯导模块的 UART2**。因此 imu 初始化必须在 servo_init() 之后 (见 4c)。
 *
 * 🆕 v5.12 网络联调: 恢复 W5500 + TCP Server (8080) 接收上位机控制帧驱动电调/滚筒;
 *    差速混合 left=clamp(speed+yaw) / right=clamp(speed-yaw), 正=推进·负=反推 (复用 esc_apply_side);
 *    bucket_speed 驱动 L298N 滚筒收放; **500ms 收不到控制帧自动全部停机** (失联保护)。
 *
 * 🆕 v6.0 惯导替换: 旧 GPS (NMEA/UART1) 与旧"亚博 10 轴 IMU"(`7E 23` 请求式/UART2)
 *    在本工程里**全部删除**, 换成一块"亚博 GPS+10 轴 IMU 一体惯导模块":
 *    单条 UART2 + 维特(WIT) 0x55 协议**主动上报** (帧 0x50~0x5A), 由 components/nav
 *    统一解析出姿态与 GPS 两套快照。控制台 `gps` / `hull` 的数据源都改成了它,
 *    另加 `nav` 看模块状态 (在线/配置/波特率/帧计数)。详见参数总览 §7.10 与 nav.h。
 *
 * 任务清单 (当前实际创建):
 *   - tcp_server         优先级 5, 栈 8192, W5500 socket 0 (8080) 收发 + 失联保护 (W5500_ENABLE=1 时)
 *   - nav_task           优先级 4, 栈 3072, 惯导模块 UART 接收/解析 (nav 组件内部创建)
 *   - nav_gps_log        优先级 4, 栈 4096, 1Hz 惯导 GPS 日志 (NAV_ENABLE=1 时创建, 默认静默)
 *   - nav_att_log        优先级 4, 栈 4096, 10Hz 惯导姿态日志 (NAV_ENABLE=1 时创建, 默认静默)
 *   - attitude           优先级 5, 栈 4096, 100Hz Madgwick 姿态解算 (MPU6050_ENABLE=1 时才启动)
 *   - ch0_test/ch1_test  优先级 4, 栈 3072, 舵机扫描测试 (SERVO_ENABLE=0 时启动即自退出)
 *   - stab_test          优先级 4, 栈 4096, 云台闭环自稳 (TEST_MODE==4; SERVO_ENABLE=0 时启动即自退出)
 *   - console_repl       优先级 5, 栈 4096, 串口标定控制台
 * 未创建 (设计保留在参数总览 §2, 代码待恢复): mpu_push_task / status_report_task / 8081 转发子帧
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ⚠️ include 顺序要求: **FreeRTOS(xtensa 系统头) 必须在 ioLibrary 之前**。
 *    原因见 components/wiznet/include/wiznet_conf.h —— 那里会 #undef 掉 Xtensa 的 `MR`,
 *    好让 w5500.h 的 `MR` 成为首次定义 (否则 "MR redefined" 警告必出且无法用开关关闭)。 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "wiznet_conf.h"
#include "wizchip_conf.h"
#include "wiznet_socket.h"

#include "esp_log.h"
#include "esp_console.h"
#include "linenoise/linenoise.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/gpio.h"          /* gpio_config: 板载 WS2812 数据脚 (GPIO21) 钉低
                                   * (wiznet_spi.h 也会带进来, 这里显式包含) */
#include "wiznet_manager.h"
#include "wiznet_spi.h"
#include "motor.h"
#include "imu.h"
#include "servo.h"
#include "gimbal.h"
#include "nav.h"
#include "control.h"

/* ========== TCP 服务器配置 ==========
 * 只启用 socket 0 = 8080 (上位机)。8081 (远端 ESP32-S3 水下节点) 暂未启用:
 * 水下节点未就绪, 转发子帧/MPU 推送的设计保留在参数总览 §2, 需要时再加 socket 1。 */
#define TCP_HOST_PORT     8080    /* 上位机 (笔记本) */
#define HOST_SOCK         0
#define RX_BUFFER_SIZE    1024

/* 失联保护: 已连接且收到过控制帧, 但超过这么久没有新帧 ⇒ 全部电机归零。
 * 上位机正常按 20Hz 下发, 500ms = 连续丢 10 帧, 足以区分"网络抖动"与"真失联"。 */
#define TCP_LINK_TIMEOUT_MS  500

/* 硬件引脚定义 */

/* (v6.0 删除) 旧 GPS 的 UART1 引脚 TX=GPIO2 / RX=GPIO1 / PPS=GPIO3 —— 硬件与驱动都已移除,
 *   2/1/3 三个脚现已空闲; 惯导模块的引脚在 components/nav 里 (UART2: RX=16 / TX=18)。 */

static uint8_t s_rx_buffer[RX_BUFFER_SIZE];

static const char *TAG = "APP";

/* ========== 网络连接状态 (TCP 任务读写; 控制台 `net` 只读) ========== */
static volatile bool     s_host_connected = false;  /* socket 0 (8080) 是否已建立 */
static volatile int64_t  s_last_ctrl_us   = 0;      /* 最近一次有效控制帧的时刻 (0=尚未收到) */
static volatile uint32_t s_ctrl_frames    = 0;

/* ========== 主控 MPU 状态回传 (v8.1) ==========
 * 上位机 `tools/tcp_console.py` 的「主控 MPU」面板收 **type=0x01** 的 16B 状态帧
 * (0xBB 0x66 + 6×int16 LE 原始 6 轴), 按 20Hz 刷新。
 * ⚠️ **不在 TCP 任务里读 I2C** —— 会与 100Hz 的 `attitude_task` 抢同一条 I2C1:
 *    由 `attitude_task` 顺手把换算好的 int16 塞进 `s_mpu_raw[]`, TCP 任务只取快照发送。
 * 换算 (参数总览 §7.5.5): 加速度 ±2g → ×16384 LSB/g; 陀螺仪 ±250°/s → ×131 LSB/(°/s)。 */
#define MPU_PUSH_PERIOD_US    50000     /* 20Hz */
static volatile int16_t s_mpu_raw[6]    = {0};     /* ax ay az gx gy gz (LSB) */
static volatile bool    s_mpu_raw_valid = false;   /* attitude_task 至少填过一次 */
static int64_t          s_mpu_next_us   = 0;       /* 下次发送时刻 (0 = 重连后立刻发) */      /* 累计收到的控制帧数 */

/* ========== 协议实现 (v5.12) ==========
 *
 * 分工: `control` 组件只做**协议解析** (帧同步 / CRC / 字段解码), 解析出的
 * ctrl_command_t 通过 control_set_local_callback() 回调到本文件的
 * tcp_apply_local_command(), 由本文件按当前硬件执行差速混合与电机驱动。
 * 原因: 原先 control.c 内部写死了旧硬件 (2 路电调 + L298N 由 |speed| 驱动),
 *       把负油门发给单向电调会被 motor_set_esc_throttle() 钳成 0, 反推两路永不动。
 *
 * 三个实现函数在文件后面 (它们要用 esc_apply_side / motor_apply_speed, 定义在控制台命令区之后):
 *   tcp_apply_local_command()        — 差速混合 + 4 路电调 + L298N 滚筒 (静默, 20Hz 不打印)
 *   tcp_server_task()                — TCP Server 8080 主循环 + 500ms 失联保护
 *   tcp_server_forward_mpu_to_host() — 状态回传占位 (当前无 MPU 数据源, 见文件头 v5.12 说明)
 *
 * 🗑️ v5.12 删除: 原 forward_to_remote / handle_socket / tcp_server_task / mpu_push_task /
 *    status_report_task 那一大段**注释代码**。它们按旧硬件写死 (2 路电调 / 8081 转发 /
 *    L298N 由 |speed| 驱动), 与现状不符, 留着只会误导。8081 转发与 MPU 推送的**设计**
 *    保留在参数总览 §2, 将来要用时照那章重新实现即可。 */

/* ========== 任务实现 ========== */

/* ========== [临时测试] PCA9685 + MPU6050 俯仰角控制任务 (Madgwick AHRS) ========== */

#define PITCH_SERVO_CH      1       /* PCA9685 通道 1, 控制俯仰 */
#define PITCH_SERVO_MIN_DEG 30.0f   /* 舵机最小角度 (对应最大俯仰) */
#define PITCH_SERVO_MAX_DEG 150.0f  /* 舵机最大角度 (对应最小俯仰) */
#define PITCH_RANGE_MIN    -45.0f   /* 输入俯仰角下限 (°) */
#define PITCH_RANGE_MAX     45.0f   /* 输入俯仰角上限 (°) */
#define PITCH_LOG_INTERVAL  5       /* 每 N 次打印一次 log */
#define AHRS_BETA           0.1f    /* Madgwick 滤波收敛速度, 越大越信加速度 */
#define AHRS_UPDATE_HZ      100     /* 姿态解算频率 (Madgwick 需要 >50Hz) */
#define AHRS_PRINT_DIV      50      /* 每 N 次解算打印一次 (即 2Hz) */
#define AHRS_CAL_SAMPLES    100     /* 启动时陀螺仪零偏标定采样数 (约 1s) */
/* 零偏标定前的等待: 开机时 servo_init 会把舵机都拉到回中位, 若云台原本不在
 * 中位, 这段位移会带着 MPU 一起转, 标定出来的零偏就是错的 (实测曾出现 Z 轴
 * +97°/s 的荒谬值). 必须等舵机走到位并停稳后再标定. */
#define AHRS_CAL_DELAY_MS   3000
/* 会驱动舵机的测试任务统一等这么久再动云台:
 * AHRS_CAL_DELAY_MS(等舵机回中到位) + AHRS_CAL_SAMPLES*10ms(采样) + 300ms 余量 */
#define AHRS_CAL_READY_MS   (AHRS_CAL_DELAY_MS + AHRS_CAL_SAMPLES * 10 + 300)

/**
 * @brief Madgwick AHRS 单步更新
 *
 * 输入: 加速度 (g) 和 陀螺仪 (rad/s), 输出四元数 q[4] = {w, x, y, z}
 * beta: 滤波增益, 典型 0.04~0.2
 */
static void madgwick_update(float ax, float ay, float az,
                            float gx, float gy, float gz,
                            float *q, float beta, float dt)
{
    float recip_norm;
    float s0, s1, s2, s3;
    float q_dot1, q_dot2, q_dot3, q_dot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    /* 加速度归一化 */
    recip_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (recip_norm == 0.0f) return;  /* 避免除零 */
    recip_norm = 1.0f / recip_norm;
    ax *= recip_norm;
    ay *= recip_norm;
    az *= recip_norm;

    /* 预计算 */
    _2q0 = 2.0f * q[0];
    _2q1 = 2.0f * q[1];
    _2q2 = 2.0f * q[2];
    _2q3 = 2.0f * q[3];
    _4q0 = 4.0f * q[0];
    _4q1 = 4.0f * q[1];
    _4q2 = 4.0f * q[2];
    _8q1 = 8.0f * q[1];
    _8q2 = 8.0f * q[2];
    q0q0 = q[0] * q[0];
    q1q1 = q[1] * q[1];
    q2q2 = q[2] * q[2];
    q3q3 = q[3] * q[3];

    /* 梯度下降法估计四元数变化率 */
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q[1] - _2q0 * ay - _4q1 +
         _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
         _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q[3] - _2q1 * ax + 4.0f * q2q2 * q[3] - _2q2 * ay;

    recip_norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (recip_norm == 0.0f) return;
    recip_norm = 1.0f / recip_norm;
    s0 *= recip_norm;
    s1 *= recip_norm;
    s2 *= recip_norm;
    s3 *= recip_norm;

    /* 四元数导数 = 陀螺仪积分 - beta * 梯度修正 */
    q_dot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz) - beta * s0;
    q_dot2 = 0.5f * ( q[0] * gx + q[2] * gz - q[3] * gy) - beta * s1;
    q_dot3 = 0.5f * ( q[0] * gy - q[1] * gz + q[3] * gx) - beta * s2;
    q_dot4 = 0.5f * ( q[0] * gz + q[1] * gy - q[2] * gx) - beta * s3;

    /* 积分 */
    q[0] += q_dot1 * dt;
    q[1] += q_dot2 * dt;
    q[2] += q_dot3 * dt;
    q[3] += q_dot4 * dt;

    /* 四元数归一化 */
    recip_norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (recip_norm == 0.0f) return;
    recip_norm = 1.0f / recip_norm;
    q[0] *= recip_norm;
    q[1] *= recip_norm;
    q[2] *= recip_norm;
    q[3] *= recip_norm;
}

/**
 * @brief 四元数转欧拉角 (yaw/pitch/roll)
 *
 * 返回: ypr[0]=yaw, ypr[1]=pitch, ypr[2]=roll (单位: 弧度)
 */
static void quaternion_to_ypr(const float *q, float *ypr)
{
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

    /* yaw (z-axis rotation) */
    ypr[0] = atan2f(2.0f * (q1 * q2 + q0 * q3),
                    q0q0 + q1q1 - q2q2 - q3q3);

    /* pitch (y-axis rotation) */
    ypr[1] = asinf(-2.0f * (q1 * q3 - q0 * q2));

    /* roll (x-axis rotation) */
    ypr[2] = atan2f(2.0f * (q0 * q1 + q2 * q3),
                    q0q0 - q1q1 - q2q2 + q3q3);
}

/* 姿态日志开关 (默认关; C1-MPU 关系测定任务已自带倾角输出, 需要融合结果再输 att 1) */
static volatile bool s_att_log_enabled = false;

/* 惯导 GPS 1Hz 日志开关 (默认关, 用 gps 1 开启) */
static volatile bool s_gps_log_enabled = false;

/* 惯导姿态 10Hz 日志开关 (默认关, 用 hull 1 开启; 不带参数读一帧) */
static volatile bool s_hull_log_enabled = false;

/* 姿态任务输出的最新欧拉角 (°)
 * 供其它任务读取, 这样它们不必自己再读 I2C —— imu 组件没有互斥保护,
 * 多任务并发调 imu_read 会有数据竞争风险. */
static volatile float s_att_yaw_deg   = 0.0f;
static volatile float s_att_pitch_deg = 0.0f;
static volatile float s_att_roll_deg  = 0.0f;

/* 姿态任务发布的原始传感器值 (°/s, g) 与融合循环计数
 * count 用于判断姿态任务是否还在正常跑 (计数不涨 = 任务卡死) */
static volatile float    s_att_gx    = 0.0f;
static volatile float    s_att_gy    = 0.0f;
static volatile float    s_att_gz    = 0.0f;
static volatile float    s_att_ax    = 0.0f;
static volatile float    s_att_ay    = 0.0f;
static volatile float    s_att_az    = 0.0f;
static volatile uint32_t s_att_count = 0;

/* ch0 (360° 舵机) 档位扫描测试开关 (默认关闭, 用 t0 1 开启) */
static volatile bool s_ch0_test_enabled = false;

/* ch1 (180° 舵机) 角度扫描测试开关 (默认关闭, 用 t1 1 开启) */
static volatile bool s_ch1_test_enabled = false;

/* ch0 扫描参数 */
#define CH0_SWEEP_MIN_US    1000    /* 循环扫描起点脉宽 (µs) */
#define CH0_SWEEP_MAX_US    2000    /* 循环扫描终点脉宽 (µs) */
#define CH0_SWEEP_STEP_US   25      /* 每档步进 (µs) */
#define CH0_SWEEP_DWELL_MS  700     /* 每档停留时间 (ms) */

/* ch0 上下限测量扫描: 开机单向扫一次, 用于测定 360° 舵机可响应的脉宽范围 */
#define CH0_SCAN_ONCE       0       /* 置 1 开启 */
#define CH0_SCAN_MIN_US     500     /* 扫描起点脉宽 (µs) */
#define CH0_SCAN_MAX_US     2800    /* 扫描终点脉宽 (µs) */
#define CH0_SCAN_STEP_US    100     /* 每档步进 (µs) */
#define CH0_SCAN_DWELL_MS   1000    /* 每档停留时间 (ms) */

/* ch0 固定脉宽测试: >0 时开机把 ch0 定死在该脉宽 (0 = 关闭, 按标定回中点)
 * 注: 现在串口控制台已支持"直接输入数字改 ch0 脉宽", 故默认关闭. */
#define CH0_HOLD_US         0

/* ch0 两点往返测试: A 点 <-> B 点循环, 每点停留 CH0_ALT_DWELL_MS */
#define CH0_ALT_ENABLE      0       /* 置 1 开启, 同时把 CH0_HOLD_US 置 0 */
#define CH0_ALT_A_US        530     /* A 点脉宽 (µs) */
#define CH0_ALT_B_US        1630    /* B 点脉宽 (µs) = 标定中点 */
#define CH0_ALT_DWELL_MS    2000    /* 每点停留时间 (ms) */

/* ch1 扫描参数 */
#define CH1_SWEEP_MIN_DEG   30.0f   /* 扫描起点角度 (°): 与软件行程限位一致, 不碰机械端点 */
#define CH1_SWEEP_MAX_DEG   150.0f  /* 扫描终点角度 (°) */
#define CH1_SWEEP_STEP_DEG  10.0f   /* 每档步进 (°) */
#define CH1_SWEEP_DWELL_MS  700     /* 每档停留时间 (ms) */

/* ch1 开机默认角度 = 回中(中位) 90° (按 ch1 标定 600~2700µs: 0°=600, 90°=1650, 180°=2700µs) */
#define CH1_HOME_DEG        90.0f

/* 【实测记录】ch1 机械安装方向:
 *   命令角 0° -> 180° 时, 云台实际顺时针转动, 与命令方向一致 => 无需取反.
 *   (先前曾误判为逆时针, 已更正)
 */
#define CH1_DIR_INVERT      0       /* 0 = 方向一致 (0°->180° 顺时针) */

/* ============ 【实测记录】舵机 <-> MPU 关系 (闭环控制的基础) ============
 *
 * 两套角度刻度 (详见 servo.h 的 servo_cal_t 说明):
 *   标称角 cmd  : 报文/控制台/上位机用, 好看好读 —— ch1: 0~180°, ch0: 0~360°
 *   物理角 φ    : 云台真实转角 = MPU 直接测到的量 —— ch1: 0~192°, ch0: 0~365°
 *   换算: φ = cmd × (phys_range_deg / range_deg), 只发生在 servo 组件内部
 *
 * --- ch1 (俯仰轴) ---
 *   * 轴向: 确认是俯仰轴 (Δpitch 主导, Δroll 仅 ±3°; 残差说明 MPU 有约 5° 安装偏斜)
 *   * 转向: **反向** —— 命令角增大, 物理角增大, 但 MPU 的 pitch **减小**
 *           pitch = 33.3° − 0.998 × φ        (物理域, 斜率 = -1, 1:1)
 *           pitch =  1.4° − 1.064 × (cmd − 30°)  (标称域, 含 192/180 的比例)
 *   * 基准: 标称角 30° 时 pitch = +1.4°、roll = -1.9°
 *   * 闭环做法: 用**物理角**做反馈, 误差 e = pitch_meas − pitch_des,
 *              下发 cmd += e × (range_deg / phys_range_deg)
 *              (即 servo_phys_to_cmd_deg), 控制律里不出现任何修正系数.
 *
 * --- ch0 (竖直轴 / 平面旋转) ---
 *   * 轴向: 与 MPU 的 Z 轴重合 (偏斜约 2.7°)
 *   * 转向: 同向 (标称角增大 -> 融合 yaw 增大), 不需要取反
 *   * 比例: 命令 360° -> 实测 yaw +365.4° (1:1, 物理域同样斜率 1)
 */

/* ch1 端点测量扫描: 开机单向扫一次, 用于测定舵机真实行程两端脉宽
 * 实测已完成: 真实量程 600~2700µs (0~180°), 故置 0 关闭;
 * 以后换舵机/重新标定时改回 1 并按需调整范围. */
#define CH1_SCAN_ONCE       0       /* 置 1 开启端点扫描 */
#define CH1_SCAN_MIN_US     2550    /* 扫描起点脉宽 (µs) */
#define CH1_SCAN_MAX_US     2760    /* 扫描终点脉宽 (µs) */
#define CH1_SCAN_STEP_US    20      /* 每档步进 (µs) */
#define CH1_SCAN_DWELL_MS   1000    /* 每档停留时间 (ms) */

/* 板载 WS2812 RGB LED 的数据脚 = **GPIO26** (用户按原理图/实测确认; **不在 40Pin 排针上**,
 * 属板内网络。Waveshare 官方 wiki 的 `RGB_LED` 示例写 GPIO21 —— 那与本板实况不符, 忽略)。
 *   ⚠️ 为什么它会常亮刺眼的白光: WS2812 把数据脚上的**任意边沿**当颜色数据采样 —— 只要该脚
 *      被当 I2C SDA / SPI / 普通翻转用, 锁存值就是随机的; 白色 = 三通道全开最亮, 所以最显眼。
 *      而且它会**一直保持最后锁存的值** (不掉电不清除) ⇒ 必须 ① 不再给它任何边沿, ② **断电重上一次**。
 *   ⚠️ 以后要拿它当状态灯, 必须用 **RMT** 精确时序驱动 (位宽容差 ±150ns, 普通 GPIO 打拍不达标)。*/
#define BOARD_RGB_LED_GPIO  26

/* ========== 测试任务选择 ==========
 * 同一时刻只能有一个"驱动舵机"的测试任务在跑, 否则两条指令流会互相打架.
 *   0 = 无测试任务 (两舵机保持开机回中的中位, 不动作) —— **当前**
 *   1 = 云台运动规划测试 (速度/加速度受限, 用于抑制大惯量载荷的冲击与过冲)
 *   2 = ch1 舵机 <-> MPU 关系测定 (俯仰轴, 用重力矢量夹角)
 *   3 = ch0 舵机 <-> MPU 关系测定 (平面旋转轴, 用融合后的偏航角)
 *   4 = 云台闭环自稳测试 (ch1 俯仰闭环, 开机自动开启并打印误差)
 *
 * ⚠️ 当前取 0 的原因: **云台暂时不接 MPU 反馈** —— MPU6050 的用途还没定,
 *    云台先做"纯开环"测试 (位置/标定/行程), 不跑任何"每周期抢写舵机"的任务:
 *      · 没有 2/3/4 ⇒ 不会开机自动动作、不会把手动指令 (`p`/`n`/`z`/`ck`) 抢回去;
 *      · `t0 1` / `t1 1` 两个扫描任务仍可用 (它们不依赖 MPU, 且默认关闭);
 *      · 闭环命令 `ge`/`gt`/`gp` 仍注册着, 但**没人自动开**, 要手动 `ge 1 1` 才会用 MPU 反馈
 *        —— 真要用闭环时, 请同时确认 MPU6050_ENABLE=1 且云台 MPU 已接好。
 */
#define TEST_MODE           0

/* [总开关] PCA9685 舵机 (云台) —— 当前启用
 *   0 = 关停: 不探测/不驱动 PCA9685, 云台运动规划与闭环自稳测试都不启用。
 *             ⚠️ 是否仍调 servo_init() 只看"还有没有别的 I2C1 使用者":
 *                云台 MPU6050 与 PCA9685 共线, 且只是**借用**总线 ⇒ MPU6050_ENABLE=1 时仍需它装;
 *                (v6.0 起船体姿态改由 components/nav 走 UART2, 不再占用 I2C1)
 *                两者都为 0 时跳过 servo_init(), 连总线都不装。
 *   1 = 启用 (当前): 恢复正常 (舵机/闭环/TEST_MODE 全生效)。
 */
#define SERVO_ENABLE        1

/* [总开关] 云台 MPU6050 + 姿态解算 (attitude_task) —— **当前暂时关停**
 *   ⚠️ MPU6050 与 PCA9685 **共用 I2C1** (**v8.0 起 SDA=16 / SCL=17**), 不再是 I2C0 (原 16/18);
 *      所以它的初始化在 servo_init() 之后 (见 start_components 的 4c)。
 *   0 = 关停 (当前): **不初始化、不做地址自检、也不读有效性** —— 不会出现"引脚自检/0x68 无应答/
 *       未找到 MPU6050"这类 ERROR; `attitude_task`(100Hz 姿态解算) 不创建, 因此
 *       ① 上位机「主控 MPU」面板恒为 `--` (回传自动停发, 无数据源);
 *       ② 云台闭环 (`ge`/`gt`/`gp`) 失去反馈源, 只能当空操作 (要闭环请先把本开关置 1)。
 *       惯导模块 (nav, UART2) 完全不受影响。
 *   1 = 启用: 恢复云台姿态/闭环反馈 (原行为)。
 *   📌 当前云台的定位是"**上位机/控制台手动摆位**"(见 cmd=0x12 与 §7.9), 不依赖 MPU。
 */
#define MPU6050_ENABLE      0

/* [总开关] 惯导模块 (亚博 GPS + 10 轴 IMU 惯导组合, WIT 0x55 协议) —— 当前启用
 *   说明: 硬件其实是**两块板** (GPS 板 + 10 轴 IMU 板), 官方"融合"后由**一条 UART** 输出,
 *         上位机侧当作**一个**模块用 —— 同一帧流里既有姿态也有 GPS。
 *   接口: UART2, ESP32 RX=IO1 <- 模块 TX / ESP32 TX=IO2 -> 模块 RX, 9600 8N1
 *         (波特率初始化时自扫描 9600~230400, 见 nav.h)
 *   0 = 关停: 不装 UART2、不初始化、不创建日志任务; 控制台 `gps` / `hull` / `nav`
 *             只提示模块未就绪。**components/nav 组件与接口代码原样保留**, 置 1 即恢复。
 *   1 = 启用 (当前): nav_init() 装 UART + 按需配置模块输出 (RSW=0x058F, RRATE=5Hz) + 收数据。
 *   注: 该模块**取代**了旧的"亚博 10 轴 IMU"(`7E 23` 请求式)与旧 GPS (NMEA/UART1),
 *       那两套代码在本工程里已删除。
 */
#define NAV_ENABLE          1

/* [总开关] 电机 —— 水上 4 路电调 (2 推进 + 2 反推) + L298N 滚筒收放
 *   1 = 初始化全部 5 个通道:
 *         推进 ESC1/ESC2 -> IO41(左) / IO42(右)   帧率 50Hz, 单向 SkyWalker V2
 *         反推 REV1/REV2 -> IO39(左) / IO40(右)   同上
 *         L298N          -> ENA=IO38 (5kHz PWM 调速) / IN1=IO48 / IN2=IO47 (方向)
 *   0 = 完全不初始化电机 —— ⚠️ 注意: 这会让 GPIO38/47/48 变回输入(悬空),
 *       拔掉 ENA 跳线帽时 L298N 的输入悬空可能误动作, 所以**保持 1 更安全**
 *       (引脚被主动驱动为 ENA=0 / IN1=IN2=0 = 停机)。
 * 控制入口: 控制台 `l/r <-100~100>` (电调)、`m <-100~100>` (L298N 滚筒);
 *           TCP 联调时由上位机帧驱动 —— speed/yaw 差速混合给 4 路电调,
 *           bucket_speed 给 L298N 滚筒, 见 tcp_apply_local_command()。
 * 反推另行开关: 见下面 REV_ESC_ENABLE。
 */
#define MOTOR_ENABLE        1

/* [总开关] 反推 (电调"反推刹车"的方向线: REV1=IO39 左 / REV2=IO40 右)
 *   0 = **暂时关闭** (当前): 控制台 / TCP 送来的**负油门一律按 0 (停) 处理** ⇒ 船不能倒退,
 *       转向只能靠"两侧推力差"(不能靠一侧反转做原地转向)。
 *       两路方向线**仍然初始化并保持在 0% (1100µs = 正向半区)** —— 这是 SkyWalker V2
 *       说明书要求的上电状态, 让它悬空反而有被干扰进"反转半区"的风险; 只是不再往反转半区打。
 *   1 = 恢复反推: 负油门 = 方向线 100% (反转半区) + 油门线给速度, 见 esc_apply_side()。
 * ⚠️ 恢复反推还需要电调参数"刹车类型"设为**反推刹车** (出厂默认"无刹车" ⇒ 方向线无效),
 *    这一步就是"反转需要额外开启"的东西, 暂时不做。 */
#define REV_ESC_ENABLE      0

/* [总开关] W5500 以太网 + TCP Server (8080 上位机)
 *   0 = 关停: 不初始化 W5500, 不等 link up, 也不创建 tcp_server_task。
 *   1 = 启用 (当前): 初始化 + 配静态 IP 192.168.29.10/24 (网关 .1), link up 后串口打印
 *             IP/掩码/网关; 网线未插则等 30s 超时后继续启动。
 *             随后创建 tcp_server_task: 监听 8080 收上位机 16B 控制帧驱动电调/滚筒,
 *             并带 500ms 失联保护 (TCP_LINK_TIMEOUT_MS)。8081 (远端) 未启用。
 */
#define W5500_ENABLE        1

/* 闭环自稳测试参数 */
#define STAB_TARGET_DEG     0.0f    /* 俯仰闭环目标物理角: 0 = 保持水平 */
#define STAB_SETTLE_MS      2000    /* 开闭环前的静置时间 (等舵机到位停稳) */

/* 云台运动规划测试参数 */
#define GIMBAL_TEST_CH      0       /* 被测通道 (0 = 平面旋转, 360°) */
#define GIMBAL_TEST_A_DEG   0.0f    /* A 端点角度 */
#define GIMBAL_TEST_B_DEG   360.0f  /* B 端点角度 */
#define GIMBAL_MAX_VEL_DPS  60.0f   /* 限速 (°/s), 建议 30~90 起调 */
#define GIMBAL_MAX_ACC_DPS2 120.0f  /* 限加速 (°/s²), 建议 60~180 起调 */
#define GIMBAL_HOLD_MS      2000    /* 两端停留时间 (ms) */

/**
 * @brief 三维姿态解算任务 (Madgwick AHRS)
 *
 * 以 100Hz 读取 MPU6050, 融合加速度计(重力参考) + 陀螺仪(积分),
 * 输出四元数并转换为 yaw/pitch/roll 欧拉角, 每 0.5s 打印一次.
 *
 * 说明:
 *   - 无磁力计, yaw 会缓慢漂移 (这是 6 轴 IMU 的固有特性, 无法绝对定北)
 *   - 启动时先做陀螺仪零偏标定, 需保持静止约 1s
 *   - 约定: 模块水平放置且 Z 轴朝上时, roll/pitch ≈ 0
 */
/* ⚠️ 只在云台 MPU6050 启用时编译该任务: 关停时没人创建它, 留着只会产生
 *    -Wunused-function 警告 (它引用的 s_att_* 与 Madgwick 函数仍被其它任务使用)。*/
#if MPU6050_ENABLE
static void attitude_task(void *pvParameters)
{
    ESP_LOGI(TAG, "[AHRS] 三维姿态解算任务启动 (%dHz, Madgwick β=%.2f)",
             AHRS_UPDATE_HZ, AHRS_BETA);

    const TickType_t period = pdMS_TO_TICKS(1000 / AHRS_UPDATE_HZ);
    TickType_t last_wake = xTaskGetTickCount();

    imu_data_t m = {0};
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};   /* 单位四元数 (初始姿态) */
    float ypr[3] = {0};

    /* ---- 启动阶段: 陀螺仪零偏标定 (静止采样取均值) ---- */
    float bias_x = 0.0f, bias_y = 0.0f, bias_z = 0.0f;
    int   cal_ok = 0;
    /* 先等舵机回中到位 (见 AHRS_CAL_DELAY_MS 说明), 否则会把转动当成零偏 */
    vTaskDelay(pdMS_TO_TICKS(AHRS_CAL_DELAY_MS));
    ESP_LOGI(TAG, "[AHRS] 陀螺仪零偏标定中, 请保持静止...");
    for (int i = 0; i < AHRS_CAL_SAMPLES; i++) {
        if (imu_read_role(IMU_ROLE_GIMBAL, &m) == ESP_OK) {
            bias_x += m.gx;
            bias_y += m.gy;
            bias_z += m.gz;
            cal_ok++;
        }
        vTaskDelay(pdMS_TO_TICKS(1000 / AHRS_UPDATE_HZ));
    }
    if (cal_ok > 0) {
        bias_x /= (float)cal_ok;
        bias_y /= (float)cal_ok;
        bias_z /= (float)cal_ok;
        ESP_LOGI(TAG, "[AHRS] 零偏标定完成: (%.2f, %.2f, %.2f) dps",
                 bias_x, bias_y, bias_z);
    } else {
        ESP_LOGW(TAG, "[AHRS] 零偏标定失败, 使用 0 偏置");
    }

    /* ---- 主循环: 100Hz 融合 ---- */
    uint32_t n = 0;
    const float dt = 1.0f / (float)AHRS_UPDATE_HZ;

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        if (imu_read_role(IMU_ROLE_GIMBAL, &m) != ESP_OK) {
            continue;
        }

        /* 陀螺仪 °/s → rad/s, 并扣除零偏 */
        float gx = (m.gx - bias_x) * (M_PI / 180.0f);
        float gy = (m.gy - bias_y) * (M_PI / 180.0f);
        float gz = (m.gz - bias_z) * (M_PI / 180.0f);

        /* Madgwick AHRS 融合 -> 四元数 */
        madgwick_update(m.ax, m.ay, m.az, gx, gy, gz, q, AHRS_BETA, dt);
        quaternion_to_ypr(q, ypr);

        /* 发布给其它任务 (它们就不必再并发访问 I2C 了) */
        s_att_yaw_deg   = ypr[0] * (180.0f / M_PI);
        s_att_pitch_deg = ypr[1] * (180.0f / M_PI);
        s_att_roll_deg  = ypr[2] * (180.0f / M_PI);
        s_att_gx        = m.gx;
        s_att_gy        = m.gy;
        s_att_gz        = m.gz;
        s_att_ax        = m.ax;
        s_att_ay        = m.ay;
        s_att_az        = m.az;
        s_att_count++;

        /* v8.1: 顺手存一份"原始 6 轴 → int16 LSB"的快照, 供 TCP 任务 20Hz 回传上位机
         * (换算见上方 s_mpu_raw 注释; 不在这里读第二次 I2C) */
        s_mpu_raw[0] = (int16_t)(m.ax * 16384.0f);
        s_mpu_raw[1] = (int16_t)(m.ay * 16384.0f);
        s_mpu_raw[2] = (int16_t)(m.az * 16384.0f);
        s_mpu_raw[3] = (int16_t)(m.gx * 131.0f);
        s_mpu_raw[4] = (int16_t)(m.gy * 131.0f);
        s_mpu_raw[5] = (int16_t)(m.gz * 131.0f);
        s_mpu_raw_valid = true;

        /* 喂给云台闭环做反馈 (MPU 与云台同一刚体, pitch/yaw 就是物理角)
         * 注: 本循环在零偏标定之后才开始跑, 所以这里喂的必是有效数据 */
        gimbal_feed_attitude(s_att_pitch_deg, s_att_yaw_deg);

        /* 周期性打印 (2Hz, 默认关闭, 用 att 1 打开) */
        if (s_att_log_enabled && ++n >= AHRS_PRINT_DIV) {
            n = 0;
            ESP_LOGI(TAG, "[ATT] yaw=%7.1f° pitch=%6.1f° roll=%6.1f°",
                     s_att_yaw_deg, s_att_pitch_deg, s_att_roll_deg);
        }
    }
}
#endif /* MPU6050_ENABLE */

/* 惯导 GPS 1Hz 日志任务 (用 gps 1 打开 / gps 0 关闭) —— 数据来自 components/nav */
#if NAV_ENABLE
static void nav_gps_log_task(void *pvParameters)
{
    (void)pvParameters;
    nav_gps_t d;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (!s_gps_log_enabled) {
            continue;
        }
        if (nav_read_gps(&d) != ESP_OK) {
            continue;       /* 模块还没数据: 启动日志里已有提示, 输 nav 看统计 */
        }
        uint64_t now = (uint64_t)esp_timer_get_time();

        ESP_LOGI(TAG, "[GPS] %s 卫星=%u PDOP=%.1f HDOP=%.1f VDOP=%.1f",
                 d.valid ? "定位" : "未定位", (unsigned)d.satellites, d.pdop, d.hdop, d.vdop);
        if (d.valid) {
            ESP_LOGI(TAG, "[GPS] 纬度=%.6f° 经度=%.6f° 速度=%.1fkm/h 航向=%.1f° 海拔=%.1fm",
                     d.latitude, d.longitude, d.speed_kmh, d.course_deg, d.altitude_m);
        } else if (d.last_fix_us == 0) {
            ESP_LOGI(TAG, "[GPS] 未定位 (坐标已清零, 尚未定位成功过)");
        } else {
            ESP_LOGI(TAG, "[GPS] 未定位 (坐标已清零, 上次有效定位 %.1fs 前)",
                     (double)(now - d.last_fix_us) / 1e6);
        }
        if (d.time_valid) {
            ESP_LOGI(TAG, "[GPS] 模块时间 20%02u-%02u-%02u %02u:%02u:%02u",
                     (unsigned)d.year, (unsigned)d.month, (unsigned)d.day,
                     (unsigned)d.hour, (unsigned)d.minute, (unsigned)d.second);
        }
    }
}

/* 惯导姿态 10Hz 日志任务 (默认关, 用 hull 1 开 / hull 0 关);
 * 数据全部来自模块内部解算 (含磁力计补偿), 直接给 roll/pitch/yaw。 */
static void nav_att_log_task(void *pvParameters)
{
    (void)pvParameters;
    nav_imu_t d;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (!s_hull_log_enabled) {
            continue;
        }
        if (nav_read_imu(&d) != ESP_OK) {
            continue;
        }
        ESP_LOGI(TAG, "[HULL] rpy=(%+7.2f %+7.2f %+7.2f)  a=(%+6.3f %+6.3f %+6.3f)g  "
                 "g=(%+7.1f %+7.1f %+7.1f)dps  T=%.1fC",
                 d.roll, d.pitch, d.yaw, d.ax, d.ay, d.az, d.gx, d.gy, d.gz, d.temperature);
    }
}
#endif /* NAV_ENABLE */

/* ========== [临时测试] 串口标定控制台 (esp_console) ========== */

#define CAL_CH_MAX 2    /* 只展示云台用到的 ch0/ch1 */

/** 打印 ch0/ch1 的标定参数与当前角度/脉宽 (cmd_deg=标称角, phys_deg=物理行程) */
static void cal_print_status(void)
{
    printf("ch  min_us  max_us  trim  cmd_deg  phys_deg  limit   center  cur_us  cur_phys\n");
    for (uint8_t ch = 0; ch < CAL_CH_MAX; ch++) {
        servo_cal_t cal;
        uint16_t cur = 0;
        float lo = 0.0f, hi = 0.0f, cmd = 0.0f;
        servo_get_cal(ch, &cal);
        servo_get_pulse_us(ch, &cur);
        servo_get_limit(ch, &lo, &hi);
        servo_get_angle(ch, &cmd);
        int center = ((int)cal.min_us + (int)cal.max_us) / 2 + cal.trim_us;
        printf("%2u  %6u  %6u  %4d  %7u  %8u  %3.0f-%-3.0f  %6d  %6u  %8.1f\n",
               (unsigned)ch, cal.min_us, cal.max_us, cal.trim_us,
               cal.range_deg, cal.phys_range_deg,
               lo, hi, center, cur, servo_cmd_to_phys_deg(ch, cmd));
    }
}

/** sl <ch> <min_deg> <max_deg> : 设置软件行程限位 */
static int cmd_set_limit(int argc, char **argv)
{
    if (argc != 4) {
        printf("用法: sl <ch> <min_deg> <max_deg>   例: sl 1 30 150\n");
        return 1;
    }
    int ch = atoi(argv[1]);
    if (ch < 0 || ch >= SERVO_CHANNEL_COUNT) {
        printf("通道号非法 (0~%d)\n", SERVO_CHANNEL_COUNT - 1);
        return 1;
    }
    servo_cal_t cal;
    servo_get_cal((uint8_t)ch, &cal);
    cal.limit_min_deg = (uint16_t)atoi(argv[2]);
    cal.limit_max_deg = (uint16_t)atoi(argv[3]);
    if (servo_set_cal((uint8_t)ch, &cal) != ESP_OK) {
        printf("设置失败\n");
        return 1;
    }
    float lo = 0.0f, hi = 0.0f;
    servo_get_limit((uint8_t)ch, &lo, &hi);
    printf("ch%d 行程限位: %.0f ~ %.0f°\n", ch, lo, hi);
    return 0;
}

/** gt <ch> <phys_deg> : 设置某通道姿态闭环的目标物理角 */
static int cmd_gimb_target(int argc, char **argv)
{
    if (argc != 3) {
        printf("用法: gt <ch> <phys_deg>   例: gt 1 0  (0 = 保持水平)\n");
        return 1;
    }
    int ch = atoi(argv[1]);
    if (ch < 0 || ch >= SERVO_CHANNEL_COUNT) {
        printf("通道号非法 (0~%d)\n", SERVO_CHANNEL_COUNT - 1);
        return 1;
    }
    float deg = strtof(argv[2], NULL);
    if (gimbal_set_target_phys((uint8_t)ch, deg) != ESP_OK) {
        printf("设置失败 (云台未初始化?)\n");
        return 1;
    }
    gimbal_stab_state_t st;
    gimbal_get_stab_state((uint8_t)ch, &st);
    printf("ch%d 闭环目标: 物理 %.1f° (实测 %.1f°, 误差 %+.1f°)\n",
           ch, st.target_phys, st.meas_phys, st.err);
    return 0;
}

/** ge <ch> <0|1> : 开关某通道姿态闭环 */
static int cmd_gimb_enable(int argc, char **argv)
{
    if (argc != 3) {
        printf("用法: ge <ch> <0|1>   例: ge 1 1 (开) / ge 1 0 (关)\n");
        return 1;
    }
    int ch = atoi(argv[1]);
    if (ch < 0 || ch >= SERVO_CHANNEL_COUNT) {
        printf("通道号非法 (0~%d)\n", SERVO_CHANNEL_COUNT - 1);
        return 1;
    }
    bool en = (atoi(argv[2]) != 0);
    if (gimbal_enable_stabilize((uint8_t)ch, en) != ESP_OK) {
        printf("操作失败 (云台未初始化?)\n");
        return 1;
    }
    printf("ch%d 姿态闭环: %s\n", ch, en ? "开" : "关");
    return 0;
}

/** gp <ch> <kp> <ki> <kd> : 设置闭环 PID */
static int cmd_gimb_pid(int argc, char **argv)
{
    if (argc != 5) {
        printf("用法: gp <ch> <kp> <ki> <kd>   例: gp 1 0.8 0.5 0\n");
        return 1;
    }
    int ch = atoi(argv[1]);
    if (ch < 0 || ch >= SERVO_CHANNEL_COUNT) {
        printf("通道号非法 (0~%d)\n", SERVO_CHANNEL_COUNT - 1);
        return 1;
    }
    gimbal_pid_t pid = {
        .kp = strtof(argv[2], NULL),
        .ki = strtof(argv[3], NULL),
        .kd = strtof(argv[4], NULL),
    };
    if (gimbal_set_pid((uint8_t)ch, &pid) != ESP_OK) {
        printf("设置失败\n");
        return 1;
    }
    printf("ch%d PID: kp=%.2f ki=%.2f kd=%.2f\n", ch, pid.kp, pid.ki, pid.kd);
    return 0;
}

/** gs : 查看云台闭环状态 */
static int cmd_gimb_status(int argc, char **argv)
{
    printf("ch  stab   target   meas      err   cmd_out   fb    kp    ki    kd\n");
    for (uint8_t ch = 0; ch < CAL_CH_MAX; ch++) {
        gimbal_stab_state_t st;
        gimbal_pid_t pid;
        float fb = 0.0f;
        gimbal_get_stab_state(ch, &st);
        gimbal_get_pid(ch, &pid);
        gimbal_get_fb_sign(ch, &fb);
        printf("%2u  %4s  %7.1f  %6.1f  %+6.1f  %7.1f  %+3.0f  %5.2f  %5.2f  %5.2f\n",
               (unsigned)ch,
               !st.enabled ? "off" : (st.nofb ? "N/FB" : "ON"),
               st.target_phys, st.meas_phys, st.err, st.cmd_deg, fb,
               pid.kp, pid.ki, pid.kd);
    }
    printf("(N/FB = 无姿态反馈: 已停 PID 并回该通道标定中位)\n");
    return 0;
}

/** gps [0|1] : 惯导模块的 GPS 快照; 带 0|1 开关 1Hz 日志
 *  ⚠️ 数据来自惯导模块 (GPS+IMU 一体, 维特 WIT 0x55 协议) 的 0x57/0x58/0x5A 帧,
 *     **不再是 NMEA**; 所以没有"原始语句回显 / 改波特率 / 发配置语句"这些子命令。 */
static int cmd_gps(int argc, char **argv)
{
    /* ---- gps 0|1: 1Hz 日志开关 ---- */
    if (argc == 2 && (argv[1][0] == '0' || argv[1][0] == '1')) {
        s_gps_log_enabled = (atoi(argv[1]) != 0);
        printf("GPS 日志: %s\n", s_gps_log_enabled ? "开 (1Hz)" : "关");
        return 0;
    }
    if (argc >= 2) {
        printf("用法: gps [0|1]    无参打快照, 0|1 开关 1Hz 日志 (模块状态用 nav)\n");
        return 1;
    }

    nav_stats_t st;
    nav_get_stats(&st);
    if (!st.ready) {
        printf("惯导模块无数据: 查 UART2 接线 (ESP32 RX=IO1 <- 模块 TX, TX=IO2 -> 模块 RX)、共地、模块 3.3V\n");
        printf("   累计字节=%lu 帧OK=%lu 帧错=%lu    (输 nav 看模块配置/波特率)\n",
               (unsigned long)st.rx_bytes, (unsigned long)st.frames_ok,
               (unsigned long)st.frames_bad);
        return 1;
    }

    nav_gps_t d;
    if (nav_read_gps(&d) != ESP_OK) {
        printf("GPS: 模块在线但还没有 GPS 帧 —— 查模块 RSW 是否开了 GPS(0x57)/VELOCITY(0x58)/GSA(0x5A)\n");
        return 1;
    }
    uint64_t now = (uint64_t)esp_timer_get_time();

    printf("GPS: 定位=%s  卫星=%u  PDOP=%.1f HDOP=%.1f VDOP=%.1f\n",
           d.valid ? "有效" : "未定位", (unsigned)d.satellites, d.pdop, d.hdop, d.vdop);
    if (d.time_valid) {
        printf("     模块时间 20%02u-%02u-%02u %02u:%02u:%02u  (TIMEZONE 寄存器默认 UTC+8)\n",
               (unsigned)d.year, (unsigned)d.month, (unsigned)d.day,
               (unsigned)d.hour, (unsigned)d.minute, (unsigned)d.second);
    } else {
        printf("     模块时间 —  (模块 RSW 未开 TIME(0x50) 帧)\n");
    }

    if (d.valid) {
        printf("     纬度=%.6f° 经度=%.6f°  速度=%.1fkm/h  航向=%.1f°  海拔=%.1fm\n",
               d.latitude, d.longitude, d.speed_kmh, d.course_deg, d.altitude_m);
    } else {
        /* ⚠️ 未定位时坐标已被清零 (驱动里不再保留旧值), 这里明确提示 */
        char ago[48];
        if (d.last_fix_us == 0) {
            snprintf(ago, sizeof(ago), "从未定位成功");
        } else {
            snprintf(ago, sizeof(ago), "上次有效定位 %.1fs 前",
                     (double)(now - d.last_fix_us) / 1e6);
        }
        printf("     纬度=— 经度=— 速度=— 航向=—  海拔=%.1fm   (%s)\n", d.altitude_m, ago);
        printf("     => 未定位: 换开阔处并让天线朝天; 卫星数=%u (为 0 说明还没搜到星)\n",
               (unsigned)d.satellites);
    }
    return 0;
}

/** hull [0|1] : 惯导模块的姿态快照 (模块内部解算, 含磁力计补偿);
 *  不带参数读一帧, 带 0|1 开关 10Hz 日志。数据源: components/nav (WIT 0x53/0x51/0x52 帧) */
static int cmd_hull(int argc, char **argv)
{
    if (argc == 2) {
        s_hull_log_enabled = (atoi(argv[1]) != 0);
        printf("惯导姿态日志: %s\n", s_hull_log_enabled ? "开 (10Hz)" : "关");
        return 0;
    }
    if (argc >= 2) {
        printf("用法: hull [0|1]    无参读一帧, 0|1 开关 10Hz 日志\n");
        return 1;
    }

    nav_imu_t d;
    if (nav_read_imu(&d) != ESP_OK) {
        printf("惯导模块无姿态数据: 查 UART2 接线 (ESP32 RX=IO1 <- 模块 TX, TX=IO2 -> 模块 RX)、共地、模块 3.3V\n");
        printf("   接好后不必重启 —— 模块上线后会自动出数据; 输 nav 看帧计数\n");
        return 1;
    }
    printf("rpy=(%+7.2f %+7.2f %+7.2f) deg  a=(%+6.3f %+6.3f %+6.3f) g  "
           "g=(%+7.1f %+7.1f %+7.1f) dps  T=%.1f C\n",
           d.roll, d.pitch, d.yaw, d.ax, d.ay, d.az, d.gx, d.gy, d.gz, d.temperature);
    return 0;
}

/** nav : 惯导模块状态 (在线/配置/波特率/帧统计/两套快照) —— 排查"没数据"第一站 */
static int cmd_nav(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    nav_config_t cfg = nav_get_default_config();
    nav_stats_t  st;
    nav_get_stats(&st);

    printf("惯导模块 : %s   UART%d: ESP32 RX=IO%d <- 模块 TX, TX=IO%d -> 模块 RX\n",
           st.ready ? "在线" : "无数据", cfg.uart_num, cfg.rx_gpio, cfg.tx_gpio);
    printf("波特率   : %lu  (初始 9600, 初始化时自动扫描 9600~230400)\n",
           (unsigned long)st.baud);
    printf("输出配置 : RSW=0x%04X (期望 0x%04X)  RRATE=0x%02X (期望 0x%02X)  %s\n",
           (unsigned)st.rsw, (unsigned)cfg.rsw, (unsigned)st.rrate, (unsigned)cfg.rrate,
           st.cfg_ok ? "已生效" : "未确认 (稍后自动重试)");
    printf("累计     : 字节=%lu 帧OK=%lu 帧错=%lu\n",
           (unsigned long)st.rx_bytes, (unsigned long)st.frames_ok, (unsigned long)st.frames_bad);
    printf("帧计数   : 时间=%lu 加计=%lu 陀螺=%lu 角度=%lu GPS=%lu 地速=%lu 精度=%lu\n",
           (unsigned long)st.cnt_time, (unsigned long)st.cnt_acc, (unsigned long)st.cnt_gyro,
           (unsigned long)st.cnt_angle, (unsigned long)st.cnt_gps, (unsigned long)st.cnt_vel,
           (unsigned long)st.cnt_dop);

    nav_imu_t im;
    if (nav_read_imu(&im) == ESP_OK) {
        printf("姿态     : rpy=(%+7.2f %+7.2f %+7.2f) a=(%+6.3f %+6.3f %+6.3f)g "
               "g=(%+7.1f %+7.1f %+7.1f)dps T=%.1fC\n",
               im.roll, im.pitch, im.yaw, im.ax, im.ay, im.az,
               im.gx, im.gy, im.gz, im.temperature);
    } else {
        printf("姿态     : 无数据\n");
    }

    nav_gps_t gp;
    bool gp_ok = (nav_read_gps(&gp) == ESP_OK);
    if (gp_ok) {
        printf("GPS      : %s 卫星=%u PDOP=%.1f HDOP=%.1f VDOP=%.1f\n",
               gp.valid ? "定位" : "未定位", (unsigned)gp.satellites, gp.pdop, gp.hdop, gp.vdop);
        if (gp.valid) {
            printf("           纬度=%.6f° 经度=%.6f° 速度=%.1fkm/h 航向=%.1f° 海拔=%.1fm\n",
                   gp.latitude, gp.longitude, gp.speed_kmh, gp.course_deg, gp.altitude_m);
        }
    } else {
        printf("GPS      : 无数据\n");
    }

    if (!st.ready) {
        printf("=> 一字节都没收到: 查模块 3.3V/GND 共地、TX-RX 是否交叉 (模块 TX -> IO1)、模块是否上电\n");
    } else if (!st.cfg_ok) {
        printf("=> 有数据但配置未确认: 模块 RSW 出厂默认 0x001E (不含 GPS 帧), 见 nav.h; 驱动每 5s 自动重试\n");
    } else if (gp_ok && !gp.valid) {
        printf("=> 配置正常但未定位: 天线朝天 + 开阔处; `gps 1` 可开 1Hz 日志观察\n");
    }
    return 0;
}

/** scan : 扫描 I2C1 (GPIO16/17), 列出所有应答地址并识别型号
 *  —— 这条总线上现在是 PCA9685(0x40) + 云台 MPU6050(0x68)。
 *     ⚠️ 惯导模块走 **UART2**, 不在 I2C 上 (它的状态用 `nav` 查)。 */
static int cmd_scan(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    printf("扫描 I2C1 (GPIO16/17)...\n");
    imu_scan_role(IMU_ROLE_GIMBAL);
    printf("(惯导模块 UART2: ESP32 RX=IO1 <- 模块 TX, TX=IO2 -> 模块 RX)\n");
    return 0;
}

/* L298N 最低有效占空比: 低于它电机启动不了 (带载尤其明显) ⇒ |speed| ∈ (0, 60) 一律按 60 下发。
 * ⚠️ 0 仍然表示"停" —— 不能把 0 也提到 60, 否则永远停不下来。 */
#define MOTOR_MIN_PCT       60.0f

/**
 * 设置 L298N 直流电机速度 (-100~100: 正=正转, 负=反转, 0=停; 绝对值 = 占空比 %)
 * ENA(IO38) 出 5kHz PWM 调速, IN1(IO48)/IN2(IO47) 定方向。
 * 供 `m` 命令与"裸数字"快捷输入共用。
 * @param quiet  true = 不打印 (TCP 20Hz 路径; 打印会刷屏)
 */
static int motor_apply_speed(float v, bool quiet)
{
    if (v >  100.0f) v =  100.0f;
    if (v < -100.0f) v = -100.0f;

    /* 下限: 1~59 提到 60, -1~-59 压到 -60 (0 不动 = 停) */
    bool raised = false;
    if (v > 0.0f && v < MOTOR_MIN_PCT) {
        v =  MOTOR_MIN_PCT;
        raised = true;
    } else if (v < 0.0f && v > -MOTOR_MIN_PCT) {
        v = -MOTOR_MIN_PCT;
        raised = true;
    }

    motor_dir_t dir = MOTOR_DIR_STOP;
    if (v > 0.0f) {
        dir = MOTOR_DIR_FORWARD;
    } else if (v < 0.0f) {
        dir = MOTOR_DIR_REVERSE;
    }

    esp_err_t r = motor_set_dc_speed(MOTOR_MAIN_DC, fabsf(v), dir);
    if (r != ESP_OK) {
        if (!quiet) printf("L298N 设置失败: %s  (MOTOR_ENABLE=0 时不可用)\n", esp_err_to_name(r));
        return 1;
    }
    if (!quiet) {
        printf("L298N speed = %+.0f%%  (%s)",
               (double)v,
               (dir == MOTOR_DIR_FORWARD) ? "正转" :
               ((dir == MOTOR_DIR_REVERSE) ? "反转" : "停"));
        if (raised) {
            printf("   <- 低于下限 %.0f%%, 已按 %.0f%% 下发", MOTOR_MIN_PCT, MOTOR_MIN_PCT);
        }
        printf("\n");
    }
    return 0;
}

/**
 * m <speed> : L298N 直流电机调速 (控制台快捷入口, 等价于直接输裸数字)
 *   speed = -100 ~ +100: 正=正转, 负=反转, 0=停 (绝对值 = 占空比 %)
 *   ⚠️ |speed| < 60 时按 60 下发 (低于 60% 电机转不动); 0 = 停。
 */
static int cmd_motor(int argc, char **argv)
{
    if (argc < 2) {
        printf("用法: m <speed>   speed = -100~100 (正=正转, 负=反转, 0=停; |speed|<60 按 60 下发)\n");
        return 1;
    }
    return motor_apply_speed((float)atof(argv[1]), /*quiet=*/false);
}

/* ===== 通道映射: 水上 4 路电调 = 2 路推进 + 2 路反推 =====
 *   推进: ESC1/ESC2  -> IO41(左) / IO42(右)
 *   反推: REV1/REV2  -> IO39(左) / IO40(右)
 *
 * **负油门自动切换到反推通道**: l 30 = 左推进 30% (IO41), l -30 = 左反推 30% (IO39)。
 * 单向电调 (SkyWalker V2) 本身不能反向, 所以"反转"靠**独立的反推电调**实现。
 * ⚠️ 若实测某一路装反了, 把下面成对的 ID/GPIO 对调即可 (纯软件映射, 不用改接线)。 */
#define ESC_LEFT_ID     MOTOR_ESC_1
#define ESC_RIGHT_ID    MOTOR_ESC_2
#define ESC_LEFT_GPIO   41
#define ESC_RIGHT_GPIO  42
#define REV_LEFT_ID     MOTOR_THRUST_REV_1
#define REV_RIGHT_ID    MOTOR_THRUST_REV_2
#define REV_LEFT_GPIO   39
#define REV_RIGHT_GPIO  40

/**
 * 输出一路(左或右)的带符号油门 (SkyWalker V2 反推刹车接线):
 *   v > 0 → 正转: 油门线给速度 v%, 反推黄线回 0 (正向半区)
 *   v < 0 → 反转: 反推黄线打到 100% (反转半区), 油门线给速度 |v|%
 *   v = 0 → 两线都回 0
 *
 * ⚠️ 黄线是**方向通道**, 不是油门通道 (好盈 SkyWalker V2 说明书 §06 反推刹车):
 *    - 通道行程 0-50% (1100~1520µs) = 默认正向, 50%-100% (1520~1940µs) = 反转;
 *    - 反转速度由**油门线**决定 (触发反转时电调先刹停, 再反转加速到油门量);
 *    - 上电时黄线必须落在正向半区 (boot 时全通道 1100µs, 已满足)。
 *    旧实现把黄线当油门映射 (v<0 时发 |v|% ≈ 1380µs, 落在正向半区, 且油门线归零)
 *    ⇒ 电机永远不反转。现在黄线只发 0/100% 两个方向位, 速度全走油门线。
 * ⚠️ 切换方向时**先写方向线、再给油门**, 避免油门先于方向导致电机瞬间正向冲一下。
 * ⚠️ 前提: 电调参数"刹车类型"必须设为**反推刹车**(出厂默认"无刹车", 黄线无效)。
 * ⚠️ **REV_ESC_ENABLE=0 时整段反推被关掉** (见该宏): 负油门按"停"处理, 方向线不动。
 * @param quiet  true = 不打印 (TCP 20Hz 路径; 打印会刷屏)
 */
static int esc_apply_side(bool is_left, float v, bool quiet)
{
    const char *side     = is_left ? "左" : "右";
    motor_id_t  fwd_id   = is_left ? ESC_LEFT_ID   : ESC_RIGHT_ID;
    motor_id_t  rev_id   = is_left ? REV_LEFT_ID   : REV_RIGHT_ID;
    int         fwd_gpio = is_left ? ESC_LEFT_GPIO  : ESC_RIGHT_GPIO;
    int         rev_gpio = is_left ? REV_LEFT_GPIO  : REV_RIGHT_GPIO;

    if (v >  100.0f) v =  100.0f;
    if (v < -100.0f) v = -100.0f;

    /* 反推暂时关闭 (REV_ESC_ENABLE=0): 负油门在这里就改成 0 ⇒ 下面的 v<0 分支实际到不了,
     * 方向线只会在 1100µs (正向半区) 待着。
     * (打开反推时 rev_off 恒为 false, 下面那条提示不会出现, 行为与旧版完全一致) */
    bool rev_off = false;
#if !REV_ESC_ENABLE
    if (v < 0.0f) { v = 0.0f; rev_off = true; }
#endif

    esp_err_t r;
    if (v > 0.0f) {
        r = motor_set_esc_throttle(rev_id, 0.0f);          /* 方向线回正向半区 */
        if (r == ESP_OK) r = motor_set_esc_throttle(fwd_id, v);
        if (r != ESP_OK) {
            if (!quiet) printf("%s电调未就绪 (油门 IO%d / 方向 IO%d): %s\n",
                               side, fwd_gpio, rev_gpio, esp_err_to_name(r));
            return 1;
        }
        if (!quiet) printf("%s: 推进 %+.0f%%   (油门 IO%d, 方向 IO%d 正向)\n",
                           side, (double)v, fwd_gpio, rev_gpio);
    } else if (v < 0.0f) {
        r = motor_set_esc_throttle(rev_id, 100.0f);        /* 方向线打进反转半区 */
        if (r == ESP_OK) r = motor_set_esc_throttle(fwd_id, -v);  /* 油门线给速度 */
        if (r != ESP_OK) {
            if (!quiet) printf("%s电调未就绪 (油门 IO%d / 方向 IO%d): %s\n",
                               side, fwd_gpio, rev_gpio, esp_err_to_name(r));
            return 1;
        }
        if (!quiet) printf("%s: 反推 %+.0f%%   (油门 IO%d 给速度, 方向 IO%d 反转)\n",
                           side, (double)(-v), fwd_gpio, rev_gpio);
    } else {
        r = motor_set_esc_throttle(fwd_id, 0.0f);
        if (r == ESP_OK) r = motor_set_esc_throttle(rev_id, 0.0f);
        if (r != ESP_OK) {
            if (!quiet) printf("%s电调未就绪 (油门 IO%d / 方向 IO%d): %s\n",
                               side, fwd_gpio, rev_gpio, esp_err_to_name(r));
            return 1;
        }
        if (!quiet) {
            if (rev_off) {
                printf("%s: 反推已关闭 (REV_ESC_ENABLE=0) ⇒ 按停处理 (油门 IO%d 已回零)\n",
                       side, fwd_gpio);
            } else {
                printf("%s: 停          (油门 IO%d / 方向 IO%d 都已回零)\n",
                       side, fwd_gpio, rev_gpio);
            }
        }
    }
    return 0;
}

/** l <v> : 左路带符号油门, v = -100~100 (正=左推进 IO41, 负=左反推 IO39, 0=停) */
static int cmd_esc_l(int argc, char **argv)
{
    if (argc < 2) {
        printf("用法: l <-100~100>   正=左推进 (IO%d), 负=左反推 (IO%d), 0=停\n",
               ESC_LEFT_GPIO, REV_LEFT_GPIO);
        return 1;
    }
    return esc_apply_side(true, (float)atof(argv[1]), /*quiet=*/false);
}

/** r <v> : 右路带符号油门, v = -100~100 (正=右推进 IO42, 负=右反推 IO40, 0=停) */
static int cmd_esc_r(int argc, char **argv)
{
    if (argc < 2) {
        printf("用法: r <-100~100>   正=右推进 (IO%d), 负=右反推 (IO%d), 0=停\n",
               ESC_RIGHT_GPIO, REV_RIGHT_GPIO);
        return 1;
    }
    return esc_apply_side(false, (float)atof(argv[1]), /*quiet=*/false);
}

/** 裸数字: 左右同时给同一个带符号油门 (两路一起试) */
static void esc_apply_both(int v)
{
    esc_apply_side(true,  (float)v, /*quiet=*/false);
    esc_apply_side(false, (float)v, /*quiet=*/false);
}

/**
 * 把通道名解析成 ID / 中文侧名 / 引脚。供 cal / pw 这类**逐通道**调试命令使用
 * (油门不用它 —— 油门靠 l/r 的负号自动切换, 见 esc_apply_side)。
 *   l  / left    左推进 (IO41)
 *   r  / right   右推进 (IO42)
 *   bl / bleft   左反推 (IO39)
 *   br / bright  右反推 (IO40)
 */
static bool esc_resolve_side(const char *arg, motor_id_t *id, const char **side, int *gpio)
{
    if (strcmp(arg, "l")  == 0 || strcmp(arg, "left")   == 0) {
        *id = ESC_LEFT_ID;  *side = "左推进"; *gpio = ESC_LEFT_GPIO;  return true;
    }
    if (strcmp(arg, "r")  == 0 || strcmp(arg, "right")  == 0) {
        *id = ESC_RIGHT_ID; *side = "右推进"; *gpio = ESC_RIGHT_GPIO; return true;
    }
    if (strcmp(arg, "bl") == 0 || strcmp(arg, "bleft")  == 0) {
        *id = REV_LEFT_ID;  *side = "左反推"; *gpio = REV_LEFT_GPIO;  return true;
    }
    if (strcmp(arg, "br") == 0 || strcmp(arg, "bright") == 0) {
        *id = REV_RIGHT_ID; *side = "右反推"; *gpio = REV_RIGHT_GPIO; return true;
    }
    return false;
}

/* ===== 电调脉宽人工测定 (pw 定点) =====
 * 用途: 手动逐点逼近电调**真实的脉宽阈值** —— 例如找"从哪一档开始转"。
 * ⚠️ 高端点 (满油门) 用这个测不出: 90% 与 100% 的转速/声音差别极小,
 *    要确认"100 是真 100"请用 `cal` 行程标定, 或上电流表/转速表。
 * ⚠️ 输出较大脉宽时电机会真的转起来, 务必先拆桨。 */

/** pw <l|r|bl|br> <us> : 直接输出指定脉宽 (500~2500 µs), 人工测定端点用 */
static int cmd_pw(int argc, char **argv)
{
    if (argc < 3) {
        printf("用法: pw <l|r|bl|br> <us>    直接输出脉宽 (500~2500 us)\n");
        printf("      l=左推进(IO%d) r=右推进(IO%d) bl=左反推(IO%d) br=右反推(IO%d)\n",
               ESC_LEFT_GPIO, ESC_RIGHT_GPIO, REV_LEFT_GPIO, REV_RIGHT_GPIO);
        printf("      油门区间 1141~1940 us; 1135 及以下是停; 1136~1140 为迟滞带\n");
        printf("      ⚠️ 上限还受帧周期限制 (周期 = 1/帧率), 超过会被拒绝\n");
        return 1;
    }

    motor_id_t id; const char *side; int gpio;
    if (!esc_resolve_side(argv[1], &id, &side, &gpio)) {
        printf("通道只能是 l / r / bl / br\n");
        return 1;
    }

    int us = atoi(argv[2]);
    if (us < 500 || us > 2500) {
        printf("脉宽请给 500~2500 us\n");
        return 1;
    }

    esp_err_t r = motor_set_esc_pulse_us(id, (uint32_t)us);
    if (r != ESP_OK) {
        printf("%s (IO%d) 设置失败: %s\n", side, gpio, esp_err_to_name(r));
        if (r == ESP_ERR_INVALID_ARG) {
            printf("  (脉宽超出可表示范围: 500 ~ min(2500, 帧周期); 当前帧率见 motor.c 的 MOTOR_ESC_FREQ_HZ)\n");
        }
        return 1;
    }

    /* 按实测阈值给出解读 (迟滞特性: 1136~1140 只在"已在转"时维持, 冷态启动不了) */
    printf("%s (IO%d) 脉宽 = %d us", side, gpio, us);
    if (us <= 1100) {
        printf("   (<= 停机点 1100us: 停, 且是上电解锁位)\n");
    } else if (us <= 1135) {
        printf("   (<= 停止点 1135us: 停)\n");
    } else if (us < 1141) {
        printf("   (1136~1140 迟滞带: 已在转可维持, 冷态启动不了)\n");
    } else if (us >= 1940) {
        printf("   (>= 上限 1940us: 全速)\n");
    } else {
        printf("   (按 1141~1940 映射约 %.0f%% 油门)\n",
               (double)(us - 1141) * 100.0 / (1940.0 - 1141.0));
    }
    return 0;
}

/* ==================== 电调油门行程标定 (好盈 SkyWalker V2 官方流程) ====================
 *
 * 官方《油门行程校准操作方法》:
 *   1) 遥控器油门打到**最高点**
 *   2) 电调接电池 -> 马达"123"提示音 (上电正常)
 *   3) N 声短鸣 = 锂电节数
 *   4) "哗-哗-" 双短鸣 = **最高点校准成功**
 *   5) **5 秒内**把油门推到最低, 等待 1 秒 = 最低点校准成功
 *   6) 一声长鸣 "哗——" = 系统就绪
 *
 * ⚠️ 第 5 步是硬性的 5 秒窗口, 而 ESP32 **听不到电调的鸣叫**, 无法用固定延时自动卡点
 *    (延时长于 5s 会超时失败, 短于电调上电自检时间又会提前降油门)。
 *    所以拆成**手动两步**: `cal l` 先输出最高油门并保持, 你听到双短鸣后立刻敲 `cal2`。
 *
 * 标定完成后, 程序的 100% 就是这台电调认定的最高点 (定义上相等)。 */
static motor_id_t  s_cal_id   = MOTOR_MAX;   /* 正在标定的通道 (MOTOR_MAX = 无) */
static int         s_cal_gpio = -1;
static const char *s_cal_side = "";

/** cal <l|r|bl|br> : 行程标定第 1 步 —— 输出并保持最高油门, 等电调确认最高点 */
static int cmd_esc_cal(int argc, char **argv)
{
    if (argc < 2) {
        printf("用法: cal <l|r|bl|br>   电调油门行程标定第 1 步 (先断电, 再上电, 听双短鸣)\n");
        printf("      l=左推进(IO%d) r=右推进(IO%d) bl=左反推(IO%d) br=右反推(IO%d)\n",
               ESC_LEFT_GPIO, ESC_RIGHT_GPIO, REV_LEFT_GPIO, REV_RIGHT_GPIO);
        return 1;
    }
    motor_id_t id; const char *side; int gpio;
    if (!esc_resolve_side(argv[1], &id, &side, &gpio)) {
        printf("通道只能是 l / r / bl / br\n");
        return 1;
    }

    esp_err_t r = motor_set_esc_throttle(id, 100.0f);
    if (r != ESP_OK) {
        printf("%s (IO%d) 输出失败: %s  (通道未配置时不可用)\n",
               side, gpio, esp_err_to_name(r));
        return 1;
    }

    s_cal_id   = id;
    s_cal_gpio = gpio;
    s_cal_side = side;

    printf("\n=== %s (IO%d) 行程标定 · 第 1/2 步 ===\n", side, gpio);
    printf("已输出**最高油门 100%% (1940us) 并保持**。现在请:\n");
    printf("  [1] 先给电调断电 (拔电池)\n");
    printf("  [2] 再给电调上电 —— 会听到 \"123\" 上电音 -> N 声短鸣(电池节数)\n");
    printf("  [3] 听到 \"哗-哗-\" **双短鸣** = 最高点已确认\n");
    printf("  [4] 听到双短鸣后 **立刻** 输入:  cal2\n");
    printf("⚠️ 双短鸣后 5 秒内必须降到最低油门, 所以听到就马上敲 cal2 (别先按回车空行)\n");
    printf("⚠️ 标定期间请不要输 l / r / pw / 裸数字, 会打断\n\n");
    return 0;
}

/** cal2 : 行程标定第 2 步 —— 降到最低油门, 完成标定 */
static int cmd_esc_cal2(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (s_cal_id == MOTOR_MAX) {
        printf("还没开始标定。请先输入: cal <l|r|bl|br>\n");
        return 1;
    }

    motor_id_t  id   = s_cal_id;
    const char *side = s_cal_side;
    int         gpio = s_cal_gpio;
    s_cal_id = MOTOR_MAX;      /* 无论成败都结束本轮, 避免误用 */

    esp_err_t r = motor_set_esc_throttle(id, 0.0f);
    if (r != ESP_OK) {
        printf("%s (IO%d) 输出失败: %s\n", side, gpio, esp_err_to_name(r));
        return 1;
    }

    printf("\n=== %s (IO%d) 行程标定 · 第 2/2 步 ===\n", side, gpio);
    printf("已降到**最低油门 0%% (1100us)** 并保持\n");
    printf("  -> 听到一声长鸣 \"哗——\" = 最低点校准成功, 标定完成\n");
    printf("  -> 之后上电应听到: \"123\" + N 声短鸣 + 一声长鸣 (正常就绪)\n");
    printf("=== 标定结束, 该通道当前为 0 (停) ===\n\n");
    return 0;
}

/** imu : 读一帧云台 MPU6050 (原始 6 轴; 姿态由上层 Madgwick 解算)
 *  ⚠️ 船体姿态/惯导数据不在本命令里 —— 已由惯导模块提供, 用 `hull` 或 `nav`。 */
static int cmd_imu(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    if (!imu_role_ready(IMU_ROLE_GIMBAL)) {
        printf("gimbal  未就绪 (云台 MPU6050 未接, 或 MPU6050_ENABLE=0)\n");
        return 1;
    }
    imu_data_t d;
    if (imu_read_role(IMU_ROLE_GIMBAL, &d) != ESP_OK) {
        printf("gimbal  读取失败\n");
        return 1;
    }
    printf("gimbal  a=(%+7.3f %+7.3f %+7.3f) g  g=(%+8.1f %+8.1f %+8.1f) dps  T=%.1f C\n",
           d.ax, d.ay, d.az, d.gx, d.gy, d.gz, d.temperature);
    return 0;
}

/** p <ch> <us> : 直接输出指定脉宽 */
static int cmd_pulse(int argc, char **argv)
{
    if (argc != 3) {
        printf("用法: p <ch> <us>   例: p 1 1520\n");
        return 1;
    }
    int ch = atoi(argv[1]);
    int us = atoi(argv[2]);
    if (ch < 0 || ch >= SERVO_CHANNEL_COUNT) {
        printf("通道越界 (0~%d)\n", SERVO_CHANNEL_COUNT - 1);
        return 1;
    }
    if (us < 100 || us > 3000) {
        printf("脉宽请给 100~3000 us\n");
        return 1;
    }
    esp_err_t ret = servo_set_pulse_us((uint8_t)ch, (uint16_t)us);
    if (ret != ESP_OK) {
        printf("失败: %s\n", esp_err_to_name(ret));
        return 1;
    }
    printf("ch%d -> %d us\n", ch, us);
    return 0;
}

/** g <ch> [deg] : **按角度**控制云台 (v6.1 新增)
 *  与 `p`(脉宽) 的分工:
 *    · `g` 走 servo_set_angle() —— 按**标称角**下发, **受"标定 + 软件行程限位"约束**
 *      (ch1 出厂限位 30~150°, 越界会被钳住);
 *    · `p` 走 servo_set_pulse_us() —— 直接定脉宽, **绕过限位**, 测机械行程/端点时用它。
 *  不带 deg 参数 = 只查询当前角度/脉宽/可用窗口, 不动作。 */
static int cmd_goto(int argc, char **argv)
{
    if (argc < 2 || argc > 3) {
        printf("用法: g <ch> [deg]   例: g 1 120  (不带 deg = 查当前角度)\n");
        return 1;
    }
    int ch = atoi(argv[1]);
    if (ch < 0 || ch >= SERVO_CHANNEL_COUNT) {
        printf("通道越界 (0~%d)\n", SERVO_CHANNEL_COUNT - 1);
        return 1;
    }
    if (!servo_is_ready()) {
        printf("PCA9685 未就绪 (SERVO_ENABLE=0 或硬件异常)\n");
        return 1;
    }

    float lo = 0.0f, hi = 0.0f;
    servo_get_limit((uint8_t)ch, &lo, &hi);

    if (argc == 2) {                     /* 只查询, 不动作 */
        float    cur = 0.0f;
        uint16_t us  = 0;
        servo_get_angle((uint8_t)ch, &cur);
        servo_get_pulse_us((uint8_t)ch, &us);
        printf("ch%d 当前 %.1f° (可用 %.0f~%.0f°, %u us)\n", ch, cur, lo, hi, us);
        return 0;
    }

    float deg = (float)atof(argv[2]);
    esp_err_t ret = servo_set_angle((uint8_t)ch, deg);
    if (ret != ESP_OK) {
        printf("失败: %s\n", esp_err_to_name(ret));
        return 1;
    }

    float    act = 0.0f;
    uint16_t us  = 0;
    servo_get_angle((uint8_t)ch, &act);
    servo_get_pulse_us((uint8_t)ch, &us);
    if (fabsf(act - deg) > 0.05f) {
        printf("ch%d -> 请求 %.1f°, 实际 %.1f° (被软件限位 %.0f~%.0f° 钳住, %u us)\n",
               ch, deg, act, lo, hi, us);
    } else {
        printf("ch%d -> %.1f° (%u us)\n", ch, act, us);
    }
    return 0;
}

/** n <ch> <dus> : 在当前脉宽上增减 */
static int cmd_nudge(int argc, char **argv)
{
    if (argc != 3) {
        printf("用法: n <ch> <dus>   例: n 1 -20\n");
        return 1;
    }
    int ch = atoi(argv[1]);
    int d  = atoi(argv[2]);
    if (ch < 0 || ch >= SERVO_CHANNEL_COUNT) {
        printf("通道越界 (0~%d)\n", SERVO_CHANNEL_COUNT - 1);
        return 1;
    }
    uint16_t cur = 0;
    servo_get_pulse_us((uint8_t)ch, &cur);
    int nv = (int)cur + d;
    if (nv < 100)  nv = 100;
    if (nv > 3000) nv = 3000;
    esp_err_t ret = servo_set_pulse_us((uint8_t)ch, (uint16_t)nv);
    if (ret != ESP_OK) {
        printf("失败: %s\n", esp_err_to_name(ret));
        return 1;
    }
    printf("ch%d: %u -> %d us\n", ch, cur, nv);
    return 0;
}

/** z [ch] : 回中 (不带参数则全部通道) */
static int cmd_center(int argc, char **argv)
{
    if (argc == 1) {
        servo_center_all();
        printf("全部通道已回中\n");
        return 0;
    }
    int ch = atoi(argv[1]);
    if (ch < 0 || ch >= SERVO_CHANNEL_COUNT) {
        printf("通道越界 (0~%d)\n", SERVO_CHANNEL_COUNT - 1);
        return 1;
    }
    servo_center((uint8_t)ch);
    uint16_t cur = 0;
    servo_get_pulse_us((uint8_t)ch, &cur);
    printf("ch%d 回中 -> %u us\n", ch, cur);
    return 0;
}

/** t <ch> <trim_us> : 设置中位微调并立即回中 */
static int cmd_trim(int argc, char **argv)
{
    if (argc != 3) {
        printf("用法: t <ch> <trim_us>   例: t 1 -30\n");
        return 1;
    }
    int ch = atoi(argv[1]);
    if (ch < 0 || ch >= SERVO_CHANNEL_COUNT) {
        printf("通道越界 (0~%d)\n", SERVO_CHANNEL_COUNT - 1);
        return 1;
    }
    servo_cal_t cal;
    servo_get_cal((uint8_t)ch, &cal);
    cal.trim_us = (int16_t)atoi(argv[2]);
    servo_set_cal((uint8_t)ch, &cal);
    servo_center((uint8_t)ch);
    uint16_t cur = 0;
    servo_get_pulse_us((uint8_t)ch, &cur);
    printf("ch%d trim=%d -> 中位 %u us\n", ch, cal.trim_us, cur);
    return 0;
}

/** sc <ch> <min> <max> <trim> [range_deg] [phys_deg] : 设置完整标定并回中 */
static int cmd_setcal(int argc, char **argv)
{
    if (argc < 5 || argc > 7) {
        printf("用法: sc <ch> <min_us> <max_us> <trim_us> [range_deg] [phys_deg]\n");
        printf("      range_deg 省略时保留原值 (标称角满量程: 180° 舵机填 180, 360° 填 360)\n");
        printf("      phys_deg  省略时保留原值 (实测物理行程: ch0=365, ch1=192)\n");
        printf("      例: sc 1 600 2700 0 180 192\n");
        return 1;
    }
    int ch = atoi(argv[1]);
    if (ch < 0 || ch >= SERVO_CHANNEL_COUNT) {
        printf("通道越界 (0~%d)\n", SERVO_CHANNEL_COUNT - 1);
        return 1;
    }
    servo_cal_t cal;
    servo_get_cal((uint8_t)ch, &cal);
    cal.min_us  = (uint16_t)atoi(argv[2]);
    cal.max_us  = (uint16_t)atoi(argv[3]);
    cal.trim_us = (int16_t)atoi(argv[4]);
    if (argc >= 6) {
        cal.range_deg = (uint16_t)atoi(argv[5]);
    }
    if (argc >= 7) {
        cal.phys_range_deg = (uint16_t)atoi(argv[6]);
    }
    servo_set_cal((uint8_t)ch, &cal);
    servo_center((uint8_t)ch);
    cal_print_status();
    printf("提示: 输入 sv 保存到 NVS\n");
    return 0;
}

/** ck <ch> : 把当前脉宽捕获为该通道的标定中位 */
static int cmd_capture(int argc, char **argv)
{
    if (argc != 2) {
        printf("用法: ck <ch>   (先用 n 微调到与测试仪一致, 再执行本指令)\n");
        return 1;
    }
    int ch = atoi(argv[1]);
    if (ch < 0 || ch >= SERVO_CHANNEL_COUNT) {
        printf("通道越界 (0~%d)\n", SERVO_CHANNEL_COUNT - 1);
        return 1;
    }
    uint16_t cur = 0;
    servo_get_pulse_us((uint8_t)ch, &cur);
    if (cur == 0) {
        printf("ch%d 当前无输出, 请先用 p/n 输出脉宽\n", ch);
        return 1;
    }
    servo_cal_t cal;
    servo_get_cal((uint8_t)ch, &cal);
    int mid = ((int)cal.min_us + (int)cal.max_us) / 2;
    cal.trim_us = (int16_t)((int)cur - mid);
    servo_set_cal((uint8_t)ch, &cal);
    printf("ch%d 已捕获中位: cur=%u us, trim=%d (中位=%d us)\n",
           ch, cur, cal.trim_us, mid + cal.trim_us);
    printf("提示: 输入 sv 保存到 NVS\n");
    return 0;
}

/** rs <ch> : 复位该通道标定 */
static int cmd_reset(int argc, char **argv)
{
    if (argc != 2) {
        printf("用法: rs <ch>\n");
        return 1;
    }
    int ch = atoi(argv[1]);
    if (ch < 0 || ch >= SERVO_CHANNEL_COUNT) {
        printf("通道越界 (0~%d)\n", SERVO_CHANNEL_COUNT - 1);
        return 1;
    }
    servo_reset_cal((uint8_t)ch);
    servo_center((uint8_t)ch);
    printf("ch%d 标定已复位为默认\n", ch);
    return 0;
}

/** st : 查看 ch0/ch1 标定与当前脉宽 */
static int cmd_status(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    cal_print_status();
    return 0;
}

/** sv : 保存标定到 NVS */
static int cmd_save(int argc, char **argv)
{
    esp_err_t ret = servo_save_cal();
    if (ret == ESP_OK) {
        printf("标定已保存到 NVS\n");
        return 0;
    }
    printf("保存失败: %s\n", esp_err_to_name(ret));
    return 1;
}

/** att <0|1> : 姿态日志开关 */
static int cmd_att(int argc, char **argv)
{
    if (argc != 2) {
        printf("用法: att <0|1>\n");
        return 1;
    }
    s_att_log_enabled = (atoi(argv[1]) != 0);
    printf("姿态日志: %s\n", s_att_log_enabled ? "开" : "关");
    return 0;
}

/** t0 <0|1> : ch0 档位扫描测试开关 */
static int cmd_test0(int argc, char **argv)
{
    if (argc != 2) {
        printf("用法: t0 <0|1>   (0=停止扫描并回中, 1=开始扫描)\n");
        return 1;
    }
    s_ch0_test_enabled = (atoi(argv[1]) != 0);
    if (!s_ch0_test_enabled) {
        servo_center(0);
        printf("ch0 扫描测试: 停止 (已回中)\n");
    } else {
        printf("ch0 扫描测试: 开始\n");
    }
    return 0;
}

/** t1 <0|1> : ch1 角度扫描测试开关 */
static int cmd_test1(int argc, char **argv)
{
    if (argc != 2) {
        printf("用法: t1 <0|1>   (0=停止扫描并回中, 1=开始扫描)\n");
        return 1;
    }
    s_ch1_test_enabled = (atoi(argv[1]) != 0);
    if (!s_ch1_test_enabled) {
        servo_set_angle(1, 90.0f);
        printf("ch1 扫描测试: 停止 (已回中 90°)\n");
    } else {
        printf("ch1 扫描测试: 开始\n");
    }
    return 0;
}

/** 纯整数判定 (允许前导 + / -), 供控制台"裸数字"快捷输入识别 */
static bool is_int_str(const char *s)
{
    if (s == NULL || *s == '\0') return false;
    if (*s == '+' || *s == '-') s++;
    if (*s == '\0') return false;
    for (; *s != '\0'; s++) {
        if (*s < '0' || *s > '9') return false;
    }
    return true;
}

/**
 * @brief 自定义串口 REPL 任务
 *
 * 与 esp_console 自带 REPL 唯一的区别: 纯整数输入走"快捷通道", 省去每次都要打命令名。
 * PCA9685 关停期间 (SERVO_ENABLE=0) 裸数字 = **左右电调同时给 0~100 油门**;
 * PCA9685 启用时裸数字 = **ch0 脉宽 100~3000us** (原行为)。
 * 其余输入照常交给 esp_console_run 处理.
 */
static void console_repl_task(void *pvParameters)
{
    linenoiseSetMaxLineLen(128);

    printf("\n");
#if SERVO_ENABLE
    printf("直接输入数字 (100~3000) = 把 ch0 定死在该脉宽 (us)\n");
#else
    printf("直接输入数字 (-100~100) = 左右同时 (正=推进, 负=反推); l <v> 只动左, r <v> 只动右\n");
#endif
    printf("输入 help 查看全部指令\n");

    while (1) {
        char *line = linenoise("gimbal> ");
        if (line == NULL) {
            continue;               /* 空行或读取被中断 */
        }
        if (line[0] == '\0') {
            linenoiseFree(line);
            continue;
        }
        linenoiseHistoryAdd(line);

        /* 纯整数 (可带 +/-) → 快捷通道 */
        if (is_int_str(line)) {
#if SERVO_ENABLE
            int us = atoi(line);
            if (us < 100 || us > 3000) {
                printf("脉宽请给 100~3000 us\n");
            } else {
                esp_err_t r = servo_set_pulse_us(0, (uint16_t)us);
                if (r == ESP_OK) {
                    printf("ch0 -> %d us\n", us);
                } else {
                    printf("设置失败: %s\n", esp_err_to_name(r));
                }
            }
#else
            /* PCA9685 关停 ⇒ ch0 脉宽没意义; 本轮测试对象是 4 路电调 ⇒ 裸数字 = 左右同时 (带符号) */
            int t = atoi(line);
            if (t < -100 || t > 100) {
                printf("电调油门请给 -100~100 (正=推进, 负=反推; 左右单独控制用 l <v> / r <v>)\n");
            } else {
                esc_apply_both(t);
            }
#endif
        } else {
            int ret = 0;
            esp_err_t err = esp_console_run(line, &ret);
            if (err == ESP_ERR_NOT_FOUND) {
                printf("Unrecognized command\n");
            } else if (err == ESP_ERR_INVALID_ARG) {
                /* 空命令, 忽略 */
            } else if (err == ESP_OK && ret != ESP_OK) {
                printf("Command returned non-zero error code: 0x%x\n", ret);
            } else if (err != ESP_OK) {
                printf("Internal error: %s\n", esp_err_to_name(err));
            }
        }
        linenoiseFree(line);
    }
}

/* ============================================================
 *  TCP 服务器 (v5.12) —— 上位机内网操控, 仅 8080
 * ============================================================ */

/** 云台手动指令 (上位机 cmd=0x12 SERVO) → 舵机。
 *
 * 掩码置位的通道按**标称角**下发 (`servo_set_angle()`), 受"标定 + 软件行程限位"约束
 * (ch1 出厂 30~150°, 越界自动钳住) —— 与控制台 `g` 命令走同一条路, 只是值来自网络。
 * ⚠️ **只在角度变化时才写 I2C**: 拖动条会连发同一个值, 重复写没有意义还占总线。
 * ⚠️ 失败只打印一次 (避免 20Hz 刷屏), 成功一次后重新武装。
 * ⚠️ 与云台闭环互斥: `ge <ch> 1` 开着时闭环每周期也会写同一通道, 二者会互相抢 ——
 *    要手动摆位请先确认该通道闭环是关的 (当前 `MPU6050_ENABLE=0`, 闭环也没有反馈源)。
 */
static void servo_apply_from_host(const ctrl_command_t *cmd)
{
#if SERVO_ENABLE
    static float s_last_deg[2]   = { 1e9f, 1e9f };   /* 上次下发的目标角 (哨兵值: 首次必下发) */
    static bool  s_err_logged[2] = { false, false };

    for (int ch = 0; ch < 2; ch++) {
        if (!(cmd->servo_mask & (1u << ch))) continue;       /* 掩码未置位: 该通道不动 */

        float deg = (float)cmd->servo_deg10[ch] / 10.0f;
        if (deg == s_last_deg[ch]) continue;                 /* 值没变: 不重复写总线 */
        s_last_deg[ch] = deg;

        esp_err_t r = servo_set_angle((uint8_t)ch, deg);
        if (r == ESP_OK) {
            s_err_logged[ch] = false;
        } else if (!s_err_logged[ch]) {
            s_err_logged[ch] = true;
            ESP_LOGW("TCP", "云台 ch%d 目标 %.1f° 下发失败: %s (SERVO_ENABLE=0 或 PCA9685 未就绪?)",
                     ch, (double)deg, esp_err_to_name(r));
        }
    }
#else
    (void)cmd;
#endif
}

/** 上位机控制帧 → 本地动作。**静默执行** —— TCP 20Hz, 打印会刷爆串口。
 *
 * ① cmd=0x10 MOTOR — 差速混合 (与参数总览 §2 一致):
 *   left  = clamp(speed + yaw, -100, +100) → 正=左推进 IO41 / 负=左反推 IO39
 *   right = clamp(speed - yaw, -100, +100) → 正=右推进 IO42 / 负=右反推 IO40
 *   bucket_speed (-100~100)                → L298N 滚筒收放电机 (IO38/48/47)
 * ② cmd=0x12 SERVO — 云台手动 (v9.2): 掩码置位的通道按标称角摆位, 见 servo_apply_from_host()。
 * ⚠️ 500ms 失联保护只把**电机**归零, 舵机保持最后位置 (PCA9685 自己维持 PWM)。 */
static void tcp_apply_local_command(const ctrl_command_t *cmd)
{
    /* 云台手动 (cmd=0x12): 与电机无关, 单独处理 */
    if (cmd->cmd == CTRL_CMD_SERVO) {
        servo_apply_from_host(cmd);
        return;
    }

    int left  = (int)cmd->speed + (int)cmd->yaw;
    int right = (int)cmd->speed - (int)cmd->yaw;
    if (left  >  100) left  =  100;
    if (left  < -100) left  = -100;
    if (right >  100) right =  100;
    if (right < -100) right = -100;

    esc_apply_side(true,  (float)left,  /*quiet=*/true);
    esc_apply_side(false, (float)right, /*quiet=*/true);
    motor_apply_speed((float)cmd->bucket_speed, /*quiet=*/true);
}

/**
 * 失联保护: 已连接且收到过控制帧, 但超过 TCP_LINK_TIMEOUT_MS 没有新帧
 * ⇒ 4 路电调 + L298N 全部归零, 并解除武装 (等下一帧重新计时)。
 *
 * 为什么需要: 没有它的话, 上位机崩溃 / 网线松掉会让船**保持最后一条指令**一直冲出去。
 * 上位机正常按 20Hz 下发, 500ms = 连续丢 10 帧, 足以区分"网络抖动"与"真失联"。
 */
static void tcp_check_link_timeout(void)
{
    if (!s_host_connected || s_last_ctrl_us == 0) return;

    int64_t age_ms = (esp_timer_get_time() - s_last_ctrl_us) / 1000;
    if (age_ms <= TCP_LINK_TIMEOUT_MS) return;

    esc_apply_side(true,  0.0f, /*quiet=*/true);
    esc_apply_side(false, 0.0f, /*quiet=*/true);
    motor_apply_speed(0.0f, /*quiet=*/true);
    s_last_ctrl_us = 0;   /* 只触发一次, 收到新帧才重新计时 */
    ESP_LOGW("TCP", "上位机失联 >%d ms ⇒ 已全部停机 (推进/反推/滚筒归零)", TCP_LINK_TIMEOUT_MS);
}

/**
 * 状态回传 ①: **透传**收到的 0xBB 0x66 帧给上位机 (type 原样保留)。
 *
 * `control` 组的 handle_mpu_frame() 每收到一条状态帧就调这里 —— 走的正是
 * "远端 (8081) → 主控 → 上位机" 这条路 (type=0x02 远端 MPU / type=0x03 v3.0 沉浮状态)。
 * ⚠️ 8081 远端转发当前未启用, 所以实际不会有帧进来; 实现留着即"到货就能用"。
 */
void tcp_server_forward_mpu_to_host(const ctrl_mpu_data_t *m)
{
    if (!s_host_connected) return;

    uint8_t f[CTRL_MPU_FRAME_SIZE];
    control_build_mpu_frame(f, m->type, m->ax, m->ay, m->az, m->gx, m->gy, m->gz);
    wiz_send(HOST_SOCK, f, sizeof(f));
}

/** 状态回传 ②: **主控自己**那颗云台 MPU6050 的原始 6 轴, 20Hz 发给上位机 (type=0x01) */
static void tcp_poll_send_local_mpu(void)
{
    if (!s_host_connected) {
        s_mpu_next_us = 0;          /* 断开 → 重连后立刻发第一条 */
        return;
    }
    if (!s_mpu_raw_valid) return;   /* MPU 未就绪/未解算: 不发 (上位机面板显示 --) */

    int64_t now = esp_timer_get_time();
    if (s_mpu_next_us == 0)  s_mpu_next_us = now;
    if (now < s_mpu_next_us) return;
    s_mpu_next_us = now + MPU_PUSH_PERIOD_US;

    uint8_t f[CTRL_MPU_FRAME_SIZE];
    control_build_mpu_frame(f, CTRL_MPU_TYPE_LOCAL,
                            s_mpu_raw[0], s_mpu_raw[1], s_mpu_raw[2],
                            s_mpu_raw[3], s_mpu_raw[4], s_mpu_raw[5]);
    int32_t n = wiz_send(HOST_SOCK, f, sizeof(f));
    if (n < 0) {
        ESP_LOGW("TCP", "主控 MPU 帧发送失败 (n=%d)", (int)n);
    }
}

/* ========== GPS 状态回传 (v9.1) ==========
 * 上位机「GPS」面板收两个 16B 状态帧, 各 **2Hz**:
 *   type=0x04 定位 (纬度/经度/海拔/卫星数/flags)
 *   type=0x05 运动+精度+时间 (航向/地速/P·H·V DOP/模块时间/flags)
 * 布局见 control.h 顶部注释。数据源是惯导模块 (components/nav) 快照 ——
 * `nav_read_gps()` 只是加锁拷结构体、不碰 UART, 所以在 TCP 任务里直接读没问题
 * (模块没 GPS 帧时**一帧都不发**, 上位机面板保持 `--`)。 */
#if NAV_ENABLE
#define GPS_PUSH_PERIOD_US    500000     /* 2Hz */
static int64_t s_gps_next_us = 0;

/** DOP (float, 典型 0.5~50) → 上报用的 uint8 (×0.1); 超出 25.5 截顶 */
static uint8_t dop_to_u8(float v)
{
    if (!(v > 0.0f)) return 0;          /* 含 NaN / 负数 */
    if (v >= 25.5f)  return 255;
    return (uint8_t)(v * 10.0f + 0.5f);
}

/** 状态回传 ③: 惯导模块的 GPS 快照, 2Hz 发给上位机 (type=0x04 + 0x05) */
static void tcp_poll_send_gps(void)
{
    if (!s_host_connected) {
        s_gps_next_us = 0;              /* 断开 → 重连后立刻发第一条 */
        return;
    }

    int64_t now = esp_timer_get_time();
    if (s_gps_next_us == 0) s_gps_next_us = now;
    if (now < s_gps_next_us) return;
    s_gps_next_us = now + GPS_PUSH_PERIOD_US;

    nav_gps_t d;
    if (nav_read_gps(&d) != ESP_OK) return;   /* 模块无 GPS 帧: 不发 */

    uint8_t flags = 0;
    if (d.valid)      flags |= CTRL_GPS_FLAG_FIX;
    if (d.time_valid) flags |= CTRL_GPS_FLAG_TIME;

    uint8_t f[CTRL_MPU_FRAME_SIZE];
    control_build_gps_pos_frame(f, flags,
        (int32_t)lround(d.latitude  * 1e7),      /* double: 用 lround, float 存不下 3e8 的整数 */
        (int32_t)lround(d.longitude * 1e7),
        (int16_t)lroundf(d.altitude_m * 10.0f),
        (uint8_t)(d.satellites > 255 ? 255 : d.satellites));
    wiz_send(HOST_SOCK, f, sizeof(f));

    control_build_gps_nav_frame(f, flags,
        (uint16_t)lroundf(d.course_deg * 100.0f),
        (uint16_t)lroundf(d.speed_kmh  * 100.0f),
        dop_to_u8(d.pdop), dop_to_u8(d.hdop), dop_to_u8(d.vdop),
        d.hour, d.minute, d.second);
    wiz_send(HOST_SOCK, f, sizeof(f));
}
#endif  /* NAV_ENABLE */

/** 处理 socket 0 (8080 上位机): listen → 接收 → 解析 → 断线重建 */
static void tcp_handle_host_socket(void)
{
    uint8_t sr = getSn_SR(HOST_SOCK);
    int32_t n;

    switch (sr) {
    case SOCK_ESTABLISHED:
        if (!s_host_connected) {
            s_host_connected = true;
            s_last_ctrl_us   = 0;      /* 等第一条控制帧才开始计时 */
            control_reset();           /* 清空上次连接的残帧 */
            ESP_LOGI("TCP", "[HOST] 上位机已连接 (8080)");
        }
        n = wiz_recv(HOST_SOCK, s_rx_buffer, sizeof(s_rx_buffer));
        if (n > 0) {
            if (control_process(s_rx_buffer, (size_t)n) > 0) {
                s_ctrl_frames++;
            }
            s_last_ctrl_us = esp_timer_get_time();   /* 收到数据 → 喂失联保护的狗 */
        } else if (n == SOCK_BUSY) {
            vTaskDelay(pdMS_TO_TICKS(2));
        } else {
            ESP_LOGI("TCP", "[HOST] 上位机已断开 (n=%d)", (int)n);
            wiz_close(HOST_SOCK);
            s_host_connected = false;
            s_last_ctrl_us   = 0;
        }
        break;

    case SOCK_CLOSE_WAIT:
    case SOCK_CLOSED:
        if (s_host_connected) {
            ESP_LOGI("TCP", "[HOST] 连接关闭 (SR=0x%02X), 重新监听 8080", sr);
        }
        s_host_connected = false;
        s_last_ctrl_us   = 0;
        wiz_close(HOST_SOCK);
        if (wiz_socket(HOST_SOCK, Sn_MR_TCP, TCP_HOST_PORT, SF_TCP_NODELAY) == (int8_t)HOST_SOCK) {
            wiz_listen(HOST_SOCK);
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        break;

    default:
        vTaskDelay(pdMS_TO_TICKS(10));
        break;
    }
}

/**
 * TCP 服务器任务 (仅 socket 0 = 8080 上位机)。
 * 8081 (远端 ESP32-S3 水下节点) 暂未启用 —— 见文件头说明, 设计在参数总览 §2。
 */
static void tcp_server_task(void *pvParameters)
{
    (void)pvParameters;

    /* 注册本地指令回调: control 只解析协议, 电机动作由上面的回调执行 */
    control_set_local_callback(tcp_apply_local_command);

    if (wiz_socket(HOST_SOCK, Sn_MR_TCP, TCP_HOST_PORT, SF_TCP_NODELAY) != HOST_SOCK) {
        ESP_LOGE("TCP", "socket 0 (8080) 创建失败");
    } else if (wiz_listen(HOST_SOCK) != SOCK_OK) {
        ESP_LOGE("TCP", "listen 8080 失败");
        wiz_close(HOST_SOCK);
    }
    ESP_LOGI("TCP", "=== TCP 服务器已启动: 8080 (上位机); 失联保护 %d ms ===",
             TCP_LINK_TIMEOUT_MS);
#if NAV_ENABLE
    ESP_LOGI("TCP", "状态回传: 主控 MPU 20Hz (type=0x01) + GPS 2Hz (type=0x04 定位 / 0x05 运动)");
#endif

    while (1) {
        wiznet_spi_check_int();     /* INT 唤醒 (如果有) */
        tcp_handle_host_socket();
        tcp_check_link_timeout();
        tcp_poll_send_local_mpu();  /* 20Hz 回传主控 MPU (type=0x01) */
#if NAV_ENABLE
        tcp_poll_send_gps();        /* 2Hz 回传惯导 GPS (type=0x04 + 0x05) */
#endif
    }
}

/** net : 查看网络 / TCP 连接状态 (联调用) */
static int cmd_net(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf("TCP Server : 端口 %d (上位机); 8081 (远端) 未启用\n", TCP_HOST_PORT);
    printf("网线链路   : %s", wiznet_manager_is_link_up() ? "已连接" : "未连接");
    if (wiznet_manager_is_link_up()) {
        printf("  (%d Mbps)", wiznet_manager_get_link_speed());
    }
    printf("\n");

    esp_netif_ip_info_t info;
    if (wiznet_manager_get_ip_info(&info) == ESP_OK) {
        /* addr 是 uint32_t, 在本工具链上等价于 unsigned long ⇒ 必须显式转 unsigned,
         * 否则 -Werror=format 会因为 %d 与实参类型不符直接编译失败 */
        printf("本机 IP    : %u.%u.%u.%u\n",
               (unsigned)(info.ip.addr & 0xFF), (unsigned)((info.ip.addr >> 8) & 0xFF),
               (unsigned)((info.ip.addr >> 16) & 0xFF), (unsigned)((info.ip.addr >> 24) & 0xFF));
    }

    printf("上位机连接 : %s\n", s_host_connected ? "已连接" : "未连接");
    printf("累计控制帧 : %lu\n", (unsigned long)s_ctrl_frames);
    if (s_host_connected) {
        if (s_last_ctrl_us == 0) {
            printf("指令计时   : 尚未收到控制帧 (收到后才启动 %d ms 失联保护)\n",
                   TCP_LINK_TIMEOUT_MS);
        } else {
            int64_t age_ms = (esp_timer_get_time() - s_last_ctrl_us) / 1000;
            printf("指令计时   : 距上一条控制帧 %lld ms (超过 %d ms 自动停机)\n",
                   (long long)age_ms, TCP_LINK_TIMEOUT_MS);
        }
    }
    printf("提示       : 上位机 = tools\\tcp_console.py, 连本机 %d 端口, 20Hz 发 16B 控制帧\n",
           TCP_HOST_PORT);
    return 0;
}

/** 初始化并启动串口控制台 */
static void console_init(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt = "gimbal>";
    repl_cfg.max_cmdline_length = 128;

    /* console 主通道随 sdkconfig 走: 必须用与主 console 匹配的 REPL 后端。
     * 用错后端 = 日志在一个口、REPL 在另一个口 —— 表现就是"能看日志、敲不进命令"
     * (例如主 console 是 USB-Serial/JTAG 时却用 UART0 的 REPL, 见参数总览 §7.9)。 */
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t usb_cfg = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usb_cfg, &repl_cfg, &repl));
#else
    esp_console_dev_uart_config_t uart_cfg = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_cfg, &repl_cfg, &repl));
#endif

    const esp_console_cmd_t cmds[] = {
        { .command = "p",  .help = "直接输出脉宽: p <ch> <us>",              .func = cmd_pulse  },
        { .command = "g",  .help = "按角度控制云台: g <ch> [deg] (不带 deg=查当前角度; 受软件限位约束, 要越限用 p)", .func = cmd_goto },
        { .command = "n",  .help = "在当前脉宽上微调: n <ch> <dus>",         .func = cmd_nudge  },
        { .command = "z",  .help = "回中: z [ch] (不带参数=全部通道)",        .func = cmd_center },
        { .command = "t",  .help = "设置中位微调并回中: t <ch> <trim_us>",    .func = cmd_trim   },
        { .command = "sc", .help = "设置完整标定: sc <ch> <min> <max> <trim> [range_deg] [phys_deg]", .func = cmd_setcal },
        { .command = "sl", .help = "设置软件行程限位: sl <ch> <min_deg> <max_deg>", .func = cmd_set_limit },
        { .command = "ge", .help = "姿态闭环开关: ge <ch> <0|1>",              .func = cmd_gimb_enable },
        { .command = "gt", .help = "闭环目标物理角: gt <ch> <phys_deg>",       .func = cmd_gimb_target },
        { .command = "gp", .help = "闭环 PID: gp <ch> <kp> <ki> <kd>",         .func = cmd_gimb_pid },
        { .command = "gs", .help = "查看云台闭环状态",                        .func = cmd_gimb_status },
        { .command = "st", .help = "查看 ch0/ch1 标定与当前脉宽",             .func = cmd_status },
        { .command = "ck", .help = "把当前脉宽捕获为该通道中位: ck <ch>",      .func = cmd_capture },
        { .command = "sv", .help = "保存标定到 NVS",                          .func = cmd_save   },
        { .command = "rs", .help = "复位通道标定: rs <ch>",                   .func = cmd_reset  },
        { .command = "att",.help = "姿态日志开关: att <0|1>",                 .func = cmd_att    },
        { .command = "t0", .help = "ch0 档位扫描测试开关: t0 <0|1>",          .func = cmd_test0  },
        { .command = "t1", .help = "ch1 角度扫描测试开关: t1 <0|1>",          .func = cmd_test1  },
        { .command = "gps",.help = "惯导 GPS 快照 (亚博 GPS+IMU 一体模块): gps 查状态 / gps 0|1 开关 1Hz 日志", .func = cmd_gps },
        { .command = "nav",.help = "惯导模块状态: 在线/输出配置(RSW/RRATE)/波特率/帧计数/两套快照", .func = cmd_nav },
        { .command = "imu",.help = "读一帧云台 MPU6050 (原始 6 轴)",              .func = cmd_imu },
        { .command = "scan",.help = "扫描 I2C1 (GPIO16/17): PCA9685 + 云台 MPU6050", .func = cmd_scan },
        { .command = "hull",.help = "惯导姿态 (亚博 GPS+IMU 一体): hull [0|1] 开关 10Hz 日志 (无参读一帧)", .func = cmd_hull },
        { .command = "m",  .help = "L298N 电机调速: m <-100~100> (正=正转, 负=反转, 0=停; |值|<60 按 60; 也可直接输裸数字)", .func = cmd_motor },
#if REV_ESC_ENABLE
        { .command = "l",  .help = "左路油门: l <-100~100> (正=推进 IO41, 负=反推 IO39, 0=停)", .func = cmd_esc_l },
        { .command = "r",  .help = "右路油门: r <-100~100> (正=推进 IO42, 负=反推 IO40, 0=停)", .func = cmd_esc_r },
#else
        { .command = "l",  .help = "左路油门: l <0~100> (正=推进 IO41; 负值=反推, 当前已关闭⇒按停处理)", .func = cmd_esc_l },
        { .command = "r",  .help = "右路油门: r <0~100> (正=推进 IO42; 负值同上按停处理)", .func = cmd_esc_r },
#endif
        { .command = "cal",.help = "电调行程标定第1步: cal <l|r|bl|br> (l/r=推进 IO41/42, bl/br=反推 IO39/40)", .func = cmd_esc_cal },
        { .command = "cal2",.help = "电调行程标定第2步: 听到双短鸣后立刻输 (降到最低油门完成标定)", .func = cmd_esc_cal2 },
        { .command = "pw", .help = "直接输出指定脉宽: pw <l|r|bl|br> <us> (500~2500, 人工测端点)", .func = cmd_pw },
        { .command = "net",.help = "网络/TCP 状态: 链路/IP/上位机连接/控制帧数/失联计时", .func = cmd_net },
    };

    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }
    ESP_ERROR_CHECK(esp_console_register_help_command());

    /* 注意: 这里刻意不调用 esp_console_start_repl().
     * 上面的 new_repl_* 已把 console 通道/VFS/linenoise 准备好 (其内部 REPL 任务会一直
     * 阻塞在 ulTaskNotifyTake 上, 不会自己去读串口), 我们改用自建任务, 以便支持"裸数字"输入.
     */
    xTaskCreate(console_repl_task, "console_repl", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "串口控制台已启动 (提示符 gimbal>): 输 help 看全部指令");
}

/* ========== [临时测试] ch0 (360° 舵机) 档位扫描任务 ========== */

/**
 * @brief ch0 档位扫描: 从 CH0_SWEEP_MIN_US 到 CH0_SWEEP_MAX_US 逐档输出, 每档停留
 *
 * 360° 连续旋转舵机: 脉宽 <1500µs 反转, =1500µs 停止, >1500µs 正转.
 * 逐档停留并打印脉宽, 便于目视找到"停止点"对应的脉宽, 再用 ck 0 + sv 固化.
 */
static void servo_ch0_test_task(void *pvParameters)
{
    /* 舵机关停/未就绪时, 扫描测试无意义, 直接退出 (避免日志谎报"已就绪") */
    if (!servo_is_ready()) {
        ESP_LOGW(TAG, "[CH0 TEST] PCA9685/servo 未就绪 (SERVO_ENABLE=0 或硬件异常) => 舵机扫描测试不启动");
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "[CH0 TEST] 扫描测试已就绪: %d~%d us, 步进 %d us, 每档停留 %d ms (默认关闭, t0 1 开启)",
             CH0_SWEEP_MIN_US, CH0_SWEEP_MAX_US, CH0_SWEEP_STEP_US, CH0_SWEEP_DWELL_MS);

    /* 先等姿态任务完成陀螺仪零偏标定, 否则云台转动会污染标定 */
    vTaskDelay(pdMS_TO_TICKS(AHRS_CAL_READY_MS));

    /* ---- 一次性上下限测量扫描 (单向, 跑完即停) ----
     * 观察要点: 注意"从哪一档开始动"和"哪一档之后不再动",
     *           即为该舵机的真实脉宽量程 (已知中点 1630µs).
     */
#if CH0_SCAN_ONCE
    ESP_LOGI(TAG, "[CH0 SCAN] 上下限测量开始: %d~%d us, 步进 %d us, 每档 %d ms (单向一次)",
             CH0_SCAN_MIN_US, CH0_SCAN_MAX_US, CH0_SCAN_STEP_US, CH0_SCAN_DWELL_MS);
    for (int us = CH0_SCAN_MIN_US; us <= CH0_SCAN_MAX_US; us += CH0_SCAN_STEP_US) {
        servo_set_pulse_us(0, (uint16_t)us);
        ESP_LOGI(TAG, "[CH0 SCAN] pulse=%d us", us);
        vTaskDelay(pdMS_TO_TICKS(CH0_SCAN_DWELL_MS));
    }
    servo_center(0);
    uint16_t scan_end_us = 0;
    servo_get_pulse_us(0, &scan_end_us);
    ESP_LOGI(TAG, "[CH0 SCAN] 上下限测量结束, 回到中点 %u us", scan_end_us);
    vTaskDelay(pdMS_TO_TICKS(1000));
#endif

#if CH0_ALT_ENABLE
    /* ---- 两点往返测试: A <-> B 循环, 每点停留 CH0_ALT_DWELL_MS ---- */
    ESP_LOGI(TAG, "[CH0 ALT] 两点往返: %d us <-> %d us, 每点 %d ms (循环)",
             CH0_ALT_A_US, CH0_ALT_B_US, CH0_ALT_DWELL_MS);
    while (1) {
        servo_set_pulse_us(0, CH0_ALT_A_US);
        ESP_LOGI(TAG, "[CH0 ALT] -> %d us", CH0_ALT_A_US);
        vTaskDelay(pdMS_TO_TICKS(CH0_ALT_DWELL_MS));

        servo_set_pulse_us(0, CH0_ALT_B_US);
        ESP_LOGI(TAG, "[CH0 ALT] -> %d us", CH0_ALT_B_US);
        vTaskDelay(pdMS_TO_TICKS(CH0_ALT_DWELL_MS));
    }
#else
    while (1) {
        if (!s_ch0_test_enabled) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        uint32_t step = 0;
        for (int us = CH0_SWEEP_MIN_US; us <= CH0_SWEEP_MAX_US; us += CH0_SWEEP_STEP_US) {
            if (!s_ch0_test_enabled) {
                break;
            }
            servo_set_pulse_us(0, (uint16_t)us);
            ESP_LOGI(TAG, "[CH0] step=%02lu  pulse=%d us", (unsigned long)step, us);
            step++;
            vTaskDelay(pdMS_TO_TICKS(CH0_SWEEP_DWELL_MS));
        }

        if (s_ch0_test_enabled) {
            /* 一轮扫描结束, 回中停 2s 便于对比 */
            servo_center(0);
            uint16_t cur = 0;
            servo_get_pulse_us(0, &cur);
            ESP_LOGI(TAG, "[CH0 TEST] 一轮结束, 回中 %u us 停 2s", cur);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
#endif
}

/**
 * @brief ch1 (180° 舵机) 角度扫描: 0° → 180° → 0° 往复, 每档停留并打印
 *
 * 便于目视检查行程是否覆盖完整、有无卡顿、两端是否顶死.
 */
static void servo_ch1_test_task(void *pvParameters)
{
    /* 舵机关停/未就绪时, 角度扫描测试无意义, 直接退出 */
    if (!servo_is_ready()) {
        ESP_LOGW(TAG, "[CH1 TEST] PCA9685/servo 未就绪 (SERVO_ENABLE=0 或硬件异常) => 舵机角度扫描测试不启动");
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "[CH1 TEST] 扫描测试已就绪: %.0f~%.0f°, 步进 %.0f°, 每档停留 %d ms (默认关闭, t1 1 开启)",
             CH1_SWEEP_MIN_DEG, CH1_SWEEP_MAX_DEG, CH1_SWEEP_STEP_DEG, CH1_SWEEP_DWELL_MS);

    /* 先等姿态任务完成陀螺仪零偏标定, 否则云台转动会污染标定 */
    vTaskDelay(pdMS_TO_TICKS(AHRS_CAL_READY_MS));

    /* ---- 一次性端点测量扫描 (单向, 跑完即停) ----
     * 观察要点:
     *   舵机"开始动"的那一档 = 真实 0° 对应脉宽 -> ch1 min_us
     *   舵机"停下不动"的那一档 = 真实 180° 对应脉宽 -> ch1 max_us
     */
#if CH1_SCAN_ONCE
    ESP_LOGI(TAG, "[CH1 SCAN] 端点测量开始: %d~%d us, 步进 %d us, 每档 %d ms (单向一次)",
             CH1_SCAN_MIN_US, CH1_SCAN_MAX_US, CH1_SCAN_STEP_US, CH1_SCAN_DWELL_MS);
    for (int us = CH1_SCAN_MIN_US; us <= CH1_SCAN_MAX_US; us += CH1_SCAN_STEP_US) {
        servo_set_pulse_us(1, (uint16_t)us);
        ESP_LOGI(TAG, "[CH1 SCAN] pulse=%d us", us);
        vTaskDelay(pdMS_TO_TICKS(CH1_SCAN_DWELL_MS));
    }
    ESP_LOGI(TAG, "[CH1 SCAN] 端点测量结束, 回到默认角 %.0f°", CH1_HOME_DEG);
    servo_set_angle(1, CH1_HOME_DEG);
    vTaskDelay(pdMS_TO_TICKS(1000));
#endif

    while (1) {
        if (!s_ch1_test_enabled) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        /* 正向: 0° -> 180° */
        ESP_LOGI(TAG, "[CH1] ===== 正向 0° -> 180° =====");
        for (float a = CH1_SWEEP_MIN_DEG; a <= CH1_SWEEP_MAX_DEG + 0.01f; a += CH1_SWEEP_STEP_DEG) {
            if (!s_ch1_test_enabled) {
                break;
            }
            servo_set_angle(1, a);
            uint16_t us = 0;
            servo_get_pulse_us(1, &us);
            ESP_LOGI(TAG, "[CH1] %.0f° -> %u us", a, us);
            vTaskDelay(pdMS_TO_TICKS(CH1_SWEEP_DWELL_MS));
        }

        /* 反向: 180° -> 0° */
        if (s_ch1_test_enabled) {
            ESP_LOGI(TAG, "[CH1] ===== 反向 180° -> 0° =====");
            for (float a = CH1_SWEEP_MAX_DEG; a >= CH1_SWEEP_MIN_DEG - 0.01f; a -= CH1_SWEEP_STEP_DEG) {
                if (!s_ch1_test_enabled) {
                    break;
                }
                servo_set_angle(1, a);
                uint16_t us = 0;
                servo_get_pulse_us(1, &us);
                ESP_LOGI(TAG, "[CH1] %.0f° -> %u us", a, us);
                vTaskDelay(pdMS_TO_TICKS(CH1_SWEEP_DWELL_MS));
            }
        }

        if (s_ch1_test_enabled) {
            /* 一轮结束, 回中 90° 停 2s */
            servo_set_angle(1, 90.0f);
            uint16_t us = 0;
            servo_get_pulse_us(1, &us);
            ESP_LOGI(TAG, "[CH1 TEST] 一轮结束, 回中 90° (%u us) 停 2s", us);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
}

/* ========== [临时测试] ch1 舵机 <-> MPU 关系测定 ========== */

#if TEST_MODE == 2

#define C1M_STEP_DEG    30.0f   /* 每档角度步进 (°) */
#define C1M_MIN_DEG     30.0f   /* 扫描起点 (°): 避开低端硬限位 */
#define C1M_MAX_DEG    150.0f   /* 扫描终点 (°): 避开高端硬限位 */
#define C1M_SETTLE_MS   500     /* 运动前静置 (ms), 让机械完全停稳 */
#define C1M_AFTER_MS    1200    /* 到位后再静置 (ms) */
#define C1M_POLL_MS     100     /* 到位轮询周期 (ms) */
#define C1M_TIMEOUT_MS  8000    /* 单次运动超时保护 (ms) */
#define C1M_MAX_VEL_DPS   60.0f /* ch1 限速 (°/s): 载荷惯量大, 用较柔的限速 */
#define C1M_MAX_ACC_DPS2  120.0f/* ch1 限加速 (°/s²) */

/**
 * @brief 测定 ch1 (俯仰轴) 舵机角度与 MPU 姿态的关系
 *
 * 原理:
 *   ch1 绕水平轴旋转, 重力在模块坐标系里的方向随之转动 ——
 *   所以"旋转前后重力方向的夹角"就等于实际转过的角度:
 *       θ = acos( g_before · g_after )   (g = 加速度计测到的单位重力矢量)
 *   这是绝对参考, 不积分、不漂移, 比陀螺积分更适合标定。
 *   同时给出融合角度的 Δpitch / Δroll, 用于判断绕的是哪个轴、转向如何。
 *
 * 预期 (若 ch1 确为俯仰轴):
 *   - 倾斜变化 ≈ 命令角增量, 比值 ≈ +1 或 -1 (-1 表示装反了)
 *   - Δpitch 主导变化, Δroll 基本不变
 *   - |a| 始终 ≈ 1g
 *
 * 说明: 复用姿态任务发布的值 (s_att_*), 不再自己访问 I2C。
 */
static void servo_ch1_mpu_task(void *pvParameters)
{
    ESP_LOGI(TAG, "[C1-MPU] 开始测定 ch1 舵机 <-> MPU 关系 (俯仰轴, 用重力矢量夹角)");
    ESP_LOGI(TAG, "[C1-MPU] ch1 每档 +%.0f°, 运动前静置 %dms, 到位后再静置 %dms",
             C1M_STEP_DEG, C1M_SETTLE_MS, C1M_AFTER_MS);

    /* 等陀螺零偏标定完成 (标定期间不能动云台) */
    vTaskDelay(pdMS_TO_TICKS(AHRS_CAL_READY_MS));

    /* ch1 载荷惯量大, 用较柔的运动限制, 避免合闸式冲击 */
    gimbal_limit_t lim = { .max_vel_dps  = C1M_MAX_VEL_DPS,
                           .max_acc_dps2 = C1M_MAX_ACC_DPS2 };
    gimbal_set_limit(1, &lim);

    while (1) {
        float    prev_cmd = 0.0f;
        float    sum_tilt = 0.0f;   /* 本轮实测转过角度累计 */
        bool     first    = true;
        uint32_t step     = 0;

        /* 只在限位窗口内走 (30~150°), 两端各留 30° 余量避开机械硬限位 */
        for (float cmd = C1M_MIN_DEG; cmd <= C1M_MAX_DEG + 0.1f; cmd += C1M_STEP_DEG) {
            /* 1. 运动前静置并记录基准 */
            vTaskDelay(pdMS_TO_TICKS(C1M_SETTLE_MS));

            float gbx = s_att_ax, gby = s_att_ay, gbz = s_att_az;
            float nb   = sqrtf(gbx * gbx + gby * gby + gbz * gbz);
            float pitch_before  = s_att_pitch_deg;
            float roll_before   = s_att_roll_deg;
            uint32_t cnt_before = s_att_count;

            /* 2. 走受限轨迹过去 (避免大惯量载荷被阶跃指令冲击) */
            gimbal_move_to(1, cmd);

            /* 3. 等到位; 期间记录陀螺峰值 (判断陀螺有没有测到旋转) */
            int   waited = 0;
            float g_peak = 0.0f;
            while (waited < C1M_TIMEOUT_MS) {
                vTaskDelay(pdMS_TO_TICKS(C1M_POLL_MS));
                waited += C1M_POLL_MS;
                gimbal_state_t st;
                gimbal_get_state(1, &st);

                float gm = sqrtf(s_att_gx * s_att_gx + s_att_gy * s_att_gy + s_att_gz * s_att_gz);
                if (gm > g_peak) {
                    g_peak = gm;
                }
                if (!st.moving) {
                    break;
                }
            }

            /* 4. 停稳后取结果 */
            vTaskDelay(pdMS_TO_TICKS(C1M_AFTER_MS));

            float gax = s_att_ax, gay = s_att_ay, gaz = s_att_az;
            float na  = sqrtf(gax * gax + gay * gay + gaz * gaz);

            /* 旋转前后重力矢量夹角 = 实际转过的角度 (绝对参考, 不漂移) */
            float tilt = 0.0f;
            if (nb > 0.01f && na > 0.01f) {
                float dot = (gbx * gax + gby * gay + gbz * gaz) / (nb * na);
                if (dot >  1.0f) dot =  1.0f;
                if (dot < -1.0f) dot = -1.0f;
                tilt = acosf(dot) * (180.0f / M_PI);
            }

            if (first) {
                ESP_LOGI(TAG, "[C1-MPU] ch1=%5.0f°  基准: pitch=%.1f° roll=%.1f°  |a|=%.2fg",
                         cmd, pitch_before, roll_before, nb);
                first = false;
            } else {
                float d_cmd = cmd - prev_cmd;
                sum_tilt += tilt;
                ESP_LOGI(TAG, "[C1-MPU] ch1=%5.0f° (Δ%+.0f°)  倾斜变化=%6.1f°  比值=%.2f  Δpitch=%+6.1f°  Δroll=%+6.1f°",
                         cmd, d_cmd, tilt, tilt / d_cmd,
                         s_att_pitch_deg - pitch_before, s_att_roll_deg - roll_before);
                /* 诊断: |g|峰值 反映陀螺有没有测到旋转; 更新次数 反映姿态任务是否还在跑 */
                ESP_LOGI(TAG, "[C1-MPU]   |g|峰值=%.1f°/s  姿态更新=%lu次  |a|=%.2fg",
                         g_peak, (unsigned long)(s_att_count - cnt_before), na);
            }

            prev_cmd = cmd;
            step++;
        }

        ESP_LOGI(TAG, "[C1-MPU] 本轮结束: ch1 命令 %.0f->%.0f° (%lu 档), 实际转过合计 %.1f°",
                 C1M_MIN_DEG, C1M_MAX_DEG, (unsigned long)step, sum_tilt);

        ESP_LOGI(TAG, "[C1-MPU] 回中 90° 停 3s");
        gimbal_move_to(1, 90.0f);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

#endif /* TEST_MODE == 2 */

/* ========== [临时测试] 云台运动规划 (速度/加速度受限) 测试 ========== */

#if TEST_MODE == 1

/**
 * @brief 让云台在 A/B 两端之间往复, 走受限轨迹, 并打印规划点与速度
 *
 * 用途: 大惯量载荷下, 对比"阶跃指令"与"受限轨迹"的差别 ——
 *       受限轨迹应该启停平缓、没有电流饱和声、到位后不过冲振荡。
 *       调 GIMBAL_MAX_VEL_DPS / GIMBAL_MAX_ACC_DPS2 两个宏找合适的柔度。
 */
static void gimbal_test_task(void *pvParameters)
{
    ESP_LOGI(TAG, "[GIMBAL] 运动规划测试: ch%d %.0f° <-> %.0f°, 限速 %.0f°/s, 限加速 %.0f°/s²",
             GIMBAL_TEST_CH, GIMBAL_TEST_A_DEG, GIMBAL_TEST_B_DEG,
             GIMBAL_MAX_VEL_DPS, GIMBAL_MAX_ACC_DPS2);

    gimbal_limit_t lim = {
        .max_vel_dps  = GIMBAL_MAX_VEL_DPS,
        .max_acc_dps2 = GIMBAL_MAX_ACC_DPS2,
    };
    gimbal_set_limit(GIMBAL_TEST_CH, &lim);

    /* 等姿态任务的陀螺零偏标定完成 (标定期间不能动云台) */
    vTaskDelay(pdMS_TO_TICKS(AHRS_CAL_READY_MS));

    const float targets[2] = { GIMBAL_TEST_A_DEG, GIMBAL_TEST_B_DEG };
    int idx = 0;

    while (1) {
        ESP_LOGI(TAG, "[GIMBAL] -> 目标 %.0f°", targets[idx]);
        gimbal_move_to(GIMBAL_TEST_CH, targets[idx]);

        /* 运动期间每 200ms 打印一次规划状态 */
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(200));

            gimbal_state_t st;
            gimbal_get_state(GIMBAL_TEST_CH, &st);
            ESP_LOGI(TAG, "[GIMBAL]   目标=%.1f° 规划点=%.1f° 速度=%.1f°/s  %s",
                     st.target_deg, st.current_deg, st.vel_dps,
                     st.moving ? "运动中" : "已到位");
            if (!st.moving) {
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(GIMBAL_HOLD_MS));
        idx = 1 - idx;
    }
}

#endif /* TEST_MODE == 1 */

/* ========== [临时测试] ch0 舵机 <-> MPU 关系测定 ========== */

#if TEST_MODE == 3

#define C0M_STEP_DEG    60.0f   /* 每档角度步进 (°) */
#define C0M_SETTLE_MS   500     /* 运动前静置 (ms), 让机械完全停稳 */
#define C0M_AFTER_MS    1200    /* 到位后再静置 (ms), 等融合值稳定 */
#define C0M_POLL_MS     100     /* 到位轮询周期 (ms) */
#define C0M_TIMEOUT_MS  8000    /* 单次运动超时保护 (ms) */

/**
 * @brief 测定 ch0 (平面旋转轴) 舵机角度与 MPU 偏航角的关系
 *
 * 原理:
 *   ch0 绕竖直轴旋转, 重力在模块坐标系里的方向不变 —— 加速度计对此完全无感,
 *   所以只能靠陀螺积分/姿态融合出来的 yaw 来测。
 *   本任务在每档"运动前"和"停稳后"各取一次姿态任务的 yaw, 差值即该档的
 *   实测旋转量; 与命令角度增量对比, 得到传动比与转向。
 *
 * 说明:
 *   - 无磁力计, yaw 是相对量且缓慢漂移, 所以只看**增量**, 不看绝对值
 *   - 增量按 (-180°, 180°] 归一化, 以消除 yaw 在 ±180° 处的环绕
 *   - 复用姿态任务的融合结果 (s_att_*), 不再自己访问 I2C
 *   - pitch/roll 每档都应基本不变, 这是"绕竖直轴旋转"的验证
 */
static void servo_ch0_mpu_task(void *pvParameters)
{
    ESP_LOGI(TAG, "[C0-MPU] 开始测定 ch0 舵机 <-> MPU 关系 (平面旋转, 用融合偏航角)");
    ESP_LOGI(TAG, "[C0-MPU] ch0 每档 +%.0f°, 运动前静置 %dms, 到位后再静置 %dms",
             C0M_STEP_DEG, C0M_SETTLE_MS, C0M_AFTER_MS);

    /* 等姿态任务的陀螺零偏标定完成 (标定期间不能动云台) */
    vTaskDelay(pdMS_TO_TICKS(AHRS_CAL_READY_MS));

    while (1) {
        float prev_cmd = 0.0f;
        float sum_dyaw = 0.0f;     /* 本轮实测旋转量累计 */
        bool  first    = true;
        uint32_t step  = 0;

        for (float cmd = 0.0f; cmd <= 360.0f + 0.1f; cmd += C0M_STEP_DEG) {
            /* 1. 运动前静置并记录基准 */
            vTaskDelay(pdMS_TO_TICKS(C0M_SETTLE_MS));
            float    yaw_before = s_att_yaw_deg;
            uint32_t cnt_before = s_att_count;

            /* 2. 走受限轨迹过去 (避免大惯量载荷被阶跃指令冲击) */
            gimbal_move_to(0, cmd);

            /* 3. 等到位 (带超时保护); 期间记录陀螺峰值, 用于判断陀螺有没有测到旋转 */
            int   waited  = 0;
            float g_peak  = 0.0f;
            float gz_peak = 0.0f;
            while (waited < C0M_TIMEOUT_MS) {
                vTaskDelay(pdMS_TO_TICKS(C0M_POLL_MS));
                waited += C0M_POLL_MS;
                gimbal_state_t st;
                gimbal_get_state(0, &st);

                float gm = sqrtf(s_att_gx * s_att_gx + s_att_gy * s_att_gy + s_att_gz * s_att_gz);
                if (gm > g_peak) {
                    g_peak = gm;
                }
                if (fabsf(s_att_gz) > fabsf(gz_peak)) {
                    gz_peak = s_att_gz;
                }

                if (!st.moving) {
                    break;
                }
            }

            /* 4. 停稳后取结果 */
            vTaskDelay(pdMS_TO_TICKS(C0M_AFTER_MS));
            float yaw_after = s_att_yaw_deg;

            /* 增量归一化到 (-180°, 180°], 消除 yaw 的 ±180° 环绕 */
            float d_yaw = yaw_after - yaw_before;
            while (d_yaw >  180.0f) d_yaw -= 360.0f;
            while (d_yaw <= -180.0f) d_yaw += 360.0f;

            if (first) {
                ESP_LOGI(TAG, "[C0-MPU] ch0=%5.0f°  基准: yaw=%.1f° pitch=%.1f° roll=%.1f°  |a|=%.2fg",
                         cmd, yaw_before, s_att_pitch_deg, s_att_roll_deg,
                         sqrtf(s_att_ax * s_att_ax + s_att_ay * s_att_ay + s_att_az * s_att_az));
                first = false;
            } else {
                float d_cmd = cmd - prev_cmd;
                sum_dyaw += d_yaw;
                ESP_LOGI(TAG, "[C0-MPU] ch0=%5.0f° (Δ%+.0f°)  Δyaw=%+6.1f°  比值=%+.2f  pitch=%5.1f° roll=%5.1f°",
                         cmd, d_cmd, d_yaw, d_yaw / d_cmd,
                         s_att_pitch_deg, s_att_roll_deg);
                /* 诊断: |g|峰值 反映陀螺有没有测到旋转; 更新次数 反映姿态任务是否还在跑 */
                ESP_LOGI(TAG, "[C0-MPU]   |g|峰值=%.1f°/s  gz峰值=%+.1f°/s  姿态更新=%lu次",
                         g_peak, gz_peak, (unsigned long)(s_att_count - cnt_before));
            }

            prev_cmd = cmd;
            step++;
        }

        ESP_LOGI(TAG, "[C0-MPU] 本轮结束: ch0 命令 0->360° (%lu 档), 实测 Δyaw 合计 %+.1f°",
                 (unsigned long)step, sum_dyaw);

        ESP_LOGI(TAG, "[C0-MPU] 回中 180° 停 3s");
        gimbal_move_to(0, 180.0f);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

#endif /* TEST_MODE == 3 */

/* ========== [临时测试] 云台闭环自稳测试 ========== */

#if TEST_MODE == 4

/**
 * @brief 开启 ch1 俯仰闭环, 目标设为 STAB_TARGET_DEG (默认水平), 周期打印误差
 *
 * 观察要点:
 *   - 误差应快速收敛到 0 附近并稳住 (静止时 |err| 应在 1° 以内)
 *   - 手动把云台/载荷推偏, 松手后应能自己回到目标
 *   - 持续振荡 -> kp 调小; 到位慢或有稳态差 -> ki 调大
 *   - 一开就朝反方向飞 -> 反馈方向反了 (改 gimbal.c 的 s_fb_sign 表)
 *   - 在线调参: gp 1 <kp> <ki> <kd> / gt 1 <目标角> / ge 1 0 (停)
 */
static void gimbal_stab_test_task(void *pvParameters)
{
    /* 舵机被关停 (SERVO_ENABLE=0) 或 PCA9685 未就绪时, 闭环根本下不了发 —— 直接退出,
     * 免得每 500ms 刷一遍无意义的 0 值, 让人误以为"稳住了"。 */
    if (!servo_is_ready()) {
        ESP_LOGW(TAG, "[STAB] PCA9685/servo 未就绪 (SERVO_ENABLE=0 或硬件异常) => 云台闭环自稳测试不启动");
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "[STAB] 云台闭环自稳测试: ch1 俯仰, 目标物理角 %.1f°", STAB_TARGET_DEG);

    /* 等陀螺零偏标定完成 (标定期间不能动云台) */
    vTaskDelay(pdMS_TO_TICKS(AHRS_CAL_READY_MS));

    /* 先回中站稳再开闭环 */
    gimbal_move_to(1, 90.0f);
    vTaskDelay(pdMS_TO_TICKS(STAB_SETTLE_MS));

    /* 反馈源自检: 云台 MPU6050 未就绪 (没接/标定失败) 时, 闭环会走"无反馈保护"
     * (见 gimbal.h): 停 PID 并把 ch1 送回标定中位, 而不是拿假 0° 当反馈。 */
    if (!imu_role_ready(IMU_ROLE_GIMBAL)) {
        ESP_LOGW(TAG, "[STAB] 云台 MPU6050 未就绪 => 闭环将停用 PID 并回中位 %.0f°",
                 (double)CH1_HOME_DEG);
    }

    esp_err_t stab_ret = gimbal_enable_stabilize(1, true);
    if (stab_ret != ESP_OK) {
        ESP_LOGE(TAG, "[STAB] 闭环开启失败: %s (PCA9685 未就绪时无法下发角度)",
                 esp_err_to_name(stab_ret));
    }
    vTaskDelay(pdMS_TO_TICKS(500));     /* 先锁在当前姿态, 再给目标, 避免一开就猛跳 */
    gimbal_set_target_phys(1, STAB_TARGET_DEG);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));

        gimbal_stab_state_t st;
        gimbal_pid_t pid;
        gimbal_get_stab_state(1, &st);
        gimbal_get_pid(1, &pid);
        ESP_LOGI(TAG, "[STAB] 目标=%+6.1f° 实测=%+6.1f° 误差=%+6.1f° 输出标称角=%6.1f° "
                      "(kp=%.2f ki=%.2f)%s",
                 st.target_phys, st.meas_phys, st.err, st.cmd_deg, pid.kp, pid.ki,
                 st.nofb ? "  [无反馈: 已停 PID, 回中位]" : "");
    }
}

#endif /* TEST_MODE == 4 */


/* ========== 启动流程 ========== */

static void start_components(void)
{
    /* 启动横幅: 模块清单与"已关停"备注都按上面的总开关**自动生成**,
     * 免得改了开关却忘了改这几行字 (v6.0.1 之前是写死的, 已经和实况不符过一次)。 */
    char off[64] = "";
    if (!NAV_ENABLE)     strcat(off, "惯导模块/");
    if (!SERVO_ENABLE)   strcat(off, "PCA9685/");
    if (!MPU6050_ENABLE) strcat(off, "云台MPU/");
    if (off[0]) off[strlen(off) - 1] = '\0';      /* 去掉末尾那个 '/' */
    char off_note[128] = "";
    if (off[0]) snprintf(off_note, sizeof(off_note), " (已关停: %s)", off);

#if W5500_ENABLE
    const char *net_txt = "W5500 + TCP Server 8080(上位机)";
#else
    const char *net_txt = "以太网已关停";
#endif
#if MOTOR_ENABLE
#if REV_ESC_ENABLE
    const char *motor_txt = " + 4路电调(推进IO41/42, 反推IO39/40) + L298N滚筒(IO38/48/47)";
#else
    const char *motor_txt = " + 水面电调(推进IO41/42, 反推已关闭) + L298N滚筒(IO38/48/47)";
#endif
#else
    const char *motor_txt = "";
#endif
    ESP_LOGI(TAG, "=== [当前模式] %s%s + 串口控制台%s ===", net_txt, motor_txt, off_note);
    ESP_LOGI(TAG, "start_components 开始执行");

    /* 0. 板载 WS2812 RGB LED (数据脚 GPIO26) —— 尽早钉在低电平
     *
     * 不给任何边沿 ⇒ WS2812 保持上电默认的"熄灭"状态 (原因见 BOARD_RGB_LED_GPIO 注释;
     * ⚠️ 需**断电重上一次**那颗灯才会真的灭)。*/
    gpio_config_t rgb_cfg = {
        .pin_bit_mask = 1ULL << BOARD_RGB_LED_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&rgb_cfg));
    gpio_set_level(BOARD_RGB_LED_GPIO, 0);

    /* 1. NVS (供 IDF 内部组件使用) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS 需要擦除, 正在擦除...");
        esp_err_t erase_ret = nvs_flash_erase();
        if (erase_ret != ESP_OK) {
            ESP_LOGE(TAG, "NVS 擦除失败: %s", esp_err_to_name(erase_ret));
        }
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS 初始化失败: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "NVS 初始化成功");
    }

    /* 2. W5500 以太网 (初始化 + 拿 IP; TCP Server 在步骤 6 之后由任务 8 启动)
     *    必须在控制 task 之前初始化
     *    v5.12: 由 W5500_ENABLE 总开关控制, 室外测 GPS 时置 0 可省掉 30s 等链路 */
#if W5500_ENABLE
    wiznet_manager_config_t wcfg = wiznet_manager_get_default_config();
    /* 板载 W5500 (v5.10 换板 ESP32-S3-ETH, 引脚由板卡固定),
     * 默认配置已是本项目约定值: IP 192.168.29.10 / 网关 192.168.29.1 / DNS 8.8.8.8,
     * 这里显式再赋一遍, 便于一眼看到本节点的网络参数 (与远端节点 .11 同 /24 网段). */
    wcfg.ip[0]      = 192; wcfg.ip[1]      = 168; wcfg.ip[2]      = 29; wcfg.ip[3]      = 10;
    wcfg.gateway[0] = 192; wcfg.gateway[1] = 168; wcfg.gateway[2] = 29; wcfg.gateway[3] = 1;
    ESP_ERROR_CHECK(wiznet_manager_init(&wcfg));
    ESP_LOGI(TAG, "W5500 初始化成功 (IP 192.168.29.10/24, 网关 192.168.29.1)");
#else
    ESP_LOGW(TAG, "以太网已关停 (W5500_ENABLE=0): 不初始化 W5500, 不等 link up, 不启动 TCP Server");
#endif

    /* 3. 云台姿态传感器 (MPU6050) —— v5.11.2 起挪到 **4c**, 与惯导模块一起初始化。
     *    原因: 云台 MPU6050 现在与 PCA9685 **共用 I2C1** (**v8.0 起 SDA=16/SCL=17**), 而这条总线由 servo 组件安装,
     *    所以它必须排在 servo_init() 之后; 它原来占用的 I2C0 (16/18) 现给惯导模块的 UART2。
     *
     * 3b. (v6.0 已删除) 旧 GPS: NMEA-0183 / UART1 (TX=2, RX=1, PPS=3) —— 组件与命令全部删除,
     *     GPS 数据改由惯导模块 (UART2, WIT 0x55 协议) 提供, 初始化见 4c。 */

    /* 4. Servo (PCA9685) - I2C1: SDA=GPIO16, SCL=GPIO17
     *    ⚠️ 是否调用 servo_init() 取决于"还有没有别的 I2C1 使用者":
     *       - 云台 MPU6050 与 PCA9685 共线, 且只是**借用**总线 ⇒ MPU6050_ENABLE=1 时必须由 servo 装;
     *       - 船体侧现在只有惯导模块, 它走 **UART2**, 不占 I2C1;
     *       - 两者都关停 ⇒ I2C1 没有任何使用者, **完全跳过 servo_init()**, 连总线都不装。 */
    esp_err_t servo_ret = ESP_ERR_NOT_SUPPORTED;
#if SERVO_ENABLE || MPU6050_ENABLE
    ESP_LOGI(TAG, "初始化 PCA9685...");
    servo_config_t scfg = servo_get_default_config();
    scfg.pca9685_enable = (SERVO_ENABLE != 0);
    servo_ret = servo_init(&scfg);
    if (servo_ret == ESP_OK) {
        /* 开机回中 (servo_init 已按各通道标定把全部通道置到中点):
         *   ch0 360° 位置舵机 -> 1630µs
         *   ch1 180° 位置舵机 -> 1650µs (= 90°)
         * CH1_HOME_DEG 可改 ch1 开机角; 串口控制台直接输数字可改 ch0 脉宽.
         */
        servo_set_angle(1, CH1_HOME_DEG);

        uint16_t ch0_us = 0, ch1_us = 0;
        servo_get_pulse_us(0, &ch0_us);
        servo_get_pulse_us(1, &ch1_us);
        ESP_LOGI(TAG, "PCA9685 初始化成功, 已回中: ch0=%u us, ch1=%u us (%.0f°)",
                 ch0_us, ch1_us, CH1_HOME_DEG);

#if CH0_HOLD_US > 0
        /* ch0 固定脉宽测试: 定死在指定脉宽, 便于观察该脉宽下舵机行为 */
        servo_set_pulse_us(0, CH0_HOLD_US);
        ESP_LOGI(TAG, "ch0 固定脉宽测试: 定死在 %d us", CH0_HOLD_US);
#endif
    } else if (!SERVO_ENABLE) {
        /* 主动关停, 不是故障: 只提示 */
        ESP_LOGW(TAG, "PCA9685 已暂时关停 (SERVO_ENABLE=0): 仅装 I2C1 总线供云台 MPU 借用, 云台舵机/闭环不启用");
    } else {
        ESP_LOGE(TAG, "PCA9685 初始化失败: %s", esp_err_to_name(servo_ret));
    }
#else
    ESP_LOGW(TAG, "PCA9685 / I2C1 已关停 (SERVO_ENABLE=0 且 MPU6050_ENABLE=0): 不装 I2C1, 不调 servo_init()");
#endif

    /* 4b. 云台运动规划 (依赖 servo, 必须在舵机初始化之后)
     *     把"阶跃指令"变成速度/加速度受限的轨迹, 抑制大惯量载荷的冲击与过冲. */
    if (servo_ret == ESP_OK) {
        if (gimbal_init() == ESP_OK) {
            ESP_LOGI(TAG, "云台运动规划初始化成功");
        } else {
            ESP_LOGE(TAG, "云台运动规划初始化失败");
        }
    }

    /* 4c. 船体侧的两类感知器件 —— 各由自己的总开关控制, 关停时完全不初始化:
     *   ① 云台 MPU6050  : I2C1 0x68, 与 PCA9685 **共用**同一条总线 (只是借用, 所以必须排在 servo_init() 之后)
     *                     —— 开关 MPU6050_ENABLE, 它是云台闭环的反馈源。
     *   ② 惯导模块       : **UART2** (ESP32 RX=IO1 <- 模块 TX, ESP32 TX=IO2 -> 模块 RX),
     *                     亚博 GPS+10 轴 IMU 一体模块, 维特(WIT) 0x55 协议**主动上报**,
     *                     一条线同时给出姿态与 GPS —— 开关 NAV_ENABLE, 详见 components/nav/nav.h。
     *                     (它只占 UART, 与 I2C 无关, 所以与 servo_init() 先后都行) */
#if MPU6050_ENABLE
    ESP_LOGI(TAG, "初始化云台 MPU6050 (I2C1 共用总线, 0x68)...");
    imu_config_t icfg = imu_get_default_config();
    esp_err_t imu_ret = imu_init_role(IMU_ROLE_GIMBAL, &icfg);
    if (imu_ret == ESP_OK && imu_role_ready(IMU_ROLE_GIMBAL)) {
        ESP_LOGI(TAG, "云台 MPU6050 初始化成功");
    } else {
        ESP_LOGE(TAG, "云台 MPU6050 初始化失败: %s", esp_err_to_name(imu_ret));
    }
#else
    /* 云台没插 / 暂不测试: 不初始化 —— 顺带省掉引脚自检、
     * "地址 0x68 无应答"、"未找到 MPU6050" 这一串报错刷屏, 姿态解算任务也不启动 */
    ESP_LOGW(TAG, "云台 MPU6050 已暂时关停 (MPU6050_ENABLE=0): 不初始化, 姿态解算任务不启动");
#endif

#if NAV_ENABLE
    ESP_LOGI(TAG, "初始化惯导模块 (UART2: ESP32 RX=IO1 <- 模块 TX, TX=IO2 -> 模块 RX)...");
    nav_config_t ncfg = nav_get_default_config();
    esp_err_t nav_ret = nav_init(&ncfg);
    if (nav_ret == ESP_OK) {
        ESP_LOGI(TAG, "惯导模块初始化成功 (GPS + 10 轴 IMU)");
    } else {
        /* 模块没接上属正常, 只提示不报错; 接好后驱动会自动补配置并出数据, 不必重启 */
        ESP_LOGW(TAG, "惯导模块未接入: %s", esp_err_to_name(nav_ret));
    }
#else
    ESP_LOGW(TAG, "惯导模块已关停 (NAV_ENABLE=0): 不装 UART2, GPS 与船体姿态都不可用");
#endif

    /* 5. Motor —— 4 路电调通道全部初始化: 2 路推进油门线 (ESC1=IO41 左 / ESC2=IO42 右)
     *    + 2 路反推方向线 (REV1=IO39 左 / REV2=IO40 右), 帧率 50Hz。
     *    好盈 SkyWalker V2 "反推刹车"接线: 方向线只给 0/100% 两个方向位, 速度走油门线;
     *    脉宽 1100µs=停 / 1141~1940µs=油门 (见 motor.c)。
     *    ⚠️ **反推当前暂时关闭** (`REV_ESC_ENABLE=0`): 负油门按停处理, 方向线只保持 1100µs
     *       正向半区 (初始化它是为了让它有确定电平, 不是让它动作) —— 见 esc_apply_side() 与 §0。
     *    L298N 通道仍照常初始化并保持停机 (ENA=IO38 / IN1=IO48 / IN2=IO47) —— 引脚被主动
     *    驱动为"停", 比放开变悬空更安全, 但本轮不参与测试。 */
#if MOTOR_ENABLE
    motor_config_t mcfg = motor_get_default_config();
    esp_err_t motor_ret = motor_init(&mcfg);
    if (motor_ret == ESP_OK) {
        ESP_LOGI(TAG, "电调就绪 (4 路, 已置最低油门 1100us 停): 推进 IO41/IO42, 反推 IO39/IO40");
#if REV_ESC_ENABLE
        ESP_LOGI(TAG, "反推已启用: 负油门 = 方向线反转半区 + 油门线给速度");
#else
        ESP_LOGW(TAG, "反推暂时关闭 (REV_ESC_ENABLE=0): 负油门按停处理; 方向线 IO39/IO40 保持 1100us 正向半区");
#endif
#if REV_ESC_ENABLE
        ESP_LOGI(TAG, "控制台: l/r <-100~100> 或裸数字 (正=推进, 负=反推); 例: l 30 / l -30 / 0");
#else
        ESP_LOGI(TAG, "控制台: l/r <0~100> 或裸数字 (正=推进; 负值按停处理); 例: l 30 / r 30 / 0");
#endif
        ESP_LOGI(TAG, "L298N 已初始化并停机 (ENA=IO38, IN1=IO48, IN2=IO47), 本轮不测试");
    } else {
        ESP_LOGE(TAG, "电机初始化失败: %s", esp_err_to_name(motor_ret));
    }
#else
    ESP_LOGW(TAG, "电机已关停 (MOTOR_ENABLE=0): 不初始化 L298N");
#endif

    /* 6. 等待 link up (30s 超时) 并打印本机 IP
     *    注: 这里只确认链路与网络参数 (拿 IP); TCP Server (8080) 在步骤 8 里由
     *        tcp_server_task 启动 —— 必须先有链路和 IP, socket 才能正常工作。
     *    W5500_ENABLE=0 时整段跳过 (不初始化自然也没有链路可等) */
#if W5500_ENABLE
    ESP_LOGI(TAG, "等待网线连接...");
    int wait_ms = 0;
    while (!wiznet_manager_is_link_up() && wait_ms < 30000) {
        vTaskDelay(pdMS_TO_TICKS(500));
        wait_ms += 500;
    }
    if (wiznet_manager_is_link_up()) {
        ESP_LOGI(TAG, "=== 网线已连接 ===");
        esp_netif_ip_info_t info;
        if (wiznet_manager_get_ip_info(&info) == ESP_OK) {
            /* 同 cmd_net: uint32_t 在本工具链上是 unsigned long, 必须转 unsigned 配 %u */
            ESP_LOGI(TAG, "本机 IP: %u.%u.%u.%u",
                     (unsigned)(info.ip.addr & 0xFF), (unsigned)((info.ip.addr >> 8) & 0xFF),
                     (unsigned)((info.ip.addr >> 16) & 0xFF), (unsigned)((info.ip.addr >> 24) & 0xFF));
            ESP_LOGI(TAG, "子网掩码: %u.%u.%u.%u",
                     (unsigned)(info.netmask.addr & 0xFF), (unsigned)((info.netmask.addr >> 8) & 0xFF),
                     (unsigned)((info.netmask.addr >> 16) & 0xFF), (unsigned)((info.netmask.addr >> 24) & 0xFF));
            ESP_LOGI(TAG, "默认网关: %u.%u.%u.%u",
                     (unsigned)(info.gw.addr & 0xFF), (unsigned)((info.gw.addr >> 8) & 0xFF),
                     (unsigned)((info.gw.addr >> 16) & 0xFF), (unsigned)((info.gw.addr >> 24) & 0xFF));
        }
    } else {
        ESP_LOGW(TAG, "=== 网线未连接 (30s 超时), 继续启动 ===");
    }
#endif

    /* 7. 日志降噪 (默认开 INFO, 但关闭驱动内不必要 tag) */
    //esp_log_level_set("wifi",       ESP_LOG_WARN);
    //esp_log_level_set("spi_master", ESP_LOG_WARN);
    //esp_log_level_set("gpio",       ESP_LOG_WARN);

    /* 8. 创建任务
     *    TCP Server (8080 上位机) —— 收 16B 控制帧驱动电调/滚筒, 带 500ms 失联保护。
     *    栈 8192: control 解析 + wiz_* 调用 + printf 缓冲较费栈。
     *    mpu_push_task / status_report_task 仍不启动 (无 MPU 数据源, 暂不回传状态)。 */
#if W5500_ENABLE
    xTaskCreate(tcp_server_task, "tcp_srv", 8192, NULL, 5, NULL);
#endif

#if MPU6050_ENABLE
    /* [临时测试] 启动 MPU 三维姿态解算任务 (依赖云台 MPU6050) */
    xTaskCreate(attitude_task, "attitude", 4096, NULL, 5, NULL);
#endif

    /* [临时测试] 惯导日志任务: GPS 1Hz (默认静默, 用 gps 1 开) + 姿态 10Hz (用 hull 1 开) */
#if NAV_ENABLE
    xTaskCreate(nav_gps_log_task, "nav_gps", 4096, NULL, 4, NULL);
    xTaskCreate(nav_att_log_task, "nav_att", 4096, NULL, 4, NULL);
#endif

    /* [临时测试] 启动 ch0 舵机档位扫描任务 */
    xTaskCreate(servo_ch0_test_task, "ch0_test", 3072, NULL, 4, NULL);

    /* [临时测试] 启动 ch1 舵机角度扫描任务 */
    xTaskCreate(servo_ch1_test_task, "ch1_test", 3072, NULL, 4, NULL);

#if TEST_MODE == 1
    /* [临时测试] 启动云台运动规划测试任务 */
    xTaskCreate(gimbal_test_task, "gimbal_test", 4096, NULL, 4, NULL);
#elif TEST_MODE == 2
    /* [临时测试] 启动 ch1 舵机 <-> MPU 关系测定任务 */
    xTaskCreate(servo_ch1_mpu_task, "c1_mpu", 4096, NULL, 4, NULL);
#elif TEST_MODE == 3
    /* [临时测试] 启动 ch0 舵机 <-> MPU 关系测定任务 */
    xTaskCreate(servo_ch0_mpu_task, "c0_mpu", 4096, NULL, 4, NULL);
#elif TEST_MODE == 4
    /* [临时测试] 启动云台闭环自稳测试任务 */
    xTaskCreate(gimbal_stab_test_task, "stab_test", 4096, NULL, 4, NULL);
#endif

    /* [临时测试] 启动串口标定控制台 (最后启动, 之前日志不干扰提示符) */
    console_init();

#if W5500_ENABLE
    ESP_LOGI(TAG, "=== [当前模式] 启动完成: W5500 + TCP Server 8080(上位机, 失联保护 500ms) + 水面电调(推进 IO41/42 + 反推方向线 IO39/40) + L298N滚筒 + 串口控制台 ===");
#else
    ESP_LOGI(TAG, "=== [当前模式] 启动完成: 以太网已关停 + 水面电调(推进 IO41/42 + 反推方向线 IO39/40) + L298N滚筒 + 串口控制台 ===");
#endif
}

void app_main(void)
{
    ESP_LOGI(TAG, "app_main 入口 - 波特率 115200");
    start_components();
}
