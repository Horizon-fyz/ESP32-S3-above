/**
 * @file main.c
 * @brief ESP32-S3 W5500 AUV 网络控制器 (主控制节点, TCP Server)
 *
 * [临时测试模式 - 仅保留 PCA9685 + MPU6050, 其他全部注释]
 *
 * 角色: 主控制节点 (核心控制 + 数据处理 + 指令分发)
 * 硬件: MPU6050 + 2 ESC + 1 L298N + 2 Servo + W5500
 * 网络: TCP Server, 双端口
 *   - 8080: 上位机 (笔记本)  - 发送控制指令, 接收 MPU 数据
 *   - 8081: 远端 ESP32-S3 节点 - 接收转发控制 + 双向 MPU 推送
 *
 * 集成组件:
 *   - wiznet    : 板载 W5500 以太网 (ioLibrary, 20MHz SPI polling, 100M FULL)
 *   - imu       : MPU6050 (I2C0: SDA=16, SCL=18) + 亚博 10轴IMU (I2C1: SDA=21, SCL=17, 0x50)
 *   - servo     : PCA9685 (I2C1: SDA=21, SCL=17)
 *   - gps       : NMEA0183 (UART1: TX=2, RX=1, PPS=3)
 *   - motor     : 4 ESC (LEDC, 42/41/40/39) + L298N (ENA=38, IN1=48, IN2=47)
 *   - control   : 16B 控制帧 + 16B 状态帧协议 (v3.0 沉浮: cmd=0x11 + type=0x03)
 *
 * 任务清单:
 *   - tcp_server_task    优先级 5, 栈 8192, 双 socket 状态机
 *   - mpu_push_task      优先级 4, 栈 2048, 周期 50ms (20Hz)
 *   - status_report_task 优先级 4, 栈 2048, 周期 500ms (保留, 可选)
 */

//#include "wiznet_conf.h"
//#include "wizchip_conf.h"
//#include "wiznet_socket.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
//#include <inttypes.h>
//#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_console.h"
#include "linenoise/linenoise.h"
#include "esp_timer.h"
//#include "esp_heap_caps.h"
//#include "esp_task_wdt.h"
#include "nvs_flash.h"
//#include "wiznet_manager.h"
//#include "wiznet_spi.h"
//#include "motor.h"
#include "imu.h"
#include "servo.h"
#include "gimbal.h"
#include "gps.h"
//#include "control.h"

/* ========== TCP 服务器配置 ========== */
//#define TCP_HOST_PORT     8080    /* 上位机 (笔记本) */
//#define TCP_REMOTE_PORT   8081    /* 远端 ESP32-S3 节点 */
//#define HOST_SOCK         0
//#define REMOTE_SOCK       1
//#define RX_BUFFER_SIZE    1024
//#define TX_BUFFER_SIZE    512
//#define MPU_FRAME_SIZE    16

/* 硬件引脚定义 */

/* GPS 引脚 (UART1) —— v5.10 换板后取左排 2/1/3 三个相邻脚 */
#define GPS_TX_GPIO       2       /* ESP32 TX -> GPS RX */
#define GPS_RX_GPIO       1       /* ESP32 RX <- GPS TX */
#define GPS_PPS_GPIO      3       /* PPS 秒脉冲输入 (strapping 脚, 仅输入), -1 = 不用 */

//static uint8_t s_rx_buffer[RX_BUFFER_SIZE];
//static uint8_t s_tx_buffer[TX_BUFFER_SIZE];
//static uint8_t s_mpu_frame[MPU_FRAME_SIZE];

static const char *TAG = "APP";

/* ========== 连接状态 ========== */
//static atomic_bool s_host_connected   = ATOMIC_VAR_INIT(false);
//static atomic_bool s_remote_connected = ATOMIC_VAR_INIT(false);

/* ========== 任务句柄 ========== */
//static TaskHandle_t s_mpu_push_task_handle = NULL;
//static TaskHandle_t s_status_report_task_handle = NULL;

/* ========== 协议实现 ========== */

///* 主 → 远端 转发 (由 control 模块通过回调调用)
// *
// * 收到上位机 16B 控制帧后, 若 flags bit1 置位, 主节点构造 8B 子帧
// * 通过 socket 1 发给远端. 8B 子帧携带 speed+yaw+light+bucket_speed, 远端
// * 自行做差速混合并驱动 L298N 铲斗电机. v3.0: cmd=0x11 DEPTH 时构造沉浮子帧
// * (目标深度 cm + 目标俯仰° + 目标横滚°). */
//static void forward_to_remote(const ctrl_command_t *cmd)
//{
//    if (!atomic_load(&s_remote_connected)) {
//        ESP_LOGW("control", "远程未连接, 转发丢弃");
//        return;
//    }
//
//    if (cmd->cmd == CTRL_CMD_DEPTH) {
//        /* v3.0 沉浮控制子帧: AA 55 0x11 depth_cm(LE) pitch(°) roll(°) CRC */
//        uint8_t fwd[CTRL_FWD_FRAME_SIZE];
//        control_build_depth_fwd_frame(fwd, cmd->target_depth_cm,
//                                      cmd->target_pitch_deg, cmd->target_roll_deg);
//        int32_t sent = wiz_send(REMOTE_SOCK, fwd, CTRL_FWD_FRAME_SIZE);
//        if (sent != CTRL_FWD_FRAME_SIZE) {
//            ESP_LOGW("control", "转发沉浮到远端失败: %d", (int)sent);
//        }
//        return;
//    }
//
//    /* 构造 8B 子帧: AA 55 cmd speed yaw remote_light bucket_speed CRC */
//    uint8_t fwd[CTRL_FWD_FRAME_SIZE];
//    fwd[0] = CTRL_CTRL_HEAD_0;
//    fwd[1] = CTRL_CTRL_HEAD_1;
//    fwd[2] = cmd->cmd;
//    fwd[3] = (uint8_t)cmd->speed;
//    fwd[4] = (uint8_t)cmd->yaw;
//    fwd[5] = cmd->remote_light;
//    fwd[6] = (uint8_t)cmd->bucket_speed;
//    uint8_t crc = 0;
//    for (int i = 0; i < 7; i++) crc ^= fwd[i];
//    fwd[7] = crc;
//
//    int32_t sent = wiz_send(REMOTE_SOCK, fwd, CTRL_FWD_FRAME_SIZE);
//    if (sent != CTRL_FWD_FRAME_SIZE) {
//        ESP_LOGW("control", "转发到远端失败: %d", (int)sent);
//    }
//}

///* MPU 数据推送到上位机 (由 control 模块通过 handle_mpu_frame 调用) */
//void tcp_server_forward_mpu_to_host(const ctrl_mpu_data_t *m)
//{
//    if (!atomic_load(&s_host_connected)) return;
//    control_build_mpu_frame(s_mpu_frame, m->type,
//                            m->ax, m->ay, m->az, m->gx, m->gy, m->gz);
//    int32_t sent = wiz_send(HOST_SOCK, s_mpu_frame, MPU_FRAME_SIZE);
//    if (sent != MPU_FRAME_SIZE) {
//        ESP_LOGW("control", "MPU 推上位机失败: %d", (int)sent);
//    }
//}

///* MPU 数据推送到远端 (主节点 MPU 自身) */
//static void push_mpu_to_remote(uint8_t type, int16_t ax, int16_t ay, int16_t az,
//                                int16_t gx, int16_t gy, int16_t gz)
//{
//    if (!atomic_load(&s_remote_connected)) return;
//    control_build_mpu_frame(s_mpu_frame, type, ax, ay, az, gx, gy, gz);
//    wiz_send(REMOTE_SOCK, s_mpu_frame, MPU_FRAME_SIZE);
//}

/* ========== 任务实现 ========== */

///* 通用 socket 处理: listen -> 接收 -> 解析 -> 关闭重建 */
//static void handle_socket(uint8_t sock, const char *name,
//                          atomic_bool *connected_flag, void (*on_connect)(void))
//{
//    uint8_t sr = getSn_SR(sock);
//    int32_t n;
//
//    switch (sr) {
//    case SOCK_ESTABLISHED:
//        if (!(*connected_flag)) {
//            *connected_flag = true;
//            ESP_LOGI("TCP", "[%s] 已连接", name);
//            control_reset();   /* 切换连接时清空残帧 */
//            if (on_connect) on_connect();
//        }
//        n = wiz_recv(sock, s_rx_buffer, sizeof(s_rx_buffer));
//        if (n > 0) {
//            /* 用独立的解析器, 因为两条链路可能并发 */
//            control_process(s_rx_buffer, (size_t)n);
//        } else if (n == SOCK_BUSY) {
//            vTaskDelay(pdMS_TO_TICKS(2));
//        } else {
//            /* 对端关闭或错误 */
//            wiz_close(sock);
//            *connected_flag = false;
//            ESP_LOGI("TCP", "[%s] 已断开 (n=%d)", name, (int)n);
//            if (wiz_socket(sock, Sn_MR_TCP, (sock == HOST_SOCK) ? TCP_HOST_PORT : TCP_REMOTE_PORT,
//                           SF_TCP_NODELAY) == (int8_t)sock) {
//                wiz_listen(sock);
//            }
//        }
//        break;
//
//    case SOCK_CLOSE_WAIT:
//        *connected_flag = false;
//        wiz_close(sock);
//        if (wiz_socket(sock, Sn_MR_TCP, (sock == HOST_SOCK) ? TCP_HOST_PORT : TCP_REMOTE_PORT,
//                       SF_TCP_NODELAY) == (int8_t)sock) {
//            wiz_listen(sock);
//        }
//        break;
//
//    case SOCK_CLOSED:
//        *connected_flag = false;
//        if (wiz_socket(sock, Sn_MR_TCP, (sock == HOST_SOCK) ? TCP_HOST_PORT : TCP_REMOTE_PORT,
//                       SF_TCP_NODELAY) == (int8_t)sock) {
//            wiz_listen(sock);
//        } else {
//            vTaskDelay(pdMS_TO_TICKS(100));
//        }
//        break;
//
//    default:
//        vTaskDelay(pdMS_TO_TICKS(10));
//        break;
//    }
//}

///**
// * @brief TCP 服务器任务 (双 socket)
// *
// * Socket 0 = 8080 (上位机)
// * Socket 1 = 8081 (远端)
// *
// * 注意: 本任务做阻塞式 SPI 通讯, 不订阅 Task WDT.
// */
//static void tcp_server_task(void *pvParameters)
//{
//    esp_task_wdt_delete(NULL);
//
//    /* 注册主→远端 转发回调 */
//    control_set_forward_callback(forward_to_remote);
//
//    /* 启动 socket 0 (8080 上位机) */
//    if (wiz_socket(HOST_SOCK, Sn_MR_TCP, TCP_HOST_PORT, SF_TCP_NODELAY) != HOST_SOCK) {
//        ESP_LOGE("TCP", "socket 0 (HOST) failed");
//    } else if (wiz_listen(HOST_SOCK) != SOCK_OK) {
//        ESP_LOGE("TCP", "listen 8080 failed");
//        wiz_close(HOST_SOCK);
//    }
//
//    /* 启动 socket 1 (8081 远端) */
//    if (wiz_socket(REMOTE_SOCK, Sn_MR_TCP, TCP_REMOTE_PORT, SF_TCP_NODELAY) != REMOTE_SOCK) {
//        ESP_LOGE("TCP", "socket 1 (REMOTE) failed");
//    } else if (wiz_listen(REMOTE_SOCK) != SOCK_OK) {
//        ESP_LOGE("TCP", "listen 8081 failed");
//        wiz_close(REMOTE_SOCK);
//    }
//
//    ESP_LOGI("TCP", "=== TCP 服务器已启动: 8080(HOST) + 8081(REMOTE) ===");
//
//    while (1) {
//        esp_task_wdt_reset();
//
//        /* INT 唤醒 (如果有) */
//        wiznet_spi_check_int();
//
//        /* 交替处理两个 socket */
//        handle_socket(HOST_SOCK,   "HOST",   &s_host_connected,   NULL);
//        handle_socket(REMOTE_SOCK, "REMOTE", &s_remote_connected, NULL);
//    }
//}

///* MPU 推送任务: 20Hz 读取本地 MPU, 推送到上位机 + 远端 */
//static void mpu_push_task(void *pvParameters)
//{
//    esp_task_wdt_delete(NULL);
//    TickType_t last_wake = xTaskGetTickCount();
//    const TickType_t period = pdMS_TO_TICKS(50);  /* 20Hz */
//
//    imu_data_t m = {0};
//
//    while (1) {
//        esp_task_wdt_reset();
//        vTaskDelayUntil(&last_wake, period);
//
//        /* 读取本地云台 MPU (静默失败时不上报) */
//        if (imu_read_role(IMU_ROLE_GIMBAL, &m) == ESP_OK) {
//            int16_t ax = (int16_t)(m.ax * 16384.0f);
//            int16_t ay = (int16_t)(m.ay * 16384.0f);
//            int16_t az = (int16_t)(m.az * 16384.0f);
//            int16_t gx = (int16_t)(m.gx * 131.0f);
//            int16_t gy = (int16_t)(m.gy * 131.0f);
//            int16_t gz = (int16_t)(m.gz * 131.0f);
//
//            /* 推上位机 (type=LOCAL) */
//            if (atomic_load(&s_host_connected)) {
//                control_build_mpu_frame(s_mpu_frame, CTRL_MPU_TYPE_LOCAL,
//                                        ax, ay, az, gx, gy, gz);
//                int32_t sent = wiz_send(HOST_SOCK, s_mpu_frame, MPU_FRAME_SIZE);
//                if (sent != MPU_FRAME_SIZE) {
//                    ESP_LOGW("TCP", "MPU 推上位机失败: %d", (int)sent);
//                }
//            }
//            /* 推远端 (type=LOCAL, 远端知道是主节点发的) */
//            push_mpu_to_remote(CTRL_MPU_TYPE_LOCAL, ax, ay, az, gx, gy, gz);
//        }
//    }
//}

///* 状态上报任务: 500ms 周期 JSON, 保留备用 (可选启动) */
//static void status_report_task(void *pvParameters)
//{
//    esp_task_wdt_delete(NULL);
//    TickType_t last_wake = xTaskGetTickCount();
//    const TickType_t period = pdMS_TO_TICKS(500);
//
//    while (1) {
//        esp_task_wdt_reset();
//        vTaskDelayUntil(&last_wake, period);
//
//        if (!atomic_load(&s_host_connected)) continue;
//
//        /* 简化的状态: 实际可由 cJSON 构造, 此处先输出最少信息 */
//        int n = snprintf((char *)s_tx_buffer, sizeof(s_tx_buffer),
//                         "{\"uptime_ms\":%lld,\"link\":true,\"host\":%d,\"remote\":%d,\"frames\":%lu}\n",
//                         (long long)(esp_timer_get_time() / 1000),
//                         atomic_load(&s_host_connected) ? 1 : 0,
//                         atomic_load(&s_remote_connected) ? 1 : 0,
//                         (unsigned long)control_get_frame_count());
//        if (n > 0) {
//            wiz_send(HOST_SOCK, s_tx_buffer, (uint16_t)n);
//        }
//    }
//}

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

/* GPS 1Hz 日志开关 (默认关, 用 gps 1 开启) */
static volatile bool s_gps_log_enabled = false;

/* 船体(亚博) 10 轴 IMU 日志开关 (默认开, 10Hz 输出; 用 hull 0 关闭) */
static volatile bool s_hull_log_enabled = true;

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

/* ========== 测试任务选择 ==========
 * 同一时刻只能有一个"驱动舵机"的测试任务在跑, 否则两条指令流会互相打架.
 *   0 = 无测试任务 (两舵机保持开机回中的中位, 不动作)
 *   1 = 云台运动规划测试 (速度/加速度受限, 用于抑制大惯量载荷的冲击与过冲)
 *   2 = ch1 舵机 <-> MPU 关系测定 (俯仰轴, 用重力矢量夹角)
 *   3 = ch0 舵机 <-> MPU 关系测定 (平面旋转轴, 用融合后的偏航角)
 *   4 = 云台闭环自稳测试 (ch1 俯仰闭环, 开机自动开启并打印误差)
 */
#define TEST_MODE           4

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

/* [临时测试] GPS 1Hz 日志任务 (用 gps 1 打开 / gps 0 关闭) */
static void gps_log_task(void *pvParameters)
{
    (void)pvParameters;
    gps_data_t d;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (!s_gps_log_enabled) {
            continue;
        }
        if (gps_get_data(&d) != ESP_OK) {
            continue;
        }
        uint64_t now = (uint64_t)esp_timer_get_time();
        uint64_t pps_us = gps_get_pps_last_us();
        ESP_LOGI(TAG, "[GPS] %s 卫星=%u HDOP=%.1f 经度=%.6f 纬度=%.6f 速度=%.1fkm/h 航向=%.1f",
                 d.valid ? "定位" : "未定位", (unsigned)d.satellites, d.hdop,
                 d.longitude, d.latitude, d.speed_kmh, d.course_deg);
        if (pps_us != 0) {
            ESP_LOGI(TAG, "[GPS] UTC %02u:%02u:%02u PPS=%lu (最近 %.2fs 前)",
                     (unsigned)d.hour, (unsigned)d.minute, (unsigned)d.second,
                     (unsigned long)gps_get_pps_count(),
                     (double)(now - pps_us) / 1e6);
        }
    }
}

/* [临时测试] 船体(亚博) 10 轴 IMU 日志任务: 默认 10Hz 输出, 用 hull 0 关、hull 1 开.
 * 数据全部来自模块内部解算 (含磁力计补偿), 直接给 roll/pitch/yaw。 */
static void hull_log_task(void *pvParameters)
{
    (void)pvParameters;
    imu_data_t d;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (!s_hull_log_enabled) {
            continue;
        }
        /* 未就绪时静默 (启动日志里已有提示; 插好后输 hull 可在线重试探测) */
        if (!imu_role_ready(IMU_ROLE_HULL)) {
            continue;
        }
        if (imu_read_role(IMU_ROLE_HULL, &d) != ESP_OK) {
            continue;
        }
        ESP_LOGI(TAG, "[HULL] rpy=(%+7.2f %+7.2f %+7.2f)  a=(%+6.3f %+6.3f %+6.3f)g  "
                 "g=(%+7.1f %+7.1f %+7.1f)dps  T=%.1fC",
                 d.roll, d.pitch, d.yaw, d.ax, d.ay, d.az, d.gx, d.gy, d.gz, d.temperature);
    }
}

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
               (unsigned)ch, st.enabled ? "ON" : "off",
               st.target_phys, st.meas_phys, st.err, st.cmd_deg, fb,
               pid.kp, pid.ki, pid.kd);
    }
    return 0;
}

/** gps [0|1] : 查看 GPS 状态; 带参数则开关 1Hz 日志 */
static int cmd_gps(int argc, char **argv)
{
    if (!gps_is_ready()) {
        printf("GPS 未初始化\n");
        return 1;
    }
    if (argc == 2) {
        s_gps_log_enabled = (atoi(argv[1]) != 0);
        printf("GPS 日志: %s\n", s_gps_log_enabled ? "开 (1Hz)" : "关");
        return 0;
    }

    gps_data_t d;
    if (gps_get_data(&d) != ESP_OK) {
        printf("读取 GPS 数据失败\n");
        return 1;
    }
    uint64_t now = (uint64_t)esp_timer_get_time();
    uint64_t pps_us = gps_get_pps_last_us();

    printf("GPS: 定位=%s 卫星=%u HDOP=%.1f 海拔=%.1fm\n",
           d.valid ? "有效" : "未定位", (unsigned)d.satellites, d.hdop, d.altitude_m);
    printf("     UTC %02u:%02u:%02u  %04u-%02u-%02u\n",
           (unsigned)d.hour, (unsigned)d.minute, (unsigned)d.second,
           (unsigned)d.year, (unsigned)d.month, (unsigned)d.day);
    printf("     纬度=%.6f° 经度=%.6f°  速度=%.1fkm/h  航向=%.1f°\n",
           d.latitude, d.longitude, d.speed_kmh, d.course_deg);
    printf("     PPS=%lu 次", (unsigned long)gps_get_pps_count());
    if (pps_us != 0) {
        printf(" (最近 %.2fs 前)", (double)(now - pps_us) / 1e6);
    }
    printf("  语句=%lu 错误=%lu\n",
           (unsigned long)d.sentence_cnt, (unsigned long)d.err_cnt);
    return 0;
}

/** hull [0|1] : 亚博 10 轴 IMU —— 不带参数读一帧(未就绪会先在线重试探测); 带参数开关 10Hz 日志 */
static int cmd_hull(int argc, char **argv)
{
    if (argc == 2) {
        s_hull_log_enabled = (atoi(argv[1]) != 0);
        printf("亚博 IMU 日志: %s\n", s_hull_log_enabled ? "开 (10Hz)" : "关");
        return 0;
    }

    /* 启动时没插上也能救回来: 未就绪就先在线重试一次初始化
     * (会先试 0x50, 不行就扫总线找 10 轴 IMU 并采用它的实际地址) */
    if (!imu_role_ready(IMU_ROLE_HULL)) {
        printf("船体 IMU 未就绪, 正在重新探测...\n");
        imu_config_t icfg = imu_get_default_config();
        esp_err_t r = imu_init_role(IMU_ROLE_HULL, &icfg);
        if (r != ESP_OK || !imu_role_ready(IMU_ROLE_HULL)) {
            printf("仍未找到: %s   (输 scan 1 看这条总线上有哪些器件)\n", esp_err_to_name(r));
            return 1;
        }
        printf("探测成功\n");
    }

    imu_data_t d;
    if (imu_read_role(IMU_ROLE_HULL, &d) != ESP_OK) {
        printf("读取失败\n");
        return 1;
    }
    printf("rpy=(%+7.2f %+7.2f %+7.2f) deg  a=(%+6.3f %+6.3f %+6.3f) g  "
           "g=(%+7.1f %+7.1f %+7.1f) dps  T=%.1f C\n",
           d.roll, d.pitch, d.yaw, d.ax, d.ay, d.az, d.gx, d.gy, d.gz, d.temperature);
    return 0;
}

/** scan [0|1] : 扫描 I2C 总线, 列出所有应答地址并识别型号 (不带参数 = 两条都扫) */
static int cmd_scan(int argc, char **argv)
{
    if (argc == 1) {
        printf("扫描 I2C0 (云台 GPIO16/18)...\n");
        imu_scan_role(IMU_ROLE_GIMBAL);
        printf("扫描 I2C1 (船体 + PCA9685 GPIO21/17)...\n");
        imu_scan_role(IMU_ROLE_HULL);
        return 0;
    }

    int bus = atoi(argv[1]);
    if (bus == 0) {
        imu_scan_role(IMU_ROLE_GIMBAL);
    } else if (bus == 1) {
        imu_scan_role(IMU_ROLE_HULL);
    } else {
        printf("用法: scan [0|1]   0=I2C0(云台 GPIO16/18)  1=I2C1(船体+PCA9685 GPIO21/17)\n");
        return 1;
    }
    return 0;
}

/** imu : 两颗 IMU 各读一帧 (云台 MPU6050 原始 6 轴 / 船体 10 轴 IMU 含内部角度) */
static int cmd_imu(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    for (int r = 0; r < IMU_ROLE_COUNT; r++) {
        imu_role_t  role = (imu_role_t)r;
        const char *name = (role == IMU_ROLE_HULL) ? "hull  " : "gimbal";

        if (!imu_role_ready(role)) {
            printf("%s  未就绪\n", name);
            continue;
        }
        imu_data_t d;
        if (imu_read_role(role, &d) != ESP_OK) {
            printf("%s  读取失败\n", name);
            continue;
        }
        printf("%s  a=(%+7.3f %+7.3f %+7.3f) g  g=(%+8.1f %+8.1f %+8.1f) dps",
               name, d.ax, d.ay, d.az, d.gx, d.gy, d.gz);
        if (role == IMU_ROLE_HULL) {
            printf("  rpy=(%+7.1f %+7.1f %+7.1f) deg", d.roll, d.pitch, d.yaw);
        }
        printf("  T=%.1f C\n", d.temperature);
    }
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

/**
 * @brief 自定义串口 REPL 任务
 *
 * 与 esp_console 自带 REPL 唯一的区别: 纯数字输入直接解释为 "把 ch0 定死在该脉宽",
 * 省去每次改脉宽都要重新编译烧录. 其余输入照常交给 esp_console_run 处理.
 */
static void console_repl_task(void *pvParameters)
{
    linenoiseSetMaxLineLen(128);

    printf("\n");
    printf("直接输入数字 = 把 ch0 定死在该脉宽 (例: 500)\n");
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

        /* 判断是否"纯数字" */
        bool is_number = true;
        for (const char *p = line; *p != '\0'; p++) {
            if (*p < '0' || *p > '9') {
                is_number = false;
                break;
            }
        }

        if (is_number) {
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

/** 初始化并启动串口控制台 */
static void console_init(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt = "gimbal>";
    repl_cfg.max_cmdline_length = 128;

    esp_console_dev_uart_config_t uart_cfg = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_cfg, &repl_cfg, &repl));

    const esp_console_cmd_t cmds[] = {
        { .command = "p",  .help = "直接输出脉宽: p <ch> <us>",              .func = cmd_pulse  },
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
        { .command = "gps",.help = "查看 GPS 状态: gps [0|1] (带参数开关 1Hz 日志)", .func = cmd_gps },
        { .command = "imu",.help = "读取两颗 IMU 各一帧数据",                    .func = cmd_imu },
        { .command = "scan",.help = "扫描 I2C 总线: scan [0|1] (0=I2C0 云台, 1=I2C1 船体)", .func = cmd_scan },
        { .command = "hull",.help = "亚博 10 轴 IMU: hull [0|1] (无参数读一帧, 带参数开关 10Hz 日志)", .func = cmd_hull },
    };

    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }
    ESP_ERROR_CHECK(esp_console_register_help_command());

    /* 注意: 这里刻意不调用 esp_console_start_repl().
     * new_repl_uart 已把 UART0/VFS/linenoise 准备好 (其内部 REPL 任务会一直阻塞
     * 在 ulTaskNotifyTake 上, 不会读串口), 我们改用自建任务, 以便支持"裸数字"输入.
     */
    xTaskCreate(console_repl_task, "console_repl", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "串口控制台已启动: 直接输数字改 ch0 脉宽, 或输 help 看全部指令");
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
    ESP_LOGI(TAG, "[STAB] 云台闭环自稳测试: ch1 俯仰, 目标物理角 %.1f°", STAB_TARGET_DEG);

    /* 等陀螺零偏标定完成 (标定期间不能动云台) */
    vTaskDelay(pdMS_TO_TICKS(AHRS_CAL_READY_MS));

    /* 先回中站稳再开闭环 */
    gimbal_move_to(1, 90.0f);
    vTaskDelay(pdMS_TO_TICKS(STAB_SETTLE_MS));

    gimbal_enable_stabilize(1, true);
    vTaskDelay(pdMS_TO_TICKS(500));     /* 先锁在当前姿态, 再给目标, 避免一开就猛跳 */
    gimbal_set_target_phys(1, STAB_TARGET_DEG);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));

        gimbal_stab_state_t st;
        gimbal_pid_t pid;
        gimbal_get_stab_state(1, &st);
        gimbal_get_pid(1, &pid);
        ESP_LOGI(TAG, "[STAB] 目标=%+6.1f° 实测=%+6.1f° 误差=%+6.1f° 输出标称角=%6.1f° (kp=%.2f ki=%.2f)",
                 st.target_phys, st.meas_phys, st.err, st.cmd_deg, pid.kp, pid.ki);
    }
}

#endif /* TEST_MODE == 4 */

/**
 * @brief 俯仰角闭环控制任务
 *
 * 使用 Madgwick AHRS 融合加速度 + 陀螺仪, 得到 yaw/pitch/roll,
 * 用 pitch 控制 PCA9685 通道 1 舵机.
 */
static void pitch_control_task(void *pvParameters)
{
    ESP_LOGI(TAG, "[PITCH] 俯仰角控制任务启动 (ch%d, Madgwick β=%.2f)", PITCH_SERVO_CH, AHRS_BETA);

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);  /* 50Hz 控制频率 */

    imu_data_t m = {0};
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  /* 初始四元数 */
    float ypr[3] = {0};
    uint32_t log_counter = 0;

    while (1) {
        vTaskDelayUntil(&last_wake, period);
        float dt = 0.02f;  /* 固定步长 20ms */

        if (imu_read_role(IMU_ROLE_GIMBAL, &m) != ESP_OK) {
            ESP_LOGW(TAG, "[MPU] 读取失败");
            continue;
        }

        /* 陀螺仪 °/s → rad/s */
        float gx_rad = m.gx * (M_PI / 180.0f);
        float gy_rad = m.gy * (M_PI / 180.0f);
        float gz_rad = m.gz * (M_PI / 180.0f);

        /* Madgwick AHRS 融合 */
        madgwick_update(m.ax, m.ay, m.az, gx_rad, gy_rad, gz_rad, q, AHRS_BETA, dt);
        quaternion_to_ypr(q, ypr);

        /* pitch 弧度 → 度 */
        float pitch_deg = ypr[1] * (180.0f / M_PI);

        /* 映射俯仰角 → 舵机角度 */
        float servo_deg;
        if (pitch_deg <= PITCH_RANGE_MIN) {
            servo_deg = PITCH_SERVO_MIN_DEG;
        } else if (pitch_deg >= PITCH_RANGE_MAX) {
            servo_deg = PITCH_SERVO_MAX_DEG;
        } else {
            servo_deg = PITCH_SERVO_MIN_DEG +
                (pitch_deg - PITCH_RANGE_MIN) /
                (PITCH_RANGE_MAX - PITCH_RANGE_MIN) *
                (PITCH_SERVO_MAX_DEG - PITCH_SERVO_MIN_DEG);
        }

        /* 驱动舵机 */
        if (servo_is_ready()) {
            servo_set_angle(PITCH_SERVO_CH, servo_deg);
        }

        /* 定期打印 log */
        log_counter++;
        if (log_counter >= PITCH_LOG_INTERVAL) {
            log_counter = 0;
            float yaw_deg   = ypr[0] * (180.0f / M_PI);
            float roll_deg  = ypr[2] * (180.0f / M_PI);
            ESP_LOGI(TAG, "[AHRS] yaw=%.1f° pitch=%.1f° roll=%.1f°  servo=%.1f°",
                     yaw_deg, pitch_deg, roll_deg, servo_deg);
        }
    }
}

/* ========== 启动流程 ========== */

static void start_components(void)
{
    ESP_LOGI(TAG, "=== [临时测试模式] MPU6050 + PCA9685 ===");
    ESP_LOGI(TAG, "start_components 开始执行");

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

    /* 2. W5500 (必须在控制 task 之前初始化) */
    //wiznet_manager_config_t wcfg = wiznet_manager_get_default_config();
    ///* 覆盖默认 IP 为项目约定值 192.168.29.10 (与远端节点 .11 同 /24 网段,
    // * 远端 TCP Client connect 192.168.29.10:8081).
    // * 默认 config 返回 192.168.1.100, 与本项目网段不符, 必须在 main.c 显式覆盖. */
    //wcfg.ip[0]      = 192; wcfg.ip[1]      = 168; wcfg.ip[2]      = 29; wcfg.ip[3]      = 10;
    //wcfg.gateway[0] = 192; wcfg.gateway[1] = 168; wcfg.gateway[2] = 29; wcfg.gateway[3] = 1;
    ///* DNS 沿用默认 8.8.8.8 */
    ///* 8 socket 缓冲 (RX+TX 各 2KB, 共 32KB) - W5500 内部 16KB/32KB/48KB 选型 */
    ///* 注: 默认配置已设为 2KB per socket, 8 个 socket 共用 32KB W5500 SRAM */
    //ESP_ERROR_CHECK(wiznet_manager_init(&wcfg));

    /* 3. 云台姿态传感器 - I2C0: SDA=GPIO16, SCL=GPIO18
     *    云台 MPU6050: 地址 0x68 (AD0 接 GND), 云台闭环的反馈源
     *    注: 船体 10 轴 IMU 挂在另一条总线 I2C1 上 (见下面 4c), 两颗 IMU 互不干扰 */
    ESP_LOGI(TAG, "初始化云台 MPU6050 (0x68)...");
    imu_config_t icfg = imu_get_default_config();
    esp_err_t imu_ret = imu_init_role(IMU_ROLE_GIMBAL, &icfg);
    if (imu_ret == ESP_OK && imu_role_ready(IMU_ROLE_GIMBAL)) {
        ESP_LOGI(TAG, "云台 MPU6050 初始化成功");
    } else {
        ESP_LOGE(TAG, "云台 MPU6050 初始化失败: %s", esp_err_to_name(imu_ret));
    }

    /* 3b. GPS - UART1: ESP32 TX=GPIO2 -> GPS RX, ESP32 RX=GPIO1 <- GPS TX,
     *     PPS 秒脉冲输入 = GPIO3 (可选, 不接也能解析 NMEA) */
    ESP_LOGI(TAG, "初始化 GPS (UART1: TX=IO%d, RX=IO%d, PPS=IO%d)...",
             GPS_TX_GPIO, GPS_RX_GPIO, GPS_PPS_GPIO);
    gps_config_t gcfg = gps_get_default_config();
    gcfg.tx_gpio  = GPS_TX_GPIO;
    gcfg.rx_gpio  = GPS_RX_GPIO;
    gcfg.pps_gpio = GPS_PPS_GPIO;
    esp_err_t gps_ret = gps_init(&gcfg);
    if (gps_ret == ESP_OK) {
        ESP_LOGI(TAG, "GPS 初始化成功");
    } else {
        /* GPS 未接入属正常, 只提示不报错 */
        ESP_LOGW(TAG, "GPS 未接入: %s", esp_err_to_name(gps_ret));
    }

    /* 4. Servo (PCA9685) - I2C1: SDA=GPIO21, SCL=GPIO17 */
    ESP_LOGI(TAG, "初始化 PCA9685...");
    servo_config_t scfg = servo_get_default_config();
    esp_err_t servo_ret = servo_init(&scfg);
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
    } else {
        ESP_LOGE(TAG, "PCA9685 初始化失败: %s", esp_err_to_name(servo_ret));
    }

    /* 4b. 云台运动规划 (依赖 servo, 必须在舵机初始化之后)
     *     把"阶跃指令"变成速度/加速度受限的轨迹, 抑制大惯量载荷的冲击与过冲. */
    if (servo_ret == ESP_OK) {
        if (gimbal_init() == ESP_OK) {
            ESP_LOGI(TAG, "云台运动规划初始化成功");
        } else {
            ESP_LOGE(TAG, "云台运动规划初始化失败");
        }
    }

    /* 4c. 船体 10 轴 IMU - 亚博惯导模块, 挂在 PCA9685 那条 I2C1 上
     *     (SDA=GPIO21, SCL=GPIO17, 地址 0x50, WIT I2C 协议)
     *     与云台 MPU6050 (I2C0) 物理分开, 两颗 IMU 互不干扰, 都不用软件模拟 I2C。
     *     选 I2C 而非 UART: GPS 已独占 UART1, 且模块本身支持寄存器式 I2C 读欧拉角。
     *     ⚠️ 必须排在 servo_init() 之后: I2C1 是 servo 组件装的, imu 只是借用
     *        (重复 i2c_driver_install 会失败); 若 PCA9685 探测失败, servo_init 会把
     *        该总线删掉, 此时船体 IMU 也会一起读不到。 */
    ESP_LOGI(TAG, "初始化船体 10 轴 IMU (I2C1, 0x50)...");
    esp_err_t imu_hull_ret = imu_init_role(IMU_ROLE_HULL, &icfg);
    if (imu_hull_ret == ESP_OK && imu_role_ready(IMU_ROLE_HULL)) {
        ESP_LOGI(TAG, "船体 10 轴 IMU 初始化成功");
    } else {
        /* 船体 IMU 还没接上属正常, 只提示不报错 —— 云台功能不受影响 */
        ESP_LOGW(TAG, "船体 10 轴 IMU 未接入: %s", esp_err_to_name(imu_hull_ret));
    }

    /* 5. Motor (4 路 ESC + L298N) */
    //motor_config_t mcfg = motor_get_default_config();
    //ESP_ERROR_CHECK(motor_init(&mcfg));

    /* 6. 等待 link up (30s 超时) */
    //ESP_LOGI(TAG, "等待网线连接...");
    //int wait_ms = 0;
    //while (!wiznet_manager_is_link_up() && wait_ms < 30000) {
    //    vTaskDelay(pdMS_TO_TICKS(500));
    //    wait_ms += 500;
    //}
    //if (wiznet_manager_is_link_up()) {
    //    ESP_LOGI(TAG, "=== 网线已连接 ===");
    //    esp_netif_ip_info_t info;
    //    if (wiznet_manager_get_ip_info(&info) == ESP_OK) {
    //        ESP_LOGI(TAG, "本机 IP: %d.%d.%d.%d",
    //                 info.ip.addr & 0xFF, (info.ip.addr >> 8) & 0xFF,
    //                 (info.ip.addr >> 16) & 0xFF, (info.ip.addr >> 24) & 0xFF);
    //        ESP_LOGI(TAG, "子网掩码: %d.%d.%d.%d",
    //                 info.netmask.addr & 0xFF, (info.netmask.addr >> 8) & 0xFF,
    //                 (info.netmask.addr >> 16) & 0xFF, (info.netmask.addr >> 24) & 0xFF);
    //        ESP_LOGI(TAG, "默认网关: %d.%d.%d.%d",
    //                 info.gw.addr & 0xFF, (info.gw.addr >> 8) & 0xFF,
    //                 (info.gw.addr >> 16) & 0xFF, (info.gw.addr >> 24) & 0xFF);
    //    }
    //} else {
    //    ESP_LOGW(TAG, "=== 网线未连接 (30s 超时) ===");
    //}

    /* 7. 日志降噪 (默认开 INFO, 但关闭驱动内不必要 tag) */
    //esp_log_level_set("wifi",       ESP_LOG_WARN);
    //esp_log_level_set("spi_master", ESP_LOG_WARN);
    //esp_log_level_set("gpio",       ESP_LOG_WARN);

    /* 8. 创建任务 */
    //xTaskCreate(tcp_server_task,    "tcp_srv", 8192, NULL, 5, NULL);
    //xTaskCreate(mpu_push_task,      "mpu",     2048, NULL, 4, &s_mpu_push_task_handle);
    /* status_report_task 暂时不启动 (MPU 推送已涵盖大部分状态) */
    /* xTaskCreate(status_report_task, "status",  2048, NULL, 4, &s_status_report_task_handle); */

    /* [临时测试] 启动 MPU 三维姿态解算任务 */
    xTaskCreate(attitude_task, "attitude", 4096, NULL, 5, NULL);

    /* [临时测试] 启动 GPS 1Hz 日志任务 (默认静默, 用 gps 1 打开) */
    if (gps_ret == ESP_OK) {
        xTaskCreate(gps_log_task, "gps_log", 4096, NULL, 4, NULL);
    }

    /* [临时测试] 启动船体(亚博) 10 轴 IMU 日志任务 (默认 10Hz 输出, 用 hull 0 关) */
    xTaskCreate(hull_log_task, "hull_log", 4096, NULL, 4, NULL);

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

    ESP_LOGI(TAG, "=== [临时测试模式] 启动完成: MPU6050 + PCA9685 ===");
}

void app_main(void)
{
    ESP_LOGI(TAG, "app_main 入口 - 波特率 115200");
    start_components();
}
