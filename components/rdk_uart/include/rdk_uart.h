/**
 * @file rdk_uart.h
 * @brief ESP32-S3 ↔ RDK X5 UART 通信组件
 *
 * 硬件: UART_NUM_0, GPIO43(TX), GPIO44(RX), 115200 8N1
 *       (注: console 已切换到 USB-Serial/JTAG, GPIO43/44 释放)
 *
 * 协议: 自定义二进制 (区别于 TCP 帧头 0xAA/0xBB)
 *   ┌─────────────────────────────────────────────────────────────┐
 *   │ 帧头 0xCC 0x77                                                │
 *   │   [0]  0xCC         帧头 1                                    │
 *   │   [1]  0x77         帧头 2                                    │
 *   │   [2]  type         类型 (见下表)                              │
 *   │   [3]  len          负载长度 (0~200)                          │
 *   │   [4..4+len-1]      payload (变长)                             │
 *   │   [4+len]           CRC8 (前 N 字节异或)                      │
 *   └─────────────────────────────────────────────────────────────┘
 *
 * 类型表 (RDK X5 主动, ESP32 应答):
 *   0x01 PING         (RDK→ESP)  心跳查询
 *   0x02 GET_MPU      (RDK→ESP)  请求 MPU 原始数据
 *   0x03 GET_STATUS   (RDK→ESP)  请求系统状态
 *   0x10 AI_RESULT    (RDK→ESP)  AI 识别结果 (reserved, 后续扩展)
 *   0x80 PONG         (ESP→RDK)  心跳应答
 *   0x81 MPU_FRAME    (ESP→RDK)  16B MPU 原始数据 (复用 TCP MPU 帧格式)
 *   0x82 STATUS       (ESP→RDK)  系统状态 (JSON 文本)
 *
 * 当前实现 (为后续扩展性预留):
 *   - RDK X5 主动发起, ESP32 应答
 *   - 心跳: RDK PING → ESP32 PONG (1s 周期可选)
 *   - 状态查询: RDK GET_MPU → ESP32 MPU_FRAME
 *   - 主动推送: 可选 ESP32 周期性推 MPU (20Hz)
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 帧常量 */
#define RDK_UART_HEAD_0           0xCC
#define RDK_UART_HEAD_1           0x77
#define RDK_UART_MAX_PAYLOAD      200
#define RDK_UART_BAUD_RATE        115200

/* 帧类型 */
typedef enum {
    RDK_TYPE_PING         = 0x01,   /* 心跳查询 */
    RDK_TYPE_GET_MPU      = 0x02,   /* 请求 MPU */
    RDK_TYPE_GET_STATUS   = 0x03,   /* 请求状态 */
    RDK_TYPE_AI_RESULT    = 0x10,   /* AI 识别结果 (RDK→ESP, 预留) */
    RDK_TYPE_PONG         = 0x80,   /* 心跳应答 */
    RDK_TYPE_MPU_FRAME    = 0x81,   /* MPU 原始数据 (16B, 复用 TCP 帧格式) */
    RDK_TYPE_STATUS       = 0x82,   /* 系统状态 (JSON 文本) */
} rdk_frame_type_t;

/* 初始化 (启动 UART 驱动 + RX 任务) */
esp_err_t rdk_uart_init(void);

/* 主动发送 PING (测试用) */
esp_err_t rdk_uart_send_ping(void);

/* 主动发送 MPU 帧 (16B, 与 TCP MPU 帧格式相同) */
esp_err_t rdk_uart_send_mpu(uint8_t type,
                            int16_t ax, int16_t ay, int16_t az,
                            int16_t gx, int16_t gy, int16_t gz);

/* 主动发送状态 (JSON 文本) */
esp_err_t rdk_uart_send_status_json(const char *json);

/* 统计 */
uint32_t rdk_uart_get_rx_count(void);
uint32_t rdk_uart_get_tx_count(void);

/* 内部使用: mpu_push_task 通过此队列监听 RDK 请求 */
QueueHandle_t rdk_uart_get_mpu_request_queue(void);

#ifdef __cplusplus
}
#endif
