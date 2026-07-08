/**
 * @file rdk_uart.c
 * @brief ESP32-S3 ↔ RDK X5 UART 通信实现
 *
 * 状态机 (与 control 组件类似, 字节流扫描):
 *   IDLE → GOT_HEAD0 → GOT_HEAD1 → GOT_TYPE → GOT_LEN → LOADING_PAYLOAD → CRC_CHECK
 *
 * 任务设计:
 *   - rdk_uart_rx_task: 优先级 4, 栈 4096, 阻塞读 UART
 *   - 收到完整帧后处理: PING → PONG, GET_MPU → 通知 mpu_push_task 立即发
 */

#include "rdk_uart.h"
#include "control.h"
#include "imu.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>

static const char *TAG = "rdk_uart";

/* 硬件引脚 (ESP32-S3-DevKitC-1 UART0 专用引脚) */
#define RDK_UART_TX_GPIO      43      /* U0TXD */
#define RDK_UART_RX_GPIO      44      /* U0RXD */
#define RDK_UART_NUM          UART_NUM_0
#define RDK_UART_BUF_SIZE     1024
#define RDK_UART_RX_TASK_PRIO 4
#define RDK_UART_RX_TASK_STACK 4096

/* 统计 */
static volatile uint32_t s_rx_count = 0;
static volatile uint32_t s_tx_count = 0;

/* 队列: 用于通知 mpu_push_task 立即发送 MPU */
static QueueHandle_t s_mpu_request_queue = NULL;
#define RDK_MPU_REQUEST_QUEUE_LEN 4

/* 解析状态 */
typedef enum {
    RDK_STATE_IDLE = 0,
    RDK_STATE_GOT_HEAD0,
    RDK_STATE_GOT_HEAD1,
    RDK_STATE_GOT_TYPE,
    RDK_STATE_GOT_LEN,
    RDK_STATE_LOADING_PAYLOAD,
} rdk_state_t;

static uint8_t calc_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}

/* 发送一帧: 0xCC 0x77 + type + len + payload + crc */
static esp_err_t send_frame(rdk_frame_type_t type, const uint8_t *payload, uint8_t payload_len)
{
    if (payload_len > RDK_UART_MAX_PAYLOAD) return ESP_ERR_INVALID_ARG;

    uint8_t buf[RDK_UART_MAX_PAYLOAD + 5];
    size_t  pos = 0;
    buf[pos++] = RDK_UART_HEAD_0;
    buf[pos++] = RDK_UART_HEAD_1;
    buf[pos++] = (uint8_t)type;
    buf[pos++] = payload_len;
    if (payload && payload_len > 0) {
        memcpy(&buf[pos], payload, payload_len);
        pos += payload_len;
    }
    uint8_t crc = calc_crc8(buf, pos);
    buf[pos] = crc;
    pos += 1;

    int n = uart_write_bytes(RDK_UART_NUM, buf, pos);
    if (n == (int)pos) {
        s_tx_count++;
        return ESP_OK;
    }
    return ESP_FAIL;
}

/* 处理收到的完整帧 */
static void handle_frame(rdk_frame_type_t type, const uint8_t *payload, uint8_t len)
{
    switch (type) {
    case RDK_TYPE_PING:
        /* 应答 PONG */
        send_frame(RDK_TYPE_PONG, NULL, 0);
        ESP_LOGI(TAG, "PING → PONG");
        break;

    case RDK_TYPE_GET_MPU:
        /* 通知 mpu_push_task 立即读 MPU 发给 RDK */
        if (s_mpu_request_queue) {
            uint8_t dummy = 1;
            xQueueSend(s_mpu_request_queue, &dummy, 0);
        }
        break;

    case RDK_TYPE_GET_STATUS:
        /* 简化: 发送基本状态 (JSON) */
        send_frame(RDK_TYPE_STATUS, (const uint8_t *)"{\"uptime\":0}", 13);
        break;

    case RDK_TYPE_AI_RESULT:
        /* RDK X5 主动发来的 AI 识别结果 - 当前忽略 (后续扩展) */
        ESP_LOGI(TAG, "AI_RESULT (len=%d) - ignored (not implemented)", len);
        break;

    default:
        ESP_LOGW(TAG, "Unknown type 0x%02X (len=%d)", (uint8_t)type, len);
        break;
    }
}

/* RX 任务: 字节流扫描 + 状态机 */
static void rdk_uart_rx_task(void *pvParameters)
{
    /* 退订 Task WDT (UART 阻塞读也可能被卡) */
    esp_task_wdt_delete(NULL);

    uint8_t  byte;
    rdk_state_t state = RDK_STATE_IDLE;
    uint8_t  ftype   = 0;
    uint8_t  flen    = 0;
    uint8_t  buf[RDK_UART_MAX_PAYLOAD];
    uint8_t  buf_pos = 0;

    while (1) {
        esp_task_wdt_reset();

        /* 阻塞读 1 字节 (10ms 超时) */
        int n = uart_read_bytes(RDK_UART_NUM, &byte, 1, pdMS_TO_TICKS(10));
        if (n <= 0) continue;
        s_rx_count++;

        switch (state) {
        case RDK_STATE_IDLE:
            if (byte == RDK_UART_HEAD_0) {
                state = RDK_STATE_GOT_HEAD0;
            }
            break;

        case RDK_STATE_GOT_HEAD0:
            if (byte == RDK_UART_HEAD_1) {
                state = RDK_STATE_GOT_HEAD1;
            } else if (byte == RDK_UART_HEAD_0) {
                /* 0xCC 0xCC, 重新开始 */
            } else {
                state = RDK_STATE_IDLE;
            }
            break;

        case RDK_STATE_GOT_HEAD1:
            ftype = byte;
            state = RDK_STATE_GOT_TYPE;
            break;

        case RDK_STATE_GOT_TYPE:
            flen = byte;
            if (flen > RDK_UART_MAX_PAYLOAD) {
                ESP_LOGW(TAG, "Bad len %d, drop frame", flen);
                state = RDK_STATE_IDLE;
            } else if (flen == 0) {
                state = RDK_STATE_GOT_LEN;  /* 直接进 CRC */
            } else {
                buf_pos = 0;
                state = RDK_STATE_LOADING_PAYLOAD;
            }
            break;

        case RDK_STATE_LOADING_PAYLOAD:
            buf[buf_pos++] = byte;
            if (buf_pos >= flen) {
                state = RDK_STATE_GOT_LEN;
            }
            break;

        case RDK_STATE_GOT_LEN:
            {
                /* 校验 CRC (前 4+flen 字节: head0, head1, type, len, payload) */
                uint8_t frame[4 + RDK_UART_MAX_PAYLOAD];
                size_t  fl = 0;
                frame[fl++] = RDK_UART_HEAD_0;
                frame[fl++] = RDK_UART_HEAD_1;
                frame[fl++] = ftype;
                frame[fl++] = flen;
                if (flen > 0) {
                    memcpy(&frame[fl], buf, flen);
                    fl += flen;
                }
                uint8_t expect_crc = calc_crc8(frame, fl);
                if (byte == expect_crc) {
                    handle_frame((rdk_frame_type_t)ftype, buf, flen);
                } else {
                    ESP_LOGW(TAG, "CRC fail: got 0x%02X expect 0x%02X", byte, expect_crc);
                }
                state = RDK_STATE_IDLE;
            }
            break;
        }
    }
}

esp_err_t rdk_uart_init(void)
{
    /* 队列: mpu_push_task 等待 RDK 请求 */
    s_mpu_request_queue = xQueueCreate(RDK_MPU_REQUEST_QUEUE_LEN, sizeof(uint8_t));
    if (s_mpu_request_queue == NULL) {
        ESP_LOGE(TAG, "Queue create failed");
        return ESP_FAIL;
    }

    uart_config_t cfg = {
        .baud_rate  = RDK_UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(RDK_UART_NUM, RDK_UART_BUF_SIZE, RDK_UART_BUF_SIZE, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = uart_param_config(RDK_UART_NUM, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = uart_set_pin(RDK_UART_NUM, RDK_UART_TX_GPIO, RDK_UART_RX_GPIO, -1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 启动 RX 任务 */
    xTaskCreate(rdk_uart_rx_task, "rdk_rx", RDK_UART_RX_TASK_STACK, NULL,
                RDK_UART_RX_TASK_PRIO, NULL);

    ESP_LOGI(TAG, "RDK UART ready: TX=%d RX=%d @%d baud", RDK_UART_TX_GPIO, RDK_UART_RX_GPIO, RDK_UART_BAUD_RATE);
    return ESP_OK;
}

esp_err_t rdk_uart_send_ping(void)
{
    return send_frame(RDK_TYPE_PING, NULL, 0);
}

esp_err_t rdk_uart_send_mpu(uint8_t type,
                            int16_t ax, int16_t ay, int16_t az,
                            int16_t gx, int16_t gy, int16_t gz)
{
    /* 复用 TCP MPU 帧格式: 0xBB 0x66 + type + 6×int16 LE + CRC8
     * 这里将 MPU 帧嵌入 RDK 帧的 payload 中 (兼容 RDK 端) */
    uint8_t payload[16];
    payload[0]  = 0xBB;   /* 复用 TCP MPU 帧头 */
    payload[1]  = 0x66;
    payload[2]  = type;
    payload[3]  = (uint8_t)(ax & 0xFF);
    payload[4]  = (uint8_t)((ax >> 8) & 0xFF);
    payload[5]  = (uint8_t)(ay & 0xFF);
    payload[6]  = (uint8_t)((ay >> 8) & 0xFF);
    payload[7]  = (uint8_t)(az & 0xFF);
    payload[8]  = (uint8_t)((az >> 8) & 0xFF);
    payload[9]  = (uint8_t)(gx & 0xFF);
    payload[10] = (uint8_t)((gx >> 8) & 0xFF);
    payload[11] = (uint8_t)(gy & 0xFF);
    payload[12] = (uint8_t)((gy >> 8) & 0xFF);
    payload[13] = (uint8_t)(gz & 0xFF);
    payload[14] = (uint8_t)((gz >> 8) & 0xFF);
    payload[15] = calc_crc8(payload, 15);

    return send_frame(RDK_TYPE_MPU_FRAME, payload, 16);
}

esp_err_t rdk_uart_send_status_json(const char *json)
{
    if (json == NULL) return ESP_ERR_INVALID_ARG;
    size_t len = strlen(json);
    if (len > RDK_UART_MAX_PAYLOAD) return ESP_ERR_INVALID_ARG;
    return send_frame(RDK_TYPE_STATUS, (const uint8_t *)json, (uint8_t)len);
}

uint32_t rdk_uart_get_rx_count(void) { return s_rx_count; }
uint32_t rdk_uart_get_tx_count(void) { return s_tx_count; }

/* 提供给 mpu_push_task 的接口: 等待 RDK 请求 (立即发 MPU) */
QueueHandle_t rdk_uart_get_mpu_request_queue(void)
{
    return s_mpu_request_queue;
}
