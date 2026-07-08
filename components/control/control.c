/**
 * @file control.c
 * @brief 控制协议解析实现 (双帧类型状态机)
 *
 * 状态机 (字节流扫描):
 *   IDLE      等待任一帧头第一字节 (0xAA 或 0xBB)
 *   GOT_H0    已收到第一字节, 等待第二字节
 *   LOADING   已识别帧类型, 累积后续字节
 *
 * 设计要点:
 *   - 16B 控制帧 (0xAA 0x55) 立即执行: 本地电机 + 转发回调
 *   - 16B MPU 帧 (0xBB 0x66) 立即转发到另一个 socket
 *   - 8B 转发子帧 (主→远端) 由 tcp_server 任务构造, 不在解析器内构造
 */

#include "control.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "motor.h"
#include "servo.h"

static const char *TAG = "control";

/* 解析器状态 */
typedef enum {
    CTRL_STATE_IDLE = 0,
    CTRL_STATE_GOT_HEAD0,        /* 已收到第一字节 */
    CTRL_STATE_LOADING_CTRL,     /* 累积 16B 控制帧 (AA 55) */
    CTRL_STATE_LOADING_FWD,      /* 累积 8B 转发子帧 (AA 55, 远端回传? 暂不用) */
    CTRL_STATE_LOADING_MPU,      /* 累积 16B MPU 帧 (BB 66) */
} ctrl_state_t;

static ctrl_state_t      s_state       = CTRL_STATE_IDLE;
static uint8_t           s_buf[CTRL_CTRL_FRAME_SIZE];   /* 复用为最长帧 = 16B */
static size_t            s_buf_len     = 0;
static uint32_t          s_frame_count = 0;
static ctrl_forward_cb_t s_forward_cb  = NULL;   /* 主→远端 转发回调 */

/* 解析后用户层处理: 提取 ctrl_command_t 并执行 */
static void handle_ctrl_command(const uint8_t *frame)
{
    ctrl_command_t cmd = {
        .cmd          = frame[2],
        .local_esc    = { (int8_t)frame[3], (int8_t)frame[4] },
        .remote_esc   = { (int8_t)frame[5], (int8_t)frame[6] },
        .dc_packed    = frame[7],
        .servo_angle  = { frame[8], frame[9] },
        .flags        = frame[10],
    };

    switch (cmd.cmd) {
    case CTRL_CMD_MOTOR:
        /* bit0: 本地执行 */
        if (cmd.flags & 0x01) {
            motor_set_esc_throttle(MOTOR_ESC_1, (float)cmd.local_esc[0]);
            motor_set_esc_throttle(MOTOR_ESC_2, (float)cmd.local_esc[1]);
            uint8_t dc_speed = (cmd.dc_packed >> 4) & 0x0F;
            uint8_t dc_dir   = cmd.dc_packed & 0x03;
            /* dc_speed 0~15 -> 0~100% */
            motor_set_dc_speed(MOTOR_MAIN_DC, (float)(dc_speed * 100 / 15), (motor_dir_t)dc_dir);
            servo_set_angle(0, cmd.servo_angle[0]);
            servo_set_angle(1, cmd.servo_angle[1]);
        }
        /* bit1: 转发到远端 (回调内完成, 由 tcp_server 任务发 8B 子帧) */
        if ((cmd.flags & 0x02) && s_forward_cb) {
            s_forward_cb(&cmd);
        }
        ESP_LOGD(TAG, "CTRL cmd=0x%02X local=[%d,%d] remote=[%d,%d] dc=0x%02X flags=0x%02X",
                 cmd.cmd, cmd.local_esc[0], cmd.local_esc[1],
                 cmd.remote_esc[0], cmd.remote_esc[1], cmd.dc_packed, cmd.flags);
        break;

    case CTRL_CMD_STOP:
        motor_emergency_stop();
        if (s_forward_cb) {
            /* 转发急停 (构造 cmd=0x20 + 远端油门=0) 由回调处理 */
            s_forward_cb(&cmd);
        }
        ESP_LOGW(TAG, "EMERGENCY STOP");
        break;

    case CTRL_CMD_REBOOT:
        ESP_LOGW(TAG, "REBOOT in 500ms");
        motor_emergency_stop();
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
        break;

    case CTRL_CMD_SHUTDOWN:
        ESP_LOGW(TAG, "DEEP SLEEP in 500ms");
        motor_emergency_stop();
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_deep_sleep_start();
        break;

    default:
        ESP_LOGW(TAG, "Unknown cmd 0x%02X", cmd.cmd);
        break;
    }
}

/* 解析后用户层处理: MPU 帧转发到上位机 */
extern void tcp_server_forward_mpu_to_host(const ctrl_mpu_data_t *m);

static void handle_mpu_frame(const uint8_t *frame)
{
    ctrl_mpu_data_t m = {
        .type = frame[2],
        .ax   = (int16_t)(frame[3]  | (frame[4]  << 8)),
        .ay   = (int16_t)(frame[5]  | (frame[6]  << 8)),
        .az   = (int16_t)(frame[7]  | (frame[8]  << 8)),
        .gx   = (int16_t)(frame[9]  | (frame[10] << 8)),
        .gy   = (int16_t)(frame[11] | (frame[12] << 8)),
        .gz   = (int16_t)(frame[13] | (frame[14] << 8)),
    };
    /* 转发 MPU 数据到上位机 (经过 tcp_server_task 持有的 socket 0) */
    tcp_server_forward_mpu_to_host(&m);
}

static uint8_t calc_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}

size_t control_process(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) return 0;

    size_t frames_done = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];

        switch (s_state) {
        case CTRL_STATE_IDLE:
            if (b == CTRL_CTRL_HEAD_0) {
                s_buf[0] = b;
                s_buf_len = 1;
                s_state = CTRL_STATE_GOT_HEAD0;
                /* 标记为控制帧 */
                /* 用一个隐藏的标志? 这里简化: 进入 GOT_HEAD0 后看第二字节区分 */
            } else if (b == CTRL_MPU_HEAD_0) {
                s_buf[0] = b;
                s_buf_len = 1;
                s_state = CTRL_STATE_GOT_HEAD0;
            }
            /* 错位字节直接丢弃 */
            break;

        case CTRL_STATE_GOT_HEAD0:
            if (s_buf[0] == CTRL_CTRL_HEAD_0) {
                if (b == CTRL_CTRL_HEAD_1) {
                    s_buf[1] = b;
                    s_buf_len = 2;
                    s_state = CTRL_STATE_LOADING_CTRL;
                } else if (b == CTRL_CTRL_HEAD_0) {
                    /* 0xAA 0xXX 0xAA, 重新开始 (避免漏检) */
                    s_buf[0] = b;
                    s_buf_len = 1;
                } else {
                    s_state = CTRL_STATE_IDLE;
                    s_buf_len = 0;
                }
            } else { /* s_buf[0] == CTRL_MPU_HEAD_0 */
                if (b == CTRL_MPU_HEAD_1) {
                    s_buf[1] = b;
                    s_buf_len = 2;
                    s_state = CTRL_STATE_LOADING_MPU;
                } else if (b == CTRL_MPU_HEAD_0) {
                    s_buf[0] = b;
                    s_buf_len = 1;
                } else {
                    s_state = CTRL_STATE_IDLE;
                    s_buf_len = 0;
                }
            }
            break;

        case CTRL_STATE_LOADING_CTRL:
            s_buf[s_buf_len++] = b;
            if (s_buf_len >= CTRL_CTRL_FRAME_SIZE) {
                uint8_t expect_crc = calc_crc8(s_buf, CTRL_CTRL_FRAME_SIZE - 1);
                if (s_buf[CTRL_CTRL_FRAME_SIZE - 1] == expect_crc) {
                    handle_ctrl_command(s_buf);
                    frames_done++;
                    s_frame_count++;
                } else {
                    ESP_LOGW(TAG, "CTRL CRC fail: got 0x%02X expect 0x%02X",
                             s_buf[CTRL_CTRL_FRAME_SIZE - 1], expect_crc);
                }
                s_state = CTRL_STATE_IDLE;
                s_buf_len = 0;
            }
            break;

        case CTRL_STATE_LOADING_MPU:
            s_buf[s_buf_len++] = b;
            if (s_buf_len >= CTRL_MPU_FRAME_SIZE) {
                uint8_t expect_crc = calc_crc8(s_buf, CTRL_MPU_FRAME_SIZE - 1);
                if (s_buf[CTRL_MPU_FRAME_SIZE - 1] == expect_crc) {
                    handle_mpu_frame(s_buf);
                    frames_done++;
                    s_frame_count++;
                } else {
                    ESP_LOGW(TAG, "MPU CRC fail: got 0x%02X expect 0x%02X",
                             s_buf[CTRL_MPU_FRAME_SIZE - 1], expect_crc);
                }
                s_state = CTRL_STATE_IDLE;
                s_buf_len = 0;
            }
            break;

        case CTRL_STATE_LOADING_FWD:
            /* 远端不会发 FWD 帧 (那是主→远端的方向), 暂不支持 */
            s_state = CTRL_STATE_IDLE;
            s_buf_len = 0;
            break;
        }
    }
    return frames_done;
}

void control_set_forward_callback(ctrl_forward_cb_t cb)
{
    s_forward_cb = cb;
}

void control_reset(void)
{
    s_state = CTRL_STATE_IDLE;
    s_buf_len = 0;
    memset(s_buf, 0, sizeof(s_buf));
}

uint32_t control_get_frame_count(void)
{
    return s_frame_count;
}

void control_build_ctrl_frame(uint8_t *frame,
                              uint8_t cmd,
                              int8_t local_esc1, int8_t local_esc2,
                              int8_t remote_esc1, int8_t remote_esc2,
                              uint8_t dc_packed,
                              uint8_t servo0, uint8_t servo1,
                              uint8_t flags)
{
    frame[0] = CTRL_CTRL_HEAD_0;
    frame[1] = CTRL_CTRL_HEAD_1;
    frame[2] = cmd;
    frame[3] = (uint8_t)local_esc1;
    frame[4] = (uint8_t)local_esc2;
    frame[5] = (uint8_t)remote_esc1;
    frame[6] = (uint8_t)remote_esc2;
    frame[7] = dc_packed;
    frame[8] = servo0;
    frame[9] = servo1;
    frame[10] = flags;
    frame[11] = 0;
    frame[12] = 0;
    frame[13] = 0;
    frame[14] = 0;
    frame[15] = calc_crc8(frame, 15);
}

void control_build_mpu_frame(uint8_t *frame,
                             uint8_t type,
                             int16_t ax, int16_t ay, int16_t az,
                             int16_t gx, int16_t gy, int16_t gz)
{
    frame[0] = CTRL_MPU_HEAD_0;
    frame[1] = CTRL_MPU_HEAD_1;
    frame[2] = type;
    frame[3] = (uint8_t)(ax & 0xFF);
    frame[4] = (uint8_t)((ax >> 8) & 0xFF);
    frame[5] = (uint8_t)(ay & 0xFF);
    frame[6] = (uint8_t)((ay >> 8) & 0xFF);
    frame[7] = (uint8_t)(az & 0xFF);
    frame[8] = (uint8_t)((az >> 8) & 0xFF);
    frame[9] = (uint8_t)(gx & 0xFF);
    frame[10] = (uint8_t)((gx >> 8) & 0xFF);
    frame[11] = (uint8_t)(gy & 0xFF);
    frame[12] = (uint8_t)((gy >> 8) & 0xFF);
    frame[13] = (uint8_t)(gz & 0xFF);
    frame[14] = (uint8_t)((gz >> 8) & 0xFF);
    frame[15] = calc_crc8(frame, 15);
}
