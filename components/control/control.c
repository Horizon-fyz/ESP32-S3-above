/**
 * @file control.c
 * @brief 控制协议 v5.0 (含 v3.0 沉浮协议): 差速驱动 + L298N 铲斗电机 + 远端灯 + 沉浮转发
 *
 * 状态机: 字节流扫描, 残帧保留
 *   IDLE → GOT_HEAD0 → LOADING_CTRL (16B) / LOADING_MPU (16B)
 *
 * 主节点本地执行: 差速混合
 *   left_esc  = clamp(speed + yaw, -100, +100) → ESC1
 *   right_esc = clamp(speed - yaw, -100, +100) → ESC2
 *   dc_speed  = |speed|                          → L298N PWM
 *   dc_dir    = sign(speed)                       → L298N IN1/IN2
 */

#include "control.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "motor.h"

static const char *TAG = "control";

typedef enum {
    CTRL_STATE_IDLE = 0,
    CTRL_STATE_GOT_HEAD0,
    CTRL_STATE_LOADING_CTRL,   /* 16B 控制帧 */
    CTRL_STATE_LOADING_MPU,    /* 16B MPU 帧 */
} ctrl_state_t;

static ctrl_state_t      s_state       = CTRL_STATE_IDLE;
static uint8_t           s_buf[CTRL_CTRL_FRAME_SIZE];
static size_t            s_buf_len     = 0;
static uint32_t          s_frame_count = 0;
static ctrl_forward_cb_t s_forward_cb  = NULL;
static ctrl_local_cb_t   s_local_cb    = NULL;

/* 差速混合与电机驱动已迁到 main.c —— 见 control.h 顶部说明.
 * 本组件只解析协议, 通过 s_local_cb 把指令交给上层, 不碰任何电机:
 * 原先这里写死了旧硬件 (2 路电调 + L298N 由 |speed| 驱动), 与现有
 * "4 路单向电调 (正=推进/负=反推) + L298N 滚筒收放" 不符 ——
 * 把负油门发给单向电调会被 motor_set_esc_throttle() 钳成 0, 反推两路永远不会动. */

/* 紧急停止: 所有电机归零 (本地) */
static void emergency_stop_local(void)
{
    motor_emergency_stop();
}

/* 解析后: 控制帧 */
static void handle_ctrl_command(const uint8_t *frame)
{
    ctrl_command_t cmd = {
        .cmd             = frame[2],
        .speed           = (int8_t)frame[3],
        .yaw             = (int8_t)frame[4],
        .remote_light    = frame[5],
        .bucket_speed    = (int8_t)frame[6],
        .flags           = frame[10],
        /* v3.0 DEPTH 字段 (cmd=0x11) */
        .target_depth_cm = (int16_t)(frame[3] | (frame[4] << 8)),
        .target_pitch_deg = (int8_t)frame[5],
        .target_roll_deg  = (int8_t)frame[6],
        /* v9.2 SERVO 字段 (cmd=0x12) */
        .servo_mask       = frame[7],
        .servo_deg10      = { (int16_t)(frame[3] | (frame[4] << 8)),
                              (int16_t)(frame[5] | (frame[6] << 8)) },
    };

    switch (cmd.cmd) {
    case CTRL_CMD_MOTOR:
        if (cmd.flags & CTRL_FLAG_ENABLE_LOCAL) {
            /* 交给上层执行本地动作 (差速混合 + 4 路电调 + L298N 滚筒) */
            if (s_local_cb) s_local_cb(&cmd);
        }
        if ((cmd.flags & CTRL_FLAG_FORWARD_REMOTE) && s_forward_cb) {
            s_forward_cb(&cmd);   /* 由 tcp_server 任务发 8B 子帧 */
        }
        ESP_LOGD(TAG, "CTRL speed=%d yaw=%d light=%d bucket=%d flags=0x%02X",
                 cmd.speed, cmd.yaw, cmd.remote_light, cmd.bucket_speed, cmd.flags);
        break;

    case CTRL_CMD_SERVO:
        /* v9.2 云台手动: 只有掩码置位的通道会被上层动作 (mask=0 ⇒ 什么都不做) */
        if ((cmd.flags & CTRL_FLAG_ENABLE_LOCAL) && cmd.servo_mask && s_local_cb) {
            s_local_cb(&cmd);
        }
        ESP_LOGD(TAG, "CTRL SERVO mask=0x%02X ch0=%.1f° ch1=%.1f°",
                 cmd.servo_mask, cmd.servo_deg10[0] / 10.0, cmd.servo_deg10[1] / 10.0);
        break;

    case CTRL_CMD_DEPTH:
        /* v3.0 沉浮控制: 目标深度 cm + 目标俯仰/横滚°, 仅转发远端 (主控不沉浮) */
        if ((cmd.flags & CTRL_FLAG_FORWARD_REMOTE) && s_forward_cb) {
            s_forward_cb(&cmd);
        }
        ESP_LOGI(TAG, "CTRL DEPTH target=%dcm pitch=%ddeg roll=%ddeg flags=0x%02X",
                 cmd.target_depth_cm, cmd.target_pitch_deg, cmd.target_roll_deg, cmd.flags);
        break;

    case CTRL_CMD_STOP:
        emergency_stop_local();
        if (s_forward_cb) {
            /* 转发急停 (cmd=0x20, speed=0, yaw=0, light=0, dir=0) */
            s_forward_cb(&cmd);
        }
        ESP_LOGW(TAG, "EMERGENCY STOP");
        break;

    case CTRL_CMD_REBOOT:
        ESP_LOGW(TAG, "REBOOT in 500ms");
        emergency_stop_local();
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
        break;

    case CTRL_CMD_SHUTDOWN:
        ESP_LOGW(TAG, "DEEP SLEEP in 500ms");
        emergency_stop_local();
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_deep_sleep_start();
        break;

    default:
        ESP_LOGW(TAG, "Unknown cmd 0x%02X", cmd.cmd);
        break;
    }
}

/* MPU 帧回调 (由 main.c 实现, 转发到上位机) */
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
                s_buf[0] = b; s_buf_len = 1;
                s_state = CTRL_STATE_GOT_HEAD0;
            } else if (b == CTRL_MPU_HEAD_0) {
                s_buf[0] = b; s_buf_len = 1;
                s_state = CTRL_STATE_GOT_HEAD0;
            }
            break;

        case CTRL_STATE_GOT_HEAD0:
            if (s_buf[0] == CTRL_CTRL_HEAD_0) {
                if (b == CTRL_CTRL_HEAD_1) {
                    s_buf[1] = b; s_buf_len = 2;
                    s_state = CTRL_STATE_LOADING_CTRL;
                } else if (b == CTRL_CTRL_HEAD_0) {
                    s_buf[0] = b; s_buf_len = 1;
                } else {
                    s_state = CTRL_STATE_IDLE;
                    s_buf_len = 0;
                }
            } else { /* MPU 帧 */
                if (b == CTRL_MPU_HEAD_1) {
                    s_buf[1] = b; s_buf_len = 2;
                    s_state = CTRL_STATE_LOADING_MPU;
                } else if (b == CTRL_MPU_HEAD_0) {
                    s_buf[0] = b; s_buf_len = 1;
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
        }
    }
    return frames_done;
}

void control_set_forward_callback(ctrl_forward_cb_t cb)
{
    s_forward_cb = cb;
}

void control_set_local_callback(ctrl_local_cb_t cb)
{
    s_local_cb = cb;
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
    uint8_t cmd, int8_t speed, int8_t yaw,
    uint8_t remote_light, int8_t bucket_speed, uint8_t flags)
{
    frame[0]  = CTRL_CTRL_HEAD_0;
    frame[1]  = CTRL_CTRL_HEAD_1;
    frame[2]  = cmd;
    frame[3]  = (uint8_t)speed;
    frame[4]  = (uint8_t)yaw;
    frame[5]  = remote_light;
    frame[6]  = (uint8_t)bucket_speed;
    frame[7]  = 0;
    frame[8]  = 0;   /* 原 servo 0, 已删除 */
    frame[9]  = 0;   /* 原 servo 1, 已删除 */
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
    frame[0]  = CTRL_MPU_HEAD_0;
    frame[1]  = CTRL_MPU_HEAD_1;
    frame[2]  = type;
    frame[3]  = (uint8_t)(ax & 0xFF);
    frame[4]  = (uint8_t)((ax >> 8) & 0xFF);
    frame[5]  = (uint8_t)(ay & 0xFF);
    frame[6]  = (uint8_t)((ay >> 8) & 0xFF);
    frame[7]  = (uint8_t)(az & 0xFF);
    frame[8]  = (uint8_t)((az >> 8) & 0xFF);
    frame[9]  = (uint8_t)(gx & 0xFF);
    frame[10] = (uint8_t)((gx >> 8) & 0xFF);
    frame[11] = (uint8_t)(gy & 0xFF);
    frame[12] = (uint8_t)((gy >> 8) & 0xFF);
    frame[13] = (uint8_t)(gz & 0xFF);
    frame[14] = (uint8_t)((gz >> 8) & 0xFF);
    frame[15] = calc_crc8(frame, 15);
}

/* v3.0 沉浮控制帧 (16B, cmd=0x11, 上位机→主控)
 *   [3-4] target_depth int16 LE (cm)  [5] target_pitch int8(°)  [6] target_roll int8(°)  [10] flags */

/* GPS 状态帧 type=0x04 (定位): [3-6] lat ×1e7  [7-10] lon ×1e7
 * [11-12] alt ×0.1m  [13] 卫星数  [14] flags —— 布局详见 control.h 顶部注释 */
void control_build_gps_pos_frame(uint8_t *frame,
    uint8_t flags, int32_t lat_e7, int32_t lon_e7, int16_t alt_dm, uint8_t sats)
{
    uint32_t lat = (uint32_t)lat_e7;
    uint32_t lon = (uint32_t)lon_e7;

    frame[0]  = CTRL_MPU_HEAD_0;
    frame[1]  = CTRL_MPU_HEAD_1;
    frame[2]  = CTRL_MPU_TYPE_GPS_POS;
    frame[3]  = (uint8_t)( lat        & 0xFF);
    frame[4]  = (uint8_t)((lat >>  8) & 0xFF);
    frame[5]  = (uint8_t)((lat >> 16) & 0xFF);
    frame[6]  = (uint8_t)((lat >> 24) & 0xFF);
    frame[7]  = (uint8_t)( lon        & 0xFF);
    frame[8]  = (uint8_t)((lon >>  8) & 0xFF);
    frame[9]  = (uint8_t)((lon >> 16) & 0xFF);
    frame[10] = (uint8_t)((lon >> 24) & 0xFF);
    frame[11] = (uint8_t)((uint16_t)alt_dm & 0xFF);
    frame[12] = (uint8_t)(((uint16_t)alt_dm >> 8) & 0xFF);
    frame[13] = sats;
    frame[14] = flags;
    frame[15] = calc_crc8(frame, 15);
}

/* GPS 状态帧 type=0x05 (运动/精度/时间): 布局详见 control.h 顶部注释 */
void control_build_gps_nav_frame(uint8_t *frame,
    uint8_t flags, uint16_t course_cdeg, uint16_t speed_ckmh,
    uint8_t pdop_d1, uint8_t hdop_d1, uint8_t vdop_d1,
    uint8_t hour, uint8_t minute, uint8_t second)
{
    frame[0]  = CTRL_MPU_HEAD_0;
    frame[1]  = CTRL_MPU_HEAD_1;
    frame[2]  = CTRL_MPU_TYPE_GPS_NAV;
    frame[3]  = (uint8_t)(course_cdeg & 0xFF);
    frame[4]  = (uint8_t)((course_cdeg >> 8) & 0xFF);
    frame[5]  = (uint8_t)(speed_ckmh & 0xFF);
    frame[6]  = (uint8_t)((speed_ckmh >> 8) & 0xFF);
    frame[7]  = pdop_d1;
    frame[8]  = hdop_d1;
    frame[9]  = vdop_d1;
    frame[10] = hour;
    frame[11] = minute;
    frame[12] = second;
    frame[13] = flags;
    frame[14] = 0;
    frame[15] = calc_crc8(frame, 15);
}

void control_build_depth_ctrl_frame(uint8_t *frame,
    int16_t target_depth_cm, int8_t target_pitch, int8_t target_roll, uint8_t flags)
{
    frame[0]  = CTRL_CTRL_HEAD_0;
    frame[1]  = CTRL_CTRL_HEAD_1;
    frame[2]  = CTRL_CMD_DEPTH;
    frame[3]  = (uint8_t)(target_depth_cm & 0xFF);
    frame[4]  = (uint8_t)((target_depth_cm >> 8) & 0xFF);
    frame[5]  = (uint8_t)target_pitch;
    frame[6]  = (uint8_t)target_roll;
    frame[7]  = 0;
    frame[8]  = 0;
    frame[9]  = 0;
    frame[10] = flags;
    frame[11] = 0;
    frame[12] = 0;
    frame[13] = 0;
    frame[14] = 0;
    frame[15] = calc_crc8(frame, 15);
}

/* v3.0 沉浮控制子帧 (8B, cmd=0x11, 主控→远端)
 *   [3-4] target_depth int16 LE (cm)  [5] target_pitch int8(°)  [6] target_roll int8(°) */
void control_build_depth_fwd_frame(uint8_t *frame,
    int16_t target_depth_cm, int8_t target_pitch, int8_t target_roll)
{
    frame[0] = CTRL_CTRL_HEAD_0;
    frame[1] = CTRL_CTRL_HEAD_1;
    frame[2] = CTRL_CMD_DEPTH;
    frame[3] = (uint8_t)(target_depth_cm & 0xFF);
    frame[4] = (uint8_t)((target_depth_cm >> 8) & 0xFF);
    frame[5] = (uint8_t)target_pitch;
    frame[6] = (uint8_t)target_roll;
    frame[7] = calc_crc8(frame, 7);
}
