/**
 * @file main.c
 * @brief ESP32-S3 W5500 AUV 网络控制器 (主控制节点, TCP Server)
 *
 * 角色: 主控制节点 (核心控制 + 数据处理 + 指令分发)
 * 硬件: MPU6050 + 2 ESC + 1 L298N + 2 Servo + W5500
 * 网络: TCP Server, 双端口
 *   - 8080: 上位机 (笔记本)  - 发送控制指令, 接收 MPU 数据
 *   - 8081: 远端 ESP32-S3 节点 - 接收转发控制 + 双向 MPU 推送
 *
 * 集成组件:
 *   - wiznet    : W5500 以太网 (ioLibrary, 20MHz SPI polling, 100M FULL)
 *   - status_led: RGB LED 状态指示 (WS2812 GPIO48)
 *   - imu       : MPU6050 (I2C0: SDA=6, SCL=7)
 *   - servo     : PCA9685 (I2C1: SDA=4, SCL=5)
 *   - motor     : 2 ESC (LEDC) + L298N
 *   - control   : 16B 控制帧 + 16B MPU 帧协议
 *
 * 任务清单:
 *   - status_led_task    优先级 5, 栈 2048, 周期 150ms
 *   - tcp_server_task    优先级 5, 栈 8192, 双 socket 状态机
 *   - mpu_push_task      优先级 4, 栈 2048, 周期 50ms (20Hz)
 *   - status_report_task 优先级 4, 栈 2048, 周期 500ms (保留, 可选)
 */

#include "wiznet_conf.h"
#include "wizchip_conf.h"
#include "wiznet_socket.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "wiznet_manager.h"
#include "wiznet_spi.h"
#include "status_led.h"
#include "tcp_parser.h"
#include "motor.h"
#include "imu.h"
#include "servo.h"
#include "control.h"

/* ========== TCP 服务器配置 ========== */
#define TCP_HOST_PORT     8080    /* 上位机 (笔记本) */
#define TCP_REMOTE_PORT   8081    /* 远端 ESP32-S3 节点 */
#define HOST_SOCK         0
#define REMOTE_SOCK       1
#define RX_BUFFER_SIZE    1024
#define TX_BUFFER_SIZE    512
#define MPU_FRAME_SIZE    16

/* 硬件引脚定义 */
#define RGB_LED_GPIO      48      /* 板载 RGB LED (WS2812) */

static uint8_t s_rx_buffer[RX_BUFFER_SIZE];
static uint8_t s_tx_buffer[TX_BUFFER_SIZE];
static uint8_t s_mpu_frame[MPU_FRAME_SIZE];

static const char *TAG = "APP";

/* ========== 连接状态 ========== */
static volatile bool s_host_connected   = false;
static volatile bool s_remote_connected = false;

/* ========== 任务句柄 ========== */
static TaskHandle_t s_mpu_push_task_handle = NULL;
static TaskHandle_t s_status_report_task_handle = NULL;

/* ========== 协议实现 ========== */

/* 主 → 远端 转发 (由 control 模块通过回调调用)
 *
 * 收到上位机 16B 控制帧后, 若 flags bit1 置位, 主节点构造 8B 子帧
 * 通过 socket 1 发给远端. */
static void forward_to_remote(const ctrl_command_t *cmd)
{
    if (!s_remote_connected) {
        ESP_LOGW("control", "远程未连接, 转发丢弃");
        return;
    }

    /* 构造 8B 子帧: AA 55 cmd remote_esc1 remote_esc2 0 0 CRC */
    uint8_t fwd[CTRL_FWD_FRAME_SIZE];
    fwd[0] = CTRL_CTRL_HEAD_0;
    fwd[1] = CTRL_CTRL_HEAD_1;
    fwd[2] = cmd->cmd;
    fwd[3] = (uint8_t)cmd->remote_esc[0];
    fwd[4] = (uint8_t)cmd->remote_esc[1];
    fwd[5] = 0;
    fwd[6] = 0;
    uint8_t crc = 0;
    for (int i = 0; i < 7; i++) crc ^= fwd[i];
    fwd[7] = crc;

    int32_t sent = wiz_send(REMOTE_SOCK, fwd, CTRL_FWD_FRAME_SIZE);
    if (sent != CTRL_FWD_FRAME_SIZE) {
        ESP_LOGW("control", "转发到远端失败: %d", (int)sent);
    }
}

/* MPU 数据推送到上位机 (由 control 模块通过 handle_mpu_frame 调用) */
void tcp_server_forward_mpu_to_host(const ctrl_mpu_data_t *m)
{
    if (!s_host_connected) return;
    control_build_mpu_frame(s_mpu_frame, m->type,
                            m->ax, m->ay, m->az, m->gx, m->gy, m->gz);
    int32_t sent = wiz_send(HOST_SOCK, s_mpu_frame, MPU_FRAME_SIZE);
    if (sent != MPU_FRAME_SIZE) {
        ESP_LOGW("control", "MPU 推上位机失败: %d", (int)sent);
    }
}

/* MPU 数据推送到远端 (主节点 MPU 自身) */
static void push_mpu_to_remote(uint8_t type, int16_t ax, int16_t ay, int16_t az,
                                int16_t gx, int16_t gy, int16_t gz)
{
    if (!s_remote_connected) return;
    control_build_mpu_frame(s_mpu_frame, type, ax, ay, az, gx, gy, gz);
    wiz_send(REMOTE_SOCK, s_mpu_frame, MPU_FRAME_SIZE);
}

/* ========== 任务实现 ========== */

static void status_led_task(void *pvParameters)
{
    esp_task_wdt_delete(NULL);
    while (1) {
        esp_task_wdt_reset();
        status_led_update();
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

/* 通用 socket 处理: listen -> 接收 -> 解析 -> 关闭重建 */
static void handle_socket(uint8_t sock, const char *name,
                          bool *connected_flag, void (*on_connect)(void))
{
    uint8_t sr = getSn_SR(sock);
    int32_t n;

    switch (sr) {
    case SOCK_ESTABLISHED:
        if (!(*connected_flag)) {
            *connected_flag = true;
            ESP_LOGI("TCP", "[%s] 已连接", name);
            control_reset();   /* 切换连接时清空残帧 */
            if (on_connect) on_connect();
        }
        n = wiz_recv(sock, s_rx_buffer, sizeof(s_rx_buffer));
        if (n > 0) {
            status_led_notify_rx((uint32_t)n);
            /* 用独立的解析器, 因为两条链路可能并发 */
            control_process(s_rx_buffer, (size_t)n);
        } else if (n == SOCK_BUSY) {
            vTaskDelay(pdMS_TO_TICKS(2));
        } else {
            /* 对端关闭或错误 */
            wiz_close(sock);
            *connected_flag = false;
            ESP_LOGI("TCP", "[%s] 已断开 (n=%d)", name, (int)n);
            if (wiz_socket(sock, Sn_MR_TCP, (sock == HOST_SOCK) ? TCP_HOST_PORT : TCP_REMOTE_PORT,
                           SF_TCP_NODELAY) == (int8_t)sock) {
                wiz_listen(sock);
            }
        }
        break;

    case SOCK_CLOSE_WAIT:
        *connected_flag = false;
        wiz_close(sock);
        if (wiz_socket(sock, Sn_MR_TCP, (sock == HOST_SOCK) ? TCP_HOST_PORT : TCP_REMOTE_PORT,
                       SF_TCP_NODELAY) == (int8_t)sock) {
            wiz_listen(sock);
        }
        break;

    case SOCK_CLOSED:
        *connected_flag = false;
        if (wiz_socket(sock, Sn_MR_TCP, (sock == HOST_SOCK) ? TCP_HOST_PORT : TCP_REMOTE_PORT,
                       SF_TCP_NODELAY) == (int8_t)sock) {
            wiz_listen(sock);
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
 * @brief TCP 服务器任务 (双 socket)
 *
 * Socket 0 = 8080 (上位机)
 * Socket 1 = 8081 (远端)
 *
 * 注意: 本任务做阻塞式 SPI 通讯, 不订阅 Task WDT.
 */
static void tcp_server_task(void *pvParameters)
{
    esp_task_wdt_delete(NULL);

    /* 注册主→远端 转发回调 */
    control_set_forward_callback(forward_to_remote);

    /* 启动 socket 0 (8080 上位机) */
    if (wiz_socket(HOST_SOCK, Sn_MR_TCP, TCP_HOST_PORT, SF_TCP_NODELAY) != HOST_SOCK) {
        ESP_LOGE("TCP", "socket 0 (HOST) failed");
    } else if (wiz_listen(HOST_SOCK) != SOCK_OK) {
        ESP_LOGE("TCP", "listen 8080 failed");
        wiz_close(HOST_SOCK);
    }

    /* 启动 socket 1 (8081 远端) */
    if (wiz_socket(REMOTE_SOCK, Sn_MR_TCP, TCP_REMOTE_PORT, SF_TCP_NODELAY) != REMOTE_SOCK) {
        ESP_LOGE("TCP", "socket 1 (REMOTE) failed");
    } else if (wiz_listen(REMOTE_SOCK) != SOCK_OK) {
        ESP_LOGE("TCP", "listen 8081 failed");
        wiz_close(REMOTE_SOCK);
    }

    ESP_LOGI("TCP", "=== TCP 服务器已启动: 8080(HOST) + 8081(REMOTE) ===");

    while (1) {
        esp_task_wdt_reset();

        /* INT 唤醒 (如果有) */
        wiznet_spi_check_int();

        /* 交替处理两个 socket */
        handle_socket(HOST_SOCK,   "HOST",   &s_host_connected,   NULL);
        handle_socket(REMOTE_SOCK, "REMOTE", &s_remote_connected, NULL);
    }
}

/* MPU 推送任务: 20Hz 读取本地 MPU, 推送到上位机 + 远端 */
static void mpu_push_task(void *pvParameters)
{
    esp_task_wdt_delete(NULL);
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(50);  /* 20Hz */

    imu_data_t m = {0};

    while (1) {
        esp_task_wdt_reset();
        vTaskDelayUntil(&last_wake, period);

        /* 读取本地 MPU (静默失败时不上报) */
        if (imu_read(&m) == ESP_OK) {
            int16_t ax = (int16_t)(m.ax * 16384.0f);
            int16_t ay = (int16_t)(m.ay * 16384.0f);
            int16_t az = (int16_t)(m.az * 16384.0f);
            int16_t gx = (int16_t)(m.gx * 131.0f);
            int16_t gy = (int16_t)(m.gy * 131.0f);
            int16_t gz = (int16_t)(m.gz * 131.0f);

            /* 推上位机 (type=LOCAL) */
            if (s_host_connected) {
                control_build_mpu_frame(s_mpu_frame, CTRL_MPU_TYPE_LOCAL,
                                        ax, ay, az, gx, gy, gz);
                wiz_send(HOST_SOCK, s_mpu_frame, MPU_FRAME_SIZE);
            }
            /* 推远端 (type=LOCAL, 远端知道是主节点发的) */
            push_mpu_to_remote(CTRL_MPU_TYPE_LOCAL, ax, ay, az, gx, gy, gz);
        }
    }
}

/* 状态上报任务: 500ms 周期 JSON, 保留备用 (可选启动) */
static void status_report_task(void *pvParameters)
{
    esp_task_wdt_delete(NULL);
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(500);

    while (1) {
        esp_task_wdt_reset();
        vTaskDelayUntil(&last_wake, period);

        if (!s_host_connected) continue;

        /* 简化的状态: 实际可由 cJSON 构造, 此处先输出最少信息 */
        int n = snprintf((char *)s_tx_buffer, sizeof(s_tx_buffer),
                         "{\"uptime_ms\":%lld,\"link\":true,\"host\":%d,\"remote\":%d,\"frames\":%lu}\n",
                         (long long)(esp_timer_get_time() / 1000),
                         s_host_connected ? 1 : 0,
                         s_remote_connected ? 1 : 0,
                         (unsigned long)control_get_frame_count());
        if (n > 0) {
            wiz_send(HOST_SOCK, s_tx_buffer, (uint16_t)n);
        }
    }
}

/* ========== 启动流程 ========== */

static void start_components(void)
{
    ESP_LOGI(TAG, "=== AUV 控制器启动 (W5500 TOE 模式) ===");

    /* 1. NVS (供 IDF 内部组件使用) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 2. 状态 LED */
    ESP_ERROR_CHECK(status_led_init(RGB_LED_GPIO));

    /* 3. W5500 (必须在 LED/控制 task 之前初始化) */
    wiznet_manager_config_t wcfg = wiznet_manager_get_default_config();
    /* 8 socket 缓冲 (RX+TX 各 2KB, 共 32KB) - W5500 内部 16KB/32KB/48KB 选型 */
    /* 注: 默认配置已设为 2KB per socket, 8 个 socket 共用 32KB W5500 SRAM */
    ESP_ERROR_CHECK(wiznet_manager_init(&wcfg));

    /* 4. IMU (MPU6050) - 可选, 失败不阻塞 */
    imu_config_t icfg = imu_get_default_config();
    if (imu_init(&icfg) != ESP_OK) {
        ESP_LOGW(TAG, "MPU6050 初始化失败, 继续运行");
    }

    /* 5. Servo (PCA9685) - 可选, 失败不阻塞 */
    servo_config_t scfg = servo_get_default_config();
    if (servo_init(&scfg) != ESP_OK) {
        ESP_LOGW(TAG, "PCA9685 初始化失败, 继续运行");
    }

    /* 6. Motor (LEDC + L298N) */
    motor_config_t mcfg = motor_get_default_config();
    ESP_ERROR_CHECK(motor_init(&mcfg));

    /* 7. 等待 link up (30s 超时) */
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
            ESP_LOGI(TAG, "本机 IP: %d.%d.%d.%d",
                     info.ip.addr & 0xFF, (info.ip.addr >> 8) & 0xFF,
                     (info.ip.addr >> 16) & 0xFF, (info.ip.addr >> 24) & 0xFF);
            ESP_LOGI(TAG, "子网掩码: %d.%d.%d.%d",
                     info.netmask.addr & 0xFF, (info.netmask.addr >> 8) & 0xFF,
                     (info.netmask.addr >> 16) & 0xFF, (info.netmask.addr >> 24) & 0xFF);
            ESP_LOGI(TAG, "默认网关: %d.%d.%d.%d",
                     info.gw.addr & 0xFF, (info.gw.addr >> 8) & 0xFF,
                     (info.gw.addr >> 16) & 0xFF, (info.gw.addr >> 24) & 0xFF);
        }
    } else {
        ESP_LOGW(TAG, "=== 网线未连接 (30s 超时) ===");
    }

    /* 8. 日志降噪 (默认全开, 这里只保留必要 tag) */
    esp_log_level_set("*", ESP_LOG_NONE);
    esp_log_level_set("APP",      ESP_LOG_INFO);
    esp_log_level_set("wiznet_mgr", ESP_LOG_INFO);
    esp_log_level_set("wiznet_spi", ESP_LOG_INFO);
    esp_log_level_set("TCP",        ESP_LOG_INFO);
    esp_log_level_set("control",    ESP_LOG_INFO);

    /* 9. 创建任务 */
    xTaskCreate(status_led_task,    "led",     2048, NULL, 5, NULL);
    xTaskCreate(tcp_server_task,    "tcp_srv", 8192, NULL, 5, NULL);
    xTaskCreate(mpu_push_task,      "mpu",     2048, NULL, 4, &s_mpu_push_task_handle);
    /* status_report_task 暂时不启动 (MPU 推送已涵盖大部分状态) */
    /* xTaskCreate(status_report_task, "status",  2048, NULL, 4, &s_status_report_task_handle); */
}

void app_main(void)
{
    start_components();
}
