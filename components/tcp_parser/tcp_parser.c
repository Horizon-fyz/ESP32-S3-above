/**
 * @file tcp_parser.c
 * @brief TCP报文解析器实现
 */

#include "tcp_parser.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "esp_log.h"
#include "lwip/inet.h"
#include "lwip/sockets.h" 

static const char *TAG = "tcp_parser";
static uint8_t s_rx_buffer[2048];

// 解析器状态
static tcp_parser_config_t s_config;
static tcp_data_callback_t s_callback = NULL;
static void *s_user_data = NULL;
static tcp_parser_stats_t s_stats = {0};
static bool s_initialized = false;

// 前导报文字符串（用于调试输出）
static const char *TAG_PREFIX = "TCP Packet";

// 客户端相关
static TaskHandle_t s_client_task_handle = NULL;
static bool s_client_running = false;
static int s_client_sock = -1;
static EventGroupHandle_t s_client_event_group = NULL;
#define CLIENT_CONNECTED_BIT BIT0
#define CLIENT_STOP_BIT      BIT1

static tcp_client_config_t s_client_config;
static tcp_data_callback_t s_client_callback = NULL;
static void *s_client_user_data = NULL;

/**
 * @brief 计算TCP校验和（简化版，用于验证）
 */
static uint16_t tcp_checksum(const uint8_t *data, size_t len)
{
    // 简化实现：实际应该包含伪头部
    // 这里仅做简单校验
    uint32_t sum = 0;
    for (size_t i = 0; i < len; i += 2) {
        uint16_t word;
        if (i + 1 < len) {
            word = (data[i] << 8) | data[i + 1];
        } else {
            word = data[i] << 8;
        }
        sum += word;
        if (sum & 0xFFFF0000) {
            sum = (sum & 0xFFFF) + (sum >> 16);
        }
    }
    return ~(sum & 0xFFFF);
}

/**
 * @brief 获取TCP标志位字符串
 */
static const char* get_tcp_flags_string(uint8_t flags)
{
    static char buffer[16];
    int pos = 0;
    
    if (flags & TCP_FLAG_FIN) buffer[pos++] = 'F';
    if (flags & TCP_FLAG_SYN) buffer[pos++] = 'S';
    if (flags & TCP_FLAG_RST) buffer[pos++] = 'R';
    if (flags & TCP_FLAG_PSH) buffer[pos++] = 'P';
    if (flags & TCP_FLAG_ACK) buffer[pos++] = 'A';
    if (flags & TCP_FLAG_URG) buffer[pos++] = 'U';
    if (flags & TCP_FLAG_ECE) buffer[pos++] = 'E';
    if (flags & TCP_FLAG_CWR) buffer[pos++] = 'C';
    if (pos == 0) buffer[pos++] = 'N';
    buffer[pos] = '\0';
    
    return buffer;
}

/**
 * @brief 打印TCP报文信息（仅当ESP_LOG级别为DEBUG时）
 */
static void log_tcp_packet(const tcp_packet_info_t *info)
{
    char flags_str[16];
    snprintf(flags_str, sizeof(flags_str), "%s", get_tcp_flags_string(info->flags));
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "TCP Packet Analysis:");
    ESP_LOGI(TAG, "  Source Port:      %u", info->src_port);
    ESP_LOGI(TAG, "  Destination Port: %u", info->dst_port);
    ESP_LOGI(TAG, "  Sequence Number:  0x%08X (%u)", info->seq_num, info->seq_num);
    ESP_LOGI(TAG, "  Acknowledgment:   0x%08X (%u)", info->ack_num, info->ack_num);
    ESP_LOGI(TAG, "  Data Offset:      %u (header length: %u bytes)", 
             info->data_offset, info->data_offset * 4);
    ESP_LOGI(TAG, "  Flags:            %s (0x%02X)", flags_str, info->flags);
    ESP_LOGI(TAG, "  Window Size:      %u", info->window);
    ESP_LOGI(TAG, "  Checksum:         0x%04X", info->checksum);
    ESP_LOGI(TAG, "  Urgent Pointer:   %u", info->urgent_ptr);
    ESP_LOGI(TAG, "  Payload Length:   %u bytes", info->payload_len);
    
    // 打印载荷内容（如果存在且长度合理）
    if (info->payload && info->payload_len > 0 && info->payload_len <= 256) {
        ESP_LOGI(TAG, "  Payload Data (hex):");
        char hex_str[256] = {0};
        char *ptr = hex_str;
        for (uint16_t i = 0; i < info->payload_len && i < 64; i++) {
            ptr += sprintf(ptr, "%02X ", info->payload[i]);
            if ((i + 1) % 16 == 0 && i + 1 < info->payload_len) {
                ESP_LOGI(TAG, "    %s", hex_str);
                ptr = hex_str;
                *ptr = '\0';
            }
        }
        if (strlen(hex_str) > 0) {
            ESP_LOGI(TAG, "    %s", hex_str);
        }
        
        // 尝试打印ASCII（仅当数据可打印）
        char ascii_str[257] = {0};
        bool printable = true;
        for (uint16_t i = 0; i < info->payload_len && i < 64; i++) {
            if (info->payload[i] >= 0x20 && info->payload[i] <= 0x7E) {
                ascii_str[i] = info->payload[i];
            } else if (info->payload[i] == 0x0A || info->payload[i] == 0x0D) {
                ascii_str[i] = ' ';
            } else {
                ascii_str[i] = '.';
                printable = false;
            }
        }
        if (printable && info->payload_len > 0) {
            ESP_LOGI(TAG, "  Payload (ASCII): %s", ascii_str);
        }
    }
    ESP_LOGI(TAG, "========================================");
}

/**
 * @brief 初始化TCP解析器
 */
esp_err_t tcp_parser_init(const tcp_parser_config_t *config)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "TCP parser already initialized");
        return ESP_OK;
    }
    
    // 使用默认配置或用户配置
    if (config != NULL) {
        memcpy(&s_config, config, sizeof(tcp_parser_config_t));
    } else {
        s_config = tcp_parser_get_default_config();
    }
    
    // 重置统计信息
    memset(&s_stats, 0, sizeof(tcp_parser_stats_t));
    s_callback = NULL;
    s_user_data = NULL;
    s_initialized = true;
    
    ESP_LOGI(TAG, "TCP Parser initialized (checksum_verify=%d, parse_payload=%d)",
             s_config.enable_checksum_verify, s_config.parse_payload);
    
    return ESP_OK;
}

/**
 * @brief 注册TCP数据回调函数
 */
esp_err_t tcp_parser_register_callback(tcp_data_callback_t callback, void *user_data)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Parser not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    s_callback = callback;
    s_user_data = user_data;
    
    ESP_LOGI(TAG, "Callback registered: %p", callback);
    return ESP_OK;
}

/**
 * @brief 解析TCP报文
 */
bool tcp_parser_parse(const uint8_t *data, size_t length, tcp_packet_info_t *info)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Parser not initialized");
        return false;
    }
    
    if (data == NULL || length < sizeof(tcp_header_t)) {
        ESP_LOGW(TAG, "Invalid TCP packet: data=%p, length=%u", data, length);
        s_stats.invalid_packets++;
        return false;
    }
    
    s_stats.total_packets++;
    
    // 解析TCP头部
    const tcp_header_t *tcp_hdr = (const tcp_header_t *)data;
    
    // 提取信息（注意网络字节序转换）
    uint16_t src_port = ntohs(tcp_hdr->src_port);
    uint16_t dst_port = ntohs(tcp_hdr->dst_port);
    uint32_t seq_num = ntohl(tcp_hdr->seq_num);
    uint32_t ack_num = ntohl(tcp_hdr->ack_num);
    uint8_t data_offset = (tcp_hdr->offset_flags >> 4) & 0x0F;
    uint8_t flags = tcp_hdr->flags;
    uint16_t window = ntohs(tcp_hdr->window);
    uint16_t checksum = ntohs(tcp_hdr->checksum);
    uint16_t urgent_ptr = ntohs(tcp_hdr->urgent_ptr);
    
    // 检查数据偏移是否有效
    if (data_offset < 5) {
        ESP_LOGW(TAG, "Invalid TCP header length: %u (min is 5)", data_offset);
        s_stats.invalid_packets++;
        return false;
    }
    
    uint16_t header_len = data_offset * 4;
    if (header_len > length) {
        ESP_LOGW(TAG, "TCP header length (%u) exceeds packet length (%u)", 
                 header_len, length);
        s_stats.invalid_packets++;
        return false;
    }
    
    // 验证校验和（如果启用）
    if (s_config.enable_checksum_verify) {
        uint16_t calc_checksum = tcp_checksum(data, length);
        if (calc_checksum != checksum && checksum != 0) {
            ESP_LOGW(TAG, "TCP checksum mismatch: calc=0x%04X, packet=0x%04X", 
                     calc_checksum, checksum);
            s_stats.checksum_errors++;
            // 仍然继续解析，但标记为无效
        }
    }
    
    // 填充解析结果
    if (info != NULL) {
        info->src_port = src_port;
        info->dst_port = dst_port;
        info->seq_num = seq_num;
        info->ack_num = ack_num;
        info->data_offset = data_offset;
        info->flags = flags;
        info->window = window;
        info->checksum = checksum;
        info->urgent_ptr = urgent_ptr;
        info->raw_data = data;
        info->total_len = length;
        
        // 载荷指针
        if (s_config.parse_payload && length > header_len) {
            info->payload = (uint8_t *)(data + header_len);
            info->payload_len = length - header_len;
            
            // 更新统计信息
            s_stats.valid_packets++;
            s_stats.total_payload_bytes += info->payload_len;
            if (s_stats.min_payload_len == 0 || info->payload_len < s_stats.min_payload_len) {
                s_stats.min_payload_len = info->payload_len;
            }
            if (info->payload_len > s_stats.max_payload_len) {
                s_stats.max_payload_len = info->payload_len;
            }
        } else {
            info->payload = NULL;
            info->payload_len = 0;
            s_stats.valid_packets++;
        }
    } else {
        s_stats.valid_packets++;
    }
    
    // 打印调试信息
    if (info != NULL) {
        log_tcp_packet(info);
    }
    
    // 调用回调函数
    if (s_callback != NULL && info != NULL) {
        s_callback(info, s_user_data);
    }
    
    return true;
}

/**
 * @brief 将TCP报文解析为字符串
 */
char* tcp_parser_to_string(const tcp_packet_info_t *info, char *buffer, size_t buffer_size)
{
    if (info == NULL || buffer == NULL || buffer_size < 128) {
        return buffer;
    }
    
    char flags_str[16];
    snprintf(flags_str, sizeof(flags_str), "%s", get_tcp_flags_string(info->flags));
    
    snprintf(buffer, buffer_size,
         "TCP: src=%u dst=%u seq=0x%08X ack=0x%08X flags=%s(0x%02X) "
         "win=%u len=%u",
         info->src_port, info->dst_port, 
         (unsigned int)info->seq_num, (unsigned int)info->ack_num,
         flags_str, info->flags, info->window, info->payload_len);
    
    return buffer;
}

/**
 * @brief 获取解析器统计信息
 */
esp_err_t tcp_parser_get_stats(tcp_parser_stats_t *stats)
{
    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(stats, &s_stats, sizeof(tcp_parser_stats_t));
    return ESP_OK;
}

/**
 * @brief 重置解析器统计信息
 */
void tcp_parser_reset_stats(void)
{
    memset(&s_stats, 0, sizeof(tcp_parser_stats_t));
    ESP_LOGI(TAG, "Stats reset");
}

/**
 * @brief 释放解析器资源
 */
void tcp_parser_deinit(void)
{
    if (!s_initialized) {
        return;
    }
    
    s_callback = NULL;
    s_user_data = NULL;
    s_initialized = false;
    memset(&s_stats, 0, sizeof(tcp_parser_stats_t));
    
    ESP_LOGI(TAG, "TCP Parser deinitialized");
}

static void tcp_client_task(void *pvParameters)
{
    uint8_t rx_buffer[2048];
    struct sockaddr_in dest_addr;
    EventBits_t bits;

    while (s_client_running) {
        // 创建 socket
        s_client_sock = socket(AF_INET, SOCK_STREAM, 0);
        if (s_client_sock < 0) {
            ESP_LOGE(TAG, "Socket creation failed");
            vTaskDelay(pdMS_TO_TICKS(s_client_config.reconnect_interval_ms));
            continue;
        }

        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(s_client_config.server_port);
        inet_pton(AF_INET, s_client_config.server_ip, &dest_addr.sin_addr);

        ESP_LOGI(TAG, "Connecting to %s:%d...", s_client_config.server_ip, s_client_config.server_port);
        int err = connect(s_client_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Connect failed, retry in %d ms", s_client_config.reconnect_interval_ms);
            close(s_client_sock);
            s_client_sock = -1;
            if (s_client_config.auto_reconnect) {
                vTaskDelay(pdMS_TO_TICKS(s_client_config.reconnect_interval_ms));
                continue;
            } else {
                break;
            }
        }

        ESP_LOGI(TAG, "Connected to server");
        xEventGroupSetBits(s_client_event_group, CLIENT_CONNECTED_BIT);

        // 接收循环
        while (s_client_running) {
            int len = recv(s_client_sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            if (len <= 0) {
                ESP_LOGW(TAG, "Disconnected or error");
                break;
            }
            rx_buffer[len] = '\0';
            ESP_LOGD(TAG, "Received %d bytes", len);

            // 解析
            tcp_packet_info_t info;
            bool ok = tcp_parser_parse(rx_buffer, len, &info);
            if (ok && s_client_callback != NULL) {
                s_client_callback(&info, s_client_user_data);
            }
        }

        // 断开连接
        close(s_client_sock);
        s_client_sock = -1;
        xEventGroupClearBits(s_client_event_group, CLIENT_CONNECTED_BIT);
        if (s_client_config.auto_reconnect) {
            ESP_LOGI(TAG, "Reconnecting in %d ms", s_client_config.reconnect_interval_ms);
            vTaskDelay(pdMS_TO_TICKS(s_client_config.reconnect_interval_ms));
        } else {
            break;
        }
    }

    // 清理
    if (s_client_sock != -1) {
        close(s_client_sock);
        s_client_sock = -1;
    }
    s_client_running = false;
    s_client_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t tcp_client_start(const tcp_client_config_t *config,
                           tcp_data_callback_t callback,
                           void *user_data)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_client_running) {
        ESP_LOGW(TAG, "Client already running");
        return ESP_OK;
    }

    // 保存配置和回调
    memcpy(&s_client_config, config, sizeof(tcp_client_config_t));
    s_client_callback = callback;
    s_client_user_data = user_data;
    s_client_running = true;

    // 创建事件组（若尚未创建）
    if (s_client_event_group == NULL) {
        s_client_event_group = xEventGroupCreate();
        if (s_client_event_group == NULL) {
            s_client_running = false;
            return ESP_ERR_NO_MEM;
        }
    }

    // 创建任务
    BaseType_t ret = xTaskCreate(tcp_client_task, "tcp_client", 8192, NULL, 5, &s_client_task_handle);
    if (ret != pdPASS) {
        s_client_running = false;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "TCP Client started, target %s:%d", config->server_ip, config->server_port);
    return ESP_OK;
}

void tcp_client_stop(void)
{
    if (!s_client_running) return;
    s_client_running = false;
    if (s_client_sock != -1) {
        shutdown(s_client_sock, SHUT_RDWR);
        close(s_client_sock);
        s_client_sock = -1;
    }
    if (s_client_task_handle) {
        vTaskDelete(s_client_task_handle);
        s_client_task_handle = NULL;
    }
    if (s_client_event_group) {
        vEventGroupDelete(s_client_event_group);
        s_client_event_group = NULL;
    }
    ESP_LOGI(TAG, "TCP Client stopped");
}

bool tcp_client_is_connected(void)
{
    if (!s_client_running || s_client_event_group == NULL) return false;
    return (xEventGroupGetBits(s_client_event_group) & CLIENT_CONNECTED_BIT) != 0;
}

esp_err_t tcp_client_send(const uint8_t *data, size_t len)
{
    if (!s_client_running || s_client_sock < 0) {
        return ESP_ERR_INVALID_STATE;
    }
    int sent = send(s_client_sock, data, len, 0);
    if (sent < 0) {
        return ESP_FAIL;
    }
    return (sent == len) ? ESP_OK : ESP_ERR_INVALID_SIZE;
}