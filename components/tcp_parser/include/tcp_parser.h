/**
 * @file tcp_parser.h
 * @brief TCP报文解析器
 * 
 * 提供TCP报文解析功能，支持解析TCP头部、载荷，并提供回调机制
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief TCP头部结构体（网络字节序）
 */
typedef struct __attribute__((packed)) {
    uint16_t src_port;      ///< 源端口
    uint16_t dst_port;      ///< 目的端口
    uint32_t seq_num;       ///< 序列号
    uint32_t ack_num;       ///< 确认号
    uint8_t offset_flags;   ///< 数据偏移 + 保留位
    uint8_t flags;          ///< 标志位
    uint16_t window;        ///< 窗口大小
    uint16_t checksum;      ///< 校验和
    uint16_t urgent_ptr;    ///< 紧急指针
} tcp_header_t;

/**
 * @brief TCP标志位定义
 */
typedef enum {
    TCP_FLAG_FIN = 0x01,    ///< FIN
    TCP_FLAG_SYN = 0x02,    ///< SYN
    TCP_FLAG_RST = 0x04,    ///< RST
    TCP_FLAG_PSH = 0x08,    ///< PSH
    TCP_FLAG_ACK = 0x10,    ///< ACK
    TCP_FLAG_URG = 0x20,    ///< URG
    TCP_FLAG_ECE = 0x40,    ///< ECE
    TCP_FLAG_CWR = 0x80,    ///< CWR
} tcp_flag_t;

/**
 * @brief 解析后的TCP报文信息
 */
typedef struct {
    // TCP头部信息
    uint16_t src_port;
    uint16_t dst_port;
    uint32_t seq_num;
    uint32_t ack_num;
    uint8_t data_offset;       ///< 数据偏移（单位：4字节）
    uint8_t flags;             ///< 所有标志位
    uint16_t window;
    uint16_t checksum;
    uint16_t urgent_ptr;
    
    // 载荷信息
    uint8_t *payload;          ///< 指向载荷数据的指针
    uint16_t payload_len;      ///< 载荷长度
    
    // 原始数据
    const uint8_t *raw_data;   ///< 原始TCP报文数据
    uint16_t total_len;        ///< 报文总长度（包含TCP头部和载荷）
} tcp_packet_info_t;

/**
 * @brief TCP解析器配置
 */
typedef struct {
    bool enable_checksum_verify;   ///< 是否启用校验和验证
    bool parse_payload;            ///< 是否解析载荷内容
} tcp_parser_config_t;

/**
 * @brief TCP数据回调函数类型
 * 
 * @param info 解析后的TCP报文信息
 * @param user_data 用户数据指针
 */
typedef void (*tcp_data_callback_t)(const tcp_packet_info_t *info, void *user_data);

/**
 * @brief 解析器统计信息
 */
typedef struct {
    uint32_t total_packets;        ///< 总解析包数
    uint32_t valid_packets;        ///< 有效包数
    uint32_t invalid_packets;      ///< 无效包数
    uint32_t checksum_errors;      ///< 校验和错误数
    uint32_t min_payload_len;      ///< 最小载荷长度
    uint32_t max_payload_len;      ///< 最大载荷长度
    uint32_t total_payload_bytes;  ///< 总载荷字节数
} tcp_parser_stats_t;

/**
 * @brief 获取默认解析器配置
 * 
 * @return tcp_parser_config_t 默认配置
 */
static inline tcp_parser_config_t tcp_parser_get_default_config(void)
{
    tcp_parser_config_t config = {
        .enable_checksum_verify = false,  // 默认不验证校验和（节省CPU）
        .parse_payload = true,
    };
    return config;
}

/**
 * @brief 初始化TCP解析器
 * 
 * @param config 配置参数（NULL则使用默认配置）
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t tcp_parser_init(const tcp_parser_config_t *config);

/**
 * @brief 注册TCP数据回调函数
 * 
 * @param callback 回调函数
 * @param user_data 用户数据（会传递给回调函数）
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t tcp_parser_register_callback(tcp_data_callback_t callback, void *user_data);

/**
 * @brief 解析TCP报文
 * 
 * @param data TCP报文数据（包含IP头之后的数据）
 * @param length 数据长度
 * @param info 输出解析结果（可选）
 * @return true 解析成功，false 解析失败
 */
bool tcp_parser_parse(const uint8_t *data, size_t length, tcp_packet_info_t *info);

/**
 * @brief 将TCP报文解析为字符串（用于调试）
 * 
 * @param info 解析后的TCP报文信息
 * @param buffer 输出缓冲区
 * @param buffer_size 缓冲区大小
 * @return char* 返回buffer指针
 */
char* tcp_parser_to_string(const tcp_packet_info_t *info, char *buffer, size_t buffer_size);

/**
 * @brief 获取解析器统计信息
 * 
 * @param stats 输出统计信息
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t tcp_parser_get_stats(tcp_parser_stats_t *stats);

/**
 * @brief 重置解析器统计信息
 */
void tcp_parser_reset_stats(void);

/**
 * @brief 释放解析器资源
 */
void tcp_parser_deinit(void);

#ifdef __cplusplus
}
#endif

/**
 * @brief TCP 客户端配置
 */
typedef struct {
    char server_ip[16];          ///< 服务器 IP 地址
    uint16_t server_port;        ///< 服务器端口
    uint32_t reconnect_interval_ms; ///< 重连间隔（毫秒）
    bool auto_reconnect;         ///< 是否自动重连
} tcp_client_config_t;

/**
 * @brief 获取默认 TCP 客户端配置
 */
static inline tcp_client_config_t tcp_client_get_default_config(void)
{
    tcp_client_config_t cfg = {
        .server_ip = "192.168.1.2",
        .server_port = 8080,
        .reconnect_interval_ms = 5000,
        .auto_reconnect = true,
    };
    return cfg;
}

/**
 * @brief 启动 TCP 客户端（阻塞式任务，会创建内部任务）
 * 
 * @param config 客户端配置
 * @param callback 接收到数据后的回调（可选，若为 NULL 则仅解析并打印日志）
 * @param user_data 用户数据指针，传给回调
 * @return esp_err_t 
 */
esp_err_t tcp_client_start(const tcp_client_config_t *config,
                           tcp_data_callback_t callback,
                           void *user_data);

/**
 * @brief 停止 TCP 客户端
 */
void tcp_client_stop(void);

/**
 * @brief 检查客户端是否已连接
 */
bool tcp_client_is_connected(void);

/**
 * @brief 向服务器发送数据（可选）
 */
esp_err_t tcp_client_send(const uint8_t *data, size_t len);