/**
 * @file status_led.c
 * @brief 网络状态指示灯实现
 *
 * 使用RMT驱动控制WS2812 RGB LED
 * ESP32-S3-DevKitC-1 板载RGB LED连接到GPIO48
 */

#include "status_led.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "wiznet_manager.h"

static const char *TAG = "status_led";

// 亮度限制 (0-255)
#define LED_BRIGHTNESS  64

// 数据交换检测阈值：两次采样间包数变化超过此值视为有数据交换
#define DATA_EXCHANGE_THRESHOLD_PKTS   3

// 数据交换保持时间（毫秒）
#define DATA_EXCHANGE_HOLD_MS          500

// 数据交换蓝色闪烁周期（毫秒）
#define DATA_BLINK_PERIOD_MS           200

static rmt_channel_handle_t s_led_channel = NULL;
static rmt_encoder_handle_t s_copy_encoder = NULL;
static int s_gpio_num = -1;
static status_led_state_t s_current_state = STATUS_LED_DISCONNECTED;

// 数据交换检测
static int64_t s_last_data_time_ms = 0;
static bool s_data_exchange_active = false;

// 全局流量计数器 (TCP 任务通过 notify_rx/notify_tx 更新)
static volatile uint32_t s_rx_bytes_total = 0;
static volatile uint32_t s_tx_bytes_total = 0;
static int64_t s_last_blink_toggle_ms = 0;
static bool s_blue_blink_on = false;

/**
 * @brief 通过RMT发送WS2812数据
 *
 * 使用简单的copy_encoder + 预构建的symbol数组
 */
static void ws2812_send_color(uint8_t r, uint8_t g, uint8_t b)
{
    if (s_led_channel == NULL) {
        return;
    }

    // 每个bit对应一个RMT symbol: {level, duration}
    // 0 bit: 1@0.3us + 0@0.9us  (3 ticks + 9 ticks @ 10MHz)
    // 1 bit: 1@0.7us + 0@0.6us  (7 ticks + 6 ticks @ 10MHz)
    rmt_symbol_word_t symbols[24];
    uint8_t grb[3] = {g, r, b};

    for (int byte_idx = 0; byte_idx < 3; byte_idx++) {
        uint8_t byte = grb[byte_idx];
        for (int bit_idx = 0; bit_idx < 8; bit_idx++) {
            int sym_idx = byte_idx * 8 + bit_idx;
            bool bit = (byte >> (7 - bit_idx)) & 0x01;
            if (bit) {
                symbols[sym_idx].level0 = 1;
                symbols[sym_idx].duration0 = 7;
                symbols[sym_idx].level1 = 0;
                symbols[sym_idx].duration1 = 6;
            } else {
                symbols[sym_idx].level0 = 1;
                symbols[sym_idx].duration0 = 3;
                symbols[sym_idx].level1 = 0;
                symbols[sym_idx].duration1 = 9;
            }
        }
    }

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    rmt_transmit(s_led_channel, s_copy_encoder, symbols, sizeof(symbols), &tx_config);
    rmt_tx_wait_all_done(s_led_channel, pdMS_TO_TICKS(100));

    // RESET: 低电平保持 >50us
    gpio_set_level(s_gpio_num, 0);
    esp_rom_delay_us(300);

}

esp_err_t status_led_init(int gpio_num)
{
    if (s_led_channel != NULL) {
        return ESP_OK;
    }

    s_gpio_num = gpio_num;

    // 配置GPIO
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << gpio_num),
    };
    gpio_config(&io_conf);
    gpio_set_level(gpio_num, 0);

    // 创建RMT TX通道，10MHz分辨率
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = gpio_num,
        .mem_block_symbols = 64,
        .resolution_hz = 10 * 1000 * 1000,
        .trans_queue_depth = 4,
    };
    esp_err_t err = rmt_new_tx_channel(&tx_chan_config, &s_led_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT channel");
        return err;
    }

    // 创建copy encoder，直接发送symbol数据
    rmt_copy_encoder_config_t encoder_config = {};
    err = rmt_new_copy_encoder(&encoder_config, &s_copy_encoder);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create copy encoder");
        rmt_del_channel(s_led_channel);
        s_led_channel = NULL;
        return err;
    }

    err = rmt_enable(s_led_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT channel");
        rmt_del_encoder(s_copy_encoder);
        s_copy_encoder = NULL;
        rmt_del_channel(s_led_channel);
        s_led_channel = NULL;
        return err;
    }

    // 初始状态：关闭LED
    ws2812_send_color(0, 0, 0);
    status_led_set_state(STATUS_LED_DISCONNECTED);

    ESP_LOGI(TAG, "Status LED initialized on GPIO%d", gpio_num);
    return ESP_OK;
}

void status_led_set_state(status_led_state_t state)
{
    s_current_state = state;

    uint8_t r = 0, g = 0, b = 0;
    switch (state) {
    case STATUS_LED_DISCONNECTED:
        r = LED_BRIGHTNESS;
        break;
    case STATUS_LED_CONNECTED:
        g = LED_BRIGHTNESS;
        break;
    case STATUS_LED_DATA_EXCHANGE:
        b = LED_BRIGHTNESS;
        break;
    default:
        break;
    }

    ws2812_send_color(r, g, b);
}

void status_led_update(void)
{
    /* 按项目参数文档, 状态优先级:
     *   1. 未连接 / 无 IP → 红色 (最高优先级)
     *   2. 已连接 + 有 IP + 数据交换活跃 (近 500ms 有收/发) → 蓝色闪烁 (200ms 周期)
     *   3. 已连接 + 有 IP + 无数据交换 → 绿色常亮
     */
    bool link_up = wiznet_manager_is_link_up();
    int64_t now_ms = esp_timer_get_time() / 1000;

    if (!link_up) {
        s_data_exchange_active = false;
        s_blue_blink_on = false;
        if (s_current_state != STATUS_LED_DISCONNECTED) {
            status_led_set_state(STATUS_LED_DISCONNECTED);
        }
        return;
    }

    /* 检测数据交换活跃: 距上次 RX/TX 通知 < DATA_EXCHANGE_HOLD_MS */
    bool exchange_active = (now_ms - s_last_data_time_ms) < DATA_EXCHANGE_HOLD_MS;

    if (exchange_active) {
        /* 蓝色闪烁: 200ms 周期切换 */
        if ((now_ms - s_last_blink_toggle_ms) >= DATA_BLINK_PERIOD_MS) {
            s_blue_blink_on = !s_blue_blink_on;
            s_last_blink_toggle_ms = now_ms;
            if (s_blue_blink_on) {
                ws2812_send_color(0, 0, LED_BRIGHTNESS);  /* 蓝 */
            } else {
                ws2812_send_color(0, 0, 0);                /* 灭 */
            }
        }
        s_current_state = STATUS_LED_DATA_EXCHANGE;
        s_data_exchange_active = true;
    } else {
        /* 无数据交换: 绿色常亮 */
        s_blue_blink_on = false;
        s_data_exchange_active = false;
        if (s_current_state != STATUS_LED_CONNECTED) {
            status_led_set_state(STATUS_LED_CONNECTED);
        }
    }
}

void status_led_notify_rx(uint32_t bytes)
{
    s_rx_bytes_total += bytes;
    s_last_data_time_ms = esp_timer_get_time() / 1000;
}

void status_led_notify_tx(uint32_t bytes)
{
    s_tx_bytes_total += bytes;
    s_last_data_time_ms = esp_timer_get_time() / 1000;
}

void status_led_deinit(void)
{
    if (s_led_channel != NULL) {
        ws2812_send_color(0, 0, 0);
        rmt_disable(s_led_channel);
        rmt_del_channel(s_led_channel);
        s_led_channel = NULL;
    }
    if (s_copy_encoder != NULL) {
        rmt_del_encoder(s_copy_encoder);
        s_copy_encoder = NULL;
    }
    s_gpio_num = -1;
}
