/**
 * @file status_led.c
 * @brief 网络状态指示灯实现
 *
 * 使用单颗板载 WS2812B RGB LED (GPIO48), 通过 RMT 单总线驱动。
 * 硬件参数:
 *   - LED 型号: XL-5050RGBC-WS2812B (5V)
 *   - 数据引脚: GPIO48
 *   - 像素数量: 1
 *   - 灯序: GRB
 *   - DO 悬空, 无级联
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

/* 默认亮度 0-255 */
#define LED_DEFAULT_BRIGHTNESS  64

/* WS2812B 时序 (RMT 10MHz 分辨率):
 * 0 码: 高 0.3us (3 ticks) + 低 0.9us (9 ticks)
 * 1 码: 高 0.7us (7 ticks) + 低 0.6us (6 ticks)
 * RESET: 低电平 >50us
 */
#define RMT_RESOLUTION_HZ       (10 * 1000 * 1000)
#define WS2812_T0H_TICKS        3
#define WS2812_T0L_TICKS        9
#define WS2812_T1H_TICKS        7
#define WS2812_T1L_TICKS        6

/* 数据交换检测阈值 */
#define DATA_EXCHANGE_HOLD_MS   500
#define DATA_BLINK_PERIOD_MS    200

/* 当前配置 */
static status_led_config_t s_cfg = {0};
static bool s_initialized = false;

/* RMT */
static rmt_channel_handle_t s_led_channel = NULL;
static rmt_encoder_handle_t s_copy_encoder = NULL;

/* 状态 */
static status_led_state_t s_current_state = STATUS_LED_DISCONNECTED;
static bool s_blue_blink_on = false;
static bool s_data_exchange_active = false;

/* 数据交换时间戳保护 */
static int64_t s_last_data_time_ms = 0;
static int64_t s_last_blink_toggle_ms = 0;
static portMUX_TYPE s_data_mux = portMUX_INITIALIZER_UNLOCKED;

/* RGB 颜色表 (0-255), 最终会根据 brightness 缩放, 按 GRB 顺序发送 */
static const uint8_t s_color_table[LED_COLOR_MAX][3] = {
    [LED_COLOR_OFF]        = {0,   0,   0},
    [LED_COLOR_RED]        = {255, 0,   0},
    [LED_COLOR_GREEN]      = {0,   255, 0},
    [LED_COLOR_BLUE]       = {0,   0,   255},    /* 深蓝 */
    [LED_COLOR_YELLOW]     = {255, 200, 0},      /* 深黄 */
    [LED_COLOR_YELLOW_DIM] = {255, 255, 100},    /* 浅黄 */
    [LED_COLOR_PURPLE]     = {255, 0,   255},
    [LED_COLOR_BLUE_DIM]   = {100, 150, 255},    /* 浅蓝 */
    [LED_COLOR_WHITE]      = {255, 255, 255},
};

/**
 * @brief 发送一个 24bit GRB 数据给 WS2812B
 */
static void ws2812_send_grb(uint8_t g, uint8_t r, uint8_t b)
{
    if (s_led_channel == NULL) {
        return;
    }

    rmt_symbol_word_t symbols[24];
    uint8_t grb[3] = {g, r, b};

    for (int byte_idx = 0; byte_idx < 3; byte_idx++) {
        uint8_t byte = grb[byte_idx];
        for (int bit_idx = 0; bit_idx < 8; bit_idx++) {
            int sym_idx = byte_idx * 8 + bit_idx;
            bool bit = (byte >> (7 - bit_idx)) & 0x01;
            if (bit) {
                symbols[sym_idx].level0 = 1;
                symbols[sym_idx].duration0 = WS2812_T1H_TICKS;
                symbols[sym_idx].level1 = 0;
                symbols[sym_idx].duration1 = WS2812_T1L_TICKS;
            } else {
                symbols[sym_idx].level0 = 1;
                symbols[sym_idx].duration0 = WS2812_T0H_TICKS;
                symbols[sym_idx].level1 = 0;
                symbols[sym_idx].duration1 = WS2812_T0L_TICKS;
            }
        }
    }

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    rmt_transmit(s_led_channel, s_copy_encoder, symbols, sizeof(symbols), &tx_config);
    rmt_tx_wait_all_done(s_led_channel, pdMS_TO_TICKS(100));

    /* RESET: 低电平保持 >50us */
    gpio_set_level(s_cfg.gpio_num, 0);
    esp_rom_delay_us(300);
}

/**
 * @brief 设置颜色, 带亮度缩放
 */
static void apply_color(led_color_t color)
{
    if (!s_initialized || s_led_channel == NULL) {
        return;
    }

    if (color < 0 || color >= LED_COLOR_MAX) {
        color = LED_COLOR_OFF;
    }

    const uint8_t *rgb = s_color_table[color];
    uint8_t r = ((uint16_t)rgb[0] * s_cfg.brightness) / 255;
    uint8_t g = ((uint16_t)rgb[1] * s_cfg.brightness) / 255;
    uint8_t b = ((uint16_t)rgb[2] * s_cfg.brightness) / 255;

    ws2812_send_grb(g, r, b);
}

status_led_config_t status_led_get_default_config(void)
{
    status_led_config_t cfg = {
        .gpio_num = 48,
        .brightness = LED_DEFAULT_BRIGHTNESS,
    };
    return cfg;
}

esp_err_t status_led_init(const status_led_config_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_initialized) {
        return ESP_OK;
    }

    memcpy(&s_cfg, cfg, sizeof(s_cfg));

    if (s_cfg.gpio_num < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_cfg.brightness == 0) {
        s_cfg.brightness = LED_DEFAULT_BRIGHTNESS;
    }

    /* 配置 GPIO */
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << s_cfg.gpio_num),
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(s_cfg.gpio_num, 0);

    /* 创建 RMT TX 通道 */
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = s_cfg.gpio_num,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .trans_queue_depth = 4,
    };
    esp_err_t err = rmt_new_tx_channel(&tx_chan_config, &s_led_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT channel");
        return err;
    }

    /* 创建 copy encoder */
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

    s_initialized = true;
    apply_color(LED_COLOR_OFF);
    status_led_set_state(STATUS_LED_DISCONNECTED);

    ESP_LOGI(TAG, "WS2812B status LED initialized on GPIO%d, brightness=%u",
             s_cfg.gpio_num, s_cfg.brightness);
    return ESP_OK;
}

void status_led_set_color(led_color_t color)
{
    apply_color(color);
}

void status_led_set_state(status_led_state_t state)
{
    s_current_state = state;

    led_color_t color = LED_COLOR_OFF;
    switch (state) {
    case STATUS_LED_DISCONNECTED:
        color = LED_COLOR_RED;
        break;
    case STATUS_LED_CONNECTED:
        color = LED_COLOR_GREEN;
        break;
    case STATUS_LED_DATA_EXCHANGE:
        color = LED_COLOR_BLUE;
        break;
    default:
        color = LED_COLOR_OFF;
        break;
    }

    apply_color(color);
}

void status_led_update(void)
{
    bool link_up = wiznet_manager_is_link_up();
    int64_t now_ms = esp_timer_get_time() / 1000;

    /* 未拿到 IP 也视为未连接 (红色) */
    bool has_ip = false;
    if (link_up) {
        esp_netif_ip_info_t ip_info = {0};
        if (wiznet_manager_get_ip_info(&ip_info) == ESP_OK && ip_info.ip.addr != 0) {
            has_ip = true;
        }
    }

    if (!link_up || !has_ip) {
        s_data_exchange_active = false;
        s_blue_blink_on = false;
        if (s_current_state != STATUS_LED_DISCONNECTED) {
            status_led_set_state(STATUS_LED_DISCONNECTED);
        }
        return;
    }

    /* 检测数据交换活跃 */
    bool exchange_active;
    portENTER_CRITICAL(&s_data_mux);
    exchange_active = (now_ms - s_last_data_time_ms) < DATA_EXCHANGE_HOLD_MS;
    portEXIT_CRITICAL(&s_data_mux);

    if (exchange_active) {
        /* 蓝色闪烁: 200ms 周期在深蓝/浅蓝之间切换 */
        if ((now_ms - s_last_blink_toggle_ms) >= DATA_BLINK_PERIOD_MS) {
            s_blue_blink_on = !s_blue_blink_on;
            s_last_blink_toggle_ms = now_ms;
            apply_color(s_blue_blink_on ? LED_COLOR_BLUE : LED_COLOR_BLUE_DIM);
        }
        s_current_state = STATUS_LED_DATA_EXCHANGE;
        s_data_exchange_active = true;
    } else {
        s_blue_blink_on = false;
        s_data_exchange_active = false;
        if (s_current_state != STATUS_LED_CONNECTED) {
            status_led_set_state(STATUS_LED_CONNECTED);
        }
    }
}

void status_led_notify_rx(uint32_t bytes)
{
    int64_t now_ms = esp_timer_get_time() / 1000;
    portENTER_CRITICAL(&s_data_mux);
    s_last_data_time_ms = now_ms;
    portEXIT_CRITICAL(&s_data_mux);
    (void)bytes;
}

void status_led_notify_tx(uint32_t bytes)
{
    int64_t now_ms = esp_timer_get_time() / 1000;
    portENTER_CRITICAL(&s_data_mux);
    s_last_data_time_ms = now_ms;
    portEXIT_CRITICAL(&s_data_mux);
    (void)bytes;
}

void status_led_deinit(void)
{
    if (!s_initialized) {
        return;
    }

    apply_color(LED_COLOR_OFF);

    if (s_led_channel != NULL) {
        rmt_disable(s_led_channel);
        rmt_del_channel(s_led_channel);
        s_led_channel = NULL;
    }
    if (s_copy_encoder != NULL) {
        rmt_del_encoder(s_copy_encoder);
        s_copy_encoder = NULL;
    }

    s_initialized = false;
    memset(&s_cfg, 0, sizeof(s_cfg));
}
