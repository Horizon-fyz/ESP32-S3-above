/**
 * @file gps.c
 * @brief GPS NMEA 解析 + PPS 计数实现
 *
 * 数据流:
 *   UART1 --(字节流)--> 行缓冲 --(整句)--> 校验和 --> RMC/GGA 字段解析 --> 快照
 *   PPS 上升沿 --(中断)--> 计数 + 时间戳
 *
 * 只解析真正需要的字段, 不做完整 NMEA 支持。
 */

#include "gps.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "gps";

#define GPS_LINE_MAX      128      /* 单条 NMEA 语句最长 (标准 82, 留余量) */
#define GPS_TASK_STACK    4096
#define GPS_TASK_PRIO     4
#define GPS_READ_TIMEOUT  100      /* ms */

static gps_config_t      s_cfg;
static bool              s_ready = false;
static SemaphoreHandle_t s_lock  = NULL;
static gps_data_t        s_data;

/* PPS: 中断里只做最小动作 */
static volatile uint32_t s_pps_count   = 0;
static volatile uint64_t s_pps_last_us = 0;

/* ==================== PPS 中断 ==================== */
static void IRAM_ATTR pps_isr(void *arg)
{
    s_pps_count++;
    s_pps_last_us = (uint64_t)esp_timer_get_time();
}

/* ==================== NMEA 基础 ==================== */

/* 校验和: '$' 与 '*' 之间所有字符逐字节异或 */
static bool nmea_checksum_ok(const char *line)
{
    if (line[0] != '$') return false;
    const char *star = strchr(line, '*');
    if (star == NULL || star - line < 2) return false;

    uint8_t sum = 0;
    for (const char *p = line + 1; p < star; p++) {
        sum ^= (uint8_t)(*p);
    }
    unsigned got = 0;
    if (sscanf(star + 1, "%2x", &got) != 1) return false;
    return (uint8_t)got == sum;
}

/* 就地切分: 逗号和 '*' 都当分隔符; 返回字段个数 */
static int nmea_split(char *s, char *fields[], int max)
{
    int n = 0;
    char *p = s;
    if (max <= 0) return 0;
    fields[n++] = p;
    while (*p != '\0' && n < max) {
        if (*p == ',' || *p == '*') {
            *p = '\0';
            if (n < max) {
                fields[n++] = p + 1;
            }
        }
        p++;
    }
    return n;
}

/* "ddmm.mmmm" + 'N'/'S' -> 十进制度 */
static double nmea_latlon(const char *val, const char *hemi)
{
    if (val == NULL || val[0] == '\0') return 0.0;
    double raw = atof(val);
    int    deg = (int)(raw / 100.0);
    double min = raw - (double)deg * 100.0;
    double d   = (double)deg + min / 60.0;
    if (hemi != NULL && (*hemi == 'S' || *hemi == 'W')) d = -d;
    return d;
}

/* "hhmmss.ss" -> 时分秒 */
static void nmea_parse_time(const char *s, gps_data_t *d)
{
    if (s == NULL || strlen(s) < 6) return;
    d->hour   = (uint8_t)((s[0] - '0') * 10 + (s[1] - '0'));
    d->minute = (uint8_t)((s[2] - '0') * 10 + (s[3] - '0'));
    d->second = (uint8_t)((s[4] - '0') * 10 + (s[5] - '0'));
    d->time_valid = true;
}

/* "ddmmyy" -> 年月日 */
static void nmea_parse_date(const char *s, gps_data_t *d)
{
    if (s == NULL || strlen(s) < 6) return;
    d->day   = (uint8_t)((s[0] - '0') * 10 + (s[1] - '0'));
    d->month = (uint8_t)((s[2] - '0') * 10 + (s[3] - '0'));
    int yy   = (s[4] - '0') * 10 + (s[5] - '0');
    d->year  = (uint16_t)(2000 + yy);
}

static void handle_rmc(char *fields[], int n)
{
    /* $--RMC,time,status,lat,N,lon,E,speed_knots,course,date,... */
    if (n < 10) return;

    gps_data_t d = s_data;              /* 在旧快照上更新 */
    nmea_parse_time(fields[1], &d);
    d.valid = (fields[2][0] == 'A');

    if (d.valid) {
        d.latitude  = nmea_latlon(fields[3], fields[4]);
        d.longitude = nmea_latlon(fields[5], fields[6]);
        d.speed_kmh = (float)(atof(fields[7]) * 1.852);   /* 节 -> km/h */
        d.course_deg = (float)atof(fields[8]);
    }
    nmea_parse_date(fields[9], &d);
    d.last_rmc_us  = (uint64_t)esp_timer_get_time();
    d.sentence_cnt++;

    if (s_lock && xSemaphoreTake(s_lock, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_data = d;
        xSemaphoreGive(s_lock);
    }
}

static void handle_gga(char *fields[], int n)
{
    /* $--GGA,time,lat,N,lon,E,quality,sats,hdop,alt,M,... */
    if (n < 10) return;
    if (s_lock && xSemaphoreTake(s_lock, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_data.satellites = (uint8_t)atoi(fields[7]);
        s_data.hdop       = (float)atof(fields[8]);
        s_data.altitude_m = (float)atof(fields[9]);
        s_data.sentence_cnt++;
        xSemaphoreGive(s_lock);
    }
}

/* 处理完整一行 (可能带 \r) */
static void handle_line(char *line)
{
    /* 去掉行尾 \r */
    size_t len = strlen(line);
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n')) {
        line[--len] = '\0';
    }
    if (len < 7 || line[0] != '$') return;

    if (!nmea_checksum_ok(line)) {
        if (s_lock && xSemaphoreTake(s_lock, pdMS_TO_TICKS(10)) == pdTRUE) {
            s_data.err_cnt++;
            xSemaphoreGive(s_lock);
        }
        return;
    }

    char *f[24] = {0};
    int   n = nmea_split(line, f, 24);
    if (n < 2 || f[0] == NULL) return;

    /* f[0] 形如 "$GNRMC" / "$GPRMC" / "$GNGGA"
     * 只看后 3 个字符判断语句类型, 兼容 GN/GP/BD/GL 等前缀 */
    const char *type = f[0];
    size_t tl = strlen(type);
    if (tl < 3) return;
    const char *kind = type + tl - 3;

    if (strcmp(kind, "RMC") == 0) {
        handle_rmc(f, n);
    } else if (strcmp(kind, "GGA") == 0) {
        handle_gga(f, n);
    }
}

/* ==================== 解析任务 ==================== */
static void gps_task(void *pvParameters)
{
    static char line[GPS_LINE_MAX];
    int  idx = 0;
    uint8_t buf[128];

    ESP_LOGI(TAG, "解析任务启动, 等待 NMEA 数据...");

    while (1) {
        int n = uart_read_bytes(s_cfg.uart_num, buf, sizeof(buf),
                                pdMS_TO_TICKS(GPS_READ_TIMEOUT));
        for (int i = 0; i < n; i++) {
            char c = (char)buf[i];
            if (c == '\n') {
                line[idx] = '\0';
                if (idx > 0) {
                    handle_line(line);
                }
                idx = 0;
            } else if (idx < GPS_LINE_MAX - 1) {
                line[idx++] = c;
            } else {
                idx = 0;   /* 超长行: 丢弃重新同步 */
            }
        }
    }
}

/* ==================== 对外接口 ==================== */

gps_config_t gps_get_default_config(void)
{
    gps_config_t cfg = {
        .uart_num = UART_NUM_1,
        .tx_gpio  = 2,     /* v5.10 换板: 左排相邻 2/1/3 */
        .rx_gpio  = 1,
        .pps_gpio = 3,
        .baud     = 9600,
    };
    return cfg;
}

esp_err_t gps_init(const gps_config_t *cfg)
{
    if (cfg == NULL) return ESP_ERR_INVALID_ARG;
    if (s_ready) {
        ESP_LOGW(TAG, "已初始化, 跳过");
        return ESP_OK;
    }
    s_cfg = *cfg;
    memset(&s_data, 0, sizeof(s_data));

    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        if (s_lock == NULL) return ESP_ERR_NO_MEM;
    }

    /* 1. UART */
    uart_config_t ucfg = {
        .baud_rate  = (int)cfg->baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t ret = uart_driver_install(cfg->uart_num, 2048, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install 失败: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = uart_param_config(cfg->uart_num, &ucfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config 失败: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = uart_set_pin(cfg->uart_num, cfg->tx_gpio, cfg->rx_gpio,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin 失败: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "UART%d 就绪: TX=IO%d -> GPS RX, RX=IO%d <- GPS TX, %lu baud",
             cfg->uart_num, cfg->tx_gpio, cfg->rx_gpio,
             (unsigned long)cfg->baud);

    /* 2. PPS 中断 */
    if (cfg->pps_gpio >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << cfg->pps_gpio,
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,   /* 无脉冲时保持低电平 */
            .intr_type    = GPIO_INTR_POSEDGE,
        };
        ret = gpio_config(&io);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "PPS gpio_config 失败: %s", esp_err_to_name(ret));
            return ret;
        }
        /* ISR service 可能已被其他组件装过, 已装则忽略 */
        esp_err_t isr_ret = gpio_install_isr_service(0);
        if (isr_ret != ESP_OK && isr_ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "gpio_install_isr_service 失败: %s", esp_err_to_name(isr_ret));
            return isr_ret;
        }
        ret = gpio_isr_handler_add((gpio_num_t)cfg->pps_gpio, pps_isr, NULL);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "PPS isr_handler_add 失败: %s", esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "PPS 中断已挂载: IO%d (上升沿)", cfg->pps_gpio);
    }

    /* 3. 解析任务 */
    if (xTaskCreate(gps_task, "gps", GPS_TASK_STACK, NULL, GPS_TASK_PRIO, NULL) != pdPASS) {
        ESP_LOGE(TAG, "创建解析任务失败");
        return ESP_FAIL;
    }

    s_ready = true;
    return ESP_OK;
}

bool gps_is_ready(void)
{
    return s_ready;
}

esp_err_t gps_get_data(gps_data_t *out)
{
    if (out == NULL)     return ESP_ERR_INVALID_ARG;
    if (!s_ready)        return ESP_ERR_INVALID_STATE;
    if (s_lock == NULL)  return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    *out = s_data;
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

uint32_t gps_get_pps_count(void)
{
    return s_pps_count;
}

uint64_t gps_get_pps_last_us(void)
{
    return s_pps_last_us;
}

esp_err_t gps_write(const char *data, size_t len)
{
    if (data == NULL || len == 0) return ESP_ERR_INVALID_ARG;
    if (!s_ready)                 return ESP_ERR_INVALID_STATE;
    int w = uart_write_bytes(s_cfg.uart_num, data, len);
    return (w == (int)len) ? ESP_OK : ESP_FAIL;
}
