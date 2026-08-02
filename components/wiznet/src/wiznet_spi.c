/**
 * @file wiznet_spi.c
 * @brief W5500 SPI 回调桥接层 (ioLibrary <-> ESP-IDF SPI 驱动)
 *
 * 实现 ioLibrary 要求的以下回调 (通过 reg_wizchip_*_cbfunc 注册):
 *   - WIZCHIP_SELECT / WIZCHIP_DESELECT            (CS 控制)
 *   - wiznet_spi_read_byte / wiznet_spi_write_byte (单字节)
 *   - wiznet_spi_read_buf / wiznet_spi_write_buf   (突发 - 提升吞吐量)
 *
 * SPI 传输格式 (W5500 VDM 模式):
 *   [addr_high][addr_mid|CB][addr_low] [data_0][data_1]...[data_n]
 *   CB = 控制字节, R/W 位在 bit2 (1=write, 0=read), 其余 0
 *
 * 所有 SPI 传输都封装在单个 spi_device_transmit 调用中,
 * 配合 DMA 通道可实现接近 SPI 时钟上限的吞吐。
 *
 * 注意: 不能命名为 WIZCHIP_READ/WRITE/WRITE_BUF/READ_BUF, 这些
 *      名字已被 w5500.c 中的 ioLibrary 内部函数占用.
 */

#include "wiznet_spi.h"
#include "wiznet_conf.h"
#include "wizchip_conf.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "wiznet_spi";

/* ========== 模块内部状态 ========== */
static spi_device_handle_t s_spi_dev     = NULL;
static int                 s_cs_gpio     = -1;
static int                 s_rst_gpio    = -1;
static int                 s_int_gpio    = -1;
static SemaphoreHandle_t   s_cs_mutex    = NULL;   /* 保护 W5500 互斥访问 */
static int                 s_cs_depth    = 0;      /* 递归互斥锁嵌套深度 */

/* INT 中断事件: W5500 拉低 INT → ISR 置位 → 任务检测后处理
 * 不用事件队列 (避免多任务同步复杂性), 直接用 volatile + ISR 标志 */
static volatile bool     s_int_pending   = false;
static volatile uint32_t s_isr_count     = 0;     /* ISR 触发计数 (调试用) */
static volatile uint32_t s_last_isr_us   = 0;     /* 上次 ISR 时间戳 (去抖) */

/* ISR: 仅置位, 不做 I/O (SPI 读写不能在 ISR 中进行)
 * 简单去抖: 两次 ISR 间隔 < 1ms 视为抖动, 忽略后续
 * (W5500 一次事件 INT 脉宽 ~1µs, 抖动通常是 < 100µs) */
static void IRAM_ATTR w5500_int_isr(void *arg)
{
    (void)arg;
    uint32_t now = esp_timer_get_time();
    if (now - s_last_isr_us < 1000) {   /* < 1ms 视为抖动 */
        return;
    }
    s_last_isr_us = now;
    s_int_pending = true;
    s_isr_count++;
}

/* ========== 临界区: ioLibrary 调用回调时进入/退出 ========== */
static void wiznet_cris_enter(void)
{
    if (s_cs_mutex) {
        /* 用 100ms 超时而不是 portMAX_DELAY, 防止其他任务卡死 SPI
         * 时本任务永远等待. 超时则放弃本次操作, 由调用方处理. */
        if (xSemaphoreTakeRecursive(s_cs_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            s_cs_depth++;
        } else {
            ESP_LOGE(TAG, "CS mutex timeout (100ms) - another task stuck in SPI");
        }
    }
}

static void wiznet_cris_exit(void)
{
    if (s_cs_mutex && s_cs_depth > 0) {
        s_cs_depth--;
        xSemaphoreGiveRecursive(s_cs_mutex);
    }
}

/* ========== CS 控制 (ioLibrary 回调) ========== */
void WIZCHIP_SELECT(void)
{
    gpio_set_level(s_cs_gpio, 0);
}

void WIZCHIP_DESELECT(void)
{
    gpio_set_level(s_cs_gpio, 1);
}

/* ========== 底层 SPI 传输 ========== */
static esp_err_t spi_xfer(const uint8_t *tx, uint8_t *rx, int len)
{
    spi_transaction_t t = {
        .length    = len * 8,   /* 单位: bit */
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    /* SPI_DMA_DISABLED 模式下, 必须用 polling_transmit (transmit 内部会失败) */
    return spi_device_polling_transmit(s_spi_dev, &t);
}

/* ========== 字节级 SPI 回调 (注册到 ioLibrary) ========== */
static uint8_t wiznet_spi_read_byte(void)
{
    uint8_t tx[1] = {0xFF};
    uint8_t rx[1] = {0x00};
    spi_xfer(tx, rx, 1);
    return rx[0];
}

static void wiznet_spi_write_byte(uint8_t wb)
{
    uint8_t tx[1] = {wb};
    uint8_t rx[1] = {0x00};
    spi_xfer(tx, rx, 1);
}

/* ========== 突发级 SPI 回调 (注册到 ioLibrary, 显著提升吞吐) ==========
 * W5500 VDM 模式下, WIZCHIP_READ_BUF/WRITE_BUF 内部会先调用
 * WIZCHIP_WRITE 发送 3 字节地址, 然后才是数据.
 * 数据部分使用单次 spi_device_transmit + DMA 完成, 避免逐字节开销. */
static void wiznet_spi_read_buf(uint8_t *pBuf, uint16_t len)
{
    if (len == 0) return;
    uint8_t dummy = 0xFF;
    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = &dummy,
        .rx_buffer = pBuf,
    };
    esp_err_t ret = spi_device_polling_transmit(s_spi_dev, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "READ_BUF failed: %s", esp_err_to_name(ret));
    }
}

static void wiznet_spi_write_buf(uint8_t *pBuf, uint16_t len)
{
    if (len == 0) return;
    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = pBuf,
        .rx_buffer = NULL,
    };
    esp_err_t ret = spi_device_polling_transmit(s_spi_dev, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WRITE_BUF failed: %s", esp_err_to_name(ret));
    }
}

/* ========== 公共 API: 初始化 SPI 桥接层 ========== */
esp_err_t wiznet_spi_init(const wiznet_spi_config_t *cfg)
{
    if (cfg == NULL || s_spi_dev != NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 创建互斥锁 (用于 ioLibrary 临界区) */
    s_cs_mutex = xSemaphoreCreateRecursiveMutex();
    if (s_cs_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_cs_gpio  = cfg->cs_gpio;
    s_rst_gpio = cfg->rst_gpio;
    s_int_gpio = cfg->int_gpio;

    /* 配置 CS 引脚 */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << s_cs_gpio),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(s_cs_gpio, 1);   /* CS 默认拉高 */

    /* 可选: 复位引脚 */
    if (s_rst_gpio >= 0) {
        gpio_config_t rst_conf = {
            .pin_bit_mask = (1ULL << s_rst_gpio),
            .mode         = GPIO_MODE_OUTPUT,
        };
        ESP_ERROR_CHECK(gpio_config(&rst_conf));
        gpio_set_level(s_rst_gpio, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(s_rst_gpio, 1);
        vTaskDelay(pdMS_TO_TICKS(50));   /* 等待 W5500 自举 */
    }

    /* 添加 SPI 设备 */
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = cfg->clock_hz,
        .mode           = 0,                 /* W5500 SPI mode 0 */
        .spics_io_num   = -1,                /* 手动控制 CS (避免与 WIZCHIP_SELECT 冲突) */
        .queue_size     = 7,
        .flags          = SPI_DEVICE_NO_DUMMY,
    };
    esp_err_t ret = spi_bus_add_device(cfg->spi_host, &devcfg, &s_spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 注册 ioLibrary 回调 */
    reg_wizchip_cris_cbfunc(wiznet_cris_enter, wiznet_cris_exit);
    reg_wizchip_cs_cbfunc(WIZCHIP_SELECT, WIZCHIP_DESELECT);
    reg_wizchip_spi_cbfunc(wiznet_spi_read_byte, wiznet_spi_write_byte);
    reg_wizchip_spiburst_cbfunc(wiznet_spi_read_buf, wiznet_spi_write_buf);

    /* 配置 INT 引脚 (下降沿触发, W5500 INT 低电平有效) */
    if (s_int_gpio >= 0) {
        gpio_config_t int_conf = {
            .pin_bit_mask = (1ULL << s_int_gpio),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_ENABLE,   /* 默认高, W5500 拉低触发 */
            .intr_type    = GPIO_INTR_NEGEDGE,
        };
        esp_err_t ir = gpio_config(&int_conf);
        if (ir != ESP_OK) {
            ESP_LOGE(TAG, "INT gpio_config failed: %s", esp_err_to_name(ir));
            return ir;
        }
        esp_err_t isr_ret = gpio_install_isr_service(0);
        if (isr_ret != ESP_OK && isr_ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "gpio_install_isr_service: %s", esp_err_to_name(isr_ret));
            return isr_ret;
        }
        esp_err_t h_ret = gpio_isr_handler_add(s_int_gpio, w5500_int_isr, NULL);
        if (h_ret != ESP_OK) {
            ESP_LOGE(TAG, "gpio_isr_handler_add: %s", esp_err_to_name(h_ret));
            return h_ret;
        }
        ESP_LOGI(TAG, "W5500 INT on GPIO%d (falling edge)", s_int_gpio);
    }

    /* SPI 初始化完成, 不再打印细节, 留给 wiznet_mgr 统一输出 */
    return ESP_OK;
}

spi_device_handle_t wiznet_spi_get_device(void)
{
    return s_spi_dev;
}

/* 供任务查询: W5500 是否有挂起事件 */
bool wiznet_spi_check_int(void)
{
    if (s_int_pending) {
        s_int_pending = false;
        return true;
    }
    return false;
}
