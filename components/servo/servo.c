/**
 * @file servo.c
 * @brief PCA9685 驱动实现
 *
 * PCA9685 关键寄存器:
 *   0x00 MODE1      - 通用模式 (RESTART, SLEEP, AI 标志)
 *   0x06 PWM freq   - 预分频 (25MHz / (4096 * freq) - 1)
 *   0x06~0x45       - 16 通道的 LED_ON (2 字节) + LED_OFF (2 字节)
 *
 * 操作流程:
 *   1. 写 MODE1 = 0x10 进入睡眠, 此时才能改 PRE_SCALE
 *   2. 写 PRE_SCALE = round(25e6 / (4096 * freq)) - 1
 *   3. 写 MODE1 = 0xA1 (RESTART + AI auto-increment)
 *   4. 等待 5ms 振荡器稳定
 *   5. 设置各通道 LED_ON=0, LED_OFF=pulse_count
 */

#include "servo.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "servo";

#define I2C_MASTER_NUM    I2C_NUM_1
#define I2C_TIMEOUT_MS    100
#define PCA9685_OSC_HZ    25000000   /* 25 MHz 内部振荡器 */
#define SERVO_MIN_US      1000       /* 0° */
#define SERVO_MAX_US      2000       /* 180° */

static bool           s_initialized = false;
static uint8_t        s_i2c_addr    = 0x70;
static servo_config_t s_cfg;

/* ========== 低层 I2C ========== */
static esp_err_t i2c_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(I2C_MASTER_NUM, s_i2c_addr, buf, sizeof(buf),
                                       I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t i2c_read(uint8_t reg, uint8_t *val)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, s_i2c_addr,
                                         &reg, 1, val, 1,
                                         I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/* 设置某通道: ON=0, OFF=count (12-bit) */
static esp_err_t pca9685_set_pwm(uint8_t channel, uint16_t off_count)
{
    if (channel >= SERVO_CHANNEL_COUNT) return ESP_ERR_INVALID_ARG;
    if (off_count > 4095) off_count = 4095;

    uint8_t reg_base = 0x06 + 4 * channel;
    uint8_t buf[5] = {
        reg_base,
        0, 0,                      /* LED_ON_L, LED_ON_H */
        (uint8_t)(off_count & 0xFF),
        (uint8_t)(off_count >> 8),
    };
    return i2c_master_write_to_device(I2C_MASTER_NUM, s_i2c_addr, buf, sizeof(buf),
                                       I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/* ========== 公共 API ========== */
servo_config_t servo_get_default_config(void)
{
    servo_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.sda_gpio    = 4;
    cfg.scl_gpio    = 5;
    cfg.i2c_freq_hz = 400 * 1000;
    cfg.i2c_addr    = 0x70;
    cfg.pwm_freq_hz = 50;          /* 标准舵机 50Hz */
    return cfg;
}

esp_err_t servo_init(const servo_config_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(&s_cfg, cfg, sizeof(s_cfg));
    s_i2c_addr = cfg->i2c_addr;

    /* 1. 初始化 I2C1 */
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = cfg->sda_gpio,
        .scl_io_num = cfg->scl_gpio,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = cfg->i2c_freq_hz,
    };
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &i2c_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 2. 探测设备 */
    uint8_t mode1 = 0;
    ret = i2c_read(0x00, &mode1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 not found at 0x%02X", s_i2c_addr);
        i2c_driver_delete(I2C_MASTER_NUM);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "PCA9685 found at 0x%02X (MODE1=0x%02X)", s_i2c_addr, mode1);

    /* 3. 进入睡眠 (修改预分频寄存器需要) */
    ret = i2c_write(0x00, 0x10);   /* MODE1.SLEEP=1, RESTART=0 */
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(5));

    /* 4. 设置预分频 -> PWM 频率 */
    /* prescale = round(25e6 / (4096 * freq)) - 1 */
    float prescale_f = (float)PCA9685_OSC_HZ / (4096.0f * cfg->pwm_freq_hz) - 1.0f;
    uint8_t prescale = (uint8_t)(prescale_f + 0.5f);
    if (prescale < 3) prescale = 3;
    if (prescale > 255) prescale = 255;
    ret = i2c_write(0xFE, prescale);
    if (ret != ESP_OK) return ret;
    ESP_LOGI(TAG, "PWM freq=%d Hz, prescale=%d", cfg->pwm_freq_hz, prescale);

    /* 5. 退出睡眠, 启用自动递增 */
    ret = i2c_write(0x00, 0xA1);   /* RESTART=1, AI=1, SLEEP=0 */
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(5));

    /* 6. 关闭所有输出 (全部 OFF) */
    for (uint8_t ch = 0; ch < SERVO_CHANNEL_COUNT; ch++) {
        pca9685_set_pwm(ch, 0);
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Servo controller ready: %d channels @ %d Hz", SERVO_CHANNEL_COUNT, cfg->pwm_freq_hz);
    return ESP_OK;
}

esp_err_t servo_set_pulse_us(uint8_t channel, uint16_t pulse_us)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (channel >= SERVO_CHANNEL_COUNT) return ESP_ERR_INVALID_ARG;

    /* period_us = 1e6 / freq, count = pulse_us / period_us * 4096 */
    uint16_t count = (uint16_t)((uint32_t)pulse_us * s_cfg.pwm_freq_hz * 4096 / 1000000U);
    return pca9685_set_pwm(channel, count);
}

esp_err_t servo_set_angle(uint8_t channel, float angle)
{
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;
    uint16_t pulse = SERVO_MIN_US + (uint16_t)((angle / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US));
    return servo_set_pulse_us(channel, pulse);
}

esp_err_t servo_sleep_all(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    for (uint8_t ch = 0; ch < SERVO_CHANNEL_COUNT; ch++) {
        pca9685_set_pwm(ch, 0);
    }
    return i2c_write(0x00, 0x10);  /* MODE1.SLEEP=1 */
}

bool servo_is_ready(void)
{
    return s_initialized;
}

void servo_deinit(void)
{
    if (s_initialized) {
        servo_sleep_all();
        i2c_driver_delete(I2C_MASTER_NUM);
        s_initialized = false;
    }
}
