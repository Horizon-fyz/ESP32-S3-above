/**
 * @file imu.c
 * @brief MPU6050 驱动实现
 *
 * 寄存器操作通过 ESP-IDF 的 I2C master API 包装.
 * 关键寄存器:
 *   0x6B PWR_MGMT_1  - 电源管理 (写 0 唤醒)
 *   0x1B GYRO_CONFIG - 陀螺仪量程
 *   0x1C ACCEL_CONFIG - 加速度量程
 *   0x3B~0x48 数据寄存器 (14 字节)
 *   0x75 WHO_AM_I    - 设备 ID (期望 0x68)
 */

#include "imu.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "imu";

#define I2C_MASTER_NUM   I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS  100

/* MPU6050 寄存器 */
#define MPU_REG_SMPLRT_DIV   0x19
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_INT_PIN_CFG  0x37
#define MPU_REG_INT_ENABLE   0x38
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_PWR_MGMT_1  0x6B
#define MPU_REG_PWR_MGMT_2  0x6C
#define MPU_REG_WHO_AM_I    0x75
#define MPU6050_WHO_AM_I_VAL 0x68

/* 状态 */
static bool            s_initialized = false;
static uint8_t         s_i2c_addr    = 0x68;
static imu_config_t    s_cfg;
static float           s_accel_lsb   = 16384.0f;   /* ±2g -> 16384 LSB/g */
static float           s_gyro_lsb    = 131.0f;     /* ±250°/s -> 131 LSB/°/s */

/* ========== 低层 I2C 辅助 ========== */
static esp_err_t i2c_write_byte(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(I2C_MASTER_NUM, s_i2c_addr, buf, sizeof(buf),
                                       I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t i2c_read_bytes(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, s_i2c_addr,
                                         &reg, 1, buf, len,
                                         I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/* 探测设备 */
static esp_err_t probe_device(uint8_t addr)
{
    uint8_t who = 0;
    esp_err_t ret = i2c_read_bytes(MPU_REG_WHO_AM_I, &who, 1);
    if (ret == ESP_OK && who == MPU6050_WHO_AM_I_VAL) {
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}

/* ========== 公共 API ========== */
imu_config_t imu_get_default_config(void)
{
    imu_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.sda_gpio    = 6;
    cfg.scl_gpio    = 7;
    cfg.i2c_freq_hz = 400 * 1000;
    cfg.i2c_addr    = 0;             /* 0 = 自动探测 */
    cfg.accel_range = IMU_ACCEL_RANGE_4G;
    cfg.gyro_range  = IMU_GYRO_RANGE_500DPS;
    return cfg;
}

esp_err_t imu_init(const imu_config_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(&s_cfg, cfg, sizeof(s_cfg));

    /* 1. 初始化 I2C0 总线 */
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
    if (cfg->i2c_addr == 0) {
        if (probe_device(0x68) == ESP_OK) {
            s_i2c_addr = 0x68;
        } else if (probe_device(0x69) == ESP_OK) {
            s_i2c_addr = 0x69;
        } else {
            ESP_LOGE(TAG, "MPU6050 not found on I2C0");
            i2c_driver_delete(I2C_MASTER_NUM);
            return ESP_ERR_NOT_FOUND;
        }
    } else {
        s_i2c_addr = cfg->i2c_addr;
    }
    ESP_LOGI(TAG, "MPU6050 found at 0x%02X", s_i2c_addr);

    /* 3. 唤醒 (写 0 到 PWR_MGMT_1) */
    ret = i2c_write_byte(MPU_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "wake up failed");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 4. 设置时钟源 (PLL with X axis gyroscope) */
    i2c_write_byte(MPU_REG_PWR_MGMT_1, 0x01);

    /* 5. 采样率 = 1kHz / (1+SMPLRT_DIV). DIV=4 -> 200Hz */
    i2c_write_byte(MPU_REG_SMPLRT_DIV, 4);

    /* 6. DLPF = 3 (Accel BW 44Hz, Gyro BW 42Hz) */
    i2c_write_byte(MPU_REG_CONFIG, 3);

    /* 7. 设置量程 */
    i2c_write_byte(MPU_REG_GYRO_CONFIG,  cfg->gyro_range  << 3);
    i2c_write_byte(MPU_REG_ACCEL_CONFIG, cfg->accel_range << 3);

    /* 量程 -> LSB 换算系数 */
    static const float accel_lsb[4] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
    static const float gyro_lsb [4] = {   131.0f,  65.5f,   32.8f,   16.4f };
    s_accel_lsb = accel_lsb[cfg->accel_range];
    s_gyro_lsb  = gyro_lsb [cfg->gyro_range];

    s_initialized = true;
    ESP_LOGI(TAG, "IMU ready: accel=±%dg, gyro=±%d°/s",
             (cfg->accel_range == 0) ? 2  : (cfg->accel_range == 1) ? 4  : (cfg->accel_range == 2) ? 8  : 16,
             (cfg->gyro_range  == 0) ? 250 : (cfg->gyro_range  == 1) ? 500 : (cfg->gyro_range  == 2) ? 1000 : 2000);
    return ESP_OK;
}

esp_err_t imu_read(imu_data_t *out)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 一次性读取 14 字节: accel(6) + temp(2) + gyro(6) */
    uint8_t buf[14];
    esp_err_t ret = i2c_read_bytes(MPU_REG_ACCEL_XOUT_H, buf, sizeof(buf));
    if (ret != ESP_OK) {
        return ret;
    }

    int16_t raw_ax = (buf[0]  << 8) | buf[1];
    int16_t raw_ay = (buf[2]  << 8) | buf[3];
    int16_t raw_az = (buf[4]  << 8) | buf[5];
    int16_t raw_t  = (buf[6]  << 8) | buf[7];
    int16_t raw_gx = (buf[8]  << 8) | buf[9];
    int16_t raw_gy = (buf[10] << 8) | buf[11];
    int16_t raw_gz = (buf[12] << 8) | buf[13];

    out->ax = (float)raw_ax / s_accel_lsb;
    out->ay = (float)raw_ay / s_accel_lsb;
    out->az = (float)raw_az / s_accel_lsb;
    out->gx = (float)raw_gx / s_gyro_lsb;
    out->gy = (float)raw_gy / s_gyro_lsb;
    out->gz = (float)raw_gz / s_gyro_lsb;
    /* MPU6050 温度公式: T(°C) = raw/340 + 36.53 */
    out->temperature = (float)raw_t / 340.0f + 36.53f;
    out->timestamp_us = esp_timer_get_time();
    return ESP_OK;
}

bool imu_is_ready(void)
{
    return s_initialized;
}

void imu_deinit(void)
{
    if (s_initialized) {
        i2c_driver_delete(I2C_MASTER_NUM);
        s_initialized = false;
    }
}
