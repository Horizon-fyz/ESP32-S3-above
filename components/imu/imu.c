/**
 * @file imu.c
 * @brief MPU6050 驱动实现 (I2C 主机)
 *
 * 寄存器定义参考 GY-521 模块资料 PS-MPU-6000A.pdf / RM-MPU-6000A.pdf
 * 初始化流程参考 51-串口-mpu6050.c 参考程序
 *
 * 量程与 LSB 换算 (来自 RM-MPU-6000A.pdf §4):
 *   加速度 ±2g:  16384 LSB/g
 *   加速度 ±4g:  8192  LSB/g
 *   加速度 ±8g:  4096  LSB/g
 *   加速度 ±16g: 2048  LSB/g
 *   陀螺仪 ±250°/s:  131.0 LSB/(°/s)
 *   陀螺仪 ±500°/s:  65.5  LSB/(°/s)
 *   陀螺仪 ±1000°/s: 32.8  LSB/(°/s)
 *   陀螺仪 ±2000°/s: 16.4  LSB/(°/s)
 *   温度:  340 LSB/°C, 偏移 -521 LSB (对应 36.53°C)
 *        T(°C) = TEMP_OUT / 340 + 36.53
 */

#include "imu.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "imu";

/* ===== I2C 地址 ===== */
#define MPU6050_I2C_ADDR_LOW   0x68  /* AD0=0 (GY-521 默认) */
#define MPU6050_I2C_ADDR_HIGH  0x69  /* AD0=1 */

/* ===== 寄存器映射 (来自 RM-MPU-6000A.pdf §6) ===== */
#define REG_SELF_TEST_X        0x0D
#define REG_SELF_TEST_Y        0x0E
#define REG_SELF_TEST_Z        0x0F
#define REG_SELF_TEST_A        0x10
#define REG_SMPLRT_DIV         0x19
#define REG_CONFIG             0x1A
#define REG_GYRO_CONFIG        0x1B
#define REG_ACCEL_CONFIG       0x1C
#define REG_FIFO_EN            0x23
#define REG_I2C_MST_CTRL       0x24
#define REG_I2C_SLV0_ADDR      0x25
#define REG_I2C_SLV0_REG       0x26
#define REG_I2C_SLV0_CTRL      0x27
#define REG_I2C_SLV1_ADDR      0x28
#define REG_I2C_SLV1_REG       0x29
#define REG_I2C_SLV1_CTRL      0x2A
#define REG_I2C_SLV2_ADDR      0x2B
#define REG_I2C_SLV2_REG       0x2C
#define REG_I2C_SLV2_CTRL      0x2D
#define REG_I2C_SLV3_ADDR      0x2E
#define REG_I2C_SLV3_REG       0x2F
#define REG_I2C_SLV3_CTRL      0x30
#define REG_I2C_SLV4_ADDR      0x31
#define REG_I2C_SLV4_REG       0x32
#define REG_I2C_SLV4_DO        0x33
#define REG_I2C_SLV4_CTRL      0x34
#define REG_I2C_SLV4_DI        0x35
#define REG_I2C_MST_STATUS     0x36
#define REG_INT_PIN_CFG        0x37
#define REG_INT_ENABLE         0x38
#define REG_INT_STATUS         0x3A
#define REG_ACCEL_XOUT_H       0x3B
#define REG_ACCEL_XOUT_L       0x3C
#define REG_ACCEL_YOUT_H       0x3D
#define REG_ACCEL_YOUT_L       0x3E
#define REG_ACCEL_ZOUT_H       0x3F
#define REG_ACCEL_ZOUT_L       0x40
#define REG_TEMP_OUT_H         0x41
#define REG_TEMP_OUT_L         0x42
#define REG_GYRO_XOUT_H        0x43
#define REG_GYRO_XOUT_L        0x44
#define REG_GYRO_YOUT_H        0x45
#define REG_GYRO_YOUT_L        0x46
#define REG_GYRO_ZOUT_H        0x47
#define REG_GYRO_ZOUT_L        0x48
#define REG_EXT_SENS_DATA_00   0x49
#define REG_I2C_SLV0_DO        0x63
#define REG_I2C_SLV1_DO        0x64
#define REG_I2C_SLV2_DO        0x65
#define REG_I2C_SLV3_DO        0x66
#define REG_I2C_MST_DELAY_CTRL 0x67
#define REG_SIGNAL_PATH_RESET  0x68
#define REG_USER_CTRL          0x6A
#define REG_PWR_MGMT_1        0x6B
#define REG_PWR_MGMT_2        0x6C
#define REG_FIFO_COUNTH        0x72
#define REG_FIFO_COUNTL        0x73
#define REG_FIFO_R_W           0x74
#define REG_WHO_AM_I           0x75

/* WHO_AM_I 应该返回 0x68 (MPU-6050) */
#define MPU6050_WHO_AM_I_VAL   0x68

/* PWR_MGMT_1 位 */
#define PWR_MGMT_1_DEVICE_RESET  0x80
#define PWR_MGMT_1_SLEEP         0x40
#define PWR_MGMT_1_CYCLE         0x20
#define PWR_MGMT_1_TEMP_DIS      0x08
#define PWR_MGMT_1_CLKSEL_PLL   0x01  /* 使用陀螺仪 X 轴作为时钟源 */

/* 陀螺仪配置位 [FS_SEL] 位于 0x1B bit4-bit3 */
#define GYRO_CONFIG_FS_250   (0x00 << 3)
#define GYRO_CONFIG_FS_500   (0x01 << 3)
#define GYRO_CONFIG_FS_1000  (0x02 << 3)
#define GYRO_CONFIG_FS_2000  (0x03 << 3)
/* 自检位 [0x1B bit7-bit5] - 不使用自检 */
#define GYRO_CONFIG_SELF_TEST_MASK  0xE0  /* X/Y/Z 自检位掩码 */
#define GYRO_CONFIG_SELF_TEST_OFF   0x00

/* 加速度配置位 [AFS_SEL] 位于 0x1C bit4-bit3 */
#define ACCEL_CONFIG_FS_2G   (0x00 << 3)
#define ACCEL_CONFIG_FS_4G   (0x01 << 3)
#define ACCEL_CONFIG_FS_8G   (0x02 << 3)
#define ACCEL_CONFIG_FS_16G  (0x03 << 3)
#define ACCEL_CONFIG_SELF_TEST_MASK  0xE0
#define ACCEL_CONFIG_SELF_TEST_OFF   0x00

/* ===== I2C 总线句柄 ===== */
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_TIMEOUT   pdMS_TO_TICKS(1000)
#define I2C_MASTER_FREQ_HZ_DEFAULT  400000

/* ===== 模块状态 ===== */
static struct {
    i2c_port_t       i2c_port;
    uint8_t          i2c_addr;
    i2c_config_t     i2c_cfg;
    bool             initialized;
    imu_accel_range_t accel_range;
    imu_gyro_range_t  gyro_range;
} s_imu = {
    .i2c_port      = I2C_MASTER_NUM,
    .i2c_addr      = MPU6050_I2C_ADDR_LOW,
    .initialized   = false,
    .accel_range   = IMU_ACCEL_RANGE_2G,
    .gyro_range    = IMU_GYRO_RANGE_250DPS,
};

/* ===== 量程 -> LSB 单位换算 ===== */
static inline float accel_lsb_per_g(imu_accel_range_t r)
{
    switch (r) {
        case IMU_ACCEL_RANGE_2G:  return 16384.0f;
        case IMU_ACCEL_RANGE_4G:  return 8192.0f;
        case IMU_ACCEL_RANGE_8G:  return 4096.0f;
        case IMU_ACCEL_RANGE_16G: return 2048.0f;
        default:                  return 16384.0f;
    }
}

static inline float gyro_lsb_per_dps(imu_gyro_range_t r)
{
    switch (r) {
        case IMU_GYRO_RANGE_250DPS:  return 131.0f;
        case IMU_GYRO_RANGE_500DPS:  return 65.5f;
        case IMU_GYRO_RANGE_1000DPS: return 32.8f;
        case IMU_GYRO_RANGE_2000DPS: return 16.4f;
        default:                     return 131.0f;
    }
}

static uint8_t gyro_fs_sel(imu_gyro_range_t r)
{
    switch (r) {
        case IMU_GYRO_RANGE_250DPS:  return GYRO_CONFIG_FS_250;
        case IMU_GYRO_RANGE_500DPS:  return GYRO_CONFIG_FS_500;
        case IMU_GYRO_RANGE_1000DPS: return GYRO_CONFIG_FS_1000;
        case IMU_GYRO_RANGE_2000DPS: return GYRO_CONFIG_FS_2000;
        default:                     return GYRO_CONFIG_FS_250;
    }
}

static uint8_t accel_fs_sel(imu_accel_range_t r)
{
    switch (r) {
        case IMU_ACCEL_RANGE_2G:  return ACCEL_CONFIG_FS_2G;
        case IMU_ACCEL_RANGE_4G:  return ACCEL_CONFIG_FS_4G;
        case IMU_ACCEL_RANGE_8G:  return ACCEL_CONFIG_FS_8G;
        case IMU_ACCEL_RANGE_16G: return ACCEL_CONFIG_FS_16G;
        default:                  return ACCEL_CONFIG_FS_2G;
    }
}

/* ===== 低层 I2C 操作 ===== */
static esp_err_t mpu_write_reg(uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_imu.i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_imu.i2c_port, cmd, I2C_MASTER_TIMEOUT);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t mpu_read_reg(uint8_t reg, uint8_t *val)
{
    if (val == NULL) return ESP_ERR_INVALID_ARG;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_imu.i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_imu.i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, val, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_imu.i2c_port, cmd, I2C_MASTER_TIMEOUT);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t mpu_read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    if (buf == NULL || len == 0) return ESP_ERR_INVALID_ARG;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_imu.i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_imu.i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buf, len, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_imu.i2c_port, cmd, I2C_MASTER_TIMEOUT);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* ===== 探测设备 ===== */
/* 尝试读 WHO_AM_I 寄存器, 探测 MPU6050 是否在总线上 */
static esp_err_t mpu_probe(uint8_t addr)
{
    /* 直接尝试读 WHO_AM_I, ESP-IDF v5.5 已无 i2c_master_probe */
    uint8_t who = 0;
    esp_err_t ret = i2c_master_write_read_device(s_imu.i2c_port, addr,
                                                 (uint8_t[]){ REG_WHO_AM_I }, 1,
                                                 &who, 1, I2C_MASTER_TIMEOUT);
    if (ret != ESP_OK) return ret;
    if (who != MPU6050_WHO_AM_I_VAL) {
        ESP_LOGW(TAG, "WHO_AM_I=0x%02X (期望 0x%02X) @ addr 0x%02X",
                 who, MPU6050_WHO_AM_I_VAL, addr);
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

/* ===== 默认配置 ===== */
imu_config_t imu_get_default_config(void)
{
    imu_config_t cfg = {
        .sda_gpio    = 6,
        .scl_gpio    = 7,
        .i2c_freq_hz = I2C_MASTER_FREQ_HZ_DEFAULT,
        .i2c_addr    = 0,   /* 0 = 自动探测 */
        /* 与 main.c 中协议推算一致:
         *   ax/ay/az  × 16384  (即 ±2g LSB)
         *   gx/gy/gz  × 131    (即 ±250°/s LSB, 防止 int16 溢出)
         * 若需要更大角速度量程, 上层需同步缩放因子 (见 main.c mpu_push_task)
         */
        .accel_range = IMU_ACCEL_RANGE_2G,
        .gyro_range  = IMU_GYRO_RANGE_250DPS,
    };
    return cfg;
}

/* ===== 初始化 ===== */
esp_err_t imu_init(const imu_config_t *cfg)
{
    if (cfg == NULL) return ESP_ERR_INVALID_ARG;
    if (s_imu.initialized) {
        ESP_LOGW(TAG, "已初始化, 跳过");
        return ESP_OK;
    }

    /* 1. 探测 I2C 地址 */
    if (cfg->i2c_addr == 0) {
        ESP_LOGI(TAG, "自动探测 MPU6050 地址...");
        if (mpu_probe(MPU6050_I2C_ADDR_LOW) == ESP_OK) {
            s_imu.i2c_addr = MPU6050_I2C_ADDR_LOW;
            ESP_LOGI(TAG, "发现 MPU6050 @ 0x%02X (AD0=GND)", s_imu.i2c_addr);
        } else if (mpu_probe(MPU6050_I2C_ADDR_HIGH) == ESP_OK) {
            s_imu.i2c_addr = MPU6050_I2C_ADDR_HIGH;
            ESP_LOGI(TAG, "发现 MPU6050 @ 0x%02X (AD0=VCC)", s_imu.i2c_addr);
        } else {
            ESP_LOGE(TAG, "MPU6050 探测失败 (检查接线/上电)");
            return ESP_ERR_NOT_FOUND;
        }
    } else {
        if (cfg->i2c_addr != MPU6050_I2C_ADDR_LOW &&
            cfg->i2c_addr != MPU6050_I2C_ADDR_HIGH) {
            ESP_LOGE(TAG, "非法地址 0x%02X", cfg->i2c_addr);
            return ESP_ERR_INVALID_ARG;
        }
        s_imu.i2c_addr = cfg->i2c_addr;
        esp_err_t ret = mpu_probe(s_imu.i2c_addr);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "指定地址 0x%02X 探测失败", s_imu.i2c_addr);
            return ret;
        }
    }

    /* 2. 初始化 I2C 总线 */
    s_imu.i2c_cfg = (i2c_config_t) {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = cfg->sda_gpio,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = cfg->scl_gpio,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = cfg->i2c_freq_hz,
    };
    esp_err_t ret = i2c_param_config(s_imu.i2c_port, &s_imu.i2c_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config 失败: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = i2c_driver_install(s_imu.i2c_port, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install 失败: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "I2C%d 初始化: SDA=IO%d, SCL=IO%d, %luHz",
             s_imu.i2c_port, cfg->sda_gpio, cfg->scl_gpio,
             (unsigned long)cfg->i2c_freq_hz);

    /* 3. 唤醒 MPU6050 (默认上电后处于 SLEEP 状态) */
    ret = mpu_write_reg(REG_PWR_MGMT_1, PWR_MGMT_1_CLKSEL_PLL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "写 PWR_MGMT_1 失败: %s", esp_err_to_name(ret));
        goto err_cleanup;
    }
    vTaskDelay(pdMS_TO_TICKS(100));  /* 等待时钟稳定 */

    /* 4. 配置采样率分频: 1kHz / (1+SMPLRT_DIV) */
    /* 0x00 = 1kHz, 0x07 = 125Hz (参考程序使用 125Hz) */
    ret = mpu_write_reg(REG_SMPLRT_DIV, 0x07);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "写 SMPLRT_DIV 失败: %s", esp_err_to_name(ret));
        goto err_cleanup;
    }

    /* 5. 低通滤波器: 0x06 = 5Hz (参考程序使用) */
    ret = mpu_write_reg(REG_CONFIG, 0x06);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "写 CONFIG 失败: %s", esp_err_to_name(ret));
        goto err_cleanup;
    }

    /* 6. 陀螺仪量程 + 关闭自检 */
    s_imu.gyro_range = cfg->gyro_range;
    ret = mpu_write_reg(REG_GYRO_CONFIG, GYRO_CONFIG_SELF_TEST_OFF | gyro_fs_sel(s_imu.gyro_range));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "写 GYRO_CONFIG 失败: %s", esp_err_to_name(ret));
        goto err_cleanup;
    }

    /* 7. 加速度量程 + 关闭自检 */
    s_imu.accel_range = cfg->accel_range;
    ret = mpu_write_reg(REG_ACCEL_CONFIG, ACCEL_CONFIG_SELF_TEST_OFF | accel_fs_sel(s_imu.accel_range));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "写 ACCEL_CONFIG 失败: %s", esp_err_to_name(ret));
        goto err_cleanup;
    }

    /* 8. 验证 WHO_AM_I */
    uint8_t who = 0;
    ret = mpu_read_reg(REG_WHO_AM_I, &who);
    if (ret != ESP_OK || who != MPU6050_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "初始化后 WHO_AM_I 验证失败: 0x%02X", who);
        ret = (ret != ESP_OK) ? ret : ESP_ERR_INVALID_RESPONSE;
        goto err_cleanup;
    }

    s_imu.initialized = true;
    ESP_LOGI(TAG, "初始化完成: 加速度 ±%dg, 陀螺仪 ±%d°/s",
             (s_imu.accel_range == IMU_ACCEL_RANGE_2G) ? 2 :
             (s_imu.accel_range == IMU_ACCEL_RANGE_4G) ? 4 :
             (s_imu.accel_range == IMU_ACCEL_RANGE_8G) ? 8 : 16,
             (s_imu.gyro_range == IMU_GYRO_RANGE_250DPS) ? 250 :
             (s_imu.gyro_range == IMU_GYRO_RANGE_500DPS) ? 500 :
             (s_imu.gyro_range == IMU_GYRO_RANGE_1000DPS) ? 1000 : 2000);
    return ESP_OK;

err_cleanup:
    i2c_driver_delete(s_imu.i2c_port);
    s_imu.initialized = false;
    return ret;
}

/* ===== 读取一帧数据 ===== */
esp_err_t imu_read(imu_data_t *out)
{
    if (out == NULL) return ESP_ERR_INVALID_ARG;
    if (!s_imu.initialized) return ESP_ERR_INVALID_STATE;

    /* 从 ACCEL_XOUT_H 开始连续读 14 字节:
     *   [0..1]  AccX, [2..3]  AccY, [4..5]  AccZ,
     *   [6..7]  Temp,  [8..9]  GyrX, [10..11] GyrY, [12..13] GyrZ
     */
    uint8_t buf[14] = {0};
    esp_err_t ret = mpu_read_regs(REG_ACCEL_XOUT_H, buf, sizeof(buf));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "读传感器数据失败: %s", esp_err_to_name(ret));
        return ret;
    }

    int16_t raw_ax = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t raw_ay = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t raw_az = (int16_t)((buf[4]  << 8) | buf[5]);
    int16_t raw_t  = (int16_t)((buf[6]  << 8) | buf[7]);
    int16_t raw_gx = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

    const float accel_lsb = accel_lsb_per_g(s_imu.accel_range);
    const float gyro_lsb  = gyro_lsb_per_dps(s_imu.gyro_range);

    out->ax = (float)raw_ax / accel_lsb;
    out->ay = (float)raw_ay / accel_lsb;
    out->az = (float)raw_az / accel_lsb;
    out->gx = (float)raw_gx / gyro_lsb;
    out->gy = (float)raw_gy / gyro_lsb;
    out->gz = (float)raw_gz / gyro_lsb;
    /* 温度换算 (RM-MPU-6000A.pdf §4.19):
     *   Temperature in degrees C = (TEMP_OUT / 340) + 36.53
     */
    out->temperature = (float)raw_t / 340.0f + 36.53f;
    out->timestamp_us = esp_timer_get_time();

    return ESP_OK;
}

/* ===== 查询是否就绪 ===== */
bool imu_is_ready(void)
{
    return s_imu.initialized;
}

/* ===== 反初始化 ===== */
void imu_deinit(void)
{
    if (!s_imu.initialized) return;
    /* 让 MPU 重新进入 SLEEP 省电 */
    mpu_write_reg(REG_PWR_MGMT_1, PWR_MGMT_1_SLEEP);
    i2c_driver_delete(s_imu.i2c_port);
    s_imu.initialized = false;
    ESP_LOGI(TAG, "已反初始化");
}
