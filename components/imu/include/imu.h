/**
 * @file imu.h
 * @brief MPU6050 惯性测量单元驱动 (I2C0)
 *
 * 硬件连接:
 *   SDA = GPIO6
 *   SCL = GPIO7
 *
 * 提供:
 *   - 3 轴加速度 (±2g/±4g/±8g/±16g 可选)
 *   - 3 轴陀螺仪 (±250/±500/±1000/±2000 °/s 可选)
 *   - 温度
 *
 * 注意: MPU6050 地址 0x68 (AD0=0) 或 0x69 (AD0=1).
 *       本驱动默认尝试 0x68, 失败时回退到 0x69.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 加速度量程
 */
typedef enum {
    IMU_ACCEL_RANGE_2G  = 0,
    IMU_ACCEL_RANGE_4G  = 1,
    IMU_ACCEL_RANGE_8G  = 2,
    IMU_ACCEL_RANGE_16G = 3,
} imu_accel_range_t;

/**
 * @brief 陀螺仪量程
 */
typedef enum {
    IMU_GYRO_RANGE_250DPS  = 0,
    IMU_GYRO_RANGE_500DPS  = 1,
    IMU_GYRO_RANGE_1000DPS = 2,
    IMU_GYRO_RANGE_2000DPS = 3,
} imu_gyro_range_t;

/**
 * @brief IMU 数据
 *
 * 单位:
 *   accel: g (重力加速度 9.8 m/s^2)
 *   gyro:  °/s
 */
typedef struct {
    float ax, ay, az;       ///< 加速度
    float gx, gy, gz;       ///< 角速度
    float temperature;      ///< 温度 (°C)
    uint64_t timestamp_us;  ///< 读取时刻 (us, esp_timer)
} imu_data_t;

/**
 * @brief IMU 配置
 */
typedef struct {
    int                sda_gpio;          ///< SDA 引脚, 默认 6
    int                scl_gpio;          ///< SCL 引脚, 默认 7
    uint32_t           i2c_freq_hz;       ///< I2C 频率, 默认 400kHz
    uint8_t            i2c_addr;          ///< I2C 地址 (0x68 或 0x69), 0=自动探测
    imu_accel_range_t  accel_range;
    imu_gyro_range_t   gyro_range;
} imu_config_t;

/**
 * @brief 获取默认配置 (I2C0, SDA=6, SCL=7, 400kHz)
 */
imu_config_t imu_get_default_config(void);

/**
 * @brief 初始化 I2C0 + MPU6050
 *
 * 包含: I2C 总线初始化, MPU6050 唤醒, 量程配置, 自检
 */
esp_err_t imu_init(const imu_config_t *cfg);

/**
 * @brief 读取一次 IMU 数据 (阻塞, ~1ms)
 */
esp_err_t imu_read(imu_data_t *out);

/**
 * @brief 检查 IMU 是否已成功初始化
 */
bool imu_is_ready(void);

/**
 * @brief 反初始化 (释放 I2C 总线)
 */
void imu_deinit(void);

#ifdef __cplusplus
}
#endif
