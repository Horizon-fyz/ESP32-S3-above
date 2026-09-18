/**
 * @file imu.h
 * @brief 云台 MPU6050 姿态传感器驱动
 *
 * ================= 接线 =================
 *
 *   云台 MPU6050 : I2C1, SDA=GPIO16, SCL=GPIO17, 地址 0x68 (AD0 接 GND)
 *                  与 PCA9685 (0x40) **共用同一条 I2C1**, 该总线由 servo 组件安装,
 *                  本组件只借用、不重复安装, 因此 **必须先执行 servo_init()**,
 *                  再调 imu_init_role(IMU_ROLE_GIMBAL, ...)。
 *                  只出原始 6 轴 (加速度 / 角速度) + 温度, 姿态由上层 Madgwick 解算。
 *
 *   船体惯导数据改由 `components/nav` 组件 (GPS+IMU 一体惯导模块) 提供;
 *   本组件只负责云台 MPU6050, 不含任何船体 IMU / UART 相关代码。
 *
 * 地址说明: MPU6050 的地址只能由 AD0 决定 (0x68/0x69), AD0 不能悬空;
 *   cfg.i2c_addr = 0 时用默认 0x68。
 *
 * 提供:
 *   - MPU6050   : 3 轴加速度 (±2/4/8/16g 可选)、3 轴陀螺仪 (±250/500/1000/2000°/s 可选)、温度
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

    /* 云台 MPU6050 只出原始 6 轴, 这三项**恒为 0**, 姿态由上层 Madgwick 解算。 */
    float roll, pitch, yaw; ///< 姿态角 (恒为 0, 仅保留字段)
} imu_data_t;

/**
 * @brief IMU 配置
 */
typedef struct {
    int                sda_gpio;          ///< SDA 引脚, 默认 16 (仅当本组件自己安装总线时生效)
    int                scl_gpio;          ///< SCL 引脚, 默认 17 (同上)
    uint32_t           i2c_freq_hz;       ///< I2C 频率, 默认 400kHz (同上)
    uint8_t            i2c_addr;          ///< I2C 地址 (MPU6050: 0x68/0x69), 0=用默认 0x68
    imu_accel_range_t  accel_range;       ///< 加速度量程 (仅 MPU6050 有效)
    imu_gyro_range_t   gyro_range;        ///< 陀螺仪量程 (仅 MPU6050 有效)
} imu_config_t;

/**
 * @brief IMU 角色 (当前只有云台 MPU6050)
 */
typedef enum {
    IMU_ROLE_GIMBAL = 0,   ///< 云台 MPU6050 (I2C1, 与 PCA9685 共线, 默认 0x68)
    IMU_ROLE_COUNT
} imu_role_t;

/**
 * @brief 获取默认配置 (云台 MPU6050: SDA=21, SCL=17, 400kHz, 地址 0x68)
 */
imu_config_t imu_get_default_config(void);

/**
 * @brief 初始化指定角色的姿态传感器
 *
 * GIMBAL (云台 MPU6050): I2C1 由 servo 组件为 PCA9685 安装, 本组件只借用、
 * 不重复安装, 因此 **必须先 servo_init()**; cfg 里的引脚/频率字段此时不生效。
 *
 * cfg->i2c_addr = 0 时用默认地址 0x68; 探测失败会重试并扫描总线兜底。
 */
esp_err_t imu_init_role(imu_role_t role, const imu_config_t *cfg);

/**
 * @brief 读取指定角色的一次 IMU 数据 (阻塞, ~1ms)
 */
esp_err_t imu_read_role(imu_role_t role, imu_data_t *out);

/**
 * @brief 检查指定角色是否已成功初始化
 */
bool imu_role_ready(imu_role_t role);

/**
 * @brief 扫描指定角色所在的总线, 列出所有应答地址并尽量识别型号
 *
 * 用于排查接线/地址问题, 可反复调用 (不必重启, 插拔后随手复查):
 *   - 是 MPU6050 : 打印 WHO_AM_I 并判定型号
 *   - 其它器件   : 只报地址 (例如同线的 PCA9685 @ 0x40)
 */
esp_err_t imu_scan_role(imu_role_t role);

/**
 * @brief 反初始化**单个角色** (清掉器件记录, 便于换个地址重新探测)
 *
 * 只清该角色的状态, **不释放 I2C1 总线** (该总线由 servo 组件安装),
 * 要连总线一起释放请用 imu_deinit()。
 *
 * 典型用法: 换地址重探 —— 先 imu_deinit_role(), 再用带 i2c_addr 的配置调
 * imu_init_role()。
 */
esp_err_t imu_deinit_role(imu_role_t role);

/**
 * @brief 反初始化 (释放本组件安装的总线, 所有角色一起失效)
 */
void imu_deinit(void);

#ifdef __cplusplus
}
#endif
