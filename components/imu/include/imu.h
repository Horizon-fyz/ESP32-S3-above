/**
 * @file imu.h
 * @brief 姿态传感器驱动 (两颗不同型号器件, 各占一条 I2C 总线)
 *
 * 两颗器件各占一条独立总线 (ESP32-S3 只有 2 个 I2C 控制器, 正好用完):
 *
 *   I2C0  SDA=GPIO16, SCL=GPIO18  ->  云台 MPU6050 (地址 0x68, AD0 接 GND)
 *                                   只出原始 6 轴, 姿态由上层 Madgwick 解算
 *
 *   I2C1  SDA=GPIO21, SCL=GPIO17  ->  船体 10 轴 IMU (地址 0x50, WIT 协议,
 *                                   亚博 10轴IMU惯导模块)
 *                                   内部已解算 Roll/Pitch/Yaw (带磁力计, yaw 不漂)
 *                                   该总线与 PCA9685 (0x40) 共用, 由 servo 组件安装
 *
 * 地址说明:
 *   - MPU6050 的地址只能由 AD0 决定 (0x68/0x69), AD0 不能悬空。
 *   - 船体 10 轴 IMU 出厂默认 0x50, 且支持软件改地址 (寄存器 0x1A, 0x01~0x7F);
 *     0x50 与云台的 0x68、PCA9685 的 0x40 都不冲突, 保持出厂默认即可。
 *
 * ⚠️ 初始化顺序: 船体 IMU 是"借用"I2C1 (重复 i2c_driver_install 会失败),
 *    所以必须 servo_init() 先执行, 再调 imu_init_role(IMU_ROLE_HULL, ...)。
 *    云台 IMU 用的 I2C0 由本组件自己安装, 顺序无关。
 *
 * 提供:
 *   - MPU6050   : 3 轴加速度 (±2/4/8/16g 可选)、3 轴陀螺仪 (±250/500/1000/2000°/s 可选)、温度
 *   - 10 轴 IMU : 3 轴加速度、3 轴陀螺仪、内部解算 Roll/Pitch/Yaw、温度
 *                 (按出厂默认量程 ±16g / ±2000°/s 换算)
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
 *   roll/pitch/yaw: °
 */
typedef struct {
    float ax, ay, az;       ///< 加速度
    float gx, gy, gz;       ///< 角速度
    float temperature;      ///< 温度 (°C)
    uint64_t timestamp_us;  ///< 读取时刻 (us, esp_timer)

    /* 以下三项只对"内部已解算姿态"的器件有效 (船体 10 轴 IMU);
     * 云台 MPU6050 只出原始 6 轴, 这里恒为 0, 姿态由上层 Madgwick 解算。 */
    float roll, pitch, yaw; ///< 内部解算姿态角
} imu_data_t;

/**
 * @brief IMU 配置
 */
typedef struct {
    int                sda_gpio;          ///< SDA 引脚, 默认 16 (仅云台 I2C0 有效)
    int                scl_gpio;          ///< SCL 引脚, 默认 18 (仅云台 I2C0 有效)
    uint32_t           i2c_freq_hz;       ///< I2C 频率, 默认 400kHz (仅云台 I2C0 有效)
    uint8_t            i2c_addr;          ///< I2C 地址 (云台 0x68 / 船体 0x50), 0=按角色取默认
    imu_accel_range_t  accel_range;       ///< 仅 MPU6050 (云台) 有效
    imu_gyro_range_t   gyro_range;        ///< 仅 MPU6050 (云台) 有效
} imu_config_t;

/**
 * @brief IMU 角色 (两颗不同型号器件, 各占一条 I2C 总线)
 */
typedef enum {
    IMU_ROLE_GIMBAL = 0,   ///< 云台 MPU6050    (I2C0, AD0=GND, 0x68)
    IMU_ROLE_HULL   = 1,   ///< 船体 10 轴 IMU (I2C1, WIT 协议, 默认 0x50)
    IMU_ROLE_COUNT
} imu_role_t;

/**
 * @brief 获取默认配置 (云台 I2C0: SDA=16, SCL=18, 400kHz, 地址按角色取默认)
 */
imu_config_t imu_get_default_config(void);

/**
 * @brief 初始化指定角色的姿态传感器
 *
 * 总线分工:
 *   - GIMBAL: I2C0 由本组件安装 (cfg->sda_gpio/scl_gpio/i2c_freq_hz 在此生效);
 *   - HULL  : I2C1 由 servo 组件为 PCA9685 安装, 本组件只借用、不重复安装,
 *             因此 **必须先 servo_init()**, cfg 里的引脚/频率字段对船体无效。
 *
 * cfg->i2c_addr = 0 时按角色取默认地址 (GIMBAL->0x68, HULL->0x50)。
 * MPU6050 探测失败会重试并扫描总线兜底; 船体 10 轴 IMU 只按地址探测 (不做扫描)。
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
 *   - 是 MPU6050 : 打印 WHO_AM_I
 *   - 是 10 轴 IMU: 打印 IICADDR / VERSION / Roll 原始值, 并判定型号
 *   - 其它器件   : 只报地址
 */
esp_err_t imu_scan_role(imu_role_t role);

/**
 * @brief 反初始化 (释放 I2C 总线, 所有角色一起失效)
 */
void imu_deinit(void);

#ifdef __cplusplus
}
#endif
