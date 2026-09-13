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
#include <math.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
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

/* ===== 船体 10 轴 IMU (亚博 10轴IMU惯导模块, WIT 协议) =====
 *
 * 寄存器表来自官方《10轴IMU模块通讯协议》:
 *   0x1A IICADDR  设备地址 (默认 0x0050 -> 0x50), 可写 0x01~0x7F
 *   0x2E VERSION  固件版本号 (只读)
 *   0x34~0x36  AX/AY/AZ   加速度
 *   0x37~0x39  GX/GY/GZ   角速度
 *   0x3A~0x3C  HX/HY/HZ   磁场
 *   0x3D~0x3F  Roll/Pitch/Yaw  内部解算姿态角
 *   0x40       TEMP       温度
 *
 * 读写时序 (与 MPU6050 不同, 数据是"低字节在前"):
 *   写: START, 地址+W, 寄存器, 数据低8位, 数据高8位, ..., STOP
 *   读: START, 地址+W, 寄存器, RESTART, 地址+R, 数据低8位(ACK), 数据高8位(NACK), STOP
 */
#define WIT_I2C_ADDR_DEFAULT   0x50
#define WIT_REG_IICADDR        0x1A   /* 回读自己配置的 I2C 地址(低字节), 用来识别型号 */
#define WIT_REG_VERSION        0x2E
#define WIT_REG_ACC_X          0x34   /* 连续 6 个寄存器: AX AY AZ GX GY GZ */
#define WIT_REG_ROLL           0x3D   /* 连续 4 个寄存器: Roll Pitch Yaw TEMP */

/* 换算系数 (官方协议给的公式, 对应出厂默认量程 ±16g / ±2000°/s):
 *   加速度 = raw / 32768 * 16   (g)
 *   角速度 = raw / 32768 * 2000 (°/s)
 *   角度   = raw / 32768 * 180  (°)
 *   温度   = raw / 100          (°C)
 */
#define WIT_ACCEL_LSB          (32768.0f / 16.0f)
#define WIT_GYRO_LSB           (32768.0f / 2000.0f)
#define WIT_ANGLE_LSB          (32768.0f / 180.0f)

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

/* ===== 采样与滤波配置 (调试项, 直接改这里) =====
 *
 * CONFIG(0x1A) 低 3 位 DLPF_CFG 选择低通带宽 (陀螺输出速率 1kHz):
 *   0 = 260Hz(accel) / 256Hz(gyro)  延迟 ~0ms   (DLPF 关闭, 陀螺升到 8kHz)
 *   1 = 184Hz / 188Hz               延迟 ~1.9ms
 *   2 =  94Hz /  98Hz               延迟 ~2.8ms
 *   3 =  44Hz /  42Hz               延迟 ~4.8ms
 *   4 =  21Hz /  20Hz               延迟 ~8.3ms
 *   5 =  10Hz /  10Hz               延迟 ~13.4ms
 *   6 =   5Hz /   5Hz               延迟 ~18.6ms  (原参考程序取值, 动态响应太钝)
 *
 * 采样率 = 1kHz / (1 + SMPLRT_DIV); 需 >= 融合频率 (main.c 的 AHRS_UPDATE_HZ = 100Hz)
 */
#define MPU6050_DLPF_CFG     3      /* 0~6, 当前 44Hz/42Hz */
#define MPU6050_SMPLRT_DIV   4      /* 1kHz/(1+4) = 200Hz */

/* ===== I2C 总线 =====
 *
 * 两颗器件各占一条独立总线 (ESP32-S3 只有 2 个 I2C 控制器, 正好用完):
 *   云台 MPU6050   : I2C0, SDA=GPIO16, SCL=GPIO18 —— 由本组件安装
 *   船体 10 轴 IMU : I2C1, SDA=GPIO21, SCL=GPIO17 —— 与 PCA9685 共用,
 *                    该总线由 servo 组件安装, 本组件只借用、不重复安装。
 *
 * 为什么不能重复安装: IDF 的 i2c_driver_install() 对已安装端口直接返回 ESP_FAIL,
 * 而 i2c_param_config() 不做"是否已安装"检查, 会重新配置引脚并把 PCA9685 的总线搞坏。
 * 因此 main.c 必须保证 servo_init() 先于 imu_init_role(IMU_ROLE_HULL, ...) 执行。
 */
#define IMU_GIMBAL_I2C_PORT  I2C_NUM_0
#define IMU_HULL_I2C_PORT    I2C_NUM_1
#define I2C_MASTER_TIMEOUT   pdMS_TO_TICKS(1000)
#define I2C_MASTER_FREQ_HZ_DEFAULT  400000

/* 探测重试: 上电初期首次访问偶尔会 NACK (实测: 同一调用晚 30ms 就成功),
 * 重试几次即可跨过, 否则会导致整个 IMU 初始化失败. */
#define MPU_PROBE_RETRY      3
#define MPU_PROBE_RETRY_MS   30

/* ===== 模块状态 ===== */
/* 每个角色挂在哪条总线上 */
static const i2c_port_t s_port[IMU_ROLE_COUNT] = {
    [IMU_ROLE_GIMBAL] = IMU_GIMBAL_I2C_PORT,
    [IMU_ROLE_HULL]   = IMU_HULL_I2C_PORT,
};

/* 每个角色的总线是否已就绪 (云台那条由本组件装, 船体那条由 servo 装) */
static bool s_bus_ready[IMU_ROLE_COUNT];

/* 每颗设备的状态, 按 imu_role_t 索引 */
static struct {
    uint8_t           i2c_addr;
    bool              initialized;
    imu_accel_range_t accel_range;
    imu_gyro_range_t  gyro_range;
} s_dev[IMU_ROLE_COUNT];

/* 各角色的出厂默认地址 (云台 MPU6050: AD0=GND -> 0x68; 船体 10 轴 IMU: WIT 默认 0x50) */
static uint8_t role_default_addr(imu_role_t role)
{
    return (role == IMU_ROLE_HULL) ? WIT_I2C_ADDR_DEFAULT : MPU6050_I2C_ADDR_LOW;
}

static const char *role_name(imu_role_t role)
{
    return (role == IMU_ROLE_HULL) ? "船体" : "云台";
}

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

/* ===== 低层 I2C 操作 (要带端口和目标地址: 两颗器件分属两条总线) ===== */
static esp_err_t i2c_write_reg(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, I2C_MASTER_TIMEOUT);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_reg(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *val)
{
    if (val == NULL) return ESP_ERR_INVALID_ARG;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, val, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, I2C_MASTER_TIMEOUT);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_regs(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len)
{
    if (buf == NULL || len == 0) return ESP_ERR_INVALID_ARG;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    /* 前 len-1 字节 ACK, 最后一字节 NACK (I2C 协议要求) */
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, I2C_MASTER_TIMEOUT);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* ===== 探测设备 ===== */
/* 尝试读 WHO_AM_I 寄存器, 探测 MPU6050 是否在总线上 */
static esp_err_t mpu_probe(i2c_port_t port, uint8_t addr)
{
    /* 直接尝试读 WHO_AM_I, ESP-IDF v5.5 已无 i2c_master_probe */
    uint8_t who = 0;
    esp_err_t ret = ESP_FAIL;

    for (int i = 0; i < MPU_PROBE_RETRY; i++) {
        ret = i2c_master_write_read_device(port, addr,
                                           (uint8_t[]){ REG_WHO_AM_I }, 1,
                                           &who, 1, I2C_MASTER_TIMEOUT);
        if (ret == ESP_OK) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(MPU_PROBE_RETRY_MS));
    }

    if (ret != ESP_OK) {
        /* 从机完全不应答 (NACK/超时), 通常是接线/供电问题 */
        ESP_LOGW(TAG, "地址 0x%02X 无应答: %s (%d 次重试)",
                 addr, esp_err_to_name(ret), MPU_PROBE_RETRY);
        return ret;
    }
    if (who != MPU6050_WHO_AM_I_VAL) {
        ESP_LOGW(TAG, "WHO_AM_I=0x%02X (期望 0x%02X) @ addr 0x%02X",
                 who, MPU6050_WHO_AM_I_VAL, addr);
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

/* ===== 船体 10 轴 IMU (WIT 协议) ===== */

/**
 * 取一个 16 位有符号量.
 * WIT 寄存器里多字节数据是"低字节在前, 高字节在后" (与 MPU6050 相反).
 */
static inline int16_t wit_s16(const uint8_t *p)
{
    return (int16_t)(((int16_t)p[1] << 8) | p[0]);
}

/**
 * 探测器件是否在线.
 * WIT 没有 WHO_AM_I, 这里读 VERSION(0x2E) 判断: 能读回来就说明器件在.
 * 带重试, 兼容上电初期偶发 NACK.
 */
static esp_err_t wit_probe(i2c_port_t port, uint8_t addr)
{
    uint8_t   ver[2] = {0};
    esp_err_t ret    = ESP_FAIL;

    for (int i = 0; i < MPU_PROBE_RETRY; i++) {
        ret = i2c_read_regs(port, addr, WIT_REG_VERSION, ver, sizeof(ver));
        if (ret == ESP_OK) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(MPU_PROBE_RETRY_MS));
    }
    if (ret != ESP_OK) {
        return ret;
    }
    ESP_LOGI(TAG, "VERSION=0x%04X @ addr 0x%02X", (unsigned)((ver[1] << 8) | ver[0]), addr);
    return ESP_OK;
}

/**
 * 读一帧: 原始 6 轴 + 模块内部解算的 Roll/Pitch/Yaw + 温度.
 * 只读不写 —— WIT 写配置寄存器要先解锁且有 10s 超时限制, 这里不需要碰.
 */
static esp_err_t wit_read(i2c_port_t port, uint8_t addr, imu_data_t *out)
{
    uint8_t raw[12] = {0};   /* 0x34~0x39, 6 个 16 位寄存器: AX AY AZ GX GY GZ */
    uint8_t ang[8]  = {0};   /* 0x3D~0x40, 4 个 16 位寄存器: Roll Pitch Yaw TEMP */

    esp_err_t ret = i2c_read_regs(port, addr, WIT_REG_ACC_X, raw, sizeof(raw));
    if (ret != ESP_OK) {
        return ret;
    }
    ret = i2c_read_regs(port, addr, WIT_REG_ROLL, ang, sizeof(ang));
    if (ret != ESP_OK) {
        return ret;
    }

    out->ax = (float)wit_s16(&raw[0]) / WIT_ACCEL_LSB;
    out->ay = (float)wit_s16(&raw[2]) / WIT_ACCEL_LSB;
    out->az = (float)wit_s16(&raw[4]) / WIT_ACCEL_LSB;
    out->gx = (float)wit_s16(&raw[6]) / WIT_GYRO_LSB;
    out->gy = (float)wit_s16(&raw[8]) / WIT_GYRO_LSB;
    out->gz = (float)wit_s16(&raw[10]) / WIT_GYRO_LSB;

    out->roll  = (float)wit_s16(&ang[0]) / WIT_ANGLE_LSB;
    out->pitch = (float)wit_s16(&ang[2]) / WIT_ANGLE_LSB;
    out->yaw   = (float)wit_s16(&ang[4]) / WIT_ANGLE_LSB;

    out->temperature  = (float)wit_s16(&ang[6]) / 100.0f;
    out->timestamp_us = (uint64_t)esp_timer_get_time();
    return ESP_OK;
}

/* ===== 默认配置 ===== */
imu_config_t imu_get_default_config(void)
{
    imu_config_t cfg = {
        .sda_gpio    = 16,
        .scl_gpio    = 18,
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

/**
 * 纯地址探测: 只发 START + 地址字节, 看器件是否 ACK, 不访问任何寄存器.
 *
 * 这才是扫总线的正确做法 —— 用"读某个寄存器"去判断器件是否存在会漏掉
 * 没有该寄存器的器件 (例如 PCA9685 就没有 0x75, 会被整个漏掉).
 */
static bool i2c_addr_alive(i2c_port_t port, uint8_t addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK;
}

/**
 * 扫描一条总线 (0x08~0x77), 打印所有应答的地址并尽量识别型号.
 *
 * @param port      总线号
 * @param mpu_addr  输出: WHO_AM_I==0x68 的地址 (没有则 0); 可为 NULL
 * @param wit_addr  输出: IICADDR(0x1A) 低字节 == 自身地址 的地址, 即 10 轴 IMU
 *                        (没有则 0); 可为 NULL
 */
static void i2c_bus_scan(i2c_port_t port, uint8_t *mpu_addr, uint8_t *wit_addr)
{
    ESP_LOGW(TAG, "扫描 I2C%d 总线 (0x08~0x77)...", (int)port);

    if (mpu_addr) *mpu_addr = 0;
    if (wit_addr) *wit_addr = 0;

    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (!i2c_addr_alive(port, addr)) {
            continue;
        }
        found++;

        /* 逐个试读几个特征寄存器, 帮助识别型号 */
        uint8_t who = 0, id[2] = {0}, ver[2] = {0}, roll[2] = {0};
        bool r_who  = (i2c_read_reg (port, addr, REG_WHO_AM_I,    &who) == ESP_OK);
        bool r_1a   = (i2c_read_regs(port, addr, WIT_REG_IICADDR, id,   2) == ESP_OK);
        bool r_2e   = (i2c_read_regs(port, addr, WIT_REG_VERSION, ver,  2) == ESP_OK);
        bool r_3d   = (i2c_read_regs(port, addr, WIT_REG_ROLL,    roll, 2) == ESP_OK);

        ESP_LOGW(TAG, "  0x%02X 有应答 | 0x75:%s 0x1A:%s 0x2E:%s 0x3D:%s", addr,
                 r_who ? "OK" : "--", r_1a ? "OK" : "--",
                 r_2e  ? "OK" : "--", r_3d ? "OK" : "--");
        if (r_who) ESP_LOGW(TAG, "      WHO_AM_I = 0x%02X", who);
        if (r_1a)  ESP_LOGW(TAG, "      IICADDR  = 0x%02X", id[0]);
        if (r_2e)  ESP_LOGW(TAG, "      VERSION  = 0x%04X", (unsigned)((ver[1] << 8) | ver[0]));
        if (r_3d)  ESP_LOGW(TAG, "      Roll_raw = %d", (int)wit_s16(roll));

        bool is_mpu = r_who && (who == MPU6050_WHO_AM_I_VAL);

        /* 识别 10 轴 IMU (没有 WHO_AM_I, 只能靠物理约束):
         *   VERSION(0x2E) 非 0 排除"读什么都返回 0"的器件 (PCA9685 就是), 再加任一条:
         *     a) 加速度模长落在 0.4~1.6 g —— 静止时必然是 1g, 随机数据几乎不可能满足
         *     b) IICADDR(0x1A) 回读值 == 自身地址 —— 模块自洽特征
         */
        bool is_wit = false;
        if (!is_mpu && r_2e && (((unsigned)ver[1] << 8) | ver[0]) != 0) {
            uint8_t acc[12] = {0};
            if (i2c_read_regs(port, addr, WIT_REG_ACC_X, acc, sizeof(acc)) == ESP_OK) {
                float ax = (float)wit_s16(&acc[0]) / WIT_ACCEL_LSB;
                float ay = (float)wit_s16(&acc[2]) / WIT_ACCEL_LSB;
                float az = (float)wit_s16(&acc[4]) / WIT_ACCEL_LSB;
                float g  = sqrtf(ax * ax + ay * ay + az * az);
                ESP_LOGW(TAG, "      加速度 = (%+.2f %+.2f %+.2f) g, |a| = %.2f g",
                         ax, ay, az, g);
                is_wit = (g > 0.4f && g < 1.6f);
            }
            if (!is_wit && r_1a && id[0] == addr) {
                is_wit = true;
            }
        }

        if (is_mpu) {
            ESP_LOGW(TAG, "      ^^ 判定为 MPU6050");
            if (mpu_addr && *mpu_addr == 0) *mpu_addr = addr;
        } else if (is_wit) {
            ESP_LOGW(TAG, "      ^^ 判定为 10 轴 IMU");
            if (wit_addr && *wit_addr == 0) *wit_addr = addr;
        }
    }

    if (found == 0) {
        ESP_LOGE(TAG, "总线上没有任何设备应答 => 检查 SDA/SCL 是否接反、VCC/GND 是否接好");
    } else {
        ESP_LOGW(TAG, "扫描结束, 共 %d 个设备应答", found);
    }
}

/**
 * I2C 引脚的物理层自检 (必须在 i2c_param_config 之前做, 用普通 GPIO 方式).
 *
 * 1) 关掉 ESP32 内部上拉后读空闲电平:
 *      两线都是 1 => 外部上拉存在 => 模块确实挂在这条线上并且已供电
 *      有 0       => 缺外部上拉  => 线没接通 / 模块没供电 / 该线被拉死
 *    (模块板上自带 4.7k 上拉, 所以"模块在线"时这里必然是 1)
 *
 * 2) 把引脚当普通输出驱动 0/1 再回读, 验证引脚本身的输出驱动与输入通路,
 *    用来区分"外部线路问题"和"这颗 GPIO 已经损坏"。
 *
 * 只对本组件自己安装的总线 (I2C0) 做 —— 做完紧接着 param_config 把引脚交还给 I2C 外设。
 */
static void i2c_pins_selftest(const char *name, int sda_gpio, int scl_gpio)
{
    const int pins[2] = { sda_gpio, scl_gpio };
    int       idle[2] = {0};
    int       drv[2][2] = {{0}};

    for (int i = 0; i < 2; i++) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << pins[i],
            .mode         = GPIO_MODE_INPUT,     /* 高阻输入, 关内部上拉 */
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
    }
    vTaskDelay(pdMS_TO_TICKS(2));

    for (int i = 0; i < 2; i++) {
        idle[i] = gpio_get_level((gpio_num_t)pins[i]);

        for (int lv = 0; lv < 2; lv++) {
            gpio_config_t io = {
                .pin_bit_mask = 1ULL << pins[i],
                .mode         = GPIO_MODE_INPUT_OUTPUT,
                .pull_up_en   = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type    = GPIO_INTR_DISABLE,
            };
            gpio_config(&io);
            gpio_set_level((gpio_num_t)pins[i], lv);
            vTaskDelay(pdMS_TO_TICKS(1));
            drv[i][lv] = gpio_get_level((gpio_num_t)pins[i]);
        }
    }

    ESP_LOGW(TAG, "[%s] 引脚自检 SDA(IO%d): 空闲=%d  输出0回读=%d  输出1回读=%d",
             name, sda_gpio, idle[0], drv[0][0], drv[0][1]);
    ESP_LOGW(TAG, "[%s] 引脚自检 SCL(IO%d): 空闲=%d  输出0回读=%d  输出1回读=%d",
             name, scl_gpio, idle[1], drv[1][0], drv[1][1]);

    if (idle[0] && idle[1]) {
        ESP_LOGW(TAG, "[%s] => 外部上拉正常: 模块确实挂在这条线上并且已供电", name);
    } else {
        ESP_LOGE(TAG, "[%s] => 缺少外部上拉: 线没接通 / 模块没供电 / 该线被拉死",
                 name);
    }
    if (drv[0][0] != 0 || drv[0][1] != 1 || drv[1][0] != 0 || drv[1][1] != 1) {
        ESP_LOGE(TAG, "[%s] => 引脚驱动/回读异常: 这颗 GPIO 可能已损坏或被硬短路", name);
    }
}

esp_err_t imu_init_role(imu_role_t role, const imu_config_t *cfg)
{
    if (cfg == NULL) return ESP_ERR_INVALID_ARG;
    if (role >= IMU_ROLE_COUNT) return ESP_ERR_INVALID_ARG;
    if (s_dev[role].initialized) {
        ESP_LOGW(TAG, "[%s] 已初始化, 跳过", role_name(role));
        return ESP_OK;
    }

    const i2c_port_t port = s_port[role];
    esp_err_t ret;

    /* 1. 准备总线
     *    云台 (I2C0): 由本组件安装
     *    船体 (I2C1): 该总线已由 servo 组件为 PCA9685 安装, 只借用不安装 ——
     *                 重复 install 会返回 ESP_FAIL, param_config 还会重配引脚 */
    if (!s_bus_ready[role]) {
        if (role == IMU_ROLE_HULL) {
            ESP_LOGI(TAG, "复用 I2C%d (PCA9685 的总线, 由 servo 组件安装, 不重复初始化)",
                     (int)port);
        } else {
            /* 装 I2C 外设之前, 先用普通 GPIO 做一次引脚物理层自检 */
            i2c_pins_selftest(role_name(role), cfg->sda_gpio, cfg->scl_gpio);

            i2c_config_t c = {
                .mode = I2C_MODE_MASTER,
                .sda_io_num = cfg->sda_gpio,
                .sda_pullup_en = GPIO_PULLUP_ENABLE,
                .scl_io_num = cfg->scl_gpio,
                .scl_pullup_en = GPIO_PULLUP_ENABLE,
                .master.clk_speed = cfg->i2c_freq_hz,
            };
            ret = i2c_param_config(port, &c);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "i2c_param_config 失败: %s", esp_err_to_name(ret));
                return ret;
            }
            ret = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "i2c_driver_install 失败: %s", esp_err_to_name(ret));
                return ret;
            }
            ESP_LOGI(TAG, "I2C%d 初始化: SDA=IO%d, SCL=IO%d, %luHz",
                     (int)port, cfg->sda_gpio, cfg->scl_gpio,
                     (unsigned long)cfg->i2c_freq_hz);
        }
        s_bus_ready[role] = true;
    }

    uint8_t addr;

    /* 2. 确定地址: cfg 指定优先, 否则按角色取出厂默认地址 */
    if (cfg->i2c_addr != 0) {
        if (cfg->i2c_addr != MPU6050_I2C_ADDR_LOW &&
            cfg->i2c_addr != MPU6050_I2C_ADDR_HIGH &&
            cfg->i2c_addr != WIT_I2C_ADDR_DEFAULT) {
            ESP_LOGE(TAG, "非法地址 0x%02X", cfg->i2c_addr);
            return ESP_ERR_INVALID_ARG;
        }
        addr = cfg->i2c_addr;
    } else {
        addr = role_default_addr(role);
    }

    /* 3. 船体 10 轴 IMU (WIT 协议): 上电即工作, 只读不写, 没有配置流程 */
    if (role == IMU_ROLE_HULL) {
        ret = wit_probe(port, addr);
        if (ret != ESP_OK && cfg->i2c_addr == 0) {
            /* 默认地址不应答: 扫一遍总线 —— 既把实际应答的地址列出来方便排查接线,
             * 也兜住"模块地址被上位机改过"的情况 */
            ESP_LOGW(TAG, "[%s] 地址 0x%02X 无应答, 扫描总线...", role_name(role), addr);
            uint8_t mpu_a = 0, wit_a = 0;
            i2c_bus_scan(port, &mpu_a, &wit_a);
            if (wit_a != 0) {
                addr = wit_a;
                ESP_LOGW(TAG, "[%s] 改用扫描到的地址 0x%02X", role_name(role), addr);
                ret = wit_probe(port, addr);
            }
        }
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%s] 10 轴 IMU @0x%02X 无应答 (查 3.3V 供电/共地/SDA-SCL 是否接反; "
                     "该总线由 PCA9685 的 servo_init() 建立)",
                     role_name(role), addr);
            return ret;
        }
        s_dev[role].i2c_addr    = addr;
        s_dev[role].initialized = true;
        ESP_LOGI(TAG, "[%s] 10 轴 IMU 就绪 @ 0x%02X (WIT 协议, 内部已解算姿态)",
                 role_name(role), addr);
        return ESP_OK;
    }

    /* 4. 以下为 MPU6050 (云台) 流程 */
    ret = mpu_probe(port, addr);
    if (ret != ESP_OK) {
        if (cfg->i2c_addr != 0) {
            ESP_LOGE(TAG, "[%s] 指定地址 0x%02X 探测失败", role_name(role), addr);
            return ret;
        }
        /* 默认地址没有应答: 扫总线找一个 WHO_AM_I 正确的设备,
         * 既兜住"AD0 接反/悬空", 也兜住上电初期偶发 NACK */
        ESP_LOGW(TAG, "[%s] 地址 0x%02X 无应答, 扫描总线...", role_name(role), addr);
        uint8_t mpu_a = 0, wit_a = 0;
        i2c_bus_scan(port, &mpu_a, &wit_a);
        if (mpu_a == 0) {
            ESP_LOGE(TAG, "[%s] 未找到 MPU6050 (检查接线/上电/AD0)", role_name(role));
            return ret;
        }
        addr = mpu_a;
        ESP_LOGW(TAG, "[%s] 改用扫描到的地址 0x%02X", role_name(role), addr);
    }
    ESP_LOGI(TAG, "[%s] MPU6050 @ 0x%02X (%s)", role_name(role), addr,
             (addr == MPU6050_I2C_ADDR_LOW) ? "AD0=GND" : "AD0=VCC");

    s_dev[role].i2c_addr = addr;
    const uint8_t A = addr;   /* 简写, 下面所有寄存器操作都针对这颗 */

    /* 5. 唤醒 MPU6050 (默认上电后处于 SLEEP 状态) */
    ret = i2c_write_reg(port, A, REG_PWR_MGMT_1, PWR_MGMT_1_CLKSEL_PLL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] 写 PWR_MGMT_1 失败: %s", role_name(role), esp_err_to_name(ret));
        goto err_cleanup;
    }
    vTaskDelay(pdMS_TO_TICKS(100));  /* 等待时钟稳定 */

    /* 6. 低通滤波带宽 (必须先设 DLPF, 它决定陀螺输出速率, 再设分频) */
    ret = i2c_write_reg(port, A, REG_CONFIG, (uint8_t)(MPU6050_DLPF_CFG & 0x07));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] 写 CONFIG 失败: %s", role_name(role), esp_err_to_name(ret));
        goto err_cleanup;
    }

    /* 7. 采样率分频: 基准 1kHz(DLPF 开启) / 8kHz(DLPF 关闭) */
    ret = i2c_write_reg(port, A, REG_SMPLRT_DIV, (uint8_t)MPU6050_SMPLRT_DIV);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] 写 SMPLRT_DIV 失败: %s", role_name(role), esp_err_to_name(ret));
        goto err_cleanup;
    }
    ESP_LOGI(TAG, "[%s] 采样配置: DLPF_CFG=%d, 采样率=%dHz", role_name(role),
             MPU6050_DLPF_CFG,
             ((MPU6050_DLPF_CFG == 0) ? 8000 : 1000) / (1 + MPU6050_SMPLRT_DIV));

    /* 8. 陀螺仪量程 + 关闭自检 */
    s_dev[role].gyro_range = cfg->gyro_range;
    ret = i2c_write_reg(port, A, REG_GYRO_CONFIG,
                        GYRO_CONFIG_SELF_TEST_OFF | gyro_fs_sel(s_dev[role].gyro_range));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] 写 GYRO_CONFIG 失败: %s", role_name(role), esp_err_to_name(ret));
        goto err_cleanup;
    }

    /* 9. 加速度量程 + 关闭自检 */
    s_dev[role].accel_range = cfg->accel_range;
    ret = i2c_write_reg(port, A, REG_ACCEL_CONFIG,
                        ACCEL_CONFIG_SELF_TEST_OFF | accel_fs_sel(s_dev[role].accel_range));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[%s] 写 ACCEL_CONFIG 失败: %s", role_name(role), esp_err_to_name(ret));
        goto err_cleanup;
    }

    /* 10. 验证 WHO_AM_I */
    uint8_t who = 0;
    ret = i2c_read_reg(port, A, REG_WHO_AM_I, &who);
    if (ret != ESP_OK || who != MPU6050_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "[%s] 初始化后 WHO_AM_I 验证失败: 0x%02X", role_name(role), who);
        ret = (ret != ESP_OK) ? ret : ESP_ERR_INVALID_RESPONSE;
        goto err_cleanup;
    }

    s_dev[role].initialized = true;
    ESP_LOGI(TAG, "[%s] 初始化完成: 加速度 ±%dg, 陀螺仪 ±%d°/s", role_name(role),
             (s_dev[role].accel_range == IMU_ACCEL_RANGE_2G) ? 2 :
             (s_dev[role].accel_range == IMU_ACCEL_RANGE_4G) ? 4 :
             (s_dev[role].accel_range == IMU_ACCEL_RANGE_8G) ? 8 : 16,
             (s_dev[role].gyro_range == IMU_GYRO_RANGE_250DPS) ? 250 :
             (s_dev[role].gyro_range == IMU_GYRO_RANGE_500DPS) ? 500 :
             (s_dev[role].gyro_range == IMU_GYRO_RANGE_1000DPS) ? 1000 : 2000);
    return ESP_OK;

err_cleanup:
    /* 只把该角色标记为失败, 不动总线 —— 另一颗可能已经正常工作 */
    s_dev[role].i2c_addr    = 0;
    s_dev[role].initialized = false;
    return ret;
}

/* ===== 读取一帧数据 ===== */
esp_err_t imu_read_role(imu_role_t role, imu_data_t *out)
{
    if (out == NULL) return ESP_ERR_INVALID_ARG;
    if (role >= IMU_ROLE_COUNT) return ESP_ERR_INVALID_ARG;
    if (!s_dev[role].initialized) return ESP_ERR_INVALID_STATE;

    const i2c_port_t port = s_port[role];
    const uint8_t    A    = s_dev[role].i2c_addr;

    /* 船体 10 轴 IMU 走 WIT 协议, 寄存器/字节序都和 MPU6050 不同 */
    if (role == IMU_ROLE_HULL) {
        return wit_read(port, A, out);
    }

    /* 从 ACCEL_XOUT_H 开始连续读 14 字节:
     *   [0..1]  AccX, [2..3]  AccY, [4..5]  AccZ,
     *   [6..7]  Temp,  [8..9]  GyrX, [10..11] GyrY, [12..13] GyrZ
     */
    uint8_t buf[14] = {0};
    esp_err_t ret = i2c_read_regs(port, A, REG_ACCEL_XOUT_H, buf, sizeof(buf));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "[%s] 读传感器数据失败: %s", role_name(role), esp_err_to_name(ret));
        return ret;
    }

    int16_t raw_ax = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t raw_ay = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t raw_az = (int16_t)((buf[4]  << 8) | buf[5]);
    int16_t raw_t  = (int16_t)((buf[6]  << 8) | buf[7]);
    int16_t raw_gx = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

    const float accel_lsb = accel_lsb_per_g(s_dev[role].accel_range);
    const float gyro_lsb  = gyro_lsb_per_dps(s_dev[role].gyro_range);

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

    /* MPU6050 不出姿态角, 由上层 Madgwick 解算 */
    out->roll = out->pitch = out->yaw = 0.0f;

    return ESP_OK;
}

/* ===== 查询是否就绪 ===== */
bool imu_role_ready(imu_role_t role)
{
    if (role >= IMU_ROLE_COUNT) return false;
    return s_dev[role].initialized;
}

/* ===== 手动扫描总线 (排查用, 可反复调用) ===== */
esp_err_t imu_scan_role(imu_role_t role)
{
    if (role >= IMU_ROLE_COUNT) return ESP_ERR_INVALID_ARG;
    if (!s_bus_ready[role]) {
        ESP_LOGE(TAG, "[%s] I2C%d 还没初始化, 无法扫描", role_name(role), (int)s_port[role]);
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t mpu_addr = 0, wit_addr = 0;
    i2c_bus_scan(s_port[role], &mpu_addr, &wit_addr);

    if (mpu_addr) {
        ESP_LOGW(TAG, "[%s] 总线上有 MPU6050 @ 0x%02X", role_name(role), mpu_addr);
    }
    if (wit_addr) {
        ESP_LOGW(TAG, "[%s] 总线上有 10 轴 IMU @ 0x%02X", role_name(role), wit_addr);
    }
    return ESP_OK;
}

/* ===== 反初始化 (释放总线, 所有角色一起失效) ===== */
void imu_deinit(void)
{
    for (int i = 0; i < IMU_ROLE_COUNT; i++) {
        if (!s_dev[i].initialized) continue;
        /* MPU6050 可以进 SLEEP 省电; 船体 10 轴 IMU 有自己的电源管理, 不要写它 */
        if (i == IMU_ROLE_GIMBAL) {
            i2c_write_reg(s_port[i], s_dev[i].i2c_addr, REG_PWR_MGMT_1, PWR_MGMT_1_SLEEP);
        }
        s_dev[i].initialized = false;
        s_dev[i].i2c_addr    = 0;
    }
    /* 只释放本组件安装的 I2C0; I2C1 是 servo 组件为 PCA9685 装的, 不能删 */
    if (s_bus_ready[IMU_ROLE_GIMBAL]) {
        i2c_driver_delete(s_port[IMU_ROLE_GIMBAL]);
        s_bus_ready[IMU_ROLE_GIMBAL] = false;
    }
    s_bus_ready[IMU_ROLE_HULL] = false;
    ESP_LOGI(TAG, "已反初始化");
}
