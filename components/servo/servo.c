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
#include "nvs.h"

static const char *TAG = "servo";

#define I2C_MASTER_NUM    I2C_NUM_1
#define I2C_TIMEOUT_MS    100
#define PCA9685_OSC_HZ    25000000   /* 25 MHz 内部振荡器 */
#define SERVO_MIN_US      1000       /* 0° */
#define SERVO_MAX_US      2000       /* 180° */

/* NVS 标定存储 */
#define SERVO_NVS_NAMESPACE "servo_cal"
#define SERVO_NVS_KEY       "cal"

/* 标定数据版本: 出厂默认值/结构有变更时必须 +1, 使旧 NVS 记录失效并按新默认重写 */
#define SERVO_CAL_VERSION   9

typedef struct {
    uint8_t     version;
    servo_cal_t cal[SERVO_CHANNEL_COUNT];
} servo_cal_blob_t;

/**
 * 出厂默认标定 (按通道), 依据实测结果
 *   ch0: 360° 位置舵机 (平面旋转云台)
 *        - 实测机械量程 530~2730µs 对应 0~360° (6.11µs/°)
 *        - 中点 = (530+2730)/2 = 1630µs = 180°
 *          (注: 曾目视得"中位 1650µs", 与端点推算差 20µs≈3°, 以端点为准)
 *   ch1: 180° 位置舵机 (俯仰云台)
 *        - 实测真实量程 600~2700µs 对应 0~180° (11.67µs/°)
 *          * 低端端点: 实测 600µs 以下舵机不动
 *          * 高端端点: 实测 2700µs 为机械最大
 *          * 中点 1650µs = 实测正向上 = 90°
 *        - 规格书标注 500~2500µs 与实际不符, 以实测为准
 *
 * 物理行程 phys_range_deg 由 MPU 实测得到 (见"两套角度刻度"说明):
 *   ch0: 命令 360° 实际转过 365.4°        -> 取 365
 *   ch1: 命令 30° 实际转过 31.9° (四档一致) -> 192/180 = 1.0667 ≈ 实测 1.064
 *
 * 行程限位 (limit_min_deg/limit_max_deg) 是"允许运行的角度窗口"(标称角域),
 * 与标定端点互相独立, 不改变角度↔脉宽换算; 只把命令角钳位在窗口内, 避免长期
 * 顶在机械硬限位上堵转发热。
 *   ch1: 限 30~150° (两端各留 30° 余量), 对应脉宽 950~2350µs
 */
static const servo_cal_t s_cal_factory[SERVO_CHANNEL_COUNT] = {
    [0] = { .min_us =  530, .max_us = 2730, .trim_us =   0, .range_deg = 360,
            .phys_range_deg = 365, .limit_min_deg = 0, .limit_max_deg = 0 },  /* 不限 */
    [1] = { .min_us =  600, .max_us = 2700, .trim_us =   0, .range_deg = 180,
            .phys_range_deg = 192, .limit_min_deg = 30, .limit_max_deg = 150 },
};

static bool           s_initialized = false;
static uint8_t        s_i2c_addr    = 0x40;
static servo_config_t s_cfg;
static servo_cal_t    s_cal[SERVO_CHANNEL_COUNT];
static uint16_t       s_pulse_us[SERVO_CHANNEL_COUNT];

/* 取某通道的出厂默认标定 */
static servo_cal_t cal_factory(uint8_t channel)
{
    if (channel < SERVO_CHANNEL_COUNT) {
        return s_cal_factory[channel];
    }
    servo_cal_t cal = {
        .min_us = SERVO_MIN_US, .max_us = SERVO_MAX_US,
        .trim_us = 0, .range_deg = 180, .phys_range_deg = 0,
        .limit_min_deg = 0, .limit_max_deg = 0,
    };
    return cal;
}

/* 物理角 / 标称角 的比例 (phys_range_deg = 0 时视为 1.0, 即两套刻度相同) */
static float cal_phys_ratio(const servo_cal_t *cal)
{
    if (cal->phys_range_deg == 0 || cal->range_deg == 0) {
        return 1.0f;
    }
    return (float)cal->phys_range_deg / (float)cal->range_deg;
}

/* 把命令角钳位到该通道的行程限位内 */
static float cal_clamp_deg(const servo_cal_t *cal, float deg)
{
    float lo = (cal->limit_min_deg == 0) ? 0.0f : (float)cal->limit_min_deg;
    float hi = (cal->limit_max_deg == 0) ? (float)cal->range_deg
                                         : (float)cal->limit_max_deg;
    if (hi > (float)cal->range_deg) hi = (float)cal->range_deg;
    if (lo > hi) lo = hi;
    if (deg < lo) return lo;
    if (deg > hi) return hi;
    return deg;
}

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

/* 连续读多个寄存器 */
static esp_err_t i2c_read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, s_i2c_addr,
                                         &reg, 1, buf, len,
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
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, s_i2c_addr, buf, sizeof(buf),
                                               I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        /* 关键诊断: I2C 写失败会导致舵机收不到新脉宽, 必须可见 */
        ESP_LOGE(TAG, "ch%u 写 PWM 失败(off=%u): %s", channel, off_count, esp_err_to_name(ret));
    }
    return ret;
}

/* 回读某通道的 LED_ON/LED_OFF 寄存器, 确认芯片确实保存了下发的值 */
static void pca9685_dump_channel(uint8_t channel)
{
    if (channel >= SERVO_CHANNEL_COUNT) return;
    uint8_t b[4] = {0};
    if (i2c_read_regs((uint8_t)(0x06 + 4 * channel), b, sizeof(b)) != ESP_OK) {
        ESP_LOGE(TAG, "ch%u 寄存器回读失败", channel);
        return;
    }
    uint16_t on  = (uint16_t)(b[0] | (b[1] << 8));
    uint16_t off = (uint16_t)(b[2] | (b[3] << 8));
    ESP_LOGI(TAG, "ch%u 回读: ON=%u OFF=%u  (OFF 对应脉宽约 %u us, 当前记录 %u us)",
             channel, on, off,
             (unsigned)((uint32_t)off * 1000000U / ((uint32_t)s_cfg.pwm_freq_hz * 4096U)),
             s_pulse_us[channel]);
}

/* ========== 公共 API ========== */
servo_config_t servo_get_default_config(void)
{
    servo_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.sda_gpio    = 16;          /* v8.0: 原 21 —— GPIO21 是板载 WS2812 数据脚, 让开 */
    cfg.scl_gpio    = 17;          /* v5.10 换板: I2C1 SCL */
    cfg.i2c_freq_hz = 400 * 1000;
    cfg.i2c_addr    = 0x40;
    cfg.pwm_freq_hz = 50;          /* 标准舵机 50Hz */
    cfg.pca9685_enable = true;     /* 默认启用; 置 false = 暂时关停 (见 servo.h) */
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

    /* 1b. [暂时关停] 只保留总线, 不探测/不驱动 PCA9685
     *     I2C1 上还挂着云台 MPU6050 (0x68, 由 imu 组件借用), 所以总线照装不误;
     *     只是不碰 PCA9685 —— s_initialized 保持 false, 所有 servo API 直接返回
     *     ESP_ERR_INVALID_STATE, 不会往不存在的芯片写。 */
    if (!cfg->pca9685_enable) {
        ESP_LOGW(TAG, "PCA9685 已关停 (pca9685_enable=false): I2C%d 总线已装好并保留给云台 MPU6050, 舵机功能关闭",
                 (int)I2C_MASTER_NUM);
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* 2. 探测设备 */
    uint8_t mode1 = 0;
    ret = i2c_read(0x00, &mode1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 not found at 0x%02X", s_i2c_addr);
        /* ⚠️ v5.11.2 起: 这里**不删 I2C1 总线**。
         * 该总线还挂着云台 MPU6050 (0x68), 由 imu 组件借用 (它不重复 install)。
         * 若在此 i2c_driver_delete(), 亚博 IMU 会一起失效, 且后续 `scan 1` / `hull`
         * 在线重探也全部报 "i2c driver not installed" —— 只能重启才能再试。
         * 保留总线: servo 自身功能关闭 (s_initialized 保持 false, 所有 servo API 直接
         * 返回 ESP_ERR_INVALID_STATE, 不会往不存在的芯片写), 但云台 MPU 照常工作。 */
        ESP_LOGW(TAG, "I2C%d 总线保留给云台 MPU6050 使用 (PCA9685 不可用, servo 功能关闭)",
                 (int)I2C_MASTER_NUM);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "PCA9685 found at 0x%02X (MODE1=0x%02X)", s_i2c_addr, mode1);

    /* 3. 进入睡眠 (修改预分频寄存器需要) */
    ret = i2c_write(0x00, 0x10);   /* MODE1.SLEEP=1, RESTART=0 */
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(5));

    /* 4. 设置预分频 -> PWM 频率 */
    /* prescale = round(25e6 / (4096 * freq)) - 1, 有效范围 3~255
     * ⚠️ 夹取必须在 float 上做: prescale 是 uint8_t, 再写 `if (prescale > 255)` 会触发
     *    `-Wtype-limits`(比较恒为假) 警告。*/
    float prescale_f = (float)PCA9685_OSC_HZ / (4096.0f * cfg->pwm_freq_hz) - 1.0f;
    if (prescale_f < 3.0f)   prescale_f = 3.0f;
    if (prescale_f > 255.0f) prescale_f = 255.0f;
    uint8_t prescale = (uint8_t)(prescale_f + 0.5f);
    ret = i2c_write(0xFE, prescale);
    if (ret != ESP_OK) return ret;
    ESP_LOGI(TAG, "PWM freq=%d Hz, prescale=%d", cfg->pwm_freq_hz, prescale);

    /* 5. 退出睡眠, 启用自动递增 */
    ret = i2c_write(0x00, 0xA1);   /* RESTART=1, AI=1, SLEEP=0 */
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(5));

    /* 5b. MODE2: OUTDRV=1 推挽输出 (舵机信号必须推挽, 开漏会驱动不出高电平) */
    ret = i2c_write(0x01, 0x04);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "写 MODE2 失败: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 5c. 回读状态寄存器 (MODE1 bit4=SLEEP, MODE2 bit2=OUTDRV) */
    uint8_t st_mode1 = 0xFF, st_mode2 = 0xFF;
    i2c_read(0x00, &st_mode1);
    i2c_read(0x01, &st_mode2);
    ESP_LOGI(TAG, "PCA9685 状态: MODE1=0x%02X (SLEEP=%d) MODE2=0x%02X (OUTDRV=%d)",
             st_mode1, (st_mode1 >> 4) & 1, st_mode2, (st_mode2 >> 2) & 1);

    /* 6. 初始化标定表 (出厂默认), 并尝试从 NVS 恢复用户标定 */
    for (uint8_t ch = 0; ch < SERVO_CHANNEL_COUNT; ch++) {
        s_cal[ch]      = cal_factory(ch);
        s_pulse_us[ch] = 0;
    }
    s_initialized = true;

    if (servo_load_cal() == ESP_OK) {
        ESP_LOGI(TAG, "已从 NVS 加载舵机标定");
    } else {
        ESP_LOGI(TAG, "使用出厂默认舵机标定 (NVS 无有效记录)");
    }

    /* 7. 全部通道输出各自标定中位 (回中) */
    servo_center_all();

    /* 7b. 回读 ch0/ch1 的 PWM 寄存器, 确认芯片确实保存了下发的脉宽
     *     (若回读值正确但舵机不动 => 问题在舵机供电 V+ 或信号线, 不在 MCU 侧) */
    pca9685_dump_channel(0);
    pca9685_dump_channel(1);

    ESP_LOGI(TAG, "Servo controller ready: %d channels @ %d Hz", SERVO_CHANNEL_COUNT, cfg->pwm_freq_hz);
    return ESP_OK;
}

/* ========== 标定辅助 ========== */

/* 标定中位脉宽 (µs) = (min+max)/2 + trim */
static uint16_t cal_center_us(const servo_cal_t *cal)
{
    int32_t c = ((int32_t)cal->min_us + (int32_t)cal->max_us) / 2 + cal->trim_us;
    if (c < 0)     c = 0;
    if (c > 20000) c = 20000;
    return (uint16_t)c;
}

/* 标定参数合法性钳位 */
static void cal_clamp(servo_cal_t *cal)
{
    if (cal->min_us < 100)    cal->min_us = 100;
    if (cal->min_us > 10000)  cal->min_us = 10000;
    if (cal->max_us < 100)    cal->max_us = 100;
    if (cal->max_us > 10000)  cal->max_us = 10000;
    if (cal->trim_us < -1000) cal->trim_us = -1000;
    if (cal->trim_us >  1000) cal->trim_us =  1000;
    if (cal->range_deg < 90)   cal->range_deg = 90;
    if (cal->range_deg > 360)  cal->range_deg = 360;
}

esp_err_t servo_set_pulse_us(uint8_t channel, uint16_t pulse_us)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (channel >= SERVO_CHANNEL_COUNT) return ESP_ERR_INVALID_ARG;

    /* period_us = 1e6 / freq, count = pulse_us / period_us * 4096 */
    uint16_t count = (uint16_t)((uint32_t)pulse_us * s_cfg.pwm_freq_hz * 4096 / 1000000U);
    esp_err_t ret = pca9685_set_pwm(channel, count);
    if (ret == ESP_OK) {
        s_pulse_us[channel] = pulse_us;
    }
    return ret;
}

esp_err_t servo_set_angle(uint8_t channel, float angle)
{
    if (channel >= SERVO_CHANNEL_COUNT) return ESP_ERR_INVALID_ARG;

    const servo_cal_t *cal = &s_cal[channel];
    float range = (float)cal->range_deg;
    if (range <= 0.0f) range = 180.0f;

    /* 先钳位到软件行程限位, 再钳位到满量程 (限位不会改变角度↔脉宽换算) */
    angle = cal_clamp_deg(cal, angle);
    if (angle < 0.0f)    angle = 0.0f;
    if (angle > range)   angle = range;

    int32_t pulse = (int32_t)cal->min_us +
                    (int32_t)((angle / range) * (float)(cal->max_us - cal->min_us)) +
                    (int32_t)cal->trim_us;
    if (pulse < 0)     pulse = 0;
    if (pulse > 20000) pulse = 20000;
    return servo_set_pulse_us(channel, (uint16_t)pulse);
}

esp_err_t servo_sleep_all(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    for (uint8_t ch = 0; ch < SERVO_CHANNEL_COUNT; ch++) {
        pca9685_set_pwm(ch, 0);
    }
    return i2c_write(0x00, 0x10);  /* MODE1.SLEEP=1 */
}

/* ==================== 标定 API ==================== */

servo_cal_t servo_get_default_cal(void)
{
    servo_cal_t cal = {
        .min_us  = SERVO_MIN_US,
        .max_us  = SERVO_MAX_US,
        .trim_us = 0,
        .range_deg = 180,
        .phys_range_deg = 0,
        .limit_min_deg = 0,
        .limit_max_deg = 0,
    };
    return cal;
}

esp_err_t servo_set_cal(uint8_t channel, const servo_cal_t *cal)
{
    if (cal == NULL) return ESP_ERR_INVALID_ARG;
    if (channel >= SERVO_CHANNEL_COUNT) return ESP_ERR_INVALID_ARG;
    s_cal[channel] = *cal;
    cal_clamp(&s_cal[channel]);
    return ESP_OK;
}

esp_err_t servo_get_cal(uint8_t channel, servo_cal_t *cal)
{
    if (cal == NULL) return ESP_ERR_INVALID_ARG;
    if (channel >= SERVO_CHANNEL_COUNT) return ESP_ERR_INVALID_ARG;
    *cal = s_cal[channel];
    return ESP_OK;
}

esp_err_t servo_reset_cal(uint8_t channel)
{
    if (channel >= SERVO_CHANNEL_COUNT) return ESP_ERR_INVALID_ARG;
    s_cal[channel] = cal_factory(channel);
    return ESP_OK;
}

esp_err_t servo_center(uint8_t channel)
{
    if (channel >= SERVO_CHANNEL_COUNT) return ESP_ERR_INVALID_ARG;
    return servo_set_pulse_us(channel, cal_center_us(&s_cal[channel]));
}

esp_err_t servo_center_all(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    for (uint8_t ch = 0; ch < SERVO_CHANNEL_COUNT; ch++) {
        servo_set_pulse_us(ch, cal_center_us(&s_cal[ch]));
    }
    return ESP_OK;
}

esp_err_t servo_get_pulse_us(uint8_t channel, uint16_t *pulse_us)
{
    if (pulse_us == NULL) return ESP_ERR_INVALID_ARG;
    if (channel >= SERVO_CHANNEL_COUNT) return ESP_ERR_INVALID_ARG;
    *pulse_us = s_pulse_us[channel];
    return ESP_OK;
}

/* 由最近一次下发的脉宽反算角度:  pulse = min + a/range*(max-min) + trim */
esp_err_t servo_get_angle(uint8_t channel, float *angle)
{
    if (angle == NULL) return ESP_ERR_INVALID_ARG;
    if (channel >= SERVO_CHANNEL_COUNT) return ESP_ERR_INVALID_ARG;

    const servo_cal_t *cal = &s_cal[channel];
    int32_t span = (int32_t)cal->max_us - (int32_t)cal->min_us;
    if (span <= 0 || cal->range_deg == 0) {
        *angle = 0.0f;
        return ESP_ERR_INVALID_STATE;
    }

    float range = (float)cal->range_deg;
    float a = ((float)s_pulse_us[channel] - (float)cal->min_us - (float)cal->trim_us)
              * range / (float)span;
    if (a < 0.0f)     a = 0.0f;
    if (a > range)    a = range;
    *angle = a;
    return ESP_OK;
}

esp_err_t servo_get_limit(uint8_t channel, float *min_deg, float *max_deg)
{
    if (min_deg == NULL || max_deg == NULL) return ESP_ERR_INVALID_ARG;
    if (channel >= SERVO_CHANNEL_COUNT) return ESP_ERR_INVALID_ARG;

    const servo_cal_t *cal = &s_cal[channel];
    float lo = (cal->limit_min_deg == 0) ? 0.0f : (float)cal->limit_min_deg;
    float hi = (cal->limit_max_deg == 0) ? (float)cal->range_deg
                                         : (float)cal->limit_max_deg;
    if (hi > (float)cal->range_deg) hi = (float)cal->range_deg;
    if (lo > hi) lo = hi;
    *min_deg = lo;
    *max_deg = hi;
    return ESP_OK;
}

/* ==================== 标称角 ↔ 物理角 换算 ==================== */

float servo_cmd_to_phys_deg(uint8_t channel, float cmd_deg)
{
    if (channel >= SERVO_CHANNEL_COUNT) return cmd_deg;
    return cmd_deg * cal_phys_ratio(&s_cal[channel]);
}

float servo_phys_to_cmd_deg(uint8_t channel, float phys_deg)
{
    if (channel >= SERVO_CHANNEL_COUNT) return phys_deg;
    float r = cal_phys_ratio(&s_cal[channel]);
    if (r <= 0.0f) return phys_deg;
    return phys_deg / r;
}

esp_err_t servo_set_phys_angle(uint8_t channel, float phys_deg)
{
    if (channel >= SERVO_CHANNEL_COUNT) return ESP_ERR_INVALID_ARG;
    return servo_set_angle(channel, servo_phys_to_cmd_deg(channel, phys_deg));
}

esp_err_t servo_save_cal(void)
{
    servo_cal_blob_t blob;
    blob.version = SERVO_CAL_VERSION;
    memcpy(blob.cal, s_cal, sizeof(blob.cal));

    nvs_handle_t h;
    esp_err_t ret = nvs_open(SERVO_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open 失败: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = nvs_set_blob(h, SERVO_NVS_KEY, &blob, sizeof(blob));
    if (ret == ESP_OK) {
        ret = nvs_commit(h);
    }
    nvs_close(h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "保存标定失败: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t servo_load_cal(void)
{
    servo_cal_blob_t blob;

    nvs_handle_t h;
    esp_err_t ret = nvs_open(SERVO_NVS_NAMESPACE, NVS_READONLY, &h);
    if (ret != ESP_OK) {
        return ret;   /* 无记录属于正常情况, 保持出厂默认标定 */
    }
    size_t len = sizeof(blob);
    ret = nvs_get_blob(h, SERVO_NVS_KEY, &blob, &len);
    nvs_close(h);
    if (ret != ESP_OK || len != sizeof(blob)) {
        return ESP_ERR_NOT_FOUND;
    }
    if (blob.version != SERVO_CAL_VERSION) {
        ESP_LOGW(TAG, "NVS 标定版本 %u != %u, 忽略并按出厂默认重写",
                 blob.version, SERVO_CAL_VERSION);
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(s_cal, blob.cal, sizeof(s_cal));
    for (uint8_t ch = 0; ch < SERVO_CHANNEL_COUNT; ch++) {
        cal_clamp(&s_cal[ch]);
    }
    return ESP_OK;
}

bool servo_is_ready(void)
{
    return s_initialized;
}

void servo_deinit(void)
{
    /* 只有成功初始化过才释放总线。若当初 PCA9685 就探测失败 (s_initialized=false),
     * 这条 I2C1 是**留给云台 MPU6050** 的 (见 servo_init 探测失败分支), 不能在这里删。 */
    if (s_initialized) {
        servo_sleep_all();
        i2c_driver_delete(I2C_MASTER_NUM);
        s_initialized = false;
    }
}
