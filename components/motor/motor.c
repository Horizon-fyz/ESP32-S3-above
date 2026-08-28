/**
 * @file motor.c
 * @brief 电机控制实现 (4 路电调 + 1 路 L298N)
 *
 * LEDC 通道分配 (避免冲突):
 *   ESC1    : Timer0, Channel0  (GPIO1)
 *   ESC2    : Timer0, Channel1  (GPIO42)
 *   REV ESC1: Timer0, Channel2  (GPIO2, 反推)
 *   REV ESC2: Timer0, Channel3  (GPIO3, 反推)
 *   DC      : Timer1, Channel0  (GPIO16)
 *
 * 电调 PWM 协议 (50Hz, 20ms 周期, 13 位分辨率):
 *   1.0ms (反向最大) ~ 1.5ms (中位) ~ 2.0ms (正向最大)
 *
 * L298N 调速 (5kHz, 13 位分辨率):
 *   ENA PWM 调速, IN1/IN2 数字控制方向
 *
 * 启动时所有输出为 0, 防止上电失控.
 */

#include "motor.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "motor";

/* 内部状态: 每个电机是否已配置 */
static bool            s_dc_configured   = false;
static bool            s_channel_configured[MOTOR_MAX] = {false};
static motor_config_t  s_cfg;

/* 电调 → LEDC 通道映射 (运行时确定, 用于 set_duty) */
typedef struct {
    ledc_mode_t  speed_mode;
    ledc_channel_t channel;
} esc_ledc_map_t;

static esc_ledc_map_t s_esc_map[MOTOR_MAX];

/* ========== LEDC 配置辅助 ========== */
static esp_err_t ledc_setup_channel(ledc_timer_t timer, ledc_channel_t channel,
                                     int gpio, uint32_t freq_hz, ledc_mode_t speed_mode)
{
    ledc_timer_config_t timer_cfg = {
        .duty_resolution = LEDC_TIMER_13_BIT,   /* 13 位分辨率, 满足电调精度 */
        .freq_hz         = freq_hz,
        .speed_mode      = speed_mode,
        .timer_num       = timer,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ledc_channel_config_t ch_cfg = {
        .gpio_num   = gpio,
        .speed_mode = speed_mode,
        .channel    = channel,
        .timer_sel  = timer,
        .duty       = 0,
        .hpoint     = 0,
    };
    ret = ledc_channel_config(&ch_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

/* ========== 公共 API 实现 ========== */
motor_config_t motor_get_default_config(void)
{
    motor_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    /* 电调 1 (MOTOR_ESC_1) - 用户指定 GPIO1
     * ⚠️ GPIO1 = U0TXD, 与日志串口冲突
     *    启用 USB-Serial/JTAG 日志输出 (CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y)
     *    可释放 GPIO1, 推荐启用 */
    cfg.esc1_gpio         = 1;
    cfg.esc1_freq_hz      = 50;
    cfg.esc1_ledc_timer   = LEDC_TIMER_0;
    cfg.esc1_ledc_channel = LEDC_CHANNEL_0;

    /* 电调 2 (MOTOR_ESC_2) - 用户指定 GPIO42 (MTMS, 可用) */
    cfg.esc2_gpio         = 42;
    cfg.esc2_freq_hz      = 50;
    cfg.esc2_ledc_timer   = LEDC_TIMER_0;        /* 同 ESC1 共享 Timer0 */
    cfg.esc2_ledc_channel = LEDC_CHANNEL_1;      /* 不同 Channel */

    /* L298N 直流电机 (MOTOR_MAIN_DC) - 用户指定 ENA=16, IN1=17, IN2=18 */
    cfg.dc_ena_gpio      = 16;
    cfg.dc_in1_gpio      = 17;
    cfg.dc_in2_gpio      = 18;
    cfg.dc_pwm_freq_hz   = 5000;  /* 5kHz, 适合直流电机 */
    cfg.dc_ledc_timer    = LEDC_TIMER_1;
    cfg.dc_ledc_channel  = LEDC_CHANNEL_0;

    /* 反推电调 1 (MOTOR_THRUST_REV_1) - GPIO2
     * 与 ESC1/ESC2 共享 Timer0 (50Hz), 使用 Channel2 */
    cfg.rev1_gpio         = 2;
    cfg.rev1_freq_hz      = 50;
    cfg.rev1_ledc_timer   = LEDC_TIMER_0;
    cfg.rev1_ledc_channel = LEDC_CHANNEL_2;

    /* 反推电调 2 (MOTOR_THRUST_REV_2) - GPIO3
     * 与 ESC1/ESC2 共享 Timer0 (50Hz), 使用 Channel3 */
    cfg.rev2_gpio         = 3;
    cfg.rev2_freq_hz      = 50;
    cfg.rev2_ledc_timer   = LEDC_TIMER_0;
    cfg.rev2_ledc_channel = LEDC_CHANNEL_3;

    return cfg;
}

esp_err_t motor_init(const motor_config_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(&s_cfg, cfg, sizeof(s_cfg));

    /* 电调 1 - GPIO1 */
    if (cfg->esc1_gpio >= 0) {
        esp_err_t ret = ledc_setup_channel(cfg->esc1_ledc_timer, cfg->esc1_ledc_channel,
                                            cfg->esc1_gpio, cfg->esc1_freq_hz, LEDC_LOW_SPEED_MODE);
        if (ret != ESP_OK) return ret;
        s_channel_configured[MOTOR_ESC_1] = true;
        s_esc_map[MOTOR_ESC_1] = (esc_ledc_map_t){ LEDC_LOW_SPEED_MODE, cfg->esc1_ledc_channel };
        ESP_LOGW(TAG, "ESC1 enabled on GPIO%d (U0TXD, log via USB-Serial/JTAG required)",
                 cfg->esc1_gpio);
    }

    /* 电调 2 - GPIO42 */
    if (cfg->esc2_gpio >= 0) {
        /* 检查是否与 ESC1 共享 timer 但频率不同 */
        if (cfg->esc1_gpio >= 0 && cfg->esc1_ledc_timer == cfg->esc2_ledc_timer
            && cfg->esc1_freq_hz != cfg->esc2_freq_hz) {
            ESP_LOGE(TAG, "ESC1/ESC2 share timer but freq differs (%d vs %d)",
                     cfg->esc1_freq_hz, cfg->esc2_freq_hz);
            return ESP_ERR_INVALID_ARG;
        }
        esp_err_t ret = ledc_setup_channel(cfg->esc2_ledc_timer, cfg->esc2_ledc_channel,
                                            cfg->esc2_gpio, cfg->esc2_freq_hz, LEDC_LOW_SPEED_MODE);
        if (ret != ESP_OK) return ret;
        s_channel_configured[MOTOR_ESC_2] = true;
        s_esc_map[MOTOR_ESC_2] = (esc_ledc_map_t){ LEDC_LOW_SPEED_MODE, cfg->esc2_ledc_channel };
        ESP_LOGI(TAG, "ESC2 enabled on GPIO%d (timer=%d, ch=%d)",
                 cfg->esc2_gpio, cfg->esc2_ledc_timer, cfg->esc2_ledc_channel);
    }

    /* 反推电调 1 - GPIO2 */
    if (cfg->rev1_gpio >= 0) {
        esp_err_t ret = ledc_setup_channel(cfg->rev1_ledc_timer, cfg->rev1_ledc_channel,
                                            cfg->rev1_gpio, cfg->rev1_freq_hz, LEDC_LOW_SPEED_MODE);
        if (ret != ESP_OK) return ret;
        s_channel_configured[MOTOR_THRUST_REV_1] = true;
        s_esc_map[MOTOR_THRUST_REV_1] = (esc_ledc_map_t){ LEDC_LOW_SPEED_MODE, cfg->rev1_ledc_channel };
        ESP_LOGI(TAG, "REV ESC1 enabled on GPIO%d (timer=%d, ch=%d)",
                 cfg->rev1_gpio, cfg->rev1_ledc_timer, cfg->rev1_ledc_channel);
    }

    /* 反推电调 2 - GPIO3 */
    if (cfg->rev2_gpio >= 0) {
        esp_err_t ret = ledc_setup_channel(cfg->rev2_ledc_timer, cfg->rev2_ledc_channel,
                                            cfg->rev2_gpio, cfg->rev2_freq_hz, LEDC_LOW_SPEED_MODE);
        if (ret != ESP_OK) return ret;
        s_channel_configured[MOTOR_THRUST_REV_2] = true;
        s_esc_map[MOTOR_THRUST_REV_2] = (esc_ledc_map_t){ LEDC_LOW_SPEED_MODE, cfg->rev2_ledc_channel };
        ESP_LOGI(TAG, "REV ESC2 enabled on GPIO%d (timer=%d, ch=%d)",
                 cfg->rev2_gpio, cfg->rev2_ledc_timer, cfg->rev2_ledc_channel);
    }

    /* L298N 直流电机 - ENA 调速 + IN1/IN2 方向 */
    if (cfg->dc_ena_gpio >= 0) {
        esp_err_t ret = ledc_setup_channel(cfg->dc_ledc_timer, cfg->dc_ledc_channel,
                                            cfg->dc_ena_gpio, cfg->dc_pwm_freq_hz, LEDC_LOW_SPEED_MODE);
        if (ret != ESP_OK) return ret;
        s_dc_configured = true;
        s_esc_map[MOTOR_MAIN_DC] = (esc_ledc_map_t){ LEDC_LOW_SPEED_MODE, cfg->dc_ledc_channel };
        ESP_LOGI(TAG, "L298N DC motor: ena=%d, in1=%d, in2=%d",
                 cfg->dc_ena_gpio, cfg->dc_in1_gpio, cfg->dc_in2_gpio);
    }

    if (cfg->dc_in1_gpio >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << cfg->dc_in1_gpio),
            .mode = GPIO_MODE_OUTPUT,
        };
        gpio_config(&io);
        gpio_set_level(cfg->dc_in1_gpio, 0);
    }
    if (cfg->dc_in2_gpio >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << cfg->dc_in2_gpio),
            .mode = GPIO_MODE_OUTPUT,
        };
        gpio_config(&io);
        gpio_set_level(cfg->dc_in2_gpio, 0);
    }

    /* 启动时强制紧急停止 */
    motor_emergency_stop();
    return ESP_OK;
}

esp_err_t motor_set_esc_throttle(motor_id_t id, float throttle)
{
    if (id >= MOTOR_MAX || id == MOTOR_MAIN_DC) {
        return ESP_ERR_INVALID_ARG;  /* DC 用 set_dc_speed */
    }
    if (!s_channel_configured[id]) return ESP_ERR_INVALID_STATE;

    if (throttle >  100.0f) throttle =  100.0f;
    if (throttle < -100.0f) throttle = -100.0f;

    /* 标准电调: 1ms = 反向最大, 1.5ms = 中位, 2ms = 正向最大 (50Hz, 20ms 周期)
     * 13 位分辨率: duty 0..8191 对应 0%~100%
     * 中位 = 1.5ms / 20ms = 7.5%, 占空比值 = 8191 * 0.075 ≈ 614
     * ±0.5ms 范围: (0.5ms / 20ms) * 8191 ≈ 205
     */
    const uint32_t mid_duty   = 614;
    const int32_t  range_duty = 205;
    int32_t duty = mid_duty + (int32_t)((throttle / 100.0f) * range_duty);
    if (duty < 0) duty = 0;
    if (duty > 8191) duty = 8191;

    ledc_set_duty(s_esc_map[id].speed_mode, s_esc_map[id].channel, (uint32_t)duty);
    ledc_update_duty(s_esc_map[id].speed_mode, s_esc_map[id].channel);
    return ESP_OK;
}

esp_err_t motor_set_dc_speed(motor_id_t id, float speed, motor_dir_t dir)
{
    if (id != MOTOR_MAIN_DC || !s_dc_configured) {
        return ESP_ERR_INVALID_STATE;
    }
    if (speed < 0)   speed = 0;
    if (speed > 100) speed = 100;

    /* 方向控制 */
    if (s_cfg.dc_in1_gpio >= 0 && s_cfg.dc_in2_gpio >= 0) {
        switch (dir) {
        case MOTOR_DIR_FORWARD:
            gpio_set_level(s_cfg.dc_in1_gpio, 1);
            gpio_set_level(s_cfg.dc_in2_gpio, 0);
            break;
        case MOTOR_DIR_REVERSE:
            gpio_set_level(s_cfg.dc_in1_gpio, 0);
            gpio_set_level(s_cfg.dc_in2_gpio, 1);
            break;
        case MOTOR_DIR_STOP:
        default:
            gpio_set_level(s_cfg.dc_in1_gpio, 0);
            gpio_set_level(s_cfg.dc_in2_gpio, 0);
            break;
        }
    }

    /* PWM 调速 (13 位分辨率, 0~8191) */
    uint32_t duty = (uint32_t)((speed / 100.0f) * 8191.0f);
    ledc_set_duty(s_esc_map[MOTOR_MAIN_DC].speed_mode,
                  s_esc_map[MOTOR_MAIN_DC].channel, duty);
    ledc_update_duty(s_esc_map[MOTOR_MAIN_DC].speed_mode,
                     s_esc_map[MOTOR_MAIN_DC].channel);
    return ESP_OK;
}

void motor_emergency_stop(void)
{
    /* 全部电调停止 */
    for (int i = 0; i < MOTOR_MAX; i++) {
        if (i == MOTOR_MAIN_DC) {
            if (s_dc_configured) {
                motor_set_dc_speed(MOTOR_MAIN_DC, 0, MOTOR_DIR_STOP);
            }
        } else if (s_channel_configured[i]) {
            motor_set_esc_throttle((motor_id_t)i, 0);
        }
    }
}

void motor_deinit(void)
{
    motor_emergency_stop();
    memset(s_channel_configured, 0, sizeof(s_channel_configured));
    s_dc_configured = false;
}
