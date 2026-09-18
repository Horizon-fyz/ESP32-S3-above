/**
 * @file motor.c
 * @brief 电机控制实现 (4 路电调 + 1 路 L298N)
 *
 * LEDC 通道分配 (避免冲突):
 *   ESC1    : Timer0, Channel0  (GPIO41, 左推进)
 *   ESC2    : Timer0, Channel1  (GPIO42, 右推进)
 *   REV ESC1: Timer0, Channel2  (GPIO39, 左反推)
 *   REV ESC2: Timer0, Channel3  (GPIO40, 右反推)
 *   DC      : Timer1, Channel4  (GPIO38, ENA)
 *
 * 电调 PWM 协议 (帧率 **50Hz**, 周期 20000µs, 13 位分辨率):
 *   单向电调 (好盈 SkyWalker V2): 1.1ms (最低/停) ~ 1.94ms (最高), **上电给最低油门才解锁**
 *   脉宽端点与帧率无关; 帧率只影响油门更新快慢与脉宽精度 (见 MOTOR_ESC_FREQ_HZ)
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

/* ===== 电调信号帧率 (油门信号刷新率) =====
 * 好盈 SkyWalker V2 规格: "Refresh rate of the throttle signal: **50Hz to 432Hz**"。
 * 当前取 **50Hz** (周期 20000µs): 标准舵机/电调帧率, 兼容性最好 —— 排查电调问题时优先用它。
 * 13 位分辨率下 1 LSB = 2.44µs。**4 路电调统一使用**。
 *   (可选值参考: 50Hz=响应20ms/LSB2.44µs; 100Hz=10ms/1.22µs; 432Hz=2.3ms/0.28µs)
 *
 * ⚠️ 4 路电调共用 LEDC_TIMER_0, 所以频率必须一致 (不能只改其中一路)。
 * ⚠️ 改这个频率**不需要**重算脉宽端点 (1100/1141/1940µs 与频率无关),
 *    也**不需要**改任何别的地方 —— 周期由 s_esc_period_us 自动跟随, 只改这一个宏即可。
 *    唯一要注意: 脉宽上限受周期约束 (见 motor_set_esc_pulse_us 里的 max_us)。
 *    (历史教训: 曾经 duty 换算写死 20000µs, 一旦改频率脉宽就会整体错掉) */
#define MOTOR_ESC_FREQ_HZ   50

/* 内部状态: 每个电机是否已配置 */
static bool            s_dc_configured   = false;
static bool            s_channel_configured[MOTOR_MAX] = {false};
static motor_config_t  s_cfg;

/* 电调帧率对应的周期 (µs), 由 motor_init() 按实际配置频率写入。
 * 脉宽→占空比换算必须用它, 不能写死 (见上面 MOTOR_ESC_FREQ_HZ 的说明)。 */
static uint32_t s_esc_period_us = (1000000u + MOTOR_ESC_FREQ_HZ / 2) / MOTOR_ESC_FREQ_HZ;

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

    /* 推进电调组 (v5.10 换板 ESP32-S3-ETH: 4 路电调排到右排 4 个连续脚 42/41/40/39)
     * 电调 1 (MOTOR_ESC_1) - GPIO41 = 左推进
     * (v9.0 左右对调: 实机接线是 GPIO42 → 右边, 故左推进改到 GPIO41)
     * 4 路电调统一帧率 MOTOR_ESC_FREQ_HZ —— 它们共用 LEDC_TIMER_0, 频率必须一致 */
    cfg.esc1_gpio         = 41;
    cfg.esc1_freq_hz      = MOTOR_ESC_FREQ_HZ;
    cfg.esc1_ledc_timer   = LEDC_TIMER_0;
    cfg.esc1_ledc_channel = LEDC_CHANNEL_0;

    /* 电调 2 (MOTOR_ESC_2) - GPIO42 = 右推进 */
    cfg.esc2_gpio         = 42;
    cfg.esc2_freq_hz      = MOTOR_ESC_FREQ_HZ;
    cfg.esc2_ledc_timer   = LEDC_TIMER_0;        /* 同 ESC1 共享 Timer0 */
    cfg.esc2_ledc_channel = LEDC_CHANNEL_1;      /* 不同 Channel */

    /* L298N 直流电机 (MOTOR_MAIN_DC) - ENA=38, IN1=48, IN2=47
     * 方向两脚 48/47 相邻, ENA 单独放 38 (避开 strapping 脚 46) */
    cfg.dc_ena_gpio      = 38;
    cfg.dc_in1_gpio      = 48;
    cfg.dc_in2_gpio      = 47;
    cfg.dc_pwm_freq_hz   = 5000;  /* 5kHz, 适合直流电机 (与电调帧率无关) */
    cfg.dc_ledc_timer    = LEDC_TIMER_1;
    cfg.dc_ledc_channel  = LEDC_CHANNEL_4;  /* Channel0~3 已被 4 路电调占用, DC 必须用其它通道 */

    /* 反推电调 1 (MOTOR_THRUST_REV_1) - GPIO39 = 左反推
     * 与 ESC1/ESC2 共享 Timer0, 故同为 MOTOR_ESC_FREQ_HZ */
    cfg.rev1_gpio         = 39;
    cfg.rev1_freq_hz      = MOTOR_ESC_FREQ_HZ;
    cfg.rev1_ledc_timer   = LEDC_TIMER_0;
    cfg.rev1_ledc_channel = LEDC_CHANNEL_2;

    /* 反推电调 2 (MOTOR_THRUST_REV_2) - GPIO40 = 右反推
     * 与 ESC1/ESC2 共享 Timer0, 故同为 MOTOR_ESC_FREQ_HZ */
    cfg.rev2_gpio         = 40;
    cfg.rev2_freq_hz      = MOTOR_ESC_FREQ_HZ;
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

    /* 电调帧率 → 周期 (µs): 4 路电调共用 LEDC_TIMER_0, 频率必须一致,
     * 所以取任一已配置通道的频率即可。脉宽→占空比换算要用它 (不能写死周期)。 */
    {
        uint32_t f = 0;
        if      (cfg->esc1_gpio >= 0) f = cfg->esc1_freq_hz;
        else if (cfg->esc2_gpio >= 0) f = cfg->esc2_freq_hz;
        else if (cfg->rev1_gpio >= 0) f = cfg->rev1_freq_hz;
        else if (cfg->rev2_gpio >= 0) f = cfg->rev2_freq_hz;
        if (f > 0) {
            s_esc_period_us = (1000000u + f / 2) / f;
            ESP_LOGI(TAG, "电调帧率 %luHz → 周期 %luus (13bit, 1 LSB = %.2fus)",
                     (unsigned long)f, (unsigned long)s_esc_period_us,
                     (double)s_esc_period_us / 8192.0);
        }
    }

    /* 电调 1 - GPIO41 (左推进) */
    if (cfg->esc1_gpio >= 0) {
        esp_err_t ret = ledc_setup_channel(cfg->esc1_ledc_timer, cfg->esc1_ledc_channel,
                                            cfg->esc1_gpio, cfg->esc1_freq_hz, LEDC_LOW_SPEED_MODE);
        if (ret != ESP_OK) return ret;
        s_channel_configured[MOTOR_ESC_1] = true;
        s_esc_map[MOTOR_ESC_1] = (esc_ledc_map_t){ LEDC_LOW_SPEED_MODE, cfg->esc1_ledc_channel };
        ESP_LOGI(TAG, "ESC1 enabled on GPIO%d (timer=%d, ch=%d, %luHz)",
                 cfg->esc1_gpio, cfg->esc1_ledc_timer, cfg->esc1_ledc_channel,
                 (unsigned long)cfg->esc1_freq_hz);
    }

    /* 电调 2 - GPIO42 (右推进) */
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

    /* 反推电调 1 - GPIO39 (左反推) */
    if (cfg->rev1_gpio >= 0) {
        esp_err_t ret = ledc_setup_channel(cfg->rev1_ledc_timer, cfg->rev1_ledc_channel,
                                            cfg->rev1_gpio, cfg->rev1_freq_hz, LEDC_LOW_SPEED_MODE);
        if (ret != ESP_OK) return ret;
        s_channel_configured[MOTOR_THRUST_REV_1] = true;
        s_esc_map[MOTOR_THRUST_REV_1] = (esc_ledc_map_t){ LEDC_LOW_SPEED_MODE, cfg->rev1_ledc_channel };
        ESP_LOGI(TAG, "REV ESC1 enabled on GPIO%d (timer=%d, ch=%d)",
                 cfg->rev1_gpio, cfg->rev1_ledc_timer, cfg->rev1_ledc_channel);
    }

    /* 反推电调 2 - GPIO40 (右反推) */
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

/* ===== 电调脉宽常量 (2026-09 实测标定) =====
 * 实测/手册型号: 好盈 SkyWalker V2 80A (天行者 V2, 固定翼无刷电调, **单向**)
 *
 * 实测结果 (用控制台 `sweep` / `pw` 逐个逼近出来的, 存在明显**迟滞**):
 *   启动门槛 = **1141µs**  —— 从冷态必须打到 1141µs 才起转 (1140µs 起不来)
 *   维持下限 =  1136µs    —— 已在转时, 可以降到 1136µs 仍维持运行
 *   停止点   = **1135µs**  —— 降到 1135µs 电机即停
 *   上限     = **1940µs**  —— 满油门
 *   ⇒ 1136~1140µs 是**迟滞带**: 能维持、但不能启动 (单看某个脉宽无法判断电机状态,
 *     必须知道它是"从冷态起转"还是"从高速降下来")。
 *
 * ⚠️ 因此"停机点"必须与"启动门槛"分开, 绝不能用同一个值:
 *   0% 给 **1100µs** (电调自身最低点, 比停止点 1135 再低 35µs 余量):
 *     ① 保证电机**确定停转**; ② 满足电调上电自检的"油门最低位"要求 (已验证可正常解锁)。
 *   若把 0% 定在 1140µs: 落在迟滞带里 ⇒ 已转的电机降到 1140µs 可能**仍然慢转不停**,
 *      对船用是安全隐患。
 *   代价: 1100~1140µs 是"死区"(输出但电机不动), 约占全程 5%, 无副作用。
 *
 * 映射: 0%        -> ESC_PULSE_STOP_US  (1100µs, 硬停 / 上电解锁位)
 *       1%~100%   -> ESC_PULSE_START_US (1141µs, 实测启动门槛) ~ ESC_PULSE_MAX_US (1940µs) 线性
 *
 * 注: 该电调**不支持负油门反转** (反推走独立的"反推刹车"黄线通道, 或物理反装桨),
 *     因此负油门一律按 0 (停) 处理。
 * 注: 若将来换成双向电调 (1500µs=中位, 1000/2000 两端反向), 需把这三个常量改回去。 */
#define ESC_PULSE_STOP_US     1100      /* 油门 0%: 停机点 / 上电解锁位 (必低于停止点 1135) */
#define ESC_PULSE_START_US    1141      /* 油门 1% 起: 实测启动门槛 (1140 起不来, 故取 1141) */
#define ESC_PULSE_MAX_US      1940      /* 油门 100%: 实测上限 */
/* 调试接口允许的输出范围 (比上面宽, 便于人工探端点; 500µs 以下/2500µs 以上对电调无意义) */
#define ESC_PULSE_SAFE_MIN    500
#define ESC_PULSE_SAFE_MAX    2500

esp_err_t motor_set_esc_pulse_us(motor_id_t id, uint32_t pulse_us)
{
    if (id >= MOTOR_MAX || id == MOTOR_MAIN_DC) {
        return ESP_ERR_INVALID_ARG;  /* DC 不受脉宽控制 */
    }
    if (!s_channel_configured[id]) return ESP_ERR_INVALID_STATE;

    /* 上限还受**帧周期**约束: 脉宽不可能超过一个周期 (周期 = 1/MOTOR_ESC_FREQ_HZ)。
     * 这里直接拒绝而不是静默钳位 —— 否则请求一个超周期的脉宽会"成功"返回却实际输出整个周期, 误导排查。 */
    uint32_t max_us = (s_esc_period_us < ESC_PULSE_SAFE_MAX) ? s_esc_period_us : ESC_PULSE_SAFE_MAX;
    if (pulse_us < ESC_PULSE_SAFE_MIN || pulse_us > max_us) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 13 位分辨率: duty = 脉宽 / 周期 × 8192。
     * ⚠️ 周期取自 s_esc_period_us (motor_init 按实际帧率写入), **不是写死的 50Hz/20000µs** ——
     *    否则一旦改帧率, 输出脉宽就会整体错掉 (例如 432Hz 下仍按 20000µs 算, 1100µs 会变成 ~12µs,
     *    电调会认为没有信号)。 */
    uint32_t duty = (uint32_t)(((uint64_t)pulse_us * 8192u) / s_esc_period_us);
    if (duty > 8191) duty = 8191;

    ledc_set_duty(s_esc_map[id].speed_mode, s_esc_map[id].channel, duty);
    ledc_update_duty(s_esc_map[id].speed_mode, s_esc_map[id].channel);
    return ESP_OK;
}

esp_err_t motor_set_esc_throttle(motor_id_t id, float throttle)
{
    if (id >= MOTOR_MAX || id == MOTOR_MAIN_DC) {
        return ESP_ERR_INVALID_ARG;  /* DC 用 set_dc_speed */
    }

    if (throttle <   0.0f) throttle =   0.0f;
    if (throttle > 100.0f) throttle = 100.0f;

    /* 0% 走"硬停"点 (1100µs), 不给启动下限 —— 否则停机不可靠 (见上面迟滞说明) */
    uint32_t pulse_us;
    if (throttle <= 0.0f) {
        pulse_us = ESC_PULSE_STOP_US;
    } else {
        pulse_us = ESC_PULSE_START_US
                 + (uint32_t)((throttle / 100.0f) * (float)(ESC_PULSE_MAX_US - ESC_PULSE_START_US) + 0.5f);
    }
    return motor_set_esc_pulse_us(id, pulse_us);
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
