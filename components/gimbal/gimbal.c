/**
 * @file gimbal.c
 * @brief 二维云台运动规划实现 (梯形速度曲线, 周期下发)
 *
 * 每个控制周期对每个"运动中"的轴做如下计算:
 *   1. 剩余角度 dist = target - current
 *   2. 制动允许速度 v_brake = sqrt(2 * max_acc * |dist|)
 *   3. 期望速度 v_des = sign(dist) * min(max_vel, v_brake)
 *   4. 按 max_acc 限制把当前速度逼近 v_des
 *   5. current += vel * dt, 并下发给舵机
 * 到达判定后把规划点钉在目标上, 速度清零。
 */

#include "gimbal.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "servo.h"

static const char *TAG = "gimbal";

#define GIMBAL_UPDATE_HZ    100     /* 规划/下发频率 */
#define GIMBAL_EPS_DEG      0.05f   /* 到达判定阈值 (°) */
#define GIMBAL_EPS_DPS      0.5f    /* 速度归零判定阈值 (°/s) */

#define GIMBAL_DEF_VEL_DPS  90.0f   /* 默认限速 */
#define GIMBAL_DEF_ACC_DPS2 180.0f  /* 默认限加速 */

/* ---- 姿态闭环参数 ---- */
#define GIMBAL_STAB_MAX_RATE_DPS  120.0f  /* 闭环输出速率上限 (°/s), 防止大误差时猛冲 */
/* 积分限幅 (°·s): 取小一点, 大误差阶段输出被速率限制饱和时积分仍在累积,
 * 限幅小才能避免饱和结束后多推一把造成过冲。ki=0.5 时最多贡献 5°。 */
#define GIMBAL_INTEG_LIMIT         10.0f

static gimbal_limit_t s_limit[GIMBAL_CH_COUNT];
static gimbal_state_t s_state[GIMBAL_CH_COUNT];
static bool           s_inited = false;

/* ---- 姿态闭环状态 ---- */
static gimbal_pid_t        s_pid[GIMBAL_CH_COUNT]  = {
    [0] = { .kp = 0.8f, .ki = 0.5f, .kd = 0.0f },
    [1] = { .kp = 0.8f, .ki = 0.5f, .kd = 0.0f },
};
static gimbal_stab_state_t s_stab[GIMBAL_CH_COUNT];      /* 对外可见部分 */
static struct {                                          /* 闭环内部量 */
    float integ;
    float err_prev;
} s_stab_i[GIMBAL_CH_COUNT];

/* 反馈方向: **目标物理角增大**时, 标称角该往哪边走 (+1 同向 / -1 反向)。
 * 实测结论 (见 main.c 的实测记录):
 *   ch1 (俯仰): -1 —— 标称角增大时 MPU 的 pitch 减小
 *   ch0 (竖直): +1 —— 标称角增大时融合 yaw 增大
 */
static float s_fb_sign[GIMBAL_CH_COUNT] = {
    [0] = +1.0f, [1] = -1.0f,
};

/* 最近一次喂入的云台实测姿态 (物理角, °) */
static volatile float s_meas_pitch = 0.0f;
static volatile float s_meas_yaw   = 0.0f;

/**
 * @brief 姿态闭环单步: 增量式 PID, 直接下发标称角
 */
static void gimbal_stab_step(uint8_t ch, float dt)
{
    gimbal_stab_state_t *st  = &s_stab[ch];
    const gimbal_pid_t  *pid = &s_pid[ch];

    st->meas_phys = (ch == 1) ? s_meas_pitch : s_meas_yaw;

    float e = st->target_phys - st->meas_phys;
    st->err = e;

    /* 积分 (限幅抗饱和) */
    s_stab_i[ch].integ += e * dt;
    if (s_stab_i[ch].integ >  GIMBAL_INTEG_LIMIT) s_stab_i[ch].integ =  GIMBAL_INTEG_LIMIT;
    if (s_stab_i[ch].integ < -GIMBAL_INTEG_LIMIT) s_stab_i[ch].integ = -GIMBAL_INTEG_LIMIT;

    float de = (e - s_stab_i[ch].err_prev) / dt;
    s_stab_i[ch].err_prev = e;

    /* 修正量 (物理角意义), 换算成标称角增量 */
    float u     = pid->kp * e + pid->ki * s_stab_i[ch].integ + pid->kd * de;
    float d_cmd = s_fb_sign[ch] * servo_phys_to_cmd_deg(ch, u);

    /* 单周期变化速率限制: 大误差时也不猛冲 */
    float d_max = GIMBAL_STAB_MAX_RATE_DPS * dt;
    if (d_cmd >  d_max) d_cmd =  d_max;
    if (d_cmd < -d_max) d_cmd = -d_max;

    st->cmd_deg += d_cmd;

    /* 同步钳到行程限位, 免得积分持续把输出往外推 */
    float lo = 0.0f, hi = 0.0f;
    if (servo_get_limit(ch, &lo, &hi) == ESP_OK) {
        if (st->cmd_deg < lo) st->cmd_deg = lo;
        if (st->cmd_deg > hi) st->cmd_deg = hi;
    }

    servo_set_angle(ch, st->cmd_deg);
}

static void gimbal_task(void *pvParameters)
{
    const float dt = 1.0f / (float)GIMBAL_UPDATE_HZ;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / GIMBAL_UPDATE_HZ));

        for (uint8_t ch = 0; ch < GIMBAL_CH_COUNT; ch++) {
            /* 闭环接管的通道: 由姿态闭环直接下发, 规划器不再插手 */
            if (s_stab[ch].enabled) {
                gimbal_stab_step(ch, dt);
                continue;
            }

            gimbal_state_t *st = &s_state[ch];
            if (!st->moving) {
                continue;
            }
            const gimbal_limit_t *lim = &s_limit[ch];

            float dist  = st->target_deg - st->current_deg;
            float adist = fabsf(dist);

            /* 制动距离约束: 剩余行程内必须能减到 0 */
            float v_cap = sqrtf(2.0f * lim->max_acc_dps2 * adist);
            if (v_cap > lim->max_vel_dps) {
                v_cap = lim->max_vel_dps;
            }

            float v_des = (dist >= 0.0f) ? v_cap : -v_cap;

            /* 按最大加速度逼近期望速度 */
            float dv     = v_des - st->vel_dps;
            float dv_max = lim->max_acc_dps2 * dt;
            if (dv >  dv_max) dv =  dv_max;
            if (dv < -dv_max) dv = -dv_max;
            st->vel_dps += dv;

            /* 积分得到新的规划点 */
            st->current_deg += st->vel_dps * dt;

            /* 到达判定 */
            if (fabsf(st->target_deg - st->current_deg) < GIMBAL_EPS_DEG &&
                fabsf(st->vel_dps) < GIMBAL_EPS_DPS) {
                st->current_deg = st->target_deg;
                st->vel_dps     = 0.0f;
                st->moving      = false;
                ESP_LOGI(TAG, "ch%u 到位: %.1f°", ch, st->target_deg);
            }

            servo_set_angle(ch, st->current_deg);
        }
    }
}

esp_err_t gimbal_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    for (uint8_t ch = 0; ch < GIMBAL_CH_COUNT; ch++) {
        s_limit[ch].max_vel_dps  = GIMBAL_DEF_VEL_DPS;
        s_limit[ch].max_acc_dps2 = GIMBAL_DEF_ACC_DPS2;

        /* 用舵机当前角度(由最近一次下发的脉宽反算)初始化规划点, 避免首次运动跳变 */
        float a = 0.0f;
        if (servo_get_angle(ch, &a) != ESP_OK) {
            a = 0.0f;
        }
        s_state[ch].target_deg  = a;
        s_state[ch].current_deg = a;
        s_state[ch].vel_dps     = 0.0f;
        s_state[ch].moving      = false;
    }

    if (xTaskCreate(gimbal_task, "gimbal", 3072, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "创建规划任务失败");
        return ESP_FAIL;
    }

    s_inited = true;
    ESP_LOGI(TAG, "云台运动规划已启动 (%dHz, 默认限速 %.0f°/s, 限加速 %.0f°/s²)",
             GIMBAL_UPDATE_HZ, GIMBAL_DEF_VEL_DPS, GIMBAL_DEF_ACC_DPS2);
    return ESP_OK;
}

esp_err_t gimbal_set_limit(uint8_t ch, const gimbal_limit_t *limit)
{
    if (ch >= GIMBAL_CH_COUNT) return ESP_ERR_INVALID_ARG;
    if (limit == NULL)         return ESP_ERR_INVALID_ARG;
    if (limit->max_vel_dps <= 0.0f || limit->max_acc_dps2 <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    s_limit[ch] = *limit;
    return ESP_OK;
}

esp_err_t gimbal_get_limit(uint8_t ch, gimbal_limit_t *limit)
{
    if (ch >= GIMBAL_CH_COUNT) return ESP_ERR_INVALID_ARG;
    if (limit == NULL)         return ESP_ERR_INVALID_ARG;
    *limit = s_limit[ch];
    return ESP_OK;
}

esp_err_t gimbal_move_to(uint8_t ch, float target_deg)
{
    if (ch >= GIMBAL_CH_COUNT) return ESP_ERR_INVALID_ARG;
    if (!s_inited)             return ESP_ERR_INVALID_STATE;
    if (s_stab[ch].enabled) {
        /* 闭环接管时规划器必须让路, 否则两条指令流会互相打架 */
        ESP_LOGW(TAG, "ch%u 闭环运行中, 规划器不接管 (先关闭闭环)", ch);
        return ESP_ERR_INVALID_STATE;
    }

    /* 钳位到该通道的软件行程限位: 否则规划点会跑到实际到不了的角度,
     * 导致 current_deg 与真实位置脱节 (舵机被 servo_set_angle 钳住不动) */
    float lo = 0.0f, hi = 0.0f;
    if (servo_get_limit(ch, &lo, &hi) == ESP_OK) {
        if (target_deg < lo) target_deg = lo;
        if (target_deg > hi) target_deg = hi;
    }

    s_state[ch].target_deg = target_deg;
    s_state[ch].moving     = true;
    return ESP_OK;
}

esp_err_t gimbal_stop(uint8_t ch)
{
    if (ch >= GIMBAL_CH_COUNT) return ESP_ERR_INVALID_ARG;
    s_state[ch].target_deg = s_state[ch].current_deg;
    s_state[ch].vel_dps    = 0.0f;
    s_state[ch].moving     = false;
    return ESP_OK;
}

esp_err_t gimbal_get_state(uint8_t ch, gimbal_state_t *st)
{
    if (ch >= GIMBAL_CH_COUNT) return ESP_ERR_INVALID_ARG;
    if (st == NULL)            return ESP_ERR_INVALID_ARG;
    *st = s_state[ch];
    return ESP_OK;
}

/* ==================== 姿态闭环 (自稳 / 指向) ==================== */

void gimbal_feed_attitude(float pitch_phys_deg, float yaw_phys_deg)
{
    s_meas_pitch = pitch_phys_deg;
    s_meas_yaw   = yaw_phys_deg;
}

esp_err_t gimbal_set_target_phys(uint8_t ch, float phys_deg)
{
    if (ch >= GIMBAL_CH_COUNT) return ESP_ERR_INVALID_ARG;
    if (!s_inited)             return ESP_ERR_INVALID_STATE;

    /* 目标也是标称角下发, 所以先换到标称域钳位, 再换回来存 */
    float cmd = servo_phys_to_cmd_deg(ch, phys_deg);
    float lo = 0.0f, hi = 0.0f;
    if (servo_get_limit(ch, &lo, &hi) == ESP_OK) {
        if (cmd < lo) cmd = lo;
        if (cmd > hi) cmd = hi;
    }
    s_stab[ch].target_phys = servo_cmd_to_phys_deg(ch, cmd);
    return ESP_OK;
}

esp_err_t gimbal_enable_stabilize(uint8_t ch, bool enable)
{
    if (ch >= GIMBAL_CH_COUNT) return ESP_ERR_INVALID_ARG;
    if (!s_inited)             return ESP_ERR_INVALID_STATE;

    if (enable) {
        if (s_stab[ch].enabled) {
            return ESP_OK;
        }
        float cur_cmd = 0.0f;
        if (servo_get_angle(ch, &cur_cmd) != ESP_OK) {
            return ESP_ERR_INVALID_STATE;
        }
        /* 目标锁定在当前姿态, 避免一开启就猛跳 */
        gimbal_stop(ch);
        s_stab_i[ch].integ    = 0.0f;
        s_stab_i[ch].err_prev = 0.0f;
        s_stab[ch].cmd_deg    = cur_cmd;
        s_stab[ch].target_phys = (ch == 1) ? s_meas_pitch : s_meas_yaw;
        s_stab[ch].err         = 0.0f;
        s_stab[ch].meas_phys   = s_stab[ch].target_phys;
        s_stab[ch].enabled     = true;
        ESP_LOGI(TAG, "ch%u 姿态闭环开启: 目标锁定当前姿态 %.1f° (标称 %.1f°)",
                 ch, s_stab[ch].target_phys, cur_cmd);
    } else {
        if (!s_stab[ch].enabled) {
            return ESP_OK;
        }
        s_stab[ch].enabled = false;
        /* 交还给规划器: 把规划点对齐到闭环最后的输出, 避免跳变 */
        s_state[ch].current_deg = s_stab[ch].cmd_deg;
        s_state[ch].target_deg  = s_stab[ch].cmd_deg;
        s_state[ch].vel_dps     = 0.0f;
        s_state[ch].moving      = false;
        ESP_LOGI(TAG, "ch%u 姿态闭环关闭 (停在标称 %.1f°)", ch, s_stab[ch].cmd_deg);
    }
    return ESP_OK;
}

esp_err_t gimbal_get_stab_state(uint8_t ch, gimbal_stab_state_t *st)
{
    if (ch >= GIMBAL_CH_COUNT) return ESP_ERR_INVALID_ARG;
    if (st == NULL)            return ESP_ERR_INVALID_ARG;
    *st = s_stab[ch];
    return ESP_OK;
}

esp_err_t gimbal_set_pid(uint8_t ch, const gimbal_pid_t *pid)
{
    if (ch >= GIMBAL_CH_COUNT) return ESP_ERR_INVALID_ARG;
    if (pid == NULL)           return ESP_ERR_INVALID_ARG;
    s_pid[ch] = *pid;
    return ESP_OK;
}

esp_err_t gimbal_get_pid(uint8_t ch, gimbal_pid_t *pid)
{
    if (ch >= GIMBAL_CH_COUNT) return ESP_ERR_INVALID_ARG;
    if (pid == NULL)           return ESP_ERR_INVALID_ARG;
    *pid = s_pid[ch];
    return ESP_OK;
}

esp_err_t gimbal_set_fb_sign(uint8_t ch, float sign)
{
    if (ch >= GIMBAL_CH_COUNT) return ESP_ERR_INVALID_ARG;
    if (sign >= 0.0f)      s_fb_sign[ch] =  1.0f;
    else                   s_fb_sign[ch] = -1.0f;
    return ESP_OK;
}

esp_err_t gimbal_get_fb_sign(uint8_t ch, float *sign)
{
    if (ch >= GIMBAL_CH_COUNT) return ESP_ERR_INVALID_ARG;
    if (sign == NULL)          return ESP_ERR_INVALID_ARG;
    *sign = s_fb_sign[ch];
    return ESP_OK;
}
