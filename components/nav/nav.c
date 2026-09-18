/**
 * @file nav.c
 * @brief 惯导模块 (亚博 GPS + 10 轴 IMU 一体) 驱动实现 —— WIT 0x55 协议, 单条 UART
 *
 * 结构:
 *   nav_init()  → 装 UART2 (16/18 @9600) → nav_apply_config() (按需配 RSW/RRATE)
 *               → 起 nav_task (阻塞读 UART → nav_feed 逐字节成帧 → 分发解析)
 *   数据按"最新快照"保存, 上层随时读; 模块是主动上报的, 不需要请求帧。
 *
 * 详见 nav.h 的协议/接线/配置说明。
 */

#include "nav.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "nav";

/* ===== UART / 任务 ===== */
#define NAV_UART_RX_BUF    2048
#define NAV_TASK_STACK     3072
#define NAV_TASK_PRIO      4
#define NAV_READ_TIMEOUT_MS 50      /* 任务阻塞读超时 (到点做配置重试等杂事) */

/* ===== WIT 协议 ===== */
#define WIT_HEAD            0x55
#define WIT_TYPE_TIME       0x50
#define WIT_TYPE_ACC        0x51
#define WIT_TYPE_GYRO       0x52
#define WIT_TYPE_ANGLE      0x53
#define WIT_TYPE_LONLAT     0x57
#define WIT_TYPE_GPS        0x58
#define WIT_TYPE_DOP        0x5A
#define WIT_TYPE_READ       0x5F
#define WIT_FRAME_LEN       11      /* 0x55 + TYPE + 8 数据 + SUM */

#define WIT_REG_SAVE        0x00    /* 0x0000=保存 0x00FF=重启 0x0001=恢复出厂 */
#define WIT_REG_RSW         0x02    /* 输出内容位图 */
#define WIT_REG_RRATE       0x03    /* 输出速率档位 */
#define WIT_REG_READADDR    0x27    /* 读寄存器: FF AA 27 <reg> 00 */
#define WIT_REG_KEY         0x69    /* 解锁 */
#define WIT_KEY_UNLOCK      0xB588  /* 解锁值 (其他值无效) */

/* 读寄存器应答等待, 配置重试间隔, 波特率扫描单档等待 */
#define NAV_CFG_RESP_MS     300
#define NAV_CFG_RETRY_MS    5000
#define NAV_SCAN_BAUD_MS    120

/* 期望输出: TIME(0x50)|ACC(0x51)|GYRO(0x52)|ANGLE(0x53)|GPS(0x57)|VELOCITY(0x58)|GSA(0x5A) */
#define NAV_RSW_DEFAULT     0x058F
#define NAV_RRATE_DEFAULT   0x05    /* 0x05 = 5Hz (0x06 = 10Hz) */

/* ===== 状态 ===== */
static nav_config_t     s_cfg;
static bool             s_inited    = false;

static portMUX_TYPE     s_mux       = portMUX_INITIALIZER_UNLOCKED;
static nav_imu_t        s_imu;                  /* 受 s_mux 保护 */
static nav_gps_t        s_gps;                  /* 受 s_mux 保护 */
static nav_stats_t      s_stats;                /* 受 s_mux 保护 */

static volatile bool    s_cfg_ok    = false;
static volatile int64_t s_cfg_try_us = 0;       /* 上次配置尝试时刻 */
static bool             s_baud_found = false;   /* 已扫到模块波特率 (只需扫一次) */
static uint16_t         s_rsw_now   = 0;        /* 最近一次读回的 RSW */
static uint16_t         s_rrate_now = 0;        /* 最近一次读回的 RRATE 档位 */

/* 0x5F 应答 (配置流程内部使用; 同一时刻只有一个未完成的读请求)
 * ⚠️ 维特的 0x5F 应答**不带地址字段**, 只带"从被读地址起的 4 个连续寄存器" ⇒ 成块保存 */
static volatile bool     s_read_got = false;
static volatile uint16_t s_read_w[4] = {0};

/* ===== 小工具 ===== */
static inline int16_t rd_s16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline int32_t rd_s32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

/**
 * 模块给的经纬度是"去掉小数点的 NMEA ddmm.mmmmm" (int32; 南纬/西经为负),
 * 换算: 度 = v / 10000000 ; 分 = (v % 10000000) / 100000 → 十进制度 = 度 + 分/60
 */
static double lonlat_to_deg(int32_t v)
{
    double sign = (v < 0) ? -1.0 : 1.0;
    int64_t a   = (v < 0) ? -(int64_t)v : (int64_t)v;
    double  dd  = (double)(a / 10000000);
    double  mm  = (double)(a % 10000000) / 100000.0;
    return sign * (dd + mm / 60.0);
}

/* ===== 帧分发 ===== */
static void nav_handle_frame(uint8_t type, const uint8_t *d)
{
    int64_t now = esp_timer_get_time();

    switch (type) {
    case WIT_TYPE_TIME: {
        portENTER_CRITICAL(&s_mux);
        s_gps.year   = d[0];
        s_gps.month  = d[1];
        s_gps.day    = d[2];
        s_gps.hour   = d[3];
        s_gps.minute = d[4];
        s_gps.second = d[5];
        s_gps.time_valid = true;
        s_stats.cnt_time++;
        portEXIT_CRITICAL(&s_mux);
        break;
    }
    case WIT_TYPE_ACC: {
        float ax = rd_s16(d)     / 32768.0f * 16.0f;
        float ay = rd_s16(d + 2) / 32768.0f * 16.0f;
        float az = rd_s16(d + 4) / 32768.0f * 16.0f;
        float tp = rd_s16(d + 6) / 100.0f;
        portENTER_CRITICAL(&s_mux);
        s_imu.ax = ax; s_imu.ay = ay; s_imu.az = az;
        s_imu.temperature = tp;
        s_imu.timestamp_us = (uint64_t)now;
        s_imu.valid = true;
        s_stats.cnt_acc++;
        portEXIT_CRITICAL(&s_mux);
        break;
    }
    case WIT_TYPE_GYRO: {
        float gx = rd_s16(d)     / 32768.0f * 2000.0f;
        float gy = rd_s16(d + 2) / 32768.0f * 2000.0f;
        float gz = rd_s16(d + 4) / 32768.0f * 2000.0f;
        portENTER_CRITICAL(&s_mux);
        s_imu.gx = gx; s_imu.gy = gy; s_imu.gz = gz;
        s_imu.timestamp_us = (uint64_t)now;
        s_imu.valid = true;
        s_stats.cnt_gyro++;
        portEXIT_CRITICAL(&s_mux);
        break;
    }
    case WIT_TYPE_ANGLE: {
        float roll  = rd_s16(d)     / 32768.0f * 180.0f;
        float pitch = rd_s16(d + 2) / 32768.0f * 180.0f;
        float yaw   = rd_s16(d + 4) / 32768.0f * 180.0f;
        portENTER_CRITICAL(&s_mux);
        s_imu.roll = roll; s_imu.pitch = pitch; s_imu.yaw = yaw;
        s_imu.timestamp_us = (uint64_t)now;
        s_imu.valid = true;
        s_stats.cnt_angle++;
        portEXIT_CRITICAL(&s_mux);
        break;
    }
    case WIT_TYPE_LONLAT: {
        int32_t lon = rd_s32(d);
        int32_t lat = rd_s32(d + 4);
        bool    ok  = (lon != 0 || lat != 0);   /* 模块不上报定位标志: 全 0 视为未定位 */
        portENTER_CRITICAL(&s_mux);
        s_gps.valid     = ok;
        s_gps.longitude = ok ? lonlat_to_deg(lon) : 0.0;
        s_gps.latitude  = ok ? lonlat_to_deg(lat) : 0.0;
        if (ok) s_gps.last_fix_us = (uint64_t)now;
        s_stats.cnt_gps++;
        portEXIT_CRITICAL(&s_mux);
        break;
    }
    case WIT_TYPE_GPS: {
        float alt = rd_s16(d)     / 10.0f;
        float crs = rd_s16(d + 2) / 100.0f;
        float spd = (float)(uint32_t)((uint32_t)d[4] | ((uint32_t)d[5] << 8) |
                                      ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24)) / 1000.0f;
        portENTER_CRITICAL(&s_mux);
        s_gps.altitude_m = alt;
        s_gps.course_deg = crs;
        s_gps.speed_kmh  = spd;
        s_stats.cnt_vel++;
        portEXIT_CRITICAL(&s_mux);
        break;
    }
    case WIT_TYPE_DOP: {
        portENTER_CRITICAL(&s_mux);
        s_gps.satellites = rd_u16(d);
        s_gps.pdop = rd_s16(d + 2) / 100.0f;
        s_gps.hdop = rd_s16(d + 4) / 100.0f;
        s_gps.vdop = rd_s16(d + 6) / 100.0f;
        s_stats.cnt_dop++;
        portEXIT_CRITICAL(&s_mux);
        break;
    }
    case WIT_TYPE_READ:
        /* 读寄存器应答: 4 个字 = 被读地址及其后 3 个寄存器 (配置流程在等它) */
        for (int i = 0; i < 4; i++) s_read_w[i] = rd_u16(d + i * 2);
        s_read_got = true;
        break;
    default:
        break;  /* 未解析的帧类型直接忽略 (计数靠 frames_ok) */
    }
}

/* ===== 帧同步 (逐字节喂入; 校验失败就丢掉这个 0x55 重新找同步) ===== */
static void nav_feed(const uint8_t *data, int n)
{
    static uint8_t fb[WIT_FRAME_LEN + 1];
    static int     fb_len = 0;

    for (int i = 0; i < n; i++) {
        if (fb_len >= (int)sizeof(fb)) {            /* 理论到不了, 兜底防溢出 */
            memmove(fb, fb + 1, sizeof(fb) - 1);
            fb_len = (int)sizeof(fb) - 1;
        }
        fb[fb_len++] = data[i];

        /* 1) 丢掉帧头之前的杂字节 */
        while (fb_len > 0 && fb[0] != WIT_HEAD) {
            fb_len--;
            memmove(fb, fb + 1, (size_t)fb_len);
        }
        if (fb_len < WIT_FRAME_LEN) {
            continue;
        }

        /* 2) 校验: SUM = 前 10 字节累加的低 8 位 */
        uint8_t sum = 0;
        for (int k = 0; k < WIT_FRAME_LEN - 1; k++) {
            sum += fb[k];
        }
        if (sum == fb[WIT_FRAME_LEN - 1]) {
            portENTER_CRITICAL(&s_mux);
            s_stats.frames_ok++;
            portEXIT_CRITICAL(&s_mux);
            nav_handle_frame(fb[1], &fb[2]);
            fb_len -= WIT_FRAME_LEN;
            memmove(fb, fb + WIT_FRAME_LEN, (size_t)fb_len);
        } else {
            portENTER_CRITICAL(&s_mux);
            s_stats.frames_bad++;
            portEXIT_CRITICAL(&s_mux);
            fb_len--;
            memmove(fb, fb + 1, (size_t)fb_len);
        }
    }

    portENTER_CRITICAL(&s_mux);
    s_stats.rx_bytes += (uint32_t)n;
    portEXIT_CRITICAL(&s_mux);
}

/* ===== 模块寄存器读写 ===== */
/* 写格式 (维特 SDK WitWriteReg 一致): FF AA <reg> <data低> <data高> */
static void wit_send_cmd(uint8_t reg, uint16_t val)
{
    uint8_t f[5] = { 0xFF, 0xAA, reg, (uint8_t)(val & 0xFF), (uint8_t)(val >> 8) };
    uart_write_bytes((uart_port_t)s_cfg.uart_num, (const char *)f, sizeof(f));
}

/**
 * 波特率自扫描 —— 官方 STM32 例程 (AutoScanSensor) 也这么做: 模块波特率可能被
 * 上位机软件或上一次配置改过, 固定 9600 会一直收不到数据。
 * 顺序: 先试出厂默认 9600 (命中就秒过), 再试常见档位。
 * 命中判据: 换完波特率后发一条"读 RSW"指令, 能在超时内收到合法帧/应答。
 */
static bool nav_scan_baud(void)
{
    static const uint32_t baud_try[] = { 9600, 115200, 57600, 38400, 19200, 4800, 230400 };

    for (size_t i = 0; i < sizeof(baud_try) / sizeof(baud_try[0]); i++) {
        uart_set_baudrate((uart_port_t)s_cfg.uart_num, baud_try[i]);
        uart_flush_input((uart_port_t)s_cfg.uart_num);

        uint32_t frames_before = s_stats.frames_ok;
        s_read_got = false;
        wit_send_cmd(WIT_REG_READADDR, WIT_REG_RSW);

        int64_t t0 = esp_timer_get_time();
        while ((esp_timer_get_time() - t0) / 1000 < NAV_SCAN_BAUD_MS) {
            uint8_t buf[64];
            int n = uart_read_bytes((uart_port_t)s_cfg.uart_num, buf, sizeof(buf),
                                    pdMS_TO_TICKS(10));
            if (n > 0) {
                nav_feed(buf, n);
            }
            if (s_read_got || s_stats.frames_ok != frames_before) {
                s_cfg.baud = baud_try[i];
                ESP_LOGI(TAG, "波特率扫描命中: %u bps", (unsigned)baud_try[i]);
                return true;
            }
        }
    }

    /* 全部失败: 回到默认值等下次重试 */
    uart_set_baudrate((uart_port_t)s_cfg.uart_num, 9600);
    s_cfg.baud = 9600;
    return false;
}

/** 读寄存器: 一条指令取回 0x5F 应答里的 **4 个连续寄存器** (reg, reg+1, reg+2, reg+3)。
 *
 * ⚠️ 维特的 0x5F 应答**不带地址字段**, 只带"从被读地址起的 4 个值" ⇒ **必须成块读**:
 *    一次读 0x02(RSW) 即可同时拿到 RSW 与 RRATE(0x03)。**不要连发两条读指令** ——
 *    前一条的**迟到应答**会被后一条误认成自己的 (v8.0.2 实测: RRATE 读回成了 RSW 的
 *    0x058F ⇒ 误判"配置不符" ⇒ 每次上电都白写一次模块 Flash)。 */
static bool nav_read_regs(uint8_t reg, uint16_t out[4])
{
    s_read_got = false;
    wit_send_cmd(WIT_REG_READADDR, reg);

    int64_t t0 = esp_timer_get_time();
    while ((esp_timer_get_time() - t0) / 1000 < NAV_CFG_RESP_MS) {
        uint8_t buf[64];
        int n = uart_read_bytes((uart_port_t)s_cfg.uart_num, buf, sizeof(buf),
                                pdMS_TO_TICKS(10));
        if (n > 0) {
            nav_feed(buf, n);       /* 顺手喂解析器, 不丢上报帧 */
        }
        if (s_read_got) {
            for (int i = 0; i < 4; i++) out[i] = s_read_w[i];
            return true;
        }
    }
    return false;
}

/**
 * 读回输出配置; **只有与期望不符时**才解锁→写→保存 (SAVE 会写模块 Flash)。
 * 可在 nav_init() 里调一次, 之后由 nav_task 在"模块后接上/上次失败"时重试。
 */
static void nav_apply_config(void)
{
    /* 第一次先扫波特率 (模块可能被改成过别的档位); 扫不到直接退出, 等下次重试 */
    if (!s_baud_found) {
        if (!nav_scan_baud()) {
            s_cfg_ok = false;
            ESP_LOGW(TAG, "模块无应答 (UART%d RX=IO%d <- 模块 TX, TX=IO%d -> 模块 RX, 已扫 9600~230400)",
                     s_cfg.uart_num, s_cfg.rx_gpio, s_cfg.tx_gpio);
            ESP_LOGW(TAG, "查: 模块 VCC=3.3V / GND 共地 / TX-RX 是否交叉 / 模块是否上电 / 波特率");
            return;
        }
        s_baud_found = true;
    }

    uint16_t blk[4] = {0};
    /* 一条指令读回 0x02~0x05: blk[0]=RSW, blk[1]=RRATE (见 nav_read_regs 注释) */
    if (!nav_read_regs(WIT_REG_RSW, blk)) {
        s_cfg_ok = false;
        ESP_LOGW(TAG, "读配置无应答 (波特率已按 %u bps), 稍后自动重试", (unsigned)s_cfg.baud);
        return;
    }
    uint16_t rsw = blk[0], rrate = blk[1];
    s_rsw_now   = rsw;
    s_rrate_now = rrate;

    if (rsw == s_cfg.rsw && rrate == s_cfg.rrate) {
        s_cfg_ok = true;
        ESP_LOGI(TAG, "输出配置已是期望值 (RSW=0x%04X RRATE=0x%02X), 不写模块 Flash",
                 (unsigned)rsw, (unsigned)rrate);
        return;
    }

    ESP_LOGI(TAG, "配置输出: RSW 0x%04X→0x%04X, RRATE 0x%02X→0x%02X (解锁→写→保存)",
             (unsigned)rsw, (unsigned)s_cfg.rsw,
             (unsigned)rrate, (unsigned)s_cfg.rrate);

    wit_send_cmd(WIT_REG_KEY, WIT_KEY_UNLOCK);          /* 解锁 (10s 内有效) */
    vTaskDelay(pdMS_TO_TICKS(10));
    if (rsw  != s_cfg.rsw)   wit_send_cmd(WIT_REG_RSW,   s_cfg.rsw);
    if (rrate != s_cfg.rrate) wit_send_cmd(WIT_REG_RRATE, s_cfg.rrate);
    vTaskDelay(pdMS_TO_TICKS(10));
    wit_send_cmd(WIT_REG_SAVE, 0x0000);                 /* 保存 */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 回读验证: 同样**一条指令**取回 0x02~0x05, 避免再出现"迟到应答串台" */
    bool ok       = nav_read_regs(WIT_REG_RSW, blk);
    bool ok_rsw   = ok && (blk[0] == s_cfg.rsw);
    bool ok_rrate = ok && (blk[1] == s_cfg.rrate);
    if (ok) {
        s_rsw_now   = blk[0];
        s_rrate_now = blk[1];
    }

    s_cfg_ok = ok_rsw && ok_rrate;
    if (s_cfg_ok) {
        ESP_LOGI(TAG, "输出配置写入成功并已保存");
    } else {
        ESP_LOGW(TAG, "配置回读不一致 (RSW %s / RRATE %s), 稍后自动重试",
                 ok_rsw ? "OK" : "NG", ok_rrate ? "OK" : "NG");
    }
}

/* ===== 接收任务 ===== */
static void nav_task(void *pvParameters)
{
    (void)pvParameters;
    uint8_t buf[128];

    while (1) {
        int n = uart_read_bytes((uart_port_t)s_cfg.uart_num, buf, sizeof(buf),
                               pdMS_TO_TICKS(NAV_READ_TIMEOUT_MS));
        if (n > 0) {
            nav_feed(buf, n);
        }

        /* 模块后接上 / 上次配置失败 → 自动补配置 (含首次无应答的情况) */
        if (!s_cfg_ok) {
            int64_t now = esp_timer_get_time();
            if (now - s_cfg_try_us > (int64_t)NAV_CFG_RETRY_MS * 1000) {
                s_cfg_try_us = now;
                nav_apply_config();
            }
        }
    }
}

/* ===== 对外 API ===== */
nav_config_t nav_get_default_config(void)
{
    nav_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.uart_num = UART_NUM_2;
    cfg.tx_gpio  = 2;
    cfg.rx_gpio  = 1;
    cfg.baud     = 9600;
    cfg.rsw      = NAV_RSW_DEFAULT;
    cfg.rrate    = NAV_RRATE_DEFAULT;
    return cfg;
}

esp_err_t nav_init(const nav_config_t *cfg)
{
    if (s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    s_cfg = cfg ? *cfg : nav_get_default_config();

    esp_err_t ret = uart_driver_install((uart_port_t)s_cfg.uart_num, NAV_UART_RX_BUF, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART%d 驱动安装失败: %s", s_cfg.uart_num, esp_err_to_name(ret));
        return ret;
    }
    uart_config_t uc = {
        .baud_rate  = (int)s_cfg.baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config((uart_port_t)s_cfg.uart_num, &uc);
    uart_set_pin((uart_port_t)s_cfg.uart_num, s_cfg.tx_gpio, s_cfg.rx_gpio,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "惯导模块 UART%d 已就绪 (RX=IO%d <- 模块 TX, TX=IO%d -> 模块 RX, %u 8N1)",
             s_cfg.uart_num, s_cfg.rx_gpio, s_cfg.tx_gpio, (unsigned)s_cfg.baud);

    s_inited     = true;
    s_cfg_try_us = esp_timer_get_time();
    nav_apply_config();     /* 读回/按需写入输出配置 */

    xTaskCreate(nav_task, "nav_task", NAV_TASK_STACK, NULL, NAV_TASK_PRIO, NULL);

    if (!s_cfg_ok && s_stats.rx_bytes == 0) {
        return ESP_ERR_NOT_FOUND;   /* 模块无应答 (接收任务已在跑, 后接上会自动恢复) */
    }
    return ESP_OK;
}

bool nav_is_ready(void)
{
    portENTER_CRITICAL(&s_mux);
    bool ok = s_stats.frames_ok > 0;
    portEXIT_CRITICAL(&s_mux);
    return ok;
}

esp_err_t nav_read_imu(nav_imu_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_mux);
    bool valid = s_imu.valid;
    *out = s_imu;
    portEXIT_CRITICAL(&s_mux);
    return valid ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t nav_read_gps(nav_gps_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_mux);
    bool got = (s_gps.last_fix_us != 0) || s_gps.time_valid;
    *out = s_gps;
    portEXIT_CRITICAL(&s_mux);
    return got ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t nav_get_stats(nav_stats_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_mux);
    *out = s_stats;
    out->ready  = (s_stats.frames_ok > 0);
    out->cfg_ok = s_cfg_ok;
    out->baud   = s_cfg.baud;
    out->rsw    = s_rsw_now;
    out->rrate  = s_rrate_now;
    portEXIT_CRITICAL(&s_mux);
    return ESP_OK;
}
