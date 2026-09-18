/**
 * @file nav.h
 * @brief 惯导模块驱动 (亚博 GPS + 10 轴 IMU 一体模块, **单条 UART**)
 *
 * ================= 协议 (维特 WIT 标准 0x55, 主动上报) =================
 *
 *   帧 (定长 11 字节):
 *     0x55 | TYPE | D1L D1H | D2L D2H | D3L D3H | D4L D4H | SUM
 *     SUM = (0x55 + TYPE + 8 个数据字节) 取低 8 位
 *
 *   解析的帧类型:
 *     0x50 时间      YY MM DD HH MN SS MSL MSH
 *     0x51 加速度     Ax Ay Az (int16, /32768*16 g) + 温度 (/100 ℃)
 *     0x52 角速度     Wx Wy Wz (int16, /32768*2000 °/s) + 电压
 *     0x53 角度       Roll Pitch Yaw (int16, /32768*180 °) + 版本
 *     0x57 经纬度     Lon(uint32) Lat(uint32)  —— 去掉小数点的 NMEA ddmm.mmmmm
 *     0x58 GPS       海拔(int16, /10 m) 航向(int16, /100 °) 地速(uint32, /1000 km/h)
 *     0x5A GPS 精度  卫星数(uint16) PDOP/HDOP/VDOP (int16, /100)
 *     0x5F 读寄存器返回 (配置流程内部使用)
 *
 * ================= 接线 (v7.0: 已改到 GPIO1/2) =================
 *
 *   ⚠️ 硬件其实是**两块板** (亚博 GPS 板 + 亚博 10 轴 IMU 板), 官方"**融合**"后由
 *      **一条 UART** 输出 —— 上位机侧当作**一个**模块: 同一帧流里既有姿态也有 GPS。
 *
 *   ESP32 RX = GPIO1 <- 模块 TX      (v7.0 从 GPIO16 改来; 见下方原因)
 *   ESP32 TX = GPIO2 -> 模块 RX      (v7.0 从 GPIO18 改来; 只用于发配置命令)
 *   UART2 @ **9600** 8N1   (模块出厂默认波特率就是 9600, 不主动改)
 *   VCC = 3.3V / 5V 都支持, GND 必须共地
 *
 *   ⚠️ **为什么改到 1/2** (v7.0): 接在 16/18 时实测 `rx_bytes == 0` (7 档波特率全试过,
 *      物理层零字节), 而 GPIO1/2 正是原来旧 GPS (NMEA/UART1) 用的两个脚 ——
 *      在旧 GPS 上**实测跑通过 UART**, 且 **1/2 都不是 strapping 脚** (GPIO3 原 PPS 是
 *      strapping 脚, 现空闲)。要回退只需改 `nav_get_default_config()` 里那两个数。
 *   ⚠️ 初始化时会**自动扫描波特率** (9600 → 115200 → … → 230400, 官方 STM32 例程也这么做):
 *      模块若被上位机软件改过波特率, 固定 9600 会一直收不到数据。
 *   ⚠️ 模块 TX 必须是 3.3V 电平; 5V 直连会打坏 S3。
 *   ⚠️ 该模块**取代**了原来的两颗器件: 旧 GPS (NMEA/UART1 TX=2/RX=1/PPS=3 —— 其中
 *      1/2 现在归本模块) 与旧亚博 10 轴 IMU (`7E 23` 请求式/UART2) 在"水上"工程里都已删除。
 *
 * ================= 输出配置 (启动时按需写入) =================
 *
 *   RSW   (寄存器 0x02) = **0x058F** = TIME|ACC|GYRO|ANGLE|GPS|VELOCITY|GSA
 *   RRATE (寄存器 0x03) = **0x05**   = 5Hz
 *
 *   ⚠️ 出厂默认 RSW = 0x001E (**不含 GPS 帧**), 所以必须配一次才拿得到经纬度;
 *      默认 RRATE = 0x06 (10Hz), 7 帧 × 11B × 10Hz ≈ 770B/s 已到 9600 的 80%,
 *      偏紧 —— 故降到 5Hz (≈385B/s, 40%)。
 *   ⚠️ **只在"读回的当前值与期望不符"时才写 + SAVE** —— SAVE 会写模块 Flash,
 *      每次上电都写会伤寿命。模块没接时配置失败, 等收到第一帧后会自动重试。
 *
 *   写寄存器流程 (模块 10s 内无指令会自动上锁):
 *     解锁 FF AA 69 88 B5  →  写 FF AA <reg> <lo> <hi>  →  保存 FF AA 00 00 00
 *   读寄存器:
 *     FF AA 27 <reg> 00    →  返回 0x55 0x5F + 连续 4 个寄存器的值
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 惯导模块解算出的姿态/原始惯性数据 (模块内部已融合, 姿态含磁力计补偿) */
typedef struct {
    float    ax, ay, az;    ///< 加速度 (g)
    float    gx, gy, gz;    ///< 角速度 (°/s)
    float    roll;          ///< 横滚角 (°)
    float    pitch;         ///< 俯仰角 (°)
    float    yaw;           ///< 航向角 (°, 9 轴算法时为绝对航向)
    float    temperature;   ///< 温度 (℃)
    uint64_t timestamp_us;  ///< 最近一次姿态数据到达时刻 (esp_timer us)
    bool     valid;         ///< 是否已收到过数据
} nav_imu_t;

/** 惯导模块内的 GPS 数据快照 */
typedef struct {
    /* ⚠️ 模块不上报"定位有效性"标志, 这里以"经纬度非 0"判定;
     *    定位失效时坐标清零 (不保留上一次的值, 避免被误读成实时坐标)。 */
    bool     valid;         ///< 已有有效经纬度
    double   latitude;      ///< 纬度 (°, 北纬为正)
    double   longitude;     ///< 经度 (°, 东经为正)

    float    altitude_m;    ///< GPS 海拔 (m)
    float    speed_kmh;     ///< 对地速度 (km/h)
    float    course_deg;    ///< GPS 航向 (°)

    uint16_t satellites;    ///< 卫星数 (0x5A)
    float    pdop;          ///< 位置精度因子 (0x5A)
    float    hdop;          ///< 水平精度因子
    float    vdop;          ///< 垂直精度因子

    bool     time_valid;    ///< 已收到时间帧 (0x50)
    uint8_t  year;          ///< 年 (模块只给 2 位: 0~99)
    uint8_t  month;         ///< 月 (1~12)
    uint8_t  day;           ///< 日
    uint8_t  hour;          ///< 时
    uint8_t  minute;        ///< 分
    uint8_t  second;        ///< 秒

    uint64_t last_fix_us;   ///< 最近一次收到经纬度帧的时刻 (0 = 从未收到)
} nav_gps_t;

/** 运行统计 (诊断用) */
typedef struct {
    bool     ready;         ///< 收到过至少一帧 (模块在线)
    bool     cfg_ok;        ///< RSW/RRATE 已是期望值
    uint32_t baud;          ///< 实际使用的波特率 (扫描命中值; 未命中时为 9600)
    uint16_t rsw;           ///< 模块当前 RSW (读回值)
    uint16_t rrate;         ///< 模块当前 RRATE 档位 (读回值)

    uint32_t rx_bytes;      ///< 累计收到的原始字节数 (0 = 一个字节都没有 → 查接线/供电)
    uint32_t frames_ok;     ///< 校验通过的帧数
    uint32_t frames_bad;    ///< 校验失败/重同步丢弃的次数
    uint32_t cnt_time;      ///< 0x50 帧计数
    uint32_t cnt_acc;       ///< 0x51
    uint32_t cnt_gyro;      ///< 0x52
    uint32_t cnt_angle;     ///< 0x53
    uint32_t cnt_gps;       ///< 0x57
    uint32_t cnt_vel;       ///< 0x58
    uint32_t cnt_dop;       ///< 0x5A
} nav_stats_t;

/** 惯导模块配置 */
typedef struct {
    int      uart_num;      ///< UART 号, 默认 UART_NUM_2
    int      tx_gpio;       ///< ESP32 TX -> 模块 RX, 默认 18
    int      rx_gpio;       ///< ESP32 RX <- 模块 TX, 默认 16
    uint32_t baud;          ///< 波特率, 默认 9600 (模块出厂默认)
    uint16_t rsw;           ///< 期望输出内容位图, 默认 0x058F
    uint8_t  rrate;         ///< 期望输出速率档位, 默认 0x05 (5Hz)
} nav_config_t;

/** 获取默认配置 (UART2, TX=18, RX=16, 9600, RSW=0x058F, RRATE=5Hz) */
nav_config_t nav_get_default_config(void);

/**
 * @brief 初始化: 装 UART → 读回并(必要时)写入输出配置 → 启动接收任务
 *
 * @return ESP_OK 已收到模块数据; ESP_ERR_NOT_FOUND 模块无应答
 *         (两种情况接收任务都已启动: 模块后接上会自动补配置并出数据, 不必重启)
 */
esp_err_t nav_init(const nav_config_t *cfg);

/** 模块是否在线 (收到过至少一帧) */
bool nav_is_ready(void);

/** 读取最新姿态快照 (线程安全); 尚无数据时返回 ESP_ERR_INVALID_STATE */
esp_err_t nav_read_imu(nav_imu_t *out);

/** 读取最新 GPS 快照 (线程安全); 尚无数据时返回 ESP_ERR_INVALID_STATE */
esp_err_t nav_read_gps(nav_gps_t *out);

/** 读取运行统计 (线程安全) */
esp_err_t nav_get_stats(nav_stats_t *out);

#ifdef __cplusplus
}
#endif
