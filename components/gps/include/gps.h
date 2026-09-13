/**
 * @file gps.h
 * @brief GPS (NMEA 0183) 驱动 —— UART1 + PPS 秒脉冲
 *
 * 引脚分配 (v5.10 换板 ESP32-S3-ETH: 左排相邻 2/1/3, 走同一接插件):
 *   ESP32 TX (-> GPS RX) = GPIO2
 *   ESP32 RX (<- GPS TX) = GPIO1
 *   PPS 输入             = GPIO3
 *
 * 说明:
 *   - 亚博 10 轴 IMU 已确定走 I2C (I2C1), 不占用 UART, 故 GPS 独占 UART1。
 *   - GPIO3 是 strapping 脚 (JTAG 源选择), 这里只作 PPS **输入**; 上电瞬间
 *     不要对它外部强拉, 若 GPS 模块上电期间会把该脚拉低, 应改用 pps_gpio = -1
 *     或换到其它空闲脚。
 *   - GPS 模块 TX 输出务必确认是 **3.3V 电平**, 5V 直连会打坏 S3。
 *
 * 解析范围:
 *   $xxRMC -> 定位有效标志 / UTC 时间日期 / 经纬度 / 地速 / **对地真航向**
 *   $xxGGA -> 定位质量 / 卫星数 / HDOP / 海拔
 * 航向 (course) 是将来 ch0 平面旋转闭环做绝对指向的关键参考 (弥补无磁力计)。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** GPS 数据快照 */
typedef struct {
    bool     valid;         ///< 定位有效 (RMC status == 'A')
    bool     time_valid;    ///< UTC 时间已解析到

    uint8_t  hour;          ///< UTC 时 (0~23)
    uint8_t  minute;        ///< UTC 分
    uint8_t  second;        ///< UTC 秒
    uint8_t  day;           ///< UTC 日
    uint8_t  month;         ///< UTC 月
    uint16_t year;          ///< UTC 年

    double   latitude;      ///< 纬度 (°, 北纬为正)
    double   longitude;     ///< 经度 (°, 东经为正)
    float    speed_kmh;     ///< 对地速度 (km/h)
    float    course_deg;    ///< **对地真航向** (°, 0~360, 正北为 0, 顺时针)
    float    altitude_m;    ///< 海拔 (m)
    uint8_t  satellites;    ///< 使用中的卫星数
    float    hdop;          ///< 水平精度因子

    uint64_t last_rmc_us;   ///< 最近一次收到 RMC 的时刻 (esp_timer us)
    uint32_t sentence_cnt;  ///< 累计解析成功的语句数
    uint32_t err_cnt;       ///< 累计校验和错误/格式错误语句数
} gps_data_t;

/** GPS 配置 */
typedef struct {
    int      uart_num;      ///< UART 号, 默认 UART_NUM_1
    int      tx_gpio;       ///< ESP32 TX -> GPS RX, 默认 2
    int      rx_gpio;       ///< ESP32 RX <- GPS TX, 默认 1
    int      pps_gpio;      ///< PPS 输入, 默认 3 (strapping 脚, 仅作输入); -1 表示不使用
    uint32_t baud;          ///< 波特率, 默认 9600
} gps_config_t;

/** 获取默认配置 (UART1, TX=2, RX=1, PPS=3, 9600) */
gps_config_t gps_get_default_config(void);

/** 初始化 UART1 + PPS 中断, 并启动解析任务 */
esp_err_t gps_init(const gps_config_t *cfg);

/** 是否已初始化 */
bool gps_is_ready(void);

/** 取一份数据快照 (线程安全) */
esp_err_t gps_get_data(gps_data_t *out);

/** PPS 累计脉冲数 (每秒 +1, 可用来判断 GPS 是否已锁定) */
uint32_t gps_get_pps_count(void);

/** 最近一次 PPS 上升沿的时刻 (esp_timer us), 0 表示还没收到过 */
uint64_t gps_get_pps_last_us(void);

/** 向 GPS 写原始数据 (用于发配置语句; 需要自己拼好 $...*hh\r\n) */
esp_err_t gps_write(const char *data, size_t len);

#ifdef __cplusplus
}
#endif
