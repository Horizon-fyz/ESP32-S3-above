# [归档] 状态指示灯方案 (status_led, v5.x)

> **归档来源**: `ESP32-S3水上机器人项目参数总览.md` 原 **§11 状态指示灯** 全章。
> **归档原因**: v5.10.0 换板 Waveshare **ESP32-S3-ETH** —— 新板**无板载 LED**,
> `components/status_led` 组件整体移除, `main.c` 的 `status_led_task` / `status_led_init` /
> `notify_rx` / `notify_tx` 调用与 `main/CMakeLists.txt` 依赖一并删除。
> **归档时间**: 2026-09-14
> **状态**: 已废弃 (v3.0 起不再适用), 仅作历史留档。如需状态指示, 请外接 LED 并另行约定引脚。

---

## 1. 状态语义 (历史)

| 状态 | 颜色 | 触发条件 (历史) |
|---|---|---|
| 未连接 / 无 IP | 红色常亮 | 网线未插 / 未获得 IP |
| 已连接 + 数据交换 | 蓝色闪烁 (200ms) | 近 500ms 有 RX/TX 数据 |
| 已连接 + 无数据 | 绿色常亮 | 链路 up, 无通信 |

LED 优先级: 红 > 蓝 > 绿

---

## 2. 驱动方式 (历史)

- **硬件**: 普通 RGB LED, 3 个独立 GPIO (R/G/B)
- **驱动**: ESP-IDF `LEDC` PWM, 定时器 0, 5kHz, 8bit 分辨率
- **接法**: 默认 `common_anode = false` (共阴, GPIO 高电平点亮); 若板子为共阳, 在 `main.c` 改 `led_cfg.common_anode = true`
- **颜色**: 通过 R/G/B 三通道 PWM 占空比混色, 颜色表包含:
  - 红、绿、深蓝
  - 深黄 (255,200,0)、浅黄 (255,255,100)
  - 紫 (255,0,255)
  - 浅蓝 (100,150,255)
  - 白 (255,255,255)

> 注: `s_last_data_time_ms` 由 `s_data_mux` 临界区保护, 防止 TCP 任务与 LED 任务并发修改.

---

## 3. 备注

- 该方案的**演进过程** (v5.4 板载 WS2812/RMT → v5.5 改为 3 路普通 RGB LED + LEDC) 见
  `ESP32-S3水上机器人项目参数总览.md` **§10 变更记录**。
- 方案中出现的引脚 (GPIO48 / GPIO47 / GPIO21) 均为**旧板**取值; 新板 (ESP32-S3-ETH) 中
  GPIO47/48 已改作 L298N 方向脚; 板载 WS2812 RGB LED 的数据脚经实测确认是 **GPIO26**
  (v8.0 曾误判为 GPIO21, **v8.0.1 已更正**; GPIO21 现为空闲脚, v8.0 起 I2C1 SDA 用 GPIO16),
  所以 GPIO26 与 GPIO21 都不可再用于外接状态灯。
