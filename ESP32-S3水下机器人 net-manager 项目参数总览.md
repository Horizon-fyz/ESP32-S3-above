# ESP32-S3 水下机器人 net-manager 项目参数总览

> 📌 本文档为唯一项目参数权威来源, 所有代码改动须与本文档一致.
> 文档版本: v3.0 (2026-XX-XX, 新架构)

---

## 1. 系统架构

### 1.1 节点角色

| 节点 | 角色 | 硬件 | 网络角色 |
|---|---|---|---|
| **主控制节点** (本项目) | 核心控制 + 数据处理 + 指令分发 | MPU6050 + 2 ESC + 1 L298N + 2 舵机 | TCP **Server** (双端口) |
| **远端执行节点** (另一 ESP32-S3, 用户负责) | 远端执行 + 本地传感 | MPU6050 + 2 ESC | TCP **Client** |
| **上位机** (笔记本) | 人工控制 + 姿态解算 + 决策 | 无 (软件) | TCP **Client** |

### 1.2 网络拓扑

```
   上位机 (笔记本)                        远端 ESP32-S3
   TCP Client 8080                       TCP Client 8081
        │                                      │
        │ 16B 控制帧 (4 电调+L298N+2 舵机)     │ 8B 子帧 (2 电调)
        │ 16B MPU 帧 (本+远端 MPU, 20Hz)      │ 16B MPU 帧 (远端 MPU, 20Hz)
        ▼                                      ▼
   ┌────────────────────────────────────────────────────┐
   │  主控制节点 (本工程) - TCP Server 双端口            │
   │  socket 0 = 8080 (HOST, 上位机)                    │
   │  socket 1 = 8081 (REMOTE, 远端)                    │
   │  + 本地执行 (ESC1/2 + L298N + 舵机)                │
   │  + 转发 (16B 控制帧 → 8B 子帧)                     │
   │  + 推送 (本 MPU 20Hz 推到主机+远端)                │
   │  + 转发 (远端 MPU 20Hz 转发到主机)                 │
   └────────────────────────────────────────────────────┘
```

### 1.3 数据流向

| 链路 | 方向 | 数据类型 | 速率 |
|---|---|---|---|
| 主↔主机 (8080) | 上→主 | 16B 控制帧 | 按需 (主机主动发) |
| 主→主机 (8080) | 主→上 | 16B MPU 帧 (本+远端合并) | 20Hz |
| 主→主机 (8080) | 主→上 | 500ms JSON 状态 (可选) | 2Hz |
| 主↔远端 (8081) | 主→远 | 8B 子帧 (2 远端电调) | 按需 |
| 主→远端 (8081) | 主→远 | 16B MPU 帧 (本节点 MPU) | 20Hz |
| 远→主 (8081) | 远→主 | 16B MPU 帧 (远端 MPU) | 20Hz |

---

## 2. TCP 协议规范

### 2.1 帧类型总览

| 帧名 | 帧头 | 长度 | 方向 | 用途 |
|---|---|---|---|---|
| 控制帧 | 0xAA 0x55 | 16B | 上位机 → 主 | 完整控制指令 (本地+远端) |
| 转发子帧 | 0xAA 0x55 | 8B | 主 → 远端 | 远端电调 + 系统命令 |
| MPU 数据帧 | 0xBB 0x66 | 16B | 双向 | 6 轴 IMU 原始数据 |

### 2.2 控制帧 (16B) — 上位机 → 主控制节点

| 字节 | 名称 | 类型 | 含义 |
|---|---|---|---|
| 0 | HEAD0 | uint8 | 0xAA (固定) |
| 1 | HEAD1 | uint8 | 0x55 (固定) |
| 2 | cmd | uint8 | 0x10=电机 / 0x20=急停 / 0x30=重启 / 0x40=关机 |
| 3 | local_esc1 | int8 | 本地电调 1 油门 (-100 ~ +100) |
| 4 | local_esc2 | int8 | 本地电调 2 油门 (-100 ~ +100) |
| 5 | remote_esc1 | int8 | 远端电调 1 油门 (-100 ~ +100) |
| 6 | remote_esc2 | int8 | 远端电调 2 油门 (-100 ~ +100) |
| 7 | dc_packed | uint8 | 高 4 位=DC 速度 (0~15), 低 4 位=方向 (0停/1正/2反) |
| 8 | servo0_angle | uint8 | 舵机 0 角度 (0~180) |
| 9 | servo1_angle | uint8 | 舵机 1 角度 (0~180) |
| 10 | flags | uint8 | bit0: 本地执行 / bit1: 转发远端 |
| 11-14 | reserved | uint8 | 保留 |
| 15 | CRC8 | uint8 | 前 15 字节异或 |

### 2.3 转发子帧 (8B) — 主控制节点 → 远端

| 字节 | 名称 | 类型 | 含义 |
|---|---|---|---|
| 0 | HEAD0 | uint8 | 0xAA |
| 1 | HEAD1 | uint8 | 0x55 |
| 2 | cmd | uint8 | 同控制帧 |
| 3 | remote_esc1 | int8 | 远端电调 1 油门 |
| 4 | remote_esc2 | int8 | 远端电调 2 油门 |
| 5-6 | reserved | uint8 | 保留 |
| 7 | CRC8 | uint8 | 前 7 字节异或 |

### 2.4 MPU 数据帧 (16B) — 双向

| 字节 | 名称 | 类型 | 含义 |
|---|---|---|---|
| 0 | HEAD0 | uint8 | 0xBB |
| 1 | HEAD1 | uint8 | 0x66 |
| 2 | type | uint8 | 0x01=本地 MPU / 0x02=远端 MPU |
| 3-4 | ax | int16 LE | 加速度 X (原始寄存器, 16384 LSB/g) |
| 5-6 | ay | int16 LE | 加速度 Y |
| 7-8 | az | int16 LE | 加速度 Z |
| 9-10 | gx | int16 LE | 角速度 X (原始寄存器, 131 LSB/(°/s)) |
| 11-12 | gy | int16 LE | 角速度 Y |
| 13-14 | gz | int16 LE | 角速度 Z |
| 15 | CRC8 | uint8 | 前 15 字节异或 |

### 2.5 flags 字段

| bit | 名称 | 含义 |
|---|---|---|
| 0 | ENABLE_LOCAL | 主控制节点执行本地电机 (ESC1/2 + L298N + 舵机) |
| 1 | FORWARD_REMOTE | 主控制节点转发 8B 子帧到远端 |
| 2-7 | reserved | 保留 |

典型组合:
- `0x01` — 只控制本地, 不转发
- `0x02` — 只转发到远端, 本地不动
- `0x03` — 同时控制本地 + 转发到远端
- `0x00` — 静默, 不执行

### 2.6 cmd 字段

| 值 | 名称 | 行为 |
|---|---|---|
| 0x10 | MOTOR | 电机控制 (按 flags 执行) |
| 0x20 | STOP | 紧急停止所有电机 (本地 + 远端) |
| 0x30 | REBOOT | 系统重启 (主控制节点, 500ms 延迟) |
| 0x40 | SHUTDOWN | 深度睡眠关机 |

> ⚠️ REBOOT / SHUTDOWN 命令作用于**主控制节点本身**, 不转发到远端.

---

## 3. FreeRTOS 任务清单

| 任务 | 文件 | 优先级 | 栈大小 | 周期 | 说明 |
|---|---|---|---|---|---|
| status_led_task | main.c | 5 | 2048 | 150ms | RGB LED 状态指示 |
| tcp_server_task | main.c | 5 | 8192 | 10ms / 100ms | 双 socket 状态机 (退订 WDT) |
| mpu_push_task | main.c | 4 | 2048 | 50ms (20Hz) | 读本地 MPU + 推送 (退订 WDT) |
| status_report_task | main.c | 4 | 2048 | 500ms (可选) | JSON 状态上报 (当前未启动) |

**所有做阻塞式 SPI 通讯的任务必须调用 `esp_task_wdt_delete(NULL)` 退订 Task WDT**, 改在循环中显式 `esp_task_wdt_reset()` 喂狗.

---

## 4. 硬件引脚定义

| 名称 | 引脚 | 文件 | 说明 | 状态 |
|---|---|---|---|---|
| RGB_LED_GPIO | GPIO48 | main.c | 板载 WS2812 RGB LED | ✅ |
| ESC1_PWM_GPIO | GPIO1 | motor.c | 电调 1 PWM (50Hz) | ⚠️ 占用 U0TXD, 需 USB-Serial/JTAG 日志 |
| ESC2_PWM_GPIO | GPIO42 | motor.c | 电调 2 PWM (50Hz) | ✅ (MTMS) |
| ENA_PWM_GPIO | GPIO16 | motor.c | L298N 调速 PWM (5kHz) | ✅ |
| L298N_IN1_GPIO | GPIO17 | motor.c | L298N 方向 1 | ✅ |
| L298N_IN2_GPIO | GPIO18 | motor.c | L298N 方向 2 | ✅ |
| IMU_I2C0_SDA | GPIO6 | imu.c | MPU6050 SDA | ✅ |
| IMU_I2C0_SCL | GPIO7 | imu.c | MPU6050 SCL | ✅ |
| SERVO_I2C1_SDA | GPIO4 | servo.c | PCA9685 SDA | ✅ |
| SERVO_I2C1_SCL | GPIO5 | servo.c | PCA9685 SCL | ✅ |
| SPI2_SCK | GPIO12 | wiznet.c | W5500 SPI 时钟 | ✅ (固定) |
| SPI2_MOSI | GPIO11 | wiznet.c | W5500 MOSI | ✅ |
| SPI2_MISO | GPIO13 | wiznet.c | W5500 MISO | ✅ |
| SPI2_CS | GPIO10 | wiznet.c | W5500 CS | ✅ |
| W5500_INT | GPIO9 | wiznet.c | W5500 中断输出 (下降沿) | ✅ |

> ⚠️ **引脚冲突警告**:
>   - GPIO12 同时是 W5500 SPI SCK 和 ESP32-S3 板载 SPI flash IO2, **不能**用作普通 GPIO
>   - GPIO1 是 U0TXD, 占用需启用 USB-Serial/JTAG 作为日志输出 (已在 sdkconfig 配置)
>   - GPIO48 是板载 RGB LED, 由 status_led 占用

---

## 5. W5500 以太网参数

| 参数 | 值 | 文件 | 说明 |
|---|---|---|---|
| SPI 时钟 | **20 MHz** | wiznet_manager.c | SPI_DMA_DISABLED (polling) 模式, 无堆碎片 |
| DMA 模式 | **DISABLED** | wiznet_manager.c | ESP-IDF v5.5 DMA 模式反复 malloc 会触发 Task WDT |
| INT GPIO | **GPIO9** | wiznet.c | 下降沿触发, ISR 置位 + 1ms 去抖 |
| INT 去抖 | 1ms | wiznet_spi.c | 防止 INT 抖动反复触发 ISR |
| PHY 模式 | 100M FULL | wiznet_manager.c | 软件强制 (可改自动协商) |
| SPI 互斥锁超时 | 100ms | wiznet_spi.c | 防止死锁 |
| 默认 IP | 192.168.29.10 | wiznet_manager.c | 静态 |
| 子网掩码 | 255.255.255.0 | wiznet_manager.c | /24 |
| 默认网关 | 192.168.29.1 | wiznet_manager.c | 路由器 |
| DNS | 8.8.8.8 | wiznet_manager.c | 公共 DNS |
| 等待 link up 超时 | 30s | main.c | 启动时阻塞等待 |
| socket 数量 | 8 | wiznet_manager.c | W5500 8 个 socket |
| socket 缓冲 | 2KB/2KB (TX/RX) | wiznet_manager.c | 共 32KB, W5500 内部 SRAM |

---

## 6. TCP 服务器端口分配

| 端口 | socket | 角色 | 连接方 | 协议 |
|---|---|---|---|---|
| **8080** | 0 | TCP Server | 上位机 (笔记本) | 16B 控制帧 (收) + 16B MPU 帧 (发, 20Hz) |
| **8081** | 1 | TCP Server | 远端 ESP32-S3 | 8B 子帧 (发) + 16B MPU 帧 (双向, 20Hz) |

---

## 7. 组件清单 (components/)

| 组件 | 路径 | 功能 | API 摘要 |
|---|---|---|---|
| wiznet | components/wiznet/ | W5500 驱动 | wiznet_manager_init, is_link_up, get_ip_info |
| wiznet_spi | (同 wiznet) | SPI 桥接 + INT | wiznet_spi_init, wiznet_spi_check_int |
| status_led | components/status_led/ | RGB LED 状态 | status_led_init, status_led_update, status_led_notify_rx |
| imu | components/imu/ | MPU6050 驱动 | imu_init, imu_read, imu_is_ready |
| servo | components/servo/ | PCA9685 驱动 | servo_init, servo_set_angle, servo_set_pulse_us |
| motor | components/motor/ | ESC + L298N | motor_init, motor_set_esc_throttle, motor_set_dc_speed, motor_emergency_stop |
| control | components/control/ | 16B 帧协议 | control_process, control_set_forward_callback, control_build_*_frame |
| tcp_parser | components/tcp_parser/ | (遗留, 暂未用) | - |

---

## 8. 控制协议解析器 (control 组件)

### 8.1 状态机

```
IDLE ──[0xAA|0xBB]──> GOT_HEAD0 ──[0x55|0x66]──> LOADING_CTRL / LOADING_MPU
                          │                              │
                          │ [其他字节]                   │ [累积到 16B]
                          ▼                              ▼
                        IDLE <────── CRC 校验 ─────[执行 + 计数]
```

### 8.2 API

```c
/* 解析器 */
size_t control_process(const uint8_t *data, size_t len);
void   control_reset(void);
void   control_set_forward_callback(ctrl_forward_cb_t cb);

/* 帧构造器 (上位机和远端节点开发用) */
void control_build_ctrl_frame(uint8_t *frame, uint8_t cmd,
    int8_t local_esc1, int8_t local_esc2,
    int8_t remote_esc1, int8_t remote_esc2,
    uint8_t dc_packed, uint8_t servo0, uint8_t servo1, uint8_t flags);

void control_build_mpu_frame(uint8_t *frame, uint8_t type,
    int16_t ax, int16_t ay, int16_t az,
    int16_t gx, int16_t gy, int16_t gz);

/* 统计 */
uint32_t control_get_frame_count(void);
```

---

## 9. 远端节点开发接口 (供用户参考)

远端 ESP32-S3 (TCP Client 8081) 需实现:

1. **连接**: 主动 connect 主节点 192.168.29.10:8081
2. **接收 8B 子帧** (帧头 0xAA 0x55):
   - 解析 cmd + 2 个远端电调油门
   - 直接控制远端 2 个电调
3. **接收 16B MPU 帧** (帧头 0xBB 0x66, type=0x01):
   - 主节点发来的本节点 MPU 数据 (备用, 不必处理)
4. **20Hz 发送 16B MPU 帧** (帧头 0xBB 0x66, type=0x02):
   - 远端 MPU 原始数据
   - 上位机做姿态解算 (ESP32 不融合)

### 远端伪代码示例

```c
// 远端 TCP Client 循环
while (1) {
    // 1. 接收并解析 8B 子帧 → 控制电调
    n = recv(sock, buf, 8, 0);
    if (n == 8 && buf[0] == 0xAA && buf[1] == 0x55) {
        if (crc_check(buf, 8)) {
            int8_t esc1 = (int8_t)buf[3];
            int8_t esc2 = (int8_t)buf[4];
            motor_set_throttle(0, esc1);
            motor_set_throttle(1, esc2);
        }
    }
    
    // 2. 20Hz 读 MPU 并发送
    if (mpu_read(&m) == OK) {
        build_mpu_frame(frame, 0x02, 
                        m.ax*16384, m.ay*16384, m.az*16384,
                        m.gx*131, m.gy*131, m.gz*131);
        send(sock, frame, 16, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
```

---

## 10. 关键工程决策 (历史)

| 时间 | 决策 | 原因 |
|---|---|---|
| 2026-01 | W5500 ioLibrary (非 LwIP) | 硬件 TOE, 性能更优 |
| 2026-01 | SPI 8 MHz → 20 MHz | 配合 polling 模式, 仍可稳定工作 |
| 2026-01 | SPI DMA → POLLING | 解决 ESP-IDF v5.5 DMA 反复 malloc 触发 Task WDT |
| 2026-01 | log 默认 NONE | 减少启动刷屏 |
| 2026-01 | Task WDT 退订长 I/O 任务 | 阻塞式 SPI 不适合 5s 默认超时 |
| 2026-01 | INT GPIO9 + 1ms 去抖 | 防止 INT 抖动反复触发 ISR |
| 2026-01 | SPI 互斥锁 100ms 超时 | 防止一个任务卡死 SPI 导致所有任务死锁 |
| 2026-01 | 8B → 16B 控制帧 | 涵盖 4 电调 (本地2+远端2) + L298N + 2 舵机 |
| 2026-01 | 双 socket 8080+8081 | 主机和远端独立连接, 互不干扰 |
| 2026-01 | 16B MPU 帧 (6 轴 int16 LE) | 紧凑 (320 字节/秒@20Hz), 上位机解算 |
| 2026-01 | USB-Serial/JTAG 日志 | 释放 GPIO1 给 ESC1 PWM |

---

## 11. 状态指示灯

| 状态 | 颜色 | 触发条件 |
|---|---|---|
| 未连接 / 无 IP | 红色常亮 | 网线未插 / 30s 未获得 IP |
| 已连接 + 数据交换 | 蓝色闪烁 (200ms) | 近 500ms 有 RX/TX 数据 |
| 已连接 + 无数据 | 绿色常亮 | 链路 up, 无通信 |

LED 优先级: 红 > 蓝 > 绿

---

## 12. 构建与烧录

```bash
# 构建
source $HOME/.espressif/v5.5.4/esp-idf/export.sh
idf.py build

# 烧录 (USB-Serial/JTAG 口, 不是 UART)
idf.py -p /dev/ttyACM0 flash monitor

# 监控
picocom -b 115200 /dev/ttyACM0
```

预期启动输出 (没有客户端时):
```
=== AUV 控制器启动 (W5500 TOE 模式) ===
W5500 初始化中 (TOE 模式)...
W5500 INT on GPIO9 (falling edge)
W5500 ready, MAC=... IP=192.168.29.10
=== 网线已连接 ===
本机 IP: 192.168.29.10
子网掩码: 255.255.255.0
默认网关: 192.168.29.1
=== TCP 服务器已启动: 8080(HOST) + 8081(REMOTE) ===
```

---

> 📝 文档变更需同步更新所有相关代码并测试.
>   最后修改: v3.0 (新架构: 主控+远端+上位机, 双 socket, 16B 帧)
