# ESP32-S3 水上机器人主控制节点

> **ESP32-S3 AUV Surface Controller**
>
> 角色：核心控制 + 数据处理 + 指令分发  
> 硬件：MPU6050 + 2 ESC + 1 L298N + 2 舵机 + W5500 以太网 + WS2812B 状态灯

---

## ⚠️ 必读：操作前请先阅读参数总览文档

**本文档为快速入口，所有工程参数、协议、引脚定义、构建细节的唯一权威来源为：**

📄 **[ESP32-S3水上机器人项目参数总览.md](./ESP32-S3水上机器人项目参数总览.md)**

> 📌 **规则**：每次与本仓库交互（代码修改、配置调整、调试、烧录、问题排查）之前，必须先阅读上述参数总览文档，确保所有改动与文档一致。

配套远端（水下执行节点）文档：
📄 [`../ESP32-S3-below/ESP32-S3水下机器人项目参数总览.md`](../ESP32-S3-below/ESP32-S3水下机器人项目参数总览.md)

---

## 快速参考

| 项目 | 值 |
|---|---|
| 默认 IP | `192.168.29.10` |
| 默认网关 | `192.168.29.1` |
| TCP Server 端口 | `8080`（上位机） / `8081`（远端节点） |
| 状态灯 | GPIO48，WS2812B，GRB 顺序，RMT 驱动 |
| MPU6050 | I2C0，SDA=GPIO6，SCL=GPIO7 |
| W5500 SPI | SCK=GPIO12，MOSI=GPIO11，MISO=GPIO13，CS=GPIO10，INT=GPIO9 |
| RDK X5 UART | TX=GPIO43，RX=GPIO44，115200 8N1 |

---

## 目录结构

```
ESP32-S3-above/
├── main/                       # 应用入口
│   └── main.c                  # 任务创建、启动流程、TCP 服务器
├── components/
│   ├── wiznet/                 # W5500 以太网驱动 (ioLibrary)
│   ├── status_led/             # WS2812B 状态指示灯
│   ├── imu/                    # MPU6050 驱动
│   ├── servo/                  # PCA9685 舵机驱动
│   ├── motor/                  # ESC + L298N 驱动
│   ├── control/                # 16B 控制/MPU 帧协议解析
│   └── rdk_uart/               # RDK X5 UART 通信
├── sdkconfig.defaults          # 默认 Kconfig (USB-Serial/JTAG console 等)
├── CMakeLists.txt              # 项目构建入口
├── ESP32-S3水上机器人项目参数总览.md   # ⚠️ 权威参数文档
└── README.md                   # 本文件
```

---

## 构建与烧录

### 环境要求

- ESP-IDF v5.5.4
- Python 虚拟环境：`C:\Espressif\tools\python\v5.5.4\venv\Scripts\python.exe`

### PowerShell 构建命令

```powershell
. 'F:\.espressif\v5.5.4\esp-idf\export.ps1'
& 'C:\Espressif\tools\python\v5.5.4\venv\Scripts\python.exe' "$env:IDF_PATH\tools\idf.py" build
```

### 烧录与监控

```powershell
# 将 COMx 替换为实际串口号
& 'C:\Espressif\tools\python\v5.5.4\venv\Scripts\python.exe' "$env:IDF_PATH\tools\idf.py" -p COMx flash monitor
```

> 烧录使用 UART 口（板载 USB 转串口芯片，如 CH340/CP210x）；日志输出使用 USB-Serial/JTAG 口（`sdkconfig.defaults` 已配置 `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`）。部分开发板两个口为同一物理 USB，详见开发板手册。

---

## 状态指示灯含义

| 状态 | 颜色 | 含义 |
|---|---|---|
| 红色常亮 | 未连接 / 无 IP | 检查网线及网络配置 |
| 绿色常亮 | 已连接，无数据交换 | 等待上位机或远端连接 |
| 蓝色闪烁 | 已连接，数据交换中 | 正常通信 |

---

## 关键约定

1. **参数总览文档优先**：任何修改以 `ESP32-S3水上机器人项目参数总览.md` 为准。
2. **代码与文档一致**：若代码与文档冲突，优先按文档更新代码（除非用户另有说明）。
3. **W5500 PHY 模式**：软件强制 100M FULL，SPI 时钟上限 80MHz，当前使用 20MHz。
4. **版本同步**：修改参数总览文档版本号时，同步检查 README.md、远端文档及代码注释。

---

## 配套项目

- **远端执行节点（水下机器人）**：[`../ESP32-S3-below/`](../ESP32-S3-below/)
