# ESP32-S3 水上机器人主控制节点

> **ESP32-S3 AUV Surface Controller**
>
> 角色：核心控制 + 数据处理 + 指令分发  
> 硬件：**Waveshare ESP32-S3-ETH**（板载 W5500）+ MPU6050 + 亚博10轴IMU + 4 ESC + 1 L298N + 2 舵机(PCA9685) + GPS

---

## ⚠️ 必读：操作前请先阅读参数总览文档

**本文档为快速入口，所有工程参数、协议、引脚定义、构建细节的唯一权威来源为：**

📄 **[ESP32-S3水上机器人项目参数总览.md](./ESP32-S3水上机器人项目参数总览.md)**

> 📌 **规则**：每次与本仓库交互（代码修改、配置调整、调试、烧录、问题排查）之前，必须先阅读上述参数总览文档，确保所有改动与文档一致。

配套远端（水下执行节点）文档：
📄 [`../ESP32-S3-below/ESP32-S3水下机器人项目参数总览.md`](../ESP32-S3-below/ESP32-S3水下机器人项目参数总览.md)

---

## 当前固件形态（v5.11.1）

> ⚠️ 当前固件为 **[临时测试/标定模式]**（`main.c` 注明"仅保留 PCA9685 + MPU6050, 其他全部注释"）。

- **已启用**：云台 MPU6050(I2C0) + PCA9685(I2C1) + 船体 10 轴 IMU(I2C1, 0x50) + GPS(UART1) + 云台运动规划/姿态闭环 + Madgwick 姿态解算 + 串口标定控制台。
- **暂未启用**（代码保留但初始化/调用已注释）：W5500 以太网、TCP Server(8080/8081)、motor 电机、control 协议。
- 详见参数总览 **§0 / §3 / §7.6~§7.9**。

---

## 快速参考

| 项目 | 值 |
|---|---|
| 板卡 | Waveshare **ESP32-S3-ETH**（ESP32-S3R8, 8MB Octal PSRAM, 16MB Flash） |
| Flash / 分区 | **16MB**（W25Q128FS）；分区表 `SINGLE_APP`（app 1MB，当前占用 ~333KB） |
| 固件形态 | **临时测试/标定模式**（网络/电机/control 暂未启用，见上） |
| 默认 IP | `192.168.29.10` （网络恢复后启用） |
| 默认网关 | `192.168.29.1` （网络恢复后启用） |
| TCP Server 端口 | `8080`（上位机） / `8081`（远端节点） （网络恢复后启用） |
| 板载 W5500 SPI | SCK=GPIO13，MOSI=GPIO11，MISO=GPIO12，CS=GPIO14，RST=GPIO9，INT=GPIO10 （网络恢复后启用） |
| 云台 MPU6050 | I2C0，SDA=GPIO16，SCL=GPIO18 |
| PCA9685 + 船体10轴IMU | I2C1，SDA=GPIO21，SCL=GPIO17 |
| 云台舵机（PCA9685） | ch0=360°（平面旋转），ch1=180°（俯仰）；实测标定见参数总览 §7.6 |
| 推进/反推 ESC ×4 | GPIO42 / GPIO41 / GPIO40 / GPIO39 （当前未启用） |
| L298N | IN1=GPIO48，IN2=GPIO47，ENA=GPIO38 （当前未启用） |
| GPS（UART1） | TX=GPIO2，RX=GPIO1，PPS=GPIO3 |
| 串口控制台 | UART0 115200，提示符 `gimbal>`（裸数字=改 ch0 脉宽），命令见参数总览 §7.9 |
| 状态灯 | ❌ 已删除（新板无板载 LED） |
| ⚠️ 不可用引脚 | **GPIO33–37**（Octal PSRAM）；GPIO4–7（MicroSD）；GPIO19/20（USB）；GPIO43/44（UART0 console）；strapping：GPIO0/3/45/46 |
| 引脚依据 | [ESP32-S3-ETH-pinout.md](./ESP32-S3-ETH-pinout.md) |
| 视觉/AI 节点 | ROCK 5C（推流 + AI 推理），AI 推理后视频流经内网直接访问，与主控无直接链路 |

---

## 目录结构

```
ESP32-S3-above/
├── main/                       # 应用入口
│   └── main.c                  # 任务创建、启动流程、临时测试/标定逻辑 (含串口控制台)
├── components/
│   ├── wiznet/                 # 板载 W5500 以太网驱动 (ioLibrary) —— 当前未启用
│   ├── imu/                    # MPU6050 (I2C0) + 亚博10轴IMU (I2C1) 双角色驱动
│   ├── servo/                  # PCA9685 舵机驱动 (I2C1) + 标定体系 (双角度刻度/NVS)
│   ├── gimbal/                 # 云台运动规划 + 姿态闭环 PID 自稳
│   ├── gps/                    # GPS (NMEA0183, UART1)
│   ├── motor/                  # 4 ESC + L298N 驱动 —— 当前未启用
│   └── control/                # 16B 控制/状态帧协议解析 (v3.0 沉浮) —— 当前未启用
├── sdkconfig.defaults          # 默认 Kconfig (UART0 console 等)
├── CMakeLists.txt              # 项目构建入口
├── idf.ps1                     # 便携式 ESP-IDF 启动脚本 (自动探测, 兼容多台电脑)
├── ESP32-S3-ETH-pinout.md      # 新板排针与板载外设占用引脚速查
├── ESP32-S3水上机器人项目参数总览.md   # ⚠️ 权威参数文档
└── README.md                   # 本文件
```

---

## 构建与烧录

### 环境要求

- ESP-IDF **v5.5.4**（本工程以此版本编译）

### 构建 / 烧录 / 监视（推荐）

项目根目录的 `idf.ps1` 会**自动探测本机的 ESP-IDF 安装位置**，加载其环境后转发给 `idf.py`，
脚本内不含任何写死的绝对路径，**多台电脑可共用**：

```powershell
.\idf.ps1 build                    # 编译
.\idf.ps1 -p COM3 flash monitor    # 烧录并监视
.\idf.ps1 -p COM3 monitor          # 只监视
```

探测顺序（取第一个有效项）：

1. 环境变量 `$env:IDF_PATH`
2. 下列根目录下的 `esp-idf*`、`frameworks\esp-idf*`、`*\esp-idf*`、`*\*\esp-idf*`：
   `C:\Espressif`、`D:\Espressif`、`E:\Espressif`、`C:\esp`、`D:\esp`、`%USERPROFILE%\esp`

多版本共存时优先 **v5.5**。若自动探测失败，手动指定后重试：

```powershell
$env:IDF_PATH = 'D:\your\esp-idf'
.\idf.ps1 build
```

> ⚠️ `idf.ps1` 含中文，必须保存为 **UTF-8 with BOM**，否则 PowerShell 5.1 会按 GBK 误读导致乱码。

### 手动方式（与上面等价）

```powershell
. $env:IDF_PATH\export.ps1
idf.py build
```

> 烧录与日志均使用 UART0（GPIO43/44，板载 USB 转串口芯片，如 CH340/CP210x），波特率 115200（`sdkconfig.defaults` 已配置 `CONFIG_ESP_CONSOLE_UART_DEFAULT=y`）。

### Flash 尺寸 / 分区 / 构建注意

- 板载 Flash **16MB**（W25Q128FS）；`sdkconfig.defaults` 已配 `CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y`。
- ⚠️ **`sdkconfig.defaults` 只对不含在 `sdkconfig` 里的项生效**：若 `sdkconfig` 是旧生成（含 `..._2MB`），改 defaults **不会覆盖**，必须删 `sdkconfig`/`sdkconfig.old` 后重新 build，16MB 才生效（`build/flasher_args.json` 的 `--flash_size` 随之变为 `16MB`）。
- 分区表 `SINGLE_APP`（默认 `partitions_singleapp.csv`，app 固定 1MB）；当前 app ≈ `0x51890`（~333KB），剩余约 68%。
- ⚠️ `build/` 与工程绝对路径绑定：工程目录换盘符/路径后需删除 `build/` 重编，否则报 `Build directory ... configured for project ... not ...`。
- 若 `.\idf.ps1` 报 `No module named 'click'`，说明 PATH 上的 `python` 不是 IDF venv；用 `<IDF_TOOLS_PATH>\tools\python\v5.5.4\venv\Scripts\python.exe` 运行 `idf.py`，或把该 `Scripts` 目录置于 PATH 最前。
- 电机 LEDC 通道分配（4 ESC 用 Timer0/Channel0~3，L298N 用 Timer1/Channel4）见参数总览 **§4.1**。

---

## 状态指示灯含义

❌ **已删除**（v5.10.0）：新板 ESP32-S3-ETH 无板载 LED，`components/status_led` 组件与相关调用已全部移除。
如需状态指示，请外接 LED 并另行约定引脚（当前引脚表已排满，且 GPIO33–37 / 4–7 / 19–20 / 43–44 不可用）。

---

## 关键约定

1. **参数总览文档优先**：任何修改以 `ESP32-S3水上机器人项目参数总览.md` 为准。
2. **代码与文档一致**：若代码与文档冲突，优先按文档更新代码（除非用户另有说明）。
3. **当前固件形态**：本工程固件当前为 **临时测试/标定模式**（网络/电机/control 暂未启用），参数总览 §0 及各"设计参考（当前未启用）"章节已如实标注；恢复网络功能时须同步撤掉这些标注。
4. **W5500 PHY 模式**（网络恢复后）：软件强制 100M FULL，SPI 时钟上限 80MHz，当前使用 20MHz。
5. **版本同步**：修改参数总览文档版本号时，同步检查 README.md、远端文档及代码注释。

---

## 配套项目

- **远端执行节点（水下机器人）**：[`../ESP32-S3-below/`](../ESP32-S3-below/)（当前 v3.0）
