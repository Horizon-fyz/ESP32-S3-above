# ESP32-S3 水上机器人主控制节点

> **ESP32-S3 AUV Surface Controller**
>
> 角色：核心控制 + 数据处理 + 指令分发  
> 硬件：**Waveshare ESP32-S3-ETH**（板载 W5500）+ 惯导模块（亚博 GPS+10 轴 IMU 一体）+ 4 ESC + 1 L298N + 2 舵机(PCA9685) + 云台 MPU6050

---

## ⚠️ 必读：操作前请先阅读参数总览文档

**本文档为快速入口，所有工程参数、协议、引脚定义、构建细节的唯一权威来源为：**

📄 **[ESP32-S3水上机器人项目参数总览.md](./ESP32-S3水上机器人项目参数总览.md)**

> 📌 **规则**：每次与本仓库交互（代码修改、配置调整、调试、烧录、问题排查）之前，必须先阅读上述参数总览文档，确保所有改动与文档一致。

配套远端（水下执行节点）文档：
📄 [`../ESP32-S3-below/ESP32-S3水下机器人项目参数总览.md`](../ESP32-S3-below/ESP32-S3水下机器人项目参数总览.md)

---

## 当前固件形态

> ⚠️ 当前固件为 **[当前模式]**：实跑 **W5500 以太网 + TCP Server 8080（上位机内网操控）+ 水面电调 + L298N 滚筒 + 惯导模块（亚博 GPS+10 轴 IMU）+ 云台（PCA9685 舵机 + 上位机拖动条）+ 串口控制台**。`W5500_ENABLE` / `MOTOR_ENABLE` / `NAV_ENABLE` / `SERVO_ENABLE` **为 `1`**，`TEST_MODE = 0`（云台纯开环）；**两个临时关停**：**反推**（`REV_ESC_ENABLE = 0`）与**云台 MPU6050**（`MPU6050_ENABLE = 0`，不初始化/不自检）。

- **已启用**：**W5500 以太网 + TCP Server 8080**（静态 IP `192.168.29.10/24`；收上位机（`tools/tcp_console.py`）20Hz 的 16B 控制帧 → **差速混合驱动水面电调**，`bucket_speed` → L298N 滚筒；**500ms 失联保护**，控制台 **`net`** 查状态）+ **水面推进电调**（ESC1/ESC2 = **IO41 左 / IO42 右**；SkyWalker V2 单向，50Hz，1100µs=停 / 1141~1940µs=油门；⚠️ **反推暂时关闭** `REV_ESC_ENABLE=0` ⇒ 负油门按 0（停）处理，方向线 REV1/REV2 = IO39/IO40 只保持在 1100µs 正向半区）+ **L298N 滚筒收放**（ENA=IO38 5kHz PWM / IN1=IO48 / IN2=IO47）+ **惯导模块**（亚博 GPS+10 轴 IMU 一体，UART2 RX=IO1 / TX=IO2 @9600，维特 WIT `0x55` 主动上报；控制台 `nav`/`gps`/`hull`）+ **云台**（PCA9685 ch0/ch1 = I2C1 IO16/17；**手动摆位两条路**：串口 `p`/`g`/`n`/`z` 与 **上位机「云台手动」两个拖动条**（`cmd=0x12`）+ **状态回传**（云台 MPU6050 原始 6 轴 `type=0x01` **20Hz** —— ⚠️ `MPU6050_ENABLE=0` 时无数据源、自动停发；**GPS `type=0x04` 定位 / `type=0x05` 运动+精度+时间 各 2Hz**）+ 串口标定控制台。
- **暂时关停**（调试开关，`main.c` 顶部）：**① 云台 MPU6050 + 姿态解算**（`MPU6050_ENABLE 0` —— 不初始化、不地址自检、不读有效性；`attitude_task` 不创建 ⇒ 「主控 MPU」面板恒 `--`，云台闭环 `ge`/`gt`/`gp` 无反馈源）；**② 反推**（`REV_ESC_ENABLE 0` —— 负油门按 0（停）处理，不能倒退；恢复需把该宏置 `1` 并把电调参数"刹车类型"设为**反推刹车**）。各组件代码与接口全部保留，置回 `1` 即恢复。
- **暂未启用**：**8081 远端转发**（水下节点未就绪；远端帧一旦进来会被 `tcp_server_forward_mpu_to_host()` **原样透传**给上位机）、`status_report_task`。设计保留在参数总览 §2/§9。
- 详见参数总览 **§0 / §2 / §3 / §5 / §6 / §7.9 / §7.10**。

---

## 快速参考

| 项目 | 值 |
|---|---|
| 板卡 | Waveshare **ESP32-S3-ETH**（ESP32-S3R8, 8MB Octal PSRAM, 16MB Flash） |
| Flash / 分区 | **16MB**（W25Q128FS）；分区表 `SINGLE_APP`（app 1MB，当前占用 ~333KB） |
| 固件形态 | **当前模式**（**W5500 + TCP Server 8080 已启用**；4 个开关为 `1`；**临时关停: 反推** `REV_ESC_ENABLE=0` + **云台 MPU** `MPU6050_ENABLE=0`；8081 转发未启用；**状态回传已启用**，见下） |
| 默认 IP | `192.168.29.10` （静态；link up 后由串口打印） |
| 默认网关 | `192.168.29.1` （静态；同上） |
| TCP Server 端口 | `8080`（上位机，**已启用**，`tools/tcp_console.py` 连接） / `8081`（远端节点，**未启用**） |
| 失联保护 | **500ms**：收到过控制帧后超时无新帧 ⇒ 水面电调 + 滚筒全部归零（`tcp_check_link_timeout()`） |
| 状态回传（主→PC） | 云台 MPU6050 原始 6 轴 `type=0x01` **20Hz**（v8.1 —— ⚠️ `MPU6050_ENABLE=0` 时**无数据源、自动停发**，面板恒 `--`）+ **GPS `type=0x04` 定位 / `type=0x05` 运动+精度+时间 各 2Hz**（v9.1，数据来自惯导模块，帧布局见参数总览 §2.4）；模块无数据时不发帧 |
| 板载 W5500 SPI | SCK=GPIO13，MOSI=GPIO11，MISO=GPIO12，CS=GPIO14，RST=GPIO9，INT=GPIO10 （**`W5500_ENABLE=1` 已启用**；20MHz polling，100M FULL） |
| 云台 MPU6050 | I2C1，SDA=GPIO16，SCL=GPIO17（与 PCA9685 **共线**，0x68）；**⏸ 当前暂时关停**（`MPU6050_ENABLE 0`：不初始化/不自检/不读有效性，`attitude_task` 不创建） |
| 云台舵机手动 | **两条路**：串口控制台 `p`/`g`/`n`/`z`，或上位机 `tools/tcp_console.py` 的「云台手动」两个拖动条（**cmd=0x12**，拖动即发；ch0 0~360° / ch1 30~150°，受固件限位约束） |
| 惯导模块 UART2 | **UART2**，ESP32 RX=**GPIO1** ← 模块 TX，ESP32 TX=**GPIO2** → 模块 RX，**9600**（v7.0 从 16/18 改来）；亚博 **GPS + 10 轴 IMU 一体**模块，**维特 WIT `0x55` 协议主动上报**（驱动 `components/nav`，初始化自动扫描波特率 9600~230400），取代了旧 GPS（NMEA/UART1 2/1/3）与旧亚博 10 轴 IMU（`7E 23`）；**已启用**（`NAV_ENABLE 1`） |
| PCA9685 | I2C1，SDA=GPIO16，SCL=GPIO17（0x40）；**已启用**（`SERVO_ENABLE 1`） |
| 云台舵机（PCA9685） | **已启用**；ch0=360°（平面旋转），ch1=180°（俯仰）；实测标定见参数总览 §7.6 |
| 推进/反推 ESC | 推进 GPIO41(左) / GPIO42(右)；反推方向线 GPIO39(左) / GPIO40(右)（**v9.0 左右对调: 42 = 右**）；50Hz；⚠️ **反推暂时关闭**（`REV_ESC_ENABLE=0`，负油门按 0/停处理，方向线只保持 1100µs 正向半区） |
| L298N | IN1=GPIO48，IN2=GPIO47，ENA=GPIO38（**已启用，滚筒收放**；控制台 `m <-100~100>` 或 TCP `bucket_speed`；`\|值\|<60` 按 60） |
| 串口控制台 | **板载原生 USB（USB-Serial/JTAG）** 115200，插**原生 USB** 那个 Type-C 口即可，提示符 `gimbal>`。**裸数字 `0~100` = 左右同时推进**（⚠️ 反推当前关闭，负值按停处理）；命令见参数总览 §7.9（**`l`/`r` 单侧油门, `m` 滚筒, `net` 网络状态**, `cal`/`cal2`/`pw` 电调标定, `gps`/`hull` 惯导快照, **`nav` 惯导模块状态**） |
| 状态灯 | ❌ 已删除（新板无板载 LED） |
| ⚠️ 不可用引脚 | **GPIO33–37**（Octal PSRAM）；GPIO4–7（MicroSD）；GPIO19/20（USB）；GPIO43/44（UART0 console）；strapping：GPIO0/3/45/46 |
| 引脚依据 | [ESP32-S3-ETH-pinout.md](./ESP32-S3-ETH-pinout.md) |
| 视觉/AI 节点 | ROCK 5C（推流 + AI 推理），AI 推理后视频流经内网直接访问，与主控无直接链路 |

---

## 目录结构

```
ESP32-S3-above/
├── main/                       # 应用入口
│   └── main.c                  # 任务创建、启动流程、当前模式逻辑 (含串口控制台)
├── components/
│   ├── wiznet/                 # 板载 W5500 以太网驱动 (ioLibrary) —— 已启用 (W5500_ENABLE=1, TCP Server 8080)
│   ├── imu/                    # 云台 MPU6050 (I2C1, 与 PCA9685 共线) —— v6.0 起只剩这一颗; ⏸ 当前关停 (MPU6050_ENABLE=0)
│   ├── nav/                    # 惯导模块 (亚博 GPS+10 轴 IMU 一体, UART2 RX=IO1 / TX=IO2 @9600, 维特 WIT 0x55 主动上报) —— v6.0 新增, 已启用 (NAV_ENABLE=1), GPS 2Hz 上报上位机
│   ├── servo/                  # PCA9685 舵机驱动 (I2C1 IO16/17) + 标定体系 —— 已启用 (SERVO_ENABLE=1)
│   ├── gimbal/                 # 云台运动规划 + 姿态闭环 PID 自稳 —— TEST_MODE=0 时不开闭环 (仅开环)
│   ├── motor/                  # 电调 (推进油门线 41/42 + 反推方向线 39/40, 反推已关闭) + L298N 滚筒 (38/48/47, 已启用)
│   └── control/                # 16B 控制/状态帧协议解析 (v3.0 沉浮) —— 已启用 (只解析, 回调交 main.c 执行)
├── docs/legacy/                # 历史归档 (换板/换方案前的内容留档)
├── sdkconfig.defaults          # 默认 Kconfig (console 走原生 USB-Serial/JTAG 等)
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

> 烧录与日志走**板载原生 USB（USB-Serial/JTAG）**——插板子**原生 USB** 那个 Type-C 口，端口在设备管理器里是 `VID_303A` 的「USB 串行设备」，115200（`sdkconfig.defaults` 已配 `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` + `CONFIG_ESP_CONSOLE_SECONDARY_NONE=y`；UART0 已不再输出日志）。
> ⚠️ 板载 USB-UART 桥（CH340）口在同样能插，但**当前 console 不在那条通道上**，插它看不到日志也敲不进命令。
> ⚠️ 改这两个 console 选项后必须**删掉 `sdkconfig` 重新 build**，否则 `sdkconfig.defaults` 不生效。

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
3. **当前固件形态**：本工程固件当前为 **[当前模式]**——**W5500 + TCP Server 8080 已启用**（收上位机 16B 控制帧 → 差速混合驱动水面电调 + L298N 滚筒，带 500ms 失联保护）；**惯导模块（亚博 GPS+10 轴 IMU 一体）/ PCA9685 云台舵机 / 云台 MPU6050 / W5500 / 电机 全部启用**（5 个总开关全 `1`，`TEST_MODE = 0` 云台开环）；**唯一临时关停的是反推**（`REV_ESC_ENABLE 0`）；**8081 远端转发未启用**（主控状态回传 `type=0x01` MPU 20Hz + `type=0x04/0x05` GPS 2Hz 已启用）。参数总览 §0 及各章节已如实标注；恢复相应功能时须同步撤掉这些标注。
4. **差速混合位置**：v5.12 起 `control` 组件**只解析协议**（`control_set_local_callback()`），差速混合与电机驱动在 `main.c` 的 `tcp_apply_local_command()`（原因见参数总览 §2.7）。
5. **W5500 PHY 模式**：软件强制 100M FULL，SPI 时钟上限 80MHz，当前使用 20MHz。
6. **版本同步**：修改参数总览文档版本号时，同步检查 README.md、远端文档及代码注释。
7. **版本号规则（v6.0 起）**：🔴 **破坏性变更**（删组件 / 删对外命令或 API、改引脚或接线、改协议字段或语义、整机形态大范围停用启用）⇒ **第一位 +1**（6.0 → 7.0）；🟡 非破坏的新增/增强 ⇒ 第二位 +1（6.0 → 6.1）；🟢 修复/文档/内部重构 ⇒ 第三位 +1（6.0 → 6.0.1）。⚠️ **历史 v5.x 不重排**（已被远端文档与代码注释引用），逐条回溯判定表见参数总览 **§10 开头**。

---

## 配套项目

- **远端执行节点（水下机器人）**：[`../ESP32-S3-below/`](../ESP32-S3-below/)（当前 v3.0）
