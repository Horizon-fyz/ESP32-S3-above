# ESP32-S3-ETH 引脚定义速查（Waveshare 版）

> 本文档基于开发板丝印图整理，供写程序时直接查阅。板卡为 40 Pin 排针设计，左侧 20 Pin（21–40），右侧 20 Pin（1–20）。
> 板载芯片：ESP32-S3 + W5500（以太网）+ MicroSD 卡座 + PoE 供电模块 + 双 Type-C USB。
>
> ⚠️ **重要更正（2026-09）**：本板为 **ESP32-S3R8（8MB Octal PSRAM）**，按 Espressif 官方说明，**GPIO33–GPIO37 已被 PSRAM 占用（SPIIO4~SPIIO7 + SPIDQS），不可用于其他功能**。丝印图上虽印有这 5 个脚，但**实测不可用**（旁证：官方 IO_Test 示例的"19 个可用输出 GPIO"恰好排除了它们）。本文档已据此更正。

---

## 一、引脚总览（按排针物理序号）

### 右侧排针（从上到下，Pin 1 → Pin 20）

| 排针序号 | 丝印名称 | 复用功能 | 类型 | 是否占用 |
|:---:|:---|:---|:---:|:---:|
| 1 | GPIO20 | USB_D+（D_P） | USB | 已被 USB 占用 |
| 2 | GPIO19 | USB_D-（D_N） | USB | 已被 USB 占用 |
| 3 | GND | 地 | 电源 | — |
| 4 | GPIO48 | 通用 IO | GPIO | 可用 |
| 5 | GPIO47 | 通用 IO | GPIO | 可用 |
| 6 | GPIO46 | 通用 IO（strapping 引脚） | GPIO | 可用，注意启动 strapping |
| 7 | GPIO45 | 通用 IO（VDD_SPI 相关 strapping） | GPIO | 可用，注意启动 strapping |
| 8 | GND | 地 | 电源 | — |
| 9 | GPIO42 | 通用 IO | GPIO | 可用 |
| 10 | GPIO41 | 通用 IO | GPIO | 可用 |
| 11 | GPIO40 | 通用 IO | GPIO | 可用 |
| 12 | GPIO39 | 通用 IO | GPIO | 可用 |
| 13 | GND | 地 | 电源 | — |
| 14 | GPIO38 | 通用 IO | GPIO | 可用 |
| 15 | GPIO37 | — | — | ❌ **不可用：R8 Octal PSRAM 占用** |
| 16 | GPIO36 | — | — | ❌ **不可用：R8 Octal PSRAM 占用** |
| 17 | GPIO35 | — | — | ❌ **不可用：R8 Octal PSRAM 占用** |
| 18 | GND | 地 | 电源 | — |
| 19 | GPIO34 | — | — | ❌ **不可用：R8 Octal PSRAM 占用** |
| 20 | GPIO33 | — | — | ❌ **不可用：R8 Octal PSRAM 占用** |

### 左侧排针（从下到上，Pin 21 → Pin 40）

| 排针序号 | 丝印名称 | 复用功能 | 类型 | 是否占用 |
|:---:|:---|:---|:---:|:---:|
| 21 | GPIO43 | UART0_TX（默认调试串口发送） | GPIO / UART | 可用，但默认用于下载/日志输出 |
| 22 | GPIO44 | UART0_RX（默认调试串口接收） | GPIO / UART | 可用，但默认用于下载/日志输入 |
| 23 | GND | 地 | 电源 | — |
| 24 | GPIO0 | BOOT 按键映射引脚（strapping 引脚） | GPIO / 启动模式 | 可用，但内部接 BOOT 按键 |
| 25 | GPIO1 | 通用 IO | GPIO | 可用 |
| 26 | GPIO2 | 通用 IO | GPIO | 可用 |
| 27 | GPIO3 | 通用 IO | GPIO | 可用 |
| 28 | GND | 地 | 电源 | — |
| 29 | GPIO15 | 通用 IO | GPIO | 可用 |
| 30 | CHIP_UP | 系统控制（通常为芯片使能/上拉控制，勿随意拉低） | 系统控制 | 建议悬空或按原理图使用 |
| 31 | GPIO18 | 通用 IO | GPIO | 可用 |
| 32 | GPIO16 | 通用 IO | GPIO | 可用 |
| 33 | GND | 地 | 电源 | — |
| 34 | GPIO17 | 通用 IO | GPIO | 可用 |
| 35 | GPIO21 | 通用 IO | GPIO | ⬜ **本项目已不再占用** (v8.1): 它曾被当作 I2C1 SDA (v8.0 已改到 GPIO16), 又被当作"候选灯脚"钉低 —— 经确认板载灯在 **GPIO26** (见 §二.5), 故恢复为普通空闲脚 |
| 36 | 3V3 | 3.3 V 电源输出 | 电源 | 为外部外设供电 |
| 37 | 3V3_EN | 3.3 V 使能控制（粉色丝印） | 系统控制 | 控制 3V3 轨是否对外输出 |
| 38 | GND | 地 | 电源 | — |
| 39 | VSYS | 系统主电源输入（通常 5 V，来自 USB 或 PoE） | 电源输入 | 供电输入脚 |
| 40 | VBUS | USB VBUS 5 V（来自 USB 口） | 电源输入 | USB 5 V 母线 |

### 排针相连顺序（顶 → 底，接线时按此核对）

- **右排（Pin 1 → 20）**：`GPIO20 (D_P)` — `GPIO19 (D_N)` — GND — `GPIO48` — `GPIO47` — `GPIO46` — `GPIO45` — GND — `GPIO42` — `GPIO41` — `GPIO40` — `GPIO39` — GND — `GPIO38` — ~~GPIO37~~ — ~~GPIO36~~ — ~~GPIO35~~ — GND — ~~GPIO34~~ — ~~GPIO33~~
- **左排（Pin 40 → 21）**：VBUS — VSYS — GND — 3V3_EN — 3V3 — `GPIO21` — `GPIO17` — GND — `GPIO16` — `GPIO18` — CHIP_UP — `GPIO15` — GND — `GPIO3` — `GPIO2` — `GPIO1` — `GPIO0` — GND — `GPIO44 (UART0_RX)` — `GPIO43 (UART0_TX)`

> 方向注解：**GPIO43 可作 TX**，**GPIO44 可作 RX**（UART0）；**GPIO20 可作 D_P**，**GPIO19 可作 D_N**（USB 相关）。
> ~~删除线~~ 的 5 个脚为 PSRAM 占用，**不可用**。

---

## 二、板载外设已占用的 GPIO（重要：写程序时不要再分配给其他用途）

### 1. MicroSD 卡座（SPI 模式）

| 丝印网络 | 连接到 GPIO | SPI 功能 | 说明 |
|:---|:---:|:---|:---|
| SD_MOSI | GPIO6 | MOSI | SD 卡数据线主出从入 |
| SD_MISO | GPIO5 | MISO | SD 卡数据线主入从出 |
| SD_CLK  | GPIO7 | SCK | SD 卡时钟 |
| SD_CS   | GPIO4 | CS  | SD 卡片选（通常低电平选中） |

> 这 4 个引脚（GPIO4、GPIO5、GPIO6、GPIO7）已固定给 SD 卡使用。如果不用 SD 卡，它们可以释放为普通 GPIO；如果要用 SD 卡，请勿再用于其他 SPI 从设备。

### 2. W5500 以太网芯片（SPI 总线 + 控制信号）

| 丝印网络 | 连接到 GPIO | 功能 | 说明 |
|:---|:---:|:---|:---|
| ETH_MOSI | GPIO11 | MOSI | 与 W5500 通信 |
| ETH_MISO | GPIO12 | MISO | 与 W5500 通信 |
| ETH_CLK  | GPIO13 | SCK  | W5500 SPI 时钟 |
| ETH_CS   | GPIO14 | CS   | W5500 片选 |
| ETH_INT  | GPIO10 | INT  | W5500 中断输出（需配置为输入） |
| ETH_RST  | GPIO9  | RST  | W5500 硬件复位（低电平复位） |

> 这 6 个引脚（GPIO9、GPIO10、GPIO11、GPIO12、GPIO13、GPIO14）已固定给 W5500。以太网功能启用时不能改作他用。

### 3. USB 接口（原生 USB）

| 丝印 | 连接到 GPIO | 说明 |
|:---|:---:|:---|
| D_P | GPIO20 | USB D+ |
| D_N | GPIO19 | USB D- |

> GPIO19 / GPIO20 是 ESP32-S3 原生 USB 串口/JTAG 引脚。如果要使用 USB 虚拟串口，不要把它们当普通 GPIO 用。

### 4. 调试串口 UART0

| 丝印 | 连接到 GPIO | 说明 |
|:---|:---:|:---|
| UART0_TX | GPIO43 | 默认下载与日志输出 TX |
| UART0_RX | GPIO44 | 默认下载与日志输入 RX |

> Arduino / ESP-IDF 默认把 UART0 绑在 GPIO43（TX）、GPIO44（RX）。如果要做自定义串口调试，可以把它们重映射到其他空闲 GPIO，但下载/日志会受影响。

### 5. 板载 WS2812 RGB LED

| 丝印/网络 | 连接到 GPIO | 说明 |
|:---|:---:|:---|
| RGB LED 数据脚 (DIN) | **GPIO26** | **实测/原理图确认（2026-09）**。GPIO26 **未引出到 40 Pin 排针**，属板内网络 |
| ~~（旧记录，已否决）~~ | ~~GPIO21~~ | Waveshare 官方 wiki 的 `RGB_LED` 示例写的是 GPIO21，**与本板实况不符**（可能对应别的板型/版本）；**本项目不再采用**，GPIO21 已恢复为空闲脚 |

> ⚠️ **WS2812 的脾气**：数据脚上任何"非 WS2812 时序"的边沿都会被它当**颜色数据**采样并锁存 —— 所以只要该脚被当 I2C SDA / SPI / 普通翻转用，它就会锁存成随机色（**白色 = 三通道全开最亮**，所以最常见、最刺眼），而且会**一直保持到掉电**。⇒ ① **GPIO26 不要再作 I2C/SPI/UART 等有翻转的信号**（GPIO21 已确认为**空闲脚**，可正常使用）；② 想让它真熄灭，**必须断电重上一次**；③ 要拿它当状态灯必须用 **RMT** 精确时序（位宽容差 ±150 ns，普通 GPIO 打拍不达标）。

---

## 三、电源相关引脚

| 引脚 | 电压/性质 | 说明 |
|:---|:---|:---|
| VBUS（Pin 40） | +5 V | 来自 USB Type-C 的 5 V 母线 |
| VSYS（Pin 39） | 约 +5 V | 系统主电源（USB 或 PoE 输入后汇总） |
| 3V3（Pin 36） | +3.3 V | 板载 LDO 输出，可对外供电（注意电流预算） |
| 3V3_EN（Pin 37） | 控制脚 | 控制 3V3 轨是否使能；按板卡设计使用，不要悬空强驱动 |
| CHIP_UP（Pin 30） | 控制脚 | 芯片使能/上拉相关系统控制脚，建议保持默认状态 |
| GND（Pin 3 / 8 / 13 / 18 / 23 / 28 / 33 / 38） | 0 V | 共 8 个接地引脚 |

**供电方式（板上接口）：**
- USB Type-C（上方，标 USB）：主供电 + 数据下载口。
- PoE Power Port（板中上方，POE 丝印）：PoE 供电输入口。
- 板上有 ACT、LINK 两个指示灯（ACT 表示活动，LINK 表示网口连接）。

---

## 四、按键与启动相关

| 板上按键/信号 | 关联引脚 | 作用 |
|:---|:---|:---|
| BOOT 按钮 | 通常映射到 GPIO0（Pin 24） | 按住 BOOT 再按 RESET，进入下载模式；运行时可作为用户按键 |
| RESET 按钮 | EN / CHIP_PU（板上复位） | 复位整个芯片，无对应排针引出 |

**Strapping 引脚提醒（影响启动模式，上电时不要强拉）：**
- GPIO0：启动模式选择。上电时为低 → 进入下载模式；为高 → 正常启动。
- GPIO3、GPIO45、GPIO46 也属于 ESP32-S3 的 strapping 相关引脚，上电瞬间电平会决定启动行为。运行时再改它们的状态才安全。

---

## 五、空闲可用 GPIO 清单（写程序时直接挑这些）

排除掉已被 SD 卡、W5500、USB、UART0、**PSRAM（GPIO33–37）** 占用的引脚后，**真正空闲、可以自由做输入/输出/PWM/I2C/SPI 从设备等用途**的 GPIO 如下：

```
GPIO1   GPIO2   GPIO3   GPIO15  GPIO16  GPIO17
GPIO18  GPIO21  GPIO38  GPIO39  GPIO40  GPIO41
GPIO42  GPIO45  GPIO46  GPIO47  GPIO48
```

> 共 **17 个**。原清单里的 GPIO33–GPIO37 已移除（R8 Octal PSRAM 占用，**不可用**）。

**接近"半空闲"（建议谨慎使用）：**
- GPIO0：接 BOOT 按键，做普通按键输入时会和 BOOT 功能冲突。
- GPIO43 / GPIO44：默认 UART0（43=TX / 44=RX），重映射串口后可用。
- GPIO19 / GPIO20：USB D_N / D_P，不用 USB 串口时可释放。

---

## 六、常用总线分配建议（避坑版）

如果你的项目要接 I2C、SPI、UART 等外设，可以参考下面的分配，避免和板载外设打架：

| 总线类型 | 推荐使用的空闲 GPIO | 备注 |
|:---|:---|:---|
| I2C（SDA / SCL） | 任选两根空闲 GPIO | ESP32-S3 引脚可任意映射。本项目已用: **I2C1 = GPIO16(SDA) / GPIO17(SCL)** (PCA9685 + 云台 MPU6050; v8.0 起 SDA 从 GPIO21 挪来, 现 GPIO21 已空闲 —— 板载 WS2812 的实际数据脚是 **GPIO26**, 见 §二.5); **GPIO47 / GPIO48 已被 L298N 方向脚占用** |
| SPI（主，外挂从设备） | SCK=GPIO12、MISO=GPIO13、MOSI=GPIO11、CS=任意空闲 GPIO | 注意：这组脚已被 W5500 占用；若共享总线需自行加 CS 隔离 |
| 独立 SPI（不与 W5500 共享） | 任选 4 根空闲 GPIO（如 SCK=GPIO38、MOSI=GPIO39、MISO=GPIO40、CS=GPIO41） | 推荐这种，避免总线冲突；**切勿使用 GPIO33–GPIO37（PSRAM 占用）** |
| 硬件 UART（除 UART0 外） | 任意两根空闲 GPIO | ESP32-S3 支持 UART1/UART2 全引脚映射 |
| PWM / 输入捕获 | 任意空闲 GPIO | LEDC / MCPWM 可映射到任意输出脚 |
| ADC | 注意：GPIO1–GPIO10 属于 ADC1，GPIO11–GPIO20 属于 ADC2（WiFi 启用时 ADC2 受限） | 选空闲脚时优先 ADC1 范围 |

---

## 七、颜色图例（对照原图）

| 颜色 | 含义 |
|:---|:---|
| 红色 | Power（电源） |
| 深灰色 | Ground（地） |
| 绿色 | GPIO（通用 IO，可用） |
| 浅绿色 | GPIO（板载已使用） |
| 灰色 | USB |
| 粉色 | System Control（系统控制） |
| 棕色 | SD Card |
| 蓝色 | ETH（以太网） |

---

## 八、快速使用提醒

1. **不要重复占用**：GPIO4–GPIO7 给 SD 卡，GPIO9–GPIO14 给 W5500，GPIO19–GPIO20 给 USB，GPIO43–GPIO44 给 UART0，**GPIO33–GPIO37 给 PSRAM（不可用）**。写程序初始化时避开这些脚。
2. **3.3 V 逻辑**：ESP32-S3 是 3.3 V 器件，所有 GPIO 禁止直接接 5 V。外接 5 V 传感器需分压或电平转换。
3. **Strapping 脚**：GPIO0、GPIO3、GPIO45、GPIO46 在上电瞬间有特殊含义，上电前不要接下拉/上拉或外部强驱动。
4. **供电电流**：3V3 排针输出电流有限，外接多舵机/大功率模块时请独立供电，不要全靠板上 LDO。
5. **PoE 与 USB 可同时接**：板上有 PoE 供电口，但请确认板卡电源电路不会倒灌到 USB 主机。

---

*文档整理自开发板丝印图，具体电气参数与 strapping 细节请同时参考 ESP32-S3 数据手册和 Waveshare 官方 wiki。*
