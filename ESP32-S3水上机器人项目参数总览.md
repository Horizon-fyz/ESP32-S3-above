# ESP32-S3 水上机器人项目参数总览

> 📌 本文档为唯一项目参数权威来源, 所有代码改动须与本文档一致.
> 文档版本: **v9.2** (① **云台新增"上位机手动"这条路 —— 上位机两个拖动条 (cmd=0x12 SERVO)** —— 之前舵机只能从串口控制台 (`p`/`g`/`n`/`z`) 摆位; 现在协议新增 **cmd=0x12 云台手动帧** (16B: `[3-4]` ch0 标称角 ×0.1° / `[5-6]` ch1 标称角 ×0.1° / `[7]` 通道掩码 bit0=ch0·bit1=ch1), 主控对置位通道调 `servo_set_angle()`(**与控制台 `g` 等价**: 受标定+软件限位约束, 只在角度变化时才写 I2C), 客户端 `tools/tcp_console.py` 增加「**云台手动**」面板两个拖动条 (ch0 0~360° / ch1 30~150°, **拖动即发** `send_now`, 需勾选「启用下发」)。**只有掩码置位的通道会动** ⇒ 老上位机不发这个 cmd / 发 `mask=0` 都不会误动舵机, **帧长仍是 16B, 故属非破坏新增 (🟡)**。顺手修客户端一个真 bug: 拖动条(UI 线程)与 20Hz 发送(循环线程)并发写同一 socket 会**交错撕裂 16B 帧**, 已给发送加互斥锁。② 文档顺带订正 §1 两处旧实况描述 (舵机已随 PCA9685 启用、MPU/GPS 回传已启用); ③ 临时开关变化: **云台 MPU6050 暂时废弃** (`MPU6050_ENABLE=0`, 只进 §0/§7.5/§11, 不记入本表)。**已完成**。上一版 v9.1: ① **新增 GPS 状态上报** —— 上位机「GPS」面板** —— 主控把惯导模块 (亚博 GPS+IMU) 的 GPS 快照打包成两个 16B 状态帧, **各 2Hz** 发给 8080 上的上位机: `type=0x04` 定位 (纬度/经度/海拔/卫星数) + `type=0x05` 运动+精度+时间 (航向/地速/P·H·V DOP/模块时分秒), flags 里带"定位有效 / 时间有效"两位; 实现 = `main.c` 的 `tcp_poll_send_gps()` (**在 `tcp_server_task` 循环里调用**; `nav_read_gps()` 只加锁拷结构体、不碰 UART, 所以不抢 nav 接收任务的串口) + `control_build_gps_pos_frame()` / `control_build_gps_nav_frame()`; **模块没有 GPS 帧时一帧都不发** (上位机面板保持 `--`), `NAV_ENABLE=0` 时整段不编译; 客户端 `tools/tcp_console.py` 新增「GPS」面板 (未定位时经纬度/航向/地速显示 `—`, DOP/卫星数/模块时间照常)。⚠️ **帧长仍是 16B 定长**, 只新增 type 值 ⇒ 非破坏性。**已完成** (同步 §0 / §2.1 / §2.4 / §7.10.8 / §8 / §11 + README + `tools/README.md`)。上一版 v9.0: ① **4 路电调左右对调 —— GPIO42 = 右推进** —— 实机接线是 **GPIO42 接右边**, 而原代码 42=左, 故把 `motor.c` 的 4 个电调引脚与 `main.c` 的 `ESC_LEFT_GPIO`/`ESC_RIGHT_GPIO`/`REV_LEFT_GPIO`/`REV_RIGHT_GPIO` **整体对调**: **左推进 IO41 / 右推进 IO42 / 左反推 IO39 / 右反推 IO40** (原 42/41/40/39), 控制台语义不变 (`l`=左=IO41, `r`=右=IO42), **只改软件映射、不用重接线**; 对调属"改引脚/接线" ⇒ 按规则 **🔴 第一位 +1**。**已完成** (同步 §2 / §4 / §4.1 / §7.9 / §11 + README + `motor.c` / `motor.h` / `main.c` / `control.h` / `tools/tcp_console.py`)。上一版 v8.1: ① **启用主控 MPU 状态回传 (20Hz, type=0x01) + 板载灯只认 GPIO26** —— 上位机「主控 MPU」面板此前一直 `--` (回传是空实现); 现由 `attitude_task` 存 int16 快照 (`accel ×16384` / `gyro ×131`)、`tcp_server_task` 每 50ms 组 `type=0x01` 帧发给上位机 (**不新增任务, 也不在 TCP 任务里读 I2C** —— 避免与 100Hz 姿态解算抢 I2C1); MPU 未就绪时不发帧 (面板 `--`)。顺带把 `tcp_server_forward_mpu_to_host()` 从空实现改为**真透传** (远端 0xBB 0x66 帧原样转发), 8081 上线即可用。灯脚只保留 **GPIO26** (已确认), v8.0 加的 GPIO21"双保险"取消 ⇒ **GPIO21 恢复空闲**。**已完成**。上一版 v8.0.2: ① **修 nav 配置读回的"迟到应答串台"** —— 首次联调时日志出现 `RRATE 0x58F→0x05` (RRATE 读回成了 RSW 的值), 导致每次上电都误判"配置不符"并 **白写一次模块 Flash**; 根因是维特 **0x5F 应答不带地址**、只带"从被读地址起的 4 个连续寄存器", 而原实现连发两条读指令 ⇒ 前一条迟到应答被后一条误认。改为 `nav_read_regs()` **一条指令读 0x02 同时取回 RSW 与 RRATE** (少一次往返 + 竞态消除)。**已完成**。上一版 v8.0.1: ① **更正板载 WS2812 的数据脚: 实际是 GPIO26, 不是 GPIO21** —— v8.0 依据 Waveshare 官方 wiki 的 `RGB_LED` 示例判为 GPIO21, 但**用户按原理图/实测确认是 GPIO26** (GPIO26 不在 40Pin 排针上, 属板内网络)。`main.c` 改为 `BOARD_RGB_LED_GPIO_A=26` + `BOARD_RGB_LED_GPIO_B=21` (双保险), 启动第 0 步把**两脚都钉成输出低**; **不需要动任何外部接线** ⇒ 非破坏性。**已完成**。上一版 v8.0: ① **I2C1 的 SDA 由 GPIO21 挪到 GPIO16, 并禁用板载 WS2812** —— 板上那颗常亮白光就是 Waveshare ESP32-S3-ETH 的**板载 WS2812 RGB LED** (官方 wiki `RGB_LED` 示例: 数据脚 = **GPIO21**), 而本项目把 GPIO21 当 **I2C1 SDA** 用 ⇒ I2C 波形被它当颜色数据锁存成白色 (云台 MPU 100Hz 读取 ⇒ 每秒重写 100 次, 软件关不住)。`servo_get_default_config().sda_gpio` **21 → 16** (SCL 仍 17, 只挪一根线; 16 即 v7.0 腾出的脚), `main.c` 新增 `BOARD_RGB_LED_GPIO=21` 并在启动**第 0 步**把它钉成**输出低** ⇒ 无任何边沿进 WS2812, 保持熄灭 (⚠️ **需断电重上一次**才真正灭; 将来要做状态灯须用 RMT 驱动)。**已完成**。上一版 v7.0: ① **惯导模块 UART 引脚由 GPIO16/18 改为 GPIO1/2** —— 模块接 16/18 时 `nav` 报 `模块无应答` 且 `rx_bytes == 0` (7 档波特率全试过), 即物理层零字节; 代码侧已确认无引脚冲突、模块供电/接线/红灯均正常, 故改**复用原 GPS 实测跑通过的 GPIO1(RX)/GPIO2(TX)** (都不是 strapping 脚, GPIO3 原 PPS 仍空闲); `nav_get_default_config()` 的 `tx_gpio/rx_gpio` 由 18/16 改为 2/1, `nav.h` 接线说明与 `main.c` 的提示/启动日志同步; 风险极低 —— 回退只需改这两个数。**已完成**。上一版 v6.1: ① **新增控制台 `g` 命令 (按角度控制云台)**: `g <ch> [deg]` 直通 `servo_set_angle()` 按**标称角**下发, **受标定 + 软件行程限位约束** (ch1 出厂 30~150°; 越界时回显"被软件限位钳住, 实际 X°"), **不带 `deg` 只查询**当前角度/脉宽/可用窗口 (不动作); 与 `p` (直接给脉宽、**绕过限位**, 测机械行程/端点用) 分工明确; 缘由是云台 MPU 用途未定、`TEST_MODE` 置 `0` (云台纯开环、开机不自动动作、不依赖 MPU 反馈), 需要一条按角度的手动控制命令。**已完成**。上一版 v6.0: ① **惯导替换: 删除旧 GPS (NMEA-0183/UART1 2-1-3/PPS) 与旧"亚博 10 轴 IMU"(`7E 23` 请求式/UART2) 两套驱动, 新增 `components/nav`** —— 改用一块**亚博 GPS+10 轴 IMU 一体惯导模块** (**维特 WIT 标准 `0x55` 协议主动上报**, 单条 **UART2** (v6.0 时 RX=IO16/TX=IO18, **v7.0 起改为 RX=IO1/TX=IO2**), **9600** 8N1), 定长 **11 字节帧** (`0x55 | TYPE | 4×int16 | SUM`, SUM = 前 10 字节累加取低 8 位), 帧型覆盖 `0x50` 时间 / `0x51` 加速度+温度 / `0x52` 角速度+电压 / `0x53` 角度 / `0x57` 经纬度 / `0x58` GPS(海拔/航向/地速) / `0x5A` 卫星数+DOP, 一条线同时给出**姿态与 GPS**; 初始化先**自动扫描波特率** (9600→115200→57600→38400→19200→4800→230400), 再**读回输出配置, 只有与期望不符才**解锁(`FF AA 69 88 B5`)→写→SAVE(`FF AA 00 00 00`)→**回读验证** (期望 RSW(0x02)=`0x058F` / RRATE(0x03)=`0x05`=5Hz; 出厂默认 RSW=`0x001E` **不含 GPS 帧**故必须配一次; 只在必要时写是为了**避免每次上电写模块 Flash**); 模块没接只返回 `ESP_ERR_NOT_FOUND`, 接收任务照常启动, **后接上会自动补配置并出数据、不必重启** (每 5s 重试配置); `components/imu` 精简为**只剩云台 MPU6050** (删除 `7E 23` 请求式 UART 后端、旧 I2C(WIT 0x50) 分支与 `IMU_HULL_IFACE_UART`/`IMU_GIMBAL_SHARED_I2C1` 两个开关, `imu_role_t` **只剩 `IMU_ROLE_GIMBAL`**); `main.c` 的 `GPS_ENABLE` + `HULL_IMU_ENABLE` **合并为 `NAV_ENABLE`** (总开关, 现况见 §0), 删除 GPS 引脚宏 (GPIO3 闲置; **1/2 于 v7.0 归惯导模块**), 控制台 `gps`/`hull` 改为读惯导快照 (**去掉 `gps raw|baud|send` 与 `hull addr`**)、**新增 `nav`** 状态命令, `imu` 只读云台一颗、`scan` 只扫 I2C1; 日志任务 `gps_log`/`hull_log` 换成 `nav_gps`(1Hz) / `nav_att`(10Hz), `nav_task`(4/3072) 由 nav 组件内部创建; 权威依据是仓库内 `10轴IMU模块通讯协议.pdf` 与 `STM32-串口应用例程(IMU)/` 的维特官方 SDK/STM32 例程。**已完成**。上一版 v5.12: ① **网络联调: 恢复 W5500 + TCP Server (仅 8080), 上位机控制帧驱动 4 路电调 + 滚筒** —— `W5500_ENABLE=1`, 新建 `tcp_server_task` (栈 8192 / 优先级 5) 监听 **8080**; 上位机 `tools/tcp_console.py` (TCP Client) 按 20Hz 下发 16B 控制帧, **差速混合在 `main.c` 执行** (`left=clamp(speed+yaw)` / `right=clamp(speed-yaw)` → 正=推进·负=反推, 复用 `esc_apply_side`), `bucket_speed` → **L298N 滚筒收放电机**; `control` 组件**改为只做协议解析** (新增 `control_set_local_callback()`, 删掉写死旧硬件的 `mix_and_execute_local()` —— 它把负油门发给单向电调会被 `motor_set_esc_throttle()` 钳成 0, 反推两路永不动); 新增 **500ms 失联保护** (连续丢 10 帧 ⇒ 4 路电调 + 滚筒全部归零) 与控制台 **`net`** 状态命令; 客户端「铲斗」区改为「滚筒收放」(收/停/放按钮); **8081 远端转发与 MPU 状态回传暂未启用**; 删除 `main.c` 里约 240 行按旧硬件写的注释代码。**已完成**。上一版 v5.11.2: ① **启用板载 W5500 以太网 (仅拿 IP)**: `wiznet_manager_init()` 配置静态 IP 192.168.29.10/24、网关 .1 (板载固定 13/11/12/14/9/10), 上电即拿到 IP; **TCP Server 与全部 TCP 收发/转发仍注释**。② **云台闭环新增"无反馈保护"**: 超过 `GIMBAL_FB_TIMEOUT_MS`(200ms) 收不到姿态反馈 (云台 MPU 未就绪 / 零偏标定失败 / 读数中断) ⇒ **停用 PID 并把该轴送回标定中位** (ch1 = 90°), 不再拿"假 0°"当反馈把俯仰顶到限位下限 (30°); 反馈恢复自动接管; 状态新增 `nofb` (`gs` 显示 `N/FB`); ③ **PCA9685 探测失败不再删 I2C1 总线** (`servo.c`, v5.11.2): 该总线还挂着船体 10 轴 IMU (0x50), 原先探测失败会 `i2c_driver_delete()` 把总线删掉, 导致亚博 IMU 一起失效、`scan 1`/`hull` 在线重探也报 `i2c driver not installed`; 现改为**保留总线** —— servo 自身功能关闭 (`servo_is_ready()==false`, API 返回 `ESP_ERR_INVALID_STATE`), 船体 IMU 照常工作并可在线重探; ④ 清理换板遗留: 删除 §11「状态指示灯」全章并章节顺延 (原 §12 构建与烧录 → §11), 内容归档至 `docs/legacy/`; 更正 `servo.h` 中旧板 I2C 引脚注释 (`SDA=4/SCL=5` → `21/17`); ⑤ 更正 `ESP32-S3-ETH-pinout.md` §六 I2C 推荐脚 (`GPIO47/48` 在本项目已作 L298N 方向脚); ⑥ **GPS 诊断增强**: `gps_data_t` 新增 `rx_bytes` (累计原始字节数), 控制台 `gps` 打印 `收到字节/语句/错误` 并自动给结论 (区分"没接线"与"波特率不对"); 确认 GPS 硬件 **ATGM336H** 与本组件兼容 (见 §7.8.1)。⑦ **修复"插原生 USB 只能看日志、敲不进命令" + 启用 L298N 电机**: console 主通道由 UART0 改为**板载原生 USB (USB-Serial/JTAG)** (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` + 次要 console 关, 需删 `sdkconfig` 重编) —— 原先插原生 USB 只能看日志、敲字会写超时 (IDF 的次要 console 只输出不收输入); 现插原生 USB 口即日志 + `gimbal>` 交互, 烧录同口。另 `#define MOTOR_ENABLE 1`: 只配 L298N 通道 (ENA=IO38 5kHz PWM / IN1=IO48 / IN2=IO47, 上电强制停机), 4 路电调 gpio 置 -1; 控制台 `m <-100~100>` (裸数字同) 调速; ⑧ **亚博 10 轴 IMU 由 I2C 改接 UART (根治"扫不到 0x50"的协议不匹配问题)**: 厂商 UART 示例表明该模块**不是维特(WIT) I2C 协议**, 而是自有协议 (帧 `7E 23 | LEN | FUNC | DATA | SUM`, `0x80` 请求式, **115200**) —— `imu` 组件新增 UART 后端 (`imu.h: IMU_HULL_IFACE_UART`) 装 **UART2 (ESP32 RX=IO16/TX=IO18)**, 上层接口不变, I2C(WIT) 分支条件编译保留可回退; 同时**云台 MPU6050 挪到 I2C1 与 PCA9685 共线** (`IMU_GIMBAL_SHARED_I2C1`) ⇒ 腾出的 **GPIO16/18 正好给 UART**, 初始化顺序随之调整 (两颗 IMU 合并到 `main.c` 4c, **必须在 `servo_init()` 之后**); ⑨ **修复 UART 角速度量纲错误**: `hull_uart_parse_raw()` 里 `gyr_ratio` 多乘了一次 `180/π`, 角速度被**放大 57.3 倍** (手转模块时显示到 ±10000dps ≈ 3 圈/秒, 与"同一时刻欧拉角只动几度"矛盾), 已改为 `gyr_ratio = 2000/32767` (原始 int16 满量程 ±2000dps 直接得 °/s), 手转回落几十~几百 dps、静止 ±2dps; ⑩ **修复 UART 欧拉角单位错误**: `0x26` 的 3×float32 出的是**弧度**, 原先被当成度直接打印 (某轴竖直 90° 时只显示 `1.58`), 已在 `hull_uart_parse_euler()` 里乘 `180/π`; ⑪ **GPS 按 NMEA-0183 补全**: 新增 `GSA`/`GSV`/`GST`/`VTG`/`ZDA`/`$GPTXT` (天线状态) 解析, RMC 失效时坐标/速度/航向**清零**、空字段不再被当成 0; 控制台新增 `gps raw` (原始语句回显) / `gps baud <n>` / `gps send <语句>`。上一版 v5.11.1: ① 修复 `motor.c` **LEDC 通道冲突**: L298N DC 的 `dc_ledc_channel` 由 `LEDC_CHANNEL_0` 改为 **`LEDC_CHANNEL_4`** (Timer1 不变), 避免与 ESC1 (Timer0/Channel0) 抢占同一通道; ② 删除 `sdkconfig`/`sdkconfig.old` 重新生成, 使板载 **16MB Flash** (`CONFIG_ESPTOOLPY_FLASHSIZE_16MB`) 从 `sdkconfig.defaults` 生效并全量重编译; ③ 更正 `motor.c` / `motor.h` 中过时的引脚注释与启动日志; 上一版 v5.11.0: ① 固件切换为 **[临时测试/标定模式]**: 以太网/Motor/control 协议**暂未启用**, 实跑模块为 云台 MPU6050 + PCA9685 + 船体10轴IMU + GPS + 云台运动规划/姿态闭环; ② 新增 `components/gps` 组件; ③ servo 组件新增**标定体系**(双角度刻度/行程限位/NVS 存取); ④ gimbal 组件新增**姿态闭环 PID 自稳**; ⑤ 新增**串口标定控制台**; 上一版 v5.10.0: 换板 Waveshare ESP32-S3-ETH + 同步远端 v3.0 沉浮协议 + 删除 `status_led`)
> 配套远端文档: [`..\ESP32-S3-below\ESP32-S3水下机器人项目参数总览.md`](../ESP32-S3-below/ESP32-S3水下机器人项目参数总览.md) (v3.0, 已完全更新)
> 新板引脚依据: [`ESP32-S3-ETH-pinout.md`](ESP32-S3-ETH-pinout.md)

---

## 0. 当前固件形态 (重要)

> 📌 **版本号规则** (v6.0 起执行): 🔴 破坏性变更 ⇒ **第一位 +1**; 🟡 非破坏的功能新增 ⇒ 第二位 +1; 🟢 修复/文档 ⇒ 第三位 +1。
> 规则全文、各代际划分与**逐条回溯判定表**见 **§10 开头** (⚠️ 历史 v5.x 不重排)。

> ⚠️ **现状**: 本工程固件当前为 **[当前模式]** —— 实跑 **W5500 + TCP Server 8080 (上位机内网操控) + 水面推进电调 (2 路) + 反推方向线 (2 路, ⚠️ **暂时关闭**) + L298N 滚筒收放 + 惯导模块 (GPS + 10 轴 IMU) + **云台 (PCA9685 舵机 ch0/ch1)** + 串口控制台**; **6 个总开关**: 5 个为 `1` (`W5500_ENABLE` / `MOTOR_ENABLE` / `NAV_ENABLE` / `SERVO_ENABLE` / `MPU6050_ENABLE`) + `REV_ESC_ENABLE = 0` (**反推暂时关闭**: 负油门按 0 (停) 处理, 方向线只保持正向半区), `TEST_MODE = 0` = 云台**纯开环** (不开闭环、开机不自动动作、不依赖 MPU 反馈); **8081 远端转发**、**control 转发子帧**未启用 (**状态回传已启用**: MPU `type=0x01` 20Hz (v8.1) + GPS `type=0x04`/`type=0x05` 各 2Hz (v9.1))。
>
> 🔄 **v6.0 起感知硬件只剩一块惯导模块** (一条 UART2 同时给姿态与 GPS): 旧的 `components/gps` **组件已删除** (NMEA-0183/UART1 TX=2/RX=1/PPS=3; **其中 1/2 于 v7.0 归惯导模块**, GPIO3 空闲), `components/imu` 里旧的"船体亚博 10 轴 IMU"代码 (`7E 23` 请求式 UART / I2C WIT 0x50) **也已删除**, 该组件现在只剩云台 MPU6050。惯导模块细节见 **§7.10**, GPS 数据来源见 **§7.8**, imu 组件见 **§7.5**。
>
> | 分类 | 内容 |
> |---|---|
> | **已启用** | **W5500 以太网 + TCP Server 8080** (静态 IP `192.168.29.10/24`; 收上位机 16B 控制帧 → **差速混合驱动 4 路电调**, `bucket_speed` → L298N 滚筒; **500ms 失联保护**; 控制台 `net` 查状态, 见 §2 / §6) + **水面推进电调 2 路 + 反推方向线 2 路** (推进 ESC1/ESC2 = **IO41 左 / IO42 右**; 反推方向线 REV1/REV2 = **IO39 左 / IO40 右**; 好盈 SkyWalker V2 "**反推刹车**"接线 —— 方向线只给 0/100% 两个方向位, 速度全走油门线; 50Hz, 1100µs=停 / 1141~1940µs=油门; ⚠️ **反推暂时关闭** `REV_ESC_ENABLE=0` ⇒ 负油门按 0 (停) 处理, 方向线只保持在正向半区 (1100µs); 控制台 `l/r <0~100>` / 裸数字 / TCP) + **L298N 滚筒收放电机** (ENA=IO38 5kHz PWM / IN1=IO48 / IN2=IO47; 控制台 `m <-100~100>` 或 TCP `bucket_speed`) + **惯导模块** (**亚博 GPS + 10 轴 IMU 惯导组合** —— 硬件是两块板, 官方融合成**一条 UART2**(ESP32 RX=IO1 / TX=IO2 @9600, 维特 WIT `0x55` 主动上报); 一台机器一次给出**姿态 + GPS**; 控制台 `nav` 查状态 / `gps` 看定位 / `hull` 看姿态, 见 §7.10) + **云台** (PCA9685 舵机 ch0/ch1 = **I2C1 IO16/17**, 地址 0x40, 开机回中; **手动摆位有两条路** —— 控制台 `st` / `p` / `n` / `z` / `t` / `ck` / `sc` / `sl` / `gs` (见 §7.9) 与 **上位机「云台手动」两个拖动条 (cmd=0x12, v9.2, 见 §2.2)**, 两者都走 `servo_set_angle()` 受标定+限位约束; ⚠️ 云台 MPU6050 **暂时废弃** `MPU6050_ENABLE=0` ⇒ 不初始化/不自检, `ge`/`gt`/`gp` 闭环**没有反馈源**; `TEST_MODE=0` = 云台**开环**, 开机不自动动作, 见 §7.6) + **状态回传 (主控 → 上位机, 8080)**: 云台 MPU6050 原始 6 轴 → 16B `0xBB 0x66` **type=0x01**, **20Hz** (v8.1; 由 `attitude_task` 存快照、`tcp_server_task` 发送, 见 §7.5.5; ⚠️ **`MPU6050_ENABLE=0` 时无数据源 ⇒ 自动停发, 面板恒 `--`**) + **GPS** → `type=0x04` 定位 / `type=0x05` 运动+精度+时间, **各 2Hz** (v9.1; 数据取自惯导模块, 布局见 §2.4, 见 §7.10) + 串口标定控制台 (**板载原生 USB / USB-Serial/JTAG**) |
> | **暂时关停** (调试开关, 见 `main.c` 顶部) | **① 云台 MPU6050 + 姿态解算 (`MPU6050_ENABLE = 0`)** —— 不初始化、不做地址自检、不读有效性 (不会再有 `0x68 无应答` 之类的 ERROR), `attitude_task` 不创建 ⇒ 「主控 MPU」回传停发、云台闭环没有反馈源; 重新启用只需把该宏置回 `1` (云台 MPU 接上即可)。**② 反推电调方向线 (`REV_ESC_ENABLE = 0`)** —— 负油门按 0 (停) 处理 (船不能倒退, 转向只能靠两侧推力差), 方向线 IO39/IO40 仍初始化并保持在 1100µs 正向半区。其余 4 个总开关 (`W5500_ENABLE` / `MOTOR_ENABLE` / `NAV_ENABLE` / `SERVO_ENABLE`) **全部为 `1`**; 要临时关停别的就置 `0` (⚠️ 但 v6.0 已删除的旧 GPS 组件与旧"亚博 10 轴 IMU"UART 代码**无法再退回**) |
> | **暂未启用** | **8081 远端转发** (水下 ESP32-S3 节点未就绪; 一旦有远端状态帧进来, 主控会**原样透传**给上位机 —— `tcp_server_forward_mpu_to_host()` 已实现) / `status_report_task`。设计保留在 §2, 代码待恢复 |
>
> 因此下文 **§2 / §6 / §8 / §9 的协议与网络参数已按实况更新** (8080 已启用并带状态回传, 8081 仍属设计);
> **§1 中与水下链路相关的内容仍是设计目标**。
> **§7.5 / §7.6 / §7.7 / §7.8 已加"关停/删除"说明; 惯导模块见新增的 §7.10**。
> **§1 / §3 / §4 / §7 / §7.9 / §7.10 / §10 / §11 及以下章节已按当前实况更新**。

---

## 1. 系统架构

> ⚠️ 本章描述**完整系统设计** (含网络链路)。当前固件形态见 **§0**: **8080 上位机链路 + Motor + control 已启用**, 而 **8081 远端转发与 MPU/状态回传仍是设计目标**, 本章中与远端相关的条目为**设计目标**。

### 1.1 节点角色

| 节点 | 角色 | 硬件 | 网络角色 |
|---|---|---|---|
| **主控制节点** (本项目) | 核心控制 + 数据处理 + 指令分发 | **惯导模块 (亚博 GPS+10 轴 IMU 一体, UART2 1/2)** + 4 ESC + 1 L298N + 2 舵机(PCA9685) + 云台 MPU6050 | TCP **Server** (双端口) |
| **远端执行节点** (另一 ESP32-S3, 用户负责) | 远端执行 + 本地传感 + **沉浮控制 (v3.0)** | **YB-MRA02 IMU (UART) + 2 水平 ESC + 3 沉浮 ESC + 1 路 L298N 铲斗 + 1× MS5837-30BA** | TCP **Client** |
| **网络摄像头** (局域网) | 视频源 | 摄像头 | IP 视频流 (RTSP/HTTP) → ROCK 5C |
| **ROCK 5C** (Radxa SBC) | **推流 + AI 推理** | SBC + 摄像头输入 | **内网视频流** (AI 推理后输出, 客户机直连) |
| **上位机** (笔记本) | 人工控制 + 姿态解算 + 决策 | 无 (软件) | TCP **Client** + 直接访问 ROCK 5C AI 视频流 |

> **v3.0 (2026-09)**: 主控转发**沉浮控制**指令 (16B cmd=0x11 → 8B 沉浮子帧 → 远端 `vertical_ctrl`), 并把远端沉浮状态 (type=0x03) **原样转发**上位机. `cmd=0x11` 字节 3-6 = 目标深度(cm) + 目标俯仰(°) + 目标横滚(°), 取代 v2.x 的"深度 cm + 模式". 详细方案见配套远端文档 §4.5 与 `docs/v3.0_vertical_ctrl_design.md`.
>
> **v2.0~v2.19**: 压载深度控制 (水舱 + 4× MS5837), 已随远端 v3.0 整体废弃, 见远端 `docs/legacy/`.
>
> **v5.9.0**: ROCK 5C (Radxa SBC) 同时承担**推流 + AI 推理**, 输出的 AI 推理视频流经**内网直接访问**. 该链路**与主控节点无任何数据交互**, ESP32 不参与视频/推流链路 (原 RDK X5 UART 方案已取消, 见 §10 变更记录).

### 1.2 网络拓扑

```
   上位机 (笔记本)                远端 ESP32-S3
   TCP Client 8080               TCP Client 8081
        │                              │
        │ 16B 控制帧 (speed+yaw)       │ 8B 子帧
        │ 16B 沉浮控制帧 (0x11) v3.0   │ 8B 沉浮子帧 v3.0
        │ 16B MPU 帧 (本+远端, 20Hz)   │ 16B MPU 帧
        │ 16B 沉浮状态帧 (0x03) v3.0   │ 16B 沉浮状态 v3.0
        ▼                              ▼
   ┌────────────────────────────────────────────────────────────┐
   │  主控制节点 (本工程) - TCP Server                          │
   │  socket 0 = 8080 (HOST)                                   │
   │  socket 1 = 8081 (REMOTE)                                 │
   │  + 本地执行 (4 ESC + L298N + 舵机)                        │
   │  + 转发 (16B 控制 → 8B 子帧) [待恢复]                     │
   │  + 转发 (16B DEPTH → 8B 沉浮子帧, v3.0) [待恢复]           │
   │  + 推送 (本 MPU 20Hz → 主机+远端)       [待恢复]           │
   │  + 转发 (远端 MPU 20Hz → 主机)          [待恢复]           │
   │  + 转发 (远端沉浮状态 type=0x03 → 主机) [待恢复]           │
   └────────────────────────────────────────────────────────────┘

   ── 内网 (LAN) 视频链路: 与主控节点无数据交互 ────────────────────
     网络摄像头 ──(RTSP/HTTP 视频流)──▶ ROCK 5C ──(AI 推理后视频流)──▶ 上位机 / 内网客户机
```

> **现状 (v9.1)**: 上图里**已生效**的是 —— "socket 0 = 8080" (收上位机 16B 控制帧 → 差速混合 → 推进电调 + 反推方向线 + L298N 滚筒, 带 500ms 失联保护)、"**本地执行**" (**含舵机**: PCA9685 已启用, 控制台 `p`/`g`/`n`/`z` 可手控, `ge`/`gt`/`gp` 可跑姿态闭环 —— 见 §7.6/§7.9)、以及**状态回传** (主控 MPU `type=0x01` 20Hz + GPS `type=0x04`/`0x05` 各 2Hz)。
> 标 **[待恢复]** 的 (socket 1 = 8081 / 转发到远端 / 远端沉浮状态 `type=0x03` 转发) **均未启用** (远端上线前无数据可转)。

### 1.3 数据流向

> ⚠️ 下表为**设计目标**。现状 (v9.1): 已生效的是 **第 1 行 (上位机 → 主控 16B 控制帧 @8080)** 与 **主控 → 上位机的状态回传 (MPU `type=0x01` 20Hz / GPS `type=0x04`·`0x05` 各 2Hz)**;
> 与远端 8081 的全部收发、以及**远端沉浮状态 (`type=0x03`) 回传**(要等远端上线才有帧) **仍未启用**, 见 §0。

| 链路 | 方向 | 数据类型 | 速率 |
|---|---|---|---|
| 主↔主机 (8080) | 上→主 | 16B 控制帧 (含 cmd=0x11 DEPTH, v3.0) | 按需 (主机主动发) |
| 主→主机 (8080) | 主→上 | 16B MPU 帧 (主控本地 MPU6050) | 20Hz (v8.1 起) |
| 主→主机 (8080) | 主→上 | 16B GPS 帧 (type=0x04 定位 / 0x05 运动, v9.1) | 2Hz (惯导上报) |
| 主→主机 (8080) | 主→上 | 16B 沉浮状态帧 (type=0x03, v3.0) | 2Hz (远端上报, 8081 上线后才有) |
| 主→主机 (8080) | 主→上 | 500ms JSON 状态 (可选) | 2Hz |
| 主↔远端 (8081) | 主→远 | 8B 子帧 (2 远端电调) | 按需 |
| 主→远端 (8081) | 主→远 | 8B 沉浮子帧 (cmd=0x11, v3.0) | 按需 |
| 主→远端 (8081) | 主→远 | 16B MPU 帧 (本节点 MPU) | 20Hz |
| 远→主 (8081) | 远→主 | 16B 沉浮状态帧 (type=0x03, v3.0) | 2Hz |

> v3.0 **取消** `type=0x02`(远端原始 MPU, 原 20Hz): 远端姿态改由 YB-MRA02 内部融合后经 `type=0x03` 的 `pitch/roll` 承载.

---

## 2. TCP 协议规范 (v5.12: 8080 收控制帧已启用; 8081 转发仍属设计)

> ✅ **v5.12 现状**: `control` 组件**已初始化**并在 `tcp_server_task` 中工作 —— socket 0 监听 **8080**, TCP Client (`tools/tcp_console.py`) 连上后按 **20Hz** 下发 **16B 控制帧**, `control_process()` 解析 → 经 `control_set_local_callback()` 回调 `main.c` 的 `tcp_apply_local_command()` 执行本地动作。
> ❌ **仍未启用**: **8081 (远端转发)** —— `cmd=0x11 DEPTH` 无转发目标, 收到后只解析不动作 (见 §0); 但远端帧一旦从 8081 进来会被 `tcp_server_forward_mpu_to_host()` **原样透传**给上位机。
> ✅ **主控状态回传 (主控 → 上位机, 均走 8080)**: `type=0x01` 主控 MPU 原始 6 轴 **20Hz** (v8.1) + `type=0x04`/`type=0x05` **GPS 定位/运动 各 2Hz** (v9.1, 数据来自惯导模块)。
> 代码侧帧常量/构造器 (见 §8) 与本章保持一致。

### 2.1 帧类型总览

| 帧名 | 帧头 | 长度 | 方向 | 用途 |
|---|---|---|---|---|
| 控制帧 | 0xAA 0x55 | 16B | 上位机 → 主 | 完整控制指令 (本地+远端) |
| **沉浮控制帧** | **0xAA 0x55** | **16B** | **上位机 → 主** | **目标深度 cm + 俯仰° + 横滚° (cmd=0x11, v3.0)** |
| **云台手动帧** | **0xAA 0x55** | **16B** | **上位机 → 主** | **ch0/ch1 目标标称角 ×0.1° + 通道掩码 (cmd=0x12, v9.2)** |
| 转发子帧 | 0xAA 0x55 | 8B | 主 → 远端 | 远端电调 + 系统命令 |
| **沉浮控制子帧** | **0xAA 0x55** | **8B** | **主 → 远端** | **目标深度 cm + 俯仰° + 横滚° (cmd=0x11, v3.0)** |
| MPU 数据帧 | 0xBB 0x66 | 16B | 双向 | 6 轴 IMU 原始数据 (type=0x01/0x02) |
| **GPS 状态帧** | **0xBB 0x66** | **16B** | **主 → 上位机** | **GPS 定位 (type=0x04) / 运动+精度+时间 (type=0x05), v9.1, 各 2Hz** |
| **沉浮状态帧** | **0xBB 0x66** | **16B** | **远端 → 主 → 上位机** | **深度/俯仰/横滚/模式/三路油门 (type=0x03, v3.0)** |

### 2.2 控制帧 (16B) — 上位机 → 主控制节点 (v4.0 差速版)

| 字节 | 名称 | 类型 | 含义 |
|---|---|---|---|
| 0 | HEAD0 | uint8 | 0xAA (固定) |
| 1 | HEAD1 | uint8 | 0x55 (固定) |
| 2 | cmd | uint8 | 0x10=电机 / **0x11=沉浮控制 (v3.0)** / **0x12=云台手动 (v9.2)** / 0x20=急停 / 0x30=重启 / 0x40=关机 |
| 3 | **speed** | **int8** | **整体推进速度 (-100~+100, 正=前进)** |
| 4 | **yaw** | **int8** | **偏航/转向 (-100~+100, 正=右转)** |
| 5 | **remote_light** | **uint8** | **远端灯开关 (0=关, 1=开)** |
| 6 | **bucket_speed** | **int8** | **远端 L298N 铲斗电机速度 (-100~+100, 0=停)** |
| 7 | reserved | uint8 | 保留 |
| 8-9 | ~~servo~~ | ~~uint8~~ | **~~已删除~~** (原舵机控制) |
| 10 | flags | uint8 | bit0: 本地执行 / bit1: 转发远端 |
| 11-14 | reserved | uint8 | 保留 |
| 15 | CRC8 | uint8 | 前 15 字节异或 |

> **v3.0 沉浮控制帧 (cmd=0x11)**: 复用 16B 结构, 字节 3-6 语义变为:
>
> | 字节 | 名称 | 类型 | 含义 |
> |---|---|---|---|
> | 3-4 | target_depth | int16 LE | **目标深度 (cm), 0=上浮至水面** |
> | 5 | target_pitch | int8 | **目标俯仰角 (°), 正=抬头** |
> | 6 | target_roll | int8 | **目标横滚角 (°), 正=右滚** |
> | 7-9 | reserved | uint8 | 保留 |
> | 10 | flags | uint8 | bit1 置位才转发远端 |
>
> 主控**本地不执行** (不沉浮), 仅按 flags bit1 构造 8B 沉浮子帧转发远端.
>
> ⚠️ **与 v2.x 的差异**: 字节 5 由 `mode (uint8)` 改为 `target_pitch (int8)`, 字节 6 由"保留"改为 `target_roll (int8)`.
> 过渡期若上位机仍按旧语义发 `mode = 1~5`, 会被远端解读为 1~5° 的俯仰目标; 改造完成前请只发 `mode = 0`.
> 不再支持"强制模式"——模式由远端 `vertical_ctrl` 依据目标/实测深度自行判定 (SURFACE/DESCEND/HOVER/ASCEND/EMERGENCY).

> **v9.2 云台手动帧 (cmd=0x12)**: 复用 16B 结构, 字节 3-7 语义变为:
>
> | 字节 | 名称 | 类型 | 含义 |
> |---|---|---|---|
> | 3-4 | ch0_deg10 | int16 LE | **ch0(平面旋转) 目标标称角 ×0.1°** (3600 → 360.0°) |
> | 5-6 | ch1_deg10 | int16 LE | **ch1(俯仰) 目标标称角 ×0.1°** |
> | 7 | ch_mask | uint8 | **bit0=下发 ch0, bit1=下发 ch1**; 未置位的通道**完全不动** |
> | 8-9 | reserved | uint8 | 保留 |
> | 10 | flags | uint8 | bit0 置位才执行 (与电机帧一致) |
> | 11-14 | reserved | uint8 | 保留 |
>
> - **本地执行**: 主控对置位通道调 `servo_set_angle()` —— 与控制台 `g` 走同一条路, **受标定 + 软件行程限位约束**
>   (ch1 出厂 30~150°, 越界自动钳住); 只在**角度变化**时才写 I2C (拖动条会连发同一值)。
> - **向后兼容**: 老上位机不发这个 cmd, 即使发了 `ch_mask=0` 也什么都不做 ⇒ 不会有误动作。
> - **上位机实现**: `tools/tcp_console.py` 「云台手动」面板的两个拖动条 (ch0 0~360°, ch1 30~150°),
>   **拖动即发** (`send_now`, 不走 20Hz 控制帧循环), 需勾选「启用下发」才会真的发。
> - ⚠️ **失联保护只归零电机, 不碰舵机** (PCA9685 自己维持 PWM); 要舵机也回中得手动点「急停」或控制台 `z`。

### 2.3 转发子帧 (8B) — 主控制节点 → 远端 (v4.0)

| 字节 | 名称 | 类型 | 含义 |
|---|---|---|---|
| 0 | HEAD0 | uint8 | 0xAA |
| 1 | HEAD1 | uint8 | 0x55 |
| 2 | cmd | uint8 | 同控制帧 |
| 3 | **speed** | **int8** | **速度** |
| 4 | **yaw** | **int8** | **偏航** |
| 5 | **remote_light** | **uint8** | **远端灯开关** |
| 6 | **bucket_speed** | **int8** | **远端 L298N 铲斗电机速度 (-100~+100, 0=停)** |
| 7 | CRC8 | uint8 | 前 7 字节异或 |

> **v3.0 沉浮控制子帧 (cmd=0x11)**: 字节 3-6 语义变为 `target_depth int16 LE (cm)` + `target_pitch int8 (°)` + `target_roll int8 (°)`,
> 由 `forward_to_remote()` 在 cmd=0x11 时用 `control_build_depth_fwd_frame(fwd, depth_cm, pitch, roll)` 构造 (CRC 仍为前 7 字节异或).

**远端做差速混合** (与主节点本地相同公式) 后驱动 2 电调, 并按 `bucket_speed` 驱动 L298N 铲斗电机, 同时控制灯.

### 2.4 MPU 数据帧 (16B) — 双向

| 字节 | 名称 | 类型 | 含义 |
|---|---|---|---|
| 0 | HEAD0 | uint8 | 0xBB |
| 1 | HEAD1 | uint8 | 0x66 |
| 2 | type | uint8 | 0x01=本地 MPU / 0x02=远端 MPU / **0x03=沉浮状态 (v3.0)** / **0x04=GPS 定位 (v9.1)** / **0x05=GPS 运动 (v9.1)** |
| 3-4 | 见下 | int16 LE | type=0x01/0x02 时为 ax; **type=0x03 时为 depth_cm** |
| 5-6 | 见下 | int16 LE | type=0x01/0x02 时为 ay; **type=0x03 时为 pitch (0.1°)** |
| 7-8 | 见下 | int16 LE | type=0x01/0x02 时为 az; **type=0x03 时为 roll (0.1°)** |
| 9-10 | 见下 | int16 LE | type=0x01/0x02 时为 gx; **type=0x03 时为 mode + out_f** |
| 11-12 | 见下 | int16 LE | type=0x01/0x02 时为 gy; **type=0x03 时为 out_rl + out_rr** |
| 13-14 | 见下 | int16 LE | type=0x01/0x02 时为 gz; **type=0x03 时为 flags + 保留** |
| 15 | CRC8 | uint8 | 前 15 字节异或 |

> **v3.0 沉浮状态帧 (type=0x03)**: 远端 `vertical_ctrl` 每 500ms 上报, 主控 `handle_mpu_frame`
> **原样转发**至上位机 (type 不变, 字节 3-14 逐字节不变). 字段布局:
>
> | 字节 | 名称 | 类型 | 含义 |
> |---|---|---|---|
> | 3-4 | depth_cm | int16 LE | 当前绝对深度 (cm) |
> | 5-6 | pitch | int16 LE | 当前俯仰角 (**0.1°**, 15 → 1.5°) |
> | 7-8 | roll | int16 LE | 当前横滚角 (**0.1°**) |
> | 9 | mode | uint8 | 0=SURFACE / 1=DESCEND / 2=HOVER / 3=ASCEND / 4=EMERGENCY |
> | 10 | out_f | int8 | 沉浮前电机油门 (%) |
> | 11 | out_rl | int8 | 沉浮后左电机油门 (%) |
> | 12 | out_rr | int8 | 沉浮后右电机油门 (%) |
> | 13 | flags | uint8 | bit0=深度有效 bit1=IMU有效 bit2=手动(沉浮已暂停) |
> | 14 | reserved | uint8 | 保留 |
>
> ⚠️ **与 v2.x 的差异 (字节位置整体前移)**: 旧 `depth_a/depth_b/comp_pressure/est_water` 四组字段被替换为 `depth_cm` 单值 +
> `pitch` + `roll`; 旧 `[11] mode` 移到 `[9]` 且取值不同 (0~4 语义改为 SURFACE/HOVER/DESCEND/ASCEND/EMERGENCY),
> 旧 `[12] pid_out`、`[13] actuator_bits` 被 `out_f/out_rl/out_rr`、`flags` 取代. 旧解析器会读出无意义的值, **必须按上表改写**.

> **v9.1 GPS 状态帧 (type=0x04 / 0x05)**: 主控把惯导模块 (亚博 GPS+IMU, `components/nav`) 的 GPS 快照
> 打包成两个 16B 帧, 由 `tcp_poll_send_gps()` 在 TCP 任务里**各 2Hz** 发给上位机「GPS」面板
> (构造器 `control_build_gps_pos_frame()` / `control_build_gps_nav_frame()`; 模块无 GPS 帧时**一帧都不发**, 面板保持 `--`).
>
> **type=0x04 (GPS 定位)**:
>
> | 字节 | 名称 | 类型 | 含义 |
> |---|---|---|---|
> | 3-6 | lat | int32 LE | 纬度 ×1e7 (°), 负 = 南纬 |
> | 7-10 | lon | int32 LE | 经度 ×1e7 (°), 负 = 西经 |
> | 11-12 | alt | int16 LE | 海拔 ×0.1 m |
> | 13 | sats | uint8 | 卫星数 |
> | 14 | flags | uint8 | bit0=定位有效 (FIX) bit1=模块时间有效 (TIME) |
>
> **type=0x05 (GPS 运动/精度/时间)**:
>
> | 字节 | 名称 | 类型 | 含义 |
> |---|---|---|---|
> | 3-4 | course | uint16 LE | 航向 ×0.01° (0~35999) |
> | 5-6 | speed | uint16 LE | 对地速度 ×0.01 km/h |
> | 7 | pdop | uint8 | 位置精度因子 ×0.1 (上限 25.5, 超出截顶) |
> | 8 | hdop | uint8 | 水平精度因子 ×0.1 |
> | 9 | vdop | uint8 | 垂直精度因子 ×0.1 |
> | 10-12 | hour / minute / second | uint8 | 模块时间 (时区由模块 TIMEZONE 决定, 默认 UTC+8) |
> | 13 | flags | uint8 | 同 type=0x04 |
> | 14 | reserved | uint8 | 保留 0 |
>
> ⚠️ **未定位时 lat/lon 已被驱动清零** (nav 不保留上一次的坐标), 上位机只看 `flags.bit0`;
> 同样地航向/地速在未定位时也不显示 (面板显示 `—`)。**VDOP/航向/地速是 8 位或 16 位定标值, 有精度损失** (DOP 0.1 / 航向 0.01° / 地速 0.01 km/h), 需要原始精度请看控制台 `gps`。

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
| 0x10 | MOTOR | 电机控制 (差速混合, 按 flags 执行) |
| 0x11 | DEPTH | 沉浮控制 (v3.0: 目标深度 cm + 目标俯仰° + 目标横滚°, 仅转发远端) |
| 0x20 | STOP | 紧急停止所有电机 (本地 + 远端) |
| 0x30 | REBOOT | 系统重启 (主控制节点, 500ms 延迟) |
| 0x40 | SHUTDOWN | 深度睡眠关机 |

> ⚠️ REBOOT / SHUTDOWN 命令作用于**主控制节点本身**, 不转发到远端.
> ⚠️ DEPTH (0x11) 命令主控**本地不执行**, 仅按 flags bit1 转发远端 (见 §2.2)。**v5.12: 8081 转发未启用 ⇒ 收到 DEPTH 帧只解析、不动作**。

### 2.7 差速混合算法 (主节点本地 + 远端)

> ⚠️ **v5.12 起差速混合与电机驱动不在 `control` 组件内**, 而是由 `main.c` 的 `tcp_apply_local_command()` 实现 (control 只解析协议并通过 `control_set_local_callback()` 回调)。
> 原因: 旧实现在 `control.c` 里写死旧硬件 (`mix_and_execute_local()`), 把**负油门**直接交给单向电调 ⇒ 被 `motor_set_esc_throttle()` 钳成 0, **两路反推永不动**。现在交给 `main.c` 按当前硬件 (单向 ESC + 独立反推通道) 混合。

主节点收到控制帧 (speed, yaw) 后, 进行差速混合并执行本地电机:

```
left  = clamp(speed + yaw, -100, +100)  →  >0 左推进: 油门线 ESC1 (IO41) 给速度, 方向线 REV1 (IO39) = 0% (正向半区)
                                            <0 左反推: 方向线 REV1 (IO39) = 100% (反转半区), 油门线 ESC1 (IO41) 给 |v|%
right = clamp(speed - yaw, -100, +100)  →  同上, 右路 = 油门线 ESC2 (IO42) + 方向线 REV2 (IO40)
bucket_speed (-100~+100)                →  L298N 滚筒收放电机 (IO38/48/47)
                                            >0 收, <0 放, 0 停
```

> 反推是**独立的方向线** (好盈 SkyWalker V2 "反推刹车"接线, 见 §4.1): 方向线只发 **0% / 100%** 两个方向位,
> 速度全由**油门线**给 —— 触发反转时电调先刹停, 再按油门量反转加速。
> ⚠️ **反推当前暂时关闭 (main.c `REV_ESC_ENABLE=0`)**: 负值一律按 **0 (停)** 处理 (上面两条 `<0` 的路径不生效),
> 方向线只保持在 1100µs (正向半区) ⇒ **只能在前进方向做差速, 不能反向 / 不能靠一侧反转原地转向**。
> 恢复方法: `REV_ESC_ENABLE` 置 `1` (且电调参数"刹车类型"要设为**反推刹车**)。
> 转向示例: 杆推到最右 (speed=0, yaw=+50) ⇒ 正常是 左 +50 推进 / 右 −50 反推 (原地右转);
> **反推关闭时变成 左 +50 推进 / 右 0** ⇒ 以左桨为圆心的右转弧线 (不是原地转)。

远端收到 8B 子帧后, 做**完全相同**的混合 (主控 8081 转发**暂未启用**):

```
remote_esc1 = clamp(speed + yaw, -100, +100)
remote_esc2 = clamp(speed - yaw, -100, +100)
remote_light = frame[5]  (0=关, 1=开)
bucket_speed = (int8_t)frame[6]  (-100~+100, 0=停)  → 远端 L298N 铲斗电机
```

控制律基于本地 MPU:
- 主机 (笔记本) 接收 20Hz MPU 原始数据 (本+远端 MPU)
- 主机做姿态解算 + 控制律 (例如保持水平、循迹)
- 主机发 speed+yaw 高速率指令 (例如 50Hz)
- ESP32 主节点只做开环差速混合, 不做姿态闭环
- 这样 ESP32 算力极小, 主要算力留给笔记本

---

## 3. FreeRTOS 任务清单 (当前实况)

| 任务 | 文件 | 优先级 | 栈大小 | 周期 | 说明 |
|---|---|---|---|---|---|
| tcp_server_task | main.c | 5 | 8192 | 轮询 (≥2ms) | **当前运行** — socket 0 监听 8080, 收上位机 16B 控制帧 → 回调差速混合驱动电调/滚筒; 内含 **500ms 失联保护** (超时全停); 循环里还按帧率**回传状态** —— MPU `type=0x01` 20Hz (v8.1) + GPS `type=0x04/0x05` 各 2Hz (v9.1, §7.10.8) |
| attitude_task | main.c | 5 | 4096 | 10ms (100Hz) | 读云台 MPU6050 + Madgwick 姿态解算; 发布 `s_att_*` 并 `gimbal_feed_attitude()` 喂闭环 (**云台 MPU 关停时不创建**) |
| gimbal_task | gimbal.c | 4 | 3072 | 10ms (100Hz) | 云台运动规划 + 姿态闭环 PID 下发 (由 `gimbal_init()` 创建; **PCA9685 关停时不创建**) |
| nav_task | nav.c | 4 | 3072 | UART 阻塞读 (50ms 超时) | 惯导模块 UART2 接收/解析 (维特 WIT `0x55` 协议主动上报), 由 `nav_init()` **内部创建**; **`NAV_ENABLE=1` 时创建 (当前启用)** (见 §7.10) |
| nav_gps | main.c | 4 | 4096 | 1000ms | 惯导模块 GPS 1Hz 日志 (默认静默, `gps 1` 打开; **仅 `NAV_ENABLE=1` 时创建**) |
| nav_att | main.c | 4 | 4096 | 100ms (10Hz) | 惯导模块姿态日志 (**默认关**, `hull 1` 开 / `hull 0` 关; 无参 `hull` 读一帧; **仅 `NAV_ENABLE=1` 时创建**) |
| servo_ch0_test_task | main.c | 4 | 3072 | 常驻 | ch0 (360°) 档位扫描 (默认关, `t0 1` 开启) |
| servo_ch1_test_task | main.c | 4 | 3072 | 常驻 | ch1 (180°) 角度扫描 (默认关, `t1 1` 开启) |
| gimbal_stab_test_task | main.c | 4 | 4096 | 500ms 打印 | 云台闭环自稳测试 (`TEST_MODE==4`), 开机自动开 ch1 俯仰闭环 —— **当前 `TEST_MODE=0`, 不创建** (云台不靠 MPU) |
| console_repl_task | main.c | 5 | 4096 | — | 串口标定控制台 (自建任务, 支持"裸数字"输入) |

> **`TEST_MODE` (main.c) 决定创建哪个"驱动舵机"的测试任务**: 1=`gimbal_test_task` / 2=`servo_ch1_mpu_task` / 3=`servo_ch0_mpu_task` / 4=`gimbal_stab_test_task`; 当前 **`TEST_MODE == 0`** = **不创建任何测试任务** (云台保持开机回中的中位、不自动动作, 也就是"不靠 MPU"); `ch0_test` / `ch1_test` 两个扫描任务仍常驻但**默认空转**, 由控制台 `t0 1` / `t1 1` 启动。
> 同一时刻只允许一个"驱动舵机"的任务在跑, 否则两条指令流会互相打架 (见 main.c 注释)。
>
> ❌ **已停用 (当前不创建)**: `status_report_task` (4/2048) —— 见 §0。**主控 MPU 状态回传不再需要独立任务** (v8.1): 由 `attitude_task` 存 int16 快照 + `tcp_server_task` 循环里按 20Hz 发送 (原 `mpu_push_task` 的设想就是它)。
> ❌ v5.10.0 已删除 `status_led_task` (新板无板载 LED, `status_led` 组件整体删除)。

**所有做阻塞式 SPI/I2C 通讯的任务不得长期占用 Task WDT**; 历史网络方案中阻塞式 SPI 任务需 `esp_task_wdt_delete(NULL)` 退订, 改在循环中显式 `esp_task_wdt_reset()` 喂狗。

---

## 4. 硬件引脚定义 (v5.11.1 复核 / v6.0 惯导替换后更新)

> ✅ 下表引脚在 v5.11.1 已逐个复核, 与 `motor.c` / `imu.c` / `servo.c` / `nav.c` / `main.c` / `wiznet_manager.c` 中的宏一致 (v7.0 起惯导从 16/18 改到 1/2; **v9.0 起 4 路电调左右对调: 42=右**)。
> 注: 4 路电调由 `motor_init()` **实际驱动** (v5.12 起启用, 上电强制最低油门停机); **L298N 也实际驱动** —— 上电强制停机 (§0 / §7.9)。

> 依据: [`ESP32-S3-ETH-pinout.md`](ESP32-S3-ETH-pinout.md)。板卡固定占用: W5500 = GPIO9~14、MicroSD = GPIO4~7、
> USB = GPIO19/20、UART0 = GPIO43/44 (本工程 console 已改走原生 USB, 见下)、Octal PSRAM = **GPIO33~37 (不可用)**。
> 可用空闲脚共 17 个: `1, 2, 3, 15, 16, 17, 18, 21, 38, 39, 40, 41, 42, 45, 46, 47, 48`。
> 本方案按"同功能引脚相邻"排布, 并**避开 strapping 脚 0/45/46** (**v6.0 起连 GPIO3 也不再占用** —— 原 GPS PPS 已随 GPS 组件一并删除)。

| 功能组 | 名称 | 引脚 | 文件 | 说明 | 状态 |
|---|---|---|---|---|---|
| **推进电调+反推方向线 (右排 4 连)** | ESC1_PWM_GPIO | GPIO41 | motor.c | **左推进油门线** PWM (50Hz) | ✅ |
| | ESC2_PWM_GPIO | GPIO42 | motor.c | **右推进油门线** PWM (50Hz) | ✅ |
| | REV1_PWM_GPIO | GPIO39 | motor.c | **左反推方向线** PWM (50Hz) — ⏸ 反推暂时关闭, 只保持 1100µs 正向半区 | ⏸ |
| | REV2_PWM_GPIO | GPIO40 | motor.c | **右反推方向线** PWM (50Hz) — ⏸ 同上 | ⏸ |
| **L298N (右排 48/47 + 38)** | L298N_IN1_GPIO | GPIO48 | motor.c | L298N 方向 1 | ✅ |
| | L298N_IN2_GPIO | GPIO47 | motor.c | L298N 方向 2 | ✅ |
| | ENA_PWM_GPIO | GPIO38 | motor.c | L298N 调速 PWM (5kHz) | ✅ |
| **I2C1 (左排相邻)** | SERVO_I2C1_SDA | **GPIO16** | servo.c | **PCA9685 (0x40) + 云台 MPU6050 (0x68) SDA** (v5.11.2 起共线; **v8.0 从 GPIO21 挪来** —— 当时误判 21 是板载 WS2812 数据脚, 真正原因是"21 不再是必要的", 见下面两行的更正) | ✅ |
| | SERVO_I2C1_SCL | GPIO17 | servo.c | 同上 SCL | ✅ |
| **GPIO21**<br>⬜ 空闲 | — | GPIO21 | — | **v8.1 起不再占用**: v8.0 曾把它当"候选灯脚"钉低, 但板载灯实测在 **GPIO26** (见下行) ⇒ GPIO21 恢复为**普通空闲脚** (I2C1 SDA 已挪到 GPIO16) | ⬜ 空闲 |
| **GPIO26**<br>🚫 板内网络 (未引出排针) | — (板载 WS2812 数据脚) | GPIO26 | main.c | **板载 WS2812 RGB LED 的数据脚** (用户按原理图/实测确认, 2026-09; 不在 40Pin 排针上)。启动第 0 步 (`BOARD_RGB_LED_GPIO`) 拉为**输出低电平** (不给任何边沿 ⇒ 保持上电默认熄灭; ⚠️ **需断电重上一次**才真灭)。当状态灯需 RMT 驱动 | 🚫 禁用 |
| ~~I2C0 云台~~ | ~~IMU_I2C0_SDA/SCL~~ | ~~GPIO16 / GPIO18~~ | — | ⚠️ **已废除** —— 云台 MPU 挪到 I2C1 后腾出, 现改作**惯导模块的 UART2** (见下) | ⛔ |
| **惯导模块 UART2 (1/2)**<br>✅ 当前 `NAV_ENABLE=1` 已启用 | UART2 TX / RX<br>(`nav_config_t.tx_gpio` / `rx_gpio`) | GPIO2 / GPIO1 | nav.c | ESP32 RX=IO1 ← **模块 TX**; ESP32 TX=IO2 → **模块 RX**, **9600** 8N1 (维特 WIT `0x55` 主动上报, 见 §7.10。⚠️ nav 组件未单独定义引脚宏, 值在 `nav_get_default_config()`) | ✅ |
| ~~GPS UART1 (左排 3 连)~~<br>⛔ **v6.0 已删除** | ~~GPS_TX_GPIO~~ | ~~GPIO2~~ | — | ⛔ 旧 `components/gps` (NMEA-0183) **组件与其驱动/命令已删除**; **GPIO2 于 v7.0 归惯导模块 (TX=2)** | ✅ 复用 |
| | ~~GPS_RX_GPIO~~ | ~~GPIO1~~ | — | ⛔ 同上; **GPIO1 于 v7.0 归惯导模块 (RX=1)** | ✅ 复用 |
| | ~~GPS_PPS_GPIO~~ | ~~GPIO3~~ | — | ⛔ 同上 (原 PPS 秒脉冲输入; GPIO3 是 strapping 脚, 现已**空闲**、不再挂中断) | ⛔ 空闲 |
| **以太网 (板载固定)** | ETH_CLK | GPIO13 | wiznet_manager.c | W5500 SPI 时钟 | ✅ (固定) |
| | ETH_MOSI | GPIO11 | wiznet_manager.c | W5500 MOSI | ✅ (固定) |
| | ETH_MISO | GPIO12 | wiznet_manager.c | W5500 MISO | ✅ (固定) |
| | ETH_CS | GPIO14 | wiznet_manager.c | W5500 片选 | ✅ (固定) |
| | ETH_RST | GPIO9 | wiznet_manager.c | W5500 复位 (独立 GPIO, 可硬复位) | ✅ (固定) |
| | ETH_INT | GPIO10 | wiznet_manager.c | W5500 中断输出 (下降沿) | ✅ (固定) |

> ⚠️ **引脚注意事项**:
>   - **GPIO3 是 strapping 脚** (JTAG 源选择), **v6.0 起已不再使用** (原先只作 GPS PPS **输入**, 该功能随 GPS 组件一并删除); **GPIO3 仍是空闲脚** (表中 ~~GPS UART1~~ 的 **GPIO1/GPIO2 不再空闲** —— **v7.0 已归惯导模块**, RX=1 / TX=2) —— 将来若要复用 GPIO3, 注意上电瞬间不要对其外部强拉。
>   - **GPIO33~GPIO37 不可用** (ESP32-S3R8 Octal PSRAM 占用): 丝印上有脚, 但不可分配给外设。
>   - **GPIO4~7 已固定给板载 MicroSD**, 本方案未使用 SD, 但也不占用这 4 个脚 (原 I2C 曾用 4/5/6/7, 已全部迁走)。
>   - **GPIO0 / GPIO45 / GPIO46 未使用** (strapping)。
>   - **console 走板载原生 USB (USB-Serial/JTAG)**, 115200 8N1 (`sdkconfig.defaults`: `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` + `CONFIG_ESP_CONSOLE_SECONDARY_NONE=y`) —— 插板子**原生 USB** 那个 Type-C 口即可日志 + `gimbal>` 交互, 烧录同口。UART0 (GPIO43/44) 现在**不再输出日志** (需要时可改回 `CONFIG_ESP_CONSOLE_UART_DEFAULT=y`)。
>   - W5500 在板卡上已固定连线, 软件侧 `spi_rst_gpio = 9` (之前外置模块方案是接芯片 RST、软件写 -1)。

### 4.1 电机 LEDC 通道分配 (v5.11.1 修复)

> 4 路电调共用 **Timer0 (50Hz)**, 分占 Channel0~3; L298N ENA 用 **Timer1 (5kHz)**, 占 **Channel4**。
> ⚠️ **v5.11.1 修复**: 原 L298N ENA 误用 `LEDC_CHANNEL_0` (与 ESC1 同通道)。`motor_init()` 先配 ESC1、后配 L298N, 第二次 `ledc_channel_config()` 会把 Channel0 的 timer 重绑到 Timer1 → ESC1 (GPIO41) 失去 50Hz PWM, 电调不动作。此前因未调用 `motor_init()` 未暴露; 恢复电机前必须保持 DC 用 Channel4。

| 输出 | 引脚 | LEDC Timer | LEDC Channel | 频率 | 分辨率 |
|---|---|---|---|---|---|
| ESC1 (左推进 油门线) | GPIO41 | Timer0 | Channel0 | 50 Hz | 13 bit |
| ESC2 (右推进 油门线) | GPIO42 | Timer0 | Channel1 | 50 Hz | 13 bit |
| REV1 (左反推 方向线) | GPIO39 | Timer0 | Channel2 | 50 Hz | 13 bit |
| REV2 (右反推 方向线) | GPIO40 | Timer0 | Channel3 | 50 Hz | 13 bit |
| L298N ENA | GPIO38 | Timer1 | **Channel4** | 5 kHz | 13 bit |

> 依据 `motor_get_default_config()`; ESP32-S3 LEDC 共 8 通道 (0~7), 4 路电调已占满 0~3。

---

## 5. W5500 以太网参数 (v5.12: 已启用, `W5500_ENABLE=1`)

> ✅ **当前启用**: `main.c` 的 `#define W5500_ENABLE 1` —— `wiznet_manager_init()` 与 "等待 link up + 打印 IP/掩码/网关" 均生效, 上电即配置静态 IP。
> ⚠️ 网线未插则 **30s 超时后告警继续启动** (不阻塞其他功能), 插上后 `net` 命令可查链路状态。
> ⚠️ **8081 (远端) 仍未启用**: `tcp_server_task` 只监听 8080, 不开 8081, `status_report_task` 不创建 (§6); 但**主控侧 MPU 回传已启用** (v8.1, 20Hz, `type=0x01`), 且远端帧一旦进来会被 `tcp_server_forward_mpu_to_host()` **原样透传**给上位机。
> 下表除"TCP 收发范围"一行为当前实况外, 其余为已生效的链路层参数。

| 参数 | 值 | 文件 | 说明 |
|---|---|---|---|
| SPI 时钟 | **20 MHz** | wiznet_manager.c | SPI_DMA_DISABLED (polling) 模式, 无堆碎片 |
| DMA 模式 | **DISABLED** | wiznet_manager.c | ESP-IDF v5.5 DMA 模式反复 malloc 会触发 Task WDT |
| RST GPIO | **GPIO9** | wiznet_manager.c | 板载 W5500 的 ETH_RST, 独立 GPIO, 初始化时发 10ms 低电平复位脉冲 + 50ms 自举等待; 另外仍用 `wizchip_sw_reset()` |
| INT GPIO | **GPIO10** | wiznet_manager.c | 板载 W5500 的 ETH_INT, 下降沿触发, ISR 置位 + <1ms 去抖 |
| INT 去抖 | < 1ms | wiznet_spi.c | 防止 INT 抖动反复触发 ISR (`s_last_isr_us < 1000` 即 <1000µs) |
| PHY 模式 | **100M FULL** | wiznet_manager.c | **软件强制** 100BASE-TX 全双工 (PHY 以太网速率, 与 SPI 时钟无关) |
| SPI 互斥锁超时 | 100ms | wiznet_spi.c | 防止死锁 |
| **TCP 收发范围** | **仅 8080 收控制帧** | main.c | `tcp_server_task` 只开 socket 0 (8080) 收上位机 16B 控制帧, **不回传任何状态**; 8081 / MPU 回传未启用 |
| 默认 IP | 192.168.29.10 | main.c | 静态。注: `wiznet_manager_get_default_config()` **本身已返回该值**(不再是 192.168.1.100), main.c 再显式赋一遍以自文档化; 远端节点 connect `.10:8081` |
| 子网掩码 | 255.255.255.0 | wiznet_manager.c | /24 |
| 默认网关 | 192.168.29.1 | main.c | 路由器; 同上, 默认配置已为该值, main.c 显式再赋一遍 |
| DNS | 8.8.8.8 | wiznet_manager.c | 公共 DNS |
| 等待 link up 超时 | 30s | main.c | 启动时阻塞等待; 超时只告警, 不中断启动 |
| socket 数量 | 8 | wiznet_manager.c | W5500 8 个 socket |
| socket 缓冲 | 2KB/2KB (TX/RX) | wiznet_manager.c | 8 sockets 各 2KB TX + 各 2KB RX, TX 共 16KB + RX 共 16KB, 共用 W5500 内部 32KB SRAM |

> ✅ **v5.2 修复 (历史)**: main.c 显式覆盖 `wiznet_manager_get_default_config()` 的 IP 为 `192.168.29.10`、网关为 `192.168.29.1`, 与远端节点 `.11` 同 /24 网段, 远端 TCP Client 可正常连接 (v5.12 起 8080 上位机链路已通; 8081 远端待恢复)。

---

## 6. TCP 服务器端口分配 (v5.12: 仅 8080 启用)

| 端口 | socket | 角色 | 连接方 | 协议 | 状态 |
|---|---|---|---|---|---|
| **8080** | 0 | TCP Server | 上位机 (笔记本, `tools/tcp_console.py`) | 16B 控制帧 (收) + **状态回传 (发)**: MPU `type=0x01` 20Hz / GPS `type=0x04`·`0x05` 各 2Hz | ✅ **已启用** (`HOST_SOCK=0`, `SF_TCP_NODELAY`, 1024B 接收缓冲) |
| **8081** | 1 | TCP Server | 远端 ESP32-S3 | 8B 子帧 (发) + 16B MPU 帧 (双向, 20Hz) | ❌ **未启用** (设计保留) |

> 8080 侧配套:
> - **失联保护**: 收到过控制帧后, 超过 `TCP_LINK_TIMEOUT_MS` = **500ms** (上位机 20Hz ⇒ 连续丢 10 帧) 无新帧 ⇒ 推进电调(2) + 反推方向线(2) + L298N 滚筒**全部归零**, 并解除武装 (等下一帧重新计时), 日志 `上位机失联 >500 ms ⇒ 已全部停机`。
> - **断线重建**: `SOCK_CLOSE_WAIT` / `SOCK_CLOSED` ⇒ `wiz_close()` → 重新 `wiz_socket()` + `wiz_listen()`, 上位机重连即可; 新连接 `control_reset()` 清残帧。
> - **NODELAY**: 关 Nagle, 保证 20Hz 小帧不被合并延迟。
> - **状态查询**: 控制台 **`net`** 打印端口 / 网线链路 / 本机 IP / 上位机连接 / 累计控制帧数 / 距上一条控制帧的毫秒数。

---

## 7. 组件清单 (components/)

| 组件 | 路径 | 功能 | API 摘要 |
|---|---|---|---|
| imu | components/imu/ | **只剩云台 MPU6050** (I2C1 16/17, 与 PCA9685 共线, 0x68) —— v6.0 已删除旧的船体"亚博 10 轴 IMU"UART(`7E 23`)/I2C(WIT 0x50) 全部代码 | imu_init_role, imu_read_role, imu_role_ready, imu_scan_role (**详见 §7.5**) |
| servo | components/servo/ | PCA9685 驱动 (I2C1 16/17) + **标定体系** (双角度刻度/行程限位/NVS) | servo_init, servo_set_angle, servo_set_pulse_us, servo_set_phys_angle, servo_set_cal/save_cal/load_cal (**详见 §7.6**) |
| gimbal | components/gimbal/ | 云台**运动规划** (速度/加速度受限轨迹) + **姿态闭环 PID 自稳** | gimbal_init, gimbal_move_to, gimbal_set_limit, gimbal_feed_attitude, gimbal_enable_stabilize, gimbal_set_target_phys, gimbal_set_pid, gimbal_set_fb_sign (**详见 §7.7**) |
| **nav** | components/nav/ | **🆕 v6.0 惯导模块** (亚博 GPS + 10 轴 IMU **一体**): 维特 WIT `0x55` 协议**主动上报**, 单条 **UART2** (1/2 @ **9600**), 11B 定长帧解析 (姿态 + GPS 一套快照), 启动按需写 RSW/RRATE (波特率自扫描) | nav_get_default_config, nav_init, nav_is_ready, nav_read_imu, nav_read_gps, nav_get_stats (**详见 §7.10**) |
| wiznet / wiznet_spi | components/wiznet/ | W5500 驱动 (板载, SPI2: 13/11/12/14, RST=9, INT=10) — ⏸ **当前 `W5500_ENABLE=0` 未启用** (组件代码保留; 置 `1` 时: 初始化 + 拿 IP + TCP Server 8080 收控制帧, §0/§5/§6) | wiznet_manager_init, is_link_up, get_ip_info; wiznet_spi_init, wiznet_spi_check_int |
| motor | components/motor/ | **电调: 推进油门线 41/42 + 反推方向线 39/40 (⏸ 反推暂时关闭, `REV_ESC_ENABLE=0`) + L298N 滚筒 (38/48/47) — ⏸ 当前 `MOTOR_ENABLE=0` 未启用** (组件代码保留, 置 `1` 时启用, §0); LEDC 通道分配见 §4.1 | motor_init, motor_set_esc_throttle, motor_set_dc_speed, motor_emergency_stop |
| control | components/control/ | 16B 帧协议 (v3.0 沉浮: cmd=0x11 + type=0x03) — **已启用: 只解析协议, 通过回调交出 `ctrl_command_t` (§0/§2.7)** | control_process, control_reset, **control_set_local_callback**, control_set_forward_callback, control_build_*_frame |
| ~~status_led~~ | ~~components/status_led/~~ | **v5.10.0 已删除** (新板无板载 LED) | — |
| ~~rdk_uart~~ | ~~components/rdk_uart/~~ | **v5.8.0 已删除** (RDK X5 UART 方案作废) | — |

---

## 7.5 IMU 组件 (imu — 只剩云台 MPU6050)

> ⚠️ **v6.0 精简**: 本组件现在**只驱动云台 MPU6050 一颗**。原先"船体亚博 10 轴 IMU"的全部代码**已删除** ——
> 包括 `7E 23` 请求式 **UART 后端** (功能码 `0x80`/`0x04`/`0x26`/`0x32`)、旧 **I2C(WIT 0x50)** 分支, 以及 `imu.h` 里的
> `IMU_HULL_IFACE_UART` / `IMU_GIMBAL_SHARED_I2C1` 两个开关; `imu_role_t` 现在**只有 `IMU_ROLE_GIMBAL`** (`IMU_ROLE_HULL` 已删除)。
> **船体姿态与 GPS 改由 `components/nav` (惯导模块) 提供, 见 §7.10。**
> 历史实现 (UART `7E 23` 协议、角速度/欧拉角换算修复) 保留在 §10 变更记录中。

### 7.5.1 硬件连接 (v5.11.2 接线调整后, v6.0 精简)

| 名称 | 引脚 | 说明 |
|---|---|---|
| IMU_I2C1_SDA / SCL | GPIO16 / GPIO17 | **PCA9685 (0x40) + 云台 MPU6050 (0x68) 共用一条 I2C1**, 总线由 servo 组件安装, imu 只借用 |
| ~~HULL_UART_RX / TX~~ | ~~GPIO16 / GPIO18~~ | ⛔ **v6.0 已撤销** (船体 IMU 的 UART 后端已删除); 这两个脚 v6.0 起曾归惯导模块, **v7.0 又改到 GPIO1/GPIO2** (见 §7.10) |
| ~~IMU_I2C0_SDA / SCL~~ | ~~GPIO16 / GPIO18~~ | ⛔ v5.11.2 废除: 云台 MPU 挪到 I2C1 后腾出这两个脚 |

- **I2C1**: `I2C_NUM_1`, 400kHz, **由 servo 组件安装** → 所以 `imu_init_role(IMU_ROLE_GIMBAL, ...)` **必须在 `servo_init()` 之后** (`main.c` 已把云台 MPU 放在 4c 段)。
- 本组件**不再安装任何 UART** (UART2 现在是 nav 组件的)。

> ✅ **v5.11.2 保留**: PCA9685 探测失败时 `servo_init()` **不再 `i2c_driver_delete()` 删掉 I2C1** —— 总线保留给云台 MPU, 插好后 `scan` 能在线重探, 不必重启。
> 此时 servo 自身功能关闭 (`servo_is_ready() == false`, servo API 一律返回 `ESP_ERR_INVALID_STATE`, 不会往不存在的芯片写数据)。

> ⏸ **云台 (GIMBAL) 角色当前暂时关停**: `main.c` 的 `#define MPU6050_ENABLE 0` ⇒ **不调用** `imu_init_role(IMU_ROLE_GIMBAL)` —— 不初始化、不做地址自检、不读有效性, **不会**再出现引脚自检、`地址 0x68 无应答`、`未找到 MPU6050` 这一串 ERROR; `imu` 命令显示 `gimbal  未就绪`; `attitude_task`(100Hz 姿态解算) 不创建 ⇒ ① 上位机「主控 MPU」面板恒 `--`; ② 云台闭环 `ge`/`gt`/`gp` **没有反馈源**。
> 云台当前的操作方式是**手动摆位**: 控制台 `p`/`g`/`n`/`z` 或**上位机「云台手动」两个拖动条 (cmd=0x12, v9.2)** —— 见 §2.2 / §7.9。
> 想恢复姿态功能: 把 `MPU6050_ENABLE` 置回 `1` 并把云台 MPU6050 接好 (同一条 I2C1, 0x68) 即可。

> 🔎 **地址排查工具**: `scan` 对 VERSION(0x2E) **连读两次比对** —— 真器件版本号是常量, 两次不一致会标 `**不一致 => 疑似伪应答**` 且不再被采用 (上拉不足 / 有未供电芯片拖线 / 串扰都会造成伪应答)。
> （v5.11.2 的 `hull addr <0xXX>` 指定地址重探命令**已随 UART/I2C 多后端一起删除**, 见 §7.9。）

### 7.5.2 默认配置

| 参数 | 默认值 | 说明 |
|---|---|---|
| I2C 地址 | 自动探测 0x68 / 0x69 | AD0=GND → 0x68, AD0=VCC → 0x69 |
| 加速度量程 | ±2g | 16384 LSB/g |
| 陀螺仪量程 | ±250°/s | 131 LSB/(°/s) |
| 采样率 | 125Hz | SMPLRT_DIV = 0x07 (1kHz / 8) |
| 低通滤波 | ~5Hz | CONFIG = 0x06 |

> 📌 船体姿态/惯导参数 (UART2、9600、RSW/RRATE 等) **已不属于本组件**, 见 §7.10。

### 7.5.3 API

```c
imu_config_t imu_get_default_config(void);   // SDA=16, SCL=17, 400kHz, 地址 0x68
esp_err_t    imu_init_role(imu_role_t role, const imu_config_t *cfg);
                                             // role 当前只有 IMU_ROLE_GIMBAL: 借用 servo 的 I2C1, 须后于 servo_init
esp_err_t    imu_read_role(imu_role_t role, imu_data_t *out);  // 阻塞读 (~1ms)
bool         imu_role_ready(imu_role_t role);
esp_err_t    imu_scan_role(imu_role_t role);                   // 扫 I2C1 + 识别型号 (连同线的 PCA9685 一起报出)
esp_err_t    imu_deinit_role(imu_role_t role);                 // 清器件记录, **不释放 I2C1** (总线由 servo 安装)
void         imu_deinit(void);
```

### 7.5.4 数据格式

`imu_data_t` 提供物理单位:
- `ax/ay/az`: g
- `gx/gy/gz`: °/s
- `temperature`: °C
- `timestamp_us`: 读取时刻 (esp_timer_get_time)
- ⚠️ **云台 MPU6050 只出原始 6 轴, `roll/pitch/yaw` 三项恒为 0** (字段保留) —— 姿态由上层 Madgwick 解算 (§7.5.6), 不在本组件里
- **量纲自检**: 静止时 `gx/gy/gz` 应在 **±2dps** 内、`a` 的模长 ≈ 1g; 手转模块为**几十~几百 dps**。(惯导模块的姿态读数自检见 §7.10)

### 7.5.5 协议映射 (仅在网络恢复后有效)

`main.c` 把 `imu_data_t` 的浮点值转回 int16 LSB, 以便复用 16B MPU 帧 (v8.1 起启用; 实现见下方 ⚠️ 注):
- ax/ay/az: `(int16)(value * 16384)` 对应 ±2g
- gx/gy/gz: `(int16)(value * 131)` 对应 ±250°/s

> ✅ **v8.1 已启用回传**: 上位机 `tools/tcp_console.py` 的「主控 MPU」面板按 **type=0x01** 收 (20Hz)。
> 实现方式: **不新增任务** —— `attitude_task`(100Hz) 读 MPU 时顺手把原始值换算成 int16 存进 `s_mpu_raw[6]` 快照 (换算公式就是上面两条),
> `tcp_server_task` 的循环里按 `MPU_PUSH_PERIOD_US`(50ms) 取快照、用 `control_build_mpu_frame()` 组帧、`wiz_send()` 发给 8080 上的上位机。
> ⚠️ 之所以**不在 TCP 任务里直接读 I2C**, 是为了避免与 100Hz 的 `attitude_task` 抢同一条 I2C1。
> ⚠️ `MPU6050_ENABLE=0` 或 MPU 未就绪时快照一直无效 ⇒ **不发帧**, 上位机面板显示 `--` (符合预期)。

### 7.5.6 Madgwick 姿态解算 (attitude_task, v5.11.0 新增)

`main.c` 的 `attitude_task` 以 **100Hz** 读云台 MPU6050, 用 **Madgwick AHRS** 融合加速度计(重力参考)+陀螺仪(积分), 输出四元数并转 yaw/pitch/roll:

| 参数 | 值 | 说明 |
|---|---|---|
| 解算频率 | **100Hz** | `AHRS_UPDATE_HZ`, Madgwick 需 >50Hz |
| 滤波增益 | β = **0.1** | `AHRS_BETA`, 越大越信加速度 |
| 打印周期 | 2Hz (默认关) | `AHRS_PRINT_DIV=50`, 用 `att 1` 打开 |
| 陀螺零偏标定 | 100 采样 (~1s) | 开机先等 `AHRS_CAL_DELAY_MS=3000` 让舵机回中停稳, 再静止采样求均值 |
| 数据发布 | `s_att_yaw/pitch/roll_deg` 等 | 供其它任务读取, 避免多任务并发访问 I2C |

- 无磁力计, **yaw 会缓慢漂移** (6 轴 IMU 固有特性, 无法绝对定北)。
- 解算结果通过 `gimbal_feed_attitude(pitch, yaw)` 喂给云台闭环做反馈 (见 §7.7)。
- 会驱动舵机的测试任务统一等到 `AHRS_CAL_READY_MS` (≈3300ms) 之后才动作, 避免污染零偏标定。

### 7.5.7 参考资料

- `RM-MPU-6000A.pdf` 寄存器映射与量程换算
- `PS-MPU-6000A.pdf` 产品规格
- `51-串口-mpu6050.c` 初始化时序参考

---

## 7.6 servo 组件 — 标定体系 (v5.11.0 新增)

### 7.6.0 PCA9685 总开关 (v5.11.2 新增, **当前 = 启用**)

> `servo_config_t` 新增 **`pca9685_enable`** (默认 `true`); `main.c` 用 `#define SERVO_ENABLE` 控制。
> **当前 `SERVO_ENABLE 1` = 启用** (v6.0 起恢复云台测试); 下表右列是"临时关停"时的行为, 保留作开关说明。
>
> | | 启用 (`SERVO_ENABLE 1`, **当前**) | 关停 (`SERVO_ENABLE 0`) |
> |---|---|---|
> | `servo_init()` 装 I2C1 | 装 | **照装** (云台 MPU6050 要用) |
> | 探测/读写 PCA9685 | 是 | **否** |
> | 返回值 / `servo_is_ready()` | `ESP_OK` / `true` | `ESP_ERR_NOT_SUPPORTED` / `false` |
> | servo API | 正常 | 一律 `ESP_ERR_INVALID_STATE` |
> | 谁还依赖它 | gimbal 规划 + 闭环自稳 | 无 (gimbal 不初始化, `stab_test`/`ch0_test`/`ch1_test` 任务启动即自退出) |
>
> ⚠️ **关停也必须调用 `servo_init()`** —— I2C1 总线由它安装, **云台 MPU6050 与 PCA9685 共用这条总线** (v5.11.2); 不调用会导致云台 MPU 也装不上 (§7.5)。
> 关停不等于“拔掉”: 若硬件在, 只是软件不碰它。

### 7.6.1 两套角度刻度 (重要)

| 刻度 | 字段 | 范围 (ch0/ch1) | 用途 |
|---|---|---|---|
| **标称角** | `range_deg` | 360° / 180° | 报文/控制台/上位机用, "看上去的角度", 可读性好 |
| **物理角** | `phys_range_deg` | 365° / 192° | 云台真实转角, 也是 MPU 直接测到的量 |

关系: **物理角 = 标称角 × (phys_range_deg / range_deg)**; 换算只发生在 servo 组件内部, 不要在其他地方再乘系数。

闭环控制律工作在**物理角**域 (与 MPU 读数 1:1, 不含修正系数); 报文与显示用**标称角**。

### 7.6.2 出厂默认标定 (按通道, 实测)

| 通道 | 用途 | min_us | max_us | 标称 range_deg | 物理 phys_range_deg | trim_us | 行程限位 (标称角) |
|---|---|---|---|---|---|---|---|
| **ch0** | 360° 位置舵机 (平面旋转) | 530 | 2730 | 360 | 365 | 0 | 不限 (0~360) |
| **ch1** | 180° 位置舵机 (俯仰) | 600 | 2700 | 180 | 192 | 0 | **30~150** |

- **中点脉宽** = (min_us + max_us)/2 + trim_us → ch0 = 1630µs (180°), ch1 = 1650µs (90°)。
- **角度→脉宽**: `pulse = min_us + cmd/range_deg × (max_us - min_us) + trim_us`。
- ch1 规格书标注 500~2500µs **与实际不符**, 以实测 600~2700µs 为准。
- **行程限位**与标定**互相独立**: 只把命令角钳位在窗口内, 不改变角度↔脉宽换算; 调整范围应改限位, 不要改端点。

### 7.6.3 API

```c
servo_config_t servo_get_default_config(void);   // I2C1 SDA=16, SCL=17, 400kHz, 0x40, 50Hz PWM
esp_err_t      servo_init(const servo_config_t *cfg);   // 安装 I2C1; pca9685_enable=true 时再初始化 PCA9685 (见 §7.6.0)
esp_err_t      servo_set_pulse_us(uint8_t ch, uint16_t us);
esp_err_t      servo_set_angle(uint8_t ch, float angle);       // 标称角
esp_err_t      servo_set_phys_angle(uint8_t ch, float phys);   // 物理角 (闭环用)
esp_err_t      servo_get_angle(uint8_t ch, float *angle);
esp_err_t      servo_get_pulse_us(uint8_t ch, uint16_t *us);
esp_err_t      servo_get_limit(uint8_t ch, float *min, float *max);
float          servo_cmd_to_phys_deg(uint8_t ch, float cmd_deg);
float          servo_phys_to_cmd_deg(uint8_t ch, float phys_deg);
esp_err_t      servo_set_cal / servo_get_cal / servo_reset_cal(uint8_t ch, servo_cal_t *cal);
esp_err_t      servo_center(uint8_t ch) / servo_center_all(void);
esp_err_t      servo_save_cal(void) / servo_load_cal(void);    // 标定存/取 NVS
bool           servo_is_ready(void);
```

---

## 7.7 gimbal 组件 — 运动规划 + 姿态闭环 PID 自稳 (v5.11.0 新增)

> ⚠️ **v5.11.2 当前未启用**: `PCA9685` 关停时 `servo_init()` 返回非 `ESP_OK`, `main.c` 的 `gimbal_init()` 不执行 →
> 本组件全部 API 返回 `ESP_ERR_INVALID_STATE`, `gimbal` 任务不存在; `[STAB]` / `[CH0 TEST]` / `[CH1 TEST]` 三个测试任务**启动即自退出**并打印一条 `PCA9685/servo 未就绪 ... 不启动` 的告警。
> 恢复: 把 `main.c` 的 `#define SERVO_ENABLE` 改回 `1` 即可 (无需改本组件)。

本模块分两层, **互斥**: 某通道开启闭环后, 规划器不再写它, 由闭环直接下发。

| 层 | 功能 | 频率 | 说明 |
|---|---|---|---|
| **执行层** | 速度/加速度受限的梯形轨迹 (开环定位) | 100Hz | 把阶跃指令变平缓, 抑制大惯量载荷的冲击/过冲 |
| **控制层** | 姿态闭环 PID (自稳/指向) | 100Hz | 用 MPU 反馈把云台锁在目标姿态 |

### 7.7.1 执行层 (运动规划)

- 制动距离约束: `v_allow = min(max_vel, sqrt(2 × max_acc × 剩余角度))`, 减速段自然生成。
- 默认限制: **90°/s、180°/s²** (`GIMBAL_DEF_VEL_DPS/ACC_DPS2`); 到达判定 0.05° / 0.5°/s。
- 目标可随时改, 轨迹自然重规划; 角度按该通道标定钳位。

### 7.7.2 控制层 (姿态闭环)

控制律 (增量式, 工作在**物理角**域):
```
e      = target_phys − meas_phys
integ += e·dt        (限幅 ±10)
Δcmd   = fb_sign × servo_phys_to_cmd_deg( kp·e + ki·integ + kd·de/dt )
cmd   += Δcmd        (单周期速率限 120°/s, 再钳到行程限位)
servo_set_angle(ch, cmd)
```

| 参数 | 默认值 | 说明 |
|---|---|---|
| PID (ch0/ch1) | kp=**0.8**, ki=**0.5**, kd=**0.0** | `gimbal_pid_t` |
| fb_sign (ch0/ch1) | **+1 / −1** | 目标物理角增大时标称角该往哪边走 |
| 输出速率上限 | 120°/s | `GIMBAL_STAB_MAX_RATE_DPS` |
| 积分限幅 | ±10 (°·s) | `GIMBAL_INTEG_LIMIT` |
| 反馈超时 | **200 ms** | `GIMBAL_FB_TIMEOUT_MS` (v5.11.2 新增, 见下) |

- **fb_sign 实测依据**: ch1 标称角增大时 MPU 的 pitch **减小** → −1; ch0 标称角增大时融合 yaw **增大** → +1。
- 开启闭环时目标默认锁定在**当前姿态**(不跳变), 积分清零。

#### 无反馈保护 (v5.11.2 新增, 重要)

> 闭环依赖 `gimbal_feed_attitude()` 持续喂入实测角 (100Hz)。若**云台 MPU6050 未就绪 / 陀螺零偏标定失败 / 读数中断**, 实测角会恒为 0 —— 那不是"真实水平"。
> 若照跑 PID: 误差恒 0, 输出停在 0°, 被行程限位钳到**下限 (ch1 = 30°)** 把俯仰顶死 (且日志只显示"稳态", 看不出异常)。
>
> **保护行为**: 超过 `GIMBAL_FB_TIMEOUT_MS` (200ms) 没收到喂入 ⇒ **停用 PID, 该轴送回标定中位** (`servo_center()` = 中位脉宽 `(min_us+max_us)/2 + trim`, ch1 = 90°), 积分清零, 并打一条明确告警:
> `ch1 无姿态反馈 (云台 MPU 未就绪 / 零偏标定失败 / 读数中断) => 停用闭环 PID, 回中位 90.0°`
> (若 PCA9685 也未就绪, 则提示 `... => 停用闭环 PID (PCA9685 也未就绪, 无法回中位)`)
> 反馈恢复后自动重新接管 (日志 `ch1 姿态反馈已恢复 => 重新启用闭环 PID`)。状态里 `gimbal_stab_state_t.nofb = true` 对外可见, 串口 `gs` 的 stab 列显示 **N/FB**, `[STAB]` 周期日志追加 `[无反馈: 已停 PID, 回中位]`。

### 7.7.3 API

```c
esp_err_t gimbal_init(void);                                  // servo_init 之后调用, 创建 gimbal_task
esp_err_t gimbal_set_limit(uint8_t ch, const gimbal_limit_t *limit);
esp_err_t gimbal_move_to(uint8_t ch, float target_deg);       // 执行层: 受限轨迹
esp_err_t gimbal_stop(uint8_t ch);
esp_err_t gimbal_get_state(uint8_t ch, gimbal_state_t *st);

void      gimbal_feed_attitude(float pitch_phys_deg, float yaw_phys_deg);  // 控制层: 喂反馈
esp_err_t gimbal_set_target_phys(uint8_t ch, float phys_deg);
esp_err_t gimbal_enable_stabilize(uint8_t ch, bool enable);
esp_err_t gimbal_get_stab_state(uint8_t ch, gimbal_stab_state_t *st);
esp_err_t gimbal_set_pid / gimbal_get_pid(uint8_t ch, gimbal_pid_t *pid);
esp_err_t gimbal_set_fb_sign / gimbal_get_fb_sign(uint8_t ch, float sign);
```

---

## 7.8 GPS 数据 (原 gps 组件 — v6.0 已删除, 数据改由惯导模块提供)

> ⛔ **v6.0 已删除** `components/gps` **整个组件**: NMEA-0183 解析、UART1 (TX=GPIO2 / RX=GPIO1 / PPS=GPIO3)、`gps_init` / `gps_get_data` / `gps_get_pps_count` / `gps_get_pps_last_us` / `gps_write` / `gps_set_baud` / `gps_set_raw_echo`, 以及控制台 `gps raw` / `gps baud` / `gps send` 三个子命令。**GPIO1/2 现归惯导模块, GPIO3 空闲** (§4)。
> **不再有**: NMEA 语句解析 (RMC/GGA/GSA/GSV/GST/VTG/ZDA/TXT)、**PPS 秒脉冲**计数、原始语句回显、运行时改 GPS 波特率、发 `$PCASxx` / `$PMTK` 配置语句。
>
> ✅ **GPS 数据现在来自惯导模块** —— 亚博 **GPS + 10 轴 IMU 一体**模块, **维特 WIT `0x55` 协议主动上报**, 单条 **UART2** (1/2) @ **9600**; GPS 由帧 `0x57` 经纬度 / `0x58` 海拔·航向·地速 / `0x5A` 卫星数·DOP / `0x50` 时间给出, **详见 §7.10**。
> 控制台改用: **`gps`** 打惯导模块的 GPS 快照 (`gps 0|1` 开关 1Hz 日志)、**`nav`** 看模块整体状态 (在线/波特率/RSW/RRATE/帧计数); 字段含义与"定位有效性"约定见 **§7.10.3**, 现场排查见 **§7.10.7**。

> 📚 **历史留档 (v5.11.0 ~ v5.12 的 `gps` 组件, 仅供参考)**:
> - 参数: **UART1** (TX=GPIO2 / RX=GPIO1 / PPS=GPIO3), **9600** 8N1; 解析 **RMC/GGA/GSA/GSV/GST/VTG/ZDA/TXT** (只看语句名后 3 字符, 兼容 `$GN`/`$GP`/`$BD`/`$GL` 前缀, 定位有效性以 RMC status 为准); 任务 `gps_task` (4/4096) 由 `gps_init()` 创建。
> - 实测硬件 **ATGM336H** (中科微 AT6558), 与本组件兼容 (供电 3.3~5V, TXD 必须 3.3V 电平); 曾解析 `$GPTXT` 天线状态 (OK/OPEN/SHORT)。
> - 诊断: `gps_data_t.rx_bytes` (累计原始字节) 配合 `收到字节 / 语句 / 错误` 三个计数区分"**没接线**"(`收到字节=0`, 查 TX-RX 是否交叉与共地) 与"**波特率不对**"(`收到字节>0` 但 `语句=0`); `语句>0` 而"未定位"则是没搜到星。
> - 未定位时 `latitude`/`longitude`/`speed_kmh`/`course_deg` **清零** (不保留旧值), `last_fix_us` 记录最近一次**有效定位**时刻 (0 = 从未成功)。
> - PPS: `gps_get_pps_count()` / `gps_get_pps_last_us()`; ⚠️ GPIO3 是 strapping 脚, 当时只作 PPS **输入**。
> - 以上实现的历史变更记录见 **§10** (v5.11.0 / v5.11.2 / v5.12 各行), 版本头"上一版 v5.12 / v5.11.2"亦保留原文。

> 📌 原 **§7.8.1 诊断判据表** (`收到字节` / `语句` / `错误`) 与 **§7.8.2 NMEA 字段表** (`quality` / `fix_type` / `sats_in_view` / `std_lat_m` / `ant_status` 等) 描述的都是**已删除的 NMEA 组件**, 内容已并入上面的"历史留档"; 惯导模块对应的字段与诊断见 **§7.10.3 / §7.10.7**。

---

## 7.9 串口标定控制台 (v5.11.0 新增)

基于 `esp_console` + `linenoise`, 走 **板载原生 USB (USB-Serial/JTAG) 115200**, 提示符 `gimbal>`。
> ⚠️ **v5.11.2 改**: 主 console 由 UART0(GPIO43/44) 改为 **USB-Serial/JTAG** —— 用板子**原生 USB 那个 Type-C 口**（设备管理器里是 `VID_303A` 的「USB 串行设备」）。原来插原生 USB 只能看日志、**敲字无反应**（次要 console 按 IDF 设计只输出不收输入，敲字会写超时）。现插原生 USB 口即可"日志 + 交互"。UART0 已不再输出日志。
刻意**不调用** `esp_console_start_repl()`, 改用自建 `console_repl_task`, 以支持"裸数字"快捷输入。

> **裸数字规则** (直接输整数, 可带 `+`/`-` 号): 随 `SERVO_ENABLE` 切换 ——
> **PCA9685 启用时 (当前 `SERVO_ENABLE 1`)** = 把 ch0 定死在该脉宽 `100~3000` µs (原行为);
> **PCA9685 关停期间** = **L298N 速度 `-100~100`** (正=正转, 负=反转, 0=停; 绝对值=占空比%), 回到电机测试。
> ⚠️ L298N 也可用 `m <speed>` 命令, 与裸数字等价 (`cmd_motor` 已注册)。

| 命令 | 用法 | 说明 |
|---|---|---|
| `p` | `p <ch> <us>` | 直接输出脉宽 |
| `g` | `g <ch> [deg]` | **按角度控制云台** (v6.1 新增): 走 `servo_set_angle()`, 按**标称角**下发, **受标定 + 软件行程限位约束** (ch1 出厂 30~150°, 越界会回显"被软件限位钳住"); **不带 `deg` = 只查询**当前角度/脉宽/可用窗口 (不动作)。想绕过限位测行程请用 `p` |
| `n` | `n <ch> <dus>` | 在当前脉宽上微调 |
| `z` | `z [ch]` | 回中 (不带参数=全部通道) |
| `t` | `t <ch> <trim_us>` | 设置中位微调并回中 |
| `sc` | `sc <ch> <min> <max> <trim> [range_deg] [phys_deg]` | 设置完整标定 |
| `sl` | `sl <ch> <min_deg> <max_deg>` | 设置软件行程限位 |
| `ge` | `ge <ch> <0\|1>` | 姿态闭环开关 |
| `gt` | `gt <ch> <phys_deg>` | 闭环目标物理角 |
| `gp` | `gp <ch> <kp> <ki> <kd>` | 闭环 PID 参数 |
| `gs` | (无参) | 查看云台闭环状态 (stab 列: `ON` / `off` / **`N/FB` = 无姿态反馈, 已停 PID 并回中位**) |
| `st` | (无参) | 查看 ch0/ch1 标定与当前脉宽 |
| `ck` | `ck <ch>` | 把当前脉宽捕获为该通道中位 |
| `sv` | (无参) | 保存标定到 NVS |
| `rs` | `rs <ch>` | 复位通道标定 |
| `att` | `att <0\|1>` | 姿态日志开关 |
| `t0` | `t0 <0\|1>` | ch0 档位扫描测试开关 |
| `t1` | `t1 <0\|1>` | ch1 角度扫描测试开关 |
| `gps` | `gps` / `gps 0\|1` | **惯导模块的 GPS 快照** (数据来自 `components/nav` 的 `0x57`/`0x58`/`0x5A` 帧, **不再是 NMEA**, 见 §7.10): 无参打快照 (定位/卫星数/PDOP·HDOP·VDOP/模块时间/经纬度/速度/航向/海拔; 未定位时坐标显示 `—` 并给出"上次有效定位 N s 前"); `0\|1` 开关 1Hz 日志。⚠️ **v6.0 去掉了 `gps raw [0\|1]` / `gps baud <n>` / `gps send <语句>` 三个子命令** (随 NMEA 组件删除); 模块整体状态改用 **`nav`** |
| `nav` | (无参) | **🆕 v6.0 惯导模块状态**: 在线/无数据 / 实际波特率 / RSW 与 RRATE (读回值 vs 期望值, 是否已生效) / 累计字节与帧计数 (`帧OK`/`帧错`) / 各类型帧计数 / 姿态与 GPS 两套快照, 并自动给结论 —— 排查"没数据"的**第一站** (详见 §7.10.7) |
| `imu` | (无参) | **只读云台 MPU6050 一帧** (原始 6 轴 + 温度; 姿态由 Madgwick 解算)。⚠️ 船体姿态**不在此命令**, 用 `hull`; 模块状态用 `nav` |
| `scan` | (无参) | **只扫 I2C1 (GPIO16/17)**, 列出所有应答地址并尽量识别型号 (当前线上是 PCA9685 `0x40` + 云台 MPU6050 `0x68`), 并提示 **惯导模块走 UART2、不在 I2C 上**。⚠️ **v6.0 去掉了原来的 `0\|1` 参数** (两端已是同一条总线) |
| `hull` | `hull [0\|1]` | **惯导模块的姿态快照** (亚博 GPS+10 轴 IMU 一体, `0x53`/`0x51`/`0x52` 帧, 见 §7.10): 无参读一帧 (`rpy`/加速度/角速度/温度), 带 `0\|1` 开关 10Hz 日志 (**默认关**, `hull 1` 开 / `hull 0` 关)。⚠️ **v6.0 去掉了 `hull addr <0xXX>`** (该命令属已删除的 I2C 分支) |
| `m` | `m <-100~100>` | **L298N 滚筒收放电机调速** (正=正转(收), 负=反转(放), 0=停; `\|值\|<60` 按 60 处理) —— 与裸数字等价 (`MOTOR_ENABLE=1` 时有效, 见 §4.1) |
| `l` | `l <-100~100>` | **左路油门**: 正=左推进 ESC1 (IO41) 油门线给速度; ⚠️ **负值 = 反推, 当前已关闭** (`REV_ESC_ENABLE=0`, 按 0/停 处理并回显提示) |
| `r` | `r <-100~100>` | **右路油门**: 正=右推进 ESC2 (IO42); ⚠️ 负值同上按停处理 |
| `cal` | `cal <l\|r\|bl\|br>` | 好盈电调行程标定**第1步** (最大油门) —— `l/r`=推进 IO41/42, `bl/br`=反推方向线 IO39/40 |
| `cal2` | (无参) | 行程标定**第2步**: 听到双短鸣后立刻输入 (降到最低油门完成标定) |
| `pw` | `pw <l\|r\|bl\|br> <us>` | 直接输出指定脉宽 (500~2500µs), 用于**人工测端点** |
| `net` | (无参) | **网络 / TCP 状态**: 端口 / 网线链路(速率) / 本机 IP / 上位机连接 / 累计控制帧数 / 距上一条控制帧 ms (失联保护计时) |
| `help` | (无参) | 查看全部指令 |

> ⚠️ **舵机还有一条网络控制路径 (v9.2)**: 上位机 `tools/tcp_console.py` 的「**云台手动**」面板用 **cmd=0x12** 下发 ch0/ch1 的标称角 (拖动即发, 需勾选「启用下发」) —— 效果与 `g <ch> <deg>` **完全等价** (同一条 `servo_set_angle()`, 同样受"标定 + 软件限位"约束; ch1 越界会被钳到 30~150°)。
> 两条路谁后写谁生效 (互相覆盖), 所以调标定/测行程时建议先别勾「启用下发」。

---

## 7.10 惯导模块 (nav) — v6.0 新增

> 🆕 **v6.0**: 新增 `components/nav` 组件, 驱动**亚博惯导模块** —— 硬件其实是**两块板** (GPS 板 + 10 轴 IMU 板), 官方"**融合**"后由**一条 UART** 输出, 上位机侧当作**一个**模块: 同一帧流里既有**姿态**(模块内部融合、含磁力计补偿)也有 **GPS**。
> 它**取代**了本工程原先的两套驱动: 旧 `components/gps` (NMEA-0183 / UART1 2-1-3 / PPS) 与旧"亚博 10 轴 IMU" (`7E 23` 请求式 / UART2) —— **两者均已删除** (见 §7.8 / §7.5 / §10)。
> 总开关: `main.c` 的 `#define NAV_ENABLE` (**当前 `1` = 已启用** —— 装 UART2、按需配置模块输出、收数据; 见 §0 / §11)。
>
> 📚 **依据资料 (仓库内)**: [`10轴IMU模块通讯协议.pdf`](10轴IMU模块通讯协议.pdf) (寄存器表 + 帧格式, 权威)
> 与 `STM32-串口应用例程(IMU)/` (维特官方 SDK `wit_c_sdk.c/h` + `REG.h` + STM32 例程 `Main.c`, 波特率扫描/寄存器读写流程的依据)。

### 7.10.1 接线

| 名称 | 引脚 | 说明 |
|---|---|---|
| 模块 TX → ESP32 RX | **GPIO1** | `nav_config_t.rx_gpio` (默认 1) |
| ESP32 TX → 模块 RX | **GPIO2** | `nav_config_t.tx_gpio` (默认 2; 只用于发配置命令) |
| 波特率 | **9600 8N1** | 模块**出厂默认** (被上位机改过也能靠 §7.10.5 的扫描兜底) |
| 供电 | **3.3V** + **GND 共地** | ⚠️ 模块 TX 必须是 3.3V 电平, 5V 直连会打坏 S3 |

- UART 号: **UART2** (`UART_NUM_2`); 数据**不再走 NMEA/UART1** —— **v7.0 起惯导模块直接占用原 GPS 的 GPIO1(RX)/GPIO2(TX)**, UART1 本身不再使用 (PPS=GPIO3 现空闲)。
- ⚠️ **TX/RX 必须交叉**: "模块 TX → IO1 / ESP32 TX(IO2) → 模块 RX"; 接反或未共地时**一个字节都收不到**。

### 7.10.2 协议 (维特 WIT 标准 `0x55`, **主动上报**)

帧**定长 11 字节**, 模块**主动连续上报**, 驱动不需要发请求帧:

```
0x55 | TYPE | D1L D1H | D2L D2H | D3L D3H | D4L D4H | SUM
```

- `SUM` = **前 10 个字节累加, 取低 8 位**; 所有数据字段为**小端 (LE)**。

| TYPE | 名称 | 内容 | 换算 |
|---|---|---|---|
| `0x50` | 时间 | YY MM DD HH MN SS MSL MSH | 年只给 2 位 (0~99); 时区由模块 `TIMEZONE` 寄存器决定 (默认 UTC+8) |
| `0x51` | 加速度 + 温度 | Ax Ay Az + T | `int16 / 32768 × 16` → **g**; 温度 `int16 / 100` → **℃** |
| `0x52` | 角速度 + 电压 | Wx Wy Wz + V | `int16 / 32768 × 2000` → **°/s** |
| `0x53` | 角度 + 版本 | Roll Pitch Yaw + 版本号 | `int16 / 32768 × 180` → **°** |
| `0x57` | 经纬度 | Lon(uint32) Lat(uint32) | **去掉小数点的 NMEA `ddmm.mmmmm`**, 见下 |
| `0x58` | GPS | 海拔 + 航向 + 地速 | 海拔 `int16 / 10` → **m**; 航向 `int16 / 100` → **°**; 地速 `uint32 / 1000` → **km/h** |
| `0x5A` | GPS 精度 | 卫星数 (uint16) + PDOP / HDOP / VDOP | DOP 各 `int16 / 100` |
| `0x5F` | 读寄存器应答 | 连续 4 个寄存器的值 | 仅配置流程内部使用 |

**经纬度 (`0x57`) 的度分转换** —— 模块给的是**去掉小数点的 NMEA `ddmm.mmmmm`** (int32):

```
度 = v / 1e7
分 = (v % 1e7) / 1e5
十进制度 = 度 + 分 / 60        (v < 0 ⇒ 南纬 / 西经)
```

### 7.10.3 GPS 有效性约定 (重要)

> ⚠️ **模块不上报"定位是否有效"这个标志位**, 驱动以 **经纬度非 0** 作为"有效"判据 (`nav_gps_t.valid`);
> **判定无效时坐标清零** (不保留上一次的坐标, 避免被误读成实时位置), 并记录 `last_fix_us` = 最近一次收到经纬度帧的时刻 (0 = 从未收到)。
> 因此控制台 `gps` 在未定位时显示 `纬度=— 经度=— 速度=— 航向=—`, 并在括号里给出"上次有效定位 N s 前"或"从未定位成功"。

### 7.10.4 寄存器与输出配置 (启动时按需写入)

| 寄存器 | 地址 | 说明 | 出厂默认 | 本工程期望值 |
|---|---|---|---|---|
| SAVE | `0x00` | 写 `0x0000` = **保存参数** (会写模块 Flash); `0x00FF` = 重启; `0x0001` = 恢复出厂 | — | 仅在**确实改了配置**时才写 |
| RSW | `0x02` | 输出内容位图 | `0x001E` (**不含 GPS 帧**) | **`0x058F`** = TIME(`0x50`)\|ACC(`0x51`)\|GYRO(`0x52`)\|ANGLE(`0x53`)\|GPS(`0x57`)\|VELOCITY(`0x58`)\|GSA(`0x5A`) |
| RRATE | `0x03` | 输出速率档位 | `0x06` (10Hz) | **`0x05`** = 5Hz |
| BAUD | `0x04` | 串口波特率 | `0x02` (9600) | 不改 (保持 9600) |
| GPSBAUD | `0x26` | 模块内部 GPS 芯片波特率 | `0x02` (9600) | 不改 |
| TIMEZONE | `0x6B` | 时区 | `0x14` (UTC+8) | 不改 |
| KEY | `0x69` | 解锁 (写 `0xB588`); **解锁后 10s 内无指令会自动上锁** | — | 写寄存器前先解锁 |

- **为什么必须配 RSW 一次**: 出厂默认 `0x001E` **不含 GPS 帧**, 不配就只能拿到姿态、拿不到经纬度。
- **为什么 RRATE 由 10Hz 降到 5Hz**: 7 种帧 × 11B × 10Hz ≈ **770 B/s**, 已到 9600 的约 **80%**, 偏紧; 5Hz ≈ 385 B/s (**约 40%**) 更稳。
- ⚠️ **SAVE 会写模块 Flash**, 所以驱动**只在读回值与期望不符时才写** (见 §7.10.5) —— **避免每次上电都写、伤寿命**.

### 7.10.5 初始化流程

1. **装 UART2** (GPIO1/2, **9600** 8N1);
2. **波特率自扫描**: 依次 **9600 → 115200 → 57600 → 38400 → 19200 → 4800 → 230400** (每档发一条"读 RSW"命令、等 120ms), **命中即用** —— 模块波特率可能被上位机软件改过, 官方 STM32 例程也这么做;
3. **读回当前 RSW / RRATE** (命令 `FF AA 27 <reg> 00`);
4. **只有与期望不符才写**: 解锁 → 写 → 保存 SAVE → **回读验证** (见下面命令格式);
5. **启动接收任务** `nav_task` (优先级 4, 栈 3072, 由 `nav_init()` 内部创建, §3)。

**命令格式** (维特寄存器读写):

```
读:   FF AA 27 <reg> 00        → 应答 0x55 0x5F + 连续 4 个寄存器的值
解锁: FF AA 69 88 B5           (KEY = 0xB588, 其他值无效)
写:   FF AA <reg> <data低> <data高>
保存: FF AA 00 00 00           (SAVE)
```

- 模块**没接入不报错**: `nav_init()` 返回 **`ESP_ERR_NOT_FOUND`**, 接收任务**照常启动**; 模块**后接上会自动补配置并出数据** (**每 5s 重试一次配置**), **不必重启**。
- 配置结果由 `nav_stats_t.cfg_ok` 体现 (控制台 `nav` 显示 `已生效` / `未确认 (稍后自动重试)`)。

### 7.10.6 API 与数据结构

```c
nav_config_t nav_get_default_config(void);  // uart=UART2, tx_gpio=18, rx_gpio=16, baud=9600, rsw=0x058F, rrate=0x05 (5Hz)
esp_err_t    nav_init(const nav_config_t *cfg);  // 装 UART + 配置模块输出 + 起 nav_task; ESP_ERR_NOT_FOUND = 模块无应答
bool         nav_is_ready(void);                 // 收到过至少一帧 (模块在线)
esp_err_t    nav_read_imu(nav_imu_t *out);       // 最新姿态快照 (线程安全); 无数据返回 ESP_ERR_INVALID_STATE
esp_err_t    nav_read_gps(nav_gps_t *out);       // 最新 GPS 快照 (线程安全)
esp_err_t    nav_get_stats(nav_stats_t *out);    // 运行统计 (线程安全)
```

`components/nav/CMakeLists.txt` **REQUIRES**: `freertos driver esp_timer`。

| 结构 | 关键字段 |
|---|---|
| `nav_imu_t` | `ax/ay/az` (g) / `gx/gy/gz` (°/s) / `roll/pitch/yaw` (°) / `temperature` (℃) / `timestamp_us` / `valid` |
| `nav_gps_t` | `valid` / `latitude` / `longitude` / `altitude_m` / `speed_kmh` / `course_deg` / `satellites` / `pdop`·`hdop`·`vdop` / `time_valid` + `year`·`month`·`day`·`hour`·`minute`·`second` / `last_fix_us` |
| `nav_stats_t` | `ready` / `cfg_ok` / `baud` / `rsw` / `rrate` / `rx_bytes` / `frames_ok` / `frames_bad` / 各类型帧计数 `cnt_time`·`cnt_acc`·`cnt_gyro`·`cnt_angle`·`cnt_gps`·`cnt_vel`·`cnt_dop` |

### 7.10.7 诊断 (控制台 `nav`)

`nav` 打印: **在线状态 / 实际波特率 / RSW 与 RRATE (读回值 vs 期望值 + 是否已生效) / 累计字节与帧计数 (`帧OK` / `帧错`) / 各类型帧计数 / 姿态与 GPS 两套快照**, 并自动给结论:

| 现象 | 结论 | 处置 |
|---|---|---|
| `无数据` 且 `字节=0` | UART **一个字节都没进来** | 查模块 **3.3V / GND 共地 / TX-RX 是否交叉 (模块 TX → IO1)**、模块是否上电 |
| 有数据但 `输出配置 … 未确认` | RSW 出厂默认 `0x001E` **不含 GPS 帧** | 驱动**每 5s 自动重试**; 也可查接线/波特率 |
| 配置正常但 `未定位` | 数据在收, 只是没搜到星 | 天线朝天 + 开阔处; `gps 1` 开 1Hz 日志观察 |

> 相关命令: `nav` (模块状态) / `gps` (GPS 快照, `gps 0|1` 开关 1Hz 日志) / `hull` (姿态快照, `hull 0|1` 开关 10Hz 日志), 见 §7.9。
> 日志任务 (仅 `NAV_ENABLE=1` 时创建): `nav_gps` 1Hz 打 `[GPS] 定位…/纬度…`; `nav_att` 10Hz 打 `[HULL] rpy=(…) a=(…)g g=(…)dps T=…C` (**两者默认静默**)。

### 7.10.8 上位机 GPS 上报 (v9.1)

> **控制台 `gps` 只能在本机看**; 要让 PC 端「GPS」面板也看到, 主控把快照打包成两个 16B 状态帧从 **8080** 发出:
>
> | 帧 | 内容 | 速率 |
> |---|---|---|
> | `type=0x04` | 纬度 / 经度 / 海拔 / 卫星数 / flags | **2Hz** |
> | `type=0x05` | 航向 / 地速 / P·H·V DOP / 模块时间 / flags | **2Hz** |
>
> - 实现: `main.c` 的 `tcp_poll_send_gps()` (由 `tcp_server_task` 循环调用) → `control_build_gps_pos_frame()` / `control_build_gps_nav_frame()`;
>   **`nav_read_gps()` 只加锁拷结构体、不碰 UART**, 所以在 TCP 任务里直接读不会与 nav 接收任务抢串口。
> - **模块没 GPS 帧时一帧都不发** ⇒ 上位机面板保持 `--` (与「主控 MPU」面板同一约定);
>   未定位但模块在线时**照发**, 只是 `flags.bit0=0`, 面板把纬度/经度/航向/地速显示成 `—`。
> - **精度取舍**: DOP 用 `uint8 ×0.1` (上限 25.5)、航向 `×0.01°`、地速 `×0.01 km/h`、海拔 `×0.1 m`;
>   要原始精度还是看控制台 `gps`。字段布局见 **§2.4**, 客户端解析见 `tools/tcp_console.py` 的 `_handle_frame()`。
> - 用 `NAV_ENABLE=0` 关掉惯导时, 这段上报**整体不编译** (不会有空帧)。

---

## 8. 控制协议解析器 (control 组件) — v5.12 已启用 (只解析)

> ✅ `control_process()` / `control_reset()` 已在 `tcp_server_task` 中调用 (8080 收到的字节流直接喂进去, §6)。
> ⚠️ **v5.12 职责变更**: 组件**只做协议解析**, 不再执行任何电机动作 ——
> 新增 `control_set_local_callback()`, 解析出 `CTRL_CMD_MOTOR` 后把 `ctrl_command_t` 回调给 `main.c` (`tcp_apply_local_command()` 做差速混合 + 驱动 4 路电调 / 滚筒);
> **删除**旧 `clamp100()` 与 `mix_and_execute_local()` (写死旧硬件, 负油门发单向电调会被钳成 0)。
> ⚠️ 回调在 **TCP 任务上下文** 中以可达 **20Hz** 的频率被调用, 实现里**不得 `printf`** (刷屏 + 拖慢收发) —— `main.c` 侧用 `motor_apply_speed(..., quiet=true)` / `esc_apply_side(..., quiet=true)` 静默执行。
> ⚠️ 转发回调 `control_set_forward_callback()` 仍**未注册** (8081 未启用), `cmd=0x11 DEPTH` 收到后只解析不动作。

### 8.1 状态机

```
IDLE ──[0xAA|0xBB]──> GOT_HEAD0 ──[0x55|0x66]──> LOADING_CTRL / LOADING_MPU
                          │                              │
                          │ [其他字节]                   │ [累积到 16B]
                          ▼                              ▼
                        IDLE <────── CRC 校验 ─────[执行 + 计数]
```

> v5.4 已删除死状态 `CTRL_STATE_LOADING_FWD`, 转发由 `handle_ctrl_command` 直接调用回调完成.

### 8.2 API

```c
/* 解析器 */
size_t control_process(const uint8_t *data, size_t len);
void   control_reset(void);
void   control_set_forward_callback(ctrl_forward_cb_t cb);   /* 8081 转发用, 当前未注册 */

/* 本地指令回调 (v5.12) —— 由 TCP 任务上下文调用, 可达 20Hz, 实现里不要 printf */
typedef void (*ctrl_local_cb_t)(const ctrl_command_t *cmd);
void   control_set_local_callback(ctrl_local_cb_t cb);

/* 帧构造器 (上位机和远端节点开发用) */
void control_build_ctrl_frame(uint8_t *frame, uint8_t cmd,
    int8_t speed, int8_t yaw,
    uint8_t remote_light, int8_t bucket_speed, uint8_t flags);

void control_build_mpu_frame(uint8_t *frame, uint8_t type,
    int16_t ax, int16_t ay, int16_t az,
    int16_t gx, int16_t gy, int16_t gz);

/* v9.1 GPS 状态帧 (16B): type=0x04 定位 / type=0x05 运动+精度+时间 */
void control_build_gps_pos_frame(uint8_t *frame, uint8_t flags,
    int32_t lat_e7, int32_t lon_e7, int16_t alt_dm, uint8_t sats);

void control_build_gps_nav_frame(uint8_t *frame, uint8_t flags,
    uint16_t course_cdeg, uint16_t speed_ckmh,
    uint8_t pdop_d1, uint8_t hdop_d1, uint8_t vdop_d1,
    uint8_t hour, uint8_t minute, uint8_t second);

/* v3.0 沉浮控制帧 (16B, cmd=0x11, 上位机→主控) */
void control_build_depth_ctrl_frame(uint8_t *frame,
    int16_t target_depth_cm, int8_t target_pitch, int8_t target_roll, uint8_t flags);

/* v3.0 沉浮控制子帧 (8B, cmd=0x11, 主控→远端) */
void control_build_depth_fwd_frame(uint8_t *frame,
    int16_t target_depth_cm, int8_t target_pitch, int8_t target_roll);

/* 统计 */
uint32_t control_get_frame_count(void);
```

> **v9.2 云台手动帧 (cmd=0x12)**: 布局见 §2.2; **主控侧不提供** `control_build_servo_frame()` ——
> 这个 cmd 只由上位机发 (python 侧 `tools/tcp_console.py: build_servo_frame()`), 主控只解析+执行。
> `ctrl_command_t` 新增字段: `uint8_t servo_mask` (bit0=ch0, bit1=ch1) 与 `int16_t servo_deg10[2]` (标称角 ×0.1°)。
> ⚠️ 解析时字节 3-6 被 DEPTH 与 SERVO **两种语义共用** (同一条 16B 帧), 由 `[2] cmd` 区分 —— 两者不会同时有效。

> **v3.0 沉浮状态帧 (type=0x03)**: 由远端 `vertical_ctrl` + `control_build_vctrl_status_frame()` 构造并上报,
> 主控 `handle_mpu_frame()` 把字节 3-14 解出再原样重建 (两条路径都是 int16 LE + 异或 CRC, 结果逐字节一致), 因此 type=0x03 也被**无损透传**至上位机.
> 主控侧**不提供** `control_build_vctrl_status_frame()` (不需要构造它).
> `ctrl_command_t` 的 DEPTH 字段为 `target_depth_cm` / `target_pitch_deg` / `target_roll_deg` (v2.x 的 `depth_mode` 已删除).

### 8.3 常量定义

```c
#define CTRL_CTRL_HEAD_0       0xAA
#define CTRL_CTRL_HEAD_1       0x55
#define CTRL_CTRL_FRAME_SIZE   16
#define CTRL_FWD_FRAME_SIZE    8

#define CTRL_MPU_HEAD_0        0xBB
#define CTRL_MPU_HEAD_1        0x66
#define CTRL_MPU_FRAME_SIZE    16

#define CTRL_CMD_MOTOR         0x10
#define CTRL_CMD_DEPTH         0x11   /* v3.0 沉浮控制 (深度cm + 俯仰° + 横滚°) */
#define CTRL_CMD_SERVO         0x12   /* v9.2 云台手动 (ch0/ch1 标称角 ×0.1° + 通道掩码) */
#define CTRL_CMD_STOP          0x20
#define CTRL_CMD_REBOOT        0x30
#define CTRL_CMD_SHUTDOWN      0x40

#define CTRL_MPU_TYPE_LOCAL    0x01
#define CTRL_MPU_TYPE_REMOTE   0x02
#define CTRL_MPU_TYPE_DEPTH    0x03   /* v3.0 沉浮状态 */
#define CTRL_MPU_TYPE_GPS_POS  0x04   /* v9.1 GPS 定位 */
#define CTRL_MPU_TYPE_GPS_NAV  0x05   /* v9.1 GPS 运动/精度/时间 */

/* v9.1 GPS 状态帧 (type=0x04/0x05) 的 flags 位 */
#define CTRL_GPS_FLAG_FIX      0x01   /* 定位有效 */
#define CTRL_GPS_FLAG_TIME     0x02   /* 模块时间有效 */

#define CTRL_FLAG_ENABLE_LOCAL   0x01
#define CTRL_FLAG_FORWARD_REMOTE 0x02

/* v3.0 沉浮状态帧 (type=0x03) 的 flags 位 */
#define CTRL_VCTRL_FLAG_DEPTH_VALID  0x01
#define CTRL_VCTRL_FLAG_IMU_VALID    0x02
#define CTRL_VCTRL_FLAG_MANUAL       0x04
```

## 9. 远端节点开发接口 (设计参考, 当前 8081 未启用)

> ⚠️ 本章描述主控**恢复 8081 转发后的**远端对接契约; v5.12 主控只开了 8080 (上位机), 远端转发与 MPU 回传未启用 (§0/§6)。远端 (ESP32-S3-below) 现为 **v3.0**, 见配套远端文档。

远端 ESP32-S3 (TCP Client 8081) 需实现:

1. **连接**: 主动 connect 主节点 192.168.29.10:8081
2. **接收 8B 子帧** (帧头 0xAA 0x55):
   - 解析 `cmd` + `speed` + `yaw` + `remote_light` + `bucket_speed`
   - 做差速混合 → 控制 2 个水平电调
   - 按 `bucket_speed` 驱动 L298N 铲斗电机
   - 直接控制灯
   - **v3.0** `cmd=0x11 DEPTH` → `vertical_ctrl_set_target(depth_cm/100.0f, pitch, roll)` (见配套远端文档 §4.5)
3. **接收 16B MPU 帧** (帧头 0xBB 0x66, type=0x01):
   - 主节点发来的本节点 MPU 数据 (备用, 不必处理)
4. ~~20Hz 发送 16B MPU 帧 (type=0x02)~~ **v3.0 已取消**: 远端姿态由 YB-MRA02 内部融合, 改由下面 type=0x03 的 `pitch/roll` 承载
5. **v3.0 每 500ms 发送 16B 沉浮状态帧** (帧头 0xBB 0x66, type=0x03):
   - `depth_cm` + `pitch (0.1°)` + `roll (0.1°)` + `mode` + 三路油门 (`out_f/out_rl/out_rr`) + `flags`
   - 由 `.11` 节点 `control_build_vctrl_status_frame()` 构造; 主控**原样透传**上位机 (不解析不修改)

### 远端伪代码示例 (v3.0)

```c
// 远端 TCP Client 循环
while (1) {
    // 1. 接收并解析 8B 子帧 → 差速混合 → 控制电调 + 灯 + 电机
    n = recv(sock, buf, 8, 0);
    if (n == 8 && buf[0] == 0xAA && buf[1] == 0x55) {
        if (crc_check(buf, 8)) {
            uint8_t cmd   = buf[2];
            int8_t  speed = (int8_t)buf[3];
            int8_t  yaw   = (int8_t)buf[4];
            uint8_t light = buf[5];
            int8_t  bucket = (int8_t)buf[6];   // v2.19: L298N 铲斗速度

            if (cmd == 0x11) {          // v3.0 沉浮控制
                int16_t depth_cm = (int16_t)(uint16_t)(buf[3] | (buf[4] << 8));
                int8_t  pitch    = (int8_t)buf[5];   // 目标俯仰 °
                int8_t  roll     = (int8_t)buf[6];   // 目标横滚 °
                vertical_ctrl_set_target((float)depth_cm / 100.0f, pitch, roll);
            } else if (cmd == 0x20) {
                // 紧急停止
                motor_set_throttle(0, 0);
                motor_set_throttle(1, 0);
                motor_set_vertical(0, 0, 0);
                set_light(0);
                set_bucket(0);
            } else {
                // 差速混合
                int16_t left  = clamp(speed + yaw, -100, 100);
                int16_t right = clamp(speed - yaw, -100, 100);
                motor_set_throttle(0, left);
                motor_set_throttle(1, right);
                set_light(light);
                set_bucket(bucket);
            }
        }
    }

    // 2. 每 500ms 回传沉浮状态 (type=0x03)
    //    v3.0 取消原 20Hz type=0x02 (远端原始 MPU); 姿态改由此帧的 pitch/roll 承载
    if (tick % 5 == 0) {
        vertical_ctrl_get_status(&st);
        control_build_vctrl_status_frame(frame,
            (int16_t)lroundf(st.depth_m * 100.0f),        // m -> cm
            (int16_t)lroundf(st.pitch_deg * 10.0f),        // ° -> 0.1°
            (int16_t)lroundf(st.roll_deg  * 10.0f),        // ° -> 0.1°
            st.mode, st.out_f, st.out_rl, st.out_rr, st.flags);
        send(sock, frame, 16, 0);
    }
}
```

---

## 10. 关键工程决策 (历史)

> 📌 **版本号规则 (v6.0 起执行)** —— 第一位 = **代际**, 以“破坏性变更”为界跳代; 第二位 = 非破坏的功能批次; 第三位 = 修复批次。
>
> | 标记 | 含义 | 是否牵连别人 | 版本位怎么走 |
> |---|---|---|---|
> | 🔴 | **破坏性变更**: 删组件 / 删对外命令或 API、改引脚或接线、改协议字段或语义、整机形态大范围停用启用 | 别人**必须**跟着改 | **第一位 +1** (6.0 → 7.0) |
> | 🟡 | 非破坏的新增/增强: 新组件、新命令、新能力 | 不影响既有 | 第二位 +1 (6.0 → 6.1) |
> | 🟢 | 修复 / 文档同步 / 内部重构 (不改对外接口) | 无 | 第三位 +1 (6.0 → 6.0.1) |
>
> ⚠️ **历史版本号不重排**: v5.0~v5.12 里也有 🔴 (v5.6 / v5.7.8 / v5.8.0 / v5.10.0 / v5.11.0 与“亚博 IMU 改接 UART”),
> 这些号已被**远端文档 / 归档 / 代码注释**引用 (远端文档里写着“配套主控 v5.7.6”“主控 main.c v5.2”), 重排会让引用全部失配。
> 所以 **5.x 原号保留、整体视为“旧代际”**, 从本次破坏性变更起进入 **v6.0**; 下表的标记是按现行规则**回溯**判定的 (🔴 = 当时其实就该跳代)。

| 时间 | 决策 | 原因 |
|---|---|---|
| 2026-01 | W5500 ioLibrary (非 LwIP) | 硬件 TOE, 性能更优 |
| 2026-01 | SPI 8 MHz → 20 MHz | 配合 polling 模式, 仍可稳定工作 |
| 2026-01 | SPI DMA → POLLING | 解决 ESP-IDF v5.5 DMA 反复 malloc 触发 Task WDT |
| 2026-01 | log 默认 NONE | 减少启动刷屏 |
| 2026-01 | Task WDT 退订长 I/O 任务 | 阻塞式 SPI 不适合 5s 默认超时 |
| 2026-01 | INT GPIO9 + 1ms 去抖 | 防止 INT 抖动反复触发 ISR |
| 2026-01 | SPI 互斥锁 100ms 超时 | 防止一个任务卡死 SPI 导致所有任务死锁 |
| 2026-01 | 双 socket 8080+8081 | 主机和远端独立连接, 互不干扰 |
| 2026-01 | 16B MPU 帧 (6 轴 int16 LE) | 紧凑 (320 字节/秒@20Hz), 上位机解算 |
| 2026-01 | USB-Serial/JTAG 日志 | 释放 GPIO1 给 ESC1 PWM |
| **2026-01** 🔴 | **差速驱动 v4.0: speed+yaw** | **简化协议, 主机发高层指令, ESP32 做开环差速混合** |
| **2026-01** 🔴 | **删除舵机 TCP 接收 (字节 8/9)** | **主控制节点舵机不再由上位机控制** |
| **2026-01** 🔴 | **新增远端灯+电机信号** | **远端有灯开关和电机方向+停止** |
| **2026-01** 🟡 | **L298N 由 speed 自动计算** | **速度用 |speed|, 方向用 sign(speed)** |
| **2026-01** 🟡 | **v5.0 新增 RDK X5 UART 通信** | **后续扩展性: 网络摄像头→RDK X5 AI 识别→ESP32 桥接** |
| **2026-01** 🟡 | **UART0 GPIO43/44 115200 8N1** | **专用硬件 UART, 与 console 释放解耦** |
| **2026-01** 🟡 | **RDK X5 主动模式 (PING/GET_MPU/GET_STATUS)** | **简化 ESP32 逻辑, 资源开销小** |
| **2026-07** 🟢 | **v5.1 文档校对** | **标题修正 (水上/水下), W5500 默认 IP 标注, rdk_uart_rx_task 补入任务清单, INT 去抖描述统一** |
| 2026-07 🟢 | **v5.2 修复主控 IP** | **main.c 显式覆盖 wiznet_manager_get_default_config() 的 IP 为 192.168.29.10, 网关为 192.168.29.1, 与远端节点 .11 同 /24 网段** |
| 2026-07 🟢 | **v5.3 重写 MPU6050 驱动** | **完全替换旧 imu.c: I2C0 自动地址探测、PLL 唤醒、±2g/±250°/s 默认量程、burst 读取 14 字节、温度输出、WHO_AM_I 验证; 兼容 ESP-IDF v5.5 的 clk_speed / i2c_master_write_read_device 等 API** |
| 2026-07 🟡 | **v5.4 状态灯 + 连接状态 + PHY** | **所有发送路径调用 status_led_notify_tx(); s_host_connected/s_remote_connected 改为 atomic_bool; control 删除 CTRL_STATE_LOADING_FWD; status_led 增加 s_data_mux 临界区 + 无 IP 红灯; sdkconfig 启用 USB-Serial/JTAG console; W5500 PHY 改软件强制 100M FULL** |
| 2026-07 🟡 | **v5.5 状态灯改为普通 RGB LED** | **完全重写 status_led: 从 WS2812/RMT 改为 3 GPIO 普通 RGB LED + LEDC PWM; 支持共阴/共阳; 颜色表覆盖红/绿/蓝/深黄/浅黄/紫/深蓝/浅蓝/白; main.c 改为 R=48/G=47/B=21 占位** |
| **2026-08** 🔴 | **v5.6 水上水下联合方案协议** | **control 协议扩展 v2.0: 新增 cmd=0x11 DEPTH (16B 控制帧+8B 子帧, 目标深度 cm+模式) + type=0x03 深度状态帧 (远端→主控→上位机原样转发); main.c `forward_to_remote()` 按 cmd 分支构造深度子帧; 配套远端文档 v2.0** |
| **2026-08** 🟢 | **删除 tcp_parser 组件** | **tcp_parser 为 lwip 早期方案遗留 (W5500 TOE 方案下无任何调用), 按文档 §7"遗留, 暂未用"确认无作用后删除组件 + main.c include + REQUIRES** |
| **2026-08** 🟢 | **v5.7.7 同步远端 v2.18** | **远端 m_comp 暂定 1.6 kg, 配重 = **1871.28 g**; 协议无变化, 仅文档同步** |
| **2026-09** 🔴 | **v5.7.8 同步远端 v2.19 (L298N 铲斗电机)** | **控制帧/转发子帧 `frame[6]` 由 `remote_dir` 改为 `bucket_speed` (int8, -100~+100, 0=停), 用于远端 L298N 铲斗电机; `control.h` 中 `ctrl_command_t.remote_dir` 重命名为 `bucket_speed` (int8), `control_build_ctrl_frame()` 签名同步修改; `control.c` 解析/日志/帧构造更新; `main.c` `forward_to_remote()` 8B 子帧转发 `bucket_speed`; 更新本文档 §1.1/§2.2/§2.3/§2.7/§8.2/§10/版本历史** |
| **2026-09** 🔴 | **v5.8.0 删除 RDK X5 UART 方案** | **RDK X5 已砍: 删除 `components/rdk_uart` 整个组件 + `main/CMakeLists.txt` REQUIRES + `main.c` include/初始化/RDK 主动请求逻辑 + `sdkconfig.defaults` RDK 注释 + README 引脚表/目录树; 本文档删除原 §9「RDK X5 UART 通信」全章 (后续章节顺延: 原 §10 远端节点开发接口 → §9) 及 §1.1/§1.2/§3/§4/§7 中全部 RDK 条目; console 保持 UART0 (GPIO43/44 已空闲)** |
| **2026-09** 🟡 | **v5.9.0 新增 ROCK 5C 视觉节点** | **ROCK 5C (Radxa SBC) 同时承担推流 + AI 推理, 输出的 AI 推理视频流经内网直接访问; 与主控节点 (ESP32) 无任何数据链路, 不占用 GPIO/UART/TCP 资源; 恢复"网络摄像头"节点条目 (作为 ROCK 5C 输入源); 更新本文档 §1.1/§1.2 节点表与拓扑图 + README** |
| **2026-09** 🔴 | **v5.10.0 换板 ESP32-S3-ETH + 同步远端 v3.0 协议** | **①硬件: 换 Waveshare ESP32-S3-ETH (板载 W5500, 外置模块弃用), W5500 引脚改为板载固定 13/11/12/14/9/10, 引脚全表按"同功能相邻 + 避开 SD 4-7 与 strapping 0/45/46"重排: 4 路电调 42/41/40/39, L298N 48/47/38, I2C0 16/18, I2C1 21/17, GPS 2/1/3; 删除 `components/status_led` 组件 (新板无板载 LED) + `main/CMakeLists.txt` REQUIRES + main.c 遗留引用. ②协议: `cmd=0x11` 字节 5/6 由 `mode`+保留 改为 `target_pitch int8(°)`+`target_roll int8(°)`; `type=0x03` 状态帧整体重定义 (depth_cm / pitch / roll / mode / out_f / out_rl / out_rr / flags); `ctrl_command_t` 的 `depth_mode` → `target_pitch_deg` + `target_roll_deg`; `control_build_depth_ctrl_frame()` / `control_build_depth_fwd_frame()` 签名同步; 主控取消 type=0x02 远端原始 MPU 上报. ③同步更新本文档 §1/§2/§3/§4/§5/§7/§7.5/§8/§9/§11/§12 + README** |
| **2026-09** 🔴 | **v5.11.0 切至临时测试/标定模式 + 文档同步实况** | **①固件: `main.c` 进入 [临时测试/标定模式] —— W5500/TCP Server/motor/control 的初始化与调用全部注释 (代码保留), 实跑 云台 MPU6050(I2C0) + PCA9685(I2C1) + 船体10轴IMU(0x50) + GPS(UART1) + 云台运动规划/姿态闭环 + Madgwick 姿态解算 + 串口控制台; `TEST_MODE=4` (云台闭环自稳)。②新增 `components/gps` 组件 (NMEA0183 RMC/GGA + PPS, UART1 TX2/RX1/PPS3 @9600)。③servo 组件新增标定体系: 双角度刻度(标称角/物理角)、按通道实测默认标定(ch0 530~2730µs=0~360°, phys=365; ch1 600~2700µs=0~180°, phys=192, 限位 30~150°)、trim、行程限位、NVS 存取、cmd↔phys 换算、`servo_set_phys_angle`。④gimbal 组件新增姿态闭环 PID 自稳: 100Hz, 增量式 PID (kp=0.8/ki=0.5/kd=0), 物理角域控制律, fb_sign(ch0=+1/ch1=−1), 积分限幅 ±10, 输出速率限 120°/s。⑤新增串口标定控制台 (esp_console, UART0 115200, 提示符 `gimbal>`, 21 条命令 + 裸数字改 ch0 脉宽)。⑥文档: 新增 §0 当前固件形态, 重写 §3 任务清单, 更新 §4/§7 组件清单, 新增 §7.6 servo 标定 / §7.7 gimbal 闭环 / §7.8 gps / §7.9 串口控制台, 给 §2/§5/§6/§8/§9 加"设计参考, 当前未启用"标识; 同步 README** |
| **2026-09** 🟢 | **v5.11.1 修复电机 LEDC 通道冲突 + 16MB Flash 生效 + 注释更正** | **① `motor.c`: L298N 的 `dc_ledc_channel` 由 `LEDC_CHANNEL_0` 改为 `LEDC_CHANNEL_4` (Timer1 不变)。原因: 4 路电调占 Timer0/Channel0~3, L298N 原也占 Channel0, `motor_init()` 先配 ESC1、后配 L298N 会把 Channel0 的 timer 重绑到 Timer1 → ESC1 失去 50Hz PWM (此前未调用 `motor_init()` 未暴露)。② 删除 `sdkconfig`/`sdkconfig.old` 重新生成, 使 `sdkconfig.defaults` 的 `CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y` 生效 (原 sdkconfig 停留 2MB), 全量重编译 (`flasher_args.json` → `--flash_size 16MB`)。③ 更正 `motor.c` 文件头通道表与 `motor_init()` 内过时引脚注释 (GPIO1/2/3 → 42/41/40/39)、ESC1 失效的 "U0TXD" 启动日志, 及 `motor.h` 中已废弃的 `MOTOR_TEST_ESC` 引用。④ 新增 §4.1 电机 LEDC 通道分配; 更新 §4/§7/§12 + README** |
| **2026-09** 🟡 | **v5.11.2 启用 W5500 (仅拿 IP) + 云台闭环无反馈保护 + 修复 I2C1 连带失效 + GPS 诊断增强 + 清理换板遗留** | **① 启用板载 W5500 以太网 (仅拿 IP): `wiznet_manager_init()` 配置静态 IP 192.168.29.10 / 网关 192.168.29.1 (板载固定 SCK13/MOSI11/MISO12/CS14/RST9/INT10, 20MHz polling, 100M FULL), 上电即拿到 IP; 网线未插则 30s 超时后告警继续启动。**TCP Server 与全部 TCP 收发/转发仍注释** (`tcp_server_task` / `mpu_push_task` / `status_report_task` 均不创建, 8080/8081 不监听), motor 电机与 control 协议当时同样保持注释 (电机已在下一行启用)。② **云台闭环新增"无反馈保护"** (`gimbal.c/.h`): 超过 `GIMBAL_FB_TIMEOUT_MS`(**200ms**) 收不到姿态反馈 (云台 MPU 未就绪 / 零偏标定失败 / 读数中断) ⇒ **停用 PID, 该轴 `servo_center()` 回标定中位** (ch1 = 90°), 不再拿"假 0°"当反馈把俯仰顶到行程限位下限 (ch1 = 30°); 反馈恢复自动接管; 状态新增 `nofb` (`gs` 显示 **N/FB**, `[STAB]` 日志追加标记); `components/gimbal/CMakeLists.txt` REQUIRES 补 `esp_timer`。③ **修复: PCA9685 探测失败不再删 I2C1 总线** (`servo.c`): 该总线还挂着船体 10 轴 IMU (0x50), 原先 `servo_init()` 探测失败会 `i2c_driver_delete(I2C1)` —— 亚博 IMU 随之失效, 且 `scan 1` / `hull` 在线重探全部报 `i2c driver not installed` (只能重启)。现改为**保留总线**: servo 自身功能关闭 (`s_initialized` 保持 false, 所有 servo API 返回 `ESP_ERR_INVALID_STATE`, 不往不存在的芯片写), 船体 IMU 照常读写并可在插好后在线重探; `servo_deinit()` 同步改为只在成功初始化过时才释放总线。④ 清理换板遗留: 删除原 §11「状态指示灯」全章 (内容归档至 [`docs/legacy/status_led_方案.md`](docs/legacy/status_led_方案.md) + [`docs/legacy/README.md`](docs/legacy/README.md) 索引; 后续章节顺延: 原 §12 构建与烧录 → §11); 更正 `servo.h` 旧板 I2C 引脚注释 (`SDA=4/SCL=5` → `21/17`) 与 [`ESP32-S3-ETH-pinout.md`](ESP32-S3-ETH-pinout.md) §六 I2C 推荐脚 (`GPIO47/48` 在本项目已作 L298N 方向脚)。⑤ **GPS 诊断增强** (`components/gps` + `main.c`): `gps_data_t` 新增 **`rx_bytes`** (累计收到的原始字节数, `gps_task` 累加、`gps_get_data()` 带出); 控制台 `gps` 改打印 `收到字节=N 语句=M 错误=K`, 并在 `收到字节=0` / `语句=0` 时**自动给出结论** (接线交叉 vs 波特率不对) —— 解决"没接线"和"波特率不对"都表现为 `语句=0 错误=0` 无法区分的问题; 同时澄清 GPS 实测硬件 **ATGM336H 与本组件兼容** (默认 9600、`$GN` 前缀、RMC/GGA 字段顺序一致), 新增 §7.8.1 诊断判据表。**已完成** (同步更新 §0 / §5 / §7.5~§7.9 / §11 / 版本头 + README)。** |
| **2026-09** 🔴 | **亚博 10 轴 IMU 由 I2C 改接 UART (根治协议不匹配) + 云台 MPU 与 PCA9685 共线 I2C1** | **① 根因: 该模块不是维特(WIT) I2C 协议** —— 厂商 UART 示例 (`IMU-UART示例/`) 表明它用自有协议: 帧 `7E 23 \| LEN \| FUNC \| DATA \| SUM(累加)`, 以 `0x80` 请求式取数 (`0x04` 六轴+磁力计 int16 LE / `0x26` 欧拉角 float32 LE / `0x32` 气压温度), **115200** 8N1 —— 这正是此前 I2C 扫不到 0x50 的原因。② `imu` 组件新增 **UART 后端** (开关 `imu.h: IMU_HULL_IFACE_UART`: 装 UART2 + 帧状态机 + 累加校验 + 量纲换算 (原始 int16 → °/s / g 等物理单位) + 版本探测), 上层 `imu_read_role` / `imu_role_ready` 接口不变, 原 I2C(WIT) 分支用条件编译保留可随时回退。③ **云台 MPU6050 从 I2C0 挪到 I2C1 与 PCA9685 共线** (开关 `IMU_GIMBAL_SHARED_I2C1`; 0x68/0x40 地址不冲突, 上拉并联 2.35k 仍合规, 400kHz 50cm 内可靠) ⇒ **腾出的 GPIO16/18 正好给亚博 IMU 的 UART**; 随之调整初始化顺序 (两颗 IMU 合并到 `main.c` 4c 段, **必须排在 `servo_init()` 之后** —— 两者都借用 servo 装的总线)。④ `imu` 组件新增 `s_bus_owned` 区分"自装/借用", `imu_deinit()` 只释放自己装的总线 (不再误删 servo 的 I2C1)。**已完成** (同步 §0 / §4 / §7.5 / §7.9 / §11 + README) |
| **2026-09** 🟢 | **修复亚博 IMU 角速度量纲错误 (数值被放大 57.3 倍)** | **现象: UART 后端接上后, **正常手转模块**时 `[HULL]` 的角速度冲到 **±10000dps** (峰值 10928dps ≈ 3 圈/秒), 而同一时刻模块内部解算的欧拉角只变化几度、加速度仍在 1g 上下。**判据** —— 手转模块最多几百 dps, 且模块自身融合认定几乎没转过, 说明是驱动换算错而非传感器在抖。**根因**: `imu.c` `hull_uart_parse_raw()` 的 `gyr_ratio = (2000/32767) × (180/π)` 多乘了一次角度换算。厂商参考实现是 `rad/s = count × (2000/32767) × (π/180)`, 反推 `count × (2000/32767)` **本身就是 °/s** (满量程 ±2000dps) —— 再乘 180/π 等于把 °/s 当成 rad/s 又换了一次 (57.3 倍)。**修复**: `gyr_ratio = 2000.0f / 32767.0f`。**验证**: 静止时角速度应在 ±2dps 内 (原始 LSB = 0.061dps), 手转时几十~几百 dps 并与转动幅度成正比。**已完成** (同步 §7.5.1 / §7.5.4 + `imu.h` 注释) |
| **2026-09** 🟢 | **修复亚博 IMU 欧拉角单位错误 (弧度被当成度直接打印)** | **现象: 模块明显转过 90° 时 (加速度 `a=(-0.977, +0.096, +0.009)g` ⇒ 重力几乎全落在第一轴上, 该轴竖直), `[HULL]` 的 `rpy` 只有 `(+1.58, +1.43, +1.39)` —— 单位像是"度"的话与重力方向直接矛盾。**判据**: ① 以重力为基准的 roll/pitch 必有一个 ≈±90°; ② 厂商示例 `IMU_UART_GetEuler()` 里是 `s_roll * 57.2957795f` 才输出度, 即 `0x26` 的 3×float32 出的是**弧度**。**验算**: 另一份日志 `a=(-0.405, -0.086, +0.902)g` ⇒ 重力偏离竖直 `arccos(0.902) ≈ 25.6°` 且朝向 -X, 按弧度换算得 pitch `+24.6°` ✓。**修复**: `hull_uart_parse_euler()` 三个角各乘 `180/π`; 该读数变为 `(+90.5°, +81.9°, +79.6°)`, 与"该轴竖直"吻合 (roll=±90° 时 pitch/yaw 本就是万向节锁下的耦合值)。**已完成** (同步 §7.5.1 / §7.5.4 + `imu.h` 注释) |
| **2026-09** 🟡 | **GPS 按 NMEA-0183 补全解析 + 新增现场自查指令 (gps raw / baud / send)** | **① 解析补全** (`gps.c`): 新增 `GSA` (定位模式 1/2D/3D + PDOP/VDOP)、`GSV` (可见卫星数 + 最大载噪比, 按每组首句 `index=1` 重置以免留历史峰值)、`GST` (经纬度误差标准差)、`VTG`/`ZDA` (RMC 缺失时兜底速度航向与时间日期)、`$GPTXT` (天线状态 OK/OPEN/SHORT) 解析; 语句类型仍只看后 3 字符, 兼容 `$GN`/`$GP`/`$BD`/`$GL`。② **合规性与易误判修复**: RMC `status=V` 时 `latitude`/`longitude`/`speed_kmh`/`course_deg` **清零** (以前保留旧值, 控制台会把上次坐标当成当前值显示), 并新增 `last_fix_us` 记录最近一次有效定位时刻; GGA 的 `quality` (0/1/2/4/5) 与 UTC 时间一并解析; 新增 `nmea_get_f()` / `nmea_get_i()` —— **空字段不再被 `atof("")` 当成 0 写进快照**。③ **现场自查指令** (`main.c`): `gps raw [0\|1]` 逐行回显原始 NMEA (非可打印字符转 `.`, 波特率不对产生的乱码也看得清)、`gps baud <n>` 运行时换波特率 (`uart_set_baudrate` + 丢弃残字节)、`gps send <语句>` 发 PCAS/PMTK 配置 (自动补 `$` 与 `*hh` 校验和 + CRLF); `gps` 快照输出与 1Hz 日志同步扩充 (质量/模式/使用与可见卫星/DOP/SNR/GST 精度/天线), 天线非 OK 时直接给排查提示。**已完成** (同步 §7.8 / §7.8.1 / §7.8.2 / §7.9) |
| **2026-09** 🟡 | **v5.12 启用网络接收上位机操控 (TCP Server 8080 + 差速混合迁出 control)** | **① 协议层职责重构** (`control.c/.h`): **删除** `clamp100()` 与 `mix_and_execute_local()` —— 旧实现写死旧硬件, 把**负油门**直接交给**单向电调**后被 `motor_set_esc_throttle()` 钳成 0, 导致**两路反推永不动**; 新增 **`control_set_local_callback()`**, 组件只解析协议并把 `ctrl_command_t` 回调上抛。**② `main.c` 接手执行**: 新增 `tcp_apply_local_command()` 按当前硬件做差速混合 (`left=clamp(speed+yaw)` / `right=clamp(speed-yaw)`, **正=推进 / 负=自动切反推通道**; `bucket_speed` → L298N 滚筒), 并给 `motor_apply_speed()` / `esc_apply_side()` 加 **`quiet` 参数** (20Hz 回调路径不打印)。**③ 恢复 TCP Server** (`W5500_ENABLE=1`): 新建 `tcp_server_task` (栈 8192 / 优先级 5) 只开 **socket 0 = 8080** (`SF_TCP_NODELAY`, 1024B 接收缓冲), 断线 (`CLOSE_WAIT`/`CLOSED`) 自动 `wiz_close` → 重建 listen, 新连接 `control_reset()` 清残帧; **8081 远端、MPU 回传 (`tcp_server_forward_mpu_to_host()` 空实现)、`status_report_task` 仍未启用**。**④ 新增 500ms 失联保护**: 收到过控制帧后超过 `TCP_LINK_TIMEOUT_MS` (上位机 20Hz ⇒ 连续丢 10 帧) 无新帧 ⇒ 4 路电调 + 滚筒**全部归零并解除武装**, 防止上位机崩溃/网线松脱后船**保持最后一条指令冲出去**。**⑤ 控制台新增 `net`** (端口 / 网线链路+速率 / 本机 IP / 上位机连接 / 累计控制帧 / 距上一条控制帧 ms)。**⑥ 客户端**: `tools/tcp_console.py` 的「铲斗」区改为「**滚筒收放**」(收/停/放 三按钮, `BUCKET_SPEED=100`), 文件头补"主控侧映射 (v5.12)"说明与失联保护提示; `tools/README.md` 同步。**⑦ 清理**: 删除 `main.c` 中约 240 行按旧硬件写的注释代码 (旧 `forward_to_remote` / `handle_socket` / `tcp_server_task` / `mpu_push_task` / `status_report_task`)。**⑧ 编译修复 (首次编译暴露)**: ① `wiznet_socket.h` 在包含 ioLibrary `socket.h` 时把 `close` 临时改名 —— 该头声明 `int8_t close(uint8_t)`, 与 newlib 的 `int close(int)` (main.c 的 `linenoise.h` → `<unistd.h>`) 同名不同签名, 同一翻译单元里会 `conflicting types for 'close'`; `wiznet_socket.c` 相应改为**先** `#include "socket.h"` 再 `#include "wiznet_socket.h"` (它自己要用原生 `close()/socket()/listen()`)。② `main.c` 打印本机 IP/掩码/网关由 `%d` 改 `%u` + 显式 `(unsigned)` 转换: `esp_ip4_addr_t.addr` 是 `uint32_t`, 在本工具链上等同 `unsigned long`, `%d` 触发 `-Werror=format` 编译失败 (cmd_net 与启动日志两处)。**已完成** (同步 §0 / 版本头 / §2 / §2.7 / §3 / §5 / §6 / §7 / §7.9 / §8 / §9 + README) |
| **2026-09** 🔴 | **v6.0 惯导替换: 删除 GPS 与旧亚博 10 轴 IMU, 新增 nav 组件 (亚博 GPS+10 轴 IMU 一体)** | **① 删除 `components/gps` 整个组件** (NMEA-0183 解析 / UART1 TX=2·RX=1·PPS=3 / `gps_init` 等 API / 控制台 `gps raw`·`gps baud`·`gps send` / PPS 中断) —— **GPIO1/2/3 现已空闲**, §7.8 改为指向惯导模块。**② `components/imu` 精简为只剩云台 MPU6050**: 删除旧的船体"亚博 10 轴 IMU"全部代码 (自有协议 `7E 23` 请求式 UART 后端 `0x80`/`0x04`/`0x26`/`0x32`、旧 I2C(WIT 0x50) 分支) 与 `imu.h` 的 `IMU_HULL_IFACE_UART` / `IMU_GIMBAL_SHARED_I2C1` 两个开关; `imu_role_t` **只剩 `IMU_ROLE_GIMBAL`** (`IMU_ROLE_HULL` 删除), `imu_data_t.roll/pitch/yaw` 对云台 MPU6050 **恒为 0** (姿态仍由上层 Madgwick 解算); API 名字与签名不变。**③ 新增 `components/nav`** (`nav.h`/`nav.c`/`CMakeLists.txt`, REQUIRES `freertos driver esp_timer`): 驱动**亚博 GPS+10 轴 IMU 一体模块**, **维特 WIT 标准 `0x55` 协议主动上报**, 单条 **UART2** (ESP32 RX=IO1 ← 模块 TX / TX=IO2 → 模块 RX) @ **9600** 8N1; 帧**定长 11B** (`0x55 | TYPE | 4×int16 | SUM`, SUM = 前 10 字节累加取低 8 位), 解析 `0x50` 时间 / `0x51` 加速度+温度 (`/32768×16` g, `/100` ℃) / `0x52` 角速度+电压 (`/32768×2000` °/s) / `0x53` 角度 (`/32768×180` °) / `0x57` 经纬度 (去小数点的 `ddmm.mmmmm`: 度=v/1e7, 分=(v%1e7)/1e5, 十进制度=度+分/60, 负=南纬/西经) / `0x58` GPS 海拔(`/10` m)·航向(`/100` °)·地速(`uint32/1000` km/h) / `0x5A` 卫星数+DOP(`/100`) / `0x5F` 读寄存器应答; 一条线同时输出**姿态与 GPS**, 新结构 `nav_imu_t` / `nav_gps_t` / `nav_stats_t` / `nav_config_t`, API `nav_get_default_config`·`nav_init`·`nav_is_ready`·`nav_read_imu`·`nav_read_gps`·`nav_get_stats`。**④ 输出配置按需写入 + 波特率自扫描**: 初始化先按 **9600→115200→57600→38400→19200→4800→230400** 逐档试 (每档发一条读 RSW、等 120ms), 命中即用; 然后**读回** RSW/RRATE (`FF AA 27 <reg> 00`), **只有与期望不符才** 解锁 (`FF AA 69 88 B5`, KEY=0xB588) → 写 → SAVE (`FF AA 00 00 00`) → **回读验证** (期望 RSW(0x02)=**0x058F** = TIME|ACC|GYRO|ANGLE|GPS 0x57|VELOCITY 0x58|GSA 0x5A, RRATE(0x03)=**0x05**=5Hz) —— 出厂默认 RSW=**0x001E 不含 GPS 帧**故必须配一次, 而**只在必要时写是为了避免每次上电都写模块 Flash**; 出厂默认 RRATE=0x06(10Hz)/BAUD=0x02(9600)/GPSBAUD=0x02/TIMEZONE=0x14(UTC+8) 均不改 (10Hz 时 7 帧×11B≈770B/s 已到 9600 的 80%, 降到 5Hz ≈40%); **模块没接不报错**: `nav_init()` 返回 `ESP_ERR_NOT_FOUND`、接收任务照常启动, 模块后接上**每 5s 自动重试配置**并出数据、**不必重启**。**⑤ GPS 有效性**: 模块**不上报定位标志**, 驱动以**经纬度非 0** 判有效, 无效时**坐标清零**并记 `last_fix_us`。**⑥ `main.c` 开关与命令改造**: `GPS_ENABLE` + `HULL_IMU_ENABLE` **合并为 `NAV_ENABLE`** (**当前 `0` 暂时关停**; 置 1 时 `nav_init()` 装 UART2 + 配置 + 出数据), 删除 GPS 引脚宏 (只留一行注释说明 GPIO2/1/3 曾用于旧 GPS); 控制台 `gps`/`hull` 改为读惯导快照 (**去掉 `gps raw|baud|send` 与 `hull addr`**)、**新增 `nav`** (在线/波特率/RSW+RRATE 读回 vs 期望/字节与帧计数/各类型帧计数/两套快照), `imu` **只读云台 MPU6050 一帧**, `scan` **只扫 I2C1** 并提示惯导走 UART; 日志任务 `gps_log`/`hull_log` → **`nav_gps`** (1Hz, 栈 4096) 与 **`nav_att`** (10Hz, 栈 4096), 仅在 `NAV_ENABLE=1` 时创建, `nav_task` (4/3072) 由 nav 组件内部创建; 启动横幅括注改为 "(惯导模块/PCA9685/云台MPU 已关停)"。**⑦ 文档**: 版本头升 v6.0、§0/§3/§4/§7 更新, §7.5 重写为"只剩云台 MPU6050", §7.8 改写为"GPS 数据来自惯导模块"(原 NMEA 内容存为历史留档), **新增 §7.10 惯导模块 (nav)** 全章, §7.9 命令表更新, §10/§11 与 README 同步。**⑧ 权威依据**: 仓库内 `10轴IMU模块通讯协议.pdf` (34 页: 寄存器表/帧格式) 与 `STM32-串口应用例程(IMU)/` 的**维特官方 SDK** (`wit_c_sdk.c/h` + `REG.h` + STM32 例程 `Main.c`, 波特率扫描与寄存器读写流程即照此实现)。**已完成** (同步 §0 / 版本头 / §3 / §4 / §7 / §7.5 / §7.8 / §7.9 / §7.10 / §10 / §11 + README) |
| **2026-09** 🟡 | **v6.1 新增控制台 `g` 命令 (按角度控制云台)** | **① `main.c` 新增 `cmd_goto()` 并注册 `g`**: `g <ch> [deg]` —— 按**标称角**调云台, 直通 `servo_set_angle()`, **受"标定 + 软件行程限位"约束** (ch1 出厂限位 30~150°; 越界时回显"被软件限位钳住, 实际 X°"), **不带 `deg` 时只打印当前角度/脉宽/可用窗口、不动作**; 与 `p` (直接给脉宽、**绕过限位**, 测机械行程/端点用) 分工明确, 补上"只能按脉宽控制、按角度得手算"的缺口。**② 背景**: 云台 MPU 用途未定, `TEST_MODE` 置 `0` (云台纯开环、开机不自动动作、不依赖 MPU 反馈), 故需要一条按角度的手动控制命令。**已完成** (同步 §7.9 命令表 + 版本头) |
| **2026-09** 🔴 | **v7.0 惯导模块 UART 引脚 16/18 → 1/2** | **① 现象**: 模块接在 **UART2 (RX=IO16 / TX=IO18)** 上时 `nav` 报 `模块无应答`, 且判据 `rx_bytes == 0` 成立 (7 档波特率 9600~230400 全试过) ⇒ **物理层没有字节进来** (驱动本身是裸计数, 波特率错也会有乱码字节)。**② 已排除**: 全工程只有 `nav.c` 占用这对脚 (`uart_set_pin(UART2, tx=18, rx=16)`), 无第二使用者; 模块 5V 供电 (取自 L298N 的 78M05 输出, 红灯亮)、TX/RX 已交叉、PCB 侧无改动。**③ 改动**: `nav_get_default_config()` 的 `tx_gpio / rx_gpio` 由 **18 / 16** 改为 **2 / 1** (`nav.h` 接线说明、`main.c` 的 4 条提示日志与启动日志同步), 即**直接复用原 GPS 的 GPIO1(RX) / GPIO2(TX)** —— 这两脚在旧 GPS (NMEA/UART1) 上**实测跑通过 UART**, 且 **GPIO1/2 都不是 strapping 脚** (GPIO3 原 PPS 是 strapping, 仍空闲)。**④ 说明**: 只改默认配置值, 若要回退到 16/18 改这两个数即可 (16/18 现空闲); 文档 §0 / §4 / §7.10 / §11 已同步。**已完成** |
| **2026-09** 🔴 | **v8.0 I2C1 的 SDA 由 GPIO21 挪到 GPIO16 (GPIO21 = 板载 WS2812 数据脚)** | **① 现象**: 板子上有一颗**常亮刺眼的白光**。查证: Waveshare ESP32-S3-ETH 有**板载 WS2812 RGB LED**, 官方 wiki 的 `RGB_LED` 示例写明其**数据脚 = GPIO21** (本仓库 `ESP32-S3-ETH-pinout.md` 原先把它当"通用 IO 可用", 未记录这一点)。而本项目把 **GPIO21 当作 I2C1 的 SDA** (PCA9685 + 云台 MPU 共用) ⇒ I2C 波形被 WS2812 当成**颜色数据**采样并锁存 ⇒ 显示**白色** (三通道全开 = 最亮), 且云台 MPU 以 **100Hz** 读取时每秒被重写 100 次 ⇒ **软件无法熄灭** (WS2812 会一直保持最后锁存值, 不掉电不清除)。**② 改动 (挪线)**: `servo_get_default_config().sda_gpio` **21 → 16**, `SCL` 仍为 **17** (只挪一根线; GPIO16 是 v7.0 惯导改到 1/2 后空出来的脚); `imu.c` 的 `imu_get_default_config().sda_gpio` 同步为 16 (仅"自装总线"时生效, 实际总线由 servo 安装)。**③ 改动 (禁灯)**: `main.c` 新增 `#define BOARD_RGB_LED_GPIO 21` 与包含 `driver/gpio.h`; `start_components()` 新增**第 0 步**把 GPIO21 `gpio_config` 为 **输出低电平**并保持 —— 不再有任何边沿进入 WS2812, 它保持上电默认的熄灭。⚠️ **必须断电重上一次**那颗灯才会真的灭。**④ 未做**: 没给它写 WS2812 驱动 (将来要做状态灯必须用 **RMT**, 位宽容差 ±150ns, 普通 GPIO 打拍不达标)。**已完成** (同步 §4 / §7 / README / `ESP32-S3-ETH-pinout.md` / `docs/legacy/status_led_方案.md`) |
| **2026-09** 🟢 | **v8.0.1 更正: 板载 WS2812 数据脚是 GPIO26 (非 GPIO21), 26 也已钉低** | **① 更正**: v8.0 依据 Waveshare 官方 wiki 的 `RGB_LED` 示例把板载 RGB 数据脚判为 **GPIO21**; 但**用户按原理图/实测确认实际是 GPIO26** —— 且 GPIO26 **不在 40 Pin 排针上**, 属板内网络 (与 wiki 描述不符, 可能对应别的板型/版本)。**② 改动**: `main.c` 宏改为 `BOARD_RGB_LED_GPIO_A = 26` (确认的灯脚) + `BOARD_RGB_LED_GPIO_B = 21` (双保险), 启动第 0 步改成循环把**两脚都钉为输出低**; 注释写清两个候选与"WS2812 不掉电不清锁存 ⇒ 必须断电重上一次"。**③ 非破坏性**: 该改动**不需要动任何外部接线** (26 未引出排针) ⇒ 按规则记 🟢。**④ 遗留待定**: v8.0 把 I2C1 的 SDA 从 21 挪到 16, 既然 21 不是灯脚, 这一步**并非必要**; 是否回退 (回退属"改接线"的 🔴) 由用户决定。**已完成** (同步 §4 两行 / `ESP32-S3-ETH-pinout.md` §二.5) |
| **2026-09** 🟢 | **v8.0.2 修复 nav 配置读回的"迟到应答串台" (会白写模块 Flash)** | **① 现象 (首次联调实测)**: 日志出现 `配置输出: RSW 0x058F→0x058F, RRATE **0x58F**→0x05` —— RRATE 的**读回值成了 RSW 的值 (0x058F)**, 于是驱动误判"配置不符" ⇒ 执行 `解锁→写→保存`, **每次上电都白写一次模块 Flash** (而"只在必要时才写"正是当初的设计目的)。**② 根因**: 维特的 **0x5F 读寄存器应答不带地址字段**, 只带"**从被读地址起的 4 个连续寄存器**"; 原实现**连发两条读指令** (先读 RSW、再读 RRATE), 前一条的**迟到应答**被后一条当成自己的 ⇒ 两个值都变成 RSW。**③ 修复**: 新增 `nav_read_regs(reg, out[4])` —— **一条指令读 0x02 就同时取回 RSW(blk[0]) 与 RRATE(blk[1])**, 读回比较与写后回读验证全部改用它 (少一次往返 + 竞态消失); 原 `nav_read_reg()` **已删除** —— 改完后它没有任何调用者, 留着会让 GCC 报 `-Wunused-function`; `s_read_val` 改为 `s_read_w[4]` (存整块)。**已完成** (同步 §7.10) |
| **2026-09** 🟡 | **v8.1 启用主控 MPU 状态回传 (20Hz, type=0x01) + 板载灯只认 GPIO26** | **① 需求**: 上位机 `tools/tcp_console.py` 的「主控 MPU」面板一直显示 `--` (v5.12 起回传只是空实现 `tcp_server_forward_mpu_to_host()`), 用户明确"**MPU 的状态回传/解算还是需要的**"。**② 实现 (不新增任务)**: `attitude_task` (100Hz, 读云台 MPU6050) **顺手**把原始 6 轴按 §7.5.5 换算成 int16 (`accel ×16384` / `gyro ×131`) 存进快照 `s_mpu_raw[6]`; `tcp_server_task` 循环里新增 `tcp_poll_send_local_mpu()` —— 每 `MPU_PUSH_PERIOD_US`(**50ms = 20Hz**) 取快照 → `control_build_mpu_frame(type=0x01)` → `wiz_send()` 发给 8080 上的上位机。⚠️ 之所以**不在 TCP 任务里直接读 I2C**, 是为了避免与 100Hz 的 `attitude_task` 抢同一条 I2C1; **MPU 未就绪时不发帧** (面板显示 `--`, 符合预期)。**③ 顺手补全**: `tcp_server_forward_mpu_to_host()` 由空实现改为**真透传** (收到远端 `0xBB 0x66` 帧原样转发给上位机) —— 8081 远端一上线即可用 (type=0x02 远端 MPU / type=0x03 v3.0 沉浮状态)。**④ 板载灯简化**: 既然已确认灯在 **GPIO26**, 去掉 v8.0 加的 GPIO21"双保险"钉低, **GPIO21 恢复为空闲脚** (宏 `BOARD_RGB_LED_GPIO_A/B` → 单个 `BOARD_RGB_LED_GPIO = 26`)。**已完成** (同步 §0 / §3 / §4 / §5 / §7.5.5 / `ESP32-S3-ETH-pinout.md` / `tools/tcp_console.py`) |
| **2026-09** 🔴 | **v9.0 4 路电调左右对调 (GPIO42 = 右推进)** | **① 缘由**: 用户实机**把 GPIO42 接在右边** (原代码里 42 = 左推进), 且报"右边电机初始化不了", 要求直接把左右对调。**② 改动 (仅软件映射)**: `motor_get_default_config()` 的 `esc1_gpio / esc2_gpio / rev1_gpio / rev2_gpio` 由 **42 / 41 / 40 / 39** 改为 **41 / 42 / 39 / 40** —— **ESC1 = 左推进 IO41 / ESC2 = 右推进 IO42 / REV1 = 左反推 IO39 / REV2 = 右反推 IO40** (LEDC 通道分配不变, 仍是 ESC1→Ch0 / ESC2→Ch1 / REV1→Ch2 / REV2→Ch3); `main.c` 的 `ESC_LEFT_GPIO`·`ESC_RIGHT_GPIO`·`REV_LEFT_GPIO`·`REV_RIGHT_GPIO` 4 个宏及全部注释/提示/启动日志同步 (差速混合公式与 `esc_apply_side()` 逻辑**不变**, `l 30` 仍是左推进 30%, 只是左现在走 IO41); `motor.c` / `motor.h` / `control.h` / `tools/tcp_console.py` 的引脚注释同步。**③ 说明**: **不必改任何接线** (纯映射对调, 原注释就写了"某一路装反了把成对 ID/GPIO 对调即可"); 只对调单侧同理。**④ 版本**: 改引脚/接线属**破坏性变更** ⇒ 按现行规则 **🔴 第一位 +1 (v8.1 → v9.0)**。**已完成** (同步 §2 差速映射 / §4 引脚表 / §4.1 LEDC 表 / §7.9 命令表 / §11 启动日志 + README) |
| **2026-09** 🟡 | **v9.1 新增 GPS 状态上报 (type=0x04 / 0x05, 2Hz) + 上位机「GPS」面板** | **① 需求**: GPS 以前只能在控制台 `gps` 看, 上位机 `tools/tcp_console.py` 没有 GPS 面板。**② 协议 (新增 type, 帧长不变)**: 沿用 16B 状态帧族 (0xBB 0x66), 新增 **type=0x04 (GPS 定位)**: [3-6] lat int32 ×1e7 / [7-10] lon int32 ×1e7 / [11-12] alt int16 ×0.1m / [13] 卫星数 u8 / [14] flags (bit0=定位有效 bit1=时间有效); **type=0x05 (GPS 运动)**: [3-4] 航向 u16 ×0.01° / [5-6] 地速 u16 ×0.01km/h / [7] pdop / [8] hdop / [9] vdop (u8 ×0.1, 上限 25.5) / [10-12] 时·分·秒 / [13] flags / [14] 保留。**帧长仍是 16B、CRC 仍是前 15 字节异或** ⇒ 老上位机只是不认这个 type, 不会解析出错。**③ 固件实现**: `control` 组件新增 `control_build_gps_pos_frame()` / `control_build_gps_nav_frame()` (只收标量, **不依赖 nav 组件**); `main.c` 新增 `tcp_poll_send_gps()` (`GPS_PUSH_PERIOD_US = 500ms`, 由 `tcp_server_task` 循环调用, 与 MPU 回传同一套"断开则重连后立刻发"的逻辑), 数据取 `nav_read_gps()` 快照 —— 该函数只加锁拷结构体, **不在 TCP 任务里碰 UART**, 与 MPU 回传同理; `flags` 由 `nav_gps_t.valid` / `.time_valid` 组装, DOP 用 `dop_to_u8()` (×0.1, 截顶 25.5) 定标, 经纬度用 `lround` (double → int32, 避免 float 存不下 3e8 级别的整数)。**④ 条件编译**: 整段包在 `#if NAV_ENABLE` 里 —— 关掉惯导时连 `tcp_poll_send_gps()` / `dop_to_u8()` / `s_gps_next_us` 都不编译 (不会有"未使用"警告)。**⑤ 客户端**: `tools/tcp_console.py` 新增常量 `TYPE_GPS_POS`/`TYPE_GPS_NAV` + `GPS_FLAG_FIX`/`GPS_FLAG_TIME`、解析辅助 `r_i32()`/`r_u16()`、中间栏「GPS」面板 (定位+卫星数+flags / 纬度 / 经度 / 海拔 / 航向+地速 / DOP / 模块时间 / 更新时刻), `_handle_frame()` 增加两个分支。**⑥ 未定位时**: 驱动已把经纬度清零, 面板据 `flags.bit0` 把纬度/经度/航向/地速显示为 `—` (海拔/卫星数/DOP/时间照常显示)。**已完成** (同步 §0 / §2.1 / §2.4 / §7.10.8 / §8 / §10 / §11 + README + `tools/README.md` + `tools/tcp_console.py`) |
| **2026-09** 🟡 | **v9.2 云台新增上位机手动控制 (cmd=0x12 SERVO) + 客户端两个拖动条** | **① 需求**: 舵机此前只能从**串口控制台** (`p`/`g`/`n`/`z`) 摆位, 上位机没有任何舵机通路 (v4.0 把帧里 [8][9] 的舵机字节删了)。**② 协议 (新增 cmd, 帧长不变)**: 16B `0xAA 0x55` 控制帧族新增 **cmd=0x12**: `[3-4]` ch0 目标**标称角** ×0.1° (int16 LE) / `[5-6]` ch1 标称角 ×0.1° / `[7]` **通道掩码** bit0=ch0·bit1=ch1 / `[10]` flags bit0=本地执行 / `[15]` CRC8 (前 15 字节异或)。**只有掩码置位的通道动作** ⇒ 老上位机不发此 cmd、或误发 `mask=0`, 都**不会有任何舵机动作** (天然向后兼容, 非破坏 ⇒ 🟡)。**③ 固件**: `control` 组件解析该 cmd (字节 3-6 与 DEPTH 共用, 由 `[2]` 区分) 并通过 `control_set_local_callback()` 交给 `main.c`; `main.c` 新增 `servo_apply_from_host()` —— 只对置位通道调 `servo_set_angle()` (**与控制台 `g` 同一条路**, 受"标定 + 软件行程限位"约束, ch1 越界由固件钳到 30~150°), **只在角度变化时才写 I2C** (拖动条会连发同一值), 失败只打印一次 (避免刷屏)。**④ 客户端**: `tools/tcp_console.py` 新增常量 `CMD_SERVO`、构造器 `build_servo_frame(deg0, deg1)` (None = 该通道不下发)、左栏「**云台手动**」面板 —— ch0/ch1 两个 **ttk.Scale 拖动条** (0~360° / 30~150°) + 数值显示 + **「启用下发」勾选** (不勾时只改本地显示, 免得误碰抢走云台) + 松手记一行日志; **拖动即时 `send_now`**, 不走 20Hz 控制帧循环; 重连后清空"已下发"记录。**⑤ 顺手修的真 bug**: 拖动条走 UI 线程 `send_now`、20Hz 帧走发送线程, 两者并发 `sendall` **同一 socket 会交错字节 ⇒ 撕坏 16B 帧** (控制帧被 CRC 丢弃, 严重时触发 500ms 失联保护) —— 已给 `TcpLink` 加发送互斥锁 (`_tx_lock`), `send_now()` 与 `_tx_loop()` 都走它。**⑥ 文档**: §2.1/§2.2 新增 cmd=0x12 布局与兼容说明, §7.9 补"两条手动路径互相覆盖"提醒, §8 补常量/字段, 另订正 §1 两处旧实况 (舵机已随 PCA9685 启用、MPU/GPS 回传已启用)。**已完成** (同步 §0 / §2.1 / §2.2 / §7.5 / §7.9 / §8 / §10 / §11 + README + `tools/README.md` + `tools/tcp_console.py`) |

---

## 11. 构建与烧录

### 推荐: 项目根目录 `idf.ps1` (Windows PowerShell, 自动探测 IDF 路径)

```powershell
.\idf.ps1 build                    # 编译
.\idf.ps1 -p COM3 flash monitor    # 烧录并监视
.\idf.ps1 -p COM3 monitor          # 只监视
```

### 手动 (与上面等价)

```powershell
. $env:IDF_PATH\export.ps1
idf.py build
```

> 环境: ESP-IDF **v5.5.4**。烧录与日志走 **板载原生 USB (USB-Serial/JTAG)** —— 插板子**原生 USB** 那个 Type-C 口, 115200 8N1
> (`sdkconfig.defaults`: `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` + `CONFIG_ESP_CONSOLE_SECONDARY_NONE=y`; UART0 已不再输出日志)。
> ⚠️ 改这两个 console 选项后**必须删掉 `sdkconfig` 重新 build** —— `sdkconfig.defaults` 只对 `sdkconfig` 中不存在的项生效 (否则改动静默不生效, 同 16MB Flash 那个坑)。

### Flash 尺寸 / 分区 / 构建注意 (v5.11.1)

| 项 | 值 | 说明 |
|---|---|---|
| 板载 Flash | **16 MB** (W25Q128FS) | `sdkconfig.defaults`: `CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y` / `"16MB"` |
| 分区表 | `SINGLE_APP` (默认 `partitions_singleapp.csv`) | app 分区固定 **1 MB** |
| 当前 app 占用 | ≈ `0x51890` (~333 KB) | 剩余约 68% (**v5.11.1 编译结果**; v5.11.2 启用 W5500 后会略增, 待重编后更新) |

> ⚠️ **`sdkconfig.defaults` 只对 sdkconfig 中不存在的项生效**。若 `sdkconfig` 已生成旧值 (如 `CONFIG_ESPTOOLPY_FLASHSIZE_2MB`), 改 defaults **不会覆盖** —— 必须**删除 `sdkconfig` / `sdkconfig.old` 后重新 configure/build**, 16MB 才生效 (`flasher_args.json` 的 `--flash_size` 随之变成 `16MB`)。
>
> ⚠️ `build/` 记录了工程**绝对路径**: 工程目录换盘符/路径后必须删除 `build/` 重编, 否则报 `Build directory ... configured for project ... not ...`。
>
> ⚠️ 若 `.\idf.ps1 build` 报 `No module named 'click'`, 说明 PATH 上的 `python` 不是 IDF 的 venv。应使用 `<IDF_TOOLS_PATH>\tools\python\<版本>\venv\Scripts\python.exe` 直接运行 `idf.py` (或把该 `Scripts` 目录置于 PATH 最前)。

预期启动输出 (当前模式: `W5500_ENABLE=1` / `MOTOR_ENABLE=1` / `NAV_ENABLE=1` / `SERVO_ENABLE=1`;**临时关停: `MPU6050_ENABLE=0` (云台 MPU 废弃) + `REV_ESC_ENABLE=0` (反推关闭)**; 且 `TEST_MODE = 0` = 云台**开环** (不自动动作、不跑闭环), 参考):
```
=== [当前模式] W5500 + TCP Server 8080(上位机) + 水面电调(推进IO41/42, 反推已关闭) + L298N滚筒(IO38/48/47) + 串口控制台 (已关停: 云台MPU) ===
start_components 开始执行
NVS 初始化成功
W5500 初始化成功 (IP 192.168.29.10/24, 网关 192.168.29.1)
初始化 PCA9685...
PCA9685 初始化成功, 已回中: ch0=<标定中位> us, ch1=<标定中位> us (<标定中位>°)
云台运动规划初始化成功
云台 MPU6050 已暂时关停 (MPU6050_ENABLE=0): 不初始化, 姿态解算任务不启动     ← MPU 废弃时的替代行
初始化惯导模块 (UART2: ESP32 RX=IO1 <- 模块 TX, TX=IO2 -> 模块 RX)...
nav: 惯导模块 UART2 已就绪 (RX=IO1 <- 模块 TX, TX=IO2 -> 模块 RX, 9600 8N1)
nav: 波特率扫描命中: 9600 bps
nav: 输出配置已是期望值 (RSW=0x058F RRATE=0x05), 不写模块 Flash     ← 首次/不符时改为 "配置输出: RSW 0x001E→0x058F, RRATE 0x06→0x05 (解锁→写→保存)" + "输出配置写入成功并已保存"
惯导模块初始化成功 (GPS + 10 轴 IMU)
电调就绪 (4 路, 已置最低油门 1100us 停): 推进 IO41/IO42, 反推 IO39/IO40
反推暂时关闭 (REV_ESC_ENABLE=0): 负油门按停处理; 方向线 IO39/IO40 保持 1100us 正向半区     ← 写操作, 见 §0
控制台: l/r <0~100> 或裸数字 (正=推进; 负值按停处理); 例: l 30 / r 30 / 0
L298N 已初始化并停机 (ENA=IO38, IN1=IO48, IN2=IO47), 本轮不测试
等待网线连接...
=== 网线已连接 ===
本机 IP: 192.168.29.10
子网掩码: 255.255.255.0
默认网关: 192.168.29.1
TCP: === TCP 服务器已启动: 8080 (上位机); 失联保护 500 ms ===
TCP: 状态回传: 主控 MPU 20Hz (type=0x01) + GPS 2Hz (type=0x04 定位 / 0x05 运动)
[CH0 TEST] 扫描测试已就绪: <起>~<止> us, 步进 <..> us, 每档停留 <..> ms (默认关闭, t0 1 开启)
[CH1 TEST] 扫描测试已就绪: <起>~<止>°, 步进 <..>°, 每档停留 <..> ms (默认关闭, t1 1 开启)
[AHRS] 三维姿态解算任务启动 (100Hz, Madgwick β=<..>)     ← 仅 MPU6050_ENABLE=1 时出现 (当前无此行)
串口控制台已启动 (提示符 gimbal>): 输 help 看全部指令
直接输入数字 (100~3000) = 把 ch0 定死在该脉宽 (us)        ← `SERVO_ENABLE=1` 时裸数字的含义
输入 help 查看全部指令
gimbal>
=== [当前模式] 启动完成: W5500 + TCP Server 8080(上位机, 失联保护 500ms) + 水面电调(推进 IO41/42 + 反推方向线 IO39/40) + L298N滚筒 + 串口控制台 ===
```
> ⚠️ **`TEST_MODE = 0` (当前) 的好处与代价**: **没有** `[STAB] 目标=... 实测=... 误差=...` 闭环打印, 也没有 `[CH1 SCAN]` / `[C1-MPU]` / `[C0-MPU]` / `[GIMBAL TEST]` (它们分别属于 `TEST_MODE=4/2/3/1`)。云台此时只"开机回中一次、之后不动", 你手动给的任何角度都不会被抢走 —— 想扫行程/测位置直接 `p <ch> <us>` / `n` / `z` / `t0 1` / `t1 1`。要用闭环才把 `TEST_MODE` 改回 `4` (或临时 `ge 1 1`, 但那需要云台 MPU 反馈有效)。
>
> ⚠️ **v6.0 起不再有任何 GPS / 亚博 10 轴 IMU 日志** —— 旧的 `GPS 已关停 (GPS_ENABLE=0): 不初始化 UART1/PPS...`、`船体亚博 IMU 已关停 (HULL_IMU_ENABLE=0): 不装 UART2...` 都随组件一起删除; 惯导模块现在是上面那 4 行 (`初始化惯导模块...` → `惯导模块初始化成功 (GPS + 10 轴 IMU)`)。
> 上位机接上后 (`tools\tcp_console.py` 连 192.168.29.10:8080) 会多出 `TCP: [HOST] 上位机已连接 (8080)`;
> 断开显示 `[HOST] 上位机已断开 (n=...)` 或 `连接关闭 (SR=0x..), 重新监听 8080`;
> 上位机崩溃/网线松脱 → `TCP: 上位机失联 >500 ms ⇒ 已全部停机 (推进/反推/滚筒归零)`。控制台 `net` 可随时查这些状态。
>
> `W5500_ENABLE=0` 时第 4 行变为 `以太网已关停 (W5500_ENABLE=0): 不初始化 W5500, 不等 link up, 不启动 TCP Server`, 且**没有** `等待网线连接...` 与 `TCP 服务器已启动` 两段, 启动横幅也换成"以太网已关停"版本。
>
> **`NAV_ENABLE=0` 时 (关停) 的差异**: 上面那 4 行 (`初始化惯导模块...` / `nav: 惯导模块 UART2 已就绪...` / `nav: 波特率扫描命中...` / `惯导模块初始化成功...`) 合并为一行
> `惯导模块已关停 (NAV_ENABLE=0): 不装 UART2, GPS 与船体姿态都不可用` —— 不装 UART2、不扫描波特率、不写模块配置, 也不创建 `nav_gps` / `nav_att` 两个日志任务; 控制台 `gps` / `hull` / `nav` 都只提示无数据。
>
> 同时新建两个日志任务 (见 §3): **`nav_gps`** (1Hz, 打 `[GPS] 定位…` / `[GPS] 纬度…`) 与 **`nav_att`** (10Hz, 打 `[HULL] rpy=(...) a=(...)g g=(...)dps T=..C`) —— **两者默认静默**, 输 `gps 1` / `hull 1` 才打印。控制台用 `nav` 看模块状态 (在线/波特率/RSW+RRATE/帧计数)。
>
> 若 `NAV_ENABLE=1` 而模块没接: `gps` 提示 `惯导模块无数据: 查 UART2 接线 (ESP32 RX=IO1 <- 模块 TX, TX=IO2 -> 模块 RX)、共地、模块 3.3V`, `hull` 提示 `惯导模块无姿态数据: ...`, `nav` 给出"一字节都没收到"的排查建议 (判据见 §7.10.7)。
>
> 云台两条线当前是 **`SERVO_ENABLE=1` + `MPU6050_ENABLE=0`**: 所以 `初始化 PCA9685...` → `PCA9685 初始化成功, 已回中...` → `云台运动规划初始化成功` 会正常出现 (**没有** `初始化云台 MPU6050` / `云台 MPU6050 初始化成功` / `[AHRS] 三维姿态解算任务启动` —— MPU 已暂时废弃, 见 §0), 控制台裸数字含义是 `直接输入数字 (100~3000) = 把 ch0 定死在该脉宽 (us)`。
> 启动横幅的"已关停"括注 (off_note) 会**按开关自动生成**, 当前含 `云台MPU/` (即 `(惯导模块?/PCA9685?/云台MPU/...)` 里只有它)。
> 两个开关各自的差异见本段末尾的备注 (v6.0.1 起启动横幅的"已关停"备注也是按开关**自动生成**的, 不会再和实况脱节)。

> ⚠️ `W5500_ENABLE=1` 且网线未插时会出现 `=== 网线未连接 (30s 超时), 继续启动 ===` 并**照常继续启动** (只告警, 不中断); 此时 TCP Server 仍会启动, 但插上网线前连不上。
> ⚠️ **不会**出现 8081 (REMOTE) 相关日志 —— 远端转发仍未启用 (§0 / §6); **状态回传**只在**上位机连上 8080 之后**才有 —— 启动时会打一行 `TCP: 状态回传: 主控 MPU 20Hz (type=0x01) + GPS 2Hz (type=0x04 定位 / 0x05 运动)`, 之后按帧率发 `type=0x01` (MPU) 与 `type=0x04/0x05` (GPS, 详见 §7.10.8)。
> ⚠️ **`MPU6050_ENABLE=0` (当前)** 时: **无** `初始化云台 MPU6050` / `云台 MPU6050 初始化成功` / `[AHRS] 三维姿态解算任务启动` 三条日志, 改为一条 `云台 MPU6050 已暂时关停 (MPU6050_ENABLE=0): 不初始化, 姿态解算任务不启动`; `imu` 报 `gimbal 未就绪`; 上位机「主控 MPU」面板恒 `--` (回传自动停发); 云台闭环 `ge`/`gt`/`gp` 无反馈源。**舵机本身不受影响** (`SERVO_ENABLE=1`, `p`/`g`/`n`/`z` 与上位机拖动条都正常)。
> ⚠️ **把 `SERVO_ENABLE` 也置 `0` 后**: **无** `初始化 PCA9685` / `PCA9685 初始化成功` / `云台运动规划初始化成功` / `[STAB] 目标=...` 等日志; `[CH0 TEST]` / `[CH1 TEST]` / `[STAB]` 三条会变成 `PCA9685/servo 未就绪 (SERVO_ENABLE=0 或硬件异常) => ...不启动`, 三个舵机测试任务启动即自退出。
> 关停期间控制台提示符 `gimbal>` 仍在, 但**所有舵机/闭环命令 (`p`/`n`/`z`/`t`/`sc`/`sl`/`ck`/`sv`/`rs`/`ge`/`gt`/`gp`/`t0`/`t1`) 都是空操作** (`st`/`gs` 仍可查看标定与状态; `scan` 扫 I2C1 只见云台 MPU、无 PCA9685; `imu` 显示 `gimbal 未就绪`); 裸数字含义回退为电机的 `-100~100`。
> ⚠️ `NAV_ENABLE=0` 时: **没有** `nav_task` / `nav_gps` / `nav_att` (§3), 因此无参 `gps` 会提示 `惯导模块无数据: 查 UART2 接线 ...`、无参 `hull` 提示 `惯导模块无姿态数据: ...`、`nav` 显示 `无数据 / 字节=0`; `gps 1` / `hull 1` 仍可输入但不产生日志 (对应任务未创建); 同时 **GPS 状态上报整段不编译** (§7.10.8), 上位机「GPS」面板一直 `--`。

---

> 📝 文档变更需同步更新所有相关代码并测试.
>
>   最后修改: **v9.2** (① **云台新增上位机手动通路 (cmd=0x12 SERVO) + 客户端两个拖动条** (§2.2 / §7.9); ② 客户端给发送加互斥锁 (修"拖动条与 20Hz 循环并发写 socket 撕裂帧"的真 bug); ③ 订正 §1 两处旧实况 (舵机已启用、MPU/GPS 回传已启用); ④ 临时开关变化: **云台 MPU6050 暂时废弃** `MPU6050_ENABLE=0` —— **临时开关, 只记在 §0 / §7.5 / §11**。**已完成**)
>
>   v9.1 摘要: **新增 GPS 状态上报** —— `type=0x04` 定位 / `type=0x05` 运动+精度+时间, 各 2Hz 发给上位机「GPS」面板 (§2.4 / §7.10.8); ② **反推暂时关闭** `REV_ESC_ENABLE=0` (负油门按停处理, 方向线只保持正向半区; **临时开关, 只记在 §0 与功能章节**); ③ 客户端 `tools/tcp_console.py` 新增 GPS 面板 + `r_i32()`/`r_u16()`; ④ 顺带订正文档陈旧项 (惯导 UART 16/18→1/2, I2C1 IO21/17→IO16/17, 目录结构与"状态回传未启用"等描述)。**已完成**)
>
>   v9.0 摘要: **4 路电调左右对调 —— GPIO42 = 右推进** (左推进 IO41 / 右推进 IO42 / 左反推 IO39 / 右反推 IO40; 只改软件映射, 不用重接线)。🔴 第一位 +1。**已完成** (同步 §2/§4/§4.1/§7.9/§11 + README)
>
>   v8.x 摘要: **v8.1** 启用主控 MPU 状态回传 (`type=0x01`, 20Hz) + 把 `tcp_server_forward_mpu_to_host()` 改成真透传 + 板载灯只认 GPIO26 (GPIO21 恢复空闲) / **v8.0.2** 修 nav 配置读回的"迟到应答串台" (原先每次上电白写一次模块 Flash) / **v8.0.1** 更正板载 WS2812 数据脚是 GPIO26 (非 GPIO21) / **v8.0** I2C1 的 SDA 由 GPIO21 挪到 GPIO16。
>
>   v6.0 摘要: (① **惯导替换** —— 删除旧 `components/gps` (NMEA-0183 / UART1 2-1-3 / PPS) 与旧"亚博 10 轴 IMU"(`7E 23` 请求式 / UART2) 两套驱动, 新增 **`components/nav`** 驱动**亚博 GPS+10 轴 IMU 一体惯导模块** (维特 WIT `0x55` 协议主动上报, 单条 UART2: RX=IO1 / TX=IO2 @9600, 11B 定长帧 `0x50`~`0x5A`); ② `imu` 组件精简为**只剩云台 MPU6050** (`imu_role_t` 只剩 `IMU_ROLE_GIMBAL`, 删 `7E 23`/I2C 双后端与两个开关); ③ `main.c` 的 `GPS_ENABLE` + `HULL_IMU_ENABLE` **合并为 `NAV_ENABLE`** (**现况见 §0: 当前为 `1`**), 控制台 `gps`/`hull` 改读惯导快照 (**去掉 `gps raw|baud|send` 与 `hull addr`**)、**新增 `nav`**, `imu` 只读云台、`scan` 只扫 I2C1; ④ 任务 `gps_log`/`hull_log` → **`nav_gps`**(1Hz) / **`nav_att`**(10Hz) + `nav_task`(4/3072, 组件内部创建); ⑤ 初始化**波特率自扫描** (9600~230400) + **按需写** RSW=`0x058F` / RRATE=`0x05`(5Hz) + SAVE + 回读验证 (避免每次上电写模块 Flash); 模块没接只返回 `ESP_ERR_NOT_FOUND`, 接上后每 5s 自动重试、不必重启; ⑥ 依据 `10轴IMU模块通讯协议.pdf` 与维特官方 SDK / STM32 例程。**已完成** (同步 §0 / 版本头 / §3 / §4 / §7 / §7.5 / §7.8 / §7.9 / §7.10 / §10 / §11 + README)
>
>   v5.12 / v5.11.2 摘要: v5.12 **启用网络接收上位机操控** (新建 `tcp_server_task` 只监听 8080 + 差速混合与电机驱动迁出 `control` 到 `main.c` + 500ms 失联保护 + 控制台 `net`) / v5.11.2: ① 启用板载 W5500 —— `main.c` 取消注释 `wiznet_manager_init()` + link up/打印 IP, 静态 192.168.29.10/24, **TCP 收发仍全部注释**; ② 云台闭环新增**无反馈保护** —— 200ms 收不到姿态反馈即停 PID 并把该轴回标定中位 (ch1=90°), 不再拿假 0° 顶到限位下限, 新增 `nofb` 状态/日志标记; ③ **修复: PCA9685 探测失败不再删 I2C1 总线** —— 总线保留给船体 10 轴 IMU, servo 功能关闭但亚博 IMU 正常可用、可在线重探; ④ 清理换板遗留: 删除 §11 状态指示灯全章并顺延为 §11 构建与烧录, 内容归档 `docs/legacy/`, 更正 servo.h 旧板引脚注释与 pinout.md I2C 推荐脚; ⑤ GPS 诊断增强 —— `gps_data_t.rx_bytes` + 控制台 `gps` 打印 `收到字节/语句/错误` 并自动给结论, 确认 ATGM336H 兼容; ⑥ 启用 L298N 直流电机 (只初始化 L298N 通道, 4 路电调 gpio 置 -1) + 控制台 `m <-100~100>` 调速; ⑦ **console 改走板载原生 USB (USB-Serial/JTAG)** —— 修复"插原生 USB 只能看日志、敲不进命令", 插原生 USB 口即日志+交互+烧录同口 (需删 `sdkconfig` 重编); ⑧ **亚博 10 轴 IMU 由 I2C 改接 UART** —— 厂商 UART 示例证实该模块不走维特(WIT) I2C 协议 (自有协议 `7E 23` 帧头 @115200, `0x80` 请求式), 这就是之前扫不到 0x50 的原因; `imu` 组件新增 UART 后端 (UART2, RX=IO1/TX=IO2, 接口不变、I2C 分支保留可回退), 云台 MPU6050 同时挪到 I2C1 与 PCA9685 共线 ⇒ 腾出的 16/18 给 UART, 两颗 IMU 的初始化顺序也随之调整 (须在 `servo_init()` 之后); ⑨ 修复 UART 角速度量纲错误 (`gyr_ratio` 多乘 `180/π` 放大 57.3 倍); ⑩ 修复 UART 欧拉角单位错误 (`0x26` 出弧度被当成度, 已乘 `180/π`); ⑪ GPS 按 NMEA-0183 补全 (`GSA`/`GSV`/`GST`/`VTG`/`ZDA`/`$GPTXT`, 未定位时坐标清零) + 控制台 `gps raw`/`gps baud`/`gps send`。**已完成** (同步 §0/§4/§7/§7.5~§7.9/§11/§10 + README)
>
>   历史: v5.11.1 修复 motor LEDC 通道冲突 (L298N DC 改 Channel4) + 16MB Flash 生效 + motor 注释更正 / v5.11.0 固件切至 [临时测试/标定模式] (新增 gps / servo 标定 / gimbal 闭环 / 串口控制台) / v5.10.0 换板 ESP32-S3-ETH + 同步远端 v3.0 / v5.9.0 ROCK 5C 视觉节点 / v5.8.0 删除 RDK X5 / v5.7.8 同步远端 v2.19 (`bucket_speed`) / v5.7.7 同步远端 v2.18 / v5.6 水上水下联合协议 v2.0
