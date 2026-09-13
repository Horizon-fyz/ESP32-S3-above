# ESP32-S3 水上机器人项目参数总览

> 📌 本文档为唯一项目参数权威来源, 所有代码改动须与本文档一致.
> 文档版本: **v5.11.1** (① 修复 `motor.c` **LEDC 通道冲突**: L298N DC 的 `dc_ledc_channel` 由 `LEDC_CHANNEL_0` 改为 **`LEDC_CHANNEL_4`** (Timer1 不变), 避免与 ESC1 (Timer0/Channel0) 抢占同一通道; ② 删除 `sdkconfig`/`sdkconfig.old` 重新生成, 使板载 **16MB Flash** (`CONFIG_ESPTOOLPY_FLASHSIZE_16MB`) 从 `sdkconfig.defaults` 生效并全量重编译; ③ 更正 `motor.c` / `motor.h` 中过时的引脚注释与启动日志; 上一版 v5.11.0: ① 固件切换为 **[临时测试/标定模式]**: 以太网/Motor/control 协议**暂未启用**, 实跑模块为 云台 MPU6050 + PCA9685 + 船体10轴IMU + GPS + 云台运动规划/姿态闭环; ② 新增 `components/gps` 组件; ③ servo 组件新增**标定体系**(双角度刻度/行程限位/NVS 存取); ④ gimbal 组件新增**姿态闭环 PID 自稳**; ⑤ 新增**串口标定控制台**; 上一版 v5.10.0: 换板 Waveshare ESP32-S3-ETH + 同步远端 v3.0 沉浮协议 + 删除 `status_led`)
> 配套远端文档: [`..\ESP32-S3-below\ESP32-S3水下机器人项目参数总览.md`](../ESP32-S3-below/ESP32-S3水下机器人项目参数总览.md) (v3.0, 已完全更新)
> 新板引脚依据: [`ESP32-S3-ETH-pinout.md`](ESP32-S3-ETH-pinout.md)

---

## 0. 当前固件形态 (重要)

> ⚠️ **v5.11.1 现状**: 本工程固件当前为 **[临时测试/标定模式]** —— `main.c` 头部与 `start_components()` 均注明 "仅保留 PCA9685 + MPU6050, 其他全部注释", 用于云台/舵机/IMU/GPS 的标定与闭环调试。
>
> | 分类 | 内容 |
> |---|---|
> | **已启用** | 云台 MPU6050 (I2C0) + PCA9685 (I2C1) + 船体 10 轴 IMU (I2C1, 0x50) + GPS (UART1) + 云台运动规划 (gimbal) + 云台姿态闭环自稳 (gimbal PID) + Madgwick 姿态解算 + 串口标定控制台 |
> | **暂未启用** (代码保留, 初始化/调用已注释) | W5500 以太网、TCP Server (8080/8081)、motor 电机、control 协议解析与转发 |
>
> 因此下文 **§2 / §5 / §6 / §8 / §9 为设计参考** (协议与网络参数已定义, 待恢复), 不代表当前固件行为;
> **§1 / §3 / §4 / §7 / §7.5 及以下新增章节已按当前实况更新**。
> §1 中与网络链路相关的内容 (节点角色/拓扑/数据流向) 同样属于设计参考。

---

## 1. 系统架构

> ⚠️ 本章描述**完整系统设计** (含网络链路)。当前固件形态见 **§0**: 以太网/TCP/Motor/control 暂未启用, 本章中与网络相关的条目为**设计目标**。

### 1.1 节点角色

| 节点 | 角色 | 硬件 | 网络角色 |
|---|---|---|---|
| **主控制节点** (本项目) | 核心控制 + 数据处理 + 指令分发 | MPU6050 + 亚博10轴IMU + 4 ESC + 1 L298N + 2 舵机(PCA9685) + GPS | TCP **Server** (双端口) |
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
   │  + 转发 (16B 控制 → 8B 子帧)                              │
   │  + 转发 (16B DEPTH → 8B 沉浮子帧, v3.0)                   │
   │  + 推送 (本 MPU 20Hz → 主机+远端)                          │
   │  + 转发 (远端 MPU 20Hz → 主机)                            │
   │  + 转发 (远端沉浮状态 type=0x03 → 主机, v3.0)             │
   └────────────────────────────────────────────────────────────┘

   ── 内网 (LAN) 视频链路: 与主控节点无数据交互 ────────────────────
     网络摄像头 ──(RTSP/HTTP 视频流)──▶ ROCK 5C ──(AI 推理后视频流)──▶ 上位机 / 内网客户机
```

### 1.3 数据流向

> ⚠️ 下表为**设计目标** (当前固件未启用网络链路, 见 §0)。

| 链路 | 方向 | 数据类型 | 速率 |
|---|---|---|---|
| 主↔主机 (8080) | 上→主 | 16B 控制帧 (含 cmd=0x11 DEPTH, v3.0) | 按需 (主机主动发) |
| 主→主机 (8080) | 主→上 | 16B MPU 帧 (本+远端合并) | 20Hz |
| 主→主机 (8080) | 主→上 | 16B 沉浮状态帧 (type=0x03, v3.0) | 2Hz (远端上报) |
| 主→主机 (8080) | 主→上 | 500ms JSON 状态 (可选) | 2Hz |
| 主↔远端 (8081) | 主→远 | 8B 子帧 (2 远端电调) | 按需 |
| 主→远端 (8081) | 主→远 | 8B 沉浮子帧 (cmd=0x11, v3.0) | 按需 |
| 主→远端 (8081) | 主→远 | 16B MPU 帧 (本节点 MPU) | 20Hz |
| 远→主 (8081) | 远→主 | 16B 沉浮状态帧 (type=0x03, v3.0) | 2Hz |

> v3.0 **取消** `type=0x02`(远端原始 MPU, 原 20Hz): 远端姿态改由 YB-MRA02 内部融合后经 `type=0x03` 的 `pitch/roll` 承载.

---

## 2. TCP 协议规范 (设计参考, 当前未启用)

> ⚠️ 本章为**协议设计参考**: `control` 组件未初始化、`main.c` 中帧收发代码已注释 (见 §0), 当前固件不进行任何 TCP 通信。
> 代码侧帧常量/构造器 (见 §8) 与本章保持一致, 便于后续恢复。

### 2.1 帧类型总览

| 帧名 | 帧头 | 长度 | 方向 | 用途 |
|---|---|---|---|---|
| 控制帧 | 0xAA 0x55 | 16B | 上位机 → 主 | 完整控制指令 (本地+远端) |
| **沉浮控制帧** | **0xAA 0x55** | **16B** | **上位机 → 主** | **目标深度 cm + 俯仰° + 横滚° (cmd=0x11, v3.0)** |
| 转发子帧 | 0xAA 0x55 | 8B | 主 → 远端 | 远端电调 + 系统命令 |
| **沉浮控制子帧** | **0xAA 0x55** | **8B** | **主 → 远端** | **目标深度 cm + 俯仰° + 横滚° (cmd=0x11, v3.0)** |
| MPU 数据帧 | 0xBB 0x66 | 16B | 双向 | 6 轴 IMU 原始数据 (type=0x01/0x02) |
| **沉浮状态帧** | **0xBB 0x66** | **16B** | **远端 → 主 → 上位机** | **深度/俯仰/横滚/模式/三路油门 (type=0x03, v3.0)** |

### 2.2 控制帧 (16B) — 上位机 → 主控制节点 (v4.0 差速版)

| 字节 | 名称 | 类型 | 含义 |
|---|---|---|---|
| 0 | HEAD0 | uint8 | 0xAA (固定) |
| 1 | HEAD1 | uint8 | 0x55 (固定) |
| 2 | cmd | uint8 | 0x10=电机 / **0x11=沉浮控制 (v3.0)** / 0x20=急停 / 0x30=重启 / 0x40=关机 |
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
| 2 | type | uint8 | 0x01=本地 MPU / 0x02=远端 MPU / **0x03=沉浮状态 (v3.0)** |
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
> ⚠️ DEPTH (0x11) 命令主控**本地不执行**, 仅按 flags bit1 转发远端 (见 §2.2).

### 2.7 差速混合算法 (主节点本地 + 远端)

主节点收到控制帧 (speed, yaw) 后, 进行差速混合并执行本地电机:

```
left_esc  = clamp(speed + yaw, -100, +100)   →  本地 ESC1
right_esc = clamp(speed - yaw, -100, +100)   →  本地 ESC2
dc_speed  = |speed|                          →  L298N PWM (0~100%)
dc_dir    = sign(speed)                      →  L298N IN1/IN2
                                              0=停, +1=正, -1=反
```

远端收到 8B 子帧后, 做**完全相同**的混合:

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

## 3. FreeRTOS 任务清单 (v5.11.1 当前实况)

| 任务 | 文件 | 优先级 | 栈大小 | 周期 | 说明 |
|---|---|---|---|---|---|
| attitude_task | main.c | 5 | 4096 | 10ms (100Hz) | 读云台 MPU6050 + Madgwick 姿态解算; 发布 `s_att_*` 并 `gimbal_feed_attitude()` 喂闭环 |
| gimbal_task | gimbal.c | 4 | 3072 | 10ms (100Hz) | 云台运动规划 + 姿态闭环 PID 下发 (由 `gimbal_init()` 创建) |
| gps_task | gps.c | 4 | 4096 | UART 驱动 | NMEA0183 解析 (由 `gps_init()` 创建) |
| gps_log_task | main.c | 4 | 4096 | 1000ms | GPS 1Hz 日志 (默认静默, `gps 1` 打开; 仅 `gps_init()` 成功时创建) |
| hull_log_task | main.c | 4 | 4096 | 100ms (10Hz) | 船体 10 轴 IMU 日志 (默认开, `hull 0` 关) |
| servo_ch0_test_task | main.c | 4 | 3072 | 常驻 | ch0 (360°) 档位扫描 (默认关, `t0 1` 开启) |
| servo_ch1_test_task | main.c | 4 | 3072 | 常驻 | ch1 (180°) 角度扫描 (默认关, `t1 1` 开启) |
| gimbal_stab_test_task | main.c | 4 | 4096 | 500ms 打印 | 云台闭环自稳测试 (`TEST_MODE==4`), 开机自动开 ch1 俯仰闭环 |
| console_repl_task | main.c | 5 | 4096 | — | 串口标定控制台 (自建任务, 支持"裸数字"输入) |

> **`TEST_MODE` (main.c) 决定创建哪个"驱动舵机"的测试任务**: 1=`gimbal_test_task` / 2=`servo_ch1_mpu_task` / 3=`servo_ch0_mpu_task` / 4=`gimbal_stab_test_task`; 当前 **`TEST_MODE == 4`**。
> 同一时刻只允许一个"驱动舵机"的任务在跑, 否则两条指令流会互相打架 (见 main.c 注释)。
>
> ❌ **已停用 (代码保留但注释, 当前不创建)**: `tcp_server_task` (5/8192)、`mpu_push_task` (4/2048)、`status_report_task` (4/2048) —— 见 §0。
> ❌ v5.10.0 已删除 `status_led_task` (新板无板载 LED, `status_led` 组件整体删除)。

**所有做阻塞式 SPI/I2C 通讯的任务不得长期占用 Task WDT**; 历史网络方案中阻塞式 SPI 任务需 `esp_task_wdt_delete(NULL)` 退订, 改在循环中显式 `esp_task_wdt_reset()` 喂狗。

---

## 4. 硬件引脚定义 (v5.11.1 复核: 与代码一致)

> ✅ 下表引脚在 v5.11.1 已逐个复核, 与 `motor.c` / `imu.c` / `servo.c` / `gps.h` / `main.c` / `wiznet_manager.c` 中的宏一致。
> 注: 电调/L298N 引脚虽仍定义在 `motor.c`, 但当前固件未调用 `motor_init()` (§0), 引脚未实际驱动。

> 依据: [`ESP32-S3-ETH-pinout.md`](ESP32-S3-ETH-pinout.md)。板卡固定占用: W5500 = GPIO9~14、MicroSD = GPIO4~7、
> USB = GPIO19/20、UART0 console = GPIO43/44、Octal PSRAM = **GPIO33~37 (不可用)**。
> 可用空闲脚共 17 个: `1, 2, 3, 15, 16, 17, 18, 21, 38, 39, 40, 41, 42, 45, 46, 47, 48`。
> 本方案按"同功能引脚相邻"排布, 并**避开 strapping 脚 0/45/46** (仅 GPS PPS 用了 GPIO3, 见下)。

| 功能组 | 名称 | 引脚 | 文件 | 说明 | 状态 |
|---|---|---|---|---|---|
| **推进电调 (右排 4 连)** | ESC1_PWM_GPIO | GPIO42 | motor.c | 推进电调 1 PWM (50Hz) | ✅ |
| | ESC2_PWM_GPIO | GPIO41 | motor.c | 推进电调 2 PWM (50Hz) | ✅ |
| | REV1_PWM_GPIO | GPIO40 | motor.c | 反推电调 1 PWM (50Hz, 预留) | ✅ |
| | REV2_PWM_GPIO | GPIO39 | motor.c | 反推电调 2 PWM (50Hz, 预留) | ✅ |
| **L298N (右排 48/47 + 38)** | L298N_IN1_GPIO | GPIO48 | motor.c | L298N 方向 1 | ✅ |
| | L298N_IN2_GPIO | GPIO47 | motor.c | L298N 方向 2 | ✅ |
| | ENA_PWM_GPIO | GPIO38 | motor.c | L298N 调速 PWM (5kHz) | ✅ |
| **I2C0 云台 (左排相邻)** | IMU_I2C0_SDA | GPIO16 | imu.c | 云台 MPU6050 SDA (0x68) | ✅ |
| | IMU_I2C0_SCL | GPIO18 | imu.c | 云台 MPU6050 SCL | ✅ |
| **I2C1 船体 (左排相邻)** | SERVO_I2C1_SDA | GPIO21 | servo.c | PCA9685 (0x40) + 亚博10轴IMU (0x50) SDA | ✅ |
| | SERVO_I2C1_SCL | GPIO17 | servo.c | 同上 SCL | ✅ |
| **GPS UART1 (左排 3 连)** | GPS_TX_GPIO | GPIO2 | main.c/gps.c | ESP32 TX → GPS RX | ✅ |
| | GPS_RX_GPIO | GPIO1 | main.c/gps.c | ESP32 RX ← GPS TX | ✅ |
| | GPS_PPS_GPIO | GPIO3 | main.c/gps.c | PPS 秒脉冲输入 (仅输入; strapping 脚, 见下) | ⚠️ |
| **以太网 (板载固定)** | ETH_CLK | GPIO13 | wiznet_manager.c | W5500 SPI 时钟 | ✅ (固定) |
| | ETH_MOSI | GPIO11 | wiznet_manager.c | W5500 MOSI | ✅ (固定) |
| | ETH_MISO | GPIO12 | wiznet_manager.c | W5500 MISO | ✅ (固定) |
| | ETH_CS | GPIO14 | wiznet_manager.c | W5500 片选 | ✅ (固定) |
| | ETH_RST | GPIO9 | wiznet_manager.c | W5500 复位 (独立 GPIO, 可硬复位) | ✅ (固定) |
| | ETH_INT | GPIO10 | wiznet_manager.c | W5500 中断输出 (下降沿) | ✅ (固定) |

> ⚠️ **引脚注意事项**:
>   - **GPIO3 是 strapping 脚** (JTAG 源选择), 这里只作 PPS **输入**; 上电瞬间不要对该脚外部强拉。若 GPS 模块上电期间会把该脚拉低, 应改用 `pps_gpio = -1` 或换脚。
>   - **GPIO33~GPIO37 不可用** (ESP32-S3R8 Octal PSRAM 占用): 丝印上有脚, 但不可分配给外设。
>   - **GPIO4~7 已固定给板载 MicroSD**, 本方案未使用 SD, 但也不占用这 4 个脚 (原 I2C 曾用 4/5/6/7, 已全部迁走)。
>   - **GPIO0 / GPIO45 / GPIO46 未使用** (strapping)。
>   - UART0 (GPIO43/44) 仍用于烧录/monitor/日志, 115200 8N1 (`sdkconfig.defaults`: `CONFIG_ESP_CONSOLE_UART_DEFAULT=y`)。
>   - W5500 在板卡上已固定连线, 软件侧 `spi_rst_gpio = 9` (之前外置模块方案是接芯片 RST、软件写 -1)。

### 4.1 电机 LEDC 通道分配 (v5.11.1 修复)

> 4 路电调共用 **Timer0 (50Hz)**, 分占 Channel0~3; L298N ENA 用 **Timer1 (5kHz)**, 占 **Channel4**。
> ⚠️ **v5.11.1 修复**: 原 L298N ENA 误用 `LEDC_CHANNEL_0` (与 ESC1 同通道)。`motor_init()` 先配 ESC1、后配 L298N, 第二次 `ledc_channel_config()` 会把 Channel0 的 timer 重绑到 Timer1 → ESC1 (GPIO42) 失去 50Hz PWM, 电调不动作。此前因未调用 `motor_init()` 未暴露; 恢复电机前必须保持 DC 用 Channel4。

| 输出 | 引脚 | LEDC Timer | LEDC Channel | 频率 | 分辨率 |
|---|---|---|---|---|---|
| ESC1 (推进 1) | GPIO42 | Timer0 | Channel0 | 50 Hz | 13 bit |
| ESC2 (推进 2) | GPIO41 | Timer0 | Channel1 | 50 Hz | 13 bit |
| REV1 (反推 1) | GPIO40 | Timer0 | Channel2 | 50 Hz | 13 bit |
| REV2 (反推 2) | GPIO39 | Timer0 | Channel3 | 50 Hz | 13 bit |
| L298N ENA | GPIO38 | Timer1 | **Channel4** | 5 kHz | 13 bit |

> 依据 `motor_get_default_config()`; ESP32-S3 LEDC 共 8 通道 (0~7), 4 路电调已占满 0~3。

---

## 5. W5500 以太网参数 (设计参考, 当前未启用)

> ⚠️ `wiznet_manager_init()` 在 `main.c` 中已注释 (§0), 当前固件不初始化以太网; 下表参数保留作恢复依据。

| 参数 | 值 | 文件 | 说明 |
|---|---|---|---|
| SPI 时钟 | **20 MHz** | wiznet_manager.c | SPI_DMA_DISABLED (polling) 模式, 无堆碎片 |
| DMA 模式 | **DISABLED** | wiznet_manager.c | ESP-IDF v5.5 DMA 模式反复 malloc 会触发 Task WDT |
| RST GPIO | **GPIO9** | wiznet_manager.c | 板载 W5500 的 ETH_RST, 独立 GPIO, 初始化时发 10ms 低电平复位脉冲 + 50ms 自举等待; 另外仍用 `wizchip_sw_reset()` |
| INT GPIO | **GPIO10** | wiznet_manager.c | 板载 W5500 的 ETH_INT, 下降沿触发, ISR 置位 + <1ms 去抖 |
| INT 去抖 | < 1ms | wiznet_spi.c | 防止 INT 抖动反复触发 ISR (`s_last_isr_us < 1000` 即 <1000µs) |
| PHY 模式 | **100M FULL** | wiznet_manager.c | **软件强制** 100BASE-TX 全双工 (PHY 以太网速率, 与 SPI 时钟无关) |
| SPI 互斥锁超时 | 100ms | wiznet_spi.c | 防止死锁 |
| 默认 IP | 192.168.29.10 | main.c | 静态, **main.c 显式覆盖了 wiznet_manager_get_default_config() 的 192.168.1.100**; 远端节点 connect `.10:8081` |
| 子网掩码 | 255.255.255.0 | wiznet_manager.c | /24 |
| 默认网关 | 192.168.29.1 | main.c | 路由器, main.c 显式覆盖了 wiznet_manager_get_default_config() 的 192.168.1.1 |
| DNS | 8.8.8.8 | wiznet_manager.c | 公共 DNS |
| 等待 link up 超时 | 30s | main.c | 启动时阻塞等待 |
| socket 数量 | 8 | wiznet_manager.c | W5500 8 个 socket |
| socket 缓冲 | 2KB/2KB (TX/RX) | wiznet_manager.c | 8 sockets 各 2KB TX + 各 2KB RX, TX 共 16KB + RX 共 16KB, 共用 W5500 内部 32KB SRAM |

> ✅ **v5.2 修复**: main.c 现在显式覆盖 `wiznet_manager_get_default_config()` 的 IP 为 `192.168.29.10`、网关为 `192.168.29.1`, 与远端节点 `.11` 同 /24 网段, 远端 TCP Client 可正常连接.

---

## 6. TCP 服务器端口分配 (设计参考, 当前未启用)

> ⚠️ 当前固件未创建 TCP Server (§0); 端口分配保留作恢复依据。

| 端口 | socket | 角色 | 连接方 | 协议 |
|---|---|---|---|---|
| **8080** | 0 | TCP Server | 上位机 (笔记本) | 16B 控制帧 (收) + 16B MPU 帧 (发, 20Hz) |
| **8081** | 1 | TCP Server | 远端 ESP32-S3 | 8B 子帧 (发) + 16B MPU 帧 (双向, 20Hz) |

---

## 7. 组件清单 (components/)

| 组件 | 路径 | 功能 | API 摘要 |
|---|---|---|---|
| imu | components/imu/ | MPU6050 (I2C0 16/18) + 亚博10轴IMU (I2C1 21/17) 双角色驱动 | imu_init_role, imu_read_role, imu_role_ready, imu_scan_role |
| servo | components/servo/ | PCA9685 驱动 (I2C1 21/17) + **标定体系** (双角度刻度/行程限位/NVS) | servo_init, servo_set_angle, servo_set_pulse_us, servo_set_phys_angle, servo_set_cal/save_cal/load_cal (**详见 §7.6**) |
| gimbal | components/gimbal/ | 云台**运动规划** (速度/加速度受限轨迹) + **姿态闭环 PID 自稳** | gimbal_init, gimbal_move_to, gimbal_set_limit, gimbal_feed_attitude, gimbal_enable_stabilize, gimbal_set_target_phys, gimbal_set_pid, gimbal_set_fb_sign (**详见 §7.7**) |
| gps | components/gps/ | NMEA0183 (UART1 TX=2/RX=1/PPS=3, 9600) | gps_init, gps_get_data, gps_get_pps_count, gps_get_pps_last_us (**详见 §7.8**) |
| wiznet / wiznet_spi | components/wiznet/ | W5500 驱动 (板载, SPI2: 13/11/12/14, RST=9, INT=10) — **当前未启用 (§0)** | wiznet_manager_init, is_link_up, get_ip_info; wiznet_spi_init, wiznet_spi_check_int |
| motor | components/motor/ | 4 ESC (42/41/40/39) + L298N (38/48/47) — **当前未启用 (§0)**; LEDC 通道分配见 §4.1 | motor_init, motor_set_esc_throttle, motor_set_dc_speed, motor_emergency_stop |
| control | components/control/ | 16B 帧协议 (v3.0 沉浮: cmd=0x11 + type=0x03) — **当前未启用 (§0)** | control_process, control_set_forward_callback, control_build_*_frame |
| ~~status_led~~ | ~~components/status_led/~~ | **v5.10.0 已删除** (新板无板载 LED) | — |
| ~~rdk_uart~~ | ~~components/rdk_uart/~~ | **v5.8.0 已删除** (RDK X5 UART 方案作废) | — |

---

## 7.5 IMU 组件 (imu, 双角色)

### 7.5.1 硬件连接 (v5.10.0 换板后)

| 名称 | 引脚 | 上拉 | 说明 |
|---|---|---|---|
| IMU_I2C0_SDA | GPIO16 | 内部上拉使能 | 云台 MPU6050 SDA (0x68) |
| IMU_I2C0_SCL | GPIO18 | 内部上拉使能 | 云台 MPU6050 SCL |
| IMU_HULL_I2C1_SDA | GPIO21 | 由 servo 组件安装 | 船体 亚博10轴IMU SDA (0x50, WIT I2C 协议) |
| IMU_HULL_I2C1_SCL | GPIO17 | 由 servo 组件安装 | 船体 亚博10轴IMU SCL |

I2C0: `I2C_NUM_0`, 400kHz, 由 imu 组件安装。
I2C1: `I2C_NUM_1`, 400kHz, **由 servo 组件为 PCA9685 安装**, imu 只借用 (因此 `servo_init()` 必须先执行)。

> 船体 10 轴 IMU 选 **I2C** 而非 UART: GPS 已独占 UART1, 且该模块支持寄存器式 I2C 直接读欧拉角 (`0x26`), 无需再占一路串口。

### 7.5.2 默认配置

| 参数 | 默认值 | 说明 |
|---|---|---|
| I2C 地址 | 自动探测 0x68 / 0x69 | AD0=GND → 0x68, AD0=VCC → 0x69 |
| 加速度量程 | ±2g | 16384 LSB/g |
| 陀螺仪量程 | ±250°/s | 131 LSB/(°/s) |
| 采样率 | 125Hz | SMPLRT_DIV = 0x07 (1kHz / 8) |
| 低通滤波 | ~5Hz | CONFIG = 0x06 |

### 7.5.3 API

```c
imu_config_t imu_get_default_config(void);   // 默认: I2C0 SDA=16, SCL=18, 400kHz
esp_err_t    imu_init_role(imu_role_t role, const imu_config_t *cfg);
                                             // GIMBAL 装 I2C0; HULL 借用 servo 的 I2C1 (须后于 servo_init)
esp_err_t    imu_read_role(imu_role_t role, imu_data_t *out);  // 阻塞读, ~1ms
bool         imu_role_ready(imu_role_t role);
esp_err_t    imu_scan_role(imu_role_t role);                   // 扫描总线 + 识别型号
void         imu_deinit(void);
```

### 7.5.4 数据格式

`imu_data_t` 提供物理单位:
- `ax/ay/az`: g
- `gx/gy/gz`: °/s
- `temperature`: °C
- `timestamp_us`: 读取时刻 (esp_timer_get_time)

### 7.5.5 协议映射 (仅在网络恢复后有效)

`main.c` 中的 `mpu_push_task` 将浮点原始值转回 int16 LSB, 以便复用 16B MPU 帧:
- ax/ay/az: `(int16)(value * 16384)` 对应 ±2g
- gx/gy/gz: `(int16)(value * 131)` 对应 ±250°/s

> ⚠️ 当前 `mpu_push_task` 已注释 (§0), 该映射暂不生效。

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
servo_config_t servo_get_default_config(void);   // I2C1 SDA=21, SCL=17, 400kHz, 0x40, 50Hz PWM
esp_err_t      servo_init(const servo_config_t *cfg);   // 安装 I2C1 + PCA9685
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

- **fb_sign 实测依据**: ch1 标称角增大时 MPU 的 pitch **减小** → −1; ch0 标称角增大时融合 yaw **增大** → +1。
- 开启闭环时目标默认锁定在**当前姿态**(不跳变), 积分清零。

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

## 7.8 gps 组件 (v5.11.0 新增)

| 参数 | 值 | 说明 |
|---|---|---|
| UART | **UART1** | TX=GPIO2, RX=GPIO1, PPS=GPIO3 |
| 波特率 | **9600** | `gps_config_t.baud` |
| 解析语句 | **$xxRMC / $xxGGA** | RMC: 定位有效/UTC/经纬度/地速/对地真航向; GGA: 定位质量/卫星数/HDOP/海拔 |
| 任务 | gps_task (4/4096) | 由 `gps_init()` 创建 |

- PPS: `gps_get_pps_count()` 累计脉冲数、`gps_get_pps_last_us()` 最近上升沿时刻 (esp_timer us)。
- `course_deg` (对地真航向) 是将来 ch0 平面旋转闭环做**绝对指向**的关键参考 (弥补 MPU 无磁力计)。
- ⚠️ GPIO3 是 strapping 脚, 只作 PPS **输入**; 若 GPS 模块上电会把该脚拉低, 应改 `pps_gpio = -1` 或换脚。
- GPS 未接入属正常, `gps_init()` 失败只告警不报错; `gps_log_task` 仅在初始化成功时创建。

---

## 7.9 串口标定控制台 (v5.11.0 新增)

基于 `esp_console` + `linenoise`, 走 **UART0 (GPIO43/44) 115200**, 提示符 `gimbal>`。
刻意**不调用** `esp_console_start_repl()`, 改用自建 `console_repl_task`, 以支持"裸数字"输入。

| 命令 | 用法 | 说明 |
|---|---|---|
| `p` | `p <ch> <us>` | 直接输出脉宽 |
| `n` | `n <ch> <dus>` | 在当前脉宽上微调 |
| `z` | `z [ch]` | 回中 (不带参数=全部通道) |
| `t` | `t <ch> <trim_us>` | 设置中位微调并回中 |
| `sc` | `sc <ch> <min> <max> <trim> [range_deg] [phys_deg]` | 设置完整标定 |
| `sl` | `sl <ch> <min_deg> <max_deg>` | 设置软件行程限位 |
| `ge` | `ge <ch> <0\|1>` | 姿态闭环开关 |
| `gt` | `gt <ch> <phys_deg>` | 闭环目标物理角 |
| `gp` | `gp <ch> <kp> <ki> <kd>` | 闭环 PID 参数 |
| `gs` | (无参) | 查看云台闭环状态 |
| `st` | (无参) | 查看 ch0/ch1 标定与当前脉宽 |
| `ck` | `ck <ch>` | 把当前脉宽捕获为该通道中位 |
| `sv` | (无参) | 保存标定到 NVS |
| `rs` | `rs <ch>` | 复位通道标定 |
| `att` | `att <0\|1>` | 姿态日志开关 |
| `t0` | `t0 <0\|1>` | ch0 档位扫描测试开关 |
| `t1` | `t1 <0\|1>` | ch1 角度扫描测试开关 |
| `gps` | `gps [0\|1]` | 查看 GPS 状态 (带参数开关 1Hz 日志) |
| `imu` | (无参) | 读取两颗 IMU 各一帧数据 |
| `scan` | `scan [0\|1]` | 扫描 I2C 总线 (0=I2C0 云台, 1=I2C1 船体) |
| `hull` | `hull [0\|1]` | 船体 10 轴 IMU (无参读一帧, 带参数开关 10Hz 日志) |
| `help` | (无参) | 查看全部指令 |

> **裸数字**: 直接输入 `100~3000` 的整数 = 把 ch0 定死在该脉宽 (便于手动找 360° 舵机的停止点)。

---

## 8. 控制协议解析器 (control 组件) — 设计参考, 当前未启用

> ⚠️ `control_process()` / 转发回调当前未在 `main.c` 中调用 (§0); 本章保留作恢复依据。

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
void   control_set_forward_callback(ctrl_forward_cb_t cb);

/* 帧构造器 (上位机和远端节点开发用) */
void control_build_ctrl_frame(uint8_t *frame, uint8_t cmd,
    int8_t speed, int8_t yaw,
    uint8_t remote_light, int8_t bucket_speed, uint8_t flags);

void control_build_mpu_frame(uint8_t *frame, uint8_t type,
    int16_t ax, int16_t ay, int16_t az,
    int16_t gx, int16_t gy, int16_t gz);

/* v3.0 沉浮控制帧 (16B, cmd=0x11, 上位机→主控) */
void control_build_depth_ctrl_frame(uint8_t *frame,
    int16_t target_depth_cm, int8_t target_pitch, int8_t target_roll, uint8_t flags);

/* v3.0 沉浮控制子帧 (8B, cmd=0x11, 主控→远端) */
void control_build_depth_fwd_frame(uint8_t *frame,
    int16_t target_depth_cm, int8_t target_pitch, int8_t target_roll);

/* 统计 */
uint32_t control_get_frame_count(void);
```

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
#define CTRL_CMD_STOP          0x20
#define CTRL_CMD_REBOOT        0x30
#define CTRL_CMD_SHUTDOWN      0x40

#define CTRL_MPU_TYPE_LOCAL    0x01
#define CTRL_MPU_TYPE_REMOTE   0x02
#define CTRL_MPU_TYPE_DEPTH    0x03   /* v3.0 沉浮状态 */

#define CTRL_FLAG_ENABLE_LOCAL   0x01
#define CTRL_FLAG_FORWARD_REMOTE 0x02

/* v3.0 沉浮状态帧 (type=0x03) 的 flags 位 */
#define CTRL_VCTRL_FLAG_DEPTH_VALID  0x01
#define CTRL_VCTRL_FLAG_IMU_VALID    0x02
#define CTRL_VCTRL_FLAG_MANUAL       0x04
```

## 9. 远端节点开发接口 (设计参考, 当前未启用)

> ⚠️ 本章描述主控恢复网络后的**远端对接契约**; 当前固件未启用转发 (§0)。远端 (ESP32-S3-below) 现为 **v3.0**, 见配套远端文档。

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
| **2026-01** | **差速驱动 v4.0: speed+yaw** | **简化协议, 主机发高层指令, ESP32 做开环差速混合** |
| **2026-01** | **删除舵机 TCP 接收 (字节 8/9)** | **主控制节点舵机不再由上位机控制** |
| **2026-01** | **新增远端灯+电机信号** | **远端有灯开关和电机方向+停止** |
| **2026-01** | **L298N 由 speed 自动计算** | **速度用 |speed|, 方向用 sign(speed)** |
| **2026-01** | **v5.0 新增 RDK X5 UART 通信** | **后续扩展性: 网络摄像头→RDK X5 AI 识别→ESP32 桥接** |
| **2026-01** | **UART0 GPIO43/44 115200 8N1** | **专用硬件 UART, 与 console 释放解耦** |
| **2026-01** | **RDK X5 主动模式 (PING/GET_MPU/GET_STATUS)** | **简化 ESP32 逻辑, 资源开销小** |
| **2026-07** | **v5.1 文档校对** | **标题修正 (水上/水下), W5500 默认 IP 标注, rdk_uart_rx_task 补入任务清单, INT 去抖描述统一** |
| 2026-07 | **v5.2 修复主控 IP** | **main.c 显式覆盖 wiznet_manager_get_default_config() 的 IP 为 192.168.29.10, 网关为 192.168.29.1, 与远端节点 .11 同 /24 网段** |
| 2026-07 | **v5.3 重写 MPU6050 驱动** | **完全替换旧 imu.c: I2C0 自动地址探测、PLL 唤醒、±2g/±250°/s 默认量程、burst 读取 14 字节、温度输出、WHO_AM_I 验证; 兼容 ESP-IDF v5.5 的 clk_speed / i2c_master_write_read_device 等 API** |
| 2026-07 | **v5.4 状态灯 + 连接状态 + PHY** | **所有发送路径调用 status_led_notify_tx(); s_host_connected/s_remote_connected 改为 atomic_bool; control 删除 CTRL_STATE_LOADING_FWD; status_led 增加 s_data_mux 临界区 + 无 IP 红灯; sdkconfig 启用 USB-Serial/JTAG console; W5500 PHY 改软件强制 100M FULL** |
| 2026-07 | **v5.5 状态灯改为普通 RGB LED** | **完全重写 status_led: 从 WS2812/RMT 改为 3 GPIO 普通 RGB LED + LEDC PWM; 支持共阴/共阳; 颜色表覆盖红/绿/蓝/深黄/浅黄/紫/深蓝/浅蓝/白; main.c 改为 R=48/G=47/B=21 占位** |
| **2026-08** | **v5.6 水上水下联合方案协议** | **control 协议扩展 v2.0: 新增 cmd=0x11 DEPTH (16B 控制帧+8B 子帧, 目标深度 cm+模式) + type=0x03 深度状态帧 (远端→主控→上位机原样转发); main.c `forward_to_remote()` 按 cmd 分支构造深度子帧; 配套远端文档 v2.0** |
| **2026-08** | **删除 tcp_parser 组件** | **tcp_parser 为 lwip 早期方案遗留 (W5500 TOE 方案下无任何调用), 按文档 §7"遗留, 暂未用"确认无作用后删除组件 + main.c include + REQUIRES** |
| **2026-08** | **v5.7.7 同步远端 v2.18** | **远端 m_comp 暂定 1.6 kg, 配重 = **1871.28 g**; 协议无变化, 仅文档同步** |
| **2026-09** | **v5.7.8 同步远端 v2.19 (L298N 铲斗电机)** | **控制帧/转发子帧 `frame[6]` 由 `remote_dir` 改为 `bucket_speed` (int8, -100~+100, 0=停), 用于远端 L298N 铲斗电机; `control.h` 中 `ctrl_command_t.remote_dir` 重命名为 `bucket_speed` (int8), `control_build_ctrl_frame()` 签名同步修改; `control.c` 解析/日志/帧构造更新; `main.c` `forward_to_remote()` 8B 子帧转发 `bucket_speed`; 更新本文档 §1.1/§2.2/§2.3/§2.7/§8.2/§10/版本历史** |
| **2026-09** | **v5.8.0 删除 RDK X5 UART 方案** | **RDK X5 已砍: 删除 `components/rdk_uart` 整个组件 + `main/CMakeLists.txt` REQUIRES + `main.c` include/初始化/RDK 主动请求逻辑 + `sdkconfig.defaults` RDK 注释 + README 引脚表/目录树; 本文档删除原 §9「RDK X5 UART 通信」全章 (后续章节顺延: 原 §10 远端节点开发接口 → §9) 及 §1.1/§1.2/§3/§4/§7 中全部 RDK 条目; console 保持 UART0 (GPIO43/44 已空闲)** |
| **2026-09** | **v5.9.0 新增 ROCK 5C 视觉节点** | **ROCK 5C (Radxa SBC) 同时承担推流 + AI 推理, 输出的 AI 推理视频流经内网直接访问; 与主控节点 (ESP32) 无任何数据链路, 不占用 GPIO/UART/TCP 资源; 恢复"网络摄像头"节点条目 (作为 ROCK 5C 输入源); 更新本文档 §1.1/§1.2 节点表与拓扑图 + README** |
| **2026-09** | **v5.10.0 换板 ESP32-S3-ETH + 同步远端 v3.0 协议** | **①硬件: 换 Waveshare ESP32-S3-ETH (板载 W5500, 外置模块弃用), W5500 引脚改为板载固定 13/11/12/14/9/10, 引脚全表按"同功能相邻 + 避开 SD 4-7 与 strapping 0/45/46"重排: 4 路电调 42/41/40/39, L298N 48/47/38, I2C0 16/18, I2C1 21/17, GPS 2/1/3; 删除 `components/status_led` 组件 (新板无板载 LED) + `main/CMakeLists.txt` REQUIRES + main.c 遗留引用. ②协议: `cmd=0x11` 字节 5/6 由 `mode`+保留 改为 `target_pitch int8(°)`+`target_roll int8(°)`; `type=0x03` 状态帧整体重定义 (depth_cm / pitch / roll / mode / out_f / out_rl / out_rr / flags); `ctrl_command_t` 的 `depth_mode` → `target_pitch_deg` + `target_roll_deg`; `control_build_depth_ctrl_frame()` / `control_build_depth_fwd_frame()` 签名同步; 主控取消 type=0x02 远端原始 MPU 上报. ③同步更新本文档 §1/§2/§3/§4/§5/§7/§7.5/§8/§9/§11/§12 + README** |
| **2026-09** | **v5.11.0 切至临时测试/标定模式 + 文档同步实况** | **①固件: `main.c` 进入 [临时测试/标定模式] —— W5500/TCP Server/motor/control 的初始化与调用全部注释 (代码保留), 实跑 云台 MPU6050(I2C0) + PCA9685(I2C1) + 船体10轴IMU(0x50) + GPS(UART1) + 云台运动规划/姿态闭环 + Madgwick 姿态解算 + 串口控制台; `TEST_MODE=4` (云台闭环自稳)。②新增 `components/gps` 组件 (NMEA0183 RMC/GGA + PPS, UART1 TX2/RX1/PPS3 @9600)。③servo 组件新增标定体系: 双角度刻度(标称角/物理角)、按通道实测默认标定(ch0 530~2730µs=0~360°, phys=365; ch1 600~2700µs=0~180°, phys=192, 限位 30~150°)、trim、行程限位、NVS 存取、cmd↔phys 换算、`servo_set_phys_angle`。④gimbal 组件新增姿态闭环 PID 自稳: 100Hz, 增量式 PID (kp=0.8/ki=0.5/kd=0), 物理角域控制律, fb_sign(ch0=+1/ch1=−1), 积分限幅 ±10, 输出速率限 120°/s。⑤新增串口标定控制台 (esp_console, UART0 115200, 提示符 `gimbal>`, 21 条命令 + 裸数字改 ch0 脉宽)。⑥文档: 新增 §0 当前固件形态, 重写 §3 任务清单, 更新 §4/§7 组件清单, 新增 §7.6 servo 标定 / §7.7 gimbal 闭环 / §7.8 gps / §7.9 串口控制台, 给 §2/§5/§6/§8/§9 加"设计参考, 当前未启用"标识; 同步 README** |
| **2026-09** | **v5.11.1 修复电机 LEDC 通道冲突 + 16MB Flash 生效 + 注释更正** | **① `motor.c`: L298N 的 `dc_ledc_channel` 由 `LEDC_CHANNEL_0` 改为 `LEDC_CHANNEL_4` (Timer1 不变)。原因: 4 路电调占 Timer0/Channel0~3, L298N 原也占 Channel0, `motor_init()` 先配 ESC1、后配 L298N 会把 Channel0 的 timer 重绑到 Timer1 → ESC1 失去 50Hz PWM (此前未调用 `motor_init()` 未暴露)。② 删除 `sdkconfig`/`sdkconfig.old` 重新生成, 使 `sdkconfig.defaults` 的 `CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y` 生效 (原 sdkconfig 停留 2MB), 全量重编译 (`flasher_args.json` → `--flash_size 16MB`)。③ 更正 `motor.c` 文件头通道表与 `motor_init()` 内过时引脚注释 (GPIO1/2/3 → 42/41/40/39)、ESC1 失效的 "U0TXD" 启动日志, 及 `motor.h` 中已废弃的 `MOTOR_TEST_ESC` 引用。④ 新增 §4.1 电机 LEDC 通道分配; 更新 §4/§7/§12 + README** |

---

## 11. 状态指示灯

> ❌ **v5.10.0 已删除**: 新板 (ESP32-S3-ETH) **无板载 LED**, `components/status_led` 组件整体移除,
> `main.c` 的 `status_led_task` / `status_led_init` / `notify_rx` / `notify_tx` 调用与 `main/CMakeLists.txt` 依赖一并删除。
> 下方内容为 v5.x 历史记录 (板载 WS2812 或外接 RGB LED 方案), v3.0 起不再适用; 如需状态指示, 请外接灯并另行约定引脚。

| 状态 | 颜色 | 触发条件 (历史) |
|---|---|---|
| 未连接 / 无 IP | 红色常亮 | 网线未插 / 未获得 IP |
| 已连接 + 数据交换 | 蓝色闪烁 (200ms) | 近 500ms 有 RX/TX 数据 |
| 已连接 + 无数据 | 绿色常亮 | 链路 up, 无通信 |

LED 优先级: 红 > 蓝 > 绿

### 11.1 驱动方式

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

## 12. 构建与烧录

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

> 环境: ESP-IDF **v5.5.4**。烧录与日志走 **UART0 (GPIO43/44)** 板载 USB 转串口, 115200 8N1
> (`sdkconfig.defaults`: `CONFIG_ESP_CONSOLE_UART_DEFAULT=y`)。

### Flash 尺寸 / 分区 / 构建注意 (v5.11.1)

| 项 | 值 | 说明 |
|---|---|---|
| 板载 Flash | **16 MB** (W25Q128FS) | `sdkconfig.defaults`: `CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y` / `"16MB"` |
| 分区表 | `SINGLE_APP` (默认 `partitions_singleapp.csv`) | app 分区固定 **1 MB** |
| 当前 app 占用 | ≈ `0x51890` (~333 KB) | 剩余约 68% (v5.11.1 编译结果) |

> ⚠️ **`sdkconfig.defaults` 只对 sdkconfig 中不存在的项生效**。若 `sdkconfig` 已生成旧值 (如 `CONFIG_ESPTOOLPY_FLASHSIZE_2MB`), 改 defaults **不会覆盖** —— 必须**删除 `sdkconfig` / `sdkconfig.old` 后重新 configure/build**, 16MB 才生效 (`flasher_args.json` 的 `--flash_size` 随之变成 `16MB`)。
>
> ⚠️ `build/` 记录了工程**绝对路径**: 工程目录换盘符/路径后必须删除 `build/` 重编, 否则报 `Build directory ... configured for project ... not ...`。
>
> ⚠️ 若 `.\idf.ps1 build` 报 `No module named 'click'`, 说明 PATH 上的 `python` 不是 IDF 的 venv。应使用 `<IDF_TOOLS_PATH>\tools\python\<版本>\venv\Scripts\python.exe` 直接运行 `idf.py` (或把该 `Scripts` 目录置于 PATH 最前)。

预期启动输出 (当前测试模式, 参考):
```
=== [临时测试模式] MPU6050 + PCA9685 ===
start_components 开始执行
NVS 初始化成功
初始化云台 MPU6050 (0x68)...   云台 MPU6050 初始化成功
初始化 GPS (UART1: TX=IO2, RX=IO1, PPS=IO3)...
初始化 PCA9685...   PCA9685 初始化成功, 已回中: ch0=1630 us, ch1=1650 us (90°)
云台运动规划初始化成功
初始化船体 10 轴 IMU (I2C1, 0x50)...
[AHRS] 三维姿态解算任务启动 (100Hz, Madgwick β=0.10)
[STAB] 云台闭环自稳测试: ch1 俯仰, 目标物理角 0.0°
串口控制台已启动: 直接输数字改 ch0 脉宽, 或输 help 看全部指令
=== [临时测试模式] 启动完成: MPU6050 + PCA9685 ===
```

> ⚠️ 网络恢复后才会出现 "W5500 初始化中 / 本机 IP: 192.168.29.10 / TCP 服务器已启动: 8080(HOST) + 8081(REMOTE)" 等启动日志 (§0)。

---

> 📝 文档变更需同步更新所有相关代码并测试.
>
>   最后修改: **v5.11.1** (修复 motor LEDC 通道冲突: L298N DC 改用 Channel4; 删除 sdkconfig 重新生成使 16MB Flash 生效并全量重编译; 更正 motor.c/motor.h 过时注释; 新增 §4.1 电机 LEDC 通道分配, 更新 §4/§7/§10/§12 与 README)
>
>   历史: v5.11.0 固件切至 [临时测试/标定模式] (新增 gps / servo 标定 / gimbal 闭环 / 串口控制台) / v5.10.0 换板 ESP32-S3-ETH + 同步远端 v3.0 / v5.9.0 ROCK 5C 视觉节点 / v5.8.0 删除 RDK X5 / v5.7.8 同步远端 v2.19 (`bucket_speed`) / v5.7.7 同步远端 v2.18 / v5.6 水上水下联合协议 v2.0
