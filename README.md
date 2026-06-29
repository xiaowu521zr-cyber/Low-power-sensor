# ESP32-C6 MPU6050 陀螺仪传感器节点

> 基于 ESP32-C6 + FreeRTOS 的无线六轴姿态监测传感器节点，搭载 LoRa 远距离无线通信与 ULP RISC-V 超低功耗协处理器。

[![ESP32-C6](https://img.shields.io/badge/MCU-ESP32--C6-blue)](https://www.espressif.com/en/products/socs/esp32-c6)
[![MPU6050](https://img.shields.io/badge/IMU-MPU6050-green)](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
[![LoRa](https://img.shields.io/badge/Wireless-LoRa-orange)](https://www.semtech.com/lora)
[![FreeRTOS](https://img.shields.io/badge/RTOS-FreeRTOS-lightgrey)](https://www.freertos.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

---

## 📋 目录

- [项目简介](#项目简介)
- [功能特性](#功能特性)
- [硬件架构](#硬件架构)
- [软件架构](#软件架构)
- [文件结构](#文件结构)
- [关键技术](#关键技术)
- [快速开始](#快速开始)
- [配置参数](#配置参数)
- [通信协议](#通信协议)
- [电源管理](#电源管理)
- [开发环境](#开发环境)

---

## 项目简介

本项目是一个**低功耗无线姿态监测传感器节点**，基于乐鑫 **ESP32-C6** 主控芯片与 **MPU6050** 六轴惯性测量单元（IMU），通过 **LoRa 射频无线通信** 将实时姿态数据上报至远程基站网关。

系统采用 **FreeRTOS 实时操作系统** 进行多任务调度，并充分利用 ESP32-C6 的 **ULP RISC-V 低功耗协处理器** 实现深度睡眠模式下的后台值守，可在进入深度睡眠以后能够达到微安级功耗，并且在系统全力工作下能够达到20~35毫安的功耗下下持续监测设备异动，适用于**工业设备姿态监测、桥梁结构健康检测、地质灾害预警**等需要长期电池供电的无线传感器应用场景。

---

## 功能特性

### 核心传感
- 🎯 **六轴姿态感知**：三轴加速度计（±16g）+ 三轴陀螺仪（±2000°/s）
- 📐 **实时姿态解算**：横滚角（Roll）、俯仰角（Pitch）、航向角（Yaw）
- 🔍 **中值滑动滤波**：窗口大小为 5 的中值滤波器，有效剔除脉冲噪声和毛刺
- 📉 **零偏漂移检测**：基于统计方差分析，智能识别传感器数据冻结/卡死/温漂

### 无线通信
- 📡 **LoRa 扩频通信**：433MHz 频段，最大发射功率 20dBm（约 100mW）
- 🎯 **定向传输模式**：支持定点/广播寻址，精准路由至目标网关
- 📦 **自定义网关协议**：包含数据帧、报警帧、状态帧、命令帧、应答帧五种帧类型
- 🔐 **同步字 + 滚动序号**：0xA5A5 帧同步魔术字，流水号防重防漏

### 智能报警
- ⚡ **多轴向边界红线盘查**：Roll/Pitch/Yaw/加速度合矢量四级独立阈值
- 🧠 **真突变 vs 温漂鉴别**：一阶微分阶跃捕捉 + 滑动窗口斜率分析，杜绝假性误报
- 🔥 **高优先级突发发射**：检测到真实危险时，无条件抢占物理信道，即刻上报
- ⏱️ **告警冷却机制**：5 秒冷却窗口，防止高频振动导致 LoRa 网络风暴阻塞

### 电源管理
- 🔋 **ADC 电池监测**：8 次过采样滤波，分压系数补偿，精确毫伏级电量感知
- 🪫 **阶梯式低电量防御**：黄线预警（3.3V）+ 红线关机保护（3.0V）
- 💤 **深度睡眠模式**：5 秒无活动自动休眠，全机关断进入微安级功耗
- 🔄 **三向唤醒矩阵**：EXT0 硬件唤醒 + ULP 协处理器唤醒 + 30 分钟保底定时器

### 远程管控
- 🎛️ **无线参数重配置**：支持远程修改 Roll/Pitch/Yaw/加速度阈值
- 📊 **设备体检上报**：网关可随时索取运行状态、当前阈值、电池电压
- 🔄 **固件热重启**：支持远程软件复位指令（CMD_SOFT_RESET）

---

## 硬件架构

```
┌─────────────────────────────────────────────────────────────┐
│                    ESP32-C6 主控芯片                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   │
│  │ I2C 总线  │  │ UART1    │  │ ADC1     │  │ RTC 域   │   │
│  │ (GPIO4,5)│  │ (GPIO6,7)│  │ (GPIO0)  │  │ (GPIO3)  │   │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘   │
│       │             │             │             │           │
│  ┌────▼─────┐  ┌────▼─────┐  ┌────▼─────┐  ┌────▼─────┐   │
│  │ MPU6050  │  │LoRa 模块  │  │ 电池分压  │  │ INT 唤醒  │   │
│  │ 6轴 IMU  │  │ATK-LORA02│  │ 采样电路  │  │ EXT0引脚  │   │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘   │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐   │
│  │            ULP RISC-V LP-Core 协处理器                │   │
│  │          (RTC 慢速内存共享，深度睡眠后台值守)           │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### 引脚映射表

| 外设 | 引脚 | 功能描述 |
|------|------|----------|
| MPU6050 SDA | GPIO 4 | I2C 数据线 |
| MPU6050 SCL | GPIO 5 | I2C 时钟线 |
| MPU6050 INT | GPIO 3 | 运动中断 / EXT0 深度睡眠唤醒 |
| LoRa TX | GPIO 6 | 串口发送（UART1） |
| LoRa RX | GPIO 7 | 串口接收（UART1） |
| LoRa AUX | GPIO 10 | 模块状态指示 |
| LoRa MD0 | GPIO 11 | 模式控制（AT 配置/通信切换） |
| 电池 ADC | GPIO 0 | 电池电压模拟采样 |
| 状态 LED | GPIO 8 | 运行状态指示灯 |

### 硬件特性

| 项目 | 参数 |
|------|------|
| 主控芯片 | ESP32-C6（RISC-V 160MHz） |
| IMU 传感器 | MPU6050（I2C 地址 0x68） |
| 无线模块 | 正点原子 ATK-LORA-02 |
| 通信频段 | 433MHz |
| I2C 速率 | 400kHz 快速模式 |
| 串口波特率 | 115200bps |
| Flash 容量 | 16MB |
| 电池分压比 | 2:1（R1=100kΩ, R2=100kΩ） |

---

## 软件架构

### 多任务调度设计

系统采用 FreeRTOS 优先级抢占式调度，共 4 个核心业务任务 + 1 个 ULP 协处理器任务：

```
优先级 ▲
   │
   8  ═══ sensor_transient_task ═══  【最高】暂态突变捕捉 & 紧急警报发射
   │         │ 读取队列
   7  ═══ command_task ═══            【较高】下行无线命令解析 & 远程配置
   │         │ 接收命令
   5  ═══ sensor_acq_task ═══         【中等】高频数据采集 & 中值滤波清洗
   │         │ 推送数据队列
   3  ═══ power_mgmt_task ═══         【较低】电池监控 & 深度睡眠调度
   │
  ─── ULP RISC-V LP-Core ───         【超低功耗】深度睡眠后台看门狗值守
```

### 任务详细职责

| 任务名称 | 优先级 | 堆栈 | 核心职责 |
|----------|--------|------|----------|
| `sensor_acq` | 5 | 4096字节 | 200Hz 高频读取 MPU6050 → 滑动中值滤波 → 数据有效性校验 → 推入消息队列 |
| `sensor_transient` | 8 | 4096字节 | 从队列拉取数据 → 多轴向边界红线盘查 → **真突变/温漂鉴别** → 突发报警无线发射 |
| `command_task` | 7 | 4096字节 | DMA+IDLE 串口接收 → 命令帧拆包解析 → 参数合法性校验 → 远程配置执行 & 应答 |
| `power_mgmt` | 3 | 3072字节 | 60 秒周期电池 ADC 采样 → 阶梯式低电量防御 → 5 秒闲置深度睡眠判定 → 三向唤醒矩阵配置 |

### 任务间通信

```
┌──────────────┐   sensor_data_queue   ┌──────────────────┐
│ sensor_acq   │ ────────────────────► │ sensor_transient │
│ (数据生产者)  │                       │ (数据消费者)      │
└──────┬───────┘                       └────────┬─────────┘
       │                                        │
       │         ┌──────────────────┐           │ alarm_queue
       │         │   g_event_group  │ ◄─────────┤
       └────────►│  (事件同步组)     │           │
                 │                  │ ◄─────────┤
   ┌─────────────┤  EVENT_ALARM     │           │
   │ command_task│  EVENT_SENSOR     │  power_mgmt_task
   │ (命令注入)   │  EVENT_CMD       │ (状态消费)
   └──────┬──────┘  EVENT_DRIFT     └───────────┘
          │        EVENT_DEEP_SLEEP
          │               ▲
 command_queue            │
          │        g_i2c_mutex
          ▼               │
   ┌──────────┐    ┌──────┴──────┐
   │ LoRa UART│    │  I2C 总线    │
   │ (DMA+IDLE)│   │ (互斥锁保护) │
   └──────────┘    └─────────────┘
```

---

## 文件结构

```
ESP32C6_MPU6050_Gyro/
├── CMakeLists.txt                    # 项目 CMake 构建配置
├── sdkconfig.defaults                # ESP-IDF SDK 默认配置
├── partitions.csv                    # Flash 分区表
├── esp32_studio.json                 # 开发工具配置
│
├── main/
│   ├── CMakeLists.txt                # 主组件构建配置
│   ├── main.h                        # 主入口头文件（状态机、全局变量、苏醒成因枚举）
│   ├── main.c                        # 系统主入口（硬件初始化、任务创建、ULP加载）
│   │
│   ├── config/
│   │   └── app_config.h              # 【核心】全局配置文件（引脚、阈值、队列深度、协议参数）
│   │
│   ├── drivers/                      # 硬件驱动层
│   │   ├── mpu6050.h                 # MPU6050 I2C 驱动头文件（寄存器映射、量程、校准结构体）
│   │   ├── mpu6050.c                 # MPU6050 驱动实现（原始读取、单位转换、运动检测配置、零偏校准）
│   │   ├── lora_uart.h               # LoRa UART 驱动头文件（定向传输、AT 指令、模式控制）
│   │   ├── lora_uart.c               # LoRa UART 驱动实现（DMA+IDLE、AUX 时序控制、休眠管理）
│   │   ├── adc_battery.h             # 电池 ADC 驱动头文件
│   │   └── adc_battery.c             # 电池 ADC 驱动实现（8 次过采样、分压补偿、电量估算）
│   │
│   ├── tasks/                        # FreeRTOS 任务层
│   │   ├── sensor_acq_task.h         # 传感器采集任务头文件
│   │   ├── sensor_acq_task.c         # 传感器采集任务实现（中值滤波、漂移检测、数据有效性检查）
│   │   ├── sensor_transient_task.h   # 暂态突变检测任务头文件
│   │   ├── sensor_transient_task.c   # 暂态突变检测任务实现（真伪鉴别算法、冷却管理、警报发射）
│   │   ├── command_task.h            # 命令解析任务头文件
│   │   ├── command_task.c            # 命令解析任务实现（帧拆包、参数校验、远程配置、跨核同步）
│   │   ├── power_mgmt_task.h         # 电源管理任务头文件
│   │   └── power_mgmt_task.c         # 电源管理任务实现（电池监控、深度睡眠判定、三向唤醒）
│   │
│   ├── protocol/                     # 通信协议层
│   │   ├── gateway_protocol.h        # 网关协议头文件（帧格式定义、API 接口）
│   │   └── gateway_protocol.c        # 网关协议实现（帧组装、序列化发送、阈值管理、NVS 持久化）
│   │
│   └── ulp/                          # ULP 低功耗协处理器
│       ├── ulp_shared.h              # 主核与 ULP 核共享的 RTC 内存布局定义
│       └── ulp_main.S                # ULP RISC-V 汇编程序（唤醒计数、标志管理、CRC 校验）
│
└── components/
    └── OpenOCD/                      # OpenOCD 调试组件
        ├── CMakeLists.txt
        ├── include/OpenOCD.h
        └── OpenOCD.c
```

---

## 关键技术

### 1. 真突变 vs 温漂智能鉴别算法

传感器在长期运行中会因温度变化、元件老化产生缓慢的零偏漂移，如果仅凭简单的阈值比较，极易产生大量假阳性误报。本系统采用**双维度联合鉴别**：

- **一阶微分阶跃捕捉**：计算相邻两次采样的一阶向后差分。若突变量超过安全阈值的 30%，判定为真实的机械突变（阶跃响应）。
- **滑动窗口斜率分析**：在 10 个样本的观测窗口内计算平均变化率。若斜率低于安全阈值 × 5%，判定为缓慢温漂，就地拦截静默，不触发无线发射。

```
              真实突变 (触发报警)
              ▲
              │ ▄▄▄
              │▐████▌
  ─ ─ ─ ─ ─ ─ ▐████▌─ ─ ─ 安全红线阈值
              │██████▌
              │██████▌
              │      ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀──── 温漂 (静默拦截)
              │
              └─────────────────────────► 时间
```

### 2. RTC 慢速内存数据共享隧道

ESP32-C6 的主 CPU 在深度睡眠期间会完全断电，但 **RTC 慢速内存**（RTC Slow Memory，8KB）由独立的 RTC 电源域持续供电，数据掉电不丢失。

系统利用该特性在主核与 ULP 协处理器之间建立了**跨核数据共享隧道**：

- 主核休眠前将安全阈值、电池电量、控制标志写入 RTC 内存
- 主核断电期间，ULP 协处理器从中读取参数执行后台值守
- 主核被唤醒后，从 RTC 内存中恢复协处理器记录的唤醒计数和状态信息
- 使用 `volatile` 关键字 + 位掩码机制，无需互斥锁即可实现跨核原子通信

### 3. MPU6050 硬件运动唤醒

系统利用 MPU6050 芯片内置的**运动检测中断（Motion Detection Interrupt）** 功能：

1. 系统进入深度睡眠前，配置 MPU6050 进入**循环模式（Cycle Mode）**
2. 芯片以超低功耗（微安级）定时唤醒加速度计检测振动
3. 当检测到加速度超过设定阈值（默认 80mg）时，MPU6050 硬件拉高 INT 引脚
4. INT 引脚直连 ESP32-C6 的 RTC 域 GPIO 3，通过 **EXT0 硬件唤醒机制** 瞬间拉醒主 CPU
5. 主 CPU 苏醒后第一时间读取 INT_STATUS 寄存器清除锁存中断

### 4. DMA + IDLE 串口接收

LoRa 模块通过 UART 与主控通信。为了高效接收变长数据包并降低 CPU 负载：

- **DMA 接收**：硬件 DMA 通道自动将串口数据搬运到内存缓冲区
- **IDLE 空闲中断**：当串口总线空闲超过 3 个字符周期（约 260μs），硬件自动触发中断，标记一帧变长数据包接收完毕
- **FIFO 批量触发**：接收 FIFO 积攒到 120 字节时触发 DMA 批量搬运，提升吞吐率
- 设计目标丢包率：< 0.5%

### 5. 三级省电深度睡眠策略

```
 全速运行 ──5秒无活动──► 准备休眠 ──满足条件──► 深度睡眠
    ▲                      │                      │
    │                      │ 拒绝（有未处理业务）   │ EXT0 / ULP / Timer
    │                      ▼                      │ 三向唤醒
    └──────────────────── 恢复运行 ◄───────────────┘
```

---

## 快速开始

### 环境要求

- **ESP-IDF** v5.0 或更高版本
- **编译目标芯片**：`esp32c6`
- **Python** 3.8+（ESP-IDF 依赖）

### 编译步骤

```bash
# 1. 克隆仓库
git clone <your-repo-url>
cd ESP32C6_MPU6050_Gyro

# 2. 设置 ESP-IDF 环境变量
. $IDF_PATH/export.sh          # Linux/macOS
# 或
%IDF_PATH%\export.bat          # Windows

# 3. 设置目标芯片
idf.py set-target esp32c6

# 4. 编译项目
idf.py build

# 5. 烧录到开发板（将 PORT 替换为实际串口号）
idf.py -p PORT flash

# 6. 打开串口监视器
idf.py -p PORT monitor
```

### 分区表说明

| 分区名 | 类型 | 偏移地址 | 大小 | 说明 |
|--------|------|----------|------|------|
| nvs | data/nvs | 0x9000 | 24KB | 非易失性存储（阈值、校准数据） |
| phy_init | data/phy | 0xF000 | 4KB | 射频物理层初始化数据 |
| factory | app/factory | 0x10000 | 3MB | 固件主程序 |
| ulp_stor | data/fat | 0x310000 | 960KB | ULP 协处理器数据存储 |

---

## 配置参数

所有核心参数集中在 `main/config/app_config.h`，可根据实际硬件和应用场景修改：

### 传感器参数

```c
#define MPU6050_ADDR                0x68        // I2C 从机地址
#define MPU6050_SAMPLE_RATE         200         // 采样率 200Hz
#define MPU6050_DLPF_CFG            2           // 数字低通滤波器（94/98Hz 截止）
#define MPU6050_GYRO_FS             MPU6050_GYRO_FS_2000  // 陀螺仪 ±2000°/s
#define MPU6050_ACCEL_FS            MPU6050_ACCEL_FS_16G  // 加速度计 ±16g
#define MPU6050_MOT_THRESHOLD       80          // 运动唤醒阈值 80mg
#define MPU6050_MOT_DURATION        2           // 运动持续时间 2ms
```

### 报警阈值

```c
#define DEFAULT_ROLL_THRESHOLD      15.0f       // 横滚角 ±15 度
#define DEFAULT_PITCH_THRESHOLD     15.0f       // 俯仰角 ±15 度
#define DEFAULT_YAW_THRESHOLD       30.0f       // 航向角 ±30 度
#define DEFAULT_ACCEL_THRESHOLD     2.5f        // 加速度合矢量 2.5g
#define ALARM_COOLDOWN_MS           5000        // 报警冷却时间 5 秒
```

### 电源管理

```c
#define BATTERY_LOW_THRESHOLD_MV    3300        // 低电量黄线 3.3V
#define BATTERY_CRIT_THRESHOLD_MV   3000        // 关机红线 3.0V
#define BATTERY_CHECK_INTERVAL_MS   60000       // 电池检测间隔 60 秒
#define DEEP_SLEEP_TIMEOUT_MS       5000        // 闲置休眠倒计时 5 秒
```

### LoRa 通信

```c
#define LORA_CHANNEL                23          // 射频信道 23（433MHz 频段）
#define LORA_TX_POWER               20          // 发射功率 20dBm
#define LORA_AIR_RATE               5           // 空中速率 19.2kbps
#define DEVICE_ADDR_H               0x0D        // 本机地址高字节
#define DEVICE_ADDR_L               0x80        // 本机地址低字节（完整地址 0x0D80）
#define GATEWAY_ADDR_H              0x04        // 目标网关地址高字节
#define GATEWAY_ADDR_L              0xD2        // 目标网关地址低字节（完整地址 0x04D2）
```

---

## 通信协议

### 协议帧结构

所有无线通信帧均采用统一的二进制协议格式：

```
┌──────────┬──────────┬──────────┬──────────┬──────────┬──────────┬──────────┬──────────┬──────────────┐
│  Sync    │ Frame    │ Src Addr │ Src Addr │ Dst Addr │ Dst Addr │ Seq Num  │ Payload  │   Payload    │
│  2 Bytes │ Type 1B  │  H   1B  │  L   1B  │  H   1B  │  L   1B  │   1B     │  Len 1B  │   N Bytes    │
├──────────┼──────────┼──────────┼──────────┼──────────┼──────────┼──────────┼──────────┼──────────────┤
│  0xA5A5  │ 帧类型码  │ 源地址高  │ 源地址低  │ 目标地址高│ 目标地址低│ 滚动序号  │ 载荷长度  │   业务数据    │
└──────────┴──────────┴──────────┴──────────┴──────────┴──────────┴──────────┴──────────┴──────────────┘
```

- **Sync**：帧同步魔术字，固定为 `0xA5A5`
- **Frame Type**：帧类型识别码
- **Seq Num**：滚动流水包序号（0~255），用于丢包统计和重包去重
- **Payload Len**：有效载荷精确字节长度

### 帧类型

| 类型码 | 帧类型 | 方向 | 说明 |
|--------|--------|------|------|
| `0x01` | DATA | 上行 | 定时常规传感器数据上报帧 |
| `0x02` | ALARM | 上行 | 突发极限超限紧急报警帧（最高优先级） |
| `0x03` | STATUS | 上行 | 设备全生命周期运行状态体检报表 |
| `0x04` | ACK | 上行 | 无线双向握手应答帧 |
| `0x10` | CMD | 下行 | 基站网关下发控制配置命令 |
| `0x11` | CMD_RESP | 上行 | 节点对下行命令执行结果的闭环应答 |

### 报警类型码

| 类型码 | 报警类型 | 说明 |
|--------|----------|------|
| `0x01` | ROLL | 横滚角越界 |
| `0x02` | PITCH | 俯仰角越界 |
| `0x03` | YAW | 航向角越界 |
| `0x04` | ACCEL | 加速度冲击过载（如撞击/跌落） |
| `0x05` | TEMP | 环境温度异常 |
| `0x06` | LOW_BATTERY | 电池严重亏电 |
| `0xFF` | SENSOR_FAULT | IMU 传感器硬件故障 |

### 网关下行命令

| 命令码 | 功能 | 参数 |
|--------|------|------|
| `0x10` | 设置 Roll 阈值 | float（4 字节，0~180°） |
| `0x11` | 设置 Pitch 阈值 | float（4 字节，0~180°） |
| `0x12` | 设置 Yaw 阈值 | float（4 字节，0~180°） |
| `0x13` | 设置加速度阈值 | float（4 字节，0~16g） |
| `0x14` | 设置运动检测阈值 | uint8（1 字节，0~255mg） |
| `0x20` | 设置采样率 | uint16（2 字节，1~1000Hz） |
| `0x21` | 设置 ULP 唤醒周期 | uint32（4 字节，单位 μs） |
| `0x30` | 索取设备状态 | 无参数 |
| `0x31` | 索取全部阈值 | 无参数 |
| `0x32` | 索取电池电压 | 无参数 |
| `0xFF` | 软件热重启 | 无参数 |

---

## 电源管理

### 状态机流程

```
   ┌──────────┐    初始化完成    ┌──────────┐   5秒无活动   ┌──────────┐
   │   INIT   │ ─────────────► │  ACTIVE  │ ───────────► │ PRE_SLEEP│
   │ 初始化阶段 │                │ 全速运行  │              │ 准备休眠  │
   └──────────┘                └──────────┘              └────┬─────┘
                                    ▲                        │
                                    │        ┌──────────┐    │
                                    │        │ MONITORING│ ◄──┤
                                    │        │ 轻量监控  │    │
                                    │        └──────────┘    │
                                    │              ▲          │
                                    │              │          │
                                    │    ┌─────────┴──────┐  │
                                    │    │    WAKING      │  │
                                    └────│   苏醒恢复      │◄─┘
                               EXT0/ULP │                │
                               /Timer   └────────────────┘
                                         ▲
                                         │
                                    ┌────┴──────────┐
                                    │  DEEP_SLEEP   │
                                    │  深度睡眠      │
                                    │ (主CPU断电)    │
                                    └───────────────┘
```

### 功耗预估

| 运行模式 | 功耗级别 | 说明 |
|----------|----------|------|
| 全速运行 | ~30-50mA | CPU 160MHz、MPU6050 200Hz、LoRa 发射 |
| 轻量监控 | ~5-10mA | 降频运行、低频采样 |
| 深度睡眠 | <10μA | 主 CPU 断电，仅 ULP + MPU6050 Motion 值守 |

---

## 开发环境

### 推荐工具

- **IDE**：VS Code + ESP-IDF 插件
- **调试器**：板载 USB-JTAG 或 外接 OpenOCD 调试器
- **串口工具**：idf.py monitor / PuTTY / SecureCRT（波特率 115200）
- **LoRa 网关**：配合正点原子 ATK-LORA-02 接收模块使用

### SDK 配置要点

```ini
CONFIG_IDF_TARGET="esp32c6"                 # 目标芯片
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=160         # CPU 主频 160MHz
CONFIG_FREERTOS_HZ=1000                     # FreeRTOS 时钟节拍 1kHz
CONFIG_ULP_COPROC_ENABLED=y                 # 使能 ULP 协处理器
CONFIG_ULP_COPROC_RISCV=y                   # ULP 架构选择 RISC-V
CONFIG_ULP_COPROC_RESERVE_MEM=4096          # ULP 保留内存 4KB
CONFIG_PM_ENABLE=y                          # 使能电源管理
CONFIG_ESP_SLEEP_POWER_DOWN_FLASH=y         # 深度睡眠关断 Flash
CONFIG_ESP_WIFI_ENABLED=n                   # 禁用 Wi-Fi（纯传感器节点）
```

### 日志级别

系统默认日志级别为 **INFO**。如需调试，可在 `sdkconfig.defaults` 中修改：

```ini
CONFIG_LOG_DEFAULT_LEVEL_DEBUG=y
CONFIG_LOG_MAXIMUM_LEVEL_DEBUG=y
```

---

## 许可证

本项目基于 MIT 许可证开源。详见 [LICENSE](LICENSE) 文件。

---

## 作者

- **wuKehong** — 初始开发

---

## 致谢

- [Espressif ESP-IDF](https://github.com/espressif/esp-idf) — 官方开发框架
- [FreeRTOS](https://www.freertos.org/) — 实时操作系统内核
- [InvenSense MPU6050](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/) — 六轴 IMU 传感器
- [正点原子 ATK-LORA-02](http://www.openedv.com/) — LoRa 无线串口模块

---

> ⚠️ **安全提示**：本设备仅用于工业监测与结构健康检测场景。在临界安全应用中（如医疗生命维持、航空飞行控制），请使用经过安全认证的冗余传感器系统。
