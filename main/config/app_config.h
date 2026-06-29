/**
 * @file    app_config.h
 * @brief   MPU6050 陀螺仪传感器节点应用层全局配置文件
 * @author  ESP32-C6 FreeRTOS 项目组
 *
 * 整个应用程序的核心配置头文件。系统所有的硬件引脚分配、
 * 软件任务参数、报警阈值以及低功耗休眠时序均在此处集中管理。
 */

#ifndef __APP_CONFIG_H__
#define __APP_CONFIG_H__

/* 引入 FreeRTOS 核心组件头文件，用于多任务同步与事件驱动 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

/* 引入 ESP-IDF 硬件及底层驱动头文件 */
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "ulp_lp_core.h"
#include "ulp_lp_core_utils.h"
#include "hal/rtc_io_ll.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * 核心系统身份识别 (System Identification)
 * ================================================================ */
#define DEVICE_TYPE                 0x01        /* 设备类型：0x01 代表当前是六轴加速度陀螺仪节点 */
#define DEVICE_ADDR_H               0x0D        /* 本机无线识别地址高字节（用于 LoRa 定向无线通信） */
#define DEVICE_ADDR_L               0x80        /* 本机无线识别地址低字节（当前完整目标定点地址为 0x0D80） */
#define FIRMWARE_VERSION            0x0100      /* 固件版本号：1.0.0 (十六进制格式) */

/* ================================================================
 * GPIO 硬件引脚分配映射表
 * ================================================================ */
/* MPU6050 专用的硬件 I2C 总线配置 */
#define I2C_MASTER_SCL_IO           GPIO_NUM_5  /* 时钟线引脚 */
#define I2C_MASTER_SDA_IO           GPIO_NUM_4  /* 数据线引脚 */
#define I2C_MASTER_NUM              I2C_NUM_0   /* 选用 ESP32-C6 的硬件 I2C 0 号外设控制器 */
#define I2C_MASTER_FREQ_HZ          400000      /* I2C 通信速率：400kHz (快速模式，确保高频采集不阻塞) */

/* MPU6050 中断引脚 (INT)
 * 核心注意：在 ESP32-C6 上，为了能在 Deep Sleep（深睡）模式下通过外部引脚电平变化唤醒主核，
 * 必须选用具备 RTC 控制器复用功能的 GPIO 引脚（GPIO 0 ~ 7 为低功耗/RTC 域引脚） */
#define MPU6050_INT_GPIO            GPIO_NUM_3  /* 映射至硬件 GPIO 3 */
#define MPU6050_INT_SEL             (1ULL << MPU6050_INT_GPIO) /* 对应的位掩码 */

/* LoRa 模块串口通信 (UART) 引脚分配 */
#define LORA_UART_NUM               UART_NUM_1  /* 选用硬件 UART 1 外设，保留 UART 0 用于 USB 日志打印 */
#define LORA_TX_GPIO                GPIO_NUM_6  /* ESP32-C6 串口发送引脚 */
#define LORA_RX_GPIO                GPIO_NUM_7  /* ESP32-C6 串口接收引脚 */
#define LORA_AUX_GPIO               GPIO_NUM_10 /* 连接 LoRa 模块的 状态指示引脚 (AUX) */
#define LORA_MD0_GPIO               GPIO_NUM_11 /* 连接 LoRa 模块的 模式控制引脚 (MD0) */
#define LORA_UART_BAUD              115200      /* 串口波特率 */
#define LORA_UART_RX_BUF_SIZE       1024        /* 串口驱动层环形接收缓冲区大小（字节） */
#define LORA_UART_TX_BUF_SIZE       512         /* 串口驱动层环形发送缓冲区大小（字节） */

/* 电池电压检测模拟通道 (ADC) */
#define BATTERY_ADC_UNIT            ADC_UNIT_1  /* 选用单次采样 ADC 单元 1 */
#define BATTERY_ADC_CHANNEL         ADC_CHANNEL_0 /* 对应 GPIO 0 模拟输入引脚 */
#define BATTERY_ADC_ATTEN           ADC_ATTEN_DB_12 /* 12dB 衰减配置：扩展输入电压检测上限至 ~3.9V */
#define BATTERY_ADC_GPIO            GPIO_NUM_0  /* 物理引脚号 */

/* 状态指示灯 GPIO */
#define STATUS_LED_GPIO             GPIO_NUM_8  /* 用于系统状态提示、断联闪烁或报警快闪 */

/* ================================================================
 * MPU6050 传感器内部寄存器配置参数
 * ================================================================ */
#define MPU6050_ADDR                0x68        /* 当 MPU6050 芯片的 AD0 引脚接地时，其 I2C 从机地址为 0x68 */
#define MPU6050_SAMPLE_RATE         200         /* 常规工作模式下数据刷新采样率：200Hz */
#define MPU6050_DLPF_CFG            2           /* 数字低通滤波器级别 2：加速度截止 94Hz，陀螺仪截止 98Hz，有效滤除电机震动噪声 */
#define MPU6050_GYRO_FS             MPU6050_GYRO_FS_2000  /* 陀螺仪全量程：±2000°/s (防止剧烈运动下波形削顶) */
#define MPU6050_ACCEL_FS            MPU6050_ACCEL_FS_16G   /* 加速度计全量程：±16g (提高高冲击力下的测量上限) */
#define MPU6050_MOT_THRESHOLD       80          /* 突发震动唤醒阈值：80mg（用于低功耗下的加速度计硬件中断唤醒） */
#define MPU6050_MOT_DURATION        2           /* 突发震动持续时间：2ms（震动超过该时间即判定为有效运动） */

/* ================================================================
 * FreeRTOS 任务运行参数配置 (堆栈与优先级分配)
 * ================================================================ */
/* 传感器常态高频数据采集任务 */
#define SENSOR_ACQ_TASK_NAME        "sensor_acq"
#define SENSOR_ACQ_TASK_STACK       4096        /* 分配 4KB 堆栈，防止数据转换及数学矩阵运算引发堆栈溢出 */
#define SENSOR_ACQ_TASK_PRIO        5           /* 常规业务中高优先级 */

/* 突发暂态异常波形捕捉任务 */
#define SENSOR_TRANSIENT_TASK_NAME  "sensor_transient"
#define SENSOR_TRANSIENT_TASK_STACK 4096
#define SENSOR_TRANSIENT_TASK_PRIO  8           /* 系统最高优先级！一旦触发姿态剧烈异动，必须无条件抢占执行 */

/* 网关下行无线命令接收与解析任务 */
#define COMMAND_TASK_NAME           "command_task"
#define COMMAND_TASK_STACK          4096
#define COMMAND_TASK_PRIO           7           /* 仅次于暂态捕捉，确保下行控制指令能够及时响应 */

/* 系统低功耗及电源健康度调度任务 */
#define POWER_MGMT_TASK_NAME        "power_mgmt"
#define POWER_MGMT_TASK_STACK       3072
#define POWER_MGMT_TASK_PRIO        3           /* 较低优先级，属于背景轮询监控任务 */

/* ================================================================
 * FreeRTOS 通信队列深度配置 (Queue Configuration)
 * ================================================================ */
#define SENSOR_DATA_QUEUE_LEN       16          /* 原始或解算传感器数据队列缓存最大长度 */
#define COMMAND_QUEUE_LEN           8           /* 接收网关控制指令的消息队列最大长度 */
#define ALARM_QUEUE_LEN             8           /* 异常触发报警消息队列的最大长度 */

/* ================================================================
 * 算法判定：安全阈值与零偏漂移过滤参数
 * ================================================================ */
#define DEFAULT_ROLL_THRESHOLD      15.0f       /* 横滚角极限触发报警阈值：±15.0 度 */
#define DEFAULT_PITCH_THRESHOLD     15.0f       /* 俯仰角极限触发报警阈值：±15.0 度 */
#define DEFAULT_YAW_THRESHOLD       30.0f       /* 航向角极限触发报警阈值：±30.0 度 */
#define DEFAULT_ACCEL_THRESHOLD     2.5f        /* 加速度绝对合矢量报警阈值：2.5g */
#define DRIFT_WINDOW_SIZE           10          /* 判定传感器零点温漂而开辟的滑动平均滤波窗口样本数 */
#define DRIFT_THRESHOLD_RATIO       0.05f       /* 温漂容忍比例：当长期零偏变化达到极限阈值的 5% 时，启动软件自校准零偏清除 */
#define ALARM_COOLDOWN_MS           5000        /* 报警冷却时间：5000ms（防止极限状态下高频无线发包造成LoRa网络风暴阻塞） */

/* ================================================================
 * 电源管理与动态省电管理参数 (Power Management)
 * ================================================================ */
#define BATTERY_LOW_THRESHOLD_MV    3300        /* 低电量预警线：锂电池电压跌破 3.3V 触发事件广播 */
#define BATTERY_CRIT_THRESHOLD_MV   3000        /* 强行关机保护线：电池电压跌破 3.0V 则不再进入常规任务，防止彻底过放损毁电芯 */
#define BATTERY_CHECK_INTERVAL_MS   60000       /* 电池监控周期：每 60 秒触发单次 ADC 测量 */
#define DEEP_SLEEP_TIMEOUT_MS       5000       /* 自动深睡倒计时：若传感器连续 5 秒处于绝对静止且未收到外部指令，系统强行切入深睡 */
#define ULP_WAKEUP_PERIOD_US        100000      /* ULP 协处理器自定时循环唤醒周期：100ms（100000 微秒） */

/* ================================================================
 * LoRa 无线射频与网关物理层通信协议配置
 * ================================================================ */
#define LORA_CHANNEL                23          /* 通信物理信道编号 23（对应 433MHz 附近的固定频段） */
#define LORA_TX_POWER               20          /* 最大射频发射功率：20dBm（约合 100mW，用以保证复杂环境穿透力） */
#define LORA_AIR_RATE               5           /* 无线空中无线速率等级代码 5（对应 19.2kbps） */
#define GATEWAY_ADDR_H              0x04        /* 目标主控中心网关的无线物理高位地址 */
#define GATEWAY_ADDR_L              0xD2        /* 目标主控中心网关的无线物理低位地址（完整接收网关目标地址为 0x04D2） */

/* ================================================================
 * FreeRTOS 全局多任务事件组位标志定义 (Event Group Bits)
 * ================================================================ */
#define EVENT_SENSOR_READY          BIT0        /* 标志位：传感器已顺利完成自检，新一轮物理数据处于可读取就绪态 */
#define EVENT_ALARM_ACTIVE          BIT1        /* 标志位：当前有任一轴向的姿态数值或者合速度越过安全警戒线 */
#define EVENT_CMD_RECEIVED          BIT2        /* 标志位：串口成功截获并校验通过了一条来自网关的下行无线AT指令 */
#define EVENT_LOW_BATTERY           BIT3        /* 标志位：当前系统供电端处于极度亏电状态 */
#define EVENT_DEEP_SLEEP_REQ        BIT4        /* 标志位：闲置超时或者主控下发了强制全机休眠的内部申请 */
#define EVENT_WOKEN_BY_SENSOR       BIT5        /* 标志位：标志当前这次启动是从深度睡眠中被 MPU6050 硬件震动所唤醒的 */
#define EVENT_DRIFT_DETECTED        BIT6        /* 标志位：检测到静态状态下由于温度或老化造成的软件基准线缓慢零漂异常 */

/* ================================================================
 * 核心全局业务交换数据结构体定义
 * ================================================================ */

/**
 * @brief MPU6050 原始数字六轴未转换数据集 (LSB)
 */
typedef struct {
    int16_t accel_x;        /* X轴加速度原始有符号数字码 */
    int16_t accel_y;        /* Y轴加速度原始有符号数字码 */
    int16_t accel_z;        /* Z轴加速度原始有符号数字码 */
    int16_t gyro_x;         /* X轴角速度原始有符号数字码 */
    int16_t gyro_y;         /* Y轴角速度原始有符号数字码 */
    int16_t gyro_z;         /* Z轴角速度原始有符号数字码 */
    int16_t temperature;    /* 内部晶振环境温度传感器数字码 */
} mpu6050_raw_data_t;

/**
 * @brief 解算完成的标准化物理单位姿态数据集
 */
typedef struct {
    float roll;             /* 结合重力过滤后的横滚角（单位：度 °） */
    float pitch;            /* 结合重力过滤后的俯仰角（单位：度 °） */
    float yaw;              /* 相对积分累加算出的水平航向角（单位：度 °） */
    float accel_mag;        /* 三轴加速度合矢量标量强度（单位：g，静态时正常理论值为 1.0g） */
    float gyro_mag;         /* 三轴角速度合矢量旋转总速率（单位：°/s） */
    float temperature_c;    /* 环境实际摄氏温度（单位：℃） */
    uint32_t timestamp_ms;  /* 该组数据采集时 FreeRTOS 内核的毫秒时钟戳 */
    uint8_t sensor_id;      /* 设备自身的硬件识别ID */
} sensor_data_t;

/**
 * @brief 用于 LoRa 无线发往基站网关的特定报警事件帧结构
 */
typedef struct {
    uint8_t alarm_type;     /* 报警成因代号：0x01=横滚越界, 0x02=俯仰越界, 0x03=航向越界, 0x04=重力冲击过载 */
    float value;            /* 发生突发异常瞬间的物理真实测量数值 */
    float threshold;        /* 此时系统内部所设定的安全防线界限值 */
    float delta;            /* 越过安全界限值的溢出差额绝对值 */
    uint32_t timestamp_ms;  /* 触发报警时间点对应的设备系统时间戳 */
} alarm_data_t;

/**
 * @brief 网关下行控制数据流命令接收结构体
 */
typedef struct {
    uint8_t cmd_type;       /* 控制命令识别码 */
    uint8_t param_len;      /* 携带附带参数的有效字节长度 */
    uint8_t params[32];     /* 存放附加指令参数的固定物理载荷缓冲区 */
} gateway_cmd_t;

/* 网关下发控制命令代号定义映射表 */
#define CMD_SET_ROLL_THRESHOLD      0x10        /* 远程修改 Roll 轴安全判定阈值 */
#define CMD_SET_PITCH_THRESHOLD     0x11        /* 远程修改 Pitch 轴安全判定阈值 */
#define CMD_SET_YAW_THRESHOLD       0x12        /* 远程修改 Yaw 轴安全判定阈值 */
#define CMD_SET_ACCEL_THRESHOLD     0x13        /* 远程修改 加速度冲击安全判定阈值 */
#define CMD_SET_MOTION_THRESHOLD    0x14        /* 远程动态调校低功耗休眠唤醒的灵敏度 */
#define CMD_SET_SAMPLE_RATE         0x20        /* 远程动态重配置正常工作模式下的运行采样率 */
#define CMD_SET_ULP_PERIOD          0x21        /* 远程动态修改低功耗核心自轮询定时周期 */
#define CMD_GET_STATUS              0x30        /* 基站索取当前节点整体健康运行状态包指令 */
#define CMD_GET_THRESHOLDS          0x31        /* 基站索取当前节点本地全部安全阈值的当前快照 */
#define CMD_GET_BATTERY             0x32        /* 基站索取当前电池精确电压报告指令 */
#define CMD_SOFT_RESET              0xFF        /* 强制主控引发全局软件热重启系统复位指令 */

/* ================================================================
 * RTC 慢速内存映射偏置表（极其重要：与 ULP LP-Core 协处理器共享）
 * * 核心原理：ESP32-C6 在主核彻底关闭进入 Deep Sleep 期间，其 RTC 慢速内存（RTC Slow Memory）
 * 依然持续保持供电。此部分偏移量用于在该内存中精准开辟空间，使得主核在休眠前写入的参数，
 * 能够被低功耗硬件或 ULP 协处理器实时读取；同时 ULP 监测到的电量或唤醒次数也能在主核醒来时被读取，
 * 实现了跨越生死的“数据共享隧道”。单位全为字节（4字节对齐）。
 * ================================================================ */
#define RTC_MEM_THRESHOLD_OFFSET    0           /* 阈值标志区基础地址偏置 */
#define RTC_MEM_ROLL_TH_OFFSET      4           /* 横滚角阈值存贮物理偏置 */
#define RTC_MEM_PITCH_TH_OFFSET     8           /* 俯仰角阈值存贮物理偏置 */
#define RTC_MEM_YAW_TH_OFFSET       12          /* 航向角阈值存贮物理偏置 */
#define RTC_MEM_ACCEL_TH_OFFSET     16          /* 加速度极限阈值存贮物理偏置 */
#define RTC_MEM_MOTION_TH_OFFSET    20          /* 突发硬件唤醒灵敏度阈值存贮物理偏置 */
#define RTC_MEM_WAKE_COUNT_OFFSET   24          /* 记录低功耗运行阶段总唤醒循环计数的存贮偏置 */
#define RTC_MEM_BATTERY_MV_OFFSET   28          /* 存贮在休眠期间测得的最新电池毫伏电压，供主核苏醒检查 */
#define RTC_MEM_ULP_PERIOD_OFFSET   32          /* 超低功耗核心自唤醒周期的固化参数偏置 */
#define RTC_MEM_CRC_OFFSET          36          /* 全局 RTC 共享数据安全校验和（CRC）存贮偏置，用于数据完整性防伪 */

/* ================================================================
 * 系统各任务间共享的全局对象句柄声明 (外部引用声明)
 * ================================================================ */
extern EventGroupHandle_t g_event_group;        /* 全局状态机和驱动条件触发的多事件通知组句柄 */
extern QueueHandle_t g_sensor_data_queue;       /* 传感器采集向后续传输交付的核心通信队列句柄 */
extern QueueHandle_t g_command_queue;           /* 串口/LoRa下行控制原语分发消息队列句柄 */
extern QueueHandle_t g_alarm_queue;             /* 紧急报警包异步无锁化传输队列句柄 */
extern SemaphoreHandle_t g_i2c_mutex;           /* 确保 I2C 总线不会在多任务环境下发生读写死锁的互斥锁 */
extern TaskHandle_t g_sensor_task_handle;       /* 数据采集任务的句柄指针（可用以实施外部强制挂起或线程销毁） */

#ifdef __cplusplus
}
#endif

#endif /* __APP_CONFIG_H__ */