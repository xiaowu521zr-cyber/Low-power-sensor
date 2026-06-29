/**
 * @file    gateway_protocol.h
 * @brief   基站网关通信协议定制层头文件
 *
 * 定义了传感器节点与网关之间无线通信的数据包物理帧格式，
 * 以及网关下发控制命令的闭环协议规约。
 */

#ifndef __GATEWAY_PROTOCOL_H__
#define __GATEWAY_PROTOCOL_H__

#include "config/app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * 协议数据帧及标志位定义 (Protocol Frame Definitions)
 * ================================================================ */
#define PROTOCOL_SYNC_WORD          0xA5A5  /* 协议专属帧同步字（魔术字），用于在连续流中捕捉帧起始 */
#define PROTOCOL_MAX_PAYLOAD        128     /* 单个数据包允许的最大有效载荷边界限制（字节） */

/* 无线通信帧类型识别码 (Frame types) */
#define FRAME_TYPE_DATA             0x01    /* 定时常规传感器物理数据上报帧 */
#define FRAME_TYPE_ALARM            0x02    /* 突发极限超限紧急报警帧（最高空中优先级） */
#define FRAME_TYPE_STATUS           0x03    /* 设备全生命周期运行状态体检报表帧 */
#define FRAME_TYPE_ACK              0x04    /* 无线双向握手应答帧（确认收到） */
#define FRAME_TYPE_CMD              0x10    /* 来自基站网关的下行控制配置命令 */
#define FRAME_TYPE_CMD_RESP         0x11    /* 节点对下行控制命令执行结果的闭环应答帧 */

/* 异动报警成因分类代码 (Alarm types) */
#define ALARM_TYPE_ROLL             0x01    /* 横滚角（Roll）越界报警 */
#define ALARM_TYPE_PITCH            0x02    /* 俯仰角（Pitch）越界报警 */
#define ALARM_TYPE_YAW              0x03    /* 航向角（Yaw）越界报警 */
#define ALARM_TYPE_ACCEL            0x04    /* 重力加速度合矢量冲击过载报警（如受到撞击或跌落） */
#define ALARM_TYPE_TEMP             0x05    /* 环境温度异常超极限报警 */
#define ALARM_TYPE_LOW_BATTERY      0x06    /* 电池电量处于极亏电状态预警 */
#define ALARM_TYPE_SENSOR_FAULT     0xFF    /* IMU 传感器芯片通信断开或内部自检硬件故障 */

/* ================================================================
 * 物理数据包结构体映射表
 * * 核心技术点：__attribute__((packed))
 * 强制编译器取消结构体内部的字节对齐填充（No Padding），使各字段紧凑排列。
 * 这是确保内存数据布局与无线 LoRa 发射/串口线缆传输的二进制字节流绝对一致的关键。
 * ================================================================ */

/**
 * @brief 所有协议数据帧通用的公共基础帧头结构体
 */
typedef struct __attribute__((packed)) {
    uint16_t sync;              /* 帧同步魔术字：固定为 0xA5A5 */
    uint8_t  frame_type;        /* 帧类型代码（如 FRAME_TYPE_DATA 等） */
    uint8_t  src_addr_h;        /* 源节点（发送方）无线物理地址高字节 */
    uint8_t  src_addr_l;        /* 源节点（发送方）无线物理地址低字节 */
    uint8_t  dst_addr_h;        /* 目标节点（接收方）无线物理地址高字节 */
    uint8_t  dst_addr_l;        /* 目标节点（接收方）无线物理地址低字节 */
    uint8_t  seq_num;           /* 滚动流水包序号（用于丢包率统计和重包去重判别） */
    uint8_t  payload_len;       /* 后续紧跟的业务有效载荷的精确字节长度 */
} proto_header_t;

/**
 * @brief 传感器常态物理数据汇报帧的有效载荷结构体
 */
typedef struct __attribute__((packed)) {
    float    roll;              /* 横滚角（单位：度 °） */
    float    pitch;             /* 俯仰角（单位：度 °） */
    float    yaw;               /* 航向角（单位：度 °） */
    float    accel_mag;         /* 加速度三轴合矢量模长标量（单位：g） */
    float    gyro_mag;          /* 角速度三轴合矢量模长标量（单位：°/s） */
    float    temp_c;            /* 核心环境真实温度（单位：摄氏度 ℃） */
    uint32_t timestamp;         /* 该组数据采集时的本地 FreeRTOS 系统毫秒时间戳 */
} proto_data_payload_t;

/**
 * @brief 突发紧急安全警报帧的有效载荷结构体
 */
typedef struct __attribute__((packed)) {
    uint8_t  alarm_type;        /* 报警成因代号（如 ALARM_TYPE_ROLL 等） */
    float    current_value;     /* 引发本次触发红线的现场实时真实测量值 */
    float    threshold_value;   /* 触发当时本地正生效的安全警戒红线设定值 */
    float    delta;             /* 越过警戒线的绝对溢出差额值 */
    uint32_t timestamp;         /* 警报突发瞬间的本地系统毫秒时间戳 */
    uint32_t battery_mv;        /* 危机发生时刻打包进去的当前供电电压（单位：mV） */
} proto_alarm_payload_t;

/**
 * @brief 设备全生命周期健康运行快照帧的有效载荷结构体
 */
typedef struct __attribute__((packed)) {
    uint8_t  device_type;       /* 固件硬件设备类型代码 (当前 0x01 = 陀螺仪节点) */
    uint16_t firmware_ver;      /* 当前运行的固件版本号 (例如 0x0100 代表 v1.0.0) */
    uint32_t uptime_ms;         /* 自本次复位开机以来的总运行时间（单位：毫秒） */
    uint32_t battery_mv;        /* 当前电池经过过采样后的真实端电压（单位：mV） */
    uint8_t  sensor_status;     /* 传感器硬件运行状态字（0x01代表在线健康，其余代表对应轴异常） */
    uint32_t sample_count;      /* 自本次复位开机以来，累计成功发送的常规物理帧总数 */
    uint32_t alarm_count;       /* 自本次复位开机以来，累计成功突发的紧急报警帧总数 */
    uint8_t  ulp_wake_count;    /* 主核睡眠期间，超低功耗 ULP 协处理器自定时唤醒检查的总次数 */
} proto_status_payload_t;

/**
 * @brief 基站下行控制命令帧的有效载荷结构体
 */
typedef struct __attribute__((packed)) {
    uint8_t  cmd_type;          /* 网关下发控制指令类型代号（如 CMD_SET_ROLL_THRESHOLD 等） */
    uint8_t  param_len;         /* 本条指令随带的后续附加有效参数长度 */
    uint8_t  params[64];        /* 存放随带配置参数的固定物理载荷缓冲区 */
} proto_cmd_payload_t;

/* ================================================================
 * 协议栈驱动级导出的 API 接口函数原型声明
 * ================================================================ */

/**
 * @brief 协议栈初始化
 * @note  开辟内部统计计数，并将初始防线阈值同步深嵌入 RTC 慢速共享内存中，打通跨核通信
 */
void gateway_protocol_init(void);

/**
 * @brief 从永久非易失性存储器 (NVS) 中检索并恢复个性化安全阈值参数
 */
void gateway_protocol_load_thresholds(void);

/**
 * @brief 将当前的运行阈值参数同时序列化同步存入 NVS 闪存与 RTC 慢速保持内存中
 */
esp_err_t gateway_protocol_save_thresholds(void);

/**
 * @brief 组装并无线向基站定向发射一帧常规六轴物理健康数据包
 * @param data 已解算转换为标准单位的姿态数据集指针
 */
esp_err_t gateway_protocol_send_data(const sensor_data_t *data);

/**
 * @brief 捕捉到危险波形时，无条件立即序列化并无线发射一帧高优先级紧急警报包
 * @param alarm 包含现场溢出特征、瞬时电压的异常痕迹数据集指针
 */
esp_err_t gateway_protocol_send_alarm(const alarm_data_t *alarm);

/**
 * @brief 组装并无线发射一帧设备全生命周期统计运行快照报表
 */
esp_err_t gateway_protocol_send_status(void);

/**
 * @brief 针对基站网关下发的某一条指令进行结果反馈（完成指令的端到端闭环确认机制）
 * @param cmd 引起本次闭环应答的原始下行控制指令句柄
 * @param result 本地业务层的具体最终执行状态状态码 (如 0x00=成功，0x01=非法参数等)
 */
esp_err_t gateway_protocol_send_cmd_response(gateway_cmd_t *cmd, uint8_t result);

esp_err_t gateway_protocol_enqueue_cmd(const uint8_t *raw_data, uint16_t len);

/**
 * @brief 获取当前驱动层内部维护滚动的最新无线发包流水序号
 */
uint8_t gateway_protocol_get_seq_num(void);

/* -------- 本地安全防线阈值统一管理接口 (Getters & Setters) -------- */
float gateway_get_roll_threshold(void);
float gateway_get_pitch_threshold(void);
float gateway_get_yaw_threshold(void);
float gateway_get_accel_threshold(void);

void gateway_set_roll_threshold(float val);
void gateway_set_pitch_threshold(float val);
void gateway_set_yaw_threshold(float val);
void gateway_set_accel_threshold(float val);

#ifdef __cplusplus
}
#endif

#endif /* __GATEWAY_PROTOCOL_H__ */