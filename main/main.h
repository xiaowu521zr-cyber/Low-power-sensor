#ifndef __MAIN_H__
#define __MAIN_H__

#include "config/app_config.h"

/* ================================================================
 * 系统苏醒/复位成因跟踪枚举 (Wake-up Reason Tracking)
 * * 核心技术点：由于 ESP32-C6 从深度睡眠（Deep Sleep）中醒来时会经历一次
 * 完整的芯片级硬件复位（从 Bootloader 重新执行），常规的 FreeRTOS 内存变量会全部丢失。
 * 主核在上电初始阶段必须通过该枚举值来判别苏醒成因，从而决定是执行冷启动全套配置
 * 还是直接切入高速捕捉任务。
 * ================================================================ */
typedef enum {
    WAKE_REASON_POWER_ON = 0,       /* 1. 冷启动：物理开机上电或按下主板硬件复位按键 */
    WAKE_REASON_MPU6050_INT,        /* 2. 核心触发：由 MPU6050 INT 引脚通过 EXT0 硬件机制瞬时拉醒 */
    WAKE_REASON_TIMER,              /* 3. 例行打卡：达到 30 分钟保底定时器设定的休眠截止时间触发苏醒 */
    WAKE_REASON_RESET,              /* 4. 软热重启：由于截获网关控制原语或异常引发的软件调用 `esp_restart()` */
} wake_reason_t;

/* ================================================================
 * 系统主电源电能状态机枚举 (System State Machine)
 * * 核心管理机理：用于给全局背景任务（如 `power_mgmt_task`）提供明确的
 * 业务流阶段标记。指导外设进行能耗换挡，确保在 ACTIVE 状态下保证算力，在 SLEEP 状态下杜绝漏电。
 * ================================================================ */
typedef enum {
    SYS_STATE_INIT = 0,             /* 1. 初始化阶段：全机外设驱动及 FreeRTOS 内核对象正在建立映射 */
    SYS_STATE_ACTIVE,               /* 2. 全速运行状态：全套任务正常就绪，I2C 保持高频采集，LoRa 允许无线突发 */
    SYS_STATE_MONITORING,           /* 3. 轻量监控状态：降低外设动作，降低无意义的能耗开销 */
    SYS_STATE_PRE_SLEEP,            /* 4. 准备休眠状态：正在进行最后的数据日志冲刷、保存快照并调校 IMU 中断寄存器 */
    SYS_STATE_DEEP_SLEEP,           /* 5. 深度睡眠状态：主 CPU 彻底断电关闭，全机切入微安级极致省电区 */
    SYS_STATE_WAKING,               /* 6. 苏醒恢复状态：捕捉到上电信号，主系统正在重新建立总线链路 */
} system_state_t;

/* ================================================================
 * 跨任务共享的全局外部引用变量声明 (Global Variables)
 * ================================================================ */
extern system_state_t g_system_state; /* 全局电能主状态机当前快照 */
extern wake_reason_t g_wake_reason;   /* 本次系统苏醒/上电的真实物理成因 */

/* ================================================================
 * 系统引导级与状态机控制函数原型声明 (Function Prototypes)
 * ================================================================ */

/**
 * @brief 初始化或重置非易失性 Flash 存储器 (NVS Partition)
 */
void app_init_nvs(void);

/**
 * @brief 动态抓取并识别底层的硬件寄存器，反馈本次开机的真实苏醒成因
 * @return wake_reason_t 苏醒原因代码
 */
wake_reason_t app_get_wake_reason(void);

/**
 * @brief 安全更新全机电源主状态机的当前运行阶段
 * @param state 目标迁徙的状态代码
 */
void app_set_system_state(system_state_t state);

/**
 * @brief 获取当前设备所处的电能状态机快照
 */
system_state_t app_get_system_state(void);

#endif /* __MAIN_H__ */