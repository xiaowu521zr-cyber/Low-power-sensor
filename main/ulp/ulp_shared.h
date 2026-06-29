#ifndef __ULP_SHARED_H__
#define __ULP_SHARED_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * RTC 慢速内存布局 (主 CPU 与 ULP 协处理器公共共享空间)
 * ================================================================
 * ESP32-C6 芯片架构中 RTC_SLOW_MEM 的物理硬件基地址：0x50000000
 * 该区域物理总容量为 8KB。在主 CPU 彻底断电时，此区域由 RTC 电源域持续供电，
 * 其内部存储的数据具有掉电不丢失特性，是跨核协同的核心“数据共享隧道”。
 */

#define RTC_MEM_BASE                0x50000000

/* 核心技术点一：4 字节对齐的物理偏移量 (Byte Offsets)
 * 下方定义的数值为相对基地址的绝对字节偏移。由于使用了 32 位 (4字节) 数据类型，
 * 为了确保硬件总线访问的高效性，各字段必须严格遵循 4 字节对齐边界进行依次排列。 */

/* 传感器安全检测防线阈值存储区 (均为 32 位宽数据) */
#define RTC_OFFSET_ROLL_THRESHOLD   0x00    /* 4字节 Float：本地生效的横滚角限制阈值（单位：度 °） */
#define RTC_OFFSET_PITCH_THRESHOLD  0x04    /* 4字节 Float：本地生效的俯仰角限制阈值（单位：度 °） */
#define RTC_OFFSET_YAW_THRESHOLD    0x08    /* 4字节 Float：本地生效的航向角限制阈值（单位：度 °） */
#define RTC_OFFSET_ACCEL_THRESHOLD  0x0C    /* 4字节 Float：本地生效的重力合成冲击限制阈值（单位：g） */
#define RTC_OFFSET_MOTION_THRESHOLD 0x10    /* 1字节 Uint8：IMU 芯片硬件突发震动唤醒的 LSB 灵敏度阈值（单位：mg） */

/* 系统常态统计指标与低功耗控制区 */
#define RTC_OFFSET_WAKE_COUNT       0x14    /* 4字节 Uint32：自复位以来，ULP 协处理器定时自苏醒的总计循环次数 */
#define RTC_OFFSET_BATTERY_MV       0x18    /* 4字节 Uint32：主核睡眠前注入或 ULP 测得的最新电池健康电压（单位：mV） */
#define RTC_OFFSET_ULP_PERIOD       0x1C    /* 4字节 Uint32：低功耗状态下 ULP 定时自唤醒扫描的硬件定时器周期（单位：微秒 us） */
#define RTC_OFFSET_SENSOR_ID        0x20    /* 1字节 Uint8：固化标识当前连接的传感器硬件类型代码 (如 0x01 = 陀螺仪) */

/* ULP 控制状态字及数据校验安全区 */
#define RTC_OFFSET_ULP_FLAGS        0x24    /* 4字节 Uint32：跨核多状态异步控制打标状态字（按位进行位掩码操作） */
#define RTC_OFFSET_LAST_SAMPLE_TS   0x28    /* 4字节 Uint32：最近一次成功采集动作发生时的虚拟滚动单调时钟戳 */
#define RTC_OFFSET_ALARM_FLAG       0x2C    /* 4字节 Uint32：由 ULP 内部判定并上报给主 CPU 待处理的悬空危机告警指示位 */
#define RTC_OFFSET_CRC16            0x30    /* 2字节 Uint16：针对全套阈值参数块执行的一阶工业级 CRC 校验和，用以防伪防乱码 */

/* 核心技术点二：控制状态字按位标志定义 (ULP Flag Bits)
 * 采用位掩码 (Bit-mask) 设计，支持主核与协处理器在不互斥加锁的情况下，依靠单字节原子读写完成轻量级状态通知。 */
#define ULP_FLAG_THRESHOLD_UPDATED  BIT0    /* BIT0 置 1：通告 ULP 主核刚刚通过无线更改了控制阈值，必须重新刷写监测防线 */
#define ULP_FLAG_BATTERY_LOW        BIT1    /* BIT1 置 1：通告当前全机处于严重亏电状态，指示 ULP 收紧算法步长限制功耗 */
#define ULP_FLAG_SENSOR_FAULT       BIT2    /* BIT2 置 1：由 ULP 或主核打标，宣告 IMU 传感器出现硬件断线或底层总线死锁错误 */
#define ULP_FLAG_CPU_WAKE_REQ       BIT3    /* BIT3 置 1：动作标记，指示 ULP 执行强制触发硬件机器码以强行上电并拉醒主 CPU */
#define ULP_FLAG_FIRST_BOOT         BIT4    /* BIT4 置 1：指示当前是全机冷上电后的第一个苏醒周期，用以触发特有的软硬件初检 */

/* ================================================================
 * ULP 协处理器独立程序入口原型
 * ================================================================ */

/**
 * @brief ULP 协处理器的中央执行主入口
 * @note  底层具体的 RISC-V 汇编流水线实现在 `ulp_main.S` 文件中。
 * 当硬件 RTC 定时器倒计时归零时，会自动触发 ULP 核心跳转至此地址开始运行。
 */
void ulp_main(void);

/* ================================================================
 * 跨核内存异步读写辅助控制宏
 * * 核心技术点三：强制 volatile 关键字防编译器寄存器缓存优化
 * 由于 RTC 慢速内存的数据可在任意时刻被另一个完全独立的处理器核心（ULP 核）
 * 异步强行覆写，如果在 C 语言中不加限定，高级编译器在优化时（如开启 -O2/-O3）可能会
 * 将该内存处的数值直接缓存至主 CPU 本地的通用寄存器中，导致读取到过期的历史脏数据。
 * 使用 `*(volatile uint32_t *)` 可以无条件强迫 CPU 每次读写都必须产生真实的物理总线
 * 寻址动作，直接去片内 SRAM 硬件单元中抓取最新鲜的真实数据流。
 * ================================================================ */

/**
 * @brief 从指定的 RTC 慢速存储偏移地址处同步强制拉取一个 32 位的字数据
 */
#define RTC_MEM_RD(offset)    (*(volatile uint32_t *)(RTC_MEM_BASE + (offset)))

/**
 * @brief 向指定的 RTC 慢速存储偏移地址处同步强制刷写写入一个 32 位的字数据
 */
#define RTC_MEM_WR(offset, val)  \
    (*(volatile uint32_t *)(RTC_MEM_BASE + (offset))) = (val)

#ifdef __cplusplus
}
#endif

#endif /* __ULP_SHARED_H__ */