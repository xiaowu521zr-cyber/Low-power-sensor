/**
 * @file    ulp_main.S
 * @brief   ULP LP-Core (RISC-V) 汇编程序实现
 *
 * ESP32-C6 ULP LP-Core 协处理器程序：
 * - 当主 CPU 进入深度睡眠 (Deep Sleep) 时，该轻量化协处理器在后台持续运行
 * - 定期轮询 RTC 慢速保持内存中的共享状态和变量
 * - 跟踪并记录自身的低功耗定时苏醒计数
 * - 能够根据特定控制标志位，直接向主系统申请硬件级电源上电，拉醒主 CPU
 *
 * 针对 MPU6050 的设计架构：主系统的主要突发唤醒源直接来自于 MPU6050 的硬件 INT 引脚（EXT0）。
 * ULP 在此处主要扮演后台低功耗软件看门狗 (Watchdog) 以及周期性全生命周期的健康检查器。
 */

#include "ulp_shared.h"

    // 定义代码段，指定存放在专用的 ULP 慢速存储器区域，并具备可执行("ax")属性
    .section .text.ulp,"ax",@progbits
    .global ulp_main
    .type ulp_main, @function

/**
 * ULP LP-Core 核心入口函数
 * 当主 CPU 处于断电深睡时，底层的硬件 RTC 定时器会定期周期性唤醒 ULP 执行此段代码
 *
 * 核心寄存器规划约定 (Register usage):
 * a0-a1: 临时通用寄存器 (Scratch)
 * s0:    保存并跟踪低功耗唤醒计数 (Wake Count)
 * s1:    缓存从 RTC 慢速内存中读取到的全局状态标志字 (RTC Flags)
 */
ulp_main:
    /* 1. 加载 RTC 慢速共享内存的绝对基地址到指针寄存器 a0 */
    li      a0, RTC_MEM_BASE
    /* 2. 读取当前已累加的历史自唤醒次数至寄存器 s0 */
    lw      s0, RTC_OFFSET_WAKE_COUNT(a0)   /* s0 = wake_count */

    /* 3. 计数器自增 1，并将全新的计数值实时刷写回原 RTC 存储区 */
    addi    s0, s0, 1
    sw      s0, RTC_OFFSET_WAKE_COUNT(a0)

    /* 4. 读取当前跨核共享的 ULP 行为控制状态字至寄存器 s1 */
    lw      s1, RTC_OFFSET_ULP_FLAGS(a0)     /* s1 = flags */

    /* 5. 就是相当于if条件语句，判断系统是不是第一次冷启动，如果是的话，那么就不需要判断是否有报警信息，如果不是的话，那么就跳转到检查数据流程 */
    andi    t0, s1, ULP_FLAG_FIRST_BOOT
    beqz    t0, ulp_check_alarm             /* 若 t0 结果为 0，说明不是首次启动，直接跳转至常规检查流程 */

    /* 6. 核心点一：位清除 (Bit-Clear) 的 RISC-V 基础汇编变通实现
     * 由于标准 RISC-V RV32I 基础指令集没有单条直接清除特定位的指令 (如 bclr)，
     * 此处采用三步标准组合拳：先加载掩码，取反 (not) 得到反转掩码，最后执行按位与 (and) 清零目标位。
     * 此处用于及时抹除 ULP_FLAG_FIRST_BOOT（首次启动）标记，防止后续重复误判。 */
    li      t0, ULP_FLAG_FIRST_BOOT
    not     t0, t0                          /* 掩码取反 */
    and     s1, s1, t0                      /* 清零 s1 中的首次启动位 */
    sw      s1, RTC_OFFSET_ULP_FLAGS(a0)    /* 将清洗后的最新状态字同步写回内存 */

ulp_check_alarm:
    /* 7. 检查当前主核或业务链是否向 ULP 协处理器发出了强行苏醒主 CPU 的请求 */
    andi    t0, s1, ULP_FLAG_CPU_WAKE_REQ
    beqz    t0, ulp_update_timestamp        /* 若未发出申请 (t0==0)，跳过唤醒控制，进入后续时间戳更新 */

    /* 
     * 给 a0 赋予唤醒原因代码 (0x01 代表由 ULP 发起主动唤醒)
     * 因为标准 RISC-V 指令集不包含电源管理控制原语，乐鑫在底层硬件层面专门复用了
     * 自定义指令。`.word 0x0000000b` 在 ESP32-C6 RISC-V LP-Core 汇编中会被直接编译
     * 为一条硬件级触发指令。一旦流经 ULP 译码器，会自动拉高电源管理单元 (PMU) 
     * 的跨核硬中断，在微秒级时间内强行为主系统 CPU (HP-Core) 恢复供电，拉醒整个主内核。 */
    li      a0, 0x01       /* 写入唤醒原因码：0x01 = ULP 唤醒 */
    .word   0x0000000b     /* 执行乐鑫 LP-Core 专属的硬件唤醒机器码指令 */

ulp_update_timestamp:
    /* 9. 滚动更新虚拟时钟戳
     * 核心技术说明：由于 ULP 运行在高度关断的断电隔离域，无法直接去读取 HP-Core 维护的
     * 高阶复杂的 64 位底层系统微秒时间戳。因此，通常推荐直接将硬件自唤醒次数 (s0) 
     * 作为一种“虚拟单调递增时钟戳”写入内存，供主 CPU 苏醒时对照追溯时间跨度。 */
    li      a0, RTC_MEM_BASE
    sw      s0, RTC_OFFSET_LAST_SAMPLE_TS(a0)

ulp_exit:
    /* 10. 退出前最后一次将最终更新过（或清除过标志）的 flags 状态字固化刷回 RTC 内存 */
    sw      s1, RTC_OFFSET_ULP_FLAGS(a0)

    /* 11. 协处理器执行退出返回。执行 ret 指令后，ULP 核心会自动切断自身时钟线，
     * 重新进入极低功耗的休眠待机状态，挂起等待硬件 RTC 定时器的下一次周期性脉冲到来 */
    ret

    .size ulp_main, .-ulp_main

/* ================================================================
 * CRC16 循环冗余校验和计算模块 (选配，用于共享内存数据安全闭环)
 * 采用标准的 CRC-CCITT (0x1021) 工业多项式
 * ================================================================ */
    .section .text.ulp,"ax",@progbits
    .global ulp_crc16
    .type ulp_crc16, @function

/**
 * @brief 对存储在 RTC 内存中的安全阈值数据块执行一轮 CRC16 强校验
 * @note  由主 CPU 在冷启动或修改参数时调用，用以研判在经历过漫长的深度睡眠掉电周期后，
 * SRAM 内存数据是否发生了由于宇宙射线、电源低压带来的数据比特位翻转或乱码。
 */
ulp_crc16:
    /* CRC16 算法占位占位符
     * 量产阶段推荐实现：通过指针前向迭代阈值物理区，利用移位寄存器无损计算出 CRC-CCITT 并反馈 */
    ret

    .size ulp_crc16, .-ulp_crc16