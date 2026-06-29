/**
 * @file    main.c
 * @brief   MPU6050 陀螺仪传感器节点 - 全局系统主入口
 *
 * 采用基于 ESP32-C6 + FreeRTOS 实时操作系统的多任务架构，并搭载 ULP RISC-V 低功耗协处理器。
 * 系统统一规划并创建 4 个核心业务任务：
 * 1. Sensor_Acq_Task (优先级 5)       - 高频读取 MPU6050 原始数据，滑动中值滤波，推入队列
 * 2. sensor_transient_Task (优先级 8) - 高优先级姿态红线盘查，捕捉物理突变阶跃并无线调度分发
 * 3. command_task (优先级 7)          - 捕获下行无线控制指令，利用 DMA+IDLE 串口接收并清洗解析
 * 4. Power_Mgmt_Task (优先级 3)       - 电源及动态省电管理，轮询电池 ADC 并实施深睡/上电控制
 *
 * 硬件拓扑：MPU6050 的物理 INT 中断引脚直连 ESP32-C6 的低功耗 RTC 域 GPIO，用以支持机械结构异动时从深睡中瞬时硬件唤醒。
 */

#include "main.h"
#include "tasks/sensor_acq_task.h"
#include "tasks/sensor_transient_task.h"
#include "tasks/command_task.h"
#include "tasks/power_mgmt_task.h"
#include "drivers/mpu6050.h"
#include "drivers/lora_uart.h"
#include "drivers/adc_battery.h"
#include "protocol/gateway_protocol.h"
#include "ulp/ulp_shared.h"

static const char *TAG = "main";

/* ================================================================
 * 全局业务及组件共享句柄 (Global Variables)
 * ================================================================ */
EventGroupHandle_t g_event_group = NULL;       /* 多任务条件同步事件组句柄 */
QueueHandle_t g_sensor_data_queue = NULL;      /* 传感器数据交换消息队列 */
QueueHandle_t g_command_queue = NULL;          /* 下行命令数据接收消息队列 */
QueueHandle_t g_alarm_queue = NULL;            /* 突发报警异步归档消息队列 */
SemaphoreHandle_t g_i2c_mutex = NULL;          /* 保证多任务抢占下 I2C 总线线程安全的互斥锁 */
TaskHandle_t g_sensor_task_handle = NULL;      /* 传感器采集任务句柄，用作外部中断交叉通知 */

system_state_t g_system_state = SYS_STATE_INIT; /* 维护全机主电能状态机的状态量 */
wake_reason_t g_wake_reason = WAKE_REASON_POWER_ON; /* 记录当前的复位或深睡唤醒成因 */

/* ================================================================
 * 全机底层组件级初始化函数
 * ================================================================ */

/**
 * @brief 初始化非易失性闪存存储器 (NVS partition)
 */
void app_init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS 闪存分区产生坏块或版本冲突，正在强行执行全盘擦除重新格式化");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS掉电非易失存储区初始化成功");
}

/**
 * @brief 自适应提取全机本次上电或从深睡中苏醒的真实成因
 */
wake_reason_t app_get_wake_reason(void)
{
    // 调用电源管理寄存器 API 提取硬件唤醒源快照
    esp_sleep_wake_cause_t cause = esp_sleep_get_wakeup_cause();

    switch (cause) {
        case ESP_SLEEP_WAKEUP_EXT0:
            // 核心路径：由连接 MPU6050 INT 引脚的高电平突发信号强行拉醒主核
            ESP_LOGI(TAG, "硬件外部 EXT0 中断");
            return WAKE_REASON_MPU6050_INT;
            
        case ESP_SLEEP_WAKEUP_TIMER:
            // 背景保底：30分钟一到的定时业务打卡唤醒
            ESP_LOGI(TAG, "定时器唤醒");
            return WAKE_REASON_TIMER;
        
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            // 冷启动：初次物理上电或按下了 RESET 硬件复位按键
            ESP_LOGI(TAG, "冷启动");
            return WAKE_REASON_POWER_ON;
    }
}

/**
 * @brief 动态创建 FreeRTOS 多任务内核同步通信对象
 */
static void app_init_rtos_objects(void)
{
    /* 1. 动态分配内核事件同步组 */
    g_event_group = xEventGroupCreate();
    configASSERT(g_event_group != NULL);        // 内存屏障断言检查，防止堆溢出导致句柄为空

    /* 2. 动态开辟各级消息队列消息缓冲区 */
    g_sensor_data_queue = xQueueCreate(SENSOR_DATA_QUEUE_LEN, sizeof(sensor_data_t));
    configASSERT(g_sensor_data_queue != NULL);      /* 内存屏障断言检查，防止堆溢出导致句柄为空 */

    g_command_queue = xQueueCreate(COMMAND_QUEUE_LEN, sizeof(gateway_cmd_t));
    configASSERT(g_command_queue != NULL);

    g_alarm_queue = xQueueCreate(ALARM_QUEUE_LEN, sizeof(alarm_data_t));
    configASSERT(g_alarm_queue != NULL);

    /* 3. 创建互斥锁（Mutex），用以防止高频数据采集和下行串口命令同时占用底层单路 I2C 硬件外设导致的死锁 */
    g_i2c_mutex = xSemaphoreCreateMutex();
    configASSERT(g_i2c_mutex != NULL);

    ESP_LOGI(TAG, "FreeRTOS 内核通信对象及无锁队列阵列建立成功");
}

/**
 * @brief 初始化基础 GPIO 引脚属性
 */
static void app_init_gpio(void)
{
    /* 1. 配置常态状态指示灯引脚：推挽输出模式 */
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << STATUS_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_conf);
    gpio_set_level(STATUS_LED_GPIO, 0);

    /* 2. 配置 MPU6050 INT 硬件唤醒引脚
     * 注意：由于该引脚在 Deep Sleep 期间要充当 EXT0 唤醒输入源，必须将其配置为
     * 具有 RTC 域功能的模拟复用状态（ESP32-C6 自动识别封装），并开启内部上拉维持静默默认电平 */
    gpio_config_t int_conf = {
        .pin_bit_mask = MPU6050_INT_SEL,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // 启用内部抗噪上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,    // 硬件配置为上升沿触发
    };
    gpio_config(&int_conf);

    ESP_LOGI(TAG, "GPIO 硬件外设引脚电气特性初始化成功");
}

/**
 * 在电源管理单元 (PMU) 中，注册挂载全套深度睡眠外部唤醒矩阵
 */
static void app_configure_sleep_wakeup(void)
{
    /* 唤醒源 A：绑定物理引脚 EXT0 触发源将 GPIO 3 设定为外部电平锁定唤醒，只要 MPU6050 运动检测器向此引脚泵入高电平 1，PMU 硬件立刻拉高供电主闸 */
    esp_sleep_enable_ext0_wakeup(MPU6050_INT_GPIO, 1);

    /* 唤醒源 B：挂载激活运行于 RTC 内存中的超低功耗 ULP RISC-V 协处理器中断触发权 */
    esp_sleep_enable_ulp_wakeup();

    /* 唤醒源 C：挂载系统最后的保底看门狗硬件定时器（固定为 30 分钟，以微秒级单位 10^6ULL 换算注入） */
    esp_sleep_enable_timer_wakeup(30 * 60 * 1000000ULL);

    ESP_LOGI(TAG, "低功耗 PMU 硬件三向唤醒源矩阵配置成功");
}

/**
 * 加载并映射 ULP LP-Core 协处理器的二进制执行映像
 * 核心技术点：汇编器二进制符号映射 (ASM Linker Magic)
 * 编译链在将独立编译的 ULP RISC-V 汇编/C 镜像链入项目时，会在最终的目标 ELF 中自动生成
 * `_binary_ulp_main_bin_start` 和 `_binary_ulp_main_bin_end` 这两个外部强符号。
 * 此处使用 C 语言的 `extern` 和 `asm` 关键字直接声明引入这两个链接期符号指针，
 * 从而在运行时计算出 ULP 二进制指令流的精确物理字节长度，并将其安全拷贝加载至 RTC 慢速专用运行内存中。
 */
static void app_load_ulp_program(void)
{
    extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
    extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");

    // 将二进制指令代码段加载至低功耗运行区
    esp_err_t ret = ulp_lp_core_load_binary(ulp_main_bin_start,
        (ulp_main_bin_end - ulp_main_bin_start));
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "ULP LP-Core 独立固件映像加载完毕 (尺寸: %d 字节)",
             (int)(ulp_main_bin_end - ulp_main_bin_start));
}

/**
 * @brief 全局底层硬件外设驱动流水线安装初始化
 */
static void app_init_hardware(void)
{
    /* 1. 初始化主控板专属 I2C 总线硬件控制器 */
    ESP_ERROR_CHECK(mpu6050_i2c_init());

    /* 2. 握手并检测 MPU6050 六轴传感器在线状态并进行量程注入 */
    ESP_ERROR_CHECK(mpu6050_init());

    /* 3. 初始化 LoRa 模块专用的 UART 串口外设并开辟 DMA 高速内存通道 */
    ESP_ERROR_CHECK(lora_uart_init());

    /* 4. 初始化电池监测专用模拟单次获取外设 (ADC Oneshot) */
    ESP_ERROR_CHECK(battery_adc_init());

    ESP_LOGI(TAG, "片上所有基础业务硬件外设底层驱动全部安装完毕");
}

/* ================================================================
 * 全局主应用程序引导入口 (Main Application Entry)
 * ================================================================ */
int app_main(void)
{
    ESP_LOGI(TAG, " 本节点专属定点通信无线物理地址: 0x%02X%02X", DEVICE_ADDR_H, DEVICE_ADDR_L);

    /* 步骤 1：第一检测时间窗口，提取当前的唤醒或冷启动硬件成因 */
    g_wake_reason = app_get_wake_reason();

    app_init_nvs();
    app_init_rtos_objects();
    app_init_gpio();
    app_configure_sleep_wakeup();

    /* 步骤 2：加载 ULP 协处理器固件，确保深睡后 RTC 内存跨核隧道的防篡改控制管理 */
    app_load_ulp_program();

    /* 步骤 3：安装所有板载芯片外设控制链 */
    app_init_hardware();

    /* 步骤 4：启动上层网关无线协议栈解析引擎 */
    gateway_protocol_init();

    /* 步骤 5：如果是系统由于首次冷上电或发生了硬复位按钮重启，
     * 必须强行对 MPU6050 内部硬件运动检测寄存器执行一次全套初试化写入，为其进入深度睡眠震动检测注入基准 */
    if (g_wake_reason == WAKE_REASON_POWER_ON ||
        g_wake_reason == WAKE_REASON_RESET) {
        mpu6050_configure_motion_detect();
    }

    /* 步骤 6：从 NVS 永久 Flash 存储区中恢复用户上次无线设定的报警阈值 */
    gateway_protocol_load_thresholds();

    /* 步骤 7：状态机跃迁，宣告全机主运行模式正式进入 ACTIVE（活跃）常态工作期 */
    g_system_state = SYS_STATE_ACTIVE;

    /* 步骤 8：创建并挂载 FreeRTOS 全套多任务并行流水线，系统会伴随优先级进行抢占和执行 */
    BaseType_t ret;

    // 创建任务 1：高频数据采集及中值滑动清洗线程 (优先级 5)
    ret = sensor_acq_task_create();
    ESP_ERROR_CHECK(ret == pdPASS ? ESP_OK : ESP_FAIL);

    // 创建任务 2：高优先级暂态阶跃突变捕捉及报警发射线程 (优先级 8 - 最高)
    ret = sensor_transient_task_create();
    ESP_ERROR_CHECK(ret == pdPASS ? ESP_OK : ESP_FAIL);

    // 创建任务 3：下行无线命令串口 DMA 捕获解析线程 (优先级 7)
    ret = command_task_create();
    ESP_ERROR_CHECK(ret == pdPASS ? ESP_OK : ESP_FAIL);

    // 创建任务 4：健康度监控与休眠定时省电控制中心线程 (优先级 3)
    ret = power_mgmt_task_create();
    ESP_ERROR_CHECK(ret == pdPASS ? ESP_OK : ESP_FAIL);

    ESP_LOGI(TAG, "多任务并行引擎创建成功，系统已开始常态化全速流转。");

    /* 特殊时序闭环：如果本次开机是从深睡中被传感器震动瞬间拉醒的，
     * 立即向全局事件组广播注入 `EVENT_WOKEN_BY_SENSOR` 事件标志，
     * 这将无条件激活后级暂态捕捉任务，迫使其在 0 毫秒延时内启动对现场第一手物理变动的全记录发射 */
    uint32_t wake_events = EVENT_WOKEN_BY_SENSOR;
    if (g_wake_reason == WAKE_REASON_MPU6050_INT) {
        xEventGroupSetBits(g_event_group, wake_events);
    }

    /* 核心技术点五：主入口线程自适应退化 Idle 机制
     * 当主函数 `app_main` 执行到此处时，全机的多任务、多硬件驱动及 ULP 协处理器均已处于内核调度接管下。
     * 本线程已顺利完成引导引导程序的历史使命。为了不消耗系统额外的计算带宽，
     * 此处让主入口进入一个长周期的常态阻塞挂起循环中（每 10 秒唤醒一次背景心跳，甚至可改用 vTaskDelete(NULL) 销毁自身）。
     * CPU 的绝对算力调度将全权让渡给 FreeRTOS 内核时间片和硬件中断服务程序。 */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
    return 0;
}