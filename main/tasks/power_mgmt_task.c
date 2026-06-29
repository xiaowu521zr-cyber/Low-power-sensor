/**
 * @file    power_mgmt_task.c
 * @brief   全局电源管理与超低功耗状态机任务实现
 *
 * 核心设计：
 * 1. 采用事件驱动型异步轮询，全面监控全机业务异动（警报、采集、下行命令）。
 * 2. 引入阶梯式电池电压生死线防御（黄线预警、红线紧急上报并拦截休眠）。
 * 3. 闭环管理外设漏电路径（LoRa 强行休眠且引脚浮空配置）。
 * 4. 完美调校硬件唤醒矩阵：EXT0 震动突发拉醒 + ULP 看门狗 + 30分钟大保底定时器。
 */

#include "tasks/power_mgmt_task.h"
#include "drivers/adc_battery.h"
#include "drivers/mpu6050.h"
#include "protocol/gateway_protocol.h"
#include "ulp/ulp_shared.h"
#include "main.h"
#include "esp_sleep.h"

static const char *TAG = "power_mgmt";

/* ================================================================
 * 全局时序踪量与状态持久化统计变量
 * ================================================================ */
static TickType_t g_last_activity_tick = 0;  // 记录最后一次发生有效业务异动（如震动、发包）的内核系统节拍戳
static TickType_t g_last_battery_check = 0;  // 记录上一次读取 ADC 电池电压的时间戳，用于时间窗口限频
static uint32_t g_sleep_count = 0;           // 本节点自芯片冷启动以来，切入深度睡眠的总次数统计计数
static uint32_t g_battery_mv = 0;            // 实时缓存最新一次测得的电池毫伏电压，供全机判定门禁

/* ================================================================
 * 电池电压健康度轮询监控层 (Battery Monitoring)
 * ================================================================ */

/**
 * @brief 定期触发 ADC 采样检查电池电量，并执行分级阶梯式保护逻辑
 */
static void check_battery(void)
{
    TickType_t now = xTaskGetTickCount();

    // 【限频防抢占】：由于 ADC 采样属于高能耗动作且占用总线，通过此机制限制每 60 秒才允许执行一次物理测量
    if ((now - g_last_battery_check) < pdMS_TO_TICKS(BATTERY_CHECK_INTERVAL_MS)) {
        return;
    }
    g_last_battery_check = now;

    uint32_t voltage_mv;
    // 调用底层硬件驱动，通过 ADC 衰减器读取电池真实毫伏数
    if (battery_read_voltage(&voltage_mv) != ESP_OK) {
        ESP_LOGW(TAG, "ADC 电池通道物理采样失败");
        return;
    }

    g_battery_mv = voltage_mv;
    uint8_t pct = battery_get_percentage(voltage_mv); // 将毫伏数转换为 0~100% 的直观百分比

    ESP_LOGI(TAG, "当前电量: %dmV,剩余百分比: %d%%", voltage_mv, pct);

    /* ----------------------------------------------------------------
     * 核心点一：动态刷新 RTC 慢速保持内存 (Retained Memory Sync)
     * ----------------------------------------------------------------
     * 必须将当前的电量信息实时同步写入 RTC 慢速保持区域。因为主 CPU 在深睡后，
     * 持续运行的 ULP 协处理器需要根据这个内存地址的数据了解电量状况，避免盲目引发高能耗动作。
     */
    uint32_t *rtc_mem = (uint32_t *)RTC_MEM_BASE;
    rtc_mem[RTC_OFFSET_BATTERY_MV / 4] = voltage_mv;

    /* 
     * 阶梯式濒死电压控制逻辑 (Multi-stage Low-Power Defense)
     */
    
    // 【情况 1】：电池电量彻底跌破强行截止防线（如 2.8V 濒死状态）
    if (voltage_mv < BATTERY_CRIT_THRESHOLD_MV) {
        ESP_LOGW(TAG, "电池电量不足");

        // 闭环危机通知：立刻在栈区组装一个最高优先级的紧急低电量警报结构体
        alarm_data_t alarm = {
            .alarm_type = ALARM_TYPE_LOW_BATTERY,
            .value = (float)voltage_mv,
            .threshold = (float)BATTERY_CRIT_THRESHOLD_MV,
            .delta = (float)(BATTERY_CRIT_THRESHOLD_MV - voltage_mv),
            .timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS,
        };

        // 通知维护人员来换电池
        gateway_protocol_send_alarm(&alarm);

        // 在持久化的 RTC 标志位中打上严重电量不足记号，并向全局 FreeRTOS 事件组广播低电量事件
        rtc_mem[RTC_OFFSET_ULP_FLAGS / 4] |= ULP_FLAG_BATTERY_LOW;
        xEventGroupSetBits(g_event_group, EVENT_LOW_BATTERY);
    } 
    // 【情况 2】：电压尚未濒死，但已跌破常态低电量黄线预警（如 3.3V 黄线）
    else if (voltage_mv < BATTERY_LOW_THRESHOLD_MV) {
        ESP_LOGI(TAG, "电池触发低电量黄线预警: %dmV", voltage_mv);
        rtc_mem[RTC_OFFSET_ULP_FLAGS / 4] |= ULP_FLAG_BATTERY_LOW; // 提早向 ULP 协处理器通报异常
    } 
    // 【情况 3】：电压处于绝对健康的安全大后方区间
    else {
        rtc_mem[RTC_OFFSET_ULP_FLAGS / 4] &= ~ULP_FLAG_BATTERY_LOW; // 及时抹除低电量异常标记
    }
}

/* ================================================================
 * 深度睡眠中央控制层 (Deep Sleep Management)
 * ================================================================ */

/**
 * @brief 休眠判断：检查当前全机各项健康和业务指标是否完全满足切入休眠的安全红线
 * @return true 代表全机完全闲置，通过审核批准休眠；false 代表有挂起业务或异常状态，强行拦截休眠
 */
static bool should_enter_deep_sleep(void)
{
    TickType_t now = xTaskGetTickCount();

    /* ----------------------------------------------------------------
     * 时间窗口检查 (Idle Timeout Window)
     * ----------------------------------------------------------------
     * 如果近期 5 秒内（DEEP_SLEEP_TIMEOUT_MS）有过任何突发活动（如刚触发过震动、刚发完无线包、
     * 或网关刚通信完），说明系统正处于交互期，必须无条件拦截休眠，继续等待。
     */
    if ((now - g_last_activity_tick) < pdMS_TO_TICKS(DEEP_SLEEP_TIMEOUT_MS)) {
        return false;
    }

    /* ----------------------------------------------------------------
     * 临迟电量锁存保护 (Critical Battery Interception)
     * ----------------------------------------------------------------
     * 如果电池已经跌破强行截止线，主核应当保持全面清醒，拒绝切入常态深度睡眠。
     * 因为一旦进深睡，设备可能会因为电压进一步跌落而再也无法醒来，必须留在常规状态执行临终异常循环轮巡或挂起。
     */
    if (g_battery_mv < BATTERY_CRIT_THRESHOLD_MV && g_battery_mv > 0) {
        ESP_LOGD(TAG, "电池电量低");
        return false;
    }

    /* ----------------------------------------------------------------
     * 悬空未处理命令原语核验 (Pending Gateway Command Check)
     * ----------------------------------------------------------------
     * 检查事件组中是否有刚从空气中截获到的网关下行控制配置指令。
     * 如果有（EVENT_CMD_RECEIVED 悬空），说明业务没办完，必须阻止休眠，优先交给指令解析任务去执行。
     */
    EventBits_t events = xEventGroupGetBits(g_event_group);
    if (events & EVENT_CMD_RECEIVED) {
        xEventGroupClearBits(g_event_group, EVENT_CMD_RECEIVED); // 消费并清除标记
        return false;
    }

    return true;            // 进入深度睡眠状态
}

/**
 * @brief 统领全机外设，配置多维硬件触发源，最终执行硬件级断电
 */
static void enter_deep_sleep(void)
{
    ESP_LOGI(TAG, "======= 系统正在准备切入第 #%d 次深度睡眠 =======", (int)(++g_sleep_count));

    /* 将关机前最后的设备数据强制写入 RTC 慢速保持域，防止掉电丢失 */
    uint32_t *rtc_mem = (uint32_t *)RTC_MEM_BASE;
    rtc_mem[RTC_OFFSET_BATTERY_MV / 4] = g_battery_mv;
    rtc_mem[RTC_OFFSET_SENSOR_ID / 4] = DEVICE_TYPE;

    // 清零主 CPU 悬空的唤醒申请标志位，准备接受全新的硬件信号
    rtc_mem[RTC_OFFSET_ULP_FLAGS / 4] &= ~ULP_FLAG_CPU_WAKE_REQ;

    /* ----------------------------------------------------------------
     * IMU 传感器硬件换挡 (换下高能耗陀螺仪，换上微安级看门哨兵)
     * ----------------------------------------------------------------
     * 此处调用 mpu6050_configure_motion_detect() 配置 MPU6050 芯片关闭内部陀螺仪，
     * 启动硬件运动自循环监测器（Cycle Mode）。在接下来的深睡期，主核彻底断电，
     * 依靠 MPU6050 在原地以极微弱的微安电流独立盯着机械震动，一旦超标，自己拉高 INT 引脚，直接闪电拉醒主核。
     */
    mpu6050_configure_motion_detect();

    /* ----------------------------------------------------------------
     * 配置 EXT0 外部硬件引脚休眠唤醒源
     * ----------------------------------------------------------------
     * 将连接 MPU6050 INT 引脚的专用 RTC 低功耗域管脚（GPIO 3）绑定为唤醒源 0 (EXT0)。
     * 配置为电平 1（高电平有效触发）。一旦 MPU6050 判定支架震动超标拉高该引脚，直接唤醒 ESP32 主核。
     */
    esp_sleep_enable_ext0_wakeup(MPU6050_INT_GPIO, 1);

    /* 使能 ULP LP-Core 协处理器唤醒，作为后台轻量化软件级定时看门狗 */
    esp_sleep_enable_ulp_wakeup();

    /* 激活最终大保底定时器唤醒
     * 哪怕现场连续几年绝对静止、没有任何震动拉醒，30分钟一到（时间单位为微秒），
     * 硬件强制主核必须强行苏醒过来一次，给基站打卡发射全生命周期体检报表，防止设备在地下失联。
     */
    esp_sleep_enable_timer_wakeup(30ULL * 60 * 1000 * 1000);

    /* 强行让板载 LoRa 模块进入硬休眠模式。
     * 并把与 LoRa 模块相连的 TX、RX、MD0 等所有引脚配置为浮空输入（FLOATING）。
     * 彻底断开源漏电路在芯片深睡期间由于电位差产生的物理漏电，将维持电流压榨到微安级。
     */
    lora_enter_sleep_mode();

    /* 关闭板载高亮状态指示灯，进入纯粹黑暗的极限制冷电能期 */
    gpio_set_level(STATUS_LED_GPIO, 0);

    // 系统大状态机跃迁标记为深度睡眠
    g_system_state = SYS_STATE_DEEP_SLEEP;

    /* 【技术调试细节】：强制推迟 100 毫秒。给板载 UART 硬件 FIFO 控制器留出最后的吐波形时间，
     * 确保上面那句 "... Zzz..." 日志被干净完整地推送到电脑串口助手上，随后再执行总閘关闭。 */
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "物理层电源准备切断... 核心进入深休眠模式... Zzz...");

    
    /* 执行硬件级主核断电，全机熄火 */
    
    esp_deep_sleep_start();

    /* 
     * 深睡后绝对死区代码 如果执行到这里，说明硬件确实发生故障
     */
    ESP_LOGE(TAG, "硬件异常！");
    g_system_state = SYS_STATE_ACTIVE;
}

/**
 * @brief 业务活动标记触发器：每当各路业务重新启动时，调用此函数重置系统闲置倒计时
 */
static void update_activity(void)
{
    g_last_activity_tick = xTaskGetTickCount();
}

/* ================================================================
 * FreeRTOS 异步控制核心任务体 (Task Implementation)
 * ================================================================ */

/**
 * @brief 全局电源管理与低功耗调度中央控制任务
 */
static void power_mgmt_task(void *arg)
{
    ESP_LOGI(TAG, "电池电压：%d mV", g_battery_mv);

    /* 开机初检：上电或复位醒来的第一个周期，强行触发一次电池采样并重置活跃时间戳 */
    check_battery();
    update_activity();

    // 默认点亮常态运行指示灯
    gpio_set_level(STATUS_LED_GPIO, 1);

    while (1) {
        /* 背景轮询：任务每隔 1 秒醒来一次，首先例行对电池进行一次健康体检 */
        check_battery();

        /* ----------------------------------------------------------------
         * 核心点六：多任务状态共享的“网格化轮询” 
         * ----------------------------------------------------------------
         * 通过非阻塞扫描多任务共享事件组，掌握任务的运行状态。
         */
        EventBits_t events = xEventGroupGetBits(g_event_group);

        // 1、姿态算法层传来警报——目前支架倾斜角处于越界严重报警状态
        if (events & EVENT_ALARM_ACTIVE) {
            update_activity();              // 只要危机未解除，高频重置活跃时间戳，死死拽住系统不允许休眠
            xEventGroupClearBits(g_event_group, EVENT_ALARM_ACTIVE); // 清除标记便于下一次捕捉
            ESP_LOGD(TAG, "当前有警报悬空未解除");
        }
        
        // 2、高频数据采集队列正在稳定刷新流入字节
        if (events & EVENT_SENSOR_READY) {
            update_activity(); // 说明液压支架当前正在剧烈运动或处于动态调整期，重置闲置倒计时
            xEventGroupClearBits(g_event_group, EVENT_SENSOR_READY);
        }

        // 3、检测到零偏漂移处于自适应慢速修正期间
        if (events & EVENT_DRIFT_DETECTED) {
            xEventGroupClearBits(g_event_group, EVENT_DRIFT_DETECTED);
            ESP_LOGD(TAG, "在温漂修正完成后进入休眠");
        }

        /* ----------------------------------------------------------------
         * 自动化休眠调度：判定全机是否满足 5 秒连续绝对闲置
         * ---------------------------------------------------------------- */
        if (should_enter_deep_sleep()) {
            enter_deep_sleep();
        }

        // /*
        //  * 突发式强行休眠事件处理：来自其他高级业务逻辑的主动申请
        //  * （例如设备检测到长期的多面绝对水平静止，判定已被现场工人拆下、塞进了物资仓库包装盒，申请立刻关机休眠）
        // */
        // if (events & EVENT_DEEP_SLEEP_REQ) {
        //     xEventGroupClearBits(g_event_group, EVENT_DEEP_SLEEP_REQ);
        //     ESP_LOGI(TAG, "接收到来自暂态姿态任务发出的强行无条件全局就地休眠特殊原语");

        //     /* 【柔性缓冲期】：强制在原地延时 2 秒，给其他背景任务（如负责回传的 LoRa 串口）留出把最后一帧应答帧吐进空气中的生存时间 */
        //     vTaskDelay(pdMS_TO_TICKS(2000));

        //     // 二次安全门禁核验通过后，强行执行系统最高优先级断电深睡闭环
        //     if (should_enter_deep_sleep()) {
        //         enter_deep_sleep();
        //     }
        // }

        /* 控制中心核心心跳监控节拍：每 1 秒在内核中醒来执行一次周密审查 */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ================================================================
 * 任务安全动态创建层 (Task Registration)
 * ================================================================ */

BaseType_t power_mgmt_task_create(void)
{
    // 调用 FreeRTOS 核心接口，注入由全局统一规划的低功耗管理专用优先级和独立堆栈深度
    BaseType_t ret = xTaskCreate(
        power_mgmt_task,        // 任务函数指针
        POWER_MGMT_TASK_NAME,   // 任务的可读文本名字
        POWER_MGMT_TASK_STACK,  // 分配给本任务独立持有的硬件堆栈尺寸（字节）
        NULL,                   // 传递给任务的入参指针（此处不需要）
        POWER_MGMT_TASK_PRIO,   // 运行优先级（3 - 中低优先级，确保不抢占采集与报警发射）
        NULL                    // 传出的任务句柄接收器（此处不需要）
    );

    // 防御型开发校验：评估当前系统剩余堆栈（RAM）是否足够安全分配本次任务空间
    if (ret == pdPASS) {
        ESP_LOGI(TAG, "全局电源省电中央控制管理任务挂载成功: 分配运行优先级=%d, 专属安全堆栈尺寸=%d 字节",
                 POWER_MGMT_TASK_PRIO, POWER_MGMT_TASK_STACK);
    } else {
        // 内存严重爆仓，触发致命异常日志
        ESP_LOGE(TAG, "致命硬件异常：内存堆栈空间严重匮乏，电源管理核心线程安装崩溃！");
    }

    return ret; // 传回创建结果成绩单
}