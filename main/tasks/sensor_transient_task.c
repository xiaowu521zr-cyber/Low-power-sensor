/**
 * @file    sensor_transient_task.c
 * @brief   传感器暂态突变检测与温漂智能鉴别任务实现
 *
 * 任务 2: sensor_transient_task (优先级: 8 - 高优先级)
 *
 * 主要职责:
 * 1. 从数据消息队列中异步读取经过清洗平滑的传感器特征包
 * 2. 与固化在 NVS/RTC 内存中的多轴向安全防线阈值进行实时精准比对
 * 3. 运行高级差分鉴别算法，智能剥离真正的突发机械结构异动与长期的缓慢零偏温漂（防止假阳性误报）
 * 4. 判定为真实危机警报：无条件剥夺常规周期，驱动 LoRa 模块即刻突发发射高优先级无线紧急告警帧
 * 5. 判定为温漂伪警报：就地拦截静默处理，触发省电事件，通知电源管理模块强制全机直接切回深睡，避免盲目无线发射消耗电能
 *
 * 温漂与突变鉴别算法核心机理：
 * - 趋势温偏（漂移）：如果在观测窗口内（DRIFT_WINDOW_SIZE），传感器的单步均值变动率小于安全阈值的 5%，
 * 但由于长期单向累加最终越过了红线 -> 判定为零偏温漂（假警报），不予无线外发上报。
 * - 真实突变（暂态）：如果传感器特征在相邻两个采样步长内展现出剧烈的阶跃（高阶导数突变）并冲破红线 -> 判定为真实危机，立即上报。
 */

#include "tasks/sensor_transient_task.h"
#include "protocol/gateway_protocol.h"
#include "drivers/lora_uart.h"
#include "main.h"
#include <math.h>

static const char *TAG = "sensor_transient";

/* ================================================================
 * 告警冷却期管理
 * ================================================================ */
static TickType_t g_last_alarm_tick = 0; // 记录最后一次成功外发突发报警的内核节拍戳
static uint32_t g_alarm_count = 0;       // 本机成功突发告警的总计总数

/**
 * @brief 研判全机告警发射冷却时间是否已安全耗尽
 * @note  用以抑制极限状态下高频剧烈震动引起的无线告警帧重复发送，保障 LoRa 信道不被占死
 * @return true 代表冷却期已过，允许继续发射；false 代表处于冷却保护中
 */
static bool alarm_cooldown_elapsed(void)
{
    TickType_t now = xTaskGetTickCount();
    return ((now - g_last_alarm_tick) > pdMS_TO_TICKS(ALARM_COOLDOWN_MS));
}

/* ================================================================
 * 真实突变暂态 VS 缓慢零偏温漂 智能鉴别核心算法
 * ================================================================ */

/**
 * @brief 轻量级历史追踪环形队列结构体（用于一阶无锁微积分求导计算）
 */
typedef struct {
    float history[DRIFT_WINDOW_SIZE]; // 存放历史窗口净荷的静态环形阵列
    uint8_t idx;                      // 当前待写入的覆盖索引指针
    uint8_t count;                    // 当前已缓存的有效样本数
} value_history_t;

/**
 * @brief 初始化历史追踪环形队列
 */
static void history_init(value_history_t *h)
{
    memset(h, 0, sizeof(value_history_t));
}

/**
 * @brief 向环形队列压入一个新的时序采样样本
 */
static void history_push(value_history_t *h, float val)
{
    h->history[h->idx] = val;
    h->idx = (h->idx + 1) % DRIFT_WINDOW_SIZE; // 指针自增并循环回绕
    if (h->count < DRIFT_WINDOW_SIZE) h->count++;
}

/**
 * @brief 智能中央鉴别器：判定当前冲破安全红线的行为是真实的物理突变还是假阳性温漂
 *
 * 算法原理：
 * - 真实突变：波形展现出极高的一阶时域导数，能量在极短时间内爆发（突发阶跃）。
 * - 长期温漂：由于温度、元器件老化产生的基准线缓慢运移，表现为平滑的低频趋势线。
 *
 * @param h         关联轴向的时序历史跟踪阵列句柄
 * @param current   当前引发超限的最新真实物理测量值
 * @param threshold 当前正生效的本地安全红线设定值
 * @return true 代表确凿无疑的真实危机暂态（触发警报），false 代表假阳性温偏（拦截并静默）
 */
static bool is_genuine_transient(value_history_t *h, float current, float threshold)
{
    // 如果观测窗口的历史样本还没有攒满，缺乏长期趋势参照物，基于安全第一原则，默认判定为真实危险
    if (h->count < DRIFT_WINDOW_SIZE) {
        return true; 
    }

    /* 计算滑动观测窗口内的总差分变化斜率
     * 核心技术点说明：由于 `history_push` 刚刚执行过，此时 `h->idx` 已经自动自增并指向了
     * 队列中“次最旧”的历史元素（即 9 个采样周期前的波形点，因为最旧的那个刚刚被 `current` 覆写了）。
     * `current` 与 `oldest` 作差再除以窗口跨度，即可精准抽象出该时间滑动段内的一阶平均变化率（导数）。 */
    float oldest = h->history[(h->idx) % DRIFT_WINDOW_SIZE];
    float total_change = fabsf(current - oldest);
    float avg_change_per_sample = total_change / DRIFT_WINDOW_SIZE;

    /* 动态确立温漂极限防线范围
     * 比如横滚角预警红线为 15.0 度，系统设定的温漂斜率容忍比率为 5%。
     * 则判定为漂移的单步最大允许变化率 = 15.0 * 0.05 = 0.75 度/步。
     * 只要整体斜率低于此防线，说明波形是在以极其平缓的步调龟速爬行，基本判定为环境温漂行为。 */
    float drift_max_delta = threshold * DRIFT_THRESHOLD_RATIO;

    if (avg_change_per_sample < drift_max_delta) {
        /* 波形呈现平滑的趋势变动 -> 确诊为缓慢温漂 */
        ESP_LOGD(TAG, "检测到长期零偏温漂行为：滑动窗口均值变动斜率=%.4f < 温漂极限允许斜率=%.4f ",
                 avg_change_per_sample, drift_max_delta);
        return false;
    }

    /* 一阶微积分阶跃捕捉 (Jump Detection)
     * 为了防范高频陡峭的冲击波，对最新进入的相邻两个相邻样本进行一阶向后差分运算。 */
    float last = h->history[(h->idx - 1 + DRIFT_WINDOW_SIZE) % DRIFT_WINDOW_SIZE]; // 当前刚刚压入的 current
    float second_last = h->history[(h->idx - 2 + DRIFT_WINDOW_SIZE) % DRIFT_WINDOW_SIZE]; // 紧随其后的前一帧样本
    float jump = fabsf(last - second_last);

    // 如果相邻两步的能量突变差额超过了安全总防线的 30%，代表遭遇了剧烈的机械阶跃响应，属于典型的确凿危险暂态
    if (jump > threshold * 0.3f) {
        /* 捕获时域高频突变阶跃 -> 确诊为物理危机暂态 */
        ESP_LOGD(TAG, "突变跳变幅值=%.2f", jump);
        return true;
    }

    return true;  /* 无法给出高置信度判定时，出于安全兜底策略，默认允许执行上报 */
}

/* ================================================================
 * 多轴向边界超限安全核验
 * ================================================================ */

/**
 * @brief 对清洗后的优质数据包进行多轴向联动边界红线盘查
 * @return 组装好的告警元数据包；若所有轴向均安全，返回的结构体中 alarm_type 标志域为 0
 */
static alarm_data_t check_thresholds(const sensor_data_t *data)
{
    alarm_data_t alarm = {0};

    // 从协议层统一获取当前全机生效的最新安全防线快照
    float roll_th  = gateway_get_roll_threshold();
    float pitch_th = gateway_get_pitch_threshold();
    float yaw_th   = gateway_get_yaw_threshold();
    float accel_th = gateway_get_accel_threshold();

    /* 横滚角 (Roll) 绝对值边界盘查 */
    if (fabsf(data->roll) > roll_th) {
        alarm.alarm_type = ALARM_TYPE_ROLL;
        alarm.value = data->roll;
        alarm.threshold = roll_th;
        alarm.delta = fabsf(data->roll) - roll_th; // 记录溢出破坏额
    }
    /* 俯仰角 (Pitch) 绝对值边界盘查 */
    else if (fabsf(data->pitch) > pitch_th) {
        alarm.alarm_type = ALARM_TYPE_PITCH;
        alarm.value = data->pitch;
        alarm.threshold = pitch_th;
        alarm.delta = fabsf(data->pitch) - pitch_th;
    }
    /* 联动核验 3：航向角 (Yaw) 积分超限盘查（通常设为较低灵敏度） */
    else if (fabsf(data->yaw) > yaw_th) {
        alarm.alarm_type = ALARM_TYPE_YAW;
        alarm.value = data->yaw;
        alarm.threshold = yaw_th;
        alarm.delta = fabsf(data->yaw) - yaw_th;
    }
    /* 联动核验 4：三轴加速度合矢量总能量标量过载冲击盘查 */
    else if (data->accel_mag > accel_th) {
        alarm.alarm_type = ALARM_TYPE_ACCEL;
        alarm.value = data->accel_mag;
        alarm.threshold = accel_th;
        alarm.delta = data->accel_mag - accel_th;
    }

    // 若判定发生了超限事件，将当前解算该帧时的系统时间戳一并封存打包入痕迹包中
    if (alarm.alarm_type != 0) {
        alarm.timestamp_ms = data->timestamp_ms;
    }

    return alarm;
}

/* ================================================================
 * FreeRTOS 任务主逻辑体
 * ================================================================ */

/**
 * @brief 暂态异动捕捉与无线告警最高优先级突发任务
 */
static void sensor_transient_task(void *arg)
{
    ESP_LOGI(TAG, "暂态突变异常捕捉及高优先级告警响应主任务已顺利建立");

    sensor_data_t data;
    value_history_t roll_hist, pitch_hist, accel_hist;
    
    history_init(&roll_hist);
    history_init(&pitch_hist);
    history_init(&accel_hist);

    while (1) {
        /* 任务流常态阻塞。专门挂载接收由前级高频采集任务（`sensor_acq_task`）
         * 清洗平滑并塞入消息队列中的最新特征包。阻塞超时设为 100ms。 */
        if (xQueueReceive(g_sensor_data_queue, &data, pdMS_TO_TICKS(100)) != pdTRUE) {
            /* 消息队列此时为空，无传感器流流入，继续保持挂起等待 */
            continue;
        }

        /* 实时向三个核心运动学物理轴向的环形历史队列中泵入最新数据，作为微分鉴别算力基础基准 */
        history_push(&roll_hist, data.roll);
        history_push(&pitch_hist, data.pitch);
        history_push(&accel_hist, data.accel_mag);

        /* 触发全轴边界检查 */
        alarm_data_t alarm = check_thresholds(&data);

        // 如果边界检查结果显示某轴发生了数据越过安全红线的动作
        if (alarm.alarm_type != 0) {
            ESP_LOGI(TAG, "系统拦截到越界超限信号：告警成因代码=%d, 现场越界测量值=%.2f, 本地安全防线=%.2f",
                     alarm.alarm_type, alarm.value, alarm.threshold);

            /* 启动核心智能中央鉴别算力：确诊是环境零偏缓慢漂移还是真实危机暂态 */
            bool genuine = false;
            switch (alarm.alarm_type) {
                case ALARM_TYPE_ROLL:
                    genuine = is_genuine_transient(&roll_hist, data.roll, gateway_get_roll_threshold());
                    break;
                case ALARM_TYPE_PITCH:
                    genuine = is_genuine_transient(&pitch_hist, data.pitch, gateway_get_pitch_threshold());
                    break;
                case ALARM_TYPE_ACCEL:
                    genuine = is_genuine_transient(&accel_hist, data.accel_mag, gateway_get_accel_threshold());
                    break;
                default:
                    genuine = true; // 其余基础事件默认执行保守放行策略
                    break;
            }

            /* 假阳性温漂就地拦截静默机制 */
            if (!genuine) {
                // 向全局事件组发送打标通知：宣告当前遭遇温偏零漂，同时向电源管理发送特殊强制申请，催促全机由于空闲直接切回深睡状态
                xEventGroupSetBits(g_event_group, EVENT_DRIFT_DETECTED);
                xEventGroupSetBits(g_event_group, EVENT_DEEP_SLEEP_REQ);
                continue; // 拦截并终结本帧业务流，直接跳回循环头部，防止下方LoRa和信号量被误触发
            }

            /* -------- 真实物理危机暂态告警发送流水线 - 无条件立即突发外发 -------- */

            /* 核验冷却定时限制，确保物理层没有处于保护闭锁期 */
            if (!alarm_cooldown_elapsed()) {
                ESP_LOGW(TAG, "上一帧数据越界，当前帧数据未触发越界，本帧数据被抛弃。");
                continue;
            }

            /* 将危机痕迹数据异步无锁地推入全局报警归档历史日志队列，留存底层痕迹 */
            xQueueSend(g_alarm_queue, &alarm, 0);

            /* 无视常态周期，立即调用底层接口强行组装无线协议，通过串口及硬件 DMA 向 LoRa 发射模块灌入二进制流 */
            esp_err_t ret = gateway_protocol_send_alarm(&alarm);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "报警信息已经发送，报警代码：.2f，超过阈值数据：.2f\r\n",
                         alarm.alarm_type, alarm.delta);
                
                // 同步刷新上一次成功发包的时间戳节拍，刷新发包计数器
                g_last_alarm_tick = xTaskGetTickCount();
                g_alarm_count++;

                /* 触发全局核心事件通知位：宣告系统处于警报激活应战态，拉开电源管理层面的休眠空闲倒计时 */
                xEventGroupSetBits(g_event_group, EVENT_ALARM_ACTIVE);
            } else {
                ESP_LOGI(TAG, "报警信息发送失败，错误代码：%d\r\n", ret);
            }
        } else {
            /* 如果红线盘查显示全机所有轴向完全安全平稳 */
            // 背景常态控制：每隔约 60 秒的正常周期，在调试端背景打印输出当前健康的运动学物理量快照，方便观察监测
            static uint32_t sample_count = 0;
            sample_count++;

            // 基于采样率与计数器进行分频周期控制：(采样率 * 60) 个样本即代表时间走过了 60 秒
            if (sample_count % (MPU6050_SAMPLE_RATE * 60) == 0) {
                ESP_LOGD(TAG, "全机常态巡检健康背景数据: 横滚=%.2f 俯仰=%.2f 合加速度=%.2f 内部温标=%.1fC",
                         data.roll, data.pitch, data.accel_mag, data.temperature_c);

            }
        }
    }
}

/* ================================================================
 * 任务安全动态挂载管理
 * ================================================================ */

/**
 * @brief 动态创建并建立姿态暂态突变捕捉及告警抢占高优先级核心处理线程
 */
BaseType_t sensor_transient_task_create(void)
{
    // 注入全局分配的专用最高任务运行优先级（优先度 8）和专属任务独立物理堆栈容量
    BaseType_t ret = xTaskCreate(
        sensor_transient_task,
        SENSOR_TRANSIENT_TASK_NAME,
        SENSOR_TRANSIENT_TASK_STACK,
        NULL,
        SENSOR_TRANSIENT_TASK_PRIO, // 极高优先级，确保突发异动能够完美无死角抢占执行
        NULL
    );

    if (ret == pdPASS) {
        ESP_LOGI(TAG, "姿态暂态超限高优先级主处理线程成功创建挂载: 当前配置运行优先级=%d, 专属安全堆栈深度=%d 字节",
                 SENSOR_TRANSIENT_TASK_PRIO, SENSOR_TRANSIENT_TASK_STACK);
    } else {
        ESP_LOGE(TAG, "致命内部异常：全机内存爆满崩溃，无法成功挂载姿态捕捉高优先级控制核！");
    }

    return ret;
}