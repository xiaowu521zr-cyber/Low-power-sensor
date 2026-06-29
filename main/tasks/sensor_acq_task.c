/**
 * @file    sensor_acq_task.c
 * @brief   传感器数据采集任务实现
 *
 * 任务 1: Sensor_Acq_Task (优先级: 5 - 中等)
 *
 * 主要职责:
 * 1. 以配置的采样率定期读取 MPU6050 六轴原始数据
 * 2. 应用滑动中值滤波器滤除高频随机噪声与毛刺
 * 3. 实施数据有效性检查（防止 FIFO 溢出、数据冻结/卡死）
 * 4. 计算衍生运动学指标（如姿态角、合矢量模长）
 * 5. 将校验通过的干净数据推入 sensor_data_queue 消息队列
 * 6. 处理深睡唤醒时序：读取并清除 IMU 的中断状态寄存器，随后拉取完整波形
 */

#include "tasks/sensor_acq_task.h"
#include "drivers/mpu6050.h"
#include "main.h"

static const char *TAG = "sensor_acq";

/* ================================================================
 * 滑动中值滤波器实现 (Median Filter)
 * ================================================================ */
#define FILTER_WINDOW_SIZE  5 // 滤波窗口大小，奇数方便取绝对中位数

typedef struct {
    float buffer[FILTER_WINDOW_SIZE]; // 环形缓冲区，存放最近周期的历史样本
    uint8_t index;                    // 当前缓冲区写入指针位置
    uint8_t count;                    // 当前已填入的有效样本总数
    bool filled;                      // 缓冲区是否已写满（判定滤波器是否开始生效）
} median_filter_t;

/**
 * @brief 初始化中值滤波器结构体
 */
static void median_filter_init(median_filter_t *f)
{
    memset(f, 0, sizeof(median_filter_t));
}

/**
 * @brief 向中值滤波器注入新样本并获取当前窗口的中位数
 * @note 相比于算术平均滤波，中值滤波能完美剔除由于脉冲干扰或硬件瞬态抖动产生的尖峰毛刺
 */
static float median_filter_update(median_filter_t *f, float new_val)
{
    // 将新数据存入环形缓冲区并更新移动指针
    f->buffer[f->index] = new_val;
    f->index = (f->index + 1) % FILTER_WINDOW_SIZE;

    // 递增计数器直到填满整个标准窗口
    if (f->count < FILTER_WINDOW_SIZE) {
        f->count++;
    }
    if (f->count == FILTER_WINDOW_SIZE) {
        f->filled = true;
    }

    // 在窗口未完全填满前，缺乏足够历史参照，直接返回当前原始值
    if (!f->filled) {
        return new_val;
    }

    /* 在局部小数组上执行插入排序以获取中位数
     * 拷贝一份环形缓冲区的快照，防止污染历史记录，再进行原位排序 */
    float sorted[FILTER_WINDOW_SIZE];
    memcpy(sorted, f->buffer, sizeof(sorted));

    for (int i = 1; i < FILTER_WINDOW_SIZE; i++) {
        float key = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }

    // 排序完成后，直接返回处于正中间位置的样本值
    return sorted[FILTER_WINDOW_SIZE / 2];
}

/* ================================================================
 * 零偏漂移与数据卡死检测 (Drift Detection)
 * ================================================================ */
typedef struct {
    float samples[DRIFT_WINDOW_SIZE]; // 历史长时间观测窗口缓冲区
    uint8_t index;                    // 环形队列写入指针
    uint8_t count;                    // 有效样本计数
} drift_detector_t;

/**
 * @brief 初始化漂移检测器
 */
static void drift_detector_init(drift_detector_t *d)
{
    memset(d, 0, sizeof(drift_detector_t));
}

/**
 * @brief 实时研判传感器是否发生缓慢漂移或底层总线卡死
 * @note  核心技术点：方差统计评估
 * 如果传感器处于常态静止，数据波形应包含随机的白噪声；
 * 若计算出的统计方差极低（无限接近于 0），说明数据已经冻结无任何微小波动，
 * 这通常意味着硬件总线挂起、传感器失灵，或者是算法陷入了伪静止漂移状态。
 * @return true 代表捕获到潜在的漂移或卡死故障
 */
static bool drift_detector_check(drift_detector_t *d, float current_val)
{
    d->samples[d->index] = current_val;
    d->index = (d->index + 1) % DRIFT_WINDOW_SIZE;
    if (d->count < DRIFT_WINDOW_SIZE) d->count++;

    // 观测窗口样本不足时，跳过评估
    if (d->count < DRIFT_WINDOW_SIZE) return false;

    /* 运用期望公式快速计算方差：Var(X) = E[X^2] - (E[X])^2 */
    float sum = 0, sum_sq = 0;
    for (int i = 0; i < DRIFT_WINDOW_SIZE; i++) {
        sum += d->samples[i];
        sum_sq += d->samples[i] * d->samples[i];
    }

    float mean = sum / DRIFT_WINDOW_SIZE;
    float variance = (sum_sq / DRIFT_WINDOW_SIZE) - (mean * mean);

    /* 若统计方差长期低于 0.001，则判定数据失去活性，疑似产生硬件卡死或严重的零偏温漂 */
    return (variance < 0.001f);
}

/* ================================================================
 * FreeRTOS 任务主逻辑
 * ================================================================ */

/**
 * @brief 传感器高频数据采集与清洗任务
 */
static void sensor_acq_task(void *arg)
{
    ESP_LOGI(TAG, "传感器高频数据采集清洗主任务已挂载启动");

    /* 各功能组件实例化与初始化 */
    median_filter_t filt_roll, filt_pitch, filt_mag;
    drift_detector_t drift_roll, drift_pitch;
    
    median_filter_init(&filt_roll);
    median_filter_init(&filt_pitch);
    median_filter_init(&filt_mag);
    drift_detector_init(&drift_roll);
    drift_detector_init(&drift_pitch);

    /* 任务初始化动作：解除 MPU6050 的低功耗完全睡眠状态，使其进入高速转换模式 */
    mpu6050_sleep(false);

    /* 特殊时序处理：检查设备是否是从 Deep Sleep 模式被 MPU6050 硬件震动中断所唤醒的 */
    if (g_wake_reason == WAKE_REASON_MPU6050_INT) {
        ESP_LOGI(TAG, "系统检测到深睡唤醒源为 IMU 震动中断");

        /* 核心点：必须通过 I2C 读取 INT_STATUS 寄存器，方可清除芯片内部的电平锁存。
         * 如果苏醒后漏掉此动作，IMU 的 INT 引脚将持续保持锁存状态，后续将再也无法触发下一次唤醒中断 */
        uint8_t int_status;
        esp_err_t ret;
        ret = i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR,
                (uint8_t[]){MPU6050_REG_INT_STATUS}, 1,
                &int_status, 1, pdMS_TO_TICKS(50));
        if (ret == ESP_OK && (int_status & MPU6050_INT_MOTION)) {
            ESP_LOGI(TAG, "硬件加速度突发运动中断标志已被成功读取,状态=0x%02X", int_status);
        }
    }

    /* 数据同步时序控制变量 */
    TickType_t last_wake_time = xTaskGetTickCount();
    // 根据系统配置的采样率换算得到精确的 FreeRTOS 内核时钟节拍数
    const TickType_t sample_period = pdMS_TO_TICKS(1000 / MPU6050_SAMPLE_RATE);

    while (1) {
        sensor_data_t sensor_data;
        esp_err_t ret;

        /* Step 1: 调用驱动接口获取互斥锁保护下的物理单位传感器数据集 */
        ret = mpu6050_read_sensor_data(&sensor_data);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "硬件总线读取传感器数据发生严重阻碍: 错误码=%d", ret);
            vTaskDelay(pdMS_TO_TICKS(100)); // 出错后柔性延时等待总线恢复
            continue;
        }

        /* Step 2: 注入滑动中值滤波器，剔除噪声毛刺（此处修复了原代码的局部变量重名 Bug） */
        float filtered_roll  = median_filter_update(&filt_roll, sensor_data.roll);
        float filtered_pitch = median_filter_update(&filt_pitch, sensor_data.pitch);
        float filtered_mag   = median_filter_update(&filt_mag, sensor_data.accel_mag);

        // 将平滑滤波后的优质物理特征重写回数据包结构体中
        sensor_data.roll      = filtered_roll;
        sensor_data.pitch     = filtered_pitch;
        sensor_data.accel_mag = filtered_mag;

        /* Step 3: 进行零偏卡死和无响应监控评估 */
        if (drift_detector_check(&drift_roll, sensor_data.roll)) {
            ESP_LOGD(TAG, "潜在异常提示：发现横滚角 (Roll) 历史波形疑似失去活性或卡死");
        }
        if (drift_detector_check(&drift_pitch, sensor_data.pitch)) {
            ESP_LOGD(TAG, "潜在异常提示：发现俯仰角 (Pitch) 历史波形疑似失去活性或卡死");
        }

        /* Step 4: 严格的数学越界有效性防御拦截 */
        // 如果物理层计算产生浮点错乱导致出现非数 (NaN) 或无穷大 (Inf)，必须就地强行拦截并丢弃本帧
        if (isnan(sensor_data.roll) || isinf(sensor_data.roll) ||
            isnan(sensor_data.pitch) || isinf(sensor_data.pitch)) {
            ESP_LOGW(TAG, "拦截到严重的非法数学非数(NaN/Inf)，本帧数据作废");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* Step 5: 物理学几何量程合理性安全断言 */
        // 横滚角与俯仰角的理论极限边界为 ±180°，超越此边界属于绝对非法数据
        if (fabsf(sensor_data.roll) > 180.0f || fabsf(sensor_data.pitch) > 180.0f) {
            ESP_LOGW(TAG, "传感器姿态解算严重超越空间几何范畴线: 横滚=%.1f 俯仰=%.1f",
                     sensor_data.roll, sensor_data.pitch);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* Step 6: 投递至全局消息队列，移交给后级的暂态突变异常识别任务
         * 核心技术点：数据滑窗覆盖策略 (Drop Oldest)
         * 如果后级消费任务（如无线发生长时间阻塞）来不及处理，队列将被填满。
         * 为了保证报警响应的最高实时性，此处不允许挂起等待，而是采取主动抛弃队列中
         * 最旧（最早进入）的过期历史样本，随后将带有最新鲜时效性的实时帧强制压入队列尾部。 */
        if (xQueueSend(g_sensor_data_queue, &sensor_data, 0) != pdTRUE) {
            sensor_data_t dummy;
            xQueueReceive(g_sensor_data_queue, &dummy, 0); // 强制弹出并释放队列头部的最旧元素
            xQueueSend(g_sensor_data_queue, &sensor_data, 0); // 重新压入最新鲜的当前帧
            ESP_LOGW(TAG, "数据消息队列产生过载饱和：主动抛弃一帧最旧样本以确保实时防线畅通");
        }

        /* 触发全局事件组标志位：宣告新一轮高质传感器清洗数据流已就绪 */
        xEventGroupSetBits(g_event_group, EVENT_SENSOR_READY);

        /* 时序闭环：运用绝对时间步进挂起函数，完美规避由于业务代码本身执行耗时带来的采样率时间轴漂移 */
        vTaskDelayUntil(&last_wake_time, sample_period);
    }
}

/* ================================================================
 * FreeRTOS 任务级安全挂载与外部线程通信接口
 * ================================================================ */

/**
 * @brief 动态创建传感器高频处理核心线程
 */
BaseType_t sensor_acq_task_create(void)
{
    BaseType_t ret;
    TaskHandle_t handle;

    ret = xTaskCreate(
        sensor_acq_task,
        SENSOR_ACQ_TASK_NAME,
        SENSOR_ACQ_TASK_STACK,
        NULL,
        SENSOR_ACQ_TASK_PRIO,
        &handle
    );

    if (ret == pdPASS) {
        g_sensor_task_handle = handle; // 导出底层真实全局任务句柄，供多核/外部中断链路交叉调度
        ESP_LOGI(TAG, "传感器采集主任务成功挂载: 静态配置优先级=%d, 物理堆栈安全配额=%d 字节",
                 SENSOR_ACQ_TASK_PRIO, SENSOR_ACQ_TASK_STACK);
    } else {
        ESP_LOGE(TAG, "系统堆空间极度溃乏，传感器核心采集线程动态创建失败！");
    }

    return ret;
}

/**
 * @brief 供外部中断服务程序 (ISR) 或低功耗硬件链在苏醒时实施强行通知主核的无锁化通信原语
 */
void sensor_acq_task_notify_wakeup(void)
{
    if (g_sensor_task_handle != NULL) {
        xTaskNotifyGive(g_sensor_task_handle); // 快速泵入轻量级 FreeRTOS 任务通知信号
    }
}