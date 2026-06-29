/**
 * @file    adc_battery.c
 * @brief   电池电压 ADC 采样驱动程序实现
 *
 * 基于 ESP-IDF 的 ADC 单次采样模式 (Oneshot) 实现电池电压检测，
 * 内置软件过采样滤波算法，并通过分压比换算及锂电池放电曲线估算剩余电量。
 */

#include "drivers/adc_battery.h"

// 用于 ESP_LOG 日志打印的标签
static const char *TAG = "adc_battery";

// ADC 单次采样单元的全局句柄
static adc_oneshot_unit_handle_t g_adc_handle = NULL;

/**
 * @brief 初始化电池电压检测所需的 ADC 外设
 */
esp_err_t battery_adc_init(void)
{
    /* 1. 配置 ADC 单次采样单元 (Oneshot Unit) */
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = BATTERY_ADC_UNIT,             // ADC 单元选择 (例如 ADC_UNIT_1)
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,    // 使用默认的数字时钟源
        .ulp_mode = ADC_ULP_MODE_DISABLE,       // 禁用超低功耗 (ULP) 模式
    };
    // 创建并初始化 ADC 单元
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &g_adc_handle));

    /* 2. 配置 ADC 采样通道参数 */
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = BATTERY_ADC_ATTEN,             // 配置衰减度 (决定了能测量的最大电压范围)
        .bitwidth = ADC_BITWIDTH_12,            // 设为 12 位分辨率，原始数值范围：0 ~ 4095
    };
    // 将通道配置应用到指定的 ADC 单元和通道上
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc_handle,
                                               BATTERY_ADC_CHANNEL,
                                               &chan_cfg));

    /* 3. 配置 ADC 对应的硬件 GPIO 引脚 */
    /* 注意：在 ESP32-C6 芯片上，一旦配置了 ADC 通信，底层驱动会自动将
     * 对应的引脚切换为模拟输入模式，因此无需手动调用 gpio_config() */

    ESP_LOGI(TAG, "电池 ADC 初始化成功: 单元=%d, 通道=%d, 衰减代码=%d",
             BATTERY_ADC_UNIT, BATTERY_ADC_CHANNEL, BATTERY_ADC_ATTEN);
    return ESP_OK;
}

/**
 * @brief 读取当前电池的实际电压值（单位：mV）
 * @param voltage_mv 接收计算后电压结果的指针
 */
esp_err_t battery_read_voltage(uint32_t *voltage_mv)
{
    int adc_raw = 0;
    int sum = 0;
    const int samples = 8;          // 过采样样本数，设为 2 的幂次方便于硬件或编译器优化

    /* 软件过采样滤波 (Oversampling)
     * 连续读取 8 次原始值并取平均，以滤除高频随机噪声和电源纹波，提高读数稳定性 */
    for (int i = 0; i < samples; i++) {
        int raw;
        esp_err_t ret = adc_oneshot_read(g_adc_handle, BATTERY_ADC_CHANNEL, &raw);
        if (ret != ESP_OK) return ret;
        sum += raw;
        vTaskDelay(pdMS_TO_TICKS(1)); // 每次采样稍微延时，避开密集的脉冲噪声
    }

    // 计算均值
    adc_raw = sum / samples;

    /* ADC 原始值转换电压 (mV)
     * - ESP32-C6 ADC：12位分辨率下最大值为 4095。
     * - 衰减配置为 12dB，ADC 满量程可测电压理论约为 3900mV 。
     * - 数据映射关系：ADC 读数 0 -> 0mV，ADC 读数 4095 -> ~3900mV。
     *
     * 分压电路补偿
     * - 电池满电通常为 4.2V，已超过 ADC 最大测范围，所以硬件上一般采用电阻分压。
     * - 分压电路：R1=100k, R2=100k，则送入 ADC 的电压是电池电压的一半。
     * - 分压比系数 (voltage_divider_ratio) = (R1 + R2) / R2 = 2.0 */
    const float voltage_divider_ratio = 2.0f;
    
    // 将 ADC 原始值换算成引脚上的实际模拟电压值 (单位：mV)
    uint32_t adc_mv = (uint32_t)((float)adc_raw * 3900.0f / 4095.0f);
    
    // 通过分压比系数还原出电池端的真实电压
    *voltage_mv = (uint32_t)((float)adc_mv * voltage_divider_ratio);

    return ESP_OK;
}

/**
 * @brief 根据电池电压粗略估算剩余电量百分比
 * @param voltage_mv 输入的电池真实电压（单位：mV）
 * @return uint8_t 返回电量百分比 (0 ~ 100)
 */
uint8_t battery_get_percentage(uint32_t voltage_mv)
{
    /* 核心点四：基于锂电池/锂离子电池放电曲线的阶段性划分
     * 标准单节锂电池在带载工作时：
     * - 电压 >= 4200mV 视为 100% 满电
     * - 电压 <= 3300mV 视为 0% 没电（达到安全放电截止电压，保护电池不被过放） */
    if (voltage_mv >= 4200) return 100;
    if (voltage_mv <= 3300) return 0;

    /* 使用一阶线性插值进行近似估算
     * 电压区间宽度：4200mV - 3300mV = 900mV
     * 公式：百分比 = (当前电压 - 截止电压) / 区间总宽度 * 100 */
    return (uint8_t)((voltage_mv - 3300) * 100 / 900);
}