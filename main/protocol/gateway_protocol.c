/**
 * @file    gateway_protocol.c
 * @brief   基站网关通信协议定制层实现
 *
 * 负责无线通信数据包的组装序列化、发送流水线控制以及下行控制命令的解析分发。
 * 本层直接调用底层的 LoRa UART 驱动接口执行数据的物理发射。
 */

#include "protocol/gateway_protocol.h"
#include "drivers/lora_uart.h"
#include "drivers/adc_battery.h"
#include "ulp/ulp_shared.h"
#include <string.h>

static const char *TAG = "gw_proto";

/* ================================================================
 * 本地状态量 (单兵内部全局变量)
 * ================================================================ */
static uint8_t g_seq_num = 0; // 无线数据包滚动流水号（0~255 循环，用于网关端防重漏判）

/* 本地暂存的安全防线阈值量 (可通过 NVS 固化或无线远程动态修改) */
static float g_roll_threshold  = DEFAULT_ROLL_THRESHOLD;
static float g_pitch_threshold = DEFAULT_PITCH_THRESHOLD;
static float g_yaw_threshold   = DEFAULT_YAW_THRESHOLD;
static float g_accel_threshold = DEFAULT_ACCEL_THRESHOLD;

/* 运行背景统计计数器 */
static uint32_t g_sample_count = 0; // 自本次开机以来常规数据包的总发送计数
static uint32_t g_alarm_count  = 0; // 自本次开机以来突发报警包的总发送计数

/**
 * @brief 网关协议栈初始化，并同步打通与 ULP 协处理器的数据共享通道
 */
void gateway_protocol_init(void)
{
    /* 获取 RTC 慢速内存的绝对基地址
     * 主核在深睡前必须将最新阈值写入此区域，以便 ULP 协处理器在主核休眠时执行常态监控 */
    uint32_t *rtc_mem = (uint32_t *)RTC_MEM_BASE;
    
    /* Float 浮点比特流无损强转
     * 由于 C 语言低功耗硬件无法原生友好支持 float 运算，
     * 此处使用 memcpy 将标准 IEEE 754 格式的 4 字节浮点数原样拷贝为 32 位无符号整型比特图 */
    uint32_t roll_bits, pitch_bits, yaw_bits, accel_bits;
    memcpy(&roll_bits, &g_roll_threshold, 4);
    memcpy(&pitch_bits, &g_pitch_threshold, 4);
    memcpy(&yaw_bits, &g_yaw_threshold, 4);
    memcpy(&accel_bits, &g_accel_threshold, 4);

    /* 4 字节对齐偏置索引
     * 因为 rtc_mem 是 uint32_t* 指针类型，其自增 1 代表移动 4 个字节，
     * 所以底层的字节级地址偏置（Byte Offset）必须强制除以 4，才能转换为正确的数组下标 */
    rtc_mem[RTC_OFFSET_ROLL_THRESHOLD / 4]  = roll_bits;
    rtc_mem[RTC_OFFSET_PITCH_THRESHOLD / 4] = pitch_bits;
    rtc_mem[RTC_OFFSET_YAW_THRESHOLD / 4]   = yaw_bits;
    rtc_mem[RTC_OFFSET_ACCEL_THRESHOLD / 4] = accel_bits;
    rtc_mem[RTC_OFFSET_MOTION_THRESHOLD / 4] = MPU6050_MOT_THRESHOLD;

    /* 初始化共享内存中的 ULP 内部控制标志位与状态数据 */
    rtc_mem[RTC_OFFSET_ULP_FLAGS / 4]  = ULP_FLAG_FIRST_BOOT; // 置位初次冷启动标记
    rtc_mem[RTC_OFFSET_WAKE_COUNT / 4] = 0;                  // 清空 ULP 独立唤醒计数器
    rtc_mem[RTC_OFFSET_SENSOR_ID / 4]  = DEVICE_TYPE;         // 固化设备类型识别码
    rtc_mem[RTC_OFFSET_ULP_PERIOD / 4] = ULP_WAKEUP_PERIOD_US; // 注入 ULP 自定时自唤醒的微秒周期

    ESP_LOGI(TAG, "控制阈值已同步固化至 RTC 慢速内存中");
}

/**
 * @brief 从非易失性存储器 (NVS) 中动态加载历史留存的个性化阈值参数
 */
void gateway_protocol_load_thresholds(void)
{
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open("thresholds", NVS_READONLY, &nvs);

    if (ret == ESP_OK) {
        float val;

        // 逐一安全检索二进制大对象 (Blob)
        if (nvs_get_blob(nvs, "roll_th", &val, NULL) == ESP_OK)
            g_roll_threshold = val;
        if (nvs_get_blob(nvs, "pitch_th", &val, NULL) == ESP_OK)
            g_pitch_threshold = val;
        if (nvs_get_blob(nvs, "yaw_th", &val, NULL) == ESP_OK)
            g_yaw_threshold = val;
        if (nvs_get_blob(nvs, "accel_th", &val, NULL) == ESP_OK)
            g_accel_threshold = val;

        nvs_close(nvs);
        ESP_LOGI(TAG, "成功从NVS中加载用户阈值");
    } else {
        ESP_LOGI(TAG, "未发现历史 NVS 配置，系统自动启用默认安全阈值");
    }
}

/**
 * @brief 将当前的最新阈值同时持久化写入 NVS 闪存以及 RTC 慢速内存
 */
esp_err_t gateway_protocol_save_thresholds(void)
{
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open("thresholds", NVS_READWRITE, &nvs);
    if (ret != ESP_OK) return ret;

    // 二进制加密序列化写入 Flash
    nvs_set_blob(nvs, "roll_th", &g_roll_threshold, sizeof(float));
    nvs_set_blob(nvs, "pitch_th", &g_pitch_threshold, sizeof(float));
    nvs_set_blob(nvs, "yaw_th", &g_yaw_threshold, sizeof(float));
    nvs_set_blob(nvs, "accel_th", &g_accel_threshold, sizeof(float));

    nvs_commit(nvs); // 强制执行扇区擦写提交
    nvs_close(nvs);

    /* 同步更新 RTC 共享内存，确保 ULP 协处理器策略同步生效 */
    uint32_t *rtc_mem = (uint32_t *)RTC_MEM_BASE;
    memcpy(&rtc_mem[RTC_OFFSET_ROLL_THRESHOLD / 4], &g_roll_threshold, 4);
    memcpy(&rtc_mem[RTC_OFFSET_PITCH_THRESHOLD / 4], &g_pitch_threshold, 4);
    memcpy(&rtc_mem[RTC_OFFSET_YAW_THRESHOLD / 4], &g_yaw_threshold, 4);
    memcpy(&rtc_mem[RTC_OFFSET_ACCEL_THRESHOLD / 4], &g_accel_threshold, 4);

    /* 注入重要通知状态：向 ULP 核发送变更信号，使其在下一次自苏醒周期重新加载策略 */
    rtc_mem[RTC_OFFSET_ULP_FLAGS / 4] |= ULP_FLAG_THRESHOLD_UPDATED;

    ESP_LOGI(TAG, "最新安全防线阈值已成功双向同步固化至 NVS 与 RTC");
    return ESP_OK;
}

/* ================================================================
 * 数据序列化与发送流水线
 * ================================================================ */

/**
 * @brief 序列化打包并无线向上发送常态下的六轴解算健康数据帧
 */
esp_err_t gateway_protocol_send_data(const sensor_data_t *data)
{
    uint8_t packet[128];
    uint16_t offset = 0;

    /* 1. 构建统一的无线协议帧头 (Header) */
    proto_header_t header = {
        .sync = PROTOCOL_SYNC_WORD,         // 协议专用的特定同步字 （0xA5A5）
        .frame_type = FRAME_TYPE_DATA,      // 标记此帧为：常规数据上报帧
        .src_addr_h = DEVICE_ADDR_H,        // 写入本机节点的高位无线物理地址
        .src_addr_l = DEVICE_ADDR_L,        // 写入本机节点的低位无线物理地址
        .dst_addr_h = GATEWAY_ADDR_H,       // 指定接收网关基站的高位目标地址
        .dst_addr_l = GATEWAY_ADDR_L,       // 指定接收网关基站的低位目标地址
        .seq_num = g_seq_num++,             // 嵌入流水帧号并自动递增
        .payload_len = sizeof(proto_data_payload_t), // 动态指定实际业务数据载荷的精确长度
    };

    // 内存拷贝序列化至大缓冲区首部
    memcpy(&packet[offset], &header, sizeof(header));
    offset += sizeof(header);

    /* 2. 嵌套构建核心业务载荷 (Payload) */
    proto_data_payload_t payload = {
        .roll = data->roll,
        .pitch = data->pitch,
        .yaw = data->yaw,
        .accel_mag = data->accel_mag,
        .gyro_mag = data->gyro_mag,
        .temp_c = data->temperature_c,
        .timestamp = data->timestamp_ms,    // 嵌入主控时间戳，方便基站追溯时序关系
    };

    memcpy(&packet[offset], &payload, sizeof(payload));
    offset += sizeof(payload);

    g_sample_count++; // 递增常规采样包累计器

    ESP_LOGD(TAG, "正在发射常规物理帧 (序号=%d): 横滚=%.1f 俯仰=%.1f 航向=%.1f",
             header.seq_num, payload.roll, payload.pitch, payload.yaw);

    // 拼装完整的 16 位目标地址，交付给具有 AUX 锁存硬件保护的定点 LoRa 发送接口
    return lora_send_directed(
        ((uint16_t)GATEWAY_ADDR_H << 8) | GATEWAY_ADDR_L,
        LORA_CHANNEL,
        packet, offset);
}

/**
 * @brief 捕捉到危险状态时，立即组装异常最高优先级的突发紧急报警帧
 */
esp_err_t gateway_protocol_send_alarm(const alarm_data_t *alarm)
{
    uint8_t packet[128];
    uint16_t offset = 0;
    uint32_t battery_mv = 0;

    /* 关键设计点：在突发严重安全危机报警时，强制同步读取当前的供电电量，
     * 使得基站网关在解析到危机波形的同时，能瞬间掌握该危机节点的现场生存电量状态 */
    battery_read_voltage(&battery_mv);

    /* 1. 组装高优先级报警帧头 */
    proto_header_t header = {
        .sync = PROTOCOL_SYNC_WORD,
        .frame_type = FRAME_TYPE_ALARM,     // 标记此帧为：紧急重度报警突发帧
        .src_addr_h = DEVICE_ADDR_H,
        .src_addr_l = DEVICE_ADDR_L,
        .dst_addr_h = GATEWAY_ADDR_H,
        .dst_addr_l = GATEWAY_ADDR_L,
        .seq_num = g_seq_num++,
        .payload_len = sizeof(proto_alarm_payload_t),
    };

    memcpy(&packet[offset], &header, sizeof(header));
    offset += sizeof(header);

    /* 2. 组装带有详细现场痕迹的报警有效载荷 */
    proto_alarm_payload_t payload = {
        .alarm_type = alarm->alarm_type,       // 报警事件分类代码 (如横滚超限或冲击过载)
        .current_value = alarm->value,         // 引发异常的现场瞬间真实值
        .threshold_value = alarm->threshold,   // 此时设备本地正处于生效期的安全红线值
        .delta = alarm->delta,                 // 跨越红线的溢出破坏额
        .timestamp = alarm->timestamp_ms,      // 异常爆发的时间戳
        .battery_mv = battery_mv,              // 现场打包进去的供电毫伏数
    };

    memcpy(&packet[offset], &payload, sizeof(payload));
    offset += sizeof(payload);

    g_alarm_count++; // 突发报警包累计器加一

    ESP_LOGW(TAG, " (流水号=%d): 类型代码=%d, 现场突发值=%.2f, 越界差额=%.2f",
             header.seq_num, alarm->alarm_type,
             alarm->value, alarm->delta);

    return lora_send_directed(
        ((uint16_t)GATEWAY_ADDR_H << 8) | GATEWAY_ADDR_L,
        LORA_CHANNEL,
        packet, offset);
}

/**
 * @brief 定时或奉命上报当前整个硬件节点全生命周期的健康体检状态帧
 */
esp_err_t gateway_protocol_send_status(void)
{
    uint8_t packet[128];
    uint16_t offset = 0;
    uint32_t battery_mv = 0;
    uint32_t *rtc_mem = (uint32_t *)RTC_MEM_BASE;

    battery_read_voltage(&battery_mv);

    /* 1. 构建状态帧头 */
    proto_header_t header = {
        .sync = PROTOCOL_SYNC_WORD,
        .frame_type = FRAME_TYPE_STATUS,    // 标记此帧为：设备运行状态快照统计帧
        .src_addr_h = DEVICE_ADDR_H,
        .src_addr_l = DEVICE_ADDR_L,
        .dst_addr_h = GATEWAY_ADDR_H,
        .dst_addr_l = GATEWAY_ADDR_L,
        .seq_num = g_seq_num++,
        .payload_len = sizeof(proto_status_payload_t),
    };

    memcpy(&packet[offset], &header, sizeof(header));
    offset += sizeof(header);

    ESP_LOGI(TAG, "传感器电压=%dmV", battery_mv);

    return lora_send_directed(
        ((uint16_t)GATEWAY_ADDR_H << 8) | GATEWAY_ADDR_L,
        LORA_CHANNEL,
        packet, offset);
}

/**
 * @brief 对网关下发的单条配置或查询指令进行应答回传（指令应答帧闭环控制）
 * @param cmd 触发本次应答的原始下行命令命令句柄
 * @param result 命令在本地的具体执行结果状态码 (如成功、非法参数等)
 */
esp_err_t gateway_protocol_send_cmd_response(gateway_cmd_t *cmd, uint8_t result)
{
    uint8_t packet[128];
    uint16_t offset = 0;

    proto_header_t header = {
        .sync = PROTOCOL_SYNC_WORD,
        .frame_type = FRAME_TYPE_CMD_RESP,  // 标记此帧为：针对网关下行控制的闭环应答帧
        .src_addr_h = DEVICE_ADDR_H,
        .src_addr_l = DEVICE_ADDR_L,
        .dst_addr_h = GATEWAY_ADDR_H,
        .dst_addr_l = GATEWAY_ADDR_L,
        .seq_num = g_seq_num++,
        .payload_len = cmd->param_len + 2,   // 载荷总长度 = 命令代号(1B) + 结果状态(1B) + 原始随带参数长度
    };

    memcpy(&packet[offset], &header, sizeof(header));
    offset += sizeof(header);

    // 填充协议应答特定载荷域
    packet[offset++] = cmd->cmd_type;  // 反馈响应是哪一条指令
    packet[offset++] = result;        // 反馈本地最终执行状态代码 (如 0x00=SUCCESS)
    memcpy(&packet[offset], cmd->params, cmd->param_len); // 原样打包附带参数作为对照检查
    offset += cmd->param_len;

    return lora_send_directed(
        ((uint16_t)GATEWAY_ADDR_H << 8) | GATEWAY_ADDR_L,
        LORA_CHANNEL,
        packet, offset);
}

esp_err_t gateway_protocol_enqueue_cmd(const uint8_t *raw_data, uint16_t len)
{
    if (raw_data == NULL || len < 2) {
        return ESP_ERR_INVALID_ARG;
    }

    gateway_cmd_t cmd;
    cmd.cmd_type  = raw_data[0];
    cmd.param_len = (len - 1 > 32) ? 32 : (len - 1);
    memset(cmd.params, 0, sizeof(cmd.params));
    if (cmd.param_len > 0) {
        memcpy(cmd.params, &raw_data[1], cmd.param_len);
    }

    if (xQueueSend(g_command_queue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGW(TAG, "命令队列已满，丢弃网关命令 0x%02X", cmd.cmd_type);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ================================================================
 * 高级业务层数据获取与设定的快捷辅助接口 (Getters & Setters)
 * ================================================================ */

uint8_t gateway_protocol_get_seq_num(void)
{
    return g_seq_num;
}

float gateway_get_roll_threshold(void){ 
    return g_roll_threshold; 
}
float gateway_get_pitch_threshold(void){ 
    return g_pitch_threshold; 
}
float gateway_get_yaw_threshold(void){ 
    return g_yaw_threshold; 
}
float gateway_get_accel_threshold(void){ 
    return g_accel_threshold; 
}

void gateway_set_roll_threshold(float val){ 
    g_roll_threshold = val; 
}
void gateway_set_pitch_threshold(float val){ 
    g_pitch_threshold = val; 
}
void gateway_set_yaw_threshold(float val){ 
    g_yaw_threshold = val; 
}
void gateway_set_accel_threshold(float val){ 
    g_accel_threshold = val; 
}
