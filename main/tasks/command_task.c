/**
 * @file    command_task.c
 * @brief   无线指令解析与下发控制任务实现
 *
 * 任务 3: command_task (优先级: 7 - 较高优先级)
 *
 * 主要职责:
 * 1. 从命令队列中接收网关下发的指令 (由 DMA+IDLE UART 驱动捕获并推入消息队列)
 * 2. 解析并校验下行指令包的数据合法性
 * 3. 实时执行安全防护阈值的在线重配置
 * 4. 动态调整传感器底层采样控制参数
 * 5. 同步刷新 RTC 慢速保持内存，确保 ULP 协处理器可以同步最新的策略线
 * 6. 构建应答帧并将命令处理结果闭环回传给基站网关
 *
 * 网关下行命令字集:
 * 0x10 - 配置横滚角报警阈值 (float, 4 字节)
 * 0x11 - 配置俯仰角报警阈值 (float, 4 字节)
 * 0x12 - 配置航向角报警阈值 (float, 4 字节)
 * 0x13 - 配置过载加速度报警阈值 (float, 4 字节)
 * 0x14 - 配置低功耗突发震动检测阈值 (uint8, 1 字节)
 * 0x20 - 配置常规运行数据采样率 (uint16, 2 字节)
 * 0x21 - 配置低功耗自唤醒扫描周期 (uint32, 4 字节)
 * 0x30 - 索取设备实时健康状态
 * 0x31 - 索取设备本地当前的全部阈值配置参数
 * 0x32 - 索取当前电池真实电压
 * 0xFF - 强行触发全局系统软件热重启
 */

#include "tasks/command_task.h"
#include "protocol/gateway_protocol.h"
#include "drivers/lora_uart.h"
#include "drivers/adc_battery.h"
#include "ulp/ulp_shared.h"
#include <string.h>
#include "drivers/mpu6050.h"
static const char *TAG = "command";

/* ================================================================
 * 命令帧物理拆包解析层
 * ================================================================ */

/**
 * @brief 解析由接收链路捕获到的网关下行原始指令数据帧
 *
 * 物理帧格式 (定向传输模式):
 * [目的地址高位][目的地址低位][目的信道][命令类型][参数长度][附加参数域...]
 *
 * 前 3 个字节 ([目的地址高低位] 与 [目的信道]) 属于无线定点传输的物理包头，
 * LoRa 模块在接收到并确认地址无误后，会自动在硬件层将其剥离丢弃。
 * 因此，MCU 串口驱动层最终截获到的实际净荷流格式为：[命令类型][参数长度][附加参数域...]
 */
static esp_err_t parse_gateway_frame(const uint8_t *raw_data, uint16_t len,
                                     gateway_cmd_t *cmd)
{
    // 防御性基础参数检查，最小有效帧长必须包含命令字和参数长度共 2 字节
    if (raw_data == NULL || len < 2) {
        return ESP_ERR_INVALID_ARG;
    }

    cmd->cmd_type = raw_data[0];  // 提取命令代号
    cmd->param_len = raw_data[1]; // 提取后续附加参数的标称长度

    // 边界过载防护：防止恶意或异常超长数据包导致内部 params 数组缓冲区溢出 (最大容纳 32 字节)
    if (cmd->param_len > 32) {
        ESP_LOGW(TAG, "检测到异常附加参数长度过长: %d, 已强行截断至 32 字节上限", cmd->param_len);
        cmd->param_len = 32;
    }

    // 验证实际物理接收到的帧长度是否满足标称载荷的最小包边界需求
    if (len >= (uint16_t)(cmd->param_len + 2)) {
        memcpy(cmd->params, &raw_data[2], cmd->param_len); // 内存安全拷贝物理参数域
    } else {
        // 如果实际字节数少于指示长度，说明无线链路上发生了严重的链路截断丢包
        ESP_LOGW(TAG, "数据帧格式残缺: 期望长度 %d, 实际接收字节数 %d",
                 cmd->param_len + 2, len);
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}

/**
 * @brief 数据防错过滤机制：对下发参数进行极其严格的数学区间与逻辑合法性校验
 */
static bool validate_command(const gateway_cmd_t *cmd)
{
    switch (cmd->cmd_type) {
        case CMD_SET_ROLL_THRESHOLD:
        case CMD_SET_PITCH_THRESHOLD:
        case CMD_SET_YAW_THRESHOLD:
        case CMD_SET_ACCEL_THRESHOLD:
            // 四大浮点型安全阈值配置，其参数域必须集齐至少 4 个字节
            if (cmd->param_len < 4) return false;
            
            /* 避免非对齐指针引发的硬件级硬异常
             * 串口缓冲区的首地址可能不满足 4 字节对齐，如果直接进行强转强制指针访问(如 *(float*) )，
             * 会在某些架构上引发 Unaligned Memory Access（非对齐访问）崩溃。
             * 此处通过标准的 memcpy 机制将原始字节流原样无损搬运至局部 float 变量，确保跨平台安全。 */
            float val;
            memcpy(&val, cmd->params, 4);
            
            /* 数学异常安全防线
             * 无线物理链路可能由于强干扰导致比特翻转，由于 float 的编码特性，某些随机破坏的数据可能拼凑成
             * 非数 (NaN) 或无穷大 (Infinity)。直接让算法计算这类脏数据会导致卡尔曼滤波或姿态解算彻底瘫痪。
             * 此处使用 standard math 库中的 isnan 与 isinf 实施边界斩断防护。 */
            if (isnan(val) || isinf(val)) return false;
            
            if (cmd->cmd_type <= CMD_SET_YAW_THRESHOLD) {
                /* 角度安全阈值范围约束：必须限制在 0.0 到 180.0 度之间 */
                if (val < 0.0f || val > 180.0f) return false;
            } else {
                /* 加速度冲击安全阈值范围约束：必须限制在 0.0 到 16.0g 之间 (受限于 IMU 硬件量程) */
                if (val < 0.0f || val > 16.0f) return false;
            }
            break;

        case CMD_SET_MOTION_THRESHOLD:
            // 硬件震动唤醒阈值：占用 1 字节
            if (cmd->param_len < 1) return false;
            // 明确的逻辑安全断言防御，防止隐式转换溢出
            if (cmd->params[0] > 255) return false;
            break;

        case CMD_SET_SAMPLE_RATE:
            // 主控工作采样刷新率配置：占用 2 字节 (uint16_t)
            if (cmd->param_len < 2) return false;
            uint16_t rate;
            memcpy(&rate, cmd->params, 2);
            // 硬件物理极限约束：限定数据流采样刷新率必须在 1Hz 到 1000Hz (1kHz DLPF 极限) 之间
            if (rate < 1 || rate > 1000) return false;
            break;

        case CMD_SET_ULP_PERIOD:
            // 低功耗协处理器唤醒定时扫描周期：占用 4 字节 (uint32_t)
            if (cmd->param_len < 4) return false;
            uint32_t period;
            memcpy(&period, cmd->params, 4);
            // 物理时序约束：限定 ULP 周期在 10ms (10000us) 到 10s (10000000us) 之间，防止过频看门狗超时或周期过大失去监控意义
            if (period < 10000 || period > 10000000) return false;
            break;

        case CMD_GET_STATUS:
        case CMD_GET_THRESHOLDS:
        case CMD_GET_BATTERY:
        case CMD_SOFT_RESET:
            
            break;

        default:
            ESP_LOGW(TAG, "无法识别的异常命令字类型: 0x%02X", cmd->cmd_type);
            return false;
    }

    return true; // 所有防线通过，判定该控制指令安全合法
}

/* ================================================================
 * FreeRTOS 核心任务逻辑体层
 * ================================================================ */

/**
 * @brief FreeRTOS 中央命令处理任务主体
 */
static void command_task(void *arg)
{
    ESP_LOGI(TAG, "无线命令解析分发核心任务已成功启动就绪");

    gateway_cmd_t cmd;

    while (1) {
        /* 核心通信机制：任务流常态阻塞挂起，挂载等待由 UART 中断服务通过 DMA+IDLE 上报推入的最新命令包。
         * 超时设为 500ms。在无外部无线控制信号的背景下，自动让出 CPU 调度权，极大地配合整机低功耗运作。 */
        if (xQueueReceive(g_command_queue, &cmd, pdMS_TO_TICKS(500)) != pdTRUE) {
            continue; // 队列为空，继续下一轮挂起轮询
        }

        /* 触发全局多任务事件组标记位：通告全机当前已捕获到下行无线联机配置命令 */
        xEventGroupSetBits(g_event_group, EVENT_CMD_RECEIVED);

        ESP_LOGI(TAG, "成功捕获下行控制原语: 指令字=0x%02X, 参数区标称长度=%d",
                 cmd.cmd_type, cmd.param_len);

        /* 过滤拦截器：检查参数域边界与数学逻辑合法性 */
        if (!validate_command(&cmd)) {
            // 参数欺骗或错乱拦截，保障核心底层数据存储安全
            ESP_LOGW(TAG, "指令安全性校验未通过，拦截非法命令字: 0x%02X", cmd.cmd_type);
            /* 闭环异常控制：向基站异步回传带有异常错误码 (0x01) 的 NAK 拒绝否定应答帧 */
            gateway_protocol_send_cmd_response(&cmd, 0x01);  /* 0x01 代表非法无效命令/无效参数 */
            continue; // 抛弃本帧，直接进入下一轮等待
        }

        /* 分发路由：将清洗校验后的控制数据交付至协议中央执行层落实执行并固化 */
        esp_err_t ret = gateway_protocol_process_cmd(&cmd);

        /* 高级流程调度：在此处专门接管并处理需要“多字节大数据反馈应答”的特殊查询类指令 */
        switch (cmd.cmd_type) {
            case CMD_GET_THRESHOLDS: {
                /* 批量获取请求：打包并返回当前本地全套四大浮点型防线阈值的当前物理快照 */
                uint8_t resp[16];
                float val;
                
                // 将 float 比特图逐一无损组装到 16 字节连续缓冲区中
                val = gateway_get_roll_threshold();
                memcpy(&resp[0], &val, 4);
                val = gateway_get_pitch_threshold();
                memcpy(&resp[4], &val, 4);
                val = gateway_get_yaw_threshold();
                memcpy(&resp[8], &val, 4);
                val = gateway_get_accel_threshold();
                memcpy(&resp[12], &val, 4);

                // 临时构造一个伪命令包，用以借用底层协议栈通用的定点应答发射通道
                gateway_cmd_t resp_cmd = {
                    .cmd_type = CMD_GET_THRESHOLDS,
                    .param_len = 16,
                };
                memcpy(resp_cmd.params, resp, 16);
                
                // 回传闭环数据包，附带状态码 0x00 表示执行成功
                gateway_protocol_send_cmd_response(&resp_cmd, 0x00);
                break;
            }

            case CMD_GET_BATTERY: {
                /* 精确电压读取：触发电量 ADC 单次获取，并将毫伏值以 4 字节整型形式返回基站 */
                uint32_t battery_mv;
                if (battery_read_voltage(&battery_mv) == ESP_OK) {
                    gateway_cmd_t resp_cmd = {
                        .cmd_type = CMD_GET_BATTERY,
                        .param_len = 4,
                    };
                    memcpy(resp_cmd.params, &battery_mv, 4);
                    gateway_protocol_send_cmd_response(&resp_cmd, 0x00);
                }
                break;
            }

            default:
                /* 通用应答控制：针对常规修改配置类指令，统一回传单字节基础 ACK。
                 * 状态：0x00 代表动作顺利执行完毕，0x02 代表底层驱动或物理 Flash 产生存储性阻碍失败 */
                gateway_protocol_send_cmd_response(&cmd,
                    (ret == ESP_OK) ? 0x00 : 0x02);
                break;
        }

        /* 核心技术点三：跨核多策略热更新同步
         * 当网关成功对四大物理防线阈值进行了动态重配置后，除了主核生效外，
         * 必须在主核下一次切入深睡前，在 RTC 共享内存中额外追加触发更新通知标志位。
         * 这样能够强制让 ULP LP-Core 协处理器在其下一个自定时苏醒周期内重新刷写监测防线，
         * 确保了跨核、跨掉电周期的数据控制链完美同步。 */
        if (cmd.cmd_type >= CMD_SET_ROLL_THRESHOLD &&
            cmd.cmd_type <= CMD_SET_ACCEL_THRESHOLD) {
            /* 物理阈值的真实转换搬运已在统一接口 gateway_protocol_process_cmd 中彻底闭环，
             * 此处通过按位或，专门对 RTC 共享区中的变更事件通知字进行精准强制打标 */
            uint32_t *rtc_mem = (uint32_t *)RTC_MEM_BASE;
            rtc_mem[RTC_OFFSET_ULP_FLAGS / 4] |= ULP_FLAG_THRESHOLD_UPDATED;

            ESP_LOGI(TAG, "跨核同步控制线已建立：已向 ULP LP-Core 发送阈值信号");
        }

        ESP_LOGI(TAG, "无线原语指令 0x%02X 全链路闭环处理完毕: 本地运行状态=%s",
                 cmd.cmd_type, (ret == ESP_OK) ? "成功(OK)" : "失败(FAILED)");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ================================================================
 * 任务安全动态挂载层
 * ================================================================ */

/**
 * @brief 安全导出接口：动态创建并挂载 FreeRTOS 任务级命令处理引擎
 */
BaseType_t command_task_create(void)
{
    // 注入全局统一配置的优先级与专属任务安全堆栈尺寸
    BaseType_t ret = xTaskCreate(
        command_task,
        COMMAND_TASK_NAME,
        COMMAND_TASK_STACK,
        NULL,
        COMMAND_TASK_PRIO,
        NULL
    );

    if (ret == pdPASS) {
        ESP_LOGI(TAG, "中央命令任务挂载成功: 当前静态分配优先级=%d, 专属安全堆栈深度=%d 字节",
                 COMMAND_TASK_PRIO, COMMAND_TASK_STACK);
    } else {
        // 关键防护报警：若内存碎片严重导致堆空，防止产生悬空崩溃
        ESP_LOGE(TAG, "严重系统错误：系统全机堆内存不足，无法成功动态创建控制主任务！");
    }

    return ret;
}
