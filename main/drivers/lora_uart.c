/**
 * @file    lora_uart.c
 * @brief   LoRa 模块 UART 驱动程序（采用 DMA + IDLE 空闲中断接收设计）
 *
 * 核心特性:
 * - 采用 DMA 进行 TX 数据传输（极低 CPU 占有率）
 * - 采用 UART IDLE（空闲）总线中断，用来实现变长数据包的自适应截断与接收
 * - 使用 FreeRTOS 队列将接收到的数据及时通知给处理任务
 * - 针对正点原子 ATK-LORA-02 模块进行了 AT 指令集深度优化
 * - 设计丢包率目标：< 0.5%
 */

#include "drivers/lora_uart.h"
#include "esp_intr_alloc.h"
#include "soc/uart_struct.h"
#include "hal/uart_ll.h"
#include <string.h>

static const char *TAG = "lora_uart";

/* ================================================================
 * 模块内部全局状态
 * ================================================================ */
static uint8_t g_lora_mode = LORA_MODE_CONFIG;       // 记录 LoRa 模块当前所处的运行模式
static QueueHandle_t g_lora_rx_queue = NULL;          // UART 底层驱动事件队列句柄
static SemaphoreHandle_t g_uart_tx_done = NULL;       // 串口发送完成的二值信号量
static void (*g_aux_callback)(bool rising) = NULL;    // AUX 引脚电平跳变的回调函数指针

/* DMA 缓冲区配置
 * 注意：使用 DRAM_ATTR 宏可以确保这些缓冲区被分配到片内内部 RAM (Internal RAM) 中，
 * 这是 ESP32 系列芯片中使用 DMA 传输的硬性要求（不能分配到外部 PSRAM）。*/
static DRAM_ATTR uint8_t g_rx_dma_buf[LORA_UART_RX_BUF_SIZE];
static DRAM_ATTR uint8_t g_tx_dma_buf[LORA_UART_TX_BUF_SIZE];

/* ================================================================
 * GPIO 硬件引脚控制宏定义
 * ================================================================ */
#define LORA_MD0_HIGH()  gpio_set_level(LORA_MD0_GPIO, 1) // MD0 拉高：进入 AT 配置模式 / 休眠模式
#define LORA_MD0_LOW()   gpio_set_level(LORA_MD0_GPIO, 0) // MD0 拉低：进入正常无线通信模式
#define LORA_AUX_READ()  gpio_get_level(LORA_AUX_GPIO)    // 读取 AUX 引脚：0 代表模块空闲/就绪状态，1 代表模块忙（正在发送/接收/配置）

/* ================================================================
 * UART DMA + IDLE 事件中断处理回调函数
 * ================================================================ */

/**
 * @brief UART 事件和数据接收的核心中断服务处理程序
 *
 * 当串口总线在接收完一帧数据后，保持空闲时间超过 1 个字符周期以上时，
 * 硬件会自动触发 IDLE（空闲）中断。
 */
// 底层事件回调（运行在受限环境或 ISR）
static bool IRAM_ATTR lora_uart_event_handler(uart_port_t uart_num, void *user_ctx)
{
    uart_event_t event;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // 收到硬件事件后，不要在原地读数据！
    while (xQueueReceiveFromISR(*(QueueHandle_t *)user_ctx, &event, NULL)) {
        if (event.type == UART_DATA) {
            // 仅仅向命令解析任务发送一个轻量级的信号量或事件（这里假设发送给处理任务），让硬件中断瞬间解放
            xQueueSendFromISR(g_command_event_queue, &event, &xHigherPriorityTaskWoken);
        }
    }
    return xHigherPriorityTaskWoken;
}

/**
 * @brief 初始化 LoRa 专用的 UART 硬件外设、DMA 通道及控制 GPIO
 */
esp_err_t lora_uart_init(void)
{
    esp_err_t ret;

    /* -------- GPIO 基础引脚配置 -------- */
    // 配置 LoRa 模块的控制引脚 (MD0 和 AUX)
    gpio_config_t ctrl_conf = {
        .pin_bit_mask = (1ULL << LORA_MD0_GPIO) | (1ULL << LORA_AUX_GPIO),
        .mode = GPIO_MODE_INPUT_OUTPUT,          // 初步设为输入输出混合模式
        .pull_up_en = GPIO_PULLUP_ENABLE,        // 启用内部上拉电阻确保默认电平稳定
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,          // 暂不开启 GPIO 硬件中断
    };
    gpio_config(&ctrl_conf);

    // 强制将模块状态引脚 AUX 重新修正为纯输入模式
    gpio_set_direction(LORA_AUX_GPIO, GPIO_MODE_INPUT);
    // 默认将 MD0 拉低，使模块开机后直接默认置于普通透传无线通信模式
    LORA_MD0_LOW();

    /* -------- UART 串口波特率及协议配置 -------- */
    uart_config_t uart_conf = {
        .baud_rate = LORA_UART_BAUD,             // 通信波特率（如 115200）
        .data_bits = UART_DATA_8_BITS,           // 8 位数据位
        .parity = UART_PARITY_DISABLE,           // 无奇偶校验
        .stop_bits = UART_STOP_BITS_1,           // 1 位停止位
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,   // 禁用硬件流控
        .source_clk = UART_SCLK_DEFAULT,         // 使用默认系统时钟源
    };

    // 安装 UART 驱动程序，系统内部会自动分配关联的 DMA 通道
    ret = uart_driver_install(LORA_UART_NUM,
                              LORA_UART_RX_BUF_SIZE,
                              LORA_UART_TX_BUF_SIZE,
                              20,                    /* 事件队列长度 */
                              &g_lora_rx_queue,
                              0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART 驱动安装失败: %d", ret);
        return ret;
    }

    // 将参数写入串口控制寄存器
    ret = uart_param_config(LORA_UART_NUM, &uart_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART 参数配置失败: %d", ret);
        return ret;
    }

    // 绑定指定的 TX/RX 复用管脚，其余 CTS/RTS 不改变
    ret = uart_set_pin(LORA_UART_NUM,
                       LORA_TX_GPIO, LORA_RX_GPIO,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART 引脚映射失败: %d", ret);
        return ret;
    }

    /* 设置接收 FIFO 触发阈值：当硬件接收 FIFO 积攒到 120 字节时自动触发 DMA 搬运，提升批量传输吞吐率 */
    uart_set_rx_full_threshold(LORA_UART_NUM, 120);

    /* 设置接收超时时间（即 IDLE 检测阈值）
     * 这里设置为 3 个字符传输周期。在 115200 波特率下，传输 1 字符约需 87us，
     * 连续 3 个字符时间内无新数据流入（约 260us），硬件即判定本帧变长数据包结束，触发超时事件 */
    uart_set_rx_timeout(LORA_UART_NUM, 3);

    // 绑定之前实现的事件总线回调函数，使用 ESP_INTR_FLAG_IRAM 标记确保中断函数运行在高速 IRAM 中
    uart_isr_register(LORA_UART_NUM, (intr_handler_t)lora_uart_event_handler,
                      NULL, ESP_INTR_FLAG_IRAM,
                      (QueueHandle_t *)&g_lora_rx_queue);

    // 激活接收中断使能
    uart_enable_rx_intr(LORA_UART_NUM);

    /* -------- 创建发送完成二值信号量 -------- */
    g_uart_tx_done = xSemaphoreCreateBinary();
    if (g_uart_tx_done == NULL) {
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreGive(g_uart_tx_done); // 初始释放信号量，代表串口当前可写

    return ESP_OK;
}

/* ================================================================
 * LoRa 模块核心控制层
 * ================================================================ */

/**
 * @brief 握手检测并初始化连接的 LoRa 硬件模块
 */
esp_err_t lora_init_module(void)
{
    esp_err_t ret;
    char response[128] = {0};

    /* 时序调优：轮询等待 AUX 引脚变低（代表 LoRa 模块内部自检完毕，处于空闲就绪状态） */
    int timeout = 100;
    while (LORA_AUX_READ() && --timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (timeout == 0) {
        ESP_LOGE(TAG, "LoRa 模块 AUX 引脚超时，器件可能未就绪或未连接");
        return ESP_ERR_TIMEOUT;
    }

    /* 临时拉高 MD0 引脚，使模块切换进入配置模式（即接收 AT 指令状态） */
    ret = lora_enter_config_mode();
    if (ret != ESP_OK) return ret;

    /* 发送标准的 "AT" 测试指令进行握手，期望收到 "OK" 响应 */
    ret = lora_send_at_cmd("AT", response, 200);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LoRa 模块对 AT 测试指令无响应，请检查连机线序");
        return ret;
    }
    ESP_LOGI(TAG, "成功识别到 LoRa 模块，回应内容为: %s", response);

    return ESP_OK;
}

/**
 * @brief 统一配置 LoRa 模块的各项核心无线通信参数
 * @param addr 模块的本节点无线地址
 * @param channel 通信频段通道
 * @param power 发射功率
 * @param rate 空中无线速率
 * @param mode 工作模式
 */
esp_err_t lora_set_config(uint16_t addr, uint8_t channel, uint8_t power,
                          uint8_t rate, uint8_t mode)
{
    char cmd_buf[64];
    char resp[128];
    uint8_t addr_h = (addr >> 8) & 0xFF; // 截取地址高 8 位
    uint8_t addr_l = addr & 0xFF;        // 截取地址低 8 位

    /* 1. 设置无线节点专属地址 (十六进制字符串格式) */
    snprintf(cmd_buf, sizeof(cmd_buf), "AT+ADDR=%02X,%02X", addr_h, addr_l);
    if (lora_send_at_cmd(cmd_buf, resp, 500) != ESP_OK) {
        return ESP_FAIL;
    }

    /* 2. 设置设备工作模式 (0 代表普通透传模式) */
    lora_send_at_cmd("AT+CWMODE=0", resp, 200);

    /* 3. 设置无线发送形式 (1 代表定向传输模式，发包前需前置目标节点地址和通道) */
    lora_send_at_cmd("AT+TMODE=1", resp, 200);

    /* 4. 设置射频最大发射功率级别 */
    snprintf(cmd_buf, sizeof(cmd_buf), "AT+TPOWER=%d", power);
    lora_send_at_cmd(cmd_buf, resp, 200);

    /* 5. 设置射频空中通信速率和物理信道 */
    snprintf(cmd_buf, sizeof(cmd_buf), "AT+WLRATE=%d,5", channel);
    lora_send_at_cmd(cmd_buf, resp, 200);

    /* 6. 设置休眠唤醒采样时间 (0 代表常开低延迟模式) */
    lora_send_at_cmd("AT+WLTIME=0", resp, 200);

    /* 7. 修改 LoRa 模块自身的硬件串口波特率，使其与主控匹配 (7 代表波特率 115200) */
    lora_send_at_cmd("AT+UART=7,0", resp, 200);

    // 将工作状态标志切回定向模式
    g_lora_mode = LORA_MODE_DIRECTED;

    ESP_LOGI(TAG, "LoRa 配置写入成功: 地址=0x%04X, 信道=%d, 功率=%ddBm, 空速 19.2kbps",
             addr, channel, power);
    return ESP_OK;
}

/* ================================================================
 * 数据发送实现层
 * ================================================================ */

/**
 * @brief 阻塞式发送原始字节流到 LoRa 模块
 */
esp_err_t lora_send_data(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) return ESP_ERR_INVALID_ARG;
    esp_err_t ret;

    /* 只有当 AUX 引脚为高电平时，才代表模块内部缓冲区未满，可以接收主控写入的数据 */
    int timeout = 500;
    while (!LORA_AUX_READ() && --timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (timeout == 0) {
        ESP_LOGE(TAG, "模块忙，禁止写入");
        return ESP_ERR_TIMEOUT;
    }

    // 通过底层的串口外设及 DMA 通道将流数据推入线缆
    int sent = uart_write_bytes(LORA_UART_NUM, (const char *)data, len);
    if (sent < 0) {
        ESP_LOGE(TAG, "UART 发送执行失败");
        return ESP_FAIL;
    }

    /* 等待 ESP32 本身的硬件 UART 控制器把 FIFO / DMA 里的数据全部吐出完毕 */
    ret = uart_wait_tx_done(LORA_UART_NUM, pdMS_TO_TICKS(500));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "等待串口硬件 FIFO 清空产生超时异常");
    }
    
    /* 此时模块收到串口数据后，AUX 引脚 会被模块拉低。
     * 我们必须等待 AUX 重新变高，通过检测其恢复空闲来确保物理层完全同步，避免后续数据冲刷导致丢包 */
    timeout = 500;
    while (LORA_AUX_READ() && --timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return ESP_OK;
}

/**
 * @brief 定向点对点（或广播）发送封装数据包
 * @note  在定点传输模式下，格式必须严格遵守：[目标地址高字节][目标地址低字节][目标无线信道][真实有效载荷数据...]
 */
esp_err_t lora_send_directed(uint16_t dest_addr, uint8_t dest_chn,
                             const uint8_t *data, uint16_t len)
{
    // ATK-LORA-02 单包无线传输的最大有效载荷限制通常为 253 字节
    if (data == NULL || len == 0 || len > 253) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 动态包装定向数据包头 */
    uint8_t pkt[256];
    uint8_t idx = 0;

    pkt[idx++] = (dest_addr >> 8) & 0xFF; // 写入目的节点地址高字节
    pkt[idx++] = dest_addr & 0xFF;        // 写入目的节点地址低字节
    pkt[idx++] = dest_chn;                // 道写入目的射频物理信道
    memcpy(&pkt[idx], data, len);         // 拷贝实际的业务数据内容
    idx += len;

    // 交付给统一的发送流水线发送
    return lora_send_data(pkt, idx);
}

/* ================================================================
 * AT 指令底层解析器
 * ================================================================ */

/**
 * @brief 发送单条 AT 指令并同步挂起等待响应结果
 */
esp_err_t lora_send_at_cmd(const char *cmd, char *response, uint32_t timeout_ms)
{
    char at_cmd[128];
    int len;

    /* 严格规范：所有 LoRa 模块的 AT 指令尾部必须强制追加 \r\n（回车换行）终止符，否则模块拒绝解析 */
    len = snprintf(at_cmd, sizeof(at_cmd), "%s\r\n", cmd);

    int sent = uart_write_bytes(LORA_UART_NUM, at_cmd, len);
    if (sent < 0) {
        return ESP_FAIL;
    }

    /* 阻塞等待模块在规定超时时间内回传的 ASCII 应答文本 */
    if (response != NULL) {
        memset(response, 0, 128);
        int rx_len = uart_read_bytes(LORA_UART_NUM, (uint8_t *)response,
                                      127, pdMS_TO_TICKS(timeout_ms));
        if (rx_len <= 0) {
            return ESP_ERR_TIMEOUT;
        }
        response[rx_len] = '\0'; // 强制在尾部加上字符串结束标志，防止越界打印
    }

    return ESP_OK;
}

/**
 * @brief 将控制引脚切换为配置状态，使模块准备接收 AT 指令
 */
esp_err_t lora_enter_config_mode(void)
{
    /* 必须等待模块完成手头的所有空中收发动作（AUX 变低代表彻底空闲） */
    while (LORA_AUX_READ()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    LORA_MD0_HIGH();  /* 核心动作：将硬件 MD0 引脚拉高，强制其进入 AT 指令解析状态 */
    vTaskDelay(pdMS_TO_TICKS(50)); // 留出充足的内部状态机建立时间
    g_lora_mode = LORA_MODE_CONFIG;

    return ESP_OK;
}

/**
 * @brief 退出配置状态，将模块切回常规定向/透传无线接收模式
 */
esp_err_t lora_exit_config_mode(void)
{
    LORA_MD0_LOW();   /* 核心动作：将硬件 MD0 引脚拉低，切回正常运作模式 */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 同样需要等待模块内部闪存保存并重启重置就绪 */
    while (LORA_AUX_READ()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    g_lora_mode = LORA_MODE_DIRECTED;
    return ESP_OK;
}

/**
 * @brief 强制 LoRa 模块进入超低功耗休眠模式，用于系统 Deep Sleep 前
 *
 * 执行流程：
 * 1. 将 MD0 拉高，进入 AT 配置模式
 * 2. 发送 AT+SLEEP 指令使模块硬件休眠
 * 3. 将 TX/RX/MD0 引脚设为浮空输入，消除漏电路径
 * 4. 工作状态标志切换为 LORA_MODE_SLEEP
 *
 * @note 模块进入硬休眠后，需通过外部中断或重新上电唤醒，不可通过 UART 恢复通信
 * @return esp_err_t ESP_OK 表示休眠指令已成功下发
 */
esp_err_t lora_enter_sleep_mode(void)
{
    esp_err_t ret;

    /* Step 1: 进入 AT 配置模式 */
    LORA_MD0_HIGH();
    vTaskDelay(pdMS_TO_TICKS(50));
    while (LORA_AUX_READ()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* Step 2: 发送 AT+SLEEP 休眠指令 */
    ret = lora_send_at_cmd("AT+SLEEP", NULL, 1000);

    /* Step 3: 拉低 MD0 退出配置模式 */
    LORA_MD0_LOW();
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Step 4: 将串口引脚和控制引脚设为浮空输入，消除漏电路径 */
    gpio_set_direction(LORA_TX_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LORA_TX_GPIO, GPIO_FLOATING);
    gpio_set_direction(LORA_RX_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LORA_RX_GPIO, GPIO_FLOATING);
    gpio_set_direction(LORA_MD0_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LORA_MD0_GPIO, GPIO_FLOATING);

    g_lora_mode = LORA_MODE_SLEEP;

    ESP_LOGI(TAG, "LoRa 模块已进入硬休眠模式");
    return ret;
}

/**
 * @brief 注册外部用于监听 AUX 状态引脚电平突发跳变的回调接口
 */
void lora_set_aux_callback(void (*callback)(bool rising))
{
    g_aux_callback = callback;
}

/**
 * @brief 获取当前驱动层维护的 LoRa 工作状态标志
 */
uint8_t lora_get_mode(void)
{
    return g_lora_mode;
}