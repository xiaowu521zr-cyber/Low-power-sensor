/**
 * @file    lora_uart.h
 * @brief   LoRa 模块 UART 驱动程序（DMA + IDLE 空闲中断）头文件
 *
 * 通过 UART 串口与正点原子 ATK-LORA-02 模块进行通信。
 * 发送（TX）采用 DMA 搬运以降低 CPU 负载，接收（RX）采用串口空闲总线（IDLE）检测自适应接收变长数据包。
 */

#ifndef __LORA_UART_H__
#define __LORA_UART_H__

#include "config/app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * LoRa 工作模式宏定义
 * ================================================================ */
#define LORA_MODE_CONFIG            0       /* AT 指令配置模式（可以通过串口配置参数） */
#define LORA_MODE_TRANSPARENT       1       /* 无线透传模式（所发即所收，不带协议包头） */
#define LORA_MODE_DIRECTED          2       /* 定向传输/定点通信模式（发包前需前置目标地址和信道） */
#define LORA_MODE_SLEEP             3       /* 超低功耗休眠模式（系统 Deep Sleep 前调用，模块进入硬件休眠状态） */

/* ================================================================
 * LoRa 定向传输模式下的数据包结构体 (Directed Mode)
 * ================================================================ */
/**
 * @brief 定点/定向无线传输时的物理数据包帧结构
 * @note 当 LoRa 模块配置为 LORA_MODE_DIRECTED 时，主控通过串口发送给模块的
 * 前三个字节必须是目标地址和信道，模块内部射频层会自动将其剥离，不发往空中。
 */
typedef struct {
    uint8_t dest_addr_h;            /* 目标节点无线地址高字节 */
    uint8_t dest_addr_l;            /* 目标节点无线地址低字节 */
    uint8_t dest_channel;           /* 目标无线通信射频信道/频段（决定频率） */
    uint8_t data_len;               /* 实际准备发送的业务有效载荷长度 */
    uint8_t data[256];              /* 数据缓冲区，存放实际的业务有效载荷 */
} lora_directed_pkt_t;

/* ================================================================
 * 驱动层接口函数原型声明
 * ================================================================ */

/**
 * @brief 初始化底层 UART 硬件外设、引脚映射、DMA 搬运及空闲线超时接收中断
 * @return esp_err_t ESP_OK 代表配置成功，其他代表初始化失败
 */
esp_err_t lora_uart_init(void);

/**
 * @brief 检查无线模块的硬件联机状态，并使其初始化进入配置状态
 * @note  内部会阻塞轮询 AUX 引脚，并发送 "AT" 进行握手应答测试
 * @return esp_err_t ESP_OK 代表模块正常在线且成功握手
 */
esp_err_t lora_init_module(void);

/**
 * @brief 统一配置 LoRa 模块的核心无线通信属性参数
 * @param addr     本节点的无线识别地址 (0x0000 ~ 0xFFFF)
 * @param channel  工作射频信道 (通常 0 ~ 31 对应不同频率)
 * @param power    发射功率档位 (dBm)
 * @param rate     空中无线传输速率代码
 * @param mode     初始工作模式
 * @return esp_err_t ESP_OK 代表所有配置指令均得到模块的正确 OK 回应
 */
esp_err_t lora_set_config(uint16_t addr, uint8_t channel, uint8_t power,
                          uint8_t rate, uint8_t mode);

/**
 * @brief 阻塞式向 LoRa 模块串口发送原始字节流数据（通用底层流发送）
 * @note  函数内部包含严格的 AUX 状态引脚检测时序，在模块空中发射未完成前会挂起等待，
 * 这是确保大批量连续发包时不发生硬件缓冲区冲刷丢包（目标丢包率<0.5%）的核心物理屏障。
 * @param data 待发送的数据缓冲区指针
 * @param len  待发送的数据字节长度
 * @return esp_err_t ESP_OK 代表数据已完整送出且模块恢复空闲
 */
esp_err_t lora_send_data(const uint8_t *data, uint16_t len);

/**
 * @brief 以定向模式（定点通信）发送一帧指定的无线数据包
 * @note  函数内部会自动在有效载荷前拼接 [目标高地址][目标低地址][目标信道] 3字节包头
 * @param dest_addr 目标接收节点的无线地址
 * @param dest_chn  目标接收节点当前监听的射频信道
 * @param data      准备发送的业务载荷数据指针
 * @param len       业务载荷长度（最大支持 253 字节）
 * @return esp_err_t ESP_OK 代表定向包组装并发送成功
 */
esp_err_t lora_send_directed(uint16_t dest_addr, uint8_t dest_chn,
                             const uint8_t *data, uint16_t len);

/**
 * @brief 发送单条标准的 AT 指令并同步挂起等待模块返回的文本应答
 * @note  函数内部会自动在指令尾部追加 \r\n 终止符，并阻塞等待直到超时或收到数据
 * @param cmd        不带回车换行的 AT 指令字符串 (例如 "AT+CSQ")
 * @param response   用于存放模块回复 ASCII 字符串应答的缓冲区指针 (可传 NULL 忽略接收)
 * @param timeout_ms 等待接收响应的最大超时时间（毫秒）
 * @return esp_err_t  ESP_OK 代表在超时前成功接收到应答
 */
esp_err_t lora_send_at_cmd(const char *cmd, char *response, uint32_t timeout_ms);

/**
 * @brief 控制硬件 MD0 引脚拉高，使 LoRa 模块强制中断无线收发并进入 AT 指令配置模式
 * @return esp_err_t ESP_OK 代表模式切换完成
 */
esp_err_t lora_enter_config_mode(void);

/**
 * @brief 控制硬件 MD0 引脚拉低，使 LoRa 模块退出配置状态并恢复常规定向无线通信模式
 * @note  退出后内部会阻塞等待 AUX 引脚拉高，确保模块重新初始化准备就绪
 * @return esp_err_t ESP_OK 代表成功切回通信模式
 */
esp_err_t lora_exit_config_mode(void);

/**
 * @brief 强制 LoRa 模块进入超低功耗休眠模式，用于系统 Deep Sleep 前
 *
 * 执行流程：
 * 1. 将 MD0 拉高，进入 AT 配置模式
 * 2. 发送 AT+SLEEP 指令使模块硬件休眠
 * 3. 将 TX/RX/MD0 引脚设为浮空输入，消除漏电路径
 * 4. 工作状态标志切换为 LORA_MODE_SLEEP
 *
 * @note 模块进入硬休眠后，需重新上电或外部中断唤醒，不可通过 UART 恢复通信
 * @return esp_err_t ESP_OK 表示休眠指令已成功下发
 */
esp_err_t lora_enter_sleep_mode(void);

/**
 * @brief 注册外部用于监听 AUX 硬件引脚电平突发跳变（如硬件中断）的回调函数
 * @param callback 指向回调函数的指针，入参 bool 代表当前是否为上升沿
 */
void lora_set_aux_callback(void (*callback)(bool rising));

/**
 * @brief 获取当前驱动内部软件层维护记录的 LoRa 工作状态标志
 * @return uint8_t 当前模式代码 (LORA_MODE_CONFIG / TRANSPARENT / DIRECTED / SLEEP)
 */
uint8_t lora_get_mode(void);

#ifdef __cplusplus
}
#endif

#endif /* __LORA_UART_H__ */