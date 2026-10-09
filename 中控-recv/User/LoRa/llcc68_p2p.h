/**
 * @file llcc68_p2p.h
 * @author Jaychen (719095404@qq.com)
 * @brief LLCC68 硬件引脚与射频参数定义
 *        （原厂商点对点示例的头文件，收发接口已迁到 lora_app.h）
 *
 * @copyright Copyright (c) 2026
 */
#ifndef LLCC68_P2P_H
#define LLCC68_P2P_H

#include "llcc68.h"
#include "llcc68_hal.h"
#ifdef __cplusplus
extern "C" {
#endif

/************************ 硬件引脚定义 ************************/
// 与 TMPLATE.ioc 一一对应，改动引脚前先改 CubeMX
#define LLCC68_SPI_HANDLE &hspi1
#define LLCC68_NSS_PORT GPIOA
#define LLCC68_NSS_PIN GPIO_PIN_4  // PA4  LORA_SPI_NSS，软件片选，空闲为高
#define LLCC68_RST_PORT GPIOB
#define LLCC68_RST_PIN GPIO_PIN_1  // PB1  LORA_SPI_RESET，空闲为高
#define LLCC68_BUSY_PORT GPIOB
#define LLCC68_BUSY_PIN GPIO_PIN_0 // PB0  LORA_BUSY，高=模块忙
#define LLCC68_DIO1_PORT GPIOB
#define LLCC68_DIO1_PIN GPIO_PIN_10 // PB10 LORA_DIO1，EXTI 上升沿（IRQ 触发时 DIO 被置位，见手册 §13.3.2.1）

/************************ LoRa核心参数 ************************/
// 这几个值必须和另一端（终端节点）完全一致，否则收不到彼此的包
#define LORA_FREQ 470500000UL	   // 470.5MHz(中国频段范围：470.0MHz~510.0MHz)
#define LORA_SF LLCC68_LORA_SF9	   // 扩频因子
#define LORA_BW LLCC68_LORA_BW_125 // 带宽
#define LORA_CR LLCC68_LORA_CR_4_5 // 编码率
#define LORA_PREAMBLE_LEN 8		   // 前导码长度
#define LORA_PAYLOAD_LEN 255	   // 数据包长度上限, llcc68最多支持255字节
#define LORA_TX_POWER_DBM 22	   // 发射功率

/************************ 对外符号 ************************/
extern llcc68_hal_context_t llcc68_ctx;

// DIO1 中断标志位：中断里存，主循环里取用（定义在 llcc68_p2p.c）
extern volatile llcc68_irq_mask_t g_dio1_irq;

// DIO1 中断处理，由 ISR_callback.c 的 HAL_GPIO_EXTI_Callback() 调用
void DIO1_EXTI_Callback(void);

#ifdef __cplusplus
}
#endif
#endif /* LLCC68_P2P_H */
/************************ (C) COPYRIGHT Jaychen ********END OF FILE********/
