/**
 * @file llcc68_p2p.c
 * @brief LLCC68 射频上下文 + DIO1 中断处理
 *        （原厂商点对点示例已拆分：收发和应用逻辑迁到 lora_app.c）
 *
 * 本文件只保留两样东西：
 *   1. llcc68_ctx         —— 硬件上下文（SPI句柄 + NSS/RST/BUSY/DIO1 四个引脚）
 *   2. DIO1_EXTI_Callback —— DIO1 中断的唯一入口
 *
 * 射频参数（频率/扩频因子/带宽/发射功率/前导码长度）在 llcc68_p2p.h 里定义；
 * 初始化、接收、解析、回ACK、帧格式都在 lora_app.c / lora_app.h 里。
 */

#include "llcc68_p2p.h"
#include "llcc68.h"
#include "stm32f1xx_hal.h"
#include "main.h"
#include "spi.h"

llcc68_hal_context_t llcc68_ctx = {
	.hspi = LLCC68_SPI_HANDLE,
	.nss_port = LLCC68_NSS_PORT,
	.nss_pin = LLCC68_NSS_PIN,
	.rst_port = LLCC68_RST_PORT,
	.rst_pin = LLCC68_RST_PIN,
	.busy_port = LLCC68_BUSY_PORT,
	.busy_pin = LLCC68_BUSY_PIN,
	.dio1_port = LLCC68_DIO1_PORT,
	.dio1_pin = LLCC68_DIO1_PIN};

// DIO1 中断标志：中断里把芯片的中断状态读出来存这里（读的动作本身就会清标志），
// 主循环的 lora_app.c 直接取用，避免在中断里做「读接收FIFO」这类耗时操作
volatile llcc68_irq_mask_t g_dio1_irq = 0;

/**
 * @brief DIO1中断回调函数，由 HAL_GPIO_EXTI_Callback() 调用
 *        （挂接点在 ISR_callback.c，对应引脚 PB10）
 *
 * 中断里只做两件事：把芯片的中断状态读出来（这是清标志的唯一手段），
 * 然后存进 g_dio1_irq 交给主循环处理。
 * 不在这里调 printf、不读接收FIFO —— 一是「中断里只置标志位」是本工程的既定约定，
 * 二是 printf 走串口，在中断里打会拖慢整个系统。
 */
void DIO1_EXTI_Callback(void)
{
	llcc68_irq_mask_t irq_status = 0;

	// 获取并清除中断状态（读寄存器即清标志）
	llcc68_get_and_clear_irq_status(&llcc68_ctx, &irq_status);

	// 把状态位并进标志位，交由主循环 LORA_AppService() 逐位处理
	g_dio1_irq |= irq_status;
}
