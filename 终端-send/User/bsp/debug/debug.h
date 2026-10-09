#ifndef DEBUG_H
#define DEBUG_H

#include "main.h"
#include "string.h"

/* 调试串口 USART1（PA9/PA10，115200）。见 debug.c 头部说明。 */

//轮询收发（教学测试函数，内部有 while(1)，不要在主循环里调）
void debug_test1(void);

#define DEBUG_RX_BUF_SIZE	128
extern uint8_t debug_buffer[DEBUG_RX_BUF_SIZE]; // debug接收缓冲  DEBUG_RX_BUF_SIZE=>128
extern uint16_t debug_rx_len;                    // debug接收长度

void debug_UART_Callback(uint16_t Size);
void Debug_UART_Receive_Start(void);

/* 硬件异常（HardFault）现场报告：由 Core/Src/stm32f1xx_it.c 的 HardFault_Handler 调用。
 * 打印完**不返回**（函数内部就是死循环），为什么这么做见 debug.c 的说明。 */
void HardFault_Report(void);

#endif // DEBUG_H
