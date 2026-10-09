#ifndef CO2_H
#define CO2_H
 
#include "main.h"
#include "usart.h"
 
#define CO2_RX_BUF_SIZE 6 // 调试接收缓冲区大小
 
extern uint8_t co2_buffer[CO2_RX_BUF_SIZE]; // CO2接收缓冲
extern volatile uint8_t co2_rx_len;         // CO2接收长度
 
void CO2_UART_Callback(uint16_t Size);		//调用
void CO2_UART_Receive_Start(void);		//启动
uint8_t CO2_get_data(uint16_t *co2_value, uint32_t timeout);
#endif
