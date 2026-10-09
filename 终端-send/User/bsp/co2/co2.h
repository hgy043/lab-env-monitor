#ifndef CO2_H
#define CO2_H
 
#include "main.h"
#include "usart.h"
 
#define CO2_RX_BUF_SIZE 6 // 调试接收缓冲区大小
 
extern uint8_t co2_buffer[CO2_RX_BUF_SIZE]; // CO2接收缓冲
extern volatile uint8_t co2_rx_len;         // CO2接收长度

/* 最近一次读到的 CO2 浓度（缓存），由 CO2_get_data() 自己维护。
 *
 *   为什么要缓存这一份：CO2 模块**没有「当前值寄存器」可读**，
 *   它是**主动周期上报**的（规格书里只有一种数据格式、没有任何命令码，
 *   co2.c 里也从不发请求），所以**只能有一个读者** ——
 *   主循环 1 秒读一次并更新这里，LoRa 上报时直接取缓存值。
 *   如果 LoRa 那边也去调 CO2_get_data()，两边会抢同一帧：
 *   谁先读到谁把接收长度清掉，另一个必然超时，还会误触发蜂鸣器报警。
 *
 *   g_co2_valid：0 = 最近一次读取失败（或还没读过），此时 g_co2_value 不可信。
 *   每次调 CO2_get_data() 都会重刷这两个变量，LoRa 帧里填的就是这里的值。
 *   当前只有终端（send）工程编译本模块，中控只按帧格式解析，不引用这两个变量。 */
extern uint16_t g_co2_value;
extern uint8_t g_co2_valid;

void CO2_UART_Callback(uint16_t Size);		//调用
void CO2_UART_Receive_Start(void);		//启动
uint8_t CO2_get_data(uint16_t *co2_value, uint32_t timeout);
#endif
