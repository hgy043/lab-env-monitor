#ifndef APP_MAIN_H // 防止头文件重复包含
#define APP_MAIN_H

#include "main.h"

extern uint16_t adc1_values[2];

// 函数声明

void app_main(void);
void gpio_test(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

// 报警复位键(SW1-PB13)按下标志：在中断里置1，在主循环 alarm_process() 里处理并清零
extern volatile uint8_t g_reset_key_pressed;

// 延时
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
void delay_s(uint32_t s);

#endif
