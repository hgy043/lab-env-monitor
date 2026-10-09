#ifndef ISR_CALLBACK_H	// 防止头文件重复包含 
#define ISR_CALLBACK_H 

#include "app_main.h"
#include "debug.h"
// 函数声明
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);	


#endif 
