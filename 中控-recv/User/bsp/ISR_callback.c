/**
 * @file    ISR_callback.c
 * @brief   所有中断回调的集中处
 *
 *  CubeMX 生成的 stm32f1xx_it.c 只负责「进中断 → 调 HAL 的中断处理函数」，
 *  HAL 处理完再去调这里这些用户回调。业务逻辑统一写在这个文件里，
 *  免得散落在各处找不到。
 *
 *  本文件包含三个回调：
 *    HAL_GPIO_EXTI_Callback()        三个 EXTI 引脚都在这里分：
 *                                      PB10 = LoRa DIO1（收发完成/超时/CRC错）
 *                                      PB13 = SW1  按键
 *                                      PB12 = SW2  按键
 *    HAL_UART_RxCpltCallback()       定长接收完成（本工程没用，空函数占位）
 *    HAL_UARTEx_RxEventCallback()    变长/空闲接收，分给两个串口：
 *                                      USART1 → debug_UART_Callback()（调试口）
 *                                      USART2 → ESP_UART_Callback()（ESP8266）
 *
 *  ⚠ 项目约定：**中断里只置标志位，不做耗时操作、不调 printf。**
 *     真正的业务处理放到主循环。DIO1 那一路是这条约定最标准的示范：
 *     DIO1_EXTI_Callback() 只读一次中断状态存进 g_dio1_irq 就返回。
 *
 *  ⚠ 已知偏差：SW1 / SW2 这两段是早期教学代码留下的，里面有
 *     delay_ms(10) 消抖和「等松手」的 while 死等 —— 都跑在中断里，
 *     而且是和 LoRa DIO1 同一个 EXTI15_10 中断向量。也就是说按键一直按着不放，
 *     LoRa 的中断就被一起挡住了。所以 DIO1 那一路放在最前面判、判完直接 return。
 *     目前中控不接按键，暂时不影响；将来要用按键，得把这两段改成
 *     「置标志位 + 主循环里做消抖和松手检测」。
 */
#include "ISR_callback.h"
#include "app_main.h"
#include "debug.h"
#include "esp.h"
#include "llcc68_p2p.h"  // DIO1_EXTI_Callback()：LoRa的DIO1中断处理

//自定义中断处理函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		/*--------------------------------------------------------------------
		 * LoRa DIO1(PB10)：收发完成/超时/CRC错都会把 DIO1 拉高触发这里
		 * （EXTI 配的是上升沿，见 llcc68_p2p.h / 手册 §13.3.2.1）。
		 * 放在最前面判，因为 SW1/SW2 那两段里有 delay_ms(10) 和松手等待，
		 * 排在它们后面的话 LoRa 的中断要等按键处理完才轮得到。
		 * DIO1_EXTI_Callback() 里只读一次中断状态并存进标志位，不调printf。
		 *------------------------------------------------------------------*/
		if (GPIO_Pin == LORA_DIO1_Pin)
		{
			DIO1_EXTI_Callback();
			return; // 提前返回，不用再往下走按键的判断
		}

		//检测SW1_Pin的中断
		//if(HAL_GPIO_ReadPin(SW1_GPIO_Port,SW1_Pin) == GPIO_PIN_RESET)
		if(GPIO_Pin == GPIO_PIN_13) //SW1触发了中断
		{
			//先延时10ms
			delay_ms(10);
			//再读取GPIO口的状态，如果还是0；说明按键低电平是一个稳定的状态
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13) == GPIO_PIN_RESET)
			{
				 //确定了按键已经是低电平,在去翻转LED灯的状态
//				HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

				//SW1作为报警复位键：只置标志位，真正的解除动作放到主循环
				//alarm_process() 里做。中断里不做耗时操作，也不调用printf。
				g_reset_key_pressed = 1;

				//松手检测:在完全松手之前，不要做下一次的动作
				while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13) == GPIO_PIN_RESET); //如果没有松手这个while一直为真
				delay_ms(10); //这个是粗延时 不是用的中断
			}
		}
		
		//检测SW2_Pin的中断
		if(GPIO_Pin == GPIO_PIN_12) //SW2触发了中断
		{
			//先延时10ms
			delay_ms(10);
			//再读取GPIO口的状态，如果还是0；说明按键低电平是一个稳定的状态
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12) == GPIO_PIN_RESET)
			{
				 //确定了按键已经是低电平,在去翻转LED灯的状态
				//HAL_GPIO_TogglePin(BEEP_GPIO_Port, BEEP_Pin);
				
				//松手检测:在完全松手之前，不要做下一次的动作
				while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12) == GPIO_PIN_RESET); //如果没有松手这个while一直为真
				delay_ms(10);
			}
		}	
}


// 定义自己的UART接收完成调用的函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

}

// 定义自己的UART接收空闲调用的函数
// 用 ReceiveToIdle 方式收：收满缓冲区 或 检测到空闲帧，都会回调到这里
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART1) // 串口1（调试口）收到变长数据
    {
        debug_UART_Callback(Size);
    }
    if (huart->Instance == USART2) // 串口2（ESP8266）收到变长数据
    {
        ESP_UART_Callback(Size);
    }
}
