#ifndef __DHT22_H
#define __DHT22_H

#include "stm32f1xx_hal.h"

/************************* 引脚配置 *************************/
// 自定义DHT22数据引脚（示例：PC51）
#define DHT22_GPIO_PORT    GPIOC
#define DHT22_GPIO_PIN     GPIO_PIN_15

/************************* 函数声明 *************************/
// 初始化DHT22引脚
void DHT22_Init(void);

// 读取DHT22温湿度数据
// 参数：temp-温度存储地址（单位：℃，精度0.1℃），humi-湿度存储地址（单位：%RH，精度0.1%）
// 返回值：0-读取成功，1-响应超时（起始信号没等到从机应答），
//         2-数据接收超时（读字节超时），3-校验失败（前4字节之和低8位≠第5字节）
//   注：这三个错误码的对应关系以 dht22.c 里的实现为准 —— 本注释曾经把 1 和 3 写反过。
uint8_t DHT22_ReadData(float *temp, float *humi);

#endif
