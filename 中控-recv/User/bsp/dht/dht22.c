#include "dht22.h"
#include "main.h" 
#include "app_main.h"
#include <stdio.h>

/************************* 私有函数声明 *************************/
// 设置引脚为输出模式
static void DHT22_SetOutputMode(void);

// 设置引脚为输入模式
static void DHT22_SetInputMode(void);

// 发送起始信号
static uint8_t DHT22_SendStartSignal(void);

// 读取一个字节数据
static uint8_t DHT22_ReadByte(void);

/************************* 引脚模式配置 *************************/
static void DHT22_SetOutputMode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    GPIO_InitStruct.Pin = DHT22_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // 推挽输出
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT22_GPIO_PORT, &GPIO_InitStruct);
}

static void DHT22_SetInputMode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    GPIO_InitStruct.Pin = DHT22_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;     // 输入模式
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT22_GPIO_PORT, &GPIO_InitStruct);
}


// 这个用STM32CubeMX已经配置了
void DHT22_Init(void)
{
    // __HAL_RCC_GPIOA_CLK_ENABLE();  // 使能GPIOA时钟（根据实际引脚修改）
    
    // DHT22_SetOutputMode();
    // HAL_GPIO_WritePin(DHT22_GPIO_PORT, DHT22_GPIO_PIN, GPIO_PIN_SET);  // 空闲状态为高电平
}

uint8_t DHT22_ReadData(float *temp, float *humi)
{
    uint8_t buf[5] = {0};  // 存储40位数据：湿度高8位、湿度低8位、温度高8位、温度低8位、校验和
    uint8_t i, ret = 0;
    
    // 1. 发送起始信号
    ret = DHT22_SendStartSignal();
    if(ret != 0)
    {
        return 1;  // 响应超时
    }
    
    // 2. 读取40位数据（5个字节）
    for(i = 0; i < 5; i++)
    {
        buf[i] = DHT22_ReadByte();
        if(buf[i] == 0xFF)  // 读取字节超时
        {
            return 2;
        }
    }
    
    DHT22_SetOutputMode();
    // 主机释放总线（拉高）
    HAL_GPIO_WritePin(DHT22_GPIO_PORT, DHT22_GPIO_PIN, GPIO_PIN_SET);

    // 3. 校验数据（前4字节之和的低8位等于第5字节）
    if(((buf[0] + buf[1] + buf[2] + buf[3]) & 0xFF) != buf[4])
    {
        return 3;  // 校验失败
    }
    
    // 4. 解析温湿度数据
    // 湿度：(buf[0]<<8 | buf[1]) / 10.0 （单位：%RH）
    *humi = (float)((buf[0] << 8) | buf[1]) / 10.0f;
    
    // 温度：(buf[2]<<8 | buf[3]) / 10.0 （单位：℃，buf[2]最高位为1表示负温度）
    if(buf[2] & 0x80)  // 负温度
    {
        *temp = (float)(((buf[2] & 0x7F) << 8) | buf[3]) / -10.0f;
    }
    else  // 正温度
    {
        *temp = (float)((buf[2] << 8) | buf[3]) / 10.0f;
    }
    
    return 0;  // 读取成功
}

/************************* 私有函数实现 *************************/
static uint8_t DHT22_SendStartSignal(void)
{
    uint32_t timeout = 0;
    
    // 1. 主机拉低总线至少500us
    DHT22_SetOutputMode();
    HAL_GPIO_WritePin(DHT22_GPIO_PORT, DHT22_GPIO_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);  
    
    // 2. 主机释放总线（拉高），等待从机响应
    HAL_GPIO_WritePin(DHT22_GPIO_PORT, DHT22_GPIO_PIN, GPIO_PIN_SET);
    delay_us(30);  // 拉高30us
    DHT22_SetInputMode();
    
    timeout = 100;
    while(HAL_GPIO_ReadPin(DHT22_GPIO_PORT, DHT22_GPIO_PIN) == GPIO_PIN_SET)
    {
        delay_us(1);
        if(--timeout == 0)
        {
            return 1;  // 响应超时
        }
    }

    // 4. 等待从机释放总线（从机拉高总线80us）
    timeout = 100;
    while(HAL_GPIO_ReadPin(DHT22_GPIO_PORT, DHT22_GPIO_PIN) == GPIO_PIN_RESET)
    {
        delay_us(1);
        if(--timeout == 0)
        {
            return 2;  // 响应超时
        }
    } 
    return 0;  // 响应成功
}

static uint8_t DHT22_ReadByte(void)
{
    uint8_t byte = 0;
    uint32_t timeout = 0;
    uint8_t i;
    
    for(i = 0; i < 8; i++)
    {
        byte <<= 1;  // 左移，准备接收下一位
        // 等待从机拉低总线
        timeout = 100;
        while(HAL_GPIO_ReadPin(DHT22_GPIO_PORT, DHT22_GPIO_PIN) == GPIO_PIN_SET)
        {
            delay_us(1);
            if(--timeout == 0)
            {
                return 0xFF;  // 超时
            }
        }

        // 等待从机拉高总线（每bit起始信号：50us低电平）
        timeout = 60;
        while(HAL_GPIO_ReadPin(DHT22_GPIO_PORT, DHT22_GPIO_PIN) == GPIO_PIN_RESET)
        {
            delay_us(1);
            if(--timeout == 0)
            {
                return 0xFF;  // 超时
            }
        }
        
        // 检测高电平持续时间：
        // 0bit：26~28us 高电平；1bit：70us 高电平
        delay_us(30);  // 等待30us后检测电平
        if(HAL_GPIO_ReadPin(DHT22_GPIO_PORT, DHT22_GPIO_PIN) == GPIO_PIN_SET)
        {
            byte |= 0x01;  // 高电平持续超过50us，为1
        }   
    }
    
    return byte;
}
