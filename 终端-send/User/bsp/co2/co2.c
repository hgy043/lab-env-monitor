#include "co2.h"
#include <string.h>
 
uint8_t co2_buffer[CO2_RX_BUF_SIZE]; // CO2接收缓冲
volatile uint8_t co2_rx_len;         // CO2接收长度

/* 最近一次读到的 CO2 浓度（缓存）。由本文件的 CO2_get_data() 维护，
 * 给 LoRa 上报层取值用 —— 详见 co2.h 里的说明，一句话：串口只有一个读者。 */
uint16_t g_co2_value = 0;
uint8_t g_co2_valid = 0;
 
// 接收中断回调函数,在HAL_UARTEx_RxEventCallback中调用
void CO2_UART_Callback(uint16_t Size)
{
    co2_rx_len = Size;                                                    // 串口接收中断
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, co2_buffer, sizeof(co2_buffer)); // 开启接收数据
    // printf("co2_rx_len = %d, uart1_buf = %s\n", co2_rx_len, co2_buffer);
}
// 启动接收中断
void CO2_UART_Receive_Start(void)
{
    co2_rx_len = 0;
    memset(co2_buffer, 0, sizeof(co2_buffer));
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, co2_buffer, sizeof(co2_buffer));
}
 
/**
 * @brief 获取CO2浓度值
 *
 * @param co2_value CO2浓度值
 * @param timeout 超时时间(ms)
 * @return uint8_t 0: 成功;  1:超时;  2:模块地址错误;  3:校验和错误
 */
uint8_t CO2_get_data(uint16_t *co2_value, uint32_t timeout)
{
    uint32_t start_time = HAL_GetTick();

    /* 先按「本次无效」算，走到最后成功返回前再置回 1。
     * 这样下面三条失败路径（超时/地址错/校验和错）都不用各写一遍清零 ——
     * 读失败时外部就该看到「无效」，而不是把上一次的旧值当成新值 */
    g_co2_valid = 0;

    while (1)
    {
        if (co2_rx_len == CO2_RX_BUF_SIZE)
        {
            break;
        }
        if (HAL_GetTick() - start_time > timeout)
        {
            return 1;
        }
    }
    
    // for (uint8_t i = 0; i < co2_rx_len; i++)
    // {
    //  printf("%02x ", co2_buffer[i]);
    // }
    co2_rx_len = 0; // 重新接收
 
    // 1. 校验模块地址（第1字节为0x2C）
    if (co2_buffer[0] != 0x2C)
    {
        return 2;
    }
 
    // 2. 计算和校验
    uint8_t check_sum = 0;
    for (uint8_t i = 0; i < CO2_RX_BUF_SIZE - 1; i++)
    {
        check_sum += co2_buffer[i];
    }
    // printf("check_sum = %02x\n", check_sum);
    if (check_sum != co2_buffer[CO2_RX_BUF_SIZE - 1]) // 校验和不匹配
    {
        return 3;
    }
 
    // 3. 解析CO?浓度值
    *co2_value = (co2_buffer[1] << 8) | co2_buffer[2];

    // 4. 缓存一份给 LoRa 上报层取值（见 co2.h 的说明）
    g_co2_value = *co2_value;
    g_co2_valid = 1;

    return 0;
}
 