/**
 * @file esp.c
 * @author bling (1137252884@qq.com)
 * @brief ESP TCP连接巴法云
 * @version 0.1
 * @date 2025-12-09
 *
 * @copyright Copyright (c) 2025
 *
 * ---------------------------------------------------------------------------
 * 【本工程适配说明】（2026-09-21 补）
 *   硬件：ESP8266-12E 挂在 USART2 上（PA2=TXD2、PA3=RXD2），
 *         USART1（PA9/PA10）留给电脑调试口。
 *   ⚠️ USART2 的波特率必须是 115200 —— 这是 ESP8266 AT 固件的出厂默认值。
 *      中控板原来的 .ioc 是从终端板继承来的，USART2 还是 9600（那是给
 *      CO2 传感器用的），所以模块收到的全是乱码、一句 AT 都不回。
 *      改波特率要动 TMPLATE.ioc 再用 CubeMX 重新生成，不要手改 usart.c。
 *
 *   连接流程：ESP_Init() → 等 2s 开机 → 退出透传 → AT/ATE0/CWMODE=3 →
 *             CWJAP 连热点 → CIPMODE=1 → CIPSTART 连巴法云 → CIPSEND 进透传。
 *             全程靠 AT 指令 + 等待回显，任何一步失败都返回 -1 并且不再往下走。
 *
 *   不阻塞的用法：ESP_Init() 本身是阻塞的（最坏十几秒），所以不要写成
 *             while(ESP_Init()){} 那种死等 —— 那样 WiFi 连不上就永远出不来，
 *             后面的 OLED 和 LoRa 一行都不会执行。正确做法见 app_main.c：
 *             启动时先跑别的，把 ESP_Init() 放到主循环里按周期重试。
 * ---------------------------------------------------------------------------
 */
#include "esp.h"
#include <string.h>
#include <stdio.h>

uint8_t ESP_buffer[ESP_RX_BUF_SIZE];
int32_t ESP_rx_len;
// 接收中断回调函数,在HAL_UARTEx_RxEventCallback中调用
void ESP_UART_Callback(uint16_t Size)
{
    ESP_rx_len = (int32_t)Size;                                                           // 串口接收中断
    HAL_UARTEx_ReceiveToIdle_IT(ESP_UART_HANDLE, ESP_buffer, sizeof(ESP_buffer)); // 开启接收数据
    // printf("ESP_rx_len = %d, uart1_buf = %s\n", ESP_rx_len, ESP_buffer);
}
// 启动接收中断
void ESP_UART_Receive_Start(void)
{
    ESP_rx_len = 0;
    memset(ESP_buffer, 0, sizeof(ESP_buffer));
    HAL_UARTEx_ReceiveToIdle_IT(ESP_UART_HANDLE, ESP_buffer, sizeof(ESP_buffer));
}

HAL_StatusTypeDef ESP_Send_data_len(const uint8_t *data, uint16_t len, uint16_t timeout)
{
    ESP_rx_len = 0;
    memset(ESP_buffer, 0, ESP_RX_BUF_SIZE);
    return HAL_UART_Transmit(ESP_UART_HANDLE, data, len, timeout); // 数据发送
}

/**
 * @brief 获取接收数据
 *
 * @param recv_buf 接收数据缓冲区
 * @param recv_len 接收数据长度
 * @param timeout 超时时间
 * @return int32_t 成功返回剩余时间，失败返回-1
 */
int32_t ESP_Get_Receive_Data(uint8_t *recv_buf, int32_t *recv_len, uint32_t timeout)
{
    uint32_t start_tick = HAL_GetTick(); // 记录计时起始时间戳
    uint32_t elapsed_time = 0;           // 计算实际消耗时间,
    uint32_t remaining_timeout = 0;      // 剩余时间

    while (ESP_rx_len <= 0)
    {
        if ((HAL_GetTick() - start_tick) >= timeout)
        {
            break;
        }
        HAL_Delay(1);
    }
    if (ESP_rx_len > 0)
    {
        elapsed_time = HAL_GetTick() - start_tick;
        remaining_timeout = (timeout >= elapsed_time) ? (timeout - elapsed_time) : 0;
        *recv_len = (*recv_len > ESP_rx_len) ? (ESP_rx_len) : (*recv_len);
        memcpy(recv_buf, ESP_buffer, *recv_len);
        ESP_rx_len = 0; // 清空接收缓冲,继续接收
        return remaining_timeout;
    }
    else
    {
        return -1;
    }
}

/**
 * @brief 发送指令，并指定时间内接收指定的数据
 *
 * @param cmd  AT指令注意带\r\n
 * @param wait_string   等待字符串
 * @param timeout  超时时间ms
 * @return int8_t 0:成功 -1:失败
 */
int8_t ESP_Send_AT_Cmd(const char *cmd, const char *wait_string, uint32_t timeout)
{
    int32_t remaining_timeout = (int32_t)timeout;
    uint8_t recv_buf[50];
    int32_t recv_len = sizeof(recv_buf) / sizeof(recv_buf[0]);
    ESP_Send_data_len((const uint8_t *)cmd, strlen(cmd), 1000); // 发送命令

    /* wait_string 传 NULL 表示「只发不等」——像退出透传的 "+++" 本来就不该有回显。
     *   必须在这里拦住：下面那句 strstr(ESP_buffer, wait_string) 收到 NULL 就是
     *   解引用空指针（未定义行为，原厂代码就是这么写的）。
     *   这类调用统一按 timeout 纯等待后返回 -1（"没等到任何东西"），
     *   调用方 ESP_Connect_WiFi() 不看这个返回值，所以不影响功能。
     *   1000ms 也正好满足 "+++" 前后各需 1s 静默的时序要求。 */
    if (wait_string == NULL)
    {
        HAL_Delay(timeout);
        return -1;
    }

    while (1)
    {
        remaining_timeout = ESP_Get_Receive_Data(recv_buf, &recv_len, (uint32_t)remaining_timeout);
        if (remaining_timeout < 0)
        {
            break;
        }
        else
        {
            if (strstr((const char *)ESP_buffer, wait_string))
            {
                return 0; // 成功
            }
            else if (remaining_timeout == 0)
            {
                break;
            }
        }
    }
    return -1;
}

// 退出透传模式
int8_t ESP_Exit_Transmit_Mode(void)
{
    if (ESP_Send_AT_Cmd("+++", NULL, 1000)) // 退出透传模式
    {
        // printf("ESP Exit Transmit Mode , Error\r\n");
        return -1;
    }
    printf("ESP Exit Transmit Mode , Success\r\n");
    return 0;
}

int8_t ESP_Connect_WiFi(char *ssid, char *password)
{
    uint8_t i = 0;
    /* 指令缓冲：AT+CWJAP="<ssid>","<密码>"。
     *   之前是 50 字节，而 10 字节 SSID + 12 字节密码拼出来就已经 40 字节，
     *   只剩 10 字节余量 —— 一旦 SSID/密码改长一点，snprintf 会静默截断，
     *   发出去的指令就是残缺的，模块只会回 ERROR 甚至不回。
     *   放到 128 字节，够 40 字节 SSID + 40 字节密码，不再有截断风险。 */
    char cmd[128];            // 指令缓冲
    ESP_Exit_Transmit_Mode(); // 退出透传模式

    while (ESP_Send_AT_Cmd("AT\r\n", "OK", 500)) // 测试模块状态
    {
        i++;
        if (i >= 3)
        {
            printf("ESP Send cmd: AT , Error\r\n");
            return -1;
        }
    }
    if (ESP_Send_AT_Cmd("ATE0\r\n", "OK", 500)) // 关闭回显
    {
        printf("ESP Send cmd: ATE0 , Error\r\n");
        return -1;
    }

    if (ESP_Send_AT_Cmd("AT+CWMODE=3\r\n", "OK", 500)) // 混合wifi模式 (去连接wifi路由器)
    {
        printf("ESP Send cmd: AT+CWMODE=3 , Error\r\n");
        return -1;
    }

    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, password); // 拼接指令
    printf("ESP 正在连接热点 \"%s\" ...\r\n", ssid);
    if (ESP_Send_AT_Cmd(cmd, "OK", 10000)) // 连接WiFi，10s 超时（关联热点本身就要几秒）
    {
        printf("ESP Send cmd: %s, Error\r\n", cmd);
        /* 把模块的真实回显打出来 —— 这是排查「连不上热点」最关键的信息。
         *   ESP8266 连接失败会回 +CWJAP:<reason>，对照这张表：
         *     1 = 连接超时（热点不在范围内 / 信号太弱 / 是 5GHz 频段）
         *     2 = 密码错误
         *     3 = 找不到该热点（SSID 写错 / 热点没开 / ESP8266 只认 2.4GHz）
         *     4 = 连接失败（加密方式不兼容等）
         *   如果这里打出来是空的、或全是乱码，说明模块压根没回话 ——
         *   那就是串口问题，不是 WiFi 参数问题：USART2 必须是 115200，
         *   且 PA2 接模块 RXD、PA3 接模块 TXD（收发不能接反）。 */
        printf("ESP 模块回显: %s\r\n", ESP_buffer);
        return -1;
    }
    printf("ESP 热点连接成功\r\n");
    return 0;
}

// 连接服务器bemfa.com，TCP端口8344, MQTT端口：9501，连接成功后，进入透传模式
int8_t ESP_Connect_Server(char *ip, char *port)
{
    char cmd[128];                                       // 指令缓冲，同 ESP_Connect_WiFi 的理由
    if (ESP_Send_AT_Cmd("AT+CIPMODE=1\r\n", "OK", 2000)) // 设置透传模式
    {
        printf("ESP Send cmd: AT+CIPMODE=1 , Error\r\n");
        return -1;
    }
    // 连接服务器和端口AT+CIPSTART="TCP","bemfa.com",8344
    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", ip, port);
    if (ESP_Send_AT_Cmd(cmd, "OK", 5000)) // 连接服务器
    {
        printf("ESP Send cmd: %s, Error\r\n", cmd);
        return -1;
    }
    // 进入透传模式，后面发的都会无条件传输
    if (ESP_Send_AT_Cmd("AT+CIPSEND\r\n", "OK", 3000)) // 进入透传模式
    {
        printf("ESP Send cmd: AT+CIPSEND\r\n, Error\r\n");
        return -1;
    }
    return 0;
}

// ESP连接WiFi并连接服务器
int8_t ESP_Init(void)
{
    HAL_Delay(2000); // 等待ESP开机
    ESP_UART_Receive_Start();
    printf("ESP Start, WIFI connecting...\r\n");
    if (ESP_Connect_WiFi(WIFI_SSID, WIFI_PASSWORD)) // 连接WiFi
    {
        printf("ESP Connect WiFi Error\r\n");
        return -1;
    }
    printf("ESP Connect WiFi Success\r\n");

    if (ESP_Connect_Server(TCP_SERVER, TCP_SERVER_PORT)) // 连接服务器
    {
        printf("ESP Connect Server Error\r\n");
        return -1;
    }
    printf("ESP Connect Server Success\r\n");
    return 0;
}
