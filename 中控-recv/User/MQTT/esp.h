#ifndef ESP_H
#define ESP_H

#include "usart.h"
#include "main.h"
#include <stdint.h>

/*---------------------------------------------------------------------------
 * ESP8266-12E WiFi 模块（巴法云 TCP 透传）
 *   接线：USART2 —— PA2(TXD2) → 模块 RXD，PA3(RXD2) → 模块 TXD
 *   ⚠️ USART2 波特率必须是 115200（ESP8266 AT 固件出厂默认值）。
 *      这个值在 TMPLATE.ioc 里配，改了要用 CubeMX 重新生成，别手改 usart.c。
 *      USART1(PA9/PA10, 115200) 是留给电脑的调试口，两者别搞混。
 *-------------------------------------------------------------------------*/

#define ESP_UART_HANDLE &huart2 // ESP串口句柄（USART2）
#define ESP_RX_BUF_SIZE 128     // ESP接收缓冲区大小

/* ↑ 缓冲区 128 字节：AT 指令的回显很短，够用。
 *   将来透传模式下要接巴法云下发的长报文时，这里要按最长报文调大，
 *   同时注意 STM32F103C8T6 只有 20KB RAM（当前静态占用约 1.8KB）。 */

/* WiFi 账号 / 巴法云私钥放在 wifi_config.h（**不进公开仓库**）：
 *   首次编译前把同目录的 wifi_config.example.h 复制成 wifi_config.h，
 *   填上自己的热点名、密码和巴法云私钥，改值只改那一个文件。 */
#include "wifi_config.h"

/* ↑ 中文 SSID 要留意：ESP8266 AT 固件对 UTF-8 的 SSID 兼容性一般，
 *   个别固件版本会退回 3 号错误（找不到热点）。
 *   如果确认热点是 2.4GHz、密码也对，却一直报 +CWJAP:3，
 *   先把热点名改成纯英文数字再试一次，能连上就是模块对中文 SSID 支持的问题。 */

#define TCP_SERVER "bemfa.com" // 巴法云服务器
#define TCP_SERVER_PORT "9501" // 巴法云MQTT端口（TCP明文透传口，不是 8344）

/* WiFi 连接失败后的重试间隔（ms）
 *   ⚠ 2026-09-22 起这个宏**已经没人用了**：重试逻辑连同 MQTT 一起搬到了
 *     User/MQTT/mqtt_app.c，周期改由那边的 MQTT_RETRY_PERIOD_MS 控制
 *     （值一样是 15s，但重试的是「WiFi + TCP + MQTT 连接 + 订阅」整条链路）。
 *     留着这个宏只是为了让原来读 esp.h 的人能找到线索。
 *   无论哪个值，注意每重试一次，ESP_Init() 会阻塞最多十几秒
 *   （等开机2s + CWJAP超时10s），这期间收完的包没法及时挂回接收，可能漏一两包。
 *   嫌漏包就把 mqtt_app.h 里那个值调大。 */
#define ESP_RETRY_PERIOD_MS (15000U)

extern uint8_t ESP_buffer[ESP_RX_BUF_SIZE]; // ESP接收缓冲
extern int32_t ESP_rx_len;                  // ESP接收长度

void ESP_UART_Callback(uint16_t Size);
void ESP_UART_Receive_Start(void);

HAL_StatusTypeDef ESP_Send_data_len(const uint8_t *data, uint16_t len, uint16_t timeout);
int32_t ESP_Get_Receive_Data(uint8_t *recv_buf, int32_t *recv_len, uint32_t timeout);
int8_t ESP_Send_AT_Cmd(const char *cmd, const char *wait_string, uint32_t timeout);
int8_t ESP_Exit_Transmit_Mode(void);
int8_t ESP_Connect_WiFi(char *ssid, char *password);
int8_t ESP_Connect_Server(char *ip, char *port);
int8_t ESP_Init(void);

#endif
