/* ===========================================================================
 * ⚠ 这个文件**已经不在 Keil 工程里编译**，只是留在这里当参考。
 *
 * 它是原厂示例（03-工程demo/26-1、27 两个网关例程里的同名文件），
 * 里面的 mqtt_demo() 函数体是一个 **while(1) 死循环**：
 *   - 挂到 app_main 的 while(1) 里，后面的 LoRa 接收和 OLED 一行都执行不到；
 *   - 里面用 while (ESP_Init()) {} 死等 WiFi，连不上就永远出不来。
 *
 * 2026-09-22 已被 User/MQTT/mqtt_app.c 的 MQTT_AppService() 取代：
 *   同样的连接 + 订阅 + 发布 + 心跳，改成了按主循环节奏推进的非阻塞状态机，
 *   发布内容也换成了 LoRa 收到的真实传感器数据。
 *
 * 不要把 mqtt_demo() 加回主循环，也不要直接照抄这个文件里的写法。
 * 拿它对照 mqtt.c 的接口用法是可以的（怎么调 CONNECT/SUBSCRIBE/PUBLISH）。
 * ========================================================================= */
#include "mqtt_demo.h"
#include "mqtt.h"
#include "esp.h"
#include <string.h>
#include <stdio.h>
#include "gpio.h"

static int32_t msg_len = MQTT_BUFFER_SIZE;
static uint8_t msg_buf[MQTT_BUFFER_SIZE];
void mqtt_demo(void)
{
    uint32_t publish_tick = HAL_GetTick();
    uint32_t heart_tick = HAL_GetTick();
    uint8_t fisrt = 1;
    uint32_t data = 0;

loop:
    while (ESP_Init()) // 等待ESP连接成功
    {
    }
    printf("连接MQTT服务器\n");

    if (mqtt_connect_QoS0(MQTT_CLIENT_ID, NULL, NULL)) // mqtt连接巴法云控制平台（私钥在 wifi_config.h）
    {
        printf("连接失败\n");
        return ;
    }

    printf("订阅主题\n");
    if (mqtt_subscribe_QoS0("light")) // 订阅主题
    {
        printf("订阅light失败\n");
    }

		
    if (mqtt_subscribe_QoS0("soil")) // 订阅主题
    {
        printf("订阅soil失败\n");
    }
		
		
    if (mqtt_subscribe_QoS0("co2")) // 订阅主题
    {
        printf("订阅co2失败\n");
    }
    while (1)
    {
        if (HAL_GetTick() - publish_tick > 20000 || fisrt) // 20秒发布消息
        {
            fisrt = 0;
            publish_tick = HAL_GetTick();
            snprintf((char *)msg_buf, MQTT_BUFFER_SIZE, "#%d", data++);
            mqtt_publish_QoS0("light/set", (char *)msg_buf); // 发布主题,set以防止自己推送的消息被自己接收
            mqtt_publish_QoS0("soil/set", (char *)msg_buf);
						mqtt_publish_QoS0("co2/set", (char *)msg_buf);
					printf("发布主题\n");
        }

        if (HAL_GetTick() - heart_tick > 60000) // 60秒发送心跳包
        {
            heart_tick = HAL_GetTick();
            if (mqtt_heart_beat())// 心跳包
            {
                printf("心跳包失败\n");
                goto loop;  // 重新连接
            }
            printf("心跳包\n");
        }

        msg_len = MQTT_BUFFER_SIZE;
        memset(msg_buf, 0, MQTT_BUFFER_SIZE);
        msg_len = ESP_Get_Receive_Data(msg_buf, &msg_len, 1000);
        if (msg_len >= 0)
        {
            printf("解析服务器下发的消息\n");
            char *msg_parse = mqtt_parse_msg("light", msg_buf, msg_len);
						
            if (msg_parse != NULL)
                printf("服务器下发的消息:%s\n", msg_parse);
            else
                printf("解析light失败\n");
				
						msg_parse = mqtt_parse_msg("soil", msg_buf, msg_len);
						if (msg_parse != NULL)
                printf("服务器下发的消息:%s\n", msg_parse);
            else
                printf("解析soil失败\n");
						
						msg_parse = mqtt_parse_msg("co2", msg_buf, msg_len);
						if (msg_parse != NULL)
                printf("服务器下发的消息:%s\n", msg_parse);
            else
                printf("解析co2失败\n");
					
        }
				
			
    }
}