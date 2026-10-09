/**
 * @file mqtt_demo.c
 * @author bling (719095404@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-01-18
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "mqtt.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "esp.h"

uint8_t mqtt_buffer[MQTT_BUFFER_SIZE] = {0}; 

/**
 * @brief MQTT连接报文
 * 
 * @param client_id 客户端ID(用户私钥)
 * @param user_name 用户名
 * @param password 密码
 * @return int32_t 成功返回0，失败返回-1
 */
int32_t mqtt_connect_QoS0(char *client_id, char *user_name, char *password)
{
    if (client_id == NULL)
        return -1;
    uint32_t client_id_len = strlen(client_id);
    uint32_t user_name_len = (user_name == NULL ? 0 : strlen(user_name));
    uint32_t password_len = (password == NULL ? 0 : strlen(password));
    uint8_t encodedByte = 0;  // 临时变量
    uint32_t data_len;        // 剩余长度(不包括固定头部)
    uint32_t mqtt_tx_len = 0; // 发送数据长度
    int32_t recv_len = MQTT_BUFFER_SIZE;

    // ---------------- 固定报头 ---------------
    mqtt_buffer[mqtt_tx_len++] = (1 << 4); // 控制报文类型

    // 剩余长度(不包括固定头部) = 可变报头 + Payload  每个字段包含两个字节的长度标识
    data_len = 10 + (client_id_len + 2) +
               (user_name_len ? user_name_len + 2 : user_name_len) +
               (password_len ? password_len + 2 : password_len);
    do
    {
        encodedByte = data_len % 128;
        data_len = data_len / 128;
        if (data_len > 0) // 如果还有数据，则设置最高位
            encodedByte = encodedByte | (1 << 7);
        mqtt_buffer[mqtt_tx_len++] = encodedByte;
    } while (data_len > 0);

    //  ---------------- 可变报头 ----------------
    // 协议名
    mqtt_buffer[mqtt_tx_len++] = 0;   // Protocol Name Length MSB
    mqtt_buffer[mqtt_tx_len++] = 4;   // Protocol Name Length LSB
    mqtt_buffer[mqtt_tx_len++] = 'M'; // ASCII Code for M
    mqtt_buffer[mqtt_tx_len++] = 'Q'; // ASCII Code for Q
    mqtt_buffer[mqtt_tx_len++] = 'T'; // ASCII Code for T
    mqtt_buffer[mqtt_tx_len++] = 'T'; // ASCII Code for T
    // 协议级别
    mqtt_buffer[mqtt_tx_len++] = 4; // MQTT Protocol version = 4

    // 连接标志
    mqtt_buffer[mqtt_tx_len++] = (!(!user_name_len) << 7) | // user name flag
                              (!(!password_len) << 6) |  // password flag
                              (0 << 5) |                 // will retain
                              (0x0 << 3) |               // will QOs
                              (0 << 2) |                 // will flag
                              (1 << 1);                  // clean session

    // 保持连接时间 60S心跳包
    mqtt_buffer[mqtt_tx_len++] = 0;  // Keep-alive Time Length MSB
    mqtt_buffer[mqtt_tx_len++] = 60; // Keep-alive Time Length LSB

    //  ---------------- 有效载荷 ----------------
    mqtt_buffer[mqtt_tx_len++] = BYTE1(client_id_len); // Client ID length MSB
    mqtt_buffer[mqtt_tx_len++] = BYTE0(client_id_len); // Client ID length LSB
    memcpy((void *)&mqtt_buffer[mqtt_tx_len], client_id, client_id_len);
    mqtt_tx_len += client_id_len;

    if (user_name_len > 0) // User Name
    {
        mqtt_buffer[mqtt_tx_len++] = BYTE1(user_name_len); // user_name length MSB
        mqtt_buffer[mqtt_tx_len++] = BYTE0(user_name_len); // user_name length LSB
        memcpy((void *)&mqtt_buffer[mqtt_tx_len], user_name, user_name_len);
        mqtt_tx_len += user_name_len;
    }

    if (password_len > 0) // Password
    {
        mqtt_buffer[mqtt_tx_len++] = BYTE1(password_len); // password length MSB
        mqtt_buffer[mqtt_tx_len++] = BYTE0(password_len); // password length LSB
        memcpy((void *)&mqtt_buffer[mqtt_tx_len], password, password_len);
        mqtt_tx_len += password_len;
    }

    // 发送数据到MQTT服务器（代理）
    ESP_Send_data_len((const uint8_t *)mqtt_buffer, mqtt_tx_len, 1000);
    
    if (ESP_Get_Receive_Data(mqtt_buffer, &recv_len, 2000) < 0)
        return -1;

    // 连接成功正常会返回20 02 01 00
    if (recv_len >= 4 && (mqtt_buffer[0] == 0x20) && (mqtt_buffer[1] == 0x02))
    {
        if (mqtt_buffer[3] == 0x00)
        {
            printf("连接已被服务器端接受，连接确认成功\r\n");
            return 0;
        }
        else
        {
            printf("返回码:%#x\r\n", mqtt_buffer[3]);
            switch (mqtt_buffer[3])
            {
            case 1:
                printf("连接已拒绝，不支持的协议版本\r\n");
                break;
            case 2:
                printf("连接已拒绝，不合格的客户端标识符\r\n");
                break;
            case 3:
                printf("连接已拒绝，服务端不可用\r\n");
                break;
            case 4:
                printf("连接已拒绝，无效的用户或密码\r\n");
                break;
            case 5:
                printf("连接已拒绝，未授权\r\n");
                break;
            default:
                printf("未知响应\r\n");
                break;
            }
            return -1;
        }
    }
    return -1;
}

/**
 * @brief MQTT订阅报文
 * 
 * @param topic 订阅主题
 * @return int32_t 成功返回0，失败返回-1
 */
int32_t mqtt_subscribe_QoS0(char *topic)
{
    if (topic == NULL)
        return -1;

    uint32_t topic_len = strlen(topic);
    uint8_t encodedByte = 0;  // 临时变量
    uint32_t data_len;        // 剩余长度(不包括固定头部)
    uint32_t mqtt_tx_len = 0; // 发送数据长度
    uint16_t pack_flag = 10;  // 报文标识符
    int32_t recv_len = MQTT_BUFFER_SIZE;

    // ---------------- 固定报头 ----------------
    mqtt_buffer[mqtt_tx_len++] = (1 << 7) | (1 << 1); // 控制报文类型

    // 剩余长度(不包括固定头部) = 可变报头 + Payload
    data_len = 2 + (topic_len + 2) + 1;
    do
    {
        encodedByte = data_len % 128;
        data_len = data_len / 128;
        if (data_len > 0) // 如果还有数据，则设置最高位
            encodedByte = encodedByte | (1 << 7);
        mqtt_buffer[mqtt_tx_len++] = encodedByte;
    } while (data_len > 0);

    //  ---------------- 可变报头 ----------------
    mqtt_buffer[mqtt_tx_len++] = BYTE1(pack_flag); // 报文标识符MSB
    mqtt_buffer[mqtt_tx_len++] = BYTE0(pack_flag); // 报文标识符LSB

    //  ---------------- 有效载荷 ----------------
    mqtt_buffer[mqtt_tx_len++] = BYTE1(topic_len); // topic length MSB
    mqtt_buffer[mqtt_tx_len++] = BYTE0(topic_len); // topic length LSB
    memcpy((void *)&mqtt_buffer[mqtt_tx_len], topic, topic_len);
    mqtt_tx_len += topic_len;
    mqtt_buffer[mqtt_tx_len++] = 0; // 服务质量等级 QoS 0

    // 发送数据到MQTT服务器（代理）
    ESP_Send_data_len((const uint8_t *)mqtt_buffer, mqtt_tx_len, 1000);
    
    if (ESP_Get_Receive_Data(mqtt_buffer, &recv_len, 2000) < 0)
        return -1;

    // 订阅成功正常会返回90 03 00 0A 00
    if (recv_len >= 5 &&
        (mqtt_buffer[0] == ((1 << 7) | (1 << 4))) &&                    // 控制报文类型
        (mqtt_buffer[1] == 0x03) &&                                 // 剩余长度
        (pack_flag == (mqtt_buffer[2] << 8 | mqtt_buffer[3])))  // 报文标识符
    {
        // 返回码
        if (mqtt_buffer[4] == 0x00)     // 订阅成功
        {
            printf("连接已被服务器端接受，连接确认成功\r\n");
            return 0;
        }
        else
        {
            printf("返回码:%#x\r\n", mqtt_buffer[4]);
            switch (mqtt_buffer[4])
            {
            case 0x80:
                printf("订阅失败\r\n");
                break;
            default:
                printf("未知响应\r\n");
                break;
            }
            return -1;
        }
    }

    return 0;
}

/**
 * @brief MQTT发布报文
 * 
 * @param topic 发布主题
 * @param msg 发布消息
 * @return int32_t 成功返回0，失败返回-1
 */
int32_t mqtt_publish_QoS0(char *topic, char *msg)
{
    if (topic == NULL)
        return -1;

    uint32_t topic_len = strlen(topic);
    uint8_t encodedByte = 0;  // 临时变量
    uint32_t data_len;        // 剩余长度(不包括固定头部)
    uint32_t mqtt_tx_len = 0; // 发送数据长度
//  uint16_t pack_flag = 10;  // 报文标识符

    // ---------------- 固定报头 ----------------
    mqtt_buffer[mqtt_tx_len++] = (1 << 4) | (1 << 5); // 控制报文类型

    // 剩余长度(不包括固定头部) = 可变报头 + Payload
    data_len = (2 + topic_len) + strlen(msg);
    do
    {
        encodedByte = data_len % 128;
        data_len = data_len / 128;
        if (data_len > 0) // 如果还有数据，则设置最高位
            encodedByte = encodedByte | (1 << 7);
        mqtt_buffer[mqtt_tx_len++] = encodedByte;
    } while (data_len > 0);

    //  ---------------- 可变报头 ----------------
    mqtt_buffer[mqtt_tx_len++] = BYTE1(topic_len);                // topic length MSB
    mqtt_buffer[mqtt_tx_len++] = BYTE0(topic_len);                // topic length LSB
    memcpy((void *)&mqtt_buffer[mqtt_tx_len], topic, topic_len); // topic
    mqtt_tx_len += topic_len;

    //  ---------------- 有效载荷 ----------------
    memcpy((void *)&mqtt_buffer[mqtt_tx_len], msg, strlen(msg));
    mqtt_tx_len += strlen(msg);

    // 发送数据到MQTT服务器（代理）
    ESP_Send_data_len((const uint8_t *)mqtt_buffer, mqtt_tx_len, 1000);

    // QoS 0 无响应

    return 0;
}

/**
 * @brief 解析获取服务器下发的消息
 *
 * @param topic 主题
 * @param msg 消息
 * @param msg_len 消息长度
 * @return char* 成功：msg内存中，有效负载的指针，失败返回NULL
 */
char *mqtt_parse_msg(char *topic, uint8_t *msg, uint32_t msg_len)
{
    if (topic == NULL || msg == NULL || msg_len <= 2)
        return NULL;
    printf("开始解析服务器下发的消息\r\n");
    uint32_t data_len = 0;    // 剩余长度(不包括固定头部)
    uint32_t index = 0;       // 处理消息下标
    uint16_t topic_len = 0;   // 主题长度
    uint32_t payload_len = 0; // 负载长度
    
    // ---------------- 固定报头 ----------------
    // 解析服务器下发的消息
    if (msg[index++] != ((1 << 4) | (1 << 5))) // 控制报文类型
        return NULL;
    // 剩余长度(不包括固定头部) = 可变报头 + Payload
    for (uint8_t i = 0; i < 4; i++) // 最多4个字节
    {
        data_len = (data_len << 8) | msg[index] & 0x7F;
        if (msg[index++] & (1 << 7)) // 长度还有多字节
        {
            continue;
        }
        else
            break;
    }
    if (data_len <= 2) // 剩余长度数据长度不合法(主题长度占2个字节)
        return NULL;

    // ---------------- 可变报头 ----------------
    topic_len = (msg[index] << 8) | msg[index + 1]; // 主题长度
    index += 2;

    if (strncmp((void *)&msg[index], topic, topic_len) != 0) // 主题不匹配
        return NULL;
    index += topic_len;

    payload_len = data_len - (topic_len + 2); // 负载长度
    if (payload_len <= 0)                     // 负载长度不合法
        return NULL;
    return (char *)&msg[index]; // 返回消息指针
}

/**
 * @brief MQTT心跳包
 * 
 * @return int32_t 成功返回0，失败返回-1
 */
int32_t mqtt_heart_beat(void)
{
    mqtt_buffer[0] = (1 << 7) | (1 << 6); // 控制报文类型
    mqtt_buffer[1] = 0;                // 剩余长度
    int32_t recv_len = MQTT_BUFFER_SIZE;
    ESP_Send_data_len((const uint8_t *)mqtt_buffer, 2, 1000);
    
    if (ESP_Get_Receive_Data(mqtt_buffer, &recv_len, 3000) < 0)
        return -1;

    // 心跳成功正常会返回D0 00
    if (recv_len < 2 ||
        (mqtt_buffer[0] != ((1 << 7) | (1 << 6) | (1 << 4))) || // 控制报文类型
        (mqtt_buffer[2] != 0x00))                                 // 剩余长度
    {
        return -1;
    }
    return 0;
}