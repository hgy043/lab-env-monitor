#ifndef __MQTT_H
#define __MQTT_H

#include "main.h"
#include "esp.h"

#define BYTE0(dwTemp) (*(char *)(&dwTemp))       // 取第0个字节
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1)) // 取第1个字节
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2)) // 取第2个字节
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3)) // 取第3个字节

#define MQTT_BUFFER_SIZE ESP_RX_BUF_SIZE
extern uint8_t mqtt_buffer[MQTT_BUFFER_SIZE];

int32_t mqtt_connect_QoS0(char *client_id, char *user_name, char *password);
int32_t mqtt_subscribe_QoS0(char *topic);
int32_t mqtt_publish_QoS0(char *topic, char *msg);
char *mqtt_parse_msg(char *topic, uint8_t *msg, uint32_t msg_len);
int32_t mqtt_heart_beat(void);

#endif