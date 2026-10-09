#ifndef __WIFI_CONFIG_H
#define __WIFI_CONFIG_H

/*==========================================================================
 * 本机私密配置 —— 模板文件
 *
 *   用法：把本文件复制一份、改名为 wifi_config.h，再把三个值填成自己的：
 *
 *     WIFI_SSID      —— 路由器热点名（ESP8266 只支持 2.4GHz）
 *     WIFI_PASSWORD  —— 热点密码
 *     MQTT_CLIENT_ID —— 巴法云账号私钥（控制台「设备云」里的 32 位十六进制串）
 *
 *   ⚠ wifi_config.h 已经在 .gitignore 里，**不要**把它提交到公开仓库。
 *
 *   ⚠ 中文 SSID 要留意：ESP8266 AT 固件对 UTF-8 的 SSID 兼容性一般，
 *     个别固件版本会退回 3 号错误（找不到热点）。
 *     如果确认热点是 2.4GHz、密码也对，却一直报 +CWJAP:3，
 *     先把热点名改成纯英文数字再试一次，能连上就是模块对中文 SSID 支持的问题。
 *========================================================================*/

#define WIFI_SSID      "your_wifi_ssid"
#define WIFI_PASSWORD  "your_wifi_password"
#define MQTT_CLIENT_ID "your_bemfa_private_key"

#endif
