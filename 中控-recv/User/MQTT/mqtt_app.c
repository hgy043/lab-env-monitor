/**
 * @file    mqtt_app.c
 * @brief   中控 MQTT 上云应用层（巴法云）—— 非阻塞状态机
 *
 * ============================================================================
 * 这一层干什么
 *   把 LoRa 收到的终端数据，经 ESP8266 发到巴法云，手机小程序就能看到。
 *
 *   终端板 --LoRa--> g_lora_rx_frame --MQTT--> 巴法云 --推送--> 小程序
 *
 *   数据只在这一层加工：从 g_lora_rx_frame 取值 → 拼成巴法云的井号格式
 *   → 交给 mqtt.c 组包 → 交给 esp.c 塞进 TCP 透传口。
 *
 * ----------------------------------------------------------------------------
 * 状态流（主循环里反复调 MQTT_AppService()）
 *
 *   [未连上] --每 MQTT_RETRY_PERIOD_MS 试一次--> 连WiFi+TCP → MQTT连接 → 订阅
 *       ↑                                              |
 *       |<---------------- 任何一步失败 ----------------|
 *       |                                              |
 *       `----------------- 全部成功 ---------------→ [已连上]
 *                                                      |
 *              ┌───────────────────────────────────────┤
 *              │  每 MQTT_HEART_PERIOD_MS              │  每 MQTT_PUBLISH_PERIOD_MS
 *              │  发心跳；失败 → 退回「未连上」重连     │  按 s_topics 表逐个发 /set
 *              └───────────────────────────────────────┤
 *                                                      │
 *                                       每轮收一次云端下发的消息（非阻塞）
 *
 * ----------------------------------------------------------------------------
 * ⚠ 阻塞代价（和原来 WiFi 单独跑时是同一笔账，现在多挂了 MQTT 两段）
 *   连一次：ESP_Init 最坏约 15.5s（等开机 2s + CWJAP 超时 10s …）
 *          + MQTT CONNECT 等 2s + **五个主题订阅各等 2s**（服务器应答快就是几十
 *            毫秒，只有链路不通才会耗满）→ 最坏约 27s。
 *   心跳：正常几十毫秒（收到 PINGRESP 就返回），链路断了才耗满 3s。
 *   发布：五包各 20~40 字节，115200 下发出去一共 15ms 左右，可以忽略。
 *
 *   只要它在阻塞，LoRa 那边 DIO1 中断照样置标志位、不会丢，但芯片处理完
 *   上一包后没法及时挂回接收、ACK 也会迟发。也就是「云端连不上的时候，
 *   LoRa 接收是带病工作的」—— 这是刻意取舍：宁可漏一两包，也不能让
 *   「云端连不上」把整块板子卡死。要彻底不影响接收，得把 esp.c 的 AT
 *   流程也改成一条一条推进的状态机，那是下一步的事。
 *
 * ----------------------------------------------------------------------------
 * ⚠ 和原厂 demo（mqtt_demo.c）的区别 —— 别再退回去
 *   原厂 demo 的 mqtt_demo() 函数体里是 while(1) 死循环，app_main 里只写了
 *   那一句，后面的 LoRa 接收和 OLED 上屏一行都执行不到（板子等于废了）。
 *   本文件是照主循环节奏反复调用的非阻塞状态机。
 *   mqtt_demo.c 已经从 Keil 工程里移出（文件留在原地当参考），
 *   不要把它重新加回主循环。
 * ============================================================================
 */

#include "mqtt_app.h"
#include "mqtt.h"     // mqtt_connect_QoS0 / mqtt_subscribe_QoS0 / mqtt_publish_QoS0 / mqtt_heart_beat
#include "esp.h"      // ESP_Init / ESP_Get_Receive_Data / ESP_rx_len
#include "lora_app.h" // g_lora_rx_frame / LORA_TEMP_INVALID / LORA_CO2_INVALID
#include "main.h"     // HAL_GetTick
#include <stdio.h>    // snprintf / printf
#include <string.h>   // memset

/*--------------------------------------------------------------------------
 * 收云端下发消息用的落地缓冲
 *   注意和 mqtt.c 里的 mqtt_buffer 不是一个东西：
 *   mqtt_buffer 是「要发出去的报文」的组装区，这个是「收进来的报文」的落地区。
 *------------------------------------------------------------------------*/
#define MQTT_RX_BUF_SIZE 128
static uint8_t s_rx_buf[MQTT_RX_BUF_SIZE];

/*--------------------------------------------------------------------------
 * 五个主题（顺序和巴法云控制台上的卡片一致）
 *   订阅、发布、收云端下发都照这张表遍历 —— 加/改主题只动 mqtt_app.h
 *   里的宏和这里的排列，别在下面几个函数里零散地写主题名。
 *
 *   ⚠ 发布顺序必须和 mqtt_app_publish() 里 payloads[] 的顺序一致。
 *------------------------------------------------------------------------*/
static char *const s_topics[] = {
    MQTT_TOPIC_HUMI,
    MQTT_TOPIC_TEMP,
    MQTT_TOPIC_LIGHT,
    MQTT_TOPIC_SOIL,
    MQTT_TOPIC_CO2,
};
#define TOPIC_NUM (sizeof(s_topics) / sizeof(s_topics[0]))

/*--------------------------------------------------------------------------
 * 状态与计时基准
 *   每个定时任务用自己的基准，不要共用（共用会被先跑到的那个重置，
 *   排在后面的块几乎永远不会执行 —— 这个坑本工作区踩过一次）。
 *------------------------------------------------------------------------*/
static uint8_t s_online;      // 0 = 没连上（按周期重试）  1 = 已连上
static uint32_t s_tick_retry; // 重连计时基准
static uint32_t s_tick_heart; // 心跳计时基准
static uint32_t s_tick_pub;   // 上云计时基准

/*==========================================================================
 * 内部函数
 *========================================================================*/
static int8_t mqtt_app_connect(void);
static void mqtt_app_publish(void);
static void mqtt_app_poll_downlink(void);

/*==========================================================================
 * 对外接口
 *========================================================================*/

void MQTT_AppInit(void)
{
    /* 先减掉一个周期，让主循环第一轮就立刻去连，不用白等 MQTT_RETRY_PERIOD_MS。
     * （无符号数回绕不影响差值比较：HAL_GetTick() - s_tick_retry 照样算得对） */
    s_tick_retry = HAL_GetTick() - MQTT_RETRY_PERIOD_MS;
    s_online = 0;

    printf("MQTT上云层已挂载（%s:%s，每 %u ms 上报一次）\r\n",
           TCP_SERVER, TCP_SERVER_PORT,
           (unsigned int)MQTT_PUBLISH_PERIOD_MS);
    printf("上云主题：%s / %s / %s / %s / %s\r\n",
           MQTT_TOPIC_HUMI, MQTT_TOPIC_TEMP, MQTT_TOPIC_LIGHT,
           MQTT_TOPIC_SOIL, MQTT_TOPIC_CO2);
}

void MQTT_AppService(void)
{
    /*------------------------------------------------------------------
     * 1) 没连上：按周期重试整条链路
     *      连上之前每个周期试一次；连上之后这一段就再也不进来了。
     *      ⚠ mqtt_app_connect() 是**阻塞**的，最坏二十几秒（见文件头）。
     *        这里没有写成 while(ESP_Init()){}：那样写 WiFi 一失败就
     *        永远出不来，后面的 OLED 和 LoRa 一行都不会执行。
     *----------------------------------------------------------------*/
    if (!s_online)
    {
        if (HAL_GetTick() - s_tick_retry < MQTT_RETRY_PERIOD_MS)
        {
            return; // 还没到重试点，立刻返回
        }

        if (mqtt_app_connect() == 0)
        {
            s_online = 1;
            printf("MQTT上云：连接完成，接下来每 %u ms 上报一次\r\n",
                   (unsigned int)MQTT_PUBLISH_PERIOD_MS);
        }
        else
        {
            printf("MQTT上云：连接失败，%u ms 后自动重试（LoRa 和 OLED 不受影响）\r\n",
                   (unsigned int)MQTT_RETRY_PERIOD_MS);
        }

        /* 计时基准取在**这次尝试结束之后**，不是开始之前。
         *   取在开始之前的话：ESP_Init 一次失败就要耗掉十几秒，几乎正好等于
         *   重试周期，等于「上一条前脚刚失败、下一条后脚就发」—— 主循环
         *   绝大部分时间都堵在 ESP_Init 里，LoRa 那点接收窗口被挤没了。
         *   取在结束之后，每次失败之间才有完整的 MQTT_RETRY_PERIOD_MS
         *   让主循环正常跑（收包、上屏都照常）。 */
        s_tick_retry = HAL_GetTick();

        /* 连上也好、失败也好，本轮到此为止：不在同一个主循环轮次里
         * 紧接着又去发消息 —— 发布用的 ESP_Send_data_len() 会清接收缓冲，
         * 刚连上时缓冲区里可能还留着订阅/连接的回包。 */
        return;
    }

    /*------------------------------------------------------------------
     * 2) 先把已经收到的云端下行消化掉 —— 必须排在心跳和发布前面
     *      ⚠ 「先收后发」的顺序不能反：下面心跳和发布都会调
     *        ESP_Send_data_len()，它每发一次都会 ESP_rx_len = 0 +
     *        memset(ESP_buffer)，也就是**把已经收进来的数据丢掉**。
     *      更要紧的是心跳：mqtt_heart_beat() 只认 PINGRESP 这一个回包，
     *      万一这期间正好来了条云端下发的消息，会被它当成「心跳失败」，
     *      白白触发一次二十几秒的断线重连。先收干净就避开了这个窗口。
     *----------------------------------------------------------------*/
    mqtt_app_poll_downlink();

    /*------------------------------------------------------------------
     * 3) 心跳：失败就判定掉线，退回「未连上」等下一轮重连
     *----------------------------------------------------------------*/
    if (HAL_GetTick() - s_tick_heart >= MQTT_HEART_PERIOD_MS)
    {
        s_tick_heart = HAL_GetTick();
        if (mqtt_heart_beat())
        {
            printf("MQTT上云：心跳失败，判定掉线，稍后重连\r\n");
            s_online = 0;
            s_tick_retry = HAL_GetTick();
            return;
        }
    }

    /*------------------------------------------------------------------
     * 4) 上云
     *----------------------------------------------------------------*/
    if (HAL_GetTick() - s_tick_pub >= MQTT_PUBLISH_PERIOD_MS)
    {
        s_tick_pub = HAL_GetTick();
        mqtt_app_publish();
    }
}

/*==========================================================================
 * 内部函数实现
 *========================================================================*/

/**
 * @brief 连上巴法云：WiFi+TCP → MQTT连接 → 订阅三个主题
 * @return 0 = 全部成功，-1 = 中途失败（失败点会打印出来）
 *
 * ⚠ 阻塞，最坏二十几秒（见文件头「阻塞代价」）。只在没连上时被调用。
 */
static int8_t mqtt_app_connect(void)
{
    uint32_t i;

    printf("MQTT上云：开始连接 WiFi 和巴法云...\r\n");

    /* 1) WiFi + TCP 透传。整条链路最重的活都在 ESP_Init() 里，
     *    成功返回 0 时，串口已经处于透传模式 —— 后面写的字节会直接进 TCP。 */
    if (ESP_Init())
    {
        printf("MQTT上云：WiFi/TCP 连接失败\r\n");
        return -1;
    }

    /* 2) MQTT 层的 CONNECT。客户端ID 就是巴法云私钥（放在 wifi_config.h），
     *    用户名/密码传 NULL。失败大多是私钥写错或服务器连不上。 */
    if (mqtt_connect_QoS0(MQTT_CLIENT_ID, NULL, NULL))
    {
        printf("MQTT上云：连接巴法云失败（检查 wifi_config.h 里的 MQTT_CLIENT_ID）\r\n");
        return -1;
    }

    /* 3) 订阅五个主题：手机小程序往主题里发消息时，这块板子能收到。
     *    订阅失败**不当致命错误**处理 —— 上云（发布）不受订阅影响，
     *    而且 mqtt.c 里 mqtt_subscribe_QoS0() 的返回值在原厂代码里
     *    对不上格式时也会返回 0，本来就不可全信，这里只打印不中断。 */
    for (i = 0; i < TOPIC_NUM; i++)
    {
        if (mqtt_subscribe_QoS0(s_topics[i]))
        {
            printf("MQTT上云：订阅 %s 失败\r\n", s_topics[i]);
        }
    }

    /* 4) 计时基准复位：
     *    上云基准减掉一个周期 → 下一轮主循环立刻发一次（手机端马上能看到数）
     *    心跳基准取当前时刻   → 60 秒后才发第一个心跳 */
    s_tick_pub = HAL_GetTick() - MQTT_PUBLISH_PERIOD_MS;
    s_tick_heart = HAL_GetTick();
    return 0;
}

/**
 * @brief 把最近一包终端数据发到五个主题（QoS0，发完不等回执）
 *
 * 数据来源 g_lora_rx_frame 由 LORA_AppService() 收到 LoRa 包时刷新，
 * 这里只读不算。终端那包里没读到某项时会给个哨兵值（温度 -32768、
 * CO2 0xFFFF），**不能原样发上云** —— 小程序上会显示成 -32768 ppm 这种
 * 明显不对的数，比 "--" 更难看出是故障。
 *
 * 五个主题各发什么、失效时发什么，见 mqtt_app.h 顶部那张表。
 */
static void mqtt_app_publish(void)
{
    char p_humi[16];
    char p_temp[16];
    char p_light[16];
    char p_soil[16];
    char p_co2[16];

    /* ⚠ 这个顺序必须和文件头的 s_topics[] 逐项对应：下面发的时候是
     *   一发一收配对取的（s_topics[i] ↔ payloads[i]），错位了就发串主题。 */
    char *const payloads[TOPIC_NUM] = { p_humi, p_temp, p_light, p_soil, p_co2 };

    char topic_set[32];
    uint32_t i;

    /* 一包都还没收到过（比如终端没上电）就什么都不发：
     * 发了就是 #0 / #0.0 / #0 一串假数据，不如让云端保持空。 */
    if (!g_lora_rx_frame.valid)
    {
        printf("MQTT上云：还没收到过终端的包，本轮不上报\r\n");
        return;
    }

    /* 温度 + 湿度（终端同一个 DHT22 的两个读数）：
     *   temp10 单位 0.1℃（可能是负的），除以 10 再发，保留一位小数。
     *   ⚠ 温度无效时湿度也必须一起发 "--"：终端那边 DHT22 读失败时只把温度
     *     填成 LORA_TEMP_INVALID，湿度那个字节停在初值 0，单独发上去会显示成
     *     「湿度 0%RH」，比 "--" 更像真的、更容易误导。 */
    if (g_lora_rx_frame.temp10 == LORA_TEMP_INVALID)
    {
        snprintf(p_temp, sizeof(p_temp), "#--");
        snprintf(p_humi, sizeof(p_humi), "#--");
    }
    else
    {
        snprintf(p_temp, sizeof(p_temp), "#%.1f", g_lora_rx_frame.temp10 / 10.0f);
        snprintf(p_humi, sizeof(p_humi), "#%u", (unsigned int)g_lora_rx_frame.humi);
    }

    /* light：光照强度，单位 lux。范围 0~350，int16 够用 */
    snprintf(p_light, sizeof(p_light), "#%d", (int)g_lora_rx_frame.lux);

    /* soil：土壤湿度等级 1~4（温度已经单独占 Temperature 主题了，这里不再带） */
    snprintf(p_soil, sizeof(p_soil), "#%u", (unsigned int)g_lora_rx_frame.soil);

    /* co2：单位 ppm。0xFFFF 是「没读到」的哨兵值，同样填 "--" */
    if (g_lora_rx_frame.co2 == LORA_CO2_INVALID)
    {
        snprintf(p_co2, sizeof(p_co2), "#--");
    }
    else
    {
        snprintf(p_co2, sizeof(p_co2), "#%u", (unsigned int)g_lora_rx_frame.co2);
    }

    /* 逐个发到 <主题>/set：
     *   巴法云的约定 —— 加 /set 是「更新云端数据 + 推送给订阅者」，
     *   加 /up 则是「只更新云端、不推送」。我们要给手机端看，用 /set。
     *   另外推送者不会收到自己发的消息，所以不用担心下面 poll_downlink()
     *   又把刚发出去的东西收回来。
     * QoS0 发完不等回执，所以这些调用没有返回值也没有失败分支；
     * 真发不出去会在下一次心跳时被发现（心跳失败 → 重连）。 */
    printf("MQTT上云：");
    for (i = 0; i < TOPIC_NUM; i++)
    {
        snprintf(topic_set, sizeof(topic_set), "%s/set", s_topics[i]);
        mqtt_publish_QoS0(topic_set, payloads[i]);
        printf("%s<-%s ", s_topics[i], payloads[i]);
    }
    printf("\r\n");
}

/**
 * @brief 收云端下发的消息（非阻塞）
 *
 * 手机小程序往主题里发消息，服务器会推给我们（发布者收不到自己发的）。
 * 目前只打印出来，没有联动动作。
 *
 * ⚠ 原厂 demo 这里写的是 ESP_Get_Receive_Data(..., 1000) —— 每轮死等 1 秒，
 *   塞进主循环会把 LoRa 的 ACK 拖到超时（终端只等 400ms）。所以先看
 *   中断收了多少（ESP_rx_len），没数据立刻返回，有数据才去取、且超时传 0。
 *
 * 想把「小程序按下按钮 → 控制终端」接起来，就在下面 printf 的位置
 * 按 payload 内容分发，再用 LoRa 下行帧发给终端 —— 那需要先在
 * recv 侧的 lora_app.c 里加一个「发任意下行帧」的接口（目前只有回 ACK 的）。
 */
static void mqtt_app_poll_downlink(void)
{
    int32_t len = MQTT_RX_BUF_SIZE;
    char *payload;
    uint32_t i;

    if (ESP_rx_len <= 0)
    {
        return; // 没收到任何东西，直接返回，一个字节都不等
    }

    memset(s_rx_buf, 0, sizeof(s_rx_buf));
    if (ESP_Get_Receive_Data(s_rx_buf, &len, 0) < 0)
    {
        return;
    }
    if (len < 2)
    {
        return;
    }

    /* 五个主题挨个试，主题名对不上的 mqtt_parse_msg() 会返回 NULL。
     * 注意它返回的是**缓冲区里的指针**，不是新字符串；上面 memset 过 0，
     * 负载后面必定有个 '\0'，所以 %s 打印是安全的（缓冲区 128 字节，
     * 透传的报文远小于它，不会把缓冲区读满而没有结尾）。 */
    for (i = 0; i < TOPIC_NUM; i++)
    {
        payload = mqtt_parse_msg(s_topics[i], s_rx_buf, (uint32_t)len);
        if (payload != NULL)
        {
            printf("云端下发[%s]：%s\r\n", s_topics[i], payload);
            return;
        }
    }

    /* 走到这里说明收到的不是这五个主题的发布报文。
     * 常见来源：服务器对订阅/心跳的应答、或者主题不匹配的推送。
     * 不是错误，打印长度方便排查即可。 */
    printf("MQTT上云：收到 %d 字节，但没匹配上这五个主题\r\n", (int)len);
}
