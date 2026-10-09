#ifndef __LORA_APP_H
#define __LORA_APP_H

/*
 * ============================================================================
 * lora_app —— LoRa 应用层（中控 / 接收端）
 *
 * 和本工作区的终端工程（TMPLATE - 15.1-中控和终端\TMPLATE - 14-LORA-send）
 * 是同一块 PCB、同一套射频参数，区别只在角色和三个引脚：
 *   终端：每 LORA_UPLOAD_PERIOD_MS 上报一包传感器数据，上报前后开接收窗口
 *   中控：常驻接收，收到一包就解析出来 + 回一个 ACK（本文件）
 * 引脚差异：中控板上没有光敏(PA0) / 土壤(PA1) / DHT22(PC15)，
 *   对应的 ADC1 + DMA 也一并去掉了，其余引脚完全一致。
 *
 * 硬件（LLCC68 / Ra-01SC 模块）：
 *   SPI1 : SCK=PA5  MISO=PA6  MOSI=PA7        （SPI 模式0，8bit，主机）
 *   NSS  : PA4   GPIO 输出，软件片选，空闲为高
 *   RST  : PB1   GPIO 输出，空闲为高
 *   BUSY : PB0   GPIO 输入，高=模块忙
 *   DIO1 : PB10  EXTI 上升沿 → EXTI15_10_IRQHandler → DIO1_EXTI_Callback
 *          （是上升沿：LLCC68 手册 §13.3.2.1 —— IRQ 触发时 DIO 被置位拉高，
 *            别按「按键=下降沿」的常识改回 IT_FALLING，改了就收不到中断）
 *
 *   射频参数（与 llcc68_p2p.h 里的一致）：470.5MHz / SF9 / BW125 / CR4-5
 *   空口包：前导码 8 符号，显式头，CRC 开，不反转 IQ
 *   —— 这几项必须和终端完全一致，否则收不到彼此的包
 *
 * 工作方式（非阻塞状态机，挂在 app_main 的 while(1) 里）：
 *   初始化（失败每秒重试）→ 常驻接收：
 *       收到一包 → 校验帧头 → 解析 → 更新 g_lora_rx_frame → 回一个 ACK
 *                → 重新挂回接收
 *
 *   收发动作本身不在主循环里死等：使能收发后立刻返回，靠 DIO1 中断置的
 *   标志位推进状态。这样 OLED 刷新不被打断、ACK 也能及时发出去。
 *   （ISR 里只置标志位，不调 printf 也不动 SPI —— 见 ISR_callback.c）
 *
 * ⚠ 中控的主循环里不要放会长时间阻塞的调用（比如 co2.c 那种带秒级超时的
 *   串口等待）。被阻塞期间收包标志不会丢（中断已经置上了），但芯片处理完
 *   上一包后没法及时挂回接收，后面紧跟着的包会漏掉，ACK 也会迟发。
 *
 * 用法（app_main.c）：
 *   LORA_AppInit();                       // 启动时调一次
 *   while (1) { ...
 *       LORA_AppService();                // 主循环里反复调
 *   ... }
 * ============================================================================
 */

#include <stdint.h>
#include <stdbool.h>

/************************ 时间参数 ************************/
// 初始化失败后的重试间隔（毫秒）
#define LORA_INIT_RETRY_MS 1000

// 回一包 ACK 最多等多久（毫秒）。两个用途：
//   ① 交给 llcc68_set_tx() 当芯片自己的发送超时
//   ② DIO1 没接上/没配好时的软件兜底，防止状态机永远卡在 LORA_ST_TX_ACK
//   ACK 只有 3 字节，SF9/BW125 下空中时间约 104ms，600ms 有近 6 倍余量。
//   （104ms 是用厂商驱动自己的 llcc68_get_lora_time_on_air_in_ms() 实算出来的）
//   ⚠ 不要调大：终端现在是每 2.5 秒上报一包（LORA_UPLOAD_PERIOD_MS=2500），
//     兜底时间太长的话，一次 ACK 失败就会把整个周期占满，连着漏掉后面的包。
#define LORA_TX_TIMEOUT_MS 600

/************************ ACK 开关 ************************/
// 收到终端的数据后要不要回 ACK：1=回（默认），0=只收不回
//   置 0 后中控就纯接收，终端那边的 LORA_ACK_WINDOW_MS 窗口会空等到超时
#define LORA_ACK_ENABLE 1

/************************ 帧 格 式 ************************
 * 与终端工程 TMPLATE - 14-LORA-send 的 lora_app.h 是同一份约定，
 * 改动任何一边都要同步改另一边，否则两边对不上。
 *
 * 【上行：终端 → 中控】固定 10 字节（LORA_FRAME_LEN），多字节字段一律小端
 *   下标  长度  含义          类型     说明
 *    0     1   帧头          uint8    固定 0xAA
 *    1     1   设备ID        uint8    终端填自己的 LORA_DEVICE_ID
 *    2     2   光照强度      int16    单位 lux，取值 0~350
 *    4     1   土壤湿度等级  uint8    1~4（1干燥 / 2微湿 / 3湿润 / 4饱和）
 *    5     2   温度          int16    单位 0.1℃，235 表示 23.5℃，可为负；
 *                                      LORA_TEMP_INVALID 表示对方没读到
 *    7     1   湿度          uint8    单位 1%RH，整数部分，0~100
 *    8     2   CO2 浓度      uint16   单位 ppm，LORA_CO2_INVALID 表示对方没读到
 *          —— 合计 10 字节
 *
 *   ⚠ CO2 是 2026-09-21 追加到帧尾的（原 8 字节），追加而不是插在中间，
 *     老位置的偏移全都不变。**两端必须同步改**：这是**定长帧**，
 *     下面 lora_handle_rx() 里长度对不上会直接丢弃并打印「长度异常的包」。
 *
 * 【下行：中控 → 终端】帧头 0xAA + 设备ID + 自定义负载
 *   负载第 1 字节是指令码，当前只定义了一个：
 *     LORA_CMD_ACK (0x01)：收到数据后的确认，整帧 3 字节
 *                          [0]=0xAA  [1]=终端的设备ID（原样回）  [2]=0x01
 *   以后要加控制指令（开灯、关阀之类），往这里继续加指令码，
 *   终端侧在自己的 .c 里实现 LORA_OnRecv() 按指令码分发。
 *
 * 关于设备ID：中控不校验ID，收到什么就回什么。这样以后加第2、3个终端节点
 * 不用改这里；真正的过滤靠射频层的 CRC（已开），噪声进不到应用层。
 * ========================================================== */

/************************ 上行帧下标 ************************/
// 逐字节按数组解析，不用结构体，避免编译器对齐填充带来的歧义
#define LORA_FRAME_LEN 10
#define LORA_IDX_HEAD 0   // 帧头 0xAA
#define LORA_IDX_DEVID 1  // 设备ID
#define LORA_IDX_LIGHT 2  // 光照 lux      int16 小端
#define LORA_IDX_SOIL 4   // 土壤湿度等级 uint8
#define LORA_IDX_TEMP 5   // 温度 0.1℃    int16 小端
#define LORA_IDX_HUMI 7   // 湿度 %RH     uint8
#define LORA_IDX_CO2 8    // CO2 浓度 ppm  uint16 小端

/************************ 帧头与指令码 ************************/
#define LORA_FRAME_HEAD 0xAA // 上下行统一的帧头
#define LORA_CMD_ACK 0x01    // 下行：收到数据的确认

// 温度无效标记：终端没读到 DHT22 时填这个值
#define LORA_TEMP_INVALID (-32768)

// CO2 无效标记：终端没读到 CO2 传感器时填这个值。
//   为什么不拿 0 当无效：0 ppm 是物理上不可能出现的读数（大气本底约 400ppm），
//   但用显式的哨兵值和温度的 LORA_TEMP_INVALID 一个路子，读代码时更清楚。
//   收到这个值就把屏上 CO2 那行显示成 "---"
#define LORA_CO2_INVALID 0xFFFF

/************************ 收到的数据 ************************/
/* 最近一次收到的终端数据。OLED 上屏和串口打印都取这里。
 *   valid=0 表示还没收到过有效帧，屏上显示 "--"。
 * 只在主循环里读写（LLCC68 那边只写 g_dio1_irq 标志位），所以不用 volatile */
typedef struct
{
	uint8_t valid;   // 0=还没收到过有效帧，1=下面各值可用
	uint8_t dev_id;  // 发来的节点ID（帧里的第2字节）
	int16_t lux;     // 光照强度 lux
	uint8_t soil;    // 土壤湿度等级 1~4
	int16_t temp10;  // 温度，单位 0.1℃；LORA_TEMP_INVALID 表示对方没读到
	uint8_t humi;    // 空气湿度 %RH
	uint16_t co2;    // CO2 浓度 ppm；LORA_CO2_INVALID 表示对方没读到
	int8_t rssi;     // 接收信号强度 dBm（数值越接近0越强）
	int8_t snr;      // 接收信噪比 dB（数值越大越好）
	uint32_t cnt;    // 累计收到的有效帧数
} LORA_RxFrame_t;

extern LORA_RxFrame_t g_lora_rx_frame;

/************************ 对外接口 ************************/

// 上电调用一次。内部只把状态机推到「等1秒」，
// 射频初始化放到 LORA_AppService() 里重试，模块不在线也不会卡死主循环
void LORA_AppInit(void);

// 主循环里反复调用。内部自己计时，不阻塞
void LORA_AppService(void);

/************************ 诊断计数（调试用） ************************/
extern volatile uint32_t g_lora_tx_ok_cnt;   // ACK 成功发出的次数
extern volatile uint32_t g_lora_tx_err_cnt;  // ACK 发送失败/超时次数
extern volatile uint32_t g_lora_rx_ok_cnt;   // 成功收到的有效帧数
extern volatile uint32_t g_lora_rx_err_cnt;  // 接收出错（CRC错/帧头错/长度不符）次数

#endif
