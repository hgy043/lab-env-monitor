/**
 * @file lora_app.c
 * @brief LoRa 应用层（终端节点）：定时上报传感器数据 + 接收网关指令
 *
 * 帧格式、引脚、参数说明见 lora_app.h 头部注释块
 *
 * 状态流转（全部非阻塞，靠 DIO1 中断标志推进）：
 *
 *     INIT_DELAY ──延时1s──► INIT_RETRY ──成功──► IDLE
 *                              ▲   │              │ 距上次上报满
 *                              │   │失败          │ LORA_UPLOAD_PERIOD_MS
 *                              └───┘              ▼
 *                                  ┌────────► WAIT_RX（挂接收窗口等网关指令）
 *                                  │            │ RX_DONE收到 / 窗口超时
 *                                  │            ▼
 *                                  │          TX（发一包传感器数据）
 *                                  │            │ TX_DONE
 *                                  │            ▼
 *                                  └──WAIT_ACK（短窗口等中控ACK）──超时/收到
 *
 * 为什么ACK窗口要单独放在发送之后：
 *   WAIT_RX 那个窗口在发送之前，是等网关主动下发的指令用的，那时候本包还没
 *   发出去，中控不可能有ACK；中控收到包才会回ACK，所以必须发完再开一个窗口。
 *
 * 注意：DIO1 中断里只置标志位（见 ISR_callback.c），
 *       读接收FIFO、动SPI这些事全都放在主循环的 LORA_AppService() 里做。
 */

#include "lora_app.h"
#include "llcc68_p2p.h" // llcc68_ctx、DIO1_EXTI_Callback、LORA_* 射频参数
#include "llcc68.h"
#include "light.h"
#include "soil.h"
#include "dht22.h"
#include "co2.h"   // g_co2_value / g_co2_valid（CO2 缓存，见 co2.h 说明）
#include "debug.h"

#include <stdio.h>

/*----------------------------------------------------------------------------
 * DIO1 中断标志位
 *   中断里 DIO1_EXTI_Callback() 已经把芯片的中断状态读出来存进 g_dio1_irq
 *   （读了就自动清了，见驱动 llcc68_get_and_clear_irq_status 的语义），
 *   所以主循环可以直接取用，不用再碰 SPI
 *----------------------------------------------------------------------------*/
extern volatile llcc68_irq_mask_t g_dio1_irq;

/* 超时保护：状态机卡住最多等这么久就放弃，避免一次异常把主循环锁死。
 * 两个用途：
 *   ① 交给 llcc68_set_tx() 当芯片自己的发送超时
 *   ② DIO1 没接上/没配好时的软件兜底，防止状态机永远卡在 LORA_ST_WAIT_TX_DONE
 * 本包 10 字节（含 CO2），SF9/BW125 下空中时间约 145ms，600ms 有 4 倍余量。
 *   （145ms 是用厂商驱动自己的 llcc68_get_lora_time_on_air_in_ms() 实算出来的，
 *     不是估的；8 字节老帧是 124ms）
 * ⚠ 不要调大：上报周期已经是 2500ms，而每轮开头还要先花 500ms 开接收窗口，
 *   兜底时间一旦超过 2000ms，一次发送失败就会把整个周期占满，连着漏掉后面的包。 */
#define LORA_TX_TIMEOUT_MS 600
#define LORA_INIT_RETRY_MS 1000

/*----------------------------------------------------------------------------
 * 清除 g_dio1_irq 里的某一位
 *
 *   为什么不能直接写 g_dio1_irq &= ~bit：
 *   那是「读-改-写」三步，如果在主循环读到值之后、写回之前 DIO1 中断来了，
 *   中断里置的位会被这一步写回操作覆盖掉，那一次收发完成的事件就丢了。
 *   关中断的临界区只有几条指令，不会影响任何时序。
 *----------------------------------------------------------------------------*/
static void lora_irq_clear(llcc68_irq_mask_t mask)
{
	__disable_irq();
	g_dio1_irq &= ~mask;
	__enable_irq();
}

// 温度无效标记：DHT22 读失败时往帧里填这个值，网关据此判断无有效温度
#define LORA_TEMP_INVALID (-32768)

// PA（功率放大器）配置，取自原 llcc68_init()，对应 +22dBm
#define LORA_PA_DUTY_CYCLE 0x03 // 手册13.1.14.1：+20dBm 最优占空比
#define LORA_PA_HP_MAX 0x05     // 手册4.4.1：+20dBm 对应最大增益等级
#define LORA_PA_DEVICE_SEL 0x00 // 0=内置PA（用外部PA才设0x01）
#define LORA_PA_LUT 0x01        // 启用 PA 校准表

// 状态机状态
enum
{
	LORA_ST_INIT_DELAY = 0, // 首次初始化前先等 1s，等模块上电稳定
	LORA_ST_INIT_RETRY,     // 初始化失败后的重试间隔
	LORA_ST_IDLE,           // 空闲，等下一次上报时刻
	LORA_ST_WAIT_RX,        // 挂在接收窗口等网关指令
	LORA_ST_TX,             // 已发起发送，等 TX_DONE
	LORA_ST_WAIT_ACK,       // 上报发完，开短窗口等中控的ACK
};

static uint8_t lora_state = LORA_ST_INIT_DELAY;
static uint32_t lora_tick = 0;         // 当前状态的计时基准
static uint32_t lora_last_upload = 0;  // 上次发起上报的时刻
static uint8_t lora_inited = 0;        // 射频是否已初始化成功

volatile uint32_t g_lora_tx_ok_cnt = 0;  // 成功发出的包数
volatile uint32_t g_lora_tx_err_cnt = 0; // 发送失败/超时次数
volatile uint32_t g_lora_rx_ok_cnt = 0;  // 成功收到的包数
volatile uint32_t g_lora_rx_err_cnt = 0; // 接收出错（CRC错等）次数

/*----------------------------------------------------------------------------
 * LORA_OnRecv 默认空实现
 *   用 __weak：以后要处理网关指令，随便找个 .c 写个同名同参数的函数
 *   就能把它覆盖掉，不用回来改这个文件
 *----------------------------------------------------------------------------*/
__weak void LORA_OnRecv(const uint8_t *payload, uint16_t len)
{
	(void)payload;
	(void)len;
}

/* ===========================================================================
 * 射频芯片初始化（硬件复位 + 校准）
 *   原 llcc68_init() 的本体，从 llcc68_p2p.c 挪过来的，流程和参数值都没改，
 *   只把「返回 llcc68_status_t」拆成「返回成功与否」，因为 HAL 层复位返回的
 *   是 llcc68_hal_status_t，两个枚举的 OK/ERROR 值恰好对应
 * ===========================================================================*/
static int lora_init_radio(void)
{
	llcc68_status_t status;
	llcc68_pa_cfg_params_t pa_cfg = {
		.pa_duty_cycle = LORA_PA_DUTY_CYCLE,
		.hp_max = LORA_PA_HP_MAX,
		.device_sel = LORA_PA_DEVICE_SEL,
		.pa_lut = LORA_PA_LUT,
	};
	llcc68_cal_mask_t cal_mask = LLCC68_CAL_PLL | LLCC68_CAL_ADC_PULSE |
								 LLCC68_CAL_ADC_BULK_N | LLCC68_CAL_ADC_BULK_P |
								 LLCC68_CAL_IMAGE;

	// 1. 硬件复位（拉低 RST 10ms 再拉高）
	if (llcc68_hal_reset(&llcc68_ctx) != LLCC68_HAL_STATUS_OK)
	{
		printf("LLCC68 复位失败\r\n");
		return -1;
	}

	// 2. 先进 RC 待机，再切 XOSC 待机（晶振要等稳定）
	status = llcc68_set_standby(&llcc68_ctx, LLCC68_STANDBY_CFG_RC);
	if (status != LLCC68_STATUS_OK)
	{
		printf("LLCC68 设置STDBY_RC失败\r\n");
		return -1;
	}
	HAL_Delay(50);

	status = llcc68_set_standby(&llcc68_ctx, LLCC68_STANDBY_CFG_XOSC);
	if (status != LLCC68_STATUS_OK)
	{
		printf("LLCC68 设置STDBY_XOSC失败\r\n");
		return -1;
	}
	HAL_Delay(1000); // XOSC 启动需等待稳定

	// 3. 清掉复位后残留的错误标志，否则会影响后面的校准
	status = llcc68_clear_device_errors(&llcc68_ctx);
	if (status != LLCC68_STATUS_OK)
	{
		printf("LLCC68 清除设备错误失败\r\n");
		return -1;
	}

	// 4. 校准
	status = llcc68_cal(&llcc68_ctx, cal_mask);
	if (status != LLCC68_STATUS_OK)
	{
		printf("LLCC68 校准失败\r\n");
		return -1;
	}

	// 5. 中国频段（470~510MHz）的镜像抑制校准（手册9.2.1）
	status = llcc68_cal_img_in_mhz(&llcc68_ctx, 470, 510);
	if (status != LLCC68_STATUS_OK)
	{
		printf("LLCC68 镜像校准失败\r\n");
		return -1;
	}

	// 6. PA 配置（发射功率相关，改 LORA_TX_POWER_DBM 时要一起看这里）
	status = llcc68_set_pa_cfg(&llcc68_ctx, &pa_cfg);
	if (status != LLCC68_STATUS_OK)
	{
		printf("LLCC68 配置PA失败\r\n");
		return -1;
	}

	// 7. 过流保护（手册5.1，避免PA损坏）
	status = llcc68_set_ocp_value(&llcc68_ctx, LLCC68_OCP_PARAM_VALUE_140_MA);
	if (status != LLCC68_STATUS_OK)
	{
		printf("LLCC68 配置OCP失败\r\n");
		return -1;
	}

	// 8. 供电模式：用 LDO（外接DCDC时改 LLCC68_REG_MODE_DCDC）
	//    原代码注释写的是「设置为DCDC」，但实际传的是 LDO，这里保持一致不动
	status = llcc68_set_reg_mode(&llcc68_ctx, LLCC68_REG_MODE_LDO);
	if (status != LLCC68_STATUS_OK)
		return -1;

	// 9. 兼容 500kHz 带宽的调制质量优化（当前用 125kHz）
	status = llcc68_tx_modulation_workaround(&llcc68_ctx, LLCC68_PKT_TYPE_LORA, LORA_BW);
	if (status != LLCC68_STATUS_OK)
	{
		printf("LLCC68 调制优化失败\r\n");
		return -1;
	}

	// 10. 增强PA抗天线失配能力
	status = llcc68_cfg_tx_clamp(&llcc68_ctx);
	if (status != LLCC68_STATUS_OK)
	{
		printf("LLCC68 配置TX钳位失败\r\n");
		return -1;
	}

	// 11. 缓冲区基地址（TX:0, RX:0）
	status = llcc68_set_buffer_base_address(&llcc68_ctx, 0, 0);
	if (status != LLCC68_STATUS_OK)
		return -1;

	// 12. 清掉初始化过程中可能产生的所有中断标志
	llcc68_clear_irq_status(&llcc68_ctx, LLCC68_IRQ_ALL);

	printf("LLCC68 初始化成功，射频=%.1f MHz\r\n", (float)LORA_FREQ / 1000000.0f);
	return 0;
}

/* ===========================================================================
 * 射频参数配置：初始化成功后配一次
 *   每次收发只改「数据包长度」这类随包变化的参数，其余保持不动
 * ===========================================================================*/
static llcc68_status_t lora_config(void)
{
	llcc68_status_t status;
	llcc68_mod_params_lora_t mod = {
		.sf = LORA_SF,
		.bw = LORA_BW,
		.cr = LORA_CR,
		.ldro = 0, // 低数据率优化：SF9+125kHz 用不着
	};

	// 包类型必须先设，后面的调制/数据包参数都依赖它
	status = llcc68_set_pkt_type(&llcc68_ctx, LLCC68_PKT_TYPE_LORA);
	if (status != LLCC68_STATUS_OK)
		return status;

	status = llcc68_set_rf_freq(&llcc68_ctx, LORA_FREQ);
	if (status != LLCC68_STATUS_OK)
		return status;

	status = llcc68_set_lora_mod_params(&llcc68_ctx, &mod);
	if (status != LLCC68_STATUS_OK)
		return status;

	status = llcc68_set_tx_params(&llcc68_ctx, LORA_TX_POWER_DBM, LLCC68_RAMP_200_US);
	if (status != LLCC68_STATUS_OK)
		return status;

	// 接收增益提升，配合接收窗口用；这一步失败不致命，不打断初始化
	llcc68_cfg_rx_boosted(&llcc68_ctx, true);

	// DIO1 上报 TX_DONE / RX_DONE / 接收超时 / CRC错，状态机靠这几位推进
	return llcc68_set_dio_irq_params(&llcc68_ctx,
									 LLCC68_IRQ_TX_DONE | LLCC68_IRQ_RX_DONE |
										 LLCC68_IRQ_TIMEOUT | LLCC68_IRQ_CRC_ERROR,
									 LLCC68_IRQ_TX_DONE | LLCC68_IRQ_RX_DONE |
										 LLCC68_IRQ_TIMEOUT | LLCC68_IRQ_CRC_ERROR,
									 LLCC68_IRQ_NONE,  // DIO2 不映射
									 LLCC68_IRQ_NONE); // DIO3 不映射
}

/* ===========================================================================
 * 组装上行数据帧
 *   光照/土壤直接取全局ADC数组（app_main 里 DMA 一直在刷），
 *   温湿度现读一次 DHT22（约5ms，可以接受）
 * ===========================================================================*/
static void lora_build_frame(uint8_t *frame, uint16_t *len)
{
	int16_t light = (int16_t)GetLux();
	uint8_t soil = GetSoilHumidity();
	int16_t temp10 = LORA_TEMP_INVALID;
	uint8_t humi = 0;
	float temp_f = 0.0f, humi_f = 0.0f;

	/* CO2 取主循环缓存的值，**不在这里调 CO2_get_data()**：
	 *   ① 那是死等循环（最长 100ms），塞进状态机里会把收包节奏拖垮；
	 *   ② 它和主循环的 CO2 采集是同一个串口帧，两边都调会互相抢，
	 *      抢输的那个必然超时，还会误触发蜂鸣器报警。
	 *   缓存由 co2.c 的 CO2_get_data() 每次读完后刷新，见 co2.h 的说明。 */
	uint16_t co2 = g_co2_valid ? g_co2_value : LORA_CO2_INVALID;

	if (DHT22_ReadData(&temp_f, &humi_f) == 0)
	{
		temp10 = (int16_t)(temp_f * 10.0f); // 23.5℃ → 235
		humi = (uint8_t)humi_f;             // 65.4%RH → 65
	}

#if LORA_USE_LEGACY_4B_FRAME
	// 老的 4 字节帧：光照2 + 土壤2，无帧头无校验
	//   只为了兼容已经按老格式写好的网关端，新网关请用下面的 10 字节帧
	//   （老帧里没有 CO2，这是它和当前帧格式的已知差异）
	frame[0] = (uint8_t)(light & 0xFF);
	frame[1] = (uint8_t)((light >> 8) & 0xFF);
	frame[2] = soil;
	frame[3] = 0;
	*len = 4;
#else
	frame[LORA_TX_IDX_HEAD] = LORA_FRAME_HEAD;
	frame[LORA_TX_IDX_DEVID] = LORA_DEVICE_ID;
	frame[LORA_TX_IDX_LIGHT] = (uint8_t)(light & 0xFF);
	frame[LORA_TX_IDX_LIGHT + 1] = (uint8_t)((light >> 8) & 0xFF);
	frame[LORA_TX_IDX_SOIL] = soil;
	frame[LORA_TX_IDX_TEMP] = (uint8_t)(temp10 & 0xFF);
	frame[LORA_TX_IDX_TEMP + 1] = (uint8_t)((temp10 >> 8) & 0xFF);
	frame[LORA_TX_IDX_HUMI] = humi;
	frame[LORA_TX_IDX_CO2] = (uint8_t)(co2 & 0xFF);
	frame[LORA_TX_IDX_CO2 + 1] = (uint8_t)((co2 >> 8) & 0xFF);
	*len = LORA_TX_FRAME_LEN;
#endif

	printf("LoRa上报: 光照=%d lux, 土壤=%d, 温度=%d.%d度, 湿度=%d%%",
		   light, soil, temp10 / 10, (temp10 % 10 + 10) % 10, humi);
	if (g_co2_valid)
		printf(", CO2=%u ppm\r\n", (unsigned int)co2);
	else
		printf(", CO2=--\r\n");
}

/* ===========================================================================
 * 挂一个接收窗口
 *   timeout_ms > 0 : 限时窗口，到点芯片自己超时（DIO1 出 TIMEOUT 位）
 *   timeout_ms = 0 : 连续接收，一直挂着不超时
 *
 * 每次都要重设「数据包参数」：发送时把 pld_len_in_bytes 改成了本帧的长度，
 * 不设回最大长度的话，后面收长一点的包会被当成长度不符丢掉
 * ===========================================================================*/
static llcc68_status_t lora_enter_rx(uint32_t timeout_ms)
{
	llcc68_status_t status;
	llcc68_pkt_params_lora_t pkt = {
		.preamble_len_in_symb = LORA_PREAMBLE_LEN,
		.header_type = LLCC68_LORA_PKT_EXPLICIT,
		.pld_len_in_bytes = LORA_PAYLOAD_LEN, // 接收用最大长度
		.crc_is_on = true,
		.invert_iq_is_on = false,
	};

	status = llcc68_set_pkt_type(&llcc68_ctx, LLCC68_PKT_TYPE_LORA);
	if (status != LLCC68_STATUS_OK)
		return status;

	status = llcc68_set_lora_pkt_params(&llcc68_ctx, &pkt);
	if (status != LLCC68_STATUS_OK)
		return status;

	if (timeout_ms == 0)
		return llcc68_set_rx_with_timeout_in_rtc_step(&llcc68_ctx, LLCC68_RX_CONTINUOUS);

	return llcc68_set_rx(&llcc68_ctx, timeout_ms);
}

/* ===========================================================================
 * 发起一次发送（非阻塞）
 *   配好数据包长度、把帧写进FIFO、置TX模式就返回；
 *   真正的「发完了」由 DIO1 的 TX_DONE 中断告诉状态机
 * ===========================================================================*/
static llcc68_status_t lora_start_tx(void)
{
	uint8_t frame[LORA_TX_FRAME_LEN];
	uint16_t len = 0;
	llcc68_status_t status;
	llcc68_pkt_params_lora_t pkt = {
		.preamble_len_in_symb = LORA_PREAMBLE_LEN,
		.header_type = LLCC68_LORA_PKT_EXPLICIT,
		.pld_len_in_bytes = LORA_TX_FRAME_LEN,
		.crc_is_on = true,
		.invert_iq_is_on = false,
	};

	lora_build_frame(frame, &len);

	// 数据包长度随帧长度变，每次发送前都要重设
	pkt.pld_len_in_bytes = len;
	status = llcc68_set_lora_pkt_params(&llcc68_ctx, &pkt);
	if (status != LLCC68_STATUS_OK)
		return status;

	status = llcc68_write_buffer(&llcc68_ctx, 0, frame, len);
	if (status != LLCC68_STATUS_OK)
		return status;

	status = llcc68_set_tx(&llcc68_ctx, LORA_TX_TIMEOUT_MS);
	return status;
}

/* ===========================================================================
 * 处理收到的包：校验帧头/设备ID，剥掉之后交给 LORA_OnRecv
 * ===========================================================================*/
static void lora_handle_rx(void)
{
	llcc68_rx_buffer_status_t buf_status;
	uint8_t tmp[LORA_PAYLOAD_LEN];

	// 读之前先把长度取出来
	if (llcc68_get_rx_buffer_status(&llcc68_ctx, &buf_status) != LLCC68_STATUS_OK)
	{
		g_lora_rx_err_cnt++;
		return;
	}
	if (buf_status.pld_len_in_bytes > LORA_PAYLOAD_LEN)
	{
		g_lora_rx_err_cnt++;
		return;
	}

	// 把数据从FIFO读出来（读的时候顺手把包状态也取一下，仅用于判断有效性）
	if (llcc68_read_buffer(&llcc68_ctx, buf_status.buffer_start_pointer, tmp,
						   buf_status.pld_len_in_bytes) != LLCC68_STATUS_OK)
	{
		g_lora_rx_err_cnt++;
		return;
	}

	// 通知驱动做收包后的收尾（清标志、准备下一次接收）
	llcc68_handle_rx_done(&llcc68_ctx);

	g_lora_rx_ok_cnt++;

	// 至少要有 帧头 + 设备ID
	if (buf_status.pld_len_in_bytes < 2 || tmp[0] != LORA_FRAME_HEAD ||
		tmp[1] != LORA_DEVICE_ID)
	{
		printf("LoRa收到无效帧（帧头/设备ID不对），已丢弃\r\n");
		return;
	}

	// 下行可能是中控的ACK，也可能是网关下发的控制指令，都交给 LORA_OnRecv()
	//   这里只把最常见的ACK认出来打印一下，方便直接在串口上确认上报到没到对面
	if (buf_status.pld_len_in_bytes == 3 && tmp[2] == LORA_CMD_ACK)
	{
		printf("LoRa收到中控ACK\r\n");
	}
	else
	{
		printf("LoRa收到下行数据, 负载长度=%d\r\n", buf_status.pld_len_in_bytes - 2);
	}
	LORA_OnRecv(&tmp[2], buf_status.pld_len_in_bytes - 2);
}

/* ===========================================================================
 * 对外接口
 * ===========================================================================*/
void LORA_AppInit(void)
{
	// 射频初始化放到 Service 里重试，这里只把状态机推到「等1s」，
	// 保证即使模块不在线，主循环也不会被卡住
	lora_state = LORA_ST_INIT_DELAY;
	lora_tick = HAL_GetTick();
	lora_last_upload = HAL_GetTick();
	printf("LoRa应用层已挂载，等待模块初始化...\r\n");
}

void LORA_AppService(void)
{
	uint32_t now = HAL_GetTick();

	switch (lora_state)
	{
	/*--- 首次初始化的稳定延时 ---*/
	case LORA_ST_INIT_DELAY:
		if (now - lora_tick >= LORA_INIT_RETRY_MS)
		{
			lora_state = LORA_ST_INIT_RETRY;
		}
		break;

	/*--- 射频初始化（失败就每秒重试一次，不卡主循环）---*/
	case LORA_ST_INIT_RETRY:
		if (now - lora_tick >= LORA_INIT_RETRY_MS)
		{
			lora_tick = now;
			if (lora_init_radio() == 0 && lora_config() == LLCC68_STATUS_OK)
			{
				lora_inited = 1;
				lora_last_upload = now;
				lora_state = LORA_ST_IDLE;
				printf("LoRa模块初始化成功\r\n");
			}
			else
			{
				printf("LoRa模块初始化失败，1秒后重试\r\n");
			}
		}
		break;

	/*--- 空闲：等上报时刻到 ---*/
	case LORA_ST_IDLE:
		if (!lora_inited)
		{
			// 兜底：只有初始化成功过才允许进入上报流程
			lora_state = LORA_ST_INIT_RETRY;
			break;
		}
		if (now - lora_last_upload >= LORA_UPLOAD_PERIOD_MS)
		{
			lora_last_upload = now;
			lora_tick = now;

			// 先挂接收窗口，等网关下发指令
			//   LORA_RX_WINDOW_MS 填 0 表示一直挂着接收（那样就不会再走上报流程了）
			if (lora_enter_rx(LORA_RX_WINDOW_MS) != LLCC68_STATUS_OK)
			{
				lora_state = LORA_ST_IDLE;
				break;
			}
			lora_state = LORA_ST_WAIT_RX;
		}
		break;

	/*--- 接收窗口：收到就处理，超时就转发送 ---*/
	case LORA_ST_WAIT_RX:
		// 注意：这里一次只处理一个标志位，处理完只清这一位。
		//   不能一次 g_dio1_irq = 0 全清掉 —— 一次中断里可能同时置了
		//   RX_DONE 和 TIMEOUT 两位（收完包紧接着窗口超时），全清就会把
		//   收到的那包丢掉。留着的位下一轮 LORA_AppService() 还会再进来处理。
		if (g_dio1_irq & LLCC68_IRQ_RX_DONE)
		{
			lora_irq_clear(LLCC68_IRQ_RX_DONE);
			lora_handle_rx();
			lora_state = LORA_ST_TX; // 收到指令后接着上报一次
			lora_tick = now;
			if (lora_start_tx() != LLCC68_STATUS_OK)
			{
				printf("LoRa发送启动失败\r\n");
				g_lora_tx_err_cnt++;
				lora_state = LORA_ST_IDLE;
			}
		}
		else if (g_dio1_irq & LLCC68_IRQ_CRC_ERROR)
		{
			// 收到包了但校验错：驱动不会给 RX_DONE，只会给 CRC_ERROR
			lora_irq_clear(LLCC68_IRQ_CRC_ERROR);
			g_lora_rx_err_cnt++;
			printf("LoRa接收CRC错误\r\n");
		}
		else if (g_dio1_irq & LLCC68_IRQ_TIMEOUT)
		{
			// 窗口内没等到指令，转入本次上报
			lora_irq_clear(LLCC68_IRQ_TIMEOUT);
			lora_state = LORA_ST_TX;
			lora_tick = now;
			if (lora_start_tx() != LLCC68_STATUS_OK)
			{
				printf("LoRa发送启动失败\r\n");
				g_lora_tx_err_cnt++;
				lora_state = LORA_ST_IDLE;
			}
		}
		break;

	/*--- 等发送完成 ---*/
	case LORA_ST_TX:
		if (g_dio1_irq & LLCC68_IRQ_TX_DONE)
		{
			lora_irq_clear(LLCC68_IRQ_TX_DONE);
			g_lora_tx_ok_cnt++;
			printf("LoRa上报完成\r\n");

#if LORA_ACK_WINDOW_MS > 0
			// 发完立刻开一个短窗口等中控的ACK。
			//   中控一收到这包就马上回，所以ACK紧跟在本包结尾之后到达。
			//   不能复用上报前那个窗口——那时候本包还没发出去
			if (lora_enter_rx(LORA_ACK_WINDOW_MS) == LLCC68_STATUS_OK)
			{
				lora_tick = now;
				lora_state = LORA_ST_WAIT_ACK;
				break;
			}
			printf("LoRa开ACK接收窗口失败\r\n");
#endif
			llcc68_set_standby(&llcc68_ctx, LLCC68_STANDBY_CFG_RC);
			lora_state = LORA_ST_IDLE;
		}
		else if (g_dio1_irq & LLCC68_IRQ_TIMEOUT)
		{
			lora_irq_clear(LLCC68_IRQ_TIMEOUT);
			g_lora_tx_err_cnt++;
			printf("LoRa发送超时\r\n");
			llcc68_set_standby(&llcc68_ctx, LLCC68_STANDBY_CFG_RC);
			lora_state = LORA_ST_IDLE;
		}
		else if (now - lora_tick >= LORA_TX_TIMEOUT_MS)
		{
			// 中断没来（DIO1没接上/没配好），靠软件超时兜底，
			// 保证主循环不会永远停在这个状态
			g_lora_tx_err_cnt++;
			printf("LoRa发送超时（软件兜底，检查DIO1是否接到PB10）\r\n");
			llcc68_set_standby(&llcc68_ctx, LLCC68_STANDBY_CFG_RC);
			lora_state = LORA_ST_IDLE;
		}
		break;

	/*--- 上报后的短窗口，等中控的ACK ---*/
	case LORA_ST_WAIT_ACK:
		// 同 WAIT_RX：一次只消费一个标志位，清一位处理一位
		if (g_dio1_irq & LLCC68_IRQ_RX_DONE)
		{
			lora_irq_clear(LLCC68_IRQ_RX_DONE);
			// 中控的ACK走的也是「帧头+设备ID+负载」这套下行帧，处理方式一样
			lora_handle_rx();
			lora_state = LORA_ST_IDLE;
		}
		else if (g_dio1_irq & LLCC68_IRQ_CRC_ERROR)
		{
			lora_irq_clear(LLCC68_IRQ_CRC_ERROR);
			g_lora_rx_err_cnt++;
			printf("LoRa接收CRC错误\r\n");
		}
		else if (g_dio1_irq & LLCC68_IRQ_TIMEOUT)
		{
			lora_irq_clear(LLCC68_IRQ_TIMEOUT);
			// 窗口内没等到ACK。不重发也不报错刷屏：中控可能压根没开机，
			//   要确认收没收到，看中控那边的「中控收到: ...」打印就行
			printf("本轮未收到中控ACK\r\n");
			lora_state = LORA_ST_IDLE;
		}
		break;

	default:
		lora_state = LORA_ST_IDLE;
		break;
	}
}
