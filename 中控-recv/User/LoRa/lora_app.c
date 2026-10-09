/**
 * @file lora_app.c
 * @brief LoRa 应用层（中控 / 接收端）：常驻接收终端上报的传感器数据，收到后回 ACK
 *
 * 帧格式、引脚、参数说明见 lora_app.h 头部注释块
 *
 * 状态流转（全部非阻塞，靠 DIO1 中断标志推进）：
 *
 *     INIT_DELAY ──延时1s──► INIT_RETRY ──成功──► RX_LISTEN（常驻接收）
 *                              ▲   │                │ RX_DONE
 *                              │   │失败            ▼
 *                              └───┘          解析并更新 g_lora_rx_frame
 *                                                   │
 *                                                   ▼
 *                                              TX_ACK（回一个 ACK）
 *                                                   │ TX_DONE / 超时
 *                                                   ▼
 *                                              RX_LISTEN
 *
 * 为什么接收用「连续模式」而不是限时窗口：
 *   中控不知道终端什么时候上报，挂限时窗口就得不停重挂，中间的空档会漏包。
 *   连续模式下芯片收完一包会自己留在接收态（见 llcc68.h 里 LLCC68_RX_CONTINUOUS
 *   的说明），所以收到后不用重挂；只有发过 ACK（模式被切到TX）之后才需要重挂。
 *
 * 注意：DIO1 中断里只置标志位（见 ISR_callback.c），
 *       读接收FIFO、动SPI这些事全都放在主循环的 LORA_AppService() 里做。
 */

#include "lora_app.h"
#include "llcc68_p2p.h" // llcc68_ctx、DIO1_EXTI_Callback、LORA_* 射频参数
#include "llcc68.h"
#include "debug.h"

#include <stdio.h>

/*----------------------------------------------------------------------------
 * DIO1 中断标志位
 *   中断里 DIO1_EXTI_Callback() 已经把芯片的中断状态读出来存进 g_dio1_irq
 *   （读了就自动清了，见驱动 llcc68_get_and_clear_irq_status 的语义），
 *   所以主循环可以直接取用，不用再碰 SPI
 *----------------------------------------------------------------------------*/
extern volatile llcc68_irq_mask_t g_dio1_irq;

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

// PA（功率放大器）配置，取自厂商 llcc68_init()，对应 +22dBm
#define LORA_PA_DUTY_CYCLE 0x03 // 手册13.1.14.1：+20dBm 最优占空比
#define LORA_PA_HP_MAX 0x05     // 手册4.4.1：+20dBm 对应最大增益等级
#define LORA_PA_DEVICE_SEL 0x00 // 0=内置PA（用外部PA才设0x01）
#define LORA_PA_LUT 0x01        // 启用 PA 校准表

// 状态机状态
enum
{
	LORA_ST_INIT_DELAY = 0, // 首次初始化前先等 1s，等模块上电稳定
	LORA_ST_INIT_RETRY,     // 初始化失败后的重试间隔
	LORA_ST_RX,             // 常驻接收
	LORA_ST_TX_ACK,         // 已发起回ACK，等 TX_DONE
};

static uint8_t lora_state = LORA_ST_INIT_DELAY;
static uint32_t lora_tick = 0;   // 当前状态的计时基准
static uint8_t lora_inited = 0;  // 射频是否已初始化成功
static uint8_t lora_node_id = 0; // 最近一次收到的是哪个节点（ACK 要回给谁）

volatile uint32_t g_lora_tx_ok_cnt = 0;  // ACK 成功发出的次数
volatile uint32_t g_lora_tx_err_cnt = 0; // ACK 发送失败/超时次数
volatile uint32_t g_lora_rx_ok_cnt = 0;  // 成功收到的有效帧数
volatile uint32_t g_lora_rx_err_cnt = 0; // 接收出错（CRC错/帧头错/长度不符）次数

LORA_RxFrame_t g_lora_rx_frame = {0}; // 最近一次收到的终端数据，OLED 取这里

/* ===========================================================================
 * 射频芯片初始化（硬件复位 + 校准）
 *   参数和终端版完全一致（两个板子必须配成一样才收得到）
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

	// 6. PA 配置（回ACK的发射功率，改 LORA_TX_POWER_DBM 时要一起看这里）
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
 *   频率/调制参数每次收发都保持不变，所以只配这一遍；
 *   随包变化的数据包长度在各自的收发函数里设
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

	// 接收增益提升，常驻接收时灵敏度更好；这一步失败不致命
	llcc68_cfg_rx_boosted(&llcc68_ctx, true);

	// DIO1 上报 TX_DONE / RX_DONE / 超时 / CRC错，状态机靠这几位推进
	return llcc68_set_dio_irq_params(&llcc68_ctx,
									 LLCC68_IRQ_TX_DONE | LLCC68_IRQ_RX_DONE |
										 LLCC68_IRQ_TIMEOUT | LLCC68_IRQ_CRC_ERROR,
									 LLCC68_IRQ_TX_DONE | LLCC68_IRQ_RX_DONE |
										 LLCC68_IRQ_TIMEOUT | LLCC68_IRQ_CRC_ERROR,
									 LLCC68_IRQ_NONE,  // DIO2 不映射
									 LLCC68_IRQ_NONE); // DIO3 不映射
}

/* ===========================================================================
 * 挂回接收（连续模式）
 *   命令顺序照抄厂商示例 llcc68_lora_receive_mode()：包类型 → 包参数 → FS → RX
 * ===========================================================================*/
static llcc68_status_t lora_enter_rx(void)
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

	status = llcc68_set_fs(&llcc68_ctx);
	if (status != LLCC68_STATUS_OK)
		return status;

	// LLCC68_RX_CONTINUOUS：芯片收完一包会自己留在接收态，不用每包重挂
	return llcc68_set_rx_with_timeout_in_rtc_step(&llcc68_ctx, LLCC68_RX_CONTINUOUS);
}

/* ===========================================================================
 * 解析收到的包
 *   校验帧头 → 按帧格式拆出各字段 → 更新 g_lora_rx_frame → 打印
 *   收到的节点ID存进 lora_node_id，回ACK时按原样回过去
 *
 * 返回值：1=本包有效、已解析；0=本包被丢弃（长度不符/帧头不对/SPI读失败）
 *   调用方靠这个返回值决定要不要回ACK —— 不能拿 g_lora_rx_frame.valid 代替，
 *   那是上一包的标志，本包被丢时它还是1，会给一个已经扔掉的包回ACK
 * ===========================================================================*/
static int lora_handle_rx(void)
{
	llcc68_rx_buffer_status_t buf_status;
	llcc68_pkt_status_lora_t pkt_status;
	uint8_t tmp[LORA_PAYLOAD_LEN];
	int16_t lux;
	int16_t temp10;
	uint16_t co2;

	// 读之前先把长度取出来
	if (llcc68_get_rx_buffer_status(&llcc68_ctx, &buf_status) != LLCC68_STATUS_OK)
	{
		g_lora_rx_err_cnt++;
		return 0;
	}
	if (buf_status.pld_len_in_bytes > LORA_PAYLOAD_LEN)
	{
		g_lora_rx_err_cnt++;
		return 0;
	}

	// 把数据从FIFO读出来
	if (llcc68_read_buffer(&llcc68_ctx, buf_status.buffer_start_pointer, tmp,
						   buf_status.pld_len_in_bytes) != LLCC68_STATUS_OK)
	{
		g_lora_rx_err_cnt++;
		return 0;
	}

	// 取信号质量（仅用于显示），读失败不影响解包
	if (llcc68_get_lora_pkt_status(&llcc68_ctx, &pkt_status) != LLCC68_STATUS_OK)
	{
		pkt_status.rssi_pkt_in_dbm = 0;
		pkt_status.snr_pkt_in_db = 0;
	}

	// 通知驱动做收包后的收尾（清标志、关掉那个没用上的RTC计时器）
	//   连续接收模式下这一步不会把芯片踢出接收态，见 llcc68_stop_rtc()
	llcc68_handle_rx_done(&llcc68_ctx);

	// 长度不符：不是本工程约定的帧，直接丢
	if (buf_status.pld_len_in_bytes != LORA_FRAME_LEN)
	{
		g_lora_rx_err_cnt++;
		printf("LoRa收到长度异常的包（%d字节，应为%d），已丢弃\r\n",
			   buf_status.pld_len_in_bytes, LORA_FRAME_LEN);
		return 0;
	}

	// 帧头不对：丢掉
	if (tmp[LORA_IDX_HEAD] != LORA_FRAME_HEAD)
	{
		g_lora_rx_err_cnt++;
		printf("LoRa收到无效帧（帧头不是0x%02X），已丢弃\r\n", LORA_FRAME_HEAD);
		return 0;
	}

	// 按小端把两字节字段拼回来
	lux = (int16_t)((uint16_t)tmp[LORA_IDX_LIGHT] |
					((uint16_t)tmp[LORA_IDX_LIGHT + 1] << 8));
	temp10 = (int16_t)((uint16_t)tmp[LORA_IDX_TEMP] |
					   ((uint16_t)tmp[LORA_IDX_TEMP + 1] << 8));
	co2 = (uint16_t)((uint16_t)tmp[LORA_IDX_CO2] |
					 ((uint16_t)tmp[LORA_IDX_CO2 + 1] << 8));

	g_lora_rx_frame.valid = 1;
	g_lora_rx_frame.dev_id = tmp[LORA_IDX_DEVID];
	g_lora_rx_frame.lux = lux;
	g_lora_rx_frame.soil = tmp[LORA_IDX_SOIL];
	g_lora_rx_frame.temp10 = temp10;
	g_lora_rx_frame.humi = tmp[LORA_IDX_HUMI];
	g_lora_rx_frame.co2 = co2;
	g_lora_rx_frame.rssi = pkt_status.rssi_pkt_in_dbm;
	g_lora_rx_frame.snr = pkt_status.snr_pkt_in_db;
	g_lora_rx_frame.cnt++;

	lora_node_id = tmp[LORA_IDX_DEVID];
	g_lora_rx_ok_cnt++;

	/* 打印：两处无效值各自换成占位符，免得印出 -3276.8 这种数
	 *   温度：对方 DHT22 没读到
	 *   CO2 ：对方 CO2 传感器没读到
	 * 预格式化成字符串再拼一句 printf，比按 temp/co2 是否有效分四种情况写四句清爽
	 *
	 * ⚠ 缓冲区尺寸要按**字节**算，中文一个字 3 字节（UTF-8），不是 1 个字符：
	 *     "CO2=--(对方未读到)"           = 7 + 5×3 + 1        = 23 字节（+1 收尾 = 24）
	 *     "温度=--(对方未读到), 湿度=--" = 6+4+5×3+1+2+6+3    = 37 字节（+1 收尾 = 38）
	 *   这里踩过一个坑（2026-09-21）：co2s 原来只开了 16 字节，
	 *   **刚好够放有效值那条分支**（"CO2=65535 ppm" 只要 14 字节），
	 *   于是「对方未读到」这条一直没被走到、也就一直没暴露；
	 *   等它第一次被走到，sprintf 多写 8 字节、直接踩坏栈 —— 现象是中控
	 *   打印完这一句就再无任何输出（HardFault 倒在 stm32f1xx_it.c 里
	 *   `HardFault_Handler` 那个 while(1) 上），终端那边从此永远收不到 ACK。
	 *   教训：**只有某条分支才会走到的越界，能在代码里潜伏很久**。
	 *   现在改用 snprintf 按 sizeof 截断，将来再改这两句文案也不会踩坏内存。 */
	{
		char th[48];
		char co2s[32];

		if (temp10 == LORA_TEMP_INVALID)
			snprintf(th, sizeof(th), "温度=--(对方未读到), 湿度=--");
		else
			snprintf(th, sizeof(th), "温度=%d.%d度, 湿度=%d%%",
					 temp10 / 10, (temp10 % 10 + 10) % 10, tmp[LORA_IDX_HUMI]);

		if (co2 == LORA_CO2_INVALID)
			snprintf(co2s, sizeof(co2s), "CO2=--(对方未读到)");
		else
			snprintf(co2s, sizeof(co2s), "CO2=%u ppm", (unsigned int)co2);

		printf("中控收到: 节点=%d, 光照=%d lux, 土壤=%d, %s, %s, RSSI=%d dBm, SNR=%d dB\r\n",
			   tmp[LORA_IDX_DEVID], lux, tmp[LORA_IDX_SOIL], th, co2s,
			   pkt_status.rssi_pkt_in_dbm, pkt_status.snr_pkt_in_db);
	}

	return 1;
}

/* ===========================================================================
 * 发起回ACK（非阻塞）
 *   帧 = 帧头 + 收到的那个设备ID + 指令码ACK，一共3字节；
 *   写进FIFO置TX模式就返回，真正发完由 DIO1 的 TX_DONE 告诉状态机
 * ===========================================================================*/
static llcc68_status_t lora_start_ack(uint8_t node_id)
{
	uint8_t frame[3];
	llcc68_status_t status;
	llcc68_pkt_params_lora_t pkt = {
		.preamble_len_in_symb = LORA_PREAMBLE_LEN,
		.header_type = LLCC68_LORA_PKT_EXPLICIT,
		.pld_len_in_bytes = sizeof(frame),
		.crc_is_on = true,
		.invert_iq_is_on = false,
	};

	frame[0] = LORA_FRAME_HEAD;
	frame[1] = node_id;    // 原样回给发来的那个节点
	frame[2] = LORA_CMD_ACK;

	// 命令顺序照抄厂商示例 llcc68_lora_send()
	status = llcc68_set_fs(&llcc68_ctx);
	if (status != LLCC68_STATUS_OK)
		return status;

	status = llcc68_set_pkt_type(&llcc68_ctx, LLCC68_PKT_TYPE_LORA);
	if (status != LLCC68_STATUS_OK)
		return status;

	status = llcc68_set_lora_pkt_params(&llcc68_ctx, &pkt);
	if (status != LLCC68_STATUS_OK)
		return status;

	status = llcc68_write_buffer(&llcc68_ctx, 0, frame, sizeof(frame));
	if (status != LLCC68_STATUS_OK)
		return status;

	return llcc68_set_tx(&llcc68_ctx, LORA_TX_TIMEOUT_MS);
}

/* ===========================================================================
 * 对外接口
 * ===========================================================================*/
void LORA_AppInit(void)
{
	lora_state = LORA_ST_INIT_DELAY;
	lora_tick = HAL_GetTick();
	printf("LoRa中控应用层已挂载，等待模块初始化...\r\n");
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
			if (lora_init_radio() == 0 && lora_config() == LLCC68_STATUS_OK &&
				lora_enter_rx() == LLCC68_STATUS_OK)
			{
				lora_inited = 1;
				lora_state = LORA_ST_RX;
				printf("LoRa中控初始化成功，已进入常驻接收\r\n");
			}
			else
			{
				printf("LoRa中控初始化失败，1秒后重试\r\n");
			}
		}
		break;

	/*--- 常驻接收：来包就解析+回ACK ---*/
	case LORA_ST_RX:
		if (!lora_inited)
		{
			// 兜底：只有初始化成功过才允许停在接收态
			lora_state = LORA_ST_INIT_RETRY;
			break;
		}

		// 注意：这里一次只处理一个标志位，处理完只清这一位。
		//   不能一次 g_dio1_irq = 0 全清掉 —— 一次中断里可能同时置了
		//   RX_DONE 和 CRC_ERROR 两位，全清会把收到的那包丢掉。
		//   留着的位下一轮 LORA_AppService() 还会再进来处理。
		if (g_dio1_irq & LLCC68_IRQ_RX_DONE)
		{
			lora_irq_clear(LLCC68_IRQ_RX_DONE);

#if LORA_ACK_ENABLE
			// 只有本包有效才回ACK；被丢弃的包不回
			//   （靠 lora_handle_rx() 的返回值判断，理由见它的函数头注释）
			if (lora_handle_rx())
			{
				if (lora_start_ack(lora_node_id) == LLCC68_STATUS_OK)
				{
					lora_tick = now;
					lora_state = LORA_ST_TX_ACK;
				}
				else
				{
					g_lora_tx_err_cnt++;
					printf("LoRa回ACK启动失败\r\n");
				}
			}
#else
			lora_handle_rx();
#endif
		}
		else if (g_dio1_irq & LLCC68_IRQ_CRC_ERROR)
		{
			lora_irq_clear(LLCC68_IRQ_CRC_ERROR);
			g_lora_rx_err_cnt++;
			printf("LoRa接收CRC错误\r\n");
		}
		else if (g_dio1_irq & LLCC68_IRQ_TIMEOUT)
		{
			// 连续接收模式下正常不会超时，真来了就重挂一次接收
			lora_irq_clear(LLCC68_IRQ_TIMEOUT);
			printf("LoRa接收超时，重新挂接收\r\n");
			lora_enter_rx();
		}
		break;

	/*--- 等ACK发完 ---*/
	case LORA_ST_TX_ACK:
		if (g_dio1_irq & LLCC68_IRQ_TX_DONE)
		{
			lora_irq_clear(LLCC68_IRQ_TX_DONE);
			g_lora_tx_ok_cnt++;
			printf("LoRa已回ACK给节点%d\r\n", lora_node_id);
			// 发完模式被切到过TX，要重新挂回接收
			//   （连续模式下收完包自己留在RX，但发过之后不会自动回来）
			if (lora_enter_rx() != LLCC68_STATUS_OK)
				printf("LoRa重新挂接收失败\r\n");
			lora_state = LORA_ST_RX;
		}
		else if (g_dio1_irq & LLCC68_IRQ_TIMEOUT)
		{
			lora_irq_clear(LLCC68_IRQ_TIMEOUT);
			g_lora_tx_err_cnt++;
			printf("LoRa回ACK超时\r\n");
			lora_enter_rx();
			lora_state = LORA_ST_RX;
		}
		else if (now - lora_tick >= LORA_TX_TIMEOUT_MS)
		{
			// 中断没来（DIO1没接上/没配好），靠软件超时兜底，
			// 保证主循环不会永远停在这个状态
			g_lora_tx_err_cnt++;
			printf("LoRa回ACK超时（软件兜底，检查DIO1是否接到PB10）\r\n");
			lora_enter_rx();
			lora_state = LORA_ST_RX;
		}
		break;

	default:
		lora_state = LORA_ST_RX;
		break;
	}
}
