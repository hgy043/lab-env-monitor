/**
 * @file    debug.c
 * @brief   调试串口（USART1 / PA9-PA10，115200 8N1）
 *
 *  硬件：USART1 经板载 CH340K 转 USB 接电脑，用串口助手（PortHelper）看打印。
 *        终端板这边 USART1 只负责调试打印；USART2 拿去接 CO2 传感器了。
 *
 *  本文件包含五样东西：
 *    fputc()                      重定向 printf → USART1（全工程可直接用 printf）
 *    debug_test1()                测试用：轮询把 PC 发来的 10 字节原样回发
 *                                 ⚠️ 里面有 while(1) 死循环，是教学用的测试函数，
 *                                    不要在主循环里调用
 *    debug_UART_Callback()        中断变长接收回调，由 ISR_callback.c 里的
 *                                 HAL_UARTEx_RxEventCallback() 分发过来
 *    Debug_UART_Receive_Start()   开机挂一次中断接收（必须调用，否则收不到）
 *    HardFault_Report()           硬件异常现场报告：由 Core/Src/stm32f1xx_it.c 里的
 *                                 HardFault_Handler 调用，打印故障寄存器后停机
 *                                 （不用 printf 的原因见该函数头注释）
 *
 *  ⚠️ debug_UART_Callback() 用 HAL_MAX_DELAY 阻塞回发，最长 128 字节 ≈ 11ms @115200，
 *     而它是跑在 USART1 中断里的 —— 这期间 LoRa 的 DIO1 中断（EXTI15_10）会被压住。
 *     目前 PC 侧基本不往这边发数据，所以没影响；将来如果要靠调试口下发指令，
 *     这个回发必须改成非阻塞（现在这样等于在中断里等 11ms）。
 */
#include "debug.h"
#include "usart.h" // STM32CubeMx生成
#include "stdio.h"

//重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/*
UART1 来轮询收发数据:将PC端发来的数据在发送给PC端
缺点：会占用大量的CPU时间，运行的效率很低。
*/
void debug_test1(void)
{
    uint8_t buff[10];
    while (1)
    {
        // 指定串口1接收10字节数据，HAL_MAX_DELAY阻塞永久等待，
        // 直到接收到数据。如果接收到数据，则返回HAL_OK    。
        // 如果你们不希望阻塞，可以指定一个超时时间，如1000 (1000ms)。
        if (HAL_UART_Receive(&huart1, buff, 10, HAL_MAX_DELAY) == HAL_OK)
        {
            // 把收到的数据原封不动的发出去, HAL_MAX_DELAY阻塞永久等待
            // 你也可以单独调用该函数主动发送你需要的数据
            HAL_UART_Transmit(&huart1, buff, 10, HAL_MAX_DELAY);
        }
    }
}

uint8_t debug_buffer[DEBUG_RX_BUF_SIZE]; // debug接收缓冲  DEBUG_RX_BUF_SIZE=>128
uint16_t debug_rx_len;                    // debug接收长度

// 这个函数被HAL_UARTEx_RxEventCallback()调用
void debug_UART_Callback(uint16_t Size)
{
		HAL_UART_Transmit(&huart1, debug_buffer, Size, HAL_MAX_DELAY);
    debug_rx_len = Size;
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, debug_buffer, sizeof(debug_buffer));// 继续开启接收数据
}

// 启动中断接收变长数据
void Debug_UART_Receive_Start(void)
{
    debug_rx_len = 0;
    memset(debug_buffer, 0, sizeof(debug_buffer));
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, debug_buffer, sizeof(debug_buffer));
}

/*==============================================================================
 * HardFault_Report() —— 硬件异常现场报告
 *
 * 调用位置：Core/Src/stm32f1xx_it.c 里 HardFault_Handler 的
 *           "USER CODE BEGIN HardFault_IRQn 0" 块内
 *           （那是生成代码里受保护的块，CubeMX 重新生成不会丢）。
 *
 * 【为什么需要它】
 *   CubeMX 生成的 HardFault_Handler 默认体就是一个 while(1)：不打印、不复位。
 *   于是「内存越界 / 野指针触发硬件异常」和「程序卡在某个等待里」在现象上
 *   一模一样 —— 串口都是突然没有下文，光看现象分不出来。
 *   2026-09-21 就被这一点坑过：一个 sprintf 栈溢出的 bug，因为现象是
 *   「打印完一句就再没下文」，被当成射频/超时问题查了很久。加上这个报告以后，
 *   同样的情况串口会直接打出 HardFault 横幅和几个故障寄存器。
 *
 * 【为什么直接写 USART1 寄存器，不用 printf、也不用 HAL_UART_Transmit】
 *   ① printf 有可能正被这次故障打断（C 库内部状态不确定），不能再进一次；
 *   ② HAL_UART_Transmit 会先看 huart1.gState，如果故障恰好发生在一次发送
 *      过程中，gState 还是 BUSY_TX，它会直接返回 HAL_BUSY —— 一个字都发不出来。
 *   直接轮询 USART1->SR 的 TXE 位、往 DR 写字节，不依赖任何库/HAL 状态，
 *   是故障处理器里唯一能保证送得出去的办法。
 *
 * 【打印完为什么停住不返回】
 *   故障已经发生，栈和寄存器都可能已经坏了，没有「恢复正常」这一说。
 *   原地停住能把现场完整留着 —— 接上 J-Link 一看 PC 停在哪就定位到具体位置；
 *   也避免在已损坏的状态下继续跑，把现场二次破坏掉。
 *
 * 【局限，要如实知道】
 *   这里没有打印出错时的 PC/LR。要拿到那个得把 HardFault_Handler 改成
 *   __attribute__((naked)) 自己写汇编写栈帧，而那个函数体是 CubeMX 生成的、
 *   每次重新生成都会覆盖，不适合动。CFSR/HFSR 已经足够区分「是取指跑飞、
 *   是数据访存越界、还是出栈失败」；要精确定位到行就接调试器看调用栈。
 *
 * 【前提】USART1 已经初始化过（本函数不做初始化）。
 *         开机阶段就跑飞的话打不出来，只能靠调试器。
 *============================================================================*/

/* 直接写 USART1 数据寄存器，绕过 HAL 和 C 库（理由见上面函数头注释） */
static void fault_putc(char c)
{
	while ((USART1->SR & USART_SR_TXE) == 0)
		;
	USART1->DR = (uint8_t)c;
}

static void fault_puts(const char *s)
{
	while (*s != '\0')
		fault_putc(*s++);
}

/* 输出 32 位十六进制（大写、不带 0x 前缀），用来打寄存器值 */
static void fault_put_hex32(uint32_t v)
{
	static const char hex_tab[] = "0123456789ABCDEF";
	int shift;

	for (shift = 28; shift >= 0; shift -= 4)
		fault_putc(hex_tab[(v >> shift) & 0x0F]);
}

void HardFault_Report(void)
{
	uint32_t hfsr = SCB->HFSR;
	uint32_t cfsr = SCB->CFSR;
	uint32_t mmfar = SCB->MMFAR;
	uint32_t bfar = SCB->BFAR;

	fault_puts("\r\n\r\n");
	fault_puts("!!!!!! 硬件异常 HardFault !!!!!!\r\n");
	fault_puts("程序跑飞了 —— 不是卡在某个等待里，是硬件异常停机。\r\n");
	fault_puts("常见原因：数组/缓冲区越界、野指针、栈溢出。\r\n");

	fault_puts("  HFSR  = 0x");
	fault_put_hex32(hfsr);
	fault_puts("   CFSR = 0x");
	fault_put_hex32(cfsr);
	fault_puts("\r\n  MMFAR = 0x");
	fault_put_hex32(mmfar);
	fault_puts("   BFAR = 0x");
	fault_put_hex32(bfar);
	fault_puts("\r\n");

	/* HFSR：只说明「是别的故障升级上来的」，真正原因在 CFSR 里 */
	if (hfsr & (1UL << 30))
		fault_puts("  HFSR.FORCED=1：由可配置故障升级而来，看下面的 CFSR\r\n");
	else if (hfsr & (1UL << 1))
		fault_puts("  HFSR.VECTTBL=1：取中断向量表时总线出错\r\n");

	/* 把 CFSR 的位翻译成人话
	 *   位号按 Cortex-M3 权威指南 / CM3 参考手册：
	 *   低 8 位 = MemManage，中间 8 位 = BusFault，高 16 位 = UsageFault */
	if (cfsr == 0)
	{
		fault_puts("  CFSR 全 0：不是可配置故障升级来的，查外部中断向量表\r\n");
	}
	else
	{
		/* --- MemManage（CFSR[7:0]）--- */
		if (cfsr & (1UL << 1))
			fault_puts("  [MemManage] 数据访存违规 DACCVIOL\r\n");
		if (cfsr & (1UL << 4))
			fault_puts("  [MemManage] 异常入栈失败 MSTKERR —— 栈指针已经坏了\r\n");
		if (cfsr & (1UL << 3))
			fault_puts("  [MemManage] 异常出栈失败 MUNSTKERR\r\n");
		if (cfsr & (1UL << 7))
			fault_puts("  [MemManage] MMFAR 有效，上面的 MMFAR 就是出问题的地址\r\n");

		/* --- BusFault（CFSR[15:8]）--- */
		if (cfsr & (1UL << 9))
			fault_puts("  [BusFault] 精确总线错误 PRECISERR —— 访问了不存在的地址\r\n");
		if (cfsr & (1UL << 10))
			fault_puts("  [BusFault] 非精确总线错误 IMPRECISERR —— 写缓冲延迟报的，多半也是越界写\r\n");
		if (cfsr & (1UL << 12))
			fault_puts("  [BusFault] 异常入栈失败 STKERR —— 栈指针已经坏了\r\n");
		if (cfsr & (1UL << 11))
			fault_puts("  [BusFault] 异常出栈失败 UNSTKERR\r\n");
		if (cfsr & (1UL << 8))
			fault_puts("  [BusFault] 取指令总线错误 IBUSERR\r\n");
		if (cfsr & (1UL << 15))
			fault_puts("  [BusFault] BFAR 有效，上面的 BFAR 就是出问题的地址\r\n");

		/* --- UsageFault（CFSR[31:16]）--- */
		if (cfsr & (1UL << 16))
			fault_puts("  [UsageFault] 未定义指令 UNDEFINSTR —— PC 跑到非代码区了\r\n");
		if (cfsr & (1UL << 17))
			fault_puts("  [UsageFault] 非法状态 INVSTATE —— 典型的栈被写坏，PC/LR 被改坏\r\n");
		if (cfsr & (1UL << 18))
			fault_puts("  [UsageFault] 非法 PC 装入 INVPC\r\n");
		if (cfsr & (1UL << 24))
			fault_puts("  [UsageFault] 非对齐访问 UNALIGNED\r\n");
		if (cfsr & (1UL << 25))
			fault_puts("  [UsageFault] 除零 DIVBYZERO\r\n");
	}

	fault_puts("--------------------------------------------------\r\n");
	fault_puts("已停机，现场保留。接上调试器看 PC 停在哪就能定位到具体位置。\r\n");

	while (1)
		;
}
