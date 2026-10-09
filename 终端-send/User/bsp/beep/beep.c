#include "beep.h"
#include "debug.h"      /* printf 调试输出 */
#include <stdio.h>

/* 蜂鸣器手动测试：每500ms在"不响"和"最响"之间切换
 * 注意：PB9 现在是 TIM4_CH4 的PWM输出，不能再当普通GPIO翻转了
 *       （HAL_GPIO_TogglePin 对复用功能引脚无效），
 *       所以这里改成切换占空比，实际效果和原来的电平翻转一样 */
void alarm(void)
{
	static uint8_t on = 0;

	delay_ms(500);//延时0.5秒
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);//继电器翻转
	on = !on;
	BEEP_SetDuty(on ? BEEP_DUTY_HIGH : BEEP_DUTY_OFF);
}

/*=========================================================================
 * CO2 蜂鸣器报警：PB9 - TIM4_CH4
 *   正常范围(800~1500ppm) -> 不响
 *   接近阈值(500~800 / 1500~2000ppm) -> 小声预警
 *   超出范围(<500 / >2000ppm) -> 大声报警
 *
 * 按键取消（静音自锁）：
 *   正在响的时候按一下SW1(PB13)，蜂鸣器立刻停；之后就算CO2一直超限也不响，
 *   直到CO2回到"本该不响"的档位(800~1500ppm)才自动解除静音，重新开始监控，
 *   再超限才会重新报警。
 *   这样做的好处是噪音只在你确认过之后响一次，不会反复折腾人。
 *=========================================================================*/

/* 每个报警等级对应的驱动占空比，下标就是 BEEP_LEVEL_xxx */
static const uint8_t BEEP_DUTY_TABLE[3] =
{
	BEEP_DUTY_OFF,      /* 等级0 正常：不响 */
	BEEP_DUTY_LOW,      /* 等级1 预警：小声 */
	BEEP_DUTY_HIGH,     /* 等级2 报警：大声 */
};

static uint8_t s_beep_level = BEEP_LEVEL_OFF;           /* 当前实际生效的报警等级 */
static uint8_t s_beep_state = BEEP_STATE_MONITOR;       /* 报警状态：监控中 / 已静音自锁 */

/* 初始化：启动PWM通道，初始占空比0%（不响） */
void BEEP_Init(void)
{
	s_beep_level = BEEP_LEVEL_OFF;
	s_beep_state = BEEP_STATE_MONITOR;

	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);   //先把比较值设成0，避免上电瞬间响一声
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);         //启动TIM4_CH4，输出到PB9
}

/* 设置驱动占空比：0 = 不响，数值越大越响 */
void BEEP_SetDuty(uint8_t percent)
{
	uint16_t ccr;   //写入CCR的值

	if (percent > 100)
		percent = 100;

	//PWM1模式下 CNT < CCR 时输出高电平，所以CCR就是高电平（导通）的计数个数
	//占空比0%时 ccr = 0，CNT永远不小于0，全程低电平 -> 蜂鸣器不响
	ccr = (uint16_t)((uint32_t)BEEP_PERIOD * percent / 100);

	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, ccr);
}

/* 根据CO2浓度自动选择报警等级并驱动蜂鸣器
 * co2       ：CO2浓度值（ppm）
 * data_valid：本次读数是否有效，传0会直接静音（不用残留值误报）
 * 返回值：本次实际生效的报警等级（静音自锁期间一律返回 BEEP_LEVEL_OFF）
 */
uint8_t BEEP_CO2Alarm(uint16_t co2, uint8_t data_valid)
{
	uint8_t raw_level;   //单看浓度"本该"处于的等级，还没考虑静音自锁

	/* 传感器读失败：先静音，避免拿上一次的残留值误报。
	 * 这种情况下不动状态，静音自锁不会被一次读失败打断 */
	if (!data_valid)
	{
		BEEP_SetDuty(BEEP_DUTY_OFF);
		s_beep_level = BEEP_LEVEL_OFF;
		return s_beep_level;
	}

	/* 第一步：只看浓度，算出这一轮本该处于的报警等级 */
	if (co2 < BEEP_CO2_LOW || co2 > BEEP_CO2_HIGH)
		raw_level = BEEP_LEVEL_LOUD;                //超出正常范围 -> 大声报警
	else if (co2 < BEEP_CO2_WARN_LOW || co2 > BEEP_CO2_WARN_HIGH)
		raw_level = BEEP_LEVEL_QUIET;               //接近阈值 -> 小声预警
	else
		raw_level = BEEP_LEVEL_OFF;                 //正常范围 -> 不响

	/* 第二步：浓度已经回到"本该不响"的档位 -> 自动解除静音自锁，重新开始监控
	 * 注意这里用的是还没被静音覆盖的 raw_level */
	if (s_beep_state == BEEP_STATE_SILENCED && raw_level == BEEP_LEVEL_OFF)
	{
		s_beep_state = BEEP_STATE_MONITOR;
		printf("[蜂鸣器] CO2已恢复正常，解除静音，重新开始监控\n");
	}

	/* 第三步：静音自锁期间，无论浓度多高都不响 */
	s_beep_level = (s_beep_state == BEEP_STATE_SILENCED) ? BEEP_LEVEL_OFF : raw_level;

	BEEP_SetDuty(BEEP_DUTY_TABLE[s_beep_level]);

	return s_beep_level;
}

/* 确认/取消报警：主循环里检测到SW1(PB13)按下就调用
 * 只有正在响的时候才起作用——没响的时候按它不做任何事，
 * 避免在报警真正发生之前就把报警"提前关掉"了 */
void BEEP_Ack(void)
{
	if (s_beep_state == BEEP_STATE_MONITOR && s_beep_level != BEEP_LEVEL_OFF)
	{
		s_beep_state = BEEP_STATE_SILENCED;
		s_beep_level = BEEP_LEVEL_OFF;
		BEEP_SetDuty(BEEP_DUTY_OFF);
		printf("[蜂鸣器] SW1按下，报警已取消（静音自锁，CO2恢复后自动复位）\n");
	}
}

uint8_t BEEP_GetLevel(void)
{
	return s_beep_level;
}

uint8_t BEEP_GetState(void)
{
	return s_beep_state;
}
