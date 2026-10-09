#include "pwm.h"

//led-pwm呼吸灯的测试
//PA8-LED2-TIM1_CH1
//PB15-LED3-TIM1_CH3N
void led_breath(void)
{
	//OC1模式的启动函数
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //启动定时器
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1000); //占空比100%等灭
	
	
	//OC1N模式 互补通道的启动函数
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3); //启动定时器
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,200);
	
	
	int cnt = 1;
	while (1)
	{
		if(cnt++ > 10)
			cnt = 1;
		
		//在这个地方添加你自己的代码
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,(1000-100*cnt)); //呼吸灯
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,(1000-100*cnt));
		HAL_Delay(500);
	}
}

/*=========================================================================
 * 光控LED：根据光照强度自动切换LED亮度等级
 *
 * 通道说明：
 *   PA8  -> TIM1_CH1  ：普通通道，HAL_TIM_PWM_Start() 开的是 CC1E
 *   PB15 -> TIM1_CH3N ：互补通道，必须用 HAL_TIMEx_PWMN_Start() 开 CC3NE，
 *                       用 HAL_TIM_PWM_Start() 是开 CC3E，PB15不会有输出
 *
 * 两个通道方向一致（都是 CNT<CCR 输出高电平）：
 *   虽然叫"互补通道"，但这里 CC3E 没有使能、只有 CC3NE 使能，
 *   按参考手册此时 OC3N 不取反，直接跟随 OC3REF，
 *   所以两个灯可以用同一个CCR值控制，不用分别处理。
 *=========================================================================*/

/* 每档亮度对应的"导通占空比(%)"，导通 = 引脚输出低电平 = LED亮 */
static const uint8_t LED_DUTY_PERCENT[LED_LEVEL_MAX + 1] =
{
	0,      /* 等级0 灭 ：不导通，全程高电平（备用档，默认不用） */
	15,     /* 等级1 暗 ：导通15% */
	50,     /* 等级2 中 ：导通50% */
	100,    /* 等级3 亮 ：导通100% */
};

/* 启动两路PWM输出（上电后调用一次） */
void LED_PWM_Init(void)
{
	LED_SetLevel(LED_LEVEL_DIM);                    //先把比较值设好，避免启动瞬间灯全亮刺眼

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);       //OC1通道：使能CC1E，输出到PA8
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);    //OC3N互补通道：使能CC3NE，输出到PB15
}

/* 设置LED亮度等级：LED_LEVEL_DIM(暗) / LED_LEVEL_MID(中) / LED_LEVEL_BRIGHT(亮) */
void LED_SetLevel(uint8_t level)
{
	uint16_t on_ticks;      //一个周期里低电平（导通）的计数个数
	uint16_t ccr;           //写入CCR的值

	if (level > LED_LEVEL_MAX)
		level = LED_LEVEL_MAX;

	on_ticks = (uint16_t)((uint32_t)LED_PERIOD * LED_DUTY_PERCENT[level] / 100);

	//低电平导通，所以CCR要取"周期减去导通时间"：CCR越大 -> 高电平越久 -> 灯越暗
	//占空比0%时 ccr = 1000 = ARR+1，CNT(0~999)永远小于CCR，全程高电平 -> 灯灭
	ccr = LED_PERIOD - on_ticks;

	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, ccr);   //PA8 -LED2
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, ccr);   //PB15-LED3
}

/* 根据环境光照强度自动选择亮度档位：环境越暗，灯越亮
 * lux：GetLux() 读到的光照强度
 * 返回值：本次选中的亮度等级
 */
uint8_t LED_LightCtrl(uint16_t lux)
{
	uint8_t level;

	if (lux < LUX_DARK_THRESHOLD)               //环境很暗 -> 最亮
		level = LED_LEVEL_BRIGHT;
	else if (lux < LUX_BRIGHT_THRESHOLD)        //环境亮度中等 -> 中档
		level = LED_LEVEL_MID;
	else                                        //环境够亮 -> 最暗
		level = LED_LEVEL_DIM;

	LED_SetLevel(level);
	return level;
}
