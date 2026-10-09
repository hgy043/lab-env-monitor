#ifndef BEEP_MAIN_H	// 防止头文件重复包含
#define BEEP_MAIN_H

#include "main.h"
#include "gpio.h"
#include "app_main.h"
#include "tim.h"   //htim4

void alarm(void);

/*=======================================================================
 * 蜂鸣器报警：PB9 - TIM4_CH4
 *   硬件接法（见原理图）：PB9 -> R6 10k -> Q2(S8050 NPN) 基极，
 *                         发射极接地，集电极接 BUZZER2(3kHz) -> VCC5V
 *                         PB9 输出高电平 -> 三极管导通 -> 蜂鸣器响
 *   TIM4 配置：PSC=72-1 -> 1MHz计数，ARR=332 -> 1MHz/333 = 3003Hz ≈ 3kHz
 *   （TIM4 挂在APB1上，APB1分频系数为2，所以定时器时钟仍是72MHz）
 *   控制原理：PWM1模式，CNT < CCR 时输出高电平
 *             CCR = 0    -> 全程低电平      -> 不响
 *             CCR = 167  -> 占空比50%       -> 最响（无源蜂鸣器谐振点占空比50%）
 *             占空比越小，平均驱动功率越低，声音越小
 *=======================================================================*/
#define BEEP_PERIOD             (333)   /* 一个PWM周期的计数值 = ARR+1 */

/* CO2 报警阈值（ppm），和 alarm.h 里的 CO2 上下限保持一致 */
#define BEEP_CO2_LOW            (500)
#define BEEP_CO2_HIGH           (2000)

/* 预警区间：还没超限但已经很接近阈值了，先小声提醒
 *   500 ~ 800   低于下限预警
 *   1500 ~ 2000 高于上限预警
 * 嫌吵或者想让报警更灵敏，改这两个值即可 */
#define BEEP_CO2_WARN_LOW       (800)
#define BEEP_CO2_WARN_HIGH      (1500)

/* 三档驱动占空比（%） */
#define BEEP_DUTY_OFF           (0)     /* 不响     */
#define BEEP_DUTY_LOW           (20)    /* 小声预警 */
#define BEEP_DUTY_HIGH          (50)    /* 大声报警 */

/* 报警等级 */
#define BEEP_LEVEL_OFF          (0)     /* CO2在正常范围，不响       */
#define BEEP_LEVEL_QUIET        (1)     /* 接近阈值，小声预警        */
#define BEEP_LEVEL_LOUD         (2)     /* 超出正常范围，大声报警    */

/* 报警状态（用于"按键取消报警"） */
#define BEEP_STATE_MONITOR      (0)     /* 正常监控中，按浓度自动报警 */
#define BEEP_STATE_SILENCED     (1)     /* 已按SW1静音自锁，等浓度恢复正常后自动解除 */

/* 初始化蜂鸣器PWM通道，上电后调用一次 */
void BEEP_Init(void);

/* 直接设置驱动占空比：0 = 不响，数值越大越响 */
void BEEP_SetDuty(uint8_t percent);

/* 按CO2浓度选择报警等级并驱动蜂鸣器，返回本次实际生效的等级
 * co2       ：CO2浓度值（ppm）
 * data_valid：本次读数是否有效，传0（读失败）会直接静音，避免用残留值误报
 */
uint8_t BEEP_CO2Alarm(uint16_t co2, uint8_t data_valid);

/* 确认/取消报警：主循环里检测到SW1(PB13)按下就调用
 * 只有在正在响的时候才起作用，静音后进入自锁状态，
 * 直到CO2浓度回到"本该不响"的档位才自动解除，重新开始监控 */
void BEEP_Ack(void);

/* 当前报警等级 */
uint8_t BEEP_GetLevel(void);

/* 当前报警状态（BEEP_STATE_MONITOR / BEEP_STATE_SILENCED） */
uint8_t BEEP_GetState(void);

#endif
