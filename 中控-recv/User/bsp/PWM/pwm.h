#ifndef __PWM_H
#define __PWM_H

#include "main.h"
#include "gpio.h"
#include "tim.h"   //官方定时器的头文件
#include "app_main.h"

//led-pwm呼吸灯的测试
void led_breath(void);


//PWM电机控速


//PWM蜂鸣器多频率


/*=======================================================================
 * 光控LED：PA8-LED2-TIM1_CH1 / PB15-LED3-TIM1_CH3N
 *   硬件接法：低电平导通（灯亮），高电平灯灭
 *   TIM1 配置：PSC=72-1 -> 1MHz计数，ARR=999 -> 一个周期1000个计数，1kHz
 *   控制原理：PWM1模式，CNT < CCR 时输出高电平（灭），
 *             CNT >= CCR 时输出低电平（亮）
 *             => CCR 越大，高电平时间越长，灯越暗
 *             => CCR = 1000（=ARR+1）为全程高电平 -> 灯灭
 *             => CCR = 0 为全程低电平 -> 最亮
 *=======================================================================*/
#define LED_PERIOD              (1000)  /* 一个PWM周期的计数值 = ARR+1 */

/* 亮度等级（数值越大越亮） */
#define LED_LEVEL_OFF           (0)     /* 灭（备用档，默认程序不使用） */
#define LED_LEVEL_DIM           (1)     /* 暗 */
#define LED_LEVEL_MID           (2)     /* 中 */
#define LED_LEVEL_BRIGHT        (3)     /* 亮 */
#define LED_LEVEL_MAX           LED_LEVEL_BRIGHT

/* 光照强度切换阈值（lux），对照 light.c 里的 GL5528 阻值-流明对照表：
 * 查表可得 0/1/2/4/5/10/17/29/45/68/124/350 这些档位
 *   < 10  ：接近全黑、极暗、深夜、昏暗角落      -> 灯最亮
 *   10~44 ：弱光、夜间灯光旁、傍晚室内、明亮室内 -> 灯中档
 *   >= 45 ：晴天窗边、户外阴影、烈日             -> 灯最暗
 */
#define LUX_DARK_THRESHOLD      (10)    /* 低于此值认为环境很暗 */
#define LUX_BRIGHT_THRESHOLD    (45)    /* 高于此值认为环境够亮 */

/* 启动两路PWM输出，上电后调用一次 */
void LED_PWM_Init(void);

/* 直接设置亮度等级：LED_LEVEL_DIM / MID / BRIGHT */
void LED_SetLevel(uint8_t level);

/* 根据环境光照强度自动切换亮度等级，返回本次选中的等级 */
uint8_t LED_LightCtrl(uint16_t lux);

#endif
