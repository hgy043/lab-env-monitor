#ifndef ALARM_H
#define ALARM_H

#include "main.h"


/*==============================================================================
 * 环境参数报警模块
 *------------------------------------------------------------------------------
 * 功能：
 *   1. 多组阈值判断：温度/湿度/CO2 各自的上限和下限，共 6 组
 *   2. 持续时间判断：某个参数连续超限达到 ALARM_HOLD_TIME_MS(3秒) 才真正报警
 *   3. 报警联动：报警时自动启动风机、遮阳帘
 *   4. 报警解除：按下复位键(SW1-PB13)后彻底静音（自锁式），
 *      等所有参数回到正常范围后自动解除静音，重新进入监测状态
 *------------------------------------------------------------------------------
 * 引脚对照表（不要改这里，改这里没用，实际接线在各模块里定义）：
 *   PB9  蜂鸣器  高电平=响     ← 报警声
 *   PA8  风机    低电平=导通   ← 温度过高时联动
 *   PB15 遮阳帘  低电平=导通   ← 温度过高/过低时联动
 *   PB13 复位键  低电平=按下   ← 手动解除报警
 *============================================================================*/

/* 报警持续时间：超限连续保持 3 秒以上才触发报警 */
#define ALARM_HOLD_TIME_MS      3000U

/* 阈值说明：
 *   温度     10℃ ~ 35℃   （>35℃ 或 <10℃ 报警）
 *   湿度     40% ~ 90%   （>90% 或 <40% 报警）
 *   CO2浓度  500 ~ 2000ppm（>2000ppm 或 <500ppm 报警）
 */
#define TEMP_HIGH_THRESHOLD     35.0f   /* 温度上限 ℃  */
#define TEMP_LOW_THRESHOLD      10.0f   /* 温度下限 ℃  */
#define HUMI_HIGH_THRESHOLD     90.0f   /* 湿度上限 %RH */
#define HUMI_LOW_THRESHOLD      40.0f   /* 湿度下限 %RH */
#define CO2_HIGH_THRESHOLD      2000U   /* CO2上限 ppm */
#define CO2_LOW_THRESHOLD       500U    /* CO2下限 ppm */

/* 报警状态 */
#define ALARM_STATE_NORMAL      0       /* 正常监测中          */
#define ALARM_STATE_WARNING     1       /* 报警中              */
#define ALARM_STATE_SILENCED    2       /* 已按复位键，静音中  */

/* 报警输出设备的通断电平（GPIO_PinState 类型，不要直接改） */
#define ALARM_ACTIVE_ON         GPIO_PIN_SET
#define ALARM_ACTIVE_OFF        GPIO_PIN_RESET

/* 复位键标志：由 EXTI 中断置位，主循环检测后清零
 * volatile 不可省略：中断里改的变量，主循环必须每次都去内存里读 */
extern volatile uint8_t g_reset_key_pressed;

/* 初始化报警模块（清零所有状态，关闭蜂鸣器和执行器） */
void alarm_init(void);

/* 周期性处理报警逻辑，放在主循环里调用，参数：温度、湿度、CO2、数据是否有效 */
void alarm_process(float temp, float humi, uint16_t co2, uint8_t data_valid);

/* 是否正在报警（蜂鸣器鸣叫中） */
uint8_t alarm_is_active(void);

/* 当前报警状态（ALARM_STATE_NORMAL / WARNING / SILENCED） */
uint8_t alarm_get_state(void);

#endif
