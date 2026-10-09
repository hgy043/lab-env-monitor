#include "alarm.h"
#include "beep.h"       /* 蜂鸣器 alarm()，引脚 PB9 */
#include "debug.h"      /* printf 调试输出 */
#include <stdio.h>

/*============================ 引脚定义 ============================*/
/* 注意：电平用的是 GPIO_PIN_RESET / GPIO_PIN_SET 枚举，不是 0/1，别写成 ! 取反 */
#define BUZZER_PORT          GPIOB
#define BUZZER_PIN           GPIO_PIN_9     /* 蜂鸣器：高电平=响         */

#define FAN_PORT             GPIOA
#define FAN_PIN              GPIO_PIN_8     /* 风机（LED2）              */
#define FAN_ON_LEVEL         GPIO_PIN_RESET /* 低电平=导通               */
#define FAN_OFF_LEVEL        GPIO_PIN_SET

#define CURTAIN_PORT         GPIOB
#define CURTAIN_PIN          GPIO_PIN_15    /* 遮阳帘（LED3）            */
#define CURTAIN_ON_LEVEL     GPIO_PIN_RESET /* 低电平=导通               */
#define CURTAIN_OFF_LEVEL    GPIO_PIN_SET

/*============================ 注意：引脚已被其它模块占用 ============
 * 本模块是早期版本，下面三个引脚现在都改成定时器PWM输出了，
 * 直接用 HAL_GPIO_WritePin 写它们不会有任何效果（复用功能引脚由定时器驱动）：
 *   PB9   -> TIM4_CH4 的PWM输出，蜂鸣器请用 beep.c 的 BEEP_CO2Alarm()
 *   PA8   -> TIM1_CH1 的PWM输出，由 pwm.c 的光控LED占用
 *   PB15  -> TIM1_CH3N的PWM输出，同上
 * 也就是说：重新启用 alarm_process() 后蜂鸣器不会响、风机/遮阳帘也不会动。
 * 要做CO2报警请用 beep.c 的接口。
 *===================================================================*/

/*============================ 报警条件表 ============================*/
#define ALARM_HIGH          1   /* 高于阈值才报警（上限） */
#define ALARM_LOW           0   /* 低于阈值才报警（下限） */

/* 超限时要联动的设备 */
#define LINK_NONE           0   /* 只报警，不联动设备       */
#define LINK_FAN_CURTAIN    1   /* 启动风机 + 拉上遮阳帘    */
#define LINK_CURTAIN        2   /* 只拉上遮阳帘             */

/* 这条报警条件读的是哪个传感器的数据 */
#define SRC_TEMP            0
#define SRC_HUMI            1
#define SRC_CO2             2

/* 一条报警条件：名称、数据来源、比较方向、阈值、联动动作、超限状态、超限开始时间 */
typedef struct
{
    const char  *name;          /* 用于串口打印的原因文字 */
    uint8_t      source;        /* SRC_TEMP / SRC_HUMI / SRC_CO2 */
    uint8_t      is_high;       /* ALARM_HIGH / ALARM_LOW  */
    float        threshold;     /* 阈值                    */
    uint8_t      link_action;   /* 超限时联动哪些设备      */
    uint8_t      violating;     /* 当前是否超限            */
    uint32_t     start_tick;    /* 本次连续超限的开始时刻  */
} alarm_group_t;

/* 6 组预设阈值，顺序就是报警优先级（先命中的先报告） */
static alarm_group_t alarm_groups[] =
{
    { "温度过高(>35C)",    SRC_TEMP, ALARM_HIGH, (float)TEMP_HIGH_THRESHOLD, LINK_FAN_CURTAIN, 0, 0 },
    { "温度过低(<10C)",    SRC_TEMP, ALARM_LOW,  (float)TEMP_LOW_THRESHOLD,  LINK_CURTAIN,     0, 0 },
    { "湿度过高(>90%)",    SRC_HUMI, ALARM_HIGH, (float)HUMI_HIGH_THRESHOLD, LINK_NONE,        0, 0 },
    { "湿度过低(<40%)",    SRC_HUMI, ALARM_LOW,  (float)HUMI_LOW_THRESHOLD,  LINK_NONE,        0, 0 },
    { "CO2过高(>2000ppm)", SRC_CO2,  ALARM_HIGH, (float)CO2_HIGH_THRESHOLD,  LINK_NONE,        0, 0 },
    { "CO2过低(<500ppm)",  SRC_CO2,  ALARM_LOW,  (float)CO2_LOW_THRESHOLD,   LINK_NONE,        0, 0 },
};

#define ALARM_GROUP_COUNT   (sizeof(alarm_groups) / sizeof(alarm_groups[0]))

/*============================ 模块内部状态 ============================*/
volatile uint8_t g_reset_key_pressed = 0;   /* 复位键按下标志，ISR 里置 1 */

static uint8_t  s_state        = ALARM_STATE_NORMAL; /* 报警状态机 */

/*============================ 执行器控制 ============================*/
/* 把执行器设置到"开"或"关"，导通电平在各引脚的宏里配置 */
static void fan_set(uint8_t on)
{
    HAL_GPIO_WritePin(FAN_PORT, FAN_PIN, on ? FAN_ON_LEVEL : FAN_OFF_LEVEL);
}

static void curtain_set(uint8_t on)
{
    HAL_GPIO_WritePin(CURTAIN_PORT, CURTAIN_PIN, on ? CURTAIN_ON_LEVEL : CURTAIN_OFF_LEVEL);
}

/* 关闭所有报警输出：蜂鸣器停、执行器复位 */
static void alarm_outputs_off(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, ALARM_ACTIVE_OFF);
    fan_set(0);
    curtain_set(0);
}

/*============================ 报警联动处理 ============================*/
/* 报警期间：蜂鸣器持续鸣叫，并按报警原因启动对应设备 */
static void alarm_outputs_on(void)
{
    uint8_t fan_on     = 0;
    uint8_t curtain_on = 0;
    uint8_t i;

    /* 蜂鸣器：持续鸣叫（PB9 高电平）*/
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, ALARM_ACTIVE_ON);

    /* 联动规则（由条件表的 link_action 决定）：
     *   温度过高 → 启动风机 + 拉上遮阳帘
     *   温度过低 → 只拉上遮阳帘（保温；此时开风机会让室温更低）
     *   湿度/CO2 → 只报警，不联动设备
     */
    for (i = 0; i < ALARM_GROUP_COUNT; i++)
    {
        if (alarm_groups[i].violating == 0)
        {
            continue;
        }

        if (alarm_groups[i].link_action == LINK_FAN_CURTAIN)
        {
            fan_on     = 1;
            curtain_on = 1;
        }
        else if (alarm_groups[i].link_action == LINK_CURTAIN)
        {
            curtain_on = 1;
        }
    }

    fan_set(fan_on);
    curtain_set(curtain_on);
}

/*============================ 对外接口 ============================*/
void alarm_init(void)
{
    uint8_t i;

    for (i = 0; i < ALARM_GROUP_COUNT; i++)
    {
        alarm_groups[i].violating  = 0;
        alarm_groups[i].start_tick = 0;
    }

    s_state              = ALARM_STATE_NORMAL;
    g_reset_key_pressed  = 0;

    alarm_outputs_off();
}

void alarm_process(float temp, float humi, uint16_t co2, uint8_t data_valid)
{
    uint32_t now = HAL_GetTick();
    uint8_t  i;
    uint8_t  any_violating = 0;

    /* 传感器读失败时直接跳过本轮，避免用 0 值误触发报警 */
    if (!data_valid)
    {
        return;
    }

    /*--------------------------------------------------------------------------
     * 第一步：逐条判断是否超限，并累计"连续超限"的持续时间
     *------------------------------------------------------------------------*/
    for (i = 0; i < ALARM_GROUP_COUNT; i++)
    {
        float   value;
        uint8_t is_violating;

        /* 按条件表里配置的数据来源取值 */
        switch (alarm_groups[i].source)
        {
            case SRC_TEMP: value = temp;       break;
            case SRC_HUMI: value = humi;       break;
            default:       value = (float)co2; break;
        }

        is_violating = alarm_groups[i].is_high ? (value > alarm_groups[i].threshold)
                                               : (value < alarm_groups[i].threshold);

        if (is_violating)
        {
            /* 刚刚开始超限：记下起始时刻，从这里开始计时 */
            if (alarm_groups[i].violating == 0)
            {
                alarm_groups[i].violating  = 1;
                alarm_groups[i].start_tick = now;
            }
            any_violating = 1;
        }
        else
        {
            /* 回到正常范围：清掉标志，下次再超限要重新累计 3 秒 */
            alarm_groups[i].violating = 0;
        }
    }

    /*--------------------------------------------------------------------------
     * 第二步：静音自锁的解除条件
     *   按过复位键之后，报警不再响；直到所有参数都回到正常范围，
     *   才自动解除静音，重新恢复到正常监测状态。
     *------------------------------------------------------------------------*/
    if (s_state == ALARM_STATE_SILENCED && !any_violating)
    {
        s_state = ALARM_STATE_NORMAL;
        for (i = 0; i < ALARM_GROUP_COUNT; i++)
        {
            alarm_groups[i].start_tick = 0;   /* 复位计时，防止下次一超限就立刻报警 */
        }
        printf("[报警] 所有参数已恢复正常，解除静音，重新进入监测状态\n");
    }

    /*--------------------------------------------------------------------------
     * 第三步：超限持续满 3 秒 → 进入报警状态
     *------------------------------------------------------------------------*/
    if (s_state == ALARM_STATE_NORMAL)
    {
        for (i = 0; i < ALARM_GROUP_COUNT; i++)
        {
            if (alarm_groups[i].violating &&
                (now - alarm_groups[i].start_tick >= ALARM_HOLD_TIME_MS))
            {
                s_state = ALARM_STATE_WARNING;
                printf("[报警] 触发报警：%s，已持续 %ums，启动蜂鸣器与联动设备\n",
                       alarm_groups[i].name, (unsigned int)ALARM_HOLD_TIME_MS);
                break;
            }
        }
    }

    /*--------------------------------------------------------------------------
     * 第四步：检测复位键 —— 按下后彻底静音（自锁）
     *------------------------------------------------------------------------*/
    if (g_reset_key_pressed)
    {
        g_reset_key_pressed = 0;   /* volatile 变量，用完马上清零 */
        if (s_state == ALARM_STATE_WARNING)
        {
            s_state = ALARM_STATE_SILENCED;
            printf("[报警] 复位键按下，报警已解除（静音自锁，参数恢复后自动复位）\n");
        }
    }

    /*--------------------------------------------------------------------------
     * 第五步：根据状态驱动蜂鸣器和联动设备
     *   WARNING  → 蜂鸣器持续鸣叫 + 启动风机/遮阳帘
     *   NORMAL / SILENCED → 全部关闭
     *------------------------------------------------------------------------*/
    if (s_state == ALARM_STATE_WARNING)
    {
        alarm_outputs_on();
    }
    else
    {
        alarm_outputs_off();
    }
}

uint8_t alarm_is_active(void)
{
    return (s_state == ALARM_STATE_WARNING);
}

uint8_t alarm_get_state(void)
{
    return s_state;
}
