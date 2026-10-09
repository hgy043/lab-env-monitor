/**
 * @file    app_main.c
 * @brief   终端节点业务主循环
 *
 * ============================================================================
 * 这块板子的角色：**终端节点**
 *
 *   和现在的 15.1 中控板是同一套引脚（.ioc 只差三个引脚：中控那边没有
 *   PA0 光敏、PA1 土壤、PC15 DHT22，也就没有 ADC1 和 DMA）。
 *   区别只在烧进去的固件角色：这块跑「发」，中控那块跑「收」。
 *
 * 引脚分工：
 *   USART1  PA9 / PA10   115200  调试口，经 CH340K 接电脑，printf 走这里
 *   USART2  PA2 / PA3     9600   CO2 传感器（6 字节定长帧，模块地址 0x2C）
 *   SPI1    PA5/PA6/PA7          LLCC68 LoRa 模块 SCK/MISO/MOSI
 *           PA4=NSS  PB1=RST  PB0=BUSY  PB10=DIO1(EXTI10)
 *   I2C1    PB6 / PB7    100kHz  OLED（SSD1306 128x64，地址 0x78）
 *   ADC1    PA0 / PA1            IN0=GL5528 光敏（10k 分压），IN1=YX55769 土壤
 *                                双通道 + DMA 循环搬运，连续转换
 *   TIM1    PA8 / PB15           CH1 + CH3N，光控 LED PWM（低电平导通）
 *   TIM4    PB9                  CH4，蜂鸣器 PWM（3kHz 无源，高电平响）
 *   PC15                         DHT22(AM2302) 单总线，时序软件模拟
 *   PC13                         板载 LED
 *   EXTI    PB12 / PB13          SW2 / SW1 两个按键
 *
 * ----------------------------------------------------------------------------
 * 主循环里的定时任务，各用各的计时基准：
 *
 *   1s   CO2    读一次浓度 → 顺带驱动蜂鸣器报警档位
 *   2s   DHT22  读一次温湿度
 *   500ms 光控  读光敏 → 自动切 LED 亮度档；顺带读一次土壤等级
 *   500ms OLED  把上面几项刷到屏上（周期见 oled_ui.h 的 OLED_UI_PERIOD_MS）
 *   每轮  SW1    按键复位（只在中断里置标志位，动作放主循环做）
 *   每轮  LoRa   每 LORA_UPLOAD_PERIOD_MS 上报一包 + 等中控的 ACK
 *                （周期/收发窗口都在 lora_app.h 里，见那个文件的头部说明）
 *
 * ⚠ 每个定时任务都必须用**自己的** tick 变量，绝对不能共用：
 *   共用的话谁先跑到就把 tick 重置成当前时刻，排在后面的块几乎永远不会执行。
 *   这个坑踩过一次 —— 光控块原来和 CO2 块共用 tick_1S，结果几乎不执行，
 *   后来才拆出独立的 tick_light / tick_oled。
 *
 * ⚠ 主循环里不要加秒级的阻塞调用：
 *   CO2_get_data() 是死等循环，占住太久会误了 LoRa 的收包和 ACK 窗口
 *   （上报发完之后只有 LORA_ACK_WINDOW_MS 那么长的时间等中控回 ACK），
 *   所以它的超时参数压到了 100ms。
 * ============================================================================
 */

#include "app_main.h"
#include "main.h"
#include "beep.h"     // BEEP_Init() / BEEP_CO2Alarm() / BEEP_Ack()
#include "pwm.h"      // LED_PWM_Init() / LED_LightCtrl()
#include "debug.h"    // Debug_UART_Receive_Start()
#include "co2.h"      // CO2_UART_Receive_Start() / CO2_get_data()
#include "dht22.h"    // DHT22_Init() / DHT22_ReadData()
#include "alarm.h"    // g_reset_key_pressed（SW1 复位标志）
#include "light.h"    // GetLux()
#include "soil.h"     // GetSoilHumidity()
#include "adc.h"      // hadc1，ADC 双通道 + DMA
#include "oled_ui.h"  // OLED_UI_Init() / OLED_UI_Refresh()
#include "lora_app.h" // LoRa应用层：定时上报 + 收网关指令
#include "stdio.h"    // printf

/* 光敏(ADC1_IN0) + 土壤(ADC1_IN1) 两路 ADC 的 DMA 目标缓冲。
 * 由 HAL_ADC_Start_DMA() 在循环模式下不停搬运，主循环直接读这两个值即可，
 * 不用每次自己去启动转换（见 light.c / soil.c 里的 GetLux / GetSoilHumidity）。 */
uint16_t adc1_values[2] = {0};

/*===========================================================================
 * 延时函数
 *   项目里统一用这三个：delay_ms() 是空循环软件延时，
 *   delay_us() 是借用 SysTick 计数器的硬件等待（用完会把 LOAD 还原，
 *   保证 SysTick 的 1ms 中断周期不受影响），delay_s() 直接转 HAL_Delay()。
 *
 *   ⚠ 这三个全是**阻塞**延时，只能在初始化阶段或者确实需要的地方用，
 *     不要放进主循环的定时任务里 —— 会把 LoRa 的收发节奏拖垮。
 *=========================================================================*/

/* STM32F103C8T6(72MHz) 专用毫秒级延时函数
 * 实测：1ms误差±0.02ms，1000ms误差±15ms，满足外设控制需求 */
void delay_ms(uint32_t ms)
{
    volatile uint32_t i, j;    // 72MHz主频精准校准值，一行修改完成适配，核心逻辑不变
    uint32_t calibrate = 8000; // 该值严格对应72MHz下约1ms延时

    for (j = 0; j < ms; j++)
    {
        for (i = 0; i < calibrate; i++)
        {
            // volatile变量保证空循环不被编译器优化，延时有效
            ;
        }
    }
}

void delay_us(uint32_t us)
{
    uint32_t save_LOAD;    // 备份原SysTick->LOAD值
    uint32_t target_ticks; // 目标us对应的SysTick重载值

    // 1. 备份原始LOAD值（核心！延时结束必须恢复）
    save_LOAD = READ_REG(SysTick->LOAD);

    // 2. 计算目标us对应的重载值：SystemCoreClock(Hz) → 1us对应计数次数
    target_ticks = (1UL * SystemCoreClock / 1000000) * us;

    // 3.关闭SysTick中断
    CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);

    // 4. 配置SysTick为「目标us计数模式」，启动计数
    if (target_ticks > 0)
        WRITE_REG(SysTick->LOAD, target_ticks - 1); // 重载值=计数值-1
    WRITE_REG(SysTick->VAL, 0UL);                   // 计数器清零，立刻开始计数

    // 5. 硬件等待：直到计数完成（COUNTFLAG置1）
    while (!READ_BIT(SysTick->CTRL, SysTick_CTRL_COUNTFLAG_Msk))
        ;

    // 6. 完全复原：恢复原始LOAD值，保证SysTick原理周期1ms不变
    WRITE_REG(SysTick->LOAD, save_LOAD);

    // 7. 恢复SysTick中断
    SET_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);
}

void delay_s(uint32_t s)
{
    // delay_ms(s * 1000);
    HAL_Delay(s * 1000);
}

/*===========================================================================
 * 业务主循环
 *   在 main.c 里被调用，替代 while(1) 写死的逻辑（见 Core/Src/main.c 末尾）。
 *   这个函数自己不会返回，返回也没意义。
 *=========================================================================*/
void app_main(void)
{
    /*------------------------------------------------------------------
     * 1. 先把各个输入的接收挂上
     *      这几个都是「挂上就返回」，不做任何等待，所以放在最前面：
     *      后面的 OLED 初始化虽然已经不会死等（见下面那段说明），
     *      但输入先挂上总归更稳妥，出问题时还能通过调试口看到日志。
     *----------------------------------------------------------------*/
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_values, 2); // 2 = 启用了 IN0、IN1 两路通道
    Debug_UART_Receive_Start(); // USART1：调试口，收到电脑发来的会原样回显
    CO2_UART_Receive_Start();   // USART2：CO2 传感器的 6 字节定长帧

    printf("\r\n==========================================\r\n");
    printf("智慧农业 - 终端节点（采集端）启动\r\n");
    printf("调试口 USART1 = 115200    CO2口 USART2 = 9600\r\n");
    printf("LoRa: 470.5MHz/SF9/BW125  OLED: I2C1 0x78\r\n");
    printf("==========================================\r\n");

    /*------------------------------------------------------------------
     * 2. 外设初始化
     *      这些全都是「配好就返回」，不会把启动流程卡住：
     *----------------------------------------------------------------*/
    DHT22_Init();  // DHT22 单总线，把数据脚置为空闲高电平
    alarm_init();  // 清报警状态、关蜂鸣器和联动设备
                   //   ⚠ alarm.c 目前只是被编译着，它的 alarm_process() 在主循环里
                   //     是注释掉的（见下面报警处理那一段），所以实际上不生效。
                   //     现在生效的是 beep.c 的 CO2 三档报警。

    /* OLED：初始化屏 + 把四行标签画一遍。
     *   ⚠ 这段现在**不会卡住启动**，不管屏插没插：
     *     oled.c 的 I2C 写是带超时的（OLED_I2C_TIMEOUT_MS=100ms），
     *     而且每次进来先探测一次（≤5ms），屏不在线就直接返回、一个字节都不发。
     *   这个坑踩过一次：原来用的是 HAL_MAX_DELAY，屏不在线就永久自旋，
     *   现象是串口只停在启动头两行、后面一句都不打，很容易误判成射频没起来。
     *   屏是运行中插上的也没关系 —— 主循环那边会探测到并补画标签。 */
    OLED_UI_Init();

    /* 光控 LED 的 PWM 输出（PA8-LED2 / PB15-LED3）。
     *   ⚠ PB15 走的是 TIM1 的**互补通道 CH3N**，必须由 pwm.c 里的
     *     HAL_TIMEx_PWMN_Start() 启动。主循环这边不要单独去调
     *     HAL_TIM_PWM_Start(..., TIM_CHANNEL_3)，那个对 CH3N 不起作用。 */
    LED_PWM_Init();

    /* 蜂鸣器 PWM 输出（PB9-TIM4_CH4，3kHz 无源），初始不响 */
    BEEP_Init();

    /* LoRa 应用层挂载。
     *   这里只是把状态机推到「等 1 秒」，真正的射频初始化是 LORA_AppService()
     *   自己按 LORA_INIT_RETRY_MS 重试的 —— 模块没插/没接好也不会卡在启动阶段。 */
    LORA_AppInit();

    /* 等各模块上电稳定再进主循环（DHT22 上电后要 1s 才稳定） */
    HAL_Delay(1000);

    /*------------------------------------------------------------------
     * 3. 各定时任务自己的计时基准
     *      上电时统一取一次当前 tick，之后各管各的，互不干扰。
     *----------------------------------------------------------------*/
    uint32_t tick_1S = HAL_GetTick();    // CO2 的计时基准
    uint32_t tick_2S = HAL_GetTick();    // DHT22 的计时基准
    uint32_t tick_light = HAL_GetTick(); // 光控 LED 的计时基准
    uint32_t tick_oled = HAL_GetTick();  // OLED 刷新的计时基准

    /* 主循环里要用的数据，全部在这里声明并给初值 */
    uint16_t co2_value = 0; // CO2 浓度值 ppm
    uint8_t co2_ok = 0;     // CO2 本次读数是否有效。
                            //   不能拿 ret 代替：ret 被下面 DHT22 那块覆盖了
    float temp = 0.0f;      // 温度值 ℃
    float humi = 0.0f;      // 湿度值 %RH
    uint8_t dht22_ok = 0;   // 温湿度数据是否可用（读成功后一直有效，直到下次读失败）
    uint16_t lux = 0;       // 光照强度值 lux
    uint8_t soil = 0;       // 土壤湿度等级 1~4
    uint8_t led_level = 0;  // 当前 LED 亮度等级：1暗 2中 3亮
    uint8_t beep_level = 0; // 当前蜂鸣器报警等级：0不响 1小声 2大声
    uint8_t ret = 0;        // 各驱动函数的返回值，后面用谁的结果就紧跟谁赋值
    OLED_UI_Data_t ui_data = {0}; // 一次刷新要显示的全部数据

    /*------------------------------------------------------------------
     * 4. 主循环
     *----------------------------------------------------------------*/
    while (1)
    {
        /*--------------------------------------------------------------
         * SW1(PB13) 报警复位键
         *   中断里只置标志位（见 ISR_callback.c），真正的取消动作放到这里做
         *   —— 中断里不做耗时操作，也不调 printf。这是本项目的既定模式。
         *------------------------------------------------------------*/
        if (g_reset_key_pressed)
        {
            g_reset_key_pressed = 0; // volatile变量，用完马上清零
            BEEP_Ack();              // 取消CO2蜂鸣器报警（静音自锁）
        }

        /*--------------------------------------------------------------
         * CO2 采集：1 秒一次
         *   超时给得短：CO2_get_data() 是死等循环，占住 1 秒会误了 LoRa 的
         *   收包和 ACK 窗口 —— 上报发完之后只有 LORA_ACK_WINDOW_MS(400ms)
         *   等中控回 ACK，拖不起。
         *------------------------------------------------------------*/
        if (HAL_GetTick() - tick_1S >= 1000)
        {
            tick_1S = HAL_GetTick();

            ret = CO2_get_data(&co2_value, 100); // 100ms 超时
            co2_ok = (ret == 0);                 // 记下本次读数是否有效，给OLED显示用

            // 不论读成功还是失败都要调一次：读失败时模块内部会自动静音，
            // 不用在外面单独处理（静音自锁的解除条件在 beep.c 里）
            beep_level = BEEP_CO2Alarm(co2_value, (ret == 0));

            if (ret)
            {
                printf("CO2传感器通信失败,ret = %d\n", ret);
            }
            else
            {
                printf("CO2浓度：%hu ppm, 报警等级：%d\n", co2_value, beep_level);
            }
        }

        /*--------------------------------------------------------------
         * DHT22 采集：2 秒一次
         *   DHT22 自己的采样率上限是 0.5Hz（2 秒一次），所以这里正好卡在
         *   它的上限上，不能再快了。
         *------------------------------------------------------------*/
        if (HAL_GetTick() - tick_2S >= 2000)
        {
            tick_2S = HAL_GetTick();

            ret = DHT22_ReadData(&temp, &humi);
            dht22_ok = (ret == 0); // 0=成功 1=响应超时 2=接收超时 3=校验失败（详见 dht22.h）
            if (dht22_ok)
            {
                printf("温度: %.1f, 湿度: %.1f\n", temp, humi);
            }
            else
            {
                printf("读 DHT22 失败!\n");
            }
        }

        /*--------------------------------------------------------------
         * 光控 LED：读光照强度 → 自动切换 LED 亮度档位（环境越暗，灯越亮）
         *   这里单独用 tick_light 计时，不能和上面共用 tick_1S ——
         *   CO2 那个块每次都会把 tick_1S 重置成当前时刻，共用的话这个块几乎不会执行
         *   分档：<10lux 亮(导通100%) / 10~44lux 中(50%) / >=45lux 暗(15%)
         *   阈值和占空比都在 pwm.h / pwm.c 里定义，改宏即可调整
         *   土壤等级顺手一起读：它和光敏共用 ADC1 的 DMA，不额外花时间
         *------------------------------------------------------------*/
        if (HAL_GetTick() - tick_light >= 500)
        {
            tick_light = HAL_GetTick();

            lux = GetLux();
            soil = GetSoilHumidity();
            led_level = LED_LightCtrl(lux); // 按光照强度自动切档
            printf("光照强度：%d lux, 土壤湿度等级：%d, LED等级：%d\n",
                   lux, soil, led_level);
        }

        /*--------------------------------------------------------------
         * OLED 数据上屏：把这一轮采到的 5 个值刷到屏上
         *   同样要独立计时 —— 上面每个块都会重置自己的 tick，
         *   复用别的 tick 的话这个块几乎不会被执行
         *   标签在 OLED_UI_Init() 里已经画好了，这里只重画数值区、不清屏
         *   刷新周期改 oled_ui.h 里的 OLED_UI_PERIOD_MS
         *------------------------------------------------------------*/
        if (HAL_GetTick() - tick_oled >= OLED_UI_PERIOD_MS)
        {
            tick_oled = HAL_GetTick();

            ui_data.temp = temp;
            ui_data.humi = humi;
            ui_data.dht_ok = dht22_ok;
            ui_data.lux = lux;
            ui_data.soil = soil;
            ui_data.co2 = co2_value;
            ui_data.co2_ok = co2_ok;

            OLED_UI_Refresh(&ui_data);
        }

        /*--------------------------------------------------------------
         * 报警处理：每次循环都跑一遍，保证复位键能被及时响应、计时准确
         *   温湿度读到 → 用新值判断；读失败 → dht22_ok=0，
         *   alarm_process() 收到 data_valid=0 会跳过本轮，避免用旧值误报警
         *
         *   ⚠ 这一句是**故意注释掉的**，不是忘了删：
         *     现在实际生效的是 beep.c 的 CO2 三档报警（上面那块），
         *     alarm.c 是更早的一套阈值表 + 联动风机/遮阳帘逻辑，
         *     而且它用的 BUZZER/FAN/CURTAIN 三个引脚（PB9/PA8/PB15）
         *     现在都改成了 PWM 输出，它里面的 HAL_GPIO_WritePin 已经失效。
         *     两套逻辑同时开会互相打架，所以先留着不启用。
         *     要启用得先按 beep.c 的写法把输出改成 PWM。
         *------------------------------------------------------------*/
        // alarm_process(temp, humi, co2_value, dht22_ok);

        /*--------------------------------------------------------------
         * LoRa 上报：每 LORA_UPLOAD_PERIOD_MS(2000ms) 一轮
         *   每轮先开 LORA_RX_WINDOW_MS(500ms) 接收窗口等中控下发的指令，
         *   没等到就发一包 8 字节传感器数据，发完再开 LORA_ACK_WINDOW_MS(400ms)
         *   短窗口等中控的 ACK。
         *   收发都不在这里死等 —— 使能之后立刻返回，靠 DIO1 中断(PB10)置的
         *   标志位推进状态机，所以不会拖慢上面任何一块。
         *   周期 / 两个窗口 / 帧格式都在 lora_app.h 里改，改了两端要一起改。
         *
         *   注意这一轮的传感器数据是 LoRa 应用层自己去读的（DHT22/光敏/土壤），
         *   和上面主循环里那几个定时块各走各的、互不知情。功能上没问题
         *   （DHT22 支持 0.5Hz 采样），但两处读数可能差一个采样周期。
         *------------------------------------------------------------*/
        LORA_AppService();
    }
}

/*===========================================================================
 * gpio_test() —— 所有 GPIO 的点灯自检（教学用，主循环里没有调用）
 *
 * 用法：把 Core/Src/main.c 里 USER CODE BEGIN SysInit 段那句
 *       //gpio_test();  的注释去掉即可。它只跑一遍就返回，
 *       返回后 main() 会继续往下走、进 app_main()。
 *       只想停在这里看灯、不进业务逻辑的话，自己在末尾加个 while(1)。
 *
 * ⚠ 这份代码是从最早的工程一路传下来的，**其中好几个引脚现在已经被外设占用了**，
 *   再用 HAL_GPIO_WritePin 写它们是没有任何效果的（引脚由定时器/外设驱动）。
 *   下面按「还能不能当普通 GPIO 用」逐个标了出来。
 *=========================================================================*/
void gpio_test(void)
{
    GPIO_InitTypeDef GPIO_Init;

    /* 1. 开 GPIO 端口时钟。
     *    这一步不能省：HAL 里不打开外设时钟就去配引脚，配置写不进去，
     *    现象是「代码跑了但引脚没反应」，很难查。 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 2. 先把要复用的参数填一次，下面每个引脚只改 .Pin。
     *    Mode=推挽输出：能输出高电平也能输出低电平（开漏就只能拉低）。
     *    Pull 这个成员在推挽输出下不起作用（只有输入/开漏才用得上），
     *    填 PULLUP / NOPULL 都一样，这里保留原样。
     *    下面这两句和直接写寄存器 GPIOC_CRH / GPIOC_ODR 是一回事，
     *    HAL 只是把「移位 + 清位 + 置位」这一套封装成了函数调用。 */
    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;   // 推挽输出
    GPIO_Init.Pin = GPIO_PIN_13;            // 引脚编号13
    GPIO_Init.Pull = GPIO_PULLUP;           // 上拉（推挽模式下该成员不起作用）
    GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH; // 输出速度：高速 50MHz

    /* PC13 —— 板载 LED，高电平亮。✓ 这个还是普通 GPIO 输出，写了有效 */
    HAL_GPIO_Init(GPIOC, &GPIO_Init);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // SET = 1 = 高电平

    /* 下面这几条都**已经失效了**，留着是为了看清「怎么配 GPIO」的写法，
     * 真要用这些灯/蜂鸣器请走对应模块的接口，别在这里写电平：
     *
     *   PA8   → TIM1_CH1   光控 LED    ✗ 改用 pwm.c 的 LED_xxx
     *   PB15  → TIM1_CH3N  光控 LED    ✗ 改用 pwm.c 的 LED_xxx
     *   PB9   → TIM4_CH4   蜂鸣器      ✗ 改用 beep.c 的 BEEP_xxx
     */

    // PA8-LED2 低电平亮（✗ 现在是 TIM1_CH1 的 PWM 输出）
    GPIO_Init.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &GPIO_Init);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

    // PB15-LED3 低电平亮（✗ 现在是 TIM1_CH3N 的 PWM 输出）
    GPIO_Init.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &GPIO_Init);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

    // PB9-BEEP 高电平响（✗ 现在是 TIM4_CH4 的 PWM 输出，改占空比才控制响度）
    GPIO_Init.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &GPIO_Init);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

    /* PC14 —— 继电器 RLY1，高电平通电
     * ⚠ 这里原来是 GPIO_PIN_15，是个笔误：初始化的是 PC15、操作的却是 PC14，
     *   结果 PC14 从来没被配成输出，下面那句 WritePin 写了也不生效。已改回 14。
     *   这个笔误还有隐患：PC15 正是 DHT22 的数据脚，被配成推挽输出以后
     *   会把 DHT22 的单总线驱动起来，真调这个函数会干扰温湿度读取。
     * ⚠ 就算改对了也不生效 —— PC14 没有在 CubeMX 里配置（TMPLATE.ioc 里
     *   没有这个引脚）。要用继电器得先在 CubeMX 里把 PC14 设成 GPIO_Output，
     *   重新生成工程，再来调这个函数。 */
    GPIO_Init.Pin = GPIO_PIN_14;
    HAL_GPIO_Init(GPIOC, &GPIO_Init);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET); // SET = 1 = 通电
}
