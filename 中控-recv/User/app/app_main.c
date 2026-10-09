/**
 * @file    app_main.c
 * @brief   中控（接收端）业务主循环
 *
 * ============================================================================
 * 这块板子的角色：**中控 / 接收端**
 *
 *   和终端板是同一块 PCB、同一套引脚（.ioc 只差三个引脚，见下），
 *   区别只在烧进去的固件角色：这块跑「收」，终端那块跑「发」。
 *
 *   .ioc 与终端工程的差异（就这三处，其余完全一致）：
 *     PA0(ADC1_IN0 光敏) / PA1(ADC1_IN1 土壤) —— 中控板上没这两个传感器，删了
 *     PC15(DHT22 单总线)                      —— 同上，删了
 *     于是 ADC1 + DMA 这两个外设也一并从工程里去掉
 *
 * 引脚分工：
 *   USART1  PA9 / PA10   115200  调试口，经 CH340K 接电脑，printf 走这里
 *   USART2  PA2 / PA3    115200  ESP8266-12E WiFi 模块（巴法云 TCP 透传）
 *                                 ⚠ 这两根在上一个工程里是接 CO2 的（9600），
 *                                    改接 WiFi 模块后波特率必须换成 115200
 *   SPI1    PA5/PA6/PA7          LLCC68 LoRa 模块 SCK/MISO/MOSI
 *           PA4=NSS  PB1=RST  PB0=BUSY  PB10=DIO1(EXTI10)
 *   I2C1    PB6 / PB7    100kHz  OLED（SSD1306 128x64，地址 0x78）
 *   TIM1    PA8 / PB15           CH1 + CH3N，LED 光控 PWM
 *   TIM4    PB9                  CH4，蜂鸣器 PWM
 *   EXTI    PB12 / PB13          两个按键
 *
 * 终端上的 CO2 / DHT22 / 光敏 / 土壤这几个外设已经移出 Keil 工程不再编译
 * （对应的 .c 文件还留在 User/bsp 下，只是不参与构建）。
 *
 * ----------------------------------------------------------------------------
 * 主循环干三件事（按代码里的先后顺序；第 2 件的连接阶段会阻塞，见下）：
 *
 *   1. LoRa 常驻接收（lora_app.c）
 *        排在最前面是为了让 ACK 发得最快。中控常驻接收态，
 *        终端什么时候上报就什么时候收：
 *        收到 → 校验帧头 → 解析 → 更新 g_lora_rx_frame → 回一个 ACK → 挂回接收。
 *        收和发都不在这里死等，靠 DIO1(PB10) 中断置的标志位推进状态机。
 *
 *   2. MQTT 上云（mqtt_app.c）
 *        把第 1 步收到的终端数据发到巴法云，手机小程序上就能看到。
 *        内部顺序：连 WiFi+TCP → MQTT 连接 → 订阅五个主题 → 每 5 秒发布一次
 *                  → 每 60 秒心跳一次，心跳失败自动重连。
 *        发布的是 g_lora_rx_frame 里**最新那一包**（终端发来的真实数据），
 *        终端 2.5 秒一包、这边 5 秒一发，中间那包会被跳过。
 *        连不上云不影响 LoRa 收数据和 OLED 上屏 —— 这是中控的本职工作。
 *        ⚠ 连接阶段是阻塞的（ESP_Init() 的锅，最坏二十几秒），
 *          期间会漏包、ACK 会迟发，这是刻意取舍，账记在 mqtt_app.c 文件头。
 *
 *   3. OLED 上屏（oled_ui.c）
 *        收到新的一包就刷（用 g_lora_rx_frame.cnt 判断），另外每
 *        OLED_REFRESH_PERIOD_MS 兜底刷一次。刷新节奏跟着终端的
 *        LORA_UPLOAD_PERIOD_MS(2.5s) 走，不在两次上报中间做无用的白刷。
 *        ⚠ 屏上显示的是**终端发来的数据**，不是这块板子自己测的。
 *          还没收到过任何包时，温湿度显示 "--"、CO2 显示 "---"；
 *          收到包但终端那个传感器没读到时，对应那项也显示占位符
 *          （温湿度 "--"、CO2 "---"）。
 *        ⚠ 屏插不插、运行中拔不拔都不影响 LoRa 和 WiFi —— 见下面第 5 条。
 *
 *   4. LoRa / MQTT 都排在 OLED 前面，是为了让 ACK 发得最快；OLED 刷一遍
 *      数值区约 60ms，排在前面的话这 60ms 会加在 ACK 的响应延迟上。
 *
 *   5. OLED 热插拔（oled.c 内部处理，这里不用管）
 *        没插屏 → 每次刷新前一次探测（≤5ms）就返回，一个 I2C 都不发
 *        中途拔掉 → 写 I2C 失败即刻置离线，最迟下一轮刷新发现
 *        重新插上 → 探测到在线，自动重新初始化 + 重画标签
 *
 * ⚠ 每个定时任务都用**自己的**计时基准（这里是 tick_oled；WiFi/MQTT 那条链路的
 *   在 mqtt_app.c 里的 s_tick_retry / s_tick_heart / s_tick_pub），绝对不要共用：
 *   共用的话谁先跑到就把 tick 重置成当前时刻，排在后面的块几乎永远不会执行。
 *   这个坑在本工作区踩过一次（tick_1S 被光控块和 OLED 块抢用）。
 * ============================================================================
 */

#include "app_main.h"
#include "main.h"
#include "pwm.h"      // LED_PWM_Init()：光控 LED 的 PWM 输出
#include "debug.h"    // Debug_UART_Receive_Start()：USART1 调试口接收
#include "oled_ui.h"  // OLED_UI_Init() / OLED_UI_Refresh() / OLED_UI_Data_t
#include "lora_app.h" // LORA_AppInit() / LORA_AppService() / g_lora_rx_frame
#include "esp.h"      // ESP_UART_Receive_Start()：USART2 的接收中断（MQTT 上云层用）
#include "stdio.h"    // printf
#include "mqtt_app.h" // MQTT_AppInit() / MQTT_AppService()：把收到的数据上云（巴法云）

/* 光敏 / 土壤两个模块用的 ADC 缓冲。
 * 这两个模块（bsp/light-R/light.c、bsp/soil/soil.c）已经移出 Keil 工程不再编译，
 * 这一行现在是空占位 —— 留着是为了以后要把它们加回来时不用再动 app_main.h。 */
uint16_t adc1_values[2] = {0};

/*===========================================================================
 * OLED 兜底刷新周期（毫秒）
 *
 *   正常情况下屏是「收到新包就刷」的，这个值只在两种时候起作用：
 *     ① 还没收到过任何包 —— 得有个节奏把 "--" 画上去
 *     ② 屏被拔掉又插回来 —— 在线探测需要有人周期性地来调一次
 *
 *   取 2500ms 是为了跟**终端的上报周期 LORA_UPLOAD_PERIOD_MS(2.5s) 对齐**：
 *   两边节奏一致，屏上不会出现「刷了但数据没变」的空刷。
 *
 *   ⚠ 改终端的上报周期时，这里要跟着改（两个数不需要严格相等，
 *     但兜底周期明显小于上报周期的话，中间那几次刷新都是白刷）。
 *   注意和 oled_ui.h 里的 OLED_UI_PERIOD_MS 不是一回事：那是终端本地
 *   传感器上屏用的 500ms，中控不走那个值。
 *=========================================================================*/
#define OLED_REFRESH_PERIOD_MS (2500U)

/*===========================================================================
 * 延时函数
 *   项目里统一用这三个：delay_ms() 是空循环软件延时，
 *   delay_us() 是借用 SysTick 计数器的硬件等待（用完会把 LOAD 还原，
 *   保证 SysTick 的 1ms 中断周期不受影响），delay_s() 直接转 HAL_Delay()。
 *
 *   ⚠ 这三个全是**阻塞**延时，只能在初始化阶段或者确实需要的地方用，
 *     不要放进主循环的定时任务里 —— 会把 LoRa 的收包节奏拖垮。
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
     * 1. 先把两个串口的接收中断挂上
     *      这两个函数是「挂上就返回」，不做任何等待，所以放在最前面：
     *      万一后面 OLED 没接好、卡在 OLED_UI_Init() 里，
     *      串口的接收也不会受影响，还能通过调试口看到日志。
     *----------------------------------------------------------------*/
    Debug_UART_Receive_Start(); // USART1：调试口，收到电脑发来的会原样回显
    ESP_UART_Receive_Start();   // USART2：ESP8266 的 AT 回显全靠它收

    printf("\r\n==========================================\r\n");
    printf("智慧农业 - 中控（接收端）启动\r\n");
    printf("调试口 USART1 = 115200    WiFi口 USART2 = 115200\r\n");
    printf("LoRa: 470.5MHz/SF9/BW125  OLED: I2C1 0x78\r\n");
    printf("==========================================\r\n");

    /*------------------------------------------------------------------
     * 2. 外设初始化
     *      这两个都是「配好就返回」，不会把启动流程卡住：
     *----------------------------------------------------------------*/

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
     *     HAL_TIM_PWM_Start(..., TIM_CHANNEL_3)，那个对 CH3N 不起作用。
     *   中控不读光敏，所以灯就停在 LED_PWM_Init() 设的那个档位不再变；
     *   想让中控本地也做光控，把 light.c / pwm.c 的 LED_LightCtrl() 加回来。 */
    LED_PWM_Init();

    /* LoRa 应用层挂载。
     *   这里只是把状态机推到「等 1 秒」，真正的射频初始化是 LORA_AppService()
     *   自己按 LORA_INIT_RETRY_MS 重试的 —— 模块没插/没接好也不会卡在启动阶段。 */
    LORA_AppInit();

    /* MQTT 上云层挂载。
     *   同样只是挂个计时基准，不在这里连网 —— 真正的连接是
     *   MQTT_AppService() 按 MQTT_RETRY_PERIOD_MS 在后台重试的，
     *   WiFi 连不上也**不会卡在启动阶段**（这是和原厂 mqtt_demo() 最大的区别：
     *   那个函数体里是 while(1)，挂进来后面的初始化一句都执行不到）。 */
    MQTT_AppInit();

    /*------------------------------------------------------------------
     * 3. 各定时任务自己的计时基准
     *      上电时统一取一次当前 tick，之后各管各的，互不干扰。
     *      ⚠ WiFi / MQTT 那条链路的重试计时基准不在这里 ——
     *        它在 mqtt_app.c 内部自己管（s_tick_retry），这里不用操心。
     *----------------------------------------------------------------*/
    uint32_t tick_oled = HAL_GetTick(); // OLED 兜底刷新的计时基准
    uint32_t last_rx_cnt = 0;           // 上次刷新时 g_lora_rx_frame.cnt 的值
    OLED_UI_Data_t ui_data = {0};       // 一次刷新要显示的全部数据

    /*------------------------------------------------------------------
     * 4. 主循环
     *----------------------------------------------------------------*/
    while (1)
    {
        /*--------------------------------------------------------------
         * 1) LoRa 常驻接收 —— 放在最前面，让 ACK 发得最快
         *   中控常驻在接收态（LLCC68_RX_CONTINUOUS），收完一包芯片不会退出 RX，
         *   处理完这一包再挂回去即可。
         *   收到 → 校验帧头 → 解析 → 更新 g_lora_rx_frame（下面 OLED 取它）
         *        → 回一个 ACK → 挂回接收
         *   内部靠 DIO1(PB10) 中断推进状态机，使能之后立刻返回，不死等。
         *
         *   为什么排第一：终端发完包只留 400ms 等 ACK（LORA_ACK_WINDOW_MS），
         *   而 OLED 刷一遍数值区要约 60ms。排到后面的话，这 60ms 会加在
         *   ACK 的响应延迟上；排第一则收到就能马上回。
         *
         *   ⚠ 主循环里不要加会长时间阻塞的调用（比如带秒级超时的串口等待）：
         *     被占住期间收包标志不会丢（中断已经置上了），但芯片没法及时挂回
         *     接收，后面紧跟着的包就漏了，ACK 也会迟发。
         *
         *   ACK 开关、帧格式都在 lora_app.h 里改，和终端工程必须保持一致。
         *------------------------------------------------------------*/
        LORA_AppService();

        /*--------------------------------------------------------------
         * 2) MQTT 上云（巴法云）
         *   连 WiFi+TCP → MQTT 连接 → 订阅三个主题 → 每 5 秒发布一次
         *   → 每 60 秒心跳一次，心跳失败自动整条重连。
         *   这些全在 mqtt_app.c 内部，这里只负责按节奏调它一次
         *   （和 LoRa 一样是个非阻塞的状态机，不会赖在循环里不出来）。
         *
         *   发上去的就是 g_lora_rx_frame 里最新那一包（终端发来的真实数据），
         *   五个主题各收一项（主题名、格式、失效时发什么见 mqtt_app.h 顶部表）：
         *     Humidity    ← #<空气湿度 %RH>
         *     Temperature ← #<温度 ℃>
         *     light       ← #<光照 lux>
         *     soil        ← #<土壤等级 1~4>
         *     co2         ← #<CO2 ppm>
         *   终端 2.5 秒一包、这边 5 秒一发，所以中间那包会被跳过。
         *
         *   ⚠ 这里没有写成 while(ESP_Init()){}：
         *     那样写的话 WiFi 一失败就永远出不来，后面的 OLED 和 LoRa
         *     一行都不会执行，整块板子等于废了。
         *     （原厂 mqtt_demo() 就是这个问题，已经从工程里移出去了。）
         *
         *   ⚠ 但要如实说清代价：连接阶段是**阻塞**的，最坏二十几秒
         *     （ESP_Init 十几秒 + MQTT 连接和五个主题订阅各 2s，最坏约 27s）。
         *     这期间 DIO1 中断照样在置标志位、不会丢，
         *     但芯片处理完上一包后没法及时挂回接收，**会漏包、ACK 会迟发**。
         *     这是刻意取舍 —— 宁可收到包慢一点，也不能让云端连不上就把板子卡死。
         *     连上之后就不再进这个分支了，心跳正常只需要几十毫秒。
         *------------------------------------------------------------*/
        MQTT_AppService();
				
        /*--------------------------------------------------------------
         * 3) OLED 上屏
         *   显示的是 LoRa 收到的终端数据，不是本机传感器。
         *   标签在 OLED_UI_Init() / 热插拔恢复时已经画好了，
         *   这里只重画数值区、不清屏。
         *   g_lora_rx_frame 由 LORA_AppService() 收到包时更新：
         *     valid=0（还没收到过任何包）或终端那个传感器没读到时，
         *     温湿度显示 "--"（DHT22 没读到）、CO2 显示 "---"（CO2 传感器没读到）
         *
         *   刷新时机 = 「收到新包」或「兜底周期到」，二选一。
         *     收到就刷：屏上永远是最新一包，而且不会在两次上报中间白刷
         *               （终端每 2.5s 才来一包，单纯按 2.5s 定时刷的话，
         *                两边的时钟各走各的，最坏会差出整整一个周期才更新）
         *     兜底刷新：处理「还没收到过包」和「屏拔了又插回来」两个场景
         *   两个周期的关系见文件头的 OLED_REFRESH_PERIOD_MS 说明
         *
         *   屏在不在线由 oled_ui.c 内部处理：不在线就直接 return，
         *   这里照常调用即可，不用自己判断。
         *------------------------------------------------------------*/
        if ((g_lora_rx_frame.cnt != last_rx_cnt) ||
            (HAL_GetTick() - tick_oled >= OLED_REFRESH_PERIOD_MS))
        {
            last_rx_cnt = g_lora_rx_frame.cnt;
            tick_oled = HAL_GetTick();

            ui_data.temp = (float)g_lora_rx_frame.temp10 / 10.0f; // 0.1℃ → ℃
            ui_data.humi = (float)g_lora_rx_frame.humi;
            /* 两个条件都满足才显示真实值：
             *   ① 收到过有效帧  ② 对方 DHT22 读数有效（不是 LORA_TEMP_INVALID） */
            ui_data.dht_ok = g_lora_rx_frame.valid &&
                             (g_lora_rx_frame.temp10 != LORA_TEMP_INVALID);
            /* 光照是 int16，屏上按 uint16 显示；负值（理论上不该出现）按 0 处理 */
            ui_data.lux = (uint16_t)((g_lora_rx_frame.lux > 0) ? g_lora_rx_frame.lux : 0);
            ui_data.soil = g_lora_rx_frame.soil;
            ui_data.co2 = g_lora_rx_frame.co2;
            /* CO2 和温湿度一样看两个条件：
             *   ① 收到过有效帧  ② 对方 CO2 传感器读到了（不是 LORA_CO2_INVALID） */
            ui_data.co2_ok = g_lora_rx_frame.valid &&
                             (g_lora_rx_frame.co2 != LORA_CO2_INVALID);

            OLED_UI_Refresh(&ui_data);
        }
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
 * ⚠ 这份代码是从最早的工程一路传下来的，**下面这些引脚现在已经被外设占用了**，
 *   再用 HAL_GPIO_WritePin 写它们是没有任何效果的（引脚由定时器/外设驱动）：
 *     PA8   → TIM1_CH1   光控 LED      要用请走 pwm.c 的 LED_xxx 接口
 *     PB15  → TIM1_CH3N  光控 LED      同上
 *     PB9   → TIM4_CH4   蜂鸣器        要用请走 beep.c 的 BEEP_xxx 接口
 *     PA4   → LoRa NSS   软件片选      乱写会把 LoRa 通信直接打断
 *     PB1   → LoRa RST   模块复位脚    乱写会把模块复位掉
 *     PA5/6/7 → SPI1     SCK/MISO/MOSI 已由 SPI 外设接管
 *   所以下面只写**中控板上确实还是普通 GPIO 输出**的两个脚。
 *=========================================================================*/
void gpio_test(void)
{
    GPIO_InitTypeDef GPIO_Init;

    /* 1. 开 GPIO 端口时钟。
     *    这一步不能省：HAL 里不打开外设时钟就去配引脚，配置写不进去，
     *    现象是「代码跑了但引脚没反应」，很难查。 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* 2. 先把要复用的参数填一次，下面每个引脚只改 .Pin。
     *    Mode=推挽输出：能输出高电平也能输出低电平（开漏就只能拉低）。
     *    Pull 这个成员在推挽输出下不起作用（只有输入/开漏才用得上），填 NOPULL 即可。 */
    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;   // 推挽输出
    GPIO_Init.Pull = GPIO_NOPULL;           // 推挽模式下该成员不起作用
    GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH; // 输出速度：高速 50MHz

    /* PC13 —— 板载 LED，高电平亮 */
    // 下面这两句和直接写寄存器 GPIOC_CRH / GPIOC_ODR 是一回事，
    // HAL 只是把「移位 + 清位 + 置位」这一套封装成了函数调用
    GPIO_Init.Pin = GPIO_PIN_13;            // 引脚编号13
    HAL_GPIO_Init(GPIOC, &GPIO_Init);       // 把配置写进寄存器
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // SET = 1 = 高电平

    /* PC14 —— 继电器 RLY1，高电平通电
     * ⚠ 这里原来是 GPIO_PIN_15，是个笔误：初始化的是 PC15，操作的却是 PC14，
     *   结果 PC14 从来没被配成输出，下面那句 WritePin 写了也不生效。已改回 14。
     * ⚠ PC14 没有在 CubeMX 里配置（TMPLATE.ioc 里没有这个引脚），
     *   所以这两句虽然语法上能跑，实际也是无效的。要用继电器得先：
     *   CubeMX 里把 PC14 设成 GPIO_Output → 重新生成 → 再来调这个函数。 */
    GPIO_Init.Pin = GPIO_PIN_14;
    HAL_GPIO_Init(GPIOC, &GPIO_Init);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
}
