#ifndef OLED_UI_H // 防止头文件重复包含
#define OLED_UI_H

#include "main.h"

/*=======================================================================
 * 数据上屏：把外设监测到的实时值刷到 OLED 上
 *   屏    ：SSD1306 128x64
 *   接口  ：I2C1（PB6 -> SCL，PB7 -> SDA），从机地址 0x78，100kHz
 *   驱动  ：User/oled/oled.c（OLED_Init / OLED_ShowString / OLED_Clear）
 *   主循环调用方式见 User/app/app_main.c
 *
 *   分屏布局（16 点阵，行高 16 像素，128x64 正好放 4 行）：
 *     y=0    温【x=16 起 6 字符】   湿【x=80 起 6 字符】
 *     y=16   光照强度: 【值从 x=72 起，7 字符】
 *     y=32   CO2浓度:  【值从 x=64 起，8 字符】
 *     y=48   土壤湿度: 【值从 x=72 起，5 字符】
 *   汉字 16 像素宽、ASCII 8 像素宽，上面每行「标签 + 定长数值」都不超过 128。
 *   四行的 y 都取 8 的倍数，正好落在 SSD1306 的页边界上，不用动显示偏移。
 *
 *   为什么不整屏清屏重画：
 *     屏上没有显存，每次显示都是直接写 I2C。16 点阵一个字符要 4 次 I2C
 *     事务约 1.8ms，整屏清一次（1024 字节）要 300ms 左右，太亏。
 *     所以标签只在 OLED_UI_Init() 里画一次，OLED_UI_Refresh() 只重画数值区，
 *     每个值右补空格到定长，写出去的空格正好盖掉上一轮的旧字符，不留残影。
 *
 *   ⚠ 能显示哪些汉字由 oledfont.h 的 g_font_dot_matrix_16_index[] 决定，
 *     当前只有 17 个：您 好 ， 陈 工 黄 国 一 温 度 湿 浓 土 壤 光 照 强。
 *     字库里没有的字会显示成方框，要加字得先用 PortHelper 取模，
 *     再把字符串和 32 字节点阵数据一起补进 _16_index / g_font_dot_matrix_16。
 *
 *   热插拔（屏插不插都不影响功能）：
 *     整块屏是不是「在线」由 oled.c 自己判断（写失败就置离线，刷新前自动探测）。
 *     这一层只做两件事：① 不在线就直接 return，一次 I2C 都不发；
 *                       ② 探测到重新上线时补画一次标签（屏刚上电是空的）。
 *     所以本文件的使用者不用关心屏在不在 —— 照常调 OLED_UI_Refresh() 即可。
 *     ⚠ 唯一的例外：热插拔重新上线时 OLED_Init() 会等 200ms 电源稳定、
 *       再走一遍寄存器配置 + 清屏（约 40ms I2C），这一下是躲不掉的，
 *       属于「插屏」这个动作的一次性代价。详见 oled.c 的 OLED_Init()。
 *=======================================================================*/

/* 字体高度。必须和 oledfont.h 里的 FONT_SIZE_16 保持一致。
 * 这里不 #include "oledfont.h"：那个头文件里放的是数组的「定义」而不是声明
 * （Ferror / F8X16 / g_font_dot_matrix_16 等都是非 static 的 const 数组），
 * 目前只有 oled.c 一个 .c 包含它；再被第二个 .c 包含会符号重复定义 */
#define OLED_UI_FONT_SIZE (16)

/* 刷新周期（ms）。数值区刷一遍约 60ms，500ms 一刷约占 12% 的 CPU；
 * 嫌刷得慢或者想省 CPU，改这一个值即可 */
#define OLED_UI_PERIOD_MS (500)

/* 四行的 y 坐标（16 点阵行高 16 像素，必须取 8 的倍数） */
#define OLED_UI_LINE1_Y (0)
#define OLED_UI_LINE2_Y (16)
#define OLED_UI_LINE3_Y (32)
#define OLED_UI_LINE4_Y (48)

/* 一次刷新的全部数据。
 * 用结构体而不是 7 个位置参数：温度/湿度都是 float、两个有效标志都是
 * uint8_t，位置参数传反了编译器不会报错，用结构体赋值就一目了然 */
typedef struct
{
    float temp;     /* 温度 ℃                        */
    float humi;     /* 空气湿度 %                    */
    uint8_t dht_ok; /* 温湿度是否有效，0 时显示 "--"  */
    uint16_t lux;   /* 光照强度 lux                  */
    uint8_t soil;   /* 土壤湿度等级 1~4              */
    uint16_t co2;   /* CO2 浓度 ppm                  */
    uint8_t co2_ok; /* CO2 是否有效，0 时显示 "---"   */
} OLED_UI_Data_t;

/* 初始化 OLED 并画一次四行标签，上电后调一次即可 */
void OLED_UI_Init(void);

/* 刷新四行数值区：不重画标签，也不清屏
 * d：本次要显示的数据。某项读失败就把对应的 ok 置 0，会显示占位符
 *    （温湿度 "--"、CO2 "---"），免得把上一轮的旧值当成新值看 */
void OLED_UI_Refresh(const OLED_UI_Data_t *d);

#endif
