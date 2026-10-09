#include "oled_ui.h"
#include "oled.h" // OLED_Init / OLED_ShowString / OLED_Clear
#include <stdio.h>  // sprintf
#include <string.h> // strlen

/* 数值区的起始 x 坐标。按「标签占多宽」算出来的：
 *   温(16) 湿(16) 是单个汉字；"光照强度:"(4*16+8=72)；"CO2浓度:"(3*8+2*16+8=64) */
#define VAL_X_TEMP (16) /* 跟在 "温" 后面，x=0~15 被占 */
#define VAL_X_HUMI (80) /* 跟在 "湿" 后面，x=64~79 被占 */
#define VAL_X_LUX (72)  /* 跟在 "光照强度:" 后面 */
#define VAL_X_CO2 (64)  /* 跟在 "CO2浓度:" 后面 */
#define VAL_X_SOIL (72) /* 跟在 "土壤湿度:" 后面 */

/* 数值区的定长（字符个数）。取值上界见 oled_ui.h 的布局说明 */
#define VAL_W_TEMP (6) /* "24.5C" / "-10.5C"   */
#define VAL_W_HUMI (6) /* "65.2%" / "100.0%"   */
#define VAL_W_LUX (7)  /* "0 lux" / "350 lux"  */
#define VAL_W_CO2 (8)  /* "850ppm" / "65535ppm"*/
#define VAL_W_SOIL (5) /* "2/4"                */

/**
 * @brief  把字符串右补空格到指定宽度
 *
 * 补出来的空格写到屏上正好盖掉上一轮的旧字符，这样不用清屏也不会留残影。
 * 已经比 width 长就不动它（宁可显示长一点，也不能越界写 buf）
 *
 * @param  buf   待补齐的字符串，必须有 width+1 字节的空间
 * @param  width 目标宽度（字符个数，不是字节数）
 */
static void pad_to(char *buf, uint8_t width)
{
    uint8_t len = (uint8_t)strlen(buf);

    while (len < width)
    {
        buf[len] = ' ';
        len++;
    }
    buf[len] = '\0';
}

/**
 * @brief  画四行标签
 *
 *   抽成单独的函数是因为它有两个调用点：
 *     OLED_UI_Init()          开机画一次
 *     oled_ui_on_hotplug()    运行中把屏重新插上时补画一次
 *   屏刚上电时显存是空的，不补画的话屏上只剩数值、没有标签。
 */
static void oled_ui_draw_labels(void)
{
    OLED_ShowString(0, OLED_UI_LINE1_Y, "温", OLED_UI_FONT_SIZE);
    OLED_ShowString(64, OLED_UI_LINE1_Y, "湿", OLED_UI_FONT_SIZE);
    OLED_ShowString(0, OLED_UI_LINE2_Y, "光照强度:", OLED_UI_FONT_SIZE);
    OLED_ShowString(0, OLED_UI_LINE3_Y, "CO2浓度:", OLED_UI_FONT_SIZE);
    OLED_ShowString(0, OLED_UI_LINE4_Y, "土壤湿度:", OLED_UI_FONT_SIZE);
}

/**
 * @brief  屏刚被插上（或第一次插上）时的恢复动作
 *
 *   屏是刚上电的，寄存器还没配、显存是空的，所以要重新初始化 + 补画标签。
 *   由 OLED_UI_Refresh() 在探测到「从不在线变成在线」时调用。
 */
static void oled_ui_on_hotplug(void)
{
    OLED_Init(); /* 内部自己会探测；不在线就直接返回 */

    if (!OLED_IsOnline())
        return;

    oled_ui_draw_labels();
}

void OLED_UI_Init(void)
{
    OLED_Init(); /* 内部已经调过 OLED_Clear()，整屏是干净的；屏不在线就直接返回 */

    /* 屏不在线就什么都不画。不用在这里等 —— 以后插上了，
     * OLED_UI_Refresh() 会探测到并调 oled_ui_on_hotplug() 补上标签 */
    if (!OLED_IsOnline())
        return;

    /* 四行标签只画这一次，之后 OLED_UI_Refresh() 只重画数值区 */
    oled_ui_draw_labels();
}

void OLED_UI_Refresh(const OLED_UI_Data_t *d)
{
    char buf[16]; /* 最长的 "65535ppm" 是 8 字节，留一倍余量 */

    if (d == NULL)
        return;

    /* ---- 热插拔：屏不在线时每次刷新前探一次 ----
     *   「运行中被拔掉」不靠这里发现：oled.c 里一旦写失败就会把在线标志清 0，
     *   所以拔屏最迟下一轮刷新就会走进这个分支。
     *   探测很便宜（屏在约 1ms，不在 5ms 以内），不影响主循环时序；
     *   屏真不在的话下面整屏刷新全部跳过，一次 I2C 都不会发。 */
    if (!OLED_IsOnline())
    {
        oled_ui_on_hotplug();
        if (!OLED_IsOnline())
            return; /* 还是没插，本次刷新直接不做 */
    }

    /* ---- 第1行左：温度（DHT22 读失败就显示 "--"，不用旧值）---- */
    if (d->dht_ok)
        sprintf(buf, "%.1fC", d->temp);
    else
        sprintf(buf, "--");
    pad_to(buf, VAL_W_TEMP);
    OLED_ShowString(VAL_X_TEMP, OLED_UI_LINE1_Y, buf, OLED_UI_FONT_SIZE);

    /* ---- 第1行右：空气湿度 ---- */
    if (d->dht_ok)
        sprintf(buf, "%.1f%%", d->humi);
    else
        sprintf(buf, "--");
    pad_to(buf, VAL_W_HUMI);
    OLED_ShowString(VAL_X_HUMI, OLED_UI_LINE1_Y, buf, OLED_UI_FONT_SIZE);

    /* ---- 第2行：光照强度（0~350 lux）---- */
    sprintf(buf, "%u lux", (unsigned int)d->lux);
    pad_to(buf, VAL_W_LUX);
    OLED_ShowString(VAL_X_LUX, OLED_UI_LINE2_Y, buf, OLED_UI_FONT_SIZE);

    /* ---- 第3行：CO2 浓度 ---- */
    if (d->co2_ok)
        sprintf(buf, "%uppm", (unsigned int)d->co2);
    else
        sprintf(buf, "---");
    pad_to(buf, VAL_W_CO2);
    OLED_ShowString(VAL_X_CO2, OLED_UI_LINE3_Y, buf, OLED_UI_FONT_SIZE);

    /* ---- 第4行：土壤湿度等级（1~4，4 表示水分饱和）---- */
    sprintf(buf, "%u/4", (unsigned int)d->soil);
    pad_to(buf, VAL_W_SOIL);
    OLED_ShowString(VAL_X_SOIL, OLED_UI_LINE4_Y, buf, OLED_UI_FONT_SIZE);
}
