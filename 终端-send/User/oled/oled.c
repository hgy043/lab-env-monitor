#include "oled.h"     // 包含 OLED 相关的头文件
#include "oledfont.h" // 包含 OLED 字体数据头文件
#include <string.h>   // 包含字符串操作函数头文件
#include <stdio.h>
#include "main.h"
#include "i2c.h"

/*===========================================================================
 * OLED 热插拔支持（在线检测）
 *
 *   为什么要做：屏只有一块，可能插在终端也可能插在中控，还可能运行中途被拔掉。
 *   不能因为「屏不在」就把主循环卡住、或者每次刷新都白等一串 I2C 超时。
 *
 *   三条机制：
 *     ① 所有 I2C 写先看 s_oled_online，不在线就直接返回，一个字都不发
 *     ② 任何一次写失败（NACK / 超时）立刻把 s_oled_online 清 0
 *        —— 这就是「运行中把屏拔掉」的检测点，不用额外轮询
 *     ③ OLED_Init() 可以被反复调用：每次进来先探测一次地址 0x78，
 *        在就重新配置 + 清屏，不在就直接返回（一次探测 5ms 以内）
 *
 *   开销（屏不在线时）：每次刷新一次探测，**5ms 以内**，零 I2C 流量。
 *   开销（刚把屏插上那一次）：除探测外多 200ms 等电源稳定，见 OLED_Init()。
 *
 *   典型用法见 oled_ui.c：主循环每次刷新前先看 OLED_IsOnline()，
 *   不在线就调一次 OLED_Init() 试探，插上了再把四行标签重画一遍。
 *===========================================================================*/

// I2C 写的超时（毫秒）。不能用 HAL_MAX_DELAY：
//   屏不在线、排线松了的时候 HAL_I2C_Mem_Write 会永远自旋，
//   整个程序就卡在 OLED_UI_Init() 里，后面一句都不会执行
#define OLED_I2C_TIMEOUT_MS 100

// SSD1306 的 8 位写地址（7 位地址是 0x3C，左移一位就是 0x78）。
// HAL 的 I2C 接口要的就是这个左移后的形式
#define OLED_I2C_ADDR 0x78

// 在线探测时每次尝试的超时（毫秒）。
//   探测很频繁（每次刷新前一次），所以给得很小：屏真在的话第一次就 ACK 了，
//   不在的话最坏 2 次尝试 × 5ms 就返回，不会拖慢主循环
#define OLED_PROBE_TIMEOUT_MS 5

// 屏当前是否在线（1=在，0=没插/被拔掉）。
// 只在本文件里改，外面统一用 OLED_IsOnline() 读
static uint8_t s_oled_online = 0;

/**
 * @brief  查询 OLED 当前是否在线
 * @return 1=在线，0=不在线（没插 / 被拔掉 / 排线松了）
 */
uint8_t OLED_IsOnline(void)
{
    return s_oled_online;
}

/**
 * @brief  探测一次屏在不在总线上（HAL_I2C_IsDeviceReady 会发地址等 ACK）
 * @return 1=在线，0=不在线
 *
 *   只更新 s_oled_online，不改屏上内容，所以可以随便调。
 *   代价：屏在约 1ms，屏不在 5ms 以内
 */
static uint8_t OLED_Probe(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_I2C_ADDR, 2, OLED_PROBE_TIMEOUT_MS) == HAL_OK)
        s_oled_online = 1;
    else
        s_oled_online = 0;

    return s_oled_online;
}

/**
 * @brief  OLED写一个字节
 *
 * @param dat 要写入的数据
 * @param cmd 0，表示写命令；1，表示写数据
 *
 *   屏不在线直接返回；写失败就把在线标志清掉（下次刷新时会重新探测）
 */
static void OLED_WR_Byte(uint8_t dat, uint8_t cmd)
{
    HAL_StatusTypeDef st;

    if (!s_oled_online)
        return;

    if (cmd == OLED_DATA) // cmd = 1, 写数据
    {
        st = HAL_I2C_Mem_Write(&hi2c1, OLED_I2C_ADDR, 0x40, I2C_MEMADD_SIZE_8BIT, &dat, 1, OLED_I2C_TIMEOUT_MS);
    }
    else
    {
        st = HAL_I2C_Mem_Write(&hi2c1, OLED_I2C_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &dat, 1, OLED_I2C_TIMEOUT_MS);
    }

    if (st != HAL_OK)
        s_oled_online = 0; // 写不进去 = 屏被拔了或线松了，标记离线
}

/**
 * @brief  OLED写多个字节数据
 *
 * @param Data 要写入的数据指针
 * @param Count 要写入的数据个数
 */
static void OLED_WR_Data(uint8_t *Data, uint8_t Count)
{
    if (!s_oled_online)
        return;

    if (HAL_I2C_Mem_Write(&hi2c1, OLED_I2C_ADDR, 0x40, I2C_MEMADD_SIZE_8BIT, Data, Count, OLED_I2C_TIMEOUT_MS) != HAL_OK)
        s_oled_online = 0;
}

static void OLED_Set_Offset_Y(uint8_t y)
{
    if (y > OLED_HEIGHT - 1)
        y = OLED_HEIGHT - 1;
    OLED_WR_Byte(0xD3, OLED_CMD);
    OLED_WR_Byte(y, OLED_CMD);
}


/**
 * @brief  设置 OLED 显示位置
 *
 * @param x 水平坐标
 * @param y 垂直坐标（页地址，0~7）
 */
static void OLED_Set_Pos(uint8_t x, uint8_t y)
{
    OLED_WR_Byte(0xb0 + y, OLED_CMD);                 // 设置页地址（0xb0 + y）
    OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD); // 设置列地址高4位
    OLED_WR_Byte((x & 0x0f), OLED_CMD);               // 设置列地址低4位
}

/**
 * @brief  开启 OLED 显示
 *
 */
void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD); // 设置 DCDC 命令
    OLED_WR_Byte(0X14, OLED_CMD); // 开启 DCDC
    OLED_WR_Byte(0XAF, OLED_CMD); // 开启显示
}

/**
 * @brief  关闭 OLED 显示,进入休眠状态（低功耗）
 *
 */
void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD); // 设置 DCDC 命令
    OLED_WR_Byte(0X10, OLED_CMD); // 关闭 DCDC
    OLED_WR_Byte(0XAE, OLED_CMD); // 关闭显示
}

/**
 * @brief  清屏，整个屏幕变为黑色
 *
 */
void OLED_Clear(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++) // 遍历每一页（共8页）
    {
        OLED_WR_Byte(0xb0 + i, OLED_CMD); // 设置页地址（0xb0 + i）
        OLED_WR_Byte(0x00, OLED_CMD);     // 设置列地址低4位
        OLED_WR_Byte(0x10, OLED_CMD);     // 设置列地址高4位
        for (n = 0; n < OLED_WIDTH; n++)         // 遍历每一列（共128列）
            OLED_WR_Byte(0, OLED_DATA);   // 写入0，清屏
    }
}

/**
 * @brief  填充指定区域
 *
 * @param x0 起始X坐标
 * @param y0 起始Y坐标
 * @param x1 结束X坐标
 * @param y1 结束Y坐标
 * @param fill_data 填充数据
 */
void OLED_Fill_Area(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t fill_data)
{
    if (x0 > x1) { uint8_t temp = x0; x0 = x1; x1 = temp; }
    if (y0 > y1) { uint8_t temp = y0; y0 = y1; y1 = temp; }
    
    if (x0 >= OLED_WIDTH) x0 = OLED_WIDTH - 1;
    if (x1 >= OLED_WIDTH) x1 = OLED_WIDTH - 1;
    if (y0 >= OLED_HEIGHT) y0 = OLED_HEIGHT - 1;
    if (y1 >= OLED_HEIGHT) y1 = OLED_HEIGHT - 1;
    
    if (x0 > x1 || y0 > y1) return;
    
    uint8_t offset_y = y0 % 8;         
    uint8_t start_page = (y0 - offset_y) / 8; 
    uint8_t end_page = y1 / 8;         

    OLED_Set_Offset_Y(offset_y);
    for (uint8_t page = start_page; page <= end_page; page++)
    {
        if (page >= 8) break;
        OLED_Set_Pos(x0, page);
        for (uint8_t x = x0; x <= x1; x++)
        {
            OLED_WR_Data((uint8_t*)&fill_data, 1);
        }
    }
    
    OLED_Set_Offset_Y(0);
}
/**
 * @brief  当显示内容不存在时，OLED 显示“错误”图标（支持任意y像素）
 *
 * @param x 水平坐标
 * @param y 垂直坐标（像素级定位）
 * @return uint8_t 返回显示字符的宽度
 */
uint8_t OLED_Show_Font_error(uint8_t x, uint8_t y)
{
    // 计算需要的垂直偏移和目标页地址
    uint8_t offset = y % 8;         
    uint8_t page = (y - offset) / 8;
    if (x + 16 > OLED_WIDTH)
        return 0; // 判断字符是否超出屏幕宽度
    if (page > 7)
        page = 7;

    // 设置垂直偏移，让屏幕整体偏移，实现像素级定位
    OLED_Set_Offset_Y(offset);
    OLED_Set_Pos(x, page); // 传入计算后的有效页地址
    OLED_WR_Data((uint8_t*)Ferror, 16);
    OLED_Set_Pos(x, page + 1); // 传入计算后的有效页地址
    OLED_WR_Data((uint8_t*)Ferror + 16, 16);

    // 恢复偏移为 0，避免影响其他显示内容
    OLED_Set_Offset_Y(0);
    return 16;
}

/**
 * @brief  OLED 显示一个字符（支持任意y像素）
 * 
 * @param x 水平坐标
 * @param y 垂直坐标（像素级定位）
 * @param chr 要显示的字符
 * @param font_size 字符大小（FONT_SIZE_8/FONT_SIZE_16/FONT_SIZE_24等）
 * @return uint8_t 返回显示字符的宽度（像素）
 */
uint8_t OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t font_size)
{
    uint8_t ret = 0;
    uint8_t c = 0, i = 0;
    c = chr - ' '; // 得到字符在字体数组中的偏移量

    uint8_t offset = y % 8;         
    uint8_t page = (y - offset) / 8; 
    if (page > 7)
        page = 7;

    OLED_Set_Offset_Y(offset);
    switch (font_size) // 根据字体大小选择不同的字符数组
    {
#if FONT_SIZE_8
    case FONT_SIZE_8:     // 字体大小为8（占1个页，8像素高度）
        if (x + 6 <= OLED_WIDTH) // 判断字符是否超出屏幕宽度
        {
            OLED_Set_Pos(x, page);
            OLED_WR_Data((uint8_t*)F6x8[c], 6);
            ret = 6;
        }

        break;
#endif
#if FONT_SIZE_16
    case FONT_SIZE_16: // 字体大小为16（占2个页，16像素高度）
        for (i = 0; (i < FONT_SIZE_16 / 8) && (x + 8 <= OLED_WIDTH); i++)
        {
            if (page + i <= 7) // 判断页地址是否越界
            {
                OLED_Set_Pos(x, page + i); // 传入计算后的有效页地址
                OLED_WR_Data((uint8_t*)&F8X16[c][i * 8], 8);
                ret = 8;
            }
        }

        break;
#endif
#if FONT_SIZE_24
    case FONT_SIZE_24:
        for (i = 0; (i < FONT_SIZE_24 / 8) && (x + 12 <= OLED_WIDTH); i++)
        {
            if (page + i <= 7) // 判断页地址是否越界
            {
                OLED_Set_Pos(x, page + i);
                OLED_WR_Data((uint8_t*)&F12X24[c][i * 12], 12);
                ret = 12;
            }
        }
        break;
#endif
    default:
        break;
    }
    OLED_Set_Offset_Y(0);
    return ret;
}

/**
 * @brief  计算 m 的 n 次方
 * 
 * @param m 底数
 * @param n 指数
 * @return uint32_t 计算结果
 */
static uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--)      // 循环 n 次
        result *= m; // 计算 m 的 n 次方
    return result;
}

/**
 * @brief  OLED 显示一个数字（支持任意y像素）
 * 
 * @param x 水平坐标
 * @param y 垂直坐标（像素级定位）
 * @param num 要显示的数字
 * @param len 数字长度
 * @param font_size 字体大小（FONT_SIZE_8/FONT_SIZE_16/FONT_SIZE_24等）
 */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t font_size)
{
    uint8_t t, temp;
    uint8_t enshow = 0;

    uint8_t offset_y = y % 8;                                      // 垂直偏移量（0~7，实现像素级精准定位）
    uint8_t char_width = (font_size == FONT_SIZE_8) ? 6 : (font_size / 2); // 单个字符的显示宽度
    OLED_Set_Offset_Y(offset_y);

    for (t = 0; t < len; t++) // 遍历每一位数字
    {
        temp = (num / oled_pow(10, len - t - 1)) % 10; // 获取当前位的数字

        // 计算当前字符的x坐标（动态更新，支持任意起始x坐标）
        uint8_t current_x = x + char_width * t;
        // 超出屏幕宽度则停止显示，避免无效操作
        if (current_x + char_width > OLED_WIDTH)
            break;

        if (enshow == 0 && t < (len - 1)) // 如果当前位为0且不是最后一位
        {
            if (temp == 0) // 如果当前位为0，显示空格
            {
                OLED_ShowChar(current_x, y, ' ', font_size);
                continue;
            }
            else
                enshow = 1; // 否则，开始显示数字
        }

        // 显示当前位的数字
        OLED_ShowChar(current_x, y, temp + '0', font_size);
    }

    OLED_Set_Offset_Y(0);
}

int GetUtf8CharLength(const unsigned char *target)
{
    if (target[0] <= 0x7F)
    {
        // 单字节UTF-8字符 (0xxxxxxx)
        return 1;
    }
    else if ((target[0] >= 0xC2 && target[0] <= 0xDF) && (target[1] & 0xC0) == 0x80)
    {
        // 双字节UTF-8字符 (110xxxxx 10xxxxxx)
        return 2;
    }
    else if ((target[0] >= 0xE0 && target[0] <= 0xEF) && (target[1] & 0xC0) == 0x80 && (target[2] & 0xC0) == 0x80)
    {
        // 三字节UTF-8字符 (1110xxxx 10xxxxxx 10xxxxxx)
        return 3;
    }
    else if ((target[0] >= 0xF0 && target[0] <= 0xF7) && (target[1] & 0xC0) == 0x80 && (target[2] & 0xC0) == 0x80 && (target[3] & 0xC0) == 0x80)
    {
        // 四字节UTF-8字符 (11110xxx 10xxxxxx 10xxxxxx 10xxxxxx)
        return 4;
    }
    // 默认单字节
    return 1;
}
/********************************************
// Find Font Index
// 功能：根据目标字符查找字体索引
********************************************/
int FindFontIndex(const char *target, uint8_t font_size)
{
    int i;
    int charLen = GetUtf8CharLength((const unsigned char *)target); // 获取字符长度
#if FONT_SIZE_16
    if (font_size == 16)
    {
        for (i = 0; i < (sizeof(g_font_dot_matrix_16_index) / sizeof(g_font_dot_matrix_16_index[0])); i++)
        {

            if (strncmp(g_font_dot_matrix_16_index[i], target, charLen) == 0) // 比较第一个字节
                return i;                                                     // 返回索引
        }
    }
#endif
#if FONT_SIZE_24
    if (font_size == 24)
    {
        for (i = 0; i < (sizeof(g_font_dot_matrix_24_index) / sizeof(g_font_dot_matrix_24_index[0])); i++)
        {

            if (strncmp(g_font_dot_matrix_24_index[i], target, charLen) == 0) // 比较第一个字节
                return i;                                                     // 返回索引
        }
    }
#endif
#if FONT_SIZE_32
    if (font_size == 32)
    {
        for (i = 0; i < (sizeof(g_font_dot_matrix_32_index) / sizeof(g_font_dot_matrix_32_index[0])); i++)
        {
            if (strncmp(g_font_dot_matrix_32_index[i], target, charLen) == 0) // 比较第一个字节
            {
                return i;
            }
            // 返回索引
        }
    }
#endif
#if FONT_SIZE_48
    if (font_size == 48)
    {
        for (i = 0; i < (sizeof(g_font_dot_matrix_48_index) / sizeof(g_font_dot_matrix_48_index[0])); i++)
        {

            if (strncmp(g_font_dot_matrix_48_index[i], target, charLen) == 0) // 比较第一个字节
                return i;                                                     // 返回索引
        }
    }
#endif
#if FONT_SIZE_64
    if (font_size == 64)
    {
        for (i = 0; i < (sizeof(g_font_dot_matrix_64_index) / sizeof(g_font_dot_matrix_64_index[0])); i++)
        {

            if (strncmp(g_font_dot_matrix_64_index[i], target, charLen) == 0) // 比较前两个字节
                return i;                                                     // 返回索引
        }
    }
#endif
    return -1; // 如果未找到，返回-1
}

/**
 * @brief 显示中文字符（支持任意y像素）
 * 
 * @param x x坐标
 * @param y y坐标
 * @param no 字符索引
 * @param font_size 字体大小（FONT_SIZE_16/FONT_SIZE_24/FONT_SIZE_32/FONT_SIZE_48/FONT_SIZE_64等）
 */
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no, uint8_t font_size)
{
    uint8_t i;

    if (x + font_size > OLED_WIDTH)
        return;

    // 计算垂直偏移和目标页地址
    uint8_t offset = y % 8;
    uint8_t page = (y - offset) / 8;

    // 设置垂直偏移
    OLED_Set_Offset_Y(offset);
    switch (font_size)
    {
#if FONT_SIZE_16
    case FONT_SIZE_16:
        for (i = 0; i < font_size / 8; i++)
        {
            if (page + i > 7)
                break;
            OLED_Set_Pos(x, page + i); 
            OLED_WR_Data((uint8_t*)&g_font_dot_matrix_16[no][font_size * i], font_size);
        }
        break;
#endif
#if FONT_SIZE_24
    case FONT_SIZE_24:
        for (i = 0; i < font_size / 8; i++)
        {
            if (page + i > 7)
                break;
            OLED_Set_Pos(x, page + i);
            OLED_WR_Data((uint8_t*)&g_font_dot_matrix_24[no][font_size * i], font_size);
        }
        break;
#endif
#if FONT_SIZE_32
    case FONT_SIZE_32:
        for (i = 0; i < font_size / 8; i++)
        {
            if (page + i > 7)
                break;
            OLED_Set_Pos(x, page + i);
            OLED_WR_Data((uint8_t*)&g_font_dot_matrix_32[no][font_size * i], font_size);
        }
        break;
#endif
#if FONT_SIZE_48
    case FONT_SIZE_48:
        for (i = 0; i < font_size / 8; i++)
        {
            if (page + i > 7)
                break;
            OLED_Set_Pos(x, page + i);
            OLED_WR_Data((uint8_t*)&g_font_dot_matrix_48[no][font_size * i], font_size);
        }
        break;
#endif
#if FONT_SIZE_64
    case FONT_SIZE_64:
        for (i = 0; i < font_size / 8; i++)
        {
            if (page + i > 7)
                break;
            OLED_Set_Pos(x, page + i);
            OLED_WR_Data((uint8_t*)&g_font_dot_matrix_64[no][font_size * i], font_size);
        }
        break;
#endif
    }
    OLED_Set_Offset_Y(0);
}

/**
 * @brief 显示字符串（支持ASCII与中文混合）
 * 
 * @param x x坐标
 * @param y y坐标
 * @param chr 字符串指针
 * @param font_size 字体大小（FONT_SIZE_8/FONT_SIZE_16/FONT_SIZE_24/FONT_SIZE_32/FONT_SIZE_48/FONT_SIZE_64等）
 */
void OLED_ShowString(uint8_t x, uint8_t y, const char *chr, uint8_t font_size)
{
    int ret = 0;
    if (chr == NULL || font_size == 0) // 入参合法性判断
        return;

    uint8_t current_x = x;                       // 当前显示x坐标（动态更新）
    uint8_t current_y = y;                       // 当前显示y坐标（换行时更新）
    unsigned char *p_str = (unsigned char *)chr; // 转换为无符号字符指针，方便UTF-8判断

    while (*p_str != '\0') // 遍历字符串直到结束符
    {
        int utf8_len = GetUtf8CharLength(p_str);

        if (utf8_len == 1) // ASCII字符（0~127）
        {
            ret = OLED_ShowChar(current_x, current_y, *p_str, font_size);
            if (ret > 0) // 如果显示成功，更新x坐标
            {
                current_x += ret;
                p_str += 1;
                continue;
            }
        }

        // 查找中文字体索引
        int font_index = FindFontIndex((const char *)p_str, font_size);
        if (font_index >= 0) // 找到对应字体，显示汉字
        {
            OLED_ShowCHinese(current_x, current_y, font_index, font_size);
            if (*p_str > 0x7F) // 如果是汉字
            {
                current_x += font_size; // 移动到下一个汉字位置
            }
            else // 如果是 ASCII 字符
            {
                current_x += font_size / 2; // 移动到下一个字符位置
            }
        }
        else // 没有找到对应字体，显示框框
        {
            current_x += OLED_Show_Font_error(current_x, current_y);
        }
        p_str += utf8_len;
        if (current_x >= OLED_WIDTH)
        {
            break;
        }
    }
}

/**
 * @brief 在 OLED 上绘制 BMP 图片
 * 
 * @param x0 起始x坐标
 * @param y0 起始y坐标
 * @param bmp_width 位图宽度
 * @param bmp_height 位图高度
 * @param BMP 位图数据
 */
void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint16_t bmp_width, uint16_t bmp_height,
                  const uint8_t BMP[])
{
    uint8_t display_x0 = x0;
    uint8_t display_y0 = y0;
    if (display_x0 >= OLED_WIDTH)
        display_x0 = OLED_WIDTH - 1;
    if (display_y0 >= OLED_HEIGHT)
        display_y0 = OLED_HEIGHT - 1;

    uint16_t bmp_x1 = display_x0 + bmp_width - 1;
    uint16_t bmp_y1 = display_y0 + bmp_height - 1;
    uint8_t display_x1 = (bmp_x1 >= OLED_WIDTH) ? (OLED_WIDTH - 1) : (uint8_t)bmp_x1;
    uint8_t display_y1 = (bmp_y1 >= OLED_HEIGHT) ? (OLED_HEIGHT - 1) : (uint8_t)bmp_y1;

    uint16_t valid_width = display_x1 - display_x0 + 1;
    uint16_t valid_height = display_y1 - display_y0 + 1;
    if (valid_width == 0 || valid_height == 0)
        return;

    uint32_t bmp_total_bytes = bmp_width * ((bmp_height % 8 == 0) ? (bmp_height / 8) : (bmp_height / 8 + 1));
    uint16_t bmp_page_num = (bmp_height % 8 == 0) ? (bmp_height / 8) : (bmp_height / 8 + 1);
    uint32_t bmp_index = 0;

    uint8_t offset_y = display_y0 % 8;
    uint8_t start_page = (display_y0 - offset_y) / 8;
    OLED_Set_Offset_Y(offset_y);

    for (uint16_t page_offset = 0; page_offset < bmp_page_num; page_offset++)
    {
        uint16_t current_page = start_page + page_offset;
        if (current_page >= 8)
            break;

        OLED_Set_Pos(display_x0, current_page);

        for (uint8_t x = display_x0; x <= display_x1; x++)
        {
            bmp_index = page_offset * bmp_width + (x - display_x0);

            if (bmp_index >= bmp_total_bytes)
                break;
            OLED_WR_Data((uint8_t *)&BMP[bmp_index], 1);
        }
    }

    OLED_Set_Offset_Y(0);
}

/**
 * @brief  OLED 初始化
 *
 *   本函数**可以被反复调用**，每次都会先探测屏在不在：
 *     在线  → 走完下面这串寄存器配置 + 清屏（热插拔重新插上时也走这条）
 *     不在线 → 立刻返回，后面一个字都不发，主循环不会被 I2C 拖住
 *
 *   200ms 电源等待只在「屏刚被发现」的那一次做：
 *     进函数时 s_oled_online 还是 0（冷启动、或者运行中刚插上）
 *     → 屏可能是刚上电的，它自己的复位还没走完，这时候直接灌配置有风险
 *     → 先等 200ms 再配寄存器
 *   反过来：
 *     探测失败（屏一直没插 / 一直拔着）→ 等都不等就返回，只花几毫秒探测
 *     已经在线 → 这个分支根本不会再走到（oled_ui.c 只在不「在线」时才调它）
 *
 *   ⚠ 这 200ms 是**阻塞**的，一次「插屏」动作只发生一次。
 *     中控那边最坏情况：插屏那一瞬间正好收到一包，ACK 会晚发最多 200ms
 *     —— 终端给的 ACK 窗口是 400ms，还装得下，而且只是一次性代价。
 *   ⚠ 200ms 这个数沿用的是原来冷启动那份延时。屏手册里「上电和断电顺序」
 *     那一节在资料库的中文版里是「不翻译」，所以这里没法引用具体条款，
 *     只是保守沿用。真出现「插上不亮、再插一次才亮」的话，把这里调大。
 */
void OLED_Init(void)
{
    uint8_t was_online = s_oled_online;

    /* 探测：屏不在就直接返回。下次调用会再探一次，
     * 所以「拔掉 → 重新插上」最迟下一轮刷新就能自动恢复 */
    if (!OLED_Probe())
        return;

    /* 屏「上一次还不在线、这次探测到了」= 它很可能是刚插上/刚上电的，
     * 等电源和它自己的复位稳定下来再灌配置寄存器 */
    if (!was_online)
        HAL_Delay(200); // 延时 200ms, 等待 OLED 电源稳定

    // 初始化 OLED 显示屏
    OLED_WR_Byte(0xAE, OLED_CMD); // 关闭显示
    OLED_WR_Byte(0x00, OLED_CMD); // 设置低列地址
    OLED_WR_Byte(0x10, OLED_CMD); // 设置高列地址
    OLED_WR_Byte(0x40, OLED_CMD); // 设置起始行地址
    OLED_WR_Byte(0xB0, OLED_CMD); // 设置页地址
    OLED_WR_Byte(0x81, OLED_CMD); // 设置对比度
    OLED_WR_Byte(0xFF, OLED_CMD); // 设置对比度值为255
    OLED_WR_Byte(0xA1, OLED_CMD); // 设置段重映射
    OLED_WR_Byte(0xA6, OLED_CMD); // 设置正常显示
    OLED_WR_Byte(0xA8, OLED_CMD); // 设置多路复用比率
    OLED_WR_Byte(0x3F, OLED_CMD); // 设置多路复用比率为1/32
    OLED_WR_Byte(0xC8, OLED_CMD); // 设置 COM 扫描方向
    OLED_WR_Byte(0xD3, OLED_CMD); // 设置显示偏移
    OLED_WR_Byte(0x00, OLED_CMD); // 设置显示偏移为0
    OLED_WR_Byte(0xD5, OLED_CMD); // 设置时钟分频
    OLED_WR_Byte(0x80, OLED_CMD); // 设置时钟分频为默认值
    OLED_WR_Byte(0xD8, OLED_CMD); // 设置区域颜色模式关闭
    OLED_WR_Byte(0x05, OLED_CMD); // 设置区域颜色模式
    OLED_WR_Byte(0xD9, OLED_CMD); // 设置预充电周期
    OLED_WR_Byte(0xF1, OLED_CMD); // 设置预充电周期
    OLED_WR_Byte(0xDA, OLED_CMD); // 设置 COM 引脚配置
    OLED_WR_Byte(0x12, OLED_CMD); // 设置 COM 引脚配置
    OLED_WR_Byte(0xDB, OLED_CMD); // 设置 VCOMH 电压
    OLED_WR_Byte(0x30, OLED_CMD); // 设置 VCOMH 电压
    OLED_WR_Byte(0x8D, OLED_CMD); // 设置电荷泵使能
    OLED_WR_Byte(0x14, OLED_CMD); // 设置电荷泵使能
    OLED_WR_Byte(0xAF, OLED_CMD); // 开启 OLED 显示
    OLED_Clear();                 // 清空屏幕
}

