#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>

#define OLED_WIDTH 128
#define OLED_HEIGHT 64

#define OLED_CMD 0	// 写命令
#define OLED_DATA 1 // 写数据

// OLED控制用函数
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Clear(void);
void OLED_Fill_Area(uint8_t x0, uint8_t y0, uint8_t x1,
					uint8_t y1, uint8_t fill_data);
uint8_t OLED_Show_Font_error(uint8_t x, uint8_t y);
uint8_t OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t font_size);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t font_size);
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no, uint8_t font_size);
void OLED_ShowString(uint8_t x, uint8_t y, const char *chr, uint8_t font_size);
void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint16_t bmp_width, uint16_t bmp_height,
				  const uint8_t BMP[]);

/* 初始化（可反复调用）。每次都会先探测屏在不在：
 *   在线  → 配置寄存器 + 清屏
 *   不在线 → 立刻返回，一个字都不发（所以「没插屏」不会拖慢主循环）
 * 冷启动的 200ms 电源等待只在第一次调用时做一次。 */
void OLED_Init(void);

/* 屏当前是否在线：1=在线，0=没插 / 运行中被拔掉 / 排线松了。
 * 写 I2C 失败会自动把状态置成「不在线」，所以拔屏最迟下一轮刷新就会被发现。
 * 用法见 oled_ui.c 的 OLED_UI_Refresh() */
uint8_t OLED_IsOnline(void);

#endif
