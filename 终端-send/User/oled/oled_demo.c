#include "oled_demo.h"
#include "oled.h"
#include "bmp.h"
void oled_demo(void)
{
  OLED_Init();
  OLED_Clear();
//	OLED_ShowNum(30, 30, 1024, 4, 16); // 显示数字1024
//  OLED_ShowString(0, 0, "您好", 16);
//	OLED_ShowString(30, 30, "黄国一", 16);
//	OLED_ShowString(45, 40, "您好黄国一", 16);
//  OLED_DrawBMP(90, 30, 16, 16, g_image_dot_tem_16x16);
	
	
//	OLED_ShowString(0, 0, "温度:", 16);
//	OLED_ShowString(0, 20, "光照强度:", 16);
//	OLED_ShowString(0, 35, "co2浓度:", 16);
//	OLED_ShowString(0, 55, "土壤湿度:", 16);
	
	//	//dror
	while(1){
	OLED_DrawBMP(45, 30, 32, 32, g_image_dot_diro_frame_0_32x32);
	OLED_DrawBMP(45, 30, 32, 32, g_image_dot_diro_frame_1_32x32);
	OLED_DrawBMP(45, 30, 32, 32, g_image_dot_diro_frame_2_32x32);
	OLED_DrawBMP(45, 30, 32, 32, g_image_dot_diro_frame_3_32x32);
	}
	
//	//stop
//	while(1){
//	OLED_DrawBMP(0, 0, 32, 32, g_image_dot_stop_frame_0_32x32);
//	OLED_DrawBMP(0, 0, 32, 32, g_image_dot_stop_frame_1_32x32);
//	OLED_DrawBMP(0, 0, 32, 32, g_image_dot_stop_frame_2_32x32);
//	OLED_DrawBMP(0, 0, 32, 32, g_image_dot_stop_frame_3_32x32);
//	}
//	
}
