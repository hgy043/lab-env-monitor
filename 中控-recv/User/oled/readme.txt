1.使用请看oled_demo.c示例函数

2.使用前请先配置好oled的引脚，默认使用I2C1。如需修改只需修改下面两个接口函数即可：
	void OLED_WR_Byte(uint8_t dat, uint8_t cmd)
	void OLED_WR_Data(uint8_t *Data, uint8_t Count)

3.使用的字库文件在oledfont.h中，使用前记得打开宏定义。如下
	#define	FONT_SIZE_16    16   // 0时关闭，16时开启16点阵字库

4.使用的图片资源在bmp.h中。

5.使用中文字体、图片，需要取模。取模软件PortHelper.exe中的“点阵生成”工具。
	下载链接：http://files.cnblogs.com/wenziqi/%E5%8D%95%E7%89%87%E6%9C%BA%E5%A4%9A%E5%8A%9F%E8%83%BD%E8%B0%83%E8%AF%95%E5%8A%A9%E6%89%8B.rar
