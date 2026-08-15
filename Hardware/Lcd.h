#ifndef __LCD_H
#define __LCD_H 

// 将原有 #include "sys.h" 替换为 STM32F4 标准库头文件
#include "stm32f4xx.h"
#include "stdlib.h"	

//----------------LCD端口定义-----------------
/***************根据自己需求更改****************/
#define LCD_SCL_GPIO_PORT    GPIOB       // LCD SCL/SCK/CLK
#define LCD_SCL_GPIO_PIN     GPIO_Pin_13

#define LCD_SDA_GPIO_PORT    GPIOB       // LCD SDA/SDI/DIN/MOSI
#define LCD_SDA_GPIO_PIN     GPIO_Pin_15

#define LCD_RST_GPIO_PORT    GPIOB       // LCD RES/RST
#define LCD_RST_GPIO_PIN     GPIO_Pin_14

#define LCD_DC_GPIO_PORT     GPIOC       // LCD DC/RS/A0
#define LCD_DC_GPIO_PIN      GPIO_Pin_5

#define LCD_CS_GPIO_PORT     GPIOB       // LCD CS
#define LCD_CS_GPIO_PIN      GPIO_Pin_12

#define LCD_BLK_GPIO_PORT    GPIOB       // LCD BL/BLK/LED
#define LCD_BLK_GPIO_PIN     GPIO_Pin_1

/*********************END**********************/

#define LCD_SCLK_Set()  GPIO_SetBits(LCD_SCL_GPIO_PORT, LCD_SCL_GPIO_PIN)
#define LCD_SCLK_Clr()  GPIO_ResetBits(LCD_SCL_GPIO_PORT, LCD_SCL_GPIO_PIN)
#define LCD_MOSI_Set()  GPIO_SetBits(LCD_SDA_GPIO_PORT, LCD_SDA_GPIO_PIN)
#define LCD_MOSI_Clr()  GPIO_ResetBits(LCD_SDA_GPIO_PORT, LCD_SDA_GPIO_PIN)
#define LCD_RES_Set()   GPIO_SetBits(LCD_RST_GPIO_PORT, LCD_RST_GPIO_PIN)
#define LCD_RES_Clr()   GPIO_ResetBits(LCD_RST_GPIO_PORT, LCD_RST_GPIO_PIN)
#define LCD_DC_Set()    GPIO_SetBits(LCD_DC_GPIO_PORT, LCD_DC_GPIO_PIN)
#define LCD_DC_Clr()    GPIO_ResetBits(LCD_DC_GPIO_PORT, LCD_DC_GPIO_PIN)
#define LCD_CS_Set()    GPIO_SetBits(LCD_CS_GPIO_PORT, LCD_CS_GPIO_PIN)
#define LCD_CS_Clr()    GPIO_ResetBits(LCD_CS_GPIO_PORT, LCD_CS_GPIO_PIN)
#define LCD_BLK_Set()   GPIO_SetBits(LCD_BLK_GPIO_PORT, LCD_BLK_GPIO_PIN)
#define LCD_BLK_Clr()   GPIO_ResetBits(LCD_BLK_GPIO_PORT, LCD_BLK_GPIO_PIN)

#define USE_HORIZONTAL 2   // 0或1为竖屏 2或3为横屏

#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 128
#define LCD_H 160
#else
#define LCD_W 160
#define LCD_H 128
#endif

#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE           	 0x001F  
#define BRED             0XF81F
#define GRED 			       0XFFE0
#define GBLUE			       0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			     0XBC40 //棕色
#define BRRED 			     0XFC07 //棕红色
#define GRAY  			     0X8430 //灰色
#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色  
#define GRAYBLUE       	 0X5458 //灰蓝色
#define LIGHTGREEN     	 0X841F //浅绿色
#define LGRAY 			     0XC618 //浅灰色(PANNEL),窗体背景色
#define LGRAYBLUE        0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE           0X2B12 //浅棕蓝色(选择条目的反色)

void LCD_GPIO_Init(void);
void LCD_Writ_Bus(u8 dat);
void LCD_WR_DATA8(u8 dat);
void LCD_WR_DATA(u16 dat);
void LCD_WR_REG(u8 dat);
void LCD_Address_Set(u16 x1, u16 y1, u16 x2, u16 y2);
void LCD_Init(void);
void LCD_Fill(u16 xsta, u16 ysta, u16 xend, u16 yend, u16 color);
void LCD_DrawPoint(u16 x, u16 y, u16 color);
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2, u16 color);
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2, u16 color);
void Draw_Circle(u16 x0, u16 y0, u8 r, u16 color);
void LCD_ShowChinese(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey, u8 mode);
void LCD_ShowChinese12x12(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey, u8 mode);
void LCD_ShowChinese16x16(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey, u8 mode);
void LCD_ShowChinese24x24(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey, u8 mode);
void LCD_ShowChinese32x32(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey, u8 mode);
void LCD_ShowChar(u16 x, u16 y, u8 num, u16 fc, u16 bc, u8 sizey, u8 mode);
void LCD_ShowString(u16 x, u16 y, const u8 *p, u16 fc, u16 bc, u8 sizey, u8 mode);
u32 mypow(u8 m, u8 n);
void LCD_ShowIntNum(u16 x, u16 y, u16 num, u8 len, u16 fc, u16 bc, u8 sizey);
void LCD_ShowFloatNum1(u16 x, u16 y, float num, u8 len, u16 fc, u16 bc, u8 sizey);
void LCD_ShowPicture(u16 x, u16 y, u16 length, u16 width, const u8 pic[]);

/* 新增 64x32 字体相关声明 */
void LCD_ShowChar64x32(u16 x, u16 y, u8 num, u16 fc, u16 bc, u8 mode);
void LCD_ShowChinese64x64(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 mode);
void LCD_ShowChar100x50(u16 x, u16 y, u8 num, u16 fc, u16 bc, u8 mode);

#endif
