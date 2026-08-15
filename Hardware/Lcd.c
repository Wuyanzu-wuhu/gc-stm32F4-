#include "lcd.h"
#include "stdlib.h"
#include "lcdfont.h"

#include "Delay.h"
/***************** 辰哥单片机设计 (适配 STM32F407VGT6) ******************
 * 文件            : TFT-LCD显示屏(1.8寸) c文件
 * 版本            : V1.0 (F407) 新增 64x32 字体支持
 * MCU             : STM32F407VGT6
 * 接口            : 软件模拟 SPI (见 lcd.h 引脚定义)
 * 作者            : 辰哥 (修改适配 F407)
***********************************************************************/

/* 修改点：GPIO 时钟使能使用 RCC_AHB1PeriphClockCmd (F407 标准库) */
void LCD_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_InitStructure.GPIO_Pin = LCD_SCL_GPIO_PIN | LCD_SDA_GPIO_PIN | LCD_RST_GPIO_PIN |
                                  LCD_CS_GPIO_PIN | LCD_BLK_GPIO_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = LCD_DC_GPIO_PIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB, LCD_SCL_GPIO_PIN | LCD_SDA_GPIO_PIN | LCD_RST_GPIO_PIN |
                        LCD_CS_GPIO_PIN | LCD_BLK_GPIO_PIN);
    GPIO_SetBits(GPIOC, LCD_DC_GPIO_PIN);
}

/******************************************************************************
 * 函数说明：LCD 串行数据写入函数 (软件 SPI)
 * 入口数据：dat 要写入的串行数据
 * 返回值：  无
******************************************************************************/
void LCD_Writ_Bus(u8 dat)
{
    u8 i;
    LCD_CS_Clr();
    for(i = 0; i < 8; i++)
    {
        LCD_SCLK_Clr();
        if(dat & 0x80)
            LCD_MOSI_Set();
        else
            LCD_MOSI_Clr();
        LCD_SCLK_Set();
        dat <<= 1;
    }
    LCD_CS_Set();
}

/******************************************************************************
 * 函数说明：LCD 写入数据 (8位)
******************************************************************************/
void LCD_WR_DATA8(u8 dat)
{
    LCD_Writ_Bus(dat);
}

/******************************************************************************
 * 函数说明：LCD 写入数据 (16位)
******************************************************************************/
void LCD_WR_DATA(u16 dat)
{
    LCD_Writ_Bus(dat >> 8);
    LCD_Writ_Bus(dat);
}

/******************************************************************************
 * 函数说明：LCD 写入命令
******************************************************************************/
void LCD_WR_REG(u8 dat)
{
    LCD_DC_Clr();          // 写命令
    LCD_Writ_Bus(dat);
    LCD_DC_Set();          // 恢复数据模式
}

/******************************************************************************
 * 函数说明：设置起始和结束地址
******************************************************************************/
void LCD_Address_Set(u16 x1, u16 y1, u16 x2, u16 y2)
{
    if(USE_HORIZONTAL == 0)
    {
        LCD_WR_REG(0x2a); // 列地址设置
        LCD_WR_DATA(x1 + 2);
        LCD_WR_DATA(x2 + 2);
        LCD_WR_REG(0x2b); // 行地址设置
        LCD_WR_DATA(y1 + 1);
        LCD_WR_DATA(y2 + 1);
        LCD_WR_REG(0x2c); // 储存器写
    }
    else if(USE_HORIZONTAL == 1)
    {
        LCD_WR_REG(0x2a);
        LCD_WR_DATA(x1 + 2);
        LCD_WR_DATA(x2 + 2);
        LCD_WR_REG(0x2b);
        LCD_WR_DATA(y1 + 1);
        LCD_WR_DATA(y2 + 1);
        LCD_WR_REG(0x2c);
    }
    else if(USE_HORIZONTAL == 2)
    {
        LCD_WR_REG(0x2a);
        LCD_WR_DATA(x1 + 1);
        LCD_WR_DATA(x2 + 1);
        LCD_WR_REG(0x2b);
        LCD_WR_DATA(y1 + 2);
        LCD_WR_DATA(y2 + 2);
        LCD_WR_REG(0x2c);
    }
    else
    {
        LCD_WR_REG(0x2a);
        LCD_WR_DATA(x1 + 1);
        LCD_WR_DATA(x2 + 1);
        LCD_WR_REG(0x2b);
        LCD_WR_DATA(y1 + 2);
        LCD_WR_DATA(y2 + 2);
        LCD_WR_REG(0x2c);
    }
}

/* LCD 初始化 (ST7735S 驱动，未作修改，仅保证延时足够) */
void LCD_Init(void)
{
    LCD_GPIO_Init();

    LCD_RES_Clr();
    Delay_ms(100);
    LCD_RES_Set();
    Delay_ms(100);

    LCD_BLK_Set();        // 打开背光
    Delay_ms(100);

    //************* Start Initial Sequence **********//
    LCD_WR_REG(0x11);     // Sleep out
    Delay_ms(120);
    //------------------------------------ST7735S Frame Rate-----------------------------------------//
    LCD_WR_REG(0xB1);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x3C);
    LCD_WR_REG(0xB2);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x3C);
    LCD_WR_REG(0xB3);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x3C);
    LCD_WR_DATA8(0x3C);
    //------------------------------------End ST7735S Frame Rate---------------------------------//
    LCD_WR_REG(0xB4);
    LCD_WR_DATA8(0x03);
    //------------------------------------ST7735S Power Sequence---------------------------------//
    LCD_WR_REG(0xC0);
    LCD_WR_DATA8(0x28);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x04);
    LCD_WR_REG(0xC1);
    LCD_WR_DATA8(0xC0);
    LCD_WR_REG(0xC2);
    LCD_WR_DATA8(0x0D);
    LCD_WR_DATA8(0x00);
    LCD_WR_REG(0xC3);
    LCD_WR_DATA8(0x8D);
    LCD_WR_DATA8(0x2A);
    LCD_WR_REG(0xC4);
    LCD_WR_DATA8(0x8D);
    LCD_WR_DATA8(0xEE);
    //---------------------------------End ST7735S Power Sequence-------------------------------------//
    LCD_WR_REG(0xC5);
    LCD_WR_DATA8(0x1A);
    LCD_WR_REG(0x36);    // MX, MY, RGB mode
    if(USE_HORIZONTAL == 0)      LCD_WR_DATA8(0x00);
    else if(USE_HORIZONTAL == 1) LCD_WR_DATA8(0xC0);
    else if(USE_HORIZONTAL == 2) LCD_WR_DATA8(0x70);
    else                         LCD_WR_DATA8(0xA0);
    //------------------------------------ST7735S Gamma Sequence---------------------------------//
    LCD_WR_REG(0xE0);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x22);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x0A);
    LCD_WR_DATA8(0x2E);
    LCD_WR_DATA8(0x30);
    LCD_WR_DATA8(0x25);
    LCD_WR_DATA8(0x2A);
    LCD_WR_DATA8(0x28);
    LCD_WR_DATA8(0x26);
    LCD_WR_DATA8(0x2E);
    LCD_WR_DATA8(0x3A);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x01);
    LCD_WR_DATA8(0x03);
    LCD_WR_DATA8(0x13);
    LCD_WR_REG(0xE1);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x16);
    LCD_WR_DATA8(0x06);
    LCD_WR_DATA8(0x0D);
    LCD_WR_DATA8(0x2D);
    LCD_WR_DATA8(0x26);
    LCD_WR_DATA8(0x23);
    LCD_WR_DATA8(0x27);
    LCD_WR_DATA8(0x27);
    LCD_WR_DATA8(0x25);
    LCD_WR_DATA8(0x2D);
    LCD_WR_DATA8(0x3B);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x01);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x13);
    //------------------------------------End ST7735S Gamma Sequence-----------------------------//
    LCD_WR_REG(0x3A);    // 65k mode
    LCD_WR_DATA8(0x05);
    LCD_WR_REG(0x29);    // Display on
}


/* ==================== 新增 100x50 英文字符支持 ==================== */
void LCD_ShowChar100x50(u16 x, u16 y, u8 num, u16 fc, u16 bc, u8 mode)
{
    u8 temp, t, m = 0;
    u16 i, TypefaceNum;
    u16 x0 = x;
    u8 sizex = 50;      // 宽度 50
    u8 sizey = 100;     // 高度 100

    // 一个字符占用的字节数 = ceil(宽度/8) * 高度 = ceil(50/8)=7 → 7*100=700 字节
    TypefaceNum = ((sizex + 7) / 8) * sizey;   // 即 (50+7)/8 = 7， 7*100=700
    num = num - ' ';   // ASCII 偏移

    LCD_Address_Set(x, y, x + sizex - 1, y + sizey - 1);
    for(i = 0; i < TypefaceNum; i++)
    {
        // 需要提供 ascii_10050[95][700] 字库
        temp = ascii_10050[num][i];
        for(t = 0; t < 8; t++)
        {
            if(!mode)
            {
                if(temp & (0x01 << t)) LCD_WR_DATA(fc);
                else LCD_WR_DATA(bc);
                m++;
                if(m % sizex == 0) { m = 0; break; }
            }
            else
            {
                if(temp & (0x01 << t)) LCD_DrawPoint(x, y, fc);
                x++;
                if((x - x0) == sizex) { x = x0; y++; break; }
            }
        }
    }
}



/* ==================== 新增 64x32 字体支持 ==================== */

/* 显示单个 64x32 字符 (高度64，宽度32) */
void LCD_ShowChar64x32(u16 x, u16 y, u8 num, u16 fc, u16 bc, u8 mode)
{
    u8 temp, t, m = 0;
    u16 i, TypefaceNum;
    u16 x0 = x;
    u8 sizex = 32;      // 宽度固定32
    u8 sizey = 64;      // 高度固定64

    TypefaceNum = (sizex / 8) * sizey;   // (32/8)*64 = 4*64 = 256 字节
    num = num - ' ';
    LCD_Address_Set(x, y, x + sizex - 1, y + sizey - 1);
    for(i = 0; i < TypefaceNum; i++)
    {
        temp = ascii_6432[num][i];       // 需要提供 ascii_6432[95][256] 字库
        for(t = 0; t < 8; t++)
        {
            if(!mode)
            {
                if(temp & (0x01 << t)) LCD_WR_DATA(fc);
                else LCD_WR_DATA(bc);
                m++;
                if(m % sizex == 0) { m = 0; break; }
            }
            else
            {
                if(temp & (0x01 << t)) LCD_DrawPoint(x, y, fc);
                x++;
                if((x - x0) == sizex) { x = x0; y++; break; }
            }
        }
    }
}

/* 显示单个 64x64 汉字 (正方形) */
void LCD_ShowChinese64x64(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 mode)
{
    u8 i, j, m = 0;
    u16 k;
    u16 HZnum = sizeof(tfont64) / sizeof(typFNT_GB64);   // 需要定义 tfont64 和 typFNT_GB64
    u16 TypefaceNum = (64/8 + ((64%8)?1:0)) * 64;        // 8*64=512 字节
    u16 x0 = x;
    for(k = 0; k < HZnum; k++)
    {
        if((tfont64[k].Index[0] == *(s)) && (tfont64[k].Index[1] == *(s+1)))
        {
            LCD_Address_Set(x, y, x+64-1, y+64-1);
            for(i = 0; i < TypefaceNum; i++)
            {
                for(j = 0; j < 8; j++)
                {
                    if(!mode)
                    {
                        if(tfont64[k].Msk[i] & (0x01<<j)) LCD_WR_DATA(fc);
                        else LCD_WR_DATA(bc);
                        m++;
                        if(m % 64 == 0) { m = 0; break; }
                    }
                    else
                    {
                        if(tfont64[k].Msk[i] & (0x01<<j)) LCD_DrawPoint(x, y, fc);
                        x++;
                        if((x-x0) == 64) { x = x0; y++; break; }
                    }
                }
            }
            break;
        }
    }
}

/* ==================== 原有函数，修改了字符串和汉字显示以支持 64 字号 ==================== */

void LCD_ShowChar(u16 x, u16 y, u8 num, u16 fc, u16 bc, u8 sizey, u8 mode)
{
    u8 temp, sizex, t, m = 0;
    u16 i, TypefaceNum;
    u16 x0 = x;

    // 新增 100x50 字体支持
    if(sizey == 100)
    {
        LCD_ShowChar100x50(x, y, num, fc, bc, mode);
        return;
    }

    // 其余原有逻辑保持不变（12,16,24,32,64）
    sizex = sizey / 2;
    TypefaceNum = (sizex / 8 + ((sizex % 8) ? 1 : 0)) * sizey;
    num = num - ' ';
    LCD_Address_Set(x, y, x + sizex - 1, y + sizey - 1);
    for(i = 0; i < TypefaceNum; i++)
    {
        if(sizey == 12)      temp = ascii_1206[num][i];
        else if(sizey == 16) temp = ascii_1608[num][i];
        else if(sizey == 24) temp = ascii_2412[num][i];
        else if(sizey == 32) temp = ascii_3216[num][i];
        else if(sizey == 64) temp = ascii_6432[num][i];   // 如已定义
        else return;
        for(t = 0; t < 8; t++)
        {
					for(t = 0; t < 8; t++)
{
    if(!mode)
    {
        if(temp & (0x01 << t))
            LCD_WR_DATA(fc);
        else
            LCD_WR_DATA(bc);

        m++;
        if(m % sizex == 0)
        {
            m = 0;
            break;
        }
    }
    else
    {
        if(temp & (0x01 << t))
            LCD_DrawPoint(x, y, fc);

        x++;
        if((x - x0) == sizex)
        {
            x = x0;
            y++;
            break;
        }
    }
}
        }
    }
}

void LCD_ShowChinese(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey, u8 mode)
{
    while(*s != 0)
    {
        if(sizey == 12)      LCD_ShowChinese12x12(x, y, s, fc, bc, sizey, mode);
        else if(sizey == 16) LCD_ShowChinese16x16(x, y, s, fc, bc, sizey, mode);
        else if(sizey == 24) LCD_ShowChinese24x24(x, y, s, fc, bc, sizey, mode);
        else if(sizey == 32) LCD_ShowChinese32x32(x, y, s, fc, bc, sizey, mode);
        else if(sizey == 64) LCD_ShowChinese64x64(x, y, s, fc, bc, mode);   // 新增
        else return;
        s += 2;
        x += sizey;
    }
}

/* 以下为原有绘图函数，未做修改（省略部分，保持完整） */
/* ... 此处保留原文件中的所有其他函数（LCD_Fill, DrawPoint, DrawLine, 以及各汉字显示函数等）... */
/* 由于篇幅，这里不再重复，实际使用时将原文件后续内容原样粘贴即可 */

/* 注意：实际使用时请将原文件从 LCD_Fill 开始到末尾的所有函数完整复制到此位置 */

/******************************************************************************
      函数说明：在指定区域填充颜色
      入口数据：xsta,ysta   起始坐标
                xend,yend   终止坐标
								color       要填充的颜色
      返回值：  无
******************************************************************************/
void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color)
{          
	u16 i,j; 
	LCD_Address_Set(xsta,ysta,xend-1,yend-1);//设置显示范围
	for(i=ysta;i<yend;i++)
	{													   	 	
		for(j=xsta;j<xend;j++)
		{
			LCD_WR_DATA(color);
		}
	} 					  	    
}

/******************************************************************************
      函数说明：在指定位置画点
      入口数据：x,y 画点坐标
                color 点的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawPoint(u16 x,u16 y,u16 color)
{
	LCD_Address_Set(x,y,x,y);//设置光标位置 
	LCD_WR_DATA(color);
} 


/******************************************************************************
      函数说明：画线
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
                color   线的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawLine(u16 x1,u16 y1,u16 x2,u16 y2,u16 color)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1;
	uRow=x1;//画线起点坐标
	uCol=y1;
	if(delta_x>0)incx=1; //设置单步方向 
	else if (delta_x==0)incx=0;//垂直线 
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;//水平线 
	else {incy=-1;delta_y=-delta_y;}
	if(delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
	else distance=delta_y;
	for(t=0;t<distance+1;t++)
	{
		LCD_DrawPoint(uRow,uCol,color);//画点
		xerr+=delta_x;
		yerr+=delta_y;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}


/******************************************************************************
      函数说明：画矩形
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
                color   矩形的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color)
{
	LCD_DrawLine(x1,y1,x2,y1,color);
	LCD_DrawLine(x1,y1,x1,y2,color);
	LCD_DrawLine(x1,y2,x2,y2,color);
	LCD_DrawLine(x2,y1,x2,y2,color);
}


/******************************************************************************
      函数说明：画圆
      入口数据：x0,y0   圆心坐标
                r       半径
                color   圆的颜色
      返回值：  无
******************************************************************************/
void Draw_Circle(u16 x0,u16 y0,u8 r,u16 color)
{
	int a,b;
	a=0;b=r;	  
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a,color);             //3           
		LCD_DrawPoint(x0+b,y0-a,color);             //0           
		LCD_DrawPoint(x0-a,y0+b,color);             //1                
		LCD_DrawPoint(x0-a,y0-b,color);             //2             
		LCD_DrawPoint(x0+b,y0+a,color);             //4               
		LCD_DrawPoint(x0+a,y0-b,color);             //5
		LCD_DrawPoint(x0+a,y0+b,color);             //6 
		LCD_DrawPoint(x0-b,y0+a,color);             //7
		a++;
		if((a*a+b*b)>(r*r))//判断要画的点是否过远
		{
			b--;
		}
	}
}

/******************************************************************************
 * 函数说明：显示单个64x64汉字（基于32x32字库缩放）
 * 入口数据：x,y显示坐标
 *           *s 要显示的汉字（2字节内码）
 *           fc 字的颜色
 *           bc 字的背景色
 *           sizey 字号（固定64）
 *           mode: 0非叠加模式（覆盖背景）  1叠加模式（仅画前景点）
 * 返回值：  无
 ******************************************************************************/


/******************************************************************************
      函数说明：显示单个12x12汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese12x12(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//汉字数目
	u16 TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	                         
	HZnum=sizeof(tfont12)/sizeof(typFNT_GB12);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if((tfont12[k].Index[0]==*(s))&&(tfont12[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
					{
						if(tfont12[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//叠加方式
					{
						if(tfont12[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
} 

/******************************************************************************
      函数说明：显示单个16x16汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese16x16(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//汉字数目
	u16 TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
  TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont16)/sizeof(typFNT_GB16);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont16[k].Index[0]==*(s))&&(tfont16[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
					{
						if(tfont16[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//叠加方式
					{
						if(tfont16[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
} 


/******************************************************************************
      函数说明：显示单个24x24汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese24x24(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//汉字数目
	u16 TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont24)/sizeof(typFNT_GB24);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont24[k].Index[0]==*(s))&&(tfont24[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
					{
						if(tfont24[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//叠加方式
					{
						if(tfont24[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
} 

/******************************************************************************
      函数说明：显示单个32x32汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese32x32(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//汉字数目
	u16 TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont32)/sizeof(typFNT_GB32);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont32[k].Index[0]==*(s))&&(tfont32[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
					{
						if(tfont32[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//叠加方式
					{
						if(tfont32[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
}


/******************************************************************************
      函数说明：显示单个字符
      入口数据：x,y显示坐标
                num 要显示的字符
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
/******************************************************************************
 * 辅助函数：缩放显示字符（基于原始点阵，最近邻放大）
 * x,y     : 目标左上角
 * num     : 字符索引（已减 ' '）
 * fc,bc   : 前景色、背景色
 * dest_sizey : 目标字符高度（如64）
 * mode    : 0非叠加 1叠加
 * src_font: 原始点阵数据（如 ascii_3216[num]）
 * src_h, src_w : 原始点阵高宽（固定32,16）
 ******************************************************************************/



/******************************************************************************
      函数说明：显示字符串
      入口数据：x,y显示坐标
                *p 要显示的字符串
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowString(u16 x,u16 y,const u8 *p,u16 fc,u16 bc,u8 sizey,u8 mode)
{         
	while(*p!='\0')
	{       
		LCD_ShowChar(x,y,*p,fc,bc,sizey,mode);
		x+=sizey/2;
		p++;
	}  
}


/******************************************************************************
      函数说明：显示数字
      入口数据：m底数，n指数
      返回值：  无
******************************************************************************/
u32 mypow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;
	return result;
}


/******************************************************************************
      函数说明：显示整数变量
      入口数据：x,y显示坐标
                num 要显示整数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void LCD_ShowIntNum(u16 x,u16 y,u16 num,u8 len,u16 fc,u16 bc,u8 sizey)
{         	
	u8 t,temp;
	u8 enshow=0;
	u8 sizex=sizey/2;
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
} 


/******************************************************************************
      函数说明：显示两位小数变量
      入口数据：x,y显示坐标
                num 要显示小数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void LCD_ShowFloatNum1(u16 x,u16 y,float num,u8 len,u16 fc,u16 bc,u8 sizey)
{         	
	u8 t,temp,sizex;
	u16 num1;
	sizex=sizey/2;
	num1=num*100;
	for(t=0;t<len;t++)
	{
		temp=(num1/mypow(10,len-t-1))%10;
		if(t==(len-2))
		{
			LCD_ShowChar(x+(len-2)*sizex,y,'.',fc,bc,sizey,0);
			t++;
			len+=1;
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
}


/******************************************************************************
      函数说明：显示图片
      入口数据：x,y起点坐标
                length 图片长度
                width  图片宽度
                pic[]  图片数组    
      返回值：  无
******************************************************************************/
void LCD_ShowPicture(u16 x,u16 y,u16 length,u16 width,const u8 pic[])
{
	u16 i,j;
	u32 k=0;
	LCD_Address_Set(x,y,x+length-1,y+width-1);
	for(i=0;i<length;i++)
	{
		for(j=0;j<width;j++)
		{
			LCD_WR_DATA8(pic[k*2]);
			LCD_WR_DATA8(pic[k*2+1]);
			k++;
		}
	}			
}
