#ifndef __MY_LCD_H
#include "stdlib.h" 
#include "stm32f4xx_hal.h"

#define  LCD_BYTES_IN_PIXEL       2 //Not done!!!
#define  LCD_PIXEL_WIDTH          ((uint16_t)160)
#define  LCD_PIXEL_HEIGHT         ((uint16_t)120)
#define  LCD_PIXELS_CNT           ((uint32_t)LCD_PIXEL_WIDTH * (uint32_t)LCD_PIXEL_HEIGHT)

#define  LCD_FRAME_BUFFER0        ((uint32_t)0x20000000) 
//#define  LCD_FRAME_BUFFER0        ((uint32_t)0x68000000)	//use extern sram fps is slower
//#define  LCD_FRAME_BUFFER0        ((uint32_t)(0x6C000000 | 0x0000007E))	//0x0000007E	

//#define LCD_ADDRESS 			  ((uint32_t)(0x6C002000)	//0x0000007E	

#define  LCD_BUFFER_SIZE          ((uint32_t)(LCD_PIXELS_CNT * LCD_BYTES_IN_PIXEL))

#define  LCD_FRAME_BUFFER1        (LCD_FRAME_BUFFER0 + LCD_BUFFER_SIZE)
#define  UVC_FRAMEBUFFER0         (LCD_FRAME_BUFFER1 + LCD_BUFFER_SIZE)
#define  UVC_FRAMEBUFFER1         (LCD_FRAME_BUFFER0 + UVC_MAX_FRAME_SIZE)

//LCD重要参数集
typedef struct  
{										    
	uint16_t width;			//LCD 宽度
	uint16_t height;			//LCD 高度
	uint16_t id;				//LCD ID
	uint8_t  dir;			//横屏还是竖屏控制：0，竖屏；1，横屏。	
	uint16_t	wramcmd;		//开始写gram指令
	uint16_t  setxcmd;		//设置x坐标指令
	uint16_t  setycmd;		//设置y坐标指令 
}_lcd_dev; 	  

//LCD参数
extern _lcd_dev lcddev;	//管理LCD重要参数
//LCD的画笔颜色和背景色	   
extern uint16_t  POINT_COLOR;//默认红色    
extern uint16_t  BACK_COLOR; //背景颜色.默认为白色


#define USE_HORIZONTAL		1	//竖屏为0  横屏为1

//////////////////////////////////////////////////////////////////////////////////	 
//-----------------LCD端口定义---------------- 
#define	LCD_LED_OPEN 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, ENABLE) 		//LCD背光    		 PC2	    
#define	LCD_LED_CLOSE 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, DISABLE)  		//LCD背光    		 PC2	

//LCD地址结构体
typedef struct
{
	__IO uint16_t LCD_REG;
	__IO uint16_t LCD_RAM;
} LCD_TypeDef;

//使用NOR/SRAM的 Bank1.sector4,地址位HADDR[27,26]=11 A6作为数据命令区分线 
//注意设置时STM32内部会右移一位对其! 111 1110=0X7E			    
#define LCD_BASE        ((uint32_t)(0x6C000000 | 0x0000007E))	//0x0000007E	
#define LCD             ((LCD_TypeDef *) LCD_BASE)

#define LCD_ADDRESS		LCD->LCD_RAM

//画笔颜色
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	 0x001F  
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40 //棕色
#define BRRED 			 0XFC07 //棕红色
#define GRAY  			 0X8430 //灰色
//GUI颜色

#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色  
#define GRAYBLUE       	 0X5458 //灰蓝色
//以上三色为PANEL的颜色 

#define LIGHTGREEN     	 0X841F //浅绿色
//#define LIGHTGRAY        0XEF5B //浅灰色(PANNEL)
#define LGRAY 			 0XC618 //浅灰色(PANNEL),窗体背景色

#define LGRAYBLUE        0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE           0X2B12 //浅棕蓝色(选择条目的反色)

void LcdInit(void);
void LCD_Clear(uint16_t color);
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);
void BlockWrite(unsigned int Xstart,unsigned int Xend,unsigned int Ystart,unsigned int Yend);
void LCD_DrawPoint(uint16_t x,uint16_t y);
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_ShowChar(uint16_t x,uint16_t y,uint16_t fc, uint16_t bc, uint8_t num,uint8_t size,uint8_t mode);
void LCD_ShowString(uint16_t x,uint16_t y,uint8_t size,uint8_t *p,uint8_t mode);
void LCD_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color);
void LCD_SetWindows(uint16_t xStar, uint16_t yStar,uint16_t xEnd,uint16_t yEnd);

void Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r);
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_SSD_BackLightSet(uint8_t pwm);

void lcd_switch_to_single_buffer_mode(void);
void lcd_switch_buffer(void);

void LCD_DrawImage(uint16_t sx, uint16_t sy, uint8_t* source);


void lcd_draw_yuyv_picture(uint8_t* source);
void lcd_convert_yuyv_pixels(uint8_t* data, uint16_t* pixel0, uint16_t* pixel1);
uint16_t lcd_convert_yuyv_to_rgb(uint8_t y, uint8_t u, uint8_t v);

#endif
