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

//LCD��Ҫ������
typedef struct  
{										    
	uint16_t width;			//LCD ���
	uint16_t height;			//LCD �߶�
	uint16_t id;				//LCD ID
	uint8_t  dir;			//���������������ƣ�0��������1��������	
	uint16_t	wramcmd;		//��ʼдgramָ��
	uint16_t  setxcmd;		//����x����ָ��
	uint16_t  setycmd;		//����y����ָ�� 
}_lcd_dev; 	  

//LCD����
extern _lcd_dev lcddev;	//����LCD��Ҫ����
//LCD�Ļ�����ɫ�ͱ���ɫ	   
extern uint16_t  POINT_COLOR;//Ĭ�Ϻ�ɫ    
extern uint16_t  BACK_COLOR; //������ɫ.Ĭ��Ϊ��ɫ


#define USE_HORIZONTAL		1	//����Ϊ0  ����Ϊ1

//////////////////////////////////////////////////////////////////////////////////	 
//-----------------LCD�˿ڶ���---------------- 
#define	LCD_LED_OPEN 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, ENABLE) 		//LCD����    		 PC2	    
#define	LCD_LED_CLOSE 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, DISABLE)  		//LCD����    		 PC2	

//LCD��ַ�ṹ��
typedef struct
{
	__IO uint16_t LCD_REG;
	__IO uint16_t LCD_RAM;
} LCD_TypeDef;

//ʹ��NOR/SRAM�� Bank1.sector4,��ַλHADDR[27,26]=11 A6��Ϊ�������������� 
//ע������ʱSTM32�ڲ�������һλ����! 111 1110=0X7E			    
#define LCD_BASE        ((uint32_t)(0x6C000000 | 0x0000007E))	//0x0000007E	
#define LCD             ((LCD_TypeDef *) LCD_BASE)

#define LCD_ADDRESS		LCD->LCD_RAM

//������ɫ
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
#define BROWN 			 0XBC40 //��ɫ
#define BRRED 			 0XFC07 //�غ�ɫ
#define GRAY  			 0X8430 //��ɫ
//GUI��ɫ

#define DARKBLUE      	 0X01CF	//����ɫ
#define LIGHTBLUE      	 0X7D7C	//ǳ��ɫ  
#define GRAYBLUE       	 0X5458 //����ɫ
//������ɫΪPANEL����ɫ 

#define LIGHTGREEN     	 0X841F //ǳ��ɫ
//#define LIGHTGRAY        0XEF5B //ǳ��ɫ(PANNEL)
#define LGRAY 			 0XC618 //ǳ��ɫ(PANNEL),���屳��ɫ

#define LGRAYBLUE        0XA651 //ǳ����ɫ(�м����ɫ)
#define LBBLUE           0X2B12 //ǳ����ɫ(ѡ����Ŀ�ķ�ɫ)

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
