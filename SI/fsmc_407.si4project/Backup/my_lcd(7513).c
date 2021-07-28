#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_fsmc.h"

#include "my_lcd.h"
#include "delay.h"
#include "font.h"

#include "usbh_video.h"
#include "sram.h"

SRAM_HandleTypeDef hsram1;
extern SRAM_HandleTypeDef hsramMy;

//LCD的画笔颜色和背景色	   
uint16_t POINT_COLOR=0x0000;	//画笔颜色
uint16_t BACK_COLOR=0xFFFF;  //背景色 

uint32_t lcd_current_framebuffer = LCD_FRAME_BUFFER0;//Buffer that is drawn now
uint32_t lcd_shadow_framebuffer = LCD_FRAME_BUFFER1;//Shadow buffer

//管理LCD重要参数
//默认为竖屏
_lcd_dev lcddev;

//写寄存器函数
//regval:寄存器值
void LCD_WR_REG(__IO uint16_t regval)
{   
	regval=regval;		//使用-O2优化的时候,必须插入的延时
	LCD->LCD_REG=regval;//写入	要写的寄存器序号	 
}

//写LCD数据
//data:要写入的值
void LCD_WR_DATA(__IO uint16_t data)
{	  
	data=data;			//使用-O2优化的时候,必须插入的延时
	LCD->LCD_RAM=data;		 
}

//读LCD数据
//返回值:读到的值
uint16_t LCD_RD_DATA(void)
{
	__IO uint16_t ram;			//防止被优化
	ram=LCD->LCD_RAM;	
	return ram;	 
}		

//写寄存器
//LCD_Reg:寄存器地址
//LCD_RegValue:要写入的数据
void LCD_WriteReg(uint16_t LCD_Reg, uint16_t LCD_RegValue)
{	
	LCD->LCD_REG = LCD_Reg;		//写入要写的寄存器序号	 
	LCD->LCD_RAM = LCD_RegValue;//写入数据	    		 
}	   

//读寄存器
//LCD_Reg:寄存器地址
//返回值:读到的数据
uint16_t LCD_ReadReg(uint16_t LCD_Reg)
{										   
	LCD_WR_REG(LCD_Reg);		//写入要读的寄存器序号
	delay_us(5);		  
	return LCD_RD_DATA();		//返回读到的值
}   

//开始写GRAM
void LCD_WriteRAM_Prepare(void)
{
 	LCD->LCD_REG=lcddev.wramcmd;	  
}	

//LCD写GRAM
//RGB_Code:颜色值
void LCD_WriteRAM(uint16_t RGB_Code)
{							    
	LCD->LCD_RAM = RGB_Code;//写十六位GRAM
}

//从ILI93xx读出的数据为GBR格式，而我们写入的时候为RGB格式。
//通过该函数转换
//c:GBR格式的颜色值
//返回值：RGB格式的颜色值
uint16_t LCD_BGR2RGB(uint16_t c)
{
	uint16_t  r,g,b,rgb;   
	b=(c>>0)&0x1f;
	g=(c>>5)&0x3f;
	r=(c>>11)&0x1f;	 
	rgb=(b<<11)+(g<<5)+(r<<0);		 
	return(rgb);
} 

//当mdk -O1时间优化时需要设置
//延时i
void opt_delay(uint8_t i)
{
	while(i--);
}

//******************************************************************
//函数名：  LCD_DrawPoint
//功能：    在指定位置写入一个像素点数据
//输入参数：(x,y):光标坐标
//返回值：  无
//修改记录：无
//******************************************************************
void LCD_DrawPoint(uint16_t x,uint16_t y)
{
	LCD_SetCursor(x,y);//设置光标位置 
	LCD_WriteRAM_Prepare();	//开始写入GRAM
	LCD_WR_DATA(POINT_COLOR); 
}

void BlockWrite(unsigned int Xstart,unsigned int Xend,unsigned int Ystart,unsigned int Yend) 
{
	//ILI9481

	LCD_WR_REG(0x2a00);   
	LCD_WR_DATA(Xstart>>8);
	LCD_WR_REG(0x2a01); 
	LCD_WR_DATA(Xstart&0xff);
	LCD_WR_REG(0x2a02); 
	LCD_WR_DATA(Xend>>8);
	LCD_WR_REG(0x2a03); 
	LCD_WR_DATA(Xend&0xff);

	LCD_WR_REG(0x2b00);   
	LCD_WR_DATA(Ystart>>8);
	LCD_WR_REG(0x2b01); 
	LCD_WR_DATA(Ystart&0xff);
	LCD_WR_REG(0x2b02); 
	LCD_WR_DATA(Yend>>8);
	LCD_WR_REG(0x2b03); 
	LCD_WR_DATA(Yend&0xff);

	LCD_WR_REG(0x2c00);
}

//SSD1963 背光设置
//pwm:背光等级,0~100.越大越亮.
void LCD_SSD_BackLightSet(uint8_t pwm)
{	
	LCD_WR_REG(0xBE);	//配置PWM输出
	LCD_WR_DATA(0x05);	//1设置PWM频率
	LCD_WR_DATA(pwm*2.55);//2设置PWM占空比
	LCD_WR_DATA(0x01);	//3设置C
	LCD_WR_DATA(0xFF);	//4设置D
	LCD_WR_DATA(0x00);	//5设置E
	LCD_WR_DATA(0x00);	//6设置F
}

void LCD_SetWindows(uint16_t xStar, uint16_t yStar,uint16_t xEnd,uint16_t yEnd)
{	
	LCD_WR_REG(0x2A00);	
	LCD_WR_DATA(xStar>>8);
	LCD_WR_REG(0x2A01);	
	LCD_WR_DATA(xStar);
	
	LCD_WR_REG(0x2A02);	
	LCD_WR_DATA(xEnd>>8);
	LCD_WR_REG(0x2A03);	
	LCD_WR_DATA(xEnd);
		
	LCD_WR_REG(0x2B00);	
	LCD_WR_DATA(yStar>>8);
	LCD_WR_REG(0x2B01);	
	LCD_WR_DATA(yStar);
	
	LCD_WR_REG(0x2B02);	
	LCD_WR_DATA(yEnd>>8);
	LCD_WR_REG(0x2B03);	
	LCD_WR_DATA(yEnd);


	LCD_WriteRAM_Prepare();	//开始写入GRAM		

}

//设置LCD参数
//方便进行横竖屏模式切换
void LCD_SetParam(void)
{ 	
	lcddev.width=480;
	lcddev.height=800;
	lcddev.wramcmd=0x2c00;
	
#if USE_HORIZONTAL	//使用横屏	  
	lcddev.dir=1;//横屏
	lcddev.width=800;
	lcddev.height=480;
			
	LCD_WriteReg(0x3600,0x65);


#else//竖屏
	lcddev.dir=0;//竖屏				 	 		
	lcddev.width=480;
	lcddev.height=800;
	
	LCD_WriteReg(0x3600,0x00);

#endif
}	

//清屏函数
//color:要清屏的填充色
void LCD_Clear(uint16_t color)
{
	uint32_t index=0;
	uint16_t i,j;      
	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);
	for(i=0;i<lcddev.height;i++)
	{
		for(j=0;j<lcddev.width;j++)
		//LCD_WR_DATA(Color>>8); 
		LCD_WR_DATA(color);   
	}
}  

/*************************************************
函数名：LCD_SetCursor
功能：设置光标位置
入口参数：xy坐标
返回值：无
*************************************************/
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{	  	    			
	LCD_SetWindows(Xpos,Ypos,Xpos,Ypos);	
} 

//在指定区域内填充单个颜色
//(sx,sy),(ex,ey):填充矩形对角坐标,区域大小为:(ex-sx+1)*(ey-sy+1)   
//color:要填充的颜色
void LCD_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color)
{    
	uint16_t i,j;
	uint16_t width = ex-sx+1;		//得到宽度
	uint16_t height = ey-sy+1;		//高度
	LCD_SetWindows(sx, sy, ex-1, ey-1);
	for(i=0; i<height; i++)
	{
		for(j=0; j<width; j++)
			LCD_WR_DATA(color);		//写入数据
	}
	LCD_SetWindows(0, 0, lcddev.width-1, lcddev.height-1);		//恢复窗口设置为全屏
}

void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	uint16_t t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 

	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; //设置单步方向 
	else if(delta_x==0)incx=0;//垂直线 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;//水平线 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )//画线输出 
	{  
		LCD_DrawPoint(uRow,uCol);//画点 
		xerr+=delta_x ; 
		yerr+=delta_y ; 
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

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#if 1
extern const unsigned char asc2_1206[95][12];
extern const unsigned char asc2_1608[95][16];
void LCD_ShowChar(uint16_t x,uint16_t y,uint16_t fc, uint16_t bc, uint8_t num,uint8_t size,uint8_t mode)
{  
    uint8_t temp;
    uint8_t pos,t;
	uint16_t colortemp=POINT_COLOR;      
		   
	num=num-' ';//得到偏移后的值
	LCD_SetWindows(x,y,x+size/2-1,y+size-1);//设置单个文字显示窗口
	if(!mode) //非叠加方式
	{
		
		for(pos=0;pos<size;pos++)
		{
			if(size==12)temp=asc2_1206[num][pos];//调用1206字体
			else temp=asc2_1608[num][pos];		 //调用1608字体
			for(t=0;t<size/2;t++)
		    {                 
		        if(temp&0x01)LCD_WR_DATA(fc); 
				else LCD_WR_DATA(bc); 
				temp>>=1; 
				
		    }
			
		}	
	}else//叠加方式
	{
		for(pos=0;pos<size;pos++)
		{
			if(size==12)temp=asc2_1206[num][pos];//调用1206字体
			else temp=asc2_1608[num][pos];		 //调用1608字体
			for(t=0;t<size/2;t++)
		    {   
				POINT_COLOR=fc;              
		        if(temp&0x01)LCD_DrawPoint(x+t,y+pos);//画一个点    
		        temp>>=1; 
		    }
		}
	}
	POINT_COLOR=colortemp;	
	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);//恢复窗口为全屏    	   	 	  
} 

void LCD_ShowString(uint16_t x,uint16_t y,uint8_t size,uint8_t *p,uint8_t mode)
{         
    while((*p<='~')&&(*p>=' '))//判断是不是非法字符!
    {   
		if(x>(lcddev.width-1)||y>(lcddev.height-1)) 
		return;     
        LCD_ShowChar(x,y,POINT_COLOR,BACK_COLOR,*p,size,mode);
        x+=size/2;
        p++;
    }  
} 
#endif

//画矩形
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	LCD_DrawLine(x1, y1, x2, y1);
	LCD_DrawLine(x1, y1, x1, y2);
	LCD_DrawLine(x1, y2, x2, y2);
	LCD_DrawLine(x2, y1, x2, y2);
}

//画圆
void Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r)
{
	int a,b;
	int di;
	a=0;b=r;
	di=3-(r<<1);	//判断下个点位置的标志
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a);             //3           
		LCD_DrawPoint(x0+b,y0-a);             //0           
		LCD_DrawPoint(x0-a,y0+b);             //1       
		LCD_DrawPoint(x0-b,y0-a);             //7           
		LCD_DrawPoint(x0-a,y0-b);             //2             
		LCD_DrawPoint(x0+b,y0+a);             //4               
		LCD_DrawPoint(x0+a,y0-b);             //5
		LCD_DrawPoint(x0+a,y0+b);             //6 
		LCD_DrawPoint(x0-b,y0+a);             
		a++;
		//使用Breasenham算法画圆
		if(di<0)di +=4*a+6;
		else
		{
			di+=10+4*(a-b);
			b--;
		}
		LCD_DrawPoint(x0+a, y0+b);
	}
}

void lcd_set_pixel(uint16_t x, uint16_t y, uint16_t color)
{
  LCD_SetCursor(x,y);//设置光标位置 
  LCD_WriteRAM_Prepare();	//开始写入GRAM
  LCD_WR_DATA(color); 
}

void lcd_switch_to_single_buffer_mode(void)
{
  lcd_current_framebuffer = LCD_FRAME_BUFFER0;
  
//  HAL_SRAM_WriteOperation_Enable(&hsram1);
//  HAL_SRAM_Write_16b(&hsram1, (uint32_t*)LCD_ADDRESS, (uint16_t*)lcd_current_framebuffer, LCD_BUFFER_SIZE);
}

void lcd_switch_buffer(void)
{
  if(lcd_current_framebuffer == LCD_FRAME_BUFFER0)
  {
	lcd_current_framebuffer = LCD_FRAME_BUFFER1;
  }
  else
  {
	lcd_current_framebuffer = LCD_FRAME_BUFFER0;
  }
  /*use exten sram, the fps is slower then in-sram*/
//  HAL_SRAM_WriteOperation_Enable(&hsramMy);
//  HAL_SRAM_Write_16b(&hsramMy, (uint32_t*)BANK1_SRAM3_ADDR, (uint16_t*)lcd_current_framebuffer, LCD_BUFFER_SIZE);

  /*not success to use lcd_sram*/
//  HAL_SRAM_WriteOperation_Enable(&hsram1);
//  HAL_SRAM_Write_16b(&hsram1, (uint32_t*)LCD_ADDRESS, (uint16_t*)lcd_current_framebuffer, LCD_BUFFER_SIZE);	
}

void lcd_draw_yuyv_picture(uint8_t* source)
{
  uint16_t y;
  uint16_t x;
  
  //uint8_t* src_ptr = (uint8_t*)&tmp_packet_framebuffer[0];
  uint16_t* image_ptr = 0;
  
  for (y = 0; y < UVC_TARGET_HEIGHT; y++)
  {
    uint32_t offset = lcd_shadow_framebuffer + y * LCD_PIXEL_WIDTH * LCD_BYTES_IN_PIXEL;
    image_ptr = (uint16_t*)offset;
    
    for (x = 0; x < UVC_TARGET_WIDTH; x+= 2)
    {
      lcd_convert_yuyv_pixels(source, image_ptr, (image_ptr + 1));
      source+= 4;//2 pixels take 4 raw bytes
      image_ptr+= 2;//2 pixels
    }
  }
}

void LCD_DrawImage(uint16_t sx, uint16_t sy, uint8_t* source)
{    
	uint16_t x=0;
    uint16_t y;
//	uint16_t width = ex-sx+1;		//得到宽度
//	uint16_t height = ey-sy+1;		//高度
    uint16_t* image_ptr = 0;
//	LCD_SetWindows(0, 0, 320-1, 240-1);
	for(y=0; y<UVC_TARGET_HEIGHT; y++)
	{
        uint32_t offset = lcd_shadow_framebuffer + y * LCD_PIXEL_WIDTH * LCD_BYTES_IN_PIXEL;
        image_ptr = (uint16_t*)offset;
		for(x=0; x<UVC_TARGET_WIDTH; x+=2)
        {
            lcd_convert_yuyv_pixels(source, image_ptr, (image_ptr + 1));
            source+= 4;//2 pixels take 4 raw bytes
			lcd_set_pixel(x +sx, y +sy, *image_ptr);		//写入数据
            lcd_set_pixel(x +sx +1, y +sy, *(image_ptr+1));		//写入数据
//            LCD_WR_DATA(*image_ptr);		//写入数据
            image_ptr+= 2;//2 pixels
        }
	}
	LCD_SetWindows(0, 0, lcddev.width-1, lcddev.height-1);		//恢复窗口设置为全屏
}

//data - pointer to a Y0 pixel
//pixel0 - pointer to RGB656 pixel, color data would be pritten there
//pixel1 - pointer to RGB656 pixel, color data would be pritten there
void lcd_convert_yuyv_pixels(uint8_t* data, uint16_t* pixel0, uint16_t* pixel1)
{
  uint8_t y0 = data[0];
  uint8_t u  = data[1];
  uint8_t y1 = data[2];
  uint8_t v  = data[3];
  
  *pixel0 = lcd_convert_yuyv_to_rgb(y0, u, v);
  *pixel1 = lcd_convert_yuyv_to_rgb(y1, u, v);
}

//return rbg565
uint16_t lcd_convert_yuyv_to_rgb(uint8_t y, uint8_t u, uint8_t v)
{
   int16_t r1, g1, b1;
   uint16_t color = 0;

   // replaces floating point coefficients
   int16_t c = y-16;
   int16_t d = u - 128;
   int16_t e = v - 128;    

   // Conversion that avoids floating point
   r1 = (298 * c + 409 * e + 128) >> 8;
   g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
   b1 = (298 * c + 516 * d + 128) >> 8;

   // Computed values may need clipping.
   if (r1 > 255) 
     r1 = 255;
   if (g1 > 255) 
     g1 = 255;
   if (b1 > 255) 
     b1 = 255;

   if (r1 < 0) 
     r1 = 0;
   if (g1 < 0) 
     g1 = 0;
   if (b1 < 0) 
     b1 = 0;
   
   r1 = r1 >> 3; //5 bits
   g1 = g1 >> 2; //6 bits
   b1 = b1 >> 3; //5 bits
   
   color|= r1 << (5 + 6);
   color|= g1 << 5;
   color|= b1;
   
   return color;
}


void LcdInit(void)
{
	__IO uint32_t i=0;

	GPIO_InitTypeDef GPIO_InitStructure;

	FSMC_NORSRAM_TimingTypeDef readWriteTiming;
	FSMC_NORSRAM_TimingTypeDef writeTiming;
	
	__HAL_RCC_FSMC_CLK_ENABLE();
	
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	//configure gpiob
	GPIO_InitStructure.Pin =  GPIO_PIN_15;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Alternate = GPIO_AF12_FSMC;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = GPIO_PIN_2;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Alternate = GPIO_AF12_FSMC;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = (3<<0)|(3<<4)|(7<<8)|(3<<14);
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Alternate = GPIO_AF12_FSMC;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = (0x1ff<<7);
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Alternate = GPIO_AF12_FSMC;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = GPIO_PIN_12;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Alternate = GPIO_AF12_FSMC;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = GPIO_PIN_12;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Alternate = GPIO_AF12_FSMC;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStructure);

	readWriteTiming.AddressSetupTime = 0xf;
	readWriteTiming.AddressHoldTime = 0x00;
	readWriteTiming.DataSetupTime = 60;
	readWriteTiming.BusTurnAroundDuration = 0x00;
	readWriteTiming.CLKDivision = 0x00;
	readWriteTiming.DataLatency = 0x00;
	readWriteTiming.AccessMode = FSMC_ACCESS_MODE_A;

	writeTiming.AddressSetupTime = 0x9;
	writeTiming.AddressHoldTime = 0x00;
	writeTiming.DataSetupTime = 8;
	writeTiming.BusTurnAroundDuration = 0x00;
	writeTiming.CLKDivision = 0x00;
	writeTiming.DataLatency = 0x00;
	writeTiming.AccessMode = FSMC_ACCESS_MODE_A;
	
	hsram1.Instance = FSMC_NORSRAM_DEVICE;
  	hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
	
	hsram1.Init.NSBank = FSMC_NORSRAM_BANK4;
	hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
	hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
	hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;	
	hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
	hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
	hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
	hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
	hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
	hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
	hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
	hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_ENABLE;
	hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
	hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;

	if(HAL_SRAM_Init(&hsram1, &readWriteTiming, &writeTiming) != HAL_OK)
	{
		Error_Handler();
	}
	
	LCD_WR_REG(0xff00);   LCD_WR_DATA(0x80);		//enable access command2
	LCD_WR_REG(0xff01);   LCD_WR_DATA(0x09);		//enable access command2
	LCD_WR_REG(0xff02);   LCD_WR_DATA(0x01);		//enable access command2
	LCD_WR_REG(0xff80);   LCD_WR_DATA(0x80);		//enable access command2
	LCD_WR_REG(0xff81);   LCD_WR_DATA(0x09);		//enable access command2
	LCD_WR_REG(0xff03);   LCD_WR_DATA(0x01);		//
	LCD_WR_REG(0xc5b1);   LCD_WR_DATA(0xA9);		//power control

	LCD_WR_REG(0xc591);   LCD_WR_DATA(0x0F);               //power control
	LCD_WR_REG(0xc0B4);   LCD_WR_DATA(0x50);
			
	//panel driving mode : column inversion

	//////	gamma
	LCD_WR_REG(0xE100);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xE101);   LCD_WR_DATA(0x09);
	LCD_WR_REG(0xE102);   LCD_WR_DATA(0x0F);
	LCD_WR_REG(0xE103);   LCD_WR_DATA(0x0E);
	LCD_WR_REG(0xE104);   LCD_WR_DATA(0x07);
	LCD_WR_REG(0xE105);   LCD_WR_DATA(0x10);
	LCD_WR_REG(0xE106);   LCD_WR_DATA(0x0B);
	LCD_WR_REG(0xE107);   LCD_WR_DATA(0x0A);
	LCD_WR_REG(0xE108);   LCD_WR_DATA(0x04);
	LCD_WR_REG(0xE109);   LCD_WR_DATA(0x07);
	LCD_WR_REG(0xE10A);   LCD_WR_DATA(0x0B);
	LCD_WR_REG(0xE10B);   LCD_WR_DATA(0x08);
	LCD_WR_REG(0xE10C);   LCD_WR_DATA(0x0F);
	LCD_WR_REG(0xE10D);   LCD_WR_DATA(0x10);
	LCD_WR_REG(0xE10E);   LCD_WR_DATA(0x0A);
	LCD_WR_REG(0xE10F);   LCD_WR_DATA(0x01);	
	LCD_WR_REG(0xE200);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xE201);   LCD_WR_DATA(0x09);
	LCD_WR_REG(0xE202);   LCD_WR_DATA(0x0F);
	LCD_WR_REG(0xE203);   LCD_WR_DATA(0x0E);
	LCD_WR_REG(0xE204);   LCD_WR_DATA(0x07);
	LCD_WR_REG(0xE205);   LCD_WR_DATA(0x10);
	LCD_WR_REG(0xE206);   LCD_WR_DATA(0x0B);
	LCD_WR_REG(0xE207);   LCD_WR_DATA(0x0A);
	LCD_WR_REG(0xE208);   LCD_WR_DATA(0x04);
	LCD_WR_REG(0xE209);   LCD_WR_DATA(0x07);
	LCD_WR_REG(0xE20A);   LCD_WR_DATA(0x0B);
	LCD_WR_REG(0xE20B);   LCD_WR_DATA(0x08);
	LCD_WR_REG(0xE20C);   LCD_WR_DATA(0x0F);
	LCD_WR_REG(0xE20D);   LCD_WR_DATA(0x10);
	LCD_WR_REG(0xE20E);   LCD_WR_DATA(0x0A);
	LCD_WR_REG(0xE20F);   LCD_WR_DATA(0x01);	
	LCD_WR_REG(0xD900);   LCD_WR_DATA(0x4E);		//vcom setting

	LCD_WR_REG(0xc181);   LCD_WR_DATA(0x66);		//osc=65HZ

	LCD_WR_REG(0xc1a1);   LCD_WR_DATA(0x08);		
	LCD_WR_REG(0xc592);   LCD_WR_DATA(0x01);		//power control

	LCD_WR_REG(0xc595);   LCD_WR_DATA(0x34);		//power control

	LCD_WR_REG(0xd800);   LCD_WR_DATA(0x79);		//GVDD / NGVDD setting

	LCD_WR_REG(0xd801);   LCD_WR_DATA(0x79);		//GVDD / NGVDD setting

	LCD_WR_REG(0xc594);   LCD_WR_DATA(0x33);		//power control

	LCD_WR_REG(0xc0a3);   LCD_WR_DATA(0x1B);       //panel timing setting
	LCD_WR_REG(0xc582);   LCD_WR_DATA(0x83);		//power control

	LCD_WR_REG(0xc481);   LCD_WR_DATA(0x83);		//source driver setting

	LCD_WR_REG(0xc1a1);   LCD_WR_DATA(0x0E);
	LCD_WR_REG(0xb3a6);   LCD_WR_DATA(0x20);
	LCD_WR_REG(0xb3a7);   LCD_WR_DATA(0x01);
	LCD_WR_REG(0xce80);   LCD_WR_DATA(0x85);		// GOA VST

	LCD_WR_REG(0xce81);   LCD_WR_DATA(0x01); 	// GOA VST

	LCD_WR_REG(0xce82);   LCD_WR_DATA(0x00);		// GOA VST	

	LCD_WR_REG(0xce83);   LCD_WR_DATA(0x84);		// GOA VST
	LCD_WR_REG(0xce84);   LCD_WR_DATA(0x01);		// GOA VST
	LCD_WR_REG(0xce85);   LCD_WR_DATA(0x00);		// GOA VST

	LCD_WR_REG(0xcea0);   LCD_WR_DATA(0x18);		// GOA CLK
	LCD_WR_REG(0xcea1);   LCD_WR_DATA(0x04);		// GOA CLK
	LCD_WR_REG(0xcea2);   LCD_WR_DATA(0x03);		// GOA CLK 
	LCD_WR_REG(0xcea3);   LCD_WR_DATA(0x39);		// GOA CLK 
	LCD_WR_REG(0xcea4);   LCD_WR_DATA(0x00);		// GOA CLK 
	LCD_WR_REG(0xcea5);   LCD_WR_DATA(0x00);		// GOA CLK 
	LCD_WR_REG(0xcea6);   LCD_WR_DATA(0x00);		// GOA CLK
	LCD_WR_REG(0xcea7);   LCD_WR_DATA(0x18);		// GOA CLK 
	LCD_WR_REG(0xcea8);   LCD_WR_DATA(0x03);		// GOA CLK
	LCD_WR_REG(0xcea9);   LCD_WR_DATA(0x03);		// GOA CLK 
	LCD_WR_REG(0xceaa);   LCD_WR_DATA(0x3a);		// GOA CLK
	LCD_WR_REG(0xceab);   LCD_WR_DATA(0x00);		// GOA CLK 
	LCD_WR_REG(0xceac);   LCD_WR_DATA(0x00);		// GOA CLK 
	LCD_WR_REG(0xcead);   LCD_WR_DATA(0x00);		// GOA CLK 
	LCD_WR_REG(0xceb0);   LCD_WR_DATA(0x18);		// GOA CLK
	LCD_WR_REG(0xceb1);   LCD_WR_DATA(0x02);		// GOA CLK 
	LCD_WR_REG(0xceb2);   LCD_WR_DATA(0x03); 		// GOA CLK
	LCD_WR_REG(0xceb3);   LCD_WR_DATA(0x3b);		// GOA CLK 
	LCD_WR_REG(0xceb4);   LCD_WR_DATA(0x00);		// GOA CLK 
	LCD_WR_REG(0xceb5);   LCD_WR_DATA(0x00);		// GOA CLK 
	LCD_WR_REG(0xceb6);   LCD_WR_DATA(0x00);		// GOA CLK
	LCD_WR_REG(0xceb7);   LCD_WR_DATA(0x18);		// GOA CLK
	LCD_WR_REG(0xceb8);   LCD_WR_DATA(0x01);		// GOA CLK 
	LCD_WR_REG(0xceb9);   LCD_WR_DATA(0x03);		// GOA CLK 
	LCD_WR_REG(0xceba);   LCD_WR_DATA(0x3c);		// GOA CLK 
	LCD_WR_REG(0xcebb);   LCD_WR_DATA(0x00);		// GOA CLK 
	LCD_WR_REG(0xcebc);   LCD_WR_DATA(0x00);		// GOA CLK 
	LCD_WR_REG(0xcebd);   LCD_WR_DATA(0x00);		// GOA CLK
	LCD_WR_REG(0xcfc0);   LCD_WR_DATA(0x01);		// GOA ECLK 
	LCD_WR_REG(0xcfc1);   LCD_WR_DATA(0x01);		// GOA ECLK
	LCD_WR_REG(0xcfc2);   LCD_WR_DATA(0x20); 		// GOA ECLK

	LCD_WR_REG(0xcfc3);   LCD_WR_DATA(0x20); 		// GOA ECLK

	LCD_WR_REG(0xcfc4);   LCD_WR_DATA(0x00); 		// GOA ECLK

	LCD_WR_REG(0xcfc5);   LCD_WR_DATA(0x00); 		// GOA ECLK

	LCD_WR_REG(0xcfc6);   LCD_WR_DATA(0x01); 		// GOA other options

	LCD_WR_REG(0xcfc7);   LCD_WR_DATA(0x00); 
			
	// GOA signal toggle option setting

	LCD_WR_REG(0xcfc8);   LCD_WR_DATA(0x00); 		//GOA signal toggle option setting
	LCD_WR_REG(0xcfc9);   LCD_WR_DATA(0x00); 
	   
	//GOA signal toggle option setting

	LCD_WR_REG(0xcfd0);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb80);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb81);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb82);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb83);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb84);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb85);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb86);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb87);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb88);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb89);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb90);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb91);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb92);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb93);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb94);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb95);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb96);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb97);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb98);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb99);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb9a);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb9b);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb9c);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb9d);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcb9e);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcba0);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcba1);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcba2);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcba3);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcba4);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcba5);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcba6);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcba7);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcba8);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcba9);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbaa);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbab);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbac);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbad);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbae);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbb0);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbb1);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbb2);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbb3);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbb4);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbb5);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbb6);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbb7);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbb8);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbb9);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbc0);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbc1);   LCD_WR_DATA(0x04);
	LCD_WR_REG(0xcbc2);   LCD_WR_DATA(0x04);
	LCD_WR_REG(0xcbc3);   LCD_WR_DATA(0x04);
	LCD_WR_REG(0xcbc4);   LCD_WR_DATA(0x04);
	LCD_WR_REG(0xcbc5);   LCD_WR_DATA(0x04);
	LCD_WR_REG(0xcbc6);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbc7);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbc8);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbc9);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbca);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbcb);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbcc);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbcd);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbce);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbd0);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbd1);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbd2);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbd3);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbd4);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbd5);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbd6);   LCD_WR_DATA(0x04);
	LCD_WR_REG(0xcbd7);   LCD_WR_DATA(0x04);
	LCD_WR_REG(0xcbd8);   LCD_WR_DATA(0x04);
	LCD_WR_REG(0xcbd9);   LCD_WR_DATA(0x04);
	LCD_WR_REG(0xcbda);   LCD_WR_DATA(0x04);
	LCD_WR_REG(0xcbdb);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbdc);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbdd);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbde);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbe0);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbe1);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbe2);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbe3);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbe4);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbe5);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbe6);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbe7);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbe8);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbe9);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcbf0);   LCD_WR_DATA(0xFF);
	LCD_WR_REG(0xcbf1);   LCD_WR_DATA(0xFF);
	LCD_WR_REG(0xcbf2);   LCD_WR_DATA(0xFF);
	LCD_WR_REG(0xcbf3);   LCD_WR_DATA(0xFF);
	LCD_WR_REG(0xcbf4);   LCD_WR_DATA(0xFF);
	LCD_WR_REG(0xcbf5);   LCD_WR_DATA(0xFF);
	LCD_WR_REG(0xcbf6);   LCD_WR_DATA(0xFF);
	LCD_WR_REG(0xcbf7);   LCD_WR_DATA(0xFF);
	LCD_WR_REG(0xcbf8);   LCD_WR_DATA(0xFF);
	LCD_WR_REG(0xcbf9);   LCD_WR_DATA(0xFF);
	LCD_WR_REG(0xcc80);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcc81);   LCD_WR_DATA(0x26);
	LCD_WR_REG(0xcc82);   LCD_WR_DATA(0x09);
	LCD_WR_REG(0xcc83);   LCD_WR_DATA(0x0B);
	LCD_WR_REG(0xcc84);   LCD_WR_DATA(0x01);
	LCD_WR_REG(0xcc85);   LCD_WR_DATA(0x25);
	LCD_WR_REG(0xcc86);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcc87);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcc88);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcc89);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcc90);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcc91);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcc92);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcc93);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcc94);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcc95);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcc96);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcc97);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcc98);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcc99);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcc9a);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcc9b);   LCD_WR_DATA(0x26);
	LCD_WR_REG(0xcc9c);   LCD_WR_DATA(0x0A);
	LCD_WR_REG(0xcc9d);   LCD_WR_DATA(0x0C);
	LCD_WR_REG(0xcc9e);   LCD_WR_DATA(0x02);
	LCD_WR_REG(0xcca0);   LCD_WR_DATA(0x25);
	LCD_WR_REG(0xcca1);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcca2);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcca3);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcca4);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcca5);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcca6);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcca7);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcca8);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcca9);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccaa);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccab);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccac);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccad);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccae);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccb0);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccb1);   LCD_WR_DATA(0x25);
	LCD_WR_REG(0xccb2);   LCD_WR_DATA(0x0C);
	LCD_WR_REG(0xccb3);   LCD_WR_DATA(0x0A);
	LCD_WR_REG(0xccb4);   LCD_WR_DATA(0x02);
	LCD_WR_REG(0xccb5);   LCD_WR_DATA(0x26);
	LCD_WR_REG(0xccb6);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccb7);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccb8);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccb9);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccc0);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccc1);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccc2);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccc3);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccc4);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccc5);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccc6);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccc7);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccc8);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccc9);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccca);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xcccb);   LCD_WR_DATA(0x25);
	LCD_WR_REG(0xcccc);   LCD_WR_DATA(0x0B);
	LCD_WR_REG(0xcccd);   LCD_WR_DATA(0x09);
	LCD_WR_REG(0xccce);   LCD_WR_DATA(0x01);
	LCD_WR_REG(0xccd0);   LCD_WR_DATA(0x26);
	LCD_WR_REG(0xccd1);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccd2);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccd3);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccd4);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccd5);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccd6);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccd7);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccd8);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccd9);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccda);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccdb);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccdc);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccdd);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0xccde);   LCD_WR_DATA(0x00);
	LCD_WR_REG(0x3A00);   LCD_WR_DATA(0x55);

	LCD_WR_REG(0x1100);
	   delay_ms(100);
	LCD_WR_REG(0x2900);
	   delay_ms(50);
	LCD_WR_REG(0x2C00);
	LCD_WR_REG(0x2A00);  LCD_WR_DATA(0x00);
	LCD_WR_REG(0x2A01);  LCD_WR_DATA(0x00);
	LCD_WR_REG(0x2A02);  LCD_WR_DATA(0x01);
	LCD_WR_REG(0x2A03);  LCD_WR_DATA(0xe0);
	LCD_WR_REG(0x2B00);  LCD_WR_DATA(0x00);
	LCD_WR_REG(0x2B01);  LCD_WR_DATA(0x00);
	LCD_WR_REG(0x2B02);  LCD_WR_DATA(0x03);
	LCD_WR_REG(0x2B03);  LCD_WR_DATA(0x20);

	LCD_SetParam();
	LCD_LED_OPEN;				//点亮背光
//	GPIO_SetBits(GPIOC, GPIO_Pin_2);
	LCD_Clear(WHITE);
	

}
