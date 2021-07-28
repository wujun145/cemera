#include "touch.h"
//#include "my_lcd.h"
#include "delay.h"
#include "bsp_flash.h"

#include <math.h>

#define TDIN(a)		{(a>0)?(TOUCH_MOSI_Port->BSRR = TOUCH_MOSI_Pin):(TOUCH_MOSI_Port->BSRR = (uint32_t)TOUCH_MOSI_Pin << 16U);}		//HAL_GPIO_WritePin(TOUCH_MOSI_Port, TOUCH_MOSI_Pin, a)
#define TCLK(a)		{(a>0)?(TOUCH_SCK_Port->BSRR = TOUCH_SCK_Pin):(TOUCH_SCK_Port->BSRR = (uint32_t)TOUCH_SCK_Pin << 16U);}			//HAL_GPIO_WritePin(TOUCH_SCK_Port, TOUCH_SCK_Pin, a)
#define TCS(a)		{(a>0)?(TOUCH_CS_Port->BSRR = TOUCH_CS_Pin):(TOUCH_CS_Port->BSRR = (uint32_t)TOUCH_CS_Pin << 16U);}				//HAL_GPIO_WritePin(TOUCH_CS_Port, TOUCH_CS_Pin, a)
#define DOUT		(TOUCH_MISO_Port->IDR & TOUCH_MISO_Pin)		//HAL_GPIO_ReadPin(TOUCH_MISO_Port, TOUCH_MISO_Pin)
#define PEN			(TOUCH_PEN_Port->IDR & TOUCH_PEN_Pin)		//HAL_GPIO_ReadPin(TOUCH_PEN_Port, TOUCH_PEN_Pin)

TouchSet touchsetnum = { 0 };

uint8_t CMD_RDX = 0XD0;
uint8_t CMD_RDY = 0X90;

_m_tp_dev tp_dev = {
	TP_Init,
	TP_Scan,
	TP_Adjust,
	0,
	0, 
	0,
	0,
	0,
	0,	  	 		
	0,
	0,	  	 	
};
	
void TP_Write_Byte(uint8_t num)
{
	uint8_t count = 0;
	for(count=0; count<8; count++)
	{
		if(num&0x80)
		{
			TDIN(1);
		}
		else
		{
			TDIN(0);
		}
		num <<= 1;
		TCLK(0);
		delay_us(1);
		TCLK(1);
	}
}

uint16_t TP_Read_AD(uint8_t CMD)
{
	uint8_t count = 0;
	uint16_t Num=0;
	TCLK(0);
	TDIN(0);
	TCS(0);
	TP_Write_Byte(CMD);
	delay_us(6);
	TCLK(0);
	delay_us(1);
	TCLK(1);
	delay_us(1);
	TCLK(0);
	for(count=0; count<16; count++)
	{
		Num <<= 1;
		TCLK(0);
		delay_us(2);	//delay
		TCLK(1);
		if(DOUT)
		{
			Num++;
		}
	}
	Num>>=4;
	TCS(1);
	return Num;
}

#define READ_TIMES 5 	//��ȡ����
#define LOST_VAL 1	  	//����ֵ

uint16_t TP_Read_XOY(uint8_t xy)
{
	uint16_t i,j;
	uint16_t buf[READ_TIMES];
	uint16_t sum=0;
	uint16_t temp;
	for(i=0; i< READ_TIMES; i++)
		buf[i] = TP_Read_AD(xy);
	for(i=0; i<READ_TIMES-1; i++)
	{
		for(j=i+1; j<READ_TIMES; j++)
		{
			if(buf[i] > buf[j])
			{
				temp = buf[i];
				buf[i] = buf[j];
				buf[j] = temp;
			}
		}
	}
	sum = 0;
	for(i=LOST_VAL; i<READ_TIMES-LOST_VAL; i++)sum+=buf[i];
	temp=sum/(READ_TIMES-2*LOST_VAL);
	return temp;   
}

uint8_t TP_Read_XY(uint16_t *x, uint16_t *y)
{
		uint16_t xtemp,ytemp;			 	 		  
		xtemp=TP_Read_XOY(CMD_RDX);
		ytemp=TP_Read_XOY(CMD_RDY);	  												   
		//if(xtemp<100||ytemp<100)return 0;//����ʧ��
		*x=xtemp;
		*y=ytemp;
		return 1;//�����ɹ�
}

#define ERR_RANGE 50 //��Χ 
uint8_t TP_Read_XY2(uint16_t *x,uint16_t *y) 
{
		uint16_t x1,y1;
		uint16_t x2,y2;
		uint8_t flag;    
    flag=TP_Read_XY(&x1,&y1);   
    if(flag==0)
		{
		    return(0);
		}
    flag=TP_Read_XY(&x2,&y2);	   
    if(flag==0)
		{
		    return(0); 
		}  
    if(((x2<=x1&&x1<x2+ERR_RANGE)||(x1<=x2&&x2<x1+ERR_RANGE))//ǰ�����β�����+-50��
    &&((y2<=y1&&y1<y2+ERR_RANGE)||(y1<=y2&&y2<y1+ERR_RANGE)))
    {
        *x=(x1+x2)/2;
        *y=(y1+y2)/2;
        return 1;
    }
	else
	{
		return 0;	  
	}		
}  

void TP_Drow_Touch_Point(uint16_t x,uint16_t y,uint16_t color)
{
	POINT_COLOR=color;
	LCD_DrawLine(x-12,y,x+13,y);//����
	LCD_DrawLine(x,y-12,x,y+13);//����
	LCD_DrawPoint(x+1,y+1);
	LCD_DrawPoint(x-1,y+1);
	LCD_DrawPoint(x+1,y-1);
	LCD_DrawPoint(x-1,y-1);
	Draw_Circle(x, y, 6);
}	  

void TP_Draw_Big_Point(uint16_t x,uint16_t y,uint16_t color)
{	    
	POINT_COLOR=color;
	LCD_DrawPoint(x,y);//���ĵ� 
	LCD_DrawPoint(x+1,y);
	LCD_DrawPoint(x,y+1);
	LCD_DrawPoint(x+1,y+1);	 	  	
}	
//////////////////////////////////////////////////////////////////////////////////		  
//��������ɨ��
//tp:0,��Ļ����;1,��������(У׼�����ⳡ����)
//����ֵ:��ǰ����״̬.
//0,�����޴���;1,�����д���
uint8_t TP_Scan(uint8_t tp)
{			   
		if(PEN==0)//�а�������
		{
				if(tp)
				{
						TP_Read_XY2(&tp_dev.x[0],&tp_dev.y[0]);//��ȡ��������	
				}
				else if(TP_Read_XY2(&tp_dev.x[0],&tp_dev.y[0]))//��ȡ��Ļ����
				{
						tp_dev.x[0]=tp_dev.xfac*tp_dev.x[0]+tp_dev.xoff;//�����ת��Ϊ��Ļ����
						tp_dev.y[0]=tp_dev.yfac*tp_dev.y[0]+tp_dev.yoff;  
				} 
				if((tp_dev.sta&TP_PRES_DOWN)==0)//֮ǰû�б�����
				{		 
						tp_dev.sta=TP_PRES_DOWN|TP_CATH_PRES;//��������  
						tp_dev.x[4]=tp_dev.x[0];//��¼��һ�ΰ���ʱ������
						tp_dev.y[4]=tp_dev.y[0];  	   		
				}			   
		}
		else
		{
				if(tp_dev.sta&TP_PRES_DOWN)//֮ǰ�Ǳ����µ�
				{
						tp_dev.sta&=~(1<<7);//��ǰ����ɿ�	
				}
				else//֮ǰ��û�б�����
				{
						tp_dev.x[4]=0;
						tp_dev.y[4]=0;
						tp_dev.x[0]=0xffff;
						tp_dev.y[0]=0xffff;
				}	    
		}
		return tp_dev.sta&TP_PRES_DOWN;//���ص�ǰ�Ĵ���״̬
}	  

void TP_Save_Adjdata(void)
{		 

	//����У�����!		   							  
		touchsetnum.xfac=tp_dev.xfac*100000000;//����xУ������      
  
		touchsetnum.yfac=tp_dev.yfac*100000000;//����yУ������    

		touchsetnum.xoff = tp_dev.xoff;//����xƫ����
	    
		touchsetnum.yoff = tp_dev.yoff;//����yƫ����

		touchsetnum.touchtype = tp_dev.touchtype;			//���津������
		
		touchsetnum.touchflag =0X0A;//���У׼����
		
  	TouchWriteFlash(&touchsetnum);

}

//�õ�������EEPROM�����У׼ֵ
//����ֵ��1���ɹ���ȡ����
//        0����ȡʧ�ܣ�Ҫ����У׼
uint8_t TP_Get_Adjdata(void)
{	
  	int32_t tempfac;	
    TouchReadFlash(&touchsetnum);//��ȡ����������	

		tempfac=touchsetnum.touchflag;//��ȡ�����,���Ƿ�У׼���� 
		if(tempfac==0X0A)//�������Ѿ�У׼����			   
		{    												 
				tempfac=touchsetnum.xfac;		   
				tp_dev.xfac=(float)tempfac/100000000;//�õ�xУ׼����
				tempfac=touchsetnum.yfac;			          
				tp_dev.yfac=(float)tempfac/100000000;//�õ�yУ׼����
					//�õ�xƫ����
				tp_dev.xoff=touchsetnum.xoff;			   	  
					//�õ�yƫ����
				tp_dev.yoff=touchsetnum.yoff;				 	  
				tp_dev.touchtype=touchsetnum.touchtype;//��ȡ�������ͱ��
				if(tp_dev.touchtype)//X,Y��������Ļ�෴
				{
						CMD_RDX=0X90;
						CMD_RDY=0XD0;	 
				}
				else				   //X,Y��������Ļ��ͬ
				{
						CMD_RDX=0XD0;
						CMD_RDY=0X90;	 
				}		 
				return 1;	 
		}
		return 0;
}	 

		 
//������У׼����
//�õ��ĸ�У׼����
void TP_Adjust(void)
{								 
		uint16_t pos_temp[4][2];//���껺��ֵ
		uint8_t  cnt=0;	
		uint16_t d1,d2;
		uint32_t tem1,tem2;
		double fac; 	
		uint16_t outtime=0;
		cnt=0;
		LCD_Clear(WHITE);//����	
		LCD_ShowString(40, 20, 12,  "point 1", 1);
		printf("touch start\r\n");	
		printf("point1 x 20, y 20\r\n");
		TP_Drow_Touch_Point(20,20, RED);	
		tp_dev.sta=0;//���������ź� 
		tp_dev.xfac=0;//xfac��������Ƿ�У׼��,����У׼֮ǰ�������!�������	
		
		while(1)//�������10����û�а���,���Զ��˳�
		{
				tp_dev.scan(1);//ɨ����������
				if((tp_dev.sta&0xc0)==TP_CATH_PRES)//����������һ��(��ʱ�����ɿ���.)
				{	
					outtime=0;		
					tp_dev.sta&=~(1<<6);//��ǰ����Ѿ����������.
													 
					pos_temp[cnt][0]=tp_dev.x[0];
					pos_temp[cnt][1]=tp_dev.y[0];
					
					printf("tp_dev.x[%d] %d, tp_dev.y[%d] %d\r\n", cnt+1, tp_dev.x[0], cnt+1, tp_dev.y[0]);
					cnt++;	  
					switch(cnt)
					{			   
							case 1:		
								printf("draw point 2\r\n");
								printf("point2 x %d, y 20", lcddev.width-20);
								  LCD_Clear(WHITE);//����		
								LCD_ShowString(lcddev.width-80, 20, 12,  "point 2", 1);
									TP_Drow_Touch_Point(lcddev.width-20,20, RED);		//����2
							break;
							case 2:
								printf("draw point 3\r\n");
								printf("point3 x 20, y %d", lcddev.height-20);
								  LCD_Clear(WHITE);//����				
								LCD_ShowString(40, lcddev.height-20, 12,  "point 3", 1);							
									TP_Drow_Touch_Point(20,lcddev.height-20, RED);			//����3
							break;
							case 3:
								printf("draw point 4\r\n");
								printf("point4 x %d, y %d", lcddev.width-20, lcddev.height-20);
								  LCD_Clear(WHITE);//����				
								LCD_ShowString(lcddev.width-80, lcddev.height-20, 12,  "point 4", 1);	
									TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20, RED);			//����4
							break;

#if 1
							{
							case 4:	 //ȫ���ĸ����Ѿ��õ�
									//�Ա����
							tem1=abs(pos_temp[0][0]-pos_temp[1][0]);//x1-x2
							tem2=abs(pos_temp[0][1]-pos_temp[1][1]);//y1-y2
							printf("tem1 %d, tem2 %d\r\n", tem1, tem2);
							tem1*=tem1;
							tem2*=tem2;
							
							d1=sqrt(tem1+tem2);//�õ�1,2�ľ���
							
							printf("get point 1,2 limit\r\n");
							printf("tem1 %d, tem2 %d, d1 %d\r\n", tem1, tem2, d1);
							
							tem1=abs(pos_temp[2][0]-pos_temp[3][0]);//x3-x4
							tem2=abs(pos_temp[2][1]-pos_temp[3][1]);//y3-y4
							printf("tem1 %d, tem2 %d\r\n", tem1, tem2);
							tem1*=tem1;
							tem2*=tem2;
							d2=sqrt(tem1+tem2);//�õ�3,4�ľ���
							fac=(float)d1/d2;
							printf("tem1 %d, tem2 %d, d2 %d, fac %f\r\n", tem1, tem2, d2, fac);
							if(fac<0.95||fac>1.05||d1==0||d2==0)//���ϸ�
							{
								printf("1part not good value, but continue\r\n");
								printf("point1 x 20, y 20\r\n");
									cnt=0;
								  LCD_Clear(WHITE);//����		
								LCD_ShowString(40, 20, 12,  "point 1", 1);
									TP_Drow_Touch_Point(20,20, RED);			//����1  
									continue;
							}
							tem1=abs(pos_temp[0][0]-pos_temp[2][0]);//x1-x3
							tem2=abs(pos_temp[0][1]-pos_temp[2][1]);//y1-y3
							printf("tem1 %d, tem2 %d\r\n", tem1, tem2);
							tem1*=tem1;
							tem2*=tem2;
							d1=sqrt(tem1+tem2);//�õ�1,3�ľ���
							
							printf("get point 1,3 limit\r\n");
							printf("tem1 %d, tem2 %d, d1 %d\r\n", tem1, tem2, d1);
							
							tem1=abs(pos_temp[1][0]-pos_temp[3][0]);//x2-x4
							tem2=abs(pos_temp[1][1]-pos_temp[3][1]);//y2-y4
							printf("tem1 %d, tem2 %d\r\n", tem1, tem2);
							tem1*=tem1;
							tem2*=tem2;
							d2=sqrt(tem1+tem2);//�õ�2,4�ľ���
							fac=(float)d1/d2;
							printf("tem1 %d, tem2 %d, d2 %d, fac %f\r\n", tem1, tem2, d2, fac);
							if(fac<0.95||fac>1.05)//���ϸ�
							{
									printf("2 part not good value, but continue\r\n");
								printf("point1 x 20, y 20\r\n");
									cnt=0;
								  LCD_Clear(WHITE);//����		
								LCD_ShowString(40, 20, 12,  "point 1", 1);
									TP_Drow_Touch_Point(20,20, RED);			//����1
									continue;
							}//��ȷ��
											 
							//�Խ������
							tem1=abs(pos_temp[1][0]-pos_temp[2][0]);//x1-x3
							tem2=abs(pos_temp[1][1]-pos_temp[2][1]);//y1-y3
							tem1*=tem1;
							tem2*=tem2;
							d1=sqrt(tem1+tem2);//�õ�1,4�ľ���
							
							printf("get point 1,4 limit\r\n");
							printf("tem1 %d, tem2 %d, d1 %d\r\n", tem1, tem2, d1);
							
							tem1=abs(pos_temp[0][0]-pos_temp[3][0]);//x2-x4
							tem2=abs(pos_temp[0][1]-pos_temp[3][1]);//y2-y4
							tem1*=tem1;
							tem2*=tem2;
							d2=sqrt(tem1+tem2);//�õ�2,3�ľ���
							fac=(float)d1/d2;
							printf("tem1 %d, tem2 %d, d2 %d, fac %f\r\n", tem1, tem2, d2, fac);
							if(fac<0.95||fac>1.05)//���ϸ�
							{
								printf("3 part not good value, but continue\r\n");
								printf("point1 x 20, y 20\r\n");
									cnt=0;
								  LCD_Clear(WHITE);//����		
								LCD_ShowString(40, 20, 12,  "point 1", 1);
									TP_Drow_Touch_Point(20,20, RED);			//����1 
									continue;
							}
							//��ȷ��
							//������
							tp_dev.xfac=(float)(lcddev.height-40)/(pos_temp[1][0]-pos_temp[0][0]);//�õ�xfac		 
							tp_dev.xoff=(lcddev.height-tp_dev.xfac*(pos_temp[1][0]+pos_temp[0][0]))/2;//�õ�xoff
									
							printf("tp_dev.xfac %f, tp_dev.xoff %f\r\n", tp_dev.xfac, tp_dev.xoff);
							
							tp_dev.yfac=(float)(lcddev.width-40)/(pos_temp[2][1]-pos_temp[0][1]);//�õ�yfac
							tp_dev.yoff=(lcddev.width-tp_dev.yfac*(pos_temp[2][1]+pos_temp[0][1]))/2;//�õ�yoff  
							
							printf("tp_dev.yfac %f, tp_dev.yoff %f\r\n", tp_dev.yfac, tp_dev.yoff);
							if(abs(tp_dev.xfac)>2||abs(tp_dev.yfac)>2)//������Ԥ����෴��.
							{
								printf("tpt lcd and touch not sure\r\n");
								printf("point1 x 20, y 20\r\n");
									cnt=0;
								  LCD_Clear(WHITE);//����		
								LCD_ShowString(40, 20, 12,  "point 1", 1);
									TP_Drow_Touch_Point(20,20, RED);		//����1
									tp_dev.touchtype=!tp_dev.touchtype;//�޸Ĵ�������.
									if(tp_dev.touchtype)//X,Y��������Ļ�෴
									{
										CMD_RDX=0X90;
										CMD_RDY=0XD0;	 
									}
									else				   //X,Y��������Ļ��ͬ
									{
										CMD_RDX=0XD0;
										CMD_RDY=0X90;	 
									}			    
									continue;
							}		
							LCD_Clear(WHITE);//����
							
							LCD_ShowString(100, 300, 12, "touch lcd Inited compelete", 1);
							delay_ms(1000);	
							TP_Save_Adjdata();  
							LCD_Clear(WHITE);//����
							return;//У�����				 
					}
#endif
					}
					
				}
				delay_ms(20);
				outtime++;
				if(outtime>500)
				{	
				  LCD_Clear(WHITE);//����
					break;
				}
							
		}
}	

uint8_t TP_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	
	/*Configure GPIO pin : TOUCH */
	GPIO_InitStruct.Pin = TOUCH_SCK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(TOUCH_SCK_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = TOUCH_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(TOUCH_CS_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = TOUCH_MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(TOUCH_MOSI_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = TOUCH_MISO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(TOUCH_MISO_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = TOUCH_PEN_Pin;		//�����ж�����
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(TOUCH_PEN_Port, &GPIO_InitStruct);

	
	TP_Read_XY(&tp_dev.x[0],&tp_dev.y[0]);//��һ�ζ�ȡ��ʼ��	
	
	if(TP_Get_Adjdata())
	{
		return 0;//�Ѿ�У׼
	}
	else			   //δУ׼?
	{ 										    

			TP_Adjust();  	//��ĻУ׼ 
			TP_Save_Adjdata();				
	}			
	TP_Get_Adjdata();	
	
	return 1; 		
}

uint8_t strBuf[50] = {0};
void rtp_test(void)
{
	uint8_t key; 
	uint8_t i = 0;
//	while(1)
	{
		tp_dev.scan(0);
		
		if(tp_dev.sta & TP_PRES_DOWN)
		{
			if(tp_dev.x[0] < lcddev.width && tp_dev.y[0] < lcddev.height)
			{
				if(tp_dev.x[0] > (lcddev.width - 24) && tp_dev.y[0] < 16)
				{
					LCD_Clear(WHITE);
				}
				else
				{
//					TP_Draw_Big_Point(tp_dev.x[0], tp_dev.y[0], RED);
					LCD_Clear(WHITE);
					sprintf(strBuf, "touch point is x = %d, y = %d", tp_dev.x[0], tp_dev.y[0]);
					LCD_ShowString(200, 100, 16,  (uint8_t*)strBuf, 1);
				}
					
			}
		}
		else
			delay_ms(10);
	}
}

uint8_t PEN_FLAG = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == TOUCH_PEN_Pin)
	{
		PEN_FLAG = 0;
	}
}

void EXTI1_IRQHandler(void)
{
	HAL_GPIO_EXTI_Callback(TOUCH_PEN_Pin);
}