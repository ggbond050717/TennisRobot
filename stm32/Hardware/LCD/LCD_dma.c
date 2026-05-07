#include "LCD/LCD_dma.h"
#include "LCD/LCD_init.h"
#include "spi.h"
#include "stdio.h"

uint8_t LCD_Show_Buffer[38400];				//38400		160*120*2
uint32_t DMA1_MEM_LEN;
uint16_t LCD_Tranform_Cnt = 0;

//等待LCD刷新时空操作
void LCD_Nop(void)
{
	uint8_t i = 0;
	for(i = 0;i < 5; i++);
}

//开始刷新LCD的屏幕
void LCD_DMA_Start(void)
{
	
	HAL_SPI_Transmit_DMA(&hspi2,LCD_Show_Buffer,DMA1_MEM_LEN);
	while(1)
	{
		if(LCD_Tranform_Cnt == 0)
		{
			break;
		}
		LCD_Nop();
	}
}

//SPI全部传输完成的回调函数
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)//SPI完全传输成功回调
{
	static uint16_t cishu = 0;
	if(++cishu < LCD_Tranform_Cnt)
	{
		HAL_SPI_Transmit_DMA(&hspi2,LCD_Show_Buffer,DMA1_MEM_LEN);
	}
	else
	{
		LCD_Tranform_Cnt = 0;
		cishu = 0;
	}
	
}

//带颜色清屏
void LCD_DMA_Clear(uint16_t Color)
{
	DMA1_MEM_LEN = 2 * LCD_W;
	LCD_Address_Set(0, 0, LCD_W - 1, LCD_H - 1);
	for(uint16_t j = 0; j < 2 * LCD_W;)
	{
		LCD_Show_Buffer[j] = Color >> 8;
		LCD_Show_Buffer[j + 1] = Color;
		j += 2;
	}
	
	for(uint16_t i = 0; i < LCD_H; i++)
	{
		LCD_Tranform_Cnt++;
		LCD_DMA_Start();
	}
}

//带颜色画矩形
void LCD_Squar(uint16_t x,uint16_t y,uint16_t sizex,uint16_t sizey,uint16_t color)	
{
	DMA1_MEM_LEN=2*sizex;
	LCD_Address_Set(x,y,x+sizex-1,y+sizey-1);
	for(int j=0;j<sizey;j++)
	{
		for(int i=0;i<sizex;i++)
		{
			LCD_Show_Buffer[2*i] = color>>8;
			LCD_Show_Buffer[2*i+1] = color;				
		}
		LCD_Tranform_Cnt++;
		LCD_DMA_Start();	
	}
}

//带颜色的点
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
{
	LCD_Squar(x,y,1,1,color);
}

//带颜色画网格
void LCD_Squar_Net(uint16_t x,uint16_t y,uint16_t sizex,uint16_t sizey,uint16_t color,uint16_t color2)	
{
	DMA1_MEM_LEN=2*sizex;
	
	LCD_Address_Set(x,y,x+sizex-1,y+sizey-1);
	for(int j=0;j<sizey;j++)
	{
		for(int i=0;i<sizex;i++)
		{
			if(j%8)
			{
				if(i%5==0)
				{
					LCD_Show_Buffer[2*i] = color>>8;
					LCD_Show_Buffer[2*i+1] = color;	
				}
				else		
				{
					LCD_Show_Buffer[2*i] = color2>>8;
					LCD_Show_Buffer[2*i+1] = color2;
				}
			}
			else
			{
				if(i%5==0)
				{
				LCD_Show_Buffer[2*i] = color2>>8;
				LCD_Show_Buffer[2*i+1] = color2;	
				}
				else		
				{
					LCD_Show_Buffer[2*i] = color>>8;
					LCD_Show_Buffer[2*i+1] = color;
				}
			}
		}
		LCD_Tranform_Cnt++;
		LCD_DMA_Start();
	}
}

void LCD_ShowPicture(uint16_t x,uint16_t y,uint16_t length,uint16_t width,const uint8_t pic[])
{
	uint16_t i,j;
	uint32_t k=0;
	DMA1_MEM_LEN=2*width;
	LCD_Address_Set(x,y,x+length-1,y+width-1);
		
	for(i=0;i<length;i++)
	{
		
		for(j=0;j<width;j++)
		{
			LCD_Show_Buffer[2*j+1]=pic[k];
			LCD_Show_Buffer[2*j]=pic[k+1];
			k+=2;	
		}
		LCD_Tranform_Cnt++;
		LCD_DMA_Start();		
	}
}

//显示图片 
//X坐标，Y坐标，图片名称，宽，高，图片颜色，字体颜色
void LCD_DrawBitmap(uint16_t x,uint16_t y,const unsigned char *bitmap,uint16_t w,uint16_t h,uint16_t BMP_color,uint16_t BACK_color)
{
	DMA1_MEM_LEN=w*h*2;
	for(uint8_t dh = 0;dh < h/8;dh ++)
	{
		for(uint16_t dw = 0;dw < w;dw++,bitmap++)
		{
			for(uint8_t t=0;t<8;t++)
			{
				if(((*bitmap)>>t) & 0x01 == 0X01)
				{
					LCD_Show_Buffer[2*(dw+(dh*8+t)*w)]=BMP_color>>8;
					LCD_Show_Buffer[2*(dw+(dh*8+t)*w)+1]=BMP_color;
				}
				else 
				{
					LCD_Show_Buffer[2*(dw+(dh*8+t)*w)]=BACK_color>>8;
					LCD_Show_Buffer[2*(dw+(dh*8+t)*w)+1]=BACK_color;
				}
			}
		}
  }
	LCD_Address_Set(x,y,x+w-1,y+h-1);
	
	LCD_Tranform_Cnt++;
	LCD_DMA_Start();
}

//显示图片 
//X坐标，Y坐标，图片名称，宽，高，图片颜色，字体颜色
void LCD_DrawBitmap_ToBuffer(uint16_t x,uint16_t y,uint16_t Offset,const unsigned char *bitmap,uint16_t w,uint16_t h,uint16_t BMP_color,uint16_t BACK_color)
{
	for(uint8_t dh = 0;dh < h/8;dh ++)
	{
		for(uint16_t dw = 0;dw < w;dw++,bitmap++)
		{
			for(uint8_t t=0;t<8;t++)
			{
				if(((*bitmap)>>t) & 0x01 == 0X01)
				{
					LCD_Show_Buffer[(Offset*y+x)*2+2*(dw+(dh*8+t)*Offset)]=BMP_color>>8;
					LCD_Show_Buffer[(Offset*y+x)*2+2*(dw+(dh*8+t)*Offset)+1]=BMP_color;
				}
				else 
				{
					LCD_Show_Buffer[(Offset*y+x)*2+2*(dw+(dh*8+t)*Offset)]=BACK_color>>8;
					LCD_Show_Buffer[(Offset*y+x)*2+2*(dw+(dh*8+t)*Offset)+1]=BACK_color;
				}
			}
		}
  }
}

