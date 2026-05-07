#include "GUI/GUI.h"
#include "OS/OS.h"
#include "LCD/LCD_init.h"
#include "LCD/LCD_dma.h"
#include "LCD/BMP.h"

const unsigned char *Char_buffer;

uint32_t mypow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;
	return result;
}

void Char_XX_24(uint8_t num)
{
	switch(num)
	{
		case 0:Char_buffer = NUM_15_24_0;break;
		case 1:Char_buffer = NUM_15_24_1;break;
		case 2:Char_buffer = NUM_15_24_2;break;
		case 3:Char_buffer = NUM_15_24_3;break;
		case 4:Char_buffer = NUM_15_24_4;break;
		case 5:Char_buffer = NUM_15_24_5;break;
		case 6:Char_buffer = NUM_15_24_6;break;
		case 7:Char_buffer = NUM_15_24_7;break;
		case 8:Char_buffer = NUM_15_24_8;break;
		case 9:Char_buffer = NUM_15_24_9;break;
		default: break;
	}
}

void LCD_Show_Interface(void)
{
	for(uint8_t i = 0; i < 10;i++)
	{
		switch(i)
		{
			case 0:
				LCD_DMA_Clear(Black);
			break;
			case 1:
				LCD_DrawBitmap(77,2,CHAR_15_24_V,15,24,Yellow,Black);
				LCD_DrawBitmap(187,2,CHAR_15_24_A,15,24,TurquoiseGreen,Black);
				LCD_DrawBitmap(302,2,CHAR_15_24_V,15,24,Red,Black);
			break;
			default:
			break;
		}
	}
}

void LCD_Vol_Data(uint16_t V_num)
{
	uint8_t temp = 0, i = 0;
	
	DMA1_MEM_LEN = 1800*2;//75*24
	LCD_Address_Set(3,2,77,26);
	
	for(i = 0; i < 4; i++)
	{
		temp = (V_num / mypow(10, 4 - i - 1)) % 10;
		Char_XX_24(temp);
		switch(i)
		{
			case 0:LCD_DrawBitmap_ToBuffer(0,0,75,Char_buffer,15,24,Yellow,Black);break;
			case 1:LCD_DrawBitmap_ToBuffer(15,0,75,Char_buffer,15,24,Yellow,Black);break;
			case 2:LCD_DrawBitmap_ToBuffer(45,0,75,Char_buffer,15,24,Yellow,Black);break;
			case 3:LCD_DrawBitmap_ToBuffer(60,0,75,Char_buffer,15,24,Yellow,Black);break;
			default: break;
		}
	}
	LCD_DrawBitmap_ToBuffer(30,0,75,CHAR_15_24_Drop,15,24,Yellow,Black);

	LCD_Tranform_Cnt++;
	LCD_DMA_Start();
}

void LCD_Cur_Data(uint16_t I_num)
{
	uint8_t temp = 0, i = 0;
	
	DMA1_MEM_LEN = 1800*2;//75*24
	LCD_Address_Set(113,2,187,26);
	
	for(i = 0; i < 4; i++)
	{
		temp = (I_num / mypow(10, 4 - i - 1)) % 10;
		Char_XX_24(temp);
		switch(i)
		{
			case 0:LCD_DrawBitmap_ToBuffer(0,0,75,Char_buffer,15,24,TurquoiseGreen,Black);break;
			case 1:LCD_DrawBitmap_ToBuffer(30,0,75,Char_buffer,15,24,TurquoiseGreen,Black);break;
			case 2:LCD_DrawBitmap_ToBuffer(45,0,75,Char_buffer,15,24,TurquoiseGreen,Black);break;
			case 3:LCD_DrawBitmap_ToBuffer(60,0,75,Char_buffer,15,24,TurquoiseGreen,Black);break;
			default: break;
		}
	}
	LCD_DrawBitmap_ToBuffer(15,0,75,CHAR_15_24_Drop,15,24,TurquoiseGreen,Black);

	LCD_Tranform_Cnt++;
	LCD_DMA_Start();
}

void LCD_Wat_Data(uint16_t P_num)
{
	uint8_t temp = 0, i = 0;
	
	DMA1_MEM_LEN = 1800*2;//75*24
	LCD_Address_Set(228,2,302,26);
	
	for(i = 0; i < 4; i++)
	{
		temp = (P_num / mypow(10, 4 - i - 1)) % 10;
		Char_XX_24(temp);
		switch(i)
		{
			case 0:LCD_DrawBitmap_ToBuffer(0,0,75,Char_buffer,15,24,Red,Black);break;
			case 1:LCD_DrawBitmap_ToBuffer(15,0,75,Char_buffer,15,24,Red,Black);break;
			case 2:LCD_DrawBitmap_ToBuffer(45,0,75,Char_buffer,15,24,Red,Black);break;
			case 3:LCD_DrawBitmap_ToBuffer(60,0,75,Char_buffer,15,24,Red,Black);break;
			default: break;
		}
	}
	LCD_DrawBitmap_ToBuffer(30,0,75,CHAR_15_24_Drop,15,24,Red,Black);

	LCD_Tranform_Cnt++;
	LCD_DMA_Start();
}

void LCD_Refresh(void)
{
//	static uint8_t once = 0;
	for(uint8_t i = 0; i < 10; i++)
	{
		switch(i)
		{
			case 0:
				LCD_Vol_Data(os.V);
			break;
			case 1:
				LCD_Cur_Data(os.I);
			break;
			case 2:
				LCD_Wat_Data(os.P);
			break;
			default:
			break;
		}
	}
}
