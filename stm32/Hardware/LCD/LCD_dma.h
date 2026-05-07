#ifndef __LCD_DMA_H
#define __LCD_DMA_H

#include "main.h"

extern uint8_t LCD_Show_Buffer[38400];				//38400		160*120*2
extern uint32_t DMA1_MEM_LEN;
extern uint16_t LCD_Tranform_Cnt;

void LCD_DMA_Start(void);
void LCD_DMA_Clear(uint16_t Color);
void LCD_Squar(uint16_t x,uint16_t y,uint16_t sizex,uint16_t sizey,uint16_t color);
void LCD_Squar_Net(uint16_t x,uint16_t y,uint16_t sizex,uint16_t sizey,uint16_t color,uint16_t color2);
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color);
void LCD_DrawBitmap(uint16_t x,uint16_t y,const unsigned char *bitmap,uint16_t w,uint16_t h,uint16_t BMP_color,uint16_t BACK_color);
void LCD_DrawBitmap_ToBuffer(uint16_t x,uint16_t y,uint16_t Offset,const unsigned char *bitmap,uint16_t w,uint16_t h,uint16_t BMP_color,uint16_t BACK_color);

#endif
