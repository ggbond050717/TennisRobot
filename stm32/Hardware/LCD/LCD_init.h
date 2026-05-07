#ifndef __LCD_INIT_H
#define __LCD_INIT_H

#include "main.h"

#define USE_HORIZONTAL 2  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏
//Alter
#if USE_HORIZONTAL==0||USE_HORIZONTAL==1  //横竖屏时的长高不唯一
#define LCD_W 240
#define LCD_H 320

#else
#define LCD_W 320
#define LCD_H 240
#endif

/* -----------------LCD端口定义---------------- */

#define LCD_RES_Clr()  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET)//RES
#define LCD_RES_Set()  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET)

#define LCD_DC_Clr()   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET)//DC
#define LCD_DC_Set()   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET)

/* ------------------颜色定义------------------ */

#define While         	 	0xFFFF	//白
#define Black         	 	0x0000	//黑 
#define Blue          	 	0x001F	//蓝
#define Red           	 	0xF800	//红
#define Green         	 	0x07E0	//绿
#define Yellow        	 	0xFFE0	//黄

#define Mint							0x14C5	//薄荷绿
#define AquaBlue					0x67FC	//水蓝
#define TurquoiseGreen		0x4F30	//绿松石绿
#define ForestGreen				0x2444	//森林绿（第二栏通道）
#define Veridian					0x13A6	//铬绿（第二栏按钮）
#define PaleGreen					0x9FD3	//灰绿（代替水蓝）
#define CobaltGreen				0x67EB	//钴绿（代替黄色）
#define LawnGreen					0x7FE0	//草坪绿（输入电源）
#define Malachite					0x2605	//孔雀石绿
#define FreshGreen				0x9FE9	//嫩绿


/* -----------------函数定义---------------- */

void LCD_Writ_Bus(uint8_t dat);//模拟SPI时序
void LCD_WR_DATA8(uint8_t dat);//写入一个字节
void LCD_WR_DATA(uint16_t dat);//写入两个字节
void LCD_WR_REG(uint8_t dat);//写入一个指令
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);//设置坐标函数
void LCD_Init(void);//LCD初始化
void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color);//LCD清屏




#endif
