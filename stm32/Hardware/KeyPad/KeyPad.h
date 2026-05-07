#ifndef __KEYPAD_H
#define __KEYPAD_H

#include "main.h"

enum //±àÂëÆ÷µ±Ç°×´Ì¬
{
	Nop,
	CH1_Vset,
	CH1_Iset,
	CH2_Vset,
	CH2_Iset,
	Fan,
};

extern uint8_t Key_Val[3];

uint8_t Bsp_BTNScan(void);
void KeyPad_Scan(void);

#endif
