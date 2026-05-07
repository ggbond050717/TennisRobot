#ifndef __OS_H
#define __OS_H

#include "main.h"

typedef struct{
	/* 系统时间 */
	uint16_t cnt;
	uint16_t cnt_10;
	
	/* 编码器*/
	int32_t LeftSpeed;
	int32_t RightSpeed;
	int32_t LeftSpeed_sum;
	int32_t RightSpeed_sum;
	int32_t target_LeftSpeed;
	int32_t target_RightSpeed;
	int32_t RollerSpeed;
	
	/* 电源参数 */
	uint16_t V;
	uint16_t I;
	float P;
	uint16_t Vref2V5;
	float Error_Ratio;
	
}os_struct;

extern os_struct os;

void _os(void);

#endif
