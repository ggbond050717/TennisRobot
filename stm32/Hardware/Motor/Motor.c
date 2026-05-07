#include "Motor/Motor.h"
#include "tim.h"


void Motor_PWM_Init(void)
{
	HAL_TIM_PWM_Init(&htim1);
	HAL_TIM_PWM_Init(&htim4);
	//滚轮
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	//左电机
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	//右电机
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
}

void Enconder_Init(void)
{
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_2);
	__HAL_TIM_SET_COUNTER(&htim2, 0);//左电机
	__HAL_TIM_SET_COUNTER(&htim3, 0);//旋钮
	__HAL_TIM_SET_COUNTER(&htim5, 0);//右电机
	__HAL_TIM_SET_COUNTER(&htim8, 0);//滚轮电机
}

void Motor_PWM(int left, int right, int roller)
{
	if(left > 0)
	{
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,7199);
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,7199-left);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,7199+left);
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,7199);
	}
	
	if(right > 0)
	{
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,7199);
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,7199-right);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,7199+right);
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,7199);
	}
	
	if(roller > 0)
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,7199);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,7199-roller);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,7199+roller);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,7199);
	}
}







