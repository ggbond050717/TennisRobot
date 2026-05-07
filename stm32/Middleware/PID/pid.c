#include "PID/pid.h"
#include "Motor/Motor.h"
#include "stdio.h"
PID_strust Left_PID;
PID_strust Right_PID;
PID_strust Roller_PID;

void PID_Init(PID_strust *pid)		
{
	pid->target_val = 0.0;
	pid->actual_val = 0.0;
	//误差
	pid->err = 0.0;
	pid->err_last = 0.0;
	pid->err_sum = 0.0;
	//参数
	pid->Kd = 300;	//快
	pid->Ki = 0.1;	//准
	pid->Kp = 30;		//稳
}

float P_realize(PID_strust *pid, float target_val)
{
	pid->target_val = target_val;
	pid->err = pid->target_val - pid->actual_val;
	pid->PWM += pid->Kp * pid->err;
	return pid->PWM;
}

float PI_realize(PID_strust *pid, float target_val)
{
	pid->target_val = target_val;
	pid->err = pid->target_val - pid->actual_val;
	pid->err_sum += pid->err;
	pid->PWM += pid->Kp * pid->err + pid->Ki * pid->err_sum;
	return pid->PWM;
}

float PID_realize(PID_strust *pid, float target_val)
{
	pid->target_val = target_val;
	pid->err = pid->target_val - pid->actual_val;
	pid->err_sum += pid->err;
	pid->PWM += pid->Kp * pid->err + pid->Ki * pid->err_sum + pid->Kd * (pid->err - pid->err_last);
	pid->err_last = pid->err;
	return pid->PWM;
}

uint8_t Motor_PWM_PID(float target_LeftSpeed, float target_RightSpeed, float target_RollerSpeed)
{
	float LeftPWM = 0, RightPWM = 0, RollerPWM = 0;
	uint8_t err = 0;
	if(target_LeftSpeed != 0)
		LeftPWM = PID_realize(&Left_PID, target_LeftSpeed);
	else
		PID_Init(&Left_PID);
	if(target_RightSpeed != 0)
		RightPWM = PID_realize(&Right_PID, target_RightSpeed);
	else
		PID_Init(&Right_PID);
	if(target_RollerSpeed != 0)
		RollerPWM = PID_realize(&Roller_PID, target_RollerSpeed);
	else
		PID_Init(&Roller_PID);
	
//	printf("LeftPWM = %.2f, RightPWM = %.2f, RollerPWM = %.2f", LeftPWM, RightPWM, RollerPWM);
	
	Motor_PWM((int)LeftPWM,(int)RightPWM,(int)RollerPWM);
	
	return err;
}

