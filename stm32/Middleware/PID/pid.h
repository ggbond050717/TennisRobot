#ifndef __PID_H
#define __PID_H
#include "main.h"

//PID
typedef struct 
{
	float target_val;
	float actual_val;
	float err;
	float err_last;
	float err_sum;
	float Kp,Ki,Kd;
	
	float PWM;
}PID_strust;

extern PID_strust Left_PID;
extern PID_strust Right_PID;
extern PID_strust Roller_PID;

void PID_Init(PID_strust *pid);
float P_realize(PID_strust *pid, float actual_val);
float PI_realize(PID_strust *pid, float actual_val);
float PID_realize(PID_strust *pid, float actual_val);
uint8_t Motor_PWM_PID(float target_LeftSpeed, float target_RightSpeed, float target_RollerSpeed);


#endif
