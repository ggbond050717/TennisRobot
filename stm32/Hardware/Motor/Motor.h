#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

void Motor_PWM_Init(void);
void Enconder_Init(void);
void Motor_PWM(int left, int right, int roller);

#endif
