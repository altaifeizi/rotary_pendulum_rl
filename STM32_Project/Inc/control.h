#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

#define PI 3.14159265
#define Mittelwert 3100

#define FILTERING_TIMES  4

extern	int Balance_Pwm,Velocity_Pwm;
void TIM1_UP_IRQHandler(void);
void Set_Pwm(int moto);
void Key(void);
void limit_Pwm(void);
int myabs(int a);
void fixCriticalSpace(void);
void invertMotorAngle(void);
	
#endif
