#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 

//Motor-Port definitions
#define PWMB   TIM3->CCR4   
#define BIN2   PBout(12)     
#define BIN1   PBout(13)     
#define AIN1   PBout(14)    
#define AIN2   PBout(15)     
#define PWMA   TIM3->CCR3    
void MiniBalance_PWM_Init(u16 arr,u16 psc);
void MiniBalance_Motor_Init(void);
#endif
