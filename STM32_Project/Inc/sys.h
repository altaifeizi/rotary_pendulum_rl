#ifndef __SYS_H
#define __SYS_H	  
#include <stm32f10x.h>   

#define SYSTEM_SUPPORT_UCOS		0		// Define whether uC/OS is supported

// Macro definitions for IO port operations
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))

// IO port address mappings
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08

// IO port operations
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  // Output
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  // Input

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  // Output
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  // Input

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  // Output
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  // Input

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  // Output
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  // Input

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  // Output
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  // Input

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  // Output
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  // Input

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  // Output
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  // Input


// External interrupt port configuration
#define GPIO_A 0
#define GPIO_B 1
#define GPIO_C 2
#define GPIO_D 3
#define GPIO_E 4
#define GPIO_F 5
#define GPIO_G 6

#define FTIR   1  // Falling edge trigger
#define RTIR   2  // Rising edge trigger

#include "delay.h"
#include "led.h"
#include "key.h"
#include "oled.h"
#include "usart.h"
#include "adc.h"
#include "timer.h"
#include "motor.h"
#include "encoder.h"
#include "show.h"
#include "exti.h"
#include "check.h"
#include "nn_inference.h"
#include "inference_control.h"

// JTAG mode definitions
#define JTAG_SWD_DISABLE   0X02
#define SWD_ENABLE         0X01
#define JTAG_SWD_ENABLE    0X00

extern u8 Flag_Stop;                        // Stop flag
extern int Encoder, Position_Zero;         // Encoder pulse count
extern int Motor;                           // Motor PWM
extern float MotorSpeed;
extern float PendulumSpeed;
extern float MotorAngleDeg;  				// Global variable for motor angle in degrees
extern float MotorAngleRad;  				// Global variable for motor angle in radians
extern int Voltage;                         // Supply voltage
extern float Angle_Balance;                // Angle sensor data
extern float PendulumAngleDeg;			    // Pendulum angle in degrees
extern float PendulumAngleDeg_1;
extern float PendulumAngleRad;			    // Pendulum angle in radians
extern float LastPendulumAngleDeg;
extern float PrevLastPendulumAngleDeg;
extern float Balance_KP, Balance_KD, Position_KP, Position_KD;  	// PID parameters
extern float Menu, Amplitude1, Amplitude2, Amplitude3, Amplitude4; // PID parameters
extern u8 system_start;
extern volatile float g_lastDuration_s;
extern uint16_t startTime, endTime;
extern uint32_t delta;
extern float time_s;
extern volatile uint8_t flag_50ms;
extern uint8_t counter_5ms;

extern float Position_KP_1, Position_KD_1;

/////////////////////////////////////////////////////////////////
void Stm32_Clock_Init(u8 PLL);  // Initialize system clock
void Sys_Soft_Reset(void);      // System soft reset
void Sys_Standby(void);         // Standby mode
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset); // Set vector table offset address
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group); // Set NVIC group
void MY_NVIC_Init(u8 NVIC_PreemptionPriority, u8 NVIC_SubPriority, u8 NVIC_Channel, u8 NVIC_Group); // Configure interrupt
void Ex_NVIC_Config(u8 GPIOx, u8 BITx, u8 TRIM); // External interrupt configuration
void JTAG_Set(u8 mode);

float NN_Inference_GetFirstOutput(float mot, float motSpeed, float pend, float pendSpeed);
void Timer2_Init_50ms(void);
void Timer3_Measure_Init(void);

void WFI_SET(void);		   // Execute WFI instruction
void INTX_DISABLE(void);   // Disable all interrupts
void INTX_ENABLE(void);	   // Enable all interrupts
void MSR_MSP(u32 addr);	   // Set main stack pointer
#endif
