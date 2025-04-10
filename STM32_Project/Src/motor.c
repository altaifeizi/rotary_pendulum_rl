#include "motor.h"

// Initializes the motor control pins
void MiniBalance_Motor_Init(void)
{
    RCC->APB2ENR |= 1 << 3;        // Enable clock for PORTB
    GPIOB->CRH &= 0x0000FFFF;      // Clear configuration for PB12 - PB15
    GPIOB->CRH |= 0x22220000;      // Set PB12, PB13, PB14, and PB15 as push-pull outputs
}

// Initializes PWM on TIM3 for motor control
void MiniBalance_PWM_Init(u16 arr, u16 psc)
{
    MiniBalance_Motor_Init();     // Initialize motor control IOs

    RCC->APB1ENR |= 1 << 1;        // Enable TIM3 clock
    RCC->APB2ENR |= 1 << 3;        // Enable PORTB clock

    GPIOB->CRL &= 0xFFFFFF00;      // Clear configuration for PB0 and PB1
    GPIOB->CRL |= 0x000000BB;      // Set PB0 and PB1 as alternate function push-pull

    TIM3->ARR = arr;               // Set auto-reload value (timer top value)
    TIM3->PSC = psc;               // Set prescaler value

    TIM3->CCMR2 |= 6 << 12;        // CH4 in PWM mode 1
    TIM3->CCMR2 |= 6 << 4;         // CH3 in PWM mode 1

    TIM3->CCMR2 |= 1 << 11;        // Enable preload for CH4
    TIM3->CCMR2 |= 1 << 3;         // Enable preload for CH3

    TIM3->CCER |= 1 << 12;         // Enable output for CH4
    TIM3->CCER |= 1 << 8;          // Enable output for CH3

    TIM3->CR1 = 0x80;              // Enable auto-reload preload (ARPE)
    TIM3->CR1 |= 0x01;             // Enable TIM3
}
