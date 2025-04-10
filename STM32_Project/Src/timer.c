#include "timer.h"
#include "led.h"

// Timer interrupt initialization
void Timer1_Init(u16 arr, u16 psc)
{
    RCC->APB2ENR |= 1 << 11;   // Enable TIM1 clock
    TIM1->ARR = arr;           // Set auto-reload register
    TIM1->PSC = psc;           // Set prescaler to divide clock (e.g., 7200 => 10 kHz)
    TIM1->DIER |= 1 << 0;      // Enable update interrupt
    TIM1->DIER |= 1 << 6;      // Enable trigger interrupt
    TIM1->CR1 |= 0x01;         // Enable timer
    MY_NVIC_Init(1, 3, TIM1_UP_IRQn, 2);  // Configure NVIC
}

// Initialize TIM2 for a 50 ms interval
void Timer2_Init_50ms(void)
{
    // 1) Enable TIM2 clock (bit 0 in APB1ENR)
    RCC->APB1ENR |= (1 << 0);

    // 2) Set prescaler and ARR
    // PSC = 71 => 72 MHz / (71+1) = 1 MHz (1 µs per tick)
    TIM2->PSC = 71;
    // For 50 ms = 50000 µs, ARR = 50000 - 1 = 49999
    TIM2->ARR = 49999;

    // 3) Enable update interrupt
    TIM2->DIER |= (1 << 0);  // Enable UIF interrupt

    // 4) Start timer
    TIM2->CR1 |= (1 << 0);

    // 5) Configure NVIC (example: set priority)
    MY_NVIC_Init(1, 3, TIM2_IRQn, 1);
}

// Initialize TIM3 for time measurement
void Timer3_Measure_Init(void)
{
    // 1) Enable TIM3 clock (bit 1 in APB1ENR)
    RCC->APB1ENR |= (1 << 1);

    // 2) Set prescaler: 71 => 1 MHz (1 µs per tick)
    TIM3->PSC = 71;

    // 3) Set auto-reload to max (16-bit timer)
    TIM3->ARR = 0xFFFF;

    // 4) Reset counter
    TIM3->CNT = 0;

    // 5) Start timer (CEN = 1)
    TIM3->CR1 |= (1 << 0);
}
