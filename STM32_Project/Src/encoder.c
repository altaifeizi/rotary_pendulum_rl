#include "encoder.h"

// Initialize TIM2 for encoder interface mode
void Encoder_Init_TIM2(void)
{
    RCC->APB1ENR |= 1 << 0;    // Enable TIM2 clock
    RCC->APB2ENR |= 1 << 2;    // Enable PORTA clock
    GPIOA->CRL &= 0XFFFFFF00;
    GPIOA->CRL |= 0X00000044;  // Set PA0 and PA1 to floating input

    TIM2->DIER |= 1 << 0;   // Enable update interrupt
    TIM2->DIER |= 1 << 6;   // Enable trigger interrupt
    MY_NVIC_Init(1, 3, TIM2_IRQn, 1);  // Initialize interrupt (priority group 1, sub-priority 3)

    TIM2->PSC = 0x0;                         // Prescaler
    TIM2->ARR = ENCODER_TIM_PERIOD;         // Auto-reload value
    TIM2->CR1 &= ~(3 << 8);                 // No clock division
    TIM2->CR1 &= ~(3 << 5);                 // Edge-aligned mode

    TIM2->CCMR1 |= 1 << 0;  // CC1S = '01': IC1 is mapped to TI1
    TIM2->CCMR1 |= 1 << 8;  // CC2S = '01': IC2 is mapped to TI2
    TIM2->CCER &= ~(1 << 1);  // CC1P = '0': non-inverted
    TIM2->CCER &= ~(1 << 5);  // CC2P = '0': non-inverted
    TIM2->CCMR1 |= 3 << 4;    // IC1F = '1000': input capture filter
    TIM2->SMCR |= 3 << 0;     // SMS = '011': both TI1 and TI2 valid on both edges
    TIM2->CNT = 10000;        // Set initial count value
    TIM2->CR1 |= 0x01;        // Enable TIM2
}

// Initialize TIM4 for encoder interface mode
void Encoder_Init_TIM4(void)
{
    RCC->APB1ENR |= 1 << 2;    // Enable TIM4 clock
    RCC->APB2ENR |= 1 << 3;    // Enable PORTB clock
    GPIOB->CRL &= 0X00FFFFFF;
    GPIOB->CRL |= 0X44000000;  // Set PB6 and PB7 to floating input

    TIM4->PSC = 0x0;                         // Prescaler
    TIM4->ARR = ENCODER_TIM_PERIOD;         // Auto-reload value
    TIM4->CR1 &= ~(3 << 8);                 // No clock division
    TIM4->CR1 &= ~(3 << 5);                 // Edge-aligned mode

    TIM4->CCMR1 |= 3 << 4;    // IC1F = '1000': input capture filter
    TIM4->SMCR |= 3 << 0;     // SMS = '011': both TI1 and TI2 valid on both edges
    TIM4->CNT = 10000;        // Set initial count value
    TIM4->CR1 |= 0x01;        // Enable TIM4
}

// Read encoder count for selected timer
int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;
    switch (TIMX)
    {
        case 2:  Encoder_TIM = (short)TIM2->CNT; break;
        case 3:  Encoder_TIM = (short)TIM3->CNT; break;
        case 4:  Encoder_TIM = (short)TIM4->CNT; break;
        default: Encoder_TIM = 0;
    }
    return Encoder_TIM;
}

/*
// TIM2 interrupt handler (optional, currently not used)
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & 0X0001) // Overflow interrupt
    {
        // Handle overflow if necessary
    }
    TIM2->SR &= ~(1 << 0); // Clear interrupt flag
}
*/
