#include "exti.h"
#include "key.h"

// External interrupt initialization
void EXTI_Init(void)
{
    KEY_Init();  // Initialize buttons

    // Configure external interrupts on falling edge for the following GPIO pins:
    Ex_NVIC_Config(GPIO_A, 7, FTIR);     // Trigger on falling edge (KEY7)
    Ex_NVIC_Config(GPIO_A, 5, FTIR);     // Trigger on falling edge (KEY5)
    Ex_NVIC_Config(GPIO_A, 11, FTIR);    // Trigger on falling edge (KEY11)
    Ex_NVIC_Config(GPIO_A, 12, FTIR);    // Trigger on falling edge (KEY12)

    // Initialize interrupt priority for EXTI9_5 and EXTI15_10
    // Preemption priority = 2, subpriority = 2, group = 2
    MY_NVIC_Init(2, 2, EXTI9_5_IRQn, 2);
    MY_NVIC_Init(2, 2, EXTI15_10_IRQn, 2);
}

// Interrupt service routine for external interrupts on lines 9 to 5
void EXTI9_5_IRQHandler(void)
{
    delay_ms(5);  // Debouncing

    if (KEY5 == 0)  // Toggle start/stop flag
    {
        Flag_Stop = !Flag_Stop;
    }

    if (KEY7 == 0)  // Change menu mode
    {
        if (Menu++ == 4) Menu = 1;
    }

    // Clear pending interrupt flags for line 5 and 7
    EXTI->PR = 1 << 5;
    EXTI->PR = 1 << 7;
}

// Interrupt service routine for external interrupts on lines 15 to 10
void EXTI15_10_IRQHandler(void)
{
    delay_ms(5);  // Debouncing

    if (KEY12 == 0)  // Decrease PID parameter
    {
        if (Menu == 1)        Balance_KP   -= Amplitude1;
        else if (Menu == 2)   Balance_KD   -= Amplitude2;
        else if (Menu == 3)   Position_KP  -= Amplitude3;
        else if (Menu == 4)   Position_KD  -= Amplitude4;
    }

    if (KEY11 == 0)  // Increase PID parameter
    {
        if (Menu == 1)        Balance_KP   += Amplitude1;
        else if (Menu == 2)   Balance_KD   += Amplitude2;
        else if (Menu == 3)   Position_KP  += Amplitude3;
        else if (Menu == 4)   Position_KD  += Amplitude4;
    }

    // Ensure parameters don't fall below 0
    if (Balance_KP   <= 0) Balance_KP   = 0;
    if (Balance_KD   <= 0) Balance_KD   = 0;
    if (Position_KP  <= 0) Position_KP  = 0;
    if (Position_KD  <= 0) Position_KD  = 0;

    // Clear pending interrupt flags for line 11 and 12
    EXTI->PR = 1 << 11;
    EXTI->PR = 1 << 12;
}
