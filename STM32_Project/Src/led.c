#include "led.h"

// Initialize LED
void LED_Init(void)
{
    RCC->APB2ENR |= 1 << 2;      // Enable clock for PORTA
    GPIOA->CRL &= 0xFFF0FFFF;
    GPIOA->CRL |= 0x00030000;    // Set PA4 as push-pull output
    GPIOA->ODR |= 1 << 4;        // Set PA4 to high (turn LED off initially)
}

// LED blink function
void Led_Flash(u16 time)
{
    static int temp;

    if (time == 0)
        LED = 0;                 // Turn LED off
    else if (++temp == time)
    {
        LED = ~LED;             // Toggle LED
        temp = 0;               // Reset counter
    }
}
