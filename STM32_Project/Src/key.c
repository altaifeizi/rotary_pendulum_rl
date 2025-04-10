#include "key.h"

// Button initialization
void KEY_Init(void)
{
    RCC->APB2ENR |= 1 << 2;     // Enable clock for PORTA

    // Configure PA11 and PA12 as input with pull-up
    GPIOA->CRH &= 0xFFF00FFF;
    GPIOA->CRH |= 0x00088000;

    // Configure PA2, PA5, and PA7 as input with pull-up
    GPIOA->CRL &= 0x0F0FF0FF;
    GPIOA->CRL |= 0x80800800;

    GPIOA->ODR |= 1 << 2;       // Enable pull-up on PA2
    GPIOA->ODR |= 1 << 7;       // Enable pull-up on PA7
    GPIOA->ODR |= 1 << 5;       // Enable pull-up on PA5
    GPIOA->ODR |= 3 << 11;      // Enable pull-up on PA11 and PA12
}

// Button scan function (detects single and double clicks)
u8 click_N_Double(u8 time)
{
    static u8 flag_key, count_key, double_key;
    static u16 count_single, Forever_count;

    if (KEY2 == 0) Forever_count++;  // Only increase if long-press flag is not set
    else Forever_count = 0;

    if (KEY2 == 0 && flag_key == 0) flag_key = 1;

    if (count_key == 0)
    {
        if (flag_key == 1)
        {
            double_key++;
            count_key = 1;
        }
        if (double_key == 2)
        {
            double_key = 0;
            count_single = 0;
            return 2; // Double click detected
        }
    }

    if (KEY2 == 1)
    {
        flag_key = 0;
        count_key = 0;
    }

    if (double_key == 1)
    {
        count_single++;
        if (count_single > time && Forever_count < time)
        {
            double_key = 0;
            count_single = 0;
            return 1; // Single click detected
        }
        if (Forever_count > time)
        {
            double_key = 0;
            count_single = 0;
        }
    }

    return 0; // No click
}

// Button scan function (returns pressed button ID)
u8 click(void)
{
    static u8 flag_key = 1; // Button release flag

    if (flag_key && (KEY5 == 0 || KEY11 == 0 || KEY12 == 0))
    {
        flag_key = 0;
        delay_ms(50); // Debounce delay

        if (KEY5 == 0) return 5;
        else if (KEY11 == 0) return 11;
        else if (KEY12 == 0) return 12;
    }
    else
    {
        flag_key = 1;
    }

    return 0; // No button pressed
}

// Long press detection function
u8 Long_Press(void)
{
    static u16 Long_Press_count, Long_Press = 0;

    if (Long_Press == 0 && KEY7 == 0)
        Long_Press_count++;  // Increase if long-press flag not set
    else
        Long_Press_count = 0;

    if (Long_Press_count > 20)
    {
        Long_Press = 1;
        Long_Press_count = 0;
        return 1;
    }

    if (Long_Press == 1)
    {
        Long_Press = 0;
    }

    return 0;
}
