#include "delay.h"

#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					// Use uC/OS
#endif

static u8  fac_us = 0;   // us delay multiplier
static u16 fac_ms = 0;   // ms delay multiplier

#ifdef OS_CRITICAL_METHOD

void SysTick_Handler(void)
{
	OSIntEnter();
	OSTimeTick();
	OSIntExit();
}
#endif

// Delay initialization
// When using uC/OS, this function initializes the uC/OS system tick
// The system clock is fixed to 1/8 of HCLK
void delay_init(u8 SYSCLK)
{
#ifdef OS_CRITICAL_METHOD 	// If defined, uC/OS II is being used
	u32 reload;
#endif
	SysTick->CTRL &= ~(1 << 2);	// Use external clock source for SYSTICK
	fac_us = SYSCLK / 8;		// Must be set regardless of OS usage

#ifdef OS_CRITICAL_METHOD
	reload = SYSCLK / 8;		// Ticks per second (in K)
	reload *= 1000000 / OS_TICKS_PER_SEC; // Overflow time based on OS tick rate
	// Reload is 24-bit (max 16,777,216). For 72 MHz, max time ~1.86 sec
	fac_ms = 1000 / OS_TICKS_PER_SEC; // Minimum delay OS can handle
	SysTick->CTRL |= 1 << 1;   	// Enable SYSTICK interrupt
	SysTick->LOAD = reload; 	// Set reload value
	SysTick->CTRL |= 1 << 0;   	// Enable SYSTICK
#else
	fac_ms = (u16)fac_us * 1000; // For non-uC/OS, define ms delay ticks
#endif
}

#ifdef OS_CRITICAL_METHOD
// Delay in microseconds
void delay_us(u32 nus)
{
	u32 ticks;
	u32 told, tnow, tcnt = 0;
	u32 reload = SysTick->LOAD;	// LOAD value
	ticks = nus * fac_us; 		// Required ticks
	tcnt = 0;
	OSSchedLock();				// Lock uC/OS scheduling to prevent delay interruption
	told = SysTick->VAL;        // Initial counter value
	while (1)
	{
		tnow = SysTick->VAL;
		if (tnow != told)
		{
			if (tnow < told) tcnt += told - tnow; // SYSTICK is counting down
			else tcnt += reload - tnow + told;
			told = tnow;
			if (tcnt >= ticks) break; // Exit loop when delay time is met
		}
	};
	OSSchedUnlock();			// Unlock uC/OS scheduling
}

// Delay in milliseconds
void delay_ms(u16 nms)
{
	if (OSRunning == OS_TRUE) // If uC/OS is running
	{
		if (nms >= fac_ms) // If delay time is longer than OS minimum tick
		{
   			OSTimeDly(nms / fac_ms); // Use uC/OS delay
		}
		nms %= fac_ms; // Handle remainder using regular delay
	}
	delay_us((u32)(nms * 1000));
}
#else // uC/OS not used

// Delay in microseconds
void delay_us(u32 nus)
{
	u32 temp;
	SysTick->LOAD = nus * fac_us; // Set delay
	SysTick->VAL = 0x00;          // Clear counter
	SysTick->CTRL = 0x01;         // Start timer
	do
	{
		temp = SysTick->CTRL;
	}
	while ((temp & 0x01) && !(temp & (1 << 16))); // Wait for timeout
	SysTick->CTRL = 0x00;       // Disable timer
	SysTick->VAL = 0X00;        // Clear counter
}

// Delay in milliseconds
// SysTick->LOAD is 24-bit. Max delay:
// nms <= 2^24 * 8 * 1000 / SYSCLK
// SYSCLK in Hz, nms in ms
// For 72 MHz: nms <= 1864
void delay_ms(u16 nms)
{
	u32 temp;
	SysTick->LOAD = (u32)nms * fac_ms; // Set delay
	SysTick->VAL = 0x00;               // Clear counter
	SysTick->CTRL = 0x01;              // Start timer
	do
	{
		temp = SysTick->CTRL;
	}
	while ((temp & 0x01) && !(temp & (1 << 16))); // Wait for timeout
	SysTick->CTRL = 0x00;       // Disable timer
	SysTick->VAL = 0X00;        // Clear counter
}
#endif
