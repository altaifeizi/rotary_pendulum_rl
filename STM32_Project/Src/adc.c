#include "adc.h"

// ADC initialization
void Adc_Init(void)
{
 	RCC->APB2ENR |= 1 << 2;     // Enable PORTA clock
	GPIOA->CRL &= 0xFFFF0FFF;   // PA3 as analog input
  GPIOA->CRL &= 0xF0FFFFFF;   // PA6 as analog input

	RCC->APB2ENR |= 3 << 9;     // Enable ADC1 and ADC2 clocks

////////////////////////////////// ADC1 ///////////////////////////////////////
	RCC->APB2RSTR |= 1 << 9;    // Reset ADC1
	RCC->APB2RSTR &= ~(1 << 9); // Finish reset
	RCC->CFGR &= ~(3 << 14);    // Reset prescaler

	RCC->CFGR |= 2 << 14;       // Set prescaler

	ADC1->CR1 &= 0xF0FFFF;      // Set mode to 0
	ADC1->CR1 |= 0 << 16;       // Independent mode
	ADC1->CR1 &= ~(1 << 8);     // Non-scan mode
	ADC1->CR2 &= ~(1 << 1);     // Single conversion mode
	ADC1->CR2 &= ~(7 << 17);
	ADC1->CR2 |= 7 << 17;	      // Software trigger
	ADC1->CR2 |= 1 << 20;       // Use external trigger (SWSTART)
	ADC1->CR2 &= ~(1 << 11);
	ADC1->SQR1 &= ~(0xF << 20);
	ADC1->SQR1 &= 0 << 20;

	// Set sample time for channel 3
	ADC1->SMPR2 &= 0xFFFF0FFF; // Reset sample time
	ADC1->SMPR2 |= 0 << 9;     // Channel: 1.5 cycles for faster speed

	// Set sample time for channel 6
	ADC1->SMPR2 &= 0xF0FFFFFF; // Reset sample time
	ADC1->SMPR2 |= 7 << 18;    // Channel: 239.5 cycles for higher accuracy

	ADC1->CR2 |= 1 << 0;	      // Enable ADC
	ADC1->CR2 |= 1 << 3;       // Reset calibration
	while(ADC1->CR2 & (1 << 3)); // Wait for calibration to complete

	ADC1->CR2 |= 1 << 2;       // Start calibration
	while(ADC1->CR2 & (1 << 2)); // Wait for calibration to complete
	delay_ms(1);
////////////////////////////////// ADC1 ///////////////////////////////////////

////////////////////////////////// ADC2 ///////////////////////////////////////
	RCC->APB2RSTR |= 1 << 10;     // Reset ADC2
	RCC->APB2RSTR &= ~(1 << 10);  // Finish reset
	RCC->CFGR &= ~(3 << 14);      // Reset prescaler

	RCC->CFGR |= 2 << 14;         // Set prescaler

	ADC2->CR1 &= 0xF0FFFF;        // Set mode to 0
	ADC2->CR1 |= 0 << 16;         // Independent mode
	ADC2->CR1 &= ~(1 << 8);       // Non-scan mode
	ADC2->CR2 &= ~(1 << 1);       // Single conversion mode
	ADC2->CR2 &= ~(7 << 17);
	ADC2->CR2 |= 7 << 17;	        // Software trigger
	ADC2->CR2 |= 1 << 20;         // Use external trigger (SWSTART)
	ADC2->CR2 &= ~(1 << 11);
	ADC2->SQR1 &= ~(0xF << 20);
	ADC2->SQR1 &= 0 << 20;

	// Set sample time for channel 3
	ADC2->SMPR2 &= 0xFFFF0FFF; // Reset sample time
	ADC2->SMPR2 |= 0 << 9;     // Channel: 1.5 cycles for faster speed

	// Set sample time for channel 6
	ADC2->SMPR2 &= 0xF0FFFFFF; // Reset sample time
	ADC2->SMPR2 |= 7 << 18;    // Channel: 239.5 cycles for higher accuracy

	ADC2->CR2 |= 1 << 0;	      // Enable ADC
	ADC2->CR2 |= 1 << 3;       // Reset calibration
	while(ADC2->CR2 & (1 << 3)); // Wait for calibration to complete

	ADC2->CR2 |= 1 << 2;       // Start calibration
	while(ADC2->CR2 & (1 << 2)); // Wait for calibration to complete
	delay_ms(1);
////////////////////////////////// ADC2 ///////////////////////////////////////
}

// ADC1 single conversion
u16 Get_Adc(u8 ch)
{
	// Set conversion sequence
	ADC1->SQR3 &= 0xFFFFFFE0; // Regular sequence 1, channel ch
	ADC1->SQR3 |= ch;
	ADC1->CR2 |= 1 << 22;     // Start conversion
	while (!(ADC1->SR & 1 << 1)); // Wait for conversion to finish
	return ADC1->DR;		      // Return ADC value
}

// ADC2 single conversion
u16 Get_Adc2(u8 ch)
{
	// Set conversion sequence
	ADC2->SQR3 &= 0xFFFFFFE0; // Regular sequence 1, channel ch
	ADC2->SQR3 |= ch;
	ADC2->CR2 |= 1 << 22;     // Start conversion
	while (!(ADC2->SR & 1 << 1)); // Wait for conversion to finish
	return ADC2->DR;		      // Return ADC value
}

// Read battery voltage
int Get_battery_volt(void)
{
	int Volt;
	Volt = Get_Adc2(Battery_Ch) * 3.3 * 11 * 100 / 1.0 / 4096;
	return Volt;
}

// Average ADC value of channel ch over "times" measurements
u16 Get_Adc_Average(u8 ch, u8 times)
{
	u32 temp_val = 0;
	u8 t;
	for (t = 0; t < times; t++)
	{
		temp_val += Get_Adc(ch);
		delay_us(200);
	}
	return temp_val / times;
}
