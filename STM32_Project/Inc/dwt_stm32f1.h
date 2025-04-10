#ifndef DWT_STM32F1_H
#define DWT_STM32F1_H

#include <stdint.h>

void DWT_Init(void);
uint32_t DWT_GetCycleCount(void);
float DWT_Seconds(uint32_t cycles);
float DWT_Us(uint32_t cycles);

#endif // DWT_STM32F1_H

