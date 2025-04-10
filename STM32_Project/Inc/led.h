#ifndef __LED_H
#define __LED_H
#include "sys.h"

//LED-Port definitions
#define LED PAout(4) // PA4
void LED_Init(void);  
void Led_Flash(u16 time);
#endif
