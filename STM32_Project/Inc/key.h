#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"

// Button port definitions
#define KEY5 PAin(5)
#define KEY2 PAin(2)
#define KEY7 PAin(7)
#define KEY11 PAin(11)
#define KEY12 PAin(12)

void KEY_Init(void);          // Button initialization
u8 click_N_Double(u8 time);   // Button scan function (single and double click)
u8 click(void);               // Button scan function (single click)
u8 Long_Press(void);          // Long press detection
#endif
