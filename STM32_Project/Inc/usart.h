#ifndef __USART_H
#define __USART_H

#include "sys.h"
#include "stdio.h"
#include <unistd.h>
int _write(int file, char *ptr, int len);
void usart1_send(u8 data);
void uart_init(u32 pclk2,u32 bound);
#endif	   
















