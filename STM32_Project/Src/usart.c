#include "usart.h"

// Support for printf function
#if 1
struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};
/* FILE is typedef’d in stdio.h. */
FILE __stdout;

// Define sys_exit to avoid using semihosting mode
void _sys_exit(int x)
{
    while (1)
    {
        // Infinite loop to stop the system
    }
}

// Redefine fputc function
int fputc(int ch, FILE *f) {
    // Correct flag: USART_SR_TXE (Transmit data register empty)
    while ((USART1->SR & USART_SR_TXE) == 0); // Wait until TX buffer is free
    USART1->DR = (uint8_t)ch;
    return ch;
}

// Redefine _write to support functions like puts() or write()
int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        usart1_send(ptr[i]); // Use custom send function
    }
    return len;
}
#endif
// End of printf support

// Send a single byte over USART1
void usart1_send(u8 data)
{
    USART1->DR = data;
    while((USART1->SR & 0x40) == 0); // Wait until transmission is complete
}

// USART1 initialization
void uart_init(u32 pclk2, u32 bound)
{
    float temp;
    u16 mantissa;
    u16 fraction;

    temp = (float)(pclk2 * 1000000) / (bound * 16); // Calculate USARTDIV
    mantissa = temp;                    // Integer part
    fraction = (temp - mantissa) * 16; // Fractional part
    mantissa <<= 4;
    mantissa += fraction;

    RCC->APB2ENR |= 1 << 2;    // Enable PORTA clock
    RCC->APB2ENR |= 1 << 14;   // Enable UART clock

    GPIOA->CRH &= 0xFFFFF00F;
    GPIOA->CRH |= 0x000008B0;  // Set IO state (PA9 as TX, PA10 as RX)

    RCC->APB2RSTR |= 1 << 14;  // Reset USART1
    RCC->APB2RSTR &= ~(1 << 14); // Release reset

    USART1->BRR = mantissa;    // Set baud rate
    USART1->CR1 |= 0x200C;     // Enable USART, 1 stop bit, no parity

    volatile uint32_t dummy = USART1->SR; // Dummy read to avoid missing the first byte
    (void)dummy;
}
