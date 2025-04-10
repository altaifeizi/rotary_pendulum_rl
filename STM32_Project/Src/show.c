#include "show.h"

unsigned char i, temp;       // Loop variable
unsigned char Send_Count;    // Number of data bytes to be sent via UART
float Vol;

// Function to display information on the OLED menu
void oled_show(void)
{
    // First row: Motor angle
    OLED_ShowString(00, 00, (const u8 *)"MOTOR:");
    OLED_ShowFloat(60, 00, MotorAngleDeg, 12, 2);

    // Second row: Pendulum angle
    OLED_ShowString(0, 20, (const u8 *)"PENDULUM:");
    //OLED_ShowNumber(60, 20, (int)PendulumAngleDeg, 3, 12);
    OLED_ShowFloat(60, 20, PendulumAngleDeg, 12, 2);

    // Fifth row: Supply voltage and zero position
    OLED_ShowString(80, 40, (const u8 *)"T:");
    OLED_ShowNumber(95, 40, Position_Zero, 5, 12);

    OLED_ShowString(00, 40, (const u8 *)"VOL:");
    OLED_ShowString(41, 40, (const u8 *)".");
    OLED_ShowString(63, 40, (const u8 *)"V");
    OLED_ShowNumber(28, 40, Voltage / 100, 2, 12);
    OLED_ShowNumber(51, 40, Voltage % 100, 2, 12);

    // If the decimal part is a single digit, show leading zero
    if (Voltage % 100 < 10)
        OLED_ShowNumber(45, 40, 0, 2, 12);

    // Sixth row: Encoder and ADC sensor values
    OLED_ShowString(80, 50, (const u8 *)"P:");
    OLED_ShowNumber(95, 50, Encoder, 5, 12);

    OLED_ShowString(0, 50, (const u8 *)"ADC:");
    OLED_ShowNumber(30, 50, Angle_Balance, 4, 12);

    // Refresh display with updated contents
    OLED_Refresh_Gram();
}
