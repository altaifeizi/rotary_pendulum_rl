#include "control.h"
#include "nn_inference.h"
#include <math.h>
#include "sys.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846  // Define PI if not already defined
#endif

// Global variables for control and status tracking
int Amplitude = 7000;        // Maximum PWM amplitude for the motor
uint8_t lock_r = 0;          // Lock flag for right direction: 0 = free, 1 = locked
uint8_t lock_l = 0;          // Lock flag for left direction: 0 = free, 1 = locked
uint8_t dir = 0;             // Current direction: 0 = left, 1 = right
uint8_t dir_lock = 0;        // Lock flag for direction changes: 0 = free, 1 = locked
int turn = 1;                // Control variable, e.g., for turning or direction reversal
// int counter = 0;         // Commented counter

// -----------------------------------------------------------------------------
// TIM1_UP_IRQHandler: Interrupt handler for TIM1 update event
// This handler is called periodically (e.g., every 5 ms) to read encoder data,
// compute angles, and perform control tasks.
void TIM1_UP_IRQHandler(void)
{
    // Check if TIM1 update interrupt occurred
    if (TIM1->SR & 0X0001)
    {
        // Clear the interrupt flag
        TIM1->SR &= ~(1 << 0);

        // Read encoder value (e.g., from Timer 4) and calculate position
        Encoder = Read_Encoder(4);

        // Compute motor angle in degrees based on encoder and offset (10000),
        // with scaling based on encoder resolution
        MotorAngleDeg = (Encoder - 10000) * 360.0 / 1079.65;

        // Invert motor angle if needed (mirroring)
        invertMotorAngle();

        // Convert motor angle from degrees to radians
        MotorAngleRad = MotorAngleDeg * 3.141592653589793 / 180.0;

        // Read the pendulum angle from ADC (average of 15 samples)
        Angle_Balance = Get_Adc_Average(3, 15);

        // Read the battery voltage
        Voltage = Get_battery_volt();

        // Compute raw pendulum angle in degrees
        float rawPendulumAngleDeg = (Angle_Balance / 4095.0) * 352.5;

        // Transform raw angle: bottom = 180°, top = 0°
        if (rawPendulumAngleDeg >= 90.0)
            PendulumAngleDeg = rawPendulumAngleDeg - 90.0;
        else
            PendulumAngleDeg = rawPendulumAngleDeg + 270.0;

        // Store current pendulum angle for later use
        PendulumAngleDeg_1 = PendulumAngleDeg;

        // Correct wrap-around issues at ±180°
        if (PendulumAngleDeg > 180.0)
            PendulumAngleDeg -= 360.0;

        // Convert pendulum angle from degrees to radians
        PendulumAngleRad = PendulumAngleDeg * M_PI / 180.0;
    }
}

// -----------------------------------------------------------------------------
// Set_Pwm: Sets the PWM signal for the motor.
// Determines direction by the sign of 'moto' and applies absolute PWM value.
void Set_Pwm(int moto)
{
    if (moto < 0)
    {
        BIN2 = 1;   // Set BIN2 high to drive backward
        BIN1 = 0;
    }
    else
    {
        BIN2 = 0;
        BIN1 = 1;   // Set BIN1 high to drive forward
    }
    // Set PWM value to the absolute of moto
    PWMB = (moto < 0) ? -moto : moto;
}

// -----------------------------------------------------------------------------
// limit_Pwm: Limits the PWM output to the defined maximum amplitude.
void limit_Pwm(void)
{
    if (Motor < -Amplitude) Motor = -Amplitude;
    if (Motor >  Amplitude) Motor =  Amplitude;
}

// -----------------------------------------------------------------------------
// Key: Handles button inputs.
// KEY5 = start, KEY7 = stop.
void Key(void)
{
    if (KEY5 == 0)  // Start button pressed
    {
        Flag_Stop = 0;  // Clear stop flag
    }
    if (KEY7 == 0)  // Stop button pressed
    {
        Flag_Stop = 1;  // Set stop flag
    }
}

// -----------------------------------------------------------------------------
// myabs: Returns the absolute value of an integer.
int myabs(int a)
{
    int temp;
    if (a < 0)
        temp = -a;
    else
        temp = a;
    return temp;
}

// -----------------------------------------------------------------------------
// invertMotorAngle: Inverts the motor angle (multiply by -1).
void invertMotorAngle(void)
{
    MotorAngleDeg = MotorAngleDeg * -1;
}
