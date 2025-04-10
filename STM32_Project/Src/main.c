#include "sys.h"
#include "control.h"

// Global variables
u8 Flag_Stop = 1;                // Stop flag (1 = stopped)
u8 system_start = 0;             // Flag for starting the adjustment function
u8 tips_flag = 0;                // Flag for displaying tips on the OLED

int Encoder, Position_Zero = 10000;  // Encoder pulses; Position_Zero initialized with offset
int Motor;                       // Motor PWM value
float MotorSpeed;                // Current motor speed
float PendulumSpeed;            // Current pendulum speed
int Voltage;                     // Power supply voltage
float MotorAngleDeg;             // Motor angle in degrees
float MotorAngleRad;             // Motor angle in radians
float Angle_Balance;             // Raw angle value from sensor (for balance)
float PendulumAngleDeg;          // Pendulum angle in degrees
float PendulumAngleDeg_1;        // Previous pendulum angle (degrees)
float PendulumAngleRad;          // Pendulum angle in radians
float LastPendulumAngleDeg = 180.0;      // Last measured pendulum angle (initial: 180°)
float PrevLastPendulumAngleDeg = 180.0;  // Previous to last pendulum angle

// PID parameters for balance and position control
float Balance_KP = 50, Balance_KD = 244, Position_KP = 25, Position_KD = 600;
float Position_KP_1 = 90, Position_KD_1 = 180;

// Additional tuning parameters (e.g., for menu configuration)
float Menu = 1, Amplitude1 = 1, Amplitude2 = 10, Amplitude3 = 1, Amplitude4 = 10;

// Timing variables and interrupt-related flags
float volatile g_lastDuration_s = 0;
volatile uint8_t flag_50ms = 0;
uint8_t counter_5ms = 0;
volatile uint16_t startTimeLocal = 0;
uint16_t startTime, endTime;
uint32_t delta;
float time_s;

int main(void)
{
    // -------------------------------------------------------------------------
    // System and peripheral initialization

    Stm32_Clock_Init(9);             // Initialize system clock; parameter sets multiplier

    delay_init(72);                  // Initialize delay functions; 72 corresponds to clock in MHz

    JTAG_Set(JTAG_SWD_DISABLE);      // Disable JTAG to free pins

    JTAG_Set(SWD_ENABLE);            // Enable SWD (Serial Wire Debug) for programming/debugging

    delay_ms(2000);                  // Short delay for hardware stabilization

    LED_Init();                      // Initialize onboard LED (for status display)

    EXTI_Init();                     // Initialize buttons with external interrupt

    OLED_Init();                     // Initialize OLED display for feedback

    uart_init(72, 115200);           // Initialize UART1: 72MHz system clock, 115200 baud rate

    MiniBalance_PWM_Init(7199, 0);   // Initialize PWM output for the motor (MiniBalance project)

    Encoder_Init_TIM4();             // Initialize encoder on TIM4 to measure position

    Adc_Init();                      // Initialize ADC (Analog-to-Digital Converter)

    Timer1_Init(49, 7199);           // Initialize Timer1 for periodic interrupts (e.g., 5ms)

    // Timer1_Init(499, 7199);       // Alternative setup for 50ms interrupts (commented out)

    Timer2_Init_50ms();              // Initialize Timer2 for 50ms interval tasks

    // Timer3_Measure_Init();       // Optional: for timing measurements, not usable with PWM

    // -------------------------------------------------------------------------
    // Main loop
    // Continuously call Tips(), likely updates the OLED or handles interaction
    while (1)
    {
        Tips();  // Display system tips or status updates on OLED
    }
}
