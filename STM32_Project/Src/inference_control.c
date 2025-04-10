#include "inference_control.h"
#include "control.h"
#include "nn_inference.h"
#include "sys.h"

// Global variables for control and state measurement
int motor = 0;                         // Calculated PWM value for the motor
float motorAngleRad, motorSpeed;       // Current motor angle (in radians) and angular velocity
float pendulumAngleRad, pendulumSpeed; // Current pendulum angle (in radians) and angular velocity
float prev_MotorAngleRad = 0.0;        // Previous motor angle (used for derivative computation)
float prev_PendulumAngleRad = 0.0;     // Previous pendulum angle
int action = 1;                        // Inferred output from the agent (discrete action)

// uint16_t startTimeLocal = 0;       // Example for local timing (commented out)

/**
 * @brief TIM2 Interrupt Handler
 *
 * This handler is called periodically (e.g., every 50 ms).
 * It performs the following steps:
 *  - Processes user input and limits the PWM
 *  - Reads the current angles of the motor and pendulum
 *  - Computes angular velocities (derivatives)
 *  - Runs inference using the neural network
 *  - Computes the PWM value based on the selected action
 */
void TIM2_IRQHandler(void)
{
    // Check if the update interrupt was triggered
    if (TIM2->SR & TIM_SR_UIF)
    {
        // Clear the update interrupt flag
        TIM2->SR &= ~TIM_SR_UIF;

        // Process user input and limit PWM
        Key();
        limit_Pwm();

        // Apply motor PWM if not stopped
        if (Flag_Stop == 0) {
            Set_Pwm(motor);

            // Optional serial debug output
            // printf("%.6f,%.6f,%d\r\n", motorAngleRad, pendulumAngleRad, action);
            // fflush(stdout);
        } else {
            Set_Pwm(0);  // Stop the motor
        }

        // Read current angle values (from sensors or global variables)
        motorAngleRad = MotorAngleRad;
        pendulumAngleRad = PendulumAngleRad;

        // Compute angular velocities (derivatives)
        compute_derivatives(motorAngleRad, &motorSpeed, pendulumAngleRad, &pendulumSpeed);

        // Run inference using the neural network (input: current angles and velocities)
        action = NN_Inference_GetArgMax(motorAngleRad, motorSpeed, pendulumAngleRad, pendulumSpeed);

        // Calculate motor PWM based on action (-1, 0, 1 scaled by 4800)
        motor = 4800 * action;

        // Adjust action to shift range to [0, 2] if needed
        action += 1;
    }
}

/**
 * @brief Computes angular velocities (derivatives)
 *
 * This function computes the derivative of motor and pendulum angles
 * using the difference between the current and previous values,
 * divided by a fixed time step.
 *
 * @param mot       Current motor angle (radians)
 * @param motSpeed  Pointer to store the motor angular velocity
 * @param pend      Current pendulum angle (radians)
 * @param pendSpeed Pointer to store the pendulum angular velocity
 */
void compute_derivatives(float mot, float* motSpeed, float pend, float* pendSpeed)
{
    const float delta_time = 0.05;  // Fixed timestep (50 ms)

    if (delta_time > 0) {
        *motSpeed = (mot - prev_MotorAngleRad) / delta_time;
        *pendSpeed = (pend - prev_PendulumAngleRad) / delta_time;
    } else {
        *motSpeed = 0.0f;
        *pendSpeed = 0.0f;
    }

    // Update previous angle values for next call
    prev_MotorAngleRad = mot;
    prev_PendulumAngleRad = pend;
}

/*
 * Example: Optional timing measurement using TIM3 (commented out)
 *
uint16_t endTimeLocal = TIM3->CNT;

uint32_t localDelta;
if (endTimeLocal >= startTimeLocal) {
    localDelta = endTimeLocal - startTimeLocal;
} else {
    localDelta = (0x10000 - startTimeLocal) + endTimeLocal;
}

float localElapsed_s = localDelta / 1e6f;  // At 1 MHz: 1 tick = 1 µs

g_lastDuration_s = localElapsed_s;

startTimeLocal = TIM3->CNT;
*/
