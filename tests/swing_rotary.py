import numpy as np
import matplotlib

# Set the backend of Matplotlib to "TkAgg", which can help avoid plot issues,
# especially in development environments like PyCharm.
matplotlib.use("TkAgg")     # For PyCharm issues
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp


def sign_smooth(x, eps=1e-4):
    """
    Smooth approximation of the sign function.

    The classical sign function jumps abruptly from -1 to 1, which can cause problems
    in some simulations. This function smooths that transition using the expression
    x / sqrt(x^2 + eps^2).

    Parameters:
      x   : Input value or array of values.
      eps : Small positive value to avoid division by zero (default: 1e-4).

    Returns:
      A value or array that behaves like the classical sign function for large |x|,
      but shows a smoother transition near zero.
    """
    return x / np.sqrt(x ** 2 + eps ** 2)


def dstate_dt_motorpendulum(t, x, params):
    """
    Model of a motor arm as a pendulum.

    This model describes the dynamics of a motor arm considered as a pendulum.
    The stable state is at theta = 0 (motor arm hanging downward).

    State vector x:
      x = [theta, theta_dot]
    where:
      - theta      : Angle of the motor arm (in radians).
      - theta_dot  : Angular velocity (in radians per second).

    Dynamic equation:
      J_r * theta_ddot = - m_r * g * r_cm * sin(theta)
                         - B_r * theta_dot - mu_r * sign_smooth(theta_dot)

    Parameters:
      t      : Time (automatically passed by solve_ivp).
      x      : Current state vector [theta, theta_dot].
      params : Dictionary with physical parameters:
               - m_r  : Mass of the motor arm (kg).
               - r_cm : Effective lever arm, i.e., distance from the pivot to the center of mass (m).
               - g    : Gravity acceleration (m/s²).
               - J_r  : Moment of inertia of the arm (kg·m²).
               - B_r  : Viscous damping coefficient (Nm/(rad/s)).
               - mu_r : Coulomb damping (Nm).

    Returns:
      An array with the derivatives: [theta_dot, theta_ddot].
    """
    # Unpack the state vector into angle (theta) and angular velocity (theta_dot)
    theta, theta_dot = x

    # --- Extract parameters from the dictionary ---
    m_r = params["m_r"]      # Mass of the motor arm in kg
    r_cm = params["r_cm"]    # Effective lever arm in m (distance from pivot to center of mass)
    g = params["g"]          # Gravity acceleration in m/s²
    J_r = params["J_r"]      # Moment of inertia in kg·m²
    B_r = params["B_r"]      # Viscous damping coefficient in Nm/(rad/s)
    mu_r = params["mu_r"]    # Coulomb damping in Nm

    # --- Compute gravitational torque ---
    # The gravitational torque pulls the motor arm back to the resting position (theta = 0).
    torque_grav = - m_r * g * r_cm * np.sin(theta)

    # --- Compute damping term ---
    # Two damping effects are combined here:
    # 1. Viscous damping proportional to angular velocity.
    # 2. Coulomb damping modeled with the smoothed sign function to avoid discontinuities.
    damping = - B_r * theta_dot - mu_r * sign_smooth(theta_dot)

    # --- Compute angular acceleration ---
    # The sum of gravitational torque and damping is divided by the moment of inertia
    # to obtain the angular acceleration (theta_ddot).
    theta_ddot = (torque_grav + damping) / J_r

    # Return the state derivatives:
    # [theta_dot (first derivative of angle), theta_ddot (second derivative)]
    return np.array([theta_dot, theta_ddot])


# Main program: simulation and visualization
if __name__ == "__main__":
    # --- 1) Define the parameters for the motor arm pendulum ---
    params = {
        "m_r": 0.196,    # Mass of the motor arm in kg (example value, can be adjusted)
        "r_cm": 0.088,   # Effective distance to center of mass in m (88 mm)
        "g": 9.81,       # Gravity acceleration in m/s²
        "J_r": 0.0026,   # Moment of inertia in kg·m²
        "B_r": 0.0105,   # Viscous damping in Nm/(rad/s)
        "mu_r": 0.025    # Coulomb damping in Nm
    }

    # --- 2) Set initial conditions ---
    # The motor arm starts from an initial position 90° (pi/2 rad) away from the stable position (theta = 0).
    theta0_deg = 90.0  # Initial angle in degrees
    theta0_rad = np.deg2rad(theta0_deg)  # Convert degrees to radians
    x0 = np.array([theta0_rad, 0.0])     # Initial state: [angle, angular velocity = 0]

    # --- 3) Define the time range for the simulation ---
    t_start = 0.0     # Start time in seconds
    t_end = 1.5       # End time in seconds (can be adjusted)
    # Create a time vector with 1500 equally spaced time points between t_start and t_end
    t_eval = np.linspace(t_start, t_end, 1500)

    # --- 4) Numerical simulation with solve_ivp ---
    # solve_ivp integrates the differential equation (here for the motor arm)
    # over the defined time range.
    sol = solve_ivp(
        fun=lambda t, x: dstate_dt_motorpendulum(t, x, params),  # Pass parameters to the dynamics function
        t_span=(t_start, t_end),  # Simulation time range
        y0=x0,                    # Initial conditions
        t_eval=t_eval             # Time points at which the solution is evaluated
    )

    # Extract the angle solution (theta) from the integration result.
    theta_sol = sol.y[0, :]

    # --- 5) Visualization of the results ---
    plt.figure(figsize=(8, 4))           # Create a new figure with the specified size
    plt.title("Motor Arm Sim: Oscillation")  # Title of the plot
    # Plot the angle theta over time
    plt.plot(sol.t, theta_sol, label=r"$\theta$ [rad]")
    plt.xlabel("Time t [s]")             # Label for the x-axis (time)
    plt.ylabel("Angle (radians)")        # Label for the y-axis (angle in radians)
    plt.grid(True)                       # Show grid for better readability
    plt.legend(loc="best")               # Show legend in the best location
    plt.tight_layout()                   # Optimize layout to avoid overlap
    plt.show()                           # Display the plot
