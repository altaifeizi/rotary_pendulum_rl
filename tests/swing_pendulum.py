import numpy as np
import matplotlib

# Set the Matplotlib backend to "TkAgg" for interactive window support
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp


def sign_smooth(x, eps=1e-4):
    """
    Smooth approximation of the sign function.

    The classical sign function jumps abruptly from -1 to 1 at x=0. This
    function smooths the transition using the expression x / sqrt(x^2 + eps^2).
    This avoids a hard jump at x=0.

    Parameters:
        x   : Input value or array of values.
        eps : Small positive value to secure the denominator (default: 1e-4).

    Returns:
        A value (or array) that approximates the classical sign function for large |x|,
        but shows a smooth transition near zero.
    """
    return x / np.sqrt(x ** 2 + eps ** 2)


def dstate_dt_pendulum(t, x, params):
    """
    Computes the derivatives of the state vector of a simple pendulum.

    This model describes the dynamics of a pendulum without a rotating arm.
    The stable lower equilibrium position is at alpha = pi (180°).

    State vector:
        x = [alpha, alpha_dot]
    where:
        - alpha     : Current angle of the pendulum (in radians).
        - alpha_dot : Angular velocity (in radians per second).

    The dynamics are governed by the torque balance:
        I_eff * alpha_ddot = m_p * g * r_cm * sin(alpha) + (friction term)
    where I_eff is the effective moment of inertia.

    Parameters:
        t      : Time (automatically passed by solve_ivp).
        x      : Current state vector [alpha, alpha_dot].
        params : Dictionary containing all relevant system parameters:
                 - m_p : Mass of the pendulum (kg).
                 - r_cm: Effective center of mass distance (m).
                 - J_p : Moment of inertia of the pendulum (kg·m²).
                 - g   : Gravitational acceleration (m/s²).
                 - B_p : Viscous damping coefficient.
                 - mu  : Coulomb friction (Nm).

    Returns:
        An array with the derivatives: [alpha_dot, alpha_ddot].
    """
    # Unpack state vector: angle and angular velocity
    alpha, alpha_dot = x

    # --- Extract parameters from the dictionary ---
    m_p = params["m_p"]       # Pendulum mass (kg)
    r_cm = params["r_cm"]     # Effective center of mass distance (m)
    J_p = params["J_p"]       # Moment of inertia (kg·m²)
    g = params["g"]           # Gravitational acceleration (m/s²)
    B_p = params["B_p"]       # Viscous damping coefficient
    mu = params["mu"]         # Coulomb friction (Nm)

    # --- Compute effective moment of inertia ---
    # Define an effective moment of inertia accounting for mass distribution:
    # I_eff = J_p + m_p * (r_cm)^2
    I_eff = J_p + m_p * (r_cm ** 2)

    # --- Compute friction torque ---
    # Includes two types of damping:
    # 1. Viscous damping: proportional to angular velocity.
    # 2. Coulomb damping: modeled using the smooth sign function to avoid discontinuities.
    friction_torque = -B_p * alpha_dot - mu * sign_smooth(alpha_dot)

    # --- Compute angular acceleration ---
    # Torque equation:
    # I_eff * alpha_ddot = m_p * g * r_cm * sin(alpha) + friction_torque
    # Solved for alpha_ddot:
    alpha_ddot = (m_p * g * r_cm * np.sin(alpha) + friction_torque) / I_eff

    # Return the state derivative vector
    return np.array([alpha_dot, alpha_ddot])


# Main program: simulation and visualization
if __name__ == "__main__":
    # --- 1) Define system parameters ---
    # Physical parameters of the pendulum are stored in a dictionary.
    params = {
        "m_p": 0.14,         # Pendulum mass in kg
        "r_cm": 0.071,       # Effective center of mass distance in m (71 mm)
        "J_p": 0.00085,      # Moment of inertia in kg·m²
        "g": 9.81,           # Gravitational acceleration in m/s²
        "B_p": 0.0000085,    # Viscous damping coefficient
        "mu": 0.000592,      # Coulomb friction in Nm
    }

    # --- 2) Set initial conditions ---
    # Define the initial angle of the pendulum (here 120 degrees converted to radians)
    alpha_0_deg = 120.0                        # Initial angle in degrees
    alpha_0_rad = np.deg2rad(alpha_0_deg)      # Convert degrees to radians
    x0 = np.array([alpha_0_rad, 0.0])          # Initial state: [angle, angular velocity]

    # --- 3) Define time range for simulation ---
    t_start = 0.0     # Start time in seconds
    t_end = 35        # End time in seconds
    # Create a time vector with 35,000 equally spaced points between t_start and t_end
    t_eval = np.linspace(t_start, t_end, 35000)

    # --- 4) Numerical simulation of the pendulum using solve_ivp ---
    # solve_ivp integrates the system of differential equations over the defined time range.
    sol = solve_ivp(
        fun=lambda t, y: dstate_dt_pendulum(t, y, params),  # Dynamics function including parameters
        t_span=(t_start, t_end),  # Time interval for the simulation
        y0=x0,                    # Initial conditions
        t_eval=t_eval,            # Time points at which the solution is evaluated
    )

    # --- 5) Extract results ---
    # Extract angles (alpha) and angular velocities (alpha_dot) from the solution.
    alpha_sol = sol.y[0, :]       # Angle over time
    alpha_dot_sol = sol.y[1, :]   # Angular velocity over time (not plotted here)

    # --- 6) Visualization using Matplotlib ---
    plt.figure(figsize=(6, 4))             # Create a new figure with specified size
    plt.title("Pendulum Sim: Computed Moment of Inertia")  # Plot title
    plt.plot(sol.t, alpha_sol, label=r"$\alpha$ [rad]")    # Plot alpha as a function of time
    plt.xlabel("Time t [s]")                # X-axis label (time)
    plt.ylabel("Angle (radians)")           # Y-axis label (angle in radians)
    plt.grid(True)                          # Show grid for better readability
    plt.legend(loc="best")                  # Display legend in the best location
    plt.tight_layout()                      # Optimize layout
    plt.show()                              # Display the plot
