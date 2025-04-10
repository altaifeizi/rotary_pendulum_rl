import numpy as np
import matplotlib

# Set the Matplotlib backend to "TkAgg". This helps avoid issues in development environments like PyCharm.
matplotlib.use("TkAgg")  # For PyCharm issues
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp


def motor_voltage(t):
    """
    Defines a piecewise constant input voltage for the motor.

    This function returns:
      - 8.0 V if 0 <= t < 0.5 s
      - 0.0 V if t >= 0.5 s

    Parameter:
      t : Current time in seconds.

    Returns:
      The input voltage V_in as a float.
    """
    return 8.0 if t < 0.5 else 0.0


def motor_torque(t, theta_dot, params):
    """
    Computes the motor torque applied to the drivetrain (motor arm side).

    The torque is computed based on a DC motor model including the gearbox.

    Parameters:
      t         : Current time in seconds.
      theta_dot : Angular velocity of the motor arm (in rad/s).
      params    : Dictionary with motor and gearbox parameters:
                  - "K_g"   : Gear ratio.
                  - "eta_g" : Gearbox efficiency.
                  - "eta_m" : Motor efficiency.
                  - "k_t"   : Motor torque constant [N·m/A].
                  - "k_m"   : Back-EMF constant [V·s/rad].
                  - "R_m"   : Motor resistance [Ohms].

    Returns:
      The resulting torque tau_out in Nm.
    """
    V_in = motor_voltage(t)        # Determine the current input voltage
    K_g = params["K_g"]            # Gear ratio
    eta_g = params["eta_g"]        # Gearbox efficiency
    eta_m = params["eta_m"]        # Motor efficiency
    k_t = params["k_t"]            # Motor torque constant
    k_m = params["k_m"]            # Back-EMF constant
    R_m = params["R_m"]            # Motor resistance

    # Compute output torque:
    # Formula: tau_out = eta_g * K_g * eta_m * k_t * ((V_in - (K_g * k_m * theta_dot)) / R_m)
    # The term (K_g * k_m * theta_dot) represents the voltage drop due to back-EMF.
    tau_out = eta_g * K_g * eta_m * k_t * ((V_in - (K_g * k_m * theta_dot)) / R_m)
    return tau_out


def sign_smooth(x, eps=1e-4):
    """
    Smooth approximation of the sign function.

    For large |x|, it matches the classical sign(x), but near x=0
    it provides a smooth transition using x / sqrt(x^2 + eps^2),
    avoiding discontinuities.

    Parameters:
      x   : Input value or array.
      eps : Small positive value to prevent division by zero (default: 1e-4).

    Returns:
      Smoothly approximated sign value.
    """
    return x / np.sqrt(x ** 2 + eps ** 2)


def dstate_dt_motor(t, x, params):
    """
    Dynamics function of the motor arm.

    This function describes the differential equation system of the motor arm,
    considering angle theta and angular velocity theta_dot.

    State x:
      [theta, theta_dot]

    Dynamics equation:
      J_r * ddot_theta = tau(t, theta_dot) - effective_B_r * theta_dot - effective_mu * sign_smooth(theta_dot)

    Special behavior:
      - If the motor is active (i.e. motor_voltage(t) != 0):
            fixed minimum damping values are used,
            and a reduced moment of inertia (J_r_active) is considered.
      - If the motor is inactive (motor_voltage(t) == 0):
            damping parameters are calculated adaptively as functions of the absolute
            angular velocity (|theta_dot|). At high speed, the damping approaches the minimum,
            and at low speed the maximum, for both effective_B_r and effective_mu.

    Parameters:
      t      : Current time in seconds.
      x      : Current state vector [theta, theta_dot].
      params : Dictionary with parameters for the motor arm and damping.

    Returns:
      Derivatives [theta_dot, ddot_theta] as a NumPy array.
    """
    theta, theta_dot = x                        # Unpack state: angle and angular velocity
    tau = motor_torque(t, theta_dot, params)    # Compute motor torque
    V = motor_voltage(t)                        # Determine if motor is active (voltage != 0)
    omega = abs(theta_dot)                      # Use absolute value for adaptive calculations

    if V != 0:
        # Motor active:
        # Use fixed minimum damping values and reduced inertia.
        effective_B_r = 0.0000085
        effective_mu = 0.0
        effective_J_r = params["J_r_active"]
    else:
        # Motor inactive: adaptive damping
        B_r_min = params["B_r_min"]     # Minimum viscous damping coefficient (high speed)
        B_r_max = params["B_r_max"]     # Maximum viscous damping coefficient (zero speed)
        mu_min = params["mu_min"]       # Minimum Coulomb damping
        mu_max = params["mu_max"]       # Maximum Coulomb damping (zero speed)
        omega_c = params["omega_c"]     # Characteristic angular velocity
        n_damp = params["n_damp"]       # Exponent controlling transition steepness

        # Compute adaptive viscous damping coefficient:
        effective_B_r = B_r_min + (B_r_max - B_r_min) / (1 + (omega / omega_c) ** n_damp)
        # Compute adaptive Coulomb damping:
        effective_mu = mu_min + (mu_max - mu_min) / (1 + (omega / omega_c) ** n_damp)
        # Use standard inertia in inactive state
        effective_J_r = params["J_r"]

    # Compute angular acceleration (ddot_theta) using rotational dynamics:
    ddot_theta = (tau - effective_B_r * theta_dot - effective_mu * sign_smooth(theta_dot)) / effective_J_r
    return np.array([theta_dot, ddot_theta])


if __name__ == "__main__":
    # --- Define parameters for the motor arm ---
    # These parameters include physical properties of the arm as well as
    # damping and motor-gearbox characteristics.
    params = {
        "J_r": 0.0026,        # Moment of inertia of the motor arm when inactive [kg·m²]
        "J_r_active": 0.0019, # Reduced moment of inertia when motor is active

        # Fixed values used in active state (implicitly applied)
        "B_r_active": 0.0000085,  # Viscous damping coefficient (active) [Nm/(rad/s)]
        "mu_r_active": 0.0,       # Coulomb damping in active state

        # Adaptive damping parameters for inactive state:
        "B_r_min": 0.0000085,     # Minimum viscous damping coefficient (high speed)
        "B_r_max": 0.0105,        # Maximum viscous damping coefficient (zero speed)
        "mu_min": 0.0,            # Minimum Coulomb damping
        "mu_max": 0.02,           # Maximum Coulomb damping (slightly reduced)
        "omega_c": 8.5,           # Characteristic angular velocity in rad/s
        "n_damp": 2,              # Exponent controlling damping curve transition

        # Motor-gearbox parameters:
        "K_g": 20.0,              # Gear ratio (e.g. 1:20)
        "eta_g": 0.64,            # Gearbox efficiency
        "eta_m": 0.61,            # Motor efficiency
        "k_t": 0.0116,            # Torque constant [N·m/A]
        "k_m": 0.0116,            # Back-EMF constant [V·s/rad]
        "R_m": 4.29               # Motor resistance [Ohm]
    }

    # --- Set initial conditions ---
    # The motor arm starts at rest:
    # theta = 0 and theta_dot = 0
    x0 = np.array([0.0, 0.0])

    # --- Define simulation time range ---
    t_start = 0.0    # Start time in seconds
    t_end = 1.4      # End time in seconds (e.g. simulate for 1.4 s)
    t_eval = np.linspace(t_start, t_end, 1400)  # Create 1400 evenly spaced time points

    # --- Numerical integration of the differential equation ---
    sol = solve_ivp(
        fun=lambda t, x: dstate_dt_motor(t, x, params),  # Pass dynamics function with parameters
        t_span=(t_start, t_end),  # Time interval for simulation
        y0=x0,                    # Initial conditions: [theta, theta_dot]
        t_eval=t_eval             # Time points for evaluating the solution
    )

    # Extract theta (angle) from the integration results.
    theta_sol = sol.y[0, :]

    # --- Visualization ---
    plt.figure(figsize=(8, 4))                      # Create new figure with specified size
    plt.title("Motor Arm Sim: Rotation")            # Plot title
    plt.plot(sol.t, theta_sol, label=r"$\theta$ [rad] (Motor Arm)")  # Plot theta over time
    plt.xlabel("Time t [s]")                        # X-axis label
    plt.ylabel("Angle (radians)")                   # Y-axis label
    plt.grid(True)                                  # Show grid for better readability
    plt.legend(loc="best")                          # Display legend at optimal position
    plt.tight_layout()                              # Optimize layout to avoid overlaps
    plt.show()                                      # Show the plot
