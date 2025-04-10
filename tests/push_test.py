import numpy as np
import matplotlib

matplotlib.use("TkAgg")  # For PyCharm issues
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp


def sign_smooth(x, eps=1e-4):
    """
    Smooth approximation of the sign function.

    For large |x| it matches the classical sign(x), but near x=0
    the formula x / sqrt(x**2 + eps**2) ensures a smooth transition and avoids discontinuities.

    Parameters:
      x   : Input value or array.
      eps : Small positive value to prevent division by zero (default: 1e-4).

    Returns:
      The smooth approximated sign value of x.
    """
    return x / np.sqrt(x ** 2 + eps ** 2)


def motor_voltage(t):
    """
    Piecewise constant motor voltage:
      - 8.0 V when 0 <= t < 0.5 s,
      - 0.0 V when t >= 0.5 s.

    Parameter:
      t : Time in seconds.

    Returns:
      The input voltage as a float.
    """
    return 8.0 if t < 0.5 else 0.0


def motor_torque(t, theta_dot, params):
    """
    Calculates the motor torque at the OUTPUT (motor arm side) based on a DC motor model including gearbox.

    The calculation uses the following formula:
      tau_out = eta_g * K_g * eta_m * k_t * ((V_in - (K_g * k_m * theta_dot)) / R_m)

    Parameters:
      t         : Current time (s).
      theta_dot : Angular velocity of the motor arm (rad/s).
      params    : Dictionary containing motor and gearbox parameters, e.g.:
                  "K_g"   : Gear ratio,
                  "eta_g" : Gearbox efficiency,
                  "eta_m" : Motor efficiency,
                  "k_t"   : Torque constant [N·m/A],
                  "k_m"   : Back-EMF constant [V·s/rad],
                  "R_m"   : Motor resistance [Ohm].

    Returns:
      The computed motor torque tau_out (Nm).
    """
    V_in = motor_voltage(t)
    K_g = params["K_g"]
    eta_g = params["eta_g"]
    eta_m = params["eta_m"]
    k_t = params["k_t"]
    k_m = params["k_m"]
    R_m = params["R_m"]
    tau_out = eta_g * K_g * eta_m * k_t * ((V_in - (K_g * k_m * theta_dot)) / R_m)
    return tau_out


def dstate_dt(t, x, params):
    """
    Computes the derivatives of the state vector for the combined system (motor arm and pendulum).

    State vector x:
      [theta, theta_dot, alpha, alpha_dot]

      - theta      : Motor arm angle (rad)
      - theta_dot  : Motor arm angular velocity (rad/s)
      - alpha      : Pendulum angle (rad)
      - alpha_dot  : Pendulum angular velocity (rad/s)

    Returns:
      dx/dt = [theta_dot, theta_ddot, alpha_dot, alpha_ddot]

    The model considers:
      - The real center of mass of the pendulum (r_cm instead of L_p/2),
      - Adaptive damping in the motor arm when no motor voltage is applied,
      - Variable inertia of the motor arm: J_r_active if V ≠ 0, else J_r.
    """
    # 1) Unpack state
    theta, theta_dot, alpha, alpha_dot = x

    # 2) Extract parameters from dictionary
    m_p = params["m_p"]
    L_p = params["L_p"]
    r_cm = params["r_cm"]
    L_r = params["L_r"]
    J_p = params["J_p"]
    J_r = params["J_r"]
    g = params["g"]
    B_p = params["B_p"]
    mu = params["mu"]

    V = motor_voltage(t)
    if V != 0:
        effective_B_r = params["B_r_active"]
        effective_mu = params["mu_r_active"]
        effective_J_r = params.get("J_r_active", 0.0019)
    else:
        omega = abs(theta_dot)
        B_r_min = params["B_r_min"]
        B_r_max = params["B_r_max"]
        mu_min = params["mu_min"]
        mu_max = params["mu_max"]
        omega_c = params["omega_c"]
        n_damp = params["n_damp"]
        effective_B_r = B_r_min + (B_r_max - B_r_min) / (1 + (omega / omega_c) ** n_damp)
        effective_mu = mu_min + (mu_max - mu_min) / (1 + (omega / omega_c) ** n_damp)
        effective_J_r = J_r

    tau = motor_torque(t, theta_dot, params)

    # 3) Build mass matrix M(α)
    A11 = m_p * (L_r ** 2) + m_p * (r_cm ** 2) * np.cos(alpha) ** 2 + effective_J_r
    A12 = - (m_p * r_cm * L_r) * np.cos(alpha)
    A21 = A12
    A22 = J_p + m_p * (r_cm ** 2)
    M = np.array([[A11, A12],
                  [A21, A22]])

    # 4) Build right-hand side (RHS)
    RHS1 = (tau
            - effective_B_r * theta_dot - effective_mu * sign_smooth(theta_dot)
            - 2.0 * m_p * (r_cm ** 2) * np.sin(alpha) * np.cos(alpha) * theta_dot * alpha_dot
            - m_p * r_cm * L_r * np.sin(alpha) * (alpha_dot ** 2))

    RHS2 = (- B_p * alpha_dot - mu * sign_smooth(alpha_dot)
            + m_p * (r_cm ** 2) * np.sin(alpha) * np.cos(alpha) * (theta_dot ** 2)
            + m_p * g * r_cm * np.sin(alpha))

    RHS = np.array([RHS1, RHS2])

    # 5) Solve for accelerations: M * [ddot_theta, ddot_alpha] = RHS
    acc = np.linalg.solve(M, RHS)
    ddot_theta = acc[0]
    ddot_alpha = acc[1]

    # 6) Return derivative vector
    return np.array([theta_dot, ddot_theta, alpha_dot, ddot_alpha])


# ======================================
# Main simulation program
# ======================================
if __name__ == "__main__":
    def tau_zero(t):
        # Helper function that always returns 0 (no external control)
        return 0.0

    # 1) Define system parameters
    params = {
        "m_p": 0.14,
        "L_p": 0.135,
        "L_r": 0.156,
        "r_cm": 0.071,
        "J_p": 0.00031,
        "J_r": 0.0026,
        "J_r_active": 0.0007,
        "g": 9.81,
        "B_p": 0.0000085,
        "mu": 0.000471,
        "B_r_active": 0.0000085,
        "mu_r_active": 0.0,
        "B_r_min": 0.0000085,
        "B_r_max": 0.0105,
        "mu_min": 0.0,
        "mu_max": 0.025,
        "omega_c": 4.1,
        "n_damp": 2,
        "K_g": 20.0,
        "eta_g": 0.63,
        "eta_m": 0.61,
        "k_t": 0.0116,
        "k_m": 0.0116,
        "R_m": 4.29,
        "tau_func": tau_zero
    }

    # 2) Initial conditions
    x0 = np.array([0.0, 0.0, np.pi, 0.0])

    # 3) Define time range
    t_start = 0.0
    t_end = 4.5
    t_eval = np.linspace(t_start, t_end, 4500)

    # 4) Run simulation
    sol = solve_ivp(
        fun=lambda t, y: dstate_dt(t, y, params),
        t_span=(t_start, t_end),
        y0=x0,
        t_eval=t_eval
    )

    # 5) Extract results
    theta_sol = sol.y[0, :]
    alpha_sol = sol.y[2, :]

    # 6) Visualization
    plt.figure(figsize=(6, 4))
    plt.title("System Sim: Adjusted Parameters")
    plt.plot(sol.t, theta_sol, label=r"$\theta$ (Arm) [rad]")
    plt.plot(sol.t, alpha_sol, label=r"$\alpha$ (Pendulum) [rad]")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [rad]")
    plt.grid(True)
    plt.legend(loc="best")
    plt.tight_layout()
    plt.show()
