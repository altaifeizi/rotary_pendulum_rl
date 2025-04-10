# This code was created using the Gymnasium documentation:
# https://gymnasium.farama.org/
#

import gymnasium as gym
import numpy as np


def sign_smooth(x, eps=1e-4):
    """
    Smooth approximation of the sign function.
    Avoids sharp transitions around x=0.

    Parameters:
      x   : Input value or array.
      eps : Small positive value to prevent division by zero.

    Returns:
      Smoothly approximated sign value of x.
    """
    return x / np.sqrt(x ** 2 + eps ** 2)


def normalize_angle(angle):
    """
    Normalize angle to the range [-pi, pi].

    Parameter:
      angle : Angle in radians.

    Returns:
      The normalized angle within [-pi, pi].
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi


def normalize_angle_2(angle):
    """
    Normalize angle to the range [-2pi, 2pi].

    Parameter:
      angle : Angle in radians.

    Returns:
      The normalized angle within [-2pi, 2pi].
    """
    return (angle + 2 * np.pi) % (4 * np.pi) - 2 * np.pi


class RotaryPendulumEnv(gym.Env):
    """
    Gymnasium environment for a rotary pendulum.

    This environment simulates a system consisting of a motor arm
    and a pendulum attached to it. It considers adaptive friction,
    variable inertia, and realistic physical parameters.

    The implementation is based on the Gymnasium documentation:
    https://gymnasium.farama.org/
    """

    def __init__(self):
        super(RotaryPendulumEnv, self).__init__()

        # -------------------------------
        # Pendulum and arm parameters
        # -------------------------------
        self.m_p = 0.14
        self.L_p = 0.135
        self.r_cm = 0.071
        self.L_r = 0.156
        self.J_r = 0.0026
        self.J_r_active = 0.0007
        self.J_p = 0.00031
        self.g = 9.81

        # Pendulum joint friction
        self.B_p = 0.0000085
        self.mu_p = 0.000471

        # Motor/arm friction (adaptive)
        self.B_r_active = 0.0000085
        self.mu_r_active = 0.0
        self.B_r_min = 0.0000085
        self.B_r_max = 0.0105
        self.mu_r_min = 0.0
        self.mu_r_max = 0.025
        self.omega_c = 4.1
        self.n_damp = 2

        # -------------------------------
        # Motor parameters
        # -------------------------------
        self.K_g = 20.0
        self.eta_g = 0.63
        self.eta_m = 0.61
        self.k_t = 0.0116
        self.k_m = 0.0116
        self.R_m = 4.29

        # -------------------------------
        # Time and RL settings
        # -------------------------------
        self.dt = 0.05
        self.max_steps = 150
        self.current_step = 0

        # -------------------------------
        # Observation and action space
        # -------------------------------
        self.observation_space = gym.spaces.Box(
            low=np.array([-np.pi, -np.inf, -np.pi, -np.inf], dtype=np.float32),
            high=np.array([np.pi, np.inf, np.pi, np.inf], dtype=np.float32),
            dtype=np.float32
        )
        self.action_space = gym.spaces.Discrete(3)

        self.state = None
        self.prev_action = 1  # start with "0 V" (index 1)

    def reset(self, **kwargs):
        """
        Resets the state: [theta, theta_dot, alpha, alpha_dot].
        Normalizes angles to keep them within [-pi, pi].

        Returns:
          Tuple (state, info) as per Gymnasium standard.
        """
        raw_alpha = np.pi + np.random.uniform(-0.1, 0.1)
        alpha = normalize_angle(raw_alpha)
        self.state = np.array([
            0.0, 0.0,
            alpha, 0.0
        ], dtype=np.float32)
        self.current_step = 0
        self.prev_action = 1
        return self.state, {}

    def motor_torque(self, voltage, theta_dot):
        """
        Computes motor torque at the arm (post-gearbox).

        Formula:
          tau_out = η_g * K_g * η_m * k_t * ((V_in - K_g * k_m * theta_dot) / R_m)

        Parameters:
          voltage   : Input voltage (V).
          theta_dot : Angular velocity of the arm (rad/s).

        Returns:
          Computed motor torque (Nm).
        """
        return (self.eta_g * self.K_g * self.eta_m * self.k_t *
                ((voltage - (self.K_g * self.k_m * theta_dot)) / self.R_m))

    def equations(self, state, tau, voltage):
        """
        Computes state derivatives [theta_dot, theta_ddot, alpha_dot, alpha_ddot]
        considering:
          - Real center of mass (r_cm instead of L_p/2)
          - Viscous + Coulomb friction in pendulum
          - Adaptive friction in the motor arm
          - Variable arm inertia (J_r_active if voltage ≠ 0, else J_r)

        Parameters:
          state   : Current state [theta, theta_dot, alpha, alpha_dot].
          tau     : Current motor torque.
          voltage : Current input voltage.

        Returns:
          Derivatives as numpy array.
        """
        theta, theta_dot, alpha, alpha_dot = state

        if abs(voltage) > 0:
            effective_B_r = self.B_r_active
            effective_mu_r = self.mu_r_active
            effective_J_r = self.J_r_active
        else:
            omega = abs(theta_dot)
            effective_B_r = self.B_r_min + (self.B_r_max - self.B_r_min) / (1 + (omega / self.omega_c) ** self.n_damp)
            effective_mu_r = self.mu_r_min + (self.mu_r_max - self.mu_r_min) / (1 + (omega / self.omega_c) ** self.n_damp)
            effective_J_r = self.J_r

        A11 = (self.m_p * self.L_r ** 2 +
               self.m_p * self.r_cm ** 2 * np.cos(alpha) ** 2 +
               effective_J_r)
        A12 = -self.m_p * self.r_cm * self.L_r * np.cos(alpha)
        A21 = A12
        A22 = self.J_p + self.m_p * self.r_cm ** 2
        M = np.array([[A11, A12],
                      [A21, A22]], dtype=np.float32)

        RHS1 = (tau
                - effective_B_r * theta_dot
                - effective_mu_r * sign_smooth(theta_dot)
                - 2.0 * self.m_p * self.r_cm ** 2 * np.sin(alpha) * np.cos(alpha) * theta_dot * alpha_dot
                - self.m_p * self.r_cm * self.L_r * np.sin(alpha) * alpha_dot ** 2)
        RHS2 = (- self.B_p * alpha_dot
                - self.mu_p * sign_smooth(alpha_dot)
                + self.m_p * self.r_cm ** 2 * np.sin(alpha) * np.cos(alpha) * theta_dot ** 2
                + self.m_p * self.g * self.r_cm * np.sin(alpha))
        RHS = np.array([RHS1, RHS2], dtype=np.float32)

        acc = np.linalg.solve(M, RHS)
        return np.array([theta_dot, acc[0], alpha_dot, acc[1]], dtype=np.float32)

    def rk4_step(self, state, tau, voltage):
        """
        Performs one Runge-Kutta 4th order integration step.

        Parameters:
          state   : Current state.
          tau     : Motor torque.
          voltage : Input voltage.

        Returns:
          New state after time step.
        """
        dt = self.dt

        def f(s):
            return self.equations(s, tau, voltage)

        k1 = f(state)
        k2 = f(state + 0.5 * dt * k1)
        k3 = f(state + 0.5 * dt * k2)
        k4 = f(state + dt * k3)

        return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def step(self, action):
        """
        Executes a single step:
          - Uses the delayed previous action.
          - Maps discrete action to voltage.
          - Computes motor torque.
          - Integrates using RK4.
          - Normalizes angles to avoid overflow.
          - Computes reward.

        Returns:
          Tuple (observation, reward, terminated, truncated, info).
        """
        delayed_action = self.prev_action
        self.prev_action = action

        voltage_values = [-8.0, 0.0, 8.0]
        voltage = voltage_values[delayed_action]

        theta_dot = self.state[1]
        tau = self.motor_torque(voltage, theta_dot)

        self.state = self.rk4_step(self.state, tau, voltage)

        self.state[0] = normalize_angle_2(self.state[0])
        self.state[2] = normalize_angle(self.state[2])

        obs = self.state.copy()

        _, theta_dot, alpha, alpha_dot = self.state
        reward = -abs(alpha) * 2

        if abs(self.state[0]) > np.pi:
            reward -= 150
        if -0.2 < abs(alpha) < 0.2:
            reward += 1
        if -0.3 < self.state[0] < 0.3:
            reward += 0.5

        self.current_step += 1
        terminated = False
        truncated = (self.current_step >= self.max_steps)

        return obs, reward, terminated, truncated, {}
