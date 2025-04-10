import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib
from mpl_toolkits.mplot3d import Axes3D
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from rotary_pendulum_env import RotaryPendulumEnv

# This code was created using the Stable Baselines3 documentation:
# https://stable-baselines3.readthedocs.io/en/master/
#
# Set the TkAgg backend for Matplotlib (especially for tkinter-based environments)
matplotlib.use('TkAgg')

# -------------------------------------------------------------------
# Define path to the trained PPO model file
model_path = "agent.zip"

# Check if the saved model exists. If not, raise an error.
if not os.path.exists(model_path):
    raise FileNotFoundError(f"The model '{model_path}' was not found. Please train the model first.")

print("Loading trained model...")

# Create a test environment wrapped in a DummyVecEnv.
# DummyVecEnv is a wrapper from Stable Baselines3 that handles a list of environments.
test_env = DummyVecEnv([lambda: RotaryPendulumEnv()])

# Load the saved PPO model and assign it to the test environment.
# custom_objects allows passing default values in case the saved model has dynamic schedules.
model = PPO.load(model_path, env=test_env, custom_objects={
    "clip_range": lambda _: 0.2,    # Default clip range
    "lr_schedule": lambda _: 0.0003  # Default learning rate
})

# -------------------------------------------------------------------
# Initialize lists to store the states (angles) and actions recorded during the test episode.
theta_values = []
alpha_values = []
action_values = []

print("Starting test episode...")

# Reset the test environment to start a new episode.
obs = test_env.reset()

# -------------------------------------------------------------------
# Simulation settings:
arm_length = 0.156       # Length of the rotating arm (m)
pendulum_length = 0.135  # Length of the pendulum (m)
steps = 150              # Maximum number of timesteps for the test episode

# -------------------------------------------------------------------
# Prepare 3D animation:
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

# Axis settings: Set the limits and labels for the 3D plot.
ax.set_xlim([-0.4, 0.4])
ax.set_ylim([-0.4, 0.4])
ax.set_zlim([-0.4, 0.4])
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("3D Animation of Rotary Pendulum (Alpha mirrored horizontally)")

# Define line objects for the arm and pendulum that will be updated in the animation.
arm_line, = ax.plot([], [], [], lw=3, label="Arm", color="blue")
pendulum_line, = ax.plot([], [], [], lw=3, label="Pendulum", color="orange")

# -------------------------------------------------------------------
# Update function for the animation:
def update(frame):
    global obs
    # Use the trained model to predict an action based on the current observation.
    action, _states = model.predict(obs, deterministic=True)
    # Take a step in the test environment and receive the new observation.
    obs, reward, done, info = test_env.step(action)

    # Extract angles from the observation:
    # theta: angle of the arm, alpha: angle of the pendulum.
    theta, _, alpha, _ = obs[0]
    theta_values.append(theta)
    alpha_values.append(alpha)
    # Store the action taken (use the first value of the action array).
    action_values.append(action[0])

    # Convert alpha for x-coordinate: mirrored horizontally.
    alpha_x = -alpha
    alpha_y = alpha  # vertical remains the same

    # Compute the position of the arm’s end in the xy-plane.
    x1 = arm_length * np.cos(theta)
    y1 = arm_length * np.sin(theta)
    z1 = 0  # The arm rotates in the xy-plane, so z=0.

    # Compute the position of the pendulum, based on the arm position and pendulum angle.
    x2 = x1 + pendulum_length * np.sin(alpha_x)
    y2 = y1  # y-coordinate remains unchanged.
    z2 = pendulum_length * np.cos(alpha_y)

    # Update the line objects in the animation:
    # Arm line: from origin (0,0,0) to the arm end point (x1, y1, z1).
    arm_line.set_data([0, x1], [0, y1])
    arm_line.set_3d_properties([0, z1])
    # Pendulum line: from arm tip (x1, y1, z1) to pendulum tip (x2, y2, z2).
    pendulum_line.set_data([x1, x2], [y1, y2])
    pendulum_line.set_3d_properties([z1, z2])

    return arm_line, pendulum_line

# Start the animation:
# FuncAnimation calls the update function for each frame (timestep) of the animation.
ani = FuncAnimation(fig, update, frames=steps, blit=False, interval=50, repeat=False)

plt.legend()
plt.show()

# -------------------------------------------------------------------
# After the animation: Plot the recorded angle trajectories and actions.
print("Plotting results...")
plt.figure(figsize=(10, 8))
plt.suptitle("Angle and Action Trajectories Agent 2", fontsize=16)

# Define the time interval between steps (50 ms per timestep)
dt = 0.05  # in seconds
# Create time axis with the same length as the collected data
time_axis = np.arange(len(theta_values)) * dt

# Plot for Theta (arm angle)
plt.subplot(3, 1, 1)
plt.plot(time_axis, theta_values, label="Theta (Arm Angle)")
plt.ylabel("Theta [rad]")
plt.legend()

# Plot for Alpha (pendulum angle)
plt.subplot(3, 1, 2)
plt.plot(time_axis, alpha_values, label="Alpha (Pendulum Angle)")
plt.ylabel("Alpha [rad]")
plt.legend()

# Plot for the action (agent output)
plt.subplot(3, 1, 3)
plt.plot(time_axis, action_values, label="Action (Agent Output)")
plt.ylabel("Action")
plt.xlabel("Time [s]")
plt.legend(loc="upper right")

plt.tight_layout(rect=[0, 0, 1, 0.95])
plt.show()
print("Results plotted.")
