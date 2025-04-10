import os
import matplotlib.pyplot as plt
import matplotlib
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from rotary_pendulum_env import RotaryPendulumEnv

# This code was created using the Stable Baselines3 documentation:
# https://stable-baselines3.readthedocs.io/en/master/
#
# Use the TkAgg backend for Matplotlib (especially for tkinter-based environments)
matplotlib.use('TkAgg')

# Define the path for saving/loading the PPO model
model_path = "agent.zip"

# Create the environment and wrap it in a DummyVecEnv,
# which is used for handling parallel environments (here: just one)
train_env = DummyVecEnv([lambda: RotaryPendulumEnv()])

# Check if a previously trained model exists
if os.path.exists(model_path):
    print("Saved model found. Loading model...")
    model = PPO.load(
        model_path,
        env=train_env,
        custom_objects={
            "clip_range": 0.2,
            "learning_rate": 0.0003
        }
    )
else:
    print("No saved model found. Creating new agent...")
    # Create a PPO agent with fixed learning rate and clip range
    model = PPO(
        "MlpPolicy",       # Use a Multi-Layer Perceptron policy
        train_env,         # Training environment
        learning_rate=0.0003,  # Constant learning rate
        clip_range=0.2,        # Constant clip range
        verbose=1
    )

# Start training the PPO agent
print("Starting training...")
model.learn(total_timesteps=10000)  # Examples: 2160000 = 30 hrs, 21600000 = 300 hrs
print("Training completed.")

# Save the trained model to the defined path
print(f"Saving model to '{model_path}'...")
model.save(model_path)

# Test (evaluation) of the trained model
print("Testing the trained model...")
# Create a new test environment, also wrapped in DummyVecEnv
test_env = DummyVecEnv([lambda: RotaryPendulumEnv()])

# These lists will store the angle values (theta and alpha) for plotting
theta_values = []
alpha_values = []

# Reset the test environment to start a new episode
obs = test_env.reset()

# Run 150 steps in the test environment
for _ in range(150):
    # Predict the action using the model (deterministic)
    action, _states = model.predict(obs, deterministic=True)
    # Execute a step in the environment
    obs, reward, done, info = test_env.step(action)

    # Extract the angles (theta, alpha) from the observation
    theta, _, alpha, _ = obs[0]
    theta_values.append(theta)
    alpha_values.append(alpha)

    if done:
        print("Test episode finished.")
        break

# Plot the recorded angle values
print("Plotting results...")
plt.figure(figsize=(10, 5))

# First subplot: motor arm angle (theta)
plt.subplot(2, 1, 1)
plt.plot(theta_values, label="Theta (Arm Angle)")
plt.ylabel("Theta [rad]")
plt.legend()

# Second subplot: pendulum angle (alpha)
plt.subplot(2, 1, 2)
plt.plot(alpha_values, label="Alpha (Pendulum Angle)")
plt.ylabel("Alpha [rad]")
plt.xlabel("Timesteps")
plt.legend()

plt.tight_layout()
plt.show()
print("Results plotted.")
