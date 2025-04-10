import torch
import numpy as np
from stable_baselines3 import PPO

# This code was created with the help of the Stable Baselines3 documentation:
# https://stable-baselines3.readthedocs.io/en/master/
#
# Load the previously trained PPO model from the specified path.
model = PPO.load("../training/agent.zip")

# Print the structure of the MLP extractor of the policy
# to get insight into the architecture of the policy.
print(model.policy.mlp_extractor)

# Extract all parameters (weights and biases) of the policy
# by accessing the state_dict() of the policy.
policy_params = model.policy.state_dict()

# Save the weights and biases of the policy in separate files.
# Only parameters of the policy are saved, not those of the value network.
for name, param in policy_params.items():
    # Skip all parameters that belong to the value network.
    if "value_net" not in name:
        # Convert the Torch tensor to a NumPy array.
        np_array = param.detach().numpy()

        # If it is a weight matrix (and not a bias vector),
        # transpose the array to match the expected format.
        if "weight" in name:
            np_array = np_array.T

        # Replace dots in the parameter name with underscores and append the ".txt" file extension.
        filename = name.replace(".", "_") + ".txt"
        # Save the array as a comma-separated text file.
        np.savetxt(filename, np_array, delimiter=",")
        print(f"Saved: {filename}")
