import numpy as np
import os

# -----------------------------------------------------------------------------
# List of files containing the weight and bias data of the policy.
# Each file contains an array that should be converted to a C-compatible format.
files = [
    "mlp_extractor_policy_net_0_weight.txt",
    "mlp_extractor_policy_net_0_bias.txt",
    "mlp_extractor_policy_net_2_weight.txt",
    "mlp_extractor_policy_net_2_bias.txt",
    "action_net_weight.txt",
    "action_net_bias.txt"
]

# -----------------------------------------------------------------------------
# Ensure that the target folder "c_arrays" exists.
# If the folder doesn't exist, it will be created automatically.
output_dir = "c_arrays"
os.makedirs(output_dir, exist_ok=True)

# -----------------------------------------------------------------------------
# For each file in the list:
# 1. Load the array from the file (assuming values are comma-separated).
# 2. Determine the shape of the array.
# 3. Generate a valid C variable name by replacing dots with underscores.
# 4. Generate the C code for the static array definition (distinguishing between 1D and 2D).
# 5. Save the generated C code to a new .c file in the target folder.
for file in files:
    # Load the array from the text file; delimiter="," assumes the values are comma-separated.
    array = np.loadtxt(file, delimiter=",")
    shape = array.shape  # Determine the shape of the array

    # Generate a C-compatible variable name:
    # Remove the ".txt" extension and replace dots with underscores.
    array_name = file.replace(".txt", "").replace(".", "_")

    # Create the C code based on the dimensionality of the array:
    if array.ndim == 2:
        # For 2D arrays:
        # Create a definition in the format:
        # static const float <array_name>[M][N] = {
        #     {val1, val2, ...},
        #     {val1, val2, ...},
        #     ...
        # };
        c_code = f"static const float {array_name}[{shape[0]}][{shape[1]}] = {{\n"
        # Generate one line in the C code for each row of the array.
        c_code += ",\n".join(
            "    {" + ", ".join(f"{val:.7f}" for val in row) + "}"
            for row in array
        )
        c_code += "\n};\n"
    else:
        # For 1D arrays:
        # Create a definition in the format:
        # static const float <array_name>[N] = { val1, val2, ... };
        c_code = f"static const float {array_name}[{shape[0]}] = {{\n"
        c_code += "    " + ", ".join(f"{val:.7f}" for val in array) + "\n};\n"

    # Determine the output file path using the array name as the filename.
    output_file = os.path.join(output_dir, f"{array_name}.c")
    # Open the file in write mode and write the generated C code into it.
    with open(output_file, "w") as f:
        # Write a comment in the file indicating the source file and the array shape.
        f.write(f"// File: {file}\n")
        f.write(f"// Shape: {shape}\n")
        f.write(c_code)

    # Inform about the successful save operation.
    print(f"C array saved: {output_file}")
