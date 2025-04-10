#include <stdio.h>
#include "matrix_ops.h"     // Contains basic matrix operations (e.g., multiplication)
#include "neural_net.h"     // Declarations for fully connected layers and activation functions

// Include the C files containing the exported weights and biases
#include "../c_arrays/mlp_extractor_policy_net_0_weight.c"
#include "../c_arrays/mlp_extractor_policy_net_0_bias.c"
#include "../c_arrays/mlp_extractor_policy_net_2_weight.c"
#include "../c_arrays/mlp_extractor_policy_net_2_bias.c"
#include "../c_arrays/action_net_weight.c"
#include "../c_arrays/action_net_bias.c"

int main() {
    // -------------------------------------------------------------------------
    // Input data:
    // The "input" array contains 4 values representing the system state.
    // Example values are given here (e.g., angles and angular velocities).
    float input[4] = {0.0, 0.0, 3.14, 0.0};

    // -------------------------------------------------------------------------
    // Buffers for intermediate results:
    // output1: result of the first fully connected layer (dimension: 64)
    // output2: result of the second fully connected layer (dimension: 64)
    // final_output: final output of the network (3 action values)
    float output1[64] = {0.0};
    float output2[64] = {0.0};
    float final_output[3] = {0.0};

    // -------------------------------------------------------------------------
    // First fully connected layer:
    // Computes: output1 = tanh( input * mlp_extractor_policy_net_0_weight + mlp_extractor_policy_net_0_bias )
    // fc_layer multiplies the input vector with the weight matrix and adds the bias.
    // Then the tanh activation function is applied.
    fc_layer(input, (float *)mlp_extractor_policy_net_0_weight, mlp_extractor_policy_net_0_bias, output1, 4, 64);
    apply_tanh(output1, 64);

    // -------------------------------------------------------------------------
    // Second fully connected layer:
    // Computes: output2 = tanh( output1 * mlp_extractor_policy_net_2_weight + mlp_extractor_policy_net_2_bias )
    fc_layer(output1, (float *)mlp_extractor_policy_net_2_weight, mlp_extractor_policy_net_2_bias, output2, 64, 64);
    apply_tanh(output2, 64);

    // -------------------------------------------------------------------------
    // Final fully connected layer (output layer):
    // Computes: final_output = output2 * action_net_weight + action_net_bias
    // No activation is applied, so raw action values are returned.
    fc_layer(output2, (float *)action_net_weight, action_net_bias, final_output, 64, 3);

    // -------------------------------------------------------------------------
    // Print the final results:
    // The computed action values are printed to the console.
    printf("Final output (Action Values):\n");
    for (size_t i = 0; i < 3; i++) {
        printf("action_value[%zu] = %f\n", i, final_output[i]);
    }

    return 0;
}
