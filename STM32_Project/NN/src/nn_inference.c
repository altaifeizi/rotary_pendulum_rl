#include "nn_inference.h"
#include "matrix_ops.h"
#include "neural_net.h"

// Include the weight and bias arrays that were previously exported from Stable Baselines3
#include "mlp_extractor_policy_net_0_weight.c"
#include "mlp_extractor_policy_net_0_bias.c"
#include "mlp_extractor_policy_net_2_weight.c"
#include "mlp_extractor_policy_net_2_bias.c"
#include "action_net_weight.c"
#include "action_net_bias.c"

/**
 * @brief Performs a forward pass through the neural network and returns
 *        the first element of the final output vector.
 *
 * The function uses three fully-connected layers:
 *   1st layer: Input -> hidden layer (64 neurons) + tanh activation
 *   2nd layer: hidden layer -> hidden layer (64 neurons) + tanh activation
 *   3rd layer: hidden layer -> output (3 neurons) (no activation on output layer)
 *
 * @param mot        Motor value
 * @param motSpeed   Motor speed
 * @param pend       Pendulum value
 * @param pendSpeed  Pendulum speed
 * @return float     First element of the output vector (e.g. for a specific inference)
 */
float NN_Inference_GetFirstOutput(float mot, float motSpeed, float pend, float pendSpeed)
{
    // Input data (system state)
    float input[4] = {mot, motSpeed, pend, pendSpeed};

    // Arrays for intermediate results and final output:
    // output1: result of first layer (dimension: 64)
    // output2: result of second layer (dimension: 64)
    // final_output: result of third layer (dimension: 3)
    float output1[64];
    float output2[64];
    float final_output[3];

    // 1st fully-connected layer + tanh activation:
    // Computes: output1 = tanh( input * weight0 + bias0 )
    fc_layer(input, (float *)mlp_extractor_policy_net_0_weight,
                    mlp_extractor_policy_net_0_bias,
                    output1, 4, 64);
    apply_tanh(output1, 64);

    // 2nd fully-connected layer + tanh activation:
    // Computes: output2 = tanh( output1 * weight2 + bias2 )
    fc_layer(output1, (float *)mlp_extractor_policy_net_2_weight,
                     mlp_extractor_policy_net_2_bias,
                     output2, 64, 64);
    apply_tanh(output2, 64);

    // 3rd fully-connected layer (output layer, no activation):
    // Computes: final_output = output2 * action_weight + action_bias
    fc_layer(output2, (float *)action_net_weight,
                     action_net_bias,
                     final_output, 64, 3);

    // Return the first element of the final output vector
    return final_output[0];
}

/**
 * @brief Performs a forward pass through the neural network and returns
 *        the index of the maximum output value.
 *
 * The index of the highest value in the final output vector is determined
 * and mapped to a discrete action:
 *   - Index 0 corresponds to -1, index 1 to 0, index 2 to 1.
 *
 * @param mot        Motor value
 * @param motSpeed   Motor speed
 * @param pend       Pendulum value
 * @param pendSpeed  Pendulum speed
 * @return int       Discretely mapped value (-1, 0 or 1)
 */
int NN_Inference_GetArgMax(float mot, float motSpeed, float pend, float pendSpeed)
{
    // Input data (system state)
    float input[4] = {mot, motSpeed, pend, pendSpeed};

    // Arrays for intermediate results and final output:
    float output1[64];
    float output2[64];
    float final_output[3];

    // 1st fully-connected layer + tanh activation:
    fc_layer(input, (float *)mlp_extractor_policy_net_0_weight,
                    mlp_extractor_policy_net_0_bias,
                    output1, 4, 64);
    apply_tanh(output1, 64);

    // 2nd fully-connected layer + tanh activation:
    fc_layer(output1, (float *)mlp_extractor_policy_net_2_weight,
                     mlp_extractor_policy_net_2_bias,
                     output2, 64, 64);
    apply_tanh(output2, 64);

    // 3rd fully-connected layer (output layer, no activation):
    fc_layer(output2, (float *)action_net_weight,
                     action_net_bias,
                     final_output, 64, 3);

    // ---------------------------------------------------------
    // Determine the index of the maximum value in the output vector:
    int argmax = 0;
    float max_val = final_output[0];
    for (int i = 1; i < 3; i++)
    {
        if (final_output[i] > max_val)
        {
            max_val = final_output[i];
            argmax = i;
        }
    }

    // Map the index to a discrete value:
    // 0 -> -1, 1 -> 0, 2 -> 1
    static const int argmax_to_motor[3] = {-1, 0, 1};

    return argmax_to_motor[argmax];
}
