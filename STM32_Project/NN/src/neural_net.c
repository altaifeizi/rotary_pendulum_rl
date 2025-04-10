#include <math.h>
#include "matrix_ops.h"
#include "neural_net.h"

// Fully-connected layer
void fc_layer(const float *input, const float *weights, const float *bias, float *output, size_t input_size, size_t output_size) {
    // Matrix multiplication: output = input * weights
    matmul(input, weights, output, 1, input_size, output_size);

    // Add bias: output += bias
    add_bias(bias, output, 1, output_size);
}

// Activation function: Tanh
void apply_tanh(float *data, size_t size) {
    for (size_t i = 0; i < size; i++) {
        data[i] = tanh(data[i]);
    }
}
