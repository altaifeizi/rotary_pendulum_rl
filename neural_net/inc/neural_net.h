#ifndef NEURAL_NET_H
#define NEURAL_NET_H

#include <stddef.h>

// Fully-Connected-Layer
void fc_layer(const float *input, const float *weights, const float *bias, float *output, size_t input_size, size_t output_size);

// Activation function: Tanh
void apply_tanh(float *data, size_t size);

#endif // NEURAL_NET_H

