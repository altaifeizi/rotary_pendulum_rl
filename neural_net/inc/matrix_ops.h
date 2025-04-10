#ifndef MATRIX_OPS_H
#define MATRIX_OPS_H

#include <stddef.h>

// Declaration of the matrix-multiplication
void matmul(const float *A, const float *B, float *C, size_t M, size_t K, size_t N);
// Declaration of the Bias-Function
void add_bias(const float *bias, float *Z, size_t M, size_t N);


#endif // MATRIX_OPS_H

