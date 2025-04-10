#include <stddef.h>
#include "matrix_ops.h"

// Matrix multiplication: C = A x B
void matmul(const float *A, const float *B, float *C, size_t M, size_t K, size_t N) {
    // M: Number of rows in A
    // K: Number of columns in A (and rows in B)
    // N: Number of columns in B
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N; j++) {
            C[i * N + j] = 0.0f; // Initialize C[i][j]
            for (size_t k = 0; k < K; k++) {
                C[i * N + j] += A[i * K + k] * B[k * N + j];
            }
        }
    }
}

// Add bias: Z = Z + bias
void add_bias(const float *bias, float *Z, size_t M, size_t N) {
    // M: Number of rows in Z
    // N: Number of columns in Z
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N; j++) {
            Z[i * N + j] += bias[j]; // Bias is added to column j
        }
    }
}
