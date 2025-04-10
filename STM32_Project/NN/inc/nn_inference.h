#ifndef NN_INFERENCE_H
#define NN_INFERENCE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Executes a forward pass (inference) on the neural network.
 * @param mot        Motor angle
 * @param motSpeed   Motor angular velocity
 * @param pend       Pendulum angle
 * @param pendSpeed  Pendulum angular velocity
 * @return           The first element of the action output vector
 */
float NN_Inference_GetFirstOutput(float mot, float motSpeed, float pend, float pendSpeed);
int NN_Inference_GetArgMax(float mot, float motSpeed, float pend, float pendSpeed);

#ifdef __cplusplus
}
#endif

#endif // NN_INFERENCE_H
