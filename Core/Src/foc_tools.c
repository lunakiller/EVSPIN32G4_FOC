/*
 * foc_tools.c
 *
 *  Various utilities for the control algorithm.
 *
 *  Created on: May 25, 2024
 *      Author: lunakiller
 *
 */

#include "foc_tools.h"

/**
 * @brief Leaky integrator filter, 32-bit int type.
 * @param lpg Pointer to the low-pass filter handle
 * @param input Input to the filter
 * @retval Filtered value
 */
int32_t FOC_LeakyIntegrator_int(LPF_int_t* lpf, int input) {
  int out = lpf->alpha * lpf->last_out + (1 - lpf->alpha) * input;
//  int out = lpf->last_out + (1 - lpf->alpha) * (input - lpf->last_out);   // TODO test!
  lpf->last_out = out;
  return out;
}

/**
 * @brief Leaky integrator filter, float type.
 * @param lpg Pointer to the low-pass filter handle
 * @param input Input to the filter
 * @retval Filtered value
 */
float FOC_LeakyIntegrator_f32(LPF_f32_t* lpf, float input) {
  float out = lpf->alpha * lpf->last_out + (1 - lpf->alpha) * input;
//  float out = lpf->last_out + (1 - lpf->alpha) * (input - lpf->last_out);   // TODO test!
  lpf->last_out = out;
  return out;
}

/**
 * @brief Computes a position on the ramp based on the elapsed time and slope.
 * @param start Initial ramp value
 * @param final Final ramp value
 * @oaram duration Duration of the ramp
 * @param elapsed Time elapsed from the ramp start
 * @retval Position on the ramp
 */
int32_t FOC_LinearRamp(int32_t start, int32_t final, int32_t duration, float elapsed) {
  return (((final - start) * elapsed / (float)duration) + start);
}

/**
 * @brief Initialize PID controller and its variables
 * @param pid Pointer to the PID handle
 * @param Kp Proportional term gain
 * @param Ki Integral term gain
 * @param Kd Derivative term gain
 * @param limit Maximum controller output
 */
void FOC_PID_Init(PID_t* pid, float Kp, float Ki, float Kd, int32_t limit) {
  pid->reg.Kp = Kp;
  pid->reg.Ki = Ki;
  pid->reg.Kd = Kd;
  arm_pid_init_f32(&pid->reg, 1);

  pid->max_output = limit;
  pid->saturated = false;
  pid->target = 0;
}

/**
 * @brief PID controller iteration.
 * @param pid Pointer to the PID handle
 * @param error Controller input - (target - actual)
 * @retval Contoller output
 */
float FOC_PID(PID_t* pid, float error) {
  volatile float out = arm_pid_f32(&pid->reg, error);

  // saturation
  if(out > pid->max_output) {
    // limit output
    out = pid->max_output;
    pid->reg.state[2] = pid->max_output;
    pid->saturated = true;
  }
  else if(out < -pid->max_output) {
    // limit output
    out = -pid->max_output;
    pid->reg.state[2] = -pid->max_output;
    pid->saturated = true;
  }
  else {
    pid->saturated = false;
  }

  return out;
}
