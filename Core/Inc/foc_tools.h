/*
 * foc_tools.h
 *
 *  Various utilities for the control algorithm.
 *
 *  Created on: May 25, 2024
 *      Author: lunakiller
 *
 */

#ifndef __FOC_TOOLS_H
#define __FOC_TOOLS_H

#include "foc_motorcontrol.h"


void FOC_PID_Init(PID_t* pid, float Kp, float Ki, float Kd, int32_t limit);
float FOC_PID(PID_t* pid, float error);

int32_t FOC_LinearRamp(int32_t start, int32_t final, int32_t duration, float elapsed);

int32_t FOC_LeakyIntegrator_int(LPF_int_t* lpf, int input);
float FOC_LeakyIntegrator_f32(LPF_f32_t* lpf, float input);


#endif /* __FOC_TOOLS_H */
