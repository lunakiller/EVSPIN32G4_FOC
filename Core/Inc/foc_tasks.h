/*
 * foc_tasks.h
 *
 *  Created on: Mar 16, 2024
 *      Author: lunakiller
 *
 */

#ifndef __FOC_TASKS_H
#define __FOC_TASKS_H

#include "foc_motorcontrol.h"
#include "typedefs.h"

int32_t FOC_LeakyIntegrator_int(LPF_int_t* lpf, int input);
float FOC_LeakyIntegrator_f32(LPF_f32_t* lpf, float input);
float FOC_MovingAverage(float input);

int32_t FOC_LinearRamp(int32_t start, int32_t final, int32_t duration, float elapsed);

void FOC_PID_Init(PID_t* pid, float Kp, float Ki, float Kd, int32_t limit);
float FOC_PID(PID_t* pid, float error);
void FOC_SystickScheduler(void);
void FOC_ChargeBootstraps(void);
void FOC_AlignRotor(void);
void FOC_EncoderProcessing(void);
void FOC_SpeedControl(void);
void FOC_RunTask(void);
void FOC_MainControl(void);
void FOC_Modulator(float Valpha, float Vbeta, int32_t* Va, int32_t* Vb, int32_t* Vc);


#endif /* __FOC_TASKS_H */
