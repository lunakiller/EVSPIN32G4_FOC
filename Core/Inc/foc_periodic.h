/*
 * foc_periodic.h
 *
 *  Procedures that are repeatedly executed while the motor is running.
 *
 *  Created on: Mar 16, 2024
 *      Author: lunakiller
 *
 */

#ifndef __FOC_TASKS_H
#define __FOC_TASKS_H

#include "foc_motorcontrol.h"


void FOC_HighFrequencyScheduler(void);
void FOC_SystickScheduler(void);

void FOC_ChargeBootstraps(void);
void FOC_AlignRotor(void);
void FOC_OpenLoop_StartUp(void);
void FOC_PositionSynchronization(void);
void FOC_RunTask(void);

void FOC_CurrentCompensation(void);
void FOC_EncoderProcessing(void);

void FOC_MainControl(void);
void FOC_MRAS(void);
void FOC_DQ_Limiter(void);
void FOC_SpeedControl(void);
void FOC_Modulator(float Valpha, float Vbeta, int32_t* Va, int32_t* Vb, int32_t* Vc);


#endif /* __FOC_TASKS_H */
