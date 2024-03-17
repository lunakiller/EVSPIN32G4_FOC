/*
 * foc_tasks.c
 *
 *  Created on: Mar 16, 2024
 *      Author: lunakiller
 *
 */

#include "foc_motorcontrol.h"

// TODO pass a pointer?
extern Board_Settings_t evspin;



/**
 * @brief Sinusoidal-PWM modulator function.
 * @param Valpha Alpha voltage in alpha-beta coordinates.
 * @param Vbeta Beta voltage in alpha-beta coordinates.
 * @param Va CCRx register value representing phase A voltage.
 * @param Vb CCRx register value representing phase B voltage.
 * @param Vc CCRx register value representing phase C voltage.
 */
void FOC_Modulator(float Valpha, float Vbeta, uint32_t* Va, uint32_t* Vb, uint32_t* Vc) {
  float tmpA, tmpB, tmpC;
  arm_inv_clarke_f32(Valpha, Vbeta, &tmpA, &tmpB);
  tmpC = -(tmpA + tmpB);

  // TODO deadtime compensation?
  // TODO limitation

  *Va = tmpA * LL_TIM_GetAutoReload(TIM1) / (2 * evspin.adc.vbus) + (LL_TIM_GetAutoReload(TIM1) / 2);
  *Vb = tmpB * LL_TIM_GetAutoReload(TIM1) / (2 * evspin.adc.vbus) + (LL_TIM_GetAutoReload(TIM1) / 2);
  *Vc = tmpC * LL_TIM_GetAutoReload(TIM1) / (2 * evspin.adc.vbus) + (LL_TIM_GetAutoReload(TIM1) / 2);

  return;
}

/**
 * @brief Main FOC task.
 */
void FOC_Task(void) {
  // TODO compute on of the currents
  evspin.adc.currents[0] = (evspin.adc.buffers.ADC1_inj_raw[0] - evspin.offsets.phase_u_offset) * evspin.adc.vdda / 4096.0f / (CS_GAIN * CS_SHUNT_VALUE);
  evspin.adc.currents[1] = (evspin.adc.buffers.ADC2_inj_raw[0] - evspin.offsets.phase_v_offset) * evspin.adc.vdda / 4096.0f / (CS_GAIN * CS_SHUNT_VALUE);
  evspin.adc.currents[2] = (evspin.adc.buffers.ADC1_inj_raw[1] - evspin.offsets.phase_w_offset) * evspin.adc.vdda / 4096.0f / (CS_GAIN * CS_SHUNT_VALUE);

  // overcurrent
  if(evspin.adc.currents[0] > SW_OVERCURRENT_THRESHOLD || evspin.adc.currents[1] > SW_OVERCURRENT_THRESHOLD || evspin.adc.currents[2] > SW_OVERCURRENT_THRESHOLD) {
    LL_TIM_DisableAllOutputs(TIM1);
    // TODO error status
  }

  int32_t localCurrU = evspin.adc.currents[0];
  int32_t localCurrV = evspin.adc.currents[1];
  int32_t localCurrW = evspin.adc.currents[2];

  /* DRAFT */
//  // TODO compute angle
//
//  float sin, cos;
//  float angle = 0;
//  arm_sin_cos_f32(angle, &sin, &cos);
//
//  // Clarke transform
//  arm_clarke_f32(localCurrU, localCurrV, &evspin.foc.Ialpha, &evspin.foc.Ibeta);
//
//  // Park transform
//  arm_park_f32(evspin.foc.Ialpha, evspin.foc.Ibeta, &evspin.foc.Id, &evspin.foc.Iq, sin, cos);
//
//  // PID regulators
//  evspin.foc.Vd = arm_pid_f32(&evspin.foc.pid.Id_pid, evspin.foc.pid.Id_target - evspin.foc.Id);
//  evspin.foc.Vq = arm_pid_f32(&evspin.foc.pid.Iq_pid, evspin.foc.pid.Iq_target - evspin.foc.Iq);
//
//  // TODO circle limitation
//
//  // inverse Park transform
//  arm_inv_park_f32(evspin.foc.Vd, evspin.foc.Vq, &evspin.foc.Valpha, &evspin.foc.Vbeta, sin, cos);
//
//  FOC_Modulator(evspin.foc.Valpha, evspin.foc.Vbeta, &evspin.foc.phU, &evspin.foc.phV, &evspin.foc.phW);
//
//  LL_TIM_OC_SetCompareCH1(TIM1, evspin.foc.phU);
//  LL_TIM_OC_SetCompareCH2(TIM1, evspin.foc.phV);
//  LL_TIM_OC_SetCompareCH3(TIM1, evspin.foc.phW);
//  LL_TIM_GenerateEvent_UPDATE(TIM1);

}
