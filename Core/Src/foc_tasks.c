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

}
