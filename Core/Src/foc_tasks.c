/*
 * foc_tasks.c
 *
 *  Created on: Mar 16, 2024
 *      Author: lunakiller
 *
 */

#include "foc_motorcontrol.h"
#include "arm_math.h"

// TODO pass a pointer?
extern Board_Settings_t evspin;

void FOC_Task(void) {
  // TODO compute on of the currents
  evspin.adc.currents[0] = (evspin.adc.buffers.ADC1_inj_raw[0] - evspin.offsets.phase_u_offset) * evspin.adc.vdda / 4096.0f / (CS_GAIN * CS_SHUNT_VALUE);
  evspin.adc.currents[1] = (evspin.adc.buffers.ADC2_inj_raw[0] - evspin.offsets.phase_v_offset) * evspin.adc.vdda / 4096.0f / (CS_GAIN * CS_SHUNT_VALUE);
  evspin.adc.currents[2] = (evspin.adc.buffers.ADC1_inj_raw[1] - evspin.offsets.phase_w_offset) * evspin.adc.vdda / 4096.0f / (CS_GAIN * CS_SHUNT_VALUE);

  if(evspin.adc.currents[0] > SW_OVERCURRENT_THRESHOLD || evspin.adc.currents[1] > SW_OVERCURRENT_THRESHOLD || evspin.adc.currents[2] > SW_OVERCURRENT_THRESHOLD) {
    LL_TIM_DisableAllOutputs(TIM1);
    // TODO error status
  }

  int32_t localCurrU = evspin.adc.currents[0];
  int32_t localCurrV = evspin.adc.currents[1];
  int32_t localCurrW = evspin.adc.currents[2];

}
