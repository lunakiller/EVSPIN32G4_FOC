/*
 * foc_handlers.c
 *
 *  Created on: Mar 11, 2024
 *      Author: lunakiller
 *
 */

#include "foc_handlers.h"
#include <stdbool.h>

#include "swo_debug.h"


extern Board_Settings_t evspin;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

bool adc1_dma = false, adc2_dma = false;

/**
 * @brief ADC1/2 interrupt handler.
 */
void ADC1_2_IRQHandler(void) {
  // injected channels ready
  if(LL_ADC_IsActiveFlag_JEOS(hadc1.Instance) && LL_ADC_IsActiveFlag_JEOS(hadc2.Instance)) {
    // read injected channels
    evspin.adc.buffers.ADC1_inj_raw[0] = LL_ADC_INJ_ReadConversionData12(hadc1.Instance, LL_ADC_INJ_RANK_1);
    evspin.adc.buffers.ADC1_inj_raw[1] = LL_ADC_INJ_ReadConversionData12(hadc1.Instance, LL_ADC_INJ_RANK_2);
    evspin.adc.buffers.ADC2_inj_raw[0] = LL_ADC_INJ_ReadConversionData12(hadc2.Instance, LL_ADC_INJ_RANK_1);

    // clear flags
    LL_ADC_ClearFlag_JEOS(hadc1.Instance);
    LL_ADC_ClearFlag_JEOS(hadc2.Instance);

    // convert from raw ADC to current
    evspin.adc.currents[0] = (evspin.adc.offsets.phase_u_offset - evspin.adc.buffers.ADC1_inj_raw[0]) * evspin.adc.vdda / 4096.0f / (CS_GAIN * CS_SHUNT_VALUE);
    evspin.adc.currents[1] = (evspin.adc.offsets.phase_v_offset - evspin.adc.buffers.ADC2_inj_raw[0]) * evspin.adc.vdda / 4096.0f / (CS_GAIN * CS_SHUNT_VALUE);
    evspin.adc.currents[2] = (evspin.adc.offsets.phase_w_offset - evspin.adc.buffers.ADC1_inj_raw[1]) * evspin.adc.vdda / 4096.0f / (CS_GAIN * CS_SHUNT_VALUE);

    // check overcurrent
    if(evspin.adc.currents[0] > SW_OVERCURRENT_THRESHOLD || evspin.adc.currents[1] > SW_OVERCURRENT_THRESHOLD || evspin.adc.currents[2] > SW_OVERCURRENT_THRESHOLD
        || evspin.adc.currents[0] < -SW_OVERCURRENT_THRESHOLD || evspin.adc.currents[1] < -SW_OVERCURRENT_THRESHOLD || evspin.adc.currents[2] < -SW_OVERCURRENT_THRESHOLD) {
      LL_TIM_DisableAllOutputs(TIM1);
      DEBUG_print("OVERCURRENT!\r\n");
      evspin.state = STATE_ERROR;
    }

    // execute high-frequency task
    FOC_HighFrequencyScheduler();

#if DEBUG_DAC == 1
    // DEBUG DAC options
    switch(evspin.dbg.dbg_opt) {
    case 0:       // do nothing
          break;
    case 1:       // MRAS position estimate
      evspin.dbg.dac1 = evspin.mras.angle_ada_deg;
      evspin.dbg.dac2 = evspin.mras.speed_mech_ada;
      break;
    case 2:       // angle comparison MRAS vs Open-loop
      evspin.dbg.dac1 = evspin.mras.angle_ada_deg;
      evspin.dbg.dac2 = evspin.foc.angle;
      break;
    case 3:       // angle comparison MRAS vs encoder
      evspin.dbg.dac1 = evspin.mras.angle_ada_deg;
      evspin.dbg.dac2 = evspin.enc.angle;
      break;
    case 4:       // speed comparison MRAS vs encoder
      evspin.dbg.dac1 = evspin.mras.speed_mech_ada;
      evspin.dbg.dac2 = evspin.enc.speed_filtered;
      break;
    case 5:       // MRAS vs FOC q currents
      evspin.dbg.dac1 = evspin.mras.Iq_ada;
      evspin.dbg.dac2 = evspin.foc.Iq;
      break;
    case 6:       // MRAS vs FOC d currents
      evspin.dbg.dac1 = evspin.mras.Id_ada / 4.0;
      evspin.dbg.dac2 = evspin.foc.Id;
      break;
    case 10:      // FOC dq currents
      evspin.dbg.dac1 = evspin.foc.Iq;
      evspin.dbg.dac2 = evspin.foc.Id;
      break;
    case 11:      // FOC alpha-beta currents
      evspin.dbg.dac1 = evspin.foc.Ialpha;
      evspin.dbg.dac2 = evspin.foc.Ibeta;
      break;
    case 12:      // FOC ab currents
      evspin.dbg.dac1 = evspin.foc.currents_comp[0];
      evspin.dbg.dac2 = evspin.foc.currents_comp[1];
      break;
    case 13:      // FOC d current
      evspin.dbg.dac1 = evspin.foc.Id;
      evspin.dbg.dac2 = 0;
      break;
    case 14:      // FOC q current
      evspin.dbg.dac1 = evspin.foc.Iq;
      evspin.dbg.dac2 = 0;
      break;
    case 20:      // MRAS dq currents
      evspin.dbg.dac1 = evspin.mras.Iq_ada;
      evspin.dbg.dac2 = evspin.mras.Id_ada / 4.0;
      break;
    case 21:      // MRAS speed error
      evspin.dbg.dac1 = evspin.mras.speed_error;
      evspin.dbg.dac2 = evspin.mras.speed_mech_ada;
      break;
    case 30:      // FOC dq voltages
      evspin.dbg.dac1 = evspin.foc.Vq / 6.0;
      evspin.dbg.dac2 = evspin.foc.Vd / 6.0;
      break;
    case 31:      // FOC alpha-beta voltages
      evspin.dbg.dac1 = evspin.foc.Valpha/ 6.0;
      evspin.dbg.dac2 = evspin.foc.Vbeta / 6.0;
      break;
    case 40:      // ab voltages from filtered ADC
      evspin.dbg.dac1 = ((evspin.adc.buffers.ADC1_reg_raw[U_ADC1] * evspin.adc.vdda / 4096.0f * VMOT_COEFF) - (evspin.adc.vbus / 2)) / 6.0;
      evspin.dbg.dac2 = ((evspin.adc.buffers.ADC1_reg_raw[V_ADC1] * evspin.adc.vdda / 4096.0f * VMOT_COEFF) - (evspin.adc.vbus / 2)) / 6.0;
      break;
    case 41:      // FOC vs ADC alpha voltages
      float a, b;
      evspin.dbg.dac1 = ((evspin.adc.buffers.ADC1_reg_raw[U_ADC1] * evspin.adc.vdda / 4096.0f * VMOT_COEFF) - (evspin.adc.vbus / 2)) / 6.0;
      arm_inv_clarke_f32(evspin.foc.Valpha, evspin.foc.Vbeta, &a, &b);
      evspin.dbg.dac2 = a / 6.0;
      break;
    case 50:      // Q-axis current PID
      evspin.dbg.dac1 = evspin.foc.Iq_pid.target;
      evspin.dbg.dac2 = evspin.foc.Iq;
      break;
    case 51:      // D-axis current PID
      evspin.dbg.dac1 = evspin.foc.Id_pid.target;
      evspin.dbg.dac2 = evspin.foc.Id;
      break;
    default:      // reset
      evspin.dbg.dac1 = 0;
      evspin.dbg.dac2 = 0;
      break;
    }
#endif
  }
  // overvoltage/undervoltage
  else if(LL_ADC_IsActiveFlag_AWD1(hadc1.Instance)) {
    LL_TIM_DisableAllOutputs(TIM1);
    LL_ADC_ClearFlag_AWD1(hadc1.Instance);

    DEBUG_print("AWDG!\r\n");
    evspin.state = STATE_ERROR;
  }
  // other non-critical error handling
  else if(LL_ADC_IsActiveFlag_OVR(hadc1.Instance)) {
    LL_ADC_ClearFlag_OVR(hadc1.Instance);
  }
  else if(LL_ADC_IsActiveFlag_OVR(hadc2.Instance)) {
    LL_ADC_ClearFlag_OVR(hadc2.Instance);
  }
  else {
    LL_ADC_ClearFlag_JEOS(hadc1.Instance);
    LL_ADC_ClearFlag_JEOS(hadc2.Instance);
  }
}

/**
 * @brief Rotary encoder zero signal interrupt handler.
 */
void EXTI9_5_IRQHandler(void) {
	// encoder index
	if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_8)) {
	  evspin.enc.zero_angle_cnt = LL_TIM_GetCounter(TIM4);
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_8);
	}
}

/**
 * @brief TIM1 break interrupt handler. Triggered by the gate driver.
 */
void TIM1_BRK_TIM15_IRQHandler(void) {
  LL_TIM_ClearFlag_BRK(TIM1);
  DEBUG_print("BRAKE!\r\n");

  FOC_Stop();
  LL_TIM_DisableAllOutputs(TIM1);     // just in case
}

/**
 * @brief ADC1/2 conversion complete handler.
 *        Used during OPAMP offset calibration.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if(hadc->Instance == ADC1) {
    adc1_dma = true;
  }
  else {
    adc2_dma = true;
  }
}

/**
 * @brief Window watchdog EarlyWakeUp interrupt handler.
 */
void WWDG_IRQHandler(void) {
  LL_WWDG_ClearFlag_EWKUP(WWDG);
  DEBUG_print("WATCHDOG!");
}
