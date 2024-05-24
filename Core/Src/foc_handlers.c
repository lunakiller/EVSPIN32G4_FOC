/*
 * IRQ_handlers.c
 *
 *  Created on: Mar 11, 2024
 *      Author: lunakiller
 *
 */

#include "foc_motorcontrol.h"
#include <stdbool.h>

#include "swo_debug.h"


extern Board_Settings_t evspin;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

bool adc1_dma = false, adc2_dma = false;


void ADC1_2_IRQHandler(void) {
  if(LL_ADC_IsActiveFlag_JEOS(hadc1.Instance) && LL_ADC_IsActiveFlag_JEOS(hadc2.Instance)) {
    evspin.adc.buffers.ADC1_inj_raw[0] = LL_ADC_INJ_ReadConversionData12(hadc1.Instance, LL_ADC_INJ_RANK_1);
    evspin.adc.buffers.ADC1_inj_raw[1] = LL_ADC_INJ_ReadConversionData12(hadc1.Instance, LL_ADC_INJ_RANK_2);
    evspin.adc.buffers.ADC2_inj_raw[0] = LL_ADC_INJ_ReadConversionData12(hadc2.Instance, LL_ADC_INJ_RANK_1);

    LL_ADC_ClearFlag_JEOS(hadc1.Instance);
    LL_ADC_ClearFlag_JEOS(hadc2.Instance);

    // TODO compute one of the currents
    evspin.adc.currents[0] = (evspin.adc.offsets.phase_u_offset - evspin.adc.buffers.ADC1_inj_raw[0]) * evspin.adc.vdda / 4096.0f / (CS_GAIN * CS_SHUNT_VALUE);
    evspin.adc.currents[1] = (evspin.adc.offsets.phase_v_offset - evspin.adc.buffers.ADC2_inj_raw[0]) * evspin.adc.vdda / 4096.0f / (CS_GAIN * CS_SHUNT_VALUE);
    evspin.adc.currents[2] = (evspin.adc.offsets.phase_w_offset - evspin.adc.buffers.ADC1_inj_raw[1]) * evspin.adc.vdda / 4096.0f / (CS_GAIN * CS_SHUNT_VALUE);

    // overcurrent
    if(evspin.adc.currents[0] > SW_OVERCURRENT_THRESHOLD || evspin.adc.currents[1] > SW_OVERCURRENT_THRESHOLD || evspin.adc.currents[2] > SW_OVERCURRENT_THRESHOLD
        || evspin.adc.currents[0] < -SW_OVERCURRENT_THRESHOLD || evspin.adc.currents[1] < -SW_OVERCURRENT_THRESHOLD || evspin.adc.currents[2] < -SW_OVERCURRENT_THRESHOLD) {
      LL_TIM_DisableAllOutputs(TIM1);
      DEBUG_print("OVERCURRENT!\r\n");
      // TODO error status
      evspin.state = STATE_ERROR;
    }

    // DQ limits
    evspin.foc.limit = ((float)DQLIM_MAX_VOLTAGE / 100.0f) * evspin.adc.vbus;
    evspin.foc.limit_squared = evspin.foc.limit * evspin.foc.limit;

    // TODO state machine
    switch(evspin.state) {
    case STATE_IDLE:
      break;
    case STATE_BOOTSTRAP:
      break;
    case STATE_READY:
      FOC_CurrentCompensation();
      FOC_MainControl();
      break;
    case STATE_ALIGNMENT:
      FOC_CurrentCompensation();
      FOC_MainControl();
      break;
    case STATE_STARTUP:
      FOC_OpenLoop_StartUp();
      FOC_EncoderProcessing();
      FOC_CurrentCompensation();
      FOC_MRAS();
      FOC_MainControl();
      break;
    case STATE_SYNCHRO:
      FOC_PositionSynchronization();
      FOC_EncoderProcessing();
      FOC_CurrentCompensation();
      FOC_MRAS();
      FOC_MainControl();
      break;
    case STATE_RUN:
      FOC_EncoderProcessing();
      FOC_CurrentCompensation();
      FOC_MRAS();
      FOC_MainControl();
      break;
    default:
      break;
    }

    // TODO DEBUG DAC
    switch(evspin.dbg.dbg_opt) {
    case 0:       // MRAS position estimate
      evspin.dbg.tmp1 = evspin.mras.angle_ada_deg;
      evspin.dbg.tmp2 = evspin.mras.speed_mech_ada;
      break;
    case 1:       // angle comparison MRAS vs Open-loop
      evspin.dbg.tmp1 = evspin.mras.angle_ada_deg;
      evspin.dbg.tmp2 = evspin.foc.angle;
      break;
    case 2:       // angle comparison MRAS vs encoder
      evspin.dbg.tmp1 = evspin.mras.angle_ada_deg;
      evspin.dbg.tmp2 = evspin.enc.angle;
      break;
    case 3:       // speed comparison MRAS vs encoder
      evspin.dbg.tmp1 = evspin.mras.speed_mech_ada;
      evspin.dbg.tmp2 = evspin.enc.speed_filtered;
      break;
    case 4:       // MRAS vs FOC q currents
      evspin.dbg.tmp1 = evspin.mras.Iq_ada;
      evspin.dbg.tmp2 = evspin.foc.Iq;
      break;
    case 5:       // MRAS vs FOC d currents
      evspin.dbg.tmp1 = evspin.mras.Id_ada / 4.0;
      evspin.dbg.tmp2 = evspin.foc.Id;
      break;
    case 10:      // FOC dq currents
      evspin.dbg.tmp1 = evspin.foc.Iq;
      evspin.dbg.tmp2 = evspin.foc.Id;
      break;
    case 11:      // FOC alpha-beta currents
      evspin.dbg.tmp1 = evspin.foc.Ialpha;
      evspin.dbg.tmp2 = evspin.foc.Ibeta;
      break;
    case 12:      // FOC ab currents
      evspin.dbg.tmp1 = evspin.foc.currents_comp[0];
      evspin.dbg.tmp2 = evspin.foc.currents_comp[1];
      break;
    case 20:      // MRAS dq currents
      evspin.dbg.tmp1 = evspin.mras.Iq_ada;
      evspin.dbg.tmp2 = evspin.mras.Id_ada / 4.0;
      break;
    case 21:      // MRAS speed error
      evspin.dbg.tmp1 = evspin.mras.speed_error;
      evspin.dbg.tmp2 = evspin.mras.speed_mech_ada;
      break;
    case 30:      // FOC dq voltages
      evspin.dbg.tmp1 = evspin.foc.Vq / 6.0;
      evspin.dbg.tmp2 = evspin.foc.Vd / 6.0;
      break;
    case 31:      // FOC alpha-beta voltages
      evspin.dbg.tmp1 = evspin.foc.Valpha/ 6.0;
      evspin.dbg.tmp2 = evspin.foc.Vbeta / 6.0;
      break;
    case 40:      // ab voltages from filtered ADC
      evspin.dbg.tmp1 = ((evspin.adc.buffers.ADC1_reg_raw[U_ADC1] * evspin.adc.vdda / 4096.0f * VMOT_COEFF) - (evspin.adc.vbus / 2)) / 6.0;
      evspin.dbg.tmp2 = ((evspin.adc.buffers.ADC1_reg_raw[V_ADC1] * evspin.adc.vdda / 4096.0f * VMOT_COEFF) - (evspin.adc.vbus / 2)) / 6.0;
      break;
    case 41:      // FOC vs ADC alpha voltages
      float a, b;
      evspin.dbg.tmp1 = ((evspin.adc.buffers.ADC1_reg_raw[U_ADC1] * evspin.adc.vdda / 4096.0f * VMOT_COEFF) - (evspin.adc.vbus / 2)) / 6.0;
      arm_inv_clarke_f32(evspin.foc.Valpha, evspin.foc.Vbeta, &a, &b);
      evspin.dbg.tmp2 = a / 6.0;
      break;
    default:
      evspin.dbg.tmp1 = 0;
      evspin.dbg.tmp2 = 0;
      break;
    }
  }
  else if(LL_ADC_IsActiveFlag_AWD1(hadc1.Instance)) {
    DEBUG_print("AWDG!\r\n");

    LL_ADC_ClearFlag_AWD1(hadc1.Instance);
    LL_TIM_DisableAllOutputs(TIM1);
    // TODO error status
    evspin.state = STATE_ERROR;
  }
  else if(LL_ADC_IsActiveFlag_OVR(hadc1.Instance)) {
//    HAL_GPIO_TogglePin(DBG_DAC1_GPIO_Port, DBG_DAC1_Pin);
    LL_ADC_ClearFlag_OVR(hadc1.Instance);
  }
  else if(LL_ADC_IsActiveFlag_OVR(hadc2.Instance)) {
//    HAL_GPIO_TogglePin(DBG_DAC2_GPIO_Port, DBG_DAC2_Pin);
    LL_ADC_ClearFlag_OVR(hadc2.Instance);
  }
  else {
    volatile uint32_t reg_adc1 = hadc1.Instance->ISR;
    volatile uint32_t reg_adc2 = hadc2.Instance->ISR;
    DEBUG_print("ADC\r\n");
//    DEBUG_print("Unknown ADC interrupt: ");
//    DEBUG_printf("ADC1 0x%02x, ADC2 0x%02x\r\n", (unsigned int)reg_adc1, (unsigned int)reg_adc2);

//    LL_TIM_DisableAllOutputs(TIM1);
//    __NOP();       // TODO DEBUG
  }
}

void EXTI9_5_IRQHandler(void) {
	// encoder index
	if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_8)) {
	  evspin.enc.zero_angle_cnt = LL_TIM_GetCounter(TIM4);
//	  LL_TIM_SetCounter(TIM4, evspin.enc.mech_position * (ENCODER_PULSES * 2) / 360.0);
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_8);
//		DEBUG_print("ZERO\r\n");
//		__NOP();
	}
}

void TIM1_BRK_TIM15_IRQHandler(void) {
  volatile uint8_t reg = 0;
  DRV_ReadReg(DRV_I2C_STATUS, &reg);
  LL_TIM_ClearFlag_BRK(TIM1);
  DEBUG_print("BRAKE!\r\n");

  FOC_Stop();
  LL_TIM_DisableAllOutputs(TIM1);     // just in case
  __NOP();
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
  DEBUG_print("ADC ERROR!\r\n");
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if(hadc->Instance == ADC1) {
    adc1_dma = true;
  }
  else {
    adc2_dma = true;
  }
}

void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg) {
  DEBUG_print("WATCHDOG!");
}
