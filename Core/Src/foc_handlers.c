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

    // TODO state machine
    switch(evspin.state) {
    case STATE_IDLE:
      break;
    case STATE_BOOTSTRAP:
      break;
    case STATE_READY:
      FOC_MainControl();
      break;
    case STATE_ALIGNMENT:
      FOC_MainControl();
      break;
    case STATE_STARTUP:
      FOC_OpenLoop_StartUp();
      FOC_MainControl();
      break;
    case STATE_SYNCHRO:
      FOC_PositionSynchronization();
      FOC_EncoderProcessing();
      FOC_MRAS();
      FOC_MainControl();
      break;
    case STATE_RUN:
      FOC_EncoderProcessing();
      FOC_MRAS();
      FOC_MainControl();
      break;
    default:
      break;
    }
  }
  else if(LL_ADC_IsActiveFlag_AWD1(hadc1.Instance)) {
    DEBUG_print("AWDG!\r\n");

    LL_ADC_ClearFlag_AWD1(hadc1.Instance);
    LL_TIM_DisableAllOutputs(TIM1);
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
    DEBUG_print("Unknown ADC interrupt: ");
    DEBUG_printf("ADC1 0x%02x, ADC2 0x%02x\r\n", (unsigned int)reg_adc1, (unsigned int)reg_adc2);

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
		DEBUG_print("ENC ZERO\r\n");
		__NOP();
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
