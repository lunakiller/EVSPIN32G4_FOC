/*
 * IRQ_handlers.c
 *
 *  Created on: Mar 11, 2024
 *      Author: lunakiller
 *
 */

#include "foc_motorcontrol.h"

#include "swo_debug.h"


extern Board_Settings_t evspin;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

void ADC1_2_IRQHandler(void) {
  if(LL_ADC_IsActiveFlag_AWD1(hadc1.Instance)) {
    DEBUG_printf("ANALOG WATCHDOG!\r\n");

    LL_ADC_ClearFlag_AWD1(hadc1.Instance);
    LL_TIM_DisableAllOutputs(TIM1);
  }
  else if(LL_ADC_IsActiveFlag_JEOS(hadc1.Instance) && LL_ADC_IsActiveFlag_JEOS(hadc2.Instance)) {
    evspin.adc.buffers.ADC1_inj_raw[0] = LL_ADC_INJ_ReadConversionData12(hadc1.Instance, LL_ADC_INJ_RANK_1);
    evspin.adc.buffers.ADC1_inj_raw[1] = LL_ADC_INJ_ReadConversionData12(hadc1.Instance, LL_ADC_INJ_RANK_2);
    evspin.adc.buffers.ADC2_inj_raw[0] = LL_ADC_INJ_ReadConversionData12(hadc2.Instance, LL_ADC_INJ_RANK_1);

    LL_ADC_ClearFlag_JEOS(hadc1.Instance);
    LL_ADC_ClearFlag_JEOS(hadc2.Instance);

    // TODO state machine
    FOC_Task();
  }
  else {
    volatile uint32_t reg_adc1 = hadc1.Instance->ISR;
    volatile uint32_t reg_adc2 = hadc2.Instance->ISR;
    DEBUG_print("Unknown ADC interrupt: ");
    DEBUG_printf("ADC1 0x%02x, ADC2 0x%02x\r\n", (unsigned int)reg_adc1, (unsigned int)reg_adc2);

    LL_TIM_DisableAllOutputs(TIM1);
    __NOP();       // TODO DEBUG
  }
}

void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg) {
  DEBUG_print("WATCHDOG!");
}
