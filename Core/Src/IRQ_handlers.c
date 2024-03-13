/*
 * IRQ_handlers.c
 *
 *  Created on: Mar 11, 2024
 *      Author: lunakiller
 *
 */

#include "IRQ_handlers.h"
#include "main.h"
#include "typedefs.h"

#include "swo_debug.h"


extern Board_Settings_t evspin;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

void ADC1_2_IRQHandler(void) {
  if(LL_ADC_IsActiveFlag_AWD1(hadc1.Instance)) {
    DEBUG_printf("ANALOG WATCHDOG!\r\n");

    LL_ADC_ClearFlag_AWD1(hadc1.Instance);
  }
  else if(LL_ADC_IsActiveFlag_JEOS(hadc1.Instance) && LL_ADC_IsActiveFlag_JEOS(hadc2.Instance)) {
    evspin.adc.buffers.ADC1_inj_raw[0] = LL_ADC_INJ_ReadConversionData12(hadc1.Instance, LL_ADC_INJ_RANK_1);
    evspin.adc.buffers.ADC1_inj_raw[1] = LL_ADC_INJ_ReadConversionData12(hadc1.Instance, LL_ADC_INJ_RANK_2);
    evspin.adc.buffers.ADC2_inj_raw[0] = LL_ADC_INJ_ReadConversionData12(hadc2.Instance, LL_ADC_INJ_RANK_1);

    evspin.adc.currents[0] = (evspin.adc.buffers.ADC1_inj_raw[0] - evspin.offsets.phase_u_offset) * evspin.adc.vdda / 4096.0 / (CS_GAIN * CS_SHUNT_VALUE);
    evspin.adc.currents[1] = (evspin.adc.buffers.ADC2_inj_raw[0] - evspin.offsets.phase_v_offset) * evspin.adc.vdda / 4096.0 / (CS_GAIN * CS_SHUNT_VALUE);
    evspin.adc.currents[2] = (evspin.adc.buffers.ADC1_inj_raw[1] - evspin.offsets.phase_w_offset) * evspin.adc.vdda / 4096.0 / (CS_GAIN * CS_SHUNT_VALUE);

    LL_ADC_ClearFlag_JEOS(hadc1.Instance);
    LL_ADC_ClearFlag_JEOS(hadc2.Instance);
  }
  else {
    volatile uint32_t reg_adc1 = hadc1.Instance->ISR;
    volatile uint32_t reg_adc2 = hadc2.Instance->ISR;
    DEBUG_printf("Unknown ADC interrupt...\r\n");
    __BKPT();       // TODO DEBUG
  }
}

void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg) {
  DEBUG_printf("WATCHDOG!");
}
