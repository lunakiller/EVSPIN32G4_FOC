/*
 * IRQ_handlers.h
 *
 *  Created on: Mar 11, 2024
 *      Author: lunakiller
 *
 */

#ifndef __FOC_HANDLERS_H
#define __FOC_HANDLERS_H


void ADC1_2_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void TIM1_BRK_TIM15_IRQHandler(void);
void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg);


#endif /* __FOC_HANDLERS_H */
