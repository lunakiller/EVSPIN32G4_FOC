/*
 * foc_handlers.h
 *
 *  Interrupt handler definitions for peripherals using LL.
 *
 *  Created on: Mar 11, 2024
 *      Author: lunakiller
 *
 */

#ifndef __FOC_HANDLERS_H
#define __FOC_HANDLERS_H

#include "foc_motorcontrol.h"


void ADC1_2_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void TIM1_BRK_TIM15_IRQHandler(void);
void WWDG_IRQHandler(void);


#endif /* __FOC_HANDLERS_H */
