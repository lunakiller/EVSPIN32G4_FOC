/*
 * foc_functions.h
 *
 *  Procedures that execute only once per motor start.
 *
 *  Created on: Mar 29, 2024
 *      Author: lunakiller
 *
 */

#ifndef __FOC_FUNCTIONS_H
#define __FOC_FUNCTIONS_H

#include "foc_motorcontrol.h"


void FOC_Start(void);
void FOC_Stop(void);
void ADC_UpdateAWDG(void);
void ADCs_Setup(void);
void OPAMPs_Setup(void);
void OPAMPs_OffsetCalibration(void);
void TIM1_Setup(void);
void ENC_Setup(void);
void EVSPIN32G4_AssignADCs(ADC_HandleTypeDef* adc1, DMA_HandleTypeDef* dma_adc1,
                          ADC_HandleTypeDef* adc2, DMA_HandleTypeDef* dma_adc2);
void EVSPIN32G4_AssignOPAMPs(OPAMP_HandleTypeDef* opamp1, OPAMP_HandleTypeDef* opamp2,
                             OPAMP_HandleTypeDef* opamp3);
//void EVSPIN32G4_AssignWWDG(WWDG_HandleTypeDef* wwdg);
void EVSPIN32G4_Init(void);

#endif /* __FOC_FUNCTIONS_H */
