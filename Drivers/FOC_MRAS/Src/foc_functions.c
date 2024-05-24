/*
 * foc_functions.c
 *
 *  Created on: Mar 29, 2024
 *      Author: lunakiller
 *
 */

#include "foc_functions.h"
#include "swo_debug.h"            // debug features

// from main.c
extern Board_Settings_t evspin;
// from foc_handlers.c
extern bool adc1_dma, adc2_dma;

/**
 * @brief Start the FOC algorithm.
 */
void FOC_Start(void) {
  DEBUG_print("FOC_Start()\r\n");
  // recalibrate OPAMPs offsets
  OPAMPs_OffsetCalibration();
  DEBUG_print("OPAMPs offsets calibrated!\r\n");

  // reconfigure ADCs
  ADCs_Setup();
  DEBUG_print("ADCs self-calibrated and running!\r\n");

  // initialize PIDs and related variables
  FOC_PID_Init(&evspin.foc.speed_pid, PID_SPEED_KP, PID_SPEED_KI, PID_SPEED_KD, PID_SPEED_LIMIT);
  evspin.foc.speed_pid.target = STARTUP_SPEED;
  FOC_PID_Init(&evspin.foc.Id_pid, PID_CURRENT_KP, PID_CURRENT_KI, PID_CURRENT_KD, PID_CURRENT_LIMIT);
  FOC_PID_Init(&evspin.foc.Iq_pid, PID_CURRENT_KP, PID_CURRENT_KI, PID_CURRENT_KD, PID_CURRENT_LIMIT);
  evspin.foc.tim.maxCCR = LL_TIM_GetAutoReload(TIM1) - DQLIM_CCR_LIMIT;
  evspin.foc.tim.compute_phase_threshold = LL_TIM_GetAutoReload(TIM1) - (LL_TIM_GetAutoReload(TIM1) / 4);
  evspin.foc.tim.phU = (LL_TIM_GetAutoReload(TIM1) / 2);
  evspin.foc.tim.phV = (LL_TIM_GetAutoReload(TIM1) / 2);
  evspin.foc.tim.phW = (LL_TIM_GetAutoReload(TIM1) / 2);
  FOC_PID_Init(&evspin.mras.omega_pid, PI_MRAS_KP, PI_MRAS_KI, 0, 100000);
  // TODO DEBUG for PID tuning
//  FOC_PID_Init(&evspin.mras.omega_pid, evspin.dbg.Kp, evspin.dbg.Ki, 0, 100000);

  // initialize variables for DQ limiter
  evspin.foc.limit = ((float)DQLIM_MAX_VOLTAGE / 100.0f) * evspin.adc.vbus;
  evspin.foc.limit_squared = evspin.foc.limit * evspin.foc.limit;

  // reset open-loop synchronization counter
  evspin.open.sync_cnt = 0;

  // reset flags
  evspin.base.bootstrap_active = false;
  evspin.base.alignment_active = false;
  evspin.base.startup_active = false;
  evspin.base.synchro_active = false;
  evspin.base.run_active = false;

  // turn off sync indicator
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  // advance to the next state
  evspin.state = STATE_BOOTSTRAP;

  return;
}

/**
 * @brief Disable outputs and reset state.
 */
void FOC_Stop(void) {
  DEBUG_print("FOC_Stop()\r\n");
  LL_TIM_DisableAllOutputs(TIM1);

  evspin.state = STATE_IDLE;
}

/**
 * @brief Update Analog Watchdog thresholds with respect to measured VDDA.
 */
void ADC_UpdateAWDG(void) {
  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};

  AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.Channel = ADC_CHANNEL_1;
  AnalogWDGConfig.ITMode = ENABLE;
  AnalogWDGConfig.HighThreshold = OVERVOLTAGE_THRESHOLD * (1.0f / VBUS_COEFF) * 4096.0f / evspin.adc.vdda;
  AnalogWDGConfig.LowThreshold = UNDERVOLTAGE_THRESHOLD * (1.0f / VBUS_COEFF) * 4096.0f / evspin.adc.vdda;
  AnalogWDGConfig.FilteringConfig = ADC_AWD_FILTERING_NONE;
  if (HAL_ADC_AnalogWDGConfig(evspin.periph.adc1, &AnalogWDGConfig) != HAL_OK) {
    Error_Handler();
  }

  return;
}

/**
 * @brief Setup ADCs for FOC.
 */
void ADCs_Setup(void) {
  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  // disable ADC and DMA
  HAL_ADC_Stop_DMA(evspin.periph.adc1);
  HAL_ADC_Stop_DMA(evspin.periph.adc2);

  // configure ADC1
  evspin.periph.adc1->Instance = ADC1;
  evspin.periph.adc1->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  evspin.periph.adc1->Init.Resolution = ADC_RESOLUTION_12B;
  evspin.periph.adc1->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  evspin.periph.adc1->Init.GainCompensation = 0;
  evspin.periph.adc1->Init.ScanConvMode = ADC_SCAN_ENABLE;
  evspin.periph.adc1->Init.EOCSelection = ADC_EOC_SEQ_CONV;
  evspin.periph.adc1->Init.LowPowerAutoWait = DISABLE;
  evspin.periph.adc1->Init.ContinuousConvMode = ENABLE;
  evspin.periph.adc1->Init.NbrOfConversion = ADC1_REG_CHANNELS;
  evspin.periph.adc1->Init.DiscontinuousConvMode = DISABLE;
  evspin.periph.adc1->Init.ExternalTrigConv = ADC_SOFTWARE_START;
  evspin.periph.adc1->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  evspin.periph.adc1->Init.DMAContinuousRequests = ENABLE;
  evspin.periph.adc1->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  evspin.periph.adc1->Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(evspin.periph.adc1) != HAL_OK) {
    Error_Handler();
  }

  // configure Analog WatchDog 1
  AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.Channel = ADC_CHANNEL_1;
  AnalogWDGConfig.ITMode = ENABLE;
  AnalogWDGConfig.HighThreshold = OVERVOLTAGE_THRESHOLD * (1.0f / VBUS_COEFF) * 4096.0f / 3.3f;
  AnalogWDGConfig.LowThreshold = UNDERVOLTAGE_THRESHOLD * (1.0f / VBUS_COEFF) * 4096.0f / 3.3f;
  AnalogWDGConfig.FilteringConfig = ADC_AWD_FILTERING_NONE;
  if (HAL_ADC_AnalogWDGConfig(evspin.periph.adc1, &AnalogWDGConfig) != HAL_OK) {
    Error_Handler();
  }

  // configure regular channels: VBUS, phase U, phase V and Vrefint
  sConfig.Channel = VBUS_ADC1_CHANNEL;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(evspin.periph.adc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = U_ADC1_CHANNEL;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(evspin.periph.adc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = V_ADC1_CHANNEL;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(evspin.periph.adc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  if (HAL_ADC_ConfigChannel(evspin.periph.adc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  // configure injected channels: shunt U, shunt W
  sConfigInjected.InjectedChannel = CS_U_ADC1_CHANNEL;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = ADC1_INJ_CHANNELS;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(evspin.periph.adc1, &sConfigInjected) != HAL_OK) {
    Error_Handler();
  }

  sConfigInjected.InjectedChannel = CS_W_ADC1_CHANNEL;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(evspin.periph.adc1, &sConfigInjected) != HAL_OK) {
    Error_Handler();
  }

  // configure ADC2
  evspin.periph.adc2->Instance = ADC2;
  evspin.periph.adc2->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  evspin.periph.adc2->Init.Resolution = ADC_RESOLUTION_12B;
  evspin.periph.adc2->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  evspin.periph.adc2->Init.GainCompensation = 0;
  evspin.periph.adc2->Init.ScanConvMode = ADC_SCAN_ENABLE;
  evspin.periph.adc2->Init.EOCSelection = ADC_EOC_SEQ_CONV;
  evspin.periph.adc2->Init.LowPowerAutoWait = DISABLE;
  evspin.periph.adc2->Init.ContinuousConvMode = ENABLE;
  evspin.periph.adc2->Init.NbrOfConversion = ADC2_REG_CHANNELS;
  evspin.periph.adc2->Init.DiscontinuousConvMode = DISABLE;
  evspin.periph.adc2->Init.ExternalTrigConv = ADC_SOFTWARE_START;
  evspin.periph.adc2->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  evspin.periph.adc2->Init.DMAContinuousRequests = ENABLE;
  evspin.periph.adc2->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  evspin.periph.adc2->Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(evspin.periph.adc2) != HAL_OK) {
    Error_Handler();
  }

  // configure regular channels: NTC, phase W and potentiometer
  sConfig.Channel = NTC_ADC2_CHANNEL;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(evspin.periph.adc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = W_ADC2_CHANNEL;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(evspin.periph.adc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = POT_ADC2_CHANNEL;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(evspin.periph.adc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  // configure injected channel: current V
  sConfigInjected.InjectedChannel = CS_V_ADC2_CHANNEL;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = ADC2_INJ_CHANNELS;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(evspin.periph.adc2, &sConfigInjected) != HAL_OK) {
    Error_Handler();
  }

  // calibrate ADCs
  HAL_ADCEx_Calibration_Start(evspin.periph.adc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(evspin.periph.adc2, ADC_SINGLE_ENDED);

  // enable interrupts, highest priority
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

  // enable DMA circular mode
  LL_DMA_SetMode(__LL_DMA_GET_INSTANCE(evspin.periph.dma_adc1->Instance), __LL_DMA_GET_CHANNEL(evspin.periph.dma_adc1->Instance), LL_DMA_MODE_CIRCULAR);
  LL_DMA_SetMode(__LL_DMA_GET_INSTANCE(evspin.periph.dma_adc2->Instance), __LL_DMA_GET_CHANNEL(evspin.periph.dma_adc2->Instance), LL_DMA_MODE_CIRCULAR);

  // run ADCs
  HAL_ADC_Start_DMA(evspin.periph.adc1, (uint32_t*)evspin.adc.buffers.ADC1_reg_raw, ADC1_REG_CHANNELS);
  HAL_ADC_Start_DMA(evspin.periph.adc2, (uint32_t*)evspin.adc.buffers.ADC2_reg_raw, ADC2_REG_CHANNELS);
  HAL_ADCEx_InjectedStart_IT(evspin.periph.adc1);
  HAL_ADCEx_InjectedStart_IT(evspin.periph.adc2);

  // disable DMA interrupts
  __HAL_DMA_DISABLE_IT(evspin.periph.dma_adc1, DMA_IT_HT|DMA_IT_TC);
  __HAL_DMA_DISABLE_IT(evspin.periph.dma_adc2, DMA_IT_HT|DMA_IT_TC);

  // disable ADC2 injected end-of-sequence IRQ since both ADCs are triggered
  // at the exact same time and ADC2 will always be faster
  __HAL_ADC_DISABLE_IT(evspin.periph.adc2, ADC_IT_JEOS);

  return;
}

/**
 * @brief Self-calibrate and start OPAMPs.
 */
void OPAMPs_Setup(void) {
  HAL_OPAMPEx_SelfCalibrateAll(evspin.periph.opamp1, evspin.periph.opamp2, evspin.periph.opamp3);
  HAL_OPAMP_Start(evspin.periph.opamp1);
  HAL_OPAMP_Start(evspin.periph.opamp2);
  HAL_OPAMP_Start(evspin.periph.opamp3);

  return;
}

/**
 * @brief Calibrate current offset by measuring ADC voltage with no current.
 */
void OPAMPs_OffsetCalibration(void) {
  uint16_t buffer[CS_OFFSET_CALIB_AVG] = {0};
  uint32_t mean;
  ADC_ChannelConfTypeDef sConfig = {0};

  // disable ADC and DMA
  HAL_ADC_Stop_DMA(evspin.periph.adc1);
  HAL_ADC_Stop_DMA(evspin.periph.adc2);

  // configure ADC1
  evspin.periph.adc1->Instance = ADC1;
  evspin.periph.adc1->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  evspin.periph.adc1->Init.Resolution = ADC_RESOLUTION_12B;
  evspin.periph.adc1->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  evspin.periph.adc1->Init.GainCompensation = 0;
  evspin.periph.adc1->Init.ScanConvMode = ADC_SCAN_DISABLE;
  evspin.periph.adc1->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  evspin.periph.adc1->Init.LowPowerAutoWait = DISABLE;
  evspin.periph.adc1->Init.ContinuousConvMode = ENABLE;
  evspin.periph.adc1->Init.NbrOfConversion = 1;
  evspin.periph.adc1->Init.DiscontinuousConvMode = DISABLE;
  evspin.periph.adc1->Init.ExternalTrigConv = ADC_SOFTWARE_START;
  evspin.periph.adc1->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  evspin.periph.adc1->Init.DMAContinuousRequests = DISABLE;
  evspin.periph.adc1->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  evspin.periph.adc1->Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(evspin.periph.adc1) != HAL_OK) {
    Error_Handler();
  }

  // calibrate ADC
  HAL_ADCEx_Calibration_Start(evspin.periph.adc1, ADC_SINGLE_ENDED);

  // disable DMA circular mode
  LL_DMA_SetMode(__LL_DMA_GET_INSTANCE(evspin.periph.dma_adc1->Instance), __LL_DMA_GET_CHANNEL(evspin.periph.dma_adc1->Instance), LL_DMA_MODE_NORMAL);

  // phase U
  sConfig.Channel = CS_U_ADC1_CHANNEL;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(evspin.periph.adc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  adc1_dma = false;
  HAL_ADC_Start_DMA(evspin.periph.adc1, (uint32_t*)buffer, 16);
  while(!adc1_dma);

  mean = 0;
  for(int i = 0; i < CS_OFFSET_CALIB_AVG; ++i) {
    mean += buffer[i];
  }
  mean /= CS_OFFSET_CALIB_AVG;
  evspin.adc.offsets.phase_u_offset = mean;

  // phase W
  sConfig.Channel = CS_W_ADC1_CHANNEL;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  if (HAL_ADC_ConfigChannel(evspin.periph.adc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  adc1_dma = false;
  HAL_ADC_Start_DMA(evspin.periph.adc1, (uint32_t*)buffer, 16);
  while(!adc1_dma);

  mean = 0;
  for(int i = 0; i < CS_OFFSET_CALIB_AVG; ++i) {
    mean += buffer[i];
  }
  mean /= CS_OFFSET_CALIB_AVG;
  evspin.adc.offsets.phase_w_offset = mean;

  // configure ADC2
  evspin.periph.adc2->Instance = ADC2;
  evspin.periph.adc2->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  evspin.periph.adc2->Init.Resolution = ADC_RESOLUTION_12B;
  evspin.periph.adc2->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  evspin.periph.adc2->Init.GainCompensation = 0;
  evspin.periph.adc2->Init.ScanConvMode = ADC_SCAN_DISABLE;
  evspin.periph.adc2->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  evspin.periph.adc2->Init.LowPowerAutoWait = DISABLE;
  evspin.periph.adc2->Init.ContinuousConvMode = ENABLE;
  evspin.periph.adc2->Init.NbrOfConversion = 1;
  evspin.periph.adc2->Init.DiscontinuousConvMode = DISABLE;
  evspin.periph.adc2->Init.ExternalTrigConv = ADC_SOFTWARE_START;
  evspin.periph.adc2->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  evspin.periph.adc2->Init.DMAContinuousRequests = DISABLE;
  evspin.periph.adc2->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  evspin.periph.adc2->Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(evspin.periph.adc2) != HAL_OK) {
    Error_Handler();
  }

  // calibrate ADC
  HAL_ADCEx_Calibration_Start(evspin.periph.adc2, ADC_SINGLE_ENDED);

  // disable DMA circular mode
  LL_DMA_SetMode(__LL_DMA_GET_INSTANCE(evspin.periph.dma_adc2->Instance), __LL_DMA_GET_CHANNEL(evspin.periph.dma_adc2->Instance), LL_DMA_MODE_NORMAL);

  // phase V
  sConfig.Channel = CS_V_ADC2_CHANNEL;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  if (HAL_ADC_ConfigChannel(evspin.periph.adc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  adc2_dma = false;
  HAL_ADC_Start_DMA(evspin.periph.adc2, (uint32_t*)buffer, 16);
  while(!adc2_dma);

  mean = 0;
  for(int i = 0; i < CS_OFFSET_CALIB_AVG; ++i) {
    mean += buffer[i];
  }
  mean /= CS_OFFSET_CALIB_AVG;
  evspin.adc.offsets.phase_v_offset = mean;

  return;
}

/**
 * @brief Setup the main timer for PWM and ADC trigger.
 */
void TIM1_Setup(void) {
  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  // enable peripheral clock
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOE);

  /**TIM1 GPIO Configuration
  PE8    ------> TIM1_CH1N (DRV_INL1)
  PE9    ------> TIM1_CH1 (DRV_INH1)
  PE10   ------> TIM1_CH2N (DRV_INL2)
  PE11   ------> TIM1_CH2 (DRV_INH2)
  PE12   ------> TIM1_CH3N (DRV_INL3)
  PE13   ------> TIM1_CH3 (DRV_INH3)
  PE15   ------> TIM1_BKIN (DRV_nFAULT)
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // register interrupt
  NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);

  // TIM1 configuration
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_CENTER_DOWN;
  TIM_InitStruct.Autoreload = (170000 / (2 * SWITCHING_FREQUENCY * (TIM_InitStruct.Prescaler + 1))) - 1;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 1;             // two PWM half-periods
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM1);
  LL_TIM_CC_EnablePreload(TIM1);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_OC4REF);
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);

  // PWM mode 1:
  //  - upcounting:   CNT < CCRx  -> CCxE high, CCxNE low   ... high-side mosfet ON
  //                  CNT >= CCRx -> CCxE low, CCxNE high   ... low-side mosfet ON
  //  - downcounting: CNT > CCRx  -> CCxE low, CCxNE high   ... low-side mosfet ON
  //                  CNT <= CCRx -> CCxE high, CCxNE low   ... high-side mosfet ON

  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);

  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);

  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);

  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH4);

  // brake settings
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = __LL_TIM_CALC_DEADTIME(170000000, LL_TIM_GetClockDivision(TIM1), DEAD_TIME);
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_ENABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_LOW;
  TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1_N4;
  TIM_BDTRInitStruct.BreakAFMode = LL_TIM_BREAK_AFMODE_INPUT;
  TIM_BDTRInitStruct.Break2State = LL_TIM_BREAK2_DISABLE;
  TIM_BDTRInitStruct.Break2Polarity = LL_TIM_BREAK2_POLARITY_LOW;
  TIM_BDTRInitStruct.Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1;
  TIM_BDTRInitStruct.Break2AFMode = LL_TIM_BREAK_AFMODE_INPUT;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);

  LL_TIM_ClearFlag_BRK(TIM1);
  LL_TIM_EnableIT_BRK(TIM1);

  // enable output channels, reset and enable the counter
  LL_TIM_CC_EnableChannel(TIM1, (LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2 |
                            LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N));

  LL_TIM_OC_SetCompareCH1(TIM1, 0);
  LL_TIM_OC_SetCompareCH2(TIM1, 0);
  LL_TIM_OC_SetCompareCH3(TIM1, 0);
  LL_TIM_OC_SetCompareCH4(TIM1, LL_TIM_GetAutoReload(TIM1) - 10);

  LL_TIM_GenerateEvent_COM(TIM1);
  LL_TIM_GenerateEvent_UPDATE(TIM1);

  LL_TIM_EnableCounter(TIM1);

  return;
}

/**
 * @brief Setup TIM4 for encoder mode.
 */
void ENC_Setup(void) {
  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  // enable peripheral clock
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

  /**ENC GPIO Configuration
  PB6   ------> TIM4_CH1 (ENC_CH1)
  PB7   ------> TIM4_CH2 (ENC_CH2)
  PB8   ------> GPIO_EXTI8 (ENC_INDEX)
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_8);
  LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_8);

  // register interrupt
  NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(EXTI9_5_IRQn);

  // TIM4 configuration
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = (ENCODER_PULSES * 2) - 1;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM4);
  LL_TIM_SetEncoderMode(TIM4, LL_TIM_ENCODERMODE_X2_TI1);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM4);

  LL_TIM_IC_SetActiveInput(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV4_N6);
  LL_TIM_IC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);

  LL_TIM_IC_SetActiveInput(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV4_N6);
  LL_TIM_IC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_FALLING);    // TODO invert polarity macro

  // enable update interrupt flag to be remapped to bit 31 of counter
  LL_TIM_EnableUIFRemap(TIM4);

  // enable channels and start the timer
  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);

  // set filter
  evspin.enc.speed_lpf.alpha = 0.95;

  LL_TIM_ClearFlag_UPDATE(TIM4);

  return;
}

/**
 * @brief Configure the window watchdog.
 */
void WWDG_Setup(void) {
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_WWDG);

  NVIC_SetPriority(WWDG_IRQn, 0);
  NVIC_EnableIRQ(WWDG_IRQn);

  LL_WWDG_SetPrescaler(WWDG, LL_WWDG_PRESCALER_2);
  LL_WWDG_EnableIT_EWKUP(WWDG);
  LL_WWDG_SetWindow(WWDG, 127);
  LL_WWDG_SetCounter(WWDG, 95);
  LL_WWDG_Enable(WWDG);
}

/**
 * @brief Assign pointers to ADC handles.
 * @param adc1 Pointer to ADC1 HAL handle.
 * @param dma_adc1 Pointer to ADC1 DMA HAL handle.
 * @param adc2 Pointer to ADC2 HAL handle.
 * @param dma_adc2 Pointer to ADC2 DMA HAL handle.
 */
void EVSPIN32G4_AssignADCs(ADC_HandleTypeDef* adc1, DMA_HandleTypeDef* dma_adc1,
                          ADC_HandleTypeDef* adc2, DMA_HandleTypeDef* dma_adc2)
{
  evspin.periph.adc1 = adc1;
  evspin.periph.dma_adc1 = dma_adc1;
  evspin.periph.adc2 = adc2;
  evspin.periph.dma_adc2 = dma_adc2;

  return;
}

/**
 * @brief Assign pointers to OPAMP handles.
 * @param opamp1 Pointer to OPAMP1 HAL handle.
 * @param opamp2 Pointer to OPAMP2 HAL handle.
 * @param opamp3 Pointer to OPAMP3 HAL handle.
 */
void EVSPIN32G4_AssignOPAMPs(OPAMP_HandleTypeDef* opamp1, OPAMP_HandleTypeDef* opamp2,
                             OPAMP_HandleTypeDef* opamp3)
{
  evspin.periph.opamp1 = opamp1;
  evspin.periph.opamp2 = opamp2;
  evspin.periph.opamp3 = opamp3;

  return;
}

/**
 * @brief Initialize EVSPIN32G4 board.
 */
void EVSPIN32G4_Init(void) {
  // check peripheral pointers
  if(evspin.periph.adc1 == NULL || evspin.periph.adc2 == NULL || evspin.periph.dma_adc1 == NULL ||
     evspin.periph.dma_adc2 == NULL || evspin.periph.opamp1 == NULL || evspin.periph.opamp2 == NULL ||
     evspin.periph.opamp3 == NULL)
  {
    DEBUG_Breakpoint();
    return;
  }

  // initialize gate driver
  if(DRV_Init() != DRV_OK)
      Error_Handler();
  DEBUG_print("Gate Driver initialized!\r\n");

  // OPAMPs
  OPAMPs_Setup();
  DEBUG_print("OPAMPs self-calibrated and running!\r\n");

  OPAMPs_OffsetCalibration();
  DEBUG_print("OPAMPs offsets calibrated!\r\n");

  // ADCs
  ADCs_Setup();
  DEBUG_print("ADCs self-calibrated and running!\r\n");

  // Main timer
  TIM1_Setup();
  DEBUG_print("Main timer initialized!\r\n");

  // Rotary encoder
  ENC_Setup();
  evspin.enc.cnt_to_deg = 360.0f / (ENCODER_PULSES * 2);
  DEBUG_print("TIM4 in encoder mode initialized!\r\n");

  // Window watchdog
  WWDG_Setup();
  DEBUG_print("WWDG activated!\r\n");

  // default state
  evspin.state = STATE_IDLE;

  return;
}
