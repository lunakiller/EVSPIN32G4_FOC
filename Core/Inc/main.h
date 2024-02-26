/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOC
#define DBG_DAC1_Pin GPIO_PIN_4
#define DBG_DAC1_GPIO_Port GPIOA
#define DBG_DAC2_Pin GPIO_PIN_5
#define DBG_DAC2_GPIO_Port GPIOA
#define DRV_WKUP_Pin GPIO_PIN_7
#define DRV_WKUP_GPIO_Port GPIOE
#define DRV_INL1_Pin GPIO_PIN_8
#define DRV_INL1_GPIO_Port GPIOE
#define DRV_INH1_Pin GPIO_PIN_9
#define DRV_INH1_GPIO_Port GPIOE
#define DRV_INL2_Pin GPIO_PIN_10
#define DRV_INL2_GPIO_Port GPIOE
#define DRV_INH2_Pin GPIO_PIN_11
#define DRV_INH2_GPIO_Port GPIOE
#define DRV_INL3_Pin GPIO_PIN_12
#define DRV_INL3_GPIO_Port GPIOE
#define DRV_INH3_Pin GPIO_PIN_13
#define DRV_INH3_GPIO_Port GPIOE
#define DRV_READY_Pin GPIO_PIN_14
#define DRV_READY_GPIO_Port GPIOE
#define DRV_NFAULT_Pin GPIO_PIN_15
#define DRV_NFAULT_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_10
#define LED3_GPIO_Port GPIOB
#define DRV_SCL_Pin GPIO_PIN_8
#define DRV_SCL_GPIO_Port GPIOC
#define DRV_SDA_Pin GPIO_PIN_9
#define DRV_SDA_GPIO_Port GPIOC
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOA
#define STLINK_RX_Pin GPIO_PIN_10
#define STLINK_RX_GPIO_Port GPIOA
#define DBG_SWDIO_Pin GPIO_PIN_13
#define DBG_SWDIO_GPIO_Port GPIOA
#define DBG_SWCLK_Pin GPIO_PIN_14
#define DBG_SWCLK_GPIO_Port GPIOA
#define DBG_SWO_Pin GPIO_PIN_3
#define DBG_SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
