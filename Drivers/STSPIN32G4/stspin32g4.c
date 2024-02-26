/*
 * stspin32g4.c
 *
 *  Created on: Feb 25, 2024
 *      Author: lunakiller
 */

#include "stspin32g4.h"
#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_def.h>

extern I2C_HandleTypeDef hi2c3;

HAL_StatusTypeDef DRV_ReadReg(uint8_t addr, uint8_t* val) {
	return HAL_I2C_Mem_Read(&hi2c3, (DRV_I2C_ADDR << 1), addr, 1, val, 1, DRV_I2C_TIMEOUT);
}

HAL_StatusTypeDef DRV_WriteReg(uint8_t addr, uint8_t val) {
	return HAL_I2C_Mem_Write(&hi2c3, (DRV_I2C_ADDR << 1), addr, 1, &val, 1, DRV_I2C_TIMEOUT);
}
