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

/*static*/ DRV_StatusTypeDef DRV_ReadReg(uint8_t addr, uint8_t* val) {
	if(HAL_I2C_Mem_Read(&hi2c3, (DRV_I2C_ADDR << 1), addr, 1, val, 1, DRV_I2C_TIMEOUT) == HAL_OK)
		return DRV_OK;
	else
		return DRV_ERROR;
}

/*static*/ DRV_StatusTypeDef DRV_WriteReg(uint8_t addr, uint8_t val) {
	if(HAL_I2C_Mem_Write(&hi2c3, (DRV_I2C_ADDR << 1), addr, 1, &val, 1, DRV_I2C_TIMEOUT) == HAL_OK)
		return DRV_OK;
	else
		return DRV_ERROR;
}

DRV_StatusTypeDef DRV_Unlock(void) {
	DRV_WriteReg(DRV_I2C_LOCK, DRV_I2C_LOCK_KEY);
	uint8_t key = 0;
	DRV_ReadReg(DRV_I2C_LOCK, &key);
	if(key == DRV_I2C_LOCK_KEY)
		return DRV_OK;
	else
		return DRV_ERROR;
}

DRV_StatusTypeDef DRV_Lock(void) {
	return DRV_WriteReg(DRV_I2C_LOCK, 0x00);
}
