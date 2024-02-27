/*
 * stspin32g4.c
 *
 *  Created on: Feb 25, 2024
 *      Author: lunakiller
 *
 *
 * ------------------------------- ! NOTICE ! ---------------------------------
 *    This library assumes that both I2C3 peripheral and GPIO pins for gate
 *    driver are correctly set and initialized!!!
 * ----------------------------------------------------------------------------
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


DRV_StatusTypeDef DRV_Init(void) {
  // unlock registers
  if(DRV_Unlock() != DRV_OK)
    return DRV_ERROR;
  // perform a reset
  DRV_Reset();
  // unlock again since the reset re-locked
  if(DRV_Unlock() != DRV_OK)
    return DRV_ERROR;
  // set VCC value
  if(DRV_SetVCC(DRV_I2C_POWMNG_VCC_VAL_0) != DRV_OK)        // VCC reg output 10V
    return DRV_ERROR;
  // set deglitch value
  if(DRV_SetDeglitch(DRV_I2C_LOGIC_VDS_P_DEG_0) != DRV_OK)  // deglitch 4us
    return DRV_ERROR;
  // lock and clear faults
  DRV_Lock();
  DRV_ClearFaults();
  return DRV_OK;
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

DRV_StatusTypeDef DRV_SetVCC(uint8_t vcc_bits) {
  uint8_t reg = 0;
  if(DRV_ReadReg(DRV_I2C_POWMNG, &reg) != DRV_OK)
    return DRV_ERROR;
  reg |= (vcc_bits & (DRV_I2C_POWMNG_VCC_VAL_0 | DRV_I2C_POWMNG_VCC_VAL_1));
  return DRV_WriteReg(DRV_I2C_POWMNG, reg);
}

DRV_StatusTypeDef DRV_SetDeglitch(uint8_t vds_p_deg_bits) {
  uint8_t reg = 0;
  if(DRV_ReadReg(DRV_I2C_POWMNG, &reg) != DRV_OK)
    return DRV_ERROR;
  reg |= (vds_p_deg_bits & (DRV_I2C_LOGIC_VDS_P_DEG_0 | DRV_I2C_LOGIC_VDS_P_DEG_1));
  return DRV_WriteReg(DRV_I2C_LOGIC, reg);
}

DRV_StatusTypeDef DRV_Reset(void) {
  return DRV_WriteReg(DRV_I2C_RESET, 0xFF);
}

DRV_StatusTypeDef DRV_ClearFaults(void) {
  return DRV_WriteReg(DRV_I2C_CLEAR, 0xFF);
}
