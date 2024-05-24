/*
 * stspin32g4.h
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

#ifndef __STSPIN32G4_H
#define __STSPIN32G4_H

#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_def.h>

/* ---------------------- Gate driver I2C definitions ---------------------- */
// General
#define DRV_I2C_TIMEOUT							100
#define DRV_I2C_ADDR 								0b1000111

// Power manager configuration register, default: 0x00, Protected
#define DRV_I2C_POWMNG							0x01
#define DRV_I2C_POWMNG_REG3V3				(0b1 << 6)			// Enabled by default, 1 to disable
#define DRV_I2C_POWMNG_VCC					(0b1 << 5)			// Enabled by default, 1 to disable
#define DRV_I2C_POWMNG_STBY_REG			(0b1 << 4)			// Disabled by default, 1 to enable
#define DRV_I2C_POWMNG_VCC_VAL_0		(0b1 << 0)			// 00: VCC = 8V (default), 01: VCC = 10V
#define DRV_I2C_POWMNG_VCC_VAL_1		(0b1 << 1)			// 10: VCC = 12V, 10: VCC = 12V

// Driving Logic configuration register, default: 0x73, Protected
#define DRV_I2C_LOGIC								0x02
#define DRV_I2C_LOGIC_VDS_P_DEG_0		(0b1 << 2)			// 00: deglitch = 6us (default), 01: deglitch = 4us
#define DRV_I2C_LOGIC_VDS_P_DEG_1		(0b1 << 3)			// 10: deglitch = 3us, 11: deglitch = 2us
#define DRV_I2C_LOGIC_DTMIN					(0b1 << 1)			// Enabled by default, 0 to disable
#define DRV_I2C_LOGIC_ILOCK					(0b1 << 0)			// Enabled by default, 0 to disable

// READY output configuration register, default: 0x09
#define DRV_I2C_READY								0x07
#define DRV_I2C_READY_STBY_RDY			(0b1 << 3)			// Enabled by default, 0 to disable
#define DRV_I2C_READY_THSD_RDY			(0b1 << 1)			// Disabled by default, 1 to enable
#define DRV_I2C_READY_VCC_UVLO_RDY	(0b1 << 0)			// Enabled by default, 0 to disable

// nFAULT output configuration register, default: 0x7F, Protected
#define DRV_I2C_NFAULT							0x08
#define DRV_I2C_NFAULT_VDS_P_FLT		(0b1 << 2)			// Enabled by default, 0 to disable
#define DRV_I2C_NFAULT_THSD_FLT			(0b1 << 1)			// Enabled by default, 0 to disable
#define DRV_I2C_NFAULT_VCC_UVLO_FLT	(0b1 << 0)			// Enabled by default, 0 to disable

// FAULT clear command register
#define DRV_I2C_CLEAR								0x09						// Write 0xff to clear faults

// Standby register, default: 0x0
#define DRV_I2C_STBY								0x0A						// Write 0x01 to request low consumption mode

// LOCK register, default: 0x0
#define DRV_I2C_LOCK								0x0B						// LOCK[3:0] = ~LOCK[7:4]
#define DRV_I2C_LOCK_KEY						0xA5

// RESET command register, Protected
#define DRV_I2C_RESET								0x0C						// Write 0xff to reset registers to their defaults

// Device STATUS register, Read-only
#define DRV_I2C_STATUS							0x80
#define DRV_I2C_STATUS_LOCK					(0b1 << 7)			// 0: unlocked, 1: locked
#define DRV_I2C_STATUS_RESET				(0b1 << 3)			// 0: no reset, 1: reset
#define DRV_I2C_STATUS_VDS_P				(0b1 << 2)			// 0: VDS protection not triggered, 1: triggered
#define DRV_I2C_STATUS_THSD					(0b1 << 1)			// 0: device not in THSD, 1: in THSD
#define DRV_I2C_STATUS_UVLO					(0b1 << 0)			// 0: device not in VCC UVLO, 1: in VCC UVLO

/* --------------------------- Type definitions ---------------------------- */
typedef enum {
  DRV_OK       = 0x00U,
  DRV_ERROR    = 0x01U
} DRV_StatusTypeDef;


/* ------------------------------ Functions -------------------------------- */
/*static*/ DRV_StatusTypeDef DRV_ReadReg(uint8_t addr, uint8_t* val);
/*static*/ DRV_StatusTypeDef DRV_WriteReg(uint8_t addr, uint8_t val);
DRV_StatusTypeDef DRV_Init(void);
DRV_StatusTypeDef DRV_Unlock(void);
DRV_StatusTypeDef DRV_Lock(void);
DRV_StatusTypeDef DRV_SetVCC(uint8_t vcc_bits);
DRV_StatusTypeDef DRV_SetDeglitch(uint8_t vds_p_deg_bits);
DRV_StatusTypeDef DRV_Reset(void);
DRV_StatusTypeDef DRV_ClearFaults(void);


#endif /* __STSPIN32G4_H */
