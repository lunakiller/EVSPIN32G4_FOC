/*
 * typedefs.h
 *
 *  Created on: Mar 11, 2024
 *      Author: lunakiller
 *
 */

#ifndef __TYPEDEFS_H
#define __TYPEDEFS_H

#include "foc_motorcontrol.h"
#include <stdint.h>
#include <stdbool.h>


#define VBUS_ADC1                         0
#define U_ADC1                            1
#define V_ADC1                            2
#define VREFINT_ADC1                      3
#define NTC_ADC2                          0
#define W_ADC2                            1
#define POT_ADC2                          2
#define CS_U_ADC1                         0
#define CS_W_ADC1                         1
#define CS_V_ADC2                         0

typedef struct {
  uint16_t phase_u_offset;
  uint16_t phase_v_offset;
  uint16_t phase_w_offset;
} CS_Offset_t;

typedef struct {
  uint16_t ADC1_reg_raw[ADC1_REG_CHANNELS];
  uint16_t ADC2_reg_raw[ADC2_REG_CHANNELS];
  uint16_t ADC1_inj_raw[ADC1_INJ_CHANNELS];
  uint16_t ADC2_inj_raw[ADC2_INJ_CHANNELS];
} ADC_Buffer_t;

//typedef struct {
//  uint32_t current_u;
//  uint32_t current_v;
//  uint32_t current_w;
//} Phase_Current_t;

typedef struct {
  arm_pid_instance_f32 Id_pid;
  arm_pid_instance_f32 Iq_pid;
  float Id_target, Iq_target;
} PID_t;

typedef struct {
  PID_t pid;
  float Ialpha, Ibeta;
  float Id, Iq;
  float Vd, Vq;
  float Valpha, Vbeta;
  uint32_t phU, phV, phW;
} FOC_t;

typedef struct {
  ADC_Buffer_t buffers;
  int32_t currents[3];      // current in mA
  uint32_t vbus;            // Vbus in mV
  uint16_t vdda;            // VDDA in mV
  uint16_t pot;
  float ntc_temp;           // temperature in degC
  bool running;
} ADC_Data_t;

typedef struct {
  ADC_Data_t adc;           // ADC data
  CS_Offset_t offsets;      // current sensing offsets
  FOC_t foc;
} Board_Settings_t;


#endif /* __TYPEDEFS_H */
