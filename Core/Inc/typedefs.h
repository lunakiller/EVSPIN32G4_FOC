/*
 * typedefs.h
 *
 *  Created on: Mar 11, 2024
 *      Author: lunakiller
 *
 */

#ifndef __TYPEDEFS_H
#define __TYPEDEFS_H

#include "settings.h"
#include "dsp/controller_functions.h"
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

typedef enum {
  STATE_ERROR = 0x0,
  STATE_IDLE,
  STATE_BOOTSTRAP,
  STATE_READY,
  STATE_ALIGNMENT,
  STATE_STARTUP,
  STATE_SYNCHRO,
  STATE_RUN
} FOC_State_t;

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
  float alpha;
  int last_out;
} LPF_int_t;

typedef struct {
  float alpha;
  float last_out;
} LPF_f32_t;

typedef struct {
  arm_pid_instance_f32 reg;
  int32_t target;
  int32_t max_output;
  bool saturated;
//  bool i_active;
} PID_t;

typedef struct {
  float el_position, mech_position, speed;
  int32_t speed_filtered, direction;
  uint16_t zero_angle_cnt;
  int32_t mech_pos_diff;
  float cnt_to_deg;
  LPF_int_t speed_lpf;
} FOC_Encoder_t;

typedef struct {
  int32_t phU, phV, phW;
  uint16_t maxCCR;
} Timer_t;

typedef struct {
  PID_t Id_pid;
  PID_t Iq_pid;
  PID_t speed_pid;
  float Ialpha, Ibeta;
  float Id, Iq;
  float angle;
  float Vd, Vq;
  float Valpha, Vbeta;
  Timer_t tim;
} FOC_t;

typedef struct {
  CS_Offset_t offsets;      // current sensing offsets
  ADC_Buffer_t buffers;
  int32_t currents[3];      // current in mA
  uint32_t vbus;            // Vbus in mV
  uint16_t vdda;            // VDDA in mV
  uint16_t pot;
  float ntc_temp;           // temperature in degC
} ADC_Data_t;

typedef struct {
  bool bootstrap_active;
  bool alignment_active;
  bool startup_active;
  bool synchro_active;
  bool run_active;
  bool speed_ramp_active;
  uint32_t clock;
} FOC_Base_t;

typedef struct {
  ADC_HandleTypeDef* adc1;
  ADC_HandleTypeDef* adc2;
  DMA_HandleTypeDef* dma_adc1;
  DMA_HandleTypeDef* dma_adc2;
  OPAMP_HandleTypeDef* opamp1;
  OPAMP_HandleTypeDef* opamp2;
  OPAMP_HandleTypeDef* opamp3;
  WWDG_HandleTypeDef* wwdg;
} FOC_Periph_t;

typedef struct {
	uint32_t voltage_u, voltage_v, voltage_w;
	float tmp;
	bool open_loop_enable;
	int open_loop_step;
} Debug_t;

typedef struct {
  FOC_State_t state;
  FOC_Base_t base;
  FOC_t foc;
  FOC_Encoder_t enc;
  ADC_Data_t adc;           // ADC data
  FOC_Periph_t periph;
  Debug_t dbg;
} Board_Settings_t;


#endif /* __TYPEDEFS_H */
