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

// FOC state machine
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

// structure with a bit more information about the internal state
typedef struct {
  bool bootstrap_active;
  bool alignment_active;
  bool startup_active;
  bool synchro_active;
  bool run_active;
  bool dq_limiter_active;
  bool speed_ramp_active;
  uint32_t clock;                                           // internal variable - do not use
} FOC_Base_t;

// structure for control of the running motor
typedef struct {
  int32_t speed_target;                                     // speed controller target speed
  int32_t speed;                                            // actual speed
  int32_t ramp_final;                                       // target of the speed ramp
  int32_t ramp_duration;                                    // duration of the ramp
  bool activate_ramp;                                       // set to true to execute the ramp
  int32_t _ramp_initial;                                    // internal variable - do not use
  float _ramp_elapsed;                                      // internal variable - do not use
} FOC_Run_t;

// opamp offset values
typedef struct {
  uint16_t phase_u_offset;
  uint16_t phase_v_offset;
  uint16_t phase_w_offset;
} CS_Offset_t;

// buffers for ADCs
typedef struct {
  uint16_t ADC1_reg_raw[ADC1_REG_CHANNELS];
  uint16_t ADC2_reg_raw[ADC2_REG_CHANNELS];
  uint16_t ADC1_inj_raw[ADC1_INJ_CHANNELS];
  uint16_t ADC2_inj_raw[ADC2_INJ_CHANNELS];
} ADC_Buffer_t;

// ADC structure
typedef struct {
  CS_Offset_t offsets;                                      // current sensing offsets
  ADC_Buffer_t buffers;
  int32_t currents[3];                                      // current in mA
  uint32_t vbus;                                            // Vbus in mV
  uint16_t vdda;                                            // VDDA in mV
  uint16_t pot;                                             // potentiometer value (not implemented yet)
  float ntc_temp;                                           // temperature in degC (not implemented yet)
} ADC_Data_t;

// integer structure for leaky integrator filter
typedef struct {
  float alpha;
  int last_out;
} LPF_int_t;

// float structure for leaky integrator filter
typedef struct {
  float alpha;
  float last_out;
} LPF_f32_t;

// PID controller structure
typedef struct {
  arm_pid_instance_f32 reg;
  int32_t target;
  int32_t max_output;
  bool saturated;
} PID_t;

// structure for rotary encoder
typedef struct {
  float el_position, mech_position, angle, speed;
  int32_t speed_filtered, direction;
  uint16_t zero_angle_cnt;                                  // counter value when zero signal was detected
  int32_t mech_pos_diff;
  float cnt_to_deg;                                         // constant to convert from counter value to degrees
  LPF_int_t speed_lpf;                                      // low-pass filter structure for speed
} FOC_Encoder_t;

// PWM timer-related variables
typedef struct {
  int32_t phU, phV, phW;                                    // outputs from modulator in counter values
  uint16_t maxCCR;                                          // maximum counter value
  uint16_t compute_phase_threshold;                         // threshold counter value for current compensation
  uint16_t phHighest;                                       // highest duty-cycle
  uint8_t phHighest_id;                                     // phase with the highest duty-cycle
} Timer_t;

// structure with field-oriented control-related values
typedef struct {
  PID_t Id_pid;
  PID_t Iq_pid;
  PID_t speed_pid;
  int32_t currents_comp[3];                                 // duty-cycle compensated current in mA
  float Ialpha, Ibeta;
  float Id, Iq;
  float angle;
  float Vd, Vq;
  float Vd_sat, Vq_sat;                                     // saturated voltages from DQ limiter
  float Valpha, Vbeta;
  float limit, limit_squared;                               // DQ limtier-related variables
  Timer_t tim;
} FOC_t;

// internal variables for open-loop acceleration ramp calculation
typedef struct {
  float accel_rate;
  float actual_speed;
  float angle_increment;
  float elapsed_time_ms;
  float angle;
  uint8_t sync_cnt;
} FOC_OpenLoop_t;

// structure with MRAS observer-related variables
typedef struct {
  float Ialpha, Ibeta;
  float Id, Iq;
  float Id_ada, Iq_ada;
  float speed_error;
  float speed_mech_ada, speed_el_ada;
  float angle_ada, angle_ada_deg;
  PID_t omega_pid;
} FOC_Observer_t;

// struture with pointers to peripherals controlled with HAL
typedef struct {
  ADC_HandleTypeDef* adc1;
  ADC_HandleTypeDef* adc2;
  DMA_HandleTypeDef* dma_adc1;
  DMA_HandleTypeDef* dma_adc2;
  OPAMP_HandleTypeDef* opamp1;
  OPAMP_HandleTypeDef* opamp2;
  OPAMP_HandleTypeDef* opamp3;
//  WWDG_HandleTypeDef* wwdg;
} FOC_Periph_t;

// debug structure
typedef struct {
	uint32_t voltage_u, voltage_v, voltage_w;
	float tmp[4];
	int32_t dac1, dac2, dbg_opt;
	bool force_sensorless;
	float Kp, Ki;
} Debug_t;

// THE MAIN STRUCTURE
typedef struct {
  FOC_State_t state;
  FOC_Base_t base;
  FOC_Run_t run;
  FOC_t foc;
  FOC_OpenLoop_t open;
  FOC_Observer_t mras;
  FOC_Encoder_t enc;
  ADC_Data_t adc;
  FOC_Periph_t periph;
  Debug_t dbg;
} Board_Settings_t;


#endif /* __TYPEDEFS_H */
