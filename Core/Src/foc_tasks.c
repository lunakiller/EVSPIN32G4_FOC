/*
 * foc_tasks.c
 *
 *  Created on: Mar 16, 2024
 *      Author: lunakiller
 *
 */

#include "foc_motorcontrol.h"

#include "swo_debug.h"
#include <stdlib.h>

// TODO pass a pointer?
extern Board_Settings_t evspin;
// TODO better watchdog
extern WWDG_HandleTypeDef hwwdg;


int32_t FOC_LeakyIntegrator_int(LPF_int_t* lpf, int input) {
  int out = lpf->alpha * lpf->last_out + (1 - lpf->alpha) * input;
  lpf->last_out = out;
  return out;
}

float FOC_LeakyIntegrator_f32(LPF_f32_t* lpf, float input) {
  float out = lpf->alpha * lpf->last_out + (1 - lpf->alpha) * input;
  lpf->last_out = out;
  return out;
}

float FOC_MovingAverage(float input) {
  static float buffer[FILTER_LENGTH] = {0};
  static uint8_t index = 0;
  float sum = 0;

  buffer[index] = input;
  index = (index + 1) % FILTER_LENGTH;

  for(uint8_t i = 0; i < FILTER_LENGTH; ++i) {
    sum += buffer[i];
  }

  return (sum / (float)FILTER_LENGTH);
}

int32_t FOC_LinearRamp(int32_t start, int32_t final, int32_t duration, float elapsed) {
  return (((final - start) * elapsed / (float)duration) + start);
}

// TODO move to the dedicated source file
/**
 * @brief Initialize PID and its variables
 */
void FOC_PID_Init(PID_t* pid, float Kp, float Ki, float Kd, int32_t limit) {
	pid->reg.Kp = Kp;
	pid->reg.Ki = Ki;
	pid->reg.Kd = Kd;
	arm_pid_init_f32(&pid->reg, 1);

	pid->max_output = limit;
	pid->saturated = false;
	pid->target = 0;
//	if(Ki != 0) {
//		pid->i_active = true;
//	}
//	else {
//		pid->i_active = false;
//	}
}

/**
 * @brief PID iteration.
 */
float FOC_PID(PID_t* pid, float error) {
  volatile float out = arm_pid_f32(&pid->reg, error);

  // saturation
  if(out > pid->max_output) {
  	// limit output
    out = pid->max_output;
    pid->reg.state[2] = pid->max_output;
    // disable integrator
//    pid->reg.Ki = 0;
//    arm_pid_init_f32(&pid->reg, 0);
    pid->saturated = true;
  }
  else if(out < -pid->max_output) {
  	// limit output
    out = -pid->max_output;
    pid->reg.state[2] = -pid->max_output;
    // disable integrator
//    pid->reg.Ki = 0;
//		arm_pid_init_f32(&pid->reg, 0);
    pid->saturated = true;
  }
  else {
    pid->saturated = false;
  }

  return out;
}

void FOC_SystickScheduler(void) {
  // ADC and necoder
  if(evspin.state > STATE_ERROR) {
    evspin.adc.vdda = (uint32_t)VREFINT_CAL_VREF * *VREFINT_CAL_ADDR / evspin.adc.buffers.ADC1_reg_raw[VREFINT_ADC1];
    evspin.adc.vbus = evspin.adc.buffers.ADC1_reg_raw[VBUS_ADC1] * evspin.adc.vdda / 4096.0f * VBUS_COEFF;
    evspin.adc.pot = evspin.adc.buffers.ADC2_reg_raw[POT_ADC2];
    // TODO recompute
    evspin.dbg.voltage_u = evspin.adc.buffers.ADC1_reg_raw[U_ADC1] * evspin.adc.vdda / 4096.0f * VMOT_COEFF;
    evspin.dbg.voltage_v= evspin.adc.buffers.ADC1_reg_raw[V_ADC1] * evspin.adc.vdda / 4096.0f * VMOT_COEFF;
    evspin.dbg.voltage_w = evspin.adc.buffers.ADC2_reg_raw[W_ADC2] * evspin.adc.vdda / 4096.0f * VMOT_COEFF;

    // TODO DEBUG - generate read events for trace
//    evspin.dbg.tmp[0] = evspin.foc.angle;
//    evspin.dbg.tmp[1] = evspin.enc.angle;
//    evspin.dbg.tmp[2] = evspin.foc.Iq;
//    evspin.dbg.tmp[3] = evspin.enc.speed_filtered;
//    evspin.dbg.tmp[0] = evspin.foc.Iq;
//    evspin.dbg.tmp[1] = evspin.mras.Iq_ada;
//    evspin.dbg.tmp[2] = evspin.foc.Valpha;
//    evspin.dbg.tmp[3] = evspin.foc.Vbeta;
    evspin.dbg.tmp[0] = evspin.foc.Vq;
    evspin.dbg.tmp[1] = evspin.foc.Vd;
  }


  switch(evspin.state) {
  case STATE_IDLE:
    break;
  case STATE_BOOTSTRAP:
    FOC_ChargeBootstraps();
    break;
  case STATE_READY:
    evspin.state = STATE_ALIGNMENT;
    break;
  case STATE_ALIGNMENT:
    FOC_AlignRotor();
    break;
  case STATE_STARTUP:
//    FOC_SpeedControl();
    break;
  case STATE_SYNCHRO:
//    FOC_SpeedControl();
    break;
  case STATE_RUN:
    FOC_SpeedControl();
    FOC_RunTask();
    break;
  default:
    break;
  }

  HAL_WWDG_Refresh(&hwwdg);
}

void FOC_ChargeBootstraps(void) {
  if(evspin.base.bootstrap_active == true) {
    if(HAL_GetTick() - evspin.base.clock > BOOTSTRAP_TIME) {
      LL_TIM_OC_SetCompareCH1(TIM1, (LL_TIM_GetAutoReload(TIM1) / 2));
      LL_TIM_OC_SetCompareCH2(TIM1, (LL_TIM_GetAutoReload(TIM1) / 2));
      LL_TIM_OC_SetCompareCH3(TIM1, (LL_TIM_GetAutoReload(TIM1) / 2));
      LL_TIM_GenerateEvent_UPDATE(TIM1);

      evspin.base.bootstrap_active = false;
      evspin.state = STATE_READY;
    }
  }
  else {
    evspin.base.clock = HAL_GetTick();
    evspin.base.bootstrap_active = true;

    LL_TIM_OC_SetCompareCH1(TIM1, 0);
    LL_TIM_OC_SetCompareCH2(TIM1, 0);
    LL_TIM_OC_SetCompareCH3(TIM1, 0);
    LL_TIM_GenerateEvent_UPDATE(TIM1);
    LL_TIM_EnableAllOutputs(TIM1);
  }
}

void FOC_AlignRotor(void) {
  if(evspin.base.alignment_active == true) {
    uint32_t elapsed = HAL_GetTick() - evspin.base.clock;
    if(elapsed > ALIGNMENT_TIME) {
      // TODO ENCODER
      evspin.enc.el_position = ALIGNMENT_ANGLE * (1 / evspin.enc.cnt_to_deg);
      evspin.enc.mech_position = evspin.enc.el_position / MOTOR_POLEPAIRS;
      LL_TIM_SetCounter(TIM4, evspin.enc.mech_position);
      LL_TIM_EnableCounter(TIM4);
      LL_TIM_ClearFlag_UPDATE(TIM4);

      evspin.foc.angle = ALIGNMENT_ANGLE;
      evspin.foc.Id_pid.target = 0;
      evspin.foc.Iq_pid.target = STARTUP_CURRENT;

      evspin.base.alignment_active = false;
      #if SENSORLESS == 1 || OPENLOOP_START == 1
        evspin.state = STATE_STARTUP;
      #else
        evspin.state = STATE_RUN;
      #endif
    }
    else if(elapsed < (ALIGNMENT_TIME / 2)) {
      evspin.foc.Id_pid.target = ALIGNMENT_CURRENT;
      evspin.foc.Id_pid.target = FOC_LinearRamp(0, ALIGNMENT_CURRENT, (ALIGNMENT_TIME/2), elapsed);
    }
  }
  else {
    evspin.base.clock = HAL_GetTick();
    evspin.base.alignment_active = true;

    evspin.foc.angle = ALIGNMENT_ANGLE;
  }
}

void FOC_EncoderProcessing(void) {
  uint32_t tim_cnt;
  int32_t act_mech_pos;
  int32_t mech_pos_diff = 0;

  evspin.enc.direction = LL_TIM_GetDirection(TIM4);

  tim_cnt = LL_TIM_GetCounter(TIM4);
  // counter overflowed?
  if(__LL_TIM_GETFLAG_UIFCPY(tim_cnt)) {
    LL_TIM_ClearFlag_UPDATE(TIM4);
    act_mech_pos = (tim_cnt & 0xFFFFFF);

    if(evspin.enc.direction == LL_TIM_COUNTERDIRECTION_UP) {
      mech_pos_diff = (ENCODER_PULSES * 2) - evspin.enc.mech_position + act_mech_pos;
    }
    else {
      mech_pos_diff = (act_mech_pos - (ENCODER_PULSES * 2)) - evspin.enc.mech_position;
    }
  }
  else {
    act_mech_pos = tim_cnt;
    mech_pos_diff = act_mech_pos - evspin.enc.mech_position;
  }

  if(mech_pos_diff < -(ENCODER_PULSES/2) || mech_pos_diff > (ENCODER_PULSES/2)) {
//    DEBUG_Breakpoint();
    DEBUG_print("DIFF\r\n");
    mech_pos_diff = 0;
  }

	evspin.enc.speed = (float)mech_pos_diff * (SWITCHING_FREQUENCY * 1000) * 60.0f / (ENCODER_PULSES * 2);
	evspin.enc.speed_filtered = FOC_LeakyIntegrator_int(&evspin.enc.speed_lpf, evspin.enc.speed);
//  evspin.enc.speed_filtered = FOC_MovingAverage(evspin.enc.speed);
  evspin.run.speed = evspin.enc.speed_filtered;

  evspin.enc.mech_pos_diff = mech_pos_diff;
  evspin.enc.mech_position = act_mech_pos;
  evspin.enc.el_position = act_mech_pos * MOTOR_POLEPAIRS;

  evspin.enc.angle = evspin.enc.el_position * evspin.enc.cnt_to_deg;

  // limit angle to the range [-180 * POLEPAIRS, 180 * POLEPAIRS]
  if(evspin.enc.angle >= 180.0f * MOTOR_POLEPAIRS) {
    evspin.enc.angle -= 360.0f * MOTOR_POLEPAIRS;
  }

  // TODO DEBUG
//  evspin.dbg.tmp1 = evspin.enc.mech_pos_diff * 256;
//	evspin.dbg.tmp2 = evspin.enc.speed_filtered;

  if(evspin.dbg.open_loop_enable == false) {
//    evspin.foc.angle = evspin.enc.angle;
  }

  if(evspin.enc.speed_filtered > 2000) {
    DEBUG_Breakpoint();
  }


//  // counter overflow
//  if(LL_TIM_IsActiveFlag_UPDATE(TIM4)) {
//    LL_TIM_ClearFlag_UPDATE(TIM4);
//    act_mech_pos = LL_TIM_GetCounter(TIM4) * evspin.enc.cnt_to_deg;
//    mech_pos_diff = (act_mech_pos - 360) - evspin.enc.mech_position;
//  }
//  else {
//    act_mech_pos = LL_TIM_GetCounter(TIM4) * evspin.enc.cnt_to_deg;
//    mech_pos_diff = act_mech_pos - evspin.enc.mech_position;
//  }
//
//  if(mech_pos_diff < -20 || mech_pos_diff > 20) {
////    DEBUG_Breakpoint();
//  }
//
//  act_el_pos = act_mech_pos * MOTOR_POLEPAIRS;
//
//  evspin.enc.mech_pos_diff = mech_pos_diff;
//  evspin.enc.speed = (mech_pos_diff / evspin.enc.speed_period) * (60.0 / 360.0);
//  evspin.enc.speed_filtered = FOC_LowPassFilter_int(&evspin.enc.speed_lpf, evspin.enc.speed);
//
//  evspin.enc.mech_position = act_mech_pos;
//  evspin.enc.el_position = act_el_pos;
//  evspin.foc.angle = act_el_pos;
}

void FOC_SpeedControl(void) {
  // PID regulator
  evspin.foc.Iq_pid.target = FOC_PID(&evspin.foc.speed_pid, evspin.foc.speed_pid.target - evspin.enc.speed_filtered);

  evspin.run.speed_target = evspin.foc.speed_pid.target;
}

void FOC_OpenLoop_StartUp(void) {
  if(evspin.base.startup_active == true) {
    evspin.open.elapsed_time_ms = HAL_GetTick() - evspin.base.clock;
    if(evspin.open.elapsed_time_ms < STARTUP_TIME) {
      evspin.open.actual_speed = evspin.open.accel_rate * evspin.open.elapsed_time_ms;

      evspin.open.angle_increment = evspin.open.actual_speed * 360.0f / 60.0f * _SWITCHING_PERIOD_MS / 1000.0f;
      evspin.open.angle += (evspin.open.angle_increment * MOTOR_POLEPAIRS);

      // limit angle to the range [-180 * POLEPAIRS, 180 * POLEPAIRS]
      if(evspin.open.angle >= 180.0f * MOTOR_POLEPAIRS) {
        evspin.open.angle -= 360.0f * MOTOR_POLEPAIRS;
      }
    }
    else {
      evspin.base.startup_active = false;
      evspin.state = STATE_SYNCHRO;
    }
  }
  else {
    evspin.base.clock = HAL_GetTick();
    evspin.base.startup_active = true;

    evspin.open.accel_rate = (float)STARTUP_SPEED / (float)STARTUP_TIME;      // rad/ms
    evspin.open.elapsed_time_ms = 0;

//    evspin.foc.angle = ALIGNMENT_ANGLE;
  }
}

void FOC_PositionSynchronization(void) {
  if(evspin.base.synchro_active == true) {
    if(HAL_GetTick() - evspin.base.clock < SYNCHRONIZATION_TIME) {
      evspin.open.angle += (evspin.open.angle_increment * MOTOR_POLEPAIRS);

      // limit angle to the range [-180 * POLEPAIRS, 180 * POLEPAIRS]
      if(evspin.open.angle >= 180.0f * MOTOR_POLEPAIRS) {
        evspin.open.angle -= 360.0f * MOTOR_POLEPAIRS;
      }

      if((abs((int)evspin.enc.angle - (int)evspin.open.angle) % 360) < 45) {
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
        evspin.base.synchro_active = false;
        evspin.state = STATE_RUN;
        return;
      }
    }
    else {
      evspin.base.synchro_active = false;
      // TODO synchro failed
      FOC_Stop();
      return;
    }
  }
  else {
    evspin.base.clock = HAL_GetTick();
    evspin.base.synchro_active = true;

    evspin.open.angle += (evspin.open.angle_increment * MOTOR_POLEPAIRS);
  }
}

void FOC_MRAS(void) {
  // TODO DEBUG
  evspin.mras.angle = evspin.foc.angle;
  evspin.mras.speed = evspin.run.speed;
  /* ADAPTIVE SYSTEM */
//  float tmpA, tmpB, tmpC;
//  arm_inv_clarke_f32(evspin.foc.Valpha, evspin.foc.Vbeta, &tmpA, &tmpB);
//
//  // convert from mV to V
//  tmpA /= 1000;
//  tmpB /= 1000;
//
//  float Ualpha, Ubeta;
//  // Clarke transform   TODO select phases
//  arm_clarke_f32(tmpA, tmpB, &Ualpha, &Ubeta);

  float sin, cos;
  arm_sin_cos_f32(evspin.mras.angle, &sin, &cos);

  // convert rpm to rad/s
  int32_t speed_rad = evspin.mras.speed * 2 * PI / 60.0f;

  float Ialpha_dot1, Ibeta_dot1;
  #if SIMPLE_EULER == 1
    // (Simple) Euler's method
    Ialpha_dot1 = evspin.mras.Ialpha + ((-evspin.mras.Ialpha * (_MRAS_MOTOR_R / _MRAS_MOTOR_L))
               + (speed_rad * sin * (MOTOR_BEMF_CONSTANT / _MRAS_MOTOR_L))
  //             + (Ualpha / _MRAS_MOTOR_L);
               + ((evspin.foc.Valpha / 1000) / _MRAS_MOTOR_L)) * (_SWITCHING_PERIOD_MS/1000);
    Ibeta_dot1 = evspin.mras.Ibeta + ((-evspin.mras.Ibeta * (_MRAS_MOTOR_R / _MRAS_MOTOR_L))
              + (speed_rad * cos * (MOTOR_BEMF_CONSTANT / _MRAS_MOTOR_L))
  //            + (Ubeta / _MRAS_MOTOR_L);
              + ((evspin.foc.Vbeta / 1000) / _MRAS_MOTOR_L)) * (_SWITCHING_PERIOD_MS/1000);

    evspin.mras.Ialpha = Ialpha_dot1;
    evspin.mras.Ibeta = Ibeta_dot1;
  #else
    // Modified Euler's method
    Ialpha_dot1 = ((-evspin.mras.Ialpha * (_MRAS_MOTOR_R / _MRAS_MOTOR_L))
                + (speed_rad * sin * (MOTOR_BEMF_CONSTANT / _MRAS_MOTOR_L))
                + ((evspin.foc.Valpha / 1000) / _MRAS_MOTOR_L));
    Ibeta_dot1 = ((-evspin.mras.Ibeta * (_MRAS_MOTOR_R / _MRAS_MOTOR_L))
               + (speed_rad * cos * (MOTOR_BEMF_CONSTANT / _MRAS_MOTOR_L))
               + ((evspin.foc.Vbeta / 1000) / _MRAS_MOTOR_L));

    float Ialpha_dot2, Ibeta_dot2;
    float Ialpha_pred = evspin.mras.Ialpha + (_SWITCHING_PERIOD_MS/1000) * Ialpha_dot1;
    float Ibeta_pred = evspin.mras.Ibeta + (_SWITCHING_PERIOD_MS/1000) * Ibeta_dot1;

    Ialpha_dot2 = ((-Ialpha_pred * (_MRAS_MOTOR_R / _MRAS_MOTOR_L))
                + (speed_rad * sin * (MOTOR_BEMF_CONSTANT / _MRAS_MOTOR_L))
                + ((evspin.foc.Valpha / 1000) / _MRAS_MOTOR_L));
    Ibeta_dot2 = ((-Ibeta_pred * (_MRAS_MOTOR_R / _MRAS_MOTOR_L))
               + (speed_rad * cos * (MOTOR_BEMF_CONSTANT / _MRAS_MOTOR_L))
               + ((evspin.foc.Vbeta / 1000) / _MRAS_MOTOR_L));

    evspin.mras.Ialpha += 0.5f * (_SWITCHING_PERIOD_MS/1000) * (Ialpha_dot1 + Ialpha_dot2);
    evspin.mras.Ibeta += 0.5f * (_SWITCHING_PERIOD_MS/1000) * (Ibeta_dot1 + Ibeta_dot2);
  #endif
//  // prevent overflow and underflow
//  if(Ialpha_dot1 > FLT_MAX)
//    Ialpha_dot1 = FLT_MAX;
//  else if(Ialpha_dot1 < FLT_MIN)
//    Ialpha_dot1 = FLT_MIN;
//  if(Ibeta_dot1 > FLT_MAX)
//    Ibeta_dot1 = FLT_MAX;
//  else if(Ibeta_dot1 < FLT_MIN)
//    Ibeta_dot1 = FLT_MIN;

  float Id_ada, Iq_ada;
  // Park transform
  arm_park_f32(evspin.mras.Ialpha, evspin.mras.Ibeta, &evspin.mras.Id_ada, &evspin.mras.Iq_ada, sin, cos);

  // TODO DEBUG
  evspin.mras.Iq_ada = FOC_LeakyIntegrator_f32(&evspin.dbg.mras_flt, evspin.mras.Iq_ada);

  /* REFERENCE SYSTEM */
  int32_t localCurrU = evspin.adc.currents[0];
  int32_t localCurrV = evspin.adc.currents[1];
  int32_t localCurrW = evspin.adc.currents[2];
  float Ialpha, Ibeta;
  // Clarke transform   TODO select phases
  arm_clarke_f32(localCurrU, localCurrV, &Ialpha, &Ibeta);

  // Park transform
  arm_park_f32(Ialpha, Ibeta, &evspin.mras.Id, &evspin.mras.Iq, sin, cos);

}

void FOC_RunTask(void) {
  if(evspin.run.activate_ramp == true) {
    if(evspin.base.speed_ramp_active == false) {
      evspin.run._ramp_initial = evspin.run.speed_target;
      evspin.run._ramp_elapsed = 0;

      evspin.base.speed_ramp_active = true;
    }
    else {
      evspin.run._ramp_elapsed += 1;

      if(evspin.run._ramp_elapsed == evspin.run.ramp_duration) {
        evspin.foc.speed_pid.target = evspin.run.ramp_final;

        evspin.base.speed_ramp_active = false;
        evspin.run.activate_ramp = false;
      }
      else {
        evspin.foc.speed_pid.target = FOC_LinearRamp(evspin.run._ramp_initial, evspin.run.ramp_final, evspin.run.ramp_duration, evspin.run._ramp_elapsed);
      }
    }
  }

  evspin.base.run_active = true;      // TODO ne tak casto
}

/**
 * @brief Main FOC task. Called after every current measurement conversion.
 */
void FOC_MainControl(void) {
  int32_t localCurrU = evspin.adc.currents[0];
  int32_t localCurrV = evspin.adc.currents[1];
  int32_t localCurrW = evspin.adc.currents[2];

  float sin, cos;

  switch(evspin.state) {
  case STATE_STARTUP:
  case STATE_SYNCHRO:
    evspin.foc.angle = evspin.open.angle;
    break;
  case STATE_RUN:
    evspin.foc.angle = evspin.enc.angle;
    break;
  default:
    break;
  }

  arm_sin_cos_f32(evspin.foc.angle, &sin, &cos);

  // Clarke transform
  arm_clarke_f32(localCurrU, localCurrV, &evspin.foc.Ialpha, &evspin.foc.Ibeta);

  // Park transform
  arm_park_f32(evspin.foc.Ialpha, evspin.foc.Ibeta, &evspin.foc.Id, &evspin.foc.Iq, sin, cos);

  // PID regulators
  evspin.foc.Vd = FOC_PID(&evspin.foc.Id_pid, evspin.foc.Id_pid.target - evspin.foc.Id);
  evspin.foc.Vq = FOC_PID(&evspin.foc.Iq_pid, evspin.foc.Iq_pid.target - evspin.foc.Iq);

  // TODO circle limitation

  // inverse Park transform
  arm_inv_park_f32(evspin.foc.Vd, evspin.foc.Vq, &evspin.foc.Valpha, &evspin.foc.Vbeta, sin, cos);

  FOC_Modulator(evspin.foc.Valpha, evspin.foc.Vbeta, &evspin.foc.tim.phU, &evspin.foc.tim.phV, &evspin.foc.tim.phW);

  LL_TIM_OC_SetCompareCH1(TIM1, evspin.foc.tim.phU);
  LL_TIM_OC_SetCompareCH2(TIM1, evspin.foc.tim.phV);
  LL_TIM_OC_SetCompareCH3(TIM1, evspin.foc.tim.phW);
  LL_TIM_GenerateEvent_UPDATE(TIM1);
}

/**
 * @brief Sinusoidal-PWM modulator function.
 * @param[in]  Valpha Alpha voltage in alpha-beta coordinates.
 * @param[in]  Vbeta Beta voltage in alpha-beta coordinates.
 * @param[out] Va CCRx register value representing phase A voltage.
 * @param[out] Vb CCRx register value representing phase B voltage.
 * @param[out] Vc CCRx register value representing phase C voltage.
 */
void FOC_Modulator(float Valpha, float Vbeta, int32_t* Va, int32_t* Vb, int32_t* Vc) {
  float tmpA, tmpB, tmpC;
  arm_inv_clarke_f32(Valpha, Vbeta, &tmpA, &tmpB);
  tmpC = -(tmpA + tmpB);

  // TODO deadtime compensation?

  *Va = tmpA * (int32_t)LL_TIM_GetAutoReload(TIM1) / (int32_t)(2 * evspin.adc.vbus) + ((int32_t)LL_TIM_GetAutoReload(TIM1) / 2);
  *Vb = tmpB * (int32_t)LL_TIM_GetAutoReload(TIM1) / (int32_t)(2 * evspin.adc.vbus) + ((int32_t)LL_TIM_GetAutoReload(TIM1) / 2);
  *Vc = tmpC * (int32_t)LL_TIM_GetAutoReload(TIM1) / (int32_t)(2 * evspin.adc.vbus) + ((int32_t)LL_TIM_GetAutoReload(TIM1) / 2);

  if(*Va > evspin.foc.tim.maxCCR)
    *Va = evspin.foc.tim.maxCCR;
  else if(*Va < 0)
    *Va = 0;
  if(*Vb > evspin.foc.tim.maxCCR)
    *Vb = evspin.foc.tim.maxCCR;
  else if(*Vb < 0)
    *Vb = 0;
  if(*Vc > evspin.foc.tim.maxCCR)
    *Vc = evspin.foc.tim.maxCCR;
  else if(*Vc < 0)
    *Vc = 0;

  // TODO DEBUG
//  evspin.dbg.tmp1 = tmpA / 8;
//  evspin.dbg.tmp2 = *Va / 4;

  return;
}
