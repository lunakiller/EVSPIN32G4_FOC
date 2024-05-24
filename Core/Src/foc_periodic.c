/*
 * foc_tasks.c
 *
 *  Procedures that are repeatedly executed while the motor is running.
 *
 *  Created on: Mar 16, 2024
 *      Author: lunakiller
 *
 */

#include "foc_periodic.h"

#include "swo_debug.h"
#include <stdlib.h>

// TODO pass a pointer?
extern Board_Settings_t evspin;
// TODO better watchdog
//extern WWDG_HandleTypeDef hwwdg;


/* --- LOCAL TOOLS */
/**
 * @brief Clamps a float value to given range.
 * @param f Value to be clamped
 * @param min Minimum value
 * @param max Maximum value
 * @retval Clamped value
 */
static inline float _clamp_f32(float f, float min, float max) {
  const float t = f < min ? min : f;
  return t > max ? max : t;
}

/**
 * @brief Returns the lesser of a and b.
 */
static inline float _min_f32(float a, float b) {
  return (a < b) ? a : b;
}

/**
 * @brief Helper function for _max3_i32().
 */
static inline int32_t _max_i32(int32_t a, int32_t b, uint8_t* id) {
  if(a > b) {
    *id += 0;
    return a;
  }
  else {
    *id += 1;
    return b;
  }
}

/**
 * @brief Finds a maximum of three numbers and returns that value and its index.
 * @param[in]  a,b,c Input values
 * @param[out] id Index of the maximum value
 * @retval Maxmimum value
 */
static inline int32_t _max3_i32(int32_t a, int32_t b, int32_t c, uint8_t* id) {
  *id = 0;
  return _max_i32(a, _max_i32(b, c, id), id);
}


/* --- HIGH-FREQUENCY TASK */
/**
 * @brief High-frequency periodic scheduler. Executes the main control loop.
 *        Trigger by the ADC interrupt handler.
 */
void FOC_HighFrequencyScheduler(void) {
  // DQ limits
  evspin.foc.limit = ((float)DQLIM_MAX_VOLTAGE / 100.0f) * evspin.adc.vbus;
  evspin.foc.limit_squared = evspin.foc.limit * evspin.foc.limit;

  switch(evspin.state) {
  case STATE_IDLE:
    break;
  case STATE_BOOTSTRAP:
    break;
  case STATE_READY:
    FOC_CurrentCompensation();
    FOC_MainControl();
    break;
  case STATE_ALIGNMENT:
    FOC_CurrentCompensation();
    FOC_MainControl();
    break;
  case STATE_STARTUP:
    FOC_OpenLoop_StartUp();
    FOC_EncoderProcessing();
    FOC_CurrentCompensation();
    FOC_MRAS();
    FOC_MainControl();
    break;
  case STATE_SYNCHRO:
    FOC_PositionSynchronization();
    FOC_EncoderProcessing();
    FOC_CurrentCompensation();
    FOC_MRAS();
    FOC_MainControl();
    break;
  case STATE_RUN:
    FOC_EncoderProcessing();
    FOC_CurrentCompensation();
    FOC_MRAS();
    FOC_MainControl();
    break;
  default:
    break;
  }
}

/* --- LOW_FREQUENCY TASK */
/**
 * @brief Low-frequency periodic scheduler.
 *        Trigger by the Systick interrupt handler.
 */
void FOC_SystickScheduler(void) {
  // convert ADC values
  if(evspin.state > STATE_ERROR) {
    evspin.adc.vdda = (uint32_t)VREFINT_CAL_VREF * *VREFINT_CAL_ADDR / evspin.adc.buffers.ADC1_reg_raw[VREFINT_ADC1];
    evspin.adc.vbus = evspin.adc.buffers.ADC1_reg_raw[VBUS_ADC1] * evspin.adc.vdda / 4096.0f * VBUS_COEFF;
    evspin.adc.pot = evspin.adc.buffers.ADC2_reg_raw[POT_ADC2];

    evspin.dbg.voltage_u = evspin.adc.buffers.ADC1_reg_raw[U_ADC1] * evspin.adc.vdda / 4096.0f * VMOT_COEFF;
    evspin.dbg.voltage_v= evspin.adc.buffers.ADC1_reg_raw[V_ADC1] * evspin.adc.vdda / 4096.0f * VMOT_COEFF;
    evspin.dbg.voltage_w = evspin.adc.buffers.ADC2_reg_raw[W_ADC2] * evspin.adc.vdda / 4096.0f * VMOT_COEFF;
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
    break;
  case STATE_SYNCHRO:
    break;
  case STATE_RUN:
    FOC_SpeedControl();
    FOC_RunTask();
    break;
  default:
    break;
  }

  // reset window watchdog
  LL_WWDG_SetCounter(WWDG, 95);
}


/* --- INDIVIDUAL PHASES */
/**
 * @brief Non-blocking function to handle charging of bootstrap capacitors.
 *        Executed by low-frequency scheduler.
 */
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

/**
 * @brief Non-blocking function to handle the rotor alignment to defined angle
 *        using a linear ramp to reduce oscillations.
 *        Also sets initial conditions for the sensorless algorithm, if enabled.
 *        Executed by low-frequency scheduler.
 */
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

#if SENSORLESS == 1
      evspin.open.angle = ALIGNMENT_ANGLE;
      evspin.mras.Id_ada = 0;
      evspin.mras.Iq_ada = STARTUP_CURRENT;
      evspin.mras.speed_mech_ada = 0;
      evspin.mras.angle_ada_deg = ALIGNMENT_ANGLE;
      evspin.mras.angle_ada = ALIGNMENT_ANGLE * 2 * PI / 360.0f;
#endif
#if OPENLOOP_START == 1
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

/**
 * @brief Non-blocking function to handle open-loop startup.
 *        Performs a linear acceleration of the motor.
 *        Executed by high-frequency scheduler.
 */
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
      else if(evspin.open.angle <= 180.0f * MOTOR_POLEPAIRS) {
        evspin.open.angle += 360.0f * MOTOR_POLEPAIRS;
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

/**
 * @brief Non-blocking function to handle the switch from open-loop to closed-loop
 *        control. Contains synchronization conditions.
 *        Executed by high-frequency scheduler.
 */
void FOC_PositionSynchronization(void) {
  if(evspin.base.synchro_active == true) {
    uint32_t elapsed = HAL_GetTick() - evspin.base.clock;
    if(elapsed < SYNCHRONIZATION_TIME) {
      evspin.open.angle += (evspin.open.angle_increment * MOTOR_POLEPAIRS);

      // limit angle to the range [-180 * POLEPAIRS, 180 * POLEPAIRS]
      if(evspin.open.angle >= 180.0f * MOTOR_POLEPAIRS) {
        evspin.open.angle -= 360.0f * MOTOR_POLEPAIRS;
      }

      // gradually lower q-axis current
      evspin.foc.Iq_pid.target = FOC_LinearRamp(STARTUP_CURRENT, (2 * STARTUP_CURRENT / 3), SYNCHRONIZATION_TIME, elapsed);
#if SENSORLESS == 1
      if((abs((int)evspin.mras.angle_ada_deg - (int)evspin.open.angle) % 360) < 45) {
        if(evspin.open.sync_cnt >= 5 && (abs((int)evspin.mras.speed_mech_ada - STARTUP_SPEED)) < 100) {
#else
      if((abs((int)evspin.enc.angle - (int)evspin.open.angle) % 360) < 45) {
        if(evspin.open.sync_cnt >= 5 && (abs((int)evspin.enc.speed_filtered - STARTUP_SPEED)) < 100) {
#endif
//          DEBUG_print("SYNC\r\n");
          HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
          evspin.base.synchro_active = false;
          evspin.state = STATE_RUN;

          // set scope trigger pin low
          HAL_GPIO_WritePin(DBG_TRG_GPIO_Port, DBG_TRG_Pin, GPIO_PIN_RESET);
          return;
        }
        evspin.open.sync_cnt++;
      }
      else {
        evspin.open.sync_cnt = 0;
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

    // set scope trigger pin high
    HAL_GPIO_WritePin(DBG_TRG_GPIO_Port, DBG_TRG_Pin, GPIO_PIN_SET);

    evspin.open.angle += (evspin.open.angle_increment * MOTOR_POLEPAIRS);
  }
}

/**
 * @brief Non-blocking function to handle ramps for changing speed during run phase.
 *        Executed by low-frequency scheduler.
 */
void FOC_RunTask(void) {
  if(evspin.run.activate_ramp == true) {
    if(evspin.base.speed_ramp_active == false) {
      evspin.run._ramp_initial = evspin.run.speed_target;
      evspin.run._ramp_elapsed = 0;

      evspin.base.speed_ramp_active = true;
    }
    else {
      evspin.run._ramp_elapsed += 1;

      if(evspin.run._ramp_elapsed >= evspin.run.ramp_duration) {
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

/* --- AUXILIARY FUNCTIONS */
/**
 * @brief Compute on of the currents based on the phase duty-cycle value
 *        to avoid wrong current measurements due to an already turned-off
 *        low-side switch.
 *        Executed by high-frequency scheduler.
 */
void FOC_CurrentCompensation(void) {
  // more than 75% duty cycle?
  if(evspin.foc.tim.phHighest > evspin.foc.tim.compute_phase_threshold) {
    // compute phase current with the highest duty-cycle
    switch(evspin.foc.tim.phHighest_id) {
      case 0:
        evspin.foc.currents_comp[0] = -(evspin.adc.currents[1] + evspin.adc.currents[2]);
        evspin.foc.currents_comp[1] = evspin.adc.currents[1];
        evspin.foc.currents_comp[2] = evspin.adc.currents[2];
        break;
      case 1:
        evspin.foc.currents_comp[0] = evspin.adc.currents[0];
        evspin.foc.currents_comp[1] = -(evspin.adc.currents[0] + evspin.adc.currents[2]);
        evspin.foc.currents_comp[2] = evspin.adc.currents[2];
        break;
      case 2:
        evspin.foc.currents_comp[0] = evspin.adc.currents[0];
        evspin.foc.currents_comp[1] = evspin.adc.currents[1];
        evspin.foc.currents_comp[2] = -(evspin.adc.currents[0] + evspin.adc.currents[1]);
        break;
      default:
        evspin.foc.currents_comp[0] = 0;
        evspin.foc.currents_comp[1] = 0;
        evspin.foc.currents_comp[2] = 0;
        // should not happen
        FOC_Stop();
        break;
    }
  }
  else {
    // take raw current measurements
    evspin.foc.currents_comp[0] = evspin.adc.currents[0];
    evspin.foc.currents_comp[1] = evspin.adc.currents[1];
    evspin.foc.currents_comp[2] = evspin.adc.currents[2];
  }
}

/**
 * @brief Process the rotary encoder output to obtain speed and angle.
 *        Executed by high-frequency scheduler.
 */
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

  evspin.enc.speed = (float)mech_pos_diff * (SWITCHING_FREQUENCY * 1000) * 60.0f / (ENCODER_PULSES * 2);
  evspin.enc.speed_filtered = FOC_LeakyIntegrator_int(&evspin.enc.speed_lpf, evspin.enc.speed);
  evspin.run.speed = evspin.enc.speed_filtered;

  evspin.enc.mech_pos_diff = mech_pos_diff;
  evspin.enc.mech_position = act_mech_pos;
  evspin.enc.el_position = act_mech_pos * MOTOR_POLEPAIRS;

  evspin.enc.angle = evspin.enc.el_position * evspin.enc.cnt_to_deg;

  // limit angle to the range [-180 * POLEPAIRS, 180 * POLEPAIRS]
  if(evspin.enc.angle >= 180.0f * MOTOR_POLEPAIRS) {
    evspin.enc.angle -= 360.0f * MOTOR_POLEPAIRS;
  }
  else if(evspin.enc.angle <= 180.0f * MOTOR_POLEPAIRS) {
    evspin.enc.angle += 360.0f * MOTOR_POLEPAIRS;
  }
}


/* --- MOTOR CONTROL */
/**
 * @brief Main FOC task. Called after every current measurement conversion.
 */
void FOC_MainControl(void) {
  switch(evspin.state) {
  case STATE_STARTUP:
  case STATE_SYNCHRO:
    evspin.foc.angle = evspin.open.angle;
    break;
  case STATE_RUN:
#if SENSORLESS == 1
    evspin.foc.angle = evspin.mras.angle_ada_deg;
#else
    evspin.foc.angle = evspin.enc.angle;

    if(evspin.dbg.force_sensorless) {
    	evspin.foc.angle = evspin.mras.angle_ada_deg;
    }
#endif
    break;
  default:
    break;
  }

  float sin, cos;
  // TODO use CORDIC
  arm_sin_cos_f32(evspin.foc.angle, &sin, &cos);

  // Clarke transform
  arm_clarke_f32(evspin.foc.currents_comp[0], evspin.foc.currents_comp[1], &evspin.foc.Ialpha, &evspin.foc.Ibeta);

  // Park transform
  arm_park_f32(evspin.foc.Ialpha, evspin.foc.Ibeta, &evspin.foc.Id, &evspin.foc.Iq, sin, cos);

  // PID regulators
  evspin.foc.Vd = FOC_PID(&evspin.foc.Id_pid, evspin.foc.Id_pid.target - evspin.foc.Id);
  evspin.foc.Vq = FOC_PID(&evspin.foc.Iq_pid, evspin.foc.Iq_pid.target - evspin.foc.Iq);

  FOC_DQ_Limiter();

  // inverse Park transform
  arm_inv_park_f32(evspin.foc.Vd_sat, evspin.foc.Vq_sat, &evspin.foc.Valpha, &evspin.foc.Vbeta, sin, cos);

  FOC_Modulator(evspin.foc.Valpha, evspin.foc.Vbeta, &evspin.foc.tim.phU, &evspin.foc.tim.phV, &evspin.foc.tim.phW);

  LL_TIM_OC_SetCompareCH1(TIM1, evspin.foc.tim.phU);
  LL_TIM_OC_SetCompareCH2(TIM1, evspin.foc.tim.phV);
  LL_TIM_OC_SetCompareCH3(TIM1, evspin.foc.tim.phW);
  LL_TIM_GenerateEvent_UPDATE(TIM1);

  evspin.foc.tim.phHighest = _max3_i32(evspin.foc.tim.phU, evspin.foc.tim.phV, evspin.foc.tim.phW, &evspin.foc.tim.phHighest_id);
}

/**
 * @brief Model Reference Adaptive System observer.
 */
void FOC_MRAS(void) {
  // TODO DEBUG
  evspin.mras.angle = evspin.foc.angle;
  if(evspin.state == STATE_RUN) {
    evspin.mras.speed = evspin.run.speed;   // only to compare
  }

  /* ADAPTIVE SYSTEM */
  float sin, cos;
  arm_sin_cos_f32(evspin.mras.angle_ada_deg, &sin, &cos);
//  arm_sin_cos_f32(evspin.mras.angle, &sin, &cos);

  // convert rpm to rad/s
  int32_t speed_mech_rad = evspin.mras.speed_mech_ada * 2 * PI / 60.0f;

//  float Id_dot1, Iq_dot1;
//#if SIMPLE_EULER == 1
//  // (Simple) Euler's method
//  Id_dot1 = evspin.mras.Id_ada + ((evspin.mras.Id_ada * (-_MRAS_MOTOR_R / _MRAS_MOTOR_L))
//             + (speed_mech_rad * MOTOR_POLEPAIRS * evspin.mras.Iq_ada)
//             + ((evspin.foc.Vd_sat /*/ 1000*/) * (1 / _MRAS_MOTOR_L))) * (_SWITCHING_PERIOD_MS/1000);
//  Iq_dot1 = evspin.mras.Iq_ada + ((evspin.mras.Iq_ada * (-_MRAS_MOTOR_R / _MRAS_MOTOR_L))
//            - (speed_mech_rad * MOTOR_POLEPAIRS * evspin.mras.Id_ada)
//            - (speed_mech_rad * ((MOTOR_BEMF_CONSTANT * 1000) * (1 / _MRAS_MOTOR_L)))
//            + ((evspin.foc.Vq_sat /*/ 1000*/) * (1 / _MRAS_MOTOR_L))) * (_SWITCHING_PERIOD_MS/1000);
//
//  evspin.mras.Id_ada = Id_dot1;
//  evspin.mras.Iq_ada = Iq_dot1;
//#else
//  // Modified Euler's method
//  Id_dot1 = ((evspin.mras.Id_ada * (-_MRAS_MOTOR_R / _MRAS_MOTOR_L))
//          + (speed_mech_rad * MOTOR_POLEPAIRS * evspin.mras.Iq_ada)
//          + ((evspin.foc.Vd_sat /*/ 1000*/) * (1 / _MRAS_MOTOR_L)));
//  Iq_dot1 = ((evspin.mras.Iq_ada * (-_MRAS_MOTOR_R / _MRAS_MOTOR_L))
//          - (speed_mech_rad * MOTOR_POLEPAIRS * evspin.mras.Id_ada)
//          - (speed_mech_rad * ((MOTOR_BEMF_CONSTANT * 1000) * (1 / _MRAS_MOTOR_L)))
//          + ((evspin.foc.Vq_sat /*/ 1000*/) * (1 / _MRAS_MOTOR_L)));
//
//  float Id_dot2, Iq_dot2;
//  float Id_pred = evspin.mras.Id_ada + (_SWITCHING_PERIOD_MS/1000) * Id_dot1;
//  float Iq_pred = evspin.mras.Iq_ada + (_SWITCHING_PERIOD_MS/1000) * Iq_dot1;
//
//  Id_dot2 = ((Id_pred * (-_MRAS_MOTOR_R / _MRAS_MOTOR_L))
//          + (speed_mech_rad * MOTOR_POLEPAIRS * Iq_pred)
//          + ((evspin.foc.Vd_sat /*/ 1000*/) * (1 / _MRAS_MOTOR_L)));
//  Iq_dot2 = ((Iq_pred * (-_MRAS_MOTOR_R / _MRAS_MOTOR_L))
//          - (speed_mech_rad * MOTOR_POLEPAIRS * Id_pred)
//          - (speed_mech_rad * ((MOTOR_BEMF_CONSTANT * 1000) * (1 / _MRAS_MOTOR_L)))
//          + ((evspin.foc.Vq_sat /*/ 1000*/) * (1 / _MRAS_MOTOR_L)));
//
//  evspin.mras.Id_ada += (0.5f * (_SWITCHING_PERIOD_MS/1000)) * (Id_dot1 + Id_dot2);
//  evspin.mras.Iq_ada += (0.5f * (_SWITCHING_PERIOD_MS/1000)) * (Iq_dot1 + Iq_dot2);
//#endif

  float Ialpha_dot1, Ibeta_dot1;
#if SIMPLE_EULER == 1
  // (Simple) Euler's method
  Ialpha_dot1 = evspin.mras.Ialpha + ((evspin.mras.Ialpha * (-_MRAS_MOTOR_R / _MRAS_MOTOR_L))
             + (speed_mech_rad * sin * ((MOTOR_BEMF_CONSTANT * 1000) * (1 / _MRAS_MOTOR_L)))
             + ((evspin.foc.Valpha /*/ 1000*/) * (1 / _MRAS_MOTOR_L))) * (_SWITCHING_PERIOD_MS/1000);
  Ibeta_dot1 = evspin.mras.Ibeta + ((evspin.mras.Ibeta * (-_MRAS_MOTOR_R / _MRAS_MOTOR_L))
            - (speed_mech_rad * cos * ((MOTOR_BEMF_CONSTANT * 1000) * (1 / _MRAS_MOTOR_L)))
            + ((evspin.foc.Vbeta /*/ 1000*/) * (1 / _MRAS_MOTOR_L))) * (_SWITCHING_PERIOD_MS/1000);

  evspin.mras.Ialpha = Ialpha_dot1;
  evspin.mras.Ibeta = Ibeta_dot1;
#else
  // Modified Euler's method
  Ialpha_dot1 = ((evspin.mras.Ialpha * (-_MRAS_MOTOR_R / _MRAS_MOTOR_L))
              + (speed_mech_rad * sin * ((MOTOR_BEMF_CONSTANT * 1000) * (1 / _MRAS_MOTOR_L)))
              + ((evspin.foc.Valpha /*/ 1000*/) * (1 / _MRAS_MOTOR_L)));
  Ibeta_dot1 = ((evspin.mras.Ibeta * (-_MRAS_MOTOR_R / _MRAS_MOTOR_L))
             - (speed_mech_rad * cos * ((MOTOR_BEMF_CONSTANT * 1000) * (1 / _MRAS_MOTOR_L)))
             + ((evspin.foc.Vbeta /*/ 1000*/) * (1 / _MRAS_MOTOR_L)));

  float Ialpha_dot2, Ibeta_dot2;
  float Ialpha_pred = evspin.mras.Ialpha + (_SWITCHING_PERIOD_MS/1000) * Ialpha_dot1;
  float Ibeta_pred = evspin.mras.Ibeta + (_SWITCHING_PERIOD_MS/1000) * Ibeta_dot1;

  Ialpha_dot2 = ((Ialpha_pred * (-_MRAS_MOTOR_R / _MRAS_MOTOR_L))
              + (speed_mech_rad * sin * ((MOTOR_BEMF_CONSTANT * 1000) * (1 / _MRAS_MOTOR_L)))
              + ((evspin.foc.Valpha /*/ 1000*/) * (1 / _MRAS_MOTOR_L)));
  Ibeta_dot2 = ((Ibeta_pred * (-_MRAS_MOTOR_R / _MRAS_MOTOR_L))
             - (speed_mech_rad * cos * ((MOTOR_BEMF_CONSTANT * 1000) * (1 / _MRAS_MOTOR_L)))
             + ((evspin.foc.Vbeta /*/ 1000*/) * (1 / _MRAS_MOTOR_L)));

  evspin.mras.Ialpha += (0.5f * (_SWITCHING_PERIOD_MS/1000)) * (Ialpha_dot1 + Ialpha_dot2);
  evspin.mras.Ibeta += (0.5f * (_SWITCHING_PERIOD_MS/1000)) * (Ibeta_dot1 + Ibeta_dot2);
#endif

  // Park transform
  arm_park_f32(evspin.mras.Ialpha, evspin.mras.Ibeta, &evspin.mras.Id_ada, &evspin.mras.Iq_ada, sin, cos);


  /* REFERENCE SYSTEM */
  float Ialpha, Ibeta;
  // Clarke transform   TODO dont compute twice
  arm_clarke_f32(evspin.foc.currents_comp[0], evspin.foc.currents_comp[1], &Ialpha, &Ibeta);

  // Park transform
  arm_park_f32(Ialpha, Ibeta, &evspin.mras.Id, &evspin.mras.Iq, sin, cos);


  /* ADAPTATION LAW */
  evspin.mras.speed_error = ((evspin.mras.Id / 1000) * (evspin.mras.Iq_ada / 1000))
                        - ((evspin.mras.Id_ada / 1000) * (evspin.mras.Iq / 1000))
                        - (((evspin.mras.Iq / 1000) - (evspin.mras.Iq_ada / 1000))
                            * (MOTOR_BEMF_CONSTANT / MOTOR_POLEPAIRS) * (1 / _MRAS_MOTOR_L));

//  float speed_el_prev = evspin.mras.speed_el_ada;       // TODO TEST!
  // speed in rad/s
  evspin.mras.speed_el_ada = FOC_PID(&evspin.mras.omega_pid, evspin.mras.speed_error);

//  float angle_tmp = 0.5 * (evspin.mras.speed_el_ada + speed_el_prev);
//  evspin.mras.angle_ada += angle_tmp * (_SWITCHING_PERIOD_MS/1000);     // TODO TEST!
  evspin.mras.angle_ada += evspin.mras.speed_el_ada * (_SWITCHING_PERIOD_MS/1000);

  // limit angle to the range [-180 * POLEPAIRS, 180 * POLEPAIRS]
  if(evspin.mras.angle_ada >= PI * MOTOR_POLEPAIRS) {
    evspin.mras.angle_ada -= 2 * PI * MOTOR_POLEPAIRS;
  }
  else if(evspin.mras.angle_ada <= PI * MOTOR_POLEPAIRS) {
    evspin.mras.angle_ada += 2 * PI * MOTOR_POLEPAIRS;
  }
  // convert angle from radians to degrees
  evspin.mras.angle_ada_deg = evspin.mras.angle_ada * (360.0f / (2 * PI));

  // convert rad/s to rpm   // TODO redundant?
  evspin.mras.speed_el_ada *= 60.0f / (2 * PI);
  // convert to mechanical speed
  evspin.mras.speed_mech_ada = evspin.mras.speed_el_ada * (1.0f / MOTOR_POLEPAIRS);
}

/**
 * @brief DQ Limiter (also called Circle limitation) with priority on the D-axis.
 */
void FOC_DQ_Limiter(void) {
  float d_squared, q_squared, sum_squared, Vq_limit_squared;

  d_squared = evspin.foc.Vd * evspin.foc.Vd;
  q_squared = evspin.foc.Vq * evspin.foc.Vq;
  // squared magnitude
  sum_squared = d_squared + q_squared;

  if(sum_squared > evspin.foc.limit_squared) {
    // d-axis voltage clamping
    if(d_squared > evspin.foc.limit_squared) {
      evspin.foc.Vd_sat = evspin.foc.limit;
      d_squared = evspin.foc.limit_squared;

      if(evspin.foc.Vd < 0) {
        evspin.foc.Vd_sat = -evspin.foc.Vd_sat;
      }
    }
    else {
      evspin.foc.Vd_sat = evspin.foc.Vd;
    }

    // compute the squared limit for Vq
    Vq_limit_squared = evspin.foc.limit_squared - d_squared;

    // q-axis voltage clamping
    if (q_squared > Vq_limit_squared) {
      evspin.foc.Vq_sat = sqrtf(Vq_limit_squared);

      if (evspin.foc.Vq < 0) {
        evspin.foc.Vq_sat = -evspin.foc.Vq_sat;
      }
    } else {
      evspin.foc.Vq_sat = evspin.foc.Vq;
    }

    evspin.base.dq_limiter_active = true;
  }
  else {
    evspin.foc.Vd_sat = evspin.foc.Vd;
    evspin.foc.Vq_sat = evspin.foc.Vq;

    evspin.base.dq_limiter_active = false;
  }
}

/**
 * @brief Speed PID controller.
 */
void FOC_SpeedControl(void) {
  // PID regulator
#if SENSORLESS == 1
  evspin.foc.Iq_pid.target = FOC_PID(&evspin.foc.speed_pid, evspin.foc.speed_pid.target - evspin.mras.speed_mech_ada);
#else
  if(!evspin.dbg.force_sensorless) {
    evspin.foc.Iq_pid.target = FOC_PID(&evspin.foc.speed_pid, evspin.foc.speed_pid.target - evspin.enc.speed_filtered);
  }
  else {
    evspin.foc.Iq_pid.target = FOC_PID(&evspin.foc.speed_pid, evspin.foc.speed_pid.target - evspin.mras.speed_mech_ada);
  }
#endif
  evspin.run.speed_target = evspin.foc.speed_pid.target;
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

  *Va = tmpA * (int32_t)LL_TIM_GetAutoReload(TIM1) / (int32_t)(2 * evspin.adc.vbus) + ((int32_t)LL_TIM_GetAutoReload(TIM1) / 2);
  *Vb = tmpB * (int32_t)LL_TIM_GetAutoReload(TIM1) / (int32_t)(2 * evspin.adc.vbus) + ((int32_t)LL_TIM_GetAutoReload(TIM1) / 2);
  *Vc = tmpC * (int32_t)LL_TIM_GetAutoReload(TIM1) / (int32_t)(2 * evspin.adc.vbus) + ((int32_t)LL_TIM_GetAutoReload(TIM1) / 2);

  if(*Va > evspin.foc.tim.maxCCR)
    *Va = evspin.foc.tim.maxCCR;
  else if(*Va < DQLIM_CCR_LIMIT)
    *Va = DQLIM_CCR_LIMIT;
  if(*Vb > evspin.foc.tim.maxCCR)
    *Vb = evspin.foc.tim.maxCCR;
  else if(*Vb < DQLIM_CCR_LIMIT)
    *Vb = DQLIM_CCR_LIMIT;
  if(*Vc > evspin.foc.tim.maxCCR)
    *Vc = evspin.foc.tim.maxCCR;
  else if(*Vc < DQLIM_CCR_LIMIT)
    *Vc = DQLIM_CCR_LIMIT;

  return;
}
