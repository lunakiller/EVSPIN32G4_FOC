/*
 * settings.h
 *
 *  Created on: Mar 4, 2024
 *      Author: lunakiller
 *
 */

#ifndef __SETTINGS_H
#define __SETTINGS_H

/* Constant definitions */
#ifndef PI
  #define PI               3.14159265358979f
#endif

/* Helper macros */
// convert Back-EMF constant from rpm/V to V.s/rad
#define BEMF_KV_TO_KE(x)            (60.0f / (2 * PI * x))


/* Library settings */
#define SENSORLESS                  (0)                            // 1 - Sensroless, 0 - Encoder
#define OPENLOOP_START              (0)                            // 1 - enable open-loop start-up, 0 - disable
#define SIMPLE_EULER                (0)                            // solve ODE using simple Euler's method (faster, less accurate)

// General
#define SWITCHING_FREQUENCY         ((uint8_t)  30)                // switching frequency in kHz
#define DEAD_TIME                   ((uint16_t) 500)               // deadtime in ns

// Motor parameters
#define MOTOR_POLEPAIRS             ((uint8_t)  7)

// Settings for the sensorless mode
#define MOTOR_TERMINAL_RESISTANCE   ((float)    1.01)               // in Ohms
#define MOTOR_TERMINAL_INDUCTANCE   ((float)    0.995)              // in mH
#define MOTOR_BEMF_CONSTANT         ((float)    BEMF_KV_TO_KE(105)) // in V.s/rad (k_e)
#define MOTOR_ROTOR_INERTIA         ()

// Encoder parameters
#define ENCODER_PULSES              ((uint16_t) 1024)

// Protections
#define UNDERVOLTAGE_THRESHOLD      ((uint8_t)  10)                 // undervoltage protection threshold in V
#define OVERVOLTAGE_THRESHOLD       ((uint8_t)  35)                 // overvoltage protection threshold in V
#define SW_OVERCURRENT_THRESHOLD    ((uint16_t) 2000)               // overcurrent protection threshold in mA

// Bootstrap charging phase settings
#define BOOTSTRAP_TIME              ((uint16_t) 250)                // time to charge bootstrap capacitors in ms

// Alignment phase settings
#define ALIGNMENT_TIME              ((uint16_t) 1000)               // alignment phase length in ms
#define ALIGNMENT_CURRENT           ((int16_t)  1000)               // d-axis current used to align the rotor in mA
#define ALIGNMENT_ANGLE             ((int16_t)  110)                 // alignment electrical angle [-180  179]

#define STARTUP_CURRENT             ((int16_t)  500)               // d-axis current applied to start rotating in mA
//#if OPENLOOP_START == 1
#define STARTUP_TIME                ((uint16_t) 500)                // startup phase length in ms
#define STARTUP_SPEED               ((int16_t)  400)
//#endif

#define SYNCHRONIZATION_TIME        ((uint16_t) 1000)               // position synchronization phase length in ms

// moving average filter
#define FILTER_LENGTH               ((uint8_t)  32)                 // length of the moving average filter

// speed PID
#define PID_SPEED_KP                ((float)    1)
#define PID_SPEED_KI                ((float)    0.02)
#define PID_SPEED_KD                ((float)    0)
#define PID_SPEED_LIMIT             ((float)    SW_OVERCURRENT_THRESHOLD)        // TODO

// D-Q current PIDs
#define PID_CURRENT_KP							((float)    16)						// 1
#define PID_CURRENT_KI							((float)    1)						// 0.1
#define PID_CURRENT_KD							((float)    0)
#define PID_CURRENT_LIMIT						((float)    35000)				// TODO

// VBUS divider
#define VBUS_R_VtoADC               ((uint32_t) 72300)              // resistor between VM and ADC input in Ohms
#define VBUS_R_ADCtoGND             ((uint32_t) 3010)               // resistor between ADC input and GND in Ohms
#define VBUS_COEFF                  ((float)    _VBUS_COEFF)        // coefficient to compute VBUS from ADC

// Applied motor voltage divider
#define VMOT_R_VtoADC               ((uint32_t) 95000)              // resistor between VM and ADC input in Ohms
#define VMOT_R_ADCtoGND             ((uint32_t) 3900)               // resistor between ADC input and GND in Ohms
#define VMOT_COEFF                  ((float)    _VMOT_COEFF)        // coefficient to compute applied voltages from ADC

// Current sensing
#define CS_SHUNT_VALUE              ((float)    0.02)              // shunt in Ohms
#define CS_R_SHUNTNtoOPN            ((uint32_t) 1500)               // resistor between negative shunt output and negative OPAMP input in Ohms
#define CS_R_OPNtoOPO               ((uint32_t) 11000)              // resistor between negative OPAMP input and OPAMP output in Ohms
#define CS_GAIN                     ((float)    _CS_GAIN)           // current sensing amplifier gain

// ADC settings
#define ADC1_REG_CHANNELS           ((uint8_t)  4)
#define ADC1_INJ_CHANNELS           ((uint8_t)  2)
#define ADC2_REG_CHANNELS           ((uint8_t)  3)
#define ADC2_INJ_CHANNELS           ((uint8_t)  1)

#define CS_OFFSET_CALIB_AVG         ((uint8_t) 16)                  // number of samples for averaging



#define VBUS_ADC1_CHANNEL           ADC_CHANNEL_1
#define U_ADC1_CHANNEL              ADC_CHANNEL_6
#define V_ADC1_CHANNEL              ADC_CHANNEL_7
#define W_ADC2_CHANNEL              ADC_CHANNEL_8
#define CS_U_ADC1_CHANNEL           ADC_CHANNEL_3
#define CS_V_ADC2_CHANNEL           ADC_CHANNEL_3
#define CS_W_ADC1_CHANNEL           ADC_CHANNEL_12
#define NTC_ADC2_CHANNEL            ADC_CHANNEL_5
#define POT_ADC2_CHANNEL            ADC_CHANNEL_9



/* automatic computations */
#define _CS_GAIN                    ((float)CS_R_OPNtoOPO / (float)CS_R_SHUNTNtoOPN)
#define _VBUS_COEFF                 ((float)(VBUS_R_VtoADC + VBUS_R_ADCtoGND) / (float)(VBUS_R_ADCtoGND))
#define _VMOT_COEFF                 ((float)(VMOT_R_VtoADC + VMOT_R_ADCtoGND) / (float)(VMOT_R_ADCtoGND))
#define _SWITCHING_PERIOD_MS        (1.0f / (float)SWITCHING_FREQUENCY)
#define _MRAS_MOTOR_R               (MOTOR_TERMINAL_RESISTANCE / 2.0f)
#define _MRAS_MOTOR_L               (MOTOR_TERMINAL_INDUCTANCE / 1000.0f / 2.0f)


#endif  /* __SETTINGS_H */
