/*
 * settings.h
 *
 *  Created on: Mar 4, 2024
 *      Author: lunakiller
 *
 */

#ifndef __SETTINGS_H
#define __SETTINGS_H

// General
#define SWITCHING_FREQUENCY         ((uint8_t)  50)                 // switching frequency in kHz
#define DEAD_TIME                   ((uint8_t)  40)                 // DEAD_TIME*5.9 = deadtime in ns

// Protections
#define UNDERVOLTAGE_THRESHOLD      ((uint8_t)  10)                 // undervoltage protection threshold in V
#define OVERVOLTAGE_THRESHOLD       ((uint8_t)  15)                 // overvoltage protection threshold in V
#define SW_OVERCURRENT_THRESHOLD    ((uint16_t) 1000)               // overcurrent protection threshold in mA

// VBUS divider
#define VBUS_R_VtoADC               ((uint32_t) 72300)              // resistor between VM and ADC input in Ohms
#define VBUS_R_ADCtoGND             ((uint32_t) 3010)               // resistor between ADC input and GND in Ohms
#define VBUS_COEFF                  ((float)    _VBUS_COEFF)        // coefficient to compute VBUS from ADC

// Current sensing
#define CS_SHUNT_VALUE              ((float)    0.005)              // shunt in Ohms
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


#endif  /* __SETTINGS_H */
