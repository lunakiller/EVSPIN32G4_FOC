/*
 * settings.h
 *
 *  Created on: Mar 4, 2024
 *      Author: lunakiller
 *
 */



// VBUS divider
#define VBUS_R_VtoADC                     72300                     // resistor between VM and ADC input in Ohms
#define VBUS_R_ADCtoGND                   3010                      // resistor between ADC input and GND in Ohms

// Current sensing
#define CS_SHUNT_VALUE                    0.005                     // shunt in Ohms
#define CS_R_SHUNTNtoOPN                  1500                      // resistor between negative shunt output and negative OPAMP input in Ohms
#define CS_R_OPNtoOPO                     11000                     // resistor between negative OPAMP input and OPAMP output in Ohms
#define CS_GAIN                           _CS_GAIN                  // current sensing amplifier gain





/* SWO debug prints
 *   - enable SWV in CubeIDE, Core Clock = 170MHz, then enable ITM Stimulus Port 0
 */
#ifdef DEBUG
  #include <stdio.h>
  #define DEBUG_printf printf

  int __io_putchar(int ch) {
    ITM_SendChar(ch);
    return ch;
  }
#else
  #define DEBUG_printf (void)
#endif


/* automatic computations */
#define _CS_GAIN                          ((float)CS_R_OPNtoOPO / (float)CS_R_SHUNTNtoOPN)
