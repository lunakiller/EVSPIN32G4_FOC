#ifndef __SWO_DEBUG_H
#define __SWO_DEBUG_H

/* SWO debug prints
 *   - enable SWV in CubeIDE, Core Clock = 170MHz, then enable ITM Stimulus Port 0
 */
#ifdef DEBUG
  #include <stdio.h>
  #define DEBUG_printf       printf
  int __io_putchar(int ch);

  __STATIC_INLINE void DEBUG_print(char* s) {
    while(*s) {
      ITM_SendChar(*s++);
    }
  }

  __attribute__((unused)) static void DEBUG_Breakpoint(void) {
    // place the breakpoint either on the function or on the NOP
    __NOP();
  }
#else
  #define DEBUG_printf      (void)
  #define DEBUG_print       (void)
  #define DEBUG_Breakpoint  (void)
#endif
  
#endif /* __SWO_DEBUG_H */
