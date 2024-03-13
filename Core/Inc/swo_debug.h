#ifndef __SWO_DEBUG_H
#define __SWO_DEBUG_H

/* SWO debug prints
 *   - enable SWV in CubeIDE, Core Clock = 170MHz, then enable ITM Stimulus Port 0
 */
#ifdef DEBUG
  #include <stdio.h>
  #define DEBUG_printf printf

  int __io_putchar(int ch);
#else
  #define DEBUG_printf (void)
#endif
  
#endif /* __SWO_DEBUG_H */
