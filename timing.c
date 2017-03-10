/*
 * timing.c
 *
 *  Created on: Feb 26, 2017
 *      Author: clausr
 */

#include <time.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"

clock_t ClockTicks(void) {
  if (!(HWREG(NVIC_DBG_INT) & 0x01000000) && !(HWREG(DWT_BASE) & 0x01)) {
    HWREG(NVIC_DBG_INT) |= 0x01000000;  /*enable TRCENA bit in NVIC_DBG_INT*/
    HWREG(DWT_BASE + 0x4) = 0;   /* reset the counter */
    HWREG(DWT_BASE) |= 0x01; /* enable the counter */
  }
  return HWREG(DWT_BASE + 0x4);
}

extern clock_t Millis(void);

extern clock_t Micros(void);

void WaitmS(uint32_t ui32Milliseconds) {
  uint32_t start = Millis();
  while((Millis() - start) < ui32Milliseconds);
}

void WaituS(uint32_t ui32Microseconds) {
  uint32_t start = Micros();
  while((Micros() - start) < ui32Microseconds);
}

