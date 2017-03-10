/*
 * time.c
 *
 * Implements time.h posix functionality
 *
 *  Created on: Nov 21, 2016
 *      Author: clausr
 */

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"

#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"

#define DWT_O_CYCCNT 0x4

//
// TODO: To use, CCS Build -> ARM Linker -> Advanced Options -> Symbol Management -> Symbol Map -> Add -> "clock=clock_patch"
//       Or add linker option "--symbol_map=clock=clock_patch"
//
clock_t clock_patch() {
  if (!(HWREG(NVIC_DBG_INT) & 0x01000000) && !(HWREG(DWT_BASE) & 0x01)) {
    HWREG(NVIC_DBG_INT) |= 0x01000000;  /*enable TRCENA bit in NVIC_DBG_INT*/
    HWREG(DWT_BASE + DWT_O_CYCCNT) = 0; /* reset the counter */
    HWREG(DWT_BASE) |= 0x01;            /* enable the counter */
  }
  return HWREG(DWT_BASE + DWT_O_CYCCNT) / MAP_SysCtlClockGet();
}
