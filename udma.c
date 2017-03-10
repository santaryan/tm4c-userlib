/*
 * udma.c
 *
 *  Created on: Nov 16, 2016
 *      Author: clausr
 */

#include "udma.h"

#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_udma.h"

#include "driverlib/interrupt.h"
#include "driverlib/udma.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

static uint32_t g_udma_ui32Status = 0, g_udma_ui32uDMAErrCount = 0;

//*****************************************************************************
//
// The simple interrupt handler for uDMA errors.  This interrupt will occur if
// the uDMA encounters a bus error while trying to perform a transfer.  This
// handler just increments a counter if an error occurs.
//
//*****************************************************************************
void
uDMAErrorHandler(void) {

  //
  // Check for uDMA error bit
  //
  g_udma_ui32Status = MAP_uDMAErrorStatusGet();

  //
  // If there is a uDMA error, then clear the error and increment
  // the error counter.
  //
  if(g_udma_ui32Status) {
     MAP_uDMAErrorStatusClear();
     g_udma_ui32uDMAErrCount++;
  }
}

//*****************************************************************************
//
// The simple interrupt handler for uDMA errors.  This interrupt will occur if
// the uDMA encounters a bus error while trying to perform a transfer.  This
// handler just increments a counter if an error occurs.
//
//*****************************************************************************
inline uint32_t uDMAGetLastError() {
  return g_udma_ui32Status;
}

inline uint32_t uDMAGetErrorCount() {
  return g_udma_ui32uDMAErrCount;
}

void
uDMAInit(void) {
  //
  // Power up the uDMA
  //
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_UDMA));

  //
  // Enable the uDMA controller.
  //
  MAP_uDMAEnable();

  //
  // Point at the control table to use for channel control structures.
  //
  MAP_uDMAControlBaseSet(ui8DMAChannelControlStructure);

  uDMAIntRegister(INT_UDMAERR, uDMAErrorHandler);
}
