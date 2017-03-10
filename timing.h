/*
 * timing.h
 *
 *  Created on: Feb 26, 2017
 *      Author: clausr
 */

#ifndef __TIMING_H__
#define __TIMING_H__

#include <time.h>

extern clock_t ClockTicks(void);

inline clock_t Millis(void) {
  return ClockTicks() / (MAP_SysCtlClockGet() / 1000);
}

inline clock_t Micros(void) {
  return ClockTicks() / (MAP_SysCtlClockGet() / 1000000);
}

void WaitmS(uint32_t ui32Milliseconds);

void WaituS(uint32_t ui32Microseconds);

#endif /* TIMING_H_ */
