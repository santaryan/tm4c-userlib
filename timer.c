/*
 * timer.c
 *
 *  Created on: Nov 8, 2015
 *      Author: Ryan
 */

#include "timer.h"

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

void TimerFullWidthInit(uint32_t ui32Base, uint32_t ui32Config, uint32_t ui32Value) {
  uint32_t ui32Peripheral;
  switch (ui32Base) {
    case TIMER0_BASE: {
      ui32Peripheral = SYSCTL_PERIPH_TIMER0;
      break;
    }
    case TIMER1_BASE: {
      ui32Peripheral = SYSCTL_PERIPH_TIMER1;
      break;
    }
    case TIMER2_BASE: {
      ui32Peripheral = SYSCTL_PERIPH_TIMER2;
      break;
    }
    case TIMER3_BASE: {
      ui32Peripheral = SYSCTL_PERIPH_TIMER3;
      break;
    }
    case TIMER4_BASE: {
      ui32Peripheral = SYSCTL_PERIPH_TIMER4;
      break;
    }
    case TIMER5_BASE: {
      ui32Peripheral = SYSCTL_PERIPH_TIMER5;
      break;
    }
    case TIMER6_BASE: {
      ui32Peripheral = SYSCTL_PERIPH_TIMER6;
      break;
    }
    case TIMER7_BASE: {
      ui32Peripheral = SYSCTL_PERIPH_TIMER7;
      break;
    }
  }

  //
  // Enable the peripherals used by the timer
  //
  MAP_SysCtlPeripheralEnable(ui32Peripheral);
  while (!MAP_SysCtlPeripheralReady(ui32Peripheral));

  //
  // Configure the 32-bit periodic timer for the status code length define.
  //
  MAP_TimerConfigure(ui32Base, ui32Config);
  MAP_TimerLoadSet(ui32Base, TIMER_A, MAP_SysCtlClockGet() / ui32Value);

  //
  // Enable the timer.
  //
  MAP_TimerEnable(ui32Base, TIMER_A);
}

uint32_t TimerFullWidthIntInit(uint32_t ui32Base, int16_t i16Priority, uint32_t ui32IntFlags, void (*pfnHandler)(void)) {
  uint32_t ui32TimerInt;
  switch (ui32Base) {
    case TIMER0_BASE: {
      ui32TimerInt = INT_TIMER0A;
      break;
    }
    case TIMER1_BASE: {
      ui32TimerInt = INT_TIMER1A;
      break;
    }
    case TIMER2_BASE: {
      ui32TimerInt = INT_TIMER2A;
      break;
    }
    case TIMER3_BASE: {
      ui32TimerInt = INT_TIMER3A;
      break;
    }
    case TIMER4_BASE: {
      ui32TimerInt = INT_TIMER4A;
      break;
    }
    case TIMER5_BASE: {
      ui32TimerInt = INT_TIMER5A;
      break;
    }
    default: {
      return 0;
    }
  }

  //
  // Setup the interrupts for the timer.
  //
  if (i16Priority >= 0) MAP_IntPrioritySet(ui32TimerInt, (i16Priority * 0xF));
  TimerIntRegister(ui32Base, TIMER_A, pfnHandler);
  MAP_IntEnable(ui32TimerInt);
  MAP_TimerIntEnable(ui32Base, ui32IntFlags);

  return ui32TimerInt;
}
