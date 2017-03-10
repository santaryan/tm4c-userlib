/*
 * gpio.c
 *
 *  Created on: Feb 22, 2015
 *      Author: Ryan
 */

#include "gpio.h"

#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

void
GPIOSysCtlInit(uint32_t ui32Port) {
  uint32_t ui32Periph;
  switch (ui32Port) {
    case GPIO_PORTA_BASE: {
      ui32Periph = SYSCTL_PERIPH_GPIOA;
      break;
    }
    case GPIO_PORTB_BASE: {
      ui32Periph = SYSCTL_PERIPH_GPIOB;
      break;
    }
    case GPIO_PORTC_BASE: {
      ui32Periph = SYSCTL_PERIPH_GPIOC;
      break;
    }
    case GPIO_PORTD_BASE: {
      ui32Periph = SYSCTL_PERIPH_GPIOD;
      break;
    }
    case GPIO_PORTE_BASE: {
      ui32Periph = SYSCTL_PERIPH_GPIOE;
      break;
    }
    case GPIO_PORTF_BASE: {
      ui32Periph = SYSCTL_PERIPH_GPIOF;
      break;
    }
    case GPIO_PORTG_BASE: {
      ui32Periph = SYSCTL_PERIPH_GPIOG;
      break;
    }
    case GPIO_PORTH_BASE: {
      ui32Periph = SYSCTL_PERIPH_GPIOH;
      break;
    }
    case GPIO_PORTJ_BASE: {
      ui32Periph = SYSCTL_PERIPH_GPIOJ;
      break;
    }
    case GPIO_PORTK_BASE: {
      ui32Periph = SYSCTL_PERIPH_GPIOK;
      break;
    }
    case GPIO_PORTL_BASE: {
      ui32Periph = SYSCTL_PERIPH_GPIOL;
      break;
    }
    case GPIO_PORTM_BASE: {
      ui32Periph = SYSCTL_PERIPH_GPIOM;
      break;
    }
    case GPIO_PORTN_BASE: {
      ui32Periph = SYSCTL_PERIPH_GPION;
      break;
    }
    case GPIO_PORTP_BASE: {
      ui32Periph = SYSCTL_PERIPH_GPIOP;
      break;
    }
    case GPIO_PORTQ_BASE: {
      ui32Periph = SYSCTL_PERIPH_GPIOQ;
      break;
    }
    case GPIO_PORTR_BASE: {
      ui32Periph = SYSCTL_PERIPH_GPIOR;
      break;
    }
    case GPIO_PORTS_BASE: {
      ui32Periph = SYSCTL_PERIPH_GPIOS;
      break;
    }
    case GPIO_PORTT_BASE: {
      ui32Periph = SYSCTL_PERIPH_GPIOT;
      break;
    }
  }

  MAP_SysCtlPeripheralEnable(ui32Periph);
  while (!MAP_SysCtlPeripheralReady(ui32Periph));
}

uint8_t
GPIOPinLow(uint32_t ui32Port, uint8_t ui8Pins) {
  MAP_GPIOPinWrite(ui32Port, ui8Pins, 0x0);
  return MAP_GPIOPinRead(ui32Port, ui8Pins);
}

uint8_t
GPIOPinHigh(uint32_t ui32Port, uint8_t ui8Pins) {
  MAP_GPIOPinWrite(ui32Port, ui8Pins, 0xFF);
  return MAP_GPIOPinRead(ui32Port, ui8Pins);
}

void
GPIOInputInit(uint32_t ui32Port, uint8_t ui8Pins, uint32_t ui32PinType) {
  GPIOSysCtlInit(ui32Port);
  MAP_GPIOPinTypeGPIOInput(ui32Port, ui8Pins);
  MAP_GPIOPadConfigSet(ui32Port, ui8Pins, GPIO_STRENGTH_2MA, ui32PinType);
}

void
GPIOOutputInit(uint32_t ui32Port, uint8_t ui8Pins, uint32_t ui32Strength, uint32_t ui32PinType) {
  GPIOSysCtlInit(ui32Port);
  MAP_GPIOPinTypeGPIOOutput(ui32Port, ui8Pins);
  MAP_GPIOPadConfigSet(ui32Port, ui8Pins, ui32Strength, ui32PinType);
}

void
GPIOOutputODInit(uint32_t ui32Port, uint8_t ui8Pins, uint32_t ui32Strength, uint32_t ui32PinType) {
  GPIOSysCtlInit(ui32Port);
  MAP_GPIOPinTypeGPIOOutputOD(ui32Port, ui8Pins);
  MAP_GPIOPadConfigSet(ui32Port, ui8Pins, ui32Strength, ui32PinType);
}

uint32_t
GPIOIntInit(uint32_t ui32Port, uint8_t ui8Pins, uint32_t ui32IntType, int16_t i16Priority, void (*pfnHandler)(void)) {
  uint32_t ui32Int;
  switch (ui32Port) {
    case GPIO_PORTA_BASE: {
      ui32Int = INT_GPIOA;
      break;
    }
    case GPIO_PORTB_BASE: {
      ui32Int = INT_GPIOB;
      break;
    }
    case GPIO_PORTC_BASE: {
      ui32Int = INT_GPIOC;
      break;
    }
    case GPIO_PORTD_BASE: {
      ui32Int = INT_GPIOD;
      break;
    }
    case GPIO_PORTE_BASE: {
      ui32Int = INT_GPIOE;
      break;
    }
    case GPIO_PORTF_BASE: {
      ui32Int = INT_GPIOF;
      break;
    }
    case GPIO_PORTG_BASE: {
      ui32Int = INT_GPIOG;
      break;
    }
    case GPIO_PORTH_BASE: {
      ui32Int = INT_GPIOH;
      break;
    }
    case GPIO_PORTJ_BASE: {
      ui32Int = INT_GPIOJ;
      break;
    }
    case GPIO_PORTK_BASE: {
      ui32Int = INT_GPIOK;
      break;
    }
    case GPIO_PORTL_BASE: {
      ui32Int = INT_GPIOL;
      break;
    }
    case GPIO_PORTM_BASE: {
      ui32Int = INT_GPIOM;
      break;
    }
    case GPIO_PORTN_BASE: {
      ui32Int = INT_GPION;
      break;
    }
    default: {
      return 0;
    }
  }

  MAP_GPIOIntTypeSet(ui32Port, ui8Pins, ui32IntType);
  GPIOIntRegister(ui32Port, pfnHandler);
  if (i16Priority >= 0) MAP_IntPrioritySet(ui32Int, (uint8_t) (i16Priority & 0xFF));
  MAP_GPIOIntClear(ui32Port, 0xFF);
  MAP_GPIOIntTypeSet(ui32Port, ui8Pins, ui32IntType);
  MAP_IntEnable(ui32Int);
  MAP_GPIOIntEnable(ui32Port, ui8Pins);
  return ui8Pins;
}

void
GPIOUnlock(uint32_t ui32Port) {
  HWREG(ui32Port + GPIO_O_LOCK) = GPIO_LOCK_KEY;
  HWREG(ui32Port + GPIO_O_CR)  |= 0x01;
  HWREG(ui32Port + GPIO_O_LOCK) = 0;
}
