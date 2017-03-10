/*
 * uart.c
 *
 *  Created on: Feb 17, 2015
 *      Author: Ryan
 */

#include "uart.h"

#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

void
UARTInit(uint32_t ui32Base, uint32_t ui32Baud, bool bPIOSC, uint32_t ui32Config) {
  uint32_t ui32PeriphGPIO, ui32PeriphUART;
  uint32_t ui32PinConfigureRx, ui32PinConfigureTx;
  uint32_t ui32GPIOBase;
  uint8_t  ui8GPIOPinRx, ui8GPIOPinTx;
  switch (ui32Base) {
    case UART0_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOA;
      ui32PeriphUART = SYSCTL_PERIPH_UART0;
      ui32PinConfigureRx = GPIO_PA0_U0RX;
      ui32PinConfigureTx = GPIO_PA1_U0TX;
      ui32GPIOBase = GPIO_PORTA_BASE;
      ui8GPIOPinRx = GPIO_PIN_0;
      ui8GPIOPinTx = GPIO_PIN_1;
      break;
    }
    case UART1_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOB;
      ui32PeriphUART = SYSCTL_PERIPH_UART1;
      ui32PinConfigureRx = GPIO_PB0_U1RX;
      ui32PinConfigureTx = GPIO_PB1_U1TX;
      ui32GPIOBase = GPIO_PORTB_BASE;
      ui8GPIOPinRx = GPIO_PIN_0;
      ui8GPIOPinTx = GPIO_PIN_1;
      break;
    }
    case UART2_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOD;
      ui32PeriphUART = SYSCTL_PERIPH_UART2;
      ui32PinConfigureRx = GPIO_PD6_U2RX;
      ui32PinConfigureTx = GPIO_PD7_U2TX;
      ui32GPIOBase = GPIO_PORTD_BASE;
      ui8GPIOPinRx = GPIO_PIN_6;
      ui8GPIOPinTx = GPIO_PIN_7;

      HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
      HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;

      break;
    }
    case UART3_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOC;
      ui32PeriphUART = SYSCTL_PERIPH_UART3;
      ui32PinConfigureRx = GPIO_PC6_U3RX;
      ui32PinConfigureTx = GPIO_PC7_U3TX;
      ui32GPIOBase = GPIO_PORTC_BASE;
      ui8GPIOPinRx = GPIO_PIN_6;
      ui8GPIOPinTx = GPIO_PIN_7;
      break;
    }
    case UART4_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOC;
      ui32PeriphUART = SYSCTL_PERIPH_UART4;
      ui32PinConfigureRx = GPIO_PC4_U4RX;
      ui32PinConfigureTx = GPIO_PC5_U4TX;
      ui32GPIOBase = GPIO_PORTC_BASE;
      ui8GPIOPinRx = GPIO_PIN_4;
      ui8GPIOPinTx = GPIO_PIN_5;
      break;
    }
    case UART5_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOE;
      ui32PeriphUART = SYSCTL_PERIPH_UART5;
      ui32PinConfigureRx = GPIO_PE4_U5RX;
      ui32PinConfigureTx = GPIO_PE5_U5TX;
      ui32GPIOBase = GPIO_PORTE_BASE;
      ui8GPIOPinRx = GPIO_PIN_4;
      ui8GPIOPinTx = GPIO_PIN_5;
      break;
    }
    case UART6_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOD;
      ui32PeriphUART = SYSCTL_PERIPH_UART6;
      ui32PinConfigureRx = GPIO_PD4_U6RX;
      ui32PinConfigureTx = GPIO_PD5_U6TX;
      ui32GPIOBase = GPIO_PORTD_BASE;
      ui8GPIOPinRx = GPIO_PIN_4;
      ui8GPIOPinTx = GPIO_PIN_5;
      break;
    }
    case UART7_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOE;
      ui32PeriphUART = SYSCTL_PERIPH_UART7;
      ui32PinConfigureRx = GPIO_PE0_U7RX;
      ui32PinConfigureTx = GPIO_PE1_U7TX;
      ui32GPIOBase = GPIO_PORTE_BASE;
      ui8GPIOPinRx = GPIO_PIN_0;
      ui8GPIOPinTx = GPIO_PIN_1;
      break;
    }
    default: {
      return;
    }
  }
  //
  // Enable Peripheral Clocks
  //
  MAP_SysCtlPeripheralEnable(ui32PeriphGPIO);
  while (!MAP_SysCtlPeripheralReady(ui32PeriphGPIO));
  MAP_SysCtlPeripheralEnable(ui32PeriphUART);
  while (!MAP_SysCtlPeripheralReady(ui32PeriphUART));

  //
  // Configure UART Pins
  //
  MAP_GPIOPinConfigure(ui32PinConfigureRx);
  MAP_GPIOPinConfigure(ui32PinConfigureTx);
  MAP_GPIOPinTypeUART(ui32GPIOBase, ui8GPIOPinRx | ui8GPIOPinTx);

  if (bPIOSC) {
    UARTClockSourceSet(ui32Base, UART_CLOCK_PIOSC);
    MAP_UARTConfigSetExpClk(ui32Base, 16000000, ui32Baud, ui32Config);
  } else {
    MAP_UARTConfigSetExpClk(ui32Base, MAP_SysCtlClockGet(), ui32Baud, ui32Config);
  }
}
void
UARTIntInit(uint32_t ui32Base, uint32_t ui32IntFlags, int16_t i16Priority, void (*pfnHandler)(void)) {
  uint32_t ui32Int;
  switch (ui32Base) {
    case UART0_BASE: {
      ui32Int = INT_UART0;
      break;
    }
    case UART1_BASE: {
      ui32Int = INT_UART1;
      break;
    }
    case UART2_BASE: {
      ui32Int = INT_UART2;
      break;
    }
    case UART3_BASE: {
      ui32Int = INT_UART3;
      break;
    }
    case UART4_BASE: {
      ui32Int = INT_UART4;
      break;
    }
    case UART5_BASE: {
      ui32Int = INT_UART5;
      break;
    }
    case UART6_BASE: {
      ui32Int = INT_UART6;
      break;
    }
    case UART7_BASE: {
      ui32Int = INT_UART7;
      break;
    }
  }
  UARTIntRegister(ui32Base, pfnHandler);
  if (i16Priority >= 0) MAP_IntPrioritySet(ui32Int, (uint8_t) (i16Priority & 0xFF));
  MAP_UARTIntClear(ui32Base, 0xFF);
  MAP_IntEnable(ui32Int);
  MAP_UARTIntEnable(ui32Base, ui32IntFlags);
}
