/*
 * ssi.c
 *
 *  Created on: Feb 17, 2015
 *      Author: Ryan
 */

#include "spi.h"

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"


void SPIInit(uint32_t ui32Base, uint32_t ui32Protocol, uint32_t ui32Mode, uint32_t ui32BitRate, uint32_t ui32DataWidth) {
  uint32_t ui32PeriphGPIO, ui32PeriphSSI;
  uint32_t ui32PinConfigureCLK, ui32PinConfigureRX, ui32PinConfigureTX ;
  uint32_t ui32GPIOBase;
  uint8_t  ui8GPIOPinCLK, ui8GPIOPinRX, ui8GPIOPinTX;

  switch (ui32Base) {
    case SSI0_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOA;
      ui32PeriphSSI = SYSCTL_PERIPH_SSI0;
      ui32PinConfigureCLK = GPIO_PA2_SSI0CLK;
      ui32PinConfigureRX = GPIO_PA4_SSI0RX;
      ui32PinConfigureTX = GPIO_PA5_SSI0TX;
      ui32GPIOBase = GPIO_PORTA_BASE;
      ui8GPIOPinCLK = GPIO_PIN_2;
      ui8GPIOPinRX = GPIO_PIN_4;
      ui8GPIOPinTX = GPIO_PIN_5;
      break;
    }
    case SSI1_BASE: {
      // First open the lock and select the bits we want to modify in the GPIO commit register.
      HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
      HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x1;

      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOF;
      ui32PeriphSSI = SYSCTL_PERIPH_SSI1;
      ui32PinConfigureCLK = GPIO_PF2_SSI1CLK;
      ui32PinConfigureRX = GPIO_PF0_SSI1RX;
      ui32PinConfigureTX = GPIO_PF1_SSI1TX;
      ui32GPIOBase = GPIO_PORTF_BASE;
      ui8GPIOPinCLK = GPIO_PIN_2;
      ui8GPIOPinRX = GPIO_PIN_0;
      ui8GPIOPinTX = GPIO_PIN_1;

      break;
    }
    case SSI2_BASE: {      
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOB;
      ui32PeriphSSI = SYSCTL_PERIPH_SSI2;
      ui32PinConfigureCLK = GPIO_PB4_SSI2CLK;
      ui32PinConfigureRX = GPIO_PB6_SSI2RX;
      ui32PinConfigureTX = GPIO_PB7_SSI2TX;
      ui32GPIOBase = GPIO_PORTB_BASE;
      ui8GPIOPinCLK = GPIO_PIN_4;
      ui8GPIOPinRX = GPIO_PIN_6;
      ui8GPIOPinTX = GPIO_PIN_7;

      break;
    }
    case SSI3_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOD;
      ui32PeriphSSI = SYSCTL_PERIPH_SSI3;
      ui32PinConfigureCLK = GPIO_PD0_SSI3CLK;
      ui32PinConfigureRX = GPIO_PD2_SSI3RX;
      ui32PinConfigureTX = GPIO_PD3_SSI3TX;
      ui32GPIOBase = GPIO_PORTD_BASE;
      ui8GPIOPinCLK = GPIO_PIN_0;
      ui8GPIOPinRX = GPIO_PIN_2;
      ui8GPIOPinTX = GPIO_PIN_3;

      break;
    }
    default: {
      return;
    }
  }

  MAP_SysCtlPeripheralEnable(ui32PeriphSSI);
  while (!MAP_SysCtlPeripheralReady(ui32PeriphSSI));
  MAP_SysCtlPeripheralReset(ui32PeriphSSI);
  while (!MAP_SysCtlPeripheralReady(ui32PeriphSSI));

  MAP_SysCtlPeripheralEnable(ui32PeriphGPIO);
  while (!MAP_SysCtlPeripheralReady(ui32PeriphGPIO));

  MAP_GPIOPinConfigure(ui32PinConfigureCLK);
  MAP_GPIOPinConfigure(ui32PinConfigureRX);
  MAP_GPIOPinConfigure(ui32PinConfigureTX);

  MAP_GPIOPinTypeSSI(ui32GPIOBase, ui8GPIOPinTX | ui8GPIOPinRX | ui8GPIOPinCLK);

  MAP_GPIOPadConfigSet(ui32GPIOBase, ui8GPIOPinRX, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);
  MAP_GPIOPadConfigSet(ui32GPIOBase, ui8GPIOPinCLK | ui8GPIOPinTX, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);

  //
  // Configure the SSI to user specification
  //
  MAP_SSIClockSourceSet(ui32Base, SSI_CLOCK_SYSTEM);
  MAP_SSIConfigSetExpClk(ui32Base, MAP_SysCtlClockGet(), ui32Protocol, ui32Mode, ui32BitRate, ui32DataWidth);

  //
  // Enable the SSI now that configuration is complete.
  //
  MAP_SSIEnable(ui32Base);
}

void SPIFSSInit(uint32_t ui32SPICSBase, uint8_t ui8SPICSPin) {
  MAP_GPIOPinTypeGPIOOutput(ui32SPICSBase, ui8SPICSPin);
  MAP_GPIOPadConfigSet(ui32SPICSBase, ui8SPICSPin, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
  MAP_GPIOPinWrite(ui32SPICSBase, ui8SPICSPin, ui8SPICSPin);
}

bool SPILoopback(uint32_t ui32Base, bool bEnable) {
 return (HWREG(ui32Base + SSI_O_CR1) ^= (-bEnable ^ HWREG(ui32Base + SSI_O_CR1)) & (1 << SSI_CR1_LBM)) & SSI_CR1_LBM;
}
