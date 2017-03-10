/*
 * i2c.c
 *
 *  Created on: Feb 17, 2015
 *      Author: Ryan
 */

#include "i2c.h"

#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

uint32_t
I2CInit(uint32_t ui32Base, bool bSpeed) {
  uint32_t ui32PeriphGPIO, ui32PeriphI2C;
  uint32_t ui32BusState;
  
  switch(ui32Base) {
    case I2C0_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOB;
      ui32PeriphI2C = SYSCTL_PERIPH_I2C0;
      break;
    }
    case I2C1_BASE: {      
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOA;
      ui32PeriphI2C = SYSCTL_PERIPH_I2C1;
      break;
    }
    case I2C2_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOE;
      ui32PeriphI2C = SYSCTL_PERIPH_I2C2;
      break;
    }
    case I2C3_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOD;
      ui32PeriphI2C = SYSCTL_PERIPH_I2C3;
      break;
    }
#if defined(GPIO_PG2_I2C4SCL) && defined(GPIO_PG3_I2C4SDA)
    case I2C4_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOG;
      ui32PeriphI2C = SYSCTL_PERIPH_I2C4;
      break;
    }
#endif
#if defined(GPIO_PG6_I2C5SCL) && defined(GPIO_PG7_I2C5SDA)
    case I2C5_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOG;
      ui32PeriphI2C = SYSCTL_PERIPH_I2C5;
      break;
    }
#endif
#if defined(GPIO_PA6_I2C6SCL) && defined(GPIO_PA7_I2C6SDA)
    case I2C6_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOA;
      ui32PeriphI2C = SYSCTL_PERIPH_I2C6;
      break;
    }
#endif
#if defined(GPIO_PD0_I2C7SCL) && defined(GPIO_PD1_I2C7SDA)
    case I2C7_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOD;
      ui32PeriphI2C = SYSCTL_PERIPH_I2C7;
      break;
    }
#endif
#if defined(GPIO_PD2_I2C8SCL) && defined(GPIO_PD3_I2C8SDA)
    case I2C8_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOD;
      ui32PeriphI2C = SYSCTL_PERIPH_I2C8;
      break;
    }
#endif
#if defined(GPIO_PA0_I2C9SCL) && defined(GPIO_PA1_I2C9SDA)
    case I2C9_BASE: {
      ui32PeriphGPIO = SYSCTL_PERIPH_GPIOA;
      ui32PeriphI2C = SYSCTL_PERIPH_I2C9;
      break;
    }
#endif
    default: {
      return I2C_INVALID_BASE;
    }
  }
  
  //
  // Enable GPIO Peripheral used by I2C
  //
  MAP_SysCtlPeripheralEnable(ui32PeriphGPIO);
  while (!MAP_SysCtlPeripheralReady(ui32PeriphGPIO));

  ui32BusState = I2CClearBus(ui32Base);
  
  //
  // Enable I2C Peripheral
  //
  MAP_SysCtlPeripheralEnable(ui32PeriphI2C);
  while (!MAP_SysCtlPeripheralReady(ui32PeriphI2C));
  MAP_SysCtlPeripheralReset(ui32PeriphI2C);
  while (!MAP_SysCtlPeripheralReady(ui32PeriphI2C));

  //
  // Enable the supplied I2C Base Clock
  //
  MAP_I2CMasterInitExpClk(ui32Base, MAP_SysCtlClockGet(), bSpeed);

  //
  // Enable supplied I2C Base Master Block
  //
  MAP_I2CMasterEnable(ui32Base);

  return ui32BusState;
}
bool
I2CBurstRead(uint32_t ui32Base, uint8_t ui8SlaveAddr, uint8_t* ui8ptrReadData, uint32_t ui32Size) {
  //
  // Use I2C single read if theres only 1 item to receive
  //
  if (ui32Size == 1)
    return I2CRead(ui32Base, ui8SlaveAddr, &ui8ptrReadData[0]);

  uint32_t ui32ByteCount;        // local variable used for byte counting/state determination
  uint32_t MasterOptionCommand; // used to assign the control commands

  //
  // Tell the master module what address it will place on the bus when
  // reading from the slave.
  //
  MAP_I2CMasterSlaveAddrSet(ui32Base, ui8SlaveAddr, I2C_MODE_READ);

  //
  // Start with BURST with more than one byte to read
  //
  MasterOptionCommand = I2C_MASTER_CMD_BURST_RECEIVE_START;

  for(ui32ByteCount = 0; ui32ByteCount < ui32Size; ui32ByteCount++)
  {
    //
    // The second and intermittent byte has to be read with CONTINUE control word
    //
    if(ui32ByteCount == 1)
      MasterOptionCommand = I2C_MASTER_CMD_BURST_RECEIVE_CONT;

    //
    // The last byte has to be send with FINISH control word
    //
    if(ui32ByteCount == ui32Size - 1)
      MasterOptionCommand = I2C_MASTER_CMD_BURST_RECEIVE_FINISH;

    //
    // Initiate read of data from the slave.
    //
    MAP_I2CMasterControl(ui32Base, MasterOptionCommand);

    //
    // Wait until master module is done reading.
    //
    while(MAP_I2CMasterBusy(ui32Base));

    //
    // Check for errors.
    //
    if (MAP_I2CMasterErr(ui32Base) != I2C_MASTER_ERR_NONE)
      return false;

    //
    // Move byte from register
    //
    ui8ptrReadData[ui32ByteCount] = (uint8_t) MAP_I2CMasterDataGet(ui32Base);
  }

  //
  // Return 1 if there is no error.
  //
  return true;
}
bool
I2CBurstWrite(uint32_t ui32Base, uint8_t ui8SlaveAddr, uint8_t ui8SendData[], uint32_t ui32Size) {
  //
  // Use I2C single write if there's only 1 item to send
  //
  if (ui32Size == 1)
    return I2CWrite(ui32Base, ui8SlaveAddr, ui8SendData[0]);

  uint32_t uiByteCount;         // local variable used for byte counting/state determination
  uint32_t MasterOptionCommand; // used to assign the control commands

  //
  // Tell the master module what address it will place on the bus when
  // writing to the slave.
  //
  MAP_I2CMasterSlaveAddrSet(ui32Base, ui8SlaveAddr, I2C_MODE_WRITE);

  //
  // The first byte has to be sent with the START control word
  //
  MasterOptionCommand = I2C_MASTER_CMD_BURST_SEND_START;

  for(uiByteCount = 0; uiByteCount < ui32Size; uiByteCount++)
  {
    //
    // The second and intermittent byte has to be send with CONTINUE control word
    //
    if(uiByteCount == 1)
      MasterOptionCommand = I2C_MASTER_CMD_BURST_SEND_CONT;

    //
    // The last byte has to be send with FINISH control word
    //
    if(uiByteCount == ui32Size - 1)
      MasterOptionCommand = I2C_MASTER_CMD_BURST_SEND_FINISH;

    //
    // Send data byte
    //
    MAP_I2CMasterDataPut(ui32Base, ui8SendData[uiByteCount]);

    //
    //
    // Initiate send of data from the master.
    //
    MAP_I2CMasterControl(ui32Base, MasterOptionCommand);

    //
    // Wait until master module is done reading.
    //
    while(MAP_I2CMasterBusy(ui32Base));

    //
    // Check for errors.
    //
    if(MAP_I2CMasterErr(ui32Base) != I2C_MASTER_ERR_NONE) return false;
  }

  //
  // Return 1 if there is no error.
  //
  return true;
}
bool
I2CRead(uint32_t ui32Base, uint8_t ui8SlaveAddr, uint8_t* ui8ptrData) {
  //
  // Tell the master module what address it will place on the bus when
  // reading from the slave.
  //
  MAP_I2CMasterSlaveAddrSet(ui32Base, ui8SlaveAddr, I2C_MODE_READ);

  //
  // Tell the master to read data.
  //
  MAP_I2CMasterControl(ui32Base, I2C_MASTER_CMD_SINGLE_RECEIVE);

  //
  // Wait until master module is done reading.
  //
  while(MAP_I2CMasterBusy(ui32Base));

  //
  // Check for errors.
  //
  if(MAP_I2CMasterErr(ui32Base) != I2C_MASTER_ERR_NONE)
    return false;

  //
  // Get data
  //
  *ui8ptrData = (uint8_t) MAP_I2CMasterDataGet(ui32Base);

  //
  // return the data from the master.
  //
  return true;
}
bool
I2CWrite(uint32_t ui32Base, uint8_t ui8SlaveAddr, uint8_t ui8SendData) {
  //
  //
  // Tell the master module what address it will place on the bus when
  // writing to the slave.
  //
  MAP_I2CMasterSlaveAddrSet(ui32Base, ui8SlaveAddr, I2C_MODE_WRITE);

  //
  // Place the command to be sent in the data register.
  //
  MAP_I2CMasterDataPut(ui32Base, ui8SendData);

  //
  // Initiate send of data from the master.
  //
  MAP_I2CMasterControl(ui32Base, I2C_MASTER_CMD_SINGLE_SEND);

  //
  // Wait until master module is done reading.
  //
  while(MAP_I2CMasterBusy(ui32Base));

  //
  // Check for errors.
  //
  if(MAP_I2CMasterErr(ui32Base) != I2C_MASTER_ERR_NONE)
    return false;

  //
  // Return 1 if there is no error.
  //
  return true;
}
uint32_t
I2CClearBus(uint32_t ui32Base) {
  /**
   * This routine turns off the I2C bus and clears it
   * on return SCA and SCL pins are returned to I2C inputs.
   *
   * returns I2C_BUS_OK if bus cleared
   *         I2C_MULTI_MASTER_DETECTED if SCL held low.
   *         I2C_SLAVE_CLOCK_STRETCH_DETECTED if SDA held low by slave clock stretch for > 2sec
   *         I2C_UNKNOWN_ERROR if SDA held low after 20 clocks.
   */

  uint32_t ui32BusState = I2C_BUS_OK;
  uint8_t ui8ClockCount;
  uint64_t ui64ClockStart = 0, ui645uSClockCount = (MAP_SysCtlClockGet() / 200000);
  bool bSCLLow, bSDALow;
  uint32_t ui32GPIOBase;
  uint8_t  ui8GPIOPinSCL, ui8GPIOPinSDA;
  uint32_t ui32PinConfigureSCL, ui32PinConfigureSDA;

  switch(ui32Base) {
    case I2C0_BASE: {
      ui32PinConfigureSCL = GPIO_PB2_I2C0SCL;
      ui32PinConfigureSDA = GPIO_PB3_I2C0SDA;
      ui32GPIOBase = GPIO_PORTB_BASE; 
      ui8GPIOPinSCL = GPIO_PIN_2;
      ui8GPIOPinSDA = GPIO_PIN_3;
      break;
    }
    case I2C1_BASE: {
      ui32PinConfigureSCL = GPIO_PA6_I2C1SCL;
      ui32PinConfigureSDA = GPIO_PA7_I2C1SDA;
      ui32GPIOBase = GPIO_PORTA_BASE; 
      ui8GPIOPinSCL = GPIO_PIN_6;
      ui8GPIOPinSDA = GPIO_PIN_7;
      break;
    }
    case I2C2_BASE: {
      ui32PinConfigureSCL = GPIO_PE4_I2C2SCL;
      ui32PinConfigureSDA = GPIO_PE5_I2C2SDA;
      ui32GPIOBase = GPIO_PORTE_BASE; 
      ui8GPIOPinSCL = GPIO_PIN_4;
      ui8GPIOPinSDA = GPIO_PIN_5;
      break;
    }
    case I2C3_BASE: {
      ui32PinConfigureSCL = GPIO_PD0_I2C3SCL;
      ui32PinConfigureSDA = GPIO_PD1_I2C3SDA;
      ui32GPIOBase = GPIO_PORTD_BASE; 
      ui8GPIOPinSCL = GPIO_PIN_0;
      ui8GPIOPinSDA = GPIO_PIN_1;
      break;
    }
#if defined(GPIO_PG2_I2C4SCL) && defined(GPIO_PG3_I2C4SDA)
    case I2C4_BASE: {
      ui32PinConfigureSCL = GPIO_PG2_I2C4SCL;
      ui32PinConfigureSDA = GPIO_PG3_I2C4SDA;
      ui32GPIOBase = GPIO_PORTG_BASE;
      ui8GPIOPinSCL = GPIO_PIN_2;
      ui8GPIOPinSDA = GPIO_PIN_3;
      break;
    }
#endif
#if defined(GPIO_PG6_I2C5SCL) && defined(GPIO_PG7_I2C5SDA)
    case I2C5_BASE: {
      ui32PinConfigureSCL = GPIO_PG6_I2C5SCL;
      ui32PinConfigureSDA = GPIO_PG7_I2C5SDA;
      ui32GPIOBase = GPIO_PORTG_BASE;
      ui8GPIOPinSCL = GPIO_PIN_6;
      ui8GPIOPinSDA = GPIO_PIN_7;
      break;
    }
#endif
#if defined(GPIO_PA6_I2C6SCL) && defined(GPIO_PA7_I2C6SDA)
    case I2C6_BASE: {
      ui32PinConfigureSCL = GPIO_PA6_I2C6SCL;
      ui32PinConfigureSDA = GPIO_PA7_I2C6SDA;
      ui32GPIOBase = GPIO_PORTA_BASE;
      ui8GPIOPinSCL = GPIO_PIN_6;
      ui8GPIOPinSDA = GPIO_PIN_7;
      break;
    }
#endif
#if defined(GPIO_PD0_I2C7SCL) && defined(GPIO_PD1_I2C7SDA)
    case I2C7_BASE: {
      ui32PinConfigureSCL = GPIO_PD0_I2C7SCL;
      ui32PinConfigureSDA = GPIO_PD1_I2C7SDA;
      ui32GPIOBase = GPIO_PORTD_BASE;
      ui8GPIOPinSCL = GPIO_PIN_0;
      ui8GPIOPinSDA = GPIO_PIN_1;
      break;
    }
#endif
#if defined(GPIO_PD2_I2C8SCL) && defined(GPIO_PD3_I2C8SDA)
    case I2C8_BASE: {
      ui32PinConfigureSCL = GPIO_PD2_I2C8SCL;
      ui32PinConfigureSDA = GPIO_PD3_I2C8SDA;
      ui32GPIOBase = GPIO_PORTD_BASE;
      ui8GPIOPinSCL = GPIO_PIN_2;
      ui8GPIOPinSDA = GPIO_PIN_3;
      break;
    }
#endif
#if defined(GPIO_PA0_I2C9SCL) && defined(GPIO_PA1_I2C9SDA)
    case I2C9_BASE: {
      ui32PinConfigureSCL = GPIO_PA0_I2C9SCL;
      ui32PinConfigureSDA = GPIO_PA1_I2C9SDA;
      ui32GPIOBase = GPIO_PORTA_BASE;
      ui8GPIOPinSCL = GPIO_PIN_0;
      ui8GPIOPinSDA = GPIO_PIN_1;
      break;
    }
#endif
    default: {
      return I2C_INVALID_BASE;
    }
  }

  // Enable timing using the debug registers if not already enabled
  if (!(HWREG(NVIC_DBG_INT) & 0x01000000) && !(HWREG(DWT_BASE) & 0x01)) {
    HWREG(NVIC_DBG_INT) |= 0x01000000;  // enable TRCENA bit in NVIC_DBG_INT
    HWREG(DWT_BASE + 0x4) = 0;          // reset the counter
    HWREG(DWT_BASE) |= 0x01;            // enable the counter
  }

  // Emulate I2C Slave configuration
  MAP_GPIOPadConfigSet(ui32GPIOBase, ui8GPIOPinSCL | ui8GPIOPinSDA, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD | GPIO_PIN_TYPE_STD_WPU);
  MAP_GPIODirModeSet(ui32GPIOBase, ui8GPIOPinSCL | ui8GPIOPinSDA, GPIO_DIR_MODE_IN);

  // Check if SCL is held low
  bSCLLow = !MAP_GPIOPinRead(ui32GPIOBase, ui8GPIOPinSCL);

  // I2C Bus error. Wouldn't be able to become a master
  if (bSCLLow) ui32BusState = I2C_MULTI_MASTER_DETECTED;

  // Check if SDA is held low
  bSDALow = !MAP_GPIOPinRead(ui32GPIOBase, ui8GPIOPinSDA);

  // If the bus is still okay (SCL is not low) and the SDA is being held low
  if (ui32BusState == I2C_BUS_OK && bSDALow) {

      // Setup maximum clock count (9 is worst case scenario)
      ui8ClockCount = 9;

      // Emulate master clock pulses at 100 kHz while
      do {
        // Take over SCL
        MAP_GPIODirModeSet(ui32GPIOBase, ui8GPIOPinSCL, GPIO_DIR_MODE_OUT);

        // Write out a 0, pulling it low
        MAP_GPIOPinWrite(ui32GPIOBase, ui8GPIOPinSCL, 0x0);

        // Delay for ~5uS
        ui64ClockStart = HWREG(DWT_BASE + 0x4);
        while ((HWREG(DWT_BASE + 0x4) - ui64ClockStart) < ui645uSClockCount);

        // Release SCL; pull-up brings it high
        MAP_GPIODirModeSet(ui32GPIOBase, ui8GPIOPinSCL, GPIO_DIR_MODE_IN);

        // Delay for ~5uS
        ui64ClockStart = HWREG(DWT_BASE + 0x4);
        while ((HWREG(DWT_BASE + 0x4) - ui64ClockStart) < ui645uSClockCount);

        // Loop waiting for SCL to become high only. Only wait for ~2 sec.
        ui64ClockStart = HWREG(DWT_BASE + 0x4);
        while ((bSCLLow = !MAP_GPIOPinRead(ui32GPIOBase, ui8GPIOPinSCL)) &&
               ((HWREG(DWT_BASE + 0x4) - ui64ClockStart) < (400000 * ui645uSClockCount)))

        // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
        if (bSCLLow) ui32BusState = I2C_SLAVE_CLOCK_STRETCH_DETECTED;

        // Check if SDA is held low
        bSDALow = !MAP_GPIOPinRead(ui32GPIOBase, ui8GPIOPinSDA);

        ui8ClockCount--;
      } while (bSDALow && ui8ClockCount);

      // I2C bus error. Could not clear. SDA data line held low
      if (bSDALow) ui32BusState = I2C_UNKNOWN_ERROR;

      // Take over SDA
      MAP_GPIODirModeSet(ui32GPIOBase, ui8GPIOPinSDA, GPIO_DIR_MODE_OUT);

      // Write out a 0, pulling it low
      MAP_GPIOPinWrite(ui32GPIOBase, ui8GPIOPinSCL, 0x0);

      // Delay for ~5uS
      ui64ClockStart = HWREG(DWT_BASE + 0x4);
      while ((HWREG(DWT_BASE + 0x4) - ui64ClockStart) < ui645uSClockCount);

      // Release SDA; pull-up brings it high
      MAP_GPIODirModeSet(ui32GPIOBase, ui8GPIOPinSDA, GPIO_DIR_MODE_IN);

      // Delay for ~5uS
      ui64ClockStart = HWREG(DWT_BASE + 0x4);
      while ((HWREG(DWT_BASE + 0x4) - ui64ClockStart) < ui645uSClockCount);
  }

  // Set the pins for proper I2C operation
  MAP_GPIOPinConfigure(ui32PinConfigureSCL);
  MAP_GPIOPinTypeI2CSCL(ui32GPIOBase, ui8GPIOPinSCL);
  MAP_GPIOPinConfigure(ui32PinConfigureSDA);
  MAP_GPIOPinTypeI2C(ui32GPIOBase, ui8GPIOPinSDA);

  return ui32BusState;
}
