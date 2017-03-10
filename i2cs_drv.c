//*****************************************************************************
//
// i2cs_drv.c - Interrupt-driven I2C slave driver.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "i2cs_drv.h"

//*****************************************************************************
//
//! \addtogroup i2cs_drv_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! Handles I2C slave interrupts.
//!
//! \param psInst is a pointer to the I2C slave instance data.
//!
//! This function performs the processing required in response to an I2C
//! interrupt.  The application-supplied interrupt handler should call this
//! function with the correct instance data in response to the I2C interrupt.
//!
//! \return None.
//
//*****************************************************************************
void I2CSIntHandler(tI2CSInstance *psInst) {
  uint32_t ui32Status = I2C_SLAVE_ACT_NONE;
  uint8_t ui8Data = 0xFF;
  tSlaveCallback *pfnCallback = psInst->pfnCallback;

  //
  // Clear the I2C interrupt.
  //
  MAP_I2CSlaveIntClear(psInst->ui32Base);
  ui32Status = HWREG(psInst->ui32Base + I2C_O_SCSR);

  switch (ui32Status) {
    case I2C_SLAVE_ACT_RREQ_FBR: {
      psInst->ui8Data = MAP_I2CSlaveDataGet(psInst->ui32Base);
      if (pfnCallback) pfnCallback(psInst, I2CS_STATUS_WRITE_START);
      break;
    }
    case I2C_SLAVE_ACT_RREQ: {
      psInst->ui8Data = MAP_I2CSlaveDataGet(psInst->ui32Base);
      if (pfnCallback) pfnCallback(psInst, I2CS_STATUS_WRITE_CONT);
      break;
    }
    case I2C_SLAVE_ACT_TREQ: {
      if (pfnCallback) pfnCallback(psInst, I2CS_STATUS_READ);
      MAP_I2CSlaveDataPut(psInst->ui32Base, ui8Data);
      break;
    }
    case I2C_SLAVE_ACT_QCMD: {
      psInst->ui8Data = ui32Status & I2C_SLAVE_ACT_QCMD_DATA;
      if (pfnCallback) pfnCallback(psInst, I2CS_STATUS_QCMD);
      break;
    }
  }
}

//*****************************************************************************
//
//! Initializes the I2C slave driver.
//!
//! \param psInst is a pointer to the I2C slave instance data.
//! \param ui32Base is the base address of the I2C module.
//! \param ui8Addr is the 7-bit slave address of the I2C module.
//! \param pfnCallback is the event callback of the I2C module.
//! \param ui8Int is the interrupt number for the I2C module.
//! \param ui8TxDMA is the uDMA channel number used for transmitting data to
//! the I2C module.
//! \param ui8RxDMA is the uDMA channel number used for receiving data from
//! the I2C module.
//!
//! This function prepares both the I2C slave module and driver for operation,
//! and must be the first I2C slave driver function called for each I2C slave
//! instance.  It is assumed that the application has enabled the I2C module,
//! configured the I2C pins, and provided an I2C interrupt handler that calls
//! I2CSIntHandler().
//!
//! The uDMA module cannot be used at present to transmit/receive data, so the
//! \e ui8TxDMA and \e ui8RxDMA parameters are unused.  They are reserved for
//! future use and should be set to 0xff in order to ensure future
//! compatibility.
//!
//! \return None.
//
//*****************************************************************************
void I2CSInit(tI2CSInstance *psInst, uint32_t ui32Base, uint8_t ui8Addr, tSlaveCallback *pfnCallback, uint_fast8_t ui8Int, uint_fast8_t ui8TxDMA, uint_fast8_t ui8RxDMA) {
  //
  // Check the arguments.
  //
  ASSERT(psInst);ASSERT((ui32Base == I2C0_BASE) || (ui32Base == I2C1_BASE) ||
      (ui32Base == I2C2_BASE) || (ui32Base == I2C3_BASE) ||
      (ui32Base == I2C4_BASE) || (ui32Base == I2C5_BASE) ||
      (ui32Base == I2C6_BASE) || (ui32Base == I2C7_BASE) ||
      (ui32Base == I2C8_BASE) || (ui32Base == I2C9_BASE));ASSERT(ui8Addr < 128);ASSERT(ui8Int);

  //
  // Initialize the state structure.
  //
  psInst->ui32Base = ui32Base;
  psInst->ui8Addr = ui8Addr;
  psInst->pfnCallback = pfnCallback;
  psInst->ui8Int = ui8Int;
  psInst->ui8TxDMA = ui8TxDMA;
  psInst->ui8RxDMA = ui8RxDMA;

  //
  // Initialize the I2C slave module.
  //
  MAP_I2CSlaveInit(ui32Base, ui8Addr);

  //
  // Enable the I2C interrupt.
  //
  MAP_IntEnable(ui8Int);
  MAP_I2CSlaveIntEnableEx(ui32Base, I2C_SLAVE_INT_DATA);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
