//*****************************************************************************
//
// spim_drv.c - Interrupt-driven SPI driver.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_ssi.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/udma.h"

#include "spim_drv.h"

//*****************************************************************************
//
//! \addtogroup spim_drv_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// The states in the interrupt handler state machine.
//
//*****************************************************************************
#define STATE_IDLE            0
#define STATE_TRANSFER_START  1
#define STATE_TRANSFER_DMA    2
#define STATE_TRANSFER_TX     3
#define STATE_TRANSFER_RX     4
#define STATE_TRANSFER_END    5
#define STATE_TRANSFER_PAUSE  6
#define STATE_CALLBACK        7

//*****************************************************************************
//
// Internal Functions
//
//*****************************************************************************
// asserts the CS pin to the card
static inline void SELECT(uint32_t ui32SPICSBase, uint8_t ui8SPICSPin) {
  MAP_GPIOPinWrite(ui32SPICSBase, ui8SPICSPin, 0);
}

// de-asserts the CS pin to the card
static inline void DESELECT(uint32_t ui32SPICSBase, uint8_t ui8SPICSPin) {
  MAP_GPIOPinWrite(ui32SPICSBase, ui8SPICSPin, ui8SPICSPin);
}

// Returns the uDMA status of the channel
static inline uint32_t SPIMCheckDMAStatus(uint32_t ui32Channel) {
  return (MAP_uDMAChannelModeGet(ui32Channel | UDMA_PRI_SELECT));
}

// Checks if a uDMA channel is stopped (No running transaction)
static inline bool SPIMIsDMAStopped(uint32_t ui32Channel) {
  return ((SPIMCheckDMAStatus(ui32Channel) == UDMA_MODE_STOP)
      && (!MAP_uDMAChannelIsEnabled(ui32Channel)));
}

// Flush the RX FIFO
static void SPIMFlushRxFIFO(uint32_t ui32Base) {
  uint32_t ui32Idx;

  //
  // Keep reading until there's no data left to read.
  //
  while (MAP_SSIDataGetNonBlocking(ui32Base, &ui32Idx))
    ;

  //
  //Clear the RX overrun interrupt flag.
  //
  SSIIntClear(ui32Base, SSI_RIS_RORRIS);
}

//*****************************************************************************
//
//! Transfers data from an SPI device.
//!
//! \param psInst is a pointer to the SPI instance data.
//! \param ui32SPICSBase is the CS GPIO base SPI device to access.
//! \param ui8SPICSPin is the CS GPIO pin of the SPI device to access.
//! \param pui8WriteData is a pointer to the data buffer to be written.
//! \param pui8ReadData is a pointer to the buffer to be filled with the read
//! data.
//! \param ui16TransferCount is the number of bytes to be transferred.
//! \param pfnCallback is the function to be called when the transfer has
//! completed (can be \b NULL if a callback is not required).
//! \param pvCallbackData is a pointer that is passed to the callback function.
//!
//! This function adds an SPI transfer to the queue of commands to be sent.  If
//! successful, the SPI transfer is then performed in the background using the
//! interrupt handler.  When the transfer is complete, the callback function, if
//! provided, is called in the context of the SPI interrupt handler.
//!
//! \return Returns 1 if the command was successfully added to the queue and 0
//! if it was not.
//
// The extern here provides a non-inline definition for this function to handle
// the case where the compiler chooses not to inline the function (which is a
// valid choice for the compiler to make).
//
//*****************************************************************************
inline uint_fast8_t SPIMTransfer(tSPIMInstance *psInst, uint32_t ui32SPICSBase,
    uint8_t ui8SPICSPin, const uint8_t *pui8WriteData, uint8_t *pui8ReadData,
    uint_fast16_t ui16TransferCount, tSPICallback *pfnCallback,
    void *pvCallbackData);

//*****************************************************************************
//
//! Transfers data in batches from an SPI device.
//!
//! \param psInst is a pointer to the SPI instance data.
//! \param ui32SPICSBase is the CS GPIO base SPI device to access.
//! \param ui8SPICSPin is the CS GPIO pin of the SPI device to access.
//! \param pui8WriteData is a pointer to the data buffer to be written.
//! \param pui8ReadData is a pointer to the buffer to be filled with the read
//! data.
//! \param ui16TransferCount is the number of bytes to be transferred.
//! \param ui16TransferBatchSize is the number of bytes in each transfer batch.
//! \param pfnCallback is the function to be called when the transfer has
//! completed (can be \b NULL if a callback is not required).
//! \param pvCallbackData is a pointer that is passed to the callback function.
//!
//! This function adds an SPI read to the queue of commands to be sent.  If
//! successful, the SPI read is then performed in the background using the
//! interrupt handler.  When the read is complete, the callback function, if
//! provided, is called in the context of the SPI interrupt handler.
//!
//! The data is transfered in batches of \e ui16TransferBatchSize.  The callback
//! function is called after each batch is transferred, and SPIMTransferResume()
//! must be called when the next batch should be transferred.
//!
//! \return Returns 1 if the command was successfully added to the queue and 0
//! if it was not.
//
// The extern here provides a non-inline definition for this function to handle
// the case where the compiler chooses not to inline the function (which is a
// valid choice for the compiler to make).
//
//*****************************************************************************
extern uint_fast8_t SPIMTransferBatched(tSPIMInstance *psInst,
    uint32_t ui32SPICSBase, uint8_t ui8SPICSPin, const uint8_t *pui8WriteData,
    uint8_t *pui8ReadData, uint_fast16_t ui16TransferCount,
    uint_fast16_t ui16TransferBatchSize, tSPICallback *pfnCallback,
    void *pvCallbackData);

//*****************************************************************************
//
// This function handles the idle state of the SPI state machine.
//
//*****************************************************************************
static void SPIMStateIdle(tSPIMInstance *psInst, tSPIMCommand *pCommand) {
  //
  // Do nothing if there is not another transfer in the queue.
  //
  if (psInst->ui8ReadPtr == psInst->ui8WritePtr) return;

  //
  // See if there is any data to be transferred.
  // If there is, move to the transfer start state
  //
  if (pCommand->ui16TransferCount != 0) psInst->ui8State = STATE_TRANSFER_START;
}

//*****************************************************************************
//
// This function handles the transfer start state of the SPI state machine.
//
//*****************************************************************************
static void SPIMStateTransferStart(tSPIMInstance *psInst,
    tSPIMCommand *pCommand) {
  //
  // Clear the receive FIFO
  //
  SPIMFlushRxFIFO(psInst->ui32Base);

  //
  // Select the chip to start the transfer
  //
  SELECT(pCommand->ui32SPICSBase, pCommand->ui8SPICSPin);

  //
  // Set transfer indexes to 0 to start transaction
  //
  psInst->ui16TxIndex = psInst->ui16RxIndex = 0;

  //
  // Determine if the transfer is using the processor or using uDMA
  // by checking if the uDMA channels are set to 0xFF
  //
  if (psInst->ui8RxDMA == 0xFF && psInst->ui8TxDMA == 0xFF) {
    //
    // Move to the byte transfer state using the processor
    //
    psInst->ui8State = STATE_TRANSFER_TX;
  } else {
    //
    // Move to the byte transfer state using uDMA
    //
    psInst->ui8State = STATE_TRANSFER_DMA;
  }
}

//*****************************************************************************
//
// This function handles the transfer TX state of the SPI state machine.
//
//*****************************************************************************
static void SPIMStateTransferDMA(tSPIMInstance *psInst, tSPIMCommand *pCommand) {
  //
  // Keep track of how many bytes to transfer
  //
  uint16_t ui16Count;

  //
  // If the RX transfer isn't done, there's nothing to do yet
  //
  if (!SPIMIsDMAStopped(psInst->ui8RxDMA)) return;

  //
  // See if we transferred everything (Rx finished implies Tx has finished)
  //
  if (psInst->ui16RxIndex == pCommand->ui16TransferCount) {
    //
    // Move to the transfer end state.
    //
    psInst->ui8State = STATE_TRANSFER_END;
  }
  //
  // See if the transfer batch has been sent.
  //
  else if (psInst->ui16RxIndex == pCommand->ui16TransferBatchSize) {
    //
    // Move to the transfer pause state.
    //
    psInst->ui8State = STATE_TRANSFER_PAUSE;

    //
    // Call the callback function.
    //
    if (pCommand->pfnCallback)
      pCommand->pfnCallback(pCommand->pvCallbackData, SPIM_STATUS_BATCH_DONE);
  } else if (psInst->ui16TxIndex != pCommand->ui16TransferBatchSize) {
    //
    // The uDMA controller can only transfer 1024 bytes in a single transaction
    // when using basic mode.  Determine how to handle the transaction by splitting
    // it if we need to.
    //
    if ((pCommand->ui16TransferBatchSize - psInst->ui16TxIndex) <= 1024)
    {
      //
      // Set the transfer size to the batch size (Batch Size <= Transfer Count)
      //
      ui16Count = pCommand->ui16TransferBatchSize - psInst->ui16TxIndex;
    }
    else
    {
      //
      // The total transfer batch size (or transfer count if batch mode isn't used) is greater
      // than our uDMA window size so we need to break up the write transaction.
      //
      ui16Count = 1024;
    }

    //
    // Set up the RX DMA transfer
    //
    MAP_uDMAChannelTransferSet(psInst->ui8RxDMA | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC,
                               (void *) (psInst->ui32Base + SSI_O_DR),
                               (void *) &(pCommand->pui8ReadData[psInst->ui16RxIndex]),
                               ui16Count);

    //
    // Set up the TX DMA transfer
    //
    MAP_uDMAChannelTransferSet(psInst->ui8TxDMA | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC,
                               (void *) &(pCommand->pui8WriteData[psInst->ui16TxIndex]),
                               (void *) (psInst->ui32Base + SSI_O_DR),
                               ui16Count);

    //
    // Increment the transfer indexes appropriately
    //
    psInst->ui16RxIndex += ui16Count;
    psInst->ui16TxIndex += ui16Count;

    //
    // Fire off the uDMA transactions
    //
    MAP_uDMAChannelEnable(psInst->ui8RxDMA);
    MAP_uDMAChannelEnable(psInst->ui8TxDMA);
  }
}

//*****************************************************************************
//
// This function handles the transfer TX state of the SPI state machine.
//
//*****************************************************************************
static void SPIMStateTransferTX(tSPIMInstance *psInst, tSPIMCommand *pCommand) {
  //
  // If we still have data to transfer out (Batch Size <= Transfer Count so use batch size to check)
  // and we haven't filled the RX FIFO, put data into the TX FIFO
  //
  while ((psInst->ui16TxIndex != pCommand->ui16TransferBatchSize)
      && !(HWREG(psInst->ui32Base + SSI_O_SR) & SSI_SR_RFF)) {
    //
    // Put next TX byte in the SSI FIFO.
    // If there is no room, revert TxIndex and break loop.
    //
    if (!MAP_SSIDataPutNonBlocking(psInst->ui32Base,
        pCommand->pui8WriteData[psInst->ui16TxIndex++])) {
      psInst->ui16TxIndex--;
      break;
    }
  }

  //
  // Move to the transfer RX state
  //
  psInst->ui8State = STATE_TRANSFER_RX;
}

//*****************************************************************************
//
// This function handles the transfer RX state of the SPI state machine.
//
//*****************************************************************************
static void SPIMStateTransferRX(tSPIMInstance *psInst, tSPIMCommand *pCommand) {
  uint32_t ui32RxData;

  //
  // If we still need to receive data, grab it now if its available
  //
  while ((psInst->ui16TxIndex != psInst->ui16RxIndex)
      && MAP_SSIDataGetNonBlocking(psInst->ui32Base, &ui32RxData)) {
    pCommand->pui8ReadData[psInst->ui16RxIndex++] = ui32RxData;
  }

  //
  // See if we transferred everything (Rx finished implies Tx has finished)
  //
  if (psInst->ui16RxIndex == pCommand->ui16TransferCount) {
    //
    // Move to the transfer end state.
    //
    psInst->ui8State = STATE_TRANSFER_END;
  }
  //
  // See if the transfer batch has been sent.
  //
  else if (psInst->ui16RxIndex == pCommand->ui16TransferBatchSize) {
    //
    // Move to the transfer pause state.
    //
    psInst->ui8State = STATE_TRANSFER_PAUSE;

    //
    // Call the callback function.
    //
    if (pCommand->pfnCallback)
      pCommand->pfnCallback(pCommand->pvCallbackData, SPIM_STATUS_BATCH_DONE);
  }
  //
  // If we have finished the current block transfer (Rx == Tx) and there's still more to transfer
  // move back to the TX state
  //
  else if ((psInst->ui16RxIndex == psInst->ui16TxIndex)
      && (psInst->ui16TxIndex != pCommand->ui16TransferCount)) {
    psInst->ui8State = STATE_TRANSFER_TX;
  }
}

//*****************************************************************************
//
// This function handles the transfer final state of the SPI state machine.
//
//*****************************************************************************
static void SPIMStateTransferEnd(tSPIMInstance *psInst, tSPIMCommand *pCommand) {
  //
  // Deselect the chip to finalize the transfer
  //
  DESELECT(pCommand->ui32SPICSBase, pCommand->ui8SPICSPin);

  //
  // The next state is the callback state.
  //
  psInst->ui8State = STATE_CALLBACK;
}

//*****************************************************************************
//
// This function handles the transfer pause state of the SPI state machine.
//
//*****************************************************************************
static void SPIMStateTransferPause(tSPIMInstance *psInst,
    tSPIMCommand *pCommand) {
  //
  // Decrement the transfer count by the batch size.
  // If the batch size is larger than the count, make batch equivalent to count
  //
  pCommand->ui16TransferCount -= pCommand->ui16TransferBatchSize;
  if (pCommand->ui16TransferCount < pCommand->ui16TransferBatchSize)
    pCommand->ui16TransferBatchSize = pCommand->ui16TransferCount;

  //
  // Set transfer indexes to 0 to start transaction
  //
  psInst->ui16TxIndex = psInst->ui16RxIndex = 0;

  //
  // Move to the byte transfer state
  //
  psInst->ui8State = STATE_TRANSFER_TX;
}

//*****************************************************************************
//
// This function handles the callback state of the SPI state machine.
//
//*****************************************************************************
static void SPIMStateCallback(tSPIMInstance *psInst, tSPIMCommand *pCommand,
    uint32_t ui32Status) {
  tSPICallback *pfnCallback;
  void *pvCallbackData;

  //
  // Save the callback information.
  //
  pfnCallback = pCommand->pfnCallback;
  pvCallbackData = pCommand->pvCallbackData;

  //
  // This command has been completed, so increment the read pointer.
  //
  psInst->ui8ReadPtr++;
  if (psInst->ui8ReadPtr == NUM_SPIM_COMMANDS) psInst->ui8ReadPtr = 0;

  //
  // If there is a callback function then call it now.
  //
  if (pfnCallback) pfnCallback(pvCallbackData, SPIM_STATUS_SUCCESS);

  //
  // The state machine is now idle.
  //
  psInst->ui8State = STATE_IDLE;
}

//*****************************************************************************
//
//! Handles SPI interrupts.
//!
//! \param psInst is a pointer to the SPI instance data.
//!
//! This function performs the processing required in response to an SPI
//! interrupt.  The application-supplied interrupt handler should call this
//! function with the correct instance data in response to the SPI interrupt.
//!
//! \return None.
//
//*****************************************************************************
void SPIMIntHandler(tSPIMInstance *psInst) {
  tSPIMCommand *pCommand;
  uint32_t ui32Status;

  //
  // Get the interrupt status
  //
  ui32Status = MAP_SSIIntStatus(psInst->ui32Base, true);

  //
  // Clear the SPI interrupt.
  //
  MAP_SSIIntClear(psInst->ui32Base, ui32Status);

  //
  // Get a pointer to the current command.
  //
  pCommand = &(psInst->pCommands[psInst->ui8ReadPtr]);

  //
  // Loop forever.  Most states will return when they have completed their
  // action.  However, a few states require multi-state processing, so those
  // states will break and this loop repeated.
  //
  while (1) {
    //
    // Determine what to do based on the current state.
    //
    switch (psInst->ui8State) {
      //
      // The idle state.
      //
      case STATE_IDLE: {
        //
        // Handle the idle state.
        //
        SPIMStateIdle(psInst, pCommand);

        //
        // If we are still idle, leave the ISR,
        // otherwise we have a new state to handle
        //
        if (psInst->ui8State == STATE_IDLE) {
          return;
        } else {
          break;
        }
      }
        //
        // The transfer start state.
        //
      case STATE_TRANSFER_START: {
        //
        // Handle the transfer start state.
        //
        SPIMStateTransferStart(psInst, pCommand);

        //
        // This state is done and the next state needs to be handled
        // immediately.
        //
        break;
      }
      case STATE_TRANSFER_DMA: {
        SPIMStateTransferDMA(psInst, pCommand);

        //
        // If the transfer is done, stay in the loop and handle it
        // otherwise, exit the ISR
        //
        if (psInst->ui8State == STATE_TRANSFER_END) {
          break;
        } else {
          return;
        }
      }
        //
        // The state for the TX portion of the data transfer.
        //
      case STATE_TRANSFER_TX: {
        //
        // Handle the transfer data state.
        //
        SPIMStateTransferTX(psInst, pCommand);

        //
        // TX state finished. SPI RX interrupt will pull us back in to handle RX state
        //
        return;
      }
        //
        // The state for the RX portion of the data transfer.
        //
      case STATE_TRANSFER_RX: {
        //
        // Handle the transfer data state.
        //
        SPIMStateTransferRX(psInst, pCommand);

        //
        // If the transfer is done, stay in the loop and handle it
        // otherwise, exit the ISR
        //
        if (psInst->ui8State == STATE_TRANSFER_END) {
          break;
        } else {
          return;
        }
      }
        //
        // The state for the final transfer of a burst sequence.
        //
      case STATE_TRANSFER_END: {
        //
        // Handle the transfer final state.
        //
        SPIMStateTransferEnd(psInst, pCommand);

        //
        // This state is done and the next state needs to be handled
        // immediately.
        //
        break;
      }
        //
        // The state for a paused transfer.
        //
      case STATE_TRANSFER_PAUSE: {
        //
        // Handle the transfer pause state.
        //
        SPIMStateTransferPause(psInst, pCommand);

        //
        // This state is done and the next state should be handled at
        // the next interrupt.
        //
        break;
      }
        //
        // This state is for providing the transaction complete callback.
        //
      case STATE_CALLBACK: {
        //
        // Handle the callback state.
        //
        SPIMStateCallback(psInst, pCommand, ui32Status);

        //
        // Update the pointer to the current command.
        //
        pCommand = &(psInst->pCommands[psInst->ui8ReadPtr]);

        //
        // This state is done and the next state needs to be handled
        // immediately.
        //
        break;
      }
    }
  }
}

//*****************************************************************************
//
//! Initializes the SPI driver.
//!
//! \param psInst is a pointer to the SPI instance data.
//! \param ui32Base is the base address of the SPI module.
//! \param ui8Int is the interrupt number for the SPI module.
//! \param ui8TxDMA is the uDMA channel number used for transmitting data to
//! the SPI module.
//! \param ui8RxDMA is the uDMA channel number used for receiving data from
//! the SPI module.
//! \param ui32Clock is the clock frequency of the input clock to the SSI
//! module.
//! \param ui32BitRate is the bit rate for the SSI. This bit rate must satisfy
//! the following clock ratio criteria: ui32Clock >= 2 * ui32BitRate
//!
//! This function prepares both the SPI module and driver for operation,
//! and must be the first SPI driver function called for each SPI instance.
//! It is assumed that the application has enabled the SPI module,
//! configured the SPI pins, and provided an SPI interrupt handler that calls
//! SPIMIntHandler().
//!
//! The uDMA module cannot be used at present to transmit/receive data, so the
//! \e ui8TxDMA and \e ui8RxDMA parameters are unused.  They are reserved for
//! future use and should be set to 0xff in order to ensure future
//! compatibility.
//!
//! \return None.
//
//*****************************************************************************
void SPIMInit(tSPIMInstance *psInst, uint32_t ui32Base, uint_fast8_t ui8Int,
    uint_fast8_t ui8TxDMA, uint_fast8_t ui8RxDMA, uint32_t ui32Clock,
    uint32_t ui32BitRate) {
  //
  // Check the arguments.
  //
  ASSERT(psInst);ASSERT((ui32Base == SSI0_BASE) || (ui32Base == SSI1_BASE) || (ui32Base == SSI2_BASE) || (ui32Base == SSI3_BASE));ASSERT(ui8Int);ASSERT(0 < ui32Clock);ASSERT(0 < ui32BitRate && (ui32BitRate * 2) <= ui32Clock);

  //
  // Initialize the state structure.
  //
  psInst->ui32Base = ui32Base;
  psInst->ui8Int = ui8Int;
  psInst->ui8TxDMA = ui8TxDMA;
  psInst->ui8RxDMA = ui8RxDMA;
  psInst->ui8State = STATE_IDLE;
  psInst->ui8ReadPtr = 0;
  psInst->ui8WritePtr = 0;

  //
  // Initialize the SPI module.
  //
  MAP_SSIConfigSetExpClk(ui32Base, ui32Clock, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, ui32BitRate, 8);
  MAP_SSIEnable(ui32Base);

  //
  // Determine if we are using uDMA or the processor
  //
  //
  if (ui8TxDMA == 0xFF && ui8RxDMA == 0xFF) {
    MAP_SSIIntEnable(ui32Base, SSI_RXTO | SSI_RXFF);
  } else {
    //
    // Enable the SSI transmit and receive DMA channels.
    //
    MAP_SSIDMAEnable(ui32Base, SSI_DMA_TX | SSI_DMA_RX);

    //
    // The uDMA TX/RX channel must be disabled.
    //
    MAP_uDMAChannelDisable(ui8RxDMA);
    MAP_uDMAChannelDisable(ui8TxDMA);

    //
    // Configure the DMA channels for the selected SPI Module
    //
    MAP_uDMAChannelAssign(ui8TxDMA);
    MAP_uDMAChannelAssign(ui8RxDMA);

    //
    // Put the attributes in a known state for the uDMA SSI TX channel. These
    // should already be disabled by default.
    //
    MAP_uDMAChannelAttributeDisable(ui8TxDMA,
        UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);

    //
    // Set the USEBURST attribute for the uDMA SSI TX channel. This will
    // force the controller to always use a burst when transferring data from
    // the TX buffer to SSI.  This is somewhat more efficient bus usage
    // than the default which allows single or burst transfers.
    //
    MAP_uDMAChannelAttributeEnable(ui8TxDMA, UDMA_ATTR_USEBURST);

    //
    // Configure the control parameters for the SSI TX channel. The data size is 8 bits.
    // The source address increment is 8-bit bytes since the data is coming from a uint8_t buffer.
    // The destination increment is none since the data is to be written to the SSI data register.
    // The arbitration size is set to 4, which matches the SSI TX FIFO trigger
    // threshold.
    //
    MAP_uDMAChannelControlSet(ui8TxDMA | UDMA_PRI_SELECT,
        UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_4);

    //
    // Put the attributes in a known state for the uDMA SSI RX channel.  These
    // should already be disabled by default.
    //
    MAP_uDMAChannelAttributeDisable(ui8RxDMA,
        UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST | UDMA_ATTR_HIGH_PRIORITY
            | UDMA_ATTR_REQMASK);

    //
    // Configure the control parameters for the SSI RX channel. The data size is 8 bits.
    // The source address does not increment since it will be reading from a
    // register. The destination address increment is 1 byte buffers since the data is going into
    // a uint8_t buffer.  The  arbitration size is set to 4 to match the RX FIFO trigger threshold.
    // The uDMA controller will use a 4 byte burst transfer if possible.  This
    // will be somewhat more efficient than single byte transfers.
    //
    MAP_uDMAChannelControlSet(ui8RxDMA | UDMA_PRI_SELECT,
        UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_4);

    //
    // Now both the uDMA SSI TX and RX channels are primed to start a
    // transfer.  As soon as the channels are enabled, the peripheral will
    // issue a transfer request and the data transfers will begin.
    //
  }

  //
  // Enable the SPI interrupt.
  //
  MAP_IntEnable(ui8Int);
}

//*****************************************************************************
//
//! Sends a command to an SPI device.
//!
//! \param psInst is a pointer to the SPI instance data.
//! \param ui32SPICSBase is the CS GPIO base SPI device to access.
//! \param ui8SPICSPin is the CS GPIO pin of the SPI device to access.
//! \param pui8WriteData is a pointer to the data buffer to be written.
//! \param pui8ReadData is a pointer to the buffer to be filled with the read
//! data.
//! \param ui16TransferCount is the number of bytes to be transferred.
//! \param ui16TransferBatchSize is the number of bytes to be transferred in each batch.
//! \param pfnCallback is the function to be called when the transfer has
//! completed (can be \b NULL if a callback is not required).
//! \param pvCallbackData is a pointer that is passed to the callback function.
//!
//! This function adds an SPI command to the queue of commands to be sent.  If
//! successful, the SPI command is then transferred in the background using the
//! interrupt handler.  When the transfer is complete, the callback function,
//! if provided, is called in the context of the SPI interrupt handler.
//!
//! If \e ui16TransferBatchSize is less than \e ui16TransferCount, the transfer is
//! broken up into as many \e ui16TransferBatchSize batches as
//! required to transfer \e ui16TransferCount bytes.  After each batch, the callback
//! function is called with an \b SPIM_STATUS_BATCH_DONE status, and the
//! transfer is paused (with the SPI bus held).  The transfer is resumed when
//! SPIMTransferResume() is called.  This procedure can be used to perform very
//! large transfers without requiring all the data be available at once or a large
//! SRAM buffer, at the expense of tying up the SPI bus for the extended
//! duration of the transfer.
//!
//! \return Returns 1 if the command was successfully added to the queue and 0
//! if it was not.
//
//*****************************************************************************
uint_fast8_t SPIMCommand(tSPIMInstance *psInst, uint32_t ui32SPICSBase,
    uint8_t ui8SPICSPin, const uint8_t *pui8WriteData, uint8_t *pui8ReadData,
    uint_fast16_t ui16TransferCount, uint_fast16_t ui16TransferBatchSize,
    tSPICallback *pfnCallback, void *pvCallbackData) {
  uint_fast8_t ui8Next, ui8Enabled;
  tSPIMCommand *pCommand;

  //
  // Check the arguments.
  //
  ASSERT(psInst);ASSERT(ui32SPICSBase && ui8SPICSPin);ASSERT(pui8WriteData && pui8ReadData);ASSERT(!ui16TransferCount || (ui16TransferBatchSize > 0));

  //
  // Disable the SPI interrupt.
  //
  if (MAP_IntIsEnabled(psInst->ui8Int)) {
    ui8Enabled = 1;
    MAP_IntDisable(psInst->ui8Int);
  } else {
    ui8Enabled = 0;
  }

  //
  // Compute the new value of the write pointer (after this command is added
  // to the queue).
  //
  ui8Next = psInst->ui8WritePtr + 1;
  if (ui8Next == NUM_SPIM_COMMANDS) ui8Next = 0;

  //
  // Return a failure if the command queue is full.
  //
  if (psInst->ui8ReadPtr == ui8Next) {
    if (ui8Enabled) MAP_IntEnable(psInst->ui8Int);
    return (0);
  }

  //
  // Get a pointer to the command structure.
  //
  pCommand = &(psInst->pCommands[psInst->ui8WritePtr]);

  //
  // Fill in the command structure with the details of this command.
  //
  pCommand->ui32SPICSBase = ui32SPICSBase;
  pCommand->ui8SPICSPin = ui8SPICSPin;
  pCommand->pui8WriteData = pui8WriteData;
  pCommand->pui8ReadData = pui8ReadData;
  pCommand->ui16TransferCount = ui16TransferCount;
  pCommand->ui16TransferBatchSize = ui16TransferBatchSize;
  pCommand->pfnCallback = pfnCallback;
  pCommand->pvCallbackData = pvCallbackData;

  //
  // Update the write pointer.
  //
  psInst->ui8WritePtr = ui8Next;

  //
  // If we are idle, generate a fake SPI interrupt, which will commence the SPI transfer.
  //
  if (psInst->ui8State == STATE_IDLE) IntTrigger(psInst->ui8Int);

  //
  // Re-enable the I2C master interrupt.
  //
  if (ui8Enabled) MAP_IntEnable(psInst->ui8Int);

  //
  // Success.
  //
  return (1);
}

//*****************************************************************************
//
//! Resumes an SPI transfer.
//!
//! \param psInst is a pointer to the SPI instance data.
//! \param pui8Data is a pointer to the buffer to be used for the next batch of
//! data.
//!
//! This function resumes an SPI transfer that has been paused via the use of
//! the transfer batch size capability.
//!
//! \return Returns 1 if the transfer was resumed and 0 if there was not a
//! paused transfer to resume.
//
//*****************************************************************************
uint_fast8_t SPIMTransferResume(tSPIMInstance *psInst,
    const uint8_t *pui8WriteData, uint8_t *pui8ReadData) {
  //
  // Check the arguments.
  //
  ASSERT(psInst);ASSERT(pui8Data);

  //
  // Return an error if there is not a paused transfer.
  //
  if (psInst->ui8State != STATE_TRANSFER_PAUSE) return (0);

  //
  // Save the pointer for the next buffer.
  //
  psInst->pCommands[psInst->ui8ReadPtr].pui8WriteData = pui8WriteData;
  psInst->pCommands[psInst->ui8ReadPtr].pui8ReadData = pui8ReadData;

  //
  // Trigger the SPI interrupt, resuming the transfer.
  //
  IntTrigger(psInst->ui8Int);

  //
  // Success.
  //
  return (1);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
