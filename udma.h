/*
 * udma.h
 *
 *  Created on: Nov 16, 2016
 *      Author: clausr
 */

#ifndef _UDMA_H_
#define _UDMA_H_

#include <stdbool.h>
#include <stdint.h>

#define DMA_CHANNEL_CONTROL_STRUCTURE_SIZE 1024

//*****************************************************************************
//
// Define the transmit and receive buffers along with the control structure
// table for the DMA engine.  Unfortunately, C99 doesn't provide us with a
// standard method of dictating the alignment of variables or indicating that
// an array should not be initialized so we need some compiler-specific
// syntax here.
//
//*****************************************************************************
//
// Case for Code Composer Studio.
//
#if defined(__CCS__) || defined(ccs)
#pragma DATA_ALIGN(ui8DMAChannelControlStructure, 1024);
static uint8_t ui8DMAChannelControlStructure[DMA_CHANNEL_CONTROL_STRUCTURE_SIZE];

//
// Case for IAR Embedded Workbench for ARM (ewarm).
//
#elif defined(__IAR_SYSTEMS_ICC__) || defined(ewarm)
#pragma data_alignment=1024
__no_init static uint8_t ui8DMAChannelControlStructure[DMA_CHANNEL_CONTROL_STRUCTURE_SIZE];

//
// Case for Sourcery CodeBench, GCC, and Keil RVMDK.
//
#else
static uint8_t ui8DMAChannelControlStructure[DMA_CHANNEL_CONTROL_STRUCTURE_SIZE] __attribute__ ((aligned(1024)));
#endif

extern uint32_t uDMAGetLastError();
extern uint32_t uDMAGetErrorCount();
extern void uDMAInit(void);

#endif /* _UDMA_H_ */
