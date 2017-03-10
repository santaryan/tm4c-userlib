/*
 * ssi.h
 *
 *  Created on: Feb 17, 2015
 *      Author: Ryan
 */

#ifndef __SPI_H_
#define __SPI_H_

#include <stdbool.h>
#include <stdint.h>

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
//
// Module function prototypes.
//
//*****************************************************************************
void SPIInit(uint32_t ui32Base, uint32_t ui32Protocol, uint32_t ui32Mode, uint32_t ui32BitRate, uint32_t ui32DataWidth);
void SPIFSSInit(uint32_t ui32SPICSBase, uint8_t ui8SPICSPin);
bool SPILoopback(uint32_t ui32Base, bool bEnable);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif /* __SPI_H_ */
