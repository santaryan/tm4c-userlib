/*
 * gpio.h
 *
 *  Created on: Feb 22, 2015
 *      Author: Ryan
 */

#ifndef __GPIO_H__
#define __GPIO_H__

#include <stdbool.h>
#include <stdint.h>

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

extern uint8_t GPIOPinLow(uint32_t ui32Port, uint8_t ui8Pins);
extern uint8_t GPIOPinHigh(uint32_t ui32Port, uint8_t ui8Pins);
extern void GPIOInputInit(uint32_t ui32Port, uint8_t ui8Pins, uint32_t ui32PinType);
extern void GPIOOutputInit(uint32_t ui32Port, uint8_t ui8Pins, uint32_t ui32Strength, uint32_t ui32PinType);
extern void GPIOOutputODInit(uint32_t ui32Port, uint8_t ui8Pins, uint32_t ui32Strength, uint32_t ui32PinType);
extern uint32_t GPIOIntInit(uint32_t ui32Port, uint8_t ui8Pins, uint32_t ui32IntType, int16_t i16Priority, void (*pfnHandler)(void));
extern void GPIOUnlock(uint32_t ui32Port);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif


#endif /* __GPIO_H__ */
