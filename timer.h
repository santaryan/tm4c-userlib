/*
 * timer.h
 *
 *  Created on: Nov 8, 2015
 *      Author: Ryan
 */

#ifndef PERIPHERALS_TIMER_H_
#define PERIPHERALS_TIMER_H_

#include <stdbool.h>
#include <stdint.h>

void TimerFullWidthInit(uint32_t ui32Base, uint32_t ui32Config, uint32_t ui32Value);
uint32_t TimerFullWidthIntInit(uint32_t ui32Base, int16_t i16Priority, uint32_t ui32IntFlags, void (*pfnHandler)(void));

#endif /* PERIPHERALS_TIMER_H_ */
