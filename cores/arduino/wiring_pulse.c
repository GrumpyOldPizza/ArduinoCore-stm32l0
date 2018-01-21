/*
 * Copyright (c) 2016-2018 Thomas Roell.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#include "Arduino.h"
#include "wiring_private.h"

static inline __attribute__((optimize("O3"),always_inline)) uint32_t countPulseInline(const volatile uint32_t *port, uint32_t bit, uint32_t stateMask, unsigned long maxloops)
{
    uint32_t micros;

    // wait for any previous pulse to end
    while ((*port & bit) == stateMask)
    {
	if (--maxloops == 0)
	{
	    return 0;
	}
    }
    
    // wait for the pulse to start
    while ((*port & bit) != stateMask)
    {
	if (--maxloops == 0)
	{
	    return 0;
	}
    }

    micros = armv6m_systick_micros();

    // wait for the pulse to stop
    while ((*port & bit) == stateMask)
    {
	if (--maxloops == 0)
	{
	    return 0;
	}
    }
    
    return (uint32_t)armv6m_systick_micros() - micros;
}

/* Measures the length (in microseconds) of a pulse on the pin; state is HIGH
 * or LOW, the type of pulse to measure.  Works on pulses from 2-3 microseconds
 * to 3 minutes in length, but must be called at least a few dozen microseconds
 * before the start of the pulse. */
uint32_t pulseIn(uint32_t pin, uint32_t state, uint32_t timeout)
{
    if ( (pin >= PINS_COUNT) || (g_APinDescription[pin].GPIO == NULL) ) {
	return 0;
    }

    // cache the port and bit of the pin in order to speed up the
    // pulse width measuring loop and achieve finer resolution.  calling
    // digitalRead() instead yields much coarser resolution.
    GPIO_TypeDef *GPIO = g_APinDescription[pin].GPIO;
    uint32_t bit = g_APinDescription[pin].bit;
    uint32_t stateMask = state ? bit : 0;

    // convert the timeout from microseconds to a number of times through
    // the initial loop; it takes (roughly) 8 clock cycles per iteration.
    uint32_t maxloops = microsecondsToClockCycles(timeout) / 8;

    return countPulseInline(&GPIO->IDR, bit, stateMask, maxloops);
}

