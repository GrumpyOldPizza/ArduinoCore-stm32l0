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

void attachInterrupt(uint32_t pin, voidFuncPtr callback, uint32_t mode)
{
    if ((pin >= PINS_COUNT) || !(g_APinDescription[pin].attr & (PIN_ATTR_EXTI | PIN_ATTR_TAMP)) || !callback) {
	return;
    }

    if (g_APinDescription[pin].attr & PIN_ATTR_EXTI) {
	switch (mode) {
	case CHANGE:
	    stm32l0_exti_attach(g_APinDescription[pin].pin, (STM32L0_EXTI_CONTROL_EDGE_FALLING | STM32L0_EXTI_CONTROL_EDGE_RISING), (stm32l0_exti_callback_t)callback, NULL);
	    break;
	case FALLING:
	    stm32l0_exti_attach(g_APinDescription[pin].pin, STM32L0_EXTI_CONTROL_EDGE_FALLING, (stm32l0_exti_callback_t)callback, NULL);
	    break;
	case RISING:
	    stm32l0_exti_attach(g_APinDescription[pin].pin, STM32L0_EXTI_CONTROL_EDGE_RISING, (stm32l0_exti_callback_t)callback, NULL);
	    break;
	default:
	    break;
	}
    } else {
	switch (mode) {
	case FALLING:
	    stm32l0_rtc_tamp_attach(g_APinDescription[pin].pin, STM32L0_RTC_TAMP_CONTROL_EDGE_FALLING, (stm32l0_rtc_callback_t)callback, NULL);
	    break;
	case RISING:
	    stm32l0_rtc_tamp_attach(g_APinDescription[pin].pin, STM32L0_RTC_TAMP_CONTROL_EDGE_RISING, (stm32l0_rtc_callback_t)callback, NULL);
	    break;
	default:
	    break;
	}
    }
}

/*
 * \brief Turns off the given interrupt.
 */
void detachInterrupt(uint32_t pin)
{
    if ((pin >= PINS_COUNT) || !(g_APinDescription[pin].attr & (PIN_ATTR_EXTI | PIN_ATTR_TAMP))) {
	return;
    }

    if (g_APinDescription[pin].attr & PIN_ATTR_EXTI) {
	stm32l0_exti_detach(g_APinDescription[pin].pin);
    } else {
	stm32l0_rtc_tamp_detach(g_APinDescription[pin].pin);
    }
}
