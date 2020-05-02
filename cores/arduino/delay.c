/*
 * Copyright (c) 2017-2018 Thomas Roell.  All rights reserved.
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

unsigned long millis(void) 
{
    uint32_t seconds;
    uint16_t subseconds;

    stm32l0_rtc_get_time(&seconds, &subseconds);

    return (seconds * 1000) + ((subseconds * 1000) / 32768);
}

unsigned long micros(void) 
{
    return armv6m_systick_micros();
}

void delay(uint32_t msec) 
{
    uint32_t timeout, start;

    if (msec == 0)
	return;

    if (__get_IPSR() == 0) {
	do
	{
	    timeout = msec;

	    /* The wakeup timer can count either in terms of seconds (17 bit),
	     * of in 1/2048 seconds (16 bit). Hence choose the appropriate
	     * timeout ...
	     */

	    if (timeout > (128 * 1024 * 1000))
	    {
		timeout = (128 * 1024 * 1000);
	    }
	    else
	    {
		if (timeout > (32 * 1000)) {
		    // timeout = (timeout / (32 * 1000)) * (32 * 1000);
		    timeout = timeout - (timeout % (32 * 1000));
		}
	    }

	    stm32l0_rtc_wakeup_start(timeout, NULL, NULL);

	    do
	    {
		armv6m_core_wait();
	    }
	    while (!stm32l0_rtc_wakeup_done());

	    msec -= timeout;
	}
	while (msec);
	
    } else {
	start = millis();
	
	do
	{
	}
	while ((millis() - start) < msec);
    }
}
