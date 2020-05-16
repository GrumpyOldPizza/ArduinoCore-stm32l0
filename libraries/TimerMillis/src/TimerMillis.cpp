/*
 * Copyright (c) 2016-2020 Thomas Roell.  All rights reserved.
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
#include "TimerMillis.h"
#include "wiring_private.h"

TimerMillis::TimerMillis()
{
    stm32l0_rtc_timer_create(&_timer, (stm32l0_rtc_timer_callback_t)TimerMillis::timeout, (void*)this);

    _clock = 0;
}

TimerMillis::~TimerMillis()
{
    if (!stm32l0_rtc_timer_destroy(&_timer)) {
	__BKPT();
    }
}

int TimerMillis::start(void(*callback)(void), uint32_t delay, uint32_t period)
{
    return start(Callback(callback), delay, period);
}

int TimerMillis::start(Callback callback, uint32_t delay, uint32_t period)
{
    uint64_t clock;
    uint32_t seconds, ticks;

    if (active()) {
	return 0;
    }

    if (delay == 0) {
	if (period == 0) {
	    return 0;
	}

	delay = period;
    }

    clock = stm32l0_rtc_clock_read();
    seconds = delay / 1000;
    ticks = ((delay - seconds * 1000) * STM32L0_RTC_CLOCK_TICKS_PER_SECOND + 999) / 1000;

    _clock = clock + (seconds * STM32L0_RTC_CLOCK_TICKS_PER_SECOND);
    _millis = delay - (seconds * 1000);
    _period = period;
    _callback = callback;
    
    stm32l0_rtc_timer_start(&_timer, _clock + ticks, STM32L0_RTC_TIMER_MODE_ABSOLUTE);

    return 1;
}

int TimerMillis::restart(uint32_t delay, uint32_t period)
{
    uint64_t clock;
    uint32_t seconds, ticks;

    if (delay == 0) {
	if (period == 0) {
	    return 0;
	}

	delay = period;
    }

    stm32l0_rtc_timer_stop(&_timer);

    clock = stm32l0_rtc_clock_read();
    seconds = delay / 1000;
    ticks = ((delay - seconds * 1000) * STM32L0_RTC_CLOCK_TICKS_PER_SECOND + 999) / 1000;

    _clock = clock + (seconds * STM32L0_RTC_CLOCK_TICKS_PER_SECOND);
    _millis = delay - (seconds * 1000);
    _period = period;

    stm32l0_rtc_timer_start(&_timer, _clock + ticks, STM32L0_RTC_TIMER_MODE_ABSOLUTE);

    return 1;
}

int TimerMillis::stop()
{
    stm32l0_rtc_timer_stop(&_timer);

    _clock = 0;

    return 1;
}

bool TimerMillis::active()
{
    return _clock != 0;
}

void TimerMillis::timeout(class TimerMillis *self)
{
    uint32_t seconds, ticks;

    if (self->_clock) {
	if (self->_period) {
	    self->_millis += self->_period;

	    seconds = self->_millis / 1000;
	    ticks = ((self->_millis - seconds * 1000) * STM32L0_RTC_CLOCK_TICKS_PER_SECOND + 999) / 1000;
	    
	    self->_clock += (seconds * STM32L0_RTC_CLOCK_TICKS_PER_SECOND);
	    self->_millis -= (seconds * 1000);
	    
	    stm32l0_rtc_timer_start(&self->_timer, self->_clock + ticks, STM32L0_RTC_TIMER_MODE_ABSOLUTE);
	} else {
	    self->_clock = 0;
	}

	self->_callback.queue(false);
    }
}
