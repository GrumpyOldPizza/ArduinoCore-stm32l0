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
#include "TimerMillis.h"
#include "wiring_private.h"

TimerMillis::TimerMillis()
{
    _timer = new stm32l0_rtc_timer_t;

    if (_timer) {
	stm32l0_rtc_timer_create(_timer, (stm32l0_rtc_timer_callback_t)timeout, (void*)this);
    }

    _adjust = 0xffff;
}

TimerMillis::~TimerMillis()
{
    if (_timer) {
	stm32l0_rtc_timer_destroy(_timer);
    }
}

int TimerMillis::start(Callback callback, uint32_t delay, uint32_t period)
{
    uint32_t seconds;
    uint16_t subseconds, adjust;

    if (!_timer) {
	return 0;
    }

    if (active()) {
	return 0;
    }

    if (delay == 0) {
	if (period == 0) {
	    return 0;
	}

	delay = period;
    }

    _callback = callback;

    stm32l0_rtc_get_time(&seconds, &subseconds);
    stm32l0_rtc_millis_to_time(delay, &_seconds, &_subseconds);

    stm32l0_rtc_time_offset(_seconds, _subseconds, seconds, subseconds, &_seconds, &_subseconds);

    adjust = (_subseconds & (STM32L0_RTC_PREDIV_A -1)) ? (STM32L0_RTC_PREDIV_A - (_subseconds & (STM32L0_RTC_PREDIV_A -1))) : 0;

    _period = period;
    _subseconds += adjust;
    _adjust = adjust;

    stm32l0_rtc_timer_start(_timer, _seconds, _subseconds, true);

    return 1;
}

int TimerMillis::restart(uint32_t delay, uint32_t period)
{
    uint32_t seconds;
    uint16_t subseconds, adjust;

    if (!_timer) {
	return 0;
    }

    if (delay == 0) {
	if (period == 0) {
	    return 0;
	}

	delay = period;
    }

    _adjust = 0xffff;

    stm32l0_rtc_timer_stop(_timer);

    stm32l0_rtc_get_time(&seconds, &subseconds);
    stm32l0_rtc_millis_to_time(delay, &_seconds, &_subseconds);

    stm32l0_rtc_time_offset(_seconds, _subseconds, seconds, subseconds, &_seconds, &_subseconds);

    adjust = (_subseconds & (STM32L0_RTC_PREDIV_A -1)) ? (STM32L0_RTC_PREDIV_A - (_subseconds & (STM32L0_RTC_PREDIV_A -1))) : 0;

    _period = period;
    _subseconds += adjust;
    _adjust = adjust;

    stm32l0_rtc_timer_start(_timer, _seconds, _subseconds, true);

    return 1;
}

int TimerMillis::stop()
{
    if (!_timer) {
	return 0;
    }

    _adjust = 0xffff;

    stm32l0_rtc_timer_stop(_timer);

    return 1;
}

bool TimerMillis::active()
{
    return (_adjust != 0xffff);
}

void TimerMillis::timeout(class TimerMillis *self, stm32l0_rtc_timer_t *timer)
{
    uint32_t seconds;
    uint16_t subseconds, adjust;

    if (timer != NULL) {
	delete timer;
	return;
    }

    if (self->_adjust != 0xffff) {
	if (self->_period) {
	    stm32l0_rtc_millis_to_time(self->_period, &seconds, &subseconds);

	    stm32l0_rtc_time_offset(self->_seconds, self->_subseconds, seconds, subseconds, &self->_seconds, &self->_subseconds);
	    stm32l0_rtc_time_subtract(self->_seconds, self->_subseconds, 0, self->_adjust, (int32_t*)&self->_seconds, &self->_subseconds);

	    adjust = (self->_subseconds & (STM32L0_RTC_PREDIV_A -1)) ? (STM32L0_RTC_PREDIV_A - (self->_subseconds & (STM32L0_RTC_PREDIV_A -1))) : 0;

	    self->_subseconds += adjust;
	    self->_adjust = adjust;

	    stm32l0_rtc_timer_start(self->_timer, self->_seconds, self->_subseconds, true);
	} else {
	    self->_adjust = 0xffff;
	}

	self->_callback.queue();
    }
}
