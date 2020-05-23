/*
 * Copyright (c) 2017-2020 Thomas Roell.  All rights reserved.
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

#if !defined(_STM32L0_LPTIM_H)
#define _STM32L0_LPTIM_H

#include "armv6m.h"
#include "stm32l0xx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STM32L0_LPTIM_IRQ_PRIORITY   1

typedef struct _stm32l0_lptim_timeout_t stm32l0_lptim_timeout_t;

typedef void (*stm32l0_lptim_callback_t)(stm32l0_lptim_timeout_t *timeout);

struct _stm32l0_lptim_timeout_t {
    stm32l0_lptim_timeout_t           *next;
    stm32l0_lptim_timeout_t           *previous;
    uint32_t                          clock;
    volatile stm32l0_lptim_callback_t callback;
};

#define STM32L0_LPTIM_TIMEOUT_INIT { NULL, NULL, 0, NULL }
#define STM32L0_LPTIM_TIMEOUT_TICKS_PER_SECOND 32768

extern void __stm32l0_lptim_initialize(void);
extern void stm32l0_lptim_timeout_create(stm32l0_lptim_timeout_t *timeout);
extern void stm32l0_lptim_timeout_destroy(stm32l0_lptim_timeout_t *timeout);
extern bool stm32l0_lptim_timeout_start(stm32l0_lptim_timeout_t *timeout, uint32_t ticks, stm32l0_lptim_callback_t callback);
extern bool stm32l0_lptim_timeout_stop(stm32l0_lptim_timeout_t *timeout);
extern bool stm32l0_lptim_timeout_done(stm32l0_lptim_timeout_t *timeout);

static inline uint32_t stm32l0_lptim_micros_to_ticks(uint32_t micros)
{
    if (micros < 1000000)
    {
        return ((micros * STM32L0_LPTIM_TIMEOUT_TICKS_PER_SECOND + 999999) / 1000000);
    }
    else
    {   
        uint32_t seconds;

        seconds = (micros / 1000000);
        micros = micros - seconds * 1000000;

        return (seconds * STM32L0_LPTIM_TIMEOUT_TICKS_PER_SECOND) + ((micros * STM32L0_LPTIM_TIMEOUT_TICKS_PER_SECOND + 999999) / 1000000);
    }
}

static inline uint32_t stm32l0_lptim_millis_to_ticks(uint32_t millis)
{
    if (millis < 60000)
    {
        return ((millis * STM32L0_LPTIM_TIMEOUT_TICKS_PER_SECOND + 999) / 1000);
    }
    else
    {
        uint32_t seconds;

        seconds = (millis / 1000);
        millis = millis - seconds * 1000;

        return (seconds * STM32L0_LPTIM_TIMEOUT_TICKS_PER_SECOND) + ((millis * STM32L0_LPTIM_TIMEOUT_TICKS_PER_SECOND + 999) / 1000);
    }
}

static inline uint32_t stm32l0_lptim_seconds_to_ticks(uint32_t seconds)
{
    return (seconds * STM32L0_LPTIM_TIMEOUT_TICKS_PER_SECOND);
}

static inline uint32_t stm32l0_lptim_ticks_to_micros(uint32_t ticks)
{
    uint32_t seconds;

    seconds = ticks / STM32L0_LPTIM_TIMEOUT_TICKS_PER_SECOND;
    ticks = ticks & (STM32L0_LPTIM_TIMEOUT_TICKS_PER_SECOND -1);

    return (seconds * 1000000) + ((ticks * 10000000) / STM32L0_LPTIM_TIMEOUT_TICKS_PER_SECOND);
}

static inline uint32_t stm32l0_lptim_ticks_to_millis(uint32_t ticks)
{
    uint32_t seconds;

    seconds = ticks / STM32L0_LPTIM_TIMEOUT_TICKS_PER_SECOND;
    ticks = ticks & (STM32L0_LPTIM_TIMEOUT_TICKS_PER_SECOND -1);

    return (seconds * 1000) + ((ticks * 1000) / STM32L0_LPTIM_TIMEOUT_TICKS_PER_SECOND);
}

static inline uint32_t stm32l0_lptim_ticks_to_seconds(uint32_t ticks)
{
    return ticks / STM32L0_LPTIM_TIMEOUT_TICKS_PER_SECOND;
}

#ifdef __cplusplus
}
#endif

#endif /* _STM32L0_LPTIM_H */
