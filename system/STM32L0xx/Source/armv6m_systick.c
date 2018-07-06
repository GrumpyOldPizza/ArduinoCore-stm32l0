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

#include "armv6m.h"

extern void SysTick_Handler(void);

typedef struct _armv6m_systick_control_t {
    volatile uint32_t         micros;
    volatile uint32_t         millis;
    uint32_t                  clock;
    uint32_t                  cycle;
    uint32_t                  scale[2];
} armv6m_systick_control_t;

static armv6m_systick_control_t armv6m_systick_control;

void armv6m_systick_initialize(void)
{
    NVIC_SetPriority(SysTick_IRQn, 0);
}

void armv6m_systick_enable()
{
    uint32_t cycle;

    cycle = SystemCoreClock / 8 -1;

    armv6m_systick_control.clock = SystemCoreClock;
    armv6m_systick_control.cycle = cycle;

    /* To get from the current counter to the microsecond offset,
     * the ((cycle - 1) - Systick->VAL) value is scaled so that the resulting
     * microseconds fit into the upper 17 bits of a 32bit value. Then
     * this is post divided by 2^15. That ensures proper scaling.
     *
     * For millisconds we can use the upper 7 bits of a 32bit value.
     */
    
    armv6m_systick_control.scale[0] = (uint64_t)32768000000ull / (uint64_t)SystemCoreClock;
    armv6m_systick_control.scale[1] = (uint64_t)33554432000ull / (uint64_t)SystemCoreClock;

    armv6m_systick_control.micros = 0;
    armv6m_systick_control.millis = 0;

    SysTick->VAL = cycle;
    SysTick->LOAD = cycle;
    SysTick->CTRL = (SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk);
}

void armv6m_systick_disable(void)
{
    SysTick->CTRL = 0;
}

uint32_t armv6m_systick_micros(void)
{
    uint32_t micros, count;
    
    do
    {
        micros = armv6m_systick_control.micros;
        count = SysTick->VAL;

        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
        {
            armv6m_systick_control.micros += 125000;
            armv6m_systick_control.millis += 125;
        }
    }
    while (micros != armv6m_systick_control.micros);

    micros += (((armv6m_systick_control.cycle - count) * armv6m_systick_control.scale[0]) >> 15);

    return micros;
}

uint32_t armv6m_systick_millis(void)
{
    uint32_t millis, count;
    
    do
    {
        millis = armv6m_systick_control.millis;
        count = SysTick->VAL;

        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
        {
            armv6m_systick_control.micros += 125000;
            armv6m_systick_control.millis += 125;
        }
    }
    while (millis != armv6m_systick_control.millis);

    millis += (((armv6m_systick_control.cycle - count) * armv6m_systick_control.scale[1]) >> 25);
    
    return millis;
}

void armv6m_systick_delay(uint32_t delay)
{
    uint32_t millis;

    millis = armv6m_systick_millis();
    
    do
    {
    }
    while ((armv6m_systick_millis() - millis) < delay);
}

void SysTick_Handler(void)
{
    if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
    {
        armv6m_systick_control.micros += 125000;
        armv6m_systick_control.millis += 125;
    }
}
