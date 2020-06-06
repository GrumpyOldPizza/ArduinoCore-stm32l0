/*
 * Copyright (c) 2014-2020 Thomas Roell.  All rights reserved.
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

typedef struct _armv6m_core_control_t {
    uint32_t               clock;
    uint32_t               scale;
} armv6m_core_control_t;

static armv6m_core_control_t armv6m_core_control;

void __armv6m_core_initialize(void)
{
    __armv6m_svcall_initialize();
    __armv6m_pendsv_initialize();
    __armv6m_systick_initialize();
    __armv6m_work_initialize();
}

void armv6m_core_udelay(uint32_t delay)
{
    uint32_t n;

    if (armv6m_core_control.clock != SystemCoreClock)
    {
        armv6m_core_control.clock = SystemCoreClock;
        armv6m_core_control.scale = SystemCoreClock / 15625;
    }

    n = (delay * armv6m_core_control.scale + 255) / 256;

    __asm__ __volatile__(
                         "1: sub  %0, #1 \n"
                         "   nop         \n"
                         "   bne  1b     \n"
                         : "+r" (n));
}

void armv6m_core_c_function(armv6m_core_callback_t *callback, void *function)
{
    callback->routine = function;
    callback->context = NULL;
}

/* The format of a member pointer is defined in the C++ ABI.
 * For ARM that document is IHI0041D, which describes the differences
 * to the Itanium C++ ABI.
 */

void armv6m_core_cxx_method(armv6m_core_callback_t *callback, const void *method, void *object)
{
    void *ptr = (void*)(((const uint32_t*)method)[0]);
    ptrdiff_t adj = ((ptrdiff_t)(((const uint32_t*)method)[1]) >> 1);

    if (!((((const uint32_t*)method)[1]) & 1))
    {
        /* non-virtual function */
        callback->routine = ptr;
    }
    else
    {
        /* virtual function */
        void *vptr = *((void**)((uintptr_t)object + adj)); 
        callback->routine = *((void**)((uint8_t*)vptr + (ptrdiff_t)ptr));
    }

    callback->context = (void*)((uintptr_t)object + adj);
}

static void __empty() { }

void __armv6m_systick_initialize(void) __attribute__ ((weak, alias("__empty")));
void __armv6m_work_initialize(void) __attribute__ ((weak, alias("__empty")));
