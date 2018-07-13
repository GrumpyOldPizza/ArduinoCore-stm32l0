/*
 * Copyright (c) 2014-2018 Thomas Roell.  All rights reserved.
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

extern void SVC_Handler(void);

void armv6m_svcall_initialize(void)
{
    NVIC_SetPriority(SVC_IRQn, ((1 << __NVIC_PRIO_BITS) -1));
}

void __attribute__((naked)) SVC_Handler(void)
{
    __asm__(
        "  mov     r2, sp                              \n"
        "  push    { r2, lr }                          \n"
        "  ldmia   r2, { r0, r1, r2, r3 }              \n"
        "  blx     r7                                  \n"
        "  pop     { r2, r3 }                          \n"
        "  mov     lr, r3                              \n"
        "  str     r0, [r2, #0]                        \n"
        "  bx      lr                                  \n"
        :
        :
        );
}
