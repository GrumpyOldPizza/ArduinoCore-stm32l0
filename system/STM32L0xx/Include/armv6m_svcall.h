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

#if !defined(_ARMV6M_SVCALL_H)
#define _ARMV6M_SVCALL_H

#ifdef __cplusplus
extern "C" {
#endif

static inline uint32_t armv6m_svcall_0(uint32_t routine)
{
    register uint32_t _a0 __asm__("r0");
    register uint32_t _routine __asm__("r7") = routine;

    __asm__ volatile ("svc 0"
                      : "=r" (_a0)
                      : "r" (_routine)
                      : "memory");

    return _a0;
}

static inline uint32_t armv6m_svcall_1(uint32_t routine, uint32_t a0)
{
    register uint32_t _a0 __asm__("r0") = a0;
    register uint32_t _routine __asm__("r7") = routine;

    __asm__ volatile ("svc 0"
                      : "+r" (_a0)
                      : "r" (_routine)
                      : "memory");

    return _a0;
}

static inline uint32_t armv6m_svcall_2(uint32_t routine, uint32_t a0, uint32_t a1)
{
    register uint32_t _a0 __asm__("r0") = a0;
    register uint32_t _a1 __asm__("r1") = a1;
    register uint32_t _routine __asm__("r7") = routine;

    __asm__ volatile ("svc 0"
                      : "+r" (_a0)
                      : "r" (_routine), "r" (_a1)
                      : "r2", "r3", "memory");

    return _a0;
}

static inline uint32_t armv6m_svcall_3(uint32_t routine, uint32_t a0, uint32_t a1, uint32_t a2)
{
    register uint32_t _a0 __asm__("r0") = a0;
    register uint32_t _a1 __asm__("r1") = a1;
    register uint32_t _a2 __asm__("r2") = a2;
    register uint32_t _routine __asm__("r7") = routine;

    __asm__ volatile ("svc 0"
                      : "+r" (_a0)
                      : "r" (_routine), "r" (_a1), "r" (_a2)
                      : "r3", "memory");

    return _a0;
}

static inline uint32_t armv6m_svcall_4(uint32_t routine, uint32_t a0, uint32_t a1, uint32_t a2, uint32_t a3)
{
    register uint32_t _a0 __asm__("r0") = a0;
    register uint32_t _a1 __asm__("r1") = a1;
    register uint32_t _a2 __asm__("r2") = a2;
    register uint32_t _a3 __asm__("r3") = a3;
    register uint32_t _routine __asm__("r7") = routine;

    __asm__ volatile ("svc 0"
                      : "+r" (_a0)
                      : "r" (_routine), "r" (_a1), "r" (_a2), "r" (_a3)
                      : "memory");

    return _a0;
}

extern void armv6m_svcall_initialize(void);

#ifdef __cplusplus
}
#endif

#endif /* _ARMV6M_SVCALL_H */
