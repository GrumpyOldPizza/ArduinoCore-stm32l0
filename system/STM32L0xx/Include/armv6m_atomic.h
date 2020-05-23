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

#if !defined(_ARMV6M_ATOMIC_H)
#define _ARMV6M_ATOMIC_H

#ifdef __cplusplus
extern "C" {
#endif

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_add(volatile uint32_t *p_data, uint32_t data)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldr     %0, [%2]     \n"
        "   add     %1, %0, %1    \n"
        "   str     %1, [%2]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (data)
        : "l" (p_data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_sub(volatile uint32_t *p_data, uint32_t data)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldr     %0, [%2]     \n"
        "   sub     %1, %0, %1   \n"
        "   str     %1, [%2]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (data)
        : "l" (p_data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_and(volatile uint32_t *p_data, uint32_t data)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldr     %0, [%2]     \n"
        "   and     %1, %0       \n"
        "   str     %1, [%2]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (data)
        : "l" (p_data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_or(volatile uint32_t *p_data, uint32_t data)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldr     %0, [%2]     \n"
        "   orr     %1, %0       \n"
        "   str     %1, [%2]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (data)
        : "l" (p_data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_xor(volatile uint32_t *p_data, uint32_t data)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldr     %0, [%2]     \n"
        "   eor     %1, %0       \n"
        "   str     %1, [%2]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (data)
        : "l" (p_data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_inc(volatile uint32_t *p_data, uint32_t data_limit)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldr     %0, [%2]     \n"
        "   cmp     %0, %1       \n"
        "   beq     1f           \n"
        "   add     %1, %0, #1   \n"
        "   str     %1, [%2]     \n"
        "1: msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (data_limit)
        : "l" (p_data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_dec(volatile uint32_t *p_data)
{
    uint32_t data_return, data_scratch;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldr     %0, [%2]     \n"
        "   cmp     %0, #0       \n"
        "   beq     1f           \n"
        "   sub     %1, %0, #1   \n"
        "   str     %1, [%2]     \n"
        "1: msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "=&l" (data_scratch)
        : "l" (p_data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_swap(volatile uint32_t *p_data, uint32_t data)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldr     %0, [%1]     \n"
        "   str     %2, [%1]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return)
        : "l" (p_data), "l" (data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_cas(volatile uint32_t *p_data, uint32_t data_expected, uint32_t data)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldr     %0, [%1]     \n"
        "   cmp     %0, %2       \n"
        "   bne     1f           \n"
        "   str     %3, [%1]     \n"
        "1: msr     PRIMASK, r12 \n"
        : "=&l" (data_return)
        : "l" (p_data), "l" (data_expected), "l" (data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_addh(volatile uint16_t *p_data, uint32_t data)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldrh    %0, [%2]     \n"
        "   add     %1, %0, %1   \n"
        "   strh    %1, [%2]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (data)
        : "l" (p_data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_subh(volatile uint16_t *p_data, uint32_t data)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldrh    %0, [%2]     \n"
        "   sub     %1, %0, %1   \n"
        "   strh    %1, [%2]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (data)
        : "l" (p_data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_andh(volatile uint16_t *p_data, uint32_t data)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldrh    %0, [%2]     \n"
        "   and     %1, %0       \n"
        "   strh    %1, [%2]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (data)
        : "l" (p_data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_orh(volatile uint16_t *p_data, uint32_t data)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldrh    %0, [%2]     \n"
        "   orr     %1, %0       \n"
        "   strh    %1, [%2]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (data)
        : "l" (p_data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_xorh(volatile uint16_t *p_data, uint32_t data)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldrh    %0, [%2]     \n"
        "   eor     %1, %0       \n"
        "   strh    %1, [%2]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (data)
        : "l" (p_data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_inch(volatile uint16_t *p_data, uint32_t data_limit)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldrh    %0, [%2]     \n"
        "   cmp     %0, %1       \n"
        "   beq     1f           \n"
        "   add     %1, %0, #1   \n"
        "   strh    %1, [%2]     \n"
        "1: msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (data_limit)
        : "l" (p_data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_dech(volatile uint16_t *p_data)
{
    uint32_t data_return, data_scratch;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldrh    %0, [%2]     \n"
        "   cmp     %0, #0       \n"
        "   beq     1f           \n"
        "   sub     %1, %0, #1   \n"
        "   strh    %1, [%2]     \n"
        "1: msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "=&l" (data_scratch)
        : "l" (p_data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_swaph(volatile uint16_t *p_data, uint32_t data)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldrh    %0, [%1]     \n"
        "   strh    %2, [%1]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return)
        : "l" (p_data), "l" (data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_cash(volatile uint16_t *p_data, uint32_t data_expected, uint32_t data)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldrh    %0, [%1]     \n"
        "   cmp     %0, %2       \n"
        "   bne     1f           \n"
        "   strh    %3, [%1]     \n"
        "1: msr     PRIMASK, r12 \n"
        : "=&l" (data_return)
        : "l" (p_data), "l" (data_expected), "l" (data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_addb(volatile uint8_t *p_data, uint32_t data)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldrb    %0, [%2]     \n"
        "   add     %1, %0, %1   \n"
        "   strb    %1, [%2]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (data)
        : "l" (p_data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_subb(volatile uint8_t *p_data, uint32_t data)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldrb    %0, [%2]     \n"
        "   sub     %1, %0, %1   \n"
        "   strb    %1, [%2]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (data)
        : "l" (p_data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_andb(volatile uint8_t *p_data, uint32_t data)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldrb    %0, [%2]     \n"
        "   and     %1, %0       \n"
        "   strb    %1, [%2]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (data)
        : "l" (p_data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_orb(volatile uint8_t *p_data, uint32_t data)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldrb    %0, [%2]     \n"
        "   orr     %1, %0       \n"
        "   strb    %1, [%2]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (data)
        : "l" (p_data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_xorb(volatile uint8_t *p_data, uint32_t data)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldrb    %0, [%2]     \n"
        "   eor     %1, %0       \n"
        "   strb    %1, [%2]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (data)
        : "l" (p_data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_incb(volatile uint8_t *p_data)
{
    uint32_t data_return, data_scratch;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldrb    %0, [%2]     \n"
        "   cmp     %0, #255     \n"
        "   beq     1f           \n"
        "   add     %1, %0, #1   \n"
        "   strb    %1, [%2]     \n"
        "1: msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "=&l" (data_scratch)
        : "l" (p_data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_decb(volatile uint8_t *p_data)
{
    uint32_t data_return, data_scratch;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldrb    %0, [%2]     \n"
        "   cmp     %0, #0       \n"
        "   beq     1f           \n"
        "   sub     %1, %0, #1   \n"
        "   strb    %1, [%2]     \n"
        "1: msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "=&l" (data_scratch)
        : "l" (p_data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_swapb(volatile uint8_t *p_data, uint32_t data)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldrb    %0, [%1]     \n"
        "   strb    %2, [%1]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return)
        : "l" (p_data), "l" (data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_casb(volatile uint8_t *p_data, uint32_t data_expected, uint32_t data)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldrb    %0, [%1]     \n"
        "   cmp     %0, %2       \n"
        "   bne     1f           \n"
        "   strb    %3, [%1]     \n"
        "1: msr     PRIMASK, r12 \n"
        : "=&l" (data_return)
        : "l" (p_data), "l" (data_expected), "l" (data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_modify(volatile uint32_t *p_data, uint32_t mask, uint32_t data)
{
    uint32_t data_return;

    mask = ~mask;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldr     %0, [%2]     \n"
        "   and     %1, %0       \n"
        "   eor     %1, %3       \n"
        "   str     %1, [%2]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (mask)
        : "l" (p_data), "l" (data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_modifyz(volatile uint32_t *p_data, uint32_t mask, uint32_t data, volatile uint32_t *p_zero, uint32_t bits)
{
    uint32_t data_return;

    mask = ~mask;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldr     %0, [%3]     \n"
        "   ldr     %2, [%2]     \n"
        "   and     %2, %5       \n"
        "   beq     1f           \n"
        "   mov     %1, %0       \n"
        "   b       2f           \n"
        "1: and     %1, %0       \n"
        "   eor     %1, %4       \n"
        "2: str     %1, [%3]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (mask), "+&l" (p_zero)
        : "l" (p_data), "l" (data), "l" (bits)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_modifyzb(volatile uint32_t *p_data, uint32_t mask, uint32_t data, volatile uint8_t *p_zero)
{
    uint32_t data_return;

    mask = ~mask;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldr     %0, [%3]     \n"
        "   ldrb    %2, [%2]     \n"
        "   cmp     %2, #0       \n"
        "   beq     1f           \n"
        "   mov     %1, %0       \n"
        "   b       2f           \n"
        "1: and     %1, %0       \n"
        "   eor     %1, %4       \n"
        "2: str     %1, [%3]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (mask), "+&l" (p_zero)
        : "l" (p_data), "l" (data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_andz(volatile uint32_t *p_data, uint32_t data, volatile uint32_t *p_zero, uint32_t bits)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldr     %0, [%3]     \n"
        "   ldr     %2, [%2]     \n"
        "   and     %2, %4       \n"
        "   beq     1f           \n"
        "   mov     %1, %0       \n"
        "   b       2f           \n"
        "1: and     %1, %0       \n"
        "2: str     %1, [%3]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (data), "+&l" (p_zero)
        : "l" (p_data), "l" (bits)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_andzb(volatile uint32_t *p_data, uint32_t data, volatile uint8_t *p_zero)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldr     %0, [%3]     \n"
        "   ldrb    %2, [%2]     \n"
        "   cmp     %2, #0       \n"
        "   beq     1f           \n"
        "   mov     %1, %0       \n"
        "   b       2f           \n"
        "1: and     %1, %0       \n"
        "2: str     %1, [%3]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (data), "+&l" (p_zero)
        : "l" (p_data)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_orz(volatile uint32_t *p_data, uint32_t data, volatile uint32_t *p_zero, uint32_t bits)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldr     %0, [%3]     \n"
        "   ldr     %2, [%2]     \n"
        "   and     %2, %4       \n"
        "   beq     1f           \n"
        "   mov     %1, %0       \n"
        "   b       2f           \n"
        "1: orr     %1, %0       \n"
        "2: str     %1, [%3]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (data), "+&l" (p_zero)
        : "l" (p_data), "l" (bits)
        : "r12", "memory"
        );

    return data_return;
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t __armv6m_atomic_orzb(volatile uint32_t *p_data, uint32_t data, volatile uint8_t *p_zero)
{
    uint32_t data_return;

    __asm__ volatile (
        "   mrs     r12, PRIMASK \n"
        "   cpsid   i            \n"
        "   ldr     %0, [%3]     \n"
        "   ldrb    %2, [%2]     \n"
        "   cmp     %2, #0       \n"
        "   beq     1f           \n"
        "   mov     %1, %0       \n"
        "   b       2f           \n"
        "1: orr     %1, %0       \n"
        "2: str     %1, [%3]     \n"
        "   msr     PRIMASK, r12 \n"
        : "=&l" (data_return), "+&l" (data), "+&l" (p_zero)
        : "l" (p_data)
        : "r12", "memory"
        );

    return data_return;
}

extern uint32_t armv6m_atomic_add(volatile uint32_t *p_data, uint32_t data);
extern uint32_t armv6m_atomic_sub(volatile uint32_t *p_data, uint32_t data);
extern uint32_t armv6m_atomic_and(volatile uint32_t *p_data, uint32_t data);
extern uint32_t armv6m_atomic_or(volatile uint32_t *p_data, uint32_t data);
extern uint32_t armv6m_atomic_xor(volatile uint32_t *p_data, uint32_t data);
extern uint32_t armv6m_atomic_inc(volatile uint32_t *p_data, uint32_t data_limit);
extern uint32_t armv6m_atomic_dec(volatile uint32_t *p_data);
extern uint32_t armv6m_atomic_swap(volatile uint32_t *p_data, uint32_t data);
extern uint32_t armv6m_atomic_cas(volatile uint32_t *p_data, uint32_t data_expected, uint32_t data);

extern uint32_t armv6m_atomic_andh(volatile uint16_t *p_data, uint32_t data);
extern uint32_t armv6m_atomic_orh(volatile uint16_t *p_data, uint32_t data);
extern uint32_t armv6m_atomic_swaph(volatile uint16_t *p_data, uint32_t data);
extern uint32_t armv6m_atomic_cash(volatile uint16_t *p_data, uint32_t data_expected, uint32_t data);
extern uint32_t armv6m_atomic_andb(volatile uint8_t *p_data, uint32_t data);
extern uint32_t armv6m_atomic_orb(volatile uint8_t *p_data, uint32_t data);
extern uint32_t armv6m_atomic_decb(volatile uint8_t *p_data);
extern uint32_t armv6m_atomic_incb(volatile uint8_t *p_data);
extern uint32_t armv6m_atomic_swapb(volatile uint8_t *p_data, uint32_t data);
extern uint32_t armv6m_atomic_casb(volatile uint8_t *p_data, uint32_t data_expected, uint32_t data);

/* *p_data = (*p_data & ~mask) ^ data */
extern uint32_t armv6m_atomic_modify(volatile uint32_t *p_data, uint32_t mask, uint32_t data);

/* *p_data = (*p_zero == 0) ? ((*p_data & ~mask) ^ data) : *p_data */
extern uint32_t armv6m_atomic_modifyz(volatile uint32_t *p_data, uint32_t mask, uint32_t data, volatile uint32_t *p_zero, uint32_t bits);
extern uint32_t armv6m_atomic_modifyzb(volatile uint32_t *p_data, uint32_t mask, uint32_t data, volatile uint8_t *p_zero);

/* *p_data = (*p_zero == 0) ? (*p_data & mask) : *p_data */
extern uint32_t armv6m_atomic_andz(volatile uint32_t *p_data, uint32_t data, volatile uint32_t *p_zero, uint32_t bits);
extern uint32_t armv6m_atomic_andzb(volatile uint32_t *p_data, uint32_t data, volatile uint8_t *p_zero);

/* *p_data = (*p_zero == 0) ? (*p_data | mask) : *p_data */
extern uint32_t armv6m_atomic_orz(volatile uint32_t *p_data, uint32_t data, volatile uint32_t *p_zero, uint32_t bits);
extern uint32_t armv6m_atomic_orzb(volatile uint32_t *p_data, uint32_t data, volatile uint8_t *p_zero);


#ifdef __cplusplus
}
#endif

#endif /* _ARMV6M_ATOMIC_H */
