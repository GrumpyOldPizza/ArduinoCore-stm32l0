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

        .arch armv6-m
        .syntax unified

        .section .text.armv6m_atomic_add, "ax"  
        .thumb
        .thumb_func
        .align  2
        .globl  armv6m_atomic_add
        .type   armv6m_atomic_add, %function

        /* uint32_t armv6m_atomic_add(volatile uint32_t *p_data, uint32_t data)
         */

armv6m_atomic_add:
        mrs     r12, PRIMASK
        cpsid   i
        ldr     r2, [r0]
        adds    r1, r1, r2
        str     r1, [r0]
        msr     PRIMASK, r12
        mov     r0, r2
        bx      lr
        

        .section .text.armv6m_atomic_sub, "ax"  
        .thumb
        .thumb_func
        .align  2
        .globl  armv6m_atomic_sub
        .type   armv6m_atomic_sub, %function


        /* uint32_t armv6m_atomic_sub(volatile uint32_t *p_data, uint32_t data)
         */

armv6m_atomic_sub:
        mrs     r12, PRIMASK
        cpsid   i
        ldr     r2, [r0]
        subs    r1, r2, r1
        str     r1, [r0]
        msr     PRIMASK, r12
        mov     r0, r2
        bx      lr
        

        .section .text.armv6m_atomic_and, "ax"  
        .thumb
        .thumb_func
        .align  2
        .globl  armv6m_atomic_and
        .type   armv6m_atomic_and, %function

        /* uint32_t armv6m_atomic_and(volatile uint32_t *p_data, uint32_t data)
         */

armv6m_atomic_and:
        mrs     r12, PRIMASK
        cpsid   i
        ldr     r2, [r0]
        ands    r1, r2
        str     r1, [r0]
        msr     PRIMASK, r12
        mov     r0, r2
        bx      lr
        

        .section .text.armv6m_atomic_or, "ax"   
        .thumb
        .thumb_func
        .align  2
        .globl  armv6m_atomic_or
        .type   armv6m_atomic_or, %function

        /* uint32_t armv6m_atomic_or(volatile uint32_t *p_data, uint32_t data)
         */

armv6m_atomic_or:
        mrs     r12, PRIMASK
        cpsid   i
        ldr     r2, [r0]
        orrs    r1, r2
        str     r1, [r0]
        msr     PRIMASK, r12
        mov     r0, r2
        bx      lr
        

        .section .text.armv6m_atomic_xor, "ax"  
        .thumb
        .thumb_func
        .align  2
        .globl  armv6m_atomic_xor
        .type   armv6m_atomic_xor, %function

        /* uint32_t armv6m_atomic_xor(volatile uint32_t *p_data, uint32_t data)
         */

armv6m_atomic_xor:
        mrs     r12, PRIMASK
        cpsid   i
        ldr     r2, [r0]
        eors    r1, r2
        str     r1, [r0]
        msr     PRIMASK, r12
        mov     r0, r2
        bx      lr


        .section .text.armv6m_atomic_swap, "ax" 
        .thumb
        .thumb_func
        .align  2
        .globl  armv6m_atomic_swap
        .type   armv6m_atomic_swap, %function

        /* uint32_t armv6m_atomic_swap(volatile uint32_t *p_data, uint32_t data)
         */

armv6m_atomic_swap:
        mrs     r12, PRIMASK
        cpsid   i
        ldr     r2, [r0]
        str     r1, [r0]
        msr     PRIMASK, r12
        mov     r0, r2
        bx      lr


        .section .text.armv6m_atomic_compare_and_swap, "ax"     
        .thumb
        .thumb_func
        .align  2
        .globl  armv6m_atomic_compare_and_swap
        .type   armv6m_atomic_compare_and_swap, %function

        /* uint32_t armv6m_atomic_compare_and_swap(volatile uint32_t *p_data, uint32_t data_expected, uint32_t data)
         */

armv6m_atomic_compare_and_swap:
        mrs     r12, PRIMASK
        cpsid   i
        ldr     r3, [r0]
        cmp     r1, r3
        bne.n   1f
        str     r2, [r0]
1:      msr     PRIMASK, r12
        mov     r0, r3
        bx      lr


        .section .text.armv6m_atomic_modify, "ax"       
        .thumb
        .thumb_func
        .align  2
        .globl  armv6m_atomic_modify
        .type   armv6m_atomic_modify, %function
        
        /* void armv6m_atomic_modify(volatile uint32_t *p_data, uint32_t mask, uint32_t data)
         */

armv6m_atomic_modify:
        mrs     r12, PRIMASK
        cpsid   i
        ldr     r3, [r0]
        bics    r3, r1
        eors    r3, r2
        str     r3, [r0]
        msr     PRIMASK, r12
        bx      lr
