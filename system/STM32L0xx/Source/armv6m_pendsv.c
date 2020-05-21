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

#define ARMV6M_PENDSV_ENTRY_COUNT 32

typedef struct _armv6m_pendsv_entry_t {
    armv6m_pendsv_routine_t          routine;
    void                             *context;
    uint32_t                         data;
} armv6m_pendsv_entry_t;

typedef struct _armv6m_pendsv_control_t {
    volatile uint32_t                swi_pending;
    volatile uint32_t                swi_mask;
    armv6m_pendsv_callback_t         hook_callback;
    volatile armv6m_pendsv_entry_t   *queue_read;
    volatile armv6m_pendsv_entry_t   *queue_write;
    armv6m_pendsv_entry_t            queue_data[ARMV6M_PENDSV_ENTRY_COUNT];
} armv6m_pendsv_control_t;

static armv6m_pendsv_control_t armv6m_pendsv_control;

static const armv6m_pendsv_callback_t armv6m_pendsv_swi_callback[] = {
    SWI0_IRQHandler,
    SWI1_IRQHandler,
    SWI2_IRQHandler,
    SWI3_IRQHandler,
    SWI4_IRQHandler,
    SWI5_IRQHandler,
    SWI6_IRQHandler,
    SWI7_IRQHandler,
    SWI8_IRQHandler,
    SWI9_IRQHandler,
    SWI10_IRQHandler,
    SWI11_IRQHandler,
    SWI12_IRQHandler,
    SWI13_IRQHandler,
    SWI14_IRQHandler,
    SWI15_IRQHandler,
    SWI16_IRQHandler,
    SWI17_IRQHandler,
    SWI18_IRQHandler,
    SWI19_IRQHandler,
    SWI20_IRQHandler,
    SWI21_IRQHandler,
    SWI22_IRQHandler,
    SWI23_IRQHandler,
    SWI24_IRQHandler,
    SWI25_IRQHandler,
    SWI26_IRQHandler,
    SWI27_IRQHandler,
    SWI28_IRQHandler,
    SWI29_IRQHandler,
    SWI30_IRQHandler,
    SWI31_IRQHandler,
};
    
void __armv6m_pendsv_initialize(void)
{
    armv6m_pendsv_control.swi_pending = 0;
    armv6m_pendsv_control.swi_mask = ~0ul;
    armv6m_pendsv_control.queue_read = &armv6m_pendsv_control.queue_data[0];
    armv6m_pendsv_control.queue_write = &armv6m_pendsv_control.queue_data[0];

    NVIC_SetPriority(PendSV_IRQn, ARMV6M_IRQ_PRIORITY_PENDSV);
}

__attribute__((optimize("O3"))) bool armv6m_pendsv_raise(uint32_t index)
{
    uint32_t mask = (1ul << index);
    
    if (!(__armv6m_atomic_or(&armv6m_pendsv_control.swi_pending, mask) & mask))
    {
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;

        return true;
    }

    return false;
}

__attribute__((optimize("O3"))) void armv6m_pendsv_block(uint32_t mask)
{
    __armv6m_atomic_and(&armv6m_pendsv_control.swi_mask, ~mask);
}

__attribute__((optimize("O3"))) void armv6m_pendsv_unblock(uint32_t mask)
{
    uint32_t o_mask;

    o_mask = __armv6m_atomic_or(&armv6m_pendsv_control.swi_mask, mask);

    if (armv6m_pendsv_control.swi_pending & (mask & ~o_mask))
    {
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }
}

__attribute__((optimize("O3"))) void armv6m_pendsv_hook(armv6m_pendsv_callback_t callback)
{
    armv6m_pendsv_control.hook_callback = callback;

    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

__attribute__((optimize("O3"))) bool armv6m_pendsv_enqueue(armv6m_pendsv_routine_t routine, void *context, uint32_t data)
{
    volatile armv6m_pendsv_entry_t *queue_write, *queue_write_next;

    do
    {
        queue_write = armv6m_pendsv_control.queue_write;
        queue_write_next = queue_write +1;
        
        if (queue_write_next == &armv6m_pendsv_control.queue_data[ARMV6M_PENDSV_ENTRY_COUNT])
        {
            queue_write_next = &armv6m_pendsv_control.queue_data[0];
        }
        
        if (queue_write_next == armv6m_pendsv_control.queue_read)
        {
            return false;
        }
    }
    while (__armv6m_atomic_cas((volatile uint32_t*)&armv6m_pendsv_control.queue_write, (uint32_t)queue_write, (uint32_t)queue_write_next) != (uint32_t)queue_write);

    queue_write->routine = routine;
    queue_write->context = context;
    queue_write->data = data;

    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;

    return true;
}

static __attribute__((optimize("O3"), used)) void armv6m_pendsv_swi_process(uint32_t mask)
{
    uint32_t index;

    __armv6m_atomic_and(&armv6m_pendsv_control.swi_pending, ~mask);

    while (mask) 
    {
        index = __builtin_ctz(mask);

        mask &= ~(1ul << index); 

        (*armv6m_pendsv_swi_callback[index])();
    }
}

static __attribute__((optimize("O3"), used)) void armv6m_pendsv_queue_process(volatile armv6m_pendsv_entry_t *queue_read)
{
    armv6m_pendsv_routine_t routine;
    void *context;
    uint32_t data;

    do
    {
        routine = queue_read->routine;
        context = queue_read->context;
        data = queue_read->data;

        queue_read = queue_read + 1;

        if (queue_read == &armv6m_pendsv_control.queue_data[ARMV6M_PENDSV_ENTRY_COUNT])
        {
            queue_read = &armv6m_pendsv_control.queue_data[0];
        }

        armv6m_pendsv_control.queue_read = queue_read;

        (*routine)(context, data);
    }
    while (queue_read != armv6m_pendsv_control.queue_write);
}

void __attribute__((naked)) PendSV_Handler(void)
{
    __asm__( 
        "   mov     r2, sp                               \n"
        "   push    { r2, lr }                           \n"
        "   .cfi_def_cfa_offset 8                        \n"
        "   .cfi_offset 2, -8                            \n"
        "   .cfi_offset 14, -4                           \n"
        "   ldr     r1, =armv6m_pendsv_control           \n"
        "   ldr     r0, [r1, %[offset_SWI_PENDING]]      \n"
        "   ldr     r2, [r1, %[offset_SWI_MASK]]         \n"
        "   and     r0, r2                               \n"
        "   beq     1f                                   \n"
        "   bl      armv6m_pendsv_swi_process            \n" // R0 is pending & mask
        "   ldr     r1, =armv6m_pendsv_control           \n"
        "1: ldr     r0, [r1, %[offset_QUEUE_READ]]       \n"
        "   ldr     r2, [r1, %[offset_QUEUE_WRITE]]      \n"
        "   cmp     r0, r2                               \n"
        "   beq     2f                                   \n"
        "   bl      armv6m_pendsv_queue_process          \n" // R0 is queue_read
        "   ldr     r1, =armv6m_pendsv_control           \n"
        "2: ldr     r0, [r1, %[offset_HOOK_CALLBACK]]    \n"
        "   pop     { r2, r3 }                           \n"
        "   mov     lr, r3                               \n"
        "   cmp     r0, #0                               \n"
        "   bne     3f                                   \n"
        "   bx      lr                                   \n"
        "3: mov     r2, #0                               \n"
        "   str     r2, [r1, %[offset_HOOK_CALLBACK]]    \n"
        "   bx      r0                                   \n"
        :
        : [offset_SWI_PENDING]     "I" (offsetof(armv6m_pendsv_control_t, swi_pending)),
          [offset_SWI_MASK]        "I" (offsetof(armv6m_pendsv_control_t, swi_mask)),
          [offset_QUEUE_READ]      "I" (offsetof(armv6m_pendsv_control_t, queue_read)),
          [offset_QUEUE_WRITE]     "I" (offsetof(armv6m_pendsv_control_t, queue_write)),
          [offset_HOOK_CALLBACK]   "I" (offsetof(armv6m_pendsv_control_t, hook_callback))
        );
}

static void __attribute__((naked)) Default_Handler(void)
{
    HardFault_Handler();
}

void SWI0_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI1_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI2_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI3_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI4_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI5_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI6_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI7_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI8_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI9_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI10_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI11_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI12_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI13_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI14_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI15_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI16_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI17_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI18_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI19_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI20_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI21_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI22_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI23_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI24_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI25_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI26_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI27_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI28_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI29_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI30_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void SWI31_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));

