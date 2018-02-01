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

extern void PendSV_Handler(void);

#define ARMV6M_PENDSV_ENTRY_COUNT 32

typedef struct _armv6m_pendsv_entry_t {
    armv6m_pendsv_routine_t          routine;
    void                             *context;
    uint32_t                         data;
} armv6m_pendsv_entry_t;

typedef struct _armv6m_pendsv_control_t {
    armv6m_pendsv_callback_t         notify_callback;
    volatile armv6m_pendsv_entry_t   *queue_read;
    volatile armv6m_pendsv_entry_t   *queue_write;
    armv6m_pendsv_entry_t            queue_data[ARMV6M_PENDSV_ENTRY_COUNT];
} armv6m_pendsv_control_t;

static armv6m_pendsv_control_t armv6m_pendsv_control;

void armv6m_pendsv_initialize(void)
{
    armv6m_pendsv_control.queue_read = &armv6m_pendsv_control.queue_data[0];
    armv6m_pendsv_control.queue_write = &armv6m_pendsv_control.queue_data[0];

    NVIC_SetPriority(PendSV_IRQn, ((1 << __NVIC_PRIO_BITS) -1));
}

void armv6m_pendsv_notify(armv6m_pendsv_callback_t callback)
{
    armv6m_pendsv_control.notify_callback = callback;

    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

bool armv6m_pendsv_enqueue(armv6m_pendsv_routine_t routine, void *context, uint32_t data)
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
    while (armv6m_atomic_compare_and_swap((volatile uint32_t*)&armv6m_pendsv_control.queue_write, (uint32_t)queue_write, (uint32_t)queue_write_next) != (uint32_t)queue_write);

    queue_write->routine = routine;
    queue_write->context = context;
    queue_write->data = data;

    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;

    return true;
}

static __attribute__((used)) void armv6m_pendsv_dequeue(volatile armv6m_pendsv_entry_t *queue_read)
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
        "  ldr     r2, =armv6m_pendsv_control          \n"
        "  ldr     r0, [r2, %[offset_QUEUE_READ]]      \n"
        "  ldr     r1, [r2, %[offset_QUEUE_WRITE]]     \n"
        "  cmp     r0, r1                              \n"
        "  beq     .Lnotify                            \n"
        "  push    { r2, lr }                          \n"
        "  bl      armv6m_pendsv_dequeue               \n"
        "  pop     { r2, r3 }                          \n"
        "  mov     lr, r3                              \n"
        ".Lnotify:                                     \n"
        "  ldr     r0, [r2, %[offset_NOTIFY_CALLBACK]] \n"
        "  cmp     r0, #0                              \n"
        "  bne     .Lcall                              \n"
        "  bx      lr                                  \n"
        ".Lcall:                                       \n"
        "  mov     r1, #0                              \n"
        "  str     r1, [r2, %[offset_NOTIFY_CALLBACK]] \n"
        "  bx      r0                                  \n"
        :
        : [offset_NOTIFY_CALLBACK] "I" (offsetof(armv6m_pendsv_control_t, notify_callback)),
          [offset_QUEUE_READ]      "I" (offsetof(armv6m_pendsv_control_t, queue_read)),
          [offset_QUEUE_WRITE]     "I" (offsetof(armv6m_pendsv_control_t, queue_write))
        );
}
