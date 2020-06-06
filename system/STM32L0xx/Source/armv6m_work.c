/*
 * Copyright (c) 2019-2020 Thomas Roell.  All rights reserved.
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

typedef struct _armv6m_work_control_t {
    volatile armv6m_core_callback_t callback;
    armv6m_work_t                   *head;
    armv6m_work_t                   *tail;
    armv6m_work_t * volatile        submit;
    volatile uint32_t               lock;
} armv6m_work_control_t;

static armv6m_work_control_t armv6m_work_control;

#define ARMV6M_WORK_TAIL   ((armv6m_work_t*)1)

static __attribute__((naked, used)) void armv6m_work_execute(void)
{
    __asm__(
        "   push    { r7, lr }                           \n"
        "   .cfi_def_cfa_offset 8                        \n"
        "   .cfi_offset 7, -8                            \n"
        "   .cfi_offset 14, -4                           \n"
        "   ldr     r3, =armv6m_work_control             \n"
        "   sub     sp, #0x20                            \n"
        "   ldr     r0, [r3, %[offset_CONTROL_CONTEXT]]  \n"
        "   adr     r1, 1f                               \n"
        "   add     r1, r1, #1                           \n"
        "   ldr     r2, [r3, %[offset_CONTROL_ROUTINE]]  \n"
        "   sub     r2, r2, #1                           \n"
        "   ldr     r3, =0xf1000000                      \n"
        "   str     r0, [sp, #0x00]                      \n"
        "   str     r1, [sp, #0x14]                      \n"
        "   str     r2, [sp, #0x18]                      \n"
        "   str     r3, [sp, #0x1c]                      \n"
        "   ldr     r0, =0xfffffff9                      \n"
        "   bx      r0                                   \n"
        "   .align 2                                     \n"
        "1: adr     r7, 2f                               \n"
        "   add     r7, r7, #1                           \n"
        "   svc     0                                    \n"
        "   .align 2                                     \n"
        "2: ldr     r3, =armv6m_work_control             \n"
        "   mov     r0, #0                               \n"
        "   str     r0, [r3, %[offset_CONTROL_ROUTINE]]  \n"
        "   bl      armv6m_work_dispatch                 \n"
        "   add     sp, #0x28                            \n"
        "   pop     { r7, pc }                           \n"
        :
        : [offset_CONTROL_ROUTINE]  "I" (offsetof(armv6m_work_control_t, callback.routine)),
          [offset_CONTROL_CONTEXT]  "I" (offsetof(armv6m_work_control_t, callback.context))
        );
}

static __attribute__((optimize("O3"), used)) void armv6m_work_dispatch(void)
{
    armv6m_work_t *work;

    if (armv6m_work_control.lock == 0)
    {
        work = armv6m_work_control.head;

        if (work != NULL)
        {
            if (armv6m_work_control.head == armv6m_work_control.tail)
            {
                armv6m_work_control.head = NULL;
                armv6m_work_control.tail = NULL;
            }
            else
            {
                armv6m_work_control.head = work->next;
            }

            armv6m_work_control.callback = work->callback;

            work->next = NULL;

            armv6m_pendsv_hook(armv6m_work_execute);
        }
    }
}

static __attribute__((optimize("O3"))) void armv6m_work_schedule(void)
{
    armv6m_work_t *work, *work_next, *work_head, *work_tail;
    
    work = (armv6m_work_t*)__armv6m_atomic_swap((volatile uint32_t*)&armv6m_work_control.submit, (uint32_t)ARMV6M_WORK_TAIL);

    if (work != ARMV6M_WORK_TAIL)
    {
        for (work_head = ARMV6M_WORK_TAIL, work_tail = work; work != ARMV6M_WORK_TAIL; work = work_next)
        {
            work_next = work->next;
            
            work->next = work_head;
            
            work_head = work;
        }

        if (armv6m_work_control.head == NULL)
        {
            armv6m_work_control.head = work_head;
        }
        else
        {
            armv6m_work_control.tail->next = work_head;
        }

        armv6m_work_control.tail = work_tail;

        if (armv6m_work_control.callback.routine == NULL)
        {
            armv6m_work_dispatch();
        }
    }
}

void __armv6m_work_initialize()
{
    armv6m_work_control.head = NULL;
    armv6m_work_control.tail = NULL;
    armv6m_work_control.submit = ARMV6M_WORK_TAIL;
    armv6m_work_control.lock = 0;
}

void armv6m_work_create(armv6m_work_t *work, armv6m_core_routine_t routine, void *context)
{
    work->next = NULL;
    work->callback.routine = routine;
    work->callback.context = context;
}

bool armv6m_work_destroy(armv6m_work_t *work)
{
    return (work->next == NULL);
}

__attribute__((optimize("O3"))) bool armv6m_work_submit(armv6m_work_t *work)
{
    armv6m_work_t *work_next;

    if (__armv6m_atomic_cas((volatile uint32_t*)&work->next, (uint32_t)NULL, (uint32_t)ARMV6M_WORK_TAIL) == (uint32_t)NULL)
    {
        work_next = (armv6m_work_t*)armv6m_atomic_swap((volatile uint32_t*)&armv6m_work_control.submit, (uint32_t)work);

        work->next = work_next;

        if (armv6m_work_control.lock == 0)
        {
            armv6m_pendsv_raise(ARMV6M_PENDSV_SWI_WORK_SCHEDULE);
        }
        
        return true;
    }

    return false;
}

__attribute__((optimize("O3"))) void armv6m_work_block()
{
    __armv6m_atomic_inc(&armv6m_work_control.lock, ~0);
}

__attribute__((optimize("O3"))) void armv6m_work_unblock()
{
    if (__armv6m_atomic_dec(&armv6m_work_control.lock) == 1)
    {
        armv6m_pendsv_raise(ARMV6M_PENDSV_SWI_WORK_SCHEDULE);
    }
}

void SWI_WORK_SCHEDULE_IRQHandler(void)
{
    armv6m_work_schedule();
}
