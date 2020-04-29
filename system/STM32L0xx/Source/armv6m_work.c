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
    void                          *stack;
    armv6m_work_callback_t        callback;
    void                          *context;
    armv6m_work_t                 *self;
    armv6m_work_t                 *head;
    armv6m_work_t                 *tail;
    armv6m_work_t * volatile      submit;
} armv6m_work_control_t;

static armv6m_work_control_t armv6m_work_control;

#define ARMV6M_WORK_TAIL   ((armv6m_work_t*)1)

static __attribute__((naked, used)) void armv6m_work_notify(void)
{
    __asm__(
        "  ldr     r3, =armv6m_work_control             \n"
        "  mov     r0, sp                               \n"
        "  str     r0, [r3, %[offset_CONTROL_STACK]]    \n"
        "  sub     sp, #0x20                            \n"
        "  ldr     r0, [r3, %[offset_CONTROL_CONTEXT]]  \n"
        "  ldr     r1, =armv6m_work_return              \n"
        "  ldr     r2, [r3, %[offset_CONTROL_CALLBACK]] \n"
        "  sub     r2, r2, #1                           \n"
        "  ldr     r3, =0xf1000000                      \n"
        "  str     r0, [sp, #0x00]                      \n"
        "  str     r1, [sp, #0x14]                      \n"
        "  str     r2, [sp, #0x18]                      \n"
        "  str     r3, [sp, #0x1c]                      \n"
        "  ldr     r0, =0xfffffff9                      \n"
        "  mov     lr, r0                               \n"
        "  bx      lr                                   \n"
        "  bkpt                                         \n"
        :
        : [offset_CONTROL_STACK]    "I" (offsetof(armv6m_work_control_t, stack)),
          [offset_CONTROL_CALLBACK] "I" (offsetof(armv6m_work_control_t, callback)),
          [offset_CONTROL_CONTEXT]  "I" (offsetof(armv6m_work_control_t, context))
        );
}

static __attribute__((naked, used)) void armv6m_work_svcall(void)
{
    __asm__(
        "  mov     r7, r0                               \n" // restore saved r7
        "  ldr     r3, =armv6m_work_control             \n"
        "  ldr     r0, [r3, %[offset_CONTROL_STACK]]    \n"
	"  mov     sp, r0                               \n"
	"  mov     r0, #0                               \n"
        "  str     r0, [r3, %[offset_CONTROL_SELF]]     \n"
        "  bl      armv6m_work_dispatch                 \n"
        "  ldr     r0, =0xfffffff9                      \n"
        "  mov     lr, r0                               \n"
        "  bx      lr                                   \n"
        "  bkpt                                         \n"
        :
        : [offset_CONTROL_STACK]    "I" (offsetof(armv6m_work_control_t, stack)),
          [offset_CONTROL_SELF]     "I" (offsetof(armv6m_work_control_t, self))
        );
}

__attribute__((naked)) void armv6m_work_return(void)
{
    __asm__(
        "  mov     r0, r7                              \n" // save r7
        "  ldr     r7, =armv6m_work_svcall             \n"
        "  svc     0                                   \n"
        "  bkpt                                        \n"
        :
        :
        );
}

static  __attribute__((used)) void armv6m_work_dispatch(void)
{
    armv6m_work_t *work;

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
	armv6m_work_control.context = work->context;
	armv6m_work_control.self = work;

	work->next = NULL;

	armv6m_pendsv_notify(armv6m_work_notify);
    }
}

static void armv6m_work_schedule(void)
{
    armv6m_work_t *work, *work_next, *work_head, *work_tail;
    
    work = (armv6m_work_t*)armv6m_atomic_swap((volatile uint32_t*)&armv6m_work_control.submit, (uint32_t)ARMV6M_WORK_TAIL);

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

	if (armv6m_work_control.self == NULL)
	{
	    armv6m_work_dispatch();
	}
    }
}

void armv6m_work_initialize()
{
    armv6m_work_control.self = NULL;
    armv6m_work_control.head = NULL;
    armv6m_work_control.tail = NULL;
    armv6m_work_control.submit = ARMV6M_WORK_TAIL;
}

void armv6m_work_create(armv6m_work_t *work, armv6m_work_callback_t callback, void *context)
{
    work->next = NULL;
    work->callback = callback;
    work->context = context;
}

bool armv6m_work_destroy(armv6m_work_t *work)
{
    return (work->next == NULL);
}

bool armv6m_work_submit(armv6m_work_t *work)
{
    armv6m_work_t *work_next;

    if (armv6m_atomic_cas((volatile uint32_t*)&work->next, (uint32_t)NULL, (uint32_t)ARMV6M_WORK_TAIL) == (uint32_t)NULL)
    {
	work_next = (armv6m_work_t*)armv6m_atomic_swap((volatile uint32_t*)&armv6m_work_control.submit, (uint32_t)work);

	work->next = work_next;

	armv6m_pendsv_raise(ARMV6M_PENDSV_SWI_WORK_SCHEDULE);

	return true;
    }

    return false;
}

void SWI_WORK_SCHEDULE_IRQHandler(void)
{
    armv6m_work_schedule();
}
