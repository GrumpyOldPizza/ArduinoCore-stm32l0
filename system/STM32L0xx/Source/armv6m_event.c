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

typedef struct _armv6m_event_frame_t {
    struct _armv6m_event_frame_t   *previous;
    armv6m_event_t                 *event;
} armv6m_event_frame_t;

typedef struct _armv6m_event_control_t {
    armv6m_event_frame_t           *stack;
    armv6m_event_t                 *self;
    armv6m_event_t                 *next;
    armv6m_event_t                 *preemptive;
    armv6m_event_t                 *cooperative;
    volatile uint8_t               ceiling;
} armv6m_event_control_t;


static armv6m_event_control_t armv6m_event_control;


static __attribute__((naked, used)) void armv6m_event_notify(void)
{
    __asm__(
        "  ldr     r3, =armv6m_event_control           \n"
        "  ldr     r0, [r3, %[offset_CONTROL_NEXT]]    \n"
        "  cmp     r0, #0                              \n"
        "  beq     .Lreturn                            \n"
        "  ldr     r0, [r3, %[offset_CONTROL_STACK]]   \n"
        "  ldr     r1, [r3, %[offset_CONTROL_SELF]]    \n"
        "  push    { r0, r1 }                          \n"
        "  mov     r0, sp                              \n"
        "  str     r0, [r3, %[offset_CONTROL_STACK]]   \n"
        "  bl      armv6m_event_switch                 \n"
        "  ldr     r3, =armv6m_event_control           \n"
        "  ldr     r0, [r3, %[offset_CONTROL_SELF]]    \n"
        "  sub     sp, #0x20                           \n"
        "  ldr     r1, [r0, %[offset_EVENT_CONTEXT]]   \n"
        "  str     r1, [sp, #0x00]                     \n"
        "  ldr     r1, =armv6m_event_return            \n"
        "  ldr     r2, [r0, %[offset_EVENT_ROUTINE]]   \n"
        "  sub     r2, r2, #1                          \n"
        "  ldr     r3, =0xf1000000                     \n"
        "  str     r1, [sp, #0x14]                     \n"
        "  str     r2, [sp, #0x18]                     \n"
        "  str     r3, [sp, #0x1c]                     \n"
        "  ldr     r0, =0xfffffff9                     \n"
        "  mov     lr, r0                              \n"
        ".Lreturn:                                     \n"
        "  bx      lr                                  \n"
        "  bkpt                                        \n"
        :
        : [offset_CONTROL_STACK]    "I" (offsetof(armv6m_event_control_t, stack)),
          [offset_CONTROL_SELF]     "I" (offsetof(armv6m_event_control_t, self)),
          [offset_CONTROL_NEXT]     "I" (offsetof(armv6m_event_control_t, next)),
          [offset_EVENT_ROUTINE]    "I" (offsetof(armv6m_event_t, routine)),
          [offset_EVENT_CONTEXT]    "I" (offsetof(armv6m_event_t, context))
        );
}

static __attribute__((naked, used)) void armv6m_event_svcall(void)
{
    __asm__(
        "  add     sp, #0x28                           \n" // drop SVC stack frame
        "  pop     { r0, r1 }                          \n"
        "  ldr     r3, =armv6m_event_control           \n"
        "  str     r0, [r3, %[offset_CONTROL_STACK]]   \n"
        "  str     r1, [r3, %[offset_CONTROL_SELF]]    \n"
        "  bl      armv6m_event_schedule               \n"
        "  ldr     r0, =0xfffffff9                     \n"
        "  mov     lr, r0                              \n"
        "  bx      lr                                  \n"
        "  bkpt                                        \n"
        :
        : [offset_CONTROL_STACK]    "I" (offsetof(armv6m_event_control_t, stack)),
          [offset_CONTROL_SELF]     "I" (offsetof(armv6m_event_control_t, self))
        );
}

static __attribute__((naked, used)) void armv6m_event_return(void)
{
    __asm__(
        "  ldr     r0, =armv6m_event_svcall            \n"
        "  mov     r7, r0                              \n"
        "  svc     0                                   \n"
        "  bkpt                                        \n"
        :
        :
        );
}

static  __attribute__((used)) void armv6m_event_schedule(void)
{
    if (armv6m_event_control.preemptive &&
        (armv6m_event_control.preemptive->priority < armv6m_event_control.ceiling) &&
        (!armv6m_event_control.self || (armv6m_event_control.preemptive->priority < armv6m_event_control.self->priority)))
    {
        armv6m_event_control.next = armv6m_event_control.preemptive;

        armv6m_pendsv_notify(armv6m_event_notify);
    }
    else
    {
        armv6m_event_control.next = NULL;
    }
}

static  __attribute__((used)) void armv6m_event_insert(armv6m_event_t *event)
{
    armv6m_event_t *element;

    if (event->priority == ARMV6M_EVENT_PRIORITY_COOPERATIVE)
    {
        if (armv6m_event_control.cooperative == NULL)
        {
            event->next = event;
            event->previous = event;
            
            armv6m_event_control.cooperative = event;
        }
        else
        {
            element = armv6m_event_control.cooperative;

            event->previous = element->previous;
            event->next = element;

            event->next->previous = event;
            event->previous->next = event;
        }
    }
    else
    {
        if (armv6m_event_control.preemptive == NULL)
        {
            event->next = event;
            event->previous = event;
            
            armv6m_event_control.preemptive = event;
            
            armv6m_event_schedule();
        }
        else
        {
            element = armv6m_event_control.preemptive;
            
            do
            {
                if (element->priority > event->priority)
                {
                    if (armv6m_event_control.preemptive == element)
                    {
                        armv6m_event_control.preemptive = event;
                        
                        armv6m_event_schedule();
                    }
                    break;
                }
                
                element = element->next;
            }
            while (armv6m_event_control.preemptive != element);
            
            event->previous = element->previous;
            event->next = element;

            event->next->previous = event;
            event->previous->next = event;
        }
    }
}

static  __attribute__((used)) void armv6m_event_remove(armv6m_event_t *event)
{
    if (event->priority == ARMV6M_EVENT_PRIORITY_COOPERATIVE)
    {
        if (event->next == event)
        {
            armv6m_event_control.cooperative = NULL;
        }
        else
        {
            if (armv6m_event_control.cooperative == event)
            {
                armv6m_event_control.cooperative = event->next;
            }
            
            event->next->previous = event->previous;
            event->previous->next = event->next;
        }
    }
    else
    {
        if (event->next == event)
        {
            armv6m_event_control.preemptive = NULL;
        }
        else
        {
            if (armv6m_event_control.preemptive == event)
            {
                armv6m_event_control.preemptive = event->next;
            }
            
            event->next->previous = event->previous;
            event->previous->next = event->next;
        }
    }

    event->next = NULL;
    event->previous = NULL;
        
    if (armv6m_event_control.next == event)
    {
        armv6m_event_schedule();
    }
}

static  __attribute__((used)) void armv6m_event_switch(void)
{
    if (armv6m_event_control.self == armv6m_event_control.next)
    {
        __BKPT();
    }

    armv6m_event_control.self = armv6m_event_control.next;
    armv6m_event_control.next = NULL;

    armv6m_event_remove(armv6m_event_control.self);

    if (--armv6m_event_control.self->count != 0)
    {
        armv6m_event_insert(armv6m_event_control.self);
    }
}

static void armv6m_event_svc_destroy(armv6m_event_t *event)
{
    if (event->next)
    {
        armv6m_event_remove(event);
    }

    event->routine = NULL;
    event->context = NULL;
    event->priority = 0;
    event->lock = 0;
    event->count = 0;
}

static void armv6m_event_svc_enqueue(armv6m_event_t *event)
{
    if (!event->count && !event->lock)
    {
        event->count = 1;

        armv6m_event_insert(event);
    }
    else
    {
        event->count++;
    }
}

static void armv6m_event_svc_lock(armv6m_event_t *event)
{
    if (event->next)
    {
        armv6m_event_remove(event);
    }

    event->lock = 1;
}

static void armv6m_event_svc_unlock(armv6m_event_t *event)
{
    event->lock = 0;

    if (event->count)
    {
        if (!event->next)
        {
            armv6m_event_insert(event);
        }
    }
}

static void armv6m_event_svc_set_priority(armv6m_event_t *event, uint32_t priority)
{
    if (event->priority != priority)
    {
        if (event->next)
        {
            armv6m_event_remove(event);

            event->priority = priority;
            
            armv6m_event_insert(event);
        }
        else
        {
            event->priority = priority;
        }
    }
}

static uint32_t armv6m_event_svc_dequeue(armv6m_core_callback_t *callback)
{
    armv6m_event_t *event;

    if (!armv6m_event_control.cooperative)
    {
        return 0;
    }
    else
    {
        event = armv6m_event_control.cooperative;

        event->count--;

        if (event->count == 0)
        {
            armv6m_event_remove(event);
        }

        callback->routine = event->routine;
        callback->context = event->context;

        return 1;
    }
}

void armv6m_event_initialize(void)
{
    armv6m_event_control.stack = NULL;
    armv6m_event_control.self = NULL;
    armv6m_event_control.next = NULL;
    armv6m_event_control.preemptive = NULL;
    armv6m_event_control.cooperative = NULL;
    armv6m_event_control.ceiling = 255;
}

void armv6m_event_create(armv6m_event_t *event, armv6m_event_routine_t routine, void *context, uint8_t priority)
{
    event->next = NULL;
    event->previous = NULL;
    event->routine = routine;
    event->context = context;
    event->priority = priority;
    event->lock = 0;
    event->count = 0;
}

void armv6m_event_destroy(armv6m_event_t *event)
{
    if (__get_IPSR() == 0)
    {
        armv6m_svcall_1((uint32_t)&armv6m_event_svc_destroy, (uint32_t)event);
    }
    else
    {
        armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)armv6m_event_svc_destroy, (void*)event, 0);
    }
}

void armv6m_event_enqueue(armv6m_event_t *event)
{
    if (__get_IPSR() == 0)
    {
        armv6m_svcall_1((uint32_t)&armv6m_event_svc_enqueue, (uint32_t)event);
    }
    else
    {
        armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)armv6m_event_svc_enqueue, (void*)event, 0);
    }
}

void armv6m_event_lock(armv6m_event_t *event)
{
    if (__get_IPSR() == 0)
    {
        armv6m_svcall_1((uint32_t)&armv6m_event_svc_lock, (uint32_t)event);
    }
    else
    {
        armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)armv6m_event_svc_lock, (void*)event, 0);
    }
}

void armv6m_event_unlock(armv6m_event_t *event)
{
    if (__get_IPSR() == 0)
    {
        armv6m_svcall_1((uint32_t)&armv6m_event_svc_unlock, (uint32_t)event);
    }
    else
    {
        armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)armv6m_event_svc_unlock, (void*)event, 0);
    }
}

void armv6m_event_set_priority(armv6m_event_t *event, uint8_t priority)
{
    if (__get_IPSR() == 0)
    {
        armv6m_svcall_2((uint32_t)&armv6m_event_svc_set_priority, (uint32_t)event, priority);
    }
    else
    {
        armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)armv6m_event_svc_set_priority, (void*)event, priority);
    }
}

uint8_t armv6m_event_acquire(uint8_t priority)
{
    uint32_t ceiling;

    ceiling = armv6m_event_control.ceiling;

    if (priority < ceiling)
    {
        armv6m_event_control.ceiling = priority;
    }

    return ceiling;
}

void armv6m_event_release(uint8_t priority)
{
    armv6m_event_control.ceiling = priority;
}

bool armv6m_event_dequeue(void)
{
    armv6m_core_callback_t callback;

    if (__get_IPSR() != 0)
    {
        return false;
    }

    if (!armv6m_svcall_1((uint32_t)&armv6m_event_svc_dequeue, (uint32_t)&callback))
    {
        return false;
    }

    (*callback.routine)(callback.routine);

    return true;
}

armv6m_event_t *armv6m_event_self(void)
{
    if (__get_IPSR() != 0)
    {
        return NULL;
    }

    return armv6m_event_control.self;
}
