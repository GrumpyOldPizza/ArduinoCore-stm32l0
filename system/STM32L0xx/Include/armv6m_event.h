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

#if !defined(_ARMV6M_EVENT_H)
#define _ARMV6M_EVENT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*armv6m_event_routine_t)(void *context);

typedef struct _armv6m_event_t {
    struct _armv6m_event_t         *next;
    struct _armv6m_event_t         *previous;
    armv6m_event_routine_t         routine;
    void                           *context;
    uint8_t                        priority;
    volatile uint8_t               lock;
    volatile uint16_t              count;
} armv6m_event_t;

#define ARMV6M_EVENT_PRIORITY_COOPERATIVE 255

extern void armv6m_event_initialize(void);
extern void armv6m_event_create(armv6m_event_t *event, armv6m_event_routine_t routine, void *context, uint8_t priority);
extern void armv6m_event_destroy(armv6m_event_t *event);
extern void armv6m_event_enqueue(armv6m_event_t *event);               /* enqueue event for execution */
extern void armv6m_event_lock(armv6m_event_t *event);                  /* lock event execution */
extern void armv6m_event_unlock(armv6m_event_t *event);                /* unlock event execution */
extern void armv6m_event_set_priority(armv6m_event_t *event, uint8_t priority);
extern uint8_t armv6m_event_acquire(uint8_t priority);                 /* only events with priority lower than "priority" get scheduled async */
extern void armv6m_event_release(uint8_t priority);                    /* restore old priority threshold */
extern bool armv6m_event_dequeue(void);                                /* execute one sync event */
extern armv6m_event_t *armv6m_event_self(void);                        /* currently executing event */

#ifdef __cplusplus
}
#endif

#endif /* _ARMV6M_EVENT_H */
