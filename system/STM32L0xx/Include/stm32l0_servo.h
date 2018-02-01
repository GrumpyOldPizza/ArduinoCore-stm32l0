/*
 * Copyright (c) 2016-2018 Thomas Roell.  All rights reserved.
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

#if !defined(_STM32L0_SERVO_H)
#define _STM32L0_SERVO_H

#include "armv6m.h"
#include "stm32l0xx.h"

#include "stm32l0_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STM32L0_SERVO_EVENT_SYNC                         0x00000001

typedef void (*stm32l0_servo_event_callback_t)(void *context, uint32_t events);

#define STM32L0_SERVO_STATE_NONE                         0
#define STM32L0_SERVO_STATE_INIT                         1
#define STM32L0_SERVO_STATE_BUSY                         2
#define STM32L0_SERVO_STATE_READY                        3
#define STM32L0_SERVO_STATE_ACTIVE                       4

#define STM32L0_SERVO_SLOT_COUNT                         10

#define STM32L0_SERVO_SYNC_WIDTH                         100     /* sync needs to be at least 100us after the last pulse    */
#define STM32L0_SERVO_FRAME_WIDTH                        20000   /* the default RC servo frame is 20000us                    */
#define STM32L0_SERVO_PULSE_WIDTH                        100     /* below a pulse is deemed illegal                          */

typedef struct _stm32l0_servo_table_t {
    uint32_t                    entries;
    struct {
        uint16_t                  pin;
        uint16_t                  width;
    }                           slot[STM32L0_SERVO_SLOT_COUNT];
} stm32l0_servo_table_t;

typedef struct _stm32l0_servo_schedule_t {
    uint16_t                    sync;
    uint16_t                    entries;
    struct {
        GPIO_TypeDef              *GPIO;
        uint16_t                  mask;
        uint16_t                  width;
    }                           slot[STM32L0_SERVO_SLOT_COUNT];
} stm32l0_servo_schedule_t;

typedef struct _stm32l0_servo_t {
    volatile uint8_t                  state;
    uint8_t                           index;
    uint16_t                          prescaler;
    stm32l0_timer_t                   *timer;
    stm32l0_servo_event_callback_t    callback;
    void                              *context;
    stm32l0_servo_schedule_t * volatile active;
    stm32l0_servo_schedule_t * volatile pending;
    stm32l0_servo_schedule_t          schedule[2];
} stm32l0_servo_t;

extern bool stm32l0_servo_create(stm32l0_servo_t *servo, stm32l0_timer_t *timer);
extern bool stm32l0_servo_destroy(stm32l0_servo_t *servo);
extern bool stm32l0_servo_enable(stm32l0_servo_t *servo, const stm32l0_servo_table_t *table, stm32l0_servo_event_callback_t callback, void *context);
extern bool stm32l0_servo_disable(stm32l0_servo_t *servo);
extern bool stm32l0_servo_configure(stm32l0_servo_t *servo, const stm32l0_servo_table_t *table);

#ifdef __cplusplus
}
#endif

#endif /* _STM32L4_SERVO_H */
