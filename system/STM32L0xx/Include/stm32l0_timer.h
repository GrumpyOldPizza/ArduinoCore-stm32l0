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

#if !defined(_STM32L0_TIMER_H)
#define _STM32L0_TIMER_H

#include "armv6m.h"
#include "stm32l0xx.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    STM32L0_TIMER_INSTANCE_TIM2 = 0,   /* GENERAL  16 */
#if defined(STM32L072xx) || defined(STM32L082xx)
    STM32L0_TIMER_INSTANCE_TIM3,       /* GENERAL  16 */
#endif /* STM32L072xx || STM32L082xx */
    STM32L0_TIMER_INSTANCE_TIM6,       /* BASIC    16 */
#if defined(STM32L072xx) || defined(STM32L082xx)
    STM32L0_TIMER_INSTANCE_TIM7,       /* BASIC    16 */
#endif /* STM32L072xx || STM32L082xx */
    STM32L0_TIMER_INSTANCE_TIM21,      /* GENERAL  16 */
    STM32L0_TIMER_INSTANCE_TIM22,      /* GENERAL  16 */
    STM32L0_TIMER_INSTANCE_COUNT
};

#define STM32L0_TIMER_CHANNEL_1                          0
#define STM32L0_TIMER_CHANNEL_2                          1
#define STM32L0_TIMER_CHANNEL_3                          2
#define STM32L0_TIMER_CHANNEL_4                          3

#define STM32L0_TIMER_OPTION_ENCODER_MODE_MASK           0x00000003
#define STM32L0_TIMER_OPTION_ENCODER_MODE_SHIFT          0
#define STM32L0_TIMER_OPTION_ENCODER_MODE_CHANNEL_1      0x00000001
#define STM32L0_TIMER_OPTION_ENCODER_MODE_CHANNEL_2      0x00000002
#define STM32L0_TIMER_OPTION_ENCODER_MODE_CHANNEL_1_OR_2 0x00000003
#define STM32L0_TIMER_OPTION_CLOCK_EXTERNAL_CHANNEL_1    0x00000004
#define STM32L0_TIMER_OPTION_CLOCK_EXTERNAL_CHANNEL_2    0x00000008
#define STM32L0_TIMER_OPTION_COUNT_UP                    0x00000000
#define STM32L0_TIMER_OPTION_COUNT_DOWN                  0x00000010
#define STM32L0_TIMER_OPTION_COUNT_CENTER_MASK           0x00000060
#define STM32L0_TIMER_OPTION_COUNT_CENTER_SHIFT          5
#define STM32L0_TIMER_OPTION_COUNT_CENTER_UP             0x00000020
#define STM32L0_TIMER_OPTION_COUNT_CENTER_DOWN           0x00000040
#define STM32L0_TIMER_OPTION_COUNT_CENTER_UP_DOWN        0x00000060
#define STM32L0_TIMER_OPTION_COUNT_PRELOAD               0x00000080

#define STM32L0_TIMER_EVENT_PERIOD                       0x08000000
#define STM32L0_TIMER_EVENT_CHANNEL_1                    0x10000000
#define STM32L0_TIMER_EVENT_CHANNEL_2                    0x20000000
#define STM32L0_TIMER_EVENT_CHANNEL_3                    0x40000000
#define STM32L0_TIMER_EVENT_CHANNEL_4                    0x80000000

#define STM32L0_TIMER_CONTROL_DISABLE                    0x00000000
#define STM32L0_TIMER_CONTROL_CAPTURE_MASK               0x00000003
#define STM32L0_TIMER_CONTROL_CAPTURE_RISING_EDGE        0x00000001
#define STM32L0_TIMER_CONTROL_CAPTURE_FALLING_EDGE       0x00000002
#define STM32L0_TIMER_CONTROL_CAPTURE_BOTH_EDGES         0x00000003
#define STM32L0_TIMER_CONTROL_CAPTURE_POLARITY           0x00000004
#define STM32L0_TIMER_CONTROL_CAPTURE_ALTERNATE          0x00000008
#define STM32L0_TIMER_CONTROL_CAPTURE_PRESCALE_MASK      0x00000030
#define STM32L0_TIMER_CONTROL_CAPTURE_PRESCALE_SHIFT     4
#define STM32L0_TIMER_CONTROL_CAPTURE_PRESCALE_1         0x00000000
#define STM32L0_TIMER_CONTROL_CAPTURE_PRESCALE_2         0x00000010
#define STM32L0_TIMER_CONTROL_CAPTURE_PRESCALE_4         0x00000020
#define STM32L0_TIMER_CONTROL_CAPTURE_PRESCALE_8         0x00000030
#define STM32L0_TIMER_CONTROL_CAPTURE_FILTER_MASK        0x000003c0
#define STM32L0_TIMER_CONTROL_CAPTURE_FILTER_SHIFT       6
#define STM32L0_TIMER_CONTROL_CAPTURE_FILTER(_n)         (((_n) << STM32L0_TIMER_CONTROL_CAPTURE_FILTER_SHIFT) & STM32L0_TIMER_CONTROL_CAPTURE_FILTER_MASK)
#define STM32L0_TIMER_CONTROL_COMPARE_MASK               0x00070000
#define STM32L0_TIMER_CONTROL_COMPARE_TIMING             0x00010000
#define STM32L0_TIMER_CONTROL_COMPARE_ACTIVE             0x00020000
#define STM32L0_TIMER_CONTROL_COMPARE_INACTIVE           0x00030000
#define STM32L0_TIMER_CONTROL_COMPARE_TOGGLE             0x00040000
#define STM32L0_TIMER_CONTROL_COMPARE_FORCED_ACTIVE      0x00050000
#define STM32L0_TIMER_CONTROL_COMPARE_FORCED_INACTIVE    0x00060000
#define STM32L0_TIMER_CONTROL_PWM                        0x00070000

typedef void (*stm32l0_timer_callback_t)(void *context, uint32_t events);

#define STM32L0_TIMER_STATE_NONE                         0
#define STM32L0_TIMER_STATE_INIT                         1
#define STM32L0_TIMER_STATE_BUSY                         2
#define STM32L0_TIMER_STATE_READY                        3
#define STM32L0_TIMER_STATE_ACTIVE                       4

typedef struct _stm32l0_timer_t {
    TIM_TypeDef                 *TIM;
    volatile uint8_t            state;
    uint8_t                     instance;
    uint8_t                     interrupt;
    uint8_t                     priority;
    stm32l0_timer_callback_t    callback;
    void                        *context;
    uint32_t                    events;
    volatile uint32_t           channels;
} stm32l0_timer_t;

extern bool     stm32l0_timer_create(stm32l0_timer_t *timer, unsigned int instance, unsigned int priority, unsigned int mode);
extern bool     stm32l0_timer_destroy(stm32l0_timer_t *timer);
extern uint32_t stm32l0_timer_clock(stm32l0_timer_t *timer);
extern bool     stm32l0_timer_enable(stm32l0_timer_t *timer, uint32_t prescaler, uint32_t option, stm32l0_timer_callback_t callback, void *context, uint32_t events);
extern bool     stm32l0_timer_disable(stm32l0_timer_t *timer);
extern bool     stm32l0_timer_configure(stm32l0_timer_t *timer, uint32_t prescaler, uint32_t option);
extern bool     stm32l0_timer_notify(stm32l0_timer_t *timer, stm32l0_timer_callback_t callback, void *context, uint32_t events);
extern bool     stm32l0_timer_start(stm32l0_timer_t *timer, uint32_t period, bool oneshot);
extern bool     stm32l0_timer_stop(stm32l0_timer_t *timer);
extern uint32_t stm32l0_timer_count(stm32l0_timer_t *timer);
extern bool     stm32l0_timer_period(stm32l0_timer_t *timer, uint32_t period, bool offset);
extern bool     stm32l0_timer_channel(stm32l0_timer_t *timer, unsigned int channel, uint32_t compare, uint32_t control);
extern bool     stm32l0_timer_compare(stm32l0_timer_t *timer, unsigned int channel, uint32_t compare);
extern uint32_t stm32l0_timer_capture(stm32l0_timer_t *timer, unsigned int channel);

#ifdef __cplusplus
}
#endif

#endif /* _STM32L0_TIMER_H */
