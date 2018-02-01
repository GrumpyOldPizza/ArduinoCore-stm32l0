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

#include "armv6m.h"
#include "stm32l0xx.h"

#include "stm32l0_gpio.h"
#include "stm32l0_servo.h"
#include "stm32l0_system.h"

static void stm32l0_servo_timer_callback(void *context, uint32_t events)
{
    stm32l0_servo_t *servo = (stm32l0_servo_t*)context;
    stm32l0_servo_schedule_t *active, *pending;
    unsigned int index;

    index = servo->index;
    active = servo->active;

    if (index != active->entries)
    {
        active->slot[index].GPIO->BRR = active->slot[index].mask;
            
        index++;

        if (index != active->entries)
        {
            active->slot[index].GPIO->BSRR = active->slot[index].mask;

            stm32l0_timer_period(servo->timer, active->slot[index].width -1, true);
        }
        else
        {
            stm32l0_timer_period(servo->timer, active->sync -1, true);
        }

        servo->index = index;
    }
    else
    {
        servo->index = 0;

        pending = servo->pending;

        if (pending == NULL)
        {
            active->slot[0].GPIO->BSRR = active->slot[0].mask;

            stm32l0_timer_period(servo->timer, active->slot[0].width -1, true);
        }
        else
        {
            if (pending->entries)
            {
                pending->slot[0].GPIO->BSRR = pending->slot[0].mask;

                stm32l0_timer_period(servo->timer, pending->slot[0].width -1, true);
            }
            else
            {
                stm32l0_timer_stop(servo->timer);

                servo->state = STM32L0_SERVO_STATE_READY;
            }

            servo->pending = NULL;
            servo->active = pending;
        }

        if (servo->callback)
        {
            (*servo->callback)(servo->context, STM32L0_SERVO_EVENT_SYNC);
        }
    }
}

bool stm32l0_servo_create(stm32l0_servo_t *servo, stm32l0_timer_t *timer)
{
    servo->state = STM32L0_SERVO_STATE_INIT;
    servo->timer = timer;
    servo->callback = NULL;
    servo->context = NULL;

    servo->index = 0;
    servo->prescaler = 0;
    servo->active = NULL;
    servo->pending = NULL;

    return true;
}

bool stm32l0_servo_destroy(stm32l0_servo_t *servo)
{
    if (servo->state != STM32L0_SERVO_STATE_INIT)
    {
        return false;
    }

    servo->state = STM32L0_SERVO_STATE_NONE;

    return true;
}

bool stm32l0_servo_enable(stm32l0_servo_t *servo, const stm32l0_servo_table_t *table, stm32l0_servo_event_callback_t callback, void *context)
{
    if (servo->state != STM32L0_SERVO_STATE_INIT)
    {
        return false;
    }

    servo->state = STM32L0_SERVO_STATE_BUSY;

    if (!stm32l0_timer_enable(servo->timer, 0, 0, stm32l0_servo_timer_callback, servo, STM32L0_TIMER_EVENT_PERIOD))
    {
        servo->state = STM32L0_SERVO_STATE_INIT;

        return false;
    }

    if (!stm32l0_servo_configure(servo, table))
    {
        stm32l0_timer_disable(servo->timer);

        servo->state = STM32L0_SERVO_STATE_INIT;

        return false;
    }

    servo->callback = callback;
    servo->context = context;

    return true;
}

bool stm32l0_servo_disable(stm32l0_servo_t *servo)
{
    if (servo->state != STM32L0_SERVO_STATE_READY)
    {
        return false;
    }

    stm32l0_timer_disable(servo->timer);

    servo->callback = NULL;
    servo->context = NULL;

    servo->state = STM32L0_SERVO_STATE_INIT;

    return true;
}

bool stm32l0_servo_configure(stm32l0_servo_t *servo, const stm32l0_servo_table_t *table)
{
    stm32l0_servo_schedule_t *pending;
    unsigned int entry, index, offset;

    if (servo->state < STM32L0_SERVO_STATE_BUSY)
    {
        return false;
    }

    servo->pending = NULL;

    pending = ((servo->active == &servo->schedule[1]) ? &servo->schedule[0] : &servo->schedule[1]);

    for (offset = 0, entry = 0, index = 0; entry < table->entries; entry++)
    {
        if ((table->slot[entry].pin != STM32L0_GPIO_PIN_NONE) && (table->slot[index].width >= STM32L0_SERVO_PULSE_WIDTH))
        {
            pending->slot[index].GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * ((table->slot[entry].pin & STM32L0_GPIO_PIN_GROUP_MASK) >> STM32L0_GPIO_PIN_GROUP_SHIFT));
            pending->slot[index].mask = (1ul << ((table->slot[entry].pin & STM32L0_GPIO_PIN_INDEX_MASK) >> STM32L0_GPIO_PIN_INDEX_SHIFT));
            pending->slot[index].width = table->slot[entry].width;

            offset += table->slot[entry].width;
            index++;
        }
    }

    if (offset == 0)
    {
        pending->sync = 0;
        pending->entries = 0;
    }
    else
    {
        pending->entries = index;

        servo->prescaler = stm32l0_timer_clock(servo->timer) / 1000000;

        if ((offset + STM32L0_SERVO_SYNC_WIDTH) >= STM32L0_SERVO_FRAME_WIDTH)
        {
            pending->sync = STM32L0_SERVO_SYNC_WIDTH;
        }
        else
        {
            pending->sync = STM32L0_SERVO_FRAME_WIDTH - offset;
        }
    }

    if (servo->active)
    {
        servo->pending = pending;
    }
    else
    {
        if (pending->entries)
        {
            servo->state = STM32L0_SERVO_STATE_ACTIVE;

            servo->active = pending;
            servo->index = 0;
            
            stm32l0_timer_configure(servo->timer, servo->prescaler -1, 0);
            stm32l0_timer_start(servo->timer, pending->slot[0].width -1, false);
            
            pending->slot[0].GPIO->BSRR = pending->slot[0].mask;
        }
    }

    return true;
}
