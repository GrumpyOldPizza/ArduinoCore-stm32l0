/*
 * Copyright (c) 2017-2020 Thomas Roell.  All rights reserved.
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

#include "stm32l0_lptim.h"
#include "stm32l0_system.h"

#define STM32L0_LPTIM_STATE_NONE    0
#define STM32L0_LPTIM_STATE_READY   1

typedef struct _stm32l0_lptim_device_t {
    volatile uint8_t        state;
    volatile uint8_t        busy;
    volatile uint8_t        next;
    volatile uint32_t       clock;
    volatile uint32_t       compare[2];
    volatile uint32_t       reference[2];
    volatile uint32_t       timeout_clock;
    volatile uint32_t       timeout_events;
    stm32l0_lptim_timeout_t *timeout_queue;
} stm32l0_lptim_device_t;

static stm32l0_lptim_device_t stm32l0_lptim_device;

void __stm32l0_lptim_initialize(void)
{
    NVIC_SetPriority(LPTIM1_IRQn, STM32L0_LPTIM_IRQ_PRIORITY);
}

static void stm32l0_lptim_clock_start(uint32_t compare)
{
    stm32l0_system_periph_enable(STM32L0_SYSTEM_PERIPH_LPTIM1);

    LPTIM1->IER = LPTIM_IER_CMPOKIE | LPTIM_IER_ARRMIE | LPTIM_IER_CMPMIE;
    LPTIM1->CFGR = 0;

    LPTIM1->CR = LPTIM_CR_ENABLE;
    LPTIM1->CMP = compare & 0xffff;
    LPTIM1->ARR = 0xffff;
    LPTIM1->CR = LPTIM_CR_CNTSTRT | LPTIM_CR_ENABLE;

    stm32l0_lptim_device.state = STM32L0_LPTIM_STATE_READY;
    stm32l0_lptim_device.busy = 1;
    stm32l0_lptim_device.next = 0;
    stm32l0_lptim_device.clock = 0;

    stm32l0_system_lock(STM32L0_SYSTEM_LOCK_RUN);
    stm32l0_system_lock(STM32L0_SYSTEM_LOCK_DEEPSLEEP);

    NVIC_EnableIRQ(LPTIM1_IRQn);
}

static void stm32l0_lptim_clock_stop()
{
    NVIC_DisableIRQ(LPTIM1_IRQn);

    stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_DEEPSLEEP);

    if (stm32l0_lptim_device.busy)
    {
        stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_RUN);
    }
    
    stm32l0_lptim_device.state = STM32L0_LPTIM_STATE_NONE;
    
    /* ERRATA: MCU may remain stuck in LPTIM interrupt when entering Stop mode
     */
    stm32l0_system_periph_reset(STM32L0_SYSTEM_PERIPH_LPTIM1);
}

static inline __attribute__((optimize("O3"),always_inline)) uint32_t stm32l0_lptim_clock_read()
{
    uint32_t clock, clock_previous;

    if (stm32l0_lptim_device.state == STM32L0_LPTIM_STATE_READY)
    {
        clock = (LPTIM1->CNT & 0xffff) + stm32l0_lptim_device.clock;

        do
        {
            clock_previous = clock;
            
            clock = (LPTIM1->CNT & 0xffff) + stm32l0_lptim_device.clock;
        }
        while (clock != clock_previous);
    }
    else
    {
        clock = 0;
    }

    return clock;
}

static void stm32l0_lptim_timeout_remove(stm32l0_lptim_timeout_t *timeout)
{
    if (timeout->next == timeout)
    {
        stm32l0_lptim_device.timeout_queue = NULL;
    }
    else
    {
        if (timeout == stm32l0_lptim_device.timeout_queue)
        {
            stm32l0_lptim_device.timeout_queue = timeout->next;
        }
        
        timeout->next->previous = timeout->previous;
        timeout->previous->next = timeout->next;
    }
    
    timeout->next = NULL;
    timeout->previous = NULL;
}

static void stm32l0_lptim_timeout_insert(stm32l0_lptim_timeout_t *timeout, uint32_t reference, uint32_t ticks)
{
    stm32l0_lptim_timeout_t *element, *elementNext;
    uint32_t clock, elapsed;

    elapsed = (reference - stm32l0_lptim_device.timeout_clock);

    if (stm32l0_lptim_device.timeout_queue == NULL)
    {
        timeout->next = timeout;
        timeout->previous = timeout;

        stm32l0_lptim_device.timeout_queue = timeout;
    }
    else
    {
        element = stm32l0_lptim_device.timeout_queue;

        do
        {
            elementNext = element->next;

            clock = element->clock;

            if (!((uint32_t)element->callback & 0x00000001))
            {
                stm32l0_lptim_timeout_remove(element);
            }
            else
            {
                if ((elapsed + ticks) < (clock - stm32l0_lptim_device.timeout_clock))
                {
                    if (element == stm32l0_lptim_device.timeout_queue)
                    {
                        stm32l0_lptim_device.timeout_queue = timeout;
                    }
                    break;
                }
            }

            element = elementNext;
        }
        while (element != stm32l0_lptim_device.timeout_queue);

        if (stm32l0_lptim_device.timeout_queue == NULL)
        {
            timeout->next = timeout;
            timeout->previous = timeout;
            
            stm32l0_lptim_device.timeout_queue = timeout;
        }
        else
        {
            timeout->previous = element->previous;
            timeout->next = element;
            
            timeout->previous->next = timeout;
            timeout->next->previous = timeout;
        }
    }

    timeout->clock = stm32l0_lptim_device.timeout_clock + elapsed + ticks;
}

static void stm32l0_lptim_timeout_flush(uint32_t reference)
{
    stm32l0_lptim_timeout_t *timeout;
    stm32l0_lptim_callback_t callback;
    uint32_t elapsed, clock;

    /* Only process the timeout queue when all the outstanding
     * start/restart/stop operatings have been resolved.
     */

    if (stm32l0_lptim_device.timeout_events == 1)
    {
        if (stm32l0_lptim_device.timeout_queue != NULL)
        {
            elapsed = reference - stm32l0_lptim_device.timeout_clock;
        
            do
            {
                timeout = stm32l0_lptim_device.timeout_queue;
                
                clock = timeout->clock;

                if (!((uint32_t)timeout->callback & 0x00000001))
                {
                    stm32l0_lptim_timeout_remove(timeout);
                }
                else
                {
                    if ((clock - stm32l0_lptim_device.timeout_clock) <= elapsed)
                    {
                        stm32l0_lptim_timeout_remove(timeout);

                        callback = timeout->callback;
                    
                        if ((uint32_t)timeout->callback & 0x00000001)
                        {
                            if ((uint32_t)callback & 0xfffffffe)
                            {
                                (*callback)(timeout);
                            }
                        }
                    }
                    else
                    {
                        break;
                    }
                }
            }
            while (stm32l0_lptim_device.timeout_queue != NULL);
        }

        stm32l0_lptim_device.timeout_clock = reference;

        if (armv6m_atomic_sub(&stm32l0_lptim_device.timeout_events, 1) == 1)
        {
            if (stm32l0_lptim_device.timeout_queue != NULL)
            {
                timeout = stm32l0_lptim_device.timeout_queue;

                clock = timeout->clock;

                if ((uint32_t)timeout->callback & 0x00000001)
                {
                    if (stm32l0_lptim_device.state != STM32L0_LPTIM_STATE_NONE)
                    {
                        if (stm32l0_lptim_device.compare[1] != clock)
                        {
                            NVIC_DisableIRQ(LPTIM1_IRQn);

                            stm32l0_lptim_device.compare[1] = clock;
                            stm32l0_lptim_device.reference[1] = reference;
                            stm32l0_lptim_device.next = 1;
                
                            NVIC_EnableIRQ(LPTIM1_IRQn);
                            
                            if (!stm32l0_lptim_device.busy)
                            {
                                if (stm32l0_lptim_device.next)
                                {
                                    stm32l0_lptim_device.next = 0;
                                    
                                    stm32l0_lptim_device.compare[0] = clock;
                                    stm32l0_lptim_device.reference[0] = reference;
                                    
                                    stm32l0_system_lock(STM32L0_SYSTEM_LOCK_RUN);
                                    
                                    stm32l0_lptim_device.busy = 1;
                                    
                                    LPTIM1->CMP = clock & 0xffff;
                                }
                            }
                        }
                    }
                    else
                    {
                        stm32l0_lptim_device.compare[0] = clock;
                        stm32l0_lptim_device.reference[0] = reference;
                        stm32l0_lptim_device.compare[1] = clock;
                        stm32l0_lptim_device.reference[1] = reference;
                        
                        stm32l0_lptim_clock_start(clock);
                    }
                }
            }
            else
            {
                if (stm32l0_lptim_device.state != STM32L0_LPTIM_STATE_NONE)
                {
                    stm32l0_lptim_clock_stop();
                    
                    stm32l0_lptim_device.timeout_clock = 0;
                }
            }
        }
    }
    else
    {
        armv6m_atomic_sub(&stm32l0_lptim_device.timeout_events, 1);
    }
}

static void stm32l0_lptim_timeout_modify_2(stm32l0_lptim_timeout_t *timeout, uint32_t ticks)
{
    uint32_t reference;

    armv6m_atomic_or((volatile uint32_t*)&timeout->callback, 0x00000001);

    reference = stm32l0_lptim_clock_read();

    if (timeout->next)
    {
        stm32l0_lptim_timeout_remove(timeout);
    }
    
    if (ticks)
    {
        stm32l0_lptim_timeout_insert(timeout, reference, ticks);
    }
    
    stm32l0_lptim_timeout_flush(reference);
}

static bool stm32l0_lptim_timeout_modify_1(stm32l0_lptim_timeout_t *timeout, uint32_t ticks, stm32l0_lptim_callback_t callback)
{
    EXCn_Type ipsr;

    ipsr = __get_IPSR();

    if ((ipsr != SVCall_EXCn) && (ipsr != PendSV_EXCn))
    {
        if (!armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)stm32l0_lptim_timeout_modify_2, (void*)timeout, ticks))
        {
            return false;
        }
    }

    armv6m_atomic_add(&stm32l0_lptim_device.timeout_events, 1);

    timeout->callback = (stm32l0_lptim_callback_t)((uint32_t)callback & ~0x00000001);

    if ((ipsr == SVCall_EXCn) || (ipsr == PendSV_EXCn))
    {
        stm32l0_lptim_timeout_modify_2(timeout, ticks);
    }

    return true;
}

void stm32l0_lptim_timeout_create(stm32l0_lptim_timeout_t *timeout)
{
    timeout->next = NULL;
    timeout->previous = NULL;
    timeout->clock = 0;
    timeout->callback = NULL;
}

void stm32l0_lptim_timeout_destroy(stm32l0_lptim_timeout_t *timeout)
{
}

bool stm32l0_lptim_timeout_start(stm32l0_lptim_timeout_t *timeout, uint32_t ticks, stm32l0_lptim_callback_t callback)
{
    if (ticks == 0)
    {
        return false;
    }

    if (__get_IPSR() == ThreadMode_EXCn)
    {
        return armv6m_svcall_3((uint32_t)&stm32l0_lptim_timeout_modify_1, (uint32_t)timeout, (uint32_t)ticks, (uint32_t)callback);
    }
    else
    {
        return stm32l0_lptim_timeout_modify_1(timeout, ticks, callback);
    }

    return true;
}

bool stm32l0_lptim_timeout_stop(stm32l0_lptim_timeout_t *timeout)
{
    if (__get_IPSR() == ThreadMode_EXCn)
    {
        return armv6m_svcall_3((uint32_t)&stm32l0_lptim_timeout_modify_1, (uint32_t)timeout, (uint32_t)0, (uint32_t)NULL);
    }
    else
    {
        return stm32l0_lptim_timeout_modify_1(timeout, 0, NULL);
    }
}

bool stm32l0_lptim_timeout_done(stm32l0_lptim_timeout_t *timeout)
{
    return (timeout->next == NULL);
}

void LPTIM1_IRQHandler(void)
{
    uint32_t lptim_isr, clock, compare, reference;
    bool flush = false;

    lptim_isr = LPTIM1->ISR;

    if (lptim_isr & LPTIM_ISR_ARRM)
    {
        LPTIM1->ICR = LPTIM_ICR_ARRMCF;
        
        if (!stm32l0_lptim_device.busy)
        {
            if ((stm32l0_lptim_device.compare[0] & 0x0000ffff) == 0x0000ffff)
            {
                LPTIM1->ICR = LPTIM_ICR_CMPMCF;
                
                if ((stm32l0_lptim_device.compare[0] & 0xffff0000) == stm32l0_lptim_device.clock)
                {
                    flush = true;
                }
            }
        }

        stm32l0_lptim_device.clock += 0x00010000;
    }

    if (lptim_isr & LPTIM_ISR_CMPM)
    {
        LPTIM1->ICR = LPTIM_ICR_CMPMCF;

        if (!stm32l0_lptim_device.busy)
        {
            if (stm32l0_lptim_device.clock == (stm32l0_lptim_device.compare[0] & 0xffff0000))
            {
                flush = true;
            }
        }
    }

    if (lptim_isr & LPTIM_ISR_CMPOK)
    {
        LPTIM1->ICR = LPTIM_ICR_CMPOKCF;

        if (stm32l0_lptim_device.busy)
        {
            compare = stm32l0_lptim_device.compare[0];
            reference = stm32l0_lptim_device.reference[0];

            clock = stm32l0_lptim_clock_read();

            if ((clock - reference) >= (compare - reference))
            {
                flush = true;
            }

            if (flush || stm32l0_lptim_device.timeout_events)
            {
                stm32l0_lptim_device.busy = 0;
                stm32l0_lptim_device.next = 0;
                    
                stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_RUN);
            }
            else
            {
                if (stm32l0_lptim_device.next)
                {
                    stm32l0_lptim_device.next = 0;
                    
                    compare = stm32l0_lptim_device.compare[1];
                    reference = stm32l0_lptim_device.reference[1];
                    
                    stm32l0_lptim_device.compare[0] = compare;
                    stm32l0_lptim_device.reference[0] = reference;
                    
                    LPTIM1->CMP = compare & 0xffff;
                }
                else
                {
                    stm32l0_lptim_device.busy = 0;
                    
                    stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_RUN);
                }
            }
        }
    }

    if (flush)
    {
        if (armv6m_pendsv_raise(ARMV6M_PENDSV_SWI_LPTIM))
        {
            armv6m_atomic_add(&stm32l0_lptim_device.timeout_events, 1);
        }
    }
}

void SWI_LPTIM_IRQHandler(void)
{
    uint32_t reference;

    reference = stm32l0_lptim_clock_read();

    stm32l0_lptim_timeout_flush(reference);
}
