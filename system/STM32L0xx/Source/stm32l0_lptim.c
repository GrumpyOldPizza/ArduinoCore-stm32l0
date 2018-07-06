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
#include "stm32l0xx.h"

#include "stm32l0_lptim.h"
#include "stm32l0_system.h"

extern void LPTIM1_IRQHandler(void);

#define STM32L0_LPTIM_STATE_NONE    0
#define STM32L0_LPTIM_STATE_READY   1
#define STM32L0_LPTIM_STATE_TIMEOUT 2

typedef struct _stm32l0_lptim_device_t {
    volatile uint32_t        state;
    stm32l0_lptim_callback_t callback;
    void                     *context;
} stm32l0_lptim_device_t;

static stm32l0_lptim_device_t stm32l0_lptim_device;

void stm32l0_lptim_configure(unsigned int priority)
{
    NVIC_SetPriority(LPTIM1_IRQn, priority);
    NVIC_EnableIRQ(LPTIM1_IRQn);
}

bool stm32l0_lptim_start(uint32_t timeout, stm32l0_lptim_callback_t callback, void *context)
{
    uint32_t period, presc;
    
    if (stm32l0_lptim_device.state == STM32L0_LPTIM_STATE_NONE)
    {
        stm32l0_system_periph_enable(STM32L0_SYSTEM_PERIPH_LPTIM1);

        stm32l0_lptim_device.state = STM32L0_LPTIM_STATE_READY;
    }
    else
    {
        armv6m_atomic_and(&LPTIM1->CR, ~LPTIM_CR_ENABLE);
    }

    stm32l0_lptim_device.callback = callback;
    stm32l0_lptim_device.context = context;

    if (timeout <= 65536)
    {
        period = timeout;
        presc = 0;
    }
    else
    {
        if (timeout > 65536 * 128)
        {
            period = 65536 * 128;
            presc = LPTIM_CFGR_PRESC_0 | LPTIM_CFGR_PRESC_1 | LPTIM_CFGR_PRESC_2;;
        }
        else
        {
            period = timeout;
            presc = 0;
    
            while (period > 65536)
            {
                period = (period + 1) / 2;
                presc += LPTIM_CFGR_PRESC_0;
            }
        }
    }

    stm32l0_lptim_device.state = STM32L0_LPTIM_STATE_TIMEOUT;

    LPTIM1->ICR = ~0ul;
    LPTIM1->IER = LPTIM_IER_ARRMIE;
    LPTIM1->CFGR = presc;

    LPTIM1->CR |= LPTIM_CR_ENABLE;
    LPTIM1->ARR = period -1;
    LPTIM1->CR |= LPTIM_CR_SNGSTRT;

    return true;
}

uint32_t stm32l0_lptim_stop(void)
{
    uint32_t count;

    armv6m_atomic_and(&LPTIM1->CR, ~LPTIM_CR_ENABLE);

    LPTIM1->ICR = LPTIM_ICR_ARRMCF;

    count = (LPTIM1->CNT & 0xffff) << ((LPTIM1->CFGR & LPTIM_CFGR_PRESC_Msk) >> LPTIM_CFGR_PRESC_Pos);

    stm32l0_system_periph_enable(STM32L0_SYSTEM_PERIPH_LPTIM1);

    stm32l0_lptim_device.state = STM32L0_LPTIM_STATE_NONE;

    return count;
}

bool stm32l0_lptim_done(void)
{
    return (stm32l0_lptim_device.state != STM32L0_LPTIM_STATE_TIMEOUT);
}

void LPTIM1_IRQHandler(void)
{
    if (LPTIM1->ISR & LPTIM_ISR_ARRM)
    {
        LPTIM1->ICR = LPTIM_ICR_ARRMCF;
        
        stm32l0_lptim_device.state = STM32L0_LPTIM_STATE_READY;
        
        if (stm32l0_lptim_device.callback)
        {
            (*stm32l0_lptim_device.callback)(stm32l0_lptim_device.context);
        }
        
        if (stm32l0_lptim_device.state == STM32L0_LPTIM_STATE_READY)
        {
            stm32l0_lptim_stop();
        }
    }
}
