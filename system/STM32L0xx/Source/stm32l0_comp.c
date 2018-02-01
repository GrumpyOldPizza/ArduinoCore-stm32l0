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

#include "stm32l0_comp.h"
#include "stm32l0_gpio.h"
#include "stm32l0_system.h"

extern void ADC1_COMP_IRQHandler(void);

#define STM32L0_COMP_STATE_NONE  0
#define STM32L0_COMP_STATE_READY 1

typedef struct _stm32l0_comp_device_t {
    volatile uint32_t       state;
    stm32l0_comp_callback_t callback;
    void                    *context;
} stm32l0_comp_device_t;

static stm32l0_comp_device_t stm32l0_comp_device;

void stm32l0_comp_configure(unsigned int priority)
{
    NVIC_SetPriority(ADC1_COMP_IRQn, priority);

    NVIC_EnableIRQ(ADC1_COMP_IRQn);
}

bool stm32l0_comp_enable(uint16_t pin, uint32_t option)
{
    uint32_t comp_csr;

    if (stm32l0_comp_device.state != STM32L0_COMP_STATE_NONE)
    {
        return false;
    }

    if      (pin == STM32L0_GPIO_PIN_PA3) { comp_csr = 0;                                               }
    else if (pin == STM32L0_GPIO_PIN_PB4) { comp_csr = COMP_CSR_COMP2INNSEL_0;                          }
    else if (pin == STM32L0_GPIO_PIN_PB5) { comp_csr = COMP_CSR_COMP2INNSEL_1;                          }
    else if (pin == STM32L0_GPIO_PIN_PB7) { comp_csr = COMP_CSR_COMP2INNSEL_1 | COMP_CSR_COMP2INNSEL_0; }
    else
    {
        return false;
    }

    if ((option & STM32L0_COMP_OPTION_REFERENCE_MASK) == STM32L0_COMP_OPTION_REFERENCE_EXTERNAL)
    {
        comp_csr |= (COMP_CSR_COMP2INPSEL_0);
    }
    else if ((option & STM32L0_COMP_OPTION_REFERENCE_MASK) == STM32L0_COMP_OPTION_REFERENCE_DAC1)
    {
        comp_csr |= (COMP_CSR_COMP2INPSEL_1);
    }
    else if ((option & STM32L0_COMP_OPTION_REFERENCE_MASK) == STM32L0_COMP_OPTION_REFERENCE_DAC2)
    {
        comp_csr |= (COMP_CSR_COMP2INPSEL_1 | COMP_CSR_COMP2INPSEL_0);
    }
    else
    {
        if ((option & STM32L0_COMP_OPTION_REFERENCE_MASK) == STM32L0_COMP_OPTION_REFERENCE_1_4_VREFINT)
        {
            comp_csr |= (COMP_CSR_COMP2INPSEL_2);
        }

        if ((option & STM32L0_COMP_OPTION_REFERENCE_MASK) == STM32L0_COMP_OPTION_REFERENCE_1_2_VREFINT)
        {
            comp_csr |= (COMP_CSR_COMP2INPSEL_2 | COMP_CSR_COMP2INPSEL_0);
        }

        if ((option & STM32L0_COMP_OPTION_REFERENCE_MASK) == STM32L0_COMP_OPTION_REFERENCE_3_4_VREFINT)
        {
            comp_csr |= (COMP_CSR_COMP2INPSEL_2 | COMP_CSR_COMP2INPSEL_1);
        }

        armv6m_atomic_or(&SYSCFG->CFGR3, (SYSCFG_CFGR3_ENBUF_VREFINT_COMP2 | SYSCFG_CFGR3_EN_VREFINT));

        while (!(SYSCFG->CFGR3 & SYSCFG_CFGR3_VREFINT_RDYF))
        {
        }

        stm32l0_system_lock(STM32L0_SYSTEM_LOCK_VREFINT);
    }

    if (option & STM32L0_COMP_OPTION_POLARITY)
    {
        comp_csr |= COMP_CSR_COMP2POLARITY;
    }

    COMP2->CSR = comp_csr;
    COMP2->CSR = comp_csr | ~COMP_CSR_COMP2EN;
    
    stm32l0_comp_device.state = STM32L0_COMP_STATE_READY;

    return true;
}

bool stm32l0_comp_disable(void)
{
    if (stm32l0_comp_device.state != STM32L0_COMP_STATE_READY)
    {
        return false;
    }

    armv6m_atomic_and(&EXTI->IMR, ~EXTI_IMR_IM22);

    COMP2->CSR &= ~COMP_CSR_COMP2EN;

    if (SYSCFG->CFGR3 & SYSCFG_CFGR3_ENBUF_VREFINT_COMP2)
    {
        stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_VREFINT);

        armv6m_atomic_and(&SYSCFG->CFGR3, ~SYSCFG_CFGR3_ENBUF_VREFINT_COMP2);
    }

    stm32l0_comp_device.state = STM32L0_COMP_STATE_NONE;

    return true;
}

bool stm32l0_comp_attach(uint32_t control, stm32l0_comp_callback_t callback, void *context)
{
    if (stm32l0_comp_device.state != STM32L0_COMP_STATE_READY)
    {
        return false;
    }

    armv6m_atomic_and(&EXTI->IMR, ~EXTI_IMR_IM22);

    stm32l0_comp_device.callback = callback;
    stm32l0_comp_device.context = context;

    if (control & STM32L0_COMP_CONTROL_EDGE_RISING)
    {
        armv6m_atomic_or(&EXTI->RTSR, EXTI_RTSR_RT22);
    }
    else
    {
        armv6m_atomic_and(&EXTI->RTSR, ~EXTI_RTSR_RT22);
    }
    
    if (control & STM32L0_COMP_CONTROL_EDGE_FALLING)
    {
        armv6m_atomic_or(&EXTI->FTSR, EXTI_FTSR_FT22);
    }
    else
    {
        armv6m_atomic_and(&EXTI->FTSR, ~EXTI_FTSR_FT22);
    }

    EXTI->PR = EXTI_PR_PIF22;

    armv6m_atomic_or(&EXTI->IMR, EXTI_IMR_IM22);

    return true;
}

bool stm32l0_comp_detach(void)
{
    if (stm32l0_comp_device.state != STM32L0_COMP_STATE_READY)
    {
        return false;
    }

    armv6m_atomic_and(&EXTI->IMR, ~EXTI_IMR_IM22);

    return true;
}

bool stm32l0_comp_status(void)
{
    if (stm32l0_comp_device.state != STM32L0_COMP_STATE_READY)
    {
        return false;
    }

    return ((COMP2->CSR & COMP_CSR_COMP2VALUE) == COMP_CSR_COMP2VALUE);
}

void ADC1_COMP_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PIF22)
    {
        EXTI->PR = EXTI_PR_PIF22;

        if (stm32l0_comp_device.callback)
        {
            (*stm32l0_comp_device.callback)(stm32l0_comp_device.context);
        }
    }
}
