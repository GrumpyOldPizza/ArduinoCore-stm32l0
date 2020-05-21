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

#include "stm32l0_exti.h"
#include "stm32l0_gpio.h"

typedef struct _stm32l0_exti_device_t {
    uint16_t                events;
    uint16_t                mask;
    uint16_t                nowakeup;
    volatile uint16_t       pending;
    volatile uint16_t       priority[4];
    stm32l0_exti_callback_t callback[16];
    void                    *context[16];
} stm32l0_exti_device_t;

static stm32l0_exti_device_t stm32l0_exti_device;

void __stm32l0_exti_initialize(void)
{
    stm32l0_exti_device.events = 0;
    stm32l0_exti_device.mask = ~0;
    stm32l0_exti_device.nowakeup = 0;
    stm32l0_exti_device.pending = 0;
    stm32l0_exti_device.priority[0] = 0;
    stm32l0_exti_device.priority[1] = 0;
    stm32l0_exti_device.priority[2] = 0;
    stm32l0_exti_device.priority[3] = 0;

    NVIC_SetPriority(EXTI0_1_IRQn, ARMV6M_IRQ_PRIORITY_CRITICAL);
    NVIC_SetPriority(EXTI2_3_IRQn, ARMV6M_IRQ_PRIORITY_CRITICAL);
    NVIC_SetPriority(EXTI4_15_IRQn, ARMV6M_IRQ_PRIORITY_CRITICAL);
    NVIC_SetPriority(SPI1_IRQn, ARMV6M_IRQ_PRIORITY_HIGH);
    NVIC_SetPriority(SPI2_IRQn, ARMV6M_IRQ_PRIORITY_MEDIUM);

    NVIC_EnableIRQ(EXTI0_1_IRQn);
    NVIC_EnableIRQ(EXTI2_3_IRQn);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
    NVIC_EnableIRQ(SPI1_IRQn);
    NVIC_EnableIRQ(SPI2_IRQn);
}

__attribute__((optimize("O3"))) void __stm32l0_exti_stop_enter(void)
{
    EXTI->IMR &= ~stm32l0_exti_device.nowakeup;
}

__attribute__((optimize("O3"))) void __stm32l0_exti_stop_leave(void)
{
    EXTI->IMR |= (stm32l0_exti_device.nowakeup & stm32l0_exti_device.mask);
}  

__attribute__((optimize("O3"))) bool stm32l0_exti_attach(uint16_t pin, uint32_t control, stm32l0_exti_callback_t callback, void *context)
{
    unsigned int mask, index, group;

    index = (pin & STM32L0_GPIO_PIN_INDEX_MASK) >> STM32L0_GPIO_PIN_INDEX_SHIFT;
    group = (pin & STM32L0_GPIO_PIN_GROUP_MASK) >> STM32L0_GPIO_PIN_GROUP_SHIFT;

    mask = 1ul << index;

    if (stm32l0_exti_device.events & mask)
    {
        return false;
    }

    stm32l0_exti_device.callback[index] = callback;
    stm32l0_exti_device.context[index] = context;

    armv6m_atomic_modify(&SYSCFG->EXTICR[index >> 2], (0x0000000f << ((index & 3) << 2)), (group << ((index & 3) << 2)));

    if (control & STM32L0_EXTI_CONTROL_EDGE_RISING)
    {
        armv6m_atomic_or(&EXTI->RTSR, mask);
    }
    else
    {
        armv6m_atomic_and(&EXTI->RTSR, ~mask);
    }
    
    if (control & STM32L0_EXTI_CONTROL_EDGE_FALLING)
    {
        armv6m_atomic_or(&EXTI->FTSR, mask);
    }
    else
    {
        armv6m_atomic_and(&EXTI->FTSR, ~mask);
    }

    if (control & STM32L0_EXTI_CONTROL_NOWAKEUP)
    {
        armv6m_atomic_orh(&stm32l0_exti_device.nowakeup, mask);
    }
    
    armv6m_atomic_andh(&stm32l0_exti_device.priority[0], ~mask);
    armv6m_atomic_andh(&stm32l0_exti_device.priority[1], ~mask);
    armv6m_atomic_andh(&stm32l0_exti_device.priority[2], ~mask);
    armv6m_atomic_andh(&stm32l0_exti_device.priority[3], ~mask);
    armv6m_atomic_orh(&stm32l0_exti_device.priority[(control & STM32L0_EXTI_CONTROL_PRIORITY_MASK) >> STM32L0_EXTI_CONTROL_PRIORITY_SHIFT], mask);
    armv6m_atomic_orh(&stm32l0_exti_device.events, mask);

    armv6m_atomic_or(&EXTI->IMR, (stm32l0_exti_device.events & stm32l0_exti_device.mask));

    return true;
}

__attribute__((optimize("O3"))) void stm32l0_exti_detach(uint16_t pin)
{
    unsigned int mask, index;

    index = (pin & STM32L0_GPIO_PIN_INDEX_MASK) >> STM32L0_GPIO_PIN_INDEX_SHIFT;

    mask = 1ul << index;

    armv6m_atomic_and(&EXTI->IMR, ~mask);
    armv6m_atomic_andh(&stm32l0_exti_device.events, ~mask);
    armv6m_atomic_andh(&stm32l0_exti_device.nowakeup, ~mask);
}

__attribute__((optimize("O3"))) bool stm32l0_exti_control(uint16_t pin, uint32_t control)
{
    unsigned int mask, index;

    index = (pin & STM32L0_GPIO_PIN_INDEX_MASK) >> STM32L0_GPIO_PIN_INDEX_SHIFT;

    mask = 1ul << index;

    if (!(stm32l0_exti_device.events & mask))
    {
        return false;
    }

    armv6m_atomic_and(&EXTI->IMR, ~mask);

    armv6m_atomic_andh(&stm32l0_exti_device.priority[0], ~mask);
    armv6m_atomic_andh(&stm32l0_exti_device.priority[1], ~mask);
    armv6m_atomic_andh(&stm32l0_exti_device.priority[2], ~mask);
    armv6m_atomic_andh(&stm32l0_exti_device.priority[3], ~mask);
    armv6m_atomic_andh(&stm32l0_exti_device.events, ~mask);

    return true;
}

__attribute__((optimize("O3"))) void stm32l0_exti_block(uint32_t mask)
{
    mask &= stm32l0_exti_device.events;

    if (mask)
    {
        armv6m_atomic_and(&EXTI->IMR, ~mask);
    }

    armv6m_atomic_andh(&stm32l0_exti_device.mask, ~mask);
}

__attribute__((optimize("O3"))) void stm32l0_exti_unblock(uint32_t mask)
{
    mask &= stm32l0_exti_device.events;

    armv6m_atomic_orh(&stm32l0_exti_device.mask, mask);

    if (mask)
    {
        armv6m_atomic_or(&EXTI->IMR, (mask & 0x0000ffff));
    }
}

static inline void stm32l0_exti_interrupt_1(uint32_t mask, uint32_t index)
{
    if (mask & (1 << index))
    {
        (*stm32l0_exti_device.callback[index])(stm32l0_exti_device.context[index]);
    }
}

static inline void stm32l0_exti_interrupt_2(uint32_t mask)
{
    if (mask)
    {
        __armv6m_atomic_orh(&stm32l0_exti_device.pending, mask);
        
        if (stm32l0_exti_device.priority[1] & mask)
        {
            NVIC_SetPendingIRQ(SPI1_IRQn);
        }
        else
        {
            if (stm32l0_exti_device.priority[2] & mask)
            {
                NVIC_SetPendingIRQ(SPI2_IRQn);
            }
            else
            {
                armv6m_pendsv_raise(ARMV6M_PENDSV_SWI_EXTI);
            }
        }
    }
}

__attribute__((optimize("O3"))) void EXTI0_1_IRQHandler(void)
{
    uint32_t mask, mask_1, mask_2;

    mask = (EXTI->PR & stm32l0_exti_device.mask) & 0x0003;
    
    EXTI->PR = mask;

    mask_1 = mask & stm32l0_exti_device.priority[0];
    
    stm32l0_exti_interrupt_1(mask_1, 0);
    stm32l0_exti_interrupt_1(mask_1, 1);

    mask_2 = mask & ~stm32l0_exti_device.priority[0];

    stm32l0_exti_interrupt_2(mask_2);
}

__attribute__((optimize("O3"))) void EXTI2_3_IRQHandler(void)
{
    uint32_t mask, mask_1, mask_2;

    mask = (EXTI->PR & stm32l0_exti_device.mask) & 0x000c;
    
    EXTI->PR = mask;
    
    mask_1 = mask & stm32l0_exti_device.priority[0];
    
    stm32l0_exti_interrupt_1(mask_1, 2);
    stm32l0_exti_interrupt_1(mask_1, 3);

    mask_2 = mask & ~stm32l0_exti_device.priority[0];

    stm32l0_exti_interrupt_2(mask_2);
}

__attribute__((optimize("O3"))) void EXTI4_15_IRQHandler(void)
{
    uint32_t mask, mask_1, mask_2;

    mask = (EXTI->PR & stm32l0_exti_device.mask) & 0xfff0;
    
    EXTI->PR = mask;
    
    mask_1 = mask & stm32l0_exti_device.priority[0];
    
    stm32l0_exti_interrupt_1(mask_1, 4);
    stm32l0_exti_interrupt_1(mask_1, 5);
    stm32l0_exti_interrupt_1(mask_1, 6);
    stm32l0_exti_interrupt_1(mask_1, 7);
    stm32l0_exti_interrupt_1(mask_1, 8);
    stm32l0_exti_interrupt_1(mask_1, 9);
    stm32l0_exti_interrupt_1(mask_1, 10);
    stm32l0_exti_interrupt_1(mask_1, 11);
    stm32l0_exti_interrupt_1(mask_1, 12);
    stm32l0_exti_interrupt_1(mask_1, 13);
    stm32l0_exti_interrupt_1(mask_1, 14);
    stm32l0_exti_interrupt_1(mask_1, 15);

    mask_2 = mask & ~stm32l0_exti_device.priority[0];

    stm32l0_exti_interrupt_2(mask_2);
}

 __attribute__((optimize("O3"))) void SPI1_IRQHandler(void)
{
    uint32_t mask, index;

    NVIC_ClearPendingIRQ(SPI1_IRQn);

    mask = (stm32l0_exti_device.pending & stm32l0_exti_device.mask) & stm32l0_exti_device.priority[1];

    armv6m_atomic_andh(&stm32l0_exti_device.pending, ~mask);

    while (mask) 
    {
        index = __builtin_ctz(mask);

        mask &= ~(1ul << index); 

        (*stm32l0_exti_device.callback[index])(stm32l0_exti_device.context[index]);
    }
}

 __attribute__((optimize("O3"))) void SPI2_IRQHandler(void)
{
    uint32_t mask, index;

    NVIC_ClearPendingIRQ(SPI2_IRQn);

    mask = (stm32l0_exti_device.pending & stm32l0_exti_device.mask) & stm32l0_exti_device.priority[2];

    armv6m_atomic_andh(&stm32l0_exti_device.pending, ~mask);

    while (mask) 
    {
        index = __builtin_ctz(mask);

        mask &= ~(1ul << index); 

        (*stm32l0_exti_device.callback[index])(stm32l0_exti_device.context[index]);
    }
}

 __attribute__((optimize("O3"))) void SWI_EXTI_IRQHandler(void)
{
    uint32_t mask, index;

    mask = (stm32l0_exti_device.pending & stm32l0_exti_device.mask) & stm32l0_exti_device.priority[3];
    
    armv6m_atomic_andh(&stm32l0_exti_device.pending, ~mask);

    while (mask) 
    {
        index = __builtin_ctz(mask);

        mask &= ~(1ul << index); 

        (*stm32l0_exti_device.callback[index])(stm32l0_exti_device.context[index]);
    }
}
