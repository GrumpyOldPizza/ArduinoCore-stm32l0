/*
 * Copyright (c) 2014-2020 Thomas Roell.  All rights reserved.
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

#include "stm32l0_gpio.h"

typedef struct _stm32l0_gpio_device_t {
    volatile uint32_t       enables[STM32L0_GPIO_PORT_COUNT];
    volatile uint32_t       mask[STM32L0_GPIO_PORT_COUNT];
    volatile uint32_t       mode[STM32L0_GPIO_PORT_COUNT];
    volatile uint32_t       pupd[STM32L0_GPIO_PORT_COUNT];
} stm32l0_gpio_device_t;

static stm32l0_gpio_device_t stm32l0_gpio_device;

void __stm32l0_gpio_initialize(void)
{
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA4, STM32L0_GPIO_MODE_ANALOG);
}

void __stm32l0_gpio_swd_enable(void)
{
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA13_SWDIO, (STM32L0_GPIO_PUPD_PULLUP   | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_ALTERNATE));
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA14_SWCLK, (STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_ALTERNATE));
}

void __stm32l0_gpio_swd_disable(void)
{
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA13, STM32L0_GPIO_MODE_ANALOG);
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA14, STM32L0_GPIO_MODE_ANALOG);
}

__attribute__((optimize("O3"))) void __stm32l0_gpio_stop_enter(stm32l0_gpio_stop_state_t *state)
{
    GPIO_TypeDef *GPIO;
    uint32_t iopen, group, port, mask, mode, pupd;

    iopen = RCC->IOPENR;

    RCC->IOPENR |= STM32L0_GPIO_PORT_RCC_IOPEN;
    RCC->IOPENR;
    
    for (port = 0; port < STM32L0_GPIO_PORT_COUNT; port++)
    {
        group = ((port == (STM32L0_GPIO_PORT_COUNT -1)) ? (STM32L0_GPIO_GROUP_COUNT -1) : port);

        GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

        mode = GPIO->MODER;
        pupd = GPIO->PUPDR;

        mask = stm32l0_gpio_device.mask[port];

        GPIO->MODER = (mode & ~mask) | stm32l0_gpio_device.mode[port];
        GPIO->PUPDR = (pupd & ~mask) | stm32l0_gpio_device.pupd[port];

        state->mode[port] = mode;
        state->pupd[port] = pupd;
    }

    RCC->IOPENR = iopen;
}

__attribute__((optimize("O3"))) void __stm32l0_gpio_stop_leave(stm32l0_gpio_stop_state_t *state)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, port, iopen;

    iopen = RCC->IOPENR;

    RCC->IOPENR |= STM32L0_GPIO_PORT_RCC_IOPEN;
    RCC->IOPENR;
    
    for (port = 0; port < STM32L0_GPIO_PORT_COUNT; port++)
    {
        group = ((port == (STM32L0_GPIO_PORT_COUNT -1)) ? (STM32L0_GPIO_GROUP_COUNT -1) : port);

        GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

        GPIO->MODER = state->mode[port];
        GPIO->PUPDR = state->pupd[port];
    }

    RCC->IOPENR = iopen;
}

__attribute__((optimize("O3"))) void __stm32l0_gpio_pin_write(uint32_t pin, uint32_t data)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, index;

    group = (pin & STM32L0_GPIO_PIN_GROUP_MASK) >> STM32L0_GPIO_PIN_GROUP_SHIFT;
    index = (pin & STM32L0_GPIO_PIN_INDEX_MASK) >> STM32L0_GPIO_PIN_INDEX_SHIFT;

    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    if (data)
    {
        GPIO->BSRR = (1 << index);
    }
    else
    {
        GPIO->BRR = (1 << index);
    }
}

__attribute__((optimize("O3"))) uint32_t __stm32l0_gpio_pin_read(uint32_t pin)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, index;

    group = (pin & STM32L0_GPIO_PIN_GROUP_MASK) >> STM32L0_GPIO_PIN_GROUP_SHIFT;
    index = (pin & STM32L0_GPIO_PIN_INDEX_MASK) >> STM32L0_GPIO_PIN_INDEX_SHIFT;

    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    return ((GPIO->IDR >> index) & 1);
}

__attribute__((optimize("O3"))) void stm32l0_gpio_pin_configure(uint32_t pin, uint32_t mode)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, index, port, afsel, mask, index_2, mask_2;

    afsel = (pin >> 8) & 15;
    group = (pin >> 4) & 7;
    index = (pin >> 0) & 15;
    port = ((group >= STM32L0_GPIO_PORT_COUNT) ? (STM32L0_GPIO_PORT_COUNT -1) : group);

    mask = (1 << index);
    index_2 = (index * 2);
    mask_2 = (3 << index_2);
    
    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    if (!(stm32l0_gpio_device.enables[port] & mask))
    {
        armv6m_atomic_or(&stm32l0_gpio_device.enables[port], mask);
        armv6m_atomic_or(&RCC->IOPENR, (RCC_IOPENR_IOPAEN << group));
        RCC->IOPENR;
    }
    
    /* If the mode is ANALOG, set MODER first */
    if (((mode & STM32L0_GPIO_MODE_MASK) >> STM32L0_GPIO_MODE_SHIFT) == STM32L0_GPIO_MODE_ANALOG)
    {
        armv6m_atomic_modify(&GPIO->MODER, mask_2, ((STM32L0_GPIO_MODE_ANALOG >> STM32L0_GPIO_MODE_SHIFT) << index_2));

        armv6m_atomic_and(&stm32l0_gpio_device.enables[port], ~mask);
        armv6m_atomic_andz(&RCC->IOPENR, ~(RCC_IOPENR_IOPAEN << group), &stm32l0_gpio_device.enables[port], ~0);
    }
    else
    {
        /* Set AFRL/AFRH */
        armv6m_atomic_modify(&GPIO->AFR[index >> 3], (0x0000000f << ((index & 7) << 2)), (afsel << ((index & 7) << 2)));
        
        /* Set OPSPEEDR */
        armv6m_atomic_modify(&GPIO->OSPEEDR, mask_2, (((mode & STM32L0_GPIO_OSPEED_MASK) >> STM32L0_GPIO_OSPEED_SHIFT) << index_2));
        
        /* Set OPTYPER */
        armv6m_atomic_modify(&GPIO->OTYPER, mask, (((mode & STM32L0_GPIO_OTYPE_MASK) >> STM32L0_GPIO_OTYPE_SHIFT) << index));
        
        /* If the mode is OUTPUT, or OUTPUT OPENDRAIN with a ODR of 0. then first switch MODER and then PUPDR
         * to avoid spurious edges. N.b. ALTERNATE is assumed to be INPUT before the peripheral drives it.
         */ 
        if (((mode & STM32L0_GPIO_MODE_MASK) == STM32L0_GPIO_MODE_OUTPUT) &&
            (((mode & STM32L0_GPIO_OTYPE_MASK) != STM32L0_GPIO_OTYPE_OPENDRAIN) || !(GPIO->ODR & mask)))
        {
            /* Set MODE */
            armv6m_atomic_modify(&GPIO->MODER, mask_2, (((mode & STM32L0_GPIO_MODE_MASK) >> STM32L0_GPIO_MODE_SHIFT) << index_2));
            
            /* Set PUPD */
            armv6m_atomic_modify(&GPIO->PUPDR, mask_2, (((mode & STM32L0_GPIO_PUPD_MASK) >> STM32L0_GPIO_PUPD_SHIFT) << index_2));
        }
        else
        {
            /* Set PUPD */
            armv6m_atomic_modify(&GPIO->PUPDR, mask_2, (((mode & STM32L0_GPIO_PUPD_MASK) >> STM32L0_GPIO_PUPD_SHIFT) << index_2));
            
            /* Set MODE */
            armv6m_atomic_modify(&GPIO->MODER, mask_2, (((mode & STM32L0_GPIO_MODE_MASK) >> STM32L0_GPIO_MODE_SHIFT) << index_2));
        }
    }

    if ((mode & STM32L0_GPIO_PARK_MASK) == STM32L0_GPIO_PARK_NONE)
    {
        armv6m_atomic_and(&stm32l0_gpio_device.mask[port], ~mask_2);
        armv6m_atomic_and(&stm32l0_gpio_device.mode[port], ~mask_2);
        armv6m_atomic_and(&stm32l0_gpio_device.pupd[port], ~mask_2);
    }
    else
    {
        armv6m_atomic_or(&stm32l0_gpio_device.mask[port], mask_2);

        if ((mode & STM32L0_GPIO_PARK_MASK) == STM32L0_GPIO_PARK_HIZ)
        {
            armv6m_atomic_modify(&stm32l0_gpio_device.mode[port], mask_2, ((STM32L0_GPIO_MODE_ANALOG >> STM32L0_GPIO_MODE_SHIFT) << index_2));
            armv6m_atomic_and(&stm32l0_gpio_device.pupd[port], ~mask_2);
        }
        else
        {
            armv6m_atomic_modify(&stm32l0_gpio_device.mode[port], mask_2, ((STM32L0_GPIO_MODE_INPUT >> STM32L0_GPIO_MODE_SHIFT) << index_2));
            armv6m_atomic_modify(&stm32l0_gpio_device.pupd[port], mask_2, (((mode & STM32L0_GPIO_PARK_MASK) >> STM32L0_GPIO_PARK_SHIFT) << index_2));
        }
    }
}

__attribute__((optimize("O3"))) void stm32l0_gpio_pin_input(uint32_t pin)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, index, port, mask, index_2, mask_2;

    group = (pin >> 4) & 7;
    index = (pin >> 0) & 15;
    port = ((group >= STM32L0_GPIO_PORT_COUNT) ? (STM32L0_GPIO_PORT_COUNT -1) : group);

    mask = (1 << index);
    index_2 = (index * 2);
    mask_2 = (3 << index_2);
    
    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    if (!(stm32l0_gpio_device.enables[port] & mask))
    {
        armv6m_atomic_or(&stm32l0_gpio_device.enables[port], mask);
        armv6m_atomic_or(&RCC->IOPENR, (RCC_IOPENR_IOPAEN << group));
        RCC->IOPENR;
    }
    
    armv6m_atomic_modify(&GPIO->MODER, mask_2, ((STM32L0_GPIO_MODE_INPUT >> STM32L0_GPIO_MODE_SHIFT) << index_2));
}

__attribute__((optimize("O3"))) void stm32l0_gpio_pin_output(uint32_t pin)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, index, port, mask, index_2, mask_2;

    group = (pin >> 4) & 7;
    index = (pin >> 0) & 15;
    port = ((group >= STM32L0_GPIO_PORT_COUNT) ? (STM32L0_GPIO_PORT_COUNT -1) : group);

    mask = (1 << index);
    index_2 = (index * 2);
    mask_2 = (3 << index_2);
    
    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    if (!(stm32l0_gpio_device.enables[port] & mask))
    {
        armv6m_atomic_or(&stm32l0_gpio_device.enables[port], mask);
        armv6m_atomic_or(&RCC->IOPENR, (RCC_IOPENR_IOPAEN << group));
        RCC->IOPENR;
    }
    
    armv6m_atomic_modify(&GPIO->MODER, mask_2, ((STM32L0_GPIO_MODE_OUTPUT >> STM32L0_GPIO_MODE_SHIFT) << index_2));
}

__attribute__((optimize("O3"))) void stm32l0_gpio_pin_alternate(uint32_t pin)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, index, port, mask, index_2, mask_2;

    group = (pin >> 4) & 7;
    index = (pin >> 0) & 15;
    port = ((group >= STM32L0_GPIO_PORT_COUNT) ? (STM32L0_GPIO_PORT_COUNT -1) : group);

    mask = (1 << index);
    index_2 = (index * 2);
    mask_2 = (3 << index_2);
    
    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    if (!(stm32l0_gpio_device.enables[port] & mask))
    {
        armv6m_atomic_or(&stm32l0_gpio_device.enables[port], mask);
        armv6m_atomic_or(&RCC->IOPENR, (RCC_IOPENR_IOPAEN << group));
        RCC->IOPENR;
    }
    
    armv6m_atomic_modify(&GPIO->MODER, mask_2, ((STM32L0_GPIO_MODE_ALTERNATE >> STM32L0_GPIO_MODE_SHIFT) << index_2));
}

__attribute__((optimize("O3"))) void stm32l0_gpio_pin_analog(uint32_t pin)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, index, port, mask, index_2, mask_2;

    group = (pin >> 4) & 7;
    index = (pin >> 0) & 15;
    port = ((group >= STM32L0_GPIO_PORT_COUNT) ? (STM32L0_GPIO_PORT_COUNT -1) : group);

    mask = (1 << index);
    index_2 = (index * 2);
    mask_2 = (3 << index_2);
    
    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    if (!(stm32l0_gpio_device.enables[port] & mask))
    {
        armv6m_atomic_or(&stm32l0_gpio_device.enables[port], mask);
        armv6m_atomic_or(&RCC->IOPENR, (RCC_IOPENR_IOPAEN << group));
        RCC->IOPENR;
    }

    armv6m_atomic_modify(&GPIO->MODER, mask_2, ((STM32L0_GPIO_MODE_ANALOG >> STM32L0_GPIO_MODE_SHIFT) << index_2));

    armv6m_atomic_and(&stm32l0_gpio_device.enables[port], ~mask);
    armv6m_atomic_andz(&RCC->IOPENR, ~(RCC_IOPENR_IOPAEN << group), &stm32l0_gpio_device.enables[port], ~0);
}
