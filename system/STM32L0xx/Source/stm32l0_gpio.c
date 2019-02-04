/*
 * Copyright (c) 2014-2019 Thomas Roell.  All rights reserved.
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
    uint16_t                enables[STM32L0_GPIO_PORT_COUNT];
    uint32_t                mode[STM32L0_GPIO_PORT_COUNT];
    uint32_t                pupd[STM32L0_GPIO_PORT_COUNT];
} stm32l0_gpio_device_t;

static stm32l0_gpio_device_t stm32l0_gpio_device;

void __stm32l0_gpio_initialize(void)
{
    /* No JTAG GPIO overrides needed.
     */

    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA4, STM32L0_GPIO_MODE_ANALOG);
}

void stm32l0_gpio_swd_enable(void)
{
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA13_SWDIO, (STM32L0_GPIO_PUPD_PULLUP   | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_ALTERNATE));
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA14_SWCLK, (STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_ALTERNATE));
}

void stm32l0_gpio_swd_disable(void)
{
    uint32_t primask, rcc_iopenr;
    bool swdio, swclk;

    primask = __get_PRIMASK();

    __disable_irq();

    rcc_iopenr = RCC->IOPENR;

    RCC->IOPENR |= RCC_IOPENR_IOPAEN;
    RCC->IOPENR;
    
    swdio = (((GPIOA->MODER >> (13 * 2)) & 3) == 2) && (((GPIOA->AFR[1] >> ((13 & 7) * 4)) & 15) == 0);
    swclk = (((GPIOA->MODER >> (14 * 2)) & 3) == 2) && (((GPIOA->AFR[1] >> ((14 & 7) * 4)) & 15) == 0);

    RCC->IOPENR = rcc_iopenr;

    if (swdio)
    {
        stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA13, STM32L0_GPIO_MODE_ANALOG);
    }
    
    if (swclk)
    {
        stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA14, STM32L0_GPIO_MODE_ANALOG);
    }

    __set_PRIMASK(primask);
}

void stm32l0_gpio_pin_configure(unsigned int pin, unsigned int mode)
{
    GPIO_TypeDef *GPIO;
    uint32_t primask, group, index, port, afsel;

    afsel = (pin >> 8) & 15;
    group = (pin >> 4) & 7;
    index = (pin >> 0) & 15;
    port  = ((group >= STM32L0_GPIO_PORT_COUNT) ? (STM32L0_GPIO_PORT_COUNT -1) : group);

    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    primask = __get_PRIMASK();

    __disable_irq();

    RCC->IOPENR |= (RCC_IOPENR_IOPAEN << group);
    RCC->IOPENR;

    /* If the mode is ANALOG, set MODER first */
    if (((mode & STM32L0_GPIO_MODE_MASK) >> STM32L0_GPIO_MODE_SHIFT) == STM32L0_GPIO_MODE_ANALOG)
    {
        GPIO->MODER |= (0x00000003 << (index << 1));

        stm32l0_gpio_device.enables[port] &= ~(1 << index);

        if (!stm32l0_gpio_device.enables[port])
        {
            RCC->IOPENR &= ~(RCC_IOPENR_IOPAEN << group);
        }
    }
    else
    {
        /* Set AFRL/AFRH */
        GPIO->AFR[index >> 3] = (GPIO->AFR[index >> 3] & ~(0x0000000f << ((index & 7) << 2))) | (afsel << ((index & 7) << 2));
        
        /* Set OPSPEEDR */
        GPIO->OSPEEDR = (GPIO->OSPEEDR & ~(0x00000003 << (index << 1))) | (((mode & STM32L0_GPIO_OSPEED_MASK) >> STM32L0_GPIO_OSPEED_SHIFT) << (index << 1));
        
        /* Set OPTYPER */
        GPIO->OTYPER = (GPIO->OTYPER & ~(0x00000001 << index)) | (((mode & STM32L0_GPIO_OTYPE_MASK) >> STM32L0_GPIO_OTYPE_SHIFT) << index);
        
        /* If the mode is OUTPUT, or OUTPUT OPENDRAIN with a ODR of 0. then first switch MODER and then PUPDR
         * to avoid spurious edges. N.b. ALTERNATE is assumed to be INPUT before the peripheral drives it.
         */ 
        if (((mode & STM32L0_GPIO_MODE_MASK) == STM32L0_GPIO_MODE_OUTPUT) &&
            (((mode & STM32L0_GPIO_OTYPE_MASK) != STM32L0_GPIO_OTYPE_OPENDRAIN) || !(GPIO->ODR & (0x00000001 << index))))
        {
            /* Set MODE */
            GPIO->MODER = (GPIO->MODER & ~(0x00000003 << (index << 1))) | (((mode & STM32L0_GPIO_MODE_MASK) >> STM32L0_GPIO_MODE_SHIFT) << (index << 1));
            
            /* Set PUPD */
            GPIO->PUPDR = (GPIO->PUPDR & ~(0x00000003 << (index << 1))) | (((mode & STM32L0_GPIO_PUPD_MASK) >> STM32L0_GPIO_PUPD_SHIFT) << (index << 1));
        }
        else
        {
            /* Set PUPD */
            GPIO->PUPDR = (GPIO->PUPDR & ~(0x00000003 << (index << 1))) | (((mode & STM32L0_GPIO_PUPD_MASK) >> STM32L0_GPIO_PUPD_SHIFT) << (index << 1));
            
            /* Set MODE */
            GPIO->MODER = (GPIO->MODER & ~(0x00000003 << (index << 1))) | (((mode & STM32L0_GPIO_MODE_MASK) >> STM32L0_GPIO_MODE_SHIFT) << (index << 1));
        }

        stm32l0_gpio_device.enables[port] |= (1 << index);
    }

    stm32l0_gpio_device.mode[port] &= ~(0x00000003 << (index << 1));
    stm32l0_gpio_device.pupd[port] &= ~(0x00000003 << (index << 1));

    if ((mode & STM32L0_GPIO_PARK_MASK) != STM32L0_GPIO_PARK_NONE)
    {
        if ((mode & STM32L0_GPIO_PARK_MASK) == STM32L0_GPIO_PARK_HIZ)
        {
            /* 11 means STM32L0_GPIO_MODE_ANLOG (MSB is MASK, LSB is DATA) 
             */
            stm32l0_gpio_device.mode[port] |= (0x00000003 << (index << 1));
        }
        else
        {
            /* 10 means STM32L0_GPIO_MODE_INPUT (MSB is MASK, LSB is DATA).
             * PUPD mask is derived from either bit being non-zero.
             */
            stm32l0_gpio_device.mode[port] |= (0x00000002 << (index << 1));
            stm32l0_gpio_device.pupd[port] |= (((mode & STM32L0_GPIO_PARK_MASK) >> STM32L0_GPIO_PARK_SHIFT) << (index << 1));
        }
    }
        
    __set_PRIMASK(primask);
}

void stm32l0_gpio_pin_input(unsigned int pin)
{
    GPIO_TypeDef *GPIO;
    uint32_t primask, group, index, port;

    group = (pin >> 4) & 7;
    index = (pin >> 0) & 15;
    port  = ((group >= STM32L0_GPIO_PORT_COUNT) ? (STM32L0_GPIO_PORT_COUNT -1) : group);

    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    primask = __get_PRIMASK();

    __disable_irq();

    RCC->IOPENR |= (RCC_IOPENR_IOPAEN << group);
    RCC->IOPENR;

    GPIO->MODER = (GPIO->MODER & ~(0x00000003 << (index << 1))) | ((STM32L0_GPIO_MODE_INPUT >> STM32L0_GPIO_MODE_SHIFT) << (index << 1));

    stm32l0_gpio_device.enables[port] |= (1 << index);

    __set_PRIMASK(primask);
}

void stm32l0_gpio_pin_output(unsigned int pin)
{
    GPIO_TypeDef *GPIO;
    uint32_t primask, group, index, port;

    group = (pin >> 4) & 7;
    index = (pin >> 0) & 15;
    port  = ((group >= STM32L0_GPIO_PORT_COUNT) ? (STM32L0_GPIO_PORT_COUNT -1) : group);

    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    primask = __get_PRIMASK();

    __disable_irq();

    RCC->IOPENR |= (RCC_IOPENR_IOPAEN << group);
    RCC->IOPENR;

    GPIO->MODER = (GPIO->MODER & ~(0x00000003 << (index << 1))) | ((STM32L0_GPIO_MODE_OUTPUT >> STM32L0_GPIO_MODE_SHIFT) << (index << 1));

    stm32l0_gpio_device.enables[port] |= (1 << index);

    __set_PRIMASK(primask);
}

void stm32l0_gpio_pin_alternate(unsigned int pin)
{
    GPIO_TypeDef *GPIO;
    uint32_t primask, group, index, port;

    group = (pin >> 4) & 7;
    index = (pin >> 0) & 15;
    port  = ((group >= STM32L0_GPIO_PORT_COUNT) ? (STM32L0_GPIO_PORT_COUNT -1) : group);

    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    primask = __get_PRIMASK();

    __disable_irq();

    RCC->IOPENR |= (RCC_IOPENR_IOPAEN << group);
    RCC->IOPENR;

    GPIO->MODER = (GPIO->MODER & ~(0x00000003 << (index << 1))) | ((STM32L0_GPIO_MODE_ALTERNATE >> STM32L0_GPIO_MODE_SHIFT) << (index << 1));

    stm32l0_gpio_device.enables[port] |= (1 << index);

    __set_PRIMASK(primask);
}

void stm32l0_gpio_pin_analog(unsigned int pin)
{
    GPIO_TypeDef *GPIO;
    uint32_t primask, group, index, port;

    group = (pin >> 4) & 7;
    index = (pin >> 0) & 15;
    port  = ((group >= STM32L0_GPIO_PORT_COUNT) ? (STM32L0_GPIO_PORT_COUNT -1) : group);

    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    primask = __get_PRIMASK();

    __disable_irq();

    RCC->IOPENR |= (RCC_IOPENR_IOPAEN << group);
    RCC->IOPENR;

    GPIO->MODER = (GPIO->MODER & ~(0x00000003 << (index << 1))) | ((STM32L0_GPIO_MODE_ANALOG >> STM32L0_GPIO_MODE_SHIFT) << (index << 1));

    stm32l0_gpio_device.enables[port] &= ~(1 << index);

    if (!stm32l0_gpio_device.enables[port])
    {
        RCC->IOPENR &= ~(RCC_IOPENR_IOPAEN << group);
    }

    __set_PRIMASK(primask);
}

void stm32l0_gpio_save(stm32l0_gpio_state_t *state)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, port, mask, data;

    for (port = 0; port < STM32L0_GPIO_PORT_COUNT; port++)
    {
        group = ((port == (STM32L0_GPIO_PORT_COUNT -1)) ? (STM32L0_GPIO_GROUP_COUNT -1) : port);

        GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

        RCC->IOPENR |= (RCC_IOPENR_IOPAEN << group);
        RCC->IOPENR;

        state->mode[port] = GPIO->MODER;
        state->pupd[port] = GPIO->PUPDR;

        /*
         * 00 means to keep the current mode value.
         *
         * 10 means STM32L0_GPIO_MODE_INPUT (MSB is MASK, LSB is DATA, 00).
         *
         * 11 means STM32L0_GPIO_MODE_ANLOG (MSB is MASK, LSB is DATA, 11) 
         */

        mask = stm32l0_gpio_device.mode[port] & 0xaaaaaaaa;
        mask |= (mask >> 1);

        data = stm32l0_gpio_device.mode[port] & 0x55555555;
        data |= (data << 1);

        GPIO->MODER = (GPIO->MODER & ~mask) | data;

        /* PUPD mask is derived from either bit being non-zero.
         */
        
        mask = (stm32l0_gpio_device.pupd[port] | (stm32l0_gpio_device.pupd[port] << 1)) & 0xaaaaaaaa;
        mask |= (mask >> 1);

        GPIO->PUPDR = (GPIO->PUPDR & ~mask) | stm32l0_gpio_device.pupd[port];

        if (!stm32l0_gpio_device.enables[port])
        {
            RCC->IOPENR &= ~(RCC_IOPENR_IOPAEN << group);
        }
    }
}

void stm32l0_gpio_restore(stm32l0_gpio_state_t *state)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, port;

    for (port = 0; port < STM32L0_GPIO_PORT_COUNT; port++)
    {
        group = ((port == (STM32L0_GPIO_PORT_COUNT -1)) ? (STM32L0_GPIO_GROUP_COUNT -1) : port);

        GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

        RCC->IOPENR |= (RCC_IOPENR_IOPAEN << group);
        RCC->IOPENR;

        GPIO->MODER = state->mode[port];
        GPIO->PUPDR = state->pupd[port];

        if (!stm32l0_gpio_device.enables[port])
        {
            RCC->IOPENR &= ~(RCC_IOPENR_IOPAEN << group);
        }
    }
}

void __stm32l0_gpio_pin_write(unsigned int pin, unsigned int data)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, index;

    group = (pin & STM32L0_GPIO_PIN_GROUP_MASK) >> STM32L0_GPIO_PIN_GROUP_SHIFT;
    index = (pin & STM32L0_GPIO_PIN_INDEX_MASK) >> STM32L0_GPIO_PIN_INDEX_SHIFT;

    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    if (data) {
        GPIO->BSRR = (1ul << index);
    } else {
        GPIO->BRR = (1ul << index);
    }
}

unsigned int __stm32l0_gpio_pin_read(unsigned int pin)
{
    GPIO_TypeDef *GPIO;
    uint32_t group, index;

    group = (pin & STM32L0_GPIO_PIN_GROUP_MASK) >> STM32L0_GPIO_PIN_GROUP_SHIFT;
    index = (pin & STM32L0_GPIO_PIN_INDEX_MASK) >> STM32L0_GPIO_PIN_INDEX_SHIFT;

    GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

    return ((GPIO->IDR >> index) & 1);
}
