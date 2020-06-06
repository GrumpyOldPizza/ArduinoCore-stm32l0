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

#include "stm32l0_dac.h"
#include "stm32l0_system.h"

typedef struct _stm32l0_dac_device_t {
    volatile uint32_t           channels;
} stm32l0_dac_device_t;

static stm32l0_dac_device_t stm32l0_dac_device;

bool stm32l0_dac_enable(uint32_t channels)
{
    if (!channels)
    {
        return false;
    }

    armv6m_atomic_or(&stm32l0_dac_device.channels, channels);

    armv6m_atomic_or(&RCC->APB1ENR, RCC_APB1ENR_DACEN);
    
    if (channels & STM32L0_DAC_CHANNEL_1)
    {
        armv6m_atomic_or(&DAC->CR, DAC_CR_EN1);
    }

#if defined(STM32L072xx) || defined(STM32L082xx)
    if (channels & STM32L0_DAC_CHANNEL_2)
    {
        armv6m_atomic_or(&DAC->CR, DAC_CR_EN2);
    }
#endif /* STM32L072xx || STM32L082xx */

    return true;
}

bool stm32l0_dac_disable(uint32_t channels)
{
    if (!channels)
    {
        return false;
    }

    armv6m_atomic_and(&stm32l0_dac_device.channels, ~channels);

    armv6m_atomic_andz(&DAC->CR, ~DAC_CR_EN1, &stm32l0_dac_device.channels, STM32L0_DAC_CHANNEL_1);
#if defined(STM32L072xx) || defined(STM32L082xx)
    armv6m_atomic_andz(&DAC->CR, ~DAC_CR_EN2, &stm32l0_dac_device.channels, STM32L0_DAC_CHANNEL_2);
    armv6m_atomic_andz(&RCC->APB1ENR, ~RCC_APB1ENR_DACEN, &stm32l0_dac_device.channels, (STM32L0_DAC_CHANNEL_1 | STM32L0_DAC_CHANNEL_2));
#else /* STM32L072xx || STM32L082xx */
    armv6m_atomic_andz(&RCC->APB1ENR, ~RCC_APB1ENR_DACEN, &stm32l0_dac_device.channels, STM32L0_DAC_CHANNEL_1);
#endif /* STM32L072xx || STM32L082xx */


    return true;
}

void stm32l0_dac_write(uint32_t channels, uint32_t output)
{
    if (stm32l0_dac_device.channels & channels)
    {
        if (channels & STM32L0_DAC_CHANNEL_1)
        {
            DAC->DHR12R1 = output & DAC_DHR12R1_DACC1DHR;
        }
        
#if defined(STM32L072xx) || defined(STM32L082xx)
        if (channels & STM32L0_DAC_CHANNEL_2)
        {
            DAC->DHR12R2 = output & DAC_DHR12R2_DACC2DHR;
        }
#endif /* STM32L072xx || STM32L082xx */
    }
}
