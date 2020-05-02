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

#include "stm32l0_dac.h"
#include "stm32l0_dma.h"
#include "stm32l0_system.h"

#define STM32L0_DAC_DMA_OPTION_TRANSMIT_8         \
    (STM32L0_DMA_OPTION_EVENT_TRANSFER_DONE |     \
     STM32L0_DMA_OPTION_MEMORY_TO_PERIPHERAL |    \
     STM32L0_DMA_OPTION_PERIPHERAL_DATA_SIZE_32 | \
     STM32L0_DMA_OPTION_MEMORY_DATA_SIZE_8 |      \
     STM32L0_DMA_OPTION_MEMORY_DATA_INCREMENT |   \
     STM32L0_DMA_OPTION_PRIORITY_LOW)

#define STM32L0_DAC_DMA_OPTION_TRANSMIT_16        \
    (STM32L0_DMA_OPTION_EVENT_TRANSFER_DONE |     \
     STM32L0_DMA_OPTION_MEMORY_TO_PERIPHERAL |    \
     STM32L0_DMA_OPTION_PERIPHERAL_DATA_SIZE_32 | \
     STM32L0_DMA_OPTION_MEMORY_DATA_SIZE_16 |     \
     STM32L0_DMA_OPTION_MEMORY_DATA_INCREMENT |   \
     STM32L0_DMA_OPTION_PRIORITY_LOW)

#define STM32L0_DAC_DMA_OPTION_TRANSMIT_32        \
    (STM32L0_DMA_OPTION_EVENT_TRANSFER_DONE |     \
     STM32L0_DMA_OPTION_MEMORY_TO_PERIPHERAL |    \
     STM32L0_DMA_OPTION_PERIPHERAL_DATA_SIZE_32 | \
     STM32L0_DMA_OPTION_MEMORY_DATA_SIZE_32 |     \
     STM32L0_DMA_OPTION_MEMORY_DATA_INCREMENT |   \
     STM32L0_DMA_OPTION_PRIORITY_LOW)

static volatile uint32_t * const stm32l0_dac_xlate_address[] = {
    &DAC->DHR12R1,
    &DAC->DHR12L1,
    &DAC->DHR8R1,
    0,
#if defined(STM32L072xx) || defined(STM32L082xx)
    &DAC->DHR12RD,
    &DAC->DHR12LD,
    &DAC->DHR8RD,
    0,
#endif /* STM32L072xx || STM32L082xx */
};

#define STM32L0_DAC_STATE_NONE    0
#define STM32L0_DAC_STATE_READY   1
#define STM32L0_DAC_STATE_CONVERT 2
#define STM32L0_DAC_STATE_DONE    3

typedef struct _stm32l0_dac_device_t {
    volatile uint8_t            state;
    uint8_t                     channels;
    uint16_t                    control;
    stm32l0_dac_done_callback_t xf_callback;
    void                        *xf_context;
} stm32l0_dac_device_t;

static stm32l0_dac_device_t stm32l0_dac_device;

bool stm32l0_dac_enable(uint32_t channels)
{
    uint32_t primask;

    if (!channels)
    {
        return false;
    }

    primask = __get_PRIMASK();
        
    __disable_irq();
    
    if (!stm32l0_dac_device.channels)
    {
        RCC->APB1ENR |= RCC_APB1ENR_DACEN;
        RCC->APB1ENR;

        stm32l0_dac_device.state = STM32L0_DAC_STATE_READY;
    }

    stm32l0_dac_device.channels |= channels;

    if (channels & STM32L0_DAC_CHANNEL_1)
    {
        DAC->CR |= DAC_CR_EN1;
    }

#if defined(STM32L072xx) || defined(STM32L082xx)
    if (channels & STM32L0_DAC_CHANNEL_2)
    {
        DAC->CR |= DAC_CR_EN2;
    }
#endif /* STM32L072xx || STM32L082xx */

    __set_PRIMASK(primask);
    
    return true;
}

bool stm32l0_dac_disable(uint32_t channels)
{
    uint32_t primask;

    if (!channels)
    {
        return false;
    }

    primask = __get_PRIMASK();
        
    __disable_irq();
    
    if (channels & STM32L0_DAC_CHANNEL_1)
    {
        DAC->CR &= ~DAC_CR_EN1;
    }

#if defined(STM32L072xx) || defined(STM32L082xx)
    if (channels & STM32L0_DAC_CHANNEL_2)
    {
        DAC->CR &= ~DAC_CR_EN2;
    }
#endif /* STM32L072xx || STM32L082xx */

    stm32l0_dac_device.channels &= ~channels;

    if (!stm32l0_dac_device.channels)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_DACEN;

        stm32l0_dac_device.state = STM32L0_DAC_STATE_NONE;
    }

    __set_PRIMASK(primask);

    return true;
}

void stm32l0_dac_write(uint32_t channels, uint32_t output)
{
    if (channels & STM32L0_DAC_CHANNEL_1)
    {
        DAC->DHR12R1 = output;
    }

#if defined(STM32L072xx) || defined(STM32L082xx)
    if (channels & STM32L0_DAC_CHANNEL_2)
    {
        DAC->DHR12R2 = output;
    }
#endif /* STM32L072xx || STM32L082xx */
}

bool stm32l0_dac_convert(const void *data, uint32_t count, uint32_t control, stm32l0_dac_done_callback_t callback, void *context)
{
    uint32_t option;

    if ((stm32l0_dac_device.state != STM32L0_DAC_STATE_READY) && (stm32l0_dac_device.state != STM32L0_DAC_STATE_DONE))
    {
        return false;
    }

    if ((stm32l0_dac_device.state == STM32L0_DAC_STATE_READY) || (stm32l0_dac_device.control != control))
    {
        if (stm32l0_dac_device.state == STM32L0_DAC_STATE_READY)
        {
            if (!stm32l0_dma_enable(STM32L0_DMA_CHANNEL_DMA1_CH2_DAC1, (stm32l0_dma_callback_t)stm32l0_dac_cancel, NULL))
            {
                return false;
            }

            stm32l0_system_lock(STM32L0_SYSTEM_LOCK_STOP);
        }
        else
        {
#if defined(STM32L072xx) || defined(STM32L082xx)
            DAC->CR &= ~(DAC_CR_DMAEN1 | DAC_CR_TEN1 | DAC_CR_TSEL1 | DAC_CR_TEN2 | DAC_CR_TSEL2);
#else /* STM32L072xx || STM32L082xx */
            DAC->CR &= ~(DAC_CR_DMAEN1 | DAC_CR_TEN1 | DAC_CR_TSEL1);
#endif /* STM32L072xx || STM32L082xx */
        }

#if defined(STM32L072xx) || defined(STM32L082xx)
        if (control & STM32L0_DAC_CONTROL_STEREO)
        {
            DAC->CR |= (DAC_CR_DMAEN1 | DAC_CR_TEN1 | DAC_CR_TEN2 |
                        (((control & STM32L0_DAC_CONTROL_TRIG_MASK) >> STM32L0_DAC_CONTROL_TRIG_SHIFT) << DAC_CR_TSEL1_Pos) |
                        (((control & STM32L0_DAC_CONTROL_TRIG_MASK) >> STM32L0_DAC_CONTROL_TRIG_SHIFT) << DAC_CR_TSEL2_Pos));
        }
        else
#endif /* STM32L072xx || STM32L082xx */
        {
            DAC->CR |= (DAC_CR_DMAEN1 | DAC_CR_TEN1 |
                        (((control & STM32L0_DAC_CONTROL_TRIG_MASK) >> STM32L0_DAC_CONTROL_TRIG_SHIFT) << DAC_CR_TSEL1_Pos));
        }
        
        stm32l0_dac_device.control = control;
    }

#if defined(STM32L072xx) || defined(STM32L082xx)
    if (control & STM32L0_DAC_CONTROL_STEREO)
    {
        if (control & STM32L0_DAC_CONTROL_BYTE_PACKED)
        {
            count = count / 2;
            option = STM32L0_DAC_DMA_OPTION_TRANSMIT_16;
        }
        else
        {
            count = count / 4;
            option = STM32L0_DAC_DMA_OPTION_TRANSMIT_32;
        }
    }
    else
#endif /* STM32L072xx || STM32L082xx */
    {
        if (control & STM32L0_DAC_CONTROL_BYTE_PACKED)
        {
            option = STM32L0_DAC_DMA_OPTION_TRANSMIT_8;
        }
        else
        {
            count = count / 2;
            option = STM32L0_DAC_DMA_OPTION_TRANSMIT_16;
        }
    }

    stm32l0_dac_device.xf_callback = callback;
    stm32l0_dac_device.xf_context = context;
    stm32l0_dac_device.state = STM32L0_DAC_STATE_CONVERT;

    stm32l0_dma_start(STM32L0_DMA_CHANNEL_DMA1_CH2_DAC1, (uint32_t)stm32l0_dac_xlate_address[control], (uint32_t)data, count, option);

    return true;
}

void stm32l0_dac_cancel(void)
{
    uint32_t count;

    if (stm32l0_dac_device.state == STM32L0_DAC_STATE_CONVERT)
    {
        count = stm32l0_dma_stop(STM32L0_DMA_CHANNEL_DMA1_CH2_DAC1);

        stm32l0_dac_device.state = STM32L0_DAC_STATE_DONE;

#if defined(STM32L072xx) || defined(STM32L082xx)
        if (stm32l0_dac_device.control & STM32L0_DAC_CONTROL_STEREO)
        {
            if (stm32l0_dac_device.control & STM32L0_DAC_CONTROL_BYTE_PACKED)
            {
                count = count * 2;
            }
            else
            {
                count = count * 4;
            }
        }
        else
#endif /* STM32L072xx || STM32L082xx */
        {
            if (!(stm32l0_dac_device.control & STM32L0_DAC_CONTROL_BYTE_PACKED))
            {
                count = count * 2;
            }
        }

        (*stm32l0_dac_device.xf_callback)(stm32l0_dac_device.xf_context, count);

        if (stm32l0_dac_device.state == STM32L0_DAC_STATE_DONE)
        {
#if defined(STM32L072xx) || defined(STM32L082xx)
            DAC->CR &= ~(DAC_CR_DMAEN1 | DAC_CR_TEN1 | DAC_CR_TSEL1 | DAC_CR_TEN2 | DAC_CR_TSEL2);
#else /* STM32L072xx || STM32L082xx */
            DAC->CR &= ~(DAC_CR_DMAEN1 | DAC_CR_TEN1 | DAC_CR_TSEL1);
#endif /* STM32L072xx || STM32L082xx */

            stm32l0_dma_disable(STM32L0_DMA_CHANNEL_DMA1_CH2_DAC1);

            stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_STOP);

            stm32l0_dac_device.state = STM32L0_DAC_STATE_READY;
        }
    }
}
