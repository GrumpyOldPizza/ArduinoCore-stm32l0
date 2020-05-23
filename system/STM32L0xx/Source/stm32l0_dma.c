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
#include "stm32l0_dma.h"
#include "stm32l0_system.h"

#define STM32L0_DMA_CHANNEL_LOCKED               0xf000

extern void DMA1_Channel1_IRQHandler(void);
extern void DMA1_Channel2_3_IRQHandler(void);
extern void DMA1_Channel4_5_6_7_IRQHandler(void);

static DMA_Channel_TypeDef * const  stm32l0_dma_xlate_DMA[7] = {
    DMA1_Channel1,
    DMA1_Channel2,
    DMA1_Channel3,
    DMA1_Channel4,
    DMA1_Channel5,
    DMA1_Channel6,
    DMA1_Channel7,
};

typedef struct _stm32l0_dma_t {
    volatile uint16_t      channel;
    uint16_t               size;
    stm32l0_dma_callback_t callback;
    void                   *context;
} stm32l0_dma_t;

typedef struct _stm32l0_dma_device_t {
    stm32l0_dma_t          channels[7];
    uint8_t                priority_1;
    uint8_t                priority_2_3;
    uint8_t                priority_4_5_6_7;
    volatile uint8_t       dma;
    volatile uint8_t       flash;
} stm32l0_dma_device_t;

static stm32l0_dma_device_t stm32l0_dma_device;

static inline void stm32l0_dma_interrupt(stm32l0_dma_t *dma, DMA_Channel_TypeDef *DMA, uint32_t dma_isr, uint32_t shift)
{
    uint32_t events;
    
    events = (dma_isr >> shift) & 0x0000000e;

    if (events & DMA->CCR)
    {
        DMA1->IFCR = (15 << shift);

        (*dma->callback)(dma->context, events);
    }
}
static inline void stm32l0_dma_track(uint16_t channel, uint32_t address)
{
    if (address < 0x10000000)
    {
        __armv6m_atomic_orb(&stm32l0_dma_device.flash, (channel & 7));
    }
}

static inline void stm32l0_dma_untrack(uint16_t channel, uint32_t address)
{
    if (address < 0x10000000)
    {
        __armv6m_atomic_andb(&stm32l0_dma_device.flash, ~(channel & 7));
    }
}

void __stm32l0_dma_initialize(void)
{
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
    NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
}

__attribute__((optimize("O3"))) void __stm32l0_dma_sleep_enter(void)
{
    if (!stm32l0_dma_device.flash)
    {
        FLASH->ACR |= FLASH_ACR_SLEEP_PD;
    }
}

__attribute__((optimize("O3"))) void __stm32l0_dma_sleep_leave(void)
{
    if (!stm32l0_dma_device.flash)
    {
        FLASH->ACR &= ~FLASH_ACR_SLEEP_PD;
    }
}

void stm32l0_dma_configure(uint8_t priority_1, uint8_t priority_2_3, uint8_t priority_4_5_6_7)
{
    stm32l0_dma_device.priority_1 = priority_1;
    stm32l0_dma_device.priority_2_3 = priority_2_3;
    stm32l0_dma_device.priority_4_5_6_7 = priority_4_5_6_7;

    NVIC_SetPriority(DMA1_Channel1_IRQn, priority_1);
    NVIC_SetPriority(DMA1_Channel2_3_IRQn, priority_2_3);
    NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, priority_4_5_6_7);
}

uint8_t stm32l0_dma_priority(uint16_t channel)
{
    if ((channel & 7) == 0)
    {
        return stm32l0_dma_device.priority_1;
    }
    else if ((channel & 7) <= 2)
    {
        return stm32l0_dma_device.priority_2_3;
    }
    else
    {
        return stm32l0_dma_device.priority_4_5_6_7;
    }
}

bool stm32l0_dma_channel(uint16_t channel)
{
    uint32_t index, mask;

    index = channel & 7;
    mask = 1ul << index;

    if (channel == STM32L0_DMA_CHANNEL_NONE)
    {
        return false;
    }
    
    if (!(stm32l0_dma_device.dma & mask))
    {
        return false;
    }

    return ((stm32l0_dma_device.channels[index].channel & STM32L0_DMA_CHANNEL_MASK) == channel);
}

bool stm32l0_dma_lock(uint16_t channel)
{
    stm32l0_dma_t *dma = &stm32l0_dma_device.channels[channel & 7];

    return (armv6m_atomic_cash(&dma->channel, STM32L0_DMA_CHANNEL_NONE, (STM32L0_DMA_CHANNEL_LOCKED | channel)) == STM32L0_DMA_CHANNEL_NONE);
}

void stm32l0_dma_unlock(uint16_t channel)
{
    stm32l0_dma_t *dma = &stm32l0_dma_device.channels[channel & 7];

    dma->channel = STM32L0_DMA_CHANNEL_NONE;
}

bool stm32l0_dma_enable(uint16_t channel, stm32l0_dma_callback_t callback, void *context)
{
    stm32l0_dma_t *dma = &stm32l0_dma_device.channels[channel & 7];
    uint32_t shift, mask, o_channel;

    o_channel = armv6m_atomic_cash(&dma->channel, STM32L0_DMA_CHANNEL_NONE, channel);

    if ((o_channel != STM32L0_DMA_CHANNEL_NONE) && (o_channel != (STM32L0_DMA_CHANNEL_LOCKED | channel)))
    {
        return false;
    }
    
    shift = (channel & 7) << 2;
    mask = 1ul << (channel & 7);

    armv6m_atomic_orb(&stm32l0_dma_device.dma, mask);
    armv6m_atomic_or(&RCC->AHBENR, RCC_AHBENR_DMAEN);
    armv6m_atomic_or(&RCC->AHBSMENR, (RCC_AHBSMENR_SRAMSMEN | RCC_AHBSMENR_MIFSMEN));

    armv6m_atomic_modify(&DMA1_CSELR->CSELR, (15 << shift), ((channel >> 4) << shift));
    
    dma->channel = channel;
    dma->callback = callback;
    dma->context = context;

    return true;
}

void stm32l0_dma_disable(uint16_t channel)
{
    stm32l0_dma_t *dma = &stm32l0_dma_device.channels[channel & 7];
    uint32_t mask;

    mask = 1ul << (channel & 7);

    armv6m_atomic_andb(&stm32l0_dma_device.dma, ~mask);
    armv6m_atomic_andzb(&RCC->AHBENR, ~RCC_AHBENR_DMAEN, &stm32l0_dma_device.dma);
    armv6m_atomic_andzb(&RCC->AHBSMENR, ~(RCC_AHBSMENR_SRAMSMEN | RCC_AHBSMENR_MIFSMEN), &stm32l0_dma_device.dma);

    if (!(dma->channel & STM32L0_DMA_CHANNEL_LOCKED))
    {
        dma->channel = STM32L0_DMA_CHANNEL_NONE;
    }
}

__attribute__((optimize("O3"))) void stm32l0_dma_start(uint16_t channel, uint32_t tx_data, uint32_t rx_data, uint16_t xf_count, uint32_t option)
{
    DMA_Channel_TypeDef *DMA = stm32l0_dma_xlate_DMA[channel & 7];
    stm32l0_dma_t *dma = &stm32l0_dma_device.channels[channel & 7];
    uint32_t shift;

    shift = (channel & 7) << 2;

    DMA1->IFCR = (15 << shift);

    if (option & STM32L0_DMA_OPTION_MEMORY_TO_PERIPHERAL)
    {
        DMA->CMAR = rx_data;
        DMA->CPAR = tx_data;
    }
    else
    {
        DMA->CMAR = tx_data;
        DMA->CPAR = rx_data;
    }

    stm32l0_dma_track(channel, DMA->CMAR);

    dma->size = xf_count;

    DMA->CNDTR = xf_count;
    DMA->CCR = option | DMA_CCR_EN;
}

__attribute__((optimize("O3"))) uint16_t stm32l0_dma_stop(uint16_t channel)
{
    DMA_Channel_TypeDef *DMA = stm32l0_dma_xlate_DMA[channel & 7];
    stm32l0_dma_t *dma = &stm32l0_dma_device.channels[channel & 7];

    stm32l0_dma_untrack(channel, DMA->CMAR);

    DMA->CCR &= ~(DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);

    return dma->size - (DMA->CNDTR & 0xffff);
}

__attribute__((optimize("O3"))) uint16_t stm32l0_dma_count(uint16_t channel)
{
    DMA_Channel_TypeDef *DMA = stm32l0_dma_xlate_DMA[channel & 7];
    stm32l0_dma_t *dma = &stm32l0_dma_device.channels[channel & 7];

    return dma->size - (DMA->CNDTR & 0xffff);
}

__attribute__((optimize("O3"))) bool stm32l0_dma_done(uint16_t channel)
{
    uint32_t shift;

    shift = (channel & 7) << 2;

    return !!(DMA1->ISR & (DMA_ISR_TCIF1 << shift));
}

__attribute__((optimize("O3"))) void DMA1_Channel1_IRQHandler(void)
{
    uint32_t dma_isr;

    dma_isr = DMA1->ISR;

    stm32l0_dma_interrupt(&stm32l0_dma_device.channels[0], DMA1_Channel1, dma_isr, 0);
}

__attribute__((optimize("O3"))) void DMA1_Channel2_3_IRQHandler(void)
{
    uint32_t dma_isr;

    dma_isr = DMA1->ISR;

    stm32l0_dma_interrupt(&stm32l0_dma_device.channels[1], DMA1_Channel2, dma_isr, 4);
    stm32l0_dma_interrupt(&stm32l0_dma_device.channels[2], DMA1_Channel3, dma_isr, 8);
}

__attribute__((optimize("O3"))) void DMA1_Channel4_5_6_7_IRQHandler(void)
{
    uint32_t dma_isr;

    dma_isr = DMA1->ISR;

    stm32l0_dma_interrupt(&stm32l0_dma_device.channels[3], DMA1_Channel4, dma_isr, 12);
    stm32l0_dma_interrupt(&stm32l0_dma_device.channels[4], DMA1_Channel5, dma_isr, 16);
    stm32l0_dma_interrupt(&stm32l0_dma_device.channels[5], DMA1_Channel6, dma_isr, 20);
    stm32l0_dma_interrupt(&stm32l0_dma_device.channels[6], DMA1_Channel7, dma_isr, 24);
}
