/*
 * Copyright (c) 2017-2019 Thomas Roell.  All rights reserved.
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
    uint8_t                channel;
    uint16_t               size;
    stm32l0_dma_callback_t callback;
    void*                  context;
} stm32l0_dma_t;

typedef struct _stm32l0_dma_device_t {
    stm32l0_dma_t          channels[7];
    uint8_t                priority_1;
    uint8_t                priority_2_3;
    uint8_t                priority_4_5_6_7;
    volatile uint16_t      dma;
    volatile uint16_t      lock;
} stm32l0_dma_device_t;

static stm32l0_dma_device_t stm32l0_dma_device;

static void stm32l0_dma_interrupt(stm32l0_dma_t *dma, unsigned int shift)
{
    uint32_t events;

    events = (DMA1->ISR >> shift) & 0x0000000e;

    DMA1->IFCR = (15 << shift);

    if (events)
    {
        (*dma->callback)(dma->context, events);
    }
}

void stm32l0_dma_configure(unsigned int priority_1, unsigned int priority_2_3, unsigned int priority_4_5_6_7)
{
    stm32l0_dma_device.priority_1 = priority_1;
    stm32l0_dma_device.priority_2_3 = priority_2_3;
    stm32l0_dma_device.priority_4_5_6_7 = priority_4_5_6_7;

    NVIC_SetPriority(DMA1_Channel1_IRQn, priority_1);
    NVIC_SetPriority(DMA1_Channel2_3_IRQn, priority_2_3);
    NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, priority_4_5_6_7);

    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
    NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
}

unsigned int stm32l0_dma_priority(unsigned int channel)
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

unsigned int stm32l0_dma_channel(unsigned int channel)
{
    return ((stm32l0_dma_device.dma & (1ul << (channel & 7))) ? stm32l0_dma_device.channels[channel & 7].channel : STM32L0_DMA_CHANNEL_UNDEFINED);
}

bool stm32l0_dma_lock(unsigned int channel)
{
    stm32l0_dma_t *dma = &stm32l0_dma_device.channels[channel & 7];
    uint32_t primask, mask;

    mask = 1ul << (channel & 7);

    primask = __get_PRIMASK();

    __disable_irq();

    if (stm32l0_dma_device.lock & mask)
    {
	__set_PRIMASK(primask);

	return false;
    }

    dma->channel = channel;

    stm32l0_dma_device.lock |= mask;

    __set_PRIMASK(primask);

    return true;
}

void stm32l0_dma_unlock(unsigned int channel)
{
    uint32_t primask, mask;

    mask = 1ul << (channel & 7);

    primask = __get_PRIMASK();

    __disable_irq();

    stm32l0_dma_device.lock &= ~mask;

    __set_PRIMASK(primask);
}

bool stm32l0_dma_enable(unsigned int channel, stm32l0_dma_callback_t callback, void *context)
{
    stm32l0_dma_t *dma = &stm32l0_dma_device.channels[channel & 7];
    unsigned int shift;
    uint32_t primask, mask;

    shift = (channel & 7) << 2;
    mask = 1ul << (channel & 7);

    primask = __get_PRIMASK();

    __disable_irq();

    if (stm32l0_dma_device.dma & mask)
    {
        __set_PRIMASK(primask);

        return false;
    }

    if (stm32l0_dma_device.lock & mask)
    {
	if (dma->channel != channel)
	{
	    __set_PRIMASK(primask);

	    return false;
	}
    }

    if (!stm32l0_dma_device.dma)
    {
	RCC->AHBENR |= RCC_AHBENR_DMAEN;
	RCC->AHBSMENR |= (RCC_AHBSMENR_SRAMSMEN | RCC_AHBSMENR_MIFSMEN);
    }

    stm32l0_dma_device.dma |= mask;

    DMA1_CSELR->CSELR = (DMA1_CSELR->CSELR & ~(15 << shift)) | ((channel >> 4) << shift);

    __set_PRIMASK(primask);

    dma->channel = channel;
    dma->callback = callback;
    dma->context = context;

    return true;
}

void stm32l0_dma_disable(unsigned int channel)
{
    uint32_t primask, mask;

    mask = 1ul << (channel & 7);

    primask = __get_PRIMASK();

    __disable_irq();

    stm32l0_dma_device.dma &= ~mask;
        
    if (!stm32l0_dma_device.dma)
    {
	RCC->AHBSMENR &= ~(RCC_AHBSMENR_SRAMSMEN | RCC_AHBSMENR_MIFSMEN);
	RCC->AHBENR &= ~RCC_AHBENR_DMAEN;
    }

    __set_PRIMASK(primask);
}

void stm32l0_dma_start(unsigned int channel, uint32_t tx_data, uint32_t rx_data, uint16_t xf_count, uint32_t option)
{
    DMA_Channel_TypeDef *DMA = stm32l0_dma_xlate_DMA[channel & 7];
    stm32l0_dma_t *dma = &stm32l0_dma_device.channels[channel & 7];
    unsigned int shift;

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

    dma->size = xf_count;

    DMA->CNDTR = xf_count;
    DMA->CCR = option | DMA_CCR_EN;
}

uint16_t stm32l0_dma_stop(unsigned int channel)
{
    DMA_Channel_TypeDef *DMA = stm32l0_dma_xlate_DMA[channel & 7];
    stm32l0_dma_t *dma = &stm32l0_dma_device.channels[channel & 7];

    DMA->CCR &= ~(DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);

    return dma->size - (DMA->CNDTR & 0xffff);
}

uint16_t stm32l0_dma_count(unsigned int channel)
{
    DMA_Channel_TypeDef *DMA = stm32l0_dma_xlate_DMA[channel & 7];
    stm32l0_dma_t *dma = &stm32l0_dma_device.channels[channel & 7];

    return dma->size - (DMA->CNDTR & 0xffff);
}

bool stm32l0_dma_done(unsigned int channel)
{
    unsigned int shift;

    shift = (channel & 7) << 2;

    return !!(DMA1->ISR & (DMA_ISR_TCIF1 << shift));
}

void DMA1_Channel1_IRQHandler(void)
{
    uint32_t dma_isr;

    dma_isr = DMA1->ISR;

    if (((dma_isr >> 0) & 0x0000000e) & DMA1_Channel1->CCR)
    {
        stm32l0_dma_interrupt(&stm32l0_dma_device.channels[0], 0);
    }
}

void DMA1_Channel2_3_IRQHandler(void)
{
    uint32_t dma_isr;

    dma_isr = DMA1->ISR;

    if (((dma_isr >> 4) & 0x0000000e) & DMA1_Channel2->CCR)
    {
        stm32l0_dma_interrupt(&stm32l0_dma_device.channels[1], 4);
    }

    if (((dma_isr >> 8) & 0x0000000e) & DMA1_Channel3->CCR)
    {
        stm32l0_dma_interrupt(&stm32l0_dma_device.channels[2], 8);
    }
}

void DMA1_Channel4_5_6_7_IRQHandler(void)
{
    uint32_t dma_isr;

    dma_isr = DMA1->ISR;

    if (((dma_isr >> 12) & 0x0000000e) & DMA1_Channel4->CCR)
    {
	stm32l0_dma_interrupt(&stm32l0_dma_device.channels[3], 12);
    }

    if (((dma_isr >> 16) & 0x0000000e) & DMA1_Channel5->CCR)
    {
        stm32l0_dma_interrupt(&stm32l0_dma_device.channels[4], 16);
    }

    if (((dma_isr >> 20) & 0x0000000e) & DMA1_Channel6->CCR)
    {
        stm32l0_dma_interrupt(&stm32l0_dma_device.channels[5], 20);
    }

    if (((dma_isr >> 24) & 0x0000000e) & DMA1_Channel7->CCR)
    {
        stm32l0_dma_interrupt(&stm32l0_dma_device.channels[6], 24);
    }
}
