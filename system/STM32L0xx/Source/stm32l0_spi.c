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

#include "stm32l0_gpio.h"
#include "stm32l0_spi.h"
#include "stm32l0_dma.h"
#include "stm32l0_exti.h"
#include "stm32l0_system.h"

typedef struct _stm32l0_spi_device_t {
    stm32l0_spi_t     *instances[STM32L0_SPI_INSTANCE_COUNT];
} stm32l0_spi_device_t;

#define STM32L0_SPI_RX_DMA_OPTION_RECEIVE_8         \
    (STM32L0_DMA_OPTION_PERIPHERAL_TO_MEMORY |      \
     STM32L0_DMA_OPTION_PERIPHERAL_DATA_SIZE_16 |   \
     STM32L0_DMA_OPTION_MEMORY_DATA_SIZE_8 |        \
     STM32L0_DMA_OPTION_MEMORY_DATA_INCREMENT |     \
     STM32L0_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32L0_SPI_TX_DMA_OPTION_RECEIVE_8         \
    (STM32L0_DMA_OPTION_MEMORY_TO_PERIPHERAL |      \
     STM32L0_DMA_OPTION_PERIPHERAL_DATA_SIZE_16 |   \
     STM32L0_DMA_OPTION_MEMORY_DATA_SIZE_8 |        \
     STM32L0_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32L0_SPI_TX_DMA_OPTION_TRANSMIT_8        \
    (STM32L0_DMA_OPTION_MEMORY_TO_PERIPHERAL |      \
     STM32L0_DMA_OPTION_PERIPHERAL_DATA_SIZE_16 |   \
     STM32L0_DMA_OPTION_MEMORY_DATA_SIZE_8 |        \
     STM32L0_DMA_OPTION_MEMORY_DATA_INCREMENT |     \
     STM32L0_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32L0_SPI_RX_DMA_OPTION_TRANSFER_8        \
    (STM32L0_DMA_OPTION_PERIPHERAL_TO_MEMORY |      \
     STM32L0_DMA_OPTION_PERIPHERAL_DATA_SIZE_16 |   \
     STM32L0_DMA_OPTION_MEMORY_DATA_SIZE_8 |        \
     STM32L0_DMA_OPTION_MEMORY_DATA_INCREMENT |     \
     STM32L0_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32L0_SPI_TX_DMA_OPTION_TRANSFER_8        \
    (STM32L0_DMA_OPTION_MEMORY_TO_PERIPHERAL |      \
     STM32L0_DMA_OPTION_PERIPHERAL_DATA_SIZE_16 |   \
     STM32L0_DMA_OPTION_MEMORY_DATA_SIZE_8 |        \
     STM32L0_DMA_OPTION_MEMORY_DATA_INCREMENT |     \
     STM32L0_DMA_OPTION_PRIORITY_MEDIUM)

static stm32l0_spi_device_t stm32l0_spi_device;

bool stm32l0_spi_create(stm32l0_spi_t *spi, const stm32l0_spi_params_t *params)
{
    if (spi->state != STM32L0_SPI_STATE_NONE)
    {
        return false;
    }

    spi->SPI = (params->instance == STM32L0_SPI_INSTANCE_SPI1) ? SPI1 : SPI2;
    spi->instance = params->instance;
    spi->priority = params->priority;
    spi->rx_dma = params->rx_dma;
    spi->tx_dma = params->tx_dma;
    spi->pins = params->pins;

    stm32l0_spi_device.instances[spi->instance] = spi;

    spi->state = STM32L0_SPI_STATE_INIT;

    return true;
}

bool stm32l0_spi_destroy(stm32l0_spi_t *spi)
{
    if (spi->state != STM32L0_SPI_STATE_INIT)
    {
        return false;
    }

    spi->state = STM32L0_SPI_STATE_NONE;

    stm32l0_spi_device.instances[spi->instance] = NULL;

    return true;
}

bool stm32l0_spi_enable(stm32l0_spi_t *spi)
{
    if (spi->state != STM32L0_SPI_STATE_INIT)
    {
        if (spi->state == STM32L0_SPI_STATE_READY)
        {
            spi->lock++;

            return true;
        }
        else
        {
            return false;
        }
    }

    spi->lock = 1;
    spi->clock = 0;
    spi->option = ~0;
    spi->mask = 0;
    spi->callback = NULL;
    spi->context = NULL;
    
    stm32l0_gpio_pin_configure(spi->pins.mosi, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_ALTERNATE));
    stm32l0_gpio_pin_configure(spi->pins.miso, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_ALTERNATE));
    stm32l0_gpio_pin_configure(spi->pins.sck,  (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_ALTERNATE));

    spi->state = STM32L0_SPI_STATE_READY;

    return true;
}

bool stm32l0_spi_disable(stm32l0_spi_t *spi)
{
    if (spi->state != STM32L0_SPI_STATE_READY)
    {
        return false;
    }

    if (spi->lock != 1)
    {
        spi->lock--;
    }
    else
    {
        spi->lock = 0;

        stm32l0_gpio_pin_configure(spi->pins.mosi, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_MODE_ANALOG));
        stm32l0_gpio_pin_configure(spi->pins.miso, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_MODE_ANALOG));
        stm32l0_gpio_pin_configure(spi->pins.sck,  (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_MODE_ANALOG));
        
        stm32l0_spi_device.instances[spi->instance] = NULL;
        
        spi->state = STM32L0_SPI_STATE_INIT;
    }

    return true;
}

bool stm32l0_spi_notify(stm32l0_spi_t *spi, stm32l0_spi_notify_callback_t callback, void *context)
{
    if (spi->state != STM32L0_SPI_STATE_READY)
    {
        return false;
    }

    spi->callback = callback;
    spi->context = context;

    return true;
}

bool stm32l0_spi_block(stm32l0_spi_t *spi, uint16_t pin)
{
    if (spi->state != STM32L0_SPI_STATE_READY)
    {
        return false;
    }

    spi->mask |= (1ul << ((pin & STM32L0_GPIO_PIN_INDEX_MASK) >> STM32L0_GPIO_PIN_INDEX_SHIFT));

    return true;
}

bool stm32l0_spi_unblock(stm32l0_spi_t *spi, uint16_t pin)
{
    if (spi->state != STM32L0_SPI_STATE_READY)
    {
        return false;
    }

    spi->mask &= ~(1ul << ((pin & STM32L0_GPIO_PIN_INDEX_MASK) >> STM32L0_GPIO_PIN_INDEX_SHIFT));

    return true;
}

bool stm32l0_spi_acquire(stm32l0_spi_t *spi, uint32_t clock, uint32_t option)
{
    SPI_TypeDef *SPI = spi->SPI;
    uint32_t pclk, spiclk, div;

    if (spi->state != STM32L0_SPI_STATE_READY)
    {
        return false;
    }

    if (spi->callback)
    {
        (*spi->callback)(spi->context, true);
    }

    if (spi->mask)
    {
        stm32l0_exti_block(spi->mask);
    }

    stm32l0_system_lock(STM32L0_SYSTEM_LOCK_STOP);
    stm32l0_system_lock(STM32L0_SYSTEM_LOCK_CLOCKS);

    stm32l0_system_periph_enable(STM32L0_SYSTEM_PERIPH_SPI1 + spi->instance);

    if (spi->rx_dma != STM32L0_DMA_CHANNEL_NONE)
    {
        stm32l0_dma_enable(spi->rx_dma, NULL, NULL);
    }
    
    if (spi->tx_dma != STM32L0_DMA_CHANNEL_NONE)
    {
        stm32l0_dma_enable(spi->tx_dma, NULL, NULL);
    }

    if (spi->instance == STM32L0_SPI_INSTANCE_SPI1)
    {
        pclk = stm32l0_system_pclk2();
    }
    else
    {
        pclk = stm32l0_system_pclk1();
    }

    if (spi->pclk != pclk)
    {
        spi->pclk = pclk;
        spi->clock = 0;
        spi->option = ~0;
    }

    if ((spi->clock != clock) || ((spi->option ^ option) & (SPI_CR1_CPHA | SPI_CR1_CPOL | SPI_CR1_LSBFIRST)))
    {
        spiclk = spi->pclk / 2;
        div = 0;

        while ((spiclk > clock) && (div < 7))
        {
            spiclk >>= 1;
            div++;
        }
    
        SPI->CR1 &= ~SPI_CR1_SPE;
        SPI->SR = 0;
        SPI->CRCPR = 0x1021;
        SPI->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (option & (STM32L0_SPI_OPTION_MODE_MASK | STM32L0_SPI_OPTION_LSB_FIRST)) | (div << SPI_CR1_BR_Pos);
        SPI->CR2 = 0;

        spi->clock = clock;
    }

    spi->option = option;

    SPI->CR1 |= SPI_CR1_SPE;

    spi->state = STM32L0_SPI_STATE_DATA;

    return true;
}

bool stm32l0_spi_release(stm32l0_spi_t *spi)
{
    SPI_TypeDef *SPI = spi->SPI;

    if (spi->state != STM32L0_SPI_STATE_DATA)
    {
        return false;
    }

    while (SPI->SR & SPI_SR_BSY)
    {
    }

    SPI->CR1 &= ~SPI_CR1_SPE;

    if (spi->rx_dma == stm32l0_dma_channel(spi->rx_dma))
    {
        stm32l0_dma_disable(spi->rx_dma);
    }
    
    if (spi->tx_dma == stm32l0_dma_channel(spi->tx_dma))
    {
        stm32l0_dma_disable(spi->tx_dma);
    }

    stm32l0_system_periph_disable(STM32L0_SYSTEM_PERIPH_SPI1 + spi->instance);

    stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_CLOCKS);
    stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_STOP);

    if (spi->mask)
    {
        stm32l0_exti_unblock(spi->mask);
    }

    if (spi->callback)
    {
        (*spi->callback)(spi->context, false);
    }

    spi->state = STM32L0_SPI_STATE_READY;

    return true;
}

__attribute__((optimize("O3"))) uint8_t stm32l0_spi_data(stm32l0_spi_t *spi, uint8_t data)
{
    SPI_TypeDef *SPI = spi->SPI;

    SPI->DR = data;
        
    while (!(SPI->SR & SPI_SR_RXNE))
    {
    }

    data = SPI->DR;
        
    return data;
}

__attribute__((optimize("O3"))) uint16_t stm32l0_spi_data16(stm32l0_spi_t *spi, uint16_t data)
{
    SPI_TypeDef *SPI = spi->SPI;
    uint8_t rx_temp, tx_temp;

    if (spi->option & STM32L0_SPI_OPTION_LSB_FIRST)
    {
        SPI->DR = ((const uint8_t*)&data)[0];

        tx_temp = ((const uint8_t*)&data)[1];

        __asm__ volatile("": : : "memory");
        
        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }

        rx_temp = SPI->DR;
        SPI->DR = tx_temp;

        __asm__ volatile("": : : "memory");

        ((uint8_t*)&data)[0] = rx_temp;

        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }

        ((uint8_t*)&data)[1] = SPI->DR;
    }
    else
    {
        SPI->DR = ((const uint8_t*)&data)[1];

        tx_temp = ((const uint8_t*)&data)[0];

        __asm__ volatile("": : : "memory");
        
        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }

        rx_temp = SPI->DR;
        SPI->DR = tx_temp;

        __asm__ volatile("": : : "memory");

        ((uint8_t*)&data)[1] = rx_temp;

        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }

        ((uint8_t*)&data)[0] = SPI->DR;
    }

    return data;
}

__attribute__((optimize("O3"))) uint32_t stm32l0_spi_data32(stm32l0_spi_t *spi, uint32_t data)
{
    SPI_TypeDef *SPI = spi->SPI;
    uint8_t rx_temp, tx_temp;

    if (spi->option & STM32L0_SPI_OPTION_LSB_FIRST)
    {
        SPI->DR = ((const uint8_t*)&data)[0];

        tx_temp = ((const uint8_t*)&data)[1];

        __asm__ volatile("": : : "memory");
        
        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }

        rx_temp = SPI->DR;
        SPI->DR = tx_temp;

        __asm__ volatile("": : : "memory");

        ((uint8_t*)&data)[0] = rx_temp;
        tx_temp = ((const uint8_t*)&data)[2];

        __asm__ volatile("": : : "memory");
        
        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }

        rx_temp = SPI->DR;
        SPI->DR = tx_temp;

        __asm__ volatile("": : : "memory");

        ((uint8_t*)&data)[1] = rx_temp;
        tx_temp = ((const uint8_t*)&data)[3];

        __asm__ volatile("": : : "memory");
        
        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }

        rx_temp = SPI->DR;
        SPI->DR = tx_temp;

        __asm__ volatile("": : : "memory");

        ((uint8_t*)&data)[2] = rx_temp;

        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }

        ((uint8_t*)&data)[3] = SPI->DR;
    }
    else
    {
        SPI->DR = ((const uint8_t*)&data)[3];

        tx_temp = ((const uint8_t*)&data)[2];

        __asm__ volatile("": : : "memory");
        
        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }

        rx_temp = SPI->DR;
        SPI->DR = tx_temp;

        __asm__ volatile("": : : "memory");

        ((uint8_t*)&data)[3] = rx_temp;
        tx_temp = ((const uint8_t*)&data)[1];

        __asm__ volatile("": : : "memory");
        
        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }

        rx_temp = SPI->DR;
        SPI->DR = tx_temp;

        __asm__ volatile("": : : "memory");

        ((uint8_t*)&data)[2] = rx_temp;
        tx_temp = ((const uint8_t*)&data)[0];

        __asm__ volatile("": : : "memory");
        
        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }

        rx_temp = SPI->DR;
        SPI->DR = tx_temp;

        __asm__ volatile("": : : "memory");

        ((uint8_t*)&data)[1] = rx_temp;

        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }

        ((uint8_t*)&data)[0] = SPI->DR;
    }

    return data;
}

__attribute__((optimize("O3"))) bool stm32l0_spi_receive(stm32l0_spi_t *spi, uint8_t *rx_data, uint32_t count)
{
    SPI_TypeDef *SPI = spi->SPI;
    uint32_t primask, spi_cr1;
    uint8_t *rx_data_e;
    uint8_t rx_temp, tx_default;

    if (spi->state != STM32L0_SPI_STATE_DATA)
    {
        return false;
    }

    if (!(spi->option & STM32L0_SPI_OPTION_HALFDUPLEX))
    {
        tx_default = 0xff;

        if ((count <= 16) || (spi->rx_dma != stm32l0_dma_channel(spi->rx_dma)))
        {
            if (count == 1)
            {
                SPI->DR = tx_default;
        
                while (!(SPI->SR & SPI_SR_RXNE))
                {
                }
        
                *rx_data = SPI->DR;
            }
            else
            {
                rx_data_e = rx_data + count -1;
        
                SPI->DR = tx_default;
        
                do 
                {
                    __asm__ volatile("": : : "memory");
            
                    while (!(SPI->SR & SPI_SR_RXNE))
                    {
                    }
            
                    rx_temp = SPI->DR;
                    SPI->DR = tx_default;
            
                    __asm__ volatile("": : : "memory");
            
                    *rx_data++ = rx_temp;
                } 
                while (rx_data != rx_data_e);
        
                while (!(SPI->SR & SPI_SR_RXNE))
                {
                }
        
                *rx_data++ = SPI->DR;
            }
        }
        else
        {
            if (spi->tx_dma != stm32l0_dma_channel(spi->tx_dma))
            {
                SPI->CR1 &= ~SPI_CR1_SPE;
                SPI->CR2 = SPI_CR2_RXDMAEN;
                SPI->CR1 |= SPI_CR1_SPE;
                
                stm32l0_dma_start(spi->rx_dma, (uint32_t)rx_data, (uint32_t)&SPI->DR, count, STM32L0_SPI_RX_DMA_OPTION_RECEIVE_8);

                do
                {
                    while (!(SPI->SR & SPI_SR_TXE))
                    {
                    }
                    
                    SPI->DR = tx_default;
                }
                while (--count);

                while (!stm32l0_dma_done(spi->rx_dma))
                {
                }
                
                stm32l0_dma_stop(spi->rx_dma);
            }
            else
            {
                SPI->CR1 &= ~SPI_CR1_SPE;
                SPI->CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
                SPI->CR1 |= SPI_CR1_SPE;
                
                stm32l0_dma_start(spi->rx_dma, (uint32_t)rx_data, (uint32_t)&SPI->DR, count, STM32L0_SPI_RX_DMA_OPTION_RECEIVE_8);
                stm32l0_dma_start(spi->tx_dma, (uint32_t)&SPI->DR, (uint32_t)&tx_default, count, STM32L0_SPI_TX_DMA_OPTION_RECEIVE_8);
                
                while (!stm32l0_dma_done(spi->rx_dma))
                {
                }
                
                stm32l0_dma_stop(spi->rx_dma);
                stm32l0_dma_stop(spi->tx_dma);
            }
                
            while (!(SPI->SR & SPI_SR_TXE))
            {
            }
            
            while (SPI->SR & SPI_SR_BSY)
            {
            }
            
            SPI->CR1 &= ~SPI_CR1_SPE;
            SPI->CR2 = 0;
            SPI->CR1 |= SPI_CR1_SPE;
        }
    }
    else
    {
        primask = __get_PRIMASK();

        SPI->CR1 &= ~SPI_CR1_SPE;
        SPI->CR1 |= SPI_CR1_BIDIMODE;

        spi_cr1 = (SPI->CR1 & ~SPI_CR1_SPE);

        rx_data_e = rx_data + count;
    
        do
        {
            __disable_irq();

            SPI->CR1 = spi_cr1 | SPI_CR1_SPE;

            __DSB();
            __DSB();

            SPI->CR1 = spi_cr1 & ~SPI_CR1_SPE;

            __set_PRIMASK(primask);

            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }
        
            *rx_data++ = SPI->DR;
        }
        while (rx_data != rx_data_e);

        SPI->CR1 &= ~SPI_CR1_BIDIMODE;
        SPI->CR1 |= SPI_CR1_SPE;
    }

    return true;
}

__attribute__((optimize("O3"))) bool stm32l0_spi_transmit(stm32l0_spi_t *spi, const uint8_t *tx_data, uint32_t count)
{
    SPI_TypeDef *SPI = spi->SPI;
    const uint8_t *tx_data_e;
    uint8_t tx_temp;

    if (spi->state != STM32L0_SPI_STATE_DATA)
    {
        return false;
    }

    if ((count <= 16) || (spi->tx_dma != stm32l0_dma_channel(spi->tx_dma)))
    {
        if (count == 1)
        {
            SPI->DR = *tx_data;
        }
        else
        {
            tx_data_e = tx_data + count;
            
            SPI->DR = *tx_data++;
            
            do 
            {
                tx_temp = *tx_data++;
                
                __asm__ volatile("": : : "memory");
                
                while (!(SPI->SR & SPI_SR_TXE))
                {
                }
                
                SPI->DR = tx_temp;
                
                __asm__ volatile("": : : "memory");
            } 
            while (tx_data != tx_data_e);
        }
        
        while (!(SPI->SR & SPI_SR_TXE))
        {
        }
        
        while (SPI->SR & SPI_SR_BSY)
        {
        }
        
        SPI->DR;
        SPI->SR;
    }
    else
    {
        SPI->CR1 &= ~SPI_CR1_SPE;
        SPI->CR2 = SPI_CR2_TXDMAEN;
        SPI->CR1 |= SPI_CR1_SPE;

        stm32l0_dma_start(spi->tx_dma, (uint32_t)&SPI->DR, (uint32_t)tx_data, count, STM32L0_SPI_TX_DMA_OPTION_TRANSMIT_8);
            
        while (!stm32l0_dma_done(spi->tx_dma))
        {
        }
        
        stm32l0_dma_stop(spi->tx_dma);
        
        while (!(SPI->SR & SPI_SR_TXE))
        {
        }
        
        while (SPI->SR & SPI_SR_BSY)
        {
        }
        
        SPI->CR1 &= ~SPI_CR1_SPE;
        SPI->CR2 = 0;
        SPI->CR1 |= SPI_CR1_SPE;
        
        SPI->DR;
        SPI->SR;
    }

    return true;
}

__attribute__((optimize("O3"))) bool stm32l0_spi_transfer(stm32l0_spi_t *spi, const uint8_t *tx_data, uint8_t *rx_data, uint32_t count)
{
    SPI_TypeDef *SPI = spi->SPI;
    const uint8_t *tx_data_e;
    uint8_t rx_temp, tx_temp;

    if (spi->state != STM32L0_SPI_STATE_DATA)
    {
        return false;
    }

    if ((count <= 16) || (spi->rx_dma != stm32l0_dma_channel(spi->rx_dma)))
    {
        if (count == 1)
        {
            SPI->DR = *tx_data;
        
            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }
        
            *rx_data = SPI->DR;
        }
        else
        {
            tx_data_e = tx_data + count;
        
            SPI->DR = *tx_data++;
        
            do 
            {
                tx_temp = *tx_data++;
            
                __asm__ volatile("": : : "memory");
            
                while (!(SPI->SR & SPI_SR_RXNE))
                {
                }
            
                rx_temp = SPI->DR;
                SPI->DR = tx_temp;
            
                __asm__ volatile("": : : "memory");
            
                *rx_data++ = rx_temp;
            } 
            while (tx_data != tx_data_e);
        
            while (!(SPI->SR & SPI_SR_RXNE))
            {
            }
        
            *rx_data++ = SPI->DR;
        }
    }
    else
    {
        if (spi->tx_dma != stm32l0_dma_channel(spi->tx_dma))
        {
            SPI->CR1 &= ~SPI_CR1_SPE;
            SPI->CR2 = SPI_CR2_RXDMAEN;
            SPI->CR1 |= SPI_CR1_SPE;
            
            stm32l0_dma_start(spi->rx_dma, (uint32_t)rx_data, (uint32_t)&SPI->DR, count, STM32L0_SPI_RX_DMA_OPTION_TRANSFER_8);

            tx_data_e = tx_data + count;
            
            SPI->DR = *tx_data++;
            
            do 
            {
                tx_temp = *tx_data++;
                
                __asm__ volatile("": : : "memory");
                
                while (!(SPI->SR & SPI_SR_TXE))
                {
                }
                
                SPI->DR = tx_temp;
                
                __asm__ volatile("": : : "memory");
            } 
            while (tx_data != tx_data_e);
            
            while (!stm32l0_dma_done(spi->rx_dma))
            {
            }
            
            stm32l0_dma_stop(spi->rx_dma);
        }
        else
        {
            SPI->CR1 &= ~SPI_CR1_SPE;
            SPI->CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
            SPI->CR1 |= SPI_CR1_SPE;
            
            stm32l0_dma_start(spi->rx_dma, (uint32_t)rx_data, (uint32_t)&SPI->DR, count, STM32L0_SPI_RX_DMA_OPTION_TRANSFER_8);
            stm32l0_dma_start(spi->tx_dma, (uint32_t)&SPI->DR, (uint32_t)tx_data, count, STM32L0_SPI_TX_DMA_OPTION_TRANSFER_8);
            
            while (!stm32l0_dma_done(spi->rx_dma))
            {
            }
            
            stm32l0_dma_stop(spi->rx_dma);
            stm32l0_dma_stop(spi->tx_dma);
        }
            
        while (!(SPI->SR & SPI_SR_TXE))
        {
        }
        
        while (SPI->SR & SPI_SR_BSY)
        {
        }
        
        SPI->CR1 &= ~SPI_CR1_SPE;
        SPI->CR2 = 0;
        SPI->CR1 |= SPI_CR1_SPE;
    }

    return true;
}
