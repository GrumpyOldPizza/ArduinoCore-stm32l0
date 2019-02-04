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
#include "stm32l0xx.h"

#include "stm32l0_gpio.h"
#include "stm32l0_exti.h"
#include "stm32l0_i2c.h"
#include "stm32l0_dma.h"
#include "stm32l0_system.h"

extern void I2C1_IRQHandler(void);
extern void I2C2_IRQHandler(void);
#if defined(STM32L072xx) || defined(STM32L082xx)
extern void I2C3_IRQHandler(void);
#endif /* STM32L072xx || STM32L082xx */

typedef struct _stm32l0_i2c_device_t {
    stm32l0_system_notify_t notify;
    volatile uint32_t       wakeup;
    stm32l0_i2c_t           *instances[STM32L0_I2C_INSTANCE_COUNT];
} stm32l0_i2c_device_t;

static stm32l0_i2c_device_t stm32l0_i2c_device;

#define STM32L0_I2C_TX_DMA_OPTION \
    (STM32L0_DMA_OPTION_MEMORY_TO_PERIPHERAL |   \
     STM32L0_DMA_OPTION_PERIPHERAL_DATA_SIZE_8 | \
     STM32L0_DMA_OPTION_MEMORY_DATA_SIZE_8 |     \
     STM32L0_DMA_OPTION_MEMORY_DATA_INCREMENT |  \
     STM32L0_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32L0_I2C_RX_DMA_OPTION \
    (STM32L0_DMA_OPTION_PERIPHERAL_TO_MEMORY |   \
     STM32L0_DMA_OPTION_PERIPHERAL_DATA_SIZE_8 | \
     STM32L0_DMA_OPTION_MEMORY_DATA_SIZE_8 |     \
     STM32L0_DMA_OPTION_MEMORY_DATA_INCREMENT |  \
     STM32L0_DMA_OPTION_PRIORITY_MEDIUM)

#define I2C_CR1_DNF_SHIFT    8
#define I2C_CR2_NBYTES_MAX   255
#define I2C_CR2_NBYTES_SHIFT 16
#define I2C_CR2_NBYTES_MASK  0x00ff0000

#define STM32L0_I2C_SUSPEND_CALLBACK ((stm32l0_i2c_suspend_callback_t)2)

static I2C_TypeDef * const stm32l0_i2c_xlate_I2C[STM32L0_I2C_INSTANCE_COUNT] = {
    I2C1,
    I2C2,
#if defined(STM32L072xx) || defined(STM32L082xx)
    I2C3,
#endif /* STM32L072xx || STM32L082xx */
};

static const IRQn_Type stm32l0_i2c_xlate_IRQn[STM32L0_I2C_INSTANCE_COUNT] = {
    I2C1_IRQn,
    I2C2_IRQn,
#if defined(STM32L072xx) || defined(STM32L082xx)
    I2C3_IRQn,
#endif /* STM32L072xx || STM32L082xx */
};

static const uint32_t stm32l0_i2c_xlate_FMP[STM32L0_I2C_INSTANCE_COUNT] = {
    SYSCFG_CFGR2_I2C1_FMP,
    SYSCFG_CFGR2_I2C2_FMP,
#if defined(STM32L072xx) || defined(STM32L082xx)
    SYSCFG_CFGR2_I2C3_FMP,
#endif /* STM32L072xx || STM32L082xx */
};

static const uint32_t stm32l0_i2c_xlate_IMR[STM32L0_I2C_INSTANCE_COUNT] = {
    EXTI_IMR_IM23,
    0,
#if defined(STM32L072xx) || defined(STM32L082xx)
    EXTI_IMR_IM24,
#endif /* STM32L072xx || STM32L082xx */
};

static void stm32l0_i2c_notify_callback(void *context, uint32_t events)
{
    /* WAR for ERRATA 2.6.1 */

    if (stm32l0_i2c_device.wakeup)
    {
        if (events & STM32L0_SYSTEM_EVENT_STOP_ENTER)
        {
            if (stm32l0_i2c_device.wakeup & (1u << STM32L0_I2C_INSTANCE_I2C1))
            {
                I2C1->CR1 &= ~I2C_CR1_PE;
            }

#if defined(STM32L072xx) || defined(STM32L082xx)
            if (stm32l0_i2c_device.wakeup & (1u << STM32L0_I2C_INSTANCE_I2C3))
            {
                I2C3->CR1 &= ~I2C_CR1_PE;
            }
#endif /* STM32L072xx || STM32L082xx */
        }

        if (events & STM32L0_SYSTEM_EVENT_STOP_LEAVE)
        {
            if (stm32l0_i2c_device.wakeup & (1u << STM32L0_I2C_INSTANCE_I2C1))
            {
                I2C1->CR1 |= I2C_CR1_PE;
            }

#if defined(STM32L072xx) || defined(STM32L082xx)
            if (stm32l0_i2c_device.wakeup & (1u << STM32L0_I2C_INSTANCE_I2C3))
            {
                I2C3->CR1 |= I2C_CR1_PE;
            }
#endif /* STM32L072xx || STM32L082xx */
        }
    }
}

static void stm32l0_i2c_start(stm32l0_i2c_t *i2c)
{
    I2C_TypeDef *I2C = i2c->I2C;
    uint32_t pclk, i2c_timingr;

    stm32l0_system_periph_enable(STM32L0_SYSTEM_PERIPH_I2C1 + i2c->instance);

    if (i2c->instance == STM32L0_I2C_INSTANCE_I2C2)
    {
        stm32l0_system_lock(STM32L0_SYSTEM_LOCK_CLOCKS);

        pclk = stm32l0_system_pclk1();

        if      (pclk == 32000000) { i2c_timingr = 0x00707cbb; }
        else if (pclk == 16000000) { i2c_timingr = 0x00303b5b; }
        else if (pclk ==  8000000) { i2c_timingr = 0x2000090e; }
        else if (pclk ==  4000000) { i2c_timingr = 0x00000e14; }
        else                       { i2c_timingr = 0x00100e16; }

        I2C->TIMINGR = i2c_timingr;
    }
    else
    {
        stm32l0_system_hsi16_enable();

        if ((i2c->option & STM32L0_I2C_OPTION_ADDRESS_MASK) && !(i2c->option & STM32L0_I2C_OPTION_WAKEUP))
        {
            armv6m_atomic_or(&stm32l0_i2c_device.wakeup, (1u << i2c->instance));
        }
    }

    I2C->CR1 |= I2C_CR1_PE;
}

static void stm32l0_i2c_stop(stm32l0_i2c_t *i2c)
{
    I2C_TypeDef *I2C = i2c->I2C;

    /* WAR for ERRATA 2.6.1 */
    I2C->CR1 &= ~I2C_CR1_PE;

    if (i2c->instance == STM32L0_I2C_INSTANCE_I2C2)
    {
        stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_CLOCKS);
    }
    else
    {
        if ((i2c->option & STM32L0_I2C_OPTION_ADDRESS_MASK) && !(i2c->option & STM32L0_I2C_OPTION_WAKEUP))
        {
            armv6m_atomic_and(&stm32l0_i2c_device.wakeup, ~(1u << i2c->instance));
        }

        stm32l0_system_hsi16_disable();
    }

    stm32l0_system_periph_disable(STM32L0_SYSTEM_PERIPH_I2C1 + i2c->instance);
}

static void stm32l0_i2c_sync(stm32l0_i2c_t *i2c)
{
    I2C_TypeDef *I2C = i2c->I2C;
    uint32_t i2c_cr1, i2c_cr2, i2c_oar1, i2c_oar2, i2c_timingr, i2c_timeoutr;

    i2c->rq_sync = false;

    if (i2c->option & STM32L0_I2C_OPTION_ADDRESS_MASK)
    {
        stm32l0_i2c_stop(i2c);

        armv6m_atomic_and(&EXTI->IMR, ~stm32l0_i2c_xlate_IMR[i2c->instance]);
    }
    
    armv6m_atomic_and(&SYSCFG->CFGR1, ~stm32l0_i2c_xlate_FMP[i2c->instance]);

    stm32l0_system_periph_enable(STM32L0_SYSTEM_PERIPH_I2C1 + i2c->instance);

    I2C->CR1 = 0;
    
    i2c_cr1 = 0;
    i2c_cr2 = 0;
    i2c_oar1 = 0;
    i2c_oar2 = 0;

    i2c_cr1 |= I2C_CR1_ERRIE;

    if (i2c->option & STM32L0_I2C_OPTION_ADDRESS_MASK)
    {
        i2c_oar1 = I2C_OAR1_OA1EN | (((i2c->option & STM32L0_I2C_OPTION_ADDRESS_MASK) >> STM32L0_I2C_OPTION_ADDRESS_SHIFT) << 1);

        if (i2c->option & STM32L0_I2C_OPTION_GENERAL_CALL)
        {
            i2c_cr1 |= I2C_CR1_GCEN;
        }

        i2c_cr1 |= I2C_CR1_ADDRIE;

        if (i2c->option & STM32L0_I2C_OPTION_WAKEUP)
        {
            i2c_cr1 |= I2C_CR1_WUPEN;

            armv6m_atomic_or(&EXTI->IMR, stm32l0_i2c_xlate_IMR[i2c->instance]);
        }
    }

    if (i2c->instance != STM32L0_I2C_INSTANCE_I2C2)
    {
        if (i2c->option & STM32L0_I2C_OPTION_MODE_1000K)
        {
            armv6m_atomic_or(&SYSCFG->CFGR1, stm32l0_i2c_xlate_FMP[i2c->instance]);
        }

        if      (i2c->option & STM32L0_I2C_OPTION_MODE_1000K) { i2c_timingr = 0x00000107; } 
        else if (i2c->option & STM32L0_I2C_OPTION_MODE_400K)  { i2c_timingr = 0x0010061a; }
        else                                                  { i2c_timingr = 0x00303d5b; }

        if (i2c->timeout) 
        {
            i2c_timeoutr = ((i2c->timeout * 125) + 15) / 16 -1;

            if (i2c_timeoutr > 4095)
            {
                i2c_timeoutr = 4095;
            }
            
            i2c_timeoutr |= I2C_TIMEOUTR_TIMOUTEN;
        }
        else
        {
            i2c_timeoutr = 0;
        }

        I2C->TIMINGR = i2c_timingr;
        I2C->TIMEOUTR = i2c_timeoutr;
    }

    I2C->OAR2 = i2c_oar2;
    I2C->OAR1 = i2c_oar1;
    I2C->CR2 = i2c_cr2;
    I2C->CR1 = i2c_cr1;

    if (i2c->option & STM32L0_I2C_OPTION_WAKEUP)
    {
        stm32l0_gpio_pin_configure(i2c->pins.scl, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_ALTERNATE));
        stm32l0_gpio_pin_configure(i2c->pins.sda, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_ALTERNATE));
    }
    else
    {
        stm32l0_gpio_pin_configure(i2c->pins.scl, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_ALTERNATE));
        stm32l0_gpio_pin_configure(i2c->pins.sda, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_ALTERNATE));
    }

    stm32l0_system_periph_disable(STM32L0_SYSTEM_PERIPH_I2C1 + i2c->instance);

    if (i2c->option & STM32L0_I2C_OPTION_ADDRESS_MASK) 
    {
        stm32l0_i2c_start(i2c);
    }
}

static void stm32l0_i2c_master_transmit(stm32l0_i2c_t *i2c)
{
    I2C_TypeDef *I2C = i2c->I2C;
    uint32_t i2c_cr2, count;

    i2c->state = STM32L0_I2C_STATE_MASTER_TRANSMIT;

    count = i2c->tx_data_e - i2c->tx_data;

    i2c_cr2 = (i2c->xf_address << 1) | I2C_CR2_START;

    if (count > I2C_CR2_NBYTES_MAX)
    {
        count = I2C_CR2_NBYTES_MAX;

        i2c_cr2 |= I2C_CR2_RELOAD;
    }
    else
    {
        if (!i2c->rx_data && !(i2c->xf_control & STM32L0_I2C_CONTROL_RESTART))
        {
            i2c_cr2 |= I2C_CR2_AUTOEND;
        }
    }

    if ((count > 1) && (i2c->tx_dma == stm32l0_dma_channel(i2c->tx_dma)))
    {
        I2C->CR1 |= I2C_CR1_TXDMAEN;

        stm32l0_dma_start(i2c->tx_dma, (uint32_t)&I2C->TXDR, (uint32_t)i2c->tx_data, (i2c->tx_data_e - i2c->tx_data), STM32L0_I2C_TX_DMA_OPTION);

        I2C->CR2 = (i2c_cr2 | (count << I2C_CR2_NBYTES_SHIFT));

        I2C->CR1 |= (I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_TCIE);

        I2C->ISR |= I2C_ISR_TXE;

        i2c->tx_data += count;
    }
    else
    {
        I2C->CR2 = (i2c_cr2 | (count << I2C_CR2_NBYTES_SHIFT));

        I2C->CR1 |= (I2C_CR1_TXIE | I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_TCIE);

        I2C->ISR |= I2C_ISR_TXE;

        I2C->TXDR = *(i2c->tx_data)++;
    }
}

static void stm32l0_i2c_master_receive(stm32l0_i2c_t *i2c)
{
    I2C_TypeDef *I2C = i2c->I2C;
    uint32_t i2c_cr2, count;

    i2c->state = STM32L0_I2C_STATE_MASTER_RECEIVE;

    i2c->xf_count = 0;

    count = i2c->rx_data_e - i2c->rx_data;

    i2c_cr2 = (i2c->xf_address << 1) | I2C_CR2_RD_WRN | I2C_CR2_START;

    /* WAR for ERRATA if NBYTES != 1 and RELOAD is set. This case is handled by using
     * interrupt driven receive mode with NBYTES set to 1 for each byte. This is
     * only affecting receive operations with more than I2C_CR2_NBYTES_MAX bytes.
     */

    if (count > I2C_CR2_NBYTES_MAX)
    {
        count = 1;

        i2c_cr2 |= I2C_CR2_RELOAD;
    }
    else
    {
        if (!(i2c->xf_control & STM32L0_I2C_CONTROL_RESTART))
        {
            i2c_cr2 |= I2C_CR2_AUTOEND;
        }
    }

    if ((count > 1) && (i2c->rx_dma == stm32l0_dma_channel(i2c->rx_dma)))
    {
        I2C->CR1 |= I2C_CR1_RXDMAEN;

        stm32l0_dma_start(i2c->rx_dma, (uint32_t)i2c->rx_data, (uint32_t)&I2C->RXDR, (i2c->rx_data_e - i2c->rx_data), STM32L0_I2C_RX_DMA_OPTION);

        I2C->CR1 |= (I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_TCIE);

        I2C->CR2 = (i2c_cr2 | (count << I2C_CR2_NBYTES_SHIFT));

        i2c->rx_data += count;
    }
    else
    {
        I2C->CR1 |= (I2C_CR1_RXIE | I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_TCIE);

        I2C->CR2 = (i2c_cr2 | (count << I2C_CR2_NBYTES_SHIFT));
    }
}

static void stm32l0_i2c_master_check(stm32l0_i2c_t *i2c)
{
    stm32l0_i2c_transaction_t *transaction, **pp_transaction, *entry, **pp_entry;

    if (i2c->xf_queue)
    {
        if ((i2c->state != STM32L0_I2C_STATE_SUSPENDED) && ((i2c->state == STM32L0_I2C_STATE_MASTER_RESTART) || !i2c->rq_callback))
        {
            do
            {
                transaction = NULL;
                
                for (pp_entry = &i2c->xf_queue, entry = *pp_entry; entry; pp_entry = &entry->next, entry = *pp_entry) 
                {
                    if ((i2c->state != STM32L0_I2C_STATE_MASTER_RESTART) || (i2c->xf_address == entry->address))
                    {
                        transaction = entry;
                        pp_transaction = pp_entry;
                    }
                }
                
                if (!transaction)
                {
                    break;
                }
            }
            while (armv6m_atomic_compare_and_swap((volatile uint32_t*)pp_transaction, (uint32_t)transaction, (uint32_t)NULL) != (uint32_t)transaction);
        
            if (transaction)
            {
                if (i2c->state == STM32L0_I2C_STATE_READY)
                {
                    stm32l0_system_lock(STM32L0_SYSTEM_LOCK_STOP);
                
                    stm32l0_i2c_start(i2c);
                
                    if (i2c->rx_dma != STM32L0_DMA_CHANNEL_NONE)
                    {
                        stm32l0_dma_enable(i2c->rx_dma, NULL, NULL);
                    }
                
                    if (i2c->tx_dma != STM32L0_DMA_CHANNEL_NONE)
                    {
                        stm32l0_dma_enable(i2c->tx_dma, NULL, NULL);
                    }
                }
            
                i2c->xf_transaction = transaction;
            
                i2c->xf_address = transaction->address;
                i2c->xf_control = transaction->control;
            
                i2c->tx_data = NULL;
                i2c->tx_data_e = NULL;
                i2c->rx_data = NULL;
                i2c->rx_data_e = NULL;
            
                if (transaction->rx_data)
                {
                    i2c->rx_data = transaction->rx_data;
                    i2c->rx_data_e = transaction->rx_data + transaction->rx_count;
                }
            
                if (transaction->tx_data)
                {
                    i2c->tx_data = transaction->tx_data;
                    i2c->tx_data_e = transaction->tx_data + transaction->tx_count;
                
                    stm32l0_i2c_master_transmit(i2c);
                }
                else
                {
                    stm32l0_i2c_master_receive(i2c);
                }
            }
        }
    }

    if (!i2c->xf_transaction)
    {
        if (i2c->state == STM32L0_I2C_STATE_MASTER_STOP)
        {
            if (i2c->rx_dma != STM32L0_DMA_CHANNEL_NONE)
            {
                stm32l0_dma_disable(i2c->rx_dma);
            }

            if (i2c->tx_dma != STM32L0_DMA_CHANNEL_NONE)
            {
                stm32l0_dma_disable(i2c->tx_dma);
            }
            
            stm32l0_i2c_stop(i2c);

            stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_STOP);

            i2c->state = STM32L0_I2C_STATE_READY;
        }

        if (i2c->state == STM32L0_I2C_STATE_READY)
        {
            if (i2c->rq_sync)
            {
                stm32l0_i2c_sync(i2c);
            }

            if (i2c->rq_callback)
            {
                stm32l0_gpio_pin_configure(i2c->pins.scl, (STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_INPUT));
                stm32l0_gpio_pin_configure(i2c->pins.sda, (STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_INPUT));
                
                i2c->state = STM32L0_I2C_STATE_SUSPENDED;

                NVIC_DisableIRQ(i2c->interrupt);

                if (i2c->rq_callback != STM32L0_I2C_SUSPEND_CALLBACK)
                {
                    (*i2c->rq_callback)(i2c->rq_context);
                }

                i2c->rq_callback = NULL;
            }
        }
    }
}

static void stm32l0_i2c_slave_check(stm32l0_i2c_t *i2c)
{
    if (i2c->rq_sync)
    {
        stm32l0_i2c_sync(i2c);
    }

    if (i2c->rq_callback)
    {
        stm32l0_i2c_stop(i2c);

        stm32l0_gpio_pin_configure(i2c->pins.scl, (STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_INPUT));
        stm32l0_gpio_pin_configure(i2c->pins.sda, (STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_INPUT));
        
        i2c->state = STM32L0_I2C_STATE_SUSPENDED;

        NVIC_DisableIRQ(i2c->interrupt);

        if (i2c->rq_callback != STM32L0_I2C_SUSPEND_CALLBACK)
        {
            (*i2c->rq_callback)(i2c->rq_context);
        }
        
        i2c->rq_callback = NULL;
    }
}

static void stm32l0_i2c_slave_address(stm32l0_i2c_t *i2c)
{
    I2C_TypeDef *I2C = i2c->I2C;

    i2c->xf_address = (I2C->ISR >> 17) & 0x7f;
    i2c->xf_count = 0;

    if (I2C->ISR & I2C_ISR_DIR)
    {
        i2c->state = STM32L0_I2C_STATE_SLAVE_TRANSMIT;

        i2c->tx_data = NULL;
        i2c->tx_data_e = NULL;

        (*i2c->ev_callback)(i2c->ev_context, STM32L0_I2C_EVENT_TRANSMIT_REQUEST | (i2c->xf_address << STM32L0_I2C_EVENT_ADDRESS_SHIFT));

        I2C->CR2 = 0;
            
        I2C->CR1 |= (I2C_CR1_TXIE | I2C_CR1_STOPIE | I2C_CR1_TCIE);

        I2C->CR1 &= ~I2C_CR1_SBC;

        I2C->ISR |= I2C_ISR_TXE;
            
        if (i2c->tx_data)
        {
            I2C->TXDR = *(i2c->tx_data)++;

            if (i2c->tx_data == i2c->tx_data_e)
            {
                i2c->tx_data = NULL;
                i2c->tx_data_e = NULL;
            }
        }
        else
        {
            I2C->TXDR = 0xff;
        }
        
        i2c->xf_count++;
    }
    else
    {
        i2c->state = STM32L0_I2C_STATE_SLAVE_RECEIVE;

        i2c->rx_data = NULL;
        i2c->rx_data_e = NULL;

        (*i2c->ev_callback)(i2c->ev_context, STM32L0_I2C_EVENT_RECEIVE_REQUEST | (i2c->xf_address << STM32L0_I2C_EVENT_ADDRESS_SHIFT));

        I2C->CR2 = (I2C_CR2_RELOAD | (1 << I2C_CR2_NBYTES_SHIFT));
                
        I2C->CR1 |= (I2C_CR1_SBC | I2C_CR1_STOPIE | I2C_CR1_TCIE);
    }

    I2C->ICR = I2C_ICR_ADDRCF;
}

static void stm32l0_i2c_interrupt(stm32l0_i2c_t *i2c)
{
    I2C_TypeDef *I2C = i2c->I2C;
    stm32l0_i2c_transaction_t *transaction;
    uint32_t i2c_isr, i2c_cr2, count;

    i2c_isr = I2C->ISR;

    I2C->ICR = I2C_ICR_ALERTCF | I2C_ICR_TIMOUTCF | I2C_ICR_PECCF | I2C_ICR_OVRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF;

    switch (i2c->state) {

    case STM32L0_I2C_STATE_NONE:
    case STM32L0_I2C_STATE_INIT:
        break;

    case STM32L0_I2C_STATE_READY:
        if (i2c->option & STM32L0_I2C_OPTION_ADDRESS_MASK)
        {
            if (i2c_isr & I2C_ISR_ADDR)
            {
                stm32l0_system_lock(STM32L0_SYSTEM_LOCK_STOP);

                stm32l0_i2c_slave_address(i2c);
            }
            else
            {
                stm32l0_i2c_slave_check(i2c);
            }
        }
        else
        {
            if (i2c->rq_sync)
            {
                stm32l0_i2c_sync(i2c);
            }

            stm32l0_i2c_master_check(i2c);
        }
        break;

    case STM32L0_I2C_STATE_SUSPENDED:
        break;
        
    case STM32L0_I2C_STATE_MASTER_STOP:
    case STM32L0_I2C_STATE_MASTER_RESTART:
        stm32l0_i2c_master_check(i2c);
        break;

    case STM32L0_I2C_STATE_MASTER_NACK:
        if (i2c_isr & I2C_ISR_STOPF)
        {
            I2C->ICR = I2C_ICR_STOPCF;

            I2C->CR1 &= ~I2C_CR1_STOPIE;

            i2c->state = STM32L0_I2C_STATE_MASTER_STOP;

            transaction = i2c->xf_transaction;
            i2c->xf_transaction = NULL;

            if (i2c->tx_data)
            {
                transaction->status = ((i2c->tx_data == transaction->tx_data) ? STM32L0_I2C_STATUS_TRANSMIT_ADDRESS_NACK : STM32L0_I2C_STATUS_TRANSMIT_DATA_NACK);
            }
            else
            {
                transaction->status = STM32L0_I2C_STATUS_RECEIVE_ADDRESS_NACK;
            }

            if (transaction->callback)
            {
                (*transaction->callback)(transaction->context);
            }

            stm32l0_i2c_master_check(i2c);
        }
        break;

    case STM32L0_I2C_STATE_MASTER_TRANSMIT:
        if (i2c_isr & (I2C_ISR_TIMEOUT | I2C_ISR_ARLO))
        {
            I2C->ICR = I2C_ICR_NACKCF;

            if (I2C->CR1 & I2C_CR1_TXDMAEN)
            {
                stm32l0_dma_stop(i2c->tx_dma);

                I2C->CR1 &= ~(I2C_CR1_NACKIE | I2C_CR1_TCIE | I2C_CR1_TXDMAEN);
            }
            else
            {
                I2C->CR1 &= ~(I2C_CR1_TXIE | I2C_CR1_NACKIE | I2C_CR1_TCIE);
            }

            i2c->state = STM32L0_I2C_STATE_MASTER_STOP;

            transaction = i2c->xf_transaction;
            i2c->xf_transaction = NULL;

            transaction->status = (i2c_isr & I2C_ISR_TIMEOUT) ? STM32L0_I2C_STATUS_TRANSMIT_TIMEOUT : STM32L0_I2C_STATUS_TRANSMIT_ARBITRATION_LOST;

            if (transaction->callback)
            {
                (*transaction->callback)(transaction->context);
            }

            stm32l0_i2c_master_check(i2c);
        }
        else
        {
            if (i2c_isr & I2C_ISR_NACKF)
            {
                I2C->ICR = I2C_ICR_NACKCF;
                
                if (I2C->CR1 & I2C_CR1_TXDMAEN)
                {
                    i2c->tx_data = i2c->xf_transaction->tx_data + stm32l0_dma_stop(i2c->tx_dma);
                    
                    I2C->CR1 &= ~(I2C_CR1_NACKIE | I2C_CR1_TCIE | I2C_CR1_TXDMAEN);
                }
                else
                {
                    I2C->CR1 &= ~(I2C_CR1_TXIE | I2C_CR1_NACKIE | I2C_CR1_TCIE);
                }
                
                /* If I2C_TXDR is not empty, there there is an unsent byte.
                 */
                if (!(I2C->ISR & I2C_ISR_TXE))
                {
                    i2c->tx_data--;
                }
                
                I2C->CR1 |= I2C_CR1_STOPIE;
                
                i2c->state = STM32L0_I2C_STATE_MASTER_NACK;
            }
            else
            {
                if (i2c_isr & I2C_ISR_TXIS)
                {
                    if (I2C->CR1 & I2C_CR1_TXIE)
                    {
                        I2C->TXDR = *(i2c->tx_data)++;
                    }
                }

                if (i2c_isr & (I2C_ISR_TCR | I2C_ISR_TC | I2C_ISR_STOPF))
                {
                    if (i2c_isr & (I2C_ISR_TC | I2C_ISR_STOPF))
                    {
                        if (I2C->CR1 & I2C_CR1_TXDMAEN)
                        {
                            stm32l0_dma_stop(i2c->tx_dma);
                            
                            I2C->CR1 &= ~(I2C_CR1_NACKIE | I2C_CR1_TCIE | I2C_CR1_TXDMAEN);
                        }
                        else
                        {
                            I2C->CR1 &= ~(I2C_CR1_TXIE | I2C_CR1_NACKIE | I2C_CR1_TCIE);
                        }

                        if (i2c_isr & I2C_ISR_TC)
                        {
                            i2c->state = STM32L0_I2C_STATE_MASTER_RESTART;
                        }
                        else
                        {
                            I2C->ICR = I2C_ICR_STOPCF;

                            i2c->state = STM32L0_I2C_STATE_MASTER_STOP;
                        }
                        
                        if (i2c->rx_data)
                        {
                            stm32l0_i2c_master_receive(i2c);
                        }
                        else
                        {
                            transaction = i2c->xf_transaction;
                            i2c->xf_transaction = NULL;

                            transaction->status = STM32L0_I2C_STATUS_SUCCESS;

                            if (transaction->callback)
                            {
                                (*transaction->callback)(transaction->context);
                            }

                            stm32l0_i2c_master_check(i2c);
                        }
                    }
                    else /* I2C_ISR_TCR */
                    {
                        i2c_cr2 = I2C->CR2 & ~(I2C_CR2_AUTOEND | I2C_CR2_RELOAD | I2C_CR2_NBYTES | I2C_CR2_NACK | I2C_CR2_STOP | I2C_CR2_START);
                        
                        count = i2c->tx_data_e - i2c->tx_data;
                        
                        if (count > I2C_CR2_NBYTES_MAX)
                        {
                            count = I2C_CR2_NBYTES_MAX;
                            
                            i2c_cr2 |= I2C_CR2_RELOAD;
                        }
                        else
                        {
                            if (!i2c->rx_data && !(i2c->xf_control & STM32L0_I2C_CONTROL_RESTART))
                            {
                                i2c_cr2 |= I2C_CR2_AUTOEND;
                            }
                        }
                        
                        I2C->CR2 = (i2c_cr2 | (count << I2C_CR2_NBYTES_SHIFT));
                        
                        if (!(I2C->CR1 & I2C_CR1_TXIE))
                        {
                            i2c->tx_data += count;
                        }
                        else
                        {
                            if (I2C->ISR & I2C_ISR_TXE)
                            {
                                I2C->TXDR = *(i2c->tx_data)++;
                            }
                        }
                    }
                }
            }
        }
        break;

    case STM32L0_I2C_STATE_MASTER_RECEIVE:
        if (i2c_isr & (I2C_ISR_TIMEOUT | I2C_ISR_ARLO))
        {
            I2C->ICR = I2C_ICR_NACKCF;

            if (I2C->CR1 & I2C_CR1_RXIE)
            {
                I2C->CR1 &= ~(I2C_CR1_RXIE | I2C_CR1_NACKIE | I2C_CR1_TCIE);
            }
            else
            {
                stm32l0_dma_stop(i2c->rx_dma);

                I2C->CR1 &= ~(I2C_CR1_NACKIE | I2C_CR1_TCIE | I2C_CR1_RXDMAEN);
            }

            i2c->state = STM32L0_I2C_STATE_MASTER_STOP;

            transaction = i2c->xf_transaction;
            i2c->xf_transaction = NULL;

            transaction->status = (I2C->ISR & I2C_ISR_TIMEOUT) ? STM32L0_I2C_STATUS_RECEIVE_TIMEOUT : STM32L0_I2C_STATUS_RECEIVE_ARBITRATION_LOST;

            if (transaction->callback)
            {
                (*transaction->callback)(transaction->context);
            }

            stm32l0_i2c_master_check(i2c);
        }
        else
        {
            if (i2c_isr & I2C_ISR_NACKF)
            {
                I2C->ICR = I2C_ICR_NACKCF;
                
                if (I2C->CR1 & I2C_CR1_RXIE)
                {
                    I2C->CR1 &= ~(I2C_CR1_RXIE | I2C_CR1_NACKIE | I2C_CR1_TCIE);
                }
                else
                {
                    stm32l0_dma_stop(i2c->rx_dma);
                    
                    I2C->CR1 &= ~(I2C_CR1_NACKIE | I2C_CR1_TCIE | I2C_CR1_RXDMAEN);
                }
                
                I2C->CR1 |= I2C_CR1_STOPIE;
                
                i2c->state = STM32L0_I2C_STATE_MASTER_NACK;
                transaction = i2c->xf_transaction;
            }
            else
            {
                if (i2c_isr & I2C_ISR_RXNE)
                {
                    if (I2C->CR1 & I2C_CR1_RXIE)
                    {
                        *(i2c->rx_data)++ = I2C->RXDR;
                    }
                }

                if (i2c_isr & (I2C_ISR_TCR | I2C_ISR_TC | I2C_ISR_STOPF))
                {
                    if (i2c_isr & (I2C_ISR_TC | I2C_ISR_STOPF))
                    {
                        if (I2C->CR1 & I2C_CR1_RXIE)
                        {
                            I2C->CR1 &= ~(I2C_CR1_RXIE | I2C_CR1_NACKIE | I2C_CR1_TCIE);
                        }
                        else
                        {
                            stm32l0_dma_stop(i2c->rx_dma);
                            
                            I2C->CR1 &= ~(I2C_CR1_NACKIE | I2C_CR1_TCIE | I2C_CR1_RXDMAEN);
                        }
                        
                        if (i2c_isr & I2C_ISR_TC)
                        {
                            i2c->state = STM32L0_I2C_STATE_MASTER_RESTART;
                        }
                        else
                        {
                            I2C->ICR = I2C_ICR_STOPCF;

                            i2c->state = STM32L0_I2C_STATE_MASTER_STOP;
                        }

                        transaction = i2c->xf_transaction;
                        i2c->xf_transaction = NULL;

                        transaction->status = STM32L0_I2C_STATUS_SUCCESS;

                        if (transaction->callback)
                        {
                            (*transaction->callback)(transaction->context);
                        }

                        stm32l0_i2c_master_check(i2c);
                    }
                    else /* I2C_ISR_TCR */
                    {
                        i2c_cr2 = I2C->CR2 & ~(I2C_CR2_AUTOEND | I2C_CR2_RELOAD | I2C_CR2_NBYTES | I2C_CR2_NACK |  I2C_CR2_STOP | I2C_CR2_START);

                        count = (i2c->rx_data_e - i2c->rx_data);
                        
                        if (count > 1)
                        {
                            i2c_cr2 |= I2C_CR2_RELOAD;
                        }
                        else
                        {
                            if (!(i2c->xf_control & STM32L0_I2C_CONTROL_RESTART))
                            {
                                i2c_cr2 |= I2C_CR2_AUTOEND;
                            }
                        }
                        
                        I2C->CR2 = (i2c_cr2 | (1 << I2C_CR2_NBYTES_SHIFT));
                    }
                }
            }
        }
        break;

    case STM32L0_I2C_STATE_SLAVE_TRANSMIT:
        /* A slave transmit is terminated by a NACK followed by a STOP of the master receiver.
         */

        if (i2c_isr & (I2C_ISR_ADDR | I2C_ISR_STOPF))
        {
            I2C->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

            I2C->CR1 &= ~(I2C_CR1_TXIE | I2C_CR1_STOPIE);
            
            /* If I2C_TXDR is not empty, there there is an unsent byte.
             */
            if (!(I2C->ISR & I2C_ISR_TXE))
            {
                i2c->xf_count--;
            }
            
            i2c->state = STM32L0_I2C_STATE_READY;
                
            (*i2c->ev_callback)(i2c->ev_context, STM32L0_I2C_EVENT_TRANSMIT_DONE | (i2c->xf_count << STM32L0_I2C_EVENT_COUNT_SHIFT));

            if (I2C->ISR & I2C_ISR_ADDR)
            {
                stm32l0_i2c_slave_address(i2c);
            }
            else
            {
                stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_STOP);

                stm32l0_i2c_slave_check(i2c);
            }
        }
        else
        {
            if (i2c_isr & I2C_ISR_TXIS)
            {
                if (i2c->tx_data)
                {
                    I2C->TXDR = *(i2c->tx_data)++;

                    if (i2c->tx_data == i2c->tx_data_e)
                    {
                        i2c->tx_data = NULL;
                        i2c->tx_data_e = NULL;
                    }
                }
                else
                {
                    I2C->TXDR = 0xff;
                }
                
                i2c->xf_count++;
            }
        }
        break;

    case STM32L0_I2C_STATE_SLAVE_RECEIVE:
        /* A slave receive is terminated either by STOP, or by a repeated
         * start, i.e. an ADDR match.
         */

        if (i2c_isr & (I2C_ISR_ADDR | I2C_ISR_STOPF))
        {
            I2C->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

            I2C->CR1 &= ~(I2C_CR1_STOPIE | I2C_CR1_TCIE);
            
            i2c->state = STM32L0_I2C_STATE_READY;
                
            (*i2c->ev_callback)(i2c->ev_context, STM32L0_I2C_EVENT_RECEIVE_DONE | (i2c->xf_count << STM32L0_I2C_EVENT_COUNT_SHIFT));

            if (i2c_isr & I2C_ISR_ADDR)
            {
                stm32l0_i2c_slave_address(i2c);
            }
            else
            {
                stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_STOP);

                stm32l0_i2c_slave_check(i2c);
            }
        }
        else
        {
            if (i2c_isr & I2C_ISR_TCR)
            {
                i2c_cr2 = I2C->CR2 & ~(I2C_CR2_AUTOEND | I2C_CR2_RELOAD | I2C_CR2_NBYTES | I2C_CR2_NACK |  I2C_CR2_STOP | I2C_CR2_START);

                if (i2c->rx_data)
                {
                    *(i2c->rx_data)++ = I2C->RXDR;

                    i2c->xf_count++;
                    
                    if (i2c->rx_data == i2c->rx_data_e)
                    {
                        i2c->rx_data = NULL;
                        i2c->rx_data_e = NULL;
                    }
                }
                else
                {
                    I2C->RXDR;
                    
                    i2c_cr2 |= I2C_CR2_NACK;
                }

                I2C->CR2 = i2c_cr2 | I2C_CR2_RELOAD | (1 << I2C_CR2_NBYTES_SHIFT);
            }
        }
        break;
    }
}

bool stm32l0_i2c_create(stm32l0_i2c_t *i2c, const stm32l0_i2c_params_t *params)
{
    if (i2c->state != STM32L0_I2C_STATE_NONE)
    {
        return false;
    }

    i2c->I2C = stm32l0_i2c_xlate_I2C[params->instance];
    i2c->instance = params->instance;
    i2c->interrupt = stm32l0_i2c_xlate_IRQn[params->instance];
    i2c->priority = params->priority;
    i2c->rx_dma = params->rx_dma;
    i2c->tx_dma = params->tx_dma;
    i2c->pins = params->pins;

    stm32l0_i2c_device.instances[i2c->instance] = i2c;

    i2c->state = STM32L0_I2C_STATE_INIT;

    if (!stm32l0_i2c_device.notify.callback)
    {
        stm32l0_system_notify(&stm32l0_i2c_device.notify, stm32l0_i2c_notify_callback, NULL, (STM32L0_SYSTEM_EVENT_STOP_ENTER | STM32L0_SYSTEM_EVENT_STOP_LEAVE));
    }

    return true;
}

bool stm32l0_i2c_destroy(stm32l0_i2c_t *i2c)
{
    if (i2c->state != STM32L0_I2C_STATE_INIT)
    {
        return false;
    }

    i2c->state = STM32L0_I2C_STATE_NONE;

    stm32l0_i2c_device.instances[i2c->instance] = NULL;

    return true;
}

bool stm32l0_i2c_enable(stm32l0_i2c_t *i2c, uint32_t option, uint32_t timeout, stm32l0_i2c_event_callback_t callback, void *context)
{
    if (i2c->state != STM32L0_I2C_STATE_INIT)
    {
        return false;
    }

    i2c->ev_callback = callback;
    i2c->ev_context = context;

    if (!stm32l0_i2c_configure(i2c, option, timeout))
    {
        return false;
    }

    if (i2c->instance == STM32L0_I2C_INSTANCE_I2C2)
    {
        stm32l0_system_reference(STM32L0_SYSTEM_REFERENCE_I2C2);
    }

    i2c->state = STM32L0_I2C_STATE_READY;

    NVIC_SetPriority(i2c->interrupt, i2c->priority);
    NVIC_EnableIRQ(i2c->interrupt);

    return true;
}

bool stm32l0_i2c_disable(stm32l0_i2c_t *i2c)
{
    if (i2c->state != STM32L0_I2C_STATE_READY)
    {
        return false;
    }

    if (i2c->option & STM32L0_I2C_OPTION_ADDRESS_MASK)
    {
        stm32l0_i2c_stop(i2c);

        armv6m_atomic_and(&EXTI->IMR, ~stm32l0_i2c_xlate_IMR[i2c->instance]);
    }

    if (i2c->instance == STM32L0_I2C_INSTANCE_I2C2)
    {
        stm32l0_system_unreference(STM32L0_SYSTEM_REFERENCE_I2C2);
    }

    armv6m_atomic_and(&SYSCFG->CFGR1, ~stm32l0_i2c_xlate_FMP[i2c->instance]);

    NVIC_DisableIRQ(i2c->interrupt);

    i2c->state = STM32L0_I2C_STATE_INIT;

    return true;
}

bool stm32l0_i2c_configure(stm32l0_i2c_t *i2c, uint32_t option, uint32_t timeout)
{
    if (i2c->state == STM32L0_I2C_STATE_NONE)
    {
        return false;
    }

    if (option & STM32L0_I2C_OPTION_ADDRESS_MASK)
    {
        if (i2c->instance == STM32L0_I2C_INSTANCE_I2C2)
        {
            return false;
        }
    }
    else
    {
        if (i2c->instance == STM32L0_I2C_INSTANCE_I2C2)
        {
            if (option & (STM32L0_I2C_OPTION_MODE_400K | STM32L0_I2C_OPTION_MODE_1000K))
            {
                return false;
            }

            if (stm32l0_system_pclk1() < 4000000)
            {
                return false;
            }

            timeout = 0;
        }

        option &= ~STM32L0_I2C_OPTION_WAKEUP;
    }

    i2c->option = option;
    i2c->timeout = timeout;

    i2c->rq_sync = true;

    NVIC_SetPendingIRQ(i2c->interrupt);

    return true;
}

bool stm32l0_i2c_suspend(stm32l0_i2c_t *i2c, stm32l0_i2c_suspend_callback_t callback, void *context)
{
    if (i2c->rq_callback)
    {
        return false;
    }

    if (i2c->state < STM32L0_I2C_STATE_READY)
    {
        return false;
    }

    if (callback)
    {
        i2c->rq_context = context;
        i2c->rq_callback = callback;
    }
    else
    {
        i2c->rq_callback = STM32L0_I2C_SUSPEND_CALLBACK;
    }

    NVIC_SetPendingIRQ(i2c->interrupt);

    return true;
}

void stm32l0_i2c_resume(stm32l0_i2c_t *i2c)
{
    if (i2c->state == STM32L0_I2C_STATE_SUSPENDED)
    {
        i2c->rq_callback = NULL;

        if (i2c->option & STM32L0_I2C_OPTION_WAKEUP)
        {
            stm32l0_gpio_pin_configure(i2c->pins.scl, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_ALTERNATE));
            stm32l0_gpio_pin_configure(i2c->pins.sda, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_ALTERNATE));
        }
        else
        {
            stm32l0_gpio_pin_configure(i2c->pins.scl, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_ALTERNATE));
            stm32l0_gpio_pin_configure(i2c->pins.sda, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_ALTERNATE));
        }
        
        if (i2c->option & STM32L0_I2C_OPTION_ADDRESS_MASK) 
        {
            stm32l0_i2c_start(i2c);
        }
        
        i2c->state = STM32L0_I2C_STATE_READY;
        
        NVIC_EnableIRQ(i2c->interrupt);
        
        NVIC_SetPendingIRQ(i2c->interrupt);
    }
}

bool stm32l0_i2c_reset(stm32l0_i2c_t *i2c)
{
    uint32_t pin_scl, pin_sda, count;

    if (i2c->state != STM32L0_I2C_STATE_SUSPENDED)
    {
        return false;
    }

    pin_scl = i2c->pins.scl;
    pin_sda = i2c->pins.sda;

    stm32l0_gpio_pin_configure(pin_scl, (STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_INPUT));
    stm32l0_gpio_pin_configure(pin_sda, (STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_INPUT));
    stm32l0_gpio_pin_write(pin_scl, 0);
    stm32l0_gpio_pin_write(pin_sda, 1);
    stm32l0_gpio_pin_output(pin_scl);
    stm32l0_gpio_pin_output(pin_sda);
    
    /* Clock 16 SCL cycles to force a slave to release SDA and then issue a manual STOP condition.
     * The spec suggest 9 SCL cycles, but in practice more are needed. 
     */
    
    for (count = 0; count < 16; count++)
    {
        armv6m_core_udelay(6);    
        
        /* Set SCL to H */ 
        stm32l0_gpio_pin_write(pin_scl, 1);
        armv6m_core_udelay(4);    
        
        /* Set SCL to L */ 
        stm32l0_gpio_pin_write(pin_scl, 0);
    }
    
    // Send a STOP condition (SCL = 1, SDA = 0 -> 1)
    armv6m_core_udelay(1);    
    stm32l0_gpio_pin_write(pin_sda, 0);
    armv6m_core_udelay(1);    
    stm32l0_gpio_pin_write(pin_scl, 1);
    armv6m_core_udelay(2);    
    stm32l0_gpio_pin_write(pin_sda, 1);
    armv6m_core_udelay(40);    

    stm32l0_gpio_pin_input(pin_sda);
    stm32l0_gpio_pin_input(pin_scl);

    return true;
}

bool stm32l0_i2c_receive(stm32l0_i2c_t *i2c, uint8_t *rx_data, uint16_t rx_count)
{
    if (i2c->state != STM32L0_I2C_STATE_SLAVE_RECEIVE)
    {
        return false;
    }

    if (rx_count)
    {
        i2c->rx_data = rx_data;
        i2c->rx_data_e = rx_data + rx_count;
    }

    return true;
}

bool stm32l0_i2c_transmit(stm32l0_i2c_t *i2c, uint8_t *tx_data, uint16_t tx_count)
{
    if (i2c->state != STM32L0_I2C_STATE_SLAVE_TRANSMIT)
    {
        return false;
    }

    if (tx_count)
    {
        i2c->tx_data = tx_data;
        i2c->tx_data_e = tx_data + tx_count;
    }

    return true;
}

bool stm32l0_i2c_submit(stm32l0_i2c_t *i2c, stm32l0_i2c_transaction_t *transaction)
{
    stm32l0_i2c_transaction_t *entry;

    if (i2c->state < STM32L0_I2C_STATE_READY)
    {
        return false;
    }

    if (i2c->option & STM32L0_I2C_OPTION_ADDRESS_MASK)
    {
        return false;
    }

    if (!transaction->tx_data && !transaction->rx_data)
    {
        return false;
    }

    transaction->status = STM32L0_I2C_STATUS_BUSY;

    do
    {
        entry = i2c->xf_queue;
        transaction->next = entry;
    }
    while (armv6m_atomic_compare_and_swap((volatile uint32_t*)&i2c->xf_queue, (uint32_t)entry, (uint32_t)transaction) != (uint32_t)entry);

    NVIC_SetPendingIRQ(i2c->interrupt);

    return true;
}

void I2C1_IRQHandler(void)
{
    stm32l0_i2c_interrupt(stm32l0_i2c_device.instances[STM32L0_I2C_INSTANCE_I2C1]);
}

void I2C2_IRQHandler(void)
{
    stm32l0_i2c_interrupt(stm32l0_i2c_device.instances[STM32L0_I2C_INSTANCE_I2C2]);
}

#if defined(STM32L072xx) || defined(STM32L082xx)

void I2C3_IRQHandler(void)
{
    stm32l0_i2c_interrupt(stm32l0_i2c_device.instances[STM32L0_I2C_INSTANCE_I2C3]);
}

#endif /* STM32L072xx || STM32L082xx */

