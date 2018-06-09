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
    stm32l0_i2c_t     *instances[STM32L0_I2C_INSTANCE_COUNT];
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

static void stm32l0_i2c_start(stm32l0_i2c_t *i2c, bool enable)
{
    I2C_TypeDef *I2C = i2c->I2C;
    uint32_t sysclk, pclk, i2cclk, i2c_sel, i2c_timingr, i2c_cr1;

    stm32l0_system_periph_enable(STM32L0_SYSTEM_PERIPH_I2C1 + i2c->instance);

    sysclk = stm32l0_system_sysclk();
    pclk = stm32l0_system_pclk1();

    if ((i2c->sysclk != sysclk) || (i2c->pclk != pclk))
    {
        i2c->sysclk = sysclk;
        i2c->pclk = pclk;

        if (i2c->option & STM32L0_I2C_OPTION_ADDRESS_MASK)
        {
            if ((i2c->option & STM32L0_I2C_OPTION_MODE_MASK) == STM32L0_I2C_OPTION_MODE_400K) { i2c_timingr = 0x00610c13; i2c_cr1 = 0; } 
            else                                                                              { i2c_timingr = 0x10991e2d; i2c_cr1 = 0; } 

            i2c_sel = RCC_CCIPR_I2C1SEL_1;
        }
        else
        {
            if ((i2c->option & STM32L0_I2C_OPTION_MODE_MASK) == STM32L0_I2C_OPTION_MODE_1000K)
            {
                i2c_timingr = 0x0051070d; i2c_cr1 = I2C_CR1_ANFOFF | (2 << I2C_CR1_DNF_SHIFT);

                i2c_sel = ((pclk == 32000000) ? 0 : RCC_CCIPR_I2C1SEL_0);
            }
            else
            {
                if ((i2c->option & STM32L0_I2C_OPTION_MODE_MASK) == STM32L0_I2C_OPTION_MODE_400K)
                {
                    if (i2c->instance == STM32L0_I2C_INSTANCE_I2C2)
                    {
                        i2cclk = pclk;
                        
                        i2c_sel = 0;
                    }
                    else
                    {
                        i2cclk = 16000000;
                        
                        if      (pclk   == 16000000) { i2c_sel = 0;                   }
                        else if (sysclk == 16000000) { i2c_sel = RCC_CCIPR_I2C1SEL_0; }
                        else                         { i2c_sel = RCC_CCIPR_I2C1SEL_1; }
                    }
                    
                    if   (i2cclk == 32000000) { i2c_timingr = 0x00c91b29; i2c_cr1 = I2C_CR1_ANFOFF | (2 << I2C_CR1_DNF_SHIFT); }
                    else                      { i2c_timingr = 0x00610b13; i2c_cr1 = I2C_CR1_ANFOFF | (1 << I2C_CR1_DNF_SHIFT); }
                }
                else
                {
                    if ((i2c->instance == STM32L0_I2C_INSTANCE_I2C2) || (pclk >= 4000000))
                    {
                        i2cclk = pclk;
                        
                        i2c_sel = 0;
                    }
                    else
                    {
                        if (sysclk >= 4000000)
                        {
                            i2cclk = sysclk;
                            
                            i2c_sel = RCC_CCIPR_I2C1SEL_0;
                        }
                        else
                        {
                            i2cclk = 16000000;
                            
                            i2c_sel = RCC_CCIPR_I2C1SEL_1;
                        }
                    }
                    
                    if      (i2cclk == 32000000) { i2c_timingr = 0x20dd293e; i2c_cr1 = I2C_CR1_ANFOFF; }
                    else if (i2cclk == 16000000) { i2c_timingr = 0x10991e2d; i2c_cr1 = I2C_CR1_ANFOFF; }
                    else if (i2cclk ==  8000000) { i2c_timingr = 0x00971c2c; i2c_cr1 = I2C_CR1_ANFOFF; }
                    else if (i2cclk ==  4000000) { i2c_timingr = 0x00420c14; i2c_cr1 = I2C_CR1_ANFOFF; }
                    else                         { i2c_timingr = 0x00530d15; i2c_cr1 = I2C_CR1_ANFOFF; }
                }
            }
        }

        armv6m_atomic_modify(&RCC->CCIPR, ((RCC_CCIPR_I2C1SEL_1 | RCC_CCIPR_I2C1SEL_0) << (i2c->instance * 2)), (i2c_sel << (i2c->instance * 2)));

        I2C->TIMINGR = i2c_timingr;
        I2C->CR1 = (I2C->CR1 & ~(I2C_CR1_ANFOFF | I2C_CR1_DNF)) | i2c_cr1;
    }

    if (RCC->CCIPR & (RCC_CCIPR_I2C1SEL_1 << (i2c->instance * 2)))
    {
        stm32l0_system_hsi16_enable();
    }
    else
    {
        stm32l0_system_lock(STM32L0_SYSTEM_LOCK_CLOCKS);
    }

    if (enable)
    {
        I2C->CR1 |= I2C_CR1_PE;
    }
}

static void stm32l0_i2c_stop(stm32l0_i2c_t *i2c)
{
    I2C_TypeDef *I2C = i2c->I2C;

    /* WAR for ERRATA 2.6.1 */
    I2C->CR1 &= ~I2C_CR1_PE;

    if (RCC->CCIPR & (RCC_CCIPR_I2C1SEL_1 << (i2c->instance * 2)))
    {
        stm32l0_system_hsi16_disable();
    }
    else
    {
        stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_CLOCKS);
    }

    stm32l0_system_periph_disable(STM32L0_SYSTEM_PERIPH_I2C1 + i2c->instance);
}

static void stm32l0_i2c_master_transmit(stm32l0_i2c_t *i2c)
{
    I2C_TypeDef *I2C = i2c->I2C;
    uint32_t i2c_cr2, count;

    i2c->state = STM32L0_I2C_STATE_MASTER_TRANSMIT;

    i2c->xf_count = 0;

    count = i2c->tx_data_e - i2c->tx_data;

    i2c_cr2 = (i2c->xf_address << 1) | I2C_CR2_START;

    if (i2c->tx2_data || (count > I2C_CR2_NBYTES_MAX))
    {
        if (count > I2C_CR2_NBYTES_MAX)
        {
            count = I2C_CR2_NBYTES_MAX;
        }

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

        i2c->xf_count++;
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

    if (count > I2C_CR2_NBYTES_MAX)
    {
        count = I2C_CR2_NBYTES_MAX;

        i2c_cr2 |= I2C_CR2_RELOAD;
    }
    else
    {
        if (!(i2c->xf_control & STM32L0_I2C_CONTROL_RESTART))
        {
            i2c_cr2 |= I2C_CR2_AUTOEND;
        }
    }

    if ((count >= 1) && (i2c->rx_dma == stm32l0_dma_channel(i2c->rx_dma)))
    {
        I2C->CR1 |= I2C_CR1_RXDMAEN;

        stm32l0_dma_start(i2c->rx_dma, (uint32_t)i2c->rx_data, (uint32_t)&I2C->RXDR, (i2c->rx_data_e - i2c->rx_data), STM32L0_I2C_RX_DMA_OPTION);

        I2C->CR2 = (i2c_cr2 | (count << I2C_CR2_NBYTES_SHIFT));

        I2C->CR1 |= (I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_TCIE);

        i2c->rx_data += count;
    }
    else
    {
        I2C->CR2 = (i2c_cr2 | I2C_CR2_START | (count << I2C_CR2_NBYTES_SHIFT));

        I2C->CR1 |= (I2C_CR1_RXIE | I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_TCIE);
    }
}

static void stm32l0_i2c_master_transaction(stm32l0_i2c_t *i2c)
{
    stm32l0_i2c_transaction_t *queue, *transaction, **pp_transaction;

    if (i2c->xf_continue)
    {
        transaction = i2c->xf_continue;
        i2c->xf_continue = NULL;
    }
    else
    {
        transaction = NULL;

        if (!(i2c->xf_control & STM32L0_I2C_CONTROL_RESTART))
        {
            do
            {
                queue = i2c->xf_queue;

                if (!queue)
                {
                    break;
                }

                for (pp_transaction = NULL, transaction = queue; transaction->next; pp_transaction = &transaction->next, transaction = transaction->next) 
                {
                }
                
                if (pp_transaction)
                {
                    *pp_transaction = NULL;
                    
                    break;
                }
            }
            while (armv6m_atomic_compare_and_swap((volatile uint32_t*)&i2c->xf_queue, (uint32_t)transaction, (uint32_t)NULL) != (uint32_t)queue);
        }
    }

    if (transaction)
    {
        if (i2c->state == STM32L0_I2C_STATE_READY)
        {
            stm32l0_system_lock(STM32L0_SYSTEM_LOCK_STOP);

            if (!(i2c->option & STM32L0_I2C_OPTION_ADDRESS_MASK))
            {
                stm32l0_i2c_start(i2c, true);
            }

            if (i2c->rx_dma != STM32L0_DMA_CHANNEL_NONE)
            {
                stm32l0_dma_enable(i2c->rx_dma, NULL, NULL);
            }

            if (i2c->tx_dma != STM32L0_DMA_CHANNEL_NONE)
            {
                stm32l0_dma_enable(i2c->tx_dma, NULL, NULL);
            }
        }

        i2c->xf_callback = transaction->callback;
        i2c->xf_context = transaction->context;
        i2c->xf_status = &transaction->status;
        
        i2c->xf_address = transaction->address;
        i2c->xf_control = transaction->control;
        
        i2c->tx_data = NULL;
        i2c->tx_data_e = NULL;
        i2c->rx_data = NULL;
        i2c->rx_data_e = NULL;
        i2c->tx2_data = NULL;
        i2c->tx2_data_e = NULL;
        
        if (transaction->control & STM32L0_I2C_CONTROL_TX)
        {
            i2c->tx_data = transaction->data;
            i2c->tx_data_e = transaction->data + transaction->count;
            
            if (transaction->control & STM32L0_I2C_CONTROL_RX)
            {
                i2c->rx_data = transaction->data2;
                i2c->rx_data_e = transaction->data2 + transaction->count2;
            }
            else
            {
                if (transaction->control & STM32L0_I2C_CONTROL_TX_SECONDARY)
                {
                    i2c->tx2_data = transaction->data2;
                    i2c->tx2_data_e = transaction->data2 + transaction->count2;
                }
            }
            
            stm32l0_i2c_master_transmit(i2c);
        }
        else
        {
            if (transaction->control & STM32L0_I2C_CONTROL_RX)
            {
                i2c->rx_data = transaction->data;
                i2c->rx_data_e = transaction->data + transaction->count;
            }
            
            stm32l0_i2c_master_receive(i2c);
        }
    }
    else
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
            
            if (!(i2c->option & STM32L0_I2C_OPTION_ADDRESS_MASK))
            {
                stm32l0_i2c_stop(i2c);
            }

            stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_STOP);

            i2c->state = STM32L0_I2C_STATE_READY;
        }
    }
}

static void stm32l0_i2c_slave_transaction(stm32l0_i2c_t *i2c)
{
    I2C_TypeDef *I2C = i2c->I2C;
    uint32_t count;

    i2c->xf_count = 0;

    if (I2C->ISR & I2C_ISR_DIR)
    {
        i2c->state = STM32L0_I2C_STATE_SLAVE_TRANSMIT;

        i2c->tx_data = NULL;
        i2c->tx_data_e = NULL;

        (*i2c->ev_callback)(i2c->ev_context, STM32L0_I2C_EVENT_TRANSMIT_REQUEST);

        if (i2c->tx_data)
        {
            I2C->CR1 |= (I2C_CR1_TXIE | I2C_CR1_STOPIE | I2C_CR1_TCIE);

            I2C->CR2 = 0;
            
            I2C->ISR |= I2C_ISR_TXE;
            
            I2C->TXDR = *(i2c->tx_data)++;
            
            i2c->xf_count++;
        }
        else
        {
            I2C->CR1 |= I2C_CR1_STOPIE;

            I2C->CR2 = I2C_CR2_NACK;

            i2c->state = STM32L0_I2C_STATE_SLAVE_NACK;
        }

        I2C->CR1 &= ~I2C_CR1_SBC;

        I2C->ICR = I2C_ICR_ADDRCF;
    }
    else
    {
        i2c->state = STM32L0_I2C_STATE_SLAVE_RECEIVE;

        i2c->rx_data = NULL;
        i2c->rx_data_e = NULL;

        (*i2c->ev_callback)(i2c->ev_context, STM32L0_I2C_EVENT_RECEIVE_REQUEST);

        if (i2c->rx_data)
        {
            count = i2c->rx_data_e - i2c->rx_data;

            if (count > I2C_CR2_NBYTES_MAX)
            {
                count = I2C_CR2_NBYTES_MAX;
            }
            
            I2C->CR2 = (I2C_CR2_RELOAD | (count << I2C_CR2_NBYTES_SHIFT));
                
            I2C->CR1 |= (I2C_CR1_RXIE | I2C_CR1_STOPIE | I2C_CR1_TCIE);
        }
        else
        {
            I2C->CR1 |= I2C_CR1_STOPIE;

            I2C->CR2 = I2C_CR2_NACK;

            i2c->state = STM32L0_I2C_STATE_SLAVE_NACK;
        }

        I2C->CR1 |= I2C_CR1_SBC;

        I2C->ICR = I2C_ICR_ADDRCF;
    }
}

static void stm32l0_i2c_interrupt(stm32l0_i2c_t *i2c)
{
    I2C_TypeDef *I2C = i2c->I2C;
    uint32_t i2c_cr2, count;

    switch (i2c->state) {

    case STM32L0_I2C_STATE_NONE:
    case STM32L0_I2C_STATE_BUSY:
        break;

    case STM32L0_I2C_STATE_READY:
        if (I2C->ISR & I2C_ISR_ADDR)
        {
            stm32l0_system_lock(STM32L0_SYSTEM_LOCK_STOP);

            stm32l0_i2c_slave_transaction(i2c);
        }
        break;

    case STM32L0_I2C_STATE_MASTER_STOP:
    case STM32L0_I2C_STATE_MASTER_RESTART:
        break;

    case STM32L0_I2C_STATE_MASTER_NACK:
        if (I2C->ISR & I2C_ISR_STOPF)
        {
            I2C->ICR = I2C_ICR_STOPCF;

            I2C->CR1 &= ~I2C_CR1_STOPIE;

            i2c->state = STM32L0_I2C_STATE_MASTER_STOP;

            i2c->xf_control = 0;

            *i2c->xf_status = ((i2c->xf_count == 0) ? STM32L0_I2C_STATUS_ADDRESS_NACK : STM32L0_I2C_STATUS_DATA_NACK);

            if (i2c->xf_callback)
            {
                (*i2c->xf_callback)(i2c->xf_context);
            }
        }
        break;

    case STM32L0_I2C_STATE_MASTER_TRANSMIT:
        if (I2C->ISR & I2C_ISR_ARLO)
        {
            I2C->ICR = I2C_ICR_ARLOCF | I2C_ICR_NACKCF;

            if (I2C->CR1 & I2C_CR1_TXIE)
            {
                I2C->CR1 &= ~(I2C_CR1_TXIE | I2C_CR1_NACKIE | I2C_CR1_TCIE);
            }
            else
            {
                i2c->xf_count += stm32l0_dma_stop(i2c->tx_dma);

                I2C->CR1 &= ~(I2C_CR1_NACKIE | I2C_CR1_TCIE | I2C_CR1_TXDMAEN);
            }

            i2c->state = STM32L0_I2C_STATE_MASTER_STOP;

            i2c->xf_control = 0;

            *i2c->xf_status = STM32L0_I2C_STATUS_ARBITRATION_LOST;

            if (i2c->xf_callback)
            {
                (*i2c->xf_callback)(i2c->xf_context);
            }
        }
        else
        {
            if (I2C->ISR & I2C_ISR_NACKF)
            {
                I2C->ICR = I2C_ICR_NACKCF;
                
                if (I2C->CR1 & I2C_CR1_TXIE)
                {
                    I2C->CR1 &= ~(I2C_CR1_TXIE | I2C_CR1_NACKIE | I2C_CR1_TCIE);
                }
                else
                {
                    i2c->xf_count += stm32l0_dma_stop(i2c->tx_dma);
                    
                    I2C->CR1 &= ~(I2C_CR1_NACKIE | I2C_CR1_TCIE | I2C_CR1_TXDMAEN);
                }
                
                /* If I2C_TXDR is not empty, there there is an unsent byte.
                 */
                if (!(I2C->ISR & I2C_ISR_TXE))
                {
                    i2c->xf_count--;
                }
                
                I2C->CR1 |= I2C_CR1_STOPIE;
                
                i2c->state = STM32L0_I2C_STATE_MASTER_NACK;
            }
            else
            {
                if (I2C->ISR & I2C_ISR_TXIS)
                {
                    if (I2C->CR1 & I2C_CR1_TXIE)
                    {
                        I2C->TXDR = *(i2c->tx_data)++;

                        i2c->xf_count++;
                    }
                }

                if (I2C->ISR & (I2C_ISR_TCR | I2C_ISR_TC | I2C_ISR_STOPF))
                {
                    if (I2C->ISR & (I2C_ISR_TC | I2C_ISR_STOPF))
                    {
                        if (I2C->CR1 & I2C_CR1_TXIE)
                        {
                            I2C->CR1 &= ~(I2C_CR1_TXIE | I2C_CR1_NACKIE | I2C_CR1_TCIE);
                        }
                        else
                        {
                            i2c->xf_count += stm32l0_dma_stop(i2c->tx_dma);
                            
                            I2C->CR1 &= ~(I2C_CR1_NACKIE | I2C_CR1_TCIE | I2C_CR1_TXDMAEN);
                        }

                        if (I2C->ISR & I2C_ISR_TC)
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
                            *i2c->xf_status = STM32L0_I2C_STATUS_SUCCESS;

                            if (i2c->xf_callback)
                            {
                                (*i2c->xf_callback)(i2c->xf_context);
                            }
                        }
                    }
                    else /* I2C_ISR_TCR */
                    {
                        if (i2c->tx_data == i2c->tx_data_e)
                        {
                            if (!(I2C->CR1 & I2C_CR1_TXIE))
                            {
                                i2c->xf_count += stm32l0_dma_stop(i2c->tx_dma);
                            }

                            i2c->tx_data = i2c->tx2_data;
                            i2c->tx_data_e = i2c->tx2_data_e;
                            
                            i2c->tx2_data = NULL;
                            i2c->tx2_data_e = NULL;
                            
                            if (((i2c->tx_data_e - i2c->tx_data) > 1) && (i2c->tx_dma == stm32l0_dma_channel(i2c->tx_dma)))
                            {
                                I2C->CR1 |= I2C_CR1_TXDMAEN;

                                I2C->CR1 &= ~I2C_CR1_TXIE;
                                
                                stm32l0_dma_start(i2c->tx_dma, (uint32_t)&I2C->TXDR, (uint32_t)i2c->tx_data, (i2c->tx_data_e - i2c->tx_data), STM32L0_I2C_TX_DMA_OPTION);
                            }
                            else
                            {
                                I2C->CR1 &= ~I2C_CR1_TXDMAEN;

                                I2C->CR1 |= I2C_CR1_TXIE;
                            }
                        }
                        
                        i2c_cr2 = (i2c->xf_address << 1);
                        
                        count = i2c->tx_data_e - i2c->tx_data;
                        
                        if (i2c->tx2_data || (count > I2C_CR2_NBYTES_MAX))
                        {
                            if (count > I2C_CR2_NBYTES_MAX)
                            {
                                count = I2C_CR2_NBYTES_MAX;
                            }
                            
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
                                
                                i2c->xf_count++;
                            }
                        }
                    }
                }
            }
        }
        break;

    case STM32L0_I2C_STATE_MASTER_RECEIVE:
        if (I2C->ISR & I2C_ISR_ARLO)
        {
            I2C->ICR = I2C_ICR_ARLOCF | I2C_ICR_NACKCF;

            if (I2C->CR1 & I2C_CR1_RXIE)
            {
                I2C->CR1 &= ~(I2C_CR1_RXIE | I2C_CR1_NACKIE | I2C_CR1_TCIE);
            }
            else
            {
                i2c->xf_count += stm32l0_dma_stop(i2c->rx_dma);

                I2C->CR1 &= ~(I2C_CR1_NACKIE | I2C_CR1_TCIE | I2C_CR1_RXDMAEN);
            }

            i2c->state = STM32L0_I2C_STATE_MASTER_STOP;

            i2c->xf_control = 0;

            *i2c->xf_status = STM32L0_I2C_STATUS_ARBITRATION_LOST;

            if (i2c->xf_callback)
            {
                (*i2c->xf_callback)(i2c->xf_context);
            }
        }
        else
        {
            if (I2C->ISR & I2C_ISR_NACKF)
            {
                I2C->ICR = I2C_ICR_NACKCF;
                
                if (I2C->CR1 & I2C_CR1_RXIE)
                {
                    I2C->CR1 &= ~(I2C_CR1_RXIE | I2C_CR1_NACKIE | I2C_CR1_TCIE);
                }
                else
                {
                    i2c->xf_count += stm32l0_dma_stop(i2c->rx_dma);
                    
                    I2C->CR1 &= ~(I2C_CR1_NACKIE | I2C_CR1_TCIE | I2C_CR1_RXDMAEN);
                }
                
                I2C->CR1 |= I2C_CR1_STOPIE;
                
                i2c->state = STM32L0_I2C_STATE_MASTER_NACK;
            }
            else
            {
                if (I2C->ISR & I2C_ISR_RXNE)
                {
                    if (I2C->CR1 & I2C_CR1_RXIE)
                    {
                        *(i2c->rx_data)++ = I2C->RXDR;

                        i2c->xf_count++;
                    }
                }

                if (I2C->ISR & (I2C_ISR_TCR | I2C_ISR_TC | I2C_ISR_STOPF))
                {
                    if (I2C->ISR & (I2C_ISR_TC | I2C_ISR_STOPF))
                    {
                        if (I2C->CR1 & I2C_CR1_RXIE)
                        {
                            I2C->CR1 &= ~(I2C_CR1_RXIE | I2C_CR1_NACKIE | I2C_CR1_TCIE);
                        }
                        else
                        {
                            i2c->xf_count += stm32l0_dma_stop(i2c->rx_dma);
                            
                            I2C->CR1 &= ~(I2C_CR1_NACKIE | I2C_CR1_TCIE | I2C_CR1_RXDMAEN);
                        }
                        
                        if (I2C->ISR & I2C_ISR_TC)
                        {
                            i2c->state = STM32L0_I2C_STATE_MASTER_RESTART;
                        }
                        else
                        {
                            I2C->ICR = I2C_ICR_STOPCF;

                            i2c->state = STM32L0_I2C_STATE_MASTER_STOP;
                        }

                        *i2c->xf_status = STM32L0_I2C_STATUS_SUCCESS;

                        if (i2c->xf_callback)
                        {
                            (*i2c->xf_callback)(i2c->xf_context);
                        }
                    }
                    else /* I2C_ISR_TCR */
                    {
                        i2c_cr2 = (i2c->xf_address << 1) | I2C_CR2_RD_WRN;

                        count = (i2c->rx_data_e - i2c->rx_data);
                        
                        if (count > I2C_CR2_NBYTES_MAX)
                        {
                            count = I2C_CR2_NBYTES_MAX;
                            
                            i2c_cr2 |= I2C_CR2_RELOAD;
                        }
                        else
                        {
                            if (!(i2c->xf_control & STM32L0_I2C_CONTROL_RESTART))
                            {
                                i2c_cr2 |= I2C_CR2_AUTOEND;
                            }
                        }
                        
                        I2C->CR2 = (i2c_cr2 | (count << I2C_CR2_NBYTES_SHIFT));
                        
                        if (!(I2C->CR1 & I2C_CR1_RXIE))
                        {
                            i2c->rx_data += count;
                        }
                    }
                }
            }
        }
        break;

    case STM32L0_I2C_STATE_SLAVE_NACK:
        if (I2C->ISR & (I2C_ISR_ADDR | I2C_ISR_STOPF))
        {
            I2C->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

            I2C->CR1 &= ~I2C_CR1_STOPIE;

            i2c->state = STM32L0_I2C_STATE_READY;

            if (I2C->ISR & I2C_ISR_ADDR)
            {
                stm32l0_i2c_slave_transaction(i2c);
            }
            else
            {
                stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_STOP);
            }
        }
        break;

    case STM32L0_I2C_STATE_SLAVE_TRANSMIT:
        /* A slave transmit is terminated by a NACK followed by a STOP of the master receiver.
         */

        if (I2C->ISR & (I2C_ISR_ADDR | I2C_ISR_STOPF))
        {
            I2C->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

            I2C->CR1 &= ~(I2C_CR1_TXIE | I2C_CR1_STOPIE | I2C_CR1_TCIE);
            
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
                stm32l0_i2c_slave_transaction(i2c);
            }
            else
            {
                stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_STOP);
            }
        }
        else
        {
            if (I2C->ISR & I2C_ISR_TXIS)
            {
                if (i2c->tx_data)
                {
                    I2C->TXDR = *(i2c->tx_data)++;

                    if (i2c->tx_data == i2c->tx_data_e)
                    {
                        i2c->tx_data = NULL;
                        i2c->tx_data_e = NULL;
                        
                        (*i2c->ev_callback)(i2c->ev_context, STM32L0_I2C_EVENT_TRANSMIT_REQUEST);
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

        if (I2C->ISR & (I2C_ISR_ADDR | I2C_ISR_STOPF))
        {
            I2C->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

            I2C->CR1 &= ~(I2C_CR1_RXIE | I2C_CR1_STOPIE | I2C_CR1_TCIE);
            
            i2c->state = STM32L0_I2C_STATE_READY;
                
            (*i2c->ev_callback)(i2c->ev_context, STM32L0_I2C_EVENT_RECEIVE_DONE | (i2c->xf_count << STM32L0_I2C_EVENT_COUNT_SHIFT));

            if (I2C->ISR & I2C_ISR_ADDR)
            {
                stm32l0_i2c_slave_transaction(i2c);
            }
            else
            {
                stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_STOP);
            }
        }
        else
        {
            if (I2C->ISR & I2C_ISR_RXNE)
            {
                if (i2c->rx_data)
                {
                    *(i2c->rx_data)++ = I2C->RXDR;
                    
                    i2c->xf_count++;
                }
                else
                {
                    I2C->RXDR;
                    
                    I2C->CR2 |= I2C_CR2_NACK;
                }
            }

            if (I2C->ISR & I2C_ISR_TCR)
            {
                if (i2c->rx_data)
                {
                    if (i2c->rx_data == i2c->rx_data_e)
                    {
                        i2c->rx_data = NULL;
                        i2c->rx_data_e = NULL;
                        
                        (*i2c->ev_callback)(i2c->ev_context, STM32L0_I2C_EVENT_RECEIVE_REQUEST);

                        /* If the receive callback passes in no data, but a NACK
                         * then rx_data will be NULL, but rx_data_e will be non-NULL.
                         *
                         * In this case NACK the current (past) data. This is useful if say
                         * the first byte is a command, and that command needed to be
                         * NACKed.
                         */
                        if (!i2c->rx_data && i2c->rx_data_e)
                        {
                            I2C->CR2 |= I2C_CR2_NACK;
                        }
                    }
                }

                if (i2c->rx_data)
                {
                    count = i2c->rx_data_e - i2c->rx_data;
            
                    if (count > I2C_CR2_NBYTES_MAX)
                    {
                        count = I2C_CR2_NBYTES_MAX;
                    }
                        
                    I2C->CR2 = (I2C_CR2_RELOAD | (count << I2C_CR2_NBYTES_SHIFT));
                }
                else
                {
                    I2C->CR2 = (I2C_CR2_RELOAD | (1 << I2C_CR2_NBYTES_SHIFT));
                }
            }
        }
        break;
    }

    /* Flush pending errors (handled and unhandled)
     */
    I2C->ICR = I2C_ICR_ARLOCF | I2C_ICR_BERRCF;

    if ((i2c->state == STM32L0_I2C_STATE_READY) || (i2c->state == STM32L0_I2C_STATE_MASTER_STOP) || (i2c->state == STM32L0_I2C_STATE_MASTER_RESTART))
    {
        stm32l0_i2c_master_transaction(i2c);
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

bool stm32l0_i2c_enable(stm32l0_i2c_t *i2c, uint32_t option, stm32l0_i2c_event_callback_t callback, void *context)
{
    if (i2c->state != STM32L0_I2C_STATE_INIT)
    {
        return false;
    }

    i2c->ev_callback = callback;
    i2c->ev_context = context;

    i2c->state = STM32L0_I2C_STATE_BUSY;

    if (!stm32l0_i2c_configure(i2c, option))
    {
        i2c->state = STM32L0_I2C_STATE_INIT;

        return false;
    }

    NVIC_SetPriority(i2c->interrupt, i2c->priority);
    NVIC_EnableIRQ(i2c->interrupt);

    i2c->state = STM32L0_I2C_STATE_READY;

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

        if (i2c->option & STM32L0_I2C_OPTION_WAKEUP)
        {
            armv6m_atomic_and(&EXTI->IMR, ~stm32l0_i2c_xlate_IMR[i2c->instance]);
        }
    }
    else
    {
        if ((i2c->option & STM32L0_I2C_OPTION_MODE_MASK) == STM32L0_I2C_OPTION_MODE_1000K)
        {
            armv6m_atomic_and(&SYSCFG->CFGR1, ~stm32l0_i2c_xlate_FMP[i2c->instance]);
            
            stm32l0_system_unreference(STM32L0_SYSTEM_REFERENCE_I2C1_FMP << i2c->instance);
        }
        else
        {
            if (i2c->instance == STM32L0_I2C_INSTANCE_I2C2)
            {
                stm32l0_system_unreference(STM32L0_SYSTEM_REFERENCE_I2C2_FM | STM32L0_SYSTEM_REFERENCE_I2C2_SM);
            }
        }
    }

    NVIC_DisableIRQ(i2c->interrupt);

    i2c->state = STM32L0_I2C_STATE_INIT;

    return true;
}

bool stm32l0_i2c_configure(stm32l0_i2c_t *i2c, uint32_t option)
{
    I2C_TypeDef *I2C = i2c->I2C;
    uint32_t i2c_cr1, i2c_cr2, i2c_oar1, i2c_oar2, sysclk, pclk, pin_scl, pin_sda;

    if ((i2c->state != STM32L0_I2C_STATE_BUSY) && (i2c->state != STM32L0_I2C_STATE_READY))
    {
        return false;
    }

    if (option & STM32L0_I2C_OPTION_ADDRESS_MASK)
    {
        if (i2c->instance == STM32L0_I2C_INSTANCE_I2C2)
        {
            return false;
        }

        if ((option & STM32L0_I2C_OPTION_MODE_MASK) == STM32L0_I2C_OPTION_MODE_1000K)
        {
            return false;
        }
    }
    else
    {
        sysclk = stm32l0_system_sysclk();
        pclk = stm32l0_system_pclk1();

        if (i2c->instance == STM32L0_I2C_INSTANCE_I2C2)
        {
            if (((option & STM32L0_I2C_OPTION_MODE_MASK) == STM32L0_I2C_OPTION_MODE_1000K) && (pclk < 32000000))
            {
                return false;
            }

            if (((option & STM32L0_I2C_OPTION_MODE_MASK) == STM32L0_I2C_OPTION_MODE_400K) && (pclk < 16000000))
            {
                return false;
            }

            if (((option & STM32L0_I2C_OPTION_MODE_MASK) == STM32L0_I2C_OPTION_MODE_100K) && (pclk < 4000000))
            {
                return false;
            }
        }
        else
        {
            if (((option & STM32L0_I2C_OPTION_MODE_MASK) == STM32L0_I2C_OPTION_MODE_1000K) && (sysclk < 32000000))
            {
                return false;
            }
        }

        option &= ~STM32L0_I2C_OPTION_WAKEUP;
    }

    if (i2c->state == STM32L0_I2C_STATE_READY)
    {
        if (i2c->option & STM32L0_I2C_OPTION_ADDRESS_MASK)
        {
            stm32l0_i2c_stop(i2c);

            if (i2c->option & STM32L0_I2C_OPTION_WAKEUP)
            {
                armv6m_atomic_and(&EXTI->IMR, ~stm32l0_i2c_xlate_IMR[i2c->instance]);
            }
        }
        else
        {
            if ((i2c->option & STM32L0_I2C_OPTION_MODE_MASK) == STM32L0_I2C_OPTION_MODE_1000K)
            {
                armv6m_atomic_and(&SYSCFG->CFGR1, ~stm32l0_i2c_xlate_FMP[i2c->instance]);
            
                stm32l0_system_unreference(STM32L0_SYSTEM_REFERENCE_I2C1_FMP << i2c->instance);
            }
            else
            {
                if (i2c->instance == STM32L0_I2C_INSTANCE_I2C2)
                {
                    stm32l0_system_unreference(STM32L0_SYSTEM_REFERENCE_I2C2_FM | STM32L0_SYSTEM_REFERENCE_I2C2_SM);
                }
            }
        }
    }

    i2c->sysclk = ~0l;
    i2c->pclk = ~0l;
    i2c->option = option;

    stm32l0_i2c_start(i2c, false);
        
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

        /* WUPEN is left enabled for I2C slaves, so that the SDA/SCL lines
         * are aways release upon entering STOP mode. However only the
         * I2C_OPTION_WAKEUP decides whether the GPIOs are connected in
         * STOP mode or not (and hence can really wakeup).
         */
        i2c_cr1 |= (I2C_CR1_ADDRIE | I2C_CR1_WUPEN);

        if (i2c->option & STM32L0_I2C_OPTION_WAKEUP)
        {
            armv6m_atomic_or(&EXTI->IMR, stm32l0_i2c_xlate_IMR[i2c->instance]);
        }
    }
    else
    {
        if ((i2c->option & STM32L0_I2C_OPTION_MODE_MASK) == STM32L0_I2C_OPTION_MODE_1000K)
        {
            armv6m_atomic_or(&SYSCFG->CFGR1, stm32l0_i2c_xlate_FMP[i2c->instance]);
            
            stm32l0_system_reference(STM32L0_SYSTEM_REFERENCE_I2C1_FMP << i2c->instance);
        }
        else
        {
            if (i2c->instance == STM32L0_I2C_INSTANCE_I2C2)
            {
                stm32l0_system_reference(((i2c->option & STM32L0_I2C_OPTION_MODE_MASK) == STM32L0_I2C_OPTION_MODE_400K) ? STM32L0_SYSTEM_REFERENCE_I2C2_FM : STM32L0_SYSTEM_REFERENCE_I2C2_SM);
            }
        }
    }

    I2C->OAR2 = i2c_oar2;
    I2C->OAR1 = i2c_oar1;
    I2C->CR2 = i2c_cr2;
    I2C->CR1 = i2c_cr1;

    pin_scl = i2c->pins.scl;
    pin_sda = i2c->pins.sda;

    if (i2c->option & STM32L0_I2C_OPTION_WAKEUP)
    {
        stm32l0_gpio_pin_configure(pin_scl, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_ALTERNATE));
        stm32l0_gpio_pin_configure(pin_sda, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_ALTERNATE));
    }
    else
    {
        stm32l0_gpio_pin_configure(pin_scl, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_ALTERNATE));
        stm32l0_gpio_pin_configure(pin_sda, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_ALTERNATE));
    }

    if (i2c->option & STM32L0_I2C_OPTION_ADDRESS_MASK) 
    {
        I2C->CR1 |= I2C_CR1_PE;
    }
    else
    {
        if ((i2c->state == STM32L0_I2C_STATE_BUSY) && !(i2c->option & STM32L0_I2C_OPTION_NORESET))
        {
            stm32l0_i2c_reset(i2c);
        }

        stm32l0_i2c_stop(i2c);
    }

    return true;
}

bool stm32l0_i2c_reset(stm32l0_i2c_t *i2c)
{
    uint32_t pin_scl, pin_sda, count;

    if ((i2c->state != STM32L0_I2C_STATE_BUSY) && (i2c->state != STM32L0_I2C_STATE_READY))
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

    if (i2c->option & STM32L0_I2C_OPTION_WAKEUP)
    {
        stm32l0_gpio_pin_configure(pin_scl, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_ALTERNATE));
        stm32l0_gpio_pin_configure(pin_sda, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_ALTERNATE));
    }
    else
    {
        stm32l0_gpio_pin_configure(pin_scl, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_ALTERNATE));
        stm32l0_gpio_pin_configure(pin_sda, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_ALTERNATE));
    }

    return true;
}

bool stm32l0_i2c_scan(stm32l0_i2c_t *i2c, uint16_t address)
{
    uint32_t pin_scl, pin_sda;
    uint8_t data, mask;
    bool ack;

    if (i2c->state != STM32L0_I2C_STATE_READY)
    {
        return false;
    }

    data = ((address & 0x7f) << 1) | 0x00;

    pin_scl = i2c->pins.scl;
    pin_sda = i2c->pins.sda;

    stm32l0_gpio_pin_configure(pin_scl, (STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_INPUT));
    stm32l0_gpio_pin_configure(pin_sda, (STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_INPUT));

    // Send a START condition (SCL = 1, SDA = 1 -> 0)
    stm32l0_gpio_pin_write(pin_sda, 1);
    stm32l0_gpio_pin_write(pin_scl, 1);
    stm32l0_gpio_pin_output(pin_sda);
    stm32l0_gpio_pin_output(pin_scl);
    armv6m_core_udelay(1);    

    stm32l0_gpio_pin_write(pin_sda, 0);
    armv6m_core_udelay(2);    

    stm32l0_gpio_pin_write(pin_scl, 0);

    // Send address byte
    for (mask = 0x80; mask; mask >>= 1)
    {
        armv6m_core_udelay(1);    
        stm32l0_gpio_pin_write(pin_sda, ((data & mask) ? 1 : 0));
        armv6m_core_udelay(3);    
        stm32l0_gpio_pin_write(pin_scl, 1);
        armv6m_core_udelay(4);    
        stm32l0_gpio_pin_write(pin_scl, 0);
    }

    // Check on ACK
    armv6m_core_udelay(1);    
    stm32l0_gpio_pin_input(pin_sda);
    armv6m_core_udelay(3);    
    stm32l0_gpio_pin_write(pin_scl, 1);
    armv6m_core_udelay(2);    

    ack = (stm32l0_gpio_pin_read(pin_sda) == 0);

    armv6m_core_udelay(2);    
    stm32l0_gpio_pin_write(pin_scl, 0);
    armv6m_core_udelay(1);    
    stm32l0_gpio_pin_output(pin_sda);

    // Send a STOP condition (SCL = 1, SDA = 0 -> 1)
    stm32l0_gpio_pin_write(pin_sda, 0);
    armv6m_core_udelay(3);    
    stm32l0_gpio_pin_write(pin_scl, 1);
    armv6m_core_udelay(2);    
    stm32l0_gpio_pin_write(pin_sda, 1);
    armv6m_core_udelay(40);    
    
    if (i2c->option & STM32L0_I2C_OPTION_WAKEUP)
    {
        stm32l0_gpio_pin_configure(pin_scl, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_ALTERNATE));
        stm32l0_gpio_pin_configure(pin_sda, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_ALTERNATE));
    }
    else
    {
        stm32l0_gpio_pin_configure(pin_scl, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_ALTERNATE));
        stm32l0_gpio_pin_configure(pin_sda, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_ALTERNATE));
    }

    return ack;
}

bool stm32l0_i2c_receive(stm32l0_i2c_t *i2c, uint8_t *rx_data, uint16_t rx_count, bool nack)
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
    else
    {
        if (nack)
        {
            i2c->rx_data_e = (uint8_t*)1;
        }
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

bool stm32l0_i2c_enqueue(stm32l0_i2c_t *i2c, stm32l0_i2c_transaction_t *transaction)
{
    stm32l0_i2c_transaction_t *queue;

    if (i2c->state < STM32L0_I2C_STATE_READY)
    {
        return false;
    }

    transaction->status = STM32L0_I2C_STATUS_BUSY;

    if (transaction->control & STM32L0_I2C_CONTROL_CONTINUE)
    {
        if (!(i2c->xf_control & STM32L0_I2C_CONTROL_RESTART) || (i2c->xf_address != transaction->address) || i2c->xf_continue)
        {
            return false;
        }

        transaction->next = NULL;

        i2c->xf_continue = transaction;
    }
    else
    {
        do
        {
            queue = i2c->xf_queue;
            transaction->next = queue;
        }
        while (armv6m_atomic_compare_and_swap((volatile uint32_t*)&i2c->xf_queue, (uint32_t)queue, (uint32_t)transaction) != (uint32_t)queue);
    }

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

