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
#include "stm32l0_uart.h"
#include "stm32l0_dma.h"
#include "stm32l0_exti.h"
#include "stm32l0_system.h"

extern void USART1_IRQHandler(void);
extern void USART2_IRQHandler(void);
#if defined(STM32L072xx) || defined(STM32L082xx)
extern void USART4_5_IRQHandler(void);
#endif /* STM32L072xx || STM32L082xx */
#if defined(STM32L082xx)
extern void AES_RNG_LPUART1_IRQHandler(void);
#else /* STM32L082xx */
extern void RNG_LPUART1_IRQHandler(void);
#endif /* STM32L082xx */

typedef struct _stm32l0_uart_device_t {
    stm32l0_system_notify_t notify;
    volatile uint32_t       wakeup;
    stm32l0_uart_t          *instances[STM32L0_UART_INSTANCE_COUNT];
} stm32l0_uart_device_t;

static stm32l0_uart_device_t stm32l0_uart_device;

#define UART_DATA_XON  0x11
#define UART_DATA_XOFF 0x13

#define STM32L0_UART_RX_DMA_OPTION               \
    (STM32L0_DMA_OPTION_EVENT_TRANSFER_DONE |    \
     STM32L0_DMA_OPTION_EVENT_TRANSFER_HALF |    \
     STM32L0_DMA_OPTION_CIRCULAR |               \
     STM32L0_DMA_OPTION_PERIPHERAL_TO_MEMORY |   \
     STM32L0_DMA_OPTION_PERIPHERAL_DATA_SIZE_8 | \
     STM32L0_DMA_OPTION_MEMORY_DATA_SIZE_8 |     \
     STM32L0_DMA_OPTION_MEMORY_DATA_INCREMENT |  \
     STM32L0_DMA_OPTION_PRIORITY_MEDIUM)

#define STM32L0_UART_TX_DMA_OPTION               \
    (STM32L0_DMA_OPTION_MEMORY_TO_PERIPHERAL |   \
     STM32L0_DMA_OPTION_PERIPHERAL_DATA_SIZE_8 | \
     STM32L0_DMA_OPTION_MEMORY_DATA_SIZE_8 |     \
     STM32L0_DMA_OPTION_MEMORY_DATA_INCREMENT |  \
     STM32L0_DMA_OPTION_PRIORITY_MEDIUM)

static USART_TypeDef * const stm32l0_uart_xlate_USART[STM32L0_UART_INSTANCE_COUNT] = {
    USART1,
    USART2,
#if defined(STM32L072xx) || defined(STM32L082xx)
    USART4,
    USART5,
#endif /* STM32L072xx || STM32L082xx */
    LPUART1,
};

static const IRQn_Type stm32l0_uart_xlate_IRQn[STM32L0_UART_INSTANCE_COUNT] = {
    USART1_IRQn,
    USART2_IRQn,
#if defined(STM32L072xx) || defined(STM32L082xx)
    USART4_5_IRQn,
    USART4_5_IRQn,
#endif /* STM32L072xx || STM32L082xx */
#if defined(STM32L082xx)
    AES_RNG_LPUART1_IRQn,
#else /* STM32L082xx */
    RNG_LPUART1_IRQn,
#endif /* STM32L082xx */
};

static const uint32_t stm32l0_uart_xlate_IMR[STM32L0_UART_INSTANCE_COUNT] = {
    EXTI_IMR_IM25,
    EXTI_IMR_IM26,
#if defined(STM32L072xx) || defined(STM32L082xx)
    0,
    0,
#endif /* STM32L072xx || STM32L082xx */
    EXTI_IMR_IM28,
};

static uint32_t stm32l0_uart_dma_receive(stm32l0_uart_t *uart, uint32_t count)
{
    uint32_t events, rx_index, rx_count, rx_size, rx_write, rx_entries;
    uint8_t rx_data;

    events = 0;

    rx_index = uart->rx_index;
    uart->rx_index = count;

    if (rx_index != uart->rx_index)
    {
        rx_size = (uart->rx_size - uart->rx_count);
        rx_count = 0;
        
        rx_write = uart->rx_write;

        if (uart->option & STM32L0_UART_OPTION_XONOFF)
        {
            do
            {
                rx_data = uart->rx_fifo[rx_index];

                /* XON == 0x11, XOFF == 0x13, hence only bit 1 differs ...
                 */
                if ((rx_data | 0x02) == 0x13)
                {
                    uart->rx_xonoff = rx_data;
                }
                else
                {
                    if (rx_count == rx_size)
                    {
                        events |= STM32L0_UART_EVENT_OVERRUN;
                    }
                    else
                    {
                        uart->rx_data[rx_write] = rx_data;
                        
                        rx_write++;
                        
                        if (rx_write == uart->rx_size)
                        {
                            rx_write = 0;
                        }
                        
                        rx_count++;
                    }
                }

                rx_index++;

                if (rx_index == uart->rx_entries)
                {
                    rx_index = 0;
                }
            }
            while (rx_index != uart->rx_index);
        }
        else
        {
            do
            {
                rx_data = uart->rx_fifo[rx_index];

                if (rx_count == rx_size)
                {
                    events |= STM32L0_UART_EVENT_OVERRUN;
                }
                else
                {
                    uart->rx_data[rx_write] = rx_data;
                
                    rx_write++;
                
                    if (rx_write == uart->rx_size)
                    {
                        rx_write = 0;
                    }
                
                    rx_count++;
                }
            
                rx_index++;

                if (rx_index == uart->rx_entries)
                {
                    rx_index = 0;
                }
            }
            while (rx_index != uart->rx_index);
        }

        uart->rx_write = rx_write;

        rx_entries = armv6m_atomic_add(&uart->rx_count, rx_count);
        
        if (rx_count && uart->rx_event)
        {
	    uart->rx_event = false;

            events |= STM32L0_UART_EVENT_RECEIVE;
        }

        if (uart->option & (STM32L0_UART_OPTION_RTS | STM32L0_UART_OPTION_XONOFF))
        {
            if ((uart->rx_count >= uart->rx_threshold) && (rx_entries < uart->rx_threshold))
            {
                if (uart->option & STM32L0_UART_OPTION_RTS)
                {
                    stm32l0_gpio_pin_write(uart->pins.rts, 1);
                    
                    /* Check for a async uart read race condition.
                     */
                    if ((uart->rx_count < uart->rx_threshold) && uart->rx_enable)
                    {
                        stm32l0_gpio_pin_write(uart->pins.rts, 0);
                    }
                }

                if (uart->option & STM32L0_UART_OPTION_XONOFF)
                {
                    uart->tx_xonoff = UART_DATA_XOFF;
                }
            }
        }
    }

    return events;
}

static void stm32l0_uart_stop_enter(stm32l0_uart_t *uart)
{
    USART_TypeDef *USART = uart->USART;
    uint32_t events, count;

    USART->CR3 &= ~(USART_CR3_DMAR | USART_CR3_DDRE);

    /* Check where by chance some new data entered via DMA,
     * before STOP could be entered. If so, drain and
     * record the proper rx_sequence to avoid enter STOP
     * again, till the next IDLE character zips by.
     */
    count = stm32l0_dma_stop(uart->rx_dma);

    if (uart->rx_index != count)
    {
        events = stm32l0_uart_dma_receive(uart, count);

        if (events)
        {
            (*uart->ev_callback)(uart->ev_context, events);
        }

        if (!uart->rx_sequence)
        {
            uart->rx_sequence = 1;
        
            stm32l0_system_lock(STM32L0_SYSTEM_LOCK_STOP);
        }
    }
}

static void stm32l0_uart_stop_leave(stm32l0_uart_t *uart)
{
    USART_TypeDef *USART = uart->USART;

    /* This code below assume that RXNE will be set in the ISR
     * waking up either from STOP, or receiving the first
     * character. In both cases RXNEIE will be set. If we manage
     * to turn on DMA before that, everything is fine, and the
     * rx_sequence will be handled normally. If this UART woke up
     * from STOP, then the first character will be in RDR. Since 
     * DMA will interrupt only every <N> characters, the UART
     * interrupt handler will drain RDR before DMA enters rx_data[].
     */

    uart->rx_index = 0;

    stm32l0_dma_start(uart->rx_dma, (uint32_t)uart->rx_fifo, (uint32_t)&USART->RDR, uart->rx_entries, STM32L0_UART_RX_DMA_OPTION);
    
    USART->CR3 |= (USART_CR3_DMAR | USART_CR3_DDRE);
}

static void stm32l0_uart_notify_callback(void *context, uint32_t events)
{
    if (stm32l0_uart_device.wakeup)
    {
        if (events & STM32L0_SYSTEM_EVENT_STOP_ENTER)
        {
            if (stm32l0_uart_device.wakeup & (1u << STM32L0_UART_INSTANCE_USART1))
            {
                stm32l0_uart_stop_enter(stm32l0_uart_device.instances[STM32L0_UART_INSTANCE_USART1]);
            }

            if (stm32l0_uart_device.wakeup & (1u << STM32L0_UART_INSTANCE_USART2))
            {
                stm32l0_uart_stop_enter(stm32l0_uart_device.instances[STM32L0_UART_INSTANCE_USART2]);
            }

            if (stm32l0_uart_device.wakeup & (1u << STM32L0_UART_INSTANCE_LPUART1))
            {
                stm32l0_uart_stop_enter(stm32l0_uart_device.instances[STM32L0_UART_INSTANCE_LPUART1]);
            }
        }

        if (events & STM32L0_SYSTEM_EVENT_STOP_LEAVE)
        {
            if (stm32l0_uart_device.wakeup & (1u << STM32L0_UART_INSTANCE_USART1))
            {
                stm32l0_uart_stop_leave(stm32l0_uart_device.instances[STM32L0_UART_INSTANCE_USART1]);
            }

            if (stm32l0_uart_device.wakeup & (1u << STM32L0_UART_INSTANCE_USART2))
            {
                stm32l0_uart_stop_leave(stm32l0_uart_device.instances[STM32L0_UART_INSTANCE_USART2]);
            }

            if (stm32l0_uart_device.wakeup & (1u << STM32L0_UART_INSTANCE_LPUART1))
            {
                stm32l0_uart_stop_leave(stm32l0_uart_device.instances[STM32L0_UART_INSTANCE_LPUART1]);
            }
        }
    }
}

static void stm32l0_uart_dma_callback(stm32l0_uart_t *uart)
{
    uint32_t events;

    events = stm32l0_uart_dma_receive(uart, stm32l0_dma_count(uart->rx_dma));

    if (events)
    {
        (*uart->ev_callback)(uart->ev_context, events);
    }
}

static void stm32l0_uart_start(stm32l0_uart_t *uart)
{
    USART_TypeDef *USART = uart->USART;

    if ((uart->instance == STM32L0_UART_INSTANCE_USART1) || (uart->instance == STM32L0_UART_INSTANCE_USART2))
    {
        stm32l0_system_hsi16_enable();
    }
    
    if ((uart->instance == STM32L0_UART_INSTANCE_LPUART1) && ((RCC->CCIPR & (RCC_CCIPR_LPUART1SEL_1 | RCC_CCIPR_LPUART1SEL_0)) == RCC_CCIPR_LPUART1SEL_1))
    {
        stm32l0_system_hsi16_enable();
    }

    if (uart->rx_dma != STM32L0_DMA_CHANNEL_NONE)
    {
        stm32l0_dma_enable(uart->rx_dma, (stm32l0_dma_callback_t)stm32l0_uart_dma_callback, uart);
    }
    
    if (uart->tx_dma != STM32L0_DMA_CHANNEL_NONE)
    {
        stm32l0_dma_enable(uart->tx_dma, NULL, NULL);
    }

    if (uart->option & STM32L0_UART_OPTION_WAKEUP)
    {
        if (uart->baudrate > 19200)
        {
            stm32l0_system_lock(STM32L0_SYSTEM_LOCK_REGULATOR);
        }
        
        armv6m_atomic_or(&EXTI->IMR, stm32l0_uart_xlate_IMR[uart->instance]);

        /* Pullup needed if peer disconnects. USART cannot detect BREAK as wakeup from STOP.
         */
        stm32l0_gpio_pin_configure(uart->pins.rx, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_ALTERNATE));

        if (uart->option & STM32L0_UART_OPTION_RTS)
        {
            stm32l0_gpio_pin_configure(uart->pins.rts, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
            stm32l0_gpio_pin_write(uart->pins.rts, 0);
        }
    }
    else
    {
        /* Pulldown needed to detect BREAK condition if peer disconnects.
         */
        stm32l0_gpio_pin_configure(uart->pins.rx, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_ALTERNATE));

        if (uart->option & STM32L0_UART_OPTION_RTS)
        {
            stm32l0_gpio_pin_configure(uart->pins.rts, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
            stm32l0_gpio_pin_write(uart->pins.rts, 0);
        }
    }

    USART->CR1 |= USART_CR1_UE;

    USART->ICR = (USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NCF | USART_ICR_ORECF | USART_ICR_IDLECF | USART_ICR_WUCF | USART_ICR_TCCF);

    USART->CR1 |= USART_CR1_RE;
    
    /* Wait for receiver to be enabled and flush partial corrupt input.
     */
    while (!(USART->ISR & USART_ISR_REACK))
    {
    }

    USART->RQR = USART_RQR_RXFRQ;

    /* The receiver always starts within outside STOP lockout zone, so that
     * there will be a spurious IDLE event. Otherwise a disconnected RX
     * line will be a STOP lockout.
     */

    /* Start DMA after the receiver is alive and flushed.
     */
    if (uart->rx_dma == stm32l0_dma_channel(uart->rx_dma))
    {
        uart->rx_index = 0;

        stm32l0_dma_start(uart->rx_dma, (uint32_t)uart->rx_fifo, (uint32_t)&USART->RDR, uart->rx_entries, STM32L0_UART_RX_DMA_OPTION);
        
        USART->CR3 |= (USART_CR3_DMAR | USART_CR3_DDRE);

        if (uart->option & STM32L0_UART_OPTION_WAKEUP)
        {
            armv6m_atomic_or(&stm32l0_uart_device.wakeup, (1u << uart->instance));
        }
    }

    if ((uart->state != STM32L0_UART_STATE_BREAK) && (uart->state != STM32L0_UART_STATE_SUSPENDED_BREAK))
    {
        if (uart->option & STM32L0_UART_OPTION_WAKEUP)
        {
          // stm32l0_gpio_pin_configure(uart->pins.tx, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_ALTERNATE));
          stm32l0_gpio_pin_configure(uart->pins.tx, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_ALTERNATE));
        }
        else
        {
          // stm32l0_gpio_pin_configure(uart->pins.tx, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_ALTERNATE));
          stm32l0_gpio_pin_configure(uart->pins.tx, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_ALTERNATE));
        }
        
        if (uart->option & STM32L0_UART_OPTION_CTS)
        {
            stm32l0_gpio_pin_configure(uart->pins.cts, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_ALTERNATE));
            
            USART->CR3 |= USART_CR3_CTSE;
        }

        USART->CR1 |= USART_CR1_TE;
            
        while (!(USART->ISR & USART_ISR_TEACK))
        {
        }
    }

    USART->CR3 |= USART_CR3_EIE;
    USART->CR1 |= (USART_CR1_PEIE | USART_CR1_IDLEIE | USART_CR1_RXNEIE);
}

static void stm32l0_uart_stop(stm32l0_uart_t *uart)
{
    USART_TypeDef *USART = uart->USART;
    uint32_t events;

    USART->CR1 &= ~(USART_CR1_PEIE | USART_CR1_IDLEIE | USART_CR1_RXNEIE);
    USART->CR3 &= ~USART_CR3_EIE;
    
    if ((uart->state != STM32L0_UART_STATE_BREAK) && (uart->state != STM32L0_UART_STATE_SUSPENDED_BREAK))
    {
        USART->CR1 &= ~USART_CR1_TE;
            
        while (!(USART->ISR & USART_ISR_TEACK))
        {
        }
        
        stm32l0_gpio_pin_configure(uart->pins.tx, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_MODE_ANALOG));
        
        if (uart->option & STM32L0_UART_OPTION_CTS)
        {
            USART->CR3 &= ~USART_CR3_CTSE;
            
            stm32l0_gpio_pin_configure(uart->pins.cts, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_MODE_ANALOG));
        }
    }

    if (USART->CR3 & USART_CR3_DMAR)
    {
        USART->CR3 &= ~(USART_CR3_DMAR | USART_CR3_DDRE);

        events = stm32l0_uart_dma_receive(uart, stm32l0_dma_stop(uart->rx_dma));
        
        if (events)
        {
            (*uart->ev_callback)(uart->ev_context, events);
        }
    }

    USART->CR1 &= ~USART_CR1_RE;

    while (!(USART->ISR & USART_ISR_REACK))
    {
    }
    
    stm32l0_gpio_pin_configure(uart->pins.rx, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_MODE_ANALOG));

    if (uart->option & STM32L0_UART_OPTION_RTS)
    {
        stm32l0_gpio_pin_configure(uart->pins.rts, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_MODE_ANALOG));
    }
        
    USART->CR1 &= ~USART_CR1_UE;
        
    if (uart->option & STM32L0_UART_OPTION_WAKEUP)
    {
        if (uart->baudrate > 9600)
        {
            stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_REGULATOR);
        }
        
        armv6m_atomic_and(&EXTI->IMR, ~stm32l0_uart_xlate_IMR[uart->instance]);
        
        armv6m_atomic_and(&stm32l0_uart_device.wakeup, ~(1u << uart->instance));
    }
        
    if (uart->rx_dma == stm32l0_dma_channel(uart->rx_dma))
    {
        stm32l0_dma_disable(uart->rx_dma);
    }
    
    if (uart->tx_dma == stm32l0_dma_channel(uart->tx_dma))
    {
        stm32l0_dma_disable(uart->tx_dma);
    }

    if ((uart->instance == STM32L0_UART_INSTANCE_USART1) || (uart->instance == STM32L0_UART_INSTANCE_USART2))
    {
        stm32l0_system_hsi16_disable();
    }
    
    if ((uart->instance == STM32L0_UART_INSTANCE_LPUART1) && ((RCC->CCIPR & (RCC_CCIPR_LPUART1SEL_1 | RCC_CCIPR_LPUART1SEL_0)) == RCC_CCIPR_LPUART1SEL_1))
    {
        stm32l0_system_hsi16_disable();
    }

    if (uart->rx_sequence)
    {
        stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_STOP);
        
        uart->rx_sequence = 0;
    }
}

static void stm32l0_uart_exti_callback(stm32l0_uart_t *uart)
{
    stm32l0_exti_detach(uart->pins.rx);
    
    stm32l0_system_periph_enable(STM32L0_SYSTEM_PERIPH_USART1 + uart->instance);

    stm32l0_uart_start(uart);
    
    NVIC_EnableIRQ(uart->interrupt);
    
    uart->state = (uart->state == STM32L0_UART_STATE_SUSPENDED_BREAK) ? STM32L0_UART_STATE_BREAK : STM32L0_UART_STATE_READY;

    if (!uart->rx_sequence)
    {
        uart->rx_sequence = 1;
        
        stm32l0_system_lock(STM32L0_SYSTEM_LOCK_STOP);
    }
    
    (*uart->ev_callback)(uart->ev_context, STM32L0_UART_EVENT_WAKEUP);
}

static void stm32l0_uart_interrupt(stm32l0_uart_t *uart)
{
    USART_TypeDef *USART = uart->USART;
    uint32_t events, rx_write, tx_count;
    uint8_t rx_data;

    events = 0;

    USART->ICR = USART_ICR_WUCF;

    if (USART->ISR & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE))
    {
        if (USART->CR3 & USART_CR3_DMAR)
        {
            events |= stm32l0_uart_dma_receive(uart, stm32l0_dma_count(uart->rx_dma));
        }

        rx_data = USART->RDR;
                
        if (USART->ISR & USART_ISR_NE)
        {
            events |= STM32L0_UART_EVENT_NOISE;
        }
        else
        {
            if (USART->ISR & USART_ISR_FE)
            {
                if (rx_data == 0x00)
                {
                    events |= STM32L0_UART_EVENT_BREAK;
                }
                else
                {
                    events |= STM32L0_UART_EVENT_FRAMING;
                }
            }
            else
            {
                events |= STM32L0_UART_EVENT_PARITY;
            }
        }
        
        USART->ICR = (USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NCF);
    }
    else
    {
        if (USART->ISR & USART_ISR_RXNE)
        {
            rx_data = USART->RDR;
    
            /* XON == 0x11, XOFF == 0x13, hence only bit 1 differs ...
             */
            if ((uart->option & STM32L0_UART_OPTION_XONOFF) && ((rx_data | 0x02) == 0x13))
            {
                uart->rx_xonoff = rx_data;
            }
            else
            {
                if (uart->rx_count == uart->rx_size)
                {
                    events |= STM32L0_UART_EVENT_OVERRUN;
                }
                else
                {
                    rx_write = uart->rx_write;
            
                    uart->rx_data[rx_write] = rx_data;
            
                    rx_write++;
            
                    if (rx_write == uart->rx_size)
                    {
                        rx_write = 0;
                    }
            
                    uart->rx_write = rx_write;
            
                    armv6m_atomic_add(&uart->rx_count, 1);
            
                    if (uart->rx_event)
                    {
			uart->rx_event = false;

                        events |= STM32L0_UART_EVENT_RECEIVE;
                    }
            
                    if (uart->option & (STM32L0_UART_OPTION_RTS | STM32L0_UART_OPTION_XONOFF))
                    {
                        if (uart->rx_count == uart->rx_threshold)
                        {
                            if (uart->option & STM32L0_UART_OPTION_RTS)
                            {
                                stm32l0_gpio_pin_write(uart->pins.rts, 1);
                        
                                /* Check for a async uart read race condition.
                                 */
                                if ((uart->rx_count < uart->rx_threshold) && uart->rx_enable)
                                {
                                    stm32l0_gpio_pin_write(uart->pins.rts, 0);
                                }
                            }
                    
                            if (uart->option & STM32L0_UART_OPTION_XONOFF)
                            {
                                uart->tx_xonoff = UART_DATA_XOFF;
                            }
                        }
                    }
                }
            }
    
            if (!uart->rx_sequence)
            {
                uart->rx_sequence = 1;
        
                stm32l0_system_lock(STM32L0_SYSTEM_LOCK_STOP);
            }
        }

        if (USART->CR3 & USART_CR3_DMAR)
        {
            /* Setting the RXNE flags triggers a DMA, which means
             * by the time we get here the flag is cleared again ...
             * Unless of course we come back from STOP, and the data 
             * entered already RDR ..
             *
             * Hence the first regular data will get here without
             * RXNE set, so rx_sequence needs to be set if RXNEIE
             * was enabled.
             */
            
            if (USART->CR1 & USART_CR1_RXNEIE) 
            {
                if (uart->rx_index != stm32l0_dma_count(uart->rx_dma))
                {
                    USART->CR1 &= ~USART_CR1_RXNEIE;

                    if (!uart->rx_sequence)
                    {
                        uart->rx_sequence = 1;
                        
                        stm32l0_system_lock(STM32L0_SYSTEM_LOCK_STOP);
                    }
                }
            }
        }
    }

    if (USART->ISR & USART_ISR_IDLE)
    {
        if (USART->CR3 & USART_CR3_DMAR)
        {
            events |= stm32l0_uart_dma_receive(uart, stm32l0_dma_count(uart->rx_dma));
        }

        if (uart->rx_sequence)
        {
            stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_STOP);
            
            uart->rx_sequence = 0;

            USART->CR1 |= USART_CR1_RXNEIE;
        }

        USART->ICR = USART_ICR_IDLECF;
    }

    if (USART->ISR & USART_ISR_ORE)
    {
        events |= STM32L0_UART_EVENT_OVERRUN;
        
        USART->ICR = USART_ICR_ORECF;
    }

    if (events)
    {
        (*uart->ev_callback)(uart->ev_context, events);
    }

    if (uart->rq_suspend)
    {
        NVIC_DisableIRQ(uart->interrupt);

        stm32l0_uart_stop(uart);

        stm32l0_system_periph_disable(STM32L0_SYSTEM_PERIPH_USART1 + uart->instance);

        uart->state = (uart->state == STM32L0_UART_STATE_BREAK) ? STM32L0_UART_STATE_SUSPENDED_BREAK : STM32L0_UART_STATE_SUSPENDED;
        
        uart->rq_suspend = 0;
        uart->rq_break = 0;

        stm32l0_gpio_pin_configure(uart->pins.rx, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));

        stm32l0_exti_attach(uart->pins.rx, STM32L0_EXTI_CONTROL_EDGE_FALLING | STM32L0_EXTI_CONTROL_EDGE_RISING, (stm32l0_exti_callback_t)stm32l0_uart_exti_callback, uart);
    }

    if (uart->rq_break)
    {
        USART->CR1 &= ~USART_CR1_TE;
        
        while (!(USART->ISR & USART_ISR_TEACK))
        {
        }
        
        stm32l0_gpio_pin_configure(uart->pins.tx, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_MODE_ANALOG));
        
        if (uart->option & STM32L0_UART_OPTION_CTS)
        {
            USART->CR3 &= ~USART_CR3_CTSE;
            
            stm32l0_gpio_pin_configure(uart->pins.cts, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_MODE_ANALOG));
        }
        
        uart->state = STM32L0_UART_STATE_BREAK;

        uart->rq_break = 0;
    }

    if (uart->rq_inject)
    {
        if ((uart->rx_count < uart->rx_threshold) && uart->rx_enable)
        {
            uart->tx_xonoff = UART_DATA_XON;
        }
        else
        {
            uart->tx_xonoff = UART_DATA_XOFF;
        }

        uart->rq_inject = 0;
    }

    if (uart->state == STM32L0_UART_STATE_TRANSMIT)
    {
        if (USART->ISR & USART_ISR_TC)
        {
            USART->ICR = USART_ICR_TCCF;
            
            USART->CR1 &= ~USART_CR1_TCIE;
            
            if (USART->CR3 & USART_CR3_DMAT)
            {
                USART->CR3 &= ~USART_CR3_DMAT;
                
                tx_count = stm32l0_dma_stop(uart->tx_dma);
                
                uart->tx_data += tx_count;
                uart->tx_count -= tx_count;
            }

            uart->tx_data = NULL;
                
            uart->state = STM32L0_UART_STATE_READY; 
                    
            stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_STOP);

            if (uart->tx_callback)
            {
                (*uart->tx_callback)(uart->tx_context);
            }
        }

        if (uart->tx_xonoff)
        {
            if (uart->state == STM32L0_UART_STATE_TRANSMIT)
            {
                if (USART->CR3 & USART_CR3_DMAT)
                {
                    USART->CR3 &= ~USART_CR3_DMAT;
                    
                    tx_count = stm32l0_dma_stop(uart->tx_dma);
                    
                    uart->tx_data += tx_count;
                    uart->tx_count -= tx_count;
                }
                
                USART->CR1 &= ~USART_CR1_TXEIE;
                USART->CR1 |= USART_CR1_TCIE;

                uart->state = STM32L0_UART_STATE_INJECT; 
            }
            else
            {
                USART->CR1 |= USART_CR1_TCIE;
                
                USART->TDR = uart->tx_xonoff;
                
                uart->tx_xonoff = 0x00;
                
                uart->state = STM32L0_UART_STATE_WAIT;

                stm32l0_system_lock(STM32L0_SYSTEM_LOCK_STOP);
            }
        }
        else
        {
            if (uart->tx_data && (uart->rx_xonoff == UART_DATA_XOFF))
            {
                if (USART->CR3 & USART_CR3_DMAT)
                {
                    USART->CR3 &= ~USART_CR3_DMAT;
                    
                    tx_count = stm32l0_dma_stop(uart->tx_dma);
                    
                    uart->tx_data += tx_count;
                    uart->tx_count -= tx_count;
                }
                
                USART->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_TCIE);
                
                uart->state = STM32L0_UART_STATE_READY; 
                
                stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_STOP);
            }
            else
            {
                if (USART->CR1 & USART_CR1_TXEIE)
                {
                    if (USART->ISR & USART_ISR_TXE)
                    {
                        uart->tx_count--;
                        
                        if (uart->tx_count == 0)
                        {
                            USART->CR1 = (USART->CR1 & ~USART_CR1_TXEIE) | USART_CR1_TCIE;
                        }
                        
                        USART->TDR = *uart->tx_data++;
                    }
                }
            }
        }
    }

    if (uart->state == STM32L0_UART_STATE_INJECT)
    {
        if (USART->ISR & USART_ISR_TC)
        {
            USART->ICR = USART_ICR_TCCF;

            USART->TDR = uart->tx_xonoff;

            uart->tx_xonoff = 0x00;

            uart->state = STM32L0_UART_STATE_WAIT;
        }
    }

    if (uart->state == STM32L0_UART_STATE_WAIT)
    {
        if (USART->ISR & USART_ISR_TC)
        {
            USART->ICR = USART_ICR_TCCF;

            if (uart->tx_xonoff)
            {
                USART->TDR = uart->tx_xonoff;
                
                uart->tx_xonoff = 0x00;
                
                uart->state = STM32L0_UART_STATE_WAIT;
            }
            else
            {
                USART->CR1 &= ~USART_CR1_TCIE;

                uart->state = STM32L0_UART_STATE_READY;

                stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_STOP);
            }
        }
    }

    if (uart->state == STM32L0_UART_STATE_READY)
    {
        if (uart->tx_xonoff)
        {
            USART->CR1 |= USART_CR1_TCIE;
                
            USART->TDR = uart->tx_xonoff;
                
            uart->tx_xonoff = 0x00;
            
            uart->state = STM32L0_UART_STATE_WAIT;

            stm32l0_system_lock(STM32L0_SYSTEM_LOCK_STOP);
        }
        else
        {
            if (uart->tx_data && (uart->rx_xonoff == UART_DATA_XON))
            {
                stm32l0_system_lock(STM32L0_SYSTEM_LOCK_STOP);

                if (uart->tx_dma == stm32l0_dma_channel(uart->tx_dma))
                {
                    USART->CR3 |= USART_CR3_DMAT;
                    
                    USART->CR1 |= USART_CR1_TCIE;
                    
                    stm32l0_dma_start(uart->tx_dma, (uint32_t)&USART->TDR, (uint32_t)uart->tx_data, uart->tx_count, STM32L0_UART_TX_DMA_OPTION);
                }
                else
                {
                    uart->tx_count--;
                    
                    if (uart->tx_count == 0)
                    {
                        USART->CR1 |= USART_CR1_TCIE;
                    }
                    else
                    {
                        USART->CR1 |= USART_CR1_TXEIE;
                    }
                    
                    USART->TDR = *uart->tx_data++;
                }

                uart->state = STM32L0_UART_STATE_TRANSMIT;
            }
        }
    }
}

bool stm32l0_uart_create(stm32l0_uart_t *uart, const stm32l0_uart_params_t *params)
{
    if (params->instance >= STM32L0_UART_INSTANCE_COUNT)
    {
        return false;
    }

    uart->USART = stm32l0_uart_xlate_USART[params->instance];
    uart->instance = params->instance;
    uart->interrupt = stm32l0_uart_xlate_IRQn[params->instance];
    uart->priority = params->priority;
    uart->rx_dma = params->rx_dma;
    uart->tx_dma = params->tx_dma;
    uart->pins = params->pins;

    uart->rx_fifo = params->rx_fifo;
    uart->rx_entries = params->rx_entries;

    stm32l0_uart_device.instances[uart->instance] = uart;

    uart->state = STM32L0_UART_STATE_INIT;

    if (!stm32l0_uart_device.notify.callback)
    {
        stm32l0_system_notify(&stm32l0_uart_device.notify, stm32l0_uart_notify_callback, NULL, (STM32L0_SYSTEM_EVENT_STOP_ENTER | STM32L0_SYSTEM_EVENT_STOP_LEAVE));
    }

    return true;
}

bool stm32l0_uart_destroy(stm32l0_uart_t *uart)
{
    if (uart->state != STM32L0_UART_STATE_INIT)
    {
        return false;
    }

    uart->state = STM32L0_UART_STATE_NONE;

    stm32l0_uart_device.instances[uart->instance] = NULL;

    return true;
}


bool stm32l0_uart_enable(stm32l0_uart_t *uart, uint8_t *rx_data, uint32_t rx_size, uint32_t baudrate, uint32_t option, stm32l0_uart_event_callback_t callback, void *context)
{
    if (uart->state != STM32L0_UART_STATE_INIT)
    {
        return false;
    }
    
    if ((rx_data == NULL) || (rx_size < 16))
    {
        return false;
    }

    uart->rx_index = 0;
    uart->rx_threshold = rx_size - (((uart->rx_entries + 1) / 2) + 4); // 4 extra characters plus half the FIFO
    uart->rx_data = rx_data;
    uart->rx_size = rx_size;
    uart->rx_read = 0;
    uart->rx_write = 0;
    uart->rx_count = 0;
    uart->rx_sequence = 0;
    uart->rx_enable = 1;
    uart->rx_xonoff = UART_DATA_XON;
    uart->rx_event = true;

    uart->tx_callback = NULL;
    uart->tx_context = NULL;
    uart->tx_data = NULL;
    uart->tx_count = 0;
    uart->tx_xonoff = 0;
    uart->rq_suspend = 0;
    uart->rq_break = 0;
    uart->rq_inject = 0;

    uart->ev_callback = callback;
    uart->ev_context = context;

    uart->state = STM32L0_UART_STATE_BUSY;

    if (!stm32l0_uart_configure(uart, baudrate, option))
    {
        uart->state = STM32L0_UART_STATE_INIT;

        return false;
    }

    NVIC_SetPriority(uart->interrupt, uart->priority);
    NVIC_EnableIRQ(uart->interrupt);

    uart->state = STM32L0_UART_STATE_READY;

    return true;
}

bool stm32l0_uart_disable(stm32l0_uart_t *uart)
{
    if ((uart->state != STM32L0_UART_STATE_READY) && (uart->state != STM32L0_UART_STATE_BREAK))
    {
        return false;
    }

    NVIC_DisableIRQ(uart->interrupt);

    stm32l0_uart_stop(uart);

    stm32l0_system_periph_disable(STM32L0_SYSTEM_PERIPH_USART1 + uart->instance);

    uart->state = STM32L0_UART_STATE_INIT;

    return true;
}

bool stm32l0_uart_configure(stm32l0_uart_t *uart, uint32_t baudrate, uint32_t option)
{
    USART_TypeDef *USART = uart->USART;
    uint32_t usart_cr1, usart_cr2, usart_cr3, usart_brr;

    if ((uart->state != STM32L0_UART_STATE_READY) && (uart->state != STM32L0_UART_STATE_BUSY))
    {
        return false;
    }

    if ((baudrate == 0) || (baudrate > 1000000))
    {
        return false;
    }

#if defined(STM32L072xx) || defined(STM32L082xx)
    if (option & STM32L0_UART_OPTION_WAKEUP)
    {
        if ((uart->instance == STM32L0_UART_INSTANCE_USART4) || (uart->instance == STM32L0_UART_INSTANCE_USART5))
        {
            return false;
        }
    }
#endif /* STM32L072xx || STM32L082xx */

    if (uart->state == STM32L0_UART_STATE_BUSY)
    {
        stm32l0_system_periph_enable(STM32L0_SYSTEM_PERIPH_USART1 + uart->instance);
    }
    else
    {
        stm32l0_uart_stop(uart);
    }

    usart_cr1 = 0;
    usart_cr2 = 0;
    usart_cr3 = 0;

    if (option & STM32L0_UART_OPTION_STOP_2)
    {
        usart_cr2 |= USART_CR2_STOP_1;
    }

    if (option & (STM32L0_UART_OPTION_PARITY_EVEN | STM32L0_UART_OPTION_PARITY_ODD))
    {
        usart_cr1 |= USART_CR1_PCE;
        
        if (option & STM32L0_UART_OPTION_PARITY_ODD)
        {
            usart_cr1 |= USART_CR1_PS;
        }
        
        usart_cr1 |= ((option & STM32L0_UART_OPTION_DATA_SIZE_8) ? USART_CR1_M0 : 0);
    }
    else
    {
        usart_cr1 |= ((option & STM32L0_UART_OPTION_DATA_SIZE_8) ? 0 : USART_CR1_M1);
    }

    if (option & STM32L0_UART_OPTION_WAKEUP)
    {
        /* This really, really requires an explanation, how deal with WAKEUP
         * and STOP.
         *
         * The very first received byte sets uart->rx_sequence and locks out STOP
         * mode. That is handled for ISR based receive as well as DMA based receive.
         * In the latter case the RXNEIE flag will be cleared after the first byte.
         * Later on when the IDLE condition is detected the uart->rx_sequence flag
         * is cleared again, and RXNEIE is set. Hence between the first byte and the 
         * subsequent IDLE condition, the system cannot enter STOP mode. Consequently
         * when entering STOP mode RXNEIE is always set, the USART/LPUART can use
         * RXNE as wakeup flag. 
         *
         * Next, if STOP mode is entered when DMA is active, DMA gets disabled
         * BEFORE entering STOP mode, and DMA gets enabled right AFTER leaving
         * STOP mode. Hence the byte in USART->RDR will be sent directly to
         * DMA and the uart->rx_sequence logic in stm32l0_uart_interrupt()
         * will block STOP mode, till the next IDLE condition.
         *
         * USART/LPUART can wakeup from STOP with the LowPower Voltage Regulator
         * up to 19200 baud. Above that the Main Voltage Regulator has to be used.
         * Hence STM32L0_SYSTEM_LOCK_REGULATOR. USART/LPUART wakeup time is 12uS
         * with PWR_CR_LPSDSR, opposed to 8.7uS without (11.4uS and 8.1uS in 
         * voltage range 1).
         *
         * Above 38400 baud HSI16 cannot wakeup fast enough to guarantee a proper
         * sampled signal. That means that HSI16 has to be kept alive during STOP
         * mode, which is done via USART_CR3_UCESM.
         *
         * For USART1/USART2 to increase the tolerance of the sampled signal,
         * USART_CR3_ONEBIT is used for the data bits.
         */

        usart_cr1 |= USART_CR1_UESM;
        usart_cr3 |= (USART_CR3_WUS_1 | USART_CR3_WUS_0 | USART_CR3_ONEBIT);

        if (baudrate > 38400)
        {
            usart_cr3 |= USART_CR3_UCESM;
        }
    }

    if (option & STM32L0_UART_OPTION_RX_INVERT)
    {
        usart_cr2 |= USART_CR2_RXINV;
    }

    if (option & STM32L0_UART_OPTION_TX_INVERT)
    {
        usart_cr2 |= USART_CR2_TXINV;
    }

    if ((uart->instance == STM32L0_UART_INSTANCE_USART1) || (uart->instance == STM32L0_UART_INSTANCE_USART2))
    {
        usart_brr = ((16000000u + (baudrate >> 1)) / baudrate);              /* HSI16 */

        armv6m_atomic_modify(&RCC->CCIPR, ((RCC_CCIPR_USART1SEL_1 | RCC_CCIPR_USART1SEL_0) << (uart->instance * 2)), ((RCC_CCIPR_USART1SEL_1) << (uart->instance * 2)));
    }
    else if (uart->instance == STM32L0_UART_INSTANCE_LPUART1)
    {
        if (baudrate <= 9600)
        {
            usart_brr = (256u * 32768u + (baudrate >> 1)) / baudrate;        /* LSE */

            armv6m_atomic_modify(&RCC->CCIPR, (RCC_CCIPR_LPUART1SEL_1 | RCC_CCIPR_LPUART1SEL_0), (RCC_CCIPR_LPUART1SEL_1 | RCC_CCIPR_LPUART1SEL_0));
        }
        else
        {
            usart_brr = (256u * 16000000u + (baudrate >> 1)) / baudrate;     /* HSI16 */

            armv6m_atomic_modify(&RCC->CCIPR, (RCC_CCIPR_LPUART1SEL_1 | RCC_CCIPR_LPUART1SEL_0), (RCC_CCIPR_LPUART1SEL_1));
        }
    }
    else
    {
        usart_brr = (stm32l0_system_pclk1() + (baudrate >> 1)) / baudrate;   /* PCLK1 */
    }

    uart->baudrate = baudrate;
    uart->option = option;

    USART->BRR = usart_brr;
    USART->CR3 = usart_cr3;
    USART->CR2 = usart_cr2;
    USART->CR1 = usart_cr1;

    stm32l0_uart_start(uart);

    return true;
}

bool stm32l0_uart_suspend(stm32l0_uart_t *uart)
{
    if (uart->state < STM32L0_UART_STATE_READY)
    {
        return false;
    }

    if (uart->rq_suspend || uart->rq_break || uart->tx_data || (uart->state >= STM32L0_UART_STATE_BREAK))
    {
        return false;
    }
    
    uart->rq_suspend = 1;

    NVIC_SetPendingIRQ(uart->interrupt);

    return true;
}

bool stm32l0_uart_resume(stm32l0_uart_t *uart)
{
    return true;

    if (uart->state < STM32L0_UART_STATE_READY)
    {
        return false;
    }

    uart->rq_suspend = 0;

    stm32l0_exti_detach(uart->pins.rx);
    
    if ((uart->state == STM32L0_UART_STATE_SUSPENDED) || (uart->state == STM32L0_UART_STATE_SUSPENDED_BREAK))
    {
        stm32l0_system_periph_enable(STM32L0_SYSTEM_PERIPH_USART1 + uart->instance);

        stm32l0_uart_start(uart);

        NVIC_EnableIRQ(uart->interrupt);

        uart->state = (uart->state == STM32L0_UART_STATE_SUSPENDED_BREAK) ? STM32L0_UART_STATE_BREAK : STM32L0_UART_STATE_READY;
    }

    return true;
}

bool stm32l0_uart_rts_enable(stm32l0_uart_t *uart, bool onoff)
{
    if (uart->state < STM32L0_UART_STATE_READY)
    {
        return false;
    }

    uart->rx_enable = onoff;

    if (uart->option & STM32L0_UART_OPTION_RTS)
    {
        if (!uart->rx_enable)
        {
            stm32l0_gpio_pin_write(uart->pins.rts, 1);
        }
        else
        {
            if (uart->rx_count < uart->rx_threshold)
            {
                stm32l0_gpio_pin_write(uart->pins.rts, 0);
            
                /* Check for a async uart interrupt race condition.
                 */
                if (uart->rx_count >= uart->rx_threshold)
                {
                    stm32l0_gpio_pin_write(uart->pins.rts, 1);
                }
            }
        }
    }

    if (uart->option & STM32L0_UART_OPTION_XONOFF)
    {
        uart->rq_inject = 1;

        NVIC_SetPendingIRQ(uart->interrupt);
    }

    return true;
}

bool stm32l0_uart_cts_holding(stm32l0_uart_t *uart)
{
    USART_TypeDef *USART = uart->USART;

    if (uart->state < STM32L0_UART_STATE_READY)
    {
        return false;
    }

    if (!(uart->option & STM32L0_UART_OPTION_CTS))
    {
        return true;
    }

    return !!(USART->ISR & USART_ISR_CTS);
}

bool stm32l0_uart_break_state(stm32l0_uart_t *uart, bool onoff)
{
    USART_TypeDef *USART = uart->USART;

    if (uart->state < STM32L0_UART_STATE_READY)
    {
        return false;
    }

    if (onoff)
    {
        if (uart->rq_suspend || uart->rq_break || uart->tx_data || (uart->state >= STM32L0_UART_STATE_BREAK))
        {
            return false;
        }

        uart->rq_break = 1;

        NVIC_SetPendingIRQ(uart->interrupt);
    }
    else
    {
        uart->rq_break = 0;

        if (uart->state == STM32L0_UART_STATE_BREAK)
        {
            if (uart->option & STM32L0_UART_OPTION_WAKEUP)
            {
                // stm32l0_gpio_pin_configure(uart->pins.tx, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_ALTERNATE));
                stm32l0_gpio_pin_configure(uart->pins.tx, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_ALTERNATE));
            }
            else
            {
                // stm32l0_gpio_pin_configure(uart->pins.tx, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_ALTERNATE));
                stm32l0_gpio_pin_configure(uart->pins.tx, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_ALTERNATE));
            }

            if (uart->option & STM32L0_UART_OPTION_CTS)
            {
                stm32l0_gpio_pin_configure(uart->pins.cts, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_ALTERNATE));

                armv6m_atomic_or(&USART->CR3, USART_CR3_CTSE);
            }

            armv6m_atomic_or(&USART->CR1, USART_CR1_TE);
            
            while (!(USART->ISR & USART_ISR_TEACK))
            {
            }
        }
    }

    return true;
}

uint32_t stm32l0_uart_count(stm32l0_uart_t *uart)
{
    if (uart->state < STM32L0_UART_STATE_READY)
    {
        return 0;
    }

    return uart->rx_count;
}

uint32_t stm32l0_uart_receive(stm32l0_uart_t *uart, uint8_t *rx_data, uint32_t rx_count, bool peek)
{
    uint32_t rx_size, rx_entries, rx_read;

    if (uart->state < STM32L0_UART_STATE_READY)
    {
        return 0;
    }

    if (rx_count > uart->rx_count)
    {
        rx_count = uart->rx_count;
    }

    rx_read = uart->rx_read;
    rx_size = rx_count;

    if ((rx_read + rx_size) > uart->rx_size)
    {
        rx_size = uart->rx_size - rx_read;
    }

    memcpy(rx_data, &uart->rx_data[rx_read], rx_size);

    rx_data += rx_size;
    rx_read += rx_size;

    if (rx_read == uart->rx_size)
    {
        rx_read = 0;
    }

    if (rx_count != rx_size)
    {
        rx_size = rx_count - rx_size;

        memcpy(rx_data, &uart->rx_data[rx_read], rx_size);

        rx_data += rx_size;
        rx_read += rx_size;
    }

    if (!peek)
    {
        uart->rx_read = rx_read;
        uart->rx_event = true;

        rx_entries = armv6m_atomic_sub(&uart->rx_count, rx_count);

        if (uart->option & (STM32L0_UART_OPTION_RTS | STM32L0_UART_OPTION_XONOFF))
        {
            if ((rx_entries >= uart->rx_threshold) && (uart->rx_count < uart->rx_threshold))
            {
                if ((uart->option & STM32L0_UART_OPTION_RTS) && uart->rx_enable)
                {
                    stm32l0_gpio_pin_write(uart->pins.rts, 0);
                
                    /* Check for a async uart interrupt race condition.
                     */
                    if (uart->rx_count >= uart->rx_threshold)
                    {
                        stm32l0_gpio_pin_write(uart->pins.rts, 1);
                    }
                }

                if (uart->option & STM32L0_UART_OPTION_XONOFF)
                {
                    uart->rq_inject = 1;

                    NVIC_SetPendingIRQ(uart->interrupt);
                }
            }
        }
    }

    return rx_count;
}


bool stm32l0_uart_transmit(stm32l0_uart_t *uart, const uint8_t *tx_data, uint32_t tx_count, stm32l0_uart_done_callback_t callback, void *context)
{
    if (uart->state < STM32L0_UART_STATE_READY)
    {
        return false;
    }

    if (uart->rq_suspend || uart->rq_break || uart->tx_data || (uart->state >= STM32L0_UART_STATE_BREAK))
    {
        return false;
    }

    uart->tx_callback = callback;
    uart->tx_context = context;
    uart->tx_count = tx_count;
    uart->tx_data = tx_data;

    NVIC_SetPendingIRQ(uart->interrupt);

    return true;
}

bool stm32l0_uart_done(stm32l0_uart_t *uart)
{
    return ((uart->state == STM32L0_UART_STATE_READY) && !uart->tx_data);
}

void USART1_IRQHandler(void)
{
    stm32l0_uart_interrupt(stm32l0_uart_device.instances[STM32L0_UART_INSTANCE_USART1]);
}

void USART2_IRQHandler(void)
{
    stm32l0_uart_interrupt(stm32l0_uart_device.instances[STM32L0_UART_INSTANCE_USART2]);
}

#if defined(STM32L072xx) || defined(STM32L082xx)

void USART4_5_IRQHandler(void)
{
    if (stm32l0_uart_device.instances[STM32L0_UART_INSTANCE_USART4])
    {
        stm32l0_uart_interrupt(stm32l0_uart_device.instances[STM32L0_UART_INSTANCE_USART4]);
    }

    if (stm32l0_uart_device.instances[STM32L0_UART_INSTANCE_USART5])
    {
        stm32l0_uart_interrupt(stm32l0_uart_device.instances[STM32L0_UART_INSTANCE_USART5]);
    }
}

#endif /* STM32L072xx || STM32L082xx */

#if defined(STM32L082xx)

void AES_RNG_LPUART1_IRQHandler(void)
{
    stm32l0_uart_interrupt(stm32l0_uart_device.instances[STM32L0_UART_INSTANCE_LPUART1]);
}

#else /* defined(STM32L082xx) */

void RNG_LPUART1_IRQHandler(void)
{
    stm32l0_uart_interrupt(stm32l0_uart_device.instances[STM32L0_UART_INSTANCE_LPUART1]);
}

#endif /* defined(STM32L082xx) */
