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

#if !defined(_STM32L0_UART_H)
#define _STM32L0_UART_H

#include "armv6m.h"
#include "stm32l0xx.h"

#include "stm32l0_dma.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    STM32L0_UART_INSTANCE_USART1 = 0,
    STM32L0_UART_INSTANCE_USART2,
#if defined(STM32L072xx) || defined(STM32L082xx)
    STM32L0_UART_INSTANCE_USART4,
    STM32L0_UART_INSTANCE_USART5,
#endif /* STM32L072xx || STM32L082xx */
    STM32L0_UART_INSTANCE_LPUART1,
    STM32L0_UART_INSTANCE_COUNT
};

#define STM32L0_UART_OPTION_STOP_MASK        0x00000001
#define STM32L0_UART_OPTION_STOP_SHIFT       0
#define STM32L0_UART_OPTION_STOP_1           0x00000000
#define STM32L0_UART_OPTION_STOP_2           0x00000001
#define STM32L0_UART_OPTION_PARITY_MASK      0x00000006
#define STM32L0_UART_OPTION_PARITY_SHIFT     1
#define STM32L0_UART_OPTION_PARITY_NONE      0x00000000
#define STM32L0_UART_OPTION_PARITY_EVEN      0x00000002
#define STM32L0_UART_OPTION_PARITY_ODD       0x00000004
#define STM32L0_UART_OPTION_DATA_SIZE_MASK   0x00000008
#define STM32L0_UART_OPTION_DATA_SIZE_SHIFT  3
#define STM32L0_UART_OPTION_DATA_SIZE_7      0x00000000
#define STM32L0_UART_OPTION_DATA_SIZE_8      0x00000008
#define STM32L0_UART_OPTION_RTS              0x00000010
#define STM32L0_UART_OPTION_CTS              0x00000020
#define STM32L0_UART_OPTION_XONOFF           0x00000040
#define STM32L0_UART_OPTION_WAKEUP           0x00000080
#define STM32L0_UART_OPTION_RX_INVERT        0x00000100
#define STM32L0_UART_OPTION_TX_INVERT        0x00000200

#define STM32L0_UART_EVENT_NOISE             0x00000001
#define STM32L0_UART_EVENT_PARITY            0x00000002
#define STM32L0_UART_EVENT_FRAMING           0x00000004
#define STM32L0_UART_EVENT_BREAK             0x00000008
#define STM32L0_UART_EVENT_OVERRUN           0x00000010
#define STM32L0_UART_EVENT_WAKEUP            0x00000020
#define STM32L0_UART_EVENT_RECEIVE           0x00000040

typedef void (*stm32l0_uart_event_callback_t)(void *context, uint32_t events);
typedef void (*stm32l0_uart_done_callback_t)(void *context);

#define STM32L0_UART_STATE_NONE              0
#define STM32L0_UART_STATE_INIT              1
#define STM32L0_UART_STATE_BUSY              2
#define STM32L0_UART_STATE_READY             3
#define STM32L0_UART_STATE_BREAK             4
#define STM32L0_UART_STATE_SUSPENDED         5
#define STM32L0_UART_STATE_SUSPENDED_BREAK   6
#define STM32L0_UART_STATE_TRANSMIT          7
#define STM32L0_UART_STATE_INJECT            8
#define STM32L0_UART_STATE_WAIT              9

typedef struct _stm32l0_uart_pins_t {
    uint16_t                      rx;
    uint16_t                      tx;
    uint16_t                      rts;
    uint16_t                      cts;
} stm32l0_uart_pins_t;

typedef struct _stm32l0_uart_params_t {
    uint8_t                       instance;
    uint8_t                       priority;
    uint8_t                       rx_dma;
    uint8_t                       tx_dma;
    uint8_t                       *rx_fifo;
    uint16_t                      rx_entries;
    stm32l0_uart_pins_t           pins;
} stm32l0_uart_params_t;

typedef struct _stm32l0_uart_t {
    USART_TypeDef                 *USART;
    volatile uint8_t              state;
    uint8_t                       instance;
    uint8_t                       interrupt;
    uint8_t                       priority;
    uint8_t                       rx_dma;
    uint8_t                       tx_dma;
    stm32l0_uart_pins_t           pins;
    uint32_t                      baudrate;
    uint32_t                      option;
    stm32l0_uart_event_callback_t ev_callback;
    void                          *ev_context;
    uint8_t                       *rx_fifo;
    uint16_t                      rx_entries;
    uint16_t                      rx_index;
    uint8_t                       *rx_data;
    uint16_t                      rx_size;
    uint16_t                      rx_threshold;
    uint16_t                      rx_read;
    uint16_t                      rx_write;
    volatile uint32_t             rx_count;
    volatile uint8_t              rx_sequence;
    volatile uint8_t              rx_enable;
    volatile uint8_t              rx_xonoff;
    volatile uint8_t              rx_event;
    stm32l0_uart_done_callback_t  tx_callback;
    void                          *tx_context;
    const uint8_t * volatile      tx_data;
    uint32_t                      tx_count;
    volatile uint8_t              tx_xonoff;
    volatile uint8_t              rq_suspend;
    volatile uint8_t              rq_break;
    volatile uint8_t              rq_inject;
} stm32l0_uart_t;

extern bool stm32l0_uart_create(stm32l0_uart_t *uart, const stm32l0_uart_params_t *params);
extern bool stm32l0_uart_destroy(stm32l0_uart_t *uart);
extern bool stm32l0_uart_enable(stm32l0_uart_t *uart, uint8_t *rx_data, uint32_t rx_size, uint32_t baudrate, uint32_t option, stm32l0_uart_event_callback_t callback, void *context);
extern bool stm32l0_uart_disable(stm32l0_uart_t *uart);
extern bool stm32l0_uart_configure(stm32l0_uart_t *uart, uint32_t baudrate, uint32_t option);
extern bool stm32l0_uart_suspend(stm32l0_uart_t *uart);
extern bool stm32l0_uart_resume(stm32l0_uart_t *uart);
extern bool stm32l0_uart_rts_enable(stm32l0_uart_t *uart, bool onoff);
extern bool stm32l0_uart_cts_holding(stm32l0_uart_t *uart);
extern bool stm32l0_uart_break_state(stm32l0_uart_t *uart, bool onoff);
extern uint32_t stm32l0_uart_count(stm32l0_uart_t *uart);
extern uint32_t stm32l0_uart_receive(stm32l0_uart_t *uart, uint8_t *rx_data, uint32_t rx_count, bool peek);
extern bool stm32l0_uart_transmit(stm32l0_uart_t *uart, const uint8_t *tx_data, uint32_t tx_count, stm32l0_uart_done_callback_t callback, void *context);

#ifdef __cplusplus
}
#endif

#endif /* _STM32L0_UART_H */
