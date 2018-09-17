/*
 * Copyright (c) 2016-2018 Thomas Roell.  All rights reserved.
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

#if !defined(_STM32L0_USBD_CDC_H)
#define _STM32L0_USBD_CDC_H

#include "stm32l0xx.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define STM32L0_USBD_CDC_INSTANCE_COUNT          1

#define STM32L0_USBD_CDC_DATA_MAX_PACKET_SIZE    64
#define STM32L0_USBD_CDC_FIFO_SIZE               128

typedef struct _stm32l0_usbd_cdc_info_t {
    volatile int32_t               dwDTERate;
    volatile uint8_t               bCharFormat;
    volatile uint8_t               bParityType;
    volatile uint8_t               bDataBits;
    volatile uint8_t               lineState;
} stm32l0_usbd_cdc_info_t;

extern volatile stm32l0_usbd_cdc_info_t stm32l0_usbd_cdc_info;

#define STM32L0_USBD_CDC_EVENT_RECEIVE           0x00000001


typedef void (*stm32l0_usbd_cdc_event_callback_t)(void *context, uint32_t events);
typedef void (*stm32l0_usbd_cdc_done_callback_t)(void *context);

#define STM32L0_USBD_CDC_STATE_NONE              0
#define STM32L0_USBD_CDC_STATE_INIT              1
#define STM32L0_USBD_CDC_STATE_READY             2
#define STM32L0_USBD_CDC_STATE_SUSPENDED         3
#define STM32L0_USBD_CDC_STATE_RESET             4

typedef struct _stm32l0_usbd_cdc_t {
    volatile uint8_t                  state;
    uint8_t                           option;
    stm32l0_usbd_cdc_event_callback_t ev_callback;
    void                              *ev_context;
    uint8_t                           *rx_data;
    uint16_t                          rx_size;
    uint16_t                          rx_read;
    uint16_t                          rx_write;
    volatile uint16_t                 rx_wrap;
    volatile uint32_t                 rx_count;
    volatile uint8_t                  rx_event;
    stm32l0_usbd_cdc_done_callback_t  tx_callback;
    void                              *tx_context;
} stm32l0_usbd_cdc_t;

extern bool stm32l0_usbd_cdc_create(stm32l0_usbd_cdc_t *usbd_cdc);
extern bool stm32l0_usbd_cdc_destroy(stm32l0_usbd_cdc_t *usbd_cdc);
extern bool stm32l0_usbd_cdc_enable(stm32l0_usbd_cdc_t *usbd_cdc, uint8_t *rx_data, uint32_t rx_size, stm32l0_usbd_cdc_event_callback_t callback, void *context);
extern bool stm32l0_usbd_cdc_disable(stm32l0_usbd_cdc_t *usbd_cdc);
extern uint32_t stm32l0_usbd_cdc_count(stm32l0_usbd_cdc_t *usbd_cdc);
extern uint32_t stm32l0_usbd_cdc_receive(stm32l0_usbd_cdc_t *usbd_cdc, uint8_t *rx_data, uint32_t rx_count, bool peek);
extern bool stm32l0_usbd_cdc_transmit(stm32l0_usbd_cdc_t *usbd_cdc, const uint8_t *tx_data, uint32_t tx_count, stm32l0_usbd_cdc_done_callback_t callback, void *context);
extern bool stm32l0_usbd_cdc_done(stm32l0_usbd_cdc_t *usbd_cdc);
extern void stm32l0_usbd_cdc_poll(stm32l0_usbd_cdc_t *usbd_cdc);

#ifdef __cplusplus
}
#endif

#endif /* _STM32L0_STM32L0_USBD_CDC_H */
