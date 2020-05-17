/*
 * Copyright (c) 2016-2020 Thomas Roell.  All rights reserved.
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
#include "stm32l0_system.h"
#include "stm32l0_lptim.h"
#include "stm32l0_usbd_cdc.h"
#include "usbd_cdc.h"

extern void USB_IRQHandler(void);

#define USBD_CDC_SET_LINE_CODING         0x20
#define USBD_CDC_GET_LINE_CODING         0x21
#define USBD_CDC_SET_CONTROL_LINE_STATE  0x22

#define USBD_CDC_RX_IDLE    0
#define USBD_CDC_RX_BUSY    1

#define USBD_CDC_TX_IDLE    0
#define USBD_CDC_TX_BUSY    1
#define USBD_CDC_TX_TIMEOUT 2

volatile stm32l0_usbd_cdc_info_t stm32l0_usbd_cdc_info;

typedef struct _stm32l0_usbd_cdc_device_t {
    struct _USBD_HandleTypeDef     *USBD;
    volatile uint8_t               rx_busy;
    volatile uint8_t               tx_busy;
    stm32l0_lptim_timeout_t        tx_timeout;
    stm32l0_lptim_timeout_t        dfu_timeout;
    stm32l0_usbd_cdc_t             *instances[STM32L0_USBD_CDC_INSTANCE_COUNT];
} stm32l0_usbd_cdc_device_t;

static stm32l0_usbd_cdc_device_t stm32l0_usbd_cdc_device;


static void stm32l0_usbd_cdc_rx_ready(uint8_t *data, uint32_t length)
{
    stm32l0_usbd_cdc_t *usbd_cdc = stm32l0_usbd_cdc_device.instances[0];

    stm32l0_usbd_cdc_device.rx_busy = USBD_CDC_RX_IDLE;

    if (usbd_cdc && (usbd_cdc->state == STM32L0_USBD_CDC_STATE_READY))
    {
        usbd_cdc->rx_write += length;
        
        armv6m_atomic_add(&usbd_cdc->rx_count, length);
        
        if ((usbd_cdc->rx_write + STM32L0_USBD_CDC_DATA_MAX_PACKET_SIZE) > usbd_cdc->rx_size)
        {
            usbd_cdc->rx_wrap = usbd_cdc->rx_write;
            usbd_cdc->rx_write = 0;
        }
        
        if ((usbd_cdc->rx_wrap - usbd_cdc->rx_count) >= STM32L0_USBD_CDC_DATA_MAX_PACKET_SIZE)
        {
            USBD_CDC_SetRxBuffer(stm32l0_usbd_cdc_device.USBD, &usbd_cdc->rx_data[usbd_cdc->rx_write]);
            USBD_CDC_ReceivePacket(stm32l0_usbd_cdc_device.USBD);
            
            stm32l0_usbd_cdc_device.rx_busy = USBD_CDC_RX_BUSY;
        }
        
        if (length && usbd_cdc->rx_event)
        {
            usbd_cdc->rx_event = false;
            
            if (usbd_cdc->ev_callback)
            {
                (*usbd_cdc->ev_callback)(usbd_cdc->ev_context, STM32L0_USBD_CDC_EVENT_RECEIVE);
            }
        }
    }
}

static void stm32l0_usbd_cdc_rx_start(void)
{
    stm32l0_usbd_cdc_t *usbd_cdc = stm32l0_usbd_cdc_device.instances[0];
    
    NVIC_DisableIRQ(USB_IRQn);

    if (stm32l0_usbd_cdc_device.USBD)
    {
        if (stm32l0_usbd_cdc_device.rx_busy == USBD_CDC_RX_IDLE)
        {
            if (usbd_cdc && (usbd_cdc->state == STM32L0_USBD_CDC_STATE_READY))
            {
                if ((usbd_cdc->rx_wrap - usbd_cdc->rx_count) >= STM32L0_USBD_CDC_DATA_MAX_PACKET_SIZE)
                {
                    USBD_CDC_SetRxBuffer(stm32l0_usbd_cdc_device.USBD, &usbd_cdc->rx_data[usbd_cdc->rx_write]);
                    USBD_CDC_ReceivePacket(stm32l0_usbd_cdc_device.USBD);
                    
                    stm32l0_usbd_cdc_device.rx_busy = USBD_CDC_RX_BUSY;
                }
            }
        }
        
        NVIC_EnableIRQ(USB_IRQn);
    }
}

static void stm32l0_usbd_cdc_tx_timeout(void)
{
    stm32l0_usbd_cdc_t *usbd_cdc = stm32l0_usbd_cdc_device.instances[0];
    stm32l0_usbd_cdc_done_callback_t callback;
    void *context;

    if (armv6m_atomic_casb(&stm32l0_usbd_cdc_device.tx_busy, USBD_CDC_TX_BUSY, USBD_CDC_TX_TIMEOUT) == USBD_CDC_TX_BUSY)
    {
        if (usbd_cdc && (usbd_cdc->state == STM32L0_USBD_CDC_STATE_READY))
        {
            callback = usbd_cdc->tx_callback;
            context = usbd_cdc->tx_context;

            usbd_cdc->tx_busy = false;

            if (callback)
            {
                (*callback)(context);
            }
        }
    }
}

static void stm32l0_usbd_cdc_tx_done(void)
{
    stm32l0_usbd_cdc_t *usbd_cdc = stm32l0_usbd_cdc_device.instances[0];
    stm32l0_usbd_cdc_done_callback_t callback;
    void *context;

    stm32l0_lptim_timeout_stop(&stm32l0_usbd_cdc_device.tx_timeout);
    
    if (armv6m_atomic_casb(&stm32l0_usbd_cdc_device.tx_busy, USBD_CDC_TX_BUSY, USBD_CDC_TX_IDLE) == USBD_CDC_TX_BUSY)
    {
        if (usbd_cdc && (usbd_cdc->state == STM32L0_USBD_CDC_STATE_READY))
        {
            callback = usbd_cdc->tx_callback;
            context = usbd_cdc->tx_context;

            usbd_cdc->tx_busy = false;

            if (callback)
            {
                (*callback)(context);
            }
        }
    }
    else
    {
        stm32l0_usbd_cdc_device.tx_busy = USBD_CDC_TX_IDLE;
    }
}

static void stm32l0_usbd_cdc_tx_start(void)
{
    stm32l0_usbd_cdc_t *usbd_cdc = stm32l0_usbd_cdc_device.instances[0];
    stm32l0_usbd_cdc_done_callback_t callback;
    void *context;
    
    NVIC_DisableIRQ(USB_IRQn);

    if (stm32l0_usbd_cdc_device.USBD && (armv6m_atomic_casb(&stm32l0_usbd_cdc_device.tx_busy, USBD_CDC_TX_IDLE, USBD_CDC_TX_BUSY) == USBD_CDC_TX_IDLE))
    {
        USBD_CDC_SetTxBuffer(stm32l0_usbd_cdc_device.USBD, usbd_cdc->tx_data, usbd_cdc->tx_count);
        USBD_CDC_TransmitPacket(stm32l0_usbd_cdc_device.USBD);
        
        stm32l0_lptim_timeout_start(&stm32l0_usbd_cdc_device.tx_timeout, stm32l0_lptim_millis_to_ticks(STM32L0_USBD_CDC_TX_TIMEOUT), (stm32l0_lptim_callback_t)stm32l0_usbd_cdc_tx_timeout);
    }
    else
    {
        callback = usbd_cdc->tx_callback;
        context = usbd_cdc->tx_context;

        usbd_cdc->tx_busy = false;

        if (callback)
        {
            (*callback)(context);
        }
    }
    
    if (stm32l0_usbd_cdc_device.USBD)
    {
        NVIC_EnableIRQ(USB_IRQn);
    }
}

static void stm32l0_usbd_cdc_init(USBD_HandleTypeDef *USBD)
{
    stm32l0_usbd_cdc_device.USBD = USBD;
    stm32l0_usbd_cdc_device.rx_busy = USBD_CDC_RX_IDLE;
    stm32l0_usbd_cdc_device.tx_busy = USBD_CDC_TX_IDLE;

    stm32l0_lptim_timeout_create(&stm32l0_usbd_cdc_device.tx_timeout);
    stm32l0_lptim_timeout_create(&stm32l0_usbd_cdc_device.dfu_timeout);

    stm32l0_usbd_cdc_info.dwDTERate = 115200;
    stm32l0_usbd_cdc_info.bCharFormat = 0;
    stm32l0_usbd_cdc_info.bParityType = 0;
    stm32l0_usbd_cdc_info.bDataBits = 8;
    stm32l0_usbd_cdc_info.lineState = 0;

    stm32l0_usbd_cdc_rx_start();
}

static void stm32l0_usbd_cdc_deinit(void)
{
    stm32l0_usbd_cdc_t *usbd_cdc = stm32l0_usbd_cdc_device.instances[0];
    stm32l0_usbd_cdc_done_callback_t callback;
    void *context;

    stm32l0_lptim_timeout_stop(&stm32l0_usbd_cdc_device.tx_timeout);
    stm32l0_lptim_timeout_stop(&stm32l0_usbd_cdc_device.dfu_timeout);

    if (armv6m_atomic_casb(&stm32l0_usbd_cdc_device.tx_busy, USBD_CDC_TX_BUSY, USBD_CDC_TX_IDLE) == USBD_CDC_TX_BUSY)
    {
        if (usbd_cdc && (usbd_cdc->state == STM32L0_USBD_CDC_STATE_READY))
        {
            callback = usbd_cdc->tx_callback;
            context = usbd_cdc->tx_context;

            usbd_cdc->tx_busy = false;

            if (callback)
            {
                (*callback)(context);
            }
        }
    }

    stm32l0_usbd_cdc_device.rx_busy = USBD_CDC_RX_IDLE;
    stm32l0_usbd_cdc_device.tx_busy = USBD_CDC_TX_IDLE;

    stm32l0_usbd_cdc_device.USBD = NULL;
}

static void stm32l0_usbd_cdc_control(uint8_t command, uint8_t *data, uint16_t length)
{
    if (command == USBD_CDC_GET_LINE_CODING)
    {
        data[0] = (uint8_t)(stm32l0_usbd_cdc_info.dwDTERate >> 0);
        data[1] = (uint8_t)(stm32l0_usbd_cdc_info.dwDTERate >> 8);
        data[2] = (uint8_t)(stm32l0_usbd_cdc_info.dwDTERate >> 16);
        data[3] = (uint8_t)(stm32l0_usbd_cdc_info.dwDTERate >> 24);
        data[4] = stm32l0_usbd_cdc_info.bCharFormat;
        data[5] = stm32l0_usbd_cdc_info.bParityType;
        data[6] = stm32l0_usbd_cdc_info.bDataBits;     
    }
    else
    {
        if (command == USBD_CDC_SET_LINE_CODING)
        {
            stm32l0_usbd_cdc_info.dwDTERate   = (uint32_t)((data[0] << 0) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
            stm32l0_usbd_cdc_info.bCharFormat = data[4];
            stm32l0_usbd_cdc_info.bParityType = data[5];
            stm32l0_usbd_cdc_info.bDataBits   = data[6];
        }

        if (command == USBD_CDC_SET_CONTROL_LINE_STATE)
        {
            stm32l0_usbd_cdc_info.lineState = (uint16_t)((data[2] << 0) | (data[3] << 8));
        }

        if ((command == USBD_CDC_SET_LINE_CODING) || (command == USBD_CDC_SET_CONTROL_LINE_STATE))
        {
            if ((stm32l0_usbd_cdc_info.dwDTERate == 1200) && !(stm32l0_usbd_cdc_info.lineState & 1))
            {
                stm32l0_lptim_timeout_start(&stm32l0_usbd_cdc_device.dfu_timeout, stm32l0_lptim_millis_to_ticks(STM32L0_USBD_CDC_DFU_TIMEOUT), (stm32l0_lptim_callback_t)stm32l0_system_dfu);
            }
            else
            {
                stm32l0_lptim_timeout_stop(&stm32l0_usbd_cdc_device.dfu_timeout);
            }
        }
    }
}

const USBD_CDC_ItfTypeDef stm32l0_usbd_cdc_interface = {
    stm32l0_usbd_cdc_init,
    stm32l0_usbd_cdc_deinit,
    stm32l0_usbd_cdc_control,
    stm32l0_usbd_cdc_rx_ready,
    stm32l0_usbd_cdc_tx_done,
};

bool stm32l0_usbd_cdc_create(stm32l0_usbd_cdc_t *usbd_cdc)
{
    if (usbd_cdc->state != STM32L0_USBD_CDC_STATE_NONE)
    {
        return false;
    }

    stm32l0_usbd_cdc_device.instances[0] = usbd_cdc;

    usbd_cdc->state = STM32L0_USBD_CDC_STATE_INIT;

    return true;
}

bool stm32l0_usbd_cdc_destroy(stm32l0_usbd_cdc_t *usbd_cdc)
{
    if (usbd_cdc->state != STM32L0_USBD_CDC_STATE_INIT)
    {
        return false;
    }

    usbd_cdc->state = STM32L0_USBD_CDC_STATE_NONE;
    
    stm32l0_usbd_cdc_device.instances[0] = NULL;

    return true;
}

bool stm32l0_usbd_cdc_enable(stm32l0_usbd_cdc_t *usbd_cdc, uint8_t *rx_data, uint32_t rx_size, stm32l0_usbd_cdc_event_callback_t callback, void *context)
{
    if (usbd_cdc->state != STM32L0_USBD_CDC_STATE_INIT)
    {
        return false;
    }

    usbd_cdc->rx_data  = rx_data;
    usbd_cdc->rx_size  = rx_size;
    usbd_cdc->rx_read  = 0;
    usbd_cdc->rx_write = 0;
    usbd_cdc->rx_wrap  = rx_size;
    usbd_cdc->rx_count = 0;
    usbd_cdc->rx_event = true;

    usbd_cdc->ev_callback = callback;
    usbd_cdc->ev_context = context;

    usbd_cdc->tx_busy = false;
    usbd_cdc->tx_data = NULL;
    usbd_cdc->tx_count = 0;
    usbd_cdc->tx_callback = NULL;
    usbd_cdc->tx_context = NULL;
    
    usbd_cdc->state = STM32L0_USBD_CDC_STATE_READY;
    
    if (__get_IPSR() == 0)
    {
        armv6m_svcall_0((uint32_t)&stm32l0_usbd_cdc_rx_start);
    }
    else
    {
        armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)&stm32l0_usbd_cdc_rx_start, NULL, 0);
    }

    return true;
}

bool stm32l0_usbd_cdc_disable(stm32l0_usbd_cdc_t *usbd_cdc)
{
    if (usbd_cdc->state < STM32L0_USBD_CDC_STATE_READY)
    {
        return false;
    }

    usbd_cdc->state = STM32L0_USBD_CDC_STATE_INIT;

    return true;
}

uint32_t stm32l0_usbd_cdc_count(stm32l0_usbd_cdc_t *usbd_cdc)
{
    if (usbd_cdc->state != STM32L0_USBD_CDC_STATE_READY)
    {
        return 0;
    }

    return usbd_cdc->rx_count;
}

uint32_t stm32l0_usbd_cdc_input(stm32l0_usbd_cdc_t *usbd_cdc, uint8_t *rx_data, uint32_t rx_count, bool consume)
{
    uint32_t rx_size, rx_read, rx_wrap;

    if (usbd_cdc->state != STM32L0_USBD_CDC_STATE_READY)
    {
        return false;
    }

    if (rx_count > usbd_cdc->rx_count)
    {
        rx_count = usbd_cdc->rx_count;
    }

    rx_wrap = usbd_cdc->rx_wrap;
    rx_read = usbd_cdc->rx_read;
    rx_size = rx_count;

    if ((rx_read + rx_size) > rx_wrap)
    {
        rx_size = rx_wrap - rx_read;
    }

    memcpy(rx_data, &usbd_cdc->rx_data[rx_read], rx_size);

    rx_data += rx_size;
    rx_read += rx_size;

    if (rx_read == rx_wrap)
    {
        rx_read = 0;
        rx_wrap = usbd_cdc->rx_size;
    }

    if (rx_count != rx_size)
    {
        rx_size = rx_count - rx_size;

        memcpy(rx_data, &usbd_cdc->rx_data[rx_read], rx_size);

        rx_data += rx_size;
        rx_read += rx_size;
    }

    if (consume)
    {
        usbd_cdc->rx_wrap = rx_wrap;
        usbd_cdc->rx_read = rx_read;
        usbd_cdc->rx_event = true;

        armv6m_atomic_sub(&usbd_cdc->rx_count, rx_count);

        if (stm32l0_usbd_cdc_device.rx_busy == USBD_CDC_RX_IDLE)
        {
            if (__get_IPSR() == 0)
            {
                armv6m_svcall_0((uint32_t)&stm32l0_usbd_cdc_rx_start);
            }
            else
            {
                armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)&stm32l0_usbd_cdc_rx_start, NULL, 0);
            }
        }
    }

    return rx_count;
}

bool stm32l0_usbd_cdc_transmit(stm32l0_usbd_cdc_t *usbd_cdc, const uint8_t *tx_data, uint32_t tx_count, stm32l0_usbd_cdc_done_callback_t callback, void *context)
{
    bool success = true;

    if (usbd_cdc->state != STM32L0_USBD_CDC_STATE_READY)
    {
        return false;
    }

    if (armv6m_atomic_swapb(&usbd_cdc->tx_busy, true))
    {
        return false;
    }

    usbd_cdc->tx_data = tx_data;
    usbd_cdc->tx_count = tx_count;
    usbd_cdc->tx_callback = callback;
    usbd_cdc->tx_context = context;
    
    if (__get_IPSR() == 0)
    {
        armv6m_svcall_0((uint32_t)&stm32l0_usbd_cdc_tx_start);
    }
    else
    {
        success = armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)&stm32l0_usbd_cdc_tx_start, NULL, 0);
    }
    
    return success;
}

bool stm32l0_usbd_cdc_done(stm32l0_usbd_cdc_t *usbd_cdc)
{
    return !usbd_cdc->tx_busy;
}

void stm32l0_usbd_cdc_poll(stm32l0_usbd_cdc_t *usbd_cdc)
{
    if (stm32l0_usbd_cdc_device.USBD)
    {
        USB_IRQHandler();
    }
}
