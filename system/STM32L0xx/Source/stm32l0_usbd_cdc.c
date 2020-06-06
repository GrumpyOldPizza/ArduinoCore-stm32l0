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
#include "stm32l0_usbd_cdc.h"
#include "stm32l0_usbd_class.h"

extern void USB_IRQHandler(void);

static void stm32l0_usbd_cdc_tx_control(void);

#define STM32L0_USBD_CDC_INT_IDLE                0
#define STM32L0_USBD_CDC_INT_BUSY                1

#define STM32L0_USBD_CDC_TX_IDLE                 0
#define STM32L0_USBD_CDC_TX_WAIT                 1
#define STM32L0_USBD_CDC_TX_BUSY                 2
#define STM32L0_USBD_CDC_TX_BUSY_TIMEOUT         3

typedef struct _stm32l0_usbd_cdc_device_t {
    struct _USBD_HandleTypeDef     *USBD;
    stm32l0_usbd_cdc_t             *instances[STM32L0_USBD_CDC_INSTANCE_COUNT];
    uint8_t                        interface;    
    uint8_t                        ep0_command;    
    uint8_t                        ep0_length;    
    uint8_t                        ep0_data[16];
    struct {
        USBD_SetupReqTypedef       req;
        uint8_t                    data[8];
    }                              notification;
    uint16_t                       serial_state;
    uint16_t                       line_state;
    stm32l0_usbd_cdc_line_coding_t line_coding;
    bool                           break_state;
    uint16_t                       tx_timeout;
    uint16_t                       dfu_timeout;
    uint16_t                       break_timeout;
    volatile uint8_t               int_busy;
    volatile uint32_t              int_sequence;
    volatile uint8_t               tx_busy;
    volatile uint8_t               tx_zlp;
    uint8_t                        rx_index;
    uint8_t                        rx_data[2][STM32L0_USBD_CDC_DATA_MAX_PACKET_SIZE];
} stm32l0_usbd_cdc_device_t;

static stm32l0_usbd_cdc_device_t stm32l0_usbd_cdc_device;

static void stm32l0_usbd_cdc_get_info(uint8_t command, uint8_t *data, uint32_t length)
{
    if (command == USB_CDC_GET_LINE_CODING)
    {
        data[0] = (uint8_t)(stm32l0_usbd_cdc_device.line_coding.dwDTERate >> 0);
        data[1] = (uint8_t)(stm32l0_usbd_cdc_device.line_coding.dwDTERate >> 8);
        data[2] = (uint8_t)(stm32l0_usbd_cdc_device.line_coding.dwDTERate >> 16);
        data[3] = (uint8_t)(stm32l0_usbd_cdc_device.line_coding.dwDTERate >> 24);
        data[4] = stm32l0_usbd_cdc_device.line_coding.bCharFormat;
        data[5] = stm32l0_usbd_cdc_device.line_coding.bParityType;
        data[6] = stm32l0_usbd_cdc_device.line_coding.bDataBits;     
    }
}

static void stm32l0_usbd_cdc_set_info(uint8_t command, const uint8_t *data, uint32_t length)
{
    stm32l0_usbd_cdc_t *usbd_cdc = stm32l0_usbd_cdc_device.instances[0];
    uint16_t line_state, timeout;

    if (command == USB_CDC_SET_LINE_CODING)
    {
        stm32l0_usbd_cdc_device.line_coding.dwDTERate   = (uint32_t)((data[0] << 0) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
        stm32l0_usbd_cdc_device.line_coding.bCharFormat = data[4];
        stm32l0_usbd_cdc_device.line_coding.bParityType = data[5];
        stm32l0_usbd_cdc_device.line_coding.bDataBits   = data[6];

        if (usbd_cdc && (usbd_cdc->state == STM32L0_USBD_CDC_STATE_READY))
        {
            if (usbd_cdc->ev_callback)
            {
                (*usbd_cdc->ev_callback)(usbd_cdc->ev_context, STM32L0_USBD_CDC_EVENT_LINE_CODING);
            }
        }
    }

    if (command == USB_CDC_SET_CONTROL_LINE_STATE)
    {
        line_state = stm32l0_usbd_cdc_device.line_state;
        
        stm32l0_usbd_cdc_device.line_state = (data[1] << 8) | (data[0] << 0);

        if ((line_state == 0) && (stm32l0_usbd_cdc_device.line_state & USB_CDC_LINE_STATE_DTR))
        {
            stm32l0_usbd_cdc_tx_control();
        }
        
        if (usbd_cdc && (usbd_cdc->state == STM32L0_USBD_CDC_STATE_READY))
        {
            if (usbd_cdc->ev_callback)
            {
                (*usbd_cdc->ev_callback)(usbd_cdc->ev_context, STM32L0_USBD_CDC_EVENT_LINE_STATE);
            }
        }
    }

    if (command == USB_CDC_SEND_BREAK)
    {
        timeout = (data[1] << 8) | (data[0] << 0);

        if (timeout == 0)
        {
            stm32l0_usbd_cdc_device.break_state = 0;

            stm32l0_usbd_cdc_device.break_timeout = 0;
        }
        else
        {
            stm32l0_usbd_cdc_device.break_state = 1;

            if (timeout == 65535)
            {
                stm32l0_usbd_cdc_device.break_timeout = 0;
            }
            else
            {
                stm32l0_usbd_cdc_device.break_timeout = timeout;
            }
        }

        if (usbd_cdc && (usbd_cdc->state == STM32L0_USBD_CDC_STATE_READY))
        {
            if (usbd_cdc->ev_callback)
            {
                (*usbd_cdc->ev_callback)(usbd_cdc->ev_context, STM32L0_USBD_CDC_EVENT_BREAK_STATE);
            }
        }
    }
    
    if ((command == USB_CDC_SET_LINE_CODING) || (command == USB_CDC_SET_CONTROL_LINE_STATE))
    {
        if ((stm32l0_usbd_cdc_device.line_coding.dwDTERate == 1200) && !(stm32l0_usbd_cdc_device.line_state & USB_CDC_LINE_STATE_DTR))
        {
            stm32l0_usbd_cdc_device.dfu_timeout = STM32L0_USBD_CDC_DFU_TIMEOUT;
        }
        else
        {
            stm32l0_usbd_cdc_device.dfu_timeout = 0;
        }
    }
}

static void stm32l0_usbd_cdc_tx_control(void)
{
    stm32l0_usbd_cdc_t *usbd_cdc = stm32l0_usbd_cdc_device.instances[0];

    NVIC_DisableIRQ(USB_IRQn);

    if (stm32l0_usbd_cdc_device.USBD)
    {
        if (stm32l0_usbd_cdc_device.line_state & USB_CDC_LINE_STATE_DTR)
        {
            if (armv6m_atomic_casb(&stm32l0_usbd_cdc_device.int_busy, STM32L0_USBD_CDC_INT_IDLE, STM32L0_USBD_CDC_INT_BUSY) == STM32L0_USBD_CDC_INT_IDLE)
            {
                stm32l0_usbd_cdc_device.int_sequence++;

                if (usbd_cdc && (usbd_cdc->state == STM32L0_USBD_CDC_STATE_READY))
                {
                    stm32l0_usbd_cdc_device.serial_state = usbd_cdc->serial_state;
                }
                else
                {
                    stm32l0_usbd_cdc_device.serial_state = 0;
                }

                stm32l0_usbd_cdc_device.notification.req.bmRequest = USB_REQ_DIRECTION_DEVICE_TO_HOST | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE;
                stm32l0_usbd_cdc_device.notification.req.bRequest = USB_CDC_SERIAL_STATE;
                stm32l0_usbd_cdc_device.notification.req.wIndex = 0;
                stm32l0_usbd_cdc_device.notification.req.wLength = 2;
                stm32l0_usbd_cdc_device.notification.data[0] = (stm32l0_usbd_cdc_device.serial_state >> 0);
                stm32l0_usbd_cdc_device.notification.data[1] = (stm32l0_usbd_cdc_device.serial_state >> 8);

                USBD_LL_Transmit(stm32l0_usbd_cdc_device.USBD, STM32L0_USBD_CDC_CONTROL_EP_ADDR, (uint8_t*)&stm32l0_usbd_cdc_device.notification, 10);
            }
        }
        
        NVIC_EnableIRQ(USB_IRQn);
    }
}

static void stm32l0_usbd_cdc_tx_data(void)
{
    stm32l0_usbd_cdc_t *usbd_cdc = stm32l0_usbd_cdc_device.instances[0];

    NVIC_DisableIRQ(USB_IRQn);

    if (stm32l0_usbd_cdc_device.USBD)
    {
        if (stm32l0_usbd_cdc_device.int_busy == STM32L0_USBD_CDC_INT_IDLE)
        {
            stm32l0_usbd_cdc_device.tx_busy = STM32L0_USBD_CDC_TX_BUSY;
                
            USBD_LL_Transmit(stm32l0_usbd_cdc_device.USBD, STM32L0_USBD_CDC_DATA_IN_EP_ADDR, (uint8_t*)usbd_cdc->tx_data, usbd_cdc->tx_count);
            
            /* Send ZLP if tx_count is a multiple of STM32L0_USBD_CDC_DATA_MAX_PACKET_SIZE
             */
            stm32l0_usbd_cdc_device.tx_zlp = ((usbd_cdc->tx_count & (STM32L0_USBD_CDC_DATA_MAX_PACKET_SIZE-1)) == 0);

            stm32l0_usbd_cdc_device.tx_timeout = STM32L0_USBD_CDC_TX_TIMEOUT;
        }
        
        NVIC_EnableIRQ(USB_IRQn);
    }
    else
    {
        if (usbd_cdc->tx_callback)
        {
            (*usbd_cdc->tx_callback)(usbd_cdc->tx_context);
        }
    }
}

uint8_t USBD_CDC_Init(USBD_HandleTypeDef *pdev,  uint8_t cfgidx)
{
    stm32l0_usbd_cdc_device.USBD = pdev;
    stm32l0_usbd_cdc_device.interface = 0;
    stm32l0_usbd_cdc_device.ep0_command = 0;
    stm32l0_usbd_cdc_device.ep0_length = 0;
    stm32l0_usbd_cdc_device.serial_state = 0;
    stm32l0_usbd_cdc_device.line_state = 0;
    stm32l0_usbd_cdc_device.line_coding.dwDTERate = STM32L0_USBD_CDC_LINE_CODING_DTE_RATE;
    stm32l0_usbd_cdc_device.line_coding.bCharFormat = STM32L0_USBD_CDC_LINE_CODING_CHAR_FORMAT;
    stm32l0_usbd_cdc_device.line_coding.bParityType = STM32L0_USBD_CDC_LINE_CODING_PARITY_TYPE;
    stm32l0_usbd_cdc_device.line_coding.bDataBits = STM32L0_USBD_CDC_LINE_CODING_DATA_BITS;
    stm32l0_usbd_cdc_device.break_state = false;
    stm32l0_usbd_cdc_device.int_busy = STM32L0_USBD_CDC_INT_IDLE;
    stm32l0_usbd_cdc_device.rx_index = 0;
    stm32l0_usbd_cdc_device.tx_busy = STM32L0_USBD_CDC_TX_IDLE;
    stm32l0_usbd_cdc_device.tx_timeout = 0;
    stm32l0_usbd_cdc_device.dfu_timeout = 0;
    stm32l0_usbd_cdc_device.break_timeout = 0;

    USBD_LL_OpenEP(pdev, STM32L0_USBD_CDC_CONTROL_EP_ADDR, USBD_EP_TYPE_INTR, STM32L0_USBD_CDC_CONTROL_MAX_PACKET_SIZE);
    pdev->ep_in[STM32L0_USBD_CDC_CONTROL_EP_ADDR & 15].is_used = 1;
    
    USBD_LL_OpenEP(pdev, STM32L0_USBD_CDC_DATA_IN_EP_ADDR, USBD_EP_TYPE_BULK, STM32L0_USBD_CDC_DATA_MAX_PACKET_SIZE);
    pdev->ep_in[STM32L0_USBD_CDC_DATA_IN_EP_ADDR & 15].is_used = 1;

    USBD_LL_OpenEP(pdev, STM32L0_USBD_CDC_DATA_OUT_EP_ADDR, USBD_EP_TYPE_BULK, STM32L0_USBD_CDC_DATA_MAX_PACKET_SIZE);
    pdev->ep_out[STM32L0_USBD_CDC_DATA_OUT_EP_ADDR & 15].is_used = 1;
    
    USBD_LL_PrepareReceive(pdev, STM32L0_USBD_CDC_DATA_OUT_EP_ADDR, &stm32l0_usbd_cdc_device.rx_data[stm32l0_usbd_cdc_device.rx_index][0], STM32L0_USBD_CDC_DATA_MAX_PACKET_SIZE);

    return USBD_OK;
}

uint8_t USBD_CDC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    stm32l0_usbd_cdc_t *usbd_cdc = stm32l0_usbd_cdc_device.instances[0];

    USBD_LL_CloseEP(pdev, STM32L0_USBD_CDC_CONTROL_EP_ADDR);
    pdev->ep_in[STM32L0_USBD_CDC_CONTROL_EP_ADDR & 15].is_used = 0;

    USBD_LL_CloseEP(pdev, STM32L0_USBD_CDC_DATA_IN_EP_ADDR);
    pdev->ep_in[STM32L0_USBD_CDC_DATA_IN_EP_ADDR & 15].is_used = 0;

    USBD_LL_CloseEP(pdev, STM32L0_USBD_CDC_DATA_OUT_EP_ADDR);
    pdev->ep_out[STM32L0_USBD_CDC_DATA_OUT_EP_ADDR & 15].is_used = 0;

    stm32l0_usbd_cdc_device.USBD = NULL;
    stm32l0_usbd_cdc_device.serial_state = 0;
    stm32l0_usbd_cdc_device.line_state = 0;
    stm32l0_usbd_cdc_device.line_coding.dwDTERate = STM32L0_USBD_CDC_LINE_CODING_DTE_RATE;
    stm32l0_usbd_cdc_device.line_coding.bCharFormat = STM32L0_USBD_CDC_LINE_CODING_CHAR_FORMAT;
    stm32l0_usbd_cdc_device.line_coding.bParityType = STM32L0_USBD_CDC_LINE_CODING_PARITY_TYPE;
    stm32l0_usbd_cdc_device.line_coding.bDataBits = STM32L0_USBD_CDC_LINE_CODING_DATA_BITS;
    stm32l0_usbd_cdc_device.break_state = false;
    stm32l0_usbd_cdc_device.int_busy = STM32L0_USBD_CDC_INT_IDLE;
    stm32l0_usbd_cdc_device.tx_timeout = 0;
    stm32l0_usbd_cdc_device.dfu_timeout = 0;
    stm32l0_usbd_cdc_device.break_timeout = 0;

    if (stm32l0_usbd_cdc_device.tx_busy >= STM32L0_USBD_CDC_TX_WAIT)
    {
        stm32l0_usbd_cdc_device.tx_busy = STM32L0_USBD_CDC_TX_IDLE;

        if (usbd_cdc && (usbd_cdc->state == STM32L0_USBD_CDC_STATE_READY))
        {
            if (usbd_cdc->tx_callback)
            {
                (*usbd_cdc->tx_callback)(usbd_cdc->tx_context);
            }
        }
    }
    else
    {
        stm32l0_usbd_cdc_device.tx_busy = STM32L0_USBD_CDC_TX_IDLE;
    }

    return USBD_OK;
}

uint8_t USBD_CDC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    switch (req->bmRequest & USB_REQ_TYPE_MASK) {
    case USB_REQ_TYPE_CLASS:
        if (req->wLength)
        {
            if (req->bmRequest & 0x80)
            {
                stm32l0_usbd_cdc_get_info(req->bRequest, &stm32l0_usbd_cdc_device.ep0_data[0], req->wLength);

                USBD_CtlSendData(pdev, &stm32l0_usbd_cdc_device.ep0_data[0], req->wLength);
            }
            else
            {
                stm32l0_usbd_cdc_device.ep0_command = req->bRequest;
                stm32l0_usbd_cdc_device.ep0_length = req->wLength;
              
                USBD_CtlPrepareRx(pdev, &stm32l0_usbd_cdc_device.ep0_data[0], req->wLength);
            }
        }
        else
        {
            stm32l0_usbd_cdc_set_info(req->bRequest, (uint8_t*)&req->wValue, 0);
        }
        break;

    case USB_REQ_TYPE_STANDARD:
        switch (req->bRequest) {      
        case USB_REQ_GET_INTERFACE:
            USBD_CtlSendData(pdev, &stm32l0_usbd_cdc_device.interface, 1);
            break;
          
        case USB_REQ_SET_INTERFACE:
            stm32l0_usbd_cdc_device.interface = (uint8_t)(req->wValue);
            break;
        }
        break;
        
    default: 
        break;
    }

    return USBD_OK;
}

uint8_t USBD_CDC_EP0_RxReady(USBD_HandleTypeDef *pdev)
{ 
    if (stm32l0_usbd_cdc_device.ep0_length)
    {
        stm32l0_usbd_cdc_set_info(stm32l0_usbd_cdc_device.ep0_command, &stm32l0_usbd_cdc_device.ep0_data[0], stm32l0_usbd_cdc_device.ep0_length);

        stm32l0_usbd_cdc_device.ep0_length = 0;
    }

    return USBD_OK;
}

uint8_t USBD_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    stm32l0_usbd_cdc_t *usbd_cdc = stm32l0_usbd_cdc_device.instances[0];
    
    if (epnum == (STM32L0_USBD_CDC_DATA_IN_EP_ADDR & 15))
    {
        /* DATA endpoint */

        if (stm32l0_usbd_cdc_device.tx_zlp)
        {
            /* Send ZLP if tx_count is a multiple of STM32L0_USBD_CDC_DATA_MAX_PACKET_SIZE
             */
            USBD_LL_Transmit(stm32l0_usbd_cdc_device.USBD, STM32L0_USBD_CDC_DATA_IN_EP_ADDR, (uint8_t*)NULL, 0);

            stm32l0_usbd_cdc_device.tx_zlp = false;
        }
        else
        {
            if (stm32l0_usbd_cdc_device.tx_busy == STM32L0_USBD_CDC_TX_BUSY)
            {
                stm32l0_usbd_cdc_device.tx_busy = STM32L0_USBD_CDC_TX_IDLE;

                stm32l0_usbd_cdc_device.tx_timeout = 0;

                if (usbd_cdc && (usbd_cdc->state == STM32L0_USBD_CDC_STATE_READY))
                {
                    if (usbd_cdc->tx_callback)
                    {
                        (*usbd_cdc->tx_callback)(usbd_cdc->tx_context);
                    }
                }
            }
            else
            {
                stm32l0_usbd_cdc_device.tx_busy = STM32L0_USBD_CDC_TX_IDLE;
            }
        }
    }
    else
    {
        /* COMMAND endpoint */

        if ((stm32l0_usbd_cdc_device.line_state & USB_CDC_LINE_STATE_DTR) && usbd_cdc && (usbd_cdc->state == STM32L0_USBD_CDC_STATE_READY) && (stm32l0_usbd_cdc_device.serial_state != usbd_cdc->serial_state))
        {
            stm32l0_usbd_cdc_device.int_busy = STM32L0_USBD_CDC_INT_BUSY;
            stm32l0_usbd_cdc_device.int_sequence++;

            stm32l0_usbd_cdc_device.serial_state = usbd_cdc->serial_state;

            stm32l0_usbd_cdc_device.notification.req.bmRequest = USB_REQ_DIRECTION_DEVICE_TO_HOST | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE;
            stm32l0_usbd_cdc_device.notification.req.bRequest = USB_CDC_SERIAL_STATE;
            stm32l0_usbd_cdc_device.notification.req.wIndex = 0;
            stm32l0_usbd_cdc_device.notification.req.wLength = 2;
            stm32l0_usbd_cdc_device.notification.data[0] = (stm32l0_usbd_cdc_device.serial_state >> 0);
            stm32l0_usbd_cdc_device.notification.data[1] = (stm32l0_usbd_cdc_device.serial_state >> 8);
            
            USBD_LL_Transmit(stm32l0_usbd_cdc_device.USBD, STM32L0_USBD_CDC_CONTROL_EP_ADDR, (uint8_t*)&stm32l0_usbd_cdc_device.notification, 10);
        }
        else
        {
            stm32l0_usbd_cdc_device.int_busy = STM32L0_USBD_CDC_INT_IDLE;

            if (stm32l0_usbd_cdc_device.tx_busy == STM32L0_USBD_CDC_TX_WAIT)
            {
                stm32l0_usbd_cdc_device.tx_busy = STM32L0_USBD_CDC_TX_BUSY;
                
                USBD_LL_Transmit(stm32l0_usbd_cdc_device.USBD, STM32L0_USBD_CDC_DATA_IN_EP_ADDR, (uint8_t*)usbd_cdc->tx_data, usbd_cdc->tx_count);
                
                /* Send ZLP if tx_count is a multiple of STM32L0_USBD_CDC_DATA_MAX_PACKET_SIZE
                 */
                stm32l0_usbd_cdc_device.tx_zlp = ((usbd_cdc->tx_count & (STM32L0_USBD_CDC_DATA_MAX_PACKET_SIZE-1)) == 0);

                stm32l0_usbd_cdc_device.tx_timeout = STM32L0_USBD_CDC_TX_TIMEOUT;
            }
        }
    }
    
    return USBD_OK;
}

uint8_t USBD_CDC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{      
    stm32l0_usbd_cdc_t *usbd_cdc = stm32l0_usbd_cdc_device.instances[0];
    uint32_t rx_count, rx_size, rx_write, events;

    rx_count = USBD_LL_GetRxDataSize(pdev, epnum);

    USBD_LL_PrepareReceive(pdev, STM32L0_USBD_CDC_DATA_OUT_EP_ADDR, &stm32l0_usbd_cdc_device.rx_data[stm32l0_usbd_cdc_device.rx_index ^ 1][0], STM32L0_USBD_CDC_DATA_MAX_PACKET_SIZE);

    if (usbd_cdc && (usbd_cdc->state == STM32L0_USBD_CDC_STATE_READY))
    {
        rx_write = usbd_cdc->rx_write;
        
        if (rx_count)
        {
            events = 0;

            if (usbd_cdc->rx_event)
            {
                usbd_cdc->rx_event = false;
                
                events = STM32L0_USBD_CDC_EVENT_RECEIVE;
            }
            else
            {
                events = 0;
            }

            if (rx_count > (usbd_cdc->rx_size - usbd_cdc->rx_count))
            {
                rx_count = (usbd_cdc->rx_size - usbd_cdc->rx_count);
                
                events |= STM32L0_USBD_CDC_EVENT_OVERRUN;
            }
            
            rx_size = rx_count;
            
            if (rx_size > (usbd_cdc->rx_size - rx_write))
            {
                rx_size = usbd_cdc->rx_size - rx_write;
            }
            
            if (rx_size)
            {
                memcpy(&usbd_cdc->rx_data[rx_write], &stm32l0_usbd_cdc_device.rx_data[stm32l0_usbd_cdc_device.rx_index][0], rx_size);
                
                rx_count -= rx_size;
                rx_write += rx_size;
                
                if (rx_write == usbd_cdc->rx_size)
                {
                    rx_write = 0;
                }
                
                armv6m_atomic_add(&usbd_cdc->rx_count, rx_size);
            }
            
            if (rx_count)
            {
                memcpy(&usbd_cdc->rx_data[rx_write], &stm32l0_usbd_cdc_device.rx_data[stm32l0_usbd_cdc_device.rx_index][rx_size], rx_count);
                
                rx_write += rx_count;
                
                if (rx_write == usbd_cdc->rx_size)
                {
                    rx_write = 0;
                }
                
                armv6m_atomic_add(&usbd_cdc->rx_count, rx_count);
            }

            usbd_cdc->rx_write = rx_write;
        
            if (events)
            {
                if (usbd_cdc->ev_callback)
                {
                    (*usbd_cdc->ev_callback)(usbd_cdc->ev_context, events);
                }
            }
        }
    }

    stm32l0_usbd_cdc_device.rx_index ^= 1;
    
    return USBD_OK;
}

uint8_t USBD_CDC_SOF(USBD_HandleTypeDef *pdev)
{      
    stm32l0_usbd_cdc_t *usbd_cdc = stm32l0_usbd_cdc_device.instances[0];

    if (stm32l0_usbd_cdc_device.tx_timeout) 
    {
        if (stm32l0_usbd_cdc_device.tx_timeout == 1)
        {
            /* Here it could be a soft disconnect, or a busy host ... So keep trying.
             */
          
            stm32l0_usbd_cdc_device.tx_busy = STM32L0_USBD_CDC_TX_BUSY_TIMEOUT;

            stm32l0_usbd_cdc_device.tx_timeout = 0;

            if (usbd_cdc && (usbd_cdc->state == STM32L0_USBD_CDC_STATE_READY))
            {
                if (usbd_cdc->tx_callback)
                {
                    (*usbd_cdc->tx_callback)(usbd_cdc->tx_context);
                }
            }
        }
        else
        {
            stm32l0_usbd_cdc_device.tx_timeout--;
        }
    }

    if (stm32l0_usbd_cdc_device.break_timeout) 
    {
        if (stm32l0_usbd_cdc_device.break_timeout == 1)
        {
            stm32l0_usbd_cdc_device.break_state = 0;

            if (usbd_cdc && (usbd_cdc->state == STM32L0_USBD_CDC_STATE_READY))
            {
                if (usbd_cdc->ev_callback)
                {
                    (*usbd_cdc->ev_callback)(usbd_cdc->ev_context, STM32L0_USBD_CDC_EVENT_BREAK_STATE);
                }
            }
        }
        else
        {
            stm32l0_usbd_cdc_device.break_timeout--;
        }
    }
    
    if (stm32l0_usbd_cdc_device.dfu_timeout) 
    {
        if (stm32l0_usbd_cdc_device.dfu_timeout == 1)
        {
            stm32l0_usbd_cdc_device.dfu_timeout = 0;

            stm32l0_system_dfu();
        }
        else
        {
            stm32l0_usbd_cdc_device.dfu_timeout--;
        }
    }

    return USBD_OK;
}

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
    usbd_cdc->rx_count = 0;
    usbd_cdc->rx_event = true;

    usbd_cdc->ev_callback = callback;
    usbd_cdc->ev_context = context;

    usbd_cdc->tx_data = NULL;
    
    usbd_cdc->state = STM32L0_USBD_CDC_STATE_READY;

    stm32l0_usbd_cdc_serial_state(usbd_cdc, USB_CDC_SERIAL_STATE_DCD);
    
    return true;
}

bool stm32l0_usbd_cdc_disable(stm32l0_usbd_cdc_t *usbd_cdc)
{
    if (usbd_cdc->state < STM32L0_USBD_CDC_STATE_READY)
    {
        return false;
    }

    stm32l0_usbd_cdc_serial_state(usbd_cdc, 0);
    
    usbd_cdc->state = STM32L0_USBD_CDC_STATE_INIT;

    return true;
}

void stm32l0_usbd_cdc_line_coding(stm32l0_usbd_cdc_t *usbd_cdc, stm32l0_usbd_cdc_line_coding_t *p_line_coding)
{
    p_line_coding->dwDTERate = stm32l0_usbd_cdc_device.line_coding.dwDTERate;
    p_line_coding->bCharFormat = stm32l0_usbd_cdc_device.line_coding.bCharFormat;
    p_line_coding->bParityType = stm32l0_usbd_cdc_device.line_coding.bParityType;
    p_line_coding->bDataBits = stm32l0_usbd_cdc_device.line_coding.bDataBits;     
}

uint16_t stm32l0_usbd_cdc_line_state(stm32l0_usbd_cdc_t *usbd_cdc)
{
    return stm32l0_usbd_cdc_device.line_state;
}

bool stm32l0_usbd_cdc_break_state(stm32l0_usbd_cdc_t *usbd_cdc)
{
    return stm32l0_usbd_cdc_device.break_state;
}

bool stm32l0_usbd_cdc_serial_state(stm32l0_usbd_cdc_t *usbd_cdc, uint16_t serial_state)
{
    if (usbd_cdc->state != STM32L0_USBD_CDC_STATE_READY)
    {
        return false;
    }

    usbd_cdc->serial_state = serial_state;

    if (!stm32l0_usbd_cdc_device.USBD)
    {
        return false;
    }
    
    if (__get_IPSR() == 0)
    {
        armv6m_svcall_0((uint32_t)&stm32l0_usbd_cdc_tx_control);
    }
    else
    {
        armv6m_pendsv_raise(ARMV6M_PENDSV_SWI_USBD_CDC_CONTROL);
    }
    
    return true;
}

uint32_t stm32l0_usbd_cdc_count(stm32l0_usbd_cdc_t *usbd_cdc)
{
    if (usbd_cdc->state != STM32L0_USBD_CDC_STATE_READY)
    {
        return 0;
    }

    usbd_cdc->rx_event = true;

    return usbd_cdc->rx_count;
}

uint32_t stm32l0_usbd_cdc_input(stm32l0_usbd_cdc_t *usbd_cdc, uint8_t *rx_data, uint32_t rx_count, bool consume)
{
    uint32_t rx_size, rx_read;

    if (usbd_cdc->state < STM32L0_USBD_CDC_STATE_READY)
    {
        return 0;
    }

    usbd_cdc->rx_event = true;
    
    if (rx_count > usbd_cdc->rx_count)
    {
        rx_count = usbd_cdc->rx_count;
    }

    if (rx_count)
    {
        rx_read = usbd_cdc->rx_read;
        rx_size = rx_count;

        if ((rx_read + rx_size) > usbd_cdc->rx_size)
        {
            rx_size = usbd_cdc->rx_size - rx_read;
        }

        memcpy(rx_data, &usbd_cdc->rx_data[rx_read], rx_size);

        rx_data += rx_size;
        rx_read += rx_size;

        if (rx_read == usbd_cdc->rx_size)
        {
            rx_read = 0;
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
            usbd_cdc->rx_read = rx_read;

            armv6m_atomic_sub(&usbd_cdc->rx_count, rx_count);
        }
    }
    
    return rx_count;
}

bool stm32l0_usbd_cdc_transmit(stm32l0_usbd_cdc_t *usbd_cdc, const uint8_t *tx_data, uint32_t tx_count, stm32l0_usbd_cdc_done_callback_t callback, void *context)
{
    if (usbd_cdc->state != STM32L0_USBD_CDC_STATE_READY)
    {
        return false;
    }

    if (!(stm32l0_usbd_cdc_device.line_state & USB_CDC_LINE_STATE_DTR))
    {
        return false;
    }

    if (armv6m_atomic_casb(&stm32l0_usbd_cdc_device.tx_busy, STM32L0_USBD_CDC_TX_IDLE, STM32L0_USBD_CDC_TX_WAIT) != STM32L0_USBD_CDC_TX_IDLE)
    {
        return false;
    }

    usbd_cdc->tx_data = tx_data;
    usbd_cdc->tx_count = tx_count;
    usbd_cdc->tx_callback = callback;
    usbd_cdc->tx_context = context;
    
    if (__get_IPSR() == 0)
    {
        armv6m_svcall_0((uint32_t)&stm32l0_usbd_cdc_tx_data);
    }
    else
    {
        armv6m_pendsv_raise(ARMV6M_PENDSV_SWI_USBD_CDC_TRANSMIT);
    }
    
    return true;
}

bool stm32l0_usbd_cdc_done(stm32l0_usbd_cdc_t *usbd_cdc)
{
    return !stm32l0_usbd_cdc_device.tx_busy;
}

void SWI_USBD_CDC_CONTROL_IRQHandler(void)
{
    stm32l0_usbd_cdc_tx_control();
}

void SWI_USBD_CDC_TRANSMIT_IRQHandler(void)
{
    stm32l0_usbd_cdc_tx_data();
}
