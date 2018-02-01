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
#include "stm32l0_gpio.h"
#include "stm32l0_usbd_hid.h"
#include "usbd_hid.h"

typedef struct _stm32l0_usbd_hid_device_t {
    struct _USBD_HandleTypeDef     *USBD;
    uint8_t                        tx_data[64];
    volatile uint32_t              tx_busy;
} stm32l0_usbd_hid_device_t;

static stm32l0_usbd_hid_device_t stm32l0_usbd_hid_device;

static void stm32l0_usbd_hid_init(USBD_HandleTypeDef *USBD)
{
    stm32l0_usbd_hid_device.USBD = USBD;
    stm32l0_usbd_hid_device.tx_busy = 0;
}

static void stm32l0_usbd_hid_deinit(void)
{
    stm32l0_usbd_hid_device.tx_busy = 0;

    stm32l0_usbd_hid_device.USBD = NULL;
}

static uint32_t stm32l0_usbd_hid_get_report(uint8_t type, uint8_t id, uint8_t *data)
{
  return 0;
}

static void stm32l0_usbd_hid_set_report(uint8_t type, uint8_t id, const uint8_t *data, uint32_t length)
{
}

static void stm32l0_usbd_hid_rx_ready(const uint8_t *data, uint32_t length)
{
}

static void stm32l0_usbd_hid_tx_done(void)
{
    stm32l0_usbd_hid_device.tx_busy = 0;
}

const USBD_HID_ItfTypeDef stm32l0_usbd_hid_interface = {
    stm32l0_usbd_hid_init,
    stm32l0_usbd_hid_deinit,
    stm32l0_usbd_hid_get_report,
    stm32l0_usbd_hid_set_report,
    stm32l0_usbd_hid_rx_ready,
    stm32l0_usbd_hid_tx_done,
};

/* Transmission is done at the PENDSV/SVC level while the USB interrupt is disabled.
 * to avoid concurrency issues between CDC/HID/MSC and incoming interrupts.
 */
static void stm32l0_usbd_hid_do_send_report(uint8_t *tx_data, uint32_t tx_count)
{
    NVIC_DisableIRQ(USB_IRQn);

    USBD_HID_SendReport(stm32l0_usbd_hid_device.USBD, tx_data, tx_count);
        
    NVIC_EnableIRQ(USB_IRQn);
}

bool stm32l0_usbd_hid_send_report(uint8_t id, const uint8_t *data, uint32_t count)
{
    bool success = true;

    if (armv6m_atomic_swap(&stm32l0_usbd_hid_device.tx_busy, 1))
    {
        return false;
    }
    else
    {
        stm32l0_usbd_hid_device.tx_data[0] = id;
        memcpy(&stm32l0_usbd_hid_device.tx_data[1], data, count);

        if (__get_IPSR() == 0)
        {
            armv6m_svcall_2((uint32_t)&stm32l0_usbd_hid_do_send_report, (uint32_t)&stm32l0_usbd_hid_device.tx_data[0], (count+1));
        }
        else
        {
            success = armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)stm32l0_usbd_hid_do_send_report, &stm32l0_usbd_hid_device.tx_data[0], (count+1));
        }

        return success;
    }
}

bool stm32l0_usbd_hid_done()
{
    return !stm32l0_usbd_hid_device.tx_busy;
}
