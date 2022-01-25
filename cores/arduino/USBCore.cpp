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

#include "Arduino.h"
#include "USBAPI.h"
#include "wiring_private.h"

#if defined(USBCON)

#define USB_TYPE_NONE        0
#define USB_TYPE_CDC         1
#define USB_TYPE_CDC_MSC     2

#if !defined(USB_TYPE)
#define USB_VID 0x1209
#define USB_PID 0x6665
#define USB_MANUFACTURER "Tlera Corporation"
#define USB_PRODUCT "CMWX1ZZABZ"
#define USB_CLASS USBD_CDC_MSC_Initialize
#else
#if (USB_TYPE == USB_TYPE_CDC)
#define USB_CLASS USBD_CDC_Initialize
#endif
#if (USB_TYPE == USB_TYPE_CDC_MSC)
#define USB_CLASS USBD_CDC_MSC_Initialize
#endif
#endif

bool USBDeviceClass::begin()
{
#if defined(USB_CLASS)
    #if defined(STM32L0_CONFIG_PIN_VBUS_HAS_DIVIDER)
    // On boards with an external divider (to allow measuring
    // voltage rather than just on/off), do not enable the pulldown
    // since that influences the divider (and the divider works as
    // an external pulldown too).
    bool needs_pulldown = false;
    #else
    bool needs_pulldown = true;
    #endif
    return USBD_Initialize(USB_VID, USB_PID, (const uint8_t*)USB_MANUFACTURER, (const uint8_t*)USB_PRODUCT, USB_CLASS,
                           STM32L0_CONFIG_PIN_VBUS, needs_pulldown, STM32L0_USB_IRQ_PRIORITY,
                           &USBDeviceClass::connectCallback, &USBDeviceClass::disconnectCallback, &USBDeviceClass::suspendCallback, &USBDeviceClass::resumeCallback);
#endif
    return false;
}

void USBDeviceClass::end()
{
#if defined(USB_CLASS)
    USBD_Teardown();
#endif    
}

void USBDeviceClass::attach()
{
#if defined(USB_CLASS)
    USBD_Attach();
#endif
}

void USBDeviceClass::detach()
{
#if defined(USB_CLASS)
    USBD_Detach();
#endif
}

void USBDeviceClass::wakeup()
{
#if defined(USB_CLASS)
    USBD_Wakeup();
#endif
}
    
bool USBDeviceClass::attached()
{
#if defined(USB_CLASS)
    return USBD_Attached();
#else
    return false;
#endif
}

bool USBDeviceClass::connected()
{
#if defined(USB_CLASS)
    return USBD_Connected();
#else
    return false;
#endif
}

bool USBDeviceClass::configured()
{
#if defined(USB_CLASS)
    return USBD_Configured();
#else
    return false;
#endif
}

bool USBDeviceClass::suspended()
{
#if defined(USB_CLASS)
    return USBD_Suspended();
#else
    return false;
#endif
}

void USBDeviceClass::onConnect(void(*callback)(void))
{
    _connectCallback = Callback(callback);
}

void USBDeviceClass::onConnect(Callback callback)
{
    _connectCallback = callback;
}

void USBDeviceClass::onDisconnect(void(*callback)(void))
{
    _disconnectCallback = Callback(callback);
}

void USBDeviceClass::onDisconnect(Callback callback)
{
    _disconnectCallback = callback;
}

void USBDeviceClass::onSuspend(void(*callback)(void))
{
    _suspendCallback = Callback(callback);
}

void USBDeviceClass::onSuspend(Callback callback)
{
    _suspendCallback = callback;
}

void USBDeviceClass::onResume(void(*callback)(void))
{
    _resumeCallback = Callback(callback);
}

void USBDeviceClass::onResume(Callback callback)
{
    _resumeCallback = callback;
}

void USBDeviceClass::enableWakeup()
{
    _wakeup = true;
}

void USBDeviceClass::disableWakeup()
{
    _wakeup = false;
}

void USBDeviceClass::setVBUSDetect(enum USBDeviceDetect mode)
{
#if defined(USB_CLASS)
    USBD_SetupVBUS(mode == SLEEP_AND_STOP);
#else
    (void)mode;
#endif
}

void USBDeviceClass::connectCallback(void)
{
    USBDevice._connectCallback.queue(USBDevice._wakeup);
}

void USBDeviceClass::disconnectCallback(void)
{
    USBDevice._disconnectCallback.queue(USBDevice._wakeup);
}

void USBDeviceClass::suspendCallback(void)
{
    USBDevice._suspendCallback.queue(USBDevice._wakeup);
}

void USBDeviceClass::resumeCallback(void)
{
    USBDevice._resumeCallback.queue(USBDevice._wakeup);
}

USBDeviceClass USBDevice;

#endif /* USBCON */

