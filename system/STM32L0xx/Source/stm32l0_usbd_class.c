/*
 * Copyright (c) 2017-2020 Thomas Roell.  All rights reserved.
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
#include "stm32l0_usbd_cdc.h"
#include "stm32l0_usbd_msc.h"
#include "stm32l0_usbd_class.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"
#include "usbd_ctlreq.h"

#define USB_CDC_CONFIG_DESC_SIZ   (9+8+(9+5+5+4+5+7)+(9+7+7))
#define USB_MSC_CONFIG_DESC_SIZ   (9+7+7)

#define USB_CDC_INTERFACE_COUNT   2

#define USB_CDC_MSC_CONFIG_DESC_SIZ      (USB_CDC_CONFIG_DESC_SIZ + USB_MSC_CONFIG_DESC_SIZ)

#define USB_CDC_MSC_INTERFACE_COUNT      (STM32L0_USBD_CDC_INTERFACE_COUNT + STM32L0_USBD_MSC_INTERFACE_COUNT)

static const uint8_t USBD_CDC_DeviceQualifierDescriptor[USB_LEN_DEV_QUALIFIER_DESC] =
{
    USB_LEN_DEV_QUALIFIER_DESC,
    USB_DESC_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x01,
    0x00,
};

static const uint8_t USBD_CDC_ConigurationDescriptor[USB_CDC_CONFIG_DESC_SIZ] =
{
    /**** Configuration Descriptor ****/
    0x09,                                                        /* bLength */
    0x02,                                                        /* bDescriptorType */
    LOBYTE(USB_CDC_CONFIG_DESC_SIZ),                             /* wTotalLength */
    HIBYTE(USB_CDC_CONFIG_DESC_SIZ),
    0x02,                                                        /* bNumInterfaces */
    0x01,                                                        /* bConfigurationValue */
    0x00,                                                        /* iConfiguration */
    0xa0,                                                        /* bmAttributes */
    0xfa,                                                        /* bMaxPower */

    /**** IAD to associate the two CDC interfaces ****/
    0x08,                                                        /* bLength */
    0x0b,                                                        /* bDescriptorType */
    0x00,                                                        /* bFirstInterface */
    0x02,                                                        /* bInterfaceCount */
    0x02,                                                        /* bFunctionClass */
    0x02,                                                        /* bFunctionSubClass */
    0x01,                                                        /* bFunctionProtocol */
    0x04,                                                        /* iFunction */

    /**** CDC Control Interface ****/
    0x09,                                                        /* bLength */
    0x04,                                                        /* bDescriptorType */
    0x00,                                                        /* bInterfaceNumber */
    0x00,                                                        /* bAlternateSetting */
    0x01,                                                        /* bNumEndpoints */
    0x02,                                                        /* bInterfaceClass */
    0x02,                                                        /* bInterfaceSubClass */
    0x01,                                                        /* bInterfaceProtocol */
    0x05,                                                        /* iInterface */
  
    /**** CDC Header ****/
    0x05,                                                        /* bLength */
    0x24,                                                        /* bDescriptorType */
    0x00,                                                        /* bDescriptorSubtype */
    0x10,                                                        /* bcdCDC */
    0x01,
  
    /**** CDC Call Management ****/
    0x05,                                                        /* bFunctionLength */
    0x24,                                                        /* bDescriptorType */
    0x01,                                                        /* bDescriptorSubtype */
    0x00,                                                        /* bmCapabilities */
    0x01,                                                        /* bDataInterface */
  
    /**** CDC ACM ****/
    0x04,                                                        /* bFunctionLength */
    0x24,                                                        /* bDescriptorType */
    0x02,                                                        /* bDescriptorSubtype */
    0x02,                                                        /* bmCapabilities */
  
    /**** CDC Union ****/
    0x05,                                                        /* bFunctionLength */
    0x24,                                                        /* bDescriptorType */
    0x06,                                                        /* bDescriptorSubtype */
    0x00,                                                        /* bMasterInterface */
    0x01,                                                        /* bSlaveInterface0 */
  
    /**** CDC Control Endpoint ****/
    0x07,                                                        /* bLength */
    0x05,                                                        /* bDescriptorType */
    STM32L0_USBD_CDC_CONTROL_EP_ADDR,                            /* bEndpointAddress */
    0x03,                                                        /* bmAttributes */
    LOBYTE(STM32L0_USBD_CDC_CONTROL_MAX_PACKET_SIZE),            /* wMaxPacketSize */
    HIBYTE(STM32L0_USBD_CDC_CONTROL_MAX_PACKET_SIZE),
    STM32L0_USBD_CDC_CONTROL_INTERVAL,                           /* bInterval */ 

    /**** CDC Data Interface ****/
    0x09,                                                        /* bLength */
    0x04,                                                        /* bDescriptorType */
    0x01,                                                        /* bInterfaceNumber */
    0x00,                                                        /* bAlternateSetting */
    0x02,                                                        /* bNumEndpoints */
    0x0a,                                                        /* bInterfaceClass */
    0x00,                                                        /* bInterfaceSubClass */
    0x00,                                                        /* bInterfaceProtocol */
    0x06,                                                        /* iInterface */

    /**** CDC Data Endpoint IN ****/
    0x07,                                                        /* bLength */
    0x05,                                                        /* bDescriptorType */
    STM32L0_USBD_CDC_DATA_IN_EP_ADDR,                            /* bEndpointAddress */
    0x02,                                                        /* bmAttributes */
    LOBYTE(STM32L0_USBD_CDC_DATA_MAX_PACKET_SIZE),               /* wMaxPacketSize */
    HIBYTE(STM32L0_USBD_CDC_DATA_MAX_PACKET_SIZE),
    0x00,                                                        /* bInterval */

    /**** CDC Data Endpoint OUT ****/
    0x07,                                                        /* bLength */
    0x05,                                                        /* bDescriptorType */
    STM32L0_USBD_CDC_DATA_OUT_EP_ADDR,                           /* bEndpointAddress */
    0x02,                                                        /* bmAttributes */
    LOBYTE(STM32L0_USBD_CDC_DATA_MAX_PACKET_SIZE),               /* wMaxPacketSize */
    HIBYTE(STM32L0_USBD_CDC_DATA_MAX_PACKET_SIZE),
    0x00,                                                        /* bInterval */
};

static const uint8_t *USBD_CDC_GetFSCfgDesc(uint16_t *length)
{
    *length = sizeof(USBD_CDC_ConigurationDescriptor);

    return USBD_CDC_ConigurationDescriptor;
}

static const uint8_t *USBD_CDC_GetHSCfgDesc(uint16_t *length)
{
    *length = 0;

    return NULL;
}

static const uint8_t *USBD_CDC_GetOtherSpeedCfgDesc(uint16_t *length)
{
    *length = 0;

    return NULL;
}

static const uint8_t *USBD_CDC_GetDeviceQualifierDescriptor(uint16_t *length)
{
    *length = sizeof(USBD_CDC_DeviceQualifierDescriptor);
    
    return USBD_CDC_DeviceQualifierDescriptor;
}

static const USBD_ClassTypeDef USBD_CDC_CLASS = 
{
    USBD_CDC_Init,
    USBD_CDC_DeInit,
    USBD_CDC_Setup,
    NULL,
    USBD_CDC_EP0_RxReady,
    USBD_CDC_DataIn,
    USBD_CDC_DataOut,
    USBD_CDC_SOF,
    NULL,
    NULL,     
    USBD_CDC_GetHSCfgDesc,  
    USBD_CDC_GetFSCfgDesc,    
    USBD_CDC_GetOtherSpeedCfgDesc, 
    USBD_CDC_GetDeviceQualifierDescriptor,
#if (USBD_SUPPORT_USER_STRING == 1)
    USBD_CDC_MSC_GetUsrStrDescriptor,
#endif  
};

static const uint8_t USBD_CDC_MSC_DeviceQualifierDescriptor[USB_LEN_DEV_QUALIFIER_DESC] =
{
    USB_LEN_DEV_QUALIFIER_DESC,
    USB_DESC_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x01,
    0x00,
};

static const uint8_t USBD_CDC_MSC_ConigurationDescriptor[USB_CDC_MSC_CONFIG_DESC_SIZ] =
{
    /**** Configuration Descriptor ****/
    0x09,                                                        /* bLength */
    0x02,                                                        /* bDescriptorType */
    LOBYTE((USB_CDC_MSC_CONFIG_DESC_SIZ)),                       /* wTotalLength */
    HIBYTE((USB_CDC_MSC_CONFIG_DESC_SIZ)),
    0x03,                                                        /* bNumInterfaces */
    0x01,                                                        /* bConfigurationValue */
    0x00,                                                        /* iConfiguration */
    0xa0,                                                        /* bmAttributes */
    0xfa,                                                        /* bMaxPower */

    /**** IAD to associate the two CDC interfaces ****/
    0x08,                                                        /* bLength */
    0x0b,                                                        /* bDescriptorType */
    0x00,                                                        /* bFirstInterface */
    0x02,                                                        /* bInterfaceCount */
    0x02,                                                        /* bFunctionClass */
    0x02,                                                        /* bFunctionSubClass */
    0x01,                                                        /* bFunctionProtocol */
    0x04,                                                        /* iFunction */

    /**** CDC Control Interface ****/
    0x09,                                                        /* bLength */
    0x04,                                                        /* bDescriptorType */
    0x00,                                                        /* bInterfaceNumber */
    0x00,                                                        /* bAlternateSetting */
    0x01,                                                        /* bNumEndpoints */
    0x02,                                                        /* bInterfaceClass */
    0x02,                                                        /* bInterfaceSubClass */
    0x01,                                                        /* bInterfaceProtocol */
    0x05,                                                        /* iInterface */
  
    /**** CDC Header ****/
    0x05,                                                        /* bLength */
    0x24,                                                        /* bDescriptorType */
    0x00,                                                        /* bDescriptorSubtype */
    0x10,                                                        /* bcdCDC */
    0x01,
  
    /**** CDC Call Management ****/
    0x05,                                                        /* bFunctionLength */
    0x24,                                                        /* bDescriptorType */
    0x01,                                                        /* bDescriptorSubtype */
    0x00,                                                        /* bmCapabilities */
    0x01,                                                        /* bDataInterface */
  
    /**** CDC ACM ****/
    0x04,                                                        /* bFunctionLength */
    0x24,                                                        /* bDescriptorType */
    0x02,                                                        /* bDescriptorSubtype */
    0x02,                                                        /* bmCapabilities */
  
    /**** CDC Union ****/
    0x05,                                                        /* bFunctionLength */
    0x24,                                                        /* bDescriptorType */
    0x06,                                                        /* bDescriptorSubtype */
    0x00,                                                        /* bMasterInterface */
    0x01,                                                        /* bSlaveInterface0 */
  
    /**** CDC Control Endpoint ****/
    0x07,                                                        /* bLength */
    0x05,                                                        /* bDescriptorType */
    STM32L0_USBD_CDC_CONTROL_EP_ADDR,                            /* bEndpointAddress */
    0x03,                                                        /* bmAttributes */
    LOBYTE(STM32L0_USBD_CDC_CONTROL_MAX_PACKET_SIZE),            /* wMaxPacketSize */
    HIBYTE(STM32L0_USBD_CDC_CONTROL_MAX_PACKET_SIZE),
    STM32L0_USBD_CDC_CONTROL_INTERVAL,                           /* bInterval */ 

    /**** CDC Data Interface ****/
    0x09,                                                        /* bLength */
    0x04,                                                        /* bDescriptorType */
    0x01,                                                        /* bInterfaceNumber */
    0x00,                                                        /* bAlternateSetting */
    0x02,                                                        /* bNumEndpoints */
    0x0a,                                                        /* bInterfaceClass */
    0x00,                                                        /* bInterfaceSubClass */
    0x00,                                                        /* bInterfaceProtocol */
    0x06,                                                        /* iInterface */

    /**** CDC Data Endpoint IN ****/
    0x07,                                                        /* bLength */
    0x05,                                                        /* bDescriptorType */
    STM32L0_USBD_CDC_DATA_IN_EP_ADDR,                            /* bEndpointAddress */
    0x02,                                                        /* bmAttributes */
    LOBYTE(STM32L0_USBD_CDC_DATA_MAX_PACKET_SIZE),               /* wMaxPacketSize */
    HIBYTE(STM32L0_USBD_CDC_DATA_MAX_PACKET_SIZE),
    0x00,                                                        /* bInterval */

    /**** CDC Data Endpoint OUT ****/
    0x07,                                                        /* bLength */
    0x05,                                                        /* bDescriptorType */
    STM32L0_USBD_CDC_DATA_OUT_EP_ADDR,                           /* bEndpointAddress */
    0x02,                                                        /* bmAttributes */
    LOBYTE(STM32L0_USBD_CDC_DATA_MAX_PACKET_SIZE),               /* wMaxPacketSize */
    HIBYTE(STM32L0_USBD_CDC_DATA_MAX_PACKET_SIZE),
    0x00,                                                        /* bInterval */

    /**** MSC Interface ****/
    0x09,                                                        /* bLength */
    0x04,                                                        /* bDescriptorType */
    0x02,                                                        /* bInterfaceNumber */
    0x00,                                                        /* bAlternateSetting */
    0x02,                                                        /* bNumEndpoints */
    0x08,                                                        /* bInterfaceClass */
    0x06,                                                        /* bInterfaceSubClass */
    0x50,                                                        /* nInterfaceProtocol */
    0x07,                                                        /* iInterface */

    /**** MSC Endpoint IN ****/
    0x07,                                                        /* bLength */
    0x05,                                                        /* bDescriptorType */
    STM32L0_USBD_MSC_DATA_IN_EP_ADDR,                            /* bEndpointAddress */
    0x02,                                                        /* bmAttributes */
    LOBYTE(STM32L0_USBD_MSC_DATA_MAX_PACKET_SIZE),               /* wMaxPacketSize */
    HIBYTE(STM32L0_USBD_MSC_DATA_MAX_PACKET_SIZE),
    0x00,                                                        /* bInterval */

    /**** MSC Endpoint OUT ****/
    0x07,                                                        /* bLength */
    0x05,                                                        /* bDescriptorType */
    STM32L0_USBD_MSC_DATA_OUT_EP_ADDR,                           /* bEndpointAddress */
    0x02,                                                        /* bmAttributes */
    LOBYTE(STM32L0_USBD_MSC_DATA_MAX_PACKET_SIZE),               /* wMaxPacketSize */
    HIBYTE(STM32L0_USBD_MSC_DATA_MAX_PACKET_SIZE),
    0x00,                                                        /* bInterval */
};

static uint8_t USBD_CDC_MSC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    USBD_CDC_Init(pdev, cfgidx);
    USBD_MSC_Init(pdev, cfgidx);
    
    return USBD_OK;
}

static uint8_t USBD_CDC_MSC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    USBD_MSC_DeInit(pdev, cfgidx);
    USBD_CDC_DeInit(pdev, cfgidx);
    
    return USBD_OK;
}

static uint8_t USBD_CDC_MSC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    switch (req->bmRequest & USB_REQ_RECIPIENT_MASK) {
    case USB_REQ_RECIPIENT_INTERFACE:
	if ((req->wIndex == STM32L0_USBD_CDC_DATA_INTERFACE) || (req->wIndex == STM32L0_USBD_CDC_CONTROL_INTERFACE))
        {
	    return USBD_CDC_Setup(pdev, req);
        }
	else
        {
	    return USBD_MSC_Setup(pdev, req);
        }
	break;
	
    case USB_REQ_RECIPIENT_ENDPOINT:
	if ((req->wIndex == STM32L0_USBD_CDC_DATA_IN_EP_ADDR) || (req->wIndex == STM32L0_USBD_CDC_DATA_OUT_EP_ADDR) || (req->wIndex == STM32L0_USBD_CDC_CONTROL_EP_ADDR))
	{
	    return USBD_CDC_Setup(pdev, req);
	}
	else
        {
	    return USBD_MSC_Setup(pdev, req);
        }
	break;

    default:
	break;
    }   

    return USBD_OK;
}

static uint8_t USBD_CDC_MSC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    if ((epnum == (STM32L0_USBD_CDC_DATA_IN_EP_ADDR & 15)) || (epnum == (STM32L0_USBD_CDC_CONTROL_EP_ADDR & 15)))
    {
	return USBD_CDC_DataIn(pdev, epnum);
    }
    else
    {
	return USBD_MSC_DataIn(pdev, epnum);
    }
}

static uint8_t USBD_CDC_MSC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{      
    if (epnum == (STM32L0_USBD_CDC_DATA_IN_EP_ADDR & 15))
    {
	return USBD_CDC_DataOut(pdev, epnum);
    }
    else
    {
	return USBD_MSC_DataOut(pdev, epnum);
    }
}

static const uint8_t *USBD_CDC_MSC_GetFSCfgDesc(uint16_t *length)
{
    *length = sizeof(USBD_CDC_MSC_ConigurationDescriptor);

    return USBD_CDC_MSC_ConigurationDescriptor;
}

static const uint8_t *USBD_CDC_MSC_GetHSCfgDesc(uint16_t *length)
{
    *length = 0;

    return NULL;
}

static const uint8_t *USBD_CDC_MSC_GetOtherSpeedCfgDesc(uint16_t *length)
{
    *length = 0;

    return NULL;
}

static const uint8_t *USBD_CDC_MSC_GetDeviceQualifierDescriptor(uint16_t *length)
{
    *length = sizeof(USBD_CDC_MSC_DeviceQualifierDescriptor);
    
    return USBD_CDC_MSC_DeviceQualifierDescriptor;
}

static const USBD_ClassTypeDef USBD_CDC_MSC_CLASS = 
{
    USBD_CDC_MSC_Init,
    USBD_CDC_MSC_DeInit,
    USBD_CDC_MSC_Setup,
    NULL,
    USBD_CDC_EP0_RxReady,
    USBD_CDC_MSC_DataIn,
    USBD_CDC_MSC_DataOut,
    USBD_CDC_SOF,
    NULL,
    NULL,     
    USBD_CDC_MSC_GetHSCfgDesc,  
    USBD_CDC_MSC_GetFSCfgDesc,    
    USBD_CDC_MSC_GetOtherSpeedCfgDesc, 
    USBD_CDC_MSC_GetDeviceQualifierDescriptor,
#if (USBD_SUPPORT_USER_STRING == 1)
    USBD_CDC_MSC_GetUsrStrDescriptor,
#endif  
};

void USBD_CDC_Initialize(USBD_HandleTypeDef *pdev)
{
    USBD_RegisterClass(pdev, &USBD_CDC_CLASS);
}

void USBD_CDC_MSC_Initialize(USBD_HandleTypeDef *pdev)
{
    USBD_RegisterClass(pdev, &USBD_CDC_MSC_CLASS);
}
