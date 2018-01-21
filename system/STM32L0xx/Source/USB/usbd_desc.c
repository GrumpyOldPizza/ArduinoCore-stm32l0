/**
  ******************************************************************************
  * @file    usbd_desc.c
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    26-February-2016
  * @brief   This file provides the USBD descriptors and string formating method.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define USBD_LANGID_STRING            1033

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
const uint8_t *USBD_CDC_MSC_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
const uint8_t *USBD_CDC_MSC_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
const uint8_t *USBD_CDC_MSC_ManufacturerStrDescriptor (USBD_SpeedTypeDef speed, uint16_t *length);
const uint8_t *USBD_CDC_MSC_ProductStrDescriptor (USBD_SpeedTypeDef speed, uint16_t *length);
const uint8_t *USBD_CDC_MSC_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
const uint8_t *USBD_CDC_MSC_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
const uint8_t *USBD_CDC_MSC_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
#ifdef USB_SUPPORT_USER_STRING_DESC
const uint8_t *USBD_CDC_MSC_USRStringDesc (USBD_SpeedTypeDef speed, uint8_t idx, uint16_t *length);  
#endif /* USB_SUPPORT_USER_STRING_DESC */  

/* Private variables ---------------------------------------------------------*/
const USBD_DescriptorsTypeDef CDC_MSC_Desc = {
  USBD_CDC_MSC_DeviceDescriptor,
  USBD_CDC_MSC_LangIDStrDescriptor, 
  USBD_CDC_MSC_ManufacturerStrDescriptor,
  USBD_CDC_MSC_ProductStrDescriptor,
  USBD_CDC_MSC_SerialStrDescriptor,
  USBD_CDC_MSC_ConfigStrDescriptor,
  USBD_CDC_MSC_InterfaceStrDescriptor,  
};

static uint8_t USBD_StringData[64];

extern uint16_t USBD_VendorID;
extern uint16_t USBD_ProductID;
extern const uint8_t * USBD_ManufacturerString;
extern const uint8_t * USBD_ProductString;
extern const uint8_t * USBD_SuffixString;

static const uint8_t USBD_DeviceDescriptor[] = {
  0x12,                       /* bLength */
  0x01,                       /* bDescriptorType */ 
  0x00, 0x02,                 /* bcdUSB */
  0xef,                       /* bDeviceClass */
  0x02,                       /* bDeviceSubClass */
  0x01,                       /* bDeviceProtocol */
  64,                         /* bMaxPacketSize */
  0x00,                       /* idVendor */
  0x00,                       /* idVendor */
  0x00,                       /* idProduct */
  0x00,                       /* idProduct */
  0x00, 0x02,                 /* bcdDevice rel. 2.00 */
  1,                          /* Index of manufacturer string */
  2,                          /* Index of product string */
  3,                          /* Index of serial number string */
  1,                          /* bNumConfigurations */
};

/* USB Standard Device Descriptor */
static const uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] = {
  USB_LEN_LANGID_STR_DESC,         
  USB_DESC_TYPE_STRING,       
  LOBYTE(USBD_LANGID_STRING),
  HIBYTE(USBD_LANGID_STRING), 
};

/* Private functions ---------------------------------------------------------*/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);
static void Get_SerialNum(void);

static void USBD_ConvertString(uint8_t *pbuf, const uint8_t *string)
{
  unsigned int i;

  pbuf[0] = 0;
  pbuf[1] = USB_DESC_TYPE_STRING;

  i = 2;

  while (*string)
  {
      pbuf[i++] = *string++;
      pbuf[i++] = 0;
  }

  pbuf[0] = i;
}

static void USBD_AppendString(uint8_t *pbuf, const uint8_t *string)
{
  unsigned int i;

  i = pbuf[0];

  while (*string)
  {
      pbuf[i++] = *string++;
      pbuf[i++] = 0;
  }

  pbuf[0] = i;
}

/**
  * @brief  Returns the device descriptor. 
  * @param  speed: Current device speed
  * @param  length: Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
const uint8_t *USBD_CDC_MSC_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    memcpy(&USBD_StringData[0], &USBD_DeviceDescriptor[0], sizeof(USBD_DeviceDescriptor));

    USBD_StringData[ 8] = LOBYTE(USBD_VendorID);
    USBD_StringData[ 9] = HIBYTE(USBD_VendorID);
    USBD_StringData[10] = LOBYTE(USBD_ProductID);
    USBD_StringData[11] = HIBYTE(USBD_ProductID);
  
    *length = USBD_StringData[0];
    return &USBD_StringData[0];
}

/**
  * @brief  Returns the LangID string descriptor.        
  * @param  speed: Current device speed
  * @param  length: Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
const uint8_t *USBD_CDC_MSC_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  *length = USBD_LangIDDesc[0];  
  return USBD_LangIDDesc;
}

/**
  * @brief  Returns the manufacturer string descriptor. 
  * @param  speed: Current device speed
  * @param  length: Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
const uint8_t *USBD_CDC_MSC_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    USBD_ConvertString(USBD_StringData, USBD_ManufacturerString);

    *length = USBD_StringData[0];
    return &USBD_StringData[0];
}

/**
  * @brief  Returns the product string descriptor. 
  * @param  speed: Current device speed
  * @param  length: Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
const uint8_t *USBD_CDC_MSC_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    USBD_ConvertString(USBD_StringData, USBD_ProductString);

    if (USBD_SuffixString) {
	USBD_AppendString(USBD_StringData, USBD_SuffixString);
    }

    *length = USBD_StringData[0];
    return &USBD_StringData[0];
}

/**
  * @brief  Returns the serial number string descriptor.        
  * @param  speed: Current device speed
  * @param  length: Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
const uint8_t *USBD_CDC_MSC_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    /* Update the serial number string descriptor with the data from the unique ID*/
    Get_SerialNum();
  
    *length = USBD_StringData[0];
    return &USBD_StringData[0];
}

/**
  * @brief  Returns the configuration string descriptor.    
  * @param  speed: Current device speed
  * @param  length: Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
const uint8_t *USBD_CDC_MSC_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  return USBD_CDC_MSC_GetUsrStrDescriptor(NULL, USBD_IDX_CONFIG_STR, length);
}

/**
  * @brief  Returns the interface string descriptor.        
  * @param  speed: Current device speed
  * @param  length: Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
const uint8_t *USBD_CDC_MSC_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  return USBD_CDC_MSC_GetUsrStrDescriptor(NULL, USBD_IDX_INTERFACE_STR, length);
}

static const char *USBD_StringTable[] = {
  "Serial",           // 4
  "CDC Control",      // 5
  "CDC Data",         // 6
  "Mass Storage",     // 7
  "Keyboard + Mouse", // 8
  "CMSIS-DAP",        // 9
};

const uint8_t *USBD_CDC_MSC_GetUsrStrDescriptor(USBD_HandleTypeDef *pdev, uint8_t index, uint16_t *length)  
{
  USBD_ConvertString(USBD_StringData, (const uint8_t*)USBD_StringTable[index-4]);

    *length = USBD_StringData[0];
    return &USBD_StringData[0];
}

/**
  * @brief  Create the serial number string descriptor 
  * @param  None 
  * @retval None
  */
static void Get_SerialNum(void)
{
  uint32_t deviceserial0, deviceserial1, deviceserial2;
  
  deviceserial0 = *(uint32_t*)DEVICE_ID1;
  deviceserial1 = *(uint32_t*)DEVICE_ID2;
  deviceserial2 = *(uint32_t*)DEVICE_ID3;
  
  deviceserial0 += deviceserial2;
  
  USBD_StringData[0] = USB_SIZ_STRING_SERIAL;
  USBD_StringData[1] = USB_DESC_TYPE_STRING;    

  IntToUnicode (deviceserial0, &USBD_StringData[2] ,8);
  IntToUnicode (deviceserial2, &USBD_StringData[18] ,4);
}

/**
  * @brief  Convert Hex 32Bits value into char 
  * @param  value: value to convert
  * @param  pbuf: pointer to the buffer 
  * @param  len: buffer length
  * @retval None
  */
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;
  
  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10; 
    }
    
    value = value << 4;
    
    pbuf[ 2* idx + 1] = 0;
  }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
