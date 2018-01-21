/**
  ******************************************************************************
  * @file    usbd_hid.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   header file for the usbd_hid.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
 
/* Define to prevent recursive inclusion -------------------------------------*/ 
#ifndef __USB_HID_H
#define __USB_HID_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */
  
/** @defgroup USBD_HID
  * @brief This file is the Header file for USBD_hid.c
  * @{
  */ 


/** @defgroup USBD_HID_Exported_Defines
  * @{
  */ 
#define HID_EPIN_ADDR                 0x84
#define HID_EPOUT_ADDR                0x04

#define HID_EPIN_SIZE                 64
#define HID_EPOUT_SIZE                64

#define HID_INTERFACE                 3

#define USB_HID_DESC_SIZE             9

#define HID_HID_DESCRIPTOR_TYPE       0x21
#define HID_REPORT_DESCRIPTOR_TYPE    0x22
#define HID_PHYSICAL_DESCRIPTOR_TYPE  0x23

#define HID_REQ_SET_PROTOCOL          0x0B
#define HID_REQ_GET_PROTOCOL          0x03

#define HID_REQ_SET_IDLE              0x0A
#define HID_REQ_GET_IDLE              0x02

#define HID_REQ_SET_REPORT            0x09
#define HID_REQ_GET_REPORT            0x01
/**
  * @}
  */ 

#define HID_REPORT_TYPE_INPUT         0x01
#define HID_REPORT_TYPE_OUTPUT        0x02
#define HID_REPORT_TYPE_FEATURE       0x03


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */

typedef struct _USBD_HID_Itf
{
  void     (* Init)        (USBD_HandleTypeDef *pdev);
  void     (* DeInit)      (void);
  uint32_t (* GetReport)   (uint8_t, uint8_t, uint8_t *);  
  void     (* SetReport)   (uint8_t, uint8_t, const uint8_t *, uint32_t);  
  void     (* RxReady)     (const uint8_t *, uint32_t);  
  void     (* TxDone)      (void);  
}USBD_HID_ItfTypeDef;

typedef struct
{
  uint32_t             ReportData[HID_EPOUT_SIZE/4]; /* Force 32bits alignment */
  uint32_t             EventData[HID_EPOUT_SIZE/4];  /* Force 32bits alignment */
  uint8_t              Protocol;   
  uint8_t              IdleState;  
  uint8_t              SetReportBusy;  
  uint8_t              SendReportBusy;  
  uint8_t              ReportType;
  uint8_t              ReportID;
}
USBD_HID_HandleTypeDef; 
/**
  * @}
  */ 



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */ 
uint8_t USBD_HID_SendReport (USBD_HandleTypeDef *pdev, 
                             uint8_t *report,
                             uint32_t len);



uint8_t  USBD_HID_RegisterInterface  (USBD_HandleTypeDef  *pdev, 
                                      const USBD_HID_ItfTypeDef *fops,
				      const uint8_t *pReportDescriptor,
				      uint16_t ReportDescriptorLength);

/**
  * @}
  */ 

uint8_t  USBD_HID_Init (USBD_HandleTypeDef *pdev, 
			uint8_t cfgidx);

uint8_t  USBD_HID_DeInit (USBD_HandleTypeDef *pdev, 
			  uint8_t cfgidx);

uint8_t  USBD_HID_Setup (USBD_HandleTypeDef *pdev, 
			 USBD_SetupReqTypedef *req);

uint8_t  USBD_HID_DataIn (USBD_HandleTypeDef *pdev, 
			  uint8_t epnum);

uint8_t  USBD_HID_DataOut (USBD_HandleTypeDef *pdev, 
			   uint8_t epnum);

uint8_t  USBD_HID_EP0_RxReady (USBD_HandleTypeDef *pdev);

#ifdef __cplusplus
}
#endif

#endif  /* __USB_HID_H */
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
