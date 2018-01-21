/**
  ******************************************************************************
  * @file    usbd_hid.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides the HID core functions.
  *
  * @verbatim
  *      
  *          ===================================================================      
  *                                HID Class  Description
  *          =================================================================== 
  *           This module manages the HID class V1.11 following the "Device Class Definition
  *           for Human Interface Devices (HID) Version 1.11 Jun 27, 2001".
  *           This driver implements the following aspects of the specification:
  *             - The Boot Interface Subclass
  *             - Usage Page : Generic Desktop
  *             - Usage : Vendor
  *             - Collection : Application 
  *      
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *           
  *      
  *  @endverbatim
  *
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

/* Includes ------------------------------------------------------------------*/
#include "usbd_hid.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_HID 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_HID_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_HID_Private_Defines
  * @{
  */ 

/**
  * @}
  */ 


/** @defgroup USBD_HID_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 
/** @defgroup USBD_HID_Private_FunctionPrototypes
  * @{
  */


uint8_t  USBD_HID_Init (USBD_HandleTypeDef *pdev, 
			uint8_t cfgidx);

uint8_t  USBD_HID_DeInit (USBD_HandleTypeDef *pdev, 
			  uint8_t cfgidx);

uint8_t  USBD_HID_Setup (USBD_HandleTypeDef *pdev, 
			 USBD_SetupReqTypedef *req);

uint8_t  USBD_HID_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);

uint8_t  USBD_HID_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);
uint8_t  USBD_HID_EP0_RxReady (USBD_HandleTypeDef  *pdev);

/**
  * @}
  */ 

static USBD_HID_HandleTypeDef USBD_HID_Handle;

static const uint8_t *USDB_HID_ReportDescriptor;
static uint16_t USDB_HID_ReportDescriptorLength;

/** @defgroup USBD_HID_Private_Functions
  * @{
  */ 

/**
  * @brief  USBD_HID_Init
  *         Initialize the HID interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
uint8_t  USBD_HID_Init (USBD_HandleTypeDef *pdev, 
			uint8_t cfgidx)
{
  USBD_HID_HandleTypeDef     *hhid;

  /* Open EP IN */
  USBD_LL_OpenEP(pdev,
                 HID_EPIN_ADDR,
                 USBD_EP_TYPE_INTR,
                 HID_EPIN_SIZE);  
  
  /* Open EP OUT */
  USBD_LL_OpenEP(pdev,
                 HID_EPOUT_ADDR,
                 USBD_EP_TYPE_INTR,
                 HID_EPOUT_SIZE);
  
  pdev->pClassData[2] = &USBD_HID_Handle;
  
  hhid = (USBD_HID_HandleTypeDef*) pdev->pClassData[2];

  hhid->Protocol = 0;
  hhid->IdleState = 0;
  hhid->SetReportBusy = 0;
  hhid->SendReportBusy = 0;

  ((const USBD_HID_ItfTypeDef *)pdev->pUserData[2])->Init(pdev);

  /* Prepare Out endpoint to receive 1st packet */ 
  USBD_LL_PrepareReceive(pdev,
			 HID_EPOUT_ADDR,
			 (uint8_t*)&hhid->EventData[0], 
			 HID_EPOUT_SIZE);
    
  return USBD_OK;
}

/**
  * @brief  USBD_HID_DeInit
  *         DeInitialize the HID layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
uint8_t  USBD_HID_DeInit (USBD_HandleTypeDef *pdev, 
			  uint8_t cfgidx)
{
  /* Close HID EP IN */
  USBD_LL_CloseEP(pdev,
                  HID_EPIN_ADDR);
  
  /* Close HID EP OUT */
  USBD_LL_CloseEP(pdev,
                  HID_EPOUT_ADDR);
  
  /* FRee allocated memory */
  if(pdev->pClassData[2] != NULL)
  {
    ((const USBD_HID_ItfTypeDef *)pdev->pUserData[2])->DeInit();
    pdev->pClassData[2] = NULL;
  }

  return USBD_OK;
}

/**
  * @brief  USBD_HID_Setup
  *         Handle the HID specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
uint8_t  USBD_HID_Setup (USBD_HandleTypeDef *pdev, 
			 USBD_SetupReqTypedef *req)
{
  uint16_t len = 0;
  const uint8_t *pbuf = NULL;
  USBD_HID_HandleTypeDef     *hhid = (USBD_HID_HandleTypeDef*)pdev->pClassData[2];
  static uint8_t ifalt = 0;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS :  
    switch (req->bRequest)
    {
    case HID_REQ_SET_PROTOCOL:
      hhid->Protocol = (uint8_t)(req->wValue);
      break;
      
    case HID_REQ_GET_PROTOCOL:
      USBD_CtlSendData (pdev, 
                        (uint8_t *)&hhid->Protocol,
                        1);    
      break;
      
    case HID_REQ_SET_IDLE:
      hhid->IdleState = (uint8_t)(req->wValue >> 8);
      break;
      
    case HID_REQ_GET_IDLE:
      USBD_CtlSendData (pdev, 
                        (uint8_t *)&hhid->IdleState,
                        1);        
      break;      

    case HID_REQ_GET_REPORT:
      len = ((const USBD_HID_ItfTypeDef *)pdev->pUserData[2])->GetReport(req->wValue >> 8, req->wValue & 0xff, (uint8_t*)&hhid->ReportData[0]);
      USBD_CtlSendData (pdev, 
			(uint8_t*)&hhid->ReportData[0],
			MIN(len, req->wLength));
      break;
    case HID_REQ_SET_REPORT:
      hhid->SetReportBusy = 1;
      hhid->ReportType = req->wValue >> 8;
      hhid->ReportID = req->wValue & 0xff;
      USBD_CtlPrepareRx (pdev, (uint8_t*)&hhid->ReportData[0], (uint8_t)(req->wLength));
      break;
    default:
      USBD_CtlError (pdev, req);
      return USBD_FAIL; 
    }
    break;
    
  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_DESCRIPTOR: 
      if( (req->wValue >> 8) == HID_REPORT_DESCRIPTOR_TYPE)
      {
	pbuf = USDB_HID_ReportDescriptor;
	len = USDB_HID_ReportDescriptorLength;
      }
      
      USBD_CtlSendData (pdev, 
			(uint8_t*)pbuf,
			MIN(len, req->wLength));
      break;
      
    case USB_REQ_GET_INTERFACE :
      USBD_CtlSendData (pdev,
                        &ifalt,
                        1);
      break;
    }
  }
  return USBD_OK;
}

/**
  * @brief  USBD_HID_SendReport 
  *         Send HID Report
  * @param  pdev: device instance
  * @param  buff: pointer to report
  * @retval status
  */
uint8_t USBD_HID_SendReport     (USBD_HandleTypeDef  *pdev, 
                                 uint8_t *report,
                                 uint32_t len)
{
  USBD_HID_HandleTypeDef     *hhid = (USBD_HID_HandleTypeDef*)pdev->pClassData[2];
  
  if (pdev->dev_state == USBD_STATE_CONFIGURED )
  {
    if(!hhid->SendReportBusy)
    {
      hhid->SendReportBusy = 1;

      USBD_LL_Transmit (pdev, 
                        HID_EPIN_ADDR,                                      
                        report,
                        len);
    }
  }
  return USBD_OK;
}

/**
  * @brief  USBD_HID_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
uint8_t  USBD_HID_DataIn (USBD_HandleTypeDef *pdev, 
			  uint8_t epnum)
{
  
  /* Ensure that the FIFO is empty before a new transfer, this condition could 
     be caused by  a new transfer before the end of the previous transfer */
  ((USBD_HID_HandleTypeDef *)pdev->pClassData[2])->SendReportBusy = 0;

  ((const USBD_HID_ItfTypeDef *)pdev->pUserData[2])->TxDone();
  
  return USBD_OK;
}

/**
  * @brief  USBD_HID_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
uint8_t  USBD_HID_DataOut (USBD_HandleTypeDef *pdev, 
			   uint8_t epnum)
{
  
  USBD_HID_HandleTypeDef     *hhid = (USBD_HID_HandleTypeDef*)pdev->pClassData[2];  
  
  ((const USBD_HID_ItfTypeDef *)pdev->pUserData[2])->RxReady((uint8_t*)&hhid->EventData[0], USBD_LL_GetRxDataSize (pdev, epnum));
    
  USBD_LL_PrepareReceive(pdev,
			 HID_EPOUT_ADDR,
			 (uint8_t*)&hhid->EventData[0], 
			 HID_EPOUT_SIZE);

  return USBD_OK;
}

/**
  * @brief  USBD_HID_EP0_RxReady
  *         Handles control request data.
  * @param  pdev: device instance
  * @retval status
  */
uint8_t USBD_HID_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  USBD_HID_HandleTypeDef     *hhid = (USBD_HID_HandleTypeDef*)pdev->pClassData[2];  

  if (hhid->SetReportBusy)
  {
    hhid->SetReportBusy = 0;      

    ((const USBD_HID_ItfTypeDef *)pdev->pUserData[2])->SetReport(hhid->ReportType, hhid->ReportID, (uint8_t*)&hhid->ReportData[0], USBD_LL_GetRxDataSize (pdev, 0));
  }

  return USBD_OK;
}

/**
* @brief  USBD_HID_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: HID Interface callback
  * @retval status
  */
uint8_t  USBD_HID_RegisterInterface  (USBD_HandleTypeDef  *pdev, 
                                      const USBD_HID_ItfTypeDef *fops,
				      const uint8_t *pReportDescriptor,
				      uint16_t ReportDescriptorLength)
{
  USDB_HID_ReportDescriptor = pReportDescriptor;
  USDB_HID_ReportDescriptorLength = ReportDescriptorLength;
  
  pdev->pUserData[2] = fops;

  return USBD_OK;    
}

/**
  * @}
  */ 


/**
  * @}
  */ 


/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
