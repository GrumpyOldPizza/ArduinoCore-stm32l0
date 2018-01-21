/**
  ******************************************************************************
  * @file    usbd_msc.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides all the MSC core functions.
  *
  * @verbatim
  *      
  *          ===================================================================      
  *                                MSC Class  Description
  *          =================================================================== 
  *           This module manages the MSC class V1.0 following the "Universal 
  *           Serial Bus Mass Storage Class (MSC) Bulk-Only Transport (BOT) Version 1.0
  *           Sep. 31, 1999".
  *           This driver implements the following aspects of the specification:
  *             - Bulk-Only Transport protocol
  *             - Subclass : SCSI transparent command set (ref. SCSI Primary Commands - 3 (SPC-3))
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
#include "usbd_msc.h"

#include "armv6m.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup MSC_CORE 
  * @brief Mass storage core module
  * @{
  */ 

/** @defgroup MSC_CORE_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup MSC_CORE_Private_Defines
  * @{
  */ 

static armv6m_event_t USBD_MSC_Event;

void USBD_MSC_Notify(uint8_t lun, int acquire)
{
    if (acquire)
    {
	armv6m_event_lock(&USBD_MSC_Event);
    }
    else
    {
	armv6m_event_unlock(&USBD_MSC_Event);
    }
}

#define USBD_MSC_BOT_EVENT_DATA_IN           0x00000001
#define USBD_MSC_BOT_EVENT_DATA_OUT          0x00000002
#define USBD_MSC_BOT_EVENT_CLEAR_FEATURE_IN  0x00000004
#define USBD_MSC_BOT_EVENT_CLEAR_FEATURE_OUT 0x00000008
#define USBD_MSC_BOT_EVENT_RESET             0x00000010

static volatile uint32_t USBD_MSC_Pending = 0;

static void USBD_MSC_Callback(void *context)
{
    uint32_t events;
    USBD_HandleTypeDef *pdev = (USBD_HandleTypeDef *)context;

    events = armv6m_atomic_swap(&USBD_MSC_Pending, 0);
  
    if (events & USBD_MSC_BOT_EVENT_DATA_IN)
    {
	MSC_BOT_DataIn(pdev, MSC_EPIN_ADDR);
    }

    if (events & USBD_MSC_BOT_EVENT_DATA_OUT)
    {
	MSC_BOT_DataOut(pdev, MSC_EPOUT_ADDR);
    }

    if (events & USBD_MSC_BOT_EVENT_CLEAR_FEATURE_IN)
    {
	MSC_BOT_CplClrFeature(pdev, MSC_EPIN_ADDR);
    }

    if (events & USBD_MSC_BOT_EVENT_CLEAR_FEATURE_OUT)
    {
	MSC_BOT_CplClrFeature(pdev, MSC_EPOUT_ADDR);
    }

    if (events & USBD_MSC_BOT_EVENT_RESET)
    {
	MSC_BOT_Reset(pdev);
    }
}

static void USBD_MSC_Enqueue(uint32_t events)
{
    if (USBD_MSC_Pending)
    {
	USBD_MSC_Pending |= events;
    }
    else
    {
	USBD_MSC_Pending = events;

	armv6m_event_enqueue(&USBD_MSC_Event);
    }
}

/**
  * @}
  */ 


/** @defgroup MSC_CORE_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup MSC_CORE_Private_FunctionPrototypes
  * @{
  */ 
uint8_t  USBD_MSC_Init (USBD_HandleTypeDef *pdev, 
                            uint8_t cfgidx);

uint8_t  USBD_MSC_DeInit (USBD_HandleTypeDef *pdev, 
                              uint8_t cfgidx);

uint8_t  USBD_MSC_Setup (USBD_HandleTypeDef *pdev, 
                             USBD_SetupReqTypedef *req);

uint8_t  USBD_MSC_DataIn (USBD_HandleTypeDef *pdev, 
                              uint8_t epnum);


uint8_t  USBD_MSC_DataOut (USBD_HandleTypeDef *pdev, 
                               uint8_t epnum);

/**
  * @}
  */ 


/** @defgroup MSC_CORE_Private_Variables
  * @{
  */ 

static USBD_MSC_BOT_HandleTypeDef USBD_MSC_BOT_Handle;

/** @defgroup MSC_CORE_Private_Functions
  * @{
  */ 

/**
  * @brief  USBD_MSC_Init
  *         Initialize  the mass storage configuration
  * @param  pdev: device instance
  * @param  cfgidx: configuration index
  * @retval status
  */
uint8_t  USBD_MSC_Init (USBD_HandleTypeDef *pdev, 
                            uint8_t cfgidx)
{
  int16_t ret = 0;
   
  /* Open EP OUT */
  USBD_LL_OpenEP(pdev,
		 MSC_EPOUT_ADDR,
		 USBD_EP_TYPE_BULK,
		 MSC_MAX_FS_PACKET);
  
  /* Open EP IN */
  USBD_LL_OpenEP(pdev,
		 MSC_EPIN_ADDR,
		 USBD_EP_TYPE_BULK,
		 MSC_MAX_FS_PACKET);  

  pdev->pClassData[1] = &USBD_MSC_BOT_Handle;
  
  if(pdev->pClassData[1] == NULL)
  {
    ret = 1; 
  }
  else
  {
    /* Init the BOT  layer */
    MSC_BOT_Init(pdev); 
    ret = 0;
  }

  armv6m_event_create(&USBD_MSC_Event, USBD_MSC_Callback, pdev, 8);

  return ret;
}

/**
  * @brief  USBD_MSC_DeInit
  *         DeInitilaize  the mass storage configuration
  * @param  pdev: device instance
  * @param  cfgidx: configuration index
  * @retval status
  */
uint8_t  USBD_MSC_DeInit (USBD_HandleTypeDef *pdev, 
                              uint8_t cfgidx)
{
  /* Close MSC EPs */
  USBD_LL_CloseEP(pdev,
                  MSC_EPOUT_ADDR);
  
  /* Open EP IN */
  USBD_LL_CloseEP(pdev,
                  MSC_EPIN_ADDR);
  
  
    /* De-Init the BOT layer */
  MSC_BOT_DeInit(pdev);
  
  /* Free MSC Class Resources */
  if(pdev->pClassData[1] != NULL)
  {
    pdev->pClassData[1]  = NULL; 
  }
  return 0;
}
/**
* @brief  USBD_MSC_Setup
*         Handle the MSC specific requests
* @param  pdev: device instance
* @param  req: USB request
* @retval status
*/
uint8_t  USBD_MSC_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_MSC_BOT_HandleTypeDef     *hmsc = (USBD_MSC_BOT_HandleTypeDef*) pdev->pClassData[1];
  
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {

  /* Class request */
  case USB_REQ_TYPE_CLASS :
    switch (req->bRequest)
    {
    case BOT_GET_MAX_LUN :

      if((req->wValue  == 0) && 
         (req->wLength == 1) &&
         ((req->bmRequest & 0x80) == 0x80))
      {
        hmsc->max_lun = ((const USBD_StorageTypeDef *)pdev->pUserData[1])->GetMaxLun();
        USBD_CtlSendData (pdev,
                          (uint8_t *)&hmsc->max_lun,
                          1);
      }
      else
      {
         USBD_CtlError(pdev , req);
         return USBD_FAIL; 
      }
      break;
      
    case BOT_RESET :
      if((req->wValue  == 0) && 
         (req->wLength == 0) &&
        ((req->bmRequest & 0x80) != 0x80))
      {      
	  USBD_MSC_Enqueue(USBD_MSC_BOT_EVENT_RESET);
      }
      else
      {
         USBD_CtlError(pdev , req);
         return USBD_FAIL; 
      }
      break;

    default:
       USBD_CtlError(pdev , req);
       return USBD_FAIL; 
    }
    break;
  /* Interface & Endpoint request */
  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_INTERFACE :
      USBD_CtlSendData (pdev,
                        (uint8_t *)&hmsc->interface,
                        1);
      break;
      
    case USB_REQ_SET_INTERFACE :
      hmsc->interface = (uint8_t)(req->wValue);
      break;
    
    case USB_REQ_CLEAR_FEATURE:  
      
      /* Flush the FIFO and Clear the stall status */    
      USBD_LL_FlushEP(pdev, (uint8_t)req->wIndex);
      
      /* Reactivate the EP */      
      USBD_LL_CloseEP (pdev , (uint8_t)req->wIndex);
      if((((uint8_t)req->wIndex) & 0x80) == 0x80)
      {
	/* Open EP IN */
	USBD_LL_OpenEP(pdev,
		       MSC_EPIN_ADDR,
		       USBD_EP_TYPE_BULK,
		       MSC_MAX_FS_PACKET);  
      }
      else
      {
	/* Open EP IN */
	USBD_LL_OpenEP(pdev,
		       MSC_EPOUT_ADDR,
		       USBD_EP_TYPE_BULK,
		       MSC_MAX_FS_PACKET);  
      }
      
      /* Handle BOT error */
      USBD_MSC_Enqueue(((uint8_t)req->wIndex & 0x80) ? USBD_MSC_BOT_EVENT_CLEAR_FEATURE_IN : USBD_MSC_BOT_EVENT_CLEAR_FEATURE_OUT);
      break;
      
    }  
    break;
   
  default:
    break;
  }
  return 0;
}

/**
* @brief  USBD_MSC_DataIn
*         handle data IN Stage
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval status
*/
uint8_t  USBD_MSC_DataIn (USBD_HandleTypeDef *pdev, 
                              uint8_t epnum)
{
    USBD_MSC_Enqueue(USBD_MSC_BOT_EVENT_DATA_IN);
    return 0;
}

/**
* @brief  USBD_MSC_DataOut
*         handle data OUT Stage
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval status
*/
uint8_t  USBD_MSC_DataOut (USBD_HandleTypeDef *pdev, 
                               uint8_t epnum)
{
    USBD_MSC_Enqueue(USBD_MSC_BOT_EVENT_DATA_OUT);
    return 0;
}

/**
* @brief  USBD_MSC_RegisterStorage
* @param  fops: storage callback
* @retval status
*/
uint8_t  USBD_MSC_RegisterStorage  (USBD_HandleTypeDef   *pdev, 
				    const USBD_StorageTypeDef *fops)
{
  if(fops != NULL)
  {
    pdev->pUserData[1]= fops;
  }
  return 0;
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
