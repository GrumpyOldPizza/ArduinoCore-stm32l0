/**
  ******************************************************************************
  * @file    usbd_cdc.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides the high layer firmware functions to manage the 
  *          following functionalities of the USB CDC Class:
  *           - Initialization and Configuration of high and low layer
  *           - Enumeration as CDC Device (and enumeration for each implemented memory interface)
  *           - OUT/IN data transfer
  *           - Command IN transfer (class requests management)
  *           - Error management
  *           
  *  @verbatim
  *      
  *          ===================================================================      
  *                                CDC Class Driver Description
  *          =================================================================== 
  *           This driver manages the "Universal Serial Bus Class Definitions for Communications Devices
  *           Revision 1.2 November 16, 2007" and the sub-protocol specification of "Universal Serial Bus 
  *           Communications Class Subclass Specification for PSTN Devices Revision 1.2 February 9, 2007"
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Enumeration as CDC device with 2 data endpoints (IN and OUT) and 1 command endpoint (IN)
  *             - Requests management (as described in section 6.2 in specification)
  *             - Abstract Control Model compliant
  *             - Union Functional collection (using 1 IN endpoint for control)
  *             - Data interface class
  * 
  *           These aspects may be enriched or modified for a specific user application.
  *          
  *            This driver doesn't implement the following aspects of the specification 
  *            (but it is possible to manage these features with some modifications on this driver):
  *             - Any class-specific aspect relative to communication classes should be managed by user application.
  *             - All communication classes other than PSTN are not managed
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
#include "usbd_cdc.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_CDC 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_CDC_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_CDC_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_CDC_Private_Macros
  * @{
  */ 

/**
  * @}
  */ 


/** @defgroup USBD_CDC_Private_FunctionPrototypes
  * @{
  */


uint8_t  USBD_CDC_Init (USBD_HandleTypeDef *pdev, 
			uint8_t cfgidx);

uint8_t  USBD_CDC_DeInit (USBD_HandleTypeDef *pdev, 
			  uint8_t cfgidx);

uint8_t  USBD_CDC_Setup (USBD_HandleTypeDef *pdev, 
			 USBD_SetupReqTypedef *req);

uint8_t  USBD_CDC_DataIn (USBD_HandleTypeDef *pdev, 
			  uint8_t epnum);

uint8_t  USBD_CDC_DataOut (USBD_HandleTypeDef *pdev, 
			   uint8_t epnum);

uint8_t  USBD_CDC_SOF (USBD_HandleTypeDef *pdev);

uint8_t  USBD_CDC_EP0_RxReady (USBD_HandleTypeDef *pdev);

/**
  * @}
  */ 

/** @defgroup USBD_CDC_Private_Variables
  * @{
  */ 


/**
  * @}
  */ 

static USBD_CDC_HandleTypeDef USBD_CDC_Handle;

/** @defgroup USBD_CDC_Private_Functions
  * @{
  */ 

/**
  * @brief  USBD_CDC_Init
  *         Initialize the CDC interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
uint8_t  USBD_CDC_Init (USBD_HandleTypeDef *pdev, 
			uint8_t cfgidx)
{
  uint8_t ret = 0;
  USBD_CDC_HandleTypeDef   *hcdc;

  /* Open EP IN */
  USBD_LL_OpenEP(pdev,
		 CDC_IN_EP,
		 USBD_EP_TYPE_BULK,
		 CDC_DATA_FS_IN_PACKET_SIZE);
  
  /* Open EP OUT */
  USBD_LL_OpenEP(pdev,
		 CDC_OUT_EP,
		 USBD_EP_TYPE_BULK,
		 CDC_DATA_FS_OUT_PACKET_SIZE);

  /* Open Command IN EP */
  USBD_LL_OpenEP(pdev,
		 CDC_CMD_EP,
		 USBD_EP_TYPE_INTR,
		 CDC_CMD_PACKET_SIZE);
  
  
  pdev->pClassData[0] = &USBD_CDC_Handle;
  
  hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData[0];
    
  /* Init  physical Interface components */
  ((const USBD_CDC_ItfTypeDef *)pdev->pUserData[0])->Init(pdev);
    
  /* Init Xfer states */
  hcdc->TxState =0;
  hcdc->RxState =0;
       
  /* Prepare Out endpoint to receive next packet */
  USBD_LL_PrepareReceive(pdev,
			 CDC_OUT_EP,
			 hcdc->RxBuffer,
			 CDC_DATA_FS_OUT_PACKET_SIZE);

  return ret;
}

/**
  * @brief  USBD_CDC_Init
  *         DeInitialize the CDC layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
uint8_t  USBD_CDC_DeInit (USBD_HandleTypeDef *pdev, 
			  uint8_t cfgidx)
{
  uint8_t ret = 0;
  
  /* Open EP IN */
  USBD_LL_CloseEP(pdev,
              CDC_IN_EP);
  
  /* Open EP OUT */
  USBD_LL_CloseEP(pdev,
              CDC_OUT_EP);
  
  /* Open Command IN EP */
  USBD_LL_CloseEP(pdev,
              CDC_CMD_EP);
  
  
  /* DeInit  physical Interface components */
  if(pdev->pClassData[0] != NULL)
  {
    ((const USBD_CDC_ItfTypeDef *)pdev->pUserData[0])->DeInit();
    pdev->pClassData[0] = NULL;
  }
  
  return ret;
}

/**
  * @brief  USBD_CDC_Setup
  *         Handle the CDC specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
uint8_t  USBD_CDC_Setup (USBD_HandleTypeDef *pdev, 
			 USBD_SetupReqTypedef *req)
{
  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData[0];
  static uint8_t ifalt = 0;
    
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS :
    if (req->wLength)
    {
      if (req->bmRequest & 0x80)
      {
        ((const USBD_CDC_ItfTypeDef *)pdev->pUserData[0])->Control(req->bRequest,
								   (uint8_t *)hcdc->data,
								   req->wLength);
	USBD_CtlSendData (pdev, 
			  (uint8_t *)hcdc->data,
			  req->wLength);
      }
      else
      {
        hcdc->CmdOpCode = req->bRequest;
        hcdc->CmdLength = req->wLength;
        
        USBD_CtlPrepareRx (pdev, 
                           (uint8_t *)hcdc->data,
                           req->wLength);
      }
      
    }
    else
    {
      ((const USBD_CDC_ItfTypeDef *)pdev->pUserData[0])->Control(req->bRequest,
								 (uint8_t*)req,
								 0);
    }
    break;

  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {      
    case USB_REQ_GET_INTERFACE :
      USBD_CtlSendData (pdev,
                        &ifalt,
                        1);
      break;
      
    case USB_REQ_SET_INTERFACE :
      break;
    }
 
  default: 
    break;
  }
  return USBD_OK;
}

/**
  * @brief  USBD_CDC_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t  USBD_CDC_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData[0];
  
  hcdc->TxState = 0;

  ((const USBD_CDC_ItfTypeDef *)pdev->pUserData[0])->TxDone();

  return USBD_OK;
}

/**
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t  USBD_CDC_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{      
  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData[0];
  
  /* USB data will be immediately processed, this allow next USB traffic being 
  NAKed till the end of the application Xfer */

  ((const USBD_CDC_ItfTypeDef *)pdev->pUserData[0])->RxReady(hcdc->RxBuffer, USBD_LL_GetRxDataSize (pdev, epnum));

  return USBD_OK;
}

uint8_t  USBD_CDC_SOF (USBD_HandleTypeDef *pdev)
{      
  ((const USBD_CDC_ItfTypeDef *)pdev->pUserData[0])->SOF();

  return USBD_OK;
}



/**
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t  USBD_CDC_EP0_RxReady (USBD_HandleTypeDef *pdev)
{ 
  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData[0];
  
  if(hcdc->CmdOpCode != 0xFF)
  {
      ((const USBD_CDC_ItfTypeDef *)pdev->pUserData[0])->Control(hcdc->CmdOpCode,
								 (uint8_t *)hcdc->data,
								 hcdc->CmdLength);
      hcdc->CmdOpCode = 0xFF; 
      
  }
  return USBD_OK;
}


/**
* @brief  USBD_CDC_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: CD  Interface callback
  * @retval status
  */
uint8_t  USBD_CDC_RegisterInterface  (USBD_HandleTypeDef   *pdev, 
                                      const USBD_CDC_ItfTypeDef *fops)
{
  uint8_t  ret = USBD_FAIL;
  
  if(fops != NULL)
  {
    pdev->pUserData[0]= fops;
    ret = USBD_OK;    
  }
  
  return ret;
}

/**
  * @brief  USBD_CDC_SetTxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Tx Buffer
  * @retval status
  */
uint8_t  USBD_CDC_SetTxBuffer  (USBD_HandleTypeDef   *pdev,
                                const uint8_t  *pbuff,
                                uint16_t length)
{
  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData[0];
  
  hcdc->TxBuffer = pbuff;
  hcdc->TxLength = length;  
  
  return USBD_OK;  
}


/**
  * @brief  USBD_CDC_SetRxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Rx Buffer
  * @retval status
  */
uint8_t  USBD_CDC_SetRxBuffer  (USBD_HandleTypeDef   *pdev,
                                   uint8_t  *pbuff)
{
  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData[0];
  
  hcdc->RxBuffer = pbuff;
  
  return USBD_OK;
}

/**
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t  USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev)
{      
  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData[0];
  
  if(hcdc->TxState == 0)
  {
    /* Tx Transfer in progress */
    hcdc->TxState = 1;
    
    /* Transmit next packet */
    USBD_LL_Transmit(pdev,
		     CDC_IN_EP,
		     (uint8_t*)hcdc->TxBuffer,
		     hcdc->TxLength);
    
    return USBD_OK;
  }
  else
  {
    return USBD_BUSY;
  }
}


/**
  * @brief  USBD_CDC_ReceivePacket
  *         prepare OUT Endpoint for reception
  * @param  pdev: device instance
  * @retval status
  */
uint8_t  USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev)
{      
  USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData[0];
  
  /* Suspend or Resume USB Out process */
  /* Prepare Out endpoint to receive next packet */
  USBD_LL_PrepareReceive(pdev,
			 CDC_OUT_EP,
			 hcdc->RxBuffer,
			 CDC_DATA_FS_OUT_PACKET_SIZE);
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
