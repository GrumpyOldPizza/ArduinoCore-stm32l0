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

USBD_MSC_BOT_HandleTypeDef USBD_MSC_Data;

static volatile uint32_t USBD_MSC_Lock;

static armv6m_work_t USBD_MSC_Work;

static void USBD_MSC_Process(USBD_HandleTypeDef *pdev)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;
    USBD_MSC_BOT_CallbackTypeDef callback;
    uint32_t queue_tail;
    
    queue_tail = hmsc->queue_tail;

    while (hmsc->queue_head != queue_tail)
    {
        callback = hmsc->queue_data[queue_tail];

        queue_tail++;

        if (queue_tail == MSC_QUEUE_ENTRIES)
        {
            queue_tail = 0;
        }

        hmsc->queue_tail = queue_tail;

        (*callback)(pdev);
    }
}

static bool USBD_MSC_Submit(USBD_MSC_BOT_CallbackTypeDef callback)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;
    uint32_t queue_head, queue_head_next;

    queue_head = hmsc->queue_head;

    queue_head_next = queue_head +1;

    if (queue_head_next == MSC_QUEUE_ENTRIES)
    {
        queue_head_next = 0;
    }

    if (queue_head_next == hmsc->queue_tail)
    {
        return false;
    }

    hmsc->queue_data[queue_head] = callback;

    hmsc->queue_head = queue_head_next;

    if (USBD_MSC_Lock == 0)
    {
        armv6m_work_submit(&USBD_MSC_Work);
    }

    return true;
}

void USBD_MSC_Notify(uint8_t lun, int acquire)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;

    if (acquire)
    {
        armv6m_atomic_add(&USBD_MSC_Lock, 1);
    }
    else
    {
        if (armv6m_atomic_sub(&USBD_MSC_Lock, 1) == 1)
        {
            if (hmsc->queue_head != hmsc->queue_tail)
            {
                armv6m_work_submit(&USBD_MSC_Work);
            }
        }
    }
}

uint8_t USBD_MSC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;

    if (!USBD_MSC_Work.callback.routine)
    {
        armv6m_work_create(&USBD_MSC_Work, (armv6m_core_routine_t)USBD_MSC_Process, pdev);
    }

    hmsc->max_lun = 0;
    hmsc->feature = 0;
    
    if (!USBD_MSC_Submit(MSC_BOT_Init))
    {
        return USBD_FAIL;
    }

    return USBD_OK;
}

uint8_t USBD_MSC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    if (!USBD_MSC_Submit(MSC_BOT_DeInit))
    {
        return USBD_FAIL;
    }
    
    return USBD_OK;
}

uint8_t USBD_MSC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;
    uint8_t ep_addr;
    
    switch (req->bmRequest & USB_REQ_TYPE_MASK) {

        /* Class request */
    case USB_REQ_TYPE_CLASS:
        switch (req->bRequest) {
        case BOT_GET_MAX_LUN:
            if((req->wValue  == 0) && 
               (req->wLength == 1) &&
               ((req->bmRequest & 0x80) == 0x80))
            {
                USBD_CtlSendData(pdev, (uint8_t*)&hmsc->max_lun, 1);
            }
            else
            {
                USBD_CtlError(pdev, req);

                return USBD_FAIL; 
            }
            break;
      
        case BOT_RESET:
            if((req->wValue  == 0) && 
               (req->wLength == 0) &&
               ((req->bmRequest & 0x80) != 0x80))
            {      
                if (!USBD_MSC_Submit(MSC_BOT_Reset))
                {
                    return USBD_FAIL;
                }
            }
            else
            {
                USBD_CtlError(pdev, req);

                return USBD_FAIL; 
            }
            break;

        default:
            USBD_CtlError(pdev, req);

            return USBD_FAIL; 
        }
        break;

        /* Interface & Endpoint request */
    case USB_REQ_TYPE_STANDARD:
        switch (req->bRequest) {
        case USB_REQ_GET_INTERFACE:
            USBD_CtlSendData(pdev, (uint8_t *)&hmsc->interface, 1);
            break;
      
        case USB_REQ_SET_INTERFACE:
            hmsc->interface = (uint8_t)(req->wValue);
            break;
            
        case USB_REQ_CLEAR_FEATURE:  
            if (req->wValue == USB_FEATURE_EP_HALT)
            {
                ep_addr = LOBYTE(req->wIndex);

                if ((ep_addr == MSC_EPIN_ADDR) || (ep_addr == MSC_EPOUT_ADDR))
                {
                    hmsc->feature = ep_addr;
            
                    if (!USBD_MSC_Submit(MSC_BOT_ClearFeature))
                    {
                        return USBD_FAIL;
                    }
                }
            }
            break;
        }  
        break;
   
    default:
        break;
    }

    return USBD_OK;
}

uint8_t USBD_MSC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    if (!USBD_MSC_Submit(MSC_BOT_DataIn))
    {
        return USBD_FAIL;
    }

    return USBD_OK;
}

uint8_t USBD_MSC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    if (!USBD_MSC_Submit(MSC_BOT_DataOut))
    {
        return USBD_FAIL;
    }

    return USBD_OK;
}

uint8_t USBD_MSC_RegisterStorage(USBD_HandleTypeDef *pdev, const USBD_StorageTypeDef *fops)
{
    if (fops != NULL)
    {
        USBD_MSC_Data.storage = fops;
    }

    return USBD_OK;
}
