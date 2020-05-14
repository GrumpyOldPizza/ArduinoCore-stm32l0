/**
  ******************************************************************************
  * @file    usbd_msc_bot.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides all the BOT protocol core functions.
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
#include "usbd_msc_bot.h"
#include "usbd_msc.h"
#include "usbd_msc_scsi.h"
#include "usbd_ioreq.h"
#include "armv6m.h"

void MSC_BOT_Init(USBD_HandleTypeDef *pdev)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;

    armv6m_svcall_1((uint32_t)&MSC_BOT_Open, (uint32_t)pdev);

    hmsc->bot_state = USBD_BOT_STATE_IDLE;
    
    SCSI_ProcessStart(pdev);
}

void MSC_BOT_DeInit(USBD_HandleTypeDef *pdev)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;
    
    SCSI_ProcessStop(pdev);

    hmsc->bot_state = USBD_BOT_STATE_NONE;

    armv6m_svcall_1((uint32_t)&MSC_BOT_Close, (uint32_t)pdev);
}

void MSC_BOT_DataIn(USBD_HandleTypeDef *pdev)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;

    switch (hmsc->bot_state) {
    case USBD_BOT_STATE_DATA_IN:
        SCSI_ProcessRead(pdev);
        break;
    
    case USBD_BOT_STATE_DATA_IN_LAST:
        armv6m_svcall_2((uint32_t)&MSC_BOT_SendCSW, (uint32_t)pdev, (uint32_t)USBD_CSW_CMD_PASSED);
        break;

    case USBD_BOT_STATE_DATA_IN_LAST_STALL:
        armv6m_svcall_2((uint32_t)&MSC_BOT_Stall, (uint32_t)pdev, (uint32_t)MSC_EPIN_ADDR);

        if (hmsc->csw.bStatus == USBD_CSW_PHASE_ERROR)
        {
            hmsc->bot_state = USBD_BOT_STATE_RECOVERY_RESET;
        }
        else
        {
            hmsc->bot_state = USBD_BOT_STATE_HALT_DATA_IN;
        }
        break;
        
    default:
        break;
    }
}

void MSC_BOT_DataOut(USBD_HandleTypeDef *pdev)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;

    switch (hmsc->bot_state) {
    case USBD_BOT_STATE_IDLE:
        hmsc->csw.dSignature = USBD_BOT_CSW_SIGNATURE;
        hmsc->csw.dTag = hmsc->cbw.dTag;
        hmsc->csw.dDataResidue = hmsc->cbw.dDataLength;

        if ((USBD_LL_GetRxDataSize(pdev, MSC_EPOUT_ADDR) != USBD_BOT_CBW_LENGTH) ||
            (hmsc->cbw.dSignature != USBD_BOT_CBW_SIGNATURE) ||
            (hmsc->cbw.bLUN > hmsc->max_lun) || 
            (hmsc->cbw.bCBLength == 0) || 
            (hmsc->cbw.bCBLength > 16))
        {
            armv6m_svcall_2((uint32_t)&MSC_BOT_Abort, (uint32_t)pdev, USBD_BOT_ABORT_CBW);
        }
        else
        {
            SCSI_ProcessCommand(pdev);
        }
        break;
        
    case USBD_BOT_STATE_DATA_OUT:
        SCSI_ProcessWrite(pdev);
        break;
        
    default:
        break;
    }
}

void MSC_BOT_Reset(USBD_HandleTypeDef *pdev)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;

    hmsc->bot_state = USBD_BOT_STATE_RECOVERY_DATA_IN;
}

uint8_t __last_feature;

void MSC_BOT_ClearFeature(USBD_HandleTypeDef *pdev)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;
    uint8_t ep_addr;
    
    __last_feature = ep_addr = hmsc->feature;

    hmsc->feature = 0;

    armv6m_svcall_2((uint32_t)&MSC_BOT_Reactivate, (uint32_t)pdev, (uint32_t)ep_addr);
}

/**********************************************************************************************************/

void MSC_BOT_Open(USBD_HandleTypeDef *pdev)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;

    NVIC_DisableIRQ(USB_IRQn);

    /* Open EP OUT */
    USBD_LL_OpenEP(pdev, MSC_EPOUT_ADDR, USBD_EP_TYPE_BULK, MSC_MAX_FS_PACKET);
  
    /* Open EP IN */
    USBD_LL_OpenEP(pdev, MSC_EPIN_ADDR, USBD_EP_TYPE_BULK, MSC_MAX_FS_PACKET);  
    
    USBD_LL_FlushEP(pdev, MSC_EPOUT_ADDR);
    USBD_LL_FlushEP(pdev, MSC_EPIN_ADDR);
    
    /* Prapare EP to Receive First BOT Cmd */
    USBD_LL_PrepareReceive(pdev, MSC_EPOUT_ADDR, (uint8_t *)&hmsc->cbw, USBD_BOT_CBW_LENGTH);    

    NVIC_EnableIRQ(USB_IRQn);
}

void MSC_BOT_Close(USBD_HandleTypeDef *pdev)
{
    NVIC_DisableIRQ(USB_IRQn);

    /* Close MSC EPs */
    USBD_LL_CloseEP(pdev, MSC_EPOUT_ADDR);
  
    /* Open EP IN */
    USBD_LL_CloseEP(pdev, MSC_EPIN_ADDR);
  
    NVIC_EnableIRQ(USB_IRQn);
}

void MSC_BOT_Abort(USBD_HandleTypeDef *pdev, uint8_t abort)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;

    NVIC_DisableIRQ(USB_IRQn);

    switch (abort) {
    case USBD_BOT_ABORT_CBW:
        USBD_LL_StallEP(pdev, MSC_EPIN_ADDR);
        USBD_LL_StallEP(pdev, MSC_EPOUT_ADDR);

        hmsc->csw.bStatus = USBD_CSW_CMD_FAILED;

        hmsc->bot_state = USBD_BOT_STATE_RECOVERY_RESET;
        break;

    case USBD_BOT_ABORT_DATA_IN_CSW_CMD_PASSED:
    case USBD_BOT_ABORT_DATA_IN_CSW_CMD_FAILED:
    case USBD_BOT_ABORT_DATA_IN_CSW_PHASE_ERROR:
        USBD_LL_StallEP(pdev, MSC_EPIN_ADDR);

        hmsc->csw.bStatus = (abort - USBD_BOT_ABORT_DATA_IN_CSW_CMD_PASSED);

        if (abort == USBD_BOT_ABORT_DATA_IN_CSW_PHASE_ERROR)
        {
            hmsc->bot_state = USBD_BOT_STATE_RECOVERY_RESET;
        }
        else
        {
            hmsc->bot_state = USBD_BOT_STATE_HALT_DATA_IN;
        }
        break;

    case USBD_BOT_ABORT_DATA_OUT_CSW_CMD_PASSED:
    case USBD_BOT_ABORT_DATA_OUT_CSW_CMD_FAILED:
    case USBD_BOT_ABORT_DATA_OUT_CSW_PHASE_ERROR:
        USBD_LL_StallEP(pdev, MSC_EPOUT_ADDR);

        hmsc->csw.bStatus = (abort - USBD_BOT_ABORT_DATA_IN_CSW_CMD_PASSED);

        if (abort == USBD_BOT_ABORT_DATA_OUT_CSW_PHASE_ERROR)
        {
            hmsc->bot_state = USBD_BOT_STATE_RECOVERY_RESET;
        }
        else
        {
            hmsc->bot_state = USBD_BOT_STATE_HALT_DATA_OUT;
        }
        break;

    default:
        break;
    }

    NVIC_EnableIRQ(USB_IRQn);
}

void MSC_BOT_Reactivate(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{      
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;
    uint8_t bot_state_previous = hmsc->bot_state;;
    
    NVIC_DisableIRQ(USB_IRQn);

    /* Flush the FIFO and Clear the stall status */    
    USBD_LL_FlushEP(pdev, ep_addr);
      
    /* Reactivate the EP */      
    USBD_LL_CloseEP(pdev, ep_addr);

    /* Open EP */
    USBD_LL_OpenEP(pdev, ep_addr, USBD_EP_TYPE_BULK, MSC_MAX_FS_PACKET);  

    if (ep_addr == MSC_EPIN_ADDR)
    {
        if (hmsc->bot_state == USBD_BOT_STATE_RECOVERY_DATA_IN)
        {
            hmsc->bot_state = USBD_BOT_STATE_RECOVERY_DATA_OUT;
        }

        if (hmsc->bot_state == USBD_BOT_STATE_HALT_DATA_IN)
        {
            hmsc->bot_state = USBD_BOT_STATE_IDLE;
        }
    }

    if (ep_addr == MSC_EPOUT_ADDR)
    {
        if (hmsc->bot_state == USBD_BOT_STATE_RECOVERY_DATA_OUT)
        {
            hmsc->bot_state = USBD_BOT_STATE_IDLE;
        }

        if (hmsc->bot_state == USBD_BOT_STATE_HALT_DATA_OUT)
        {
            hmsc->bot_state = USBD_BOT_STATE_IDLE;
        }
    }

    if ((hmsc->bot_state != bot_state_previous) && (hmsc->bot_state == USBD_BOT_STATE_IDLE))
    {
        USBD_LL_Transmit(pdev, MSC_EPIN_ADDR, (uint8_t *)&hmsc->csw, USBD_BOT_CSW_LENGTH);

        /* Prepare EP to Receive next Cmd */
        USBD_LL_PrepareReceive(pdev, MSC_EPOUT_ADDR, (uint8_t *)&hmsc->cbw, USBD_BOT_CBW_LENGTH);  
    }
        
    NVIC_EnableIRQ(USB_IRQn);
}

void MSC_BOT_Stall(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{      
    NVIC_DisableIRQ(USB_IRQn);

    USBD_LL_StallEP(pdev, ep_addr);

    NVIC_EnableIRQ(USB_IRQn);
}

void MSC_BOT_SendCSW(USBD_HandleTypeDef *pdev, uint8_t status)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;

    NVIC_DisableIRQ(USB_IRQn);

    hmsc->csw.bStatus = status;

    hmsc->bot_state = USBD_BOT_STATE_IDLE;

    USBD_LL_Transmit(pdev, MSC_EPIN_ADDR, (uint8_t *)&hmsc->csw, USBD_BOT_CSW_LENGTH);
  
    /* Prepare EP to Receive next Cmd */
    USBD_LL_PrepareReceive(pdev, MSC_EPOUT_ADDR, (uint8_t *)&hmsc->cbw, USBD_BOT_CBW_LENGTH);  

    NVIC_EnableIRQ(USB_IRQn);
}

void MSC_BOT_Transmit(USBD_HandleTypeDef *pdev, const uint8_t *data, uint32_t length, bool last)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;
    uint32_t count;
    
    NVIC_DisableIRQ(USB_IRQn);

    count = MIN(hmsc->csw.dDataResidue, length);

    hmsc->csw.dDataResidue -= count;

    if (last)
    {
        if (hmsc->csw.dDataResidue != 0)
        {
            /* case (5) Hi > Di */
            hmsc->bot_state = USBD_BOT_STATE_DATA_IN_LAST_STALL;

            hmsc->csw.bStatus = USBD_CSW_CMD_PASSED;
        }
        else
        {
            /* case (6) Hi == Di */
            hmsc->bot_state = USBD_BOT_STATE_DATA_IN_LAST;
        }
    }
    else
    {
        hmsc->bot_state = USBD_BOT_STATE_DATA_IN;
    }

    USBD_LL_Transmit(pdev, MSC_EPIN_ADDR, (uint8_t*)data, length);

    NVIC_EnableIRQ(USB_IRQn);
}

void MSC_BOT_Receive(USBD_HandleTypeDef *pdev, uint8_t *data, uint32_t length)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;

    NVIC_DisableIRQ(USB_IRQn);

    hmsc->bot_state = USBD_BOT_STATE_DATA_OUT;  

    USBD_LL_PrepareReceive(pdev, MSC_EPOUT_ADDR, data, length);

    NVIC_EnableIRQ(USB_IRQn);
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
