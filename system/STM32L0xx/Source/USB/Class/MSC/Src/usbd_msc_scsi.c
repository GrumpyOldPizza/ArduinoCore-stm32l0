/**
  ******************************************************************************
  * @file    usbd_msc_scsi.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides all the USBD SCSI layer functions.
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
#include "usbd_msc_scsi.h"
#include "usbd_msc.h"
#include "armv6m.h"

/*
 * This code is more complex than expected. USB/MSC uses really only one logical connection
 * over 2 pipe. First the host sends a CBW (Command Block Wrapper), then optionally data is
 * exchanged, then finally the device sends a CSW (Command Status Wrapper) back. If there
 * a mismatch between what the host or devices expects one has to properly resync. If the
 * the CBW is invalid, the device stalls both IN and OUT pipe. If the host expects to
 * receive data, the IN pipe gets stalled, and if the host expects to send data the OUT
 * pipe gets stalled. If both pipes are stalled, the host issues a reset sequence
 * (reset token, then clear halt on IN, then clear halt on OUT). Otherwise the halt on
 * IN or OUT are cleared as needed. When that is done, the devices sends the CSV.
 *
 * The spec lays this out in terms of Hi (host receives), Ho (host wants to send) and
 * Hn (host neither wants to receiver or send). Same for the device, Di, Do, Dn.
 *
 * Hi, Ho & Hn are coded in the CBW:
 *
 * if (dCBWDataTransferLength == 0) {
 *     Hn;
 * } else {
 *     if (bmCBWFlags & 0x80) {
 *         Hi(dCBWDataTransferLength);
 *     } else {
 *         Ho(dCBWDataTransferLength);
 *     }
 * }
 *
 * For the device side it dependends up the command:
 *
 * TEST_UNIT_READY           Dn
 * START_STOP_UNIT           Dn
 * ALLOW_MEDIUM_REMOVAL      Dn
 * REQUEST_SENSE             Do (Dn if "Allocation Length" is 0)
 * INQUIRY                   Do (Dn if "Allocation Length" is 0)
 * MODE_SENSE_6              Do
 * MODE_SENSE_10 *           Do
 * READ_FORMAT_CAPACITIES *  Do (Dn if "Allocation Length" is 0)
 * READ_CAPACITY_10          Do
 * READ_10                   Do (Dn if "Transfer Length" is 0)
 * WRITE_10                  Di (Dn if "Transfer Length" is 0)
 * VERIFY_10                 Dn
 * <illegal command>         Dn
 *
 * In general up to dCBWDataTransferLength get transferred; if there is less data, less gets
 * transfered, and dCSWDataResidue contains the delta.
 *
 * The spec identifies 13 cases:
 *
 *                  CSW               STALL     HOST            BOT STATE
 *  (1) Hn = Dn     PASSED / FAILED   -         -               IDLE
 *  (2) Hn < Di     PHASE_ERROR       DATA_IN   RESET_RECOVERY  RECOVERY_RESET
 *  (3) Hn < Do     PHASE_ERROR       DATA_IN   RESET_RECOVERY  RECOVERY_RESET
 *  (4) Hi > Dn     PASSED / FAILED   DATA_IN   -               HALT_DATA_IN
 *  (5) Hi > Di     PASSED / FAILED   DATA_IN   -               HALT_DATA_IN
 *  (6) Hi = Di     PASSED / FAILED   -         -               IDLE
 *  (7) Hi < Di     PHASE_ERROR       DATA_IN   RESET_RECOVERY  RECOVERY_RESET
 *  (8) Hi <> Do    PHASE_ERROR       DATA_IN   RESET_RECOVERY  RECOVERY_RESET
 *  (9) Ho > Dn     PASSED / FAILED   DATA_OUT  -               HALT_DATA_OUT
 * (10) Ho <> Di    PHASE_ERROR       DATA_OUT  RESET_RECOVERY  RECOVERY_RESET
 * (11) Ho > Do     PASSED / FAILED   DATA_OUT  -               HALT_DATA_OUT
 * (12) Ho = Do     PASSED / FAILED   -         -               IDLE
 * (13) Ho < Do     PHASE_ERROR       DATA_OUT  RESET_RECOVERY  RECOVERY_RESET
 *
 * The spec does not say what CSW to send on a invalid CBW. Bot stalled are DATA_IN/DATA_OUT
 * followed by a RESET_RECOVERY.
 */
/***********************************************************************************************************************/

static void SCSI_SenseCode(USBD_HandleTypeDef *pdev, uint8_t SKey, uint16_t ASC, uint32_t blk_address)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;

    hmsc->scsi_sense_skey = SKey;
    hmsc->scsi_sense_asc = ASC;
    hmsc->scsi_sense_address = blk_address;
}

static int SCSI_CommandPassed(USBD_HandleTypeDef *pdev)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;

    if (hmsc->cbw.dDataLength != 0)
    {
        if (hmsc->cbw.bmFlags & 0x80)
        {
            /* case (4) Hi > Dn */
            return SCSI_STATUS_DATA_IN_CSW_CMD_PASSED;
        }
        else
        {
            /* case (9) Ho > Dn */
            return SCSI_STATUS_DATA_OUT_CSW_CMD_PASSED;
        }
    }  
    else
    {
        /* case (1) Hn = Dn */
        return SCSI_STATUS_CSW_CMD_PASSED;
    }
}

static int SCSI_CommandFailed(USBD_HandleTypeDef *pdev, uint8_t SKey, uint16_t ASC, uint32_t blk_address)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;

    SCSI_SenseCode(pdev, SKey, ASC, blk_address);

    if (hmsc->cbw.dDataLength == 0)
    {
        /* case (1) Hn = Dn */
        return SCSI_STATUS_CSW_CMD_FAILED;
    }
    else
    {
        if (hmsc->cbw.bmFlags & 0x80)
        {
            /* case (4) Hi > Dn */
            return SCSI_STATUS_DATA_IN_CSW_CMD_FAILED;
        }
        else
        {
            /* case (9) Ho > Dn */
            return SCSI_STATUS_DATA_OUT_CSW_CMD_FAILED;
        }
    }
}

static bool SCSI_CheckLength(USBD_HandleTypeDef *pdev, uint32_t length, int *p_status)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;

    if (hmsc->cbw.dDataLength == 0)
    {
        /* case (1) Hn = Dn */
        *p_status = SCSI_STATUS_CSW_CMD_PASSED;

        return false;
    }
    else
    {
        if (!(hmsc->cbw.bmFlags & 0x80))
        {
            /* case (10) Ho <> Di */
            *p_status = SCSI_STATUS_DATA_OUT_CSW_PHASE_ERROR;
                
            return false;
        }
    }

    return true;
}

static bool SCSI_CheckAllocation(USBD_HandleTypeDef *pdev, uint32_t allocation, uint32_t length, int *p_status)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;

    if (allocation)
    {
        if (hmsc->cbw.dDataLength == 0)
        {
            /* case (2) Hn < Di */
            *p_status = SCSI_STATUS_DATA_IN_CSW_PHASE_ERROR;

            return false;
        }
        else
        {
            if (hmsc->cbw.bmFlags & 0x80)
            {
                if (hmsc->cbw.dDataLength != allocation)
                {
                    *p_status = SCSI_CommandFailed(pdev, SCSI_SKEY_ILLEGAL_REQUEST, SCSI_ASC_INVALID_FIELD_IN_COMMAND_PACKET, 0);

                    return false;
                }
            }
            else
            {
                /* case (10) Ho <> Di */
                *p_status = SCSI_STATUS_DATA_OUT_CSW_PHASE_ERROR;
                
                return false;
            }
        }
    }
    else
    {
        if (hmsc->cbw.dDataLength == 0)
        {
            /* case (1) Hn = Dn */
            *p_status =SCSI_STATUS_CSW_CMD_PASSED;
        }
        else
        {
            if (hmsc->cbw.bmFlags & 0x80)
            {
                /* case (4) Hi > Dn */
                *p_status = SCSI_STATUS_DATA_IN_CSW_CMD_PASSED;
            }
            else
            {
                /* case (9) Ho > Dn */
                *p_status = SCSI_STATUS_DATA_OUT_CSW_CMD_PASSED;
            }
        }

        return false;
    }

    return true;
}

/***********************************************************************************************************************/

static int SCSI_TestUnitReady(USBD_HandleTypeDef *pdev, const uint8_t *params)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;

    if (!hmsc->storage->IsReady())
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
    } 

    return SCSI_CommandPassed(pdev);
}

static int SCSI_StartStopUnit(USBD_HandleTypeDef *pdev, const uint8_t *params)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;
    
    if (!hmsc->storage->StartStopUnit(((params[4] >> 0) & 1), ((params[4] >> 1) & 1)))
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
    }
    
    return SCSI_CommandPassed(pdev);
}

static int SCSI_AllowMediumRemoval(USBD_HandleTypeDef *pdev, const uint8_t *params)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;

    if (!hmsc->storage->PreventAllowMediumRemoval((params[4] >> 0) & 1))
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
    } 

    return SCSI_CommandPassed(pdev);
}

static int SCSI_Inquiry(USBD_HandleTypeDef *pdev, const uint8_t *params)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;
    const uint8_t *data;
    uint32_t length, allocation;
    int status;
    
    allocation = params[4];

    if (params[1] & 0x01)
    {
        hmsc->bot_data[0]  = 0x00;
        hmsc->bot_data[1]  = 0x00;
        hmsc->bot_data[2]  = 0x00;
        hmsc->bot_data[3]  = 0x03;
        hmsc->bot_data[4]  = 0x00;
        hmsc->bot_data[5]  = 0x00;
        hmsc->bot_data[6]  = 0x80;
        hmsc->bot_data[7]  = 0x83;

        data = &(hmsc->bot_data[0]);

        length = SCSI_DATA_LENGTH_INQUIRE_PAGE_00;
    }
    else
    {
        data = (const uint8_t *)hmsc->scsi_inquiry_data;

        length = data[4] + 5;
    }

    if (!SCSI_CheckAllocation(pdev, allocation, length, &status))
    {
        return status;
    }

    armv6m_svcall_4((uint32_t)&MSC_BOT_Transmit, (uint32_t)pdev, (uint32_t)data, (uint32_t)MIN(allocation, length), true);
    
    return SCSI_STATUS_DATA;
}

static int SCSI_RequestSense(USBD_HandleTypeDef  *pdev, const uint8_t *params)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;
    uint32_t allocation;
    int status;
    
    allocation = params[4];
    
    hmsc->bot_data[0]  = 0x70;
    hmsc->bot_data[1]  = 0x00;
    hmsc->bot_data[2]  = hmsc->scsi_sense_skey;
    hmsc->bot_data[3]  = 0x00;
    hmsc->bot_data[4]  = 0x00;
    hmsc->bot_data[5]  = 0x00;
    hmsc->bot_data[6]  = 0x00;
    hmsc->bot_data[7]  = 0x0a;
    hmsc->bot_data[8]  = 0x00;
    hmsc->bot_data[9]  = 0x00;
    hmsc->bot_data[10] = 0x00;
    hmsc->bot_data[11] = 0x00;
    hmsc->bot_data[12] = (uint8_t)(hmsc->scsi_sense_asc >> 8);
    hmsc->bot_data[13] = (uint8_t)(hmsc->scsi_sense_asc >> 0);
    hmsc->bot_data[14] = 0x00;
    hmsc->bot_data[15] = 0x00;
    hmsc->bot_data[16] = 0x00;
    hmsc->bot_data[17] = 0x00;

    if (hmsc->scsi_sense_skey == SCSI_SKEY_HARDWARE_ERROR)
    {
        hmsc->bot_data[0] |= 0x80;

        hmsc->bot_data[3]  = (uint8_t)(hmsc->scsi_sense_address >> 24);
        hmsc->bot_data[4]  = (uint8_t)(hmsc->scsi_sense_address >> 16);
        hmsc->bot_data[5]  = (uint8_t)(hmsc->scsi_sense_address >>  8);
        hmsc->bot_data[6]  = (uint8_t)(hmsc->scsi_sense_address >>  0);
    }

    hmsc->scsi_sense_skey = SCSI_SKEY_NO_SENSE;
    hmsc->scsi_sense_asc = SCSI_ASC_NO_SENSE;
    hmsc->scsi_sense_address = 0;
    
    if (!SCSI_CheckAllocation(pdev, allocation, SCSI_DATA_LENGTH_REQUEST_SENSE, &status))
    {
        return status;
    }

    armv6m_svcall_4((uint32_t)&MSC_BOT_Transmit, (uint32_t)pdev, (uint32_t)hmsc->bot_data, (uint32_t)MIN(allocation, SCSI_DATA_LENGTH_REQUEST_SENSE), true);
    
    return SCSI_STATUS_DATA;
}

static int SCSI_ModeSense6(USBD_HandleTypeDef *pdev, const uint8_t *params)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;
    bool write_protected;
    int status;

    if (!hmsc->storage->GetWriteProtected(&write_protected))
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
    } 

    hmsc->bot_data[0]  = 0x03;
    hmsc->bot_data[1]  = 0x00;
    hmsc->bot_data[2]  = write_protected ? 0x80 : 0x00;
    hmsc->bot_data[3]  = 0x00;

    if (!SCSI_CheckLength(pdev, SCSI_DATA_LENGTH_MODE_SENSE_6, &status))
    {
        return status;
    }

    armv6m_svcall_4((uint32_t)&MSC_BOT_Transmit, (uint32_t)pdev, (uint32_t)hmsc->bot_data, (uint32_t)SCSI_DATA_LENGTH_MODE_SENSE_6, true);
        
    return SCSI_STATUS_DATA;
}

static int SCSI_ModeSense10(USBD_HandleTypeDef *pdev, const uint8_t *params)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;
    bool write_protected;
    int status;
    
    if (!hmsc->storage->GetWriteProtected(&write_protected))
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
    } 

    hmsc->bot_data[0]  = 0x00;
    hmsc->bot_data[1]  = 0x06;
    hmsc->bot_data[2]  = 0x00;
    hmsc->bot_data[3]  = write_protected ? 0x80 : 0x00;
    hmsc->bot_data[4]  = 0x00;
    hmsc->bot_data[5]  = 0x00;
    hmsc->bot_data[6]  = 0x00;
    hmsc->bot_data[7]  = 0x00;
    
    if (!SCSI_CheckLength(pdev, SCSI_DATA_LENGTH_MODE_SENSE_10, &status))
    {
        return status;
    }

    armv6m_svcall_4((uint32_t)&MSC_BOT_Transmit, (uint32_t)pdev, (uint32_t)hmsc->bot_data, (uint32_t)SCSI_DATA_LENGTH_MODE_SENSE_10, true);

    return SCSI_STATUS_DATA;
}

static int SCSI_ReadFormatCapacities(USBD_HandleTypeDef *pdev, const uint8_t *params)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;
    uint32_t allocation;
    int status;
    
    allocation = (params[7] << 8) | (params[8]);

    hmsc->bot_data[0]  = 0x00;
    hmsc->bot_data[1]  = 0x00;
    hmsc->bot_data[2]  = 0x00;
    hmsc->bot_data[3]  = 0x08;
    hmsc->bot_data[4]  = (uint8_t)(hmsc->scsi_blk_count >> 24);
    hmsc->bot_data[5]  = (uint8_t)(hmsc->scsi_blk_count >> 16);
    hmsc->bot_data[6]  = (uint8_t)(hmsc->scsi_blk_count >>  8);
    hmsc->bot_data[7]  = (uint8_t)(hmsc->scsi_blk_count >>  0);
    hmsc->bot_data[8]  = 0x02;
    hmsc->bot_data[9]  = (uint8_t)(hmsc->scsi_blk_size >> 16);
    hmsc->bot_data[10] = (uint8_t)(hmsc->scsi_blk_size >>  8);
    hmsc->bot_data[11] = (uint8_t)(hmsc->scsi_blk_size >>  0);
    
    if (!SCSI_CheckAllocation(pdev, allocation, SCSI_DATA_LENGTH_READ_FORMAT_CAPACITIES, &status))
    {
        return status;
    }

    armv6m_svcall_4((uint32_t)&MSC_BOT_Transmit, (uint32_t)pdev, (uint32_t)hmsc->bot_data, (uint32_t)MIN(allocation, SCSI_DATA_LENGTH_READ_FORMAT_CAPACITIES), true);
    
    return SCSI_STATUS_DATA;
}

static int SCSI_ReadCapacity10(USBD_HandleTypeDef *pdev, const uint8_t *params)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;
    int status;
    
    if (!hmsc->storage->GetCapacity(&hmsc->scsi_blk_count, &hmsc->scsi_blk_size))
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
    } 

    hmsc->bot_data[0] = (uint8_t)((hmsc->scsi_blk_count -1) >> 24);
    hmsc->bot_data[1] = (uint8_t)((hmsc->scsi_blk_count -1) >> 16);
    hmsc->bot_data[2] = (uint8_t)((hmsc->scsi_blk_count -1) >>  8);
    hmsc->bot_data[3] = (uint8_t)((hmsc->scsi_blk_count -1) >>  0);
    hmsc->bot_data[4] = (uint8_t)(hmsc->scsi_blk_size >> 24);
    hmsc->bot_data[5] = (uint8_t)(hmsc->scsi_blk_size >> 16);
    hmsc->bot_data[6] = (uint8_t)(hmsc->scsi_blk_size >>  8);
    hmsc->bot_data[7] = (uint8_t)(hmsc->scsi_blk_size >>  0);

    if (!SCSI_CheckLength(pdev, SCSI_DATA_LENGTH_READ_CAPACITY_10, &status))
    {
        return status;
    }

    armv6m_svcall_4((uint32_t)&MSC_BOT_Transmit, (uint32_t)pdev, (uint32_t)hmsc->bot_data, (uint32_t)SCSI_DATA_LENGTH_READ_CAPACITY_10, true);
    
    return SCSI_STATUS_DATA;
}

static int SCSI_Read10(USBD_HandleTypeDef *pdev, const uint8_t *params)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;
    bool changed;

    hmsc->scsi_blk_address = (params[2] << 24) | (params[3] << 16) | (params[4] << 8) | (params[5] << 0);
    hmsc->scsi_blk_length = (params[7] << 8) | (params[8] << 0);
    
    /* Check in Media is changed */
    if (!hmsc->storage->GetChanged(&changed))
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
    } 

    if (changed)
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_UNIT_ATTENTION, SCSI_ASC_NOT_READY_TO_READY_TRANSITION_MEDIA_CHANGED, 0);
    } 
    
    if (hmsc->scsi_blk_count == 0)
    {
        if (!hmsc->storage->GetCapacity(&hmsc->scsi_blk_count, &hmsc->scsi_blk_size))
        {
            return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
        } 
    }
    
    /* Check address range */
    if ((hmsc->scsi_blk_address + hmsc->scsi_blk_length) > hmsc->scsi_blk_count)
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_ILLEGAL_REQUEST, SCSI_ASC_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE, 0);
    }

    if (hmsc->cbw.dDataLength != (hmsc->scsi_blk_length * hmsc->scsi_blk_size))
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_ILLEGAL_REQUEST, SCSI_ASC_INVALID_FIELD_IN_COMMAND_PACKET, 0);
    }

    if (hmsc->scsi_blk_length == 0)
    {
        return SCSI_CommandPassed(pdev);
    }
    else
    {
        if (!(hmsc->cbw.bmFlags & 0x80))
        {
            /* case (10) Ho <> Di */
            return SCSI_STATUS_DATA_OUT_CSW_PHASE_ERROR;
        }

        /* Acquire Media Lock */
        if (!hmsc->storage->Acquire())
        {
            return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
        } 

        if (!hmsc->storage->Read(hmsc->scsi_data[hmsc->scsi_index], hmsc->scsi_blk_address, 1, (hmsc->scsi_blk_length == 1)))
        {
            return SCSI_CommandFailed(pdev, SCSI_SKEY_HARDWARE_ERROR, SCSI_ASC_UNRECOVERED_READ_ERROR, hmsc->scsi_blk_address);
        }
        
        hmsc->scsi_blk_address += 1;
        hmsc->scsi_blk_length -= 1;
        
        hmsc->scsi_blk_fault = 0;
        hmsc->scsi_blk_busy = true;

        SCSI_ProcessRead(pdev);
        
        return SCSI_STATUS_DATA;
    }
}

static int SCSI_Write10(USBD_HandleTypeDef *pdev, const uint8_t *params)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;
    bool write_protected, changed;

    hmsc->scsi_blk_address = (params[2] << 24) | (params[3] << 16) | (params[4] << 8) | (params[5] << 0);
    hmsc->scsi_blk_length = (params[7] << 8) | (params[8] << 0);

    /* Check in Media is changed */
    if (!hmsc->storage->GetChanged(&changed))
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
    } 

    if (changed)
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_UNIT_ATTENTION, SCSI_ASC_NOT_READY_TO_READY_TRANSITION_MEDIA_CHANGED, 0);
    } 
    
    /* Check If media is write-protected */
    if (!hmsc->storage->GetWriteProtected(&write_protected))
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
    } 

    if (write_protected)
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_WRITE_PROTECTED_MEDIA, 0);
    } 

    if (hmsc->scsi_blk_count == 0)
    {
        if (!hmsc->storage->GetCapacity(&hmsc->scsi_blk_count, &hmsc->scsi_blk_size))
        {
            return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
        } 
    }
    
    /* Check address range */
    if ((hmsc->scsi_blk_address + hmsc->scsi_blk_length) > hmsc->scsi_blk_count)
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_ILLEGAL_REQUEST, SCSI_ASC_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE, 0);
    }

    if (hmsc->cbw.dDataLength != (hmsc->scsi_blk_length * hmsc->scsi_blk_size))
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_ILLEGAL_REQUEST, SCSI_ASC_INVALID_FIELD_IN_COMMAND_PACKET, 0);
    }

    if (hmsc->scsi_blk_length == 0)
    {
        return SCSI_CommandPassed(pdev);
    }
    else
    {
        if (hmsc->cbw.bmFlags & 0x80)
        {
            /* case (8) Hi <> Do */
            return SCSI_STATUS_DATA_IN_CSW_PHASE_ERROR;
        }

        /* Acquire Media Lock */
        if (!hmsc->storage->Acquire())
        {
            return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
        } 

        hmsc->scsi_blk_fault = 0;
        hmsc->scsi_blk_busy = true;

        armv6m_svcall_3((uint32_t)&MSC_BOT_Receive, (uint32_t)pdev, (uint32_t)hmsc->scsi_data[hmsc->scsi_index], MSC_MEDIA_PACKET);

        return SCSI_STATUS_DATA;
    }
}

static int SCSI_Verify10(USBD_HandleTypeDef *pdev, const uint8_t *params)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;
    uint32_t blk_address, blk_length;
    bool changed;

    blk_address = (params[2] << 24) | (params[3] << 16) | (params[4] << 8) | (params[5] << 0);
    blk_length = (params[7] << 8) | (params[8] << 0);

    if ((params[1] & 0x02) == 0x02) 
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_ILLEGAL_REQUEST, SCSI_ASC_INVALID_FIELD_IN_COMMAND_PACKET, 0);
    }

    /* Check in Media is changed */
    if (!hmsc->storage->GetChanged(&changed))
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
    } 

    if (changed)
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_UNIT_ATTENTION, SCSI_ASC_NOT_READY_TO_READY_TRANSITION_MEDIA_CHANGED, 0);
    } 
    
    if (hmsc->scsi_blk_count == 0)
    {
        if (!hmsc->storage->GetCapacity(&hmsc->scsi_blk_count, &hmsc->scsi_blk_size))
        {
            return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
        } 
    }
    
    /* Check address range */
    if ((blk_address + blk_length) > hmsc->scsi_blk_count)
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_ILLEGAL_REQUEST, SCSI_ASC_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE, 0);
    }

    return SCSI_CommandPassed(pdev);
}

void SCSI_ProcessStart(USBD_HandleTypeDef *pdev)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;
    uint8_t *cache_data;
    const uint8_t *inquiry_data;
    
    hmsc->storage->Init(&cache_data, &inquiry_data);

    hmsc->scsi_sense_skey = SCSI_SKEY_NO_SENSE;
    hmsc->scsi_sense_asc = SCSI_ASC_NO_SENSE;
    hmsc->scsi_sense_address = 0;

    hmsc->scsi_blk_size = 0;
    hmsc->scsi_blk_count = 0;
    hmsc->scsi_blk_address = 0;
    hmsc->scsi_blk_length = 0;
    hmsc->scsi_blk_fault = 0;
    hmsc->scsi_blk_busy = false;

    hmsc->scsi_index = 0;
    hmsc->scsi_data[0] = cache_data;
    hmsc->scsi_data[1] = cache_data + MSC_MEDIA_PACKET;

    hmsc->scsi_inquiry_data = inquiry_data;
    
}

void SCSI_ProcessStop(USBD_HandleTypeDef *pdev)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;

    hmsc->storage->StartStopUnit(true, false);
    
    hmsc->storage->DeInit();
}

void SCSI_ProcessReset(USBD_HandleTypeDef *pdev)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;

    hmsc->scsi_blk_fault = false;
    hmsc->scsi_blk_busy = false;
}

void SCSI_ProcessCommand(USBD_HandleTypeDef *pdev)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;
    const uint8_t *params = (const uint8_t*)&hmsc->cbw.CB[0];
    int status;

    switch (params[0]) {
    case SCSI_OPCODE_TEST_UNIT_READY:
        status = SCSI_TestUnitReady(pdev, params);
        break;
        
    case SCSI_OPCODE_START_STOP_UNIT:
        status = SCSI_StartStopUnit(pdev, params);
        break;
    
    case SCSI_OPCODE_ALLOW_MEDIUM_REMOVAL:
        status = SCSI_AllowMediumRemoval(pdev, params);
        break;
    
    case SCSI_OPCODE_INQUIRY:
        status = SCSI_Inquiry(pdev, params);
        break;
    
    case SCSI_OPCODE_REQUEST_SENSE:
        status = SCSI_RequestSense(pdev, params);
        break;

    case SCSI_OPCODE_MODE_SENSE_6:
        status = SCSI_ModeSense6(pdev, params);
        break;

    case SCSI_OPCODE_MODE_SENSE_10:
        status = SCSI_ModeSense10(pdev, params);
        break;
        
    case SCSI_OPCODE_READ_FORMAT_CAPACITIES:
        status = SCSI_ReadFormatCapacities(pdev, params);
        break;
        
    case SCSI_OPCODE_READ_CAPACITY_10:
        status = SCSI_ReadCapacity10(pdev, params);
        break;
    
    case SCSI_OPCODE_READ_10:
        status = SCSI_Read10(pdev, params); 
        break;
    
    case SCSI_OPCODE_WRITE_10:
        status = SCSI_Write10(pdev, params);
        break;
    
    case SCSI_OPCODE_VERIFY_10:
        status = SCSI_Verify10(pdev, params);
        break;

    default:
        status = SCSI_CommandFailed(pdev, SCSI_SKEY_ILLEGAL_REQUEST, SCSI_ASC_INVALID_COMMAND_OPERATION_CODE, 0);
        break;
    }

    switch (status) {
    case SCSI_STATUS_CSW_CMD_PASSED:
    case SCSI_STATUS_CSW_CMD_FAILED:
        armv6m_svcall_2((int32_t)&MSC_BOT_SendCSW, (uint32_t)pdev, (uint32_t)status);
        break;

    case SCSI_STATUS_DATA:
        break;

    case SCSI_STATUS_DATA_IN_CSW_CMD_PASSED:
    case SCSI_STATUS_DATA_IN_CSW_CMD_FAILED:
    case SCSI_STATUS_DATA_IN_CSW_PHASE_ERROR:
    case SCSI_STATUS_DATA_OUT_CSW_CMD_PASSED:
    case SCSI_STATUS_DATA_OUT_CSW_CMD_FAILED:
    case SCSI_STATUS_DATA_OUT_CSW_PHASE_ERROR:
        armv6m_svcall_2((uint32_t)&MSC_BOT_Abort, (uint32_t)pdev, (uint32_t)status);
        break;

    default:
        break;
    }
}

void SCSI_ProcessRead(USBD_HandleTypeDef *pdev)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;

    if (!hmsc->scsi_blk_busy)
    {
        /* Reset while read ... */
        return;
    }

    if (hmsc->scsi_blk_fault)
    {
        armv6m_svcall_2((uint32_t)&MSC_BOT_Abort, (uint32_t)pdev, (uint32_t)hmsc->scsi_blk_fault);

        hmsc->scsi_blk_fault = 0;
        hmsc->scsi_blk_busy = false;
        
        return;
    }

    /* case (5) Hi > Di */
    /* case (6) Hi = Di */
    armv6m_svcall_4((uint32_t)&MSC_BOT_Transmit, (uint32_t)pdev, (uint32_t)hmsc->scsi_data[hmsc->scsi_index], (uint32_t)hmsc->scsi_blk_size, (uint32_t)(hmsc->scsi_blk_length == 0));

    hmsc->scsi_index ^= 1;
    
    if (hmsc->scsi_blk_length == 0)
    {
        hmsc->scsi_blk_busy = false;
    }
    else
    {
        if (!hmsc->storage->Read(hmsc->scsi_data[hmsc->scsi_index], hmsc->scsi_blk_address, 1, (hmsc->scsi_blk_length == 1)))
        {
            SCSI_SenseCode(pdev, SCSI_SKEY_HARDWARE_ERROR, SCSI_ASC_UNRECOVERED_READ_ERROR, hmsc->scsi_blk_address);
            
            /* case (5) Hi > Di */
            hmsc->scsi_blk_fault = USBD_BOT_ABORT_DATA_IN_CSW_CMD_FAILED;

            return;
        }
    
        hmsc->scsi_blk_address += 1;
        hmsc->scsi_blk_length -= 1;
    }
}

void SCSI_ProcessWrite(USBD_HandleTypeDef *pdev)
{
    USBD_MSC_BOT_HandleTypeDef *hmsc = &USBD_MSC_Data;
    
    if (!hmsc->scsi_blk_busy)
    {
        /* Reset while read ... */
        return;
    }

    hmsc->csw.dDataResidue -= hmsc->scsi_blk_size;
    
    if (hmsc->scsi_blk_fault)
    {
        armv6m_svcall_2((uint32_t)&MSC_BOT_Abort, (uint32_t)pdev, (uint32_t)hmsc->scsi_blk_fault);

        hmsc->scsi_blk_fault = 0;
        hmsc->scsi_blk_busy = false;
        
        return;
    }

    if (hmsc->scsi_blk_length == 1)
    {
        if (!hmsc->storage->Write(hmsc->scsi_data[hmsc->scsi_index], hmsc->scsi_blk_address, 1, true))
        {
            SCSI_SenseCode(pdev, SCSI_SKEY_HARDWARE_ERROR, SCSI_ASC_WRITE_FAULT, hmsc->scsi_blk_address);
            
            armv6m_svcall_2((uint32_t)&MSC_BOT_SendCSW, (uint32_t)pdev, (uint32_t)USBD_CSW_CMD_FAILED);
        }
        else
        {
            armv6m_svcall_2((uint32_t)&MSC_BOT_SendCSW, (uint32_t)pdev, (uint32_t)USBD_CSW_CMD_PASSED);
        }
            
        hmsc->scsi_blk_fault = 0;
        hmsc->scsi_blk_busy = true;
    }
    else
    {
        armv6m_svcall_3((uint32_t)&MSC_BOT_Receive, (uint32_t)pdev, (uint32_t)hmsc->scsi_data[hmsc->scsi_index ^ 1], MSC_MEDIA_PACKET);
        
        if (!hmsc->storage->Write(hmsc->scsi_data[hmsc->scsi_index], hmsc->scsi_blk_address, 1, false))
        {
            SCSI_SenseCode(pdev, SCSI_SKEY_HARDWARE_ERROR, SCSI_ASC_WRITE_FAULT, hmsc->scsi_blk_address);

            /* case (11) Ho > Do */
            hmsc->scsi_blk_fault = USBD_BOT_ABORT_DATA_OUT_CSW_CMD_FAILED;
        }
        else
        {
            hmsc->scsi_blk_address += 1;
            hmsc->scsi_blk_length -= 1;
            
            hmsc->scsi_index ^= 1;   
        }
    }
}
