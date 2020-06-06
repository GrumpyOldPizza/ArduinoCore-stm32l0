/*
 * Copyright (c) 2019-2020 Thomas Roell.  All rights reserved.
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
#include "stm32l0_usbd_msc.h"
#include "stm32l0_usbd_class.h"
#include "dosfs_storage.h"

extern void USB_IRQHandler(void);

/***********************************************************************************************************************/

#define SCSI_OPCODE_TEST_UNIT_READY                               0x00
#define SCSI_OPCODE_REQUEST_SENSE                                 0x03
#define SCSI_OPCODE_INQUIRY                                       0x12
#define SCSI_OPCODE_MODE_SENSE_6                                  0x1a
#define SCSI_OPCODE_START_STOP_UNIT                               0x1b
#define SCSI_OPCODE_ALLOW_MEDIUM_REMOVAL                          0x1e
#define SCSI_OPCODE_MODE_SENSE_10                                 0x5a
#define SCSI_OPCODE_READ_FORMAT_CAPACITIES                        0x23
#define SCSI_OPCODE_READ_CAPACITY_10                              0x25
#define SCSI_OPCODE_READ_10                                       0x28
#define SCSI_OPCODE_WRITE_10                                      0x2a
#define SCSI_OPCODE_VERIFY_10                                     0x2f

#define SCSI_STATUS_CSW_CMD_PASSED                                0
#define SCSI_STATUS_CSW_CMD_FAILED                                1
#define SCSI_STATUS_DATA                                          3
#define SCSI_STATUS_DATA_IN_CSW_CMD_PASSED                        USBD_BOT_ABORT_DATA_IN_CSW_CMD_PASSED
#define SCSI_STATUS_DATA_IN_CSW_CMD_FAILED                        USBD_BOT_ABORT_DATA_IN_CSW_CMD_FAILED
#define SCSI_STATUS_DATA_IN_CSW_PHASE_ERROR                       USBD_BOT_ABORT_DATA_IN_CSW_PHASE_ERROR
#define SCSI_STATUS_DATA_OUT_CSW_CMD_PASSED                       USBD_BOT_ABORT_DATA_OUT_CSW_CMD_PASSED
#define SCSI_STATUS_DATA_OUT_CSW_CMD_FAILED                       USBD_BOT_ABORT_DATA_OUT_CSW_CMD_FAILED
#define SCSI_STATUS_DATA_OUT_CSW_PHASE_ERROR                      USBD_BOT_ABORT_DATA_OUT_CSW_PHASE_ERROR

#define SCSI_DATA_LENGTH_INQUIRE_PAGE_00                          7
#define SCSI_DATA_LENGTH_INQUIRE_STANDARD                         36
#define SCSI_DATA_LENGTH_REQUEST_SENSE                            18
#define SCSI_DATA_LENGTH_MODE_SENSE_6                             4
#define SCSI_DATA_LENGTH_MODE_SENSE_10                            8
#define SCSI_DATA_LENGTH_READ_CAPACITY_10                         8
#define SCSI_DATA_LENGTH_READ_FORMAT_CAPACITIES                   12

#define SCSI_SKEY_NO_SENSE                                        0x00
#define SCSI_SKEY_RECOVERED_ERROR                                 0x01
#define SCSI_SKEY_NOT_READY                                       0x02
#define SCSI_SKEY_MEDIUM_ERROR                                    0x03
#define SCSI_SKEY_HARDWARE_ERROR                                  0x04
#define SCSI_SKEY_ILLEGAL_REQUEST                                 0x05
#define SCSI_SKEY_UNIT_ATTENTION                                  0x06
#define SCSI_SKEY_DATA_PROTECT                                    0x07
#define SCSI_SKEY_BLANK_CHECK                                     0x08
#define SCSI_SKEY_VENDOR_SPECIFIC                                 0x09
#define SCSI_SKEY_ABORTED_COMMAND                                 0x0b
#define SCSI_SKEY_VOLUME_OVERFLOW                                 0x0d
#define SCSI_SKEY_MISCOMPARE                                      0x0e

#define SCSI_ASC_NO_SENSE                                         0x0000
#define SCSI_ASC_NO_SEEK_COMPLETE                                 0x0200
#define SCSI_ASC_WRITE_FAULT                                      0x0300
#define SCSI_ASC_LOGICAL_DRIVE_NOT_READY_BECOMING_READY           0x0401
#define SCSI_ASC_LOGICAL_DRIVE_NOT_READY_INITIALIZATION_REQUIRED  0x0402
#define SCSI_ASC_LOGICAL_UNIT_NOT_READY_FORMAT_IN_PROGRESS        0x0404
#define SCSI_ASC_LOGICAL_DRIVE_NOT_READY_DEVICE_IS_BUSY           0x04FF
#define SCSI_ASC_NO_REFERENCE_POSITION_FOUND                      0x0600
#define SCSI_ASC_LOGICAL_UNIT_COMMUNICATION_FAILURE               0x0800
#define SCSI_ASC_LOGICAL_UNIT_COMMUNICATION_TIMEOUT               0x0801
#define SCSI_ASC_LOGICAL_UNIT_COMMUNICATION_OVERRUN               0x0880
#define SCSI_ASC_ID_CRC_ERROR                                     0x1000
#define SCSI_ASC_UNRECOVERED_READ_ERROR                           0x1100
#define SCSI_ASC_ADDRESS_MARK_NOT_FOUND_FOR_ID_FIELD              0x1200
#define SCSI_ASC_ADDRESS_MARK_NOT_FOUND_FOR_DATA_FIELD            0x1300
#define SCSI_ASC_RECORDED_ENTITY_NOT_FOUND                        0x1400
#define SCSI_ASC_RECOVERED_DATA_WITH_RETRIES                      0x1701
#define SCSI_ASC_RECOVERED_DATA_WITH_ECC                          0x1800
#define SCSI_ASC_PARAMETER_LIST_LENGTH_ERROR                      0x1A00
#define SCSI_ASC_INVALID_COMMAND_OPERATION_CODE                   0x2000
#define SCSI_ASC_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE               0x2100
#define SCSI_ASC_INVALID_FIELD_IN_COMMAND_PACKET                  0x2400
#define SCSI_ASC_LOGICAL_UNIT_NOT_SUPPORTED                       0x2500
#define SCSI_ASC_INVALID_FIELD_IN_PARAMETER_LIST                  0x2600
#define SCSI_ASC_PARAMETER_NOT_SUPPORTED                          0x2601
#define SCSI_ASC_PARAMETER_VALUE_INVALID                          0x2602
#define SCSI_ASC_WRITE_PROTECTED_MEDIA                            0x2700
#define SCSI_ASC_NOT_READY_TO_READY_TRANSITION_MEDIA_CHANGED      0x2800
#define SCSI_ASC_POWER_ON_RESET_OR_BUS_DEVICE_RESET_OCCURRED      0x2900
#define SCSI_ASC_COMMANDS_CLEARED_BY_ANOTHER_INITIATOR            0x2F00
#define SCSI_ASC_CANNOT_READ_MEDIUM_UNKNOWN_FORMAT                0x3001
#define SCSI_ASC_FORMAT_COMMAND_FAILED                            0x3101
#define SCSI_ASC_SAVING_PARAMETERS_NOT_SUPPORT                    0x3900
#define SCSI_ASC_MEDIUM_NOT_PRESENT                               0x3A00
#define SCSI_ASC_OVERLAPPED_COMMAND_ATTEMPTED                     0x4E00
#define SCSI_ASC_USB_TO_HOST_SYSTEM_INTERFACE_FAILURE             0x5400
#define SCSI_ASC_INSUFFICIENT_RESOURCES                           0x8000
#define SCSI_ASC_UNKNOWN_ERROR                                    0xFFFF

static void SCSI_ProcessStart(USBD_HandleTypeDef *pdev);
static void SCSI_ProcessStop(USBD_HandleTypeDef *pdev);
static void SCSI_ProcessReset(USBD_HandleTypeDef *pdev);
static void SCSI_ProcessCommand(USBD_HandleTypeDef *pdev);
static void SCSI_ProcessRead(USBD_HandleTypeDef *pdev);
static void SCSI_ProcessWrite(USBD_HandleTypeDef *pdev);

/***********************************************************************************************************************/

#define USBD_BOT_STATE_NONE                     0       /* None */
#define USBD_BOT_STATE_IDLE                     1       /* Idle */
#define USBD_BOT_STATE_RECOVERY_RESET           2       /* Waiting for reset */
#define USBD_BOT_STATE_RECOVERY_DATA_IN         3       /* Waiting for clear feature halt on data in */
#define USBD_BOT_STATE_RECOVERY_DATA_OUT        4       /* Waiting for clear feature halt on data out */
#define USBD_BOT_STATE_HALT_DATA_IN             5       /* Waiting for clear feature halt on data in */
#define USBD_BOT_STATE_HALT_DATA_OUT            6       /* Waiting for clear feature halt on data out */
#define USBD_BOT_STATE_DATA_IN                  7       /* Data In state */
#define USBD_BOT_STATE_DATA_IN_LAST             8       /* Last Data In Last (passed) */
#define USBD_BOT_STATE_DATA_IN_LAST_STALL       9       /* Last Data In Last (failed or phase error) */
#define USBD_BOT_STATE_DATA_OUT                 10      /* Data Out state */

#define USBD_BOT_ABORT_CBW                      3
#define USBD_BOT_ABORT_DATA_IN_CSW_CMD_PASSED   4
#define USBD_BOT_ABORT_DATA_IN_CSW_CMD_FAILED   5
#define USBD_BOT_ABORT_DATA_IN_CSW_PHASE_ERROR  6
#define USBD_BOT_ABORT_DATA_OUT_CSW_CMD_PASSED  7
#define USBD_BOT_ABORT_DATA_OUT_CSW_CMD_FAILED  8
#define USBD_BOT_ABORT_DATA_OUT_CSW_PHASE_ERROR 9
   
#define USBD_BOT_CBW_SIGNATURE                  0x43425355
#define USBD_BOT_CSW_SIGNATURE                  0x53425355
#define USBD_BOT_CBW_LENGTH                     31
#define USBD_BOT_CSW_LENGTH                     13

#define USBD_BOT_CSW_CMD_PASSED                 0
#define USBD_BOT_CSW_CMD_FAILED                 1
#define USBD_BOT_CSW_PHASE_ERROR                2

#define USBD_BOT_GET_MAX_LUN                    0xfe
#define USBD_BOT_RESET                          0xff

static void MSC_BOT_Open(USBD_HandleTypeDef *pdev);
static void MSC_BOT_Close(USBD_HandleTypeDef *pdev);
static void MSC_BOT_Abort(USBD_HandleTypeDef *pdev, uint8_t abort);
static void MSC_BOT_Stall(USBD_HandleTypeDef *pdev, uint8_t epnum);
static void MSC_BOT_ClearHalt(USBD_HandleTypeDef *pdev, uint8_t ep_addr);
static void MSC_BOT_SendCSW(USBD_HandleTypeDef *pdev, uint8_t status);
static void MSC_BOT_Transmit(USBD_HandleTypeDef *pdev, const uint8_t *data, uint32_t length);

static void MSC_BOT_SetupTransmit(USBD_HandleTypeDef *pdev, uint32_t length);
static void MSC_BOT_ContinueTransmit(USBD_HandleTypeDef *pdev);
static void MSC_BOT_FinishTransmit(USBD_HandleTypeDef *pdev, uint8_t status);

static void MSC_BOT_SetupReceive(USBD_HandleTypeDef *pdev, uint32_t length);
static void MSC_BOT_ContinueReceive(USBD_HandleTypeDef *pdev);
static void MSC_BOT_FinishReceive(USBD_HandleTypeDef *pdev, uint8_t status);

/***********************************************************************************************************************/
/***********************************************************************************************************************/

#define USBD_MSC_QUEUE_ENTRIES   8
#define USBD_MSC_QUEUE_THRESHOLD 4

typedef void (*stm32l0_usbd_msc_routine_t)(USBD_HandleTypeDef*);
   
typedef struct
{
    uint32_t dSignature;
    uint32_t dTag;
    uint32_t dDataLength;
    uint8_t  bmFlags;
    uint8_t  bLUN;
    uint8_t  bCBLength;
    uint8_t  CB[16];
} USBD_MSC_BOT_CBWTypeDef;

typedef struct
{
    uint32_t dSignature;
    uint32_t dTag;
    uint32_t dDataResidue;
    uint8_t  bStatus;
} USBD_MSC_BOT_CSWTypeDef;

typedef struct _stm32l0_usbd_msc_device_t {
    uint8_t                  interface;  
    uint8_t                  max_lun;   
    uint16_t                 feature; 
    const dosfs_storage_interface_t *storage;

    volatile uint32_t        queue_lock;
    armv6m_work_t            queue_work;
    volatile uint32_t        queue_head;
    volatile uint32_t        queue_tail;
    stm32l0_usbd_msc_routine_t queue_data[USBD_MSC_QUEUE_ENTRIES];

    uint8_t                  bot_state;
    bool                     bot_fault;
    uint8_t                  bot_data[18]; // should be SCSI_DATA_LENGTH_REQUEST_SENSE
    USBD_MSC_BOT_CBWTypeDef  bot_cbw;
    USBD_MSC_BOT_CSWTypeDef  bot_csw;

    volatile uint8_t         bot_count;
    uint8_t                  bot_index;
    volatile uint8_t         bot_rx_busy;
    volatile uint8_t         bot_tx_busy;
    volatile uint32_t        bot_rx_length;
    volatile uint32_t        bot_tx_length;
  
    uint8_t                  scsi_sense_skey;
    uint16_t                 scsi_sense_asc;
    uint32_t                 scsi_sense_address;
    uint32_t                 scsi_blk_count;
    uint32_t                 scsi_blk_size;
    uint32_t                 scsi_blk_address;
    uint32_t                 scsi_blk_length;
    uint32_t                 scsi_blk_queue;
    uint8_t                  scsi_blk_fault;
    bool                     scsi_blk_busy;

    uint32_t                 scsi_index;
    uint8_t                  *scsi_data[2];
    const uint8_t            *scsi_inquiry_data;
} stm32l0_usbd_msc_device_t;

static stm32l0_usbd_msc_device_t stm32l0_usbd_msc_device;

/***********************************************************************************************************************/

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

static void SCSI_SenseCode(USBD_HandleTypeDef *pdev, uint8_t SKey, uint16_t ASC, uint32_t blk_address)
{
    stm32l0_usbd_msc_device.scsi_sense_skey = SKey;
    stm32l0_usbd_msc_device.scsi_sense_asc = ASC;
    stm32l0_usbd_msc_device.scsi_sense_address = blk_address;
}

static int SCSI_CommandPassed(USBD_HandleTypeDef *pdev)
{
    if (stm32l0_usbd_msc_device.bot_cbw.dDataLength != 0)
    {
        if (stm32l0_usbd_msc_device.bot_cbw.bmFlags & 0x80)
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
    SCSI_SenseCode(pdev, SKey, ASC, blk_address);

    if (stm32l0_usbd_msc_device.bot_cbw.dDataLength == 0)
    {
        /* case (1) Hn = Dn */
        return SCSI_STATUS_CSW_CMD_FAILED;
    }
    else
    {
        if (stm32l0_usbd_msc_device.bot_cbw.bmFlags & 0x80)
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
    if (stm32l0_usbd_msc_device.bot_cbw.dDataLength == 0)
    {
        /* case (1) Hn = Dn */
        *p_status = SCSI_STATUS_CSW_CMD_PASSED;

        return false;
    }
    else
    {
        if (!(stm32l0_usbd_msc_device.bot_cbw.bmFlags & 0x80))
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
    if (allocation)
    {
        if (stm32l0_usbd_msc_device.bot_cbw.dDataLength == 0)
        {
            /* case (2) Hn < Di */
            *p_status = SCSI_STATUS_DATA_IN_CSW_PHASE_ERROR;

            return false;
        }
        else
        {
            if (stm32l0_usbd_msc_device.bot_cbw.bmFlags & 0x80)
            {
                if (stm32l0_usbd_msc_device.bot_cbw.dDataLength != allocation)
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
        if (stm32l0_usbd_msc_device.bot_cbw.dDataLength == 0)
        {
            /* case (1) Hn = Dn */
            *p_status =SCSI_STATUS_CSW_CMD_PASSED;
        }
        else
        {
            if (stm32l0_usbd_msc_device.bot_cbw.bmFlags & 0x80)
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

static int SCSI_TestUnitReady(USBD_HandleTypeDef *pdev, const uint8_t *params)
{
    if (!stm32l0_usbd_msc_device.storage->IsReady())
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
    } 

    return SCSI_CommandPassed(pdev);
}

static int SCSI_StartStopUnit(USBD_HandleTypeDef *pdev, const uint8_t *params)
{
    if (!stm32l0_usbd_msc_device.storage->StartStopUnit(((params[4] >> 0) & 1), ((params[4] >> 1) & 1)))
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
    }
    
    return SCSI_CommandPassed(pdev);
}

static int SCSI_AllowMediumRemoval(USBD_HandleTypeDef *pdev, const uint8_t *params)
{
    if (!stm32l0_usbd_msc_device.storage->PreventAllowMediumRemoval((params[4] >> 0) & 1))
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
    } 

    return SCSI_CommandPassed(pdev);
}

static int SCSI_Inquiry(USBD_HandleTypeDef *pdev, const uint8_t *params)
{
    const uint8_t *data;
    uint32_t length, allocation;
    int status;
    
    allocation = params[4];

    if (params[1] & 0x01)
    {
        stm32l0_usbd_msc_device.bot_data[0]  = 0x00;
        stm32l0_usbd_msc_device.bot_data[1]  = 0x00;
        stm32l0_usbd_msc_device.bot_data[2]  = 0x00;
        stm32l0_usbd_msc_device.bot_data[3]  = 0x03;
        stm32l0_usbd_msc_device.bot_data[4]  = 0x00;
        stm32l0_usbd_msc_device.bot_data[5]  = 0x00;
        stm32l0_usbd_msc_device.bot_data[6]  = 0x80;
        stm32l0_usbd_msc_device.bot_data[7]  = 0x83;

        data = &(stm32l0_usbd_msc_device.bot_data[0]);

        length = SCSI_DATA_LENGTH_INQUIRE_PAGE_00;
    }
    else
    {
        data = (const uint8_t *)stm32l0_usbd_msc_device.scsi_inquiry_data;

        length = data[4] + 5;
    }

    if (!SCSI_CheckAllocation(pdev, allocation, length, &status))
    {
        return status;
    }

    armv6m_svcall_3((uint32_t)&MSC_BOT_Transmit, (uint32_t)pdev, (uint32_t)data, (uint32_t)MIN(allocation, length));
    
    return SCSI_STATUS_DATA;
}

static int SCSI_RequestSense(USBD_HandleTypeDef  *pdev, const uint8_t *params)
{
    uint32_t allocation;
    int status;
    
    allocation = params[4];
    
    stm32l0_usbd_msc_device.bot_data[0]  = 0x70;
    stm32l0_usbd_msc_device.bot_data[1]  = 0x00;
    stm32l0_usbd_msc_device.bot_data[2]  = stm32l0_usbd_msc_device.scsi_sense_skey;
    stm32l0_usbd_msc_device.bot_data[3]  = 0x00;
    stm32l0_usbd_msc_device.bot_data[4]  = 0x00;
    stm32l0_usbd_msc_device.bot_data[5]  = 0x00;
    stm32l0_usbd_msc_device.bot_data[6]  = 0x00;
    stm32l0_usbd_msc_device.bot_data[7]  = 0x0a;
    stm32l0_usbd_msc_device.bot_data[8]  = 0x00;
    stm32l0_usbd_msc_device.bot_data[9]  = 0x00;
    stm32l0_usbd_msc_device.bot_data[10] = 0x00;
    stm32l0_usbd_msc_device.bot_data[11] = 0x00;
    stm32l0_usbd_msc_device.bot_data[12] = (uint8_t)(stm32l0_usbd_msc_device.scsi_sense_asc >> 8);
    stm32l0_usbd_msc_device.bot_data[13] = (uint8_t)(stm32l0_usbd_msc_device.scsi_sense_asc >> 0);
    stm32l0_usbd_msc_device.bot_data[14] = 0x00;
    stm32l0_usbd_msc_device.bot_data[15] = 0x00;
    stm32l0_usbd_msc_device.bot_data[16] = 0x00;
    stm32l0_usbd_msc_device.bot_data[17] = 0x00;

    if (stm32l0_usbd_msc_device.scsi_sense_skey == SCSI_SKEY_HARDWARE_ERROR)
    {
        stm32l0_usbd_msc_device.bot_data[0] |= 0x80;

        stm32l0_usbd_msc_device.bot_data[3]  = (uint8_t)(stm32l0_usbd_msc_device.scsi_sense_address >> 24);
        stm32l0_usbd_msc_device.bot_data[4]  = (uint8_t)(stm32l0_usbd_msc_device.scsi_sense_address >> 16);
        stm32l0_usbd_msc_device.bot_data[5]  = (uint8_t)(stm32l0_usbd_msc_device.scsi_sense_address >>  8);
        stm32l0_usbd_msc_device.bot_data[6]  = (uint8_t)(stm32l0_usbd_msc_device.scsi_sense_address >>  0);
    }

    stm32l0_usbd_msc_device.scsi_sense_skey = SCSI_SKEY_NO_SENSE;
    stm32l0_usbd_msc_device.scsi_sense_asc = SCSI_ASC_NO_SENSE;
    stm32l0_usbd_msc_device.scsi_sense_address = 0;
    
    if (!SCSI_CheckAllocation(pdev, allocation, SCSI_DATA_LENGTH_REQUEST_SENSE, &status))
    {
        return status;
    }

    armv6m_svcall_3((uint32_t)&MSC_BOT_Transmit, (uint32_t)pdev, (uint32_t)stm32l0_usbd_msc_device.bot_data, (uint32_t)MIN(allocation, SCSI_DATA_LENGTH_REQUEST_SENSE));
    
    return SCSI_STATUS_DATA;
}

static int SCSI_ModeSense6(USBD_HandleTypeDef *pdev, const uint8_t *params)
{
    bool write_protected;
    int status;

    if (!stm32l0_usbd_msc_device.storage->GetWriteProtected(&write_protected))
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
    } 

    stm32l0_usbd_msc_device.bot_data[0]  = 0x03;
    stm32l0_usbd_msc_device.bot_data[1]  = 0x00;
    stm32l0_usbd_msc_device.bot_data[2]  = write_protected ? 0x80 : 0x00;
    stm32l0_usbd_msc_device.bot_data[3]  = 0x00;

    if (!SCSI_CheckLength(pdev, SCSI_DATA_LENGTH_MODE_SENSE_6, &status))
    {
        return status;
    }

    armv6m_svcall_3((uint32_t)&MSC_BOT_Transmit, (uint32_t)pdev, (uint32_t)stm32l0_usbd_msc_device.bot_data, (uint32_t)SCSI_DATA_LENGTH_MODE_SENSE_6);
        
    return SCSI_STATUS_DATA;
}

static int SCSI_ModeSense10(USBD_HandleTypeDef *pdev, const uint8_t *params)
{
    bool write_protected;
    int status;
    
    if (!stm32l0_usbd_msc_device.storage->GetWriteProtected(&write_protected))
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
    } 

    stm32l0_usbd_msc_device.bot_data[0]  = 0x00;
    stm32l0_usbd_msc_device.bot_data[1]  = 0x06;
    stm32l0_usbd_msc_device.bot_data[2]  = 0x00;
    stm32l0_usbd_msc_device.bot_data[3]  = write_protected ? 0x80 : 0x00;
    stm32l0_usbd_msc_device.bot_data[4]  = 0x00;
    stm32l0_usbd_msc_device.bot_data[5]  = 0x00;
    stm32l0_usbd_msc_device.bot_data[6]  = 0x00;
    stm32l0_usbd_msc_device.bot_data[7]  = 0x00;
    
    if (!SCSI_CheckLength(pdev, SCSI_DATA_LENGTH_MODE_SENSE_10, &status))
    {
        return status;
    }

    armv6m_svcall_3((uint32_t)&MSC_BOT_Transmit, (uint32_t)pdev, (uint32_t)stm32l0_usbd_msc_device.bot_data, (uint32_t)SCSI_DATA_LENGTH_MODE_SENSE_10);

    return SCSI_STATUS_DATA;
}

static int SCSI_ReadFormatCapacities(USBD_HandleTypeDef *pdev, const uint8_t *params)
{
    uint32_t allocation;
    int status;
    
    allocation = (params[7] << 8) | (params[8]);

    stm32l0_usbd_msc_device.bot_data[0]  = 0x00;
    stm32l0_usbd_msc_device.bot_data[1]  = 0x00;
    stm32l0_usbd_msc_device.bot_data[2]  = 0x00;
    stm32l0_usbd_msc_device.bot_data[3]  = 0x08;
    stm32l0_usbd_msc_device.bot_data[4]  = (uint8_t)(stm32l0_usbd_msc_device.scsi_blk_count >> 24);
    stm32l0_usbd_msc_device.bot_data[5]  = (uint8_t)(stm32l0_usbd_msc_device.scsi_blk_count >> 16);
    stm32l0_usbd_msc_device.bot_data[6]  = (uint8_t)(stm32l0_usbd_msc_device.scsi_blk_count >>  8);
    stm32l0_usbd_msc_device.bot_data[7]  = (uint8_t)(stm32l0_usbd_msc_device.scsi_blk_count >>  0);
    stm32l0_usbd_msc_device.bot_data[8]  = 0x02;
    stm32l0_usbd_msc_device.bot_data[9]  = (uint8_t)(stm32l0_usbd_msc_device.scsi_blk_size >> 16);
    stm32l0_usbd_msc_device.bot_data[10] = (uint8_t)(stm32l0_usbd_msc_device.scsi_blk_size >>  8);
    stm32l0_usbd_msc_device.bot_data[11] = (uint8_t)(stm32l0_usbd_msc_device.scsi_blk_size >>  0);
    
    if (!SCSI_CheckAllocation(pdev, allocation, SCSI_DATA_LENGTH_READ_FORMAT_CAPACITIES, &status))
    {
        return status;
    }

    armv6m_svcall_3((uint32_t)&MSC_BOT_Transmit, (uint32_t)pdev, (uint32_t)stm32l0_usbd_msc_device.bot_data, (uint32_t)MIN(allocation, SCSI_DATA_LENGTH_READ_FORMAT_CAPACITIES));
    
    return SCSI_STATUS_DATA;
}

static int SCSI_ReadCapacity10(USBD_HandleTypeDef *pdev, const uint8_t *params)
{
    int status;
    
    if (!stm32l0_usbd_msc_device.storage->GetCapacity(&stm32l0_usbd_msc_device.scsi_blk_count, &stm32l0_usbd_msc_device.scsi_blk_size))
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
    } 

    stm32l0_usbd_msc_device.bot_data[0] = (uint8_t)((stm32l0_usbd_msc_device.scsi_blk_count -1) >> 24);
    stm32l0_usbd_msc_device.bot_data[1] = (uint8_t)((stm32l0_usbd_msc_device.scsi_blk_count -1) >> 16);
    stm32l0_usbd_msc_device.bot_data[2] = (uint8_t)((stm32l0_usbd_msc_device.scsi_blk_count -1) >>  8);
    stm32l0_usbd_msc_device.bot_data[3] = (uint8_t)((stm32l0_usbd_msc_device.scsi_blk_count -1) >>  0);
    stm32l0_usbd_msc_device.bot_data[4] = (uint8_t)(stm32l0_usbd_msc_device.scsi_blk_size >> 24);
    stm32l0_usbd_msc_device.bot_data[5] = (uint8_t)(stm32l0_usbd_msc_device.scsi_blk_size >> 16);
    stm32l0_usbd_msc_device.bot_data[6] = (uint8_t)(stm32l0_usbd_msc_device.scsi_blk_size >>  8);
    stm32l0_usbd_msc_device.bot_data[7] = (uint8_t)(stm32l0_usbd_msc_device.scsi_blk_size >>  0);

    if (!SCSI_CheckLength(pdev, SCSI_DATA_LENGTH_READ_CAPACITY_10, &status))
    {
        return status;
    }

    armv6m_svcall_3((uint32_t)&MSC_BOT_Transmit, (uint32_t)pdev, (uint32_t)stm32l0_usbd_msc_device.bot_data, (uint32_t)SCSI_DATA_LENGTH_READ_CAPACITY_10);
    
    return SCSI_STATUS_DATA;
}

static int SCSI_Read10(USBD_HandleTypeDef *pdev, const uint8_t *params)
{
    bool changed;

    stm32l0_usbd_msc_device.scsi_blk_address = (params[2] << 24) | (params[3] << 16) | (params[4] << 8) | (params[5] << 0);
    stm32l0_usbd_msc_device.scsi_blk_length = (params[7] << 8) | (params[8] << 0);
    
    /* Check in Media is changed */
    if (!stm32l0_usbd_msc_device.storage->GetChanged(&changed))
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
    } 

    if (changed)
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_UNIT_ATTENTION, SCSI_ASC_NOT_READY_TO_READY_TRANSITION_MEDIA_CHANGED, 0);
    } 
    
    if (stm32l0_usbd_msc_device.scsi_blk_count == 0)
    {
        if (!stm32l0_usbd_msc_device.storage->GetCapacity(&stm32l0_usbd_msc_device.scsi_blk_count, &stm32l0_usbd_msc_device.scsi_blk_size))
        {
            return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
        } 
    }
    
    /* Check address range */
    if ((stm32l0_usbd_msc_device.scsi_blk_address + stm32l0_usbd_msc_device.scsi_blk_length) > stm32l0_usbd_msc_device.scsi_blk_count)
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_ILLEGAL_REQUEST, SCSI_ASC_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE, 0);
    }

    if (stm32l0_usbd_msc_device.bot_cbw.dDataLength != (stm32l0_usbd_msc_device.scsi_blk_length * stm32l0_usbd_msc_device.scsi_blk_size))
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_ILLEGAL_REQUEST, SCSI_ASC_INVALID_FIELD_IN_COMMAND_PACKET, 0);
    }

    if (stm32l0_usbd_msc_device.scsi_blk_length == 0)
    {
        return SCSI_CommandPassed(pdev);
    }
    else
    {
        if (!(stm32l0_usbd_msc_device.bot_cbw.bmFlags & 0x80))
        {
            /* case (10) Ho <> Di */
            return SCSI_STATUS_DATA_OUT_CSW_PHASE_ERROR;
        }

        /* Acquire Media Lock */
        if (!stm32l0_usbd_msc_device.storage->Acquire())
        {
            return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
        } 

        stm32l0_usbd_msc_device.scsi_blk_queue = 0;
        stm32l0_usbd_msc_device.scsi_blk_fault = false;
        stm32l0_usbd_msc_device.scsi_blk_busy = true;

        stm32l0_usbd_msc_device.scsi_index = 0;

        if (!stm32l0_usbd_msc_device.storage->Read(stm32l0_usbd_msc_device.scsi_data[stm32l0_usbd_msc_device.scsi_index], stm32l0_usbd_msc_device.scsi_blk_address, 1, (stm32l0_usbd_msc_device.scsi_blk_length == 1)))
        {
            SCSI_CommandFailed(pdev, SCSI_SKEY_HARDWARE_ERROR, SCSI_ASC_UNRECOVERED_READ_ERROR, stm32l0_usbd_msc_device.scsi_blk_address);

	    stm32l0_usbd_msc_device.scsi_blk_fault = true;
	}

        armv6m_svcall_2((uint32_t)&MSC_BOT_SetupTransmit, (uint32_t)pdev, (uint32_t)stm32l0_usbd_msc_device.scsi_blk_length);
	
        stm32l0_usbd_msc_device.scsi_blk_queue += 1;
        stm32l0_usbd_msc_device.scsi_index ^= 1;

        stm32l0_usbd_msc_device.scsi_blk_address += 1;
        stm32l0_usbd_msc_device.scsi_blk_length -= 1;

	if (stm32l0_usbd_msc_device.scsi_blk_length != 0)
	{
	    if (!stm32l0_usbd_msc_device.scsi_blk_fault)
	    {
		if (!stm32l0_usbd_msc_device.storage->Read(stm32l0_usbd_msc_device.scsi_data[stm32l0_usbd_msc_device.scsi_index], stm32l0_usbd_msc_device.scsi_blk_address, 1, (stm32l0_usbd_msc_device.scsi_blk_length == 1)))
		{
		    SCSI_CommandFailed(pdev, SCSI_SKEY_HARDWARE_ERROR, SCSI_ASC_UNRECOVERED_READ_ERROR, stm32l0_usbd_msc_device.scsi_blk_address);
		    
		    stm32l0_usbd_msc_device.scsi_blk_fault = true;
		}
	    }
	    
	    armv6m_svcall_1((uint32_t)&MSC_BOT_ContinueTransmit, (uint32_t)pdev);
	    
	    stm32l0_usbd_msc_device.scsi_blk_queue += 1;
	    stm32l0_usbd_msc_device.scsi_index ^= 1;
	    
	    stm32l0_usbd_msc_device.scsi_blk_address += 1;
	    stm32l0_usbd_msc_device.scsi_blk_length -= 1;
	}
	
        return SCSI_STATUS_DATA;
    }
}

static int SCSI_Write10(USBD_HandleTypeDef *pdev, const uint8_t *params)
{
    bool write_protected, changed;

    stm32l0_usbd_msc_device.scsi_blk_address = (params[2] << 24) | (params[3] << 16) | (params[4] << 8) | (params[5] << 0);
    stm32l0_usbd_msc_device.scsi_blk_length = (params[7] << 8) | (params[8] << 0);

    /* Check in Media is changed */
    if (!stm32l0_usbd_msc_device.storage->GetChanged(&changed))
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
    } 

    if (changed)
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_UNIT_ATTENTION, SCSI_ASC_NOT_READY_TO_READY_TRANSITION_MEDIA_CHANGED, 0);
    } 
    
    /* Check If media is write-protected */
    if (!stm32l0_usbd_msc_device.storage->GetWriteProtected(&write_protected))
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
    } 

    if (write_protected)
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_WRITE_PROTECTED_MEDIA, 0);
    } 

    if (stm32l0_usbd_msc_device.scsi_blk_count == 0)
    {
        if (!stm32l0_usbd_msc_device.storage->GetCapacity(&stm32l0_usbd_msc_device.scsi_blk_count, &stm32l0_usbd_msc_device.scsi_blk_size))
        {
            return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
        } 
    }
    
    /* Check address range */
    if ((stm32l0_usbd_msc_device.scsi_blk_address + stm32l0_usbd_msc_device.scsi_blk_length) > stm32l0_usbd_msc_device.scsi_blk_count)
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_ILLEGAL_REQUEST, SCSI_ASC_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE, 0);
    }

    if (stm32l0_usbd_msc_device.bot_cbw.dDataLength != (stm32l0_usbd_msc_device.scsi_blk_length * stm32l0_usbd_msc_device.scsi_blk_size))
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_ILLEGAL_REQUEST, SCSI_ASC_INVALID_FIELD_IN_COMMAND_PACKET, 0);
    }

    if (stm32l0_usbd_msc_device.scsi_blk_length == 0)
    {
        return SCSI_CommandPassed(pdev);
    }
    else
    {
        if (stm32l0_usbd_msc_device.bot_cbw.bmFlags & 0x80)
        {
            /* case (8) Hi <> Do */
            return SCSI_STATUS_DATA_IN_CSW_PHASE_ERROR;
        }

        /* Acquire Media Lock */
        if (!stm32l0_usbd_msc_device.storage->Acquire())
        {
            return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
        } 

	stm32l0_usbd_msc_device.scsi_blk_queue = 0;
        stm32l0_usbd_msc_device.scsi_blk_fault = false;
        stm32l0_usbd_msc_device.scsi_blk_busy = true;

        stm32l0_usbd_msc_device.scsi_index = 0;
	
        armv6m_svcall_2((uint32_t)&MSC_BOT_SetupReceive, (uint32_t)pdev, (uint32_t)stm32l0_usbd_msc_device.scsi_blk_length);

        return SCSI_STATUS_DATA;
    }
}

static int SCSI_Verify10(USBD_HandleTypeDef *pdev, const uint8_t *params)
{
    uint32_t blk_address, blk_length;
    bool changed;

    blk_address = (params[2] << 24) | (params[3] << 16) | (params[4] << 8) | (params[5] << 0);
    blk_length = (params[7] << 8) | (params[8] << 0);

    if ((params[1] & 0x02) == 0x02) 
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_ILLEGAL_REQUEST, SCSI_ASC_INVALID_FIELD_IN_COMMAND_PACKET, 0);
    }

    /* Check in Media is changed */
    if (!stm32l0_usbd_msc_device.storage->GetChanged(&changed))
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
    } 

    if (changed)
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_UNIT_ATTENTION, SCSI_ASC_NOT_READY_TO_READY_TRANSITION_MEDIA_CHANGED, 0);
    } 
    
    if (stm32l0_usbd_msc_device.scsi_blk_count == 0)
    {
        if (!stm32l0_usbd_msc_device.storage->GetCapacity(&stm32l0_usbd_msc_device.scsi_blk_count, &stm32l0_usbd_msc_device.scsi_blk_size))
        {
            return SCSI_CommandFailed(pdev, SCSI_SKEY_NOT_READY, SCSI_ASC_MEDIUM_NOT_PRESENT, 0);
        } 
    }
    
    /* Check address range */
    if ((blk_address + blk_length) > stm32l0_usbd_msc_device.scsi_blk_count)
    {
        return SCSI_CommandFailed(pdev, SCSI_SKEY_ILLEGAL_REQUEST, SCSI_ASC_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE, 0);
    }

    return SCSI_CommandPassed(pdev);
}

static void SCSI_ProcessStart(USBD_HandleTypeDef *pdev)
{
    uint8_t *cache_data;
    const uint8_t *inquiry_data;
    
    stm32l0_usbd_msc_device.storage->Init(&cache_data, &inquiry_data);

    stm32l0_usbd_msc_device.scsi_sense_skey = SCSI_SKEY_NO_SENSE;
    stm32l0_usbd_msc_device.scsi_sense_asc = SCSI_ASC_NO_SENSE;
    stm32l0_usbd_msc_device.scsi_sense_address = 0;

    stm32l0_usbd_msc_device.scsi_blk_size = 0;
    stm32l0_usbd_msc_device.scsi_blk_count = 0;
    stm32l0_usbd_msc_device.scsi_blk_address = 0;
    stm32l0_usbd_msc_device.scsi_blk_length = 0;
    stm32l0_usbd_msc_device.scsi_blk_fault = 0;
    stm32l0_usbd_msc_device.scsi_blk_busy = false;

    stm32l0_usbd_msc_device.scsi_index = 0;
    stm32l0_usbd_msc_device.scsi_data[0] = cache_data;
    stm32l0_usbd_msc_device.scsi_data[1] = cache_data + MSC_MEDIA_PACKET;

    stm32l0_usbd_msc_device.scsi_inquiry_data = inquiry_data;
    
}

static void SCSI_ProcessStop(USBD_HandleTypeDef *pdev)
{
    stm32l0_usbd_msc_device.storage->StartStopUnit(true, false);
    
    stm32l0_usbd_msc_device.storage->DeInit();
}

static void SCSI_ProcessReset(USBD_HandleTypeDef *pdev)
{
    stm32l0_usbd_msc_device.scsi_blk_fault = false;
    stm32l0_usbd_msc_device.scsi_blk_busy = false;
}

static void SCSI_ProcessCommand(USBD_HandleTypeDef *pdev)
{
    const uint8_t *params = (const uint8_t*)&stm32l0_usbd_msc_device.bot_cbw.CB[0];
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

static void SCSI_ProcessRead(USBD_HandleTypeDef *pdev)
{
    if (!stm32l0_usbd_msc_device.scsi_blk_busy)
    {
        /* Reset while read ... */
        return;
    }

    stm32l0_usbd_msc_device.scsi_blk_queue -= 1;

    if (stm32l0_usbd_msc_device.scsi_blk_length == 0)
    {
	if (stm32l0_usbd_msc_device.scsi_blk_queue == 0)
	{
	    armv6m_svcall_2((uint32_t)&MSC_BOT_FinishTransmit, (uint32_t)pdev, (stm32l0_usbd_msc_device.scsi_blk_fault ? USBD_BOT_CSW_CMD_FAILED : USBD_BOT_CSW_CMD_PASSED));

	    stm32l0_usbd_msc_device.scsi_blk_fault = false;
	    stm32l0_usbd_msc_device.scsi_blk_busy = false;
	}
    }
    else
    {
	do
	{
	    if (!stm32l0_usbd_msc_device.scsi_blk_fault)
	    {
		if (!stm32l0_usbd_msc_device.storage->Read(stm32l0_usbd_msc_device.scsi_data[stm32l0_usbd_msc_device.scsi_index], stm32l0_usbd_msc_device.scsi_blk_address, 1, (stm32l0_usbd_msc_device.scsi_blk_length == 1)))
		{
		    SCSI_CommandFailed(pdev, SCSI_SKEY_HARDWARE_ERROR, SCSI_ASC_UNRECOVERED_READ_ERROR, stm32l0_usbd_msc_device.scsi_blk_address);
		    
		    stm32l0_usbd_msc_device.scsi_blk_fault = true;
		}
	    }
	    
	    armv6m_svcall_1((uint32_t)&MSC_BOT_ContinueTransmit, (uint32_t)pdev);

	    stm32l0_usbd_msc_device.scsi_blk_queue += 1;
	    stm32l0_usbd_msc_device.scsi_index ^= 1;
	    
	    stm32l0_usbd_msc_device.scsi_blk_address += 1;
	    stm32l0_usbd_msc_device.scsi_blk_length -= 1;
	}
	while ((stm32l0_usbd_msc_device.scsi_blk_length != 0) && (stm32l0_usbd_msc_device.scsi_blk_queue < USBD_MSC_QUEUE_THRESHOLD) && (stm32l0_usbd_msc_device.bot_count != 2));
    }
}

static void SCSI_ProcessWrite(USBD_HandleTypeDef *pdev)
{
    if (!stm32l0_usbd_msc_device.scsi_blk_busy)
    {
        /* Reset while read ... */
        return;
    }

    stm32l0_usbd_msc_device.bot_csw.dDataResidue -= stm32l0_usbd_msc_device.scsi_blk_size;
    
    if (!stm32l0_usbd_msc_device.scsi_blk_fault)
    {
	if (!stm32l0_usbd_msc_device.storage->Write(stm32l0_usbd_msc_device.scsi_data[stm32l0_usbd_msc_device.scsi_index], stm32l0_usbd_msc_device.scsi_blk_address, 1, true))
        {
            SCSI_SenseCode(pdev, SCSI_SKEY_HARDWARE_ERROR, SCSI_ASC_WRITE_FAULT, stm32l0_usbd_msc_device.scsi_blk_address);

	    stm32l0_usbd_msc_device.scsi_blk_fault = true;
	}
    }
    
    stm32l0_usbd_msc_device.scsi_blk_address += 1;
    stm32l0_usbd_msc_device.scsi_blk_length -= 1;
    
    stm32l0_usbd_msc_device.scsi_index ^= 1;

    if (stm32l0_usbd_msc_device.scsi_blk_length == 0)
    {
	armv6m_svcall_2((uint32_t)&MSC_BOT_FinishReceive, (uint32_t)pdev, (stm32l0_usbd_msc_device.scsi_blk_fault ? USBD_BOT_CSW_CMD_FAILED : USBD_BOT_CSW_CMD_PASSED));

	stm32l0_usbd_msc_device.scsi_blk_fault = false;
	stm32l0_usbd_msc_device.scsi_blk_busy = false;

    }
    else
    {
	armv6m_svcall_1((uint32_t)&MSC_BOT_ContinueReceive, (uint32_t)pdev);
    }
}

/***********************************************************************************************************************/

static void MSC_BOT_Init(USBD_HandleTypeDef *pdev)
{
    armv6m_svcall_1((uint32_t)&MSC_BOT_Open, (uint32_t)pdev);

    stm32l0_usbd_msc_device.bot_state = USBD_BOT_STATE_IDLE;
    
    SCSI_ProcessStart(pdev);
}

static void MSC_BOT_DeInit(USBD_HandleTypeDef *pdev)
{
    SCSI_ProcessStop(pdev);

    stm32l0_usbd_msc_device.bot_state = USBD_BOT_STATE_NONE;

    armv6m_svcall_1((uint32_t)&MSC_BOT_Close, (uint32_t)pdev);
}

static void MSC_BOT_DataIn(USBD_HandleTypeDef *pdev)
{
    switch (stm32l0_usbd_msc_device.bot_state) {
    case USBD_BOT_STATE_DATA_IN:
        SCSI_ProcessRead(pdev);
        break;
    
    case USBD_BOT_STATE_DATA_IN_LAST:
        armv6m_svcall_2((uint32_t)&MSC_BOT_SendCSW, (uint32_t)pdev, (uint32_t)USBD_BOT_CSW_CMD_PASSED);
        break;

    case USBD_BOT_STATE_DATA_IN_LAST_STALL:
	armv6m_svcall_2((uint32_t)&MSC_BOT_Stall, (uint32_t)pdev, (uint32_t)STM32L0_USBD_MSC_DATA_IN_EP_ADDR);

        if (stm32l0_usbd_msc_device.bot_csw.bStatus == USBD_BOT_CSW_PHASE_ERROR)
        {
            stm32l0_usbd_msc_device.bot_state = USBD_BOT_STATE_RECOVERY_RESET;
        }
        else
        {
            stm32l0_usbd_msc_device.bot_state = USBD_BOT_STATE_HALT_DATA_IN;
        }
        break;
        
    default:
        break;
    }
}

static void MSC_BOT_DataCBW(USBD_HandleTypeDef *pdev)
{
    if (stm32l0_usbd_msc_device.bot_state == USBD_BOT_STATE_IDLE)
    {
        stm32l0_usbd_msc_device.bot_csw.dSignature = USBD_BOT_CSW_SIGNATURE;
        stm32l0_usbd_msc_device.bot_csw.dTag = stm32l0_usbd_msc_device.bot_cbw.dTag;
        stm32l0_usbd_msc_device.bot_csw.dDataResidue = stm32l0_usbd_msc_device.bot_cbw.dDataLength;
	
        if ((USBD_LL_GetRxDataSize(pdev, STM32L0_USBD_MSC_DATA_OUT_EP_ADDR) != USBD_BOT_CBW_LENGTH) ||
            (stm32l0_usbd_msc_device.bot_cbw.dSignature != USBD_BOT_CBW_SIGNATURE) ||
            (stm32l0_usbd_msc_device.bot_cbw.bLUN > stm32l0_usbd_msc_device.max_lun) || 
            (stm32l0_usbd_msc_device.bot_cbw.bCBLength == 0) || 
            (stm32l0_usbd_msc_device.bot_cbw.bCBLength > 16))
        {
	    armv6m_svcall_2((uint32_t)&MSC_BOT_Abort, (uint32_t)pdev, USBD_BOT_ABORT_CBW);
        }
        else
        {
            SCSI_ProcessCommand(pdev);
        }
    }
}

static void MSC_BOT_DataTransmit(USBD_HandleTypeDef *pdev)
{
    SCSI_ProcessRead(pdev);
}

static void MSC_BOT_DataReceive(USBD_HandleTypeDef *pdev)
{
    SCSI_ProcessWrite(pdev);
}

static void MSC_BOT_Reset(USBD_HandleTypeDef *pdev)
{
    stm32l0_usbd_msc_device.bot_state = USBD_BOT_STATE_RECOVERY_DATA_IN;
}

static void MSC_BOT_ClearFeature(USBD_HandleTypeDef *pdev)
{
    uint8_t ep_addr;
    
    ep_addr = stm32l0_usbd_msc_device.feature;

    stm32l0_usbd_msc_device.feature = 0;

    armv6m_svcall_2((uint32_t)&MSC_BOT_ClearHalt, (uint32_t)pdev, (uint32_t)ep_addr);
}

static void MSC_BOT_Open(USBD_HandleTypeDef *pdev)
{
    NVIC_DisableIRQ(USB_IRQn);

    /* Open EP OUT */
    USBD_LL_OpenEP(pdev, STM32L0_USBD_MSC_DATA_OUT_EP_ADDR, USBD_EP_TYPE_BULK, STM32L0_USBD_MSC_DATA_MAX_PACKET_SIZE);
    pdev->ep_out[STM32L0_USBD_MSC_DATA_OUT_EP_ADDR & 15].is_used = 1;
  
    /* Open EP IN */
    USBD_LL_OpenEP(pdev, STM32L0_USBD_MSC_DATA_IN_EP_ADDR, USBD_EP_TYPE_BULK, STM32L0_USBD_MSC_DATA_MAX_PACKET_SIZE);  
    pdev->ep_in[STM32L0_USBD_MSC_DATA_IN_EP_ADDR & 15].is_used = 1;
    
    USBD_LL_FlushEP(pdev, STM32L0_USBD_MSC_DATA_OUT_EP_ADDR);
    USBD_LL_FlushEP(pdev, STM32L0_USBD_MSC_DATA_IN_EP_ADDR);
    
    /* Prapare EP to Receive First BOT Cmd */
    USBD_LL_PrepareReceive(pdev, STM32L0_USBD_MSC_DATA_OUT_EP_ADDR, (uint8_t *)&stm32l0_usbd_msc_device.bot_cbw, USBD_BOT_CBW_LENGTH);    

    NVIC_EnableIRQ(USB_IRQn);
}

static void MSC_BOT_Close(USBD_HandleTypeDef *pdev)
{
    NVIC_DisableIRQ(USB_IRQn);

    /* Close MSC EPs */
    USBD_LL_CloseEP(pdev, STM32L0_USBD_MSC_DATA_OUT_EP_ADDR);
    pdev->ep_out[STM32L0_USBD_MSC_DATA_OUT_EP_ADDR & 15].is_used = 0;
  
    /* Open EP IN */
    USBD_LL_CloseEP(pdev, STM32L0_USBD_MSC_DATA_IN_EP_ADDR);
    pdev->ep_in[STM32L0_USBD_MSC_DATA_IN_EP_ADDR & 15].is_used = 0;
  
    NVIC_EnableIRQ(USB_IRQn);
}

static void MSC_BOT_Abort(USBD_HandleTypeDef *pdev, uint8_t abort)
{
    NVIC_DisableIRQ(USB_IRQn);

    switch (abort) {
    case USBD_BOT_ABORT_CBW:
        USBD_LL_StallEP(pdev, STM32L0_USBD_MSC_DATA_IN_EP_ADDR);
        USBD_LL_StallEP(pdev, STM32L0_USBD_MSC_DATA_OUT_EP_ADDR);

        stm32l0_usbd_msc_device.bot_csw.bStatus = USBD_BOT_CSW_CMD_FAILED;

        stm32l0_usbd_msc_device.bot_state = USBD_BOT_STATE_RECOVERY_RESET;
        break;

    case USBD_BOT_ABORT_DATA_IN_CSW_CMD_PASSED:
    case USBD_BOT_ABORT_DATA_IN_CSW_CMD_FAILED:
    case USBD_BOT_ABORT_DATA_IN_CSW_PHASE_ERROR:
        USBD_LL_StallEP(pdev, STM32L0_USBD_MSC_DATA_IN_EP_ADDR);

        stm32l0_usbd_msc_device.bot_csw.bStatus = (abort - USBD_BOT_ABORT_DATA_IN_CSW_CMD_PASSED);

        if (abort == USBD_BOT_ABORT_DATA_IN_CSW_PHASE_ERROR)
        {
            stm32l0_usbd_msc_device.bot_state = USBD_BOT_STATE_RECOVERY_RESET;
        }
        else
        {
            stm32l0_usbd_msc_device.bot_state = USBD_BOT_STATE_HALT_DATA_IN;
        }
        break;

    case USBD_BOT_ABORT_DATA_OUT_CSW_CMD_PASSED:
    case USBD_BOT_ABORT_DATA_OUT_CSW_CMD_FAILED:
    case USBD_BOT_ABORT_DATA_OUT_CSW_PHASE_ERROR:
        USBD_LL_StallEP(pdev, STM32L0_USBD_MSC_DATA_OUT_EP_ADDR);

        stm32l0_usbd_msc_device.bot_csw.bStatus = (abort - USBD_BOT_ABORT_DATA_IN_CSW_CMD_PASSED);

        if (abort == USBD_BOT_ABORT_DATA_OUT_CSW_PHASE_ERROR)
        {
            stm32l0_usbd_msc_device.bot_state = USBD_BOT_STATE_RECOVERY_RESET;
        }
        else
        {
            stm32l0_usbd_msc_device.bot_state = USBD_BOT_STATE_HALT_DATA_OUT;
        }
        break;

    default:
        break;
    }

    NVIC_EnableIRQ(USB_IRQn);
}

static void MSC_BOT_Stall(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{      
    NVIC_DisableIRQ(USB_IRQn);

    USBD_LL_StallEP(pdev, ep_addr);

    NVIC_EnableIRQ(USB_IRQn);
}

static void MSC_BOT_ClearHalt(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{      
    uint8_t bot_state_previous = stm32l0_usbd_msc_device.bot_state;;
    
    NVIC_DisableIRQ(USB_IRQn);

    USBD_LL_FlushEP(pdev, ep_addr);
    
#if 0
    USBD_LL_CloseEP(pdev, ep_addr);
    USBD_LL_OpenEP(pdev, ep_addr, USBD_EP_TYPE_BULK, STM32L0_USBD_MSC_DATA_MAX_PACKET_SIZE);
#endif
    
    if (ep_addr == STM32L0_USBD_MSC_DATA_IN_EP_ADDR)
    {
        if (stm32l0_usbd_msc_device.bot_state == USBD_BOT_STATE_RECOVERY_DATA_IN)
        {
            stm32l0_usbd_msc_device.bot_state = USBD_BOT_STATE_RECOVERY_DATA_OUT;
        }

        if (stm32l0_usbd_msc_device.bot_state == USBD_BOT_STATE_HALT_DATA_IN)
        {
            stm32l0_usbd_msc_device.bot_state = USBD_BOT_STATE_IDLE;
        }
    }

    if (ep_addr == STM32L0_USBD_MSC_DATA_OUT_EP_ADDR)
    {
        if (stm32l0_usbd_msc_device.bot_state == USBD_BOT_STATE_RECOVERY_DATA_OUT)
        {
            stm32l0_usbd_msc_device.bot_state = USBD_BOT_STATE_IDLE;
        }

        if (stm32l0_usbd_msc_device.bot_state == USBD_BOT_STATE_HALT_DATA_OUT)
        {
            stm32l0_usbd_msc_device.bot_state = USBD_BOT_STATE_IDLE;
        }
    }

    if ((stm32l0_usbd_msc_device.bot_state != bot_state_previous) && (stm32l0_usbd_msc_device.bot_state == USBD_BOT_STATE_IDLE))
    {
        USBD_LL_Transmit(pdev, STM32L0_USBD_MSC_DATA_IN_EP_ADDR, (uint8_t *)&stm32l0_usbd_msc_device.bot_csw, USBD_BOT_CSW_LENGTH);

        /* Prepare EP to Receive next Cmd */
        USBD_LL_PrepareReceive(pdev, STM32L0_USBD_MSC_DATA_OUT_EP_ADDR, (uint8_t *)&stm32l0_usbd_msc_device.bot_cbw, USBD_BOT_CBW_LENGTH);  
    }
        
    NVIC_EnableIRQ(USB_IRQn);
}

static void MSC_BOT_SendCSW(USBD_HandleTypeDef *pdev, uint8_t status)
{
    NVIC_DisableIRQ(USB_IRQn);

    stm32l0_usbd_msc_device.bot_csw.bStatus = status;

    stm32l0_usbd_msc_device.bot_state = USBD_BOT_STATE_IDLE;

    USBD_LL_Transmit(pdev, STM32L0_USBD_MSC_DATA_IN_EP_ADDR, (uint8_t *)&stm32l0_usbd_msc_device.bot_csw, USBD_BOT_CSW_LENGTH);
  
    /* Prepare EP to Receive next Cmd */
    USBD_LL_PrepareReceive(pdev, STM32L0_USBD_MSC_DATA_OUT_EP_ADDR, (uint8_t *)&stm32l0_usbd_msc_device.bot_cbw, USBD_BOT_CBW_LENGTH);  

    NVIC_EnableIRQ(USB_IRQn);
}

static void MSC_BOT_Transmit(USBD_HandleTypeDef *pdev, const uint8_t *data, uint32_t length)
{
    uint32_t count;
    
    NVIC_DisableIRQ(USB_IRQn);

    count = MIN(stm32l0_usbd_msc_device.bot_csw.dDataResidue, length);

    stm32l0_usbd_msc_device.bot_csw.dDataResidue -= count;

    if (stm32l0_usbd_msc_device.bot_csw.dDataResidue != 0)
    {
	/* case (5) Hi > Di */
	stm32l0_usbd_msc_device.bot_state = USBD_BOT_STATE_DATA_IN_LAST_STALL;
	// stm32l0_usbd_msc_device.bot_state = USBD_BOT_STATE_DATA_IN_LAST;
	
	stm32l0_usbd_msc_device.bot_csw.bStatus = USBD_BOT_CSW_CMD_PASSED;
    }
    else
    {
	/* case (6) Hi == Di */
	stm32l0_usbd_msc_device.bot_state = USBD_BOT_STATE_DATA_IN_LAST;
    }

    USBD_LL_Transmit(pdev, STM32L0_USBD_MSC_DATA_IN_EP_ADDR, (uint8_t*)data, length);

    NVIC_EnableIRQ(USB_IRQn);
}

static void MSC_BOT_SetupTransmit(USBD_HandleTypeDef *pdev, uint32_t length)
{
    NVIC_DisableIRQ(USB_IRQn);

    stm32l0_usbd_msc_device.bot_state = USBD_BOT_STATE_DATA_IN;  
    stm32l0_usbd_msc_device.bot_index = 0;
    stm32l0_usbd_msc_device.bot_count = 1;

    stm32l0_usbd_msc_device.bot_tx_busy = true;
    stm32l0_usbd_msc_device.bot_tx_length = length;

    USBD_LL_Transmit(pdev, STM32L0_USBD_MSC_DATA_IN_EP_ADDR, (uint8_t *)&stm32l0_usbd_msc_device.scsi_data[stm32l0_usbd_msc_device.bot_index][0], STM32L0_USBD_MSC_DATA_BLOCK_SIZE);

    NVIC_EnableIRQ(USB_IRQn);
}

static void MSC_BOT_ContinueTransmit(USBD_HandleTypeDef *pdev)
{
    NVIC_DisableIRQ(USB_IRQn);

    stm32l0_usbd_msc_device.bot_count++;

    if (!stm32l0_usbd_msc_device.bot_tx_busy)
    {
	stm32l0_usbd_msc_device.bot_tx_busy = true;
		
	USBD_LL_Transmit(pdev, STM32L0_USBD_MSC_DATA_IN_EP_ADDR, (uint8_t *)&stm32l0_usbd_msc_device.scsi_data[stm32l0_usbd_msc_device.bot_index][0], STM32L0_USBD_MSC_DATA_BLOCK_SIZE);
    }
    
    NVIC_EnableIRQ(USB_IRQn);
}

static void MSC_BOT_FinishTransmit(USBD_HandleTypeDef *pdev, uint8_t status)
{
    MSC_BOT_SendCSW(pdev, status);
}

static void MSC_BOT_SetupReceive(USBD_HandleTypeDef *pdev, uint32_t length)
{
    NVIC_DisableIRQ(USB_IRQn);

    stm32l0_usbd_msc_device.bot_state = USBD_BOT_STATE_DATA_OUT;  
    stm32l0_usbd_msc_device.bot_index = 0;
    stm32l0_usbd_msc_device.bot_count = 1;

    stm32l0_usbd_msc_device.bot_rx_busy = true;
    stm32l0_usbd_msc_device.bot_rx_length = length;

    USBD_LL_PrepareReceive(pdev, STM32L0_USBD_MSC_DATA_OUT_EP_ADDR, (uint8_t *)&stm32l0_usbd_msc_device.scsi_data[stm32l0_usbd_msc_device.bot_index][0], STM32L0_USBD_MSC_DATA_BLOCK_SIZE);

    NVIC_EnableIRQ(USB_IRQn);
}

static void MSC_BOT_ContinueReceive(USBD_HandleTypeDef *pdev)
{
    NVIC_DisableIRQ(USB_IRQn);

    stm32l0_usbd_msc_device.bot_count--;

    if (!stm32l0_usbd_msc_device.bot_rx_busy)
    {
	if (stm32l0_usbd_msc_device.bot_rx_length >= 1)
	{
	    stm32l0_usbd_msc_device.bot_count++;
	
	    stm32l0_usbd_msc_device.bot_rx_busy = true;
	    
	    USBD_LL_PrepareReceive(pdev, STM32L0_USBD_MSC_DATA_OUT_EP_ADDR, (uint8_t *)&stm32l0_usbd_msc_device.scsi_data[stm32l0_usbd_msc_device.bot_index][0], STM32L0_USBD_MSC_DATA_BLOCK_SIZE);
	}
    }

    NVIC_EnableIRQ(USB_IRQn);
}

static void MSC_BOT_FinishReceive(USBD_HandleTypeDef *pdev, uint8_t status)
{
    armv6m_atomic_decb(&stm32l0_usbd_msc_device.bot_count);

    MSC_BOT_SendCSW(pdev, status);
}

/***********************************************************************************************************************/

static void USBD_MSC_Process(USBD_HandleTypeDef *pdev)
{
    stm32l0_usbd_msc_routine_t routine;
    uint32_t queue_tail;
    
    queue_tail = stm32l0_usbd_msc_device.queue_tail;

    while (stm32l0_usbd_msc_device.queue_head != queue_tail)
    {
        routine = stm32l0_usbd_msc_device.queue_data[queue_tail];

        queue_tail++;

        if (queue_tail == USBD_MSC_QUEUE_ENTRIES)
        {
            queue_tail = 0;
        }

        stm32l0_usbd_msc_device.queue_tail = queue_tail;

        (*routine)(pdev);
    }
}

static bool USBD_MSC_Submit(stm32l0_usbd_msc_routine_t routine)
{
    uint32_t queue_head, queue_head_next;

    queue_head = stm32l0_usbd_msc_device.queue_head;

    queue_head_next = queue_head +1;

    if (queue_head_next == USBD_MSC_QUEUE_ENTRIES)
    {
        queue_head_next = 0;
    }

    if (queue_head_next == stm32l0_usbd_msc_device.queue_tail)
    {
        return false;
    }

    stm32l0_usbd_msc_device.queue_data[queue_head] = routine;

    stm32l0_usbd_msc_device.queue_head = queue_head_next;

    if (stm32l0_usbd_msc_device.queue_lock == 0)
    {
        armv6m_work_submit(&stm32l0_usbd_msc_device.queue_work);
    }

    return true;
}

void USBD_MSC_Notify(uint8_t lun, int acquire)
{
    if (acquire)
    {
        armv6m_atomic_add(&stm32l0_usbd_msc_device.queue_lock, 1);
    }
    else
    {
        if (armv6m_atomic_sub(&stm32l0_usbd_msc_device.queue_lock, 1) == 1)
        {
            if (stm32l0_usbd_msc_device.queue_head != stm32l0_usbd_msc_device.queue_tail)
            {
                armv6m_work_submit(&stm32l0_usbd_msc_device.queue_work);
            }
        }
    }
}

uint8_t USBD_MSC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    if (!stm32l0_usbd_msc_device.queue_work.callback.routine)
    {
        armv6m_work_create(&stm32l0_usbd_msc_device.queue_work, (armv6m_core_routine_t)USBD_MSC_Process, pdev);
    }

    stm32l0_usbd_msc_device.storage = &dosfs_storage_interface;
    stm32l0_usbd_msc_device.max_lun = 0;
    stm32l0_usbd_msc_device.feature = 0;
    
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
    uint8_t ep_addr;
    
    switch (req->bmRequest & USB_REQ_TYPE_MASK) {

        /* Class request */
    case USB_REQ_TYPE_CLASS:
        switch (req->bRequest) {
        case USBD_BOT_GET_MAX_LUN:
	    if((req->wValue  == 0) && 
               (req->wLength == 1) &&
               ((req->bmRequest & 0x80) == 0x80))
            {
                USBD_CtlSendData(pdev, (uint8_t*)&stm32l0_usbd_msc_device.max_lun, 1);
            }
            else
            {
                USBD_CtlError(pdev, req);

                return USBD_FAIL; 
            }
            break;
      
        case USBD_BOT_RESET:
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
            USBD_CtlSendData(pdev, (uint8_t *)&stm32l0_usbd_msc_device.interface, 1);
            break;
      
        case USB_REQ_SET_INTERFACE:
            stm32l0_usbd_msc_device.interface = (uint8_t)(req->wValue);
            break;
            
        case USB_REQ_CLEAR_FEATURE:  
            if (req->wValue == USB_FEATURE_EP_HALT)
            {
                ep_addr = LOBYTE(req->wIndex);

                if ((ep_addr == STM32L0_USBD_MSC_DATA_IN_EP_ADDR) || (ep_addr == STM32L0_USBD_MSC_DATA_OUT_EP_ADDR))
                {
                    stm32l0_usbd_msc_device.feature = ep_addr;
            
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
    if (stm32l0_usbd_msc_device.bot_tx_length)
    {
	stm32l0_usbd_msc_device.bot_csw.dDataResidue -= STM32L0_USBD_MSC_DATA_BLOCK_SIZE;

	stm32l0_usbd_msc_device.bot_tx_busy = false;
	stm32l0_usbd_msc_device.bot_tx_length -= 1;
	
	stm32l0_usbd_msc_device.bot_count -= 1;
	stm32l0_usbd_msc_device.bot_index ^= 1;
	
	if (!USBD_MSC_Submit(MSC_BOT_DataTransmit))
	{
	    return USBD_FAIL;
	}

	if (stm32l0_usbd_msc_device.bot_tx_length >= 1)
	{
	    if (stm32l0_usbd_msc_device.bot_count != 0)
	    {
		stm32l0_usbd_msc_device.bot_tx_busy = true;
		
		USBD_LL_Transmit(pdev, STM32L0_USBD_MSC_DATA_IN_EP_ADDR, (uint8_t *)&stm32l0_usbd_msc_device.scsi_data[stm32l0_usbd_msc_device.bot_index][0], STM32L0_USBD_MSC_DATA_BLOCK_SIZE);
	    }
	}
    }
    else
    {
	if (!USBD_MSC_Submit(MSC_BOT_DataIn))
	{
	    return USBD_FAIL;
	}
    }

    return USBD_OK;
}

uint8_t USBD_MSC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    if (stm32l0_usbd_msc_device.bot_rx_length)
    {
	stm32l0_usbd_msc_device.bot_rx_busy = false;
	stm32l0_usbd_msc_device.bot_rx_length -= 1;

	stm32l0_usbd_msc_device.bot_index ^= 1;
	
	if (!USBD_MSC_Submit(MSC_BOT_DataReceive))
	{
	    return USBD_FAIL;
	}

	if (stm32l0_usbd_msc_device.bot_rx_length >= 1)
	{
	    if (stm32l0_usbd_msc_device.bot_count != 2)
	    {
		stm32l0_usbd_msc_device.bot_count++;
		
		stm32l0_usbd_msc_device.bot_rx_busy = true;

		USBD_LL_PrepareReceive(pdev, STM32L0_USBD_MSC_DATA_OUT_EP_ADDR, (uint8_t *)&stm32l0_usbd_msc_device.scsi_data[stm32l0_usbd_msc_device.bot_index][0], STM32L0_USBD_MSC_DATA_BLOCK_SIZE);
	    }
	}
    }
    else
    {
	if (!USBD_MSC_Submit(MSC_BOT_DataCBW))
	{
	    return USBD_FAIL;
	}
    }
    
    return USBD_OK;
}
