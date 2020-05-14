/**
  ******************************************************************************
  * @file    usbd_msc_scsi.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   Header for the usbd_msc_scsi.c file
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
#ifndef __USBD_MSC_SCSI_H
#define __USBD_MSC_SCSI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_def.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */
  
/** @defgroup USBD_SCSI
  * @brief header file for the storage disk file
  * @{
  */ 

/** @defgroup USBD_SCSI_Exported_Defines
  * @{
  */ 

   
/* SCSI Commands */
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

/** @defgroup USBD_SCSI_Exported_TypesDefinitions
  * @{
  */

/**
  * @}
  */ 

/** @defgroup USBD_SCSI_Exported_Macros
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup USBD_SCSI_Exported_Variables
  * @{
  */ 

/**
  * @}
  */ 
/** @defgroup USBD_SCSI_Exported_FunctionsPrototype
  * @{
  */ 
void SCSI_ProcessStart(USBD_HandleTypeDef *pdev);
void SCSI_ProcessStop(USBD_HandleTypeDef *pdev);
void SCSI_ProcessCommand(USBD_HandleTypeDef *pdev);
void SCSI_ProcessRead(USBD_HandleTypeDef *pdev);
void SCSI_ProcessWrite(USBD_HandleTypeDef *pdev);
   
/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif /* __USBD_MSC_SCSI_H */
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

