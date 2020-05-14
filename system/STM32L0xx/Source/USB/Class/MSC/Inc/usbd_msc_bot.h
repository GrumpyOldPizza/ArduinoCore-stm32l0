/**
  ******************************************************************************
  * @file    usbd_msc_bot.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   Header for the usbd_msc_bot.c file
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
#ifndef __USBD_MSC_BOT_H
#define __USBD_MSC_BOT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_core.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */
  
/** @defgroup MSC_BOT
  * @brief This file is the Header file for usbd_msc_bot.c
  * @{
  */ 


/** @defgroup USBD_CORE_Exported_Defines
  * @{
  */ 
#define USBD_BOT_STATE_NONE                0       /* None */
#define USBD_BOT_STATE_IDLE                1       /* Idle */
#define USBD_BOT_STATE_RECOVERY_RESET      2       /* Waiting for reset */
#define USBD_BOT_STATE_RECOVERY_DATA_IN    3       /* Waiting for clear feature halt on data in */
#define USBD_BOT_STATE_RECOVERY_DATA_OUT   4       /* Waiting for clear feature halt on data out */
#define USBD_BOT_STATE_HALT_DATA_IN        5       /* Waiting for clear feature halt on data in */
#define USBD_BOT_STATE_HALT_DATA_OUT       6       /* Waiting for clear feature halt on data out */
#define USBD_BOT_STATE_DATA_IN             7       /* Data In state */
#define USBD_BOT_STATE_DATA_IN_LAST        8       /* Last Data In Last (passed) */
#define USBD_BOT_STATE_DATA_IN_LAST_STALL  9       /* Last Data In Last (failed or phase error) */
#define USBD_BOT_STATE_DATA_OUT            10      /* Data Out state */

#define USBD_BOT_ABORT_CBW                      3
#define USBD_BOT_ABORT_DATA_IN_CSW_CMD_PASSED   4
#define USBD_BOT_ABORT_DATA_IN_CSW_CMD_FAILED   5
#define USBD_BOT_ABORT_DATA_IN_CSW_PHASE_ERROR  6
#define USBD_BOT_ABORT_DATA_OUT_CSW_CMD_PASSED  7
#define USBD_BOT_ABORT_DATA_OUT_CSW_CMD_FAILED  8
#define USBD_BOT_ABORT_DATA_OUT_CSW_PHASE_ERROR 9
   
#define USBD_BOT_CBW_SIGNATURE             0x43425355
#define USBD_BOT_CSW_SIGNATURE             0x53425355
#define USBD_BOT_CBW_LENGTH                31
#define USBD_BOT_CSW_LENGTH                13
#define USBD_BOT_MAX_DATA                  256

/* CSW Status Definitions */
#define USBD_CSW_CMD_PASSED                0x00
#define USBD_CSW_CMD_FAILED                0x01
#define USBD_CSW_PHASE_ERROR               0x02

#define USBD_DIR_IN                        0
#define USBD_DIR_OUT                       1
#define USBD_BOTH_DIR                      2


#define MSC_QUEUE_ENTRIES 8
   
/**
  * @}
  */ 

/** @defgroup MSC_CORE_Private_TypesDefinitions
  * @{
  */ 
   
typedef void (*USBD_MSC_BOT_CallbackTypeDef)(USBD_HandleTypeDef*);
   
typedef struct
{
  uint32_t dSignature;
  uint32_t dTag;
  uint32_t dDataLength;
  uint8_t  bmFlags;
  uint8_t  bLUN;
  uint8_t  bCBLength;
  uint8_t  CB[16];
  uint8_t  ReservedForAlign;
}
USBD_MSC_BOT_CBWTypeDef;


typedef struct
{
  uint32_t dSignature;
  uint32_t dTag;
  uint32_t dDataResidue;
  uint8_t  bStatus;
  uint8_t  ReservedForAlign[3];  
}
USBD_MSC_BOT_CSWTypeDef;

/**
  * @}
  */ 


/** @defgroup USBD_CORE_Exported_Types
  * @{
  */

/**
  * @}
  */ 
/** @defgroup USBD_CORE_Exported_FunctionsPrototypes
  * @{
  */ 
extern void MSC_BOT_Init(USBD_HandleTypeDef *pdev);
extern void MSC_BOT_DeInit(USBD_HandleTypeDef *pdev);
extern void MSC_BOT_DataIn(USBD_HandleTypeDef *pdev);
extern void MSC_BOT_DataOut(USBD_HandleTypeDef *pdev);
extern void MSC_BOT_Reset(USBD_HandleTypeDef *pdev);
extern void MSC_BOT_ClearFeature(USBD_HandleTypeDef *pdev);

extern void MSC_BOT_Open(USBD_HandleTypeDef *pdev);
extern void MSC_BOT_Close(USBD_HandleTypeDef *pdev);
extern void MSC_BOT_Abort(USBD_HandleTypeDef *pdev, uint8_t abort);
extern void MSC_BOT_Reactivate(USBD_HandleTypeDef *pdev, uint8_t epnum);
extern void MSC_BOT_Stall(USBD_HandleTypeDef *pdev, uint8_t epnum);
extern void MSC_BOT_SendCSW(USBD_HandleTypeDef *pdev, uint8_t status);
extern void MSC_BOT_Transmit(USBD_HandleTypeDef *pdev, const uint8_t *data, uint32_t length, bool last);
extern void MSC_BOT_Receive(USBD_HandleTypeDef *pdev, uint8_t *data, uint32_t length);
   
/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif /* __USBD_MSC_BOT_H */
/**
  * @}
  */ 

/**
* @}
*/ 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

