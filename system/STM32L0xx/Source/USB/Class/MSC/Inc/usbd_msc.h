/**
  ******************************************************************************
  * @file    usbd_msc.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   Header for the usbd_msc.c file
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
#ifndef __USBD_MSC_H
#define __USBD_MSC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_msc_bot.h"
#include  "usbd_msc_scsi.h"
#include  "usbd_ioreq.h"

/** @addtogroup USBD_MSC_BOT
  * @{
  */
  
/** @defgroup USBD_MSC
  * @brief This file is the Header file for usbd_msc.c
  * @{
  */ 


/** @defgroup USBD_BOT_Exported_Defines
  * @{
  */ 
#define MSC_MAX_FS_PACKET            0x40

#define BOT_GET_MAX_LUN              0xFE
#define BOT_RESET                    0xFF

#define MSC_EPIN_ADDR                0x83
#define MSC_EPOUT_ADDR               0x03 
   

/**
  * @}
  */ 

/** @defgroup USB_CORE_Exported_Types
  * @{
  */ 
typedef struct _USBD_STORAGE
{
  bool (* Init) (uint8_t **p_cache_data, const uint8_t **p_inquiry_data);
  bool (* DeInit) (void);
  bool (* IsReady) (void);
  bool (* GetCapacity) (uint32_t *pblock_count, uint32_t *p_block_size);
  bool (* GetWriteProtected) (bool *p_write_protected);
  bool (* GetChanged) (bool *p_changed);
  bool (* StartStopUnit) (bool start, bool loej);
  bool (* PreventAllowMediumRemoval) (bool prevent);
  bool (* Acquire) (void);
  void (* Release) (void);
  bool (* Read) (uint8_t *data, uint32_t blk_addr, uint32_t blk_len, bool release);
  bool (* Write) (const uint8_t *data, uint32_t blk_addr, uint32_t blk_len, bool release);
}USBD_StorageTypeDef;

   
typedef void (*USBD_MSC_BOT_CallbackTypeDef)(USBD_HandleTypeDef*);
   
typedef struct
{
  uint8_t                  interface;  
  uint8_t                  max_lun;   
  uint16_t                 feature; 
  const USBD_StorageTypeDef *storage;

  volatile uint32_t        queue_head;
  volatile uint32_t        queue_tail;
  USBD_MSC_BOT_CallbackTypeDef queue_data[MSC_QUEUE_ENTRIES];
  
  uint8_t                  bot_state;
  bool                     bot_fault;
  uint8_t                  bot_data[18]; // should be SCSI_DATA_LENGTH_REQUEST_SENSE
  USBD_MSC_BOT_CBWTypeDef  cbw;
  USBD_MSC_BOT_CSWTypeDef  csw;
  
  uint8_t                  scsi_sense_skey;
  uint16_t                 scsi_sense_asc;
  uint32_t                 scsi_sense_address;
  uint32_t                 scsi_blk_count;
  uint32_t                 scsi_blk_size;
  uint32_t                 scsi_blk_address;
  uint32_t                 scsi_blk_length;
  uint8_t                  scsi_blk_fault;
  bool                     scsi_blk_busy;

  uint32_t                 scsi_index;
  uint8_t                  *scsi_data[2];
  const uint8_t            *scsi_inquiry_data;
}
USBD_MSC_BOT_HandleTypeDef; 

/* Structure for MSC process */
extern const USBD_ClassTypeDef  USBD_MSC;
#define USBD_MSC_CLASS    &USBD_MSC

extern USBD_MSC_BOT_HandleTypeDef USBD_MSC_Data;
   
extern void USBD_MSC_Notify(uint8_t lun, int acquire);
extern uint8_t USBD_MSC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
extern uint8_t USBD_MSC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
extern uint8_t USBD_MSC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
extern uint8_t USBD_MSC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
extern uint8_t USBD_MSC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
extern uint8_t USBD_MSC_RegisterStorage(USBD_HandleTypeDef *pdev, const USBD_StorageTypeDef *fops);

/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif  /* __USBD_MSC_H */
/**
  * @}
  */ 
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
