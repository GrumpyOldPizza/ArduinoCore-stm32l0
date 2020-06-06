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

#if !defined(_STM32L0_USBD_CLASS_H)
#define _STM32L0_USBD_CLASS_H

#include "stm32l0xx.h"
#include "usbd_ioreq.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t USBD_CDC_Init(USBD_HandleTypeDef *pdev,  uint8_t cfgidx);
extern uint8_t USBD_CDC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
extern uint8_t USBD_CDC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
extern uint8_t USBD_CDC_EP0_RxReady(USBD_HandleTypeDef *pdev);
extern uint8_t USBD_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
extern uint8_t USBD_CDC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
extern uint8_t USBD_CDC_SOF(USBD_HandleTypeDef *pdev);

extern uint8_t USBD_MSC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
extern uint8_t USBD_MSC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
extern uint8_t USBD_MSC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
extern uint8_t USBD_MSC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
extern uint8_t USBD_MSC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);

extern void USBD_CDC_Initialize(USBD_HandleTypeDef *pdev);
extern void USBD_CDC_MSC_Initialize(USBD_HandleTypeDef *pdev);
  
#ifdef __cplusplus
}
#endif

#endif /* _STM32L0_STM32L0_USBD_CLASS_H */
