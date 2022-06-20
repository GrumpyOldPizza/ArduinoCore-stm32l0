/**
  ******************************************************************************
  * @file    usbd_conf.c
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    26-February-2016
  * @brief   This file implements the USB Device library callbacks and MSP
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx.h"
#include "stm32l0xx_hal.h"
#include "usbd_def.h"
#include "usbd_core.h"
#include "usbd_desc.h"

#include "armv6m.h"
#include "stm32l0_system.h"
#include "stm32l0_exti.h"
#include "stm32l0_lptim.h"
#include "stm32l0_gpio.h"

void USBD_Detach();

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

#define USBD_STATE_NONE            0
#define USBD_STATE_STARTED         1
#define USBD_STATE_CONNECTED       2
//#define USBD_STATE_SUSPENDED       4

static PCD_HandleTypeDef hpcd_USB;

static uint16_t usbd_pin_vbus;
static uint32_t usbd_mask_vbus;
static bool usbd_wakeup = false;
static bool usbd_enabled = false;
static bool usbd_attached = false;
static uint8_t usbd_state = USBD_STATE_NONE; 
static void (*usbd_connect_callback)(void) = NULL;
static void (*usbd_disconnect_callback)(void) = NULL;
static void (*usbd_suspend_callback)(void) = NULL;
static void (*usbd_resume_callback)(void) = NULL;

/* Private functions ---------------------------------------------------------*/

uint16_t USBD_VendorID;
uint16_t USBD_ProductID;
const uint8_t * USBD_ManufacturerString = NULL;
const uint8_t * USBD_ProductString = NULL;
const uint8_t * USBD_SuffixString = NULL;

static void (*USBD_ClassInitialize)(struct _USBD_HandleTypeDef *pdev) = NULL;

static void (*USBD_IRQHandler)(PCD_HandleTypeDef *hpcd) = NULL;

static USBD_HandleTypeDef USBD_Device;

static stm32l0_lptim_timeout_t USBD_VBUSTimeout;

static void USBD_VBUSTimeoutIrq(void)
{
    unsigned int state;

    state = stm32l0_gpio_pin_read(usbd_pin_vbus);

    if (state)
    {
        usbd_attached = true;

        if (usbd_enabled)
        {
            if (usbd_state == USBD_STATE_NONE)
            {
                usbd_state = USBD_STATE_STARTED;
                
                stm32l0_system_lock(STM32L0_SYSTEM_LOCK_RUN);

                USBD_Init(&USBD_Device, &CDC_MSC_Desc);
                
                (*USBD_ClassInitialize)(&USBD_Device);
                
                USBD_Start(&USBD_Device);

                stm32l0_system_hook(USB_IRQHandler);
            }
        }
    }
}

static void USBD_VBUSChangedIrq(void)
{
    unsigned int state;

    state = stm32l0_gpio_pin_read(usbd_pin_vbus);

    if (state)
    {
        if (!usbd_attached)
        {
            stm32l0_lptim_timeout_start(&USBD_VBUSTimeout, stm32l0_lptim_millis_to_ticks(40), (stm32l0_lptim_callback_t)USBD_VBUSTimeoutIrq); /* 40ms */
        }
    }
    else
    {
        if (!usbd_attached)
        {
            stm32l0_lptim_timeout_stop(&USBD_VBUSTimeout);
        }
        else
        {
            usbd_attached = false;

            if (usbd_state >= USBD_STATE_STARTED)
            {
                stm32l0_system_hook(NULL);

                NVIC_DisableIRQ(USB_IRQn);
                
                USBD_DeInit(&USBD_Device);

                if (usbd_state != USBD_STATE_SUSPENDED)
                {
                    stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_RUN);
                }

                if (usbd_state >= USBD_STATE_CONNECTED)
                {
                    if (usbd_disconnect_callback)
                    {
                        usbd_state = USBD_STATE_NONE;
                        
                        (*usbd_disconnect_callback)();
                    }
                }
                
                usbd_state = USBD_STATE_NONE;
            }
        }
    }
}

bool USBD_Initialize(uint16_t vid, uint16_t pid, const uint8_t *manufacturer, const uint8_t *product, void(*initialize)(struct _USBD_HandleTypeDef *),
                     unsigned int pin_vbus, bool pin_vbus_needs_pulldown, unsigned int priority,
                     void(*connect_callback)(void), void(*disconnect_callback)(void), void(*suspend_callback)(void), void(*resume_callback)(void))
{
    if (stm32l0_system_pclk1() < 10000000)
    {
        return false;
    }

    USBD_IRQHandler = HAL_PCD_IRQHandler;

    USBD_VendorID = vid;
    USBD_ProductID = pid;
    USBD_ManufacturerString = manufacturer;
    USBD_ProductString = product;
    USBD_ClassInitialize = initialize;
    
    /* Set USB Interrupt priority */
    NVIC_SetPriority(USB_IRQn, priority);

    usbd_connect_callback = connect_callback;
    usbd_disconnect_callback = disconnect_callback;
    usbd_suspend_callback = suspend_callback;
    usbd_resume_callback = resume_callback;
    
    /* Configure USB FS GPIOs */
    usbd_pin_vbus = pin_vbus;
    if (pin_vbus != STM32L0_GPIO_PIN_NONE) {
        usbd_mask_vbus = 1 << ((pin_vbus & STM32L0_GPIO_PIN_INDEX_MASK) >> STM32L0_GPIO_PIN_INDEX_SHIFT);

        stm32l0_lptim_timeout_create(&USBD_VBUSTimeout);

        uint32_t mode = (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT);
        if (pin_vbus_needs_pulldown)
            mode |= STM32L0_GPIO_PUPD_PULLDOWN;
        stm32l0_gpio_pin_configure(usbd_pin_vbus, mode);

        if (stm32l0_gpio_pin_read(usbd_pin_vbus))
        {
            stm32l0_lptim_timeout_start(&USBD_VBUSTimeout, stm32l0_lptim_millis_to_ticks(40), (stm32l0_lptim_callback_t)USBD_VBUSTimeoutIrq); /* 40ms */
        }
        
        stm32l0_exti_attach(usbd_pin_vbus,
                            (STM32L0_EXTI_CONTROL_NOWAKEUP | STM32L0_EXTI_CONTROL_PRIORITY_LOW | STM32L0_EXTI_CONTROL_EDGE_RISING | STM32L0_EXTI_CONTROL_EDGE_FALLING),
                            (stm32l0_exti_callback_t)USBD_VBUSChangedIrq, NULL);
    } else {
        // No VBUS pin, assume VBUS is always present
        usbd_attached = true;
        usbd_mask_vbus = 0;
    }
        
    
    return true;
}

void USBD_Teardown()
{
    USBD_Detach();

    if (usbd_pin_vbus != STM32L0_GPIO_PIN_NONE) {
        stm32l0_exti_detach(usbd_pin_vbus);

        stm32l0_lptim_timeout_stop(&USBD_VBUSTimeout);

        stm32l0_gpio_pin_configure(usbd_pin_vbus, STM32L0_GPIO_MODE_ANALOG);
    }

    usbd_wakeup = false;
    usbd_enabled = false;
    usbd_attached = false;
    usbd_state = USBD_STATE_NONE;
    usbd_connect_callback = NULL;
    usbd_disconnect_callback = NULL;
    usbd_suspend_callback = NULL;
    usbd_resume_callback = NULL;

    USBD_IRQHandler = NULL;
}

void USBD_Attach(void)
{
    if (!usbd_enabled)
    {
        usbd_enabled = true;

        stm32l0_exti_block(usbd_mask_vbus);

        if (usbd_attached)
        {
            usbd_state = USBD_STATE_STARTED;
                
            stm32l0_system_lock(STM32L0_SYSTEM_LOCK_RUN);
            
            USBD_Init(&USBD_Device, &CDC_MSC_Desc);
            
            (*USBD_ClassInitialize)(&USBD_Device);
            
            USBD_Start(&USBD_Device);
        }

        stm32l0_exti_unblock(usbd_mask_vbus);
    }
}

void USBD_Detach(void)
{
    if (usbd_enabled)
    {
        usbd_enabled = false;

        stm32l0_exti_block(usbd_mask_vbus);

        if (usbd_state >= USBD_STATE_STARTED)
        {
            NVIC_DisableIRQ(USB_IRQn);
            
            USBD_DeInit(&USBD_Device);

            if (usbd_state != USBD_STATE_SUSPENDED)
            {
                stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_RUN);
            }
            
            if (usbd_state >= USBD_STATE_CONNECTED)
            {
                if (usbd_disconnect_callback)
                {
                    usbd_state = USBD_STATE_NONE;
                    
                    (*usbd_disconnect_callback)();
                }
            }
                
            usbd_state = USBD_STATE_NONE;
        }

        stm32l0_exti_unblock(usbd_mask_vbus);
        
    }
}

void USBD_Wakeup(void)
{
    if (usbd_state == USBD_STATE_SUSPENDED)
    {
        stm32l0_exti_block(usbd_mask_vbus);

        NVIC_DisableIRQ(USB_IRQn);

        if (USBD_Device.dev_remote_wakeup == 1)
        {
            armv6m_core_udelay(5000);
            
            HAL_PCD_ActivateRemoteWakeup(&hpcd_USB);
            
            armv6m_core_udelay(5000);
            
            HAL_PCD_DeActivateRemoteWakeup(&hpcd_USB);
            
            USBD_Device.dev_remote_wakeup = 0;
        }
        
        NVIC_EnableIRQ(USB_IRQn);

        stm32l0_exti_unblock(usbd_mask_vbus);
    }
}

bool USBD_Attached(void)
{
    return usbd_attached;
}

bool USBD_Connected(void)
{
    return (usbd_state >= USBD_STATE_CONNECTED);
}

bool USBD_Configured(void)
{
    if (usbd_state >= USBD_STATE_CONNECTED)
    {
        return ((USBD_Device.dev_state == USBD_STATE_CONFIGURED) || ((USBD_Device.dev_state == USBD_STATE_SUSPENDED) && (USBD_Device.dev_old_state == USBD_STATE_CONFIGURED)));
    }
    else
    {
        return false;
    }
}

bool USBD_Suspended(void)
{
    return (usbd_state == USBD_STATE_SUSPENDED);
}

void USBD_SetupVBUS(bool wakeup)
{
    if (usbd_wakeup != wakeup)
    {
        usbd_wakeup = wakeup;

        if (usbd_pin_vbus != STM32L0_GPIO_PIN_NONE) {
            stm32l0_exti_detach(usbd_pin_vbus);
            
            stm32l0_gpio_pin_configure(usbd_pin_vbus,
                                       (usbd_wakeup ? STM32L0_GPIO_PARK_NONE : STM32L0_GPIO_PARK_HIZ) |
                                       (STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));

            stm32l0_exti_attach(usbd_pin_vbus,
                                ((usbd_wakeup ? 0 : STM32L0_EXTI_CONTROL_NOWAKEUP) |
                                 (STM32L0_EXTI_CONTROL_PRIORITY_LOW | STM32L0_EXTI_CONTROL_EDGE_RISING | STM32L0_EXTI_CONTROL_EDGE_FALLING)),
                                (stm32l0_exti_callback_t)USBD_VBUSChangedIrq, NULL);
        }
    }
}

/*******************************************************************************
                       PCD BSP Routines
*******************************************************************************/

void USB_IRQHandler(void)
{
    if (USBD_IRQHandler) { (*USBD_IRQHandler)(&hpcd_USB); }
}

/**
  * @brief  Initializes the PCD MSP.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
    stm32l0_system_reference(STM32L0_SYSTEM_REFERENCE_USB);

    stm32l0_system_clk48_enable();
    
    stm32l0_system_periph_enable(STM32L0_SYSTEM_PERIPH_USB);

    armv6m_atomic_or(&EXTI->IMR, EXTI_IMR_IM18);
    
    NVIC_EnableIRQ(USB_IRQn);
}

/**
  * @brief  De-Initializes the PCD MSP.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{  
    NVIC_DisableIRQ(USB_IRQn);
    
    armv6m_atomic_and(&EXTI->IMR, ~EXTI_IMR_IM18);

    stm32l0_system_periph_disable(STM32L0_SYSTEM_PERIPH_USB);
    
    stm32l0_system_unreference(STM32L0_SYSTEM_REFERENCE_USB);

    stm32l0_system_clk48_disable();
}


/*******************************************************************************
                       LL Driver Callbacks (PCD -> USB Device Library)
*******************************************************************************/

/**
  * @brief  SetupStage callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SetupStage(hpcd->pData, (uint8_t *)hpcd->Setup);
}

/**
  * @brief  DataOut Stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_DataOutStage(hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

/**
  * @brief  DataIn Stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_DataInStage(hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

/**
  * @brief  SOF callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SOF(hpcd->pData);
}

/**
  * @brief  Reset callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{   
    /* Reset Device */
    USBD_LL_Reset(hpcd->pData);
    
    /* Set USB Current Speed */ 
    USBD_LL_SetSpeed(hpcd->pData, USBD_SPEED_FULL);
}

/**
 * @brief  Suspend callback.
 * @param  hpcd: PCD handle
 * @retval None
 */
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{ 
    if (usbd_state == USBD_STATE_CONNECTED)
    {
        usbd_state = USBD_STATE_SUSPENDED;

        stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_RUN);
        
        if (usbd_suspend_callback)
        {
            (*usbd_suspend_callback)();
        }
    }
    
    USBD_LL_Suspend(hpcd->pData);
}

/**
 * @brief  Resume callback.
 * @param  hpcd: PCD handle
 * @retval None
 */
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_Resume(hpcd->pData);

    if (usbd_state == USBD_STATE_SUSPENDED)
    {
        usbd_state = USBD_STATE_CONNECTED;

        stm32l0_system_lock(STM32L0_SYSTEM_LOCK_RUN);
        
        if (usbd_resume_callback)
        {
            (*usbd_resume_callback)();
        }
    }
    
    if (usbd_state == USBD_STATE_STARTED)
    {
        usbd_state = USBD_STATE_CONNECTED;

        if (usbd_connect_callback)
        {
            (*usbd_connect_callback)();
        }
    }
}

/**
  * @brief  ISOOUTIncomplete callback.
  * @param  hpcd: PCD handle 
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_IsoOUTIncomplete(hpcd->pData, epnum);
}

/**
  * @brief  ISOINIncomplete callback.
  * @param  hpcd: PCD handle 
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_IsoINIncomplete(hpcd->pData, epnum);
}

/**
  * @brief  ConnectCallback callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_DevConnected(hpcd->pData);
}

/**
  * @brief  Disconnect callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_DevDisconnected(hpcd->pData);
}



/*******************************************************************************
                       LL Driver Interface (USB Device Library --> PCD)
*******************************************************************************/

/**
  * @brief  Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
    /* Set LL Driver parameters */
    hpcd_USB.Instance = USB;
    hpcd_USB.Init.dev_endpoints = 8;
    hpcd_USB.Init.speed = PCD_SPEED_FULL;
    hpcd_USB.Init.ep0_mps = DEP0CTL_MPS_64;
    hpcd_USB.Init.phy_itface = PCD_PHY_EMBEDDED;
    hpcd_USB.Init.low_power_enable = 1;
    hpcd_USB.Init.Sof_enable = 0;
#if (USBD_LPM_ENABLE == 1)
    hpcd_USB.Init.lpm_enable = 0;
#endif
#if (USBD_LPM_ENABLE == 1)
    hpcd_USB.Init.battery_charging_enable = 0;
#endif
    /* Link The driver to the stack */
    hpcd_USB.pData = pdev;
    pdev->pData = &hpcd_USB;
    /* Initialize LL Driver */
    HAL_PCD_Init(&hpcd_USB);
    
    /* First offset needs to be n * 8, where n is the number of endpoints.
     */
    HAL_PCDEx_PMAConfig(&hpcd_USB, 0x00, PCD_SNG_BUF, 0x00000040); /*  64 bytes EP0/control out  */
    HAL_PCDEx_PMAConfig(&hpcd_USB, 0x80, PCD_SNG_BUF, 0x00000080); /*  64 bytes EP0/control in   */
    HAL_PCDEx_PMAConfig(&hpcd_USB, 0x81, PCD_SNG_BUF, 0x000000c0); /*  16 bytes EP1/CDC/CTRL in  */
    HAL_PCDEx_PMAConfig(&hpcd_USB, 0x82, PCD_SNG_BUF, 0x000000d0); /*  64 bytes EP2/CDC/DATA in  */
    HAL_PCDEx_PMAConfig(&hpcd_USB, 0x02, PCD_SNG_BUF, 0x00000110); /*  64 bytes EP2/CDC/DATA out */
    HAL_PCDEx_PMAConfig(&hpcd_USB, 0x83, PCD_SNG_BUF, 0x00000150); /*  64 bytes EP3/MSC in       */ 
    HAL_PCDEx_PMAConfig(&hpcd_USB, 0x03, PCD_SNG_BUF, 0x00000190); /*  64 bytes EP3/MSC out      */ 

    HAL_PCDEx_PMAConfig(&hpcd_USB, 0x84, PCD_DBL_BUF, 0x02400200); /*  64 bytes EP3/MSC in       */ 
    HAL_PCDEx_PMAConfig(&hpcd_USB, 0x05, PCD_DBL_BUF, 0x02800240); /*  64 bytes EP3/MSC out      */ 

    return USBD_OK;
}

/**
  * @brief  De-Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
    HAL_PCD_DeInit(pdev->pData);

    return USBD_OK;
}

/**
  * @brief  Starts the Low Level portion of the Device driver. 
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
    HAL_PCD_Start(pdev->pData);

    return USBD_OK;
}

/**
  * @brief  Stops the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
    HAL_PCD_Stop(pdev->pData);

    return USBD_OK;
}

/**
  * @brief  Opens an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  ep_type: Endpoint Type
  * @param  ep_mps: Endpoint Max Packet Size
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev,
                                  uint8_t ep_addr,
                                  uint8_t ep_type,
                                  uint16_t ep_mps)
{
    HAL_PCD_EP_Open(pdev->pData,
                    ep_addr,
                    ep_mps,
                    ep_type);
    
    return USBD_OK;
}

/**
  * @brief  Closes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_PCD_EP_Close(pdev->pData, ep_addr);

    return USBD_OK;
}

/**
  * @brief  Flushes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_PCD_EP_Flush(pdev->pData, ep_addr);

    return USBD_OK;
}

/**
  * @brief  Sets a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_PCD_EP_SetStall(pdev->pData, ep_addr);

    return USBD_OK;
}

/**
  * @brief  Clears a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);

    return USBD_OK; 
}

/**
  * @brief  Returns Stall condition.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Stall (1: Yes, 0: No)
  */
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  PCD_HandleTypeDef *hpcd = pdev->pData;
  
  if((ep_addr & 0x80) == 0x80)
  {
      return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
  }
  else
  {
      return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
  }
}

/**
  * @brief  Assigns a USB address to the device.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
    HAL_PCD_SetAddress(pdev->pData, dev_addr);

    return USBD_OK; 
}

/**
  * @brief  Transmits data over an endpoint.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf: Pointer to data to be sent
  * @param  size: Data size    
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, 
                                    uint8_t ep_addr,
                                    const uint8_t *pbuf,
                                    uint16_t size)
{
    HAL_PCD_EP_Transmit(pdev->pData, ep_addr, (uint8_t*)pbuf, size);

    return USBD_OK;
}

/**
  * @brief  Prepares an endpoint for reception.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf: Pointer to data to be received
  * @param  size: Data size
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, 
                                          uint8_t ep_addr,
                                          uint8_t *pbuf,
                                          uint16_t size)
{
    HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);

    return USBD_OK;
}

/**
  * @brief  Returns the last transfered packet size.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Recived Data Size
  */
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return HAL_PCD_EP_GetRxCount(pdev->pData, ep_addr);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
