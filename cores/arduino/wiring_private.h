/*
 * Copyright (c) 2017-2018 Thomas Roell.  All rights reserved.
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

#pragma once

#include "stm32l0_adc.h"
#include "stm32l0_aes.h"
#include "stm32l0_comp.h"
#include "stm32l0_dac.h"
#include "stm32l0_dma.h"
#include "stm32l0_eeprom.h"
#include "stm32l0_exti.h"
#include "stm32l0_flash.h"
#include "stm32l0_gpio.h"
#include "stm32l0_i2c.h"
#include "stm32l0_iwdg.h"
#include "stm32l0_lptim.h"
#include "stm32l0_random.h"
#include "stm32l0_rtc.h"
#include "stm32l0_sdspi.h"
#include "stm32l0_servo.h"
#include "stm32l0_sfspi.h"
#include "stm32l0_spi.h"
#include "stm32l0_system.h"
#include "stm32l0_timer.h"
#include "stm32l0_uart.h"
#include "stm32l0_usbd_cdc.h"
#include "stm32l0_usbd_hid.h"

#ifdef __cplusplus
extern "C" {
#endif

extern void USBD_CDC_Initialize(void *);
extern void USBD_CDC_MSC_Initialize(void *);
extern void USBD_CDC_HID_Initialize(void *);
extern void USBD_CDC_MSC_HID_Initialize(void *);

extern void USBD_Initialize(uint16_t vid, uint16_t pid, const uint8_t *manufacturer, const uint8_t *product, void(*initialize)(void *), unsigned int pin_vbus, unsigned int priority);
extern void USBD_Attach(void);
extern void USBD_Detach(void);
extern void USBD_Poll(void);
extern bool USBD_Connected(void);
extern bool USBD_Configured(void);
extern bool USBD_Suspended(void);

extern void CMWX1ZZABZ_Initialize(uint16_t pin_tcxo, uint16_t pin_stsafe);
extern void SX1272MB2DAS_Initialize(void);
extern void WMSGSM42_Initialize(void);

extern int g_swdStatus;  /* 0, default, 1 = enable, 2 = disable, 3 = forced disable */

extern void (*g_serialEventRun)(void);

extern uint32_t __analogReadInternal(uint32_t channel, uint32_t smp);
extern void __analogWriteDisable(uint32_t pin);

/*
 * TIM2   PWM
 * TIM3   PWM
 * TIM6   TONE/DAC
 * TIM7   SERVO
 * TIM21  HSI16/MSI calibration
 * TIM22  ADC
 */

#define STM32L0_PENDSV_IRQ_PRIORITY  3
#define STM32L0_SVCALL_IRQ_PRIORITY  3
#define STM32L0_SYSTICK_IRQ_PRIORITY 3

#define STM32L0_I2C_IRQ_PRIORITY     2
#define STM32L0_SPI_IRQ_PRIORITY     2
#define STM32L0_USB_IRQ_PRIORITY     2

#define STM32L0_DAC_IRQ_PRIORITY     1
#define STM32L0_PWM_IRQ_PRIORITY     1
#define STM32L0_UART_IRQ_PRIORITY    1

#define STM32L0_ADC_IRQ_PRIORITY     0
#define STM32L0_EXTI_IRQ_PRIORITY    0
#define STM32L0_LPTIM_IRQ_PRIORITY   0
#define STM32L0_RTC_IRQ_PRIORITY     0
#define STM32L0_SERVO_IRQ_PRIORITY   0
#define STM32L0_TONE_IRQ_PRIORITY    0


#ifdef __cplusplus
} // extern "C"
#endif
