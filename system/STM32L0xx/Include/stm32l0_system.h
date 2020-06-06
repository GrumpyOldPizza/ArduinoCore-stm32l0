/*
 * Copyright (c) 2017-2020 Thomas Roell.  All rights reserved.
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

#if !defined(_STM32L0_SYSTEM_H)
#define _STM32L0_SYSTEM_H

#include "armv6m.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    STM32L0_SYSTEM_PERIPH_ADC = 0,
    STM32L0_SYSTEM_PERIPH_DAC,
    STM32L0_SYSTEM_PERIPH_USB,
    STM32L0_SYSTEM_PERIPH_USART1,
    STM32L0_SYSTEM_PERIPH_USART2,
#if defined(STM32L072xx) || defined(STM32L082xx)
    STM32L0_SYSTEM_PERIPH_USART4,
    STM32L0_SYSTEM_PERIPH_USART5,
#endif /* STM32L072xx || STM32L082xx */
    STM32L0_SYSTEM_PERIPH_LPUART1,
    STM32L0_SYSTEM_PERIPH_I2C1,
    STM32L0_SYSTEM_PERIPH_I2C2,
#if defined(STM32L072xx) || defined(STM32L082xx)
    STM32L0_SYSTEM_PERIPH_I2C3,
#endif /* STM32L072xx || STM32L082xx */
    STM32L0_SYSTEM_PERIPH_SPI1,
    STM32L0_SYSTEM_PERIPH_SPI2,
    STM32L0_SYSTEM_PERIPH_TIM2,
#if defined(STM32L072xx) || defined(STM32L082xx)
    STM32L0_SYSTEM_PERIPH_TIM3,
#endif /* STM32L072xx || STM32L082xx */
    STM32L0_SYSTEM_PERIPH_TIM6,
#if defined(STM32L072xx) || defined(STM32L082xx)
    STM32L0_SYSTEM_PERIPH_TIM7,
#endif /* STM32L072xx || STM32L082xx */
    STM32L0_SYSTEM_PERIPH_TIM21,
    STM32L0_SYSTEM_PERIPH_TIM22,
    STM32L0_SYSTEM_PERIPH_LPTIM1,
    STM32L0_SYSTEM_PERIPH_CRC,
    STM32L0_SYSTEM_PERIPH_RNG,
    STM32L0_SYSTEM_PERIPH_COUNT
};

#define STM32L0_SYSTEM_OPTION_LSE_BYPASS       0x00000001
#define STM32L0_SYSTEM_OPTION_HSE_BYPASS       0x00000002
#define STM32L0_SYSTEM_OPTION_HSE_PLL          0x00000004

#define STM32L0_SYSTEM_MCO_SOURCE_MASK        0x0000000f
#define STM32L0_SYSTEM_MCO_SOURCE_SHIFT       0
#define STM32L0_SYSTEM_MCO_SOURCE_NONE        0x00000000
#define STM32L0_SYSTEM_MCO_SOURCE_SYSCLK      0x00000001
#define STM32L0_SYSTEM_MCO_SOURCE_MSI         0x00000002
#define STM32L0_SYSTEM_MCO_SOURCE_HSI16       0x00000003
#define STM32L0_SYSTEM_MCO_SOURCE_HSE         0x00000004
#define STM32L0_SYSTEM_MCO_SOURCE_PLL         0x00000005
#define STM32L0_SYSTEM_MCO_SOURCE_LSI         0x00000006
#define STM32L0_SYSTEM_MCO_SOURCE_LSE         0x00000007
#define STM32L0_SYSTEM_MCO_SOURCE_HSI48       0x00000008
#define STM32L0_SYSTEM_MCO_DIVIDE_MASK        0x00000070
#define STM32L0_SYSTEM_MCO_DIVIDE_SHIFT       4
#define STM32L0_SYSTEM_MCO_DIVIDE_BY_1        0x00000000
#define STM32L0_SYSTEM_MCO_DIVIDE_BY_2        0x00000010
#define STM32L0_SYSTEM_MCO_DIVIDE_BY_4        0x00000020
#define STM32L0_SYSTEM_MCO_DIVIDE_BY_8        0x00000030
#define STM32L0_SYSTEM_MCO_DIVIDE_BY_16       0x00000040

#define STM32L0_SYSTEM_LOCK_RUN                0
#define STM32L0_SYSTEM_LOCK_SLEEP              1
#define STM32L0_SYSTEM_LOCK_DEEPSLEEP          2
#define STM32L0_SYSTEM_LOCK_REGULATOR          3
#define STM32L0_SYSTEM_LOCK_EEPROM             4
#define STM32L0_SYSTEM_LOCK_COUNT              5

#define STM32L0_SYSTEM_REFERENCE_SWD           0x00000001
#define STM32L0_SYSTEM_REFERENCE_RNG           0x00000002
#define STM32L0_SYSTEM_REFERENCE_USB           0x00000004  /* force pclk1  >= 16MHz */
#define STM32L0_SYSTEM_REFERENCE_I2C2          0x00000008  /* force pclk1  >=  4MHz */

#define STM32L0_SYSTEM_NOTIFY_CLOCKS           0x00000001
#define STM32L0_SYSTEM_NOTIFY_SLEEP            0x00000002
#define STM32L0_SYSTEM_NOTIFY_STOP_ENTER       0x00000004
#define STM32L0_SYSTEM_NOTIFY_STOP_LEAVE       0x00000008
#define STM32L0_SYSTEM_NOTIFY_STANDBY          0x00000010
#define STM32L0_SYSTEM_NOTIFY_RESET            0x00000020
#define STM32L0_SYSTEM_NOTIFY_DFU              0x00000040

typedef void (*stm32l0_system_fatal_callback_t)(void);

typedef void (*stm32l0_system_callback_t)(void *context, uint32_t events);

typedef struct _stm32l0_system_notify_t {
    struct _stm32l0_system_notify_t *next;
    stm32l0_system_callback_t       callback;
    void                            *context;
    uint32_t                        mask;
} stm32l0_system_notify_t;

#define STM32L0_SYSTEM_RESET_POWERON           0
#define STM32L0_SYSTEM_RESET_EXTERNAL          1
#define STM32L0_SYSTEM_RESET_INTERNAL          2
#define STM32L0_SYSTEM_RESET_SOFTWARE          3
#define STM32L0_SYSTEM_RESET_FIREWALL          4
#define STM32L0_SYSTEM_RESET_WATCHDOG          5
#define STM32L0_SYSTEM_RESET_CRASH             6
#define STM32L0_SYSTEM_RESET_STANDBY           7

#define STM32L0_SYSTEM_WAKEUP_NONE             0x00000000
#define STM32L0_SYSTEM_WAKEUP_PIN              0x00000001
#define STM32L0_SYSTEM_WAKEUP_TIMEOUT          0x00000100
#define STM32L0_SYSTEM_WAKEUP_ALARM            0x00000200
#define STM32L0_SYSTEM_WAKEUP_TAMP_1           0x00000400
#define STM32L0_SYSTEM_WAKEUP_TAMP_2           0x00000800
#define STM32L0_SYSTEM_WAKEUP_TAMP_3           0x00001000
#define STM32L0_SYSTEM_WAKEUP_WATCHDOG         0x00002000
#define STM32L0_SYSTEM_WAKEUP_RESET            0x00004000

#define STM32L0_SYSTEM_EVENT_APPLICATION       0x00000001
#define STM32L0_SYSTEM_EVENT_TIMEOUT           0x80000000

#define STM32L0_SYSTEM_TIMEOUT_NONE            0x00000000
#define STM32L0_SYSTEM_TIMEOUT_FOREVER         0xffffffff

#define STM32L0_SYSTEM_POLICY_RUN              0
#define STM32L0_SYSTEM_POLICY_SLEEP            1
#define STM32L0_SYSTEM_POLICY_DEEPSLEEP        2

#define STM32L0_SYSTEM_STANDBY_PIN_1_RISING    0x00000001
#define STM32L0_SYSTEM_STANDBY_PIN_2_RISING    0x00000002
#define STM32L0_SYSTEM_STANDBY_ALARM           0x00010000
#define STM32L0_SYSTEM_STANDBY_TAMP_1          0x00020000
#define STM32L0_SYSTEM_STANDBY_TAMP_2          0x00040000

extern void     SystemInit(void);

extern void     stm32l0_system_initialize(uint32_t hclk, uint32_t pclk1, uint32_t pclk2, uint32_t lseclk, uint32_t hseclk, uint32_t options);
extern bool     stm32l0_system_sysclk_configure(uint32_t hclk, uint32_t pclk1, uint32_t pclk2);
extern void     stm32l0_system_mco_configure(uint32_t mco);
extern void     stm32l0_system_hse_enable(void);
extern void     stm32l0_system_hse_disable(void);
extern void     stm32l0_system_lsi_enable(void);
extern void     stm32l0_system_lsi_disable(void);
extern void     stm32l0_system_hsi16_enable(void);
extern void     stm32l0_system_hsi16_disable(void);
extern void     stm32l0_system_hsi48_enable(void);
extern void     stm32l0_system_hsi48_disable(void);
extern void     stm32l0_system_clk48_enable(void);
extern void     stm32l0_system_clk48_disable(void);
extern void     stm32l0_system_vrefint_enable(void);
extern void     stm32l0_system_vrefint_disable(void);
extern uint32_t stm32l0_system_reset_cause(void);
extern uint32_t stm32l0_system_wakeup_reason(void);
extern uint32_t stm32l0_system_lseclk(void);
extern uint32_t stm32l0_system_hseclk(void);
extern uint32_t stm32l0_system_lsiclk(void);
extern uint32_t stm32l0_system_sysclk(void);
extern uint32_t stm32l0_system_hclk(void);
extern uint32_t stm32l0_system_pclk1(void);
extern uint32_t stm32l0_system_pclk2(void);
extern uint64_t stm32l0_system_serial(void);
extern void     stm32l0_system_uid(uint32_t *uid);
extern void     stm32l0_system_periph_reset(unsigned int periph);
extern void     stm32l0_system_periph_enable(unsigned int periph);
extern void     stm32l0_system_periph_disable(unsigned int periph);
extern void     stm32l0_system_swd_enable(void);
extern void     stm32l0_system_swd_disable(void);
extern void     stm32l0_system_register(stm32l0_system_notify_t *notify, stm32l0_system_callback_t callback, void *context, uint32_t mask); 
extern void     stm32l0_system_unregister(stm32l0_system_notify_t *notify);
extern void     stm32l0_system_notify(uint32_t notify); 
extern void     stm32l0_system_lock(uint32_t lock); 
extern void     stm32l0_system_unlock(uint32_t lock);
extern void     stm32l0_system_reference(uint32_t reference); 
extern void     stm32l0_system_unreference(uint32_t reference); 
extern void     stm32l0_system_sleep(uint32_t policy, uint32_t mask, uint32_t timeout);
extern void     stm32l0_system_wakeup(uint32_t events);
extern void     stm32l0_system_standby(uint32_t control, uint32_t timeout);
extern void     stm32l0_system_hook(stm32l0_system_fatal_callback_t callback);
extern void     stm32l0_system_fatal(void) __attribute__((noreturn));
extern void     stm32l0_system_reset(void) __attribute__((noreturn));
extern void     stm32l0_system_dfu(void) __attribute__((noreturn));

  
extern void WWDG_IRQHandler(void);
extern void PVD_IRQHandler(void);
extern void RTC_IRQHandler(void);
extern void FLASH_IRQHandler(void);
extern void RCC_IRQHandler(void);
extern void EXTI0_1_IRQHandler(void);
extern void EXTI2_3_IRQHandler(void);
extern void EXTI4_15_IRQHandler(void);
extern void TSC_IRQHandler(void);
extern void DMA1_Channel1_IRQHandler(void);
extern void DMA1_Channel2_3_IRQHandler(void);
extern void DMA1_Channel4_5_6_7_IRQHandler(void);
extern void ADC1_COMP_IRQHandler(void);
extern void LPTIM1_IRQHandler(void);
#if defined(STM32L072xx) || defined(STM32L082xx)
extern void USART4_5_IRQHandler(void);
#endif /* defined(STM32L072xx) || defined(STM32L082xx) */
extern void TIM2_IRQHandler(void);
#if defined(STM32L072xx) || defined(STM32L082xx)
extern void TIM3_IRQHandler(void);
#endif /* defined(STM32L072xx) || defined(STM32L082xx) */
extern void TIM6_IRQHandler(void);
#if defined(STM32L072xx) || defined(STM32L082xx)
extern void TIM7_IRQHandler(void);
#endif /* defined(STM32L072xx) || defined(STM32L082xx) */
extern void TIM21_IRQHandler(void);
#if defined(STM32L072xx) || defined(STM32L082xx)
extern void I2C3_IRQHandler(void);
#endif /* defined(STM32L072xx) || defined(STM32L082xx) */
extern void TIM22_IRQHandler(void);
extern void I2C1_IRQHandler(void);
extern void I2C2_IRQHandler(void);
extern void SPI1_IRQHandler(void);
extern void SPI2_IRQHandler(void);
extern void USART1_IRQHandler(void);
extern void USART2_IRQHandler(void);
extern void LPUART1_IRQHandler(void);
extern void USB_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* _STM32L0_SYSTEM_H */
