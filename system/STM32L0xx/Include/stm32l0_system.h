/*
 * Copyright (c) 2017-2019 Thomas Roell.  All rights reserved.
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

#define STM32L0_SYSTEM_MCO_MODE_NONE           0
#define STM32L0_SYSTEM_MCO_MODE_SYSCLK         1
#define STM32L0_SYSTEM_MCO_MODE_HSI16          2
#define STM32L0_SYSTEM_MCO_MODE_MSI            3
#define STM32L0_SYSTEM_MCO_MODE_HSE            4
#define STM32L0_SYSTEM_MCO_MODE_PLL            5
#define STM32L0_SYSTEM_MCO_MODE_LSI            6
#define STM32L0_SYSTEM_MCO_MODE_LSE            7
#define STM32L0_SYSTEM_MCO_MODE_HSI48          8

#define STM32L0_SYSTEM_LOCK_CLOCKS             0
#define STM32L0_SYSTEM_LOCK_SLEEP              1
#define STM32L0_SYSTEM_LOCK_STOP               2
#define STM32L0_SYSTEM_LOCK_STANDBY            3
#define STM32L0_SYSTEM_LOCK_RANGE_2_3          4
#define STM32L0_SYSTEM_LOCK_RANGE_3            5
#define STM32L0_SYSTEM_LOCK_REGULATOR          6
#define STM32L0_SYSTEM_LOCK_VREFINT            7
#define STM32L0_SYSTEM_LOCK_COUNT              8

#define STM32L0_SYSTEM_REFERENCE_USB           0x00000001  /* force pclk1  >= 16MHz */
#define STM32L0_SYSTEM_REFERENCE_I2C2          0x00000002  /* force pclk1  >=  4MHz */

#define STM32L0_SYSTEM_EVENT_CLOCKS            0x00000001
#define STM32L0_SYSTEM_EVENT_SLEEP             0x00000002
#define STM32L0_SYSTEM_EVENT_STOP_ENTER        0x00000004
#define STM32L0_SYSTEM_EVENT_STOP_LEAVE        0x00000008
#define STM32L0_SYSTEM_EVENT_STANDBY           0x00000010
#define STM32L0_SYSTEM_EVENT_RESET             0x00000020

typedef void (*stm32l0_system_callback_t)(void *context, uint32_t events);

typedef struct _stm32l0_system_notify_t {
    struct _stm32l0_system_notify_t *next;
    stm32l0_system_callback_t       callback;
    void                            *context;
    uint32_t                        events;
} stm32l0_system_notify_t;

#define STM32L0_SYSTEM_RESET_POWERON           0
#define STM32L0_SYSTEM_RESET_EXTERNAL          1
#define STM32L0_SYSTEM_RESET_SOFTWARE          2
#define STM32L0_SYSTEM_RESET_WATCHDOG          3
#define STM32L0_SYSTEM_RESET_FIREWALL          4
#define STM32L0_SYSTEM_RESET_OTHER             5
#define STM32L0_SYSTEM_RESET_STANDBY           6

#define STM32L0_SYSTEM_WAKEUP_WKUP             0x00000001
#define STM32L0_SYSTEM_WAKEUP_ALARM            0x00000002
#define STM32L0_SYSTEM_WAKEUP_TIMEOUT          0x00000004
#define STM32L0_SYSTEM_WAKEUP_WATCHDOG         0x00000008
#define STM32L0_SYSTEM_WAKEUP_RESET            0x00000010

#define STM32L0_SYSTEM_TIMEOUT_FOREVER         0xffffffff

#define STM32L0_SYSTEM_POLICY_NONE             0
#define STM32L0_SYSTEM_POLICY_RUN              1
#define STM32L0_SYSTEM_POLICY_SLEEP            2
#define STM32L0_SYSTEM_POLICY_STOP             3

#define STM32L0_SYSTEM_CONFIG_WKUP1            0x00000001
#define STM32L0_SYSTEM_CONFIG_WKUP2            0x00000002
#if defined(STM32L072xx)
#define STM32L0_SYSTEM_CONFIG_WKUP3            0x00000004
#endif /* STM32L072xx */

/* This bit is documented in the reference manuals,
 * but is not part of the CMSIS/Device headers. Turns out
 * that if FWU and ULP are set, VREFINT sometimes does not
 * come back, unless this bit is set ...
 *
 * So set this bit always as the opposite of ULP.
 */
#define SYSCFG_CFGR3_EN_VREFINT          0x00000001
#define SYSCFG_CFGR3_ENBUF_VREFINT_COMP2 SYSCFG_CFGR3_ENBUFLP_VREFINT_COMP

extern void     SystemInit(void);

extern void     stm32l0_system_initialize(uint32_t hclk, uint32_t pclk1, uint32_t pclk2, uint32_t lseclk, uint32_t hseclk, uint32_t option);
extern bool     stm32l0_system_sysclk_configure(uint32_t hclk, uint32_t pclk1, uint32_t pclk2);
extern void     stm32l0_system_lsi_enable(void);
extern void     stm32l0_system_lsi_disable(void);
extern void     stm32l0_system_hsi16_enable(void);
extern void     stm32l0_system_hsi16_disable(void);
extern void     stm32l0_system_hsi48_enable(void);
extern void     stm32l0_system_hsi48_disable(void);
extern void     stm32l0_system_mco_configure(unsigned int mode, unsigned int scale);
extern uint32_t stm32l0_system_reset_cause(void);
extern uint32_t stm32l0_system_wakeup_reason(void);
extern uint32_t stm32l0_system_lseclk(void);
extern uint32_t stm32l0_system_hseclk(void);
extern uint32_t stm32l0_system_lsiclk(void);
extern uint32_t stm32l0_system_sysclk(void);
extern uint32_t stm32l0_system_hclk(void);
extern uint32_t stm32l0_system_pclk1(void);
extern uint32_t stm32l0_system_pclk2(void);
extern void     stm32l0_system_uid(uint32_t *uid);
extern void     stm32l0_system_periph_reset(unsigned int periph);
extern void     stm32l0_system_periph_enable(unsigned int periph);
extern void     stm32l0_system_periph_disable(unsigned int periph);
extern void     stm32l0_system_swd_enable(void);
extern void     stm32l0_system_swd_disable(void);
extern uint32_t stm32l0_system_read_backup(unsigned int index);
extern void     stm32l0_system_write_backup(unsigned int index, uint32_t data);
extern void     stm32l0_system_notify(stm32l0_system_notify_t *notify, stm32l0_system_callback_t callback, void *context, uint32_t events); 
extern void     stm32l0_system_lock(uint32_t lock); 
extern void     stm32l0_system_unlock(uint32_t lock);
extern void     stm32l0_system_reference(uint32_t reference); 
extern void     stm32l0_system_unreference(uint32_t reference); 
extern uint32_t stm32l0_system_policy(uint32_t policy);
extern void     stm32l0_system_sleep(uint32_t policy, uint32_t timeout);
extern void     stm32l0_system_wakeup(void);
extern void     stm32l0_system_standby(uint32_t config);
extern void     stm32l0_system_reset(void);
extern void     stm32l0_system_dfu(void);

#ifdef __cplusplus
}
#endif

#endif /* _STM32L0_SYSTEM_H */
