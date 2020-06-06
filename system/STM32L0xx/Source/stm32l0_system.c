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

#include "armv6m.h"
#include "stm32l0_gpio.h"
#include "stm32l0_exti.h"
#include "stm32l0_dma.h"
#include "stm32l0_rtc.h"
#include "stm32l0_lptim.h"
#include "stm32l0_eeprom.h"
#include "stm32l0_system.h"

#undef abs
#define abs(_i)  (((_i) >= 0) ? (_i) : -(_i))

#define STM32L0_SYSTEM_REFERENCE_RANGE_1_2      \
    (STM32L0_SYSTEM_REFERENCE_RNG |             \
     STM32L0_SYSTEM_REFERENCE_USB)

#define STM32L0_SYSTEM_REFERENCE_RANGE_1        \
    (0)

#define STM32L0_CRASH_SIGNATURE_DATA    0xfeeb6667
#define STM32L0_CRASH_SIGNATURE_ADDRESS 0x20000000

extern uint32_t __StackTop;

__attribute__((section(".noinit"))) uint32_t SystemCoreClock;

typedef struct _stm32l0_system_device_t {
    uint32_t                  options;
    uint16_t                  reset;
    uint16_t                  wakeup;
    uint32_t                  lseclk;
    uint32_t                  hseclk;
    uint32_t                  lsiclk;
    uint32_t                  sysclk;
    uint32_t                  hclk;
    uint32_t                  pclk1;
    uint32_t                  pclk2;
    uint8_t                   mco;
    uint8_t                   hse;
    uint8_t                   msi;
    uint8_t                   lsi;
    uint8_t                   hsi16;
    uint8_t                   hsi48;
    uint8_t                   clk48;
    uint8_t                   vrefint;
    uint8_t                   pllsys;
    uint32_t                  busspre;
    stm32l0_system_notify_t   *notify;
    volatile uint8_t          lock[STM32L0_SYSTEM_LOCK_COUNT];
    volatile uint32_t         reference;
    volatile uint32_t         events;
    stm32l0_rtc_timer_t       timeout;
    stm32l0_system_fatal_callback_t callback;
} stm32l0_system_device_t;

static stm32l0_system_device_t stm32l0_system_device;

static volatile uint32_t * const stm32l0_system_xlate_RSTR[STM32L0_SYSTEM_PERIPH_COUNT] = {
    &RCC->APB2RSTR,            /* STM32L0_SYSTEM_PERIPH_ADC */
    &RCC->APB1RSTR,            /* STM32L0_SYSTEM_PERIPH_DAC */
    &RCC->APB1RSTR,            /* STM32L0_SYSTEM_PERIPH_USB */
    &RCC->APB2RSTR,            /* STM32L0_SYSTEM_PERIPH_USART1 */
    &RCC->APB1RSTR,            /* STM32L0_SYSTEM_PERIPH_USART2 */
#if defined(STM32L072xx) || defined(STM32L082xx)
    &RCC->APB1RSTR,            /* STM32L0_SYSTEM_PERIPH_USART4 */
    &RCC->APB1RSTR,            /* STM32L0_SYSTEM_PERIPH_USART5 */
#endif /* STM32L072xx || STM32L082xx */
    &RCC->APB1RSTR,            /* STM32L0_SYSTEM_PERIPH_LPUART1 */
    &RCC->APB1RSTR,            /* STM32L0_SYSTEM_PERIPH_I2C1 */
    &RCC->APB1RSTR,            /* STM32L0_SYSTEM_PERIPH_I2C2 */
#if defined(STM32L072xx) || defined(STM32L082xx)
    &RCC->APB1RSTR,            /* STM32L0_SYSTEM_PERIPH_I2C3 */
#endif /* STM32L072xx || STM32L082xx */
    &RCC->APB2RSTR,            /* STM32L0_SYSTEM_PERIPH_SPI1 */
    &RCC->APB1RSTR,            /* STM32L0_SYSTEM_PERIPH_SPI2 */
    &RCC->APB1RSTR,            /* STM32L0_SYSTEM_PERIPH_TIM2 */
#if defined(STM32L072xx) || defined(STM32L082xx)
    &RCC->APB1RSTR,            /* STM32L0_SYSTEM_PERIPH_TIM3 */
#endif /* STM32L072xx || STM32L082xx */
    &RCC->APB1RSTR,            /* STM32L0_SYSTEM_PERIPH_TIM6 */
#if defined(STM32L072xx) || defined(STM32L082xx)
    &RCC->APB1RSTR,            /* STM32L0_SYSTEM_PERIPH_TIM7 */
#endif /* STM32L072xx || STM32L082xx */
    &RCC->APB2RSTR,            /* STM32L0_SYSTEM_PERIPH_TIM21 */
    &RCC->APB2RSTR,            /* STM32L0_SYSTEM_PERIPH_TIM22 */
    &RCC->APB1RSTR,            /* STM32L0_SYSTEM_PERIPH_LPTIM1 */
    &RCC->AHBRSTR,             /* STM32L0_SYSTEM_PERIPH_CRC */
    &RCC->AHBRSTR,             /* STM32L0_SYSTEM_PERIPH_RNG */
};

static uint32_t const stm32l0_system_xlate_RSTMSK[STM32L0_SYSTEM_PERIPH_COUNT] = {
    RCC_APB2RSTR_ADCRST,       /* STM32L0_SYSTEM_PERIPH_ADC */
    RCC_APB1RSTR_DACRST,       /* STM32L0_SYSTEM_PERIPH_DAC */
    RCC_APB1RSTR_USBRST,       /* STM32L0_SYSTEM_PERIPH_USB */
    RCC_APB2RSTR_USART1RST,    /* STM32L0_SYSTEM_PERIPH_USART1 */
    RCC_APB1RSTR_USART2RST,    /* STM32L0_SYSTEM_PERIPH_USART2 */
#if defined(STM32L072xx) || defined(STM32L082xx)
    RCC_APB1RSTR_USART4RST,    /* STM32L0_SYSTEM_PERIPH_USART4 */
    RCC_APB1RSTR_USART5RST,    /* STM32L0_SYSTEM_PERIPH_USART5 */
#endif /* STM32L072xx || STM32L082xx */
    RCC_APB1RSTR_LPUART1RST,   /* STM32L0_SYSTEM_PERIPH_LPUART1 */
    RCC_APB1RSTR_I2C1RST,      /* STM32L0_SYSTEM_PERIPH_I2C1 */
    RCC_APB1RSTR_I2C2RST,      /* STM32L0_SYSTEM_PERIPH_I2C2 */
#if defined(STM32L072xx) || defined(STM32L082xx)
    RCC_APB1RSTR_I2C3RST,      /* STM32L0_SYSTEM_PERIPH_I2C3 */
#endif /* STM32L072xx || STM32L082xx */
    RCC_APB2RSTR_SPI1RST,      /* STM32L0_SYSTEM_PERIPH_SPI1 */
    RCC_APB1RSTR_SPI2RST,      /* STM32L0_SYSTEM_PERIPH_SPI2 */
    RCC_APB1RSTR_TIM2RST,      /* STM32L0_SYSTEM_PERIPH_TIM2 */
#if defined(STM32L072xx) || defined(STM32L082xx)
    RCC_APB1RSTR_TIM3RST,      /* STM32L0_SYSTEM_PERIPH_TIM3 */
#endif /* STM32L072xx || STM32L082xx */
    RCC_APB1RSTR_TIM6RST,      /* STM32L0_SYSTEM_PERIPH_TIM6 */
#if defined(STM32L072xx) || defined(STM32L082xx)
    RCC_APB1RSTR_TIM7RST,      /* STM32L0_SYSTEM_PERIPH_TIM7 */
#endif /* STM32L072xx || STM32L082xx */
    RCC_APB2RSTR_TIM21RST,     /* STM32L0_SYSTEM_PERIPH_TIM21 */
    RCC_APB2RSTR_TIM22RST,     /* STM32L0_SYSTEM_PERIPH_TIM22 */
    RCC_APB1RSTR_LPTIM1RST,    /* STM32L0_SYSTEM_PERIPH_LPTIM1 */
    RCC_AHBRSTR_CRCRST,        /* STM32L0_SYSTEM_PERIPH_CRC */
    RCC_AHBRSTR_RNGRST,        /* STM32L0_SYSTEM_PERIPH_RNG */
};

static volatile uint32_t * const stm32l0_system_xlate_ENR[STM32L0_SYSTEM_PERIPH_COUNT] = {
    &RCC->APB2ENR,             /* STM32L0_SYSTEM_PERIPH_ADC */
    &RCC->APB1ENR,             /* STM32L0_SYSTEM_PERIPH_DAC */
    &RCC->APB1ENR,             /* STM32L0_SYSTEM_PERIPH_USB */
    &RCC->APB2ENR,             /* STM32L0_SYSTEM_PERIPH_USART1 */
    &RCC->APB1ENR,             /* STM32L0_SYSTEM_PERIPH_USART2 */
#if defined(STM32L072xx) || defined(STM32L082xx)
    &RCC->APB1ENR,             /* STM32L0_SYSTEM_PERIPH_USART4 */
    &RCC->APB1ENR,             /* STM32L0_SYSTEM_PERIPH_USART5 */
#endif /* STM32L072xx || STM32L082xx */
    &RCC->APB1ENR,             /* STM32L0_SYSTEM_PERIPH_LPUART1 */
    &RCC->APB1ENR,             /* STM32L0_SYSTEM_PERIPH_I2C1 */
    &RCC->APB1ENR,             /* STM32L0_SYSTEM_PERIPH_I2C2 */
#if defined(STM32L072xx) || defined(STM32L082xx)
    &RCC->APB1ENR,             /* STM32L0_SYSTEM_PERIPH_I2C3 */
#endif /* STM32L072xx || STM32L082xx */
    &RCC->APB2ENR,             /* STM32L0_SYSTEM_PERIPH_SPI1 */
    &RCC->APB1ENR,             /* STM32L0_SYSTEM_PERIPH_SPI2 */
    &RCC->APB1ENR,             /* STM32L0_SYSTEM_PERIPH_TIM2 */
#if defined(STM32L072xx) || defined(STM32L082xx)
    &RCC->APB1ENR,             /* STM32L0_SYSTEM_PERIPH_TIM3 */
#endif /* STM32L072xx || STM32L082xx */
    &RCC->APB1ENR,             /* STM32L0_SYSTEM_PERIPH_TIM6 */
#if defined(STM32L072xx) || defined(STM32L082xx)
    &RCC->APB1ENR,             /* STM32L0_SYSTEM_PERIPH_TIM7 */
#endif /* STM32L072xx || STM32L082xx */
    &RCC->APB2ENR,             /* STM32L0_SYSTEM_PERIPH_TIM21 */
    &RCC->APB2ENR,             /* STM32L0_SYSTEM_PERIPH_TIM22 */
    &RCC->APB1ENR,             /* STM32L0_SYSTEM_PERIPH_LPTIM1 */
    &RCC->AHBENR,              /* STM32L0_SYSTEM_PERIPH_CRC */
    &RCC->AHBENR,              /* STM32L0_SYSTEM_PERIPH_RNG */
};

static uint32_t const stm32l0_system_xlate_ENMSK[STM32L0_SYSTEM_PERIPH_COUNT] = {
    RCC_APB2ENR_ADCEN,         /* STM32L0_SYSTEM_PERIPH_ADC */
    RCC_APB1ENR_DACEN,         /* STM32L0_SYSTEM_PERIPH_DAC */
    RCC_APB1ENR_USBEN,         /* STM32L0_SYSTEM_PERIPH_USB */
    RCC_APB2ENR_USART1EN,      /* STM32L0_SYSTEM_PERIPH_USART1 */
    RCC_APB1ENR_USART2EN,      /* STM32L0_SYSTEM_PERIPH_USART2 */
#if defined(STM32L072xx) || defined(STM32L082xx)
    RCC_APB1ENR_USART4EN,      /* STM32L0_SYSTEM_PERIPH_USART4 */
    RCC_APB1ENR_USART5EN,      /* STM32L0_SYSTEM_PERIPH_USART5 */
#endif /* STM32L072xx || STM32L082xx */
    RCC_APB1ENR_LPUART1EN,     /* STM32L0_SYSTEM_PERIPH_LPUART1 */
    RCC_APB1ENR_I2C1EN,        /* STM32L0_SYSTEM_PERIPH_I2C1 */
    RCC_APB1ENR_I2C2EN,        /* STM32L0_SYSTEM_PERIPH_I2C2 */
#if defined(STM32L072xx) || defined(STM32L082xx)
    RCC_APB1ENR_I2C3EN,        /* STM32L0_SYSTEM_PERIPH_I2C3 */
#endif /* STM32L072xx || STM32L082xx */
    RCC_APB2ENR_SPI1EN,        /* STM32L0_SYSTEM_PERIPH_SPI1 */
    RCC_APB1ENR_SPI2EN,        /* STM32L0_SYSTEM_PERIPH_SPI2 */
    RCC_APB1ENR_TIM2EN,        /* STM32L0_SYSTEM_PERIPH_TIM2 */
#if defined(STM32L072xx) || defined(STM32L082xx)
    RCC_APB1ENR_TIM3EN,        /* STM32L0_SYSTEM_PERIPH_TIM3 */
#endif /* STM32L072xx || STM32L082xx */
    RCC_APB1ENR_TIM6EN,        /* STM32L0_SYSTEM_PERIPH_TIM6 */
#if defined(STM32L072xx) || defined(STM32L082xx)
    RCC_APB1ENR_TIM7EN,        /* STM32L0_SYSTEM_PERIPH_TIM7 */
#endif /* STM32L072xx || STM32L082xx */
    RCC_APB2ENR_TIM21EN,       /* STM32L0_SYSTEM_PERIPH_TIM21 */
    RCC_APB2ENR_TIM22EN,       /* STM32L0_SYSTEM_PERIPH_TIM22 */
    RCC_APB1ENR_LPTIM1EN,      /* STM32L0_SYSTEM_PERIPH_LPTIM1 */
    RCC_AHBENR_CRCEN,          /* STM32L0_SYSTEM_PERIPH_CRC */
    RCC_AHBENR_RNGEN,          /* STM32L0_SYSTEM_PERIPH_RNG */
};

void SystemInit(void)
{
    RCC->CIER = 0x00000000;

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    /* Switch to Main Flash @ 0x00000000. Make sure all buffers are disabled.
     */

    FLASH->ACR = FLASH_ACR_DISAB_BUF;
        
    SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_MEM_MODE;

    PWR->CR |= PWR_CR_DBP;
    
    while (!(PWR->CR & PWR_CR_DBP))
    {
    }

    if (RCC->CSR & RCC_CSR_RTCEN)
    {
        RTC->WPR = 0xca;
        RTC->WPR = 0x53;

        /* If RTC_CR_BCK is set it means the reset was triggered to
         * branch throu to the STM32 BOOTLOADER.
         */
        if (RTC->CR & RTC_CR_BKP)
        {
            RTC->CR &= ~RTC_CR_BKP;

            RTC->WPR = 0x00;

            SYSCFG->CFGR1 |= SYSCFG_CFGR1_MEM_MODE_0;

            FLASH->ACR = 0;

            RCC->IOPENR |= (RCC_IOPENR_IOPAEN | RCC_IOPENR_IOPBEN | RCC_IOPENR_IOPCEN);

            GPIOA->LCKR = 0x0001e7ff;
            GPIOA->LCKR = 0x0000e7ff;
            GPIOA->LCKR = 0x0001e7ff;
            GPIOA->LCKR;
            GPIOB->LCKR = 0x0001ffff;
            GPIOB->LCKR = 0x0000ffff;
            GPIOB->LCKR = 0x0001ffff;
            GPIOB->LCKR;
            GPIOC->LCKR = 0x00011fff;
            GPIOC->LCKR = 0x00001fff;
            GPIOC->LCKR = 0x00011fff;
            GPIOC->LCKR;

            RCC->IOPENR &= ~(RCC_IOPENR_IOPAEN | RCC_IOPENR_IOPBEN | RCC_IOPENR_IOPCEN);

            RCC->APB2ENR &= ~RCC_APB2ENR_SYSCFGEN;

            RCC->APB1ENR &= ~RCC_APB1ENR_PWREN;
        
            /* This needs to be assembly code as GCC catches NULL 
             * dereferences ...
             */
            __asm__ volatile ("   mov     r2, #0                         \n"
                              "   ldr     r0, [r2, #0]                   \n"
                              "   ldr     r1, [r2, #4]                   \n"
                              "   msr     MSP, r0                        \n"
                              "   dsb                                    \n"
                              "   isb                                    \n"
                              "   bx      r1                             \n");
        }
    }
    
    /* We should be at a 2MHz MSI clock, so switch to HSI16 for the
     * init code to be half way fast.
     */
    FLASH->ACR = FLASH_ACR_PRFTEN;

    RCC->CR |= RCC_CR_HSION;
        
    while (!(RCC->CR & RCC_CR_HSIRDY))
    {
    }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;

    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
    {
    }

    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) | (RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV2);
    __DSB();
    __NOP();
    __NOP();
    __NOP();
    __NOP();

    RCC->CR &= ~RCC_CR_MSION;

    RCC->APB1ENR &= ~RCC_APB1ENR_PWREN;

    SystemCoreClock = 16000000;
}

void stm32l0_system_initialize(uint32_t hclk, uint32_t pclk1, uint32_t pclk2, uint32_t lseclk, uint32_t hseclk, uint32_t options)
{
    uint32_t primask, n, count, hsiclk, hsitrim, clk, trim, tc0, tc1;

    primask = __get_PRIMASK();

    __disable_irq();

    stm32l0_system_device.options = options;
    
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    if (PWR->CSR & PWR_CSR_SBF)
    {
        stm32l0_system_device.reset = STM32L0_SYSTEM_RESET_STANDBY;
        stm32l0_system_device.wakeup = STM32L0_SYSTEM_WAKEUP_NONE;
        
        if (PWR->CSR & PWR_CSR_WUF)
        {
            if (RTC->ISR & (RTC_ISR_ALRAF | RTC_ISR_ALRBF | RTC_ISR_TAMP1F | RTC_ISR_TAMP2F))
            {
                if (RTC->ISR & RTC_ISR_ALRAF)
                {
                    stm32l0_system_device.wakeup |= STM32L0_SYSTEM_WAKEUP_ALARM;
                }
                
                if (RTC->ISR & RTC_ISR_ALRBF)
                {
                    stm32l0_system_device.wakeup |= STM32L0_SYSTEM_WAKEUP_TIMEOUT;
                }

                if (RTC->ISR & RTC_ISR_TAMP1F)
                {
                    stm32l0_system_device.wakeup |= STM32L0_SYSTEM_WAKEUP_TAMP_1;
                }

                if (RTC->ISR & RTC_ISR_TAMP2F)
                {
                    stm32l0_system_device.wakeup |= STM32L0_SYSTEM_WAKEUP_TAMP_2;
                }
            }
            else
            {
                stm32l0_system_device.wakeup |= STM32L0_SYSTEM_WAKEUP_PIN;
            }

            PWR->CR |= PWR_CR_CSBF;
        }

        if (RCC->CSR & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF))
        {
            stm32l0_system_device.wakeup |= STM32L0_SYSTEM_WAKEUP_WATCHDOG;
        }
        
        if (RCC->CSR & (RCC_CSR_FWRSTF | RCC_CSR_SFTRSTF | RCC_CSR_LPWRRSTF | RCC_CSR_OBLRSTF | RCC_CSR_PORRSTF))
        {
            stm32l0_system_device.wakeup |= STM32L0_SYSTEM_WAKEUP_RESET;
        }
    }
    else
    {
        stm32l0_system_device.reset = STM32L0_SYSTEM_RESET_EXTERNAL;
        stm32l0_system_device.wakeup = STM32L0_SYSTEM_WAKEUP_NONE;

        if (RCC->CSR & RCC_CSR_SFTRSTF)
        {
            if (*((volatile uint32_t*)STM32L0_CRASH_SIGNATURE_ADDRESS) == STM32L0_CRASH_SIGNATURE_DATA)
            {
                stm32l0_system_device.reset = STM32L0_SYSTEM_RESET_CRASH;
            }
            else
            {
                stm32l0_system_device.reset = STM32L0_SYSTEM_RESET_SOFTWARE;
            }
        }
        else
        {
            if (RCC->CSR & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF))
            {
                stm32l0_system_device.reset = STM32L0_SYSTEM_RESET_WATCHDOG;
            }
            
            else if (RCC->CSR & RCC_CSR_FWRSTF)
            {
                stm32l0_system_device.reset = STM32L0_SYSTEM_RESET_FIREWALL;
            }
            
            else if (RCC->CSR & (RCC_CSR_LPWRRSTF | RCC_CSR_OBLRSTF))
            {
                stm32l0_system_device.reset = STM32L0_SYSTEM_RESET_INTERNAL;
            }
            
            else if (RCC->CSR & RCC_CSR_PORRSTF)
            {
                stm32l0_system_device.reset = STM32L0_SYSTEM_RESET_POWERON;
            }
        }
    }
    
    RCC->CSR |= RCC_CSR_RMVF;
    RCC->CSR &= ~RCC_CSR_RMVF;

    if (RCC->CSR & RCC_CSR_RTCEN)
    {
        RTC->CR &= ~(RTC_CR_TSIE | RTC_CR_WUTIE | RTC_CR_ALRBIE | RTC_CR_ALRAIE | RTC_CR_TSE | RTC_CR_WUTE | RTC_CR_ALRBE | RTC_CR_ALRAE);
        RTC->TAMPCR &= ~(RTC_TAMPCR_TAMP2IE | RTC_TAMPCR_TAMP2E | RTC_TAMPCR_TAMP1IE | RTC_TAMPCR_TAMP1E);
        RTC->ISR = 0;

        /* If RTC was setup wrong here, there is a Firmware mismatch ...
         */
        if ((RTC->PRER & (RTC_PRER_PREDIV_S_Msk | RTC_PRER_PREDIV_A_Msk)) != (((STM32L0_RTC_PREDIV_S -1) << RTC_PRER_PREDIV_S_Pos) | ((STM32L0_RTC_PREDIV_A -1) << RTC_PRER_PREDIV_A_Pos)))
        {
            RCC->CSR &= ~RCC_CSR_RTCEN;
            RCC->CSR |= RCC_CSR_RTCRST;
        }
    }

    RCC->CSR &= ~RCC_CSR_RTCRST;
    
    *((volatile uint32_t*)STM32L0_CRASH_SIGNATURE_ADDRESS) = ~STM32L0_CRASH_SIGNATURE_DATA;
    
    stm32l0_system_device.lseclk = lseclk;
    stm32l0_system_device.hseclk = hseclk;
    stm32l0_system_device.lsiclk = 37000;

    if (stm32l0_system_device.reset == STM32L0_SYSTEM_RESET_STANDBY)
    {
        /* Use saved initialization data from BKP4R when returning from STANDBY to avoid
         * calbibration.
         */

        if (!(RCC->CSR & RCC_CSR_LSEON))
        {
            stm32l0_system_device.lseclk = 0;

            stm32l0_system_device.lsi |= 0x80;
        }

        if (RTC->BKP4R & 0x00004000)
        {
            if (options & STM32L0_SYSTEM_OPTION_HSE_BYPASS)
            {
                RCC->CR |= RCC_CR_HSEBYP;
            }
        }
        else
        {
            stm32l0_system_device.hseclk = 0;
        }

        stm32l0_system_device.lsiclk = ((RTC->BKP4R & STM32L0_RTC_BKP4R_LSICLK_MASK) >> STM32L0_RTC_BKP4R_LSICLK_SHIFT) * STM32L0_RTC_BKP4R_LSICLK_SCALE;

        RCC->ICSCR = ((RCC->ICSCR & ~RCC_ICSCR_HSITRIM) | (((RTC->BKP4R & STM32L0_RTC_BKP4R_HSITRIM_MASK) >> STM32L0_RTC_BKP4R_HSITRIM_SHIFT) << RCC_ICSCR_HSITRIM_Pos));
    }
    else
    {
        if (lseclk)
        {
            if (options & STM32L0_SYSTEM_OPTION_LSE_BYPASS)
            {
                RCC->CSR |= RCC_CSR_LSEBYP;
            }

            RCC->CSR |= RCC_CSR_LSEON;
                
            /* The loop below take about 8 cycles per iteration. The startup time for
             * LSE is 5000ms. At 16MHz this corresponds to about 10000000 iterations.
             */
            count = 0;
            
            while (!(RCC->CSR & RCC_CSR_LSERDY))
            {
                if (++count >= 10000000)
                {
                    stm32l0_system_device.lseclk = 0;
                    
                    RCC->CSR &= ~RCC_CSR_LSEON;
                    break;
                }
            }
        }

        if (hseclk)
        {
            if (!(options & STM32L0_SYSTEM_OPTION_HSE_BYPASS))
            {
                RCC->CR |= RCC_CR_HSEON;

                /* The loop below take about 8 cycles per iteration. The startup time for
                 * HSE is 100ms. At 16MHz this corresponds to about 200000 iterations.
                 */
                count = 0;
                
                while (!(RCC->CR & RCC_CR_HSERDY))
                {
                    if (++count >= 200000)
                    {
                        stm32l0_system_device.hseclk = 0;
                        break;
                    }
                }
                
                RCC->CR &= ~RCC_CR_HSEON;
            }
        }

        RCC->CSR |= RCC_CSR_LSION;
        
        while (!(RCC->CSR & RCC_CSR_LSIRDY))
        {
        }

        if (stm32l0_system_device.lseclk)
        {
            /* Here HSI16 is the main clock, so count 32 pulses of LSE via TIM21.
             * This is 1/1024 of a second, so one can compute hsiclk from there.
             * Iterate over all hsitrim values to find the closest match.
             */

            hsiclk = 0;
            hsitrim = 0x10;

            RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;

            TIM21->CR1   = TIM_CR1_URS;
            TIM21->CR2   = 0;
            TIM21->DIER  = 0;
            TIM21->PSC   = 0;
            TIM21->ARR   = 0xffff;
            TIM21->OR    = TIM21_OR_TI1_RMP_2; /* Select LSE as TI1 */
            TIM21->EGR   = TIM_EGR_UG;
            TIM21->CR1   |= TIM_CR1_CEN;

            TIM21->CCER  = 0;
            TIM21->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_IC1PSC_1 | TIM_CCMR1_IC1PSC_0; /* Count every 8th pulse */
            TIM21->CCER  = TIM_CCER_CC1E;

            for (trim = 0; trim <= 0x1f; trim++)
            {
                RCC->ICSCR = (RCC->ICSCR & ~RCC_ICSCR_HSITRIM) | (trim << RCC_ICSCR_HSITRIM_Pos);

                TIM21->SR = 0;
        
                while (!(TIM21->SR & TIM_SR_CC1IF))
                {
                }
            
                tc0 = TIM21->CCR1;
        
                for (n = 0, count = 0; n < (32 / 8); n++)
                {
                    TIM21->SR = 0;
                
                    while (!(TIM21->SR & TIM_SR_CC1IF))
                    {
                    }
                
                    tc1 = TIM21->CCR1;
                
                    if (tc1 < tc0)
                    {
                        count += ((65536 + tc1) - tc0);
                    }
                    else
                    {
                        count += (tc1 - tc0);
                    }

                    tc0 = tc1;
                }

                clk = (32768 / 32) * count;

                if (abs((int)(16000000 - hsiclk)) > abs((int)(16000000 - clk)))
                {
                    hsiclk = clk;
                    hsitrim = trim;
                }
            }

            RCC->ICSCR = (RCC->ICSCR & ~RCC_ICSCR_HSITRIM) | (hsitrim << RCC_ICSCR_HSITRIM_Pos);

            /* Here HSI16 is the main clock, so count 256 pulses of LSI via TIM21.
             * By knowing hsiclk then lsiclk can be computed. lsiclk is needed for IWDG.
             */

            TIM21->CR1   &= ~TIM_CR1_CEN;
            TIM21->OR    = TIM21_OR_TI1_RMP_0 | TIM21_OR_TI1_RMP_2; /* Select LSI as TI1 */
            TIM21->EGR   = TIM_EGR_UG;
            TIM21->CR1   |= TIM_CR1_CEN;

            TIM21->CCER  = 0;
            TIM21->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_IC1PSC_1 | TIM_CCMR1_IC1PSC_0; /* Count every 8th pulse */
            TIM21->CCER  = TIM_CCER_CC1E;

            TIM21->SR = 0;
        
            while (!(TIM21->SR & TIM_SR_CC1IF))
            {
            }
        
            tc0 = TIM21->CCR1;
        
            for (n = 0, count = 0; n < (64 / 8); n++)
            {
                TIM21->SR = 0;
            
                while (!(TIM21->SR & TIM_SR_CC1IF))
                {
                }
            
                tc1 = TIM21->CCR1;
            
                if (tc1 < tc0)
                {
                    count += ((65536 + tc1) - tc0);
                }
                else
                {
                    count += (tc1 - tc0);
                }

                tc0 = tc1;
            }

            stm32l0_system_device.lsiclk = (16000000 * 64) / count;
        
            TIM21->CCER  = 0;
            TIM21->CCMR1 = 0;
            TIM21->CR1   = 0;
            TIM21->EGR   = 0;
            TIM21->OR    = 0;
            TIM21->SR    = 0;

            RCC->CSR &= ~RCC_CSR_LSION;

            RCC->APB2ENR &= ~RCC_APB2ENR_TIM21EN;
        }
        else
        {
            stm32l0_system_device.lsi |= 0x80;
        }

        /* Save cablibration to BKP4R for reuse when returning from STANDBY.
         */

        RTC->BKP4R = ((RTC->BKP4R & ~(STM32L0_RTC_BKP4R_LSICLK_MASK | STM32L0_RTC_BKP4R_HSEON_MASK | STM32L0_RTC_BKP4R_HSITRIM_MASK)) |
                      (((stm32l0_system_device.lsiclk + (STM32L0_RTC_BKP4R_LSICLK_SCALE / 2)) / STM32L0_RTC_BKP4R_LSICLK_SCALE) << STM32L0_RTC_BKP4R_LSICLK_SHIFT) |
                      (stm32l0_system_device.hseclk ? STM32L0_RTC_BKP4R_HSEON : 0) |
                      (((RCC->ICSCR & RCC_ICSCR_HSITRIM) >> RCC_ICSCR_HSITRIM_Pos) << STM32L0_RTC_BKP4R_HSITRIM_SHIFT));
    }

    /* Setup DBGMCU so that debugging (other than SLEEP/STOP/STANDBY) works like expected.
     */
    RCC->APB2ENR |= RCC_APB2ENR_DBGEN;

    DBGMCU->CR = (DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY);
    DBGMCU->APB1FZ = (DBGMCU_APB1_FZ_DBG_RTC_STOP | DBGMCU_APB1_FZ_DBG_IWDG_STOP | DBGMCU_APB1_FZ_DBG_LPTIMER_STOP);
    DBGMCU->APB2FZ = 0;

    RCC->APB2ENR &= ~RCC_APB2ENR_DBGEN;
    
    /* Setup CRS for HSI48 either via LSE
     */
    RCC->APB1ENR |= RCC_APB1ENR_CRSEN;

    CRS->CFGR = (((1465 -1) << CRS_CFGR_RELOAD_Pos) | (2 << CRS_CFGR_FELIM_Pos) | CRS_CFGR_SYNCSRC_0);
    
    RCC->APB1ENR &= ~RCC_APB1ENR_CRSEN;
    
    /* Setup some default sleep mode clock gating.
     */
    RCC->IOPSMENR = 0;

    RCC->AHBSMENR  &= ~(RCC_AHBSMENR_MIFSMEN |
                        RCC_AHBSMENR_SRAMSMEN);
    
    RCC->APB1SMENR &= ~(RCC_APB1SMENR_PWRSMEN);

    RCC->APB2SMENR &= ~(RCC_APB2SMENR_DBGSMEN | 
                        RCC_APB2SMENR_SYSCFGSMEN);

    /* Setup the independent clocks for the peripherals to HSI16, except
     * LUPART which is LSE. CLK48 is driven by HSI48 with CRS.
     */
    RCC->CCIPR = (RCC_CCIPR_LPTIM1SEL_0 | RCC_CCIPR_LPTIM1SEL_1 |   /* LSE   */
                  RCC_CCIPR_I2C1SEL_1 |                             /* HSI16 */
#if defined(STM32L072xx) || defined(STM32L082xx)
                  RCC_CCIPR_I2C3SEL_1 |                             /* HSI16 */
#endif /* STM32L072xx || STM32L082xx */
                  RCC_CCIPR_HSI48SEL);                              /* HSI48 */

    RCC->APB1ENR &= ~RCC_APB1ENR_PWREN;

    __armv6m_core_initialize();
    
    __stm32l0_gpio_initialize();
    __stm32l0_exti_initialize();
    __stm32l0_dma_initialize();
    __stm32l0_rtc_initialize();
    __stm32l0_lptim_initialize();
    __stm32l0_eeprom_initialize();

    stm32l0_system_sysclk_configure(hclk, pclk1, pclk2);

    __set_PRIMASK(primask);
}

bool stm32l0_system_sysclk_configure(uint32_t hclk, uint32_t pclk1, uint32_t pclk2)
{
    uint32_t primask, sysclk, hpre, ppre1, ppre2, msirange, pllcfg, latency, buspre, busspre;
    uint8_t pllsys;
    
    if (hclk <= 4200000)
    {
        /* Use MSI */

        /* Reading RTC requires HCLK >= 7 * RTCCLK. Hence clocks
         * below 229kHz will not work, and hence LPRUN will not 
         * be usable.
         */
        
        if      (hclk >= 4000000) { sysclk = 4194304; msirange = RCC_ICSCR_MSIRANGE_6; }
        else if (hclk >= 2000000) { sysclk = 2097152; msirange = RCC_ICSCR_MSIRANGE_5; }
        else if (hclk >= 1000000) { sysclk = 1048576; msirange = RCC_ICSCR_MSIRANGE_4; }
        else if (hclk >=  500000) { sysclk =  524288; msirange = RCC_ICSCR_MSIRANGE_3; }
        else                      { sysclk =  262144; msirange = RCC_ICSCR_MSIRANGE_2; }

        hclk   = sysclk;
        hpre   = RCC_CFGR_HPRE_DIV1;
        pllcfg = 0;

        latency = 0;

        pllsys = false;
    }
    else
    {
        if (hclk <= 16000000)
        {
            /* Use HSI16 */
                
            if (hclk <= 8000000)
            {
                sysclk = 16000000;
                hclk   = 8000000;
                hpre   = RCC_CFGR_HPRE_DIV2;
            }
            else
            {
                sysclk = 16000000;
                hclk   = 16000000;
                hpre   = RCC_CFGR_HPRE_DIV1;
            }
                
            msirange = 0;
            pllcfg   = 0;
                
            latency = ((hclk > 8000000) ? FLASH_ACR_LATENCY: 0);
                
            pllsys = false;
        }
        else
        {
            /* Use PLL via HSI16/HSE */
                
            if (stm32l0_system_device.options & STM32L0_SYSTEM_OPTION_HSE_PLL)
            {
                if      (stm32l0_system_device.hseclk == 4000000) { pllcfg = RCC_CFGR_PLLMUL24 | RCC_CFGR_PLLSRC_HSE; }
                else if (stm32l0_system_device.hseclk == 8000000) { pllcfg = RCC_CFGR_PLLMUL12 | RCC_CFGR_PLLSRC_HSE; }
                else                                              { pllcfg = RCC_CFGR_PLLMUL6  | RCC_CFGR_PLLSRC_HSE; }
            }
            else
            {
                pllcfg = RCC_CFGR_PLLMUL6 | RCC_CFGR_PLLSRC_HSI;
            }
                
            if (hclk <= 24000000)
            {
                sysclk = 24000000;
                    
                pllcfg |= RCC_CFGR_PLLDIV4;
            }
            else
            {
                sysclk = 32000000;
                    
                pllcfg |= RCC_CFGR_PLLDIV3;
            }
                
            hclk     = sysclk;
            hpre     = RCC_CFGR_HPRE_DIV1;
            msirange = 0;
                
            latency = FLASH_ACR_LATENCY;
                
            pllsys = true;
        }
    }
    
    if (pclk1)
    {
        if      (pclk1 >= hclk      ) { pclk1 = hclk;      ppre1 = RCC_CFGR_PPRE1_DIV1;  }
        else if (pclk1 >= (hclk / 2)) { pclk1 = hclk /  2; ppre1 = RCC_CFGR_PPRE1_DIV2;  }
        else if (pclk1 >= (hclk / 4)) { pclk1 = hclk /  4; ppre1 = RCC_CFGR_PPRE1_DIV4;  }
        else if (pclk1 >= (hclk / 8)) { pclk1 = hclk /  8; ppre1 = RCC_CFGR_PPRE1_DIV8;  }
        else                          { pclk1 = hclk / 16; ppre1 = RCC_CFGR_PPRE1_DIV16; }
    }
    else
    {
        if (hclk <= 2100000)
        {
            pclk1 = hclk;

            ppre1 = RCC_CFGR_PPRE1_DIV1;
        }
        else
        {
            pclk1 = hclk / 2;

            ppre1 = RCC_CFGR_PPRE1_DIV2;
        }
    }

    if (pclk2)
    {
        if      (pclk2 >= hclk      ) { pclk2 = hclk;      ppre2 = RCC_CFGR_PPRE2_DIV1;  }
        else if (pclk2 >= (hclk / 2)) { pclk2 = hclk /  2; ppre2 = RCC_CFGR_PPRE2_DIV2;  }
        else if (pclk2 >= (hclk / 4)) { pclk2 = hclk /  4; ppre2 = RCC_CFGR_PPRE2_DIV4;  }
        else if (pclk2 >= (hclk / 8)) { pclk2 = hclk /  8; ppre2 = RCC_CFGR_PPRE2_DIV8;  }
        else                          { pclk2 = hclk / 16; ppre2 = RCC_CFGR_PPRE2_DIV16; }
    }
    else
    {
        if (hclk <= 4200000)
        {
            pclk2 = hclk;

            ppre2 = RCC_CFGR_PPRE2_DIV1;
        }
        else
        {
            pclk2 = hclk / 2;

            ppre2 = RCC_CFGR_PPRE2_DIV2;
        }
    }

    buspre = hpre | ppre1 | ppre2;    
    
    if      (sysclk >= 32000000) { busspre = (RCC_CFGR_HPRE_DIV16 | RCC_CFGR_PPRE1_DIV1 | RCC_CFGR_PPRE2_DIV1); }
    else if (sysclk >= 16000000) { busspre = (RCC_CFGR_HPRE_DIV8  | RCC_CFGR_PPRE1_DIV1 | RCC_CFGR_PPRE2_DIV1); }
    else if (sysclk >=  8000000) { busspre = (RCC_CFGR_HPRE_DIV4  | RCC_CFGR_PPRE1_DIV1 | RCC_CFGR_PPRE2_DIV1); }
    else if (sysclk >=  4000000) { busspre = (RCC_CFGR_HPRE_DIV2  | RCC_CFGR_PPRE1_DIV1 | RCC_CFGR_PPRE2_DIV1); }
    else                         { busspre = buspre;                                                            }
        
    primask = __get_PRIMASK();

    __disable_irq();

    if (stm32l0_system_device.lock[STM32L0_SYSTEM_LOCK_RUN] ||
        ((stm32l0_system_device.reference & STM32L0_SYSTEM_REFERENCE_RNG)  && (hclk   <=  4200000)) ||
        ((stm32l0_system_device.reference & STM32L0_SYSTEM_REFERENCE_USB)  && (pclk1  <  16000000)) ||
        ((stm32l0_system_device.reference & STM32L0_SYSTEM_REFERENCE_I2C2) && (pclk1  <   4000000)))
    {
        __set_PRIMASK(primask);
        
        return false;
    }
    
    armv6m_systick_disable();

    if (stm32l0_system_device.sysclk != sysclk)
    {
        RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) | (RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV2);
        __DSB();
        __NOP();
        __NOP();
        __NOP();
        __NOP();

        RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    
        if (stm32l0_system_device.sysclk < sysclk)
        {
            /* Switch to Range 1 */
            PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS_0;
        
            while (PWR->CSR & PWR_CSR_VOSF)
            {
            }

            FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY;
        }

        if (pllsys)
        {
            if (stm32l0_system_device.options & STM32L0_SYSTEM_OPTION_HSE_PLL)
            {
                stm32l0_system_device.hse |= 0x80;
                stm32l0_system_device.hsi16 &= ~0x80;
            }
            else
            {
                stm32l0_system_device.hse &= ~0x80;
                stm32l0_system_device.hsi16 |= 0x80;
            }

            stm32l0_system_device.msi &= ~0x80;

            RCC->CFGR |= RCC_CFGR_STOPWUCK;
        }
        else
        {
            if (sysclk <= 4200000)
            {
                stm32l0_system_device.hse &= ~0x80;
                stm32l0_system_device.msi |= 0x80;
                stm32l0_system_device.hsi16 &= ~0x80;
                
                RCC->CFGR &= ~RCC_CFGR_STOPWUCK;
            }
            else
            {
                stm32l0_system_device.hse &= ~0x80;
                stm32l0_system_device.msi &= ~0x80;
                stm32l0_system_device.hsi16 |= 0x80;
                
                RCC->CFGR |= RCC_CFGR_STOPWUCK;
            }
        }
        
        if (stm32l0_system_device.hse)
        {
            if (stm32l0_system_device.options & STM32L0_SYSTEM_OPTION_HSE_BYPASS)
            {
                RCC->CR |= RCC_CR_HSEBYP;
            }
            else
            {
                RCC->CR |= RCC_CR_HSEON;
                
                while (!(RCC->CR & RCC_CR_HSERDY))
                {
                }
            }
        }
        
        if (stm32l0_system_device.hsi16)
        {
            if (!(RCC->CR & RCC_CR_HSION))
            {
                RCC->CR |= RCC_CR_HSION;
                
                while (!(RCC->CR & RCC_CR_HSIRDY))
                {
                }
            }
        }
        
        if (stm32l0_system_device.pllsys || pllsys)
        {
            if (!(RCC->CR & RCC_CR_HSION))
            {
                RCC->CR |= RCC_CR_HSION;
                
                while (!(RCC->CR & RCC_CR_HSIRDY))
                {
                }
            }

            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
                
            while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
            {
            }

            if (stm32l0_system_device.pllsys)
            {
                RCC->CR &= ~RCC_CR_PLLON;
                
                while (RCC->CR & RCC_CR_PLLRDY)
                {
                }
            }
        }

        if (stm32l0_system_device.msi)
        {
            RCC->ICSCR = (RCC->ICSCR & ~RCC_ICSCR_MSIRANGE) | msirange;
            
            RCC->CR |= RCC_CR_MSION;
    
            while (!(RCC->CR & RCC_CR_MSIRDY))
            {
            }
        }
        
        if (pllsys)
        {
            RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL | RCC_CFGR_PLLDIV)) | pllcfg;
            
            RCC->CR |= RCC_CR_PLLON;
            
            while (!(RCC->CR & RCC_CR_PLLRDY))
            {
            }
            
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
            
            while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
            {
            }
        }
        else
        {
            if (sysclk <= 4200000)
            {
                RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_MSI;
                    
                while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI)
                {
                }
            }
            else
            {
                RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
                    
                while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
                {
                }
            }
        }
        
        if (!stm32l0_system_device.hsi16)
        {
            RCC->CR &= ~RCC_CR_HSION;
        }

        if (!stm32l0_system_device.msi)
        {
            RCC->CR &= ~RCC_CR_MSION;
        }

        if (!stm32l0_system_device.hse)
        {
            if (stm32l0_system_device.options & STM32L0_SYSTEM_OPTION_HSE_BYPASS)
            {
                RCC->CR &= ~RCC_CR_HSEBYP;
            }
            else
            {
                RCC->CR &= ~RCC_CR_HSEON;
            }
        }

        /* Enter Range 3 */
        if ((sysclk <= 4200000) && !(stm32l0_system_device.reference & STM32L0_SYSTEM_REFERENCE_RANGE_1_2))
        {
            if ((PWR->CR & PWR_CR_VOS) != (PWR_CR_VOS_0 | PWR_CR_VOS_1))
            {
                PWR->CR = (PWR->CR & ~PWR_CR_VOS) | (PWR_CR_VOS_0 | PWR_CR_VOS_1);
                
                while (PWR->CSR & PWR_CSR_VOSF)
                {
                }
            }
        }
        else
        {
            /* Enter Range_2 */
            if ((sysclk <= 16000000) && !(stm32l0_system_device.reference & STM32L0_SYSTEM_REFERENCE_RANGE_1))
            {
                if ((PWR->CR & PWR_CR_VOS) != PWR_CR_VOS_1)
                {
                    PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS_1;
                    
                    while (PWR->CSR & PWR_CSR_VOSF)
                    {
                    }
                }
            }
        }

        FLASH->ACR = FLASH_ACR_PRFTEN | latency;

        RCC->APB1ENR &= ~RCC_APB1ENR_PWREN;
    }
    
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) | buspre;
    __DSB();
    __NOP();
    __NOP();
    __NOP();
    __NOP();

    SystemCoreClock = hclk;
    
    armv6m_systick_enable();

    stm32l0_system_device.sysclk = sysclk;
    stm32l0_system_device.hclk = hclk;
    stm32l0_system_device.pclk1 = pclk1;
    stm32l0_system_device.pclk2 = pclk2;

    stm32l0_system_device.pllsys = pllsys;
    stm32l0_system_device.busspre = busspre;

    stm32l0_system_notify(STM32L0_SYSTEM_NOTIFY_CLOCKS);

    __set_PRIMASK(primask);

    return true;
}

void stm32l0_system_mco_configure(uint32_t mco)
{
    uint32_t mcosel, mcopre;

    armv6m_atomic_and(&RCC->CFGR, ~(RCC_CFGR_MCOSEL | RCC_CFGR_MCOPRE));

    if ((stm32l0_system_device.mco & STM32L0_SYSTEM_MCO_SOURCE_MASK) != (mco & STM32L0_SYSTEM_MCO_SOURCE_MASK))
    {
        switch (stm32l0_system_device.mco & STM32L0_SYSTEM_MCO_SOURCE_MASK) {
        case STM32L0_SYSTEM_MCO_SOURCE_NONE:
        case STM32L0_SYSTEM_MCO_SOURCE_SYSCLK:
        case STM32L0_SYSTEM_MCO_SOURCE_MSI:
        case STM32L0_SYSTEM_MCO_SOURCE_PLL:
        case STM32L0_SYSTEM_MCO_SOURCE_LSE:
            break;

        case STM32L0_SYSTEM_MCO_SOURCE_HSE:
            if (stm32l0_system_device.hseclk)
            {
                stm32l0_system_hse_disable();
            }
            break;

        case STM32L0_SYSTEM_MCO_SOURCE_LSI:
            stm32l0_system_lsi_disable();
            break;

        case STM32L0_SYSTEM_MCO_SOURCE_HSI16:
            stm32l0_system_hsi16_disable();
            break;

#if defined(STM32L032xx) || defined(STM32L033xx) || defined(STM32L096xx)
        case STM32L0_SYSTEM_MCO_SOURCE_HSI48:
            stm32l0_system_hsi48_disable();
            break;
#endif
        }

        stm32l0_system_device.mco = mco;

        switch (mco & STM32L0_SYSTEM_MCO_SOURCE_MASK) {
        case STM32L0_SYSTEM_MCO_SOURCE_NONE:
        case STM32L0_SYSTEM_MCO_SOURCE_SYSCLK:
        case STM32L0_SYSTEM_MCO_SOURCE_MSI:
        case STM32L0_SYSTEM_MCO_SOURCE_PLL:
        case STM32L0_SYSTEM_MCO_SOURCE_LSE:
            break;

        case STM32L0_SYSTEM_MCO_SOURCE_HSE:
            if (stm32l0_system_device.hseclk)
            {
                stm32l0_system_hse_enable();
            }
            break;

        case STM32L0_SYSTEM_MCO_SOURCE_LSI:
            stm32l0_system_lsi_enable();
            break;

        case STM32L0_SYSTEM_MCO_SOURCE_HSI16:
            stm32l0_system_hsi16_enable();
            break;

#if defined(STM32L032xx) || defined(STM32L033xx) || defined(STM32L096xx)
        case STM32L0_SYSTEM_MCO_SOURCE_HSI48:
            stm32l0_system_hsi48_enable();
            break;
#endif
        }
    }

    mcosel = (mco & STM32L0_SYSTEM_MCO_SOURCE_MASK) >> STM32L0_SYSTEM_MCO_SOURCE_SHIFT;
    mcopre = (mco & STM32L0_SYSTEM_MCO_DIVIDE_MASK) >> STM32L0_SYSTEM_MCO_DIVIDE_SHIFT;

    armv6m_atomic_or(&RCC->CFGR, ((mcosel << RCC_CFGR_MCOSEL_Pos) | (mcopre << RCC_CFGR_MCOPRE_Pos)));
}

static void stm32l0_system_voltage_increase(void)
{
    uint32_t rcc_apb1enr;

    rcc_apb1enr = RCC->APB1ENR;

    if (!(rcc_apb1enr & RCC_APB1ENR_PWREN))
    {
        armv6m_atomic_or(&RCC->APB1ENR, RCC_APB1ENR_PWREN);
    }

    /* Leave Range 3 */
    if ((stm32l0_system_device.hclk <= 4200000) && (stm32l0_system_device.reference & STM32L0_SYSTEM_REFERENCE_RANGE_1_2))
    {
        armv6m_atomic_modify(&PWR->CR, PWR_CR_VOS, PWR_CR_VOS_1);
            
        while (PWR->CSR & PWR_CSR_REGLPF)
        {
        }
    }

    /* Leave Range_2 */
    if ((stm32l0_system_device.hclk <= 16000000) && (stm32l0_system_device.reference & STM32L0_SYSTEM_REFERENCE_RANGE_1))
    {
        armv6m_atomic_modify(&PWR->CR, PWR_CR_VOS, PWR_CR_VOS_0);
        
        while (PWR->CSR & PWR_CSR_VOSF)
        {
        }
    }

    if (!(rcc_apb1enr & RCC_APB1ENR_PWREN))
    {
        armv6m_atomic_and(&RCC->APB1ENR, ~RCC_APB1ENR_PWREN);
    }
}

static void stm32l0_system_voltage_decrease(void)
{
    uint32_t rcc_apb1enr;

    rcc_apb1enr = RCC->APB1ENR;

    if (!(rcc_apb1enr & RCC_APB1ENR_PWREN))
    {
        armv6m_atomic_or(&RCC->APB1ENR, RCC_APB1ENR_PWREN);
    }

    /* Enter Range_2 */
    if ((stm32l0_system_device.hclk <= 16000000) && !(stm32l0_system_device.reference & STM32L0_SYSTEM_REFERENCE_RANGE_1))
    {
        armv6m_atomic_modifyz(&PWR->CR, PWR_CR_VOS, (PWR_CR_VOS_1), &stm32l0_system_device.reference, STM32L0_SYSTEM_REFERENCE_RANGE_1);
        
        while (PWR->CSR & PWR_CSR_VOSF)
        {
            if (stm32l0_system_device.reference & STM32L0_SYSTEM_REFERENCE_RANGE_1)
            {
                break;
            }
        }
    }
    
    /* Enter Range 3 */
    if ((stm32l0_system_device.hclk <= 4200000) && !(stm32l0_system_device.reference & STM32L0_SYSTEM_REFERENCE_RANGE_1_2))
    {
        armv6m_atomic_modifyz(&PWR->CR, PWR_CR_VOS, (PWR_CR_VOS_0 | PWR_CR_VOS_1), &stm32l0_system_device.reference, STM32L0_SYSTEM_REFERENCE_RANGE_1_2);
        
        while (PWR->CSR & PWR_CSR_VOSF)
        {
            if (stm32l0_system_device.reference & STM32L0_SYSTEM_REFERENCE_RANGE_1_2)
            {
                break;
            }
        }
    }

    if (!(rcc_apb1enr & RCC_APB1ENR_PWREN))
    {
        armv6m_atomic_and(&RCC->APB1ENR, ~RCC_APB1ENR_PWREN);
    }
}

void stm32l0_system_hse_enable(void)
{
    armv6m_atomic_incb(&stm32l0_system_device.hse);
    
    if (stm32l0_system_device.options & STM32L0_SYSTEM_OPTION_HSE_BYPASS)
    {
        armv6m_atomic_or(&RCC->CR, RCC_CR_HSEBYP);
    }
    else
    {
        armv6m_atomic_or(&RCC->CR, RCC_CR_HSEON);
        
        while (!(RCC->CR & RCC_CR_HSERDY))
        {
        }
    }
}

void stm32l0_system_hse_disable(void)
{
    armv6m_atomic_decb(&stm32l0_system_device.hse);

    if (stm32l0_system_device.options & STM32L0_SYSTEM_OPTION_HSE_BYPASS)
    {
        armv6m_atomic_andzb(&RCC->CR, ~RCC_CR_HSEBYP, &stm32l0_system_device.hse);
    }
    else
    {
        armv6m_atomic_andzb(&RCC->CR, ~RCC_CR_HSEON, &stm32l0_system_device.hse);
    }
}

void stm32l0_system_lsi_enable(void)
{
    armv6m_atomic_incb(&stm32l0_system_device.lsi);
    armv6m_atomic_or(&RCC->CSR, RCC_CSR_LSION);
    
    while (!(RCC->CSR & RCC_CSR_LSIRDY))
    {
    }
}

void stm32l0_system_lsi_disable(void)
{
    armv6m_atomic_decb(&stm32l0_system_device.lsi);
    armv6m_atomic_andzb(&RCC->CSR, ~RCC_CSR_LSION, &stm32l0_system_device.lsi);
}

void stm32l0_system_hsi16_enable(void)
{
    armv6m_atomic_incb(&stm32l0_system_device.hsi16);
    armv6m_atomic_or(&RCC->CR, RCC_CR_HSION);
    
    while (!(RCC->CR & RCC_CR_HSIRDY))
    {
    }
}

void stm32l0_system_hsi16_disable(void)
{
    armv6m_atomic_decb(&stm32l0_system_device.hsi16);
    armv6m_atomic_andzb(&RCC->CR, ~RCC_CR_HSION, &stm32l0_system_device.hsi16);
}

void stm32l0_system_hsi48_enable(void)
{
    stm32l0_system_vrefint_enable();

    armv6m_atomic_incb(&stm32l0_system_device.hsi48);
    
    armv6m_atomic_or(&RCC->APB1ENR, RCC_APB1ENR_CRSEN);
    armv6m_atomic_or(&SYSCFG->CFGR3, SYSCFG_CFGR3_ENREF_HSI48);
    armv6m_atomic_or(&RCC->CRRCR, RCC_CRRCR_HSI48ON);

    while(!(RCC->CRRCR & RCC_CRRCR_HSI48RDY))
    {
    }

    armv6m_atomic_or(&CRS->CR, (CRS_CR_AUTOTRIMEN | CRS_CR_CEN));
}

void stm32l0_system_hsi48_disable(void)
{
    armv6m_atomic_decb(&stm32l0_system_device.hsi48);

    armv6m_atomic_andzb(&CRS->CR, ~(CRS_CR_AUTOTRIMEN | CRS_CR_CEN), &stm32l0_system_device.hsi48);
    armv6m_atomic_andzb(&RCC->CRRCR, ~RCC_CRRCR_HSI48ON, &stm32l0_system_device.hsi48);
    armv6m_atomic_andzb(&SYSCFG->CFGR3, ~SYSCFG_CFGR3_ENREF_HSI48, &stm32l0_system_device.hsi48);
    armv6m_atomic_andzb(&RCC->APB1ENR, ~RCC_APB1ENR_CRSEN, &stm32l0_system_device.hsi48);

    stm32l0_system_vrefint_disable();
}

void stm32l0_system_clk48_enable(void)
{
    armv6m_atomic_incb(&stm32l0_system_device.clk48);

    stm32l0_system_voltage_increase();
    stm32l0_system_hsi48_enable();
}

void stm32l0_system_clk48_disable(void)
{
    armv6m_atomic_decb(&stm32l0_system_device.clk48);

    stm32l0_system_hsi48_disable();
    stm32l0_system_voltage_decrease();
}

void stm32l0_system_vrefint_enable(void)
{
    armv6m_atomic_incb(&stm32l0_system_device.vrefint);

    armv6m_atomic_or(&SYSCFG->CFGR3, SYSCFG_CFGR3_EN_VREFINT);

    while (!(SYSCFG->CFGR3 & SYSCFG_CFGR3_VREFINT_RDYF))
    {
    }
}

void stm32l0_system_vrefint_disable(void)
{
    armv6m_atomic_decb(&stm32l0_system_device.vrefint);

    armv6m_atomic_andzb(&SYSCFG->CFGR3, ~SYSCFG_CFGR3_EN_VREFINT, &stm32l0_system_device.vrefint);
}

uint32_t stm32l0_system_reset_cause(void)
{
    return stm32l0_system_device.reset;
}

uint32_t stm32l0_system_wakeup_reason(void)
{
    return stm32l0_system_device.wakeup;
}

uint32_t stm32l0_system_lseclk(void)
{
    return stm32l0_system_device.lseclk;
}

uint32_t stm32l0_system_hseclk(void)
{
    return stm32l0_system_device.hseclk;
}

uint32_t stm32l0_system_lsiclk(void)
{
    return stm32l0_system_device.lsiclk;
}

uint32_t stm32l0_system_sysclk(void)
{
    return stm32l0_system_device.sysclk;
}

uint32_t stm32l0_system_hclk(void)
{
    return stm32l0_system_device.hclk;
}

uint32_t stm32l0_system_pclk1(void)
{
    return stm32l0_system_device.pclk1;
}

uint32_t stm32l0_system_pclk2(void)
{
    return stm32l0_system_device.pclk2;
}

uint64_t stm32l0_system_serial(void)
{
    uint32_t uid[3];

    /* STM32 BOOTLOADER uses a different UID system
     * than documented in the reference ...
     */
    uid[0] = *((const uint32_t*)(UID_BASE + 0x00));
    uid[1] = *((const uint32_t*)(UID_BASE + 0x04));
    uid[2] = *((const uint32_t*)(UID_BASE + 0x08));

    /* This crummy value is what the USB/DFU bootloader uses.
     */
    return (((uint64_t)(uid[0] + uid[2]) << 16) | (uint64_t)(uid[1] >> 16));
}

void stm32l0_system_uid(uint32_t *uid)
{
    uid[0] = *((const uint32_t*)(UID_BASE + 0x00));
    uid[1] = *((const uint32_t*)(UID_BASE + 0x04));
    uid[2] = *((const uint32_t*)(UID_BASE + 0x14));
}

void stm32l0_system_periph_reset(unsigned int periph)
{
    __armv6m_atomic_or(stm32l0_system_xlate_RSTR[periph], stm32l0_system_xlate_RSTMSK[periph]);
    __armv6m_atomic_and(stm32l0_system_xlate_RSTR[periph], ~stm32l0_system_xlate_RSTMSK[periph]);
}

void stm32l0_system_periph_enable(unsigned int periph)
{
    __armv6m_atomic_or(stm32l0_system_xlate_ENR[periph], stm32l0_system_xlate_ENMSK[periph]);
    *stm32l0_system_xlate_ENR[periph];
}

void stm32l0_system_periph_disable(unsigned int periph)
{
    __armv6m_atomic_and(stm32l0_system_xlate_ENR[periph], ~stm32l0_system_xlate_ENMSK[periph]);
}

void stm32l0_system_swd_enable(void)
{
    armv6m_atomic_or(&RCC->APB2ENR, RCC_APB2ENR_DBGEN);
    RCC->APB2ENR;

    DBGMCU->CR = (DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY);
    DBGMCU->APB1FZ = (DBGMCU_APB1_FZ_DBG_RTC_STOP | DBGMCU_APB1_FZ_DBG_IWDG_STOP | DBGMCU_APB1_FZ_DBG_LPTIMER_STOP);
    DBGMCU->APB2FZ = 0;

    __stm32l0_gpio_swd_enable();

    stm32l0_system_reference(STM32L0_SYSTEM_REFERENCE_SWD);
}

void stm32l0_system_swd_disable(void)
{
    stm32l0_system_unreference(STM32L0_SYSTEM_REFERENCE_SWD);

    __stm32l0_gpio_swd_disable();

    armv6m_atomic_or(&RCC->APB2ENR, RCC_APB2ENR_DBGEN);
    RCC->APB2ENR;

    DBGMCU->CR = 0;
    DBGMCU->APB1FZ = 0;
    DBGMCU->APB2FZ = 0;

    armv6m_atomic_and(&RCC->APB2ENR, ~RCC_APB2ENR_DBGEN);
}
void stm32l0_system_register(stm32l0_system_notify_t *notify, stm32l0_system_callback_t callback, void *context, uint32_t mask)
{
    stm32l0_system_notify_t **pp_entry, *entry;

    for (pp_entry = &stm32l0_system_device.notify, entry = *pp_entry; entry; pp_entry = &entry->next, entry = *pp_entry)
    {
    }

    *pp_entry = notify;

    notify->next = NULL;
    notify->callback = callback;
    notify->context = context;
    notify->mask = mask;
}

void stm32l0_system_unregister(stm32l0_system_notify_t *notify)
{
    stm32l0_system_notify_t **pp_entry, *entry;

    notify->mask = 0;
    notify->callback = NULL;

    for (pp_entry = &stm32l0_system_device.notify, entry = *pp_entry; entry; pp_entry = &entry->next, entry = *pp_entry)
    {
        if (entry == notify)
        {
            *pp_entry = notify->next;
            
            notify->next = NULL;

            break;
        }
    }
}

void stm32l0_system_notify(uint32_t notify)
{
    stm32l0_system_notify_t *entry;

    for (entry = stm32l0_system_device.notify; entry; entry = entry->next)
    {
        if (entry->mask & notify)
        {
            (*entry->callback)(entry->context, entry->mask & notify);
        }
    }
}

void stm32l0_system_lock(uint32_t lock)
{
    __armv6m_atomic_incb(&stm32l0_system_device.lock[lock]);
}

void stm32l0_system_unlock(uint32_t lock)
{
    __armv6m_atomic_decb(&stm32l0_system_device.lock[lock]);
}

void stm32l0_system_reference(uint32_t reference)
{
    __armv6m_atomic_or(&stm32l0_system_device.reference, reference);
}

void stm32l0_system_unreference(uint32_t reference)
{
    __armv6m_atomic_and(&stm32l0_system_device.reference, ~reference);
}

void stm32l0_system_sleep(uint32_t policy, uint32_t mask, uint32_t timeout)
{
    uint32_t primask, rcc_cfgr, rcc_apb1enr, pwr_cr;
    stm32l0_gpio_stop_state_t gpio_stop_state;

    if (timeout != STM32L0_SYSTEM_TIMEOUT_NONE)
    {
        if (!(stm32l0_system_device.events & mask))
        {
            if (timeout != STM32L0_SYSTEM_TIMEOUT_FOREVER)
            {
                mask |= STM32L0_SYSTEM_EVENT_TIMEOUT;

                if (stm32l0_system_device.timeout.callback == NULL)
                {
                    stm32l0_rtc_timer_create(&stm32l0_system_device.timeout, (stm32l0_rtc_timer_callback_t)stm32l0_system_wakeup, (void*)STM32L0_SYSTEM_EVENT_TIMEOUT);
                }
                
                stm32l0_rtc_timer_start(&stm32l0_system_device.timeout, stm32l0_rtc_millis_to_clock(timeout), STM32L0_RTC_TIMER_MODE_RELATIVE);
            }

            if (!(stm32l0_system_device.events & mask))
            {
                if (policy >= STM32L0_SYSTEM_POLICY_SLEEP)
                {
                    stm32l0_system_notify(STM32L0_SYSTEM_NOTIFY_SLEEP);
                }

                while (!(stm32l0_system_device.events & mask))
                {
                    if ((policy <= STM32L0_SYSTEM_POLICY_RUN) || stm32l0_system_device.lock[STM32L0_SYSTEM_LOCK_RUN])
                    {
                        __WFE();
                    }
                    else
                    {
                        primask = __get_PRIMASK();

                        __disable_irq();

                        if (!stm32l0_system_device.lock[STM32L0_SYSTEM_LOCK_RUN])
                        {
                            if ((policy <= STM32L0_SYSTEM_POLICY_SLEEP) || stm32l0_system_device.lock[STM32L0_SYSTEM_LOCK_SLEEP])
                            {
                                if (stm32l0_system_device.reference & STM32L0_SYSTEM_REFERENCE_SWD)
                                {
                                    __WFI();
                                }
                                else
                                {
                                    __stm32l0_dma_sleep_enter();

                                    rcc_cfgr = RCC->CFGR;                                   

                                    RCC->CFGR = (rcc_cfgr & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) | stm32l0_system_device.busspre;
                                    __DSB();
                                    __NOP();
                                    __NOP();
                                    __NOP();
                                    __NOP();

                                    __WFI();
                                    __NOP();
                                    __NOP();
                                    __NOP();
                                    __NOP();

                                    RCC->CFGR = rcc_cfgr;
                                    __DSB();
                                    __NOP();
                                    __NOP();
                                    __NOP();
                                    __NOP();
                                    
                                    __stm32l0_dma_sleep_leave();
                                }
                            }
                            else
                            {
                                if (!(SCB->ICSR & SCB_ICSR_ISRPENDING_Msk))
                                {
                                    stm32l0_system_notify(STM32L0_SYSTEM_NOTIFY_STOP_ENTER);

                                    if (!(SCB->ICSR & SCB_ICSR_ISRPENDING_Msk))
                                    {
                                        __stm32l0_exti_stop_enter();
                                        __stm32l0_gpio_stop_enter(&gpio_stop_state);

                                        if (!(SCB->ICSR & SCB_ICSR_ISRPENDING_Msk))
                                        {
                                            rcc_apb1enr = RCC->APB1ENR;

                                            RCC->APB1ENR = rcc_apb1enr | RCC_APB1ENR_PWREN;

                                            if (stm32l0_system_device.hsi48)
                                            {
                                                CRS->CR &= ~(CRS_CR_AUTOTRIMEN | CRS_CR_CEN);

                                                RCC->CRRCR &= ~RCC_CRRCR_HSI48ON;
                                            }

                                            pwr_cr = PWR->CR;
                                            
                                            /* The lowpower voltage regulator adds 3.3uS wakeup time, plus it prevents HSIKERON, 
                                             * which may be needed by USART1/USART2/LPUART to wakeup. In those cases do not
                                             * enable the low power voltage regulator in stop mode.
                                             */
                                            if (!stm32l0_system_device.lock[STM32L0_SYSTEM_LOCK_REGULATOR])
                                            {
                                                pwr_cr |= PWR_CR_LPSDSR;
                                            }
                    
                                            if (!stm32l0_system_device.vrefint)
                                            {
                                                pwr_cr |= (PWR_CR_FWU | PWR_CR_ULP);
                                            }

                                            /* Clear WUF flag */
                                            pwr_cr |= PWR_CR_CWUF;
                    
                                            /* Select STOP */
                                            pwr_cr &= ~PWR_CR_PDDS;

                                            PWR->CR = pwr_cr;
                                            
                                            if (!(SCB->ICSR & SCB_ICSR_ISRPENDING_Msk))
                                            {
                                                SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

                                                if (stm32l0_system_device.reference & STM32L0_SYSTEM_REFERENCE_SWD)
                                                {
                                                    __WFI();
                                                }
                                                else
                                                {
                                                    FLASH->ACR |= FLASH_ACR_SLEEP_PD;
                        
                                                    __WFI();
                                                    __NOP();
                                                    __NOP();
                                                    __NOP();
                                                    __NOP();
                        
                                                    FLASH->ACR &= ~FLASH_ACR_SLEEP_PD;
                                                }

                                                SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
                                                
                                                /* Clear ULP to enable VREFINT, disable lowpower voltage regulator */
                                                PWR->CR = pwr_cr;

                                                if (stm32l0_system_device.hse)
                                                {
                                                    if (stm32l0_system_device.options & STM32L0_SYSTEM_OPTION_HSE_BYPASS)
                                                    {
                                                        RCC->CR |= RCC_CR_HSEBYP;
                                                    }
                                                    else
                                                    {
                                                        RCC->CR |= RCC_CR_HSEON;
                                                        
                                                        while (!(RCC->CR & RCC_CR_HSERDY))
                                                        {
                                                        }
                                                    }
                                                }

                                                if (stm32l0_system_device.pllsys)
                                                {
                                                    RCC->CR |= RCC_CR_PLLON;
                                                        
                                                    while (!(RCC->CR & RCC_CR_PLLRDY))
                                                    {
                                                    }
                                                    
                                                    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
                                                    
                                                    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
                                                    {
                                                    }
                                                }
                                            }
                                            else
                                            {
                                                /* Clear ULP to enable VREFINT, disable lowpower voltage regulator */
                                                PWR->CR = pwr_cr;
                                            }
                                            
                                            if (!stm32l0_system_device.hsi16)
                                            {
                                                RCC->CR &= ~RCC_CR_HSION;
                                            }
                                            
                                            if (stm32l0_system_device.hsi48)
                                            {
                                                RCC->CRRCR |= RCC_CRRCR_HSI48ON;

                                                while(!(RCC->CRRCR & RCC_CRRCR_HSI48RDY))
                                                {
                                                }
                                                
                                                CRS->CR |= (CRS_CR_AUTOTRIMEN | CRS_CR_CEN);
                                            }
                                            
                                            RCC->APB1ENR = rcc_apb1enr;
                                        }
                                        
                                        __stm32l0_gpio_stop_leave(&gpio_stop_state);
                                        __stm32l0_exti_stop_leave();
                                    }
                                    
                                    stm32l0_system_notify(STM32L0_SYSTEM_NOTIFY_STOP_LEAVE);
                                }
                            }
                        }
                        
                        __set_PRIMASK(primask);
                    }
                }
            }
            
            if (timeout != STM32L0_SYSTEM_TIMEOUT_FOREVER)
            {
                stm32l0_rtc_timer_stop(&stm32l0_system_device.timeout);
            }
        }
    }
    
    __armv6m_atomic_and(&stm32l0_system_device.events, ~mask);
}

void stm32l0_system_wakeup(uint32_t events)
{
    __armv6m_atomic_or(&stm32l0_system_device.events, events);
}

void stm32l0_system_standby(uint32_t control, uint32_t timeout)
{
    uint32_t primask;

    if (timeout == STM32L0_SYSTEM_TIMEOUT_NONE)
    {
        return;
    }
    
    primask = __get_PRIMASK();
    
    __disable_irq();
    
    if (stm32l0_system_device.lock[STM32L0_SYSTEM_LOCK_RUN] ||
        stm32l0_system_device.lock[STM32L0_SYSTEM_LOCK_SLEEP] ||
        stm32l0_system_device.lock[STM32L0_SYSTEM_LOCK_DEEPSLEEP])
    {
        __set_PRIMASK(primask);
                        
        return;
    }
    
    stm32l0_system_notify(STM32L0_SYSTEM_NOTIFY_STANDBY);

    stm32l0_rtc_standby(control, timeout);
    
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    
    if (control & STM32L0_SYSTEM_STANDBY_PIN_1_RISING)
    {
        PWR->CSR |= PWR_CSR_EWUP1;
    }
    
    if (control & STM32L0_SYSTEM_STANDBY_PIN_2_RISING)
    {
        PWR->CSR |= PWR_CSR_EWUP2;
    }
    
    /* Enable the low power voltage regulator */
    PWR->CR |= PWR_CR_LPSDSR;

    /* COMP cannot wake from STANDBY, so nuke VREFINT
     */
    SYSCFG->CFGR3 &= ~SYSCFG_CFGR3_EN_VREFINT;

    PWR->CR |= PWR_CR_ULP;
    
    /* Clear WUF flag */
    PWR->CR |= PWR_CR_CWUF;
    
    /* Select STANDBY */
    PWR->CR |= PWR_CR_PDDS;

    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    
    __DSB();
    __SEV();
    __WFE();
    
    while (1)
    {
        __WFE();
    }
}

void stm32l0_system_hook(stm32l0_system_fatal_callback_t callback)
{
    stm32l0_system_device.callback = callback;
}

void stm32l0_system_fatal(void)
{
    __disable_irq();

    if (stm32l0_system_device.reference & STM32L0_SYSTEM_REFERENCE_SWD)
    {
        __BKPT();
    }

    __set_MSP( (uint32_t)&__StackTop );
    __set_PSP( (uint32_t)&__StackTop );
        
    while (stm32l0_system_device.callback)
    {
        (*stm32l0_system_device.callback)();
    }

    *((volatile uint32_t*)STM32L0_CRASH_SIGNATURE_ADDRESS) = STM32L0_CRASH_SIGNATURE_DATA;
    
    stm32l0_rtc_reset();

    NVIC_SystemReset();
    
    while (1)
    {
    }
}

void stm32l0_system_reset(void)
{
    while (stm32l0_system_device.lock[STM32L0_SYSTEM_LOCK_EEPROM])
    {
        __WFE();
    }

    __disable_irq();

    stm32l0_system_notify(STM32L0_SYSTEM_NOTIFY_RESET);

    stm32l0_rtc_reset();

    NVIC_SystemReset();
    
    while (1)
    {
    }
}

void stm32l0_system_dfu(void)
{
    while (stm32l0_system_device.lock[STM32L0_SYSTEM_LOCK_EEPROM])
    {
        __WFE();
    }

    __disable_irq();

    stm32l0_system_notify(STM32L0_SYSTEM_NOTIFY_DFU);

    /* Set the BCK bit to flag a reboot into the booloader */
    RTC->CR |= RTC_CR_BKP;

    stm32l0_rtc_reset();

    NVIC_SystemReset();
    
    while (1)
    {
    }
}

static void __empty() { }

void __stm32l0_lptim_initialize(void) __attribute__ ((weak, alias("__empty")));
void __stm32l0_eeprom_initialize(void) __attribute__ ((weak, alias("__empty")));

void armv6m_systick_enable(void) __attribute__ ((weak, alias("__empty")));
void armv6m_systick_disable(void) __attribute__ ((weak, alias("__empty")));

void NMI_Handler(void) __attribute__ ((weak, alias("stm32l0_system_fatal")));
void HardFault_Handler(void) __attribute__ ((weak, alias("stm32l0_system_fatal")));
void PendSV_Handler(void) __attribute__ ((weak, alias("stm32l0_system_fatal")));
void SVC_Handler(void) __attribute__ ((weak, alias("stm32l0_system_fatal")));
void SysTick_Handler(void) __attribute__ ((weak, alias("stm32l0_system_fatal")));
void WWDG_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void PVD_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void RTC_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void FLASH_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void RCC_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void EXTI0_1_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void EXTI2_3_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void EXTI4_15_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void TSC_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void DMA1_Channel1_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void DMA1_Channel2_3_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void DMA1_Channel4_5_6_7_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void ADC1_COMP_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void LPTIM1_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void USART4_5_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void TIM2_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void TIM3_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void TIM6_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void TIM7_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void TIM21_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void I2C3_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void TIM22_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void I2C1_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void I2C2_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void SPI1_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void SPI2_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void USART1_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void USART2_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void LPUART1_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
void USB_IRQHandler(void)  __attribute__ ((weak, alias("stm32l0_system_fatal")));
