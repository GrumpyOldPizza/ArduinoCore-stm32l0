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

#include "armv6m.h"
#include "stm32l0_gpio.h"
#include "stm32l0_rtc.h"
#include "stm32l0_eeprom.h"
#include "stm32l0_system.h"

#undef abs
#define abs(_i)  (((_i) >= 0) ? (_i) : -(_i))

uint32_t SystemCoreClock = 16000000;

typedef struct _stm32l0_system_device_t {
    uint16_t                  reset;
    uint16_t                  wakeup;
    uint32_t                  lseclk;
    uint32_t                  hseclk;
    uint32_t                  lsiclk;
    uint32_t                  sysclk;
    uint32_t                  hclk;
    uint32_t                  pclk1;
    uint32_t                  pclk2;
    uint8_t                   lsi;
    uint8_t                   hsi16;
    uint8_t                   hsi48;
    uint8_t                   mco;
    volatile uint8_t          policy;
    volatile uint8_t          event;
    volatile uint8_t          lock[STM32L0_SYSTEM_LOCK_COUNT];
    volatile uint32_t         reference;
    stm32l0_system_notify_t   *notify;
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

    /* Unlock RTC early.
     */
    RTC->WPR = 0xca;
    RTC->WPR = 0x53;

    /* If RTC_CR_BCK is set it means the reset was triggered to
     * branch throu to the STM32 BOOTLOADER.
     */
    if (RTC->CR & RTC_CR_BCK)
    {
        RTC->CR &= ~RTC_CR_BCK;
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
        __asm__ volatile ("   ldr     r2, =0x00000000                \n"
                          "   ldr     r0, [r2, #0]                   \n"
                          "   ldr     r1, [r2, #4]                   \n"
                          "   msr     MSP, r0                        \n"
                          "   dsb                                    \n"
                          "   isb                                    \n"
                          "   bx      r1                             \n");
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

    RCC->APB1ENR &= ~RCC_APB1ENR_PWREN;

    SystemCoreClock = 16000000;
}

void stm32l0_system_initialize(uint32_t hclk, uint32_t pclk1, uint32_t pclk2, uint32_t lseclk, uint32_t hseclk, uint32_t option)
{
    uint32_t primask, n, count, hsiclk, hsitrim, clk, trim, tc0, tc1;

    primask = __get_PRIMASK();

    __disable_irq();

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    if (PWR->CSR & PWR_CSR_SBF)
    {
        stm32l0_system_device.reset = STM32L0_SYSTEM_RESET_STANDBY;
        stm32l0_system_device.wakeup = STM32L0_SYSTEM_WAKEUP_RESET;
        
        if (PWR->CSR & PWR_CSR_WUF)
        {
            if (RTC->ISR & (RTC_ISR_ALRAF | RTC_ISR_ALRBF))
            {
                if (RTC->ISR & RTC_ISR_ALRAF)
                {
                    stm32l0_system_device.wakeup |= STM32L0_SYSTEM_WAKEUP_ALARM;
                }
                
                if (RTC->ISR & RTC_ISR_ALRBF)
                {
                    stm32l0_system_device.wakeup |= STM32L0_SYSTEM_WAKEUP_TIMEOUT;
                }
            }
            else
            {
                stm32l0_system_device.wakeup = STM32L0_SYSTEM_WAKEUP_WKUP;
            }
        }
        else
        {
            if (RCC->CSR & RCC_CSR_IWDGRSTF)
            {
                stm32l0_system_device.reset = STM32L0_SYSTEM_WAKEUP_WATCHDOG;
            }
        }
    }
    else
    {
        stm32l0_system_device.reset = STM32L0_SYSTEM_RESET_POWERON;
        stm32l0_system_device.wakeup = 0;
        
        if (RCC->CSR & (RCC_CSR_LPWRRSTF | RCC_CSR_OBLRSTF))
       {
            stm32l0_system_device.reset = STM32L0_SYSTEM_RESET_OTHER;
        }
        
        else if (RCC->CSR & RCC_CSR_FWRSTF)
        {
            stm32l0_system_device.reset = STM32L0_SYSTEM_RESET_FIREWALL;
        }
        
        else if (RCC->CSR & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF))
        {
            stm32l0_system_device.reset = STM32L0_SYSTEM_RESET_WATCHDOG;
        }
        
        else if (RCC->CSR & RCC_CSR_SFTRSTF)
        {
            stm32l0_system_device.reset = STM32L0_SYSTEM_RESET_SOFTWARE;
        }
        
        else if (RCC->CSR & RCC_CSR_PINRSTF)
        {
            stm32l0_system_device.reset = STM32L0_SYSTEM_RESET_EXTERNAL;
        }
    }
    
    RCC->CSR |= RCC_CSR_RMVF;
    RCC->CSR &= ~RCC_CSR_RMVF;
    
    PWR->CR |= (PWR_CR_CSBF | PWR_CR_CWUF);
    
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
            if (option & STM32L0_SYSTEM_OPTION_HSE_BYPASS)
            {
                RCC->CR |= RCC_CR_HSEBYP;
            }
        }
        else
        {
            stm32l0_system_device.hseclk = 0;
        }

        stm32l0_system_device.lsiclk = 29000 + (RTC->BKP4R & 0x00003fff); /* 29k to 45k, typical 37k */

        RCC->ICSCR = ((RCC->ICSCR & ~RCC_ICSCR_HSITRIM) | (((RTC->BKP4R & 0x000f8000) >> 15) << RCC_ICSCR_HSITRIM_Pos));
    }
    else
    {
        if (lseclk)
        {
            if (option & STM32L0_SYSTEM_OPTION_LSE_BYPASS)
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
            if (option & STM32L0_SYSTEM_OPTION_HSE_BYPASS)
            {
                RCC->CR |= RCC_CR_HSEBYP;
            }

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

                    RCC->CR &= ~RCC_CR_HSEBYP;
                    break;
                }
            }
            
            RCC->CR &= ~RCC_CR_HSEON;
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
         *
         * 0x00003fff is lsiclk - 29000
         * 0x00004000 is HSE present
         * 0x000f8000 is HSITRIM
         * 0xfff00000 is RTC subseconds offset
         */

        RTC->BKP4R = (RTC->BKP4R & 0xfff00000) | ((stm32l0_system_device.lsiclk - 29000) & 0x00003fff);

        if (stm32l0_system_device.hseclk) 
        {
            RTC->BKP4R |= 0x00004000;
        }

        RTC->BKP4R |= (((RCC->ICSCR & RCC_ICSCR_HSITRIM) >> RCC_ICSCR_HSITRIM_Pos) << 15);
    }

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

    __stm32l0_gpio_initialize();
    __stm32l0_rtc_initialize();
    __stm32l0_eeprom_initialize();

    stm32l0_system_sysclk_configure(hclk, pclk1, pclk2);

    stm32l0_system_device.policy = STM32L0_SYSTEM_POLICY_STOP;

    __set_PRIMASK(primask);
}

bool stm32l0_system_sysclk_configure(uint32_t hclk, uint32_t pclk1, uint32_t pclk2)
{
    stm32l0_system_notify_t *entry;
    uint32_t primask, sysclk, hpre, ppre1, ppre2, msirange, pllmul, plldiv;

    if (hclk < 8000000)
    {
        /* Range 3, use MSI */

        /* Reading RTC requires HCLK >= 7 * RTCCLK. Hence clocks
         * below 229kHz will not work, and hence LPRUN will not 
         * be usable.
         */
        
        if      (hclk >= 4000000) { sysclk = 4194304; msirange = RCC_ICSCR_MSIRANGE_6; }
        else if (hclk >= 2000000) { sysclk = 2097152; msirange = RCC_ICSCR_MSIRANGE_5; }
        else if (hclk >= 1000000) { sysclk = 1048576; msirange = RCC_ICSCR_MSIRANGE_4; }
        else if (hclk >=  500000) { sysclk =  524288; msirange = RCC_ICSCR_MSIRANGE_3; }
        else                      { sysclk =  262144; msirange = RCC_ICSCR_MSIRANGE_2; }

        hclk = sysclk;

        hpre   = RCC_CFGR_HPRE_DIV1;
        pllmul = 0;
        plldiv = 0;
    }
    else
    {
        if (hclk < 32000000)
        {
            /* Range 2, use HSI16 or PLL on HSE */
                
            sysclk   = 16000000;
            hclk     = sysclk;
                
            hpre     = RCC_CFGR_HPRE_DIV1;
            msirange = 0;

            if (stm32l0_system_device.hseclk == 8000000)
            {
                pllmul   = RCC_CFGR_PLLMUL6;
                plldiv   = RCC_CFGR_PLLDIV3;
            }
            else
            {
                pllmul   = RCC_CFGR_PLLMUL3;
                plldiv   = RCC_CFGR_PLLDIV3;
            }
        }
        else
        {
            /* Range 1, use PLL on HSI16 or HSE */
            
            sysclk   = 32000000;
            hclk     = sysclk;
            
            hpre     = RCC_CFGR_HPRE_DIV1;
            msirange = 0;
            
            if (stm32l0_system_device.hseclk == 8000000)
            {
                pllmul   = RCC_CFGR_PLLMUL12;
                plldiv   = RCC_CFGR_PLLDIV3;
            }
            else
            {
                pllmul   = RCC_CFGR_PLLMUL6;
                plldiv   = RCC_CFGR_PLLDIV3;
            }
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
        if (hclk <= 4200000)
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
        if (hclk < 8000000)
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

    primask = __get_PRIMASK();

    __disable_irq();

    if (stm32l0_system_device.lock[STM32L0_SYSTEM_LOCK_CLOCKS] ||
        ((stm32l0_system_device.reference & STM32L0_SYSTEM_REFERENCE_USB)  && (pclk1  < 16000000)) ||
        ((stm32l0_system_device.reference & STM32L0_SYSTEM_REFERENCE_I2C2) && (pclk1  <  4000000)))
    {
        __set_PRIMASK(primask);
        
        return false;
    }

    armv6m_systick_disable();

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    
    FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY;

    /* Switch to Range 1 */
    PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS_0;
        
    while (PWR->CSR & PWR_CSR_VOSF)
    {
    }

    /* Switch to HSI16 as system clock */
    RCC->CR |= RCC_CR_HSION;
        
    while (!(RCC->CR & RCC_CR_HSIRDY))
    {
    }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;

    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
    {
    }

    /* Turn MSI off so that the range can be switched */
    RCC->CR &= ~RCC_CR_MSION;

    /* Turn PLL off so that the range can be switched */
    RCC->CR &= ~RCC_CR_PLLON;
    
    while (RCC->CR & RCC_CR_PLLRDY)
    {
    }

    /* Turn off HSE */
    if (stm32l0_system_device.hseclk)
    {
        RCC->CR &= ~RCC_CR_HSEON;
    }

    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) | (RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV2);
    __DSB();
    __NOP();
    __NOP();
    __NOP();

    if (hclk < 8000000)
    {
        /* Range 3, use MSI */

        RCC->ICSCR = (RCC->ICSCR & ~RCC_ICSCR_MSIRANGE) | msirange;

        RCC->CR |= RCC_CR_MSION;
    
        while (!(RCC->CR & RCC_CR_MSIRDY))
        {
        }
        
        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_MSI;

        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI)
        {
        }

        stm32l0_system_device.hsi16 &= ~0x80;

        RCC->CFGR &= ~RCC_CFGR_STOPWUCK;
    }
    else
    {
        if (stm32l0_system_device.hseclk == 8000000)
        {
            RCC->CR |= RCC_CR_HSEON;

            while (!(RCC->CR & RCC_CR_HSERDY))
            {
            }

            RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL | RCC_CFGR_PLLDIV)) | (RCC_CFGR_PLLSRC_HSE | pllmul | plldiv);
                
            RCC->CR |= RCC_CR_PLLON;
            
            while (!(RCC->CR & RCC_CR_PLLRDY))
            {
            }
            
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
            
            while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
            {
            }

            stm32l0_system_device.hsi16 &= ~0x80;
        }
        else
        {
            if (hclk < 32000000)
            {
                /* Range 2, use HSI16 directly */
            }
            else
            {
                /* Range 1, use PLL on HSI16 */
                
                RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL | RCC_CFGR_PLLDIV)) | (RCC_CFGR_PLLSRC_HSI | pllmul | plldiv);
                
                RCC->CR |= RCC_CR_PLLON;
                
                while (!(RCC->CR & RCC_CR_PLLRDY))
                {
                }
                
                RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
                
                while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
                {
                }
            }
            
            stm32l0_system_device.hsi16 |= 0x80;
        }

        RCC->CFGR |= RCC_CFGR_STOPWUCK;
    }

    if (!stm32l0_system_device.hsi16)
    {
        RCC->CR &= ~RCC_CR_HSION;
    }

    if ((hclk < 8000000) && !stm32l0_system_device.lock[STM32L0_SYSTEM_LOCK_RANGE_3] && !stm32l0_system_device.lock[STM32L0_SYSTEM_LOCK_RANGE_2_3])
    {
        /* Switch to Range 3 */
        PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS_0 | PWR_CR_VOS_1;
        
        while (PWR->CSR & PWR_CSR_VOSF)
        {
        }

        FLASH->ACR &= ~FLASH_ACR_LATENCY;
    }
    else if ((hclk < 32000000) && !stm32l0_system_device.lock[STM32L0_SYSTEM_LOCK_RANGE_2_3])
    {
        /* Switch to Range 2 */
        PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS_1;
        
        while (PWR->CSR & PWR_CSR_VOSF)
        {
        }

        if (hclk <= 8000000)
        {
            FLASH->ACR &= ~FLASH_ACR_LATENCY;
        }
    }
    else
    {
        if (hclk <= 16000000)
        {
            FLASH->ACR &= ~FLASH_ACR_LATENCY;
        }
    }

    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) | (hpre | ppre1 | ppre2);
    __DSB();
    __NOP();
    __NOP();
    __NOP();

    RCC->APB1ENR &= ~RCC_APB1ENR_PWREN;

    SystemCoreClock = hclk;
    
    stm32l0_system_device.sysclk = sysclk;
    stm32l0_system_device.hclk = hclk;
    stm32l0_system_device.pclk1 = pclk1;
    stm32l0_system_device.pclk2 = pclk2;

    armv6m_systick_enable();

    for (entry = stm32l0_system_device.notify; entry; entry = entry->next)
    {
        if (entry->events & STM32L0_SYSTEM_EVENT_CLOCKS)
        {
            (*entry->callback)(entry->context, STM32L0_SYSTEM_EVENT_CLOCKS);
        }
    }

    __set_PRIMASK(primask);

    return true;
}

void stm32l0_system_lsi_enable(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();

    __disable_irq();

    stm32l0_system_device.lsi++;
    
    if (stm32l0_system_device.lsi == 1)
    {
        RCC->CSR |= RCC_CSR_LSION;

        __set_PRIMASK(primask);
        
        while (!(RCC->CSR & RCC_CSR_LSIRDY))
        {
        }
    }
    else
    {
        __set_PRIMASK(primask);
    }
}

void stm32l0_system_lsi_disable(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();

    __disable_irq();

    stm32l0_system_device.lsi--;
    
    if (stm32l0_system_device.lsi == 0)
    {
        RCC->CSR &= ~RCC_CSR_LSION;
        
        __set_PRIMASK(primask);
        
        while (RCC->CSR & RCC_CSR_LSIRDY)
        {
        }
    }
    else
    {
        __set_PRIMASK(primask);
    }
}

void stm32l0_system_hsi16_enable(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();

    __disable_irq();

    stm32l0_system_device.hsi16++;    

    if (stm32l0_system_device.hsi16 == 1)
    {
        RCC->CR |= RCC_CR_HSION;

        __set_PRIMASK(primask);
        
        while (!(RCC->CR & RCC_CR_HSIRDY))
        {
        }
    }
    else
    {
        __set_PRIMASK(primask);
    }
}

void stm32l0_system_hsi16_disable(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();

    __disable_irq();

    stm32l0_system_device.hsi16--;
    
    if (stm32l0_system_device.hsi16 == 0)
    {
        RCC->CR &= ~RCC_CR_HSION;
    }

    __set_PRIMASK(primask);
}

void stm32l0_system_hsi48_enable()
{
    uint32_t primask;

    primask = __get_PRIMASK();

    __disable_irq();

    stm32l0_system_device.hsi48++;
    
    if (stm32l0_system_device.hsi48 == 1)
    {
        SYSCFG->CFGR3 |= (SYSCFG_CFGR3_ENREF_HSI48 | SYSCFG_CFGR3_EN_VREFINT);

        while (!(SYSCFG->CFGR3 & SYSCFG_CFGR3_VREFINT_RDYF))
        {
        }

        RCC->CRRCR |= RCC_CRRCR_HSI48ON;

        while(!(RCC->CRRCR & RCC_CRRCR_HSI48RDY))
        {
        }

        /* Enable CRS on HSI48 */
        RCC->APB1ENR |= RCC_APB1ENR_CRSEN;
        RCC->APB1ENR;

        /* 32768Hz * 1465 = 48005120Hz, Sync Source is LSE */
        CRS->CFGR = ((1465 -1) << CRS_CFGR_RELOAD_Pos) | (1 << CRS_CFGR_FELIM_Pos) | CRS_CFGR_SYNCSRC_0;
        CRS->CR |= (CRS_CR_AUTOTRIMEN | CRS_CR_CEN);
        
        stm32l0_system_lock(STM32L0_SYSTEM_LOCK_VREFINT);
    }

    __set_PRIMASK(primask);
}

void stm32l0_system_hsi48_disable(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();

    __disable_irq();

    stm32l0_system_device.hsi48--;
    
    if (stm32l0_system_device.hsi48 == 0)
    {
        stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_VREFINT);

        /* Disable CRS on HSI48 */
        CRS->CR &= ~(CRS_CR_AUTOTRIMEN | CRS_CR_CEN);
            
        RCC->APB1ENR &= ~RCC_APB1ENR_CRSEN;

        RCC->CRRCR &= ~RCC_CRRCR_HSI48ON;

        SYSCFG->CFGR3 &= ~SYSCFG_CFGR3_ENREF_HSI48;
    }

    __set_PRIMASK(primask);
}

void stm32l0_system_mco_configure(unsigned int mode, unsigned int divider)
{
    uint32_t mcopre;

    armv6m_atomic_and(&RCC->CFGR, ~(RCC_CFGR_MCOSEL | RCC_CFGR_MCOPRE));

    switch (stm32l0_system_device.mco) {
    case STM32L0_SYSTEM_MCO_MODE_NONE:
    case STM32L0_SYSTEM_MCO_MODE_SYSCLK:
    case STM32L0_SYSTEM_MCO_MODE_MSI:
    case STM32L0_SYSTEM_MCO_MODE_HSE:
    case STM32L0_SYSTEM_MCO_MODE_PLL:
    case STM32L0_SYSTEM_MCO_MODE_LSE:
        break;
      
    case STM32L0_SYSTEM_MCO_MODE_LSI:
        stm32l0_system_lsi_disable();
        break;
        
    case STM32L0_SYSTEM_MCO_MODE_HSI16:
        stm32l0_system_hsi16_disable();
        break;
        
    case STM32L0_SYSTEM_MCO_MODE_HSI48:
        stm32l0_system_hsi48_disable();
        break;
    }
    
    stm32l0_system_device.mco = mode;
    
    switch (stm32l0_system_device.mco) {
    case STM32L0_SYSTEM_MCO_MODE_NONE:
    case STM32L0_SYSTEM_MCO_MODE_SYSCLK:
    case STM32L0_SYSTEM_MCO_MODE_MSI:
    case STM32L0_SYSTEM_MCO_MODE_HSE:
    case STM32L0_SYSTEM_MCO_MODE_PLL:
    case STM32L0_SYSTEM_MCO_MODE_LSE:
        break;
        
    case STM32L0_SYSTEM_MCO_MODE_LSI:
        stm32l0_system_lsi_enable();
        break;
        
    case STM32L0_SYSTEM_MCO_MODE_HSI16:
        stm32l0_system_hsi16_enable();
        break;
        
    case STM32L0_SYSTEM_MCO_MODE_HSI48:
        stm32l0_system_hsi48_enable();
        break;
    }

    if      (divider <=  1) { mcopre = RCC_CFGR_MCOPRE_DIV1;  }
    else if (divider <=  2) { mcopre = RCC_CFGR_MCOPRE_DIV2;  }
    else if (divider <=  4) { mcopre = RCC_CFGR_MCOPRE_DIV4;  }
    else if (divider <=  8) { mcopre = RCC_CFGR_MCOPRE_DIV8;  }
    else                    { mcopre = RCC_CFGR_MCOPRE_DIV16; }
    
    armv6m_atomic_or(&RCC->CFGR, ((mode << RCC_CFGR_MCOSEL_Pos) | mcopre));
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

void stm32l0_system_uid(uint32_t *uid)
{
    uid[0] = *((const uint32_t*)(UID_BASE + 0x00));
    uid[1] = *((const uint32_t*)(UID_BASE + 0x04));
    uid[2] = *((const uint32_t*)(UID_BASE + 0x14));
}

void stm32l0_system_periph_reset(unsigned int periph)
{
    uint32_t primask;

    primask = __get_PRIMASK();

    __disable_irq();

    *stm32l0_system_xlate_RSTR[periph] |= stm32l0_system_xlate_RSTMSK[periph];
    *stm32l0_system_xlate_RSTR[periph] &= ~stm32l0_system_xlate_RSTMSK[periph];

    __set_PRIMASK(primask);
}

void stm32l0_system_periph_enable(unsigned int periph)
{
    uint32_t primask;

    primask = __get_PRIMASK();

    __disable_irq();

    *stm32l0_system_xlate_ENR[periph] |= stm32l0_system_xlate_ENMSK[periph];
    *stm32l0_system_xlate_ENR[periph];

    __set_PRIMASK(primask);
}

void stm32l0_system_periph_disable(unsigned int periph)
{
    uint32_t primask;

    primask = __get_PRIMASK();

    __disable_irq();

    *stm32l0_system_xlate_ENR[periph] &= ~stm32l0_system_xlate_ENMSK[periph];

    __set_PRIMASK(primask);
}

void stm32l0_system_swd_enable(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();

    __disable_irq();

    RCC->APB2ENR |= RCC_APB2ENR_DBGEN;
    RCC->APB2ENR;

    DBGMCU->CR = (DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY);
    DBGMCU->APB1FZ = (DBGMCU_APB1_FZ_DBG_RTC_STOP | DBGMCU_APB1_FZ_DBG_IWDG_STOP);
    DBGMCU->APB2FZ = 0;

    stm32l0_gpio_swd_enable();

    __set_PRIMASK(primask);
}

void stm32l0_system_swd_disable(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();

    __disable_irq();

    stm32l0_gpio_swd_disable();

    RCC->APB2ENR |= RCC_APB2ENR_DBGEN;
    RCC->APB2ENR;

    DBGMCU->CR = 0;
    DBGMCU->APB1FZ = 0;
    DBGMCU->APB2FZ = 0;

    RCC->APB2ENR &= ~RCC_APB2ENR_DBGEN;

    __set_PRIMASK(primask);
}

uint32_t stm32l0_system_read_backup(unsigned int index)
{
    if (index <= 3)
    {
        return (&RTC->BKP0R)[index];
    }

    return 0xffffffff;
}

void stm32l0_system_write_backup(unsigned int index, uint32_t data)
{
    if (index <= 3)
    {
        (&RTC->BKP0R)[index] = data;
    }
}

void stm32l0_system_notify(stm32l0_system_notify_t *notify, stm32l0_system_callback_t callback, void *context, uint32_t events)
{
    stm32l0_system_notify_t **pp_entry, *entry;

    notify->events = 0;

    for (pp_entry = &stm32l0_system_device.notify, entry = *pp_entry; entry; pp_entry = &entry->next, entry = *pp_entry)
    {
        if (entry == notify)
        {
            if (callback == NULL)
            {
                *pp_entry = notify->next;

                notify->next = NULL;
            }
            else
            {
                notify->callback = callback;
                notify->context = context;
                notify->events = events;
            }

            return;
        }
    }

    notify->next = NULL;
    notify->callback = callback;
    notify->context = context;
    notify->events = events;

    *pp_entry = notify;
}

void stm32l0_system_lock(uint32_t lock)
{
    uint32_t primask;

    primask = __get_PRIMASK();

    __disable_irq();

    stm32l0_system_device.lock[lock]++;

    __set_PRIMASK(primask);
}

void stm32l0_system_unlock(uint32_t lock)
{
    uint32_t primask;

    primask = __get_PRIMASK();

    __disable_irq();

    stm32l0_system_device.lock[lock]--;

    __set_PRIMASK(primask);
}

void stm32l0_system_reference(uint32_t reference)
{
    uint32_t primask;

    primask = __get_PRIMASK();

    __disable_irq();

    stm32l0_system_device.reference |= reference;

    __set_PRIMASK(primask);
}

void stm32l0_system_unreference(uint32_t reference)
{
    uint32_t primask;

    primask = __get_PRIMASK();

    __disable_irq();

    stm32l0_system_device.reference &= ~reference;

    __set_PRIMASK(primask);
}

uint32_t stm32l0_system_policy(uint32_t policy)
{
    uint32_t primask, o_policy;

    primask = __get_PRIMASK();

    __disable_irq();

    o_policy = stm32l0_system_device.policy;

    if (policy != STM32L0_SYSTEM_POLICY_NONE)
    {
        stm32l0_system_device.policy = policy;
    }

    __set_PRIMASK(primask);

    return o_policy;
}

void stm32l0_system_sleep(uint32_t policy, uint32_t timeout)
{
    uint32_t primask, n_policy;
    stm32l0_gpio_state_t gpio_state;
    stm32l0_system_notify_t *entry;
    
    if (!stm32l0_system_device.event)
    {
        for (entry = stm32l0_system_device.notify; entry; entry = entry->next)
        {
            if (entry->events & STM32L0_SYSTEM_EVENT_SLEEP)
            {
                (*entry->callback)(entry->context, STM32L0_SYSTEM_EVENT_SLEEP);
            }
        }

        if (!stm32l0_system_device.event)
        {
            if (timeout)
            {
                stm32l0_rtc_wakeup_start(timeout, (stm32l0_rtc_callback_t)stm32l0_system_wakeup, NULL);
            }

            primask = __get_PRIMASK();

            while (!stm32l0_system_device.event)
            {
                __disable_irq();

                n_policy = stm32l0_system_device.policy;
                
                if (policy != STM32L0_SYSTEM_POLICY_NONE)
                {
                    if (n_policy > policy)
                    {
                        n_policy = policy;
                    }
                }
                
                if ((n_policy >= STM32L0_SYSTEM_POLICY_STOP) && !stm32l0_system_device.lock[STM32L0_SYSTEM_LOCK_STOP])
                {
                    for (entry = stm32l0_system_device.notify; entry; entry = entry->next)
                    {
                        if (entry->events & STM32L0_SYSTEM_EVENT_STOP_ENTER)
                        {
                            (*entry->callback)(entry->context, STM32L0_SYSTEM_EVENT_STOP_ENTER);
                        }
                    }

                    stm32l0_gpio_save(&gpio_state);

                    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
                    
                    /* The lowpower voltage regulator adds 3.3uS wakeup time, plus it prevents HSIKERON, 
                     * which may be needed by USART1/USART2/LPUART to wakeup. In those cases do not
                     * enable the low power voltage regulator in stop mode.
                     */
                    if (!stm32l0_system_device.lock[STM32L0_SYSTEM_LOCK_REGULATOR])
                    {
                        PWR->CR |= PWR_CR_LPSDSR;
                    }
                    
                    if (!stm32l0_system_device.lock[STM32L0_SYSTEM_LOCK_VREFINT])
                    {
                        /* Set ULP to disable VREFINT */
                        SYSCFG->CFGR3 &= ~SYSCFG_CFGR3_EN_VREFINT;
                        
                        PWR->CR |= (PWR_CR_FWU | PWR_CR_ULP);
                    }

                    /* Clear WUF flag */
                    PWR->CR |= PWR_CR_CWUF;
                    
                    /* Select STOP */
                    PWR->CR &= ~PWR_CR_PDDS;
                    
                    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

                    if (RCC->APB2ENR & RCC_APB2ENR_DBGEN)
                    {
                        __WFI();
                    }
                    else
                    {
                        FLASH->ACR |= FLASH_ACR_SLEEP_PD;
                        
                        __WFI();
                        
                        FLASH->ACR &= ~FLASH_ACR_SLEEP_PD;
                    }

                    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
                    
                    /* Clear ULP to enable VREFINT, disable lowpower voltage regulator */
                    PWR->CR &= ~(PWR_CR_FWU | PWR_CR_ULP | PWR_CR_LPSDSR);

                    if (stm32l0_system_device.hclk <= 4200000)
                    {
                        /* Range 3, MSI is wakeup clock */
                        
                        if (stm32l0_system_device.hsi16)
                        {
                            RCC->CR |= RCC_CR_HSION;
                            
                            while (!(RCC->CR & RCC_CR_HSIRDY))
                            {
                            }
                        }
                    }
                    else
                    {
                        /* Range 1 & 2, HSI16 is wakeup clock */

                        if (stm32l0_system_device.hseclk == 8000000)
                        {
                            RCC->CR |= RCC_CR_HSEON;
                            
                            while (!(RCC->CR & RCC_CR_HSERDY))
                            {
                            }
                            
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
                            if (stm32l0_system_device.hclk > 16000000)
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
                    }
                    
                    if (!stm32l0_system_device.hsi16)
                    {
                        RCC->CR &= ~RCC_CR_HSION;
                    }
                    
                    RCC->APB1ENR &= ~RCC_APB1ENR_PWREN;

                    stm32l0_gpio_restore(&gpio_state);

                    for (entry = stm32l0_system_device.notify; entry; entry = entry->next)
                    {
                        if (entry->events & STM32L0_SYSTEM_EVENT_STOP_LEAVE)
                        {
                            (*entry->callback)(entry->context, STM32L0_SYSTEM_EVENT_STOP_LEAVE);
                        }
                    }
                }
                else 
                {
                    if ((n_policy >= STM32L0_SYSTEM_POLICY_SLEEP) && !stm32l0_system_device.lock[STM32L0_SYSTEM_LOCK_SLEEP])
                    {
                        __WFI();
                    }
                }

                __set_PRIMASK(primask);
            }
            
            if (timeout)
            {
                stm32l0_rtc_wakeup_stop();
            }
        }
    }
    
    stm32l0_system_device.event = 0;
}

void stm32l0_system_wakeup(void)
{
    stm32l0_system_device.event = 1;
}

void stm32l0_system_standby(uint32_t config)
{
    uint32_t primask;
    stm32l0_system_notify_t *entry;
    
    primask = __get_PRIMASK();
    
    __disable_irq();
    
    if (stm32l0_system_device.lock[STM32L0_SYSTEM_LOCK_STANDBY])
    {
        __set_PRIMASK(primask);
                        
        return;
    }
    
    for (entry = stm32l0_system_device.notify; entry; entry = entry->next)
    {
        if (entry->events & STM32L0_SYSTEM_EVENT_STANDBY)
        {
            (*entry->callback)(entry->context, STM32L0_SYSTEM_EVENT_STANDBY);
        }
    }

    stm32l0_rtc_standby(config);
    
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    
    if (config & STM32L0_SYSTEM_CONFIG_WKUP1)
    {
        PWR->CSR |= PWR_CSR_EWUP1;
    }
    
    if (config & STM32L0_SYSTEM_CONFIG_WKUP2)
    {
        PWR->CSR |= PWR_CSR_EWUP2;
    }

#if defined(STM32L072xx)
    if (config & STM32L0_SYSTEM_CONFIG_WKUP3)
    {
        PWR->CSR |= PWR_CSR_EWUP3;
    }
#endif /* STM32L072xx */
    
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

void stm32l0_system_reset(void)
{
    stm32l0_system_notify_t *entry;

    __disable_irq();

    for (entry = stm32l0_system_device.notify; entry; entry = entry->next)
    {
        if (entry->events & STM32L0_SYSTEM_EVENT_RESET)
        {
            (*entry->callback)(entry->context, STM32L0_SYSTEM_EVENT_RESET);
        }
    }

    stm32l0_rtc_reset();

    NVIC_SystemReset();
    
    while (1)
    {
    }
}

void stm32l0_system_dfu(void)
{
    __disable_irq();

    /* Set the BCK bit to flag a reboot into the booloader */
    RTC->CR |= RTC_CR_BCK;

    stm32l0_system_reset();
}

static void __empty() { }

void __stm32l0_eeprom_initialize(void) __attribute__ ((weak, alias("__empty")));
