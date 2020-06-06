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
#include "stm32l0xx.h"

#include "stm32l0_adc.h"
#include "stm32l0_system.h"

#define ADC_CFGR2_CKMODE_HSI16      0
#define ADC_CFGR2_CKMODE_PCLK_DIV_2 (ADC_CFGR2_CKMODE_0)
#define ADC_CFGR2_CKMODE_PCLK_DIV_4 (ADC_CFGR2_CKMODE_1)
#define ADC_CFGR2_CKMODE_PCLK_DIV_1 (ADC_CFGR2_CKMODE_1 | ADC_CFGR2_CKMODE_0)

#define ADC_SMPR_SMP_1_5            (0)
#define ADC_SMPR_SMP_3_5            (ADC_SMPR_SMP_0)
#define ADC_SMPR_SMP_7_5            (ADC_SMPR_SMP_1)
#define ADC_SMPR_SMP_12_5           (ADC_SMPR_SMP_1 | ADC_SMPR_SMP_0)
#define ADC_SMPR_SMP_19_5           (ADC_SMPR_SMP_2)
#define ADC_SMPR_SMP_39_5           (ADC_SMPR_SMP_2 | ADC_SMPR_SMP_0)
#define ADC_SMPR_SMP_79_5           (ADC_SMPR_SMP_2 | ADC_SMPR_SMP_1)
#define ADC_SMPR_SMP_160_5          (ADC_SMPR_SMP_2 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_0)

#define ADC_CCR_PRESC_DIV_1         0
#define ADC_CCR_PRESC_DIV_2         (ADC_CCR_PRESC_0)
#define ADC_CCR_PRESC_DIV_4         (ADC_CCR_PRESC_1)
#define ADC_CCR_PRESC_DIV_6         (ADC_CCR_PRESC_1 | ADC_CCR_PRESC_0)
#define ADC_CCR_PRESC_DIV_8         (ADC_CCR_PRESC_2)
#define ADC_CCR_PRESC_DIV_10        (ADC_CCR_PRESC_2 | ADC_CCR_PRESC_0)
#define ADC_CCR_PRESC_DIV_12        (ADC_CCR_PRESC_2 | ADC_CCR_PRESC_1)
#define ADC_CCR_PRESC_DIV_16        (ADC_CCR_PRESC_2 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_0)
#define ADC_CCR_PRESC_DIV_32        (ADC_CCR_PRESC_3)
#define ADC_CCR_PRESC_DIV_64        (ADC_CCR_PRESC_3 | ADC_CCR_PRESC_0)
#define ADC_CCR_PRESC_DIV_128       (ADC_CCR_PRESC_3 | ADC_CCR_PRESC_1)
#define ADC_CCR_PRESC_DIV_256       (ADC_CCR_PRESC_3 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_0)

#define STM32L0_ADC_STATE_NONE      0
#define STM32L0_ADC_STATE_READY     1

typedef struct _stm32l0_adc_device_t {
    volatile uint8_t             state;
    uint8_t                      calibration;
    uint16_t                     period;
} stm32l0_adc_device_t;

static stm32l0_adc_device_t stm32l0_adc_device;

bool stm32l0_adc_enable(void)
{
    uint32_t hclk, pclk, adcclk;

    if (stm32l0_adc_device.state != STM32L0_ADC_STATE_NONE)
    {
        return false;
    }

    stm32l0_system_periph_enable(STM32L0_SYSTEM_PERIPH_ADC);

    hclk = stm32l0_system_hclk();
    pclk = stm32l0_system_pclk2();

    /* Clocking is off PCLK. 16MHz max for voltage range 1,
     * 8MHz for voltage range 2, and 4MHz for voltage range 3.
     *
     * To allow a consistent sampling time, the ADCCLK is
     * scaled to be 4MHz per default.
     */

    if ((hclk < 8000000) && (hclk == pclk))
    {
        ADC1->CFGR2 = ADC_CFGR2_CKMODE_PCLK_DIV_1;
        
        adcclk = pclk;
    }
    else
    {
        if (pclk < 16000000)
        {
            ADC1->CFGR2 = ADC_CFGR2_CKMODE_PCLK_DIV_2;
            
            adcclk = pclk / 2;
        }
        else
        {
            ADC1->CFGR2 = ADC_CFGR2_CKMODE_PCLK_DIV_4;
            
            adcclk = pclk / 4;
        }
    }
    
    ADC1_COMMON->CCR = (ADC1_COMMON->CCR & ~ADC_CCR_LFMEN) | ((adcclk < 3500000) ? ADC_CCR_LFMEN : 0);

    ADC1->CR |= ADC_CR_ADVREGEN;

    armv6m_core_udelay(20);

    if (!stm32l0_adc_device.calibration)
    {
        ADC1->CR |= ADC_CR_ADCAL;
    
        while (ADC1->CR & ADC_CR_ADCAL)
        {
        }

        stm32l0_adc_device.calibration = 1;
    }

    stm32l0_adc_device.state = STM32L0_ADC_STATE_READY;
    
    return true;
}

bool stm32l0_adc_disable(void)
{
    if (stm32l0_adc_device.state != STM32L0_ADC_STATE_READY)
    {
        return false;
    }

    ADC1->CR &= ~ADC_CR_ADVREGEN;

    stm32l0_system_periph_disable(STM32L0_SYSTEM_PERIPH_ADC);

    stm32l0_adc_device.state = STM32L0_ADC_STATE_NONE;

    return true;
}

uint32_t stm32l0_adc_read(unsigned int channel, uint16_t period)
{
    uint32_t hclk, pclk, adcclk, adc_cfgr2, adc_smpr, threshold, data;

    if (stm32l0_adc_device.state != STM32L0_ADC_STATE_READY)
    {
        return 0;
    }

    if (channel > STM32L0_ADC_CHANNEL_15)
    {
        if (channel == STM32L0_ADC_CHANNEL_VREFINT)
        {
            stm32l0_system_vrefint_enable();
            
            armv6m_atomic_or(&SYSCFG->CFGR3, SYSCFG_CFGR3_ENBUF_VREFINT_ADC);

            ADC1_COMMON->CCR |= ADC_CCR_VREFEN;
        }

        if (channel == STM32L0_ADC_CHANNEL_TSENSE)
        {
            stm32l0_system_vrefint_enable();

            armv6m_atomic_or(&SYSCFG->CFGR3, SYSCFG_CFGR3_ENBUF_SENSOR_ADC);

            ADC1_COMMON->CCR |= ADC_CCR_TSEN;

            armv6m_core_udelay(20);
        }
    }

    hclk = stm32l0_system_hclk();
    pclk = stm32l0_system_pclk2();

    if ((hclk < 8000000) && (hclk == pclk))
    {
        adc_cfgr2 = ADC_CFGR2_CKMODE_PCLK_DIV_1;

        adcclk = pclk;
    }
    else
    {
        if (pclk < 16000000)
        {
            adc_cfgr2 = ADC_CFGR2_CKMODE_PCLK_DIV_2;
            
            adcclk = pclk / 2;
        }
        else
        {
            adc_cfgr2 = ADC_CFGR2_CKMODE_PCLK_DIV_4;

            adcclk = pclk / 4;
        }
    }

    if (period > 50)
    {
        period = 50;
    }
    
    threshold = ((uint32_t)period * adcclk);

    if      (threshold < (uint32_t)(  1.5 * 1e6)) { adc_smpr = ADC_SMPR_SMP_1_5;   }
    else if (threshold < (uint32_t)(  3.5 * 1e6)) { adc_smpr = ADC_SMPR_SMP_3_5;   }
    else if (threshold < (uint32_t)(  7.5 * 1e6)) { adc_smpr = ADC_SMPR_SMP_7_5;   }
    else if (threshold < (uint32_t)( 12.5 * 1e6)) { adc_smpr = ADC_SMPR_SMP_12_5;  }
    else if (threshold < (uint32_t)( 19.5 * 1e6)) { adc_smpr = ADC_SMPR_SMP_19_5;  }
    else if (threshold < (uint32_t)( 39.5 * 1e6)) { adc_smpr = ADC_SMPR_SMP_39_5;  }
    else if (threshold < (uint32_t)( 79.5 * 1e6)) { adc_smpr = ADC_SMPR_SMP_79_5;  }
    else                                          { adc_smpr = ADC_SMPR_SMP_160_5; }

    ADC1_COMMON->CCR = (ADC1_COMMON->CCR & ~ADC_CCR_LFMEN) | ((adcclk < 3500000) ? ADC_CCR_LFMEN : 0);

    ADC1->CFGR1 = ADC_CFGR1_OVRMOD;
    ADC1->CFGR2 = adc_cfgr2;
    ADC1->SMPR = adc_smpr;
    ADC1->CHSELR = (1ul << channel) & ADC_CHSELR_CHSEL;

    ADC1->ISR = ADC_ISR_ADRDY;

    ADC1->CR |= ADC_CR_ADEN;

    while (!(ADC1->ISR & ADC_ISR_ADRDY))
    {
    }

    ADC1->ISR = ADC_ISR_EOC;
    
    ADC1->CR |= ADC_CR_ADSTART;

    while (!(ADC1->ISR & ADC_ISR_EOC))
    {
    }
    
    data = ADC1->DR & ADC_DR_DATA;

    ADC1->CR |= ADC_CR_ADDIS;

    while (ADC1->CR & ADC_CR_ADEN)
    {
    }

    if (channel > STM32L0_ADC_CHANNEL_15)
    {
        if (channel == STM32L0_ADC_CHANNEL_VREFINT)
        {
            armv6m_atomic_and(&SYSCFG->CFGR3, ~SYSCFG_CFGR3_ENBUF_VREFINT_ADC);

            stm32l0_system_vrefint_disable();
        }

        if (channel == STM32L0_ADC_CHANNEL_TSENSE)
        {
            ADC1_COMMON->CCR &= ~ADC_CCR_TSEN;

            armv6m_atomic_and(&SYSCFG->CFGR3, ~SYSCFG_CFGR3_ENBUF_SENSOR_ADC);

            stm32l0_system_vrefint_disable();
        }
    }

    return data;
}
