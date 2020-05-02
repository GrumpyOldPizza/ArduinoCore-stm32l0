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

#include "armv6m.h"
#include "stm32l0xx.h"

#include "stm32l0_adc.h"
#include "stm32l0_dma.h"
#include "stm32l0_system.h"


#define STM32L0_ADC_DMA_OPTION_RECEIVE_8            \
    (STM32L0_DMA_OPTION_EVENT_TRANSFER_DONE |       \
     STM32L0_DMA_OPTION_PERIPHERAL_TO_MEMORY |      \
     STM32L0_DMA_OPTION_PERIPHERAL_DATA_SIZE_32 |   \
     STM32L0_DMA_OPTION_MEMORY_DATA_SIZE_8 |        \
     STM32L0_DMA_OPTION_MEMORY_DATA_INCREMENT |     \
     STM32L0_DMA_OPTION_PRIORITY_HIGH)

#define STM32L0_ADC_DMA_OPTION_RECEIVE_16           \
    (STM32L0_DMA_OPTION_EVENT_TRANSFER_DONE |       \
     STM32L0_DMA_OPTION_PERIPHERAL_TO_MEMORY |      \
     STM32L0_DMA_OPTION_PERIPHERAL_DATA_SIZE_32 |   \
     STM32L0_DMA_OPTION_MEMORY_DATA_SIZE_16 |       \
     STM32L0_DMA_OPTION_MEMORY_DATA_INCREMENT |     \
     STM32L0_DMA_OPTION_PRIORITY_HIGH)

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

#define STM32L0_ADC_STATE_NONE    0
#define STM32L0_ADC_STATE_READY   1
#define STM32L0_ADC_STATE_CONVERT 2
#define STM32L0_ADC_STATE_DONE    3

typedef struct _stm32l0_adc_device_t {
    volatile uint8_t             state;
    uint8_t                      calibration;
    uint8_t                      channels;
    uint16_t                     period;
    uint16_t                     mask;
    uint32_t                     control;
    stm32l0_adc_done_callback_t  xf_callback;
    void                         *xf_context;
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
            armv6m_atomic_or(&SYSCFG->CFGR3, (SYSCFG_CFGR3_ENBUF_VREFINT_ADC | SYSCFG_CFGR3_EN_VREFINT));

            while (!(SYSCFG->CFGR3 & SYSCFG_CFGR3_VREFINT_RDYF))
            {
            }

            ADC1_COMMON->CCR |= ADC_CCR_VREFEN;
        }

        if (channel == STM32L0_ADC_CHANNEL_TSENSE)
        {
            armv6m_atomic_or(&SYSCFG->CFGR3, (SYSCFG_CFGR3_ENBUF_SENSOR_ADC | SYSCFG_CFGR3_EN_VREFINT));

            while (!(SYSCFG->CFGR3 & SYSCFG_CFGR3_VREFINT_RDYF))
            {
            }

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
        }

        if (channel == STM32L0_ADC_CHANNEL_TSENSE)
        {
            ADC1_COMMON->CCR &= ~ADC_CCR_TSEN;

            armv6m_atomic_and(&SYSCFG->CFGR3, ~SYSCFG_CFGR3_ENBUF_SENSOR_ADC);
        }
    }

    return data;
}

bool stm32l0_adc_convert(void *data, uint32_t count, uint16_t mask, uint16_t period, uint32_t control, stm32l0_adc_done_callback_t callback, void *context)
{
    uint32_t hclk, pclk, adcclk, adc_cfgr1, adc_cfgr2, adc_smpr, adc_ccr, threshold, channels, option;

    if ((stm32l0_adc_device.state != STM32L0_ADC_STATE_READY) && (stm32l0_adc_device.state != STM32L0_ADC_STATE_DONE))
    {
        return false;
    }

    hclk = stm32l0_system_hclk();
    pclk = stm32l0_system_pclk2();

    if ((stm32l0_adc_device.state == STM32L0_ADC_STATE_READY) || (stm32l0_adc_device.mask != mask) || (stm32l0_adc_device.period != period) || (stm32l0_adc_device.control != control))
    {
        if (((control & STM32L0_ADC_CONTROL_MODE_MASK) >= STM32L0_ADC_CONTROL_MODE_CONTINUOUS_1000000) && (hclk < 32000000))
        {
            return false;
        }

        if (stm32l0_adc_device.state == STM32L0_ADC_STATE_READY)
        {
            if (!stm32l0_dma_enable(STM32L0_DMA_CHANNEL_DMA1_CH1_ADC, (stm32l0_dma_callback_t)stm32l0_adc_cancel, NULL))
            {
                return false;
            }

            stm32l0_system_lock(STM32L0_SYSTEM_LOCK_CLOCKS);
            stm32l0_system_lock(STM32L0_SYSTEM_LOCK_STOP);
        }
        else
        {
            ADC1->CR |= ADC_CR_ADSTP;
            
            while (ADC1->CR & (ADC_CR_ADSTP | ADC_CR_ADSTART))
            {
            }
            
            ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN;
            
            ADC1->CR |= ADC_CR_ADDIS;
            
            while (ADC1->CR & ADC_CR_ADEN)
            {
            }

            if ((stm32l0_adc_device.control & STM32L0_ADC_CONTROL_MODE_MASK) >= STM32L0_ADC_CONTROL_MODE_CONTINUOUS_1000000)
            {
                stm32l0_system_hsi16_disable();
            }
        }

        adc_cfgr1 = ADC_CFGR1_OVRMOD | ADC_CFGR1_DMAEN;
        adc_cfgr2 = 0;
        adc_smpr = 0;
        adc_ccr = 0;

        if (control & STM32L0_ADC_CONTROL_BYTE_PACKED)
        {
            adc_cfgr1 |= ADC_CFGR1_RES_1;
        }
        else
        {
            if (control & STM32L0_ADC_CONTROL_LEFT_ALIGNED)
            {
                adc_cfgr1 |= ADC_CFGR1_ALIGN;
            }
            else
            {
                if ((control & STM32L0_ADC_CONTROL_RATIO_MASK) != STM32L0_ADC_CONTROL_RATIO_1)
                {
                    adc_cfgr2 |= ADC_CFGR2_OVSE;
                    adc_cfgr2 |= ((((control & STM32L0_ADC_CONTROL_RATIO_MASK) >> STM32L0_ADC_CONTROL_RATIO_SHIFT) -1) << ADC_CFGR2_OVSR_Pos);
                }
                
                adc_cfgr2 |= (((control & STM32L0_ADC_CONTROL_SHIFT_MASK) >> STM32L0_ADC_CONTROL_SHIFT_SHIFT) << ADC_CFGR2_OVSS_Pos);
            }
        }
        
        if ((control & STM32L0_ADC_CONTROL_MODE_MASK) >= STM32L0_ADC_CONTROL_MODE_CONTINUOUS_1000000)
        {
            adc_cfgr1 |= ADC_CFGR1_CONT;
            adc_cfgr2 |= ADC_CFGR2_CKMODE_HSI16;
            adc_smpr = ((control & STM32L0_ADC_CONTROL_MODE_MASK) == STM32L0_ADC_CONTROL_MODE_CONTINUOUS_1000000) ? ADC_SMPR_SMP_3_5 : ADC_SMPR_SMP_19_5;
            adc_ccr = ADC_CCR_PRESC_DIV_1;

            adcclk = 16000000;

            stm32l0_system_hsi16_enable();
        }
        else
        {
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

            if (adcclk < 3500000)
            {
                adc_ccr |= ADC_CCR_LFMEN;
            }

            if ((control & STM32L0_ADC_CONTROL_MODE_MASK) == STM32L0_ADC_CONTROL_MODE_SINGLE)
            {
                if ((control & STM32L0_ADC_CONTROL_TRIG_MASK) != STM32L0_ADC_CONTROL_TRIG_EXTERNAL)
                {
                    adc_cfgr1 |= ((((control & STM32L0_ADC_CONTROL_TRIG_MASK) >> STM32L0_ADC_CONTROL_TRIG_SHIFT) << ADC_CFGR1_EXTSEL_Pos) | ADC_CFGR1_EXTEN_0);
                }
                else
                {
                    adc_cfgr1 |= ((((control & STM32L0_ADC_CONTROL_EDGE_MASK) >> STM32L0_ADC_CONTROL_EDGE_SHIFT) << ADC_CFGR1_EXTEN_Pos) | ADC_CFGR1_EXTSEL);
                }

                if (control & STM32L0_ADC_CONTROL_DISCONTINUOUS)
                {
                    adc_cfgr1 |= ADC_CFGR1_DISCEN;
                    
                    if ((control & STM32L0_ADC_CONTROL_RATIO_MASK) != STM32L0_ADC_CONTROL_RATIO_1)
                    {
                        adc_cfgr2 |= ADC_CFGR2_TOVS;
                    }
                }

                if (!(control & STM32L0_ADC_CONTROL_NOSLEEP))
                {
                    adc_cfgr1 |= ADC_CFGR1_AUTOFF;
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
        }

        ADC1_COMMON->CCR = (ADC1_COMMON->CCR & ~(ADC_CCR_LFMEN | ADC_CCR_PRESC)) | adc_ccr;

        ADC1->CFGR1 = adc_cfgr1;
        ADC1->CFGR2 = adc_cfgr2;
        ADC1->SMPR = adc_smpr;
        ADC1->CHSELR = mask & 0xffff;

        stm32l0_adc_device.period = period;
        stm32l0_adc_device.mask = mask;
        stm32l0_adc_device.control = control;

        for (mask &= 0xffff, channels = 0; mask; mask >>= 1)
        {
            if (mask & 1) 
            {
                channels++;
            }
        }

        stm32l0_adc_device.channels = channels;
    }

    if (stm32l0_adc_device.control & STM32L0_ADC_CONTROL_BYTE_PACKED)
    {
        count = (count / stm32l0_adc_device.channels) * stm32l0_adc_device.channels;

        option = STM32L0_ADC_DMA_OPTION_RECEIVE_8;
    }
    else
    {
        count = ((count / 2) / stm32l0_adc_device.channels) * stm32l0_adc_device.channels;

        option = STM32L0_ADC_DMA_OPTION_RECEIVE_16;
    }

    stm32l0_adc_device.xf_callback = callback;
    stm32l0_adc_device.xf_context = context;
    stm32l0_adc_device.state = STM32L0_ADC_STATE_CONVERT;

    stm32l0_dma_start(STM32L0_DMA_CHANNEL_DMA1_CH1_ADC, (uint32_t)data, (uint32_t)&ADC1->DR, count, option);

    if ((stm32l0_adc_device.control & STM32L0_ADC_CONTROL_MODE_MASK) != STM32L0_ADC_CONTROL_MODE_SINGLE)
    {
        ADC1->ISR = ADC_ISR_EOC;
    
        ADC1->CR |= ADC_CR_ADSTART;
    }

    return true;
}

void stm32l0_adc_cancel(void)
{
    uint32_t count;

    if (stm32l0_adc_device.state == STM32L0_ADC_STATE_CONVERT)
    {
        count = stm32l0_dma_stop(STM32L0_DMA_CHANNEL_DMA1_CH1_ADC);

        stm32l0_adc_device.state = STM32L0_ADC_STATE_DONE;

        if (!(stm32l0_adc_device.control & STM32L0_ADC_CONTROL_BYTE_PACKED))
        {
            count = count * 2;
        }

        if (stm32l0_adc_device.xf_callback)
        {
            (*stm32l0_adc_device.xf_callback)(stm32l0_adc_device.xf_context, count);
        }

        if (stm32l0_adc_device.state == STM32L0_ADC_STATE_DONE)
        {
            ADC1->CR |= ADC_CR_ADSTP;

            while (ADC1->CR & (ADC_CR_ADSTP | ADC_CR_ADSTART))
            {
            }

            ADC1->CFGR1 &= ~(ADC_CFGR1_EXTEN | ADC_CFGR1_CONT);
            
            ADC1->CR |= ADC_CR_ADDIS;

            while (ADC1->CR & ADC_CR_ADEN)
            {
            }

            stm32l0_dma_disable(STM32L0_DMA_CHANNEL_DMA1_CH1_ADC);

            if ((stm32l0_adc_device.control & STM32L0_ADC_CONTROL_MODE_MASK) >= STM32L0_ADC_CONTROL_MODE_CONTINUOUS_1000000)
            {
                stm32l0_system_hsi16_disable();
            }

            stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_STOP);
            stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_CLOCKS);
        
            stm32l0_adc_device.state = STM32L0_ADC_STATE_READY;
        }
    }
}

bool stm32l0_adc_done(void)
{
    return (stm32l0_adc_device.state != STM32L0_ADC_STATE_CONVERT);
}
