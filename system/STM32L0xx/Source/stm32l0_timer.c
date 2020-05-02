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

#include "stm32l0_timer.h"
#include "stm32l0_system.h"

extern void TIM2_IRQHandler(void);
#if defined(STM32L072xx) || defined(STM32L082xx)
extern void TIM3_IRQHandler(void);
#endif /* STM32L072xx || STM32L082xx */
extern void TIM6_DAC_IRQHandler(void);
#if defined(STM32L072xx) || defined(STM32L082xx)
extern void TIM7_IRQHandler(void);
#endif /* STM32L072xx || STM32L082xx */
extern void TIM21_IRQHandler(void);
extern void TIM22_IRQHandler(void);

typedef struct _stm32l0_timer_device_t {
    stm32l0_timer_t   *instances[STM32L0_TIMER_INSTANCE_COUNT];
} stm32l0_timer_device_t;

static stm32l0_timer_device_t stm32l0_timer_device;

static TIM_TypeDef * const stm32l0_timer_xlate_TIM[STM32L0_TIMER_INSTANCE_COUNT] = {
    TIM2,
#if defined(STM32L072xx) || defined(STM32L082xx)
    TIM3,
#endif /* STM32L072xx || STM32L082xx */
    TIM6,
#if defined(STM32L072xx) || defined(STM32L082xx)
    TIM7,
#endif /* STM32L072xx || STM32L082xx */
    TIM21,
    TIM22,
};

static const IRQn_Type stm32l0_timer_xlate_IRQn[STM32L0_TIMER_INSTANCE_COUNT] = {
    TIM2_IRQn,
#if defined(STM32L072xx) || defined(STM32L082xx)
    TIM3_IRQn,
#endif /* STM32L072xx || STM32L082xx */
    TIM6_DAC_IRQn,
#if defined(STM32L072xx) || defined(STM32L082xx)
    TIM7_IRQn,
#endif /* STM32L072xx || STM32L082xx */
    TIM21_IRQn,
    TIM22_IRQn,
};

static void stm32l0_timer_interrupt(stm32l0_timer_t *timer)
{
    TIM_TypeDef *TIM = timer->TIM;
    uint32_t tim_sr, events;

    events = 0;

    tim_sr = TIM->SR;

    TIM->SR = 0;

    if (timer)
    {
        if (tim_sr & TIM_SR_UIF)
        {
            events |= STM32L0_TIMER_EVENT_PERIOD;
        }

        if (tim_sr & (TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF))
        {
            if (tim_sr & TIM_SR_CC1IF)
            {
                events |= STM32L0_TIMER_EVENT_CHANNEL_1;
            }

            if (tim_sr & TIM_SR_CC2IF)
            {
                events |= STM32L0_TIMER_EVENT_CHANNEL_2;
            }

            if (tim_sr & TIM_SR_CC3IF)
            {
                events |= STM32L0_TIMER_EVENT_CHANNEL_3;
            }

            if (tim_sr & TIM_SR_CC4IF)
            {
                events |= STM32L0_TIMER_EVENT_CHANNEL_4;
            }
        }
    
        events &= timer->events;

        if (events)
        {
            (*timer->callback)(timer->context, events);
        }
    }
}

bool stm32l0_timer_create(stm32l0_timer_t *timer, unsigned int instance, unsigned int priority, unsigned int mode)
{
    if (instance >= STM32L0_TIMER_INSTANCE_COUNT)
    {
        return false;
    }

    timer->TIM = stm32l0_timer_xlate_TIM[instance];
    timer->instance = instance;
    timer->interrupt = stm32l0_timer_xlate_IRQn[instance];
    timer->priority = priority;

    stm32l0_timer_device.instances[timer->instance] = timer;

    timer->state = STM32L0_TIMER_STATE_INIT;

    return true;
}

bool stm32l0_timer_destroy(stm32l0_timer_t *timer)
{
    if (timer->state != STM32L0_TIMER_STATE_INIT)
    {
        return false;
    }

    timer->state = STM32L0_TIMER_STATE_NONE;

    stm32l0_timer_device.instances[timer->instance] = NULL;

    return true;
}

uint32_t stm32l0_timer_clock(stm32l0_timer_t *timer)
{
    uint32_t hclk, pclk;

    hclk = stm32l0_system_hclk();

    if ((timer->instance == STM32L0_TIMER_INSTANCE_TIM2)
#if defined(STM32L072xx) || defined(STM32L082xx)
        || (timer->instance == STM32L0_TIMER_INSTANCE_TIM3)
#endif /* STM32L072xx || STM32L082xx */
        || (timer->instance == STM32L0_TIMER_INSTANCE_TIM6)
#if defined(STM32L072xx) || defined(STM32L082xx)
        || (timer->instance == STM32L0_TIMER_INSTANCE_TIM7)
#endif /* STM32L072xx || STM32L082xx */
        )
    {
        pclk = stm32l0_system_pclk1();
    }
    else
    {
        pclk = stm32l0_system_pclk2();
    }

    return ((hclk == pclk) ? hclk : (2 * pclk));
}

bool stm32l0_timer_enable(stm32l0_timer_t *timer, uint32_t prescaler, uint32_t option, stm32l0_timer_callback_t callback, void *context, uint32_t events)
{
    if (timer->state != STM32L0_TIMER_STATE_INIT)
    {
        return false;
    }

    timer->events = 0;
    timer->callback = NULL;
    timer->context = NULL;
    timer->channels = 0;

    NVIC_SetPriority(timer->interrupt, timer->priority);
    NVIC_EnableIRQ(timer->interrupt);

    stm32l0_system_periph_enable(STM32L0_SYSTEM_PERIPH_TIM2 + timer->instance);
    
    timer->state = STM32L0_TIMER_STATE_BUSY;

    stm32l0_timer_configure(timer, prescaler, option);

    stm32l0_timer_notify(timer, callback, context, events);

    timer->state = STM32L0_TIMER_STATE_READY;

    return true;
}

bool stm32l0_timer_disable(stm32l0_timer_t *timer)
{
    if (timer->state != STM32L0_TIMER_STATE_READY)
    {
        return false;
    }

    stm32l0_system_periph_disable(STM32L0_SYSTEM_PERIPH_TIM2 + timer->instance);

    NVIC_DisableIRQ(timer->interrupt);

    timer->state = STM32L0_TIMER_STATE_INIT;

    return true;
}

bool stm32l0_timer_configure(stm32l0_timer_t *timer, uint32_t prescaler, uint32_t option)
{
    TIM_TypeDef *TIM = timer->TIM;
    uint32_t tim_cr1, tim_smcr;

    if ((timer->state != STM32L0_TIMER_STATE_BUSY) && (timer->state != STM32L0_TIMER_STATE_READY))
    {
        return false;
    }

    tim_cr1 = 0;
    tim_smcr = 0;

    if (option & STM32L0_TIMER_OPTION_ENCODER_MODE_MASK)
    {
        tim_smcr |= (((option & STM32L0_TIMER_OPTION_ENCODER_MODE_MASK) >> STM32L0_TIMER_OPTION_ENCODER_MODE_SHIFT) << 0);
    }
    else
    {
        if (option & (STM32L0_TIMER_OPTION_CLOCK_EXTERNAL_CHANNEL_1 | STM32L0_TIMER_OPTION_CLOCK_EXTERNAL_CHANNEL_2))
        {
            tim_smcr |= (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2); /* EXTERNAL 1 */

            if (option & STM32L0_TIMER_OPTION_CLOCK_EXTERNAL_CHANNEL_1)
            {
                tim_smcr |= (TIM_SMCR_TS_0 | TIM_SMCR_TS_2); /* TI1FP1 */
        
            }
            else
            {
                tim_smcr |= (TIM_SMCR_TS_1 | TIM_SMCR_TS_2); /* TI2FP2 */
            }
        }

        if (option & STM32L0_TIMER_OPTION_COUNT_CENTER_MASK)
        {
            tim_cr1 |= (((option & STM32L0_TIMER_OPTION_COUNT_CENTER_MASK) >> STM32L0_TIMER_OPTION_COUNT_CENTER_SHIFT) << 5);
        }
        else
        {
            if (option & STM32L0_TIMER_OPTION_COUNT_DOWN)
            {
                tim_cr1 |= TIM_CR1_DIR;
            }
        }

        if (option & STM32L0_TIMER_OPTION_COUNT_PRELOAD)
        {
            tim_cr1 |= TIM_CR1_ARPE;
        }
    }

    TIM->CR1  = tim_cr1;
    TIM->CR2  = TIM_CR2_MMS_1; /* Update event as TRGO */
    TIM->SMCR = tim_smcr;
    TIM->PSC  = prescaler;

    return true;
}

bool stm32l0_timer_notify(stm32l0_timer_t *timer, stm32l0_timer_callback_t callback, void *context, uint32_t events)
{
    TIM_TypeDef *TIM = timer->TIM;
    uint32_t dier; 

    if ((timer->state != STM32L0_TIMER_STATE_BUSY) && (timer->state != STM32L0_TIMER_STATE_READY))
    {
        return false;
    }

    timer->callback = callback;
    timer->context = context;
    timer->events = events;

    dier = 0;

    if (events & STM32L0_TIMER_EVENT_PERIOD)
    {
        dier |= TIM_DIER_UIE;
    }

    if ((events & timer->channels) & STM32L0_TIMER_EVENT_CHANNEL_1)
    {
        dier |= TIM_DIER_CC1IE;
    }

    if ((events & timer->channels) & STM32L0_TIMER_EVENT_CHANNEL_2)
    {
        dier |= TIM_DIER_CC2IE;
    }

    if ((events & timer->channels) & STM32L0_TIMER_EVENT_CHANNEL_3)
    {
        dier |= TIM_DIER_CC3IE;
    }

    if ((events & timer->channels) & STM32L0_TIMER_EVENT_CHANNEL_4)
    {
        dier |= TIM_DIER_CC4IE;
    }
    
    armv6m_atomic_modify(&TIM->DIER, (TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE), dier);

    return true;
}

bool stm32l0_timer_start(stm32l0_timer_t *timer, uint32_t period, bool oneshot)
{
    TIM_TypeDef *TIM = timer->TIM;

    if ((timer->state != STM32L0_TIMER_STATE_READY) && (timer->state != STM32L0_TIMER_STATE_ACTIVE))
    {
        return false;
    }

    if (timer->state == STM32L0_TIMER_STATE_READY)
    {
        stm32l0_system_lock(STM32L0_SYSTEM_LOCK_CLOCKS);
    }
    else
    {
        armv6m_atomic_and(&TIM->CR1, ~TIM_CR1_CEN);
    }

    TIM->SR = 0;
    TIM->ARR = period;

    if (oneshot)
    {
        armv6m_atomic_or(&TIM->CR1, (TIM_CR1_OPM | TIM_CR1_CEN));
    }
    else
    {
        armv6m_atomic_and(&TIM->CR1, ~TIM_CR1_OPM);
        armv6m_atomic_or(&TIM->CR1, TIM_CR1_CEN);
    }

    timer->state = STM32L0_TIMER_STATE_ACTIVE;

    return true;
}

bool stm32l0_timer_stop(stm32l0_timer_t *timer)
{
    TIM_TypeDef *TIM = timer->TIM;

    if (timer->state != STM32L0_TIMER_STATE_ACTIVE)
    {
        return false;
    }

    armv6m_atomic_and(&TIM->CR1, ~TIM_CR1_CEN);

    stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_CLOCKS);

    timer->state = STM32L0_TIMER_STATE_READY;

    return true;
}

uint32_t stm32l0_timer_count(stm32l0_timer_t *timer)
{
    TIM_TypeDef *TIM = timer->TIM;

    if ((timer->state != STM32L0_TIMER_STATE_READY) && (timer->state != STM32L0_TIMER_STATE_ACTIVE))
    {
        return 0;
    }

    return TIM->CNT;
}

bool stm32l0_timer_period(stm32l0_timer_t *timer, uint32_t period, bool offset)
{
    TIM_TypeDef *TIM = timer->TIM;

    if ((timer->state != STM32L0_TIMER_STATE_READY) && (timer->state != STM32L0_TIMER_STATE_ACTIVE))
    {
        return false;
    }

    if (offset)
    {
        NVIC_DisableIRQ(timer->interrupt);

        TIM->ARR = period - TIM->CNT;

        NVIC_EnableIRQ(timer->interrupt);
    }
    else
    {
        TIM->ARR = period;
    }

    return true;
}

bool stm32l0_timer_channel(stm32l0_timer_t *timer, unsigned int channel, uint32_t compare, uint32_t control)
{
    TIM_TypeDef *TIM = timer->TIM;
    uint32_t tim_ccmr, tim_ccer;

    if ((timer->state != STM32L0_TIMER_STATE_READY) && (timer->state != STM32L0_TIMER_STATE_ACTIVE))
    {
        return false;
    }

    armv6m_atomic_and(&TIM->CCER, ~(TIM_CCER_CC1E << (channel * 4)));

    tim_ccmr = 0;
    tim_ccer = 0;

    if (control & (STM32L0_TIMER_CONTROL_CAPTURE_MASK | STM32L0_TIMER_CONTROL_COMPARE_MASK))
    {
        if (control & STM32L0_TIMER_CONTROL_CAPTURE_MASK)
        {
            tim_ccer |= TIM_CCER_CC1E;

            tim_ccmr |= ((control & STM32L0_TIMER_CONTROL_CAPTURE_ALTERNATE ? TIM_CCMR1_CC1S_1 : TIM_CCMR1_CC1S_0) |
                         (((control & STM32L0_TIMER_CONTROL_CAPTURE_PRESCALE_MASK) >> STM32L0_TIMER_CONTROL_CAPTURE_PRESCALE_SHIFT) << 2) |
                         (((control & STM32L0_TIMER_CONTROL_CAPTURE_FILTER_MASK) >> STM32L0_TIMER_CONTROL_CAPTURE_FILTER_SHIFT) << 4));
            
            if (control & STM32L0_TIMER_CONTROL_CAPTURE_POLARITY)
            {
                if (control & STM32L0_TIMER_CONTROL_CAPTURE_RISING_EDGE)
                {
                    tim_ccer |= ((control & STM32L0_TIMER_CONTROL_CAPTURE_FALLING_EDGE) ? (TIM_CCER_CC1P | TIM_CCER_CC1NP) : TIM_CCER_CC1P);
                }
            }
            else
            {
                if (control & STM32L0_TIMER_CONTROL_CAPTURE_FALLING_EDGE)
                {
                    tim_ccer |= ((control & STM32L0_TIMER_CONTROL_CAPTURE_RISING_EDGE) ? (TIM_CCER_CC1P | TIM_CCER_CC1NP) : TIM_CCER_CC1P);
                }
            }
        }
        else
        {
            switch (control & STM32L0_TIMER_CONTROL_COMPARE_MASK) {
            case STM32L0_TIMER_CONTROL_COMPARE_TIMING:          tim_ccmr |= (                  (0 << 4)); break;
            case STM32L0_TIMER_CONTROL_COMPARE_ACTIVE:          tim_ccmr |= (                  (1 << 4)); break;
            case STM32L0_TIMER_CONTROL_COMPARE_INACTIVE:        tim_ccmr |= (                  (2 << 4)); break;
            case STM32L0_TIMER_CONTROL_COMPARE_TOGGLE:          tim_ccmr |= (                  (3 << 4)); break;
            case STM32L0_TIMER_CONTROL_COMPARE_FORCED_ACTIVE:   tim_ccmr |= (                  (5 << 4)); break;
            case STM32L0_TIMER_CONTROL_COMPARE_FORCED_INACTIVE: tim_ccmr |= (                  (4 << 4)); break;
            case STM32L0_TIMER_CONTROL_PWM:                     tim_ccmr |= (TIM_CCMR1_OC1PE | (6 << 4)); break;
            }

            tim_ccer |= TIM_CCER_CC1E;
        }

        armv6m_atomic_or(&timer->channels, (STM32L0_TIMER_EVENT_CHANNEL_1 << channel));
    }
    else
    {
        /* If no CAPTURE/COMPARE is used the input can still be used clock input.
         */

        tim_ccer |= TIM_CCER_CC1E;

        tim_ccmr |= ((control & STM32L0_TIMER_CONTROL_CAPTURE_ALTERNATE ? TIM_CCMR1_CC1S_1 : TIM_CCMR1_CC1S_0) |
                     (((control & STM32L0_TIMER_CONTROL_CAPTURE_PRESCALE_MASK) >> STM32L0_TIMER_CONTROL_CAPTURE_PRESCALE_SHIFT) << 2) |
                     (((control & STM32L0_TIMER_CONTROL_CAPTURE_FILTER_MASK) >> STM32L0_TIMER_CONTROL_CAPTURE_FILTER_SHIFT) << 4));
        
        if (control & STM32L0_TIMER_CONTROL_CAPTURE_POLARITY)
        {
            tim_ccer |= TIM_CCER_CC1P;
        }
            
        if (channel > STM32L0_TIMER_CHANNEL_4)
        {
            armv6m_atomic_and(&timer->channels, ~(STM32L0_TIMER_EVENT_CHANNEL_1 << channel));
        }
    }

    if ((timer->channels & timer->events) & (STM32L0_TIMER_EVENT_CHANNEL_1 << channel))
    {
        armv6m_atomic_or(&TIM->DIER, (TIM_DIER_CC1IE << channel));
    }
    else
    {
        armv6m_atomic_and(&TIM->DIER, ~(TIM_DIER_CC1IE << channel));
    }

    switch (channel) {
    case STM32L0_TIMER_CHANNEL_1: armv6m_atomic_modify(&TIM->CCMR1, 0x000100ff, (tim_ccmr << 0)); TIM->CCR1 = compare; break;
    case STM32L0_TIMER_CHANNEL_2: armv6m_atomic_modify(&TIM->CCMR1, 0x0100ff00, (tim_ccmr << 8)); TIM->CCR2 = compare; break;
    case STM32L0_TIMER_CHANNEL_3: armv6m_atomic_modify(&TIM->CCMR2, 0x000100ff, (tim_ccmr << 0)); TIM->CCR3 = compare; break;
    case STM32L0_TIMER_CHANNEL_4: armv6m_atomic_modify(&TIM->CCMR2, 0x0100ff00, (tim_ccmr << 8)); TIM->CCR4 = compare; break;
    }
        
    armv6m_atomic_modify(&TIM->CCER, (0x0000000f << (channel * 4)), (tim_ccer << (channel * 4)));
        
    return true;
}

bool stm32l0_timer_compare(stm32l0_timer_t *timer, unsigned int channel, uint32_t compare)
{
    TIM_TypeDef *TIM = timer->TIM;

    if ((timer->state != STM32L0_TIMER_STATE_READY) && (timer->state != STM32L0_TIMER_STATE_ACTIVE))
    {
        return false;
    }

    switch (channel) {
    case STM32L0_TIMER_CHANNEL_1: TIM->CCR1 = compare; break;
    case STM32L0_TIMER_CHANNEL_2: TIM->CCR2 = compare; break;
    case STM32L0_TIMER_CHANNEL_3: TIM->CCR3 = compare; break;
    case STM32L0_TIMER_CHANNEL_4: TIM->CCR4 = compare; break;
    }
    
    return true;
}

uint32_t stm32l0_timer_capture(stm32l0_timer_t *timer, unsigned int channel)
{
    TIM_TypeDef *TIM = timer->TIM;

    switch (channel) {
    case STM32L0_TIMER_CHANNEL_1: return TIM->CCR1; break;
    case STM32L0_TIMER_CHANNEL_2: return TIM->CCR2; break;
    case STM32L0_TIMER_CHANNEL_3: return TIM->CCR3; break;
    case STM32L0_TIMER_CHANNEL_4: return TIM->CCR4; break;
    default:
        return 0;
    }
}

void TIM2_IRQHandler(void)
{
    stm32l0_timer_interrupt(stm32l0_timer_device.instances[STM32L0_TIMER_INSTANCE_TIM2]);
}

#if defined(STM32L072xx) || defined(STM32L082xx)

void TIM3_IRQHandler(void)
{
    stm32l0_timer_interrupt(stm32l0_timer_device.instances[STM32L0_TIMER_INSTANCE_TIM3]);
}

#endif /* STM32L072xx || STM32L082xx */

void TIM6_DAC_IRQHandler(void)
{
    stm32l0_timer_interrupt(stm32l0_timer_device.instances[STM32L0_TIMER_INSTANCE_TIM6]);
}

#if defined(STM32L072xx) || defined(STM32L082xx)

void TIM7_IRQHandler(void)
{
    stm32l0_timer_interrupt(stm32l0_timer_device.instances[STM32L0_TIMER_INSTANCE_TIM7]);
}

#endif /* STM32L072xx || STM32L082xx */

void TIM21_IRQHandler(void)
{
    stm32l0_timer_interrupt(stm32l0_timer_device.instances[STM32L0_TIMER_INSTANCE_TIM21]);
}

void TIM22_IRQHandler(void)
{
    stm32l0_timer_interrupt(stm32l0_timer_device.instances[STM32L0_TIMER_INSTANCE_TIM22]);
}
