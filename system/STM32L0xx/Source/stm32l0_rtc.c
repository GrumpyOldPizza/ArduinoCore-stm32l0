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

#include "stm32l0_rtc.h"
#include "stm32l0_system.h"

extern void RTC_IRQHandler(void);

typedef struct _stm32l0_rtc_device_t {
    stm32l0_rtc_callback_t  alarm_callback;
    void                    *alarm_context;
    stm32l0_rtc_callback_t  wakeup_callback;
    void                    *wakeup_context;
    int32_t                 reference_seconds;
    uint16_t                reference_subseconds;
    volatile uint8_t        timer_busy;
    volatile uint32_t       timer_events;
    stm32l0_rtc_timer_t     *timer_queue;
    stm32l0_rtc_calendar_t  timer_start;
    stm32l0_rtc_calendar_t  timer_stop;
} stm32l0_rtc_device_t;

static stm32l0_rtc_device_t stm32l0_rtc_device;

static const uint8_t stm32l0_rtc_int_to_bcd[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 
    0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 
    0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 
    0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 
    0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 
    0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 
    0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 
    0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 
};

static const uint8_t stm32l0_rtc_bcd_to_int[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};


static inline void stm32l0_rtc_calendar_pack(const stm32l0_rtc_calendar_t *calendar, uint32_t *p_c0, uint32_t *p_c1)
{
    *p_c0 = ((const uint32_t*)calendar)[0];
    *p_c1 = ((const uint32_t*)calendar)[1];
}

static inline void stm32l0_rtc_calendar_unpack(uint32_t c0, uint32_t c1, stm32l0_rtc_calendar_t *p_calendar)
{
    ((uint32_t*)p_calendar)[0] = c0;
    ((uint32_t*)p_calendar)[1] = c1;
}

static void stm32l0_rtc_timer_routine(void);
static void stm32l0_rtc_timer_sync(const stm32l0_rtc_calendar_t *stop, const stm32l0_rtc_calendar_t *start);
static void stm32l0_rtc_timer_sync_1(uint32_t c0, uint32_t c1);
static void stm32l0_rtc_timer_sync_2(uint32_t c0, uint32_t c1);
static void stm32l0_rtc_timer_alarm(const stm32l0_rtc_calendar_t *start);

void __stm32l0_rtc_initialize(void)
{
    uint32_t r_seconds;
    uint16_t r_subseconds;
    stm32l0_rtc_calendar_t calendar;

    RTC->WPR = 0xca;
    RTC->WPR = 0x53;

    if (!(RCC->CSR & RCC_CSR_RTCEN))
    {
        if (stm32l0_system_lseclk())
        {
            /* Use LSE as source for RTC */
        
            RCC->CSR = (RCC->CSR & ~RCC_CSR_RTCSEL) | (RCC_CSR_RTCSEL_0 | RCC_CSR_RTCEN);
        }
        else
        {
            RCC->CSR = (RCC->CSR & ~RCC_CSR_RTCSEL) | (RCC_CSR_RTCSEL_1 | RCC_CSR_RTCEN);
        }
            
        RTC->ISR = RTC_ISR_INIT;
        
        while (!(RTC->ISR & RTC_ISR_INITF))
        {
        }
        
        RTC->CR = RTC_CR_BYPSHAD;
        
        RTC->PRER = (STM32L0_RTC_PREDIV_S -1) << RTC_PRER_PREDIV_S_Pos;
        
        if (stm32l0_system_lseclk())
        {
            RTC->PRER |= (STM32L0_RTC_PREDIV_A -1) << RTC_PRER_PREDIV_A_Pos;
        }
        else
        {
            RTC->PRER |= (((stm32l0_system_lsiclk() + (STM32L0_RTC_PREDIV_S / 2)) / STM32L0_RTC_PREDIV_S) -1) << RTC_PRER_PREDIV_A_Pos;
        }
    }

    RTC->CR &= ~(RTC_CR_WUTIE | RTC_CR_ALRBIE | RTC_CR_ALRAIE | RTC_CR_WUTE | RTC_CR_ALRBE | RTC_CR_ALRAE);
    RTC->ISR = 0;

    EXTI->PR = EXTI_PR_PIF17 | EXTI_PR_PIF20;

    stm32l0_rtc_get_calendar(&calendar);
    stm32l0_rtc_calendar_to_time(&calendar, &r_seconds, &r_subseconds);
    stm32l0_rtc_time_delta(0, 0, r_seconds, r_subseconds, &stm32l0_rtc_device.reference_seconds, &stm32l0_rtc_device.reference_subseconds);

    stm32l0_rtc_device.timer_queue = NULL;

    armv6m_systick_sync(0, 0);
}

void stm32l0_rtc_configure(unsigned int priority)
{
    NVIC_SetPriority(RTC_IRQn, priority);
    NVIC_EnableIRQ(RTC_IRQn);

    armv6m_atomic_and(&EXTI->IMR, ~(EXTI_IMR_IM17 | EXTI_IMR_IM19 | EXTI_IMR_IM20));

    EXTI->PR = (EXTI_PR_PIF17 | EXTI_PR_PIF19 | EXTI_PR_PIF20);

    armv6m_atomic_or(&EXTI->RTSR, (EXTI_RTSR_RT17 | EXTI_RTSR_RT19 | EXTI_RTSR_RT20));
    armv6m_atomic_or(&EXTI->IMR, (EXTI_IMR_IM17 | EXTI_IMR_IM20));
}

static void stm32l0_rtc_modify_calendar(uint32_t mask, const stm32l0_rtc_calendar_t *calendar)
{
    stm32l0_rtc_calendar_t before, after;
    int32_t d_seconds;
    uint16_t d_subseconds;
    uint32_t ipsr, c0, c1;

    armv6m_atomic_and(&RTC->CR, ~(RTC_CR_ALRBIE | RTC_CR_ALRBE));
    armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_ALRBF | RTC_ISR_INIT));
    
    armv6m_atomic_add(&stm32l0_rtc_device.timer_events, 1);

    NVIC_DisableIRQ(RTC_IRQn);
    
    stm32l0_rtc_get_calendar(&before);

    after.subseconds = 0;
    after.seconds    = (mask & STM32L0_RTC_CALENDAR_MASK_SECONDS) ? calendar->seconds : before.seconds;
    after.minutes    = (mask & STM32L0_RTC_CALENDAR_MASK_MINUTES) ? calendar->minutes : before.minutes;
    after.hours      = (mask & STM32L0_RTC_CALENDAR_MASK_HOURS)   ? calendar->hours   : before.hours;
    after.day        = (mask & STM32L0_RTC_CALENDAR_MASK_DAY)     ? calendar->day     : before.day;
    after.month      = (mask & STM32L0_RTC_CALENDAR_MASK_MONTH)   ? calendar->month   : before.month;
    after.year       = (mask & STM32L0_RTC_CALENDAR_MASK_YEAR)    ? calendar->year    : before.year;

    /* Set all bits including RTC_ISR_INIT, hence no ISR flag gets cleaned.
     */
    RTC->ISR = ~0u | RTC_ISR_INIT;
    
    while (!(RTC->ISR & RTC_ISR_INITF))
    {
    }

    RTC->TR = ((stm32l0_rtc_int_to_bcd[after.seconds] << RTC_TR_SU_Pos) |
               (stm32l0_rtc_int_to_bcd[after.minutes] << RTC_TR_MNU_Pos) |
               (stm32l0_rtc_int_to_bcd[after.hours] << RTC_TR_HU_Pos));

    RTC->DR = ((stm32l0_rtc_int_to_bcd[after.day] << RTC_DR_DU_Pos) |
               (stm32l0_rtc_int_to_bcd[after.month] << RTC_DR_MU_Pos) |
               (stm32l0_rtc_int_to_bcd[after.year] << RTC_DR_YU_Pos));
    
    /* Set all bits excluding RTC_ISR_INIT, hence no ISR flag gets cleaned
     */
    RTC->ISR = ~0u & ~RTC_ISR_INIT;

    /* With RTC_CR_BYPSHAD there is no indication when the RTC can
     * be read again. So there needs to be a delay that covers
     * the 4 RTCCLK periods before the clock is restarted.
     */

    armv6m_core_udelay(150);

    stm32l0_rtc_calendar_delta(&after, &before, &d_seconds, &d_subseconds);
    stm32l0_rtc_time_subtract(stm32l0_rtc_device.reference_seconds, stm32l0_rtc_device.reference_subseconds, d_seconds, d_subseconds, &stm32l0_rtc_device.reference_seconds, &stm32l0_rtc_device.reference_subseconds);

    NVIC_EnableIRQ(RTC_IRQn);

    ipsr = __get_IPSR();

    if ((ipsr == 11) || (ipsr == 14))
    {
        stm32l0_rtc_timer_sync(&before, &after);
    }
    else
    {
        stm32l0_rtc_calendar_pack(&before, &c0, &c1);

        armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)stm32l0_rtc_timer_sync_1, (void*)c0, c1);
        
        stm32l0_rtc_calendar_pack(&after, &c0, &c1);

        armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)stm32l0_rtc_timer_sync_2, (void*)c0, c1);
    }
}

void stm32l0_rtc_get_calendar(stm32l0_rtc_calendar_t *p_calendar)
{
    uint32_t rtc_ssr, rtc_tr, rtc_dr;

    do
    {
        rtc_ssr = RTC->SSR;
        rtc_tr = RTC->TR;
        rtc_dr = RTC->DR;
    }
    while (rtc_ssr != RTC->SSR);
        
    p_calendar->subseconds = ((STM32L0_RTC_PREDIV_S - 1) - (rtc_ssr & (STM32L0_RTC_PREDIV_S - 1))) * STM32L0_RTC_PREDIV_A;
    p_calendar->seconds    = stm32l0_rtc_bcd_to_int[(rtc_tr & (RTC_TR_SU_Msk | RTC_TR_ST_Msk)) >> RTC_TR_SU_Pos];
    p_calendar->minutes    = stm32l0_rtc_bcd_to_int[(rtc_tr & (RTC_TR_MNU_Msk | RTC_TR_MNT_Msk)) >> RTC_TR_MNU_Pos];
    p_calendar->hours      = stm32l0_rtc_bcd_to_int[(rtc_tr & (RTC_TR_HU_Msk | RTC_TR_HT_Msk)) >> RTC_TR_HU_Pos];
    p_calendar->day        = stm32l0_rtc_bcd_to_int[(rtc_dr & (RTC_DR_DU_Msk | RTC_DR_DT_Msk)) >> RTC_DR_DU_Pos];
    p_calendar->month      = stm32l0_rtc_bcd_to_int[(rtc_dr & (RTC_DR_MU_Msk | RTC_DR_MT_Msk)) >> RTC_DR_MU_Pos];
    p_calendar->year       = stm32l0_rtc_bcd_to_int[(rtc_dr & (RTC_DR_YU_Msk | RTC_DR_YT_Msk)) >> RTC_DR_YU_Pos];
}

void stm32l0_rtc_set_calendar(unsigned int mask, const stm32l0_rtc_calendar_t *calendar)
{
    if (__get_IPSR() == 0)
    {
        armv6m_svcall_2((uint32_t)&stm32l0_rtc_modify_calendar, (uint32_t)mask, (uint32_t)calendar);
    }
    else
    {
        stm32l0_rtc_modify_calendar(mask, calendar);
    }
}

int32_t stm32l0_rtc_get_calibration(void)
{
    if (RTC->CALR & RTC_CALR_CALP)
    {
        return 512 - ((int32_t)((RTC->CALR & RTC_CALR_CALM_Msk) >> RTC_CALR_CALM_Pos));
    }
    else
    {
        return - ((int32_t)((RTC->CALR & RTC_CALR_CALM_Msk) >> RTC_CALR_CALM_Pos));
    }
}

uint32_t stm32l0_rtc_get_subseconds(void)
{
    return ((STM32L0_RTC_PREDIV_S - 1) - (RTC->SSR & (STM32L0_RTC_PREDIV_S - 1))) * STM32L0_RTC_PREDIV_A;
}

void stm32l0_rtc_adjust_subseconds(int32_t delta)
{
    if (delta > 0) 
    {
        if (delta > 32768)
        {
            delta = 32768;
        }

        RTC->SHIFTR = RTC_SHIFTR_ADD1S | (((uint32_t)(32768 - delta) / STM32L0_RTC_PREDIV_A) << RTC_SHIFTR_SUBFS_Pos);
    }
    else
    {
        if (delta < -32767)
        {
            delta = -32767;
        }

        RTC->SHIFTR = (((uint32_t)(-delta) / STM32L0_RTC_PREDIV_A) << RTC_SHIFTR_SUBFS_Pos);
    }

    while (RTC->ISR & RTC_ISR_SHPF)
    {
    }
}

void stm32l0_rtc_set_calibration(int32_t calibration)
{
    if (calibration > 0)
    {
        if (calibration > 512)
        {
            calibration = 512;
        }

        RTC->CALR = RTC_CALR_CALP | ((uint32_t)(512 - calibration) << RTC_CALR_CALM_Pos);
    }
    else
    {
        if (calibration < -511)
        {
            calibration = -511;
        }

        RTC->CALR = ((uint32_t)(-calibration) << RTC_CALR_CALM_Pos);
    }

    while (RTC->ISR & RTC_ISR_RECALPF)
    {
    }
}

void stm32l0_rtc_alarm_attach(unsigned int match, const stm32l0_rtc_alarm_t *alarm, stm32l0_rtc_callback_t callback, void *context)
{
    uint32_t n_alrmr;

    armv6m_atomic_and(&RTC->CR, ~(RTC_CR_ALRAIE | RTC_CR_ALRAE));
    armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_ALRAF | RTC_ISR_INIT));

    stm32l0_rtc_device.alarm_callback = callback;
    stm32l0_rtc_device.alarm_context = context;

    n_alrmr = 0;
    
    if (match & STM32L0_RTC_ALARM_MATCH_SECONDS) 
    {
        n_alrmr |= (stm32l0_rtc_int_to_bcd[alarm->seconds] << RTC_ALRMAR_SU_Pos);
    }
    else 
    {
        n_alrmr |= RTC_ALRMAR_MSK1;
    }
    
    if (match & STM32L0_RTC_ALARM_MATCH_MINUTES) 
    {
        n_alrmr |= (stm32l0_rtc_int_to_bcd[alarm->minutes] << RTC_ALRMAR_MNU_Pos);
    }
    else 
    {
        n_alrmr |= RTC_ALRMAR_MSK2;
    }
    
    if (match & STM32L0_RTC_ALARM_MATCH_HOURS) 
    {
        n_alrmr |= (stm32l0_rtc_int_to_bcd[alarm->hours] << RTC_ALRMAR_HU_Pos);
    }
    else 
    {
        n_alrmr |= RTC_ALRMAR_MSK3;
    }
    
    if (match & STM32L0_RTC_ALARM_MATCH_DAY) 
    {
        n_alrmr |= (stm32l0_rtc_int_to_bcd[alarm->day] << RTC_ALRMAR_DU_Pos);
    }
    else 
    {
        n_alrmr |= RTC_ALRMAR_MSK4;
    }
    
    while (!(RTC->ISR & RTC_ISR_ALRAWF))
    {
    }
    
    RTC->ALRMAR = n_alrmr;
    RTC->ALRMASSR = 0;
    
    armv6m_atomic_or(&RTC->CR, (RTC_CR_ALRAIE | RTC_CR_ALRAE));
}

void stm32l0_rtc_alarm_detach(void)
{
    stm32l0_rtc_device.alarm_callback = NULL;
    stm32l0_rtc_device.alarm_context = NULL;

    armv6m_atomic_and(&RTC->CR, ~(RTC_CR_ALRAIE | RTC_CR_ALRAE));
    armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_ALRAF | RTC_ISR_INIT));
}

static void stm32l0_rtc_timer_callout(const stm32l0_rtc_calendar_t *stop)
{
    int32_t seconds;
    uint16_t subseconds;
    stm32l0_rtc_timer_t *timer;
    stm32l0_rtc_timer_callback_t callback;

    stm32l0_rtc_calendar_delta(stop, &stm32l0_rtc_device.timer_start, &seconds, &subseconds);
        
    while (stm32l0_rtc_device.timer_queue != NULL)
    {
        timer = stm32l0_rtc_device.timer_queue;

        if ((timer->seconds > seconds) || ((timer->seconds == seconds) && (timer->subseconds > subseconds)))
        {
            stm32l0_rtc_time_subtract(timer->seconds, timer->subseconds, seconds, subseconds, &timer->seconds, &timer->subseconds);
            break;
        }

        stm32l0_rtc_time_subtract(seconds, subseconds, timer->seconds, timer->subseconds, &seconds, &subseconds);

        if (timer->next == timer)
        {
            stm32l0_rtc_device.timer_queue = NULL;
        }
        else
        {
            stm32l0_rtc_device.timer_queue = timer->next;

            timer->next->previous = timer->previous;
            timer->previous->next = timer->next;
        }

        timer->next = NULL;
        timer->previous = NULL;
        timer->seconds = 0;
        timer->subseconds = 0;
        
        callback = timer->callback;
        
        if ((uint32_t)callback & 1)
        {
            (*callback)(timer->context, NULL);
        }
    }
}

static void stm32l0_rtc_timer_routine(void)
{
    if (stm32l0_rtc_device.timer_busy)
    {
        stm32l0_rtc_device.timer_busy = 0;

        stm32l0_rtc_timer_callout(&stm32l0_rtc_device.timer_stop);
    }

    if (armv6m_atomic_sub(&stm32l0_rtc_device.timer_events, 1) == 1)
    {
        if (stm32l0_rtc_device.timer_queue)
        {
            stm32l0_rtc_timer_alarm(&stm32l0_rtc_device.timer_stop);
        }
    }
}

static void stm32l0_rtc_timer_sync(const stm32l0_rtc_calendar_t *stop, const stm32l0_rtc_calendar_t *start)
{
    if (stm32l0_rtc_device.timer_busy)
    {
        stm32l0_rtc_device.timer_busy = 0;

        stm32l0_rtc_timer_callout(stop);
    }

    if (armv6m_atomic_sub(&stm32l0_rtc_device.timer_events, 1) == 1)
    {
        if (stm32l0_rtc_device.timer_queue)
        {
            stm32l0_rtc_timer_alarm(start);
        }
    }
}

static void stm32l0_rtc_timer_sync_1(uint32_t c0, uint32_t c1)
{
    stm32l0_rtc_calendar_t stop;

    if (stm32l0_rtc_device.timer_busy)
    {
        stm32l0_rtc_device.timer_busy = 0;

        stm32l0_rtc_calendar_unpack(c0, c1, &stop);

        stm32l0_rtc_timer_callout(&stop);
    }
}

static void stm32l0_rtc_timer_sync_2(uint32_t c0, uint32_t c1)
{
    stm32l0_rtc_calendar_t start;

    if (armv6m_atomic_sub(&stm32l0_rtc_device.timer_events, 1) == 1)
    {
        if (stm32l0_rtc_device.timer_queue)
        {
            stm32l0_rtc_calendar_unpack(c0, c1, &start);

            stm32l0_rtc_timer_alarm(&start);
        }
    }
}

static void stm32l0_rtc_timer_alarm(const stm32l0_rtc_calendar_t *start)
{
    int32_t seconds, d_seconds;
    uint16_t subseconds, d_subseconds;
    uint32_t primask;
    stm32l0_rtc_calendar_t calendar;

    seconds = stm32l0_rtc_device.timer_queue->seconds;
    subseconds = stm32l0_rtc_device.timer_queue->subseconds;

    if (seconds > STM32L0_RTC_TIMER_MAX_SECONDS)
    {
        seconds = STM32L0_RTC_TIMER_MAX_SECONDS;
        subseconds = 0;
    }

    stm32l0_rtc_device.timer_start = *start;
    stm32l0_rtc_calendar_offset(&stm32l0_rtc_device.timer_start, seconds, subseconds, &stm32l0_rtc_device.timer_stop);
    
    while (!(RTC->ISR & RTC_ISR_ALRBWF))
    {
    }
    
    RTC->ALRMBR = ((stm32l0_rtc_int_to_bcd[stm32l0_rtc_device.timer_stop.seconds] << RTC_ALRMAR_SU_Pos) |
                   (stm32l0_rtc_int_to_bcd[stm32l0_rtc_device.timer_stop.minutes] << RTC_ALRMAR_MNU_Pos) |
                   (stm32l0_rtc_int_to_bcd[stm32l0_rtc_device.timer_stop.hours] << RTC_ALRMAR_HU_Pos) |
                   (stm32l0_rtc_int_to_bcd[stm32l0_rtc_device.timer_stop.day] << RTC_ALRMAR_DU_Pos));
    
    RTC->ALRMBSSR = ((STM32L0_RTC_PREDIV_S - 1) - (stm32l0_rtc_device.timer_stop.subseconds / STM32L0_RTC_PREDIV_A)) | STM32L0_RTC_ALRMSSR_MASKSS;

    primask = __get_PRIMASK();

    __disable_irq();

    if (!stm32l0_rtc_device.timer_events)
    {
        stm32l0_rtc_device.timer_busy = 1;
    
        RTC->CR |= (RTC_CR_ALRBIE | RTC_CR_ALRBE);
    }

    __set_PRIMASK(primask);

    if (stm32l0_rtc_device.timer_busy)
    {
        stm32l0_rtc_get_calendar(&calendar);
        stm32l0_rtc_calendar_delta(&calendar, &stm32l0_rtc_device.timer_stop, &d_seconds, &d_subseconds);
        
        if (d_seconds >= 0)
        {
            /* Ok, here we started an alarm that was in the past (or same time).
             * Hence call the timeout handler and queue the next alarm.
             */
            
            armv6m_atomic_and(&RTC->CR, ~(RTC_CR_ALRBIE | RTC_CR_ALRBE));
            armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_ALRBF | RTC_ISR_INIT));
            
            if (!stm32l0_rtc_device.timer_events)
            {
                armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)stm32l0_rtc_timer_routine, NULL, 0);
            }
        }
        else
        {
            if ((d_seconds == -1) && (d_subseconds == (32768 - STM32L0_RTC_PREDIV_A)))
            {
                /* STM32L0 has an issue with RTC where if the alarm is exactly the next
                 * tick, it will not raise an interrupt. Hence wait till the tick happened,
                 * and if really no ISR fired, call the timeout handler and queue the next
                 * alarm.
                 */
                do
                {
                    stm32l0_rtc_get_calendar(&calendar);
                }
                while (!stm32l0_rtc_device.timer_events && (stm32l0_rtc_calendar_compare(&calendar, &stm32l0_rtc_device.timer_stop) < 0));

                armv6m_atomic_and(&RTC->CR, ~(RTC_CR_ALRBIE | RTC_CR_ALRBE));
                armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_ALRBF | RTC_ISR_INIT));

                if (!stm32l0_rtc_device.timer_events)
                {
                    armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)stm32l0_rtc_timer_routine, NULL, 0);
                }
            }
        }
    }
}

static void stm32l0_rtc_timer_insert(stm32l0_rtc_timer_t *timer, int32_t seconds, uint16_t subseconds, bool absolute)
{
    int32_t d_seconds;
    uint32_t r_seconds;
    uint16_t d_subseconds, r_subseconds;
    stm32l0_rtc_calendar_t calendar;
    stm32l0_rtc_timer_t *element;

    if (timer->next)
    {
        if (timer->next == timer)
        {
            stm32l0_rtc_device.timer_queue = NULL;
        }
        else
        {
            if (timer->next != stm32l0_rtc_device.timer_queue)
            {
                stm32l0_rtc_time_add(timer->next->seconds, timer->next->subseconds, timer->seconds, timer->subseconds, &timer->next->seconds, &timer->next->subseconds);
            }
            
            if (timer == stm32l0_rtc_device.timer_queue)
            {
                stm32l0_rtc_device.timer_queue = timer->next;
            }

            timer->next->previous = timer->previous;
            timer->previous->next = timer->next;
        }
        
        timer->next = NULL;
        timer->previous = NULL;
        timer->seconds = 0;
        timer->subseconds = 0;
    }

    stm32l0_rtc_get_calendar(&calendar);

    if (absolute)
    {
        stm32l0_rtc_calendar_to_time(&calendar, &r_seconds, &r_subseconds);
        stm32l0_rtc_time_offset(r_seconds, r_subseconds, stm32l0_rtc_device.reference_seconds, stm32l0_rtc_device.reference_subseconds, &r_seconds, &r_subseconds);

        if (stm32l0_rtc_time_compare(seconds, subseconds, r_seconds, r_subseconds) <= 0)
        {
            seconds = 0;
            subseconds = 0;
        }
        else
        {
            stm32l0_rtc_time_subtract(seconds, subseconds, r_seconds, r_subseconds, &seconds, &subseconds);
        }

        /* Round down for "at most" semantics.
         */
        subseconds &= ~(STM32L0_RTC_PREDIV_A-1);
    }
    else
    {
        /* Round up for "at least" semantics.
         */
        if (subseconds & (STM32L0_RTC_PREDIV_A-1)) 
        {
            stm32l0_rtc_time_add(seconds, subseconds, 0, (STM32L0_RTC_PREDIV_A - (subseconds & (STM32L0_RTC_PREDIV_A-1))), &seconds, &subseconds);
        }
    }

    if (stm32l0_rtc_device.timer_queue == NULL)
    {
        timer->next = timer;
        timer->previous = timer;

        stm32l0_rtc_device.timer_queue = timer;
        stm32l0_rtc_device.timer_start = calendar;
    }
    else
    {
        stm32l0_rtc_calendar_delta(&calendar, &stm32l0_rtc_device.timer_start, &d_seconds, &d_subseconds);

        stm32l0_rtc_time_add(seconds, subseconds, d_seconds, d_subseconds, &seconds, &subseconds);

        element = stm32l0_rtc_device.timer_queue;

        do
        {
            if ((element->seconds > seconds) || ((element->seconds == seconds) && (element->subseconds > subseconds)))
            {
                stm32l0_rtc_time_subtract(element->seconds, element->subseconds, seconds, subseconds, &element->seconds, &element->subseconds);

                if (stm32l0_rtc_device.timer_queue == element)
                {
                    stm32l0_rtc_device.timer_queue = timer;
                }
                break;
            }

            stm32l0_rtc_time_subtract(seconds, subseconds, element->seconds, element->subseconds, &seconds, &subseconds);

            element = element->next;
        }
        while (stm32l0_rtc_device.timer_queue != element);

        timer->previous = element->previous;
        timer->next = element;
    }

    timer->seconds = seconds;
    timer->subseconds = subseconds;
    timer->callback = (stm32l0_rtc_timer_callback_t)((uint32_t)timer->callback | 1);

    timer->next->previous = timer;
    timer->previous->next = timer;

    if (timer == stm32l0_rtc_device.timer_queue)
    {
        armv6m_atomic_and(&RTC->CR, ~(RTC_CR_ALRBIE | RTC_CR_ALRBE));
        armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_ALRBF | RTC_ISR_INIT));

        if (!stm32l0_rtc_device.timer_events)
        {
            stm32l0_rtc_timer_alarm(&stm32l0_rtc_device.timer_start);
        }
    }
}

static void stm32l0_rtc_timer_insert_1(stm32l0_rtc_timer_t *timer, uint32_t subseconds)
{
    timer->adjust = subseconds;
}

static void stm32l0_rtc_timer_insert_2(stm32l0_rtc_timer_t *timer, uint32_t seconds)
{
    stm32l0_rtc_timer_insert(timer, seconds, (timer->adjust & 0x7fff), !!(timer->adjust & 0x8000));
}

static void stm32l0_rtc_timer_remove(stm32l0_rtc_timer_t *timer)
{
    if (timer->next)
    {
        if (timer->next == timer)
        {
            stm32l0_rtc_device.timer_queue = NULL;
        }
        else
        {
            if (timer->next != stm32l0_rtc_device.timer_queue)
            {
                stm32l0_rtc_time_subtract(timer->next->seconds, timer->next->subseconds, timer->seconds, timer->subseconds, &timer->next->seconds, &timer->next->subseconds);
            }
            
            if (timer == stm32l0_rtc_device.timer_queue)
            {
                stm32l0_rtc_device.timer_queue = timer->next;
            }

            timer->next->previous = timer->previous;
            timer->previous->next = timer->next;
        }
        
        timer->next = NULL;
        timer->previous = NULL;
        timer->seconds = 0;
        timer->subseconds = 0;
        
        armv6m_atomic_and(&RTC->CR, ~(RTC_CR_ALRBIE | RTC_CR_ALRBE));
        armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_ALRBF | RTC_ISR_INIT));

        if (stm32l0_rtc_device.timer_queue)
        {
            if (!stm32l0_rtc_device.timer_events)
            {
                stm32l0_rtc_timer_alarm(&stm32l0_rtc_device.timer_start);
            }
        }
    }
}

static void stm32l0_rtc_timer_release(stm32l0_rtc_timer_t *timer)
{
    stm32l0_rtc_timer_callback_t callback;

    stm32l0_rtc_timer_remove(timer);

    callback = (stm32l0_rtc_timer_callback_t)((uint32_t)timer->callback | 1);

    (*callback)(timer->context, timer);
}

void stm32l0_rtc_timer_reference(uint32_t *p_seconds, uint16_t *p_subseconds)
{
    uint32_t r_seconds;
    uint16_t r_subseconds;

    stm32l0_rtc_calendar_t calendar;

    stm32l0_rtc_get_calendar(&calendar);
    stm32l0_rtc_calendar_to_time(&calendar, &r_seconds, &r_subseconds);
    stm32l0_rtc_time_offset(r_seconds, r_subseconds, stm32l0_rtc_device.reference_seconds, stm32l0_rtc_device.reference_subseconds, &r_seconds, &r_subseconds);

    *p_seconds = r_seconds;
    *p_subseconds = r_subseconds;
}

void stm32l0_rtc_timer_create(stm32l0_rtc_timer_t *timer, stm32l0_rtc_timer_callback_t callback, void *context)
{
    timer->next = NULL;
    timer->previous = NULL;
    timer->callback = callback;
    timer->context = context;
    timer->seconds = 0;
    timer->subseconds = 0;
}

bool stm32l0_rtc_timer_destroy(stm32l0_rtc_timer_t *timer)
{
    bool success = true;

    if (__get_IPSR() == 0)
    {
        armv6m_svcall_1((uint32_t)&stm32l0_rtc_timer_release, (uint32_t)timer);
    }
    else
    {
        success = armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)stm32l0_rtc_timer_release, (void*)timer, 0);
    }

    return success;
}

bool stm32l0_rtc_timer_start(stm32l0_rtc_timer_t *timer, uint32_t seconds, uint16_t subseconds, bool absolute)
{
    bool success = true;

    if (__get_IPSR() == 0)
    {
        armv6m_svcall_4((uint32_t)&stm32l0_rtc_timer_insert, (uint32_t)timer, seconds, subseconds, absolute);
    }
    else
    {
        success = armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)stm32l0_rtc_timer_insert_1, (void*)timer, subseconds | (absolute ? 0x8000 : 0x0000));

        if (success)
        {
            success = armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)stm32l0_rtc_timer_insert_2, (void*)timer, seconds);
        }
    }

    return success;
}

bool stm32l0_rtc_timer_stop(stm32l0_rtc_timer_t *timer)
{
    bool success = true;

    if (__get_IPSR() == 0)
    {
        armv6m_svcall_2((uint32_t)&stm32l0_rtc_timer_remove, (uint32_t)timer, 0);
    }
    else
    {
        success = armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)stm32l0_rtc_timer_remove, (void*)timer, 0);

        if (success)
        {
            armv6m_atomic_and((volatile uint32_t *)&timer->callback, ~1);
        }
    }

    return success;
}

void stm32l0_rtc_wakeup_start(uint32_t timeout, stm32l0_rtc_callback_t callback, void *context)
{
    uint32_t ticks, seconds, rtc_wutr, rtc_cr;

    stm32l0_rtc_device.wakeup_callback = callback;
    stm32l0_rtc_device.wakeup_context = context;

    if (timeout <= 32000)
    {
        ticks = (timeout * 2048) / 1000;
 
        rtc_wutr = ticks - 1;
        rtc_cr = 0;
    }
    else
    {
        seconds = timeout / 1000;
        
        if (seconds <= 65536)
        {
            rtc_wutr = seconds - 1;
            rtc_cr = (RTC_CR_WUCKSEL_2);
        }
        else
        {
            if (seconds > 131072)
            {
                seconds = 131072;
            }

            rtc_wutr = (seconds - 65536) - 1;
            rtc_cr = (RTC_CR_WUCKSEL_2 | RTC_CR_WUCKSEL_1);
        }
    }
    
    while (!(RTC->ISR & RTC_ISR_WUTWF))
    {
    }

    RTC->WUTR = rtc_wutr;

    armv6m_atomic_or(&RTC->CR, (RTC_CR_WUTIE | RTC_CR_WUTE | rtc_cr));
}

void stm32l0_rtc_wakeup_stop(void)
{
    armv6m_atomic_and(&RTC->CR, ~(RTC_CR_WUTIE | RTC_CR_WUTE | RTC_CR_WUCKSEL));
    armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_WUTF | RTC_ISR_INIT));

    EXTI->PR = EXTI_PR_PIF20;
}

bool stm32l0_rtc_wakeup_done(void)
{
  return !(RTC->CR & RTC_CR_WUTE);
}

void stm32l0_rtc_standby(void)
{
    /* Called with IRQs disabled */

    RTC->CR &= ~(RTC_CR_TSIE | RTC_CR_WUTIE | RTC_CR_ALRBIE | RTC_CR_TSE | RTC_CR_WUTE | RTC_CR_ALRBE);
    RTC->ISR = 0;

    EXTI->PR = EXTI_PR_PIF20 | EXTI_PR_PIF19 | EXTI_PR_PIF17;
}

void stm32l0_rtc_reset(void)
{
    /* Called with IRQs disabled */

    RTC->CR &= ~(RTC_CR_TSIE | RTC_CR_WUTIE | RTC_CR_ALRBIE | RTC_CR_ALRAIE | RTC_CR_TSE | RTC_CR_WUTE | RTC_CR_ALRBE | RTC_CR_ALRAE);
    RTC->ISR = 0;

    EXTI->PR = EXTI_PR_PIF20 | EXTI_PR_PIF19 | EXTI_PR_PIF17;

    /* Lock RTC throu reset */
    RTC->WPR = 0x00;
}

static const uint16_t stm32l0_rtc_days_since_month[2][12] = {
    {   0,  31,  59,  90, 120, 151, 181, 212, 243, 273, 304, 334, },
    {   0,  31,  60,  91, 121, 152, 182, 213, 244, 274, 305, 335, },
};

void stm32l0_rtc_calendar_to_time(const stm32l0_rtc_calendar_t *calendar, uint32_t *p_seconds, uint16_t *p_subseconds)
{
    *p_seconds = ((((((((calendar->year * 365 + ((calendar->year +3) / 4)) +
                        stm32l0_rtc_days_since_month[(calendar->year & 3) == 0][calendar->month -1] +
                        (calendar->day -1)) * 24) +
                      calendar->hours) * 60) +
                    calendar->minutes) * 60) +
                  calendar->seconds);
    
    *p_subseconds = calendar->subseconds;
}

void stm32l0_rtc_time_to_calendar(uint32_t seconds, uint16_t subseconds, stm32l0_rtc_calendar_t *p_calendar)
{
    uint32_t minutes, hours, days, months, years;

    p_calendar->subseconds = subseconds;
    p_calendar->seconds = seconds % 60; minutes = seconds / 60;
    p_calendar->minutes = minutes % 60; hours = minutes / 60;
    p_calendar->hours = hours % 24; days = hours / 24;

    years = (days / 365);
    
    if ((years * 365 + ((years +3) / 4)) > days)
    {
        years--;
    }

    days -= (years * 365 + ((years +3) / 4));

    months = (days / 29);

    if ((months >= 12) || (stm32l0_rtc_days_since_month[(years & 3) == 0][months] > days))
    {
        months--;
    }

    days -= stm32l0_rtc_days_since_month[(years & 3) == 0][months];
    
    p_calendar->day = days +1;
    p_calendar->month = months +1;
    p_calendar->year = years;
}

int stm32l0_rtc_calendar_compare(const stm32l0_rtc_calendar_t *a_calendar, const stm32l0_rtc_calendar_t *b_calendar)
{
    int delta;

    /* This code depends upon the ordering withing the stm32l0_rtc_calendar_t struct.
     */
    delta = ((const uint32_t*)a_calendar)[1] - ((const uint32_t*)b_calendar)[1];
    
    if (!delta)
    {
        delta = ((const uint32_t*)a_calendar)[0] - ((const uint32_t*)b_calendar)[0];
    }

    if (delta < 0) delta = -1;
    if (delta > 0) delta =  1;

    return delta;
}

void stm32l0_rtc_calendar_delta(const stm32l0_rtc_calendar_t *a_calendar, const stm32l0_rtc_calendar_t *b_calendar, int32_t *p_seconds, uint16_t *p_subseconds)
{
    int32_t seconds;
    uint16_t subseconds;

    seconds = (((((((((a_calendar->year * 365 + ((a_calendar->year +3) / 4)) +
                      stm32l0_rtc_days_since_month[(a_calendar->year & 3) == 0][a_calendar->month -1] +
                      (a_calendar->day -1)) -
                     ((b_calendar->year * 365 + ((b_calendar->year +3) / 4)) +
                      stm32l0_rtc_days_since_month[(b_calendar->year & 3) == 0][b_calendar->month -1] +
                      (b_calendar->day -1))) * 24) +
                   a_calendar->hours -
                   b_calendar->hours) * 60) +
                 a_calendar->minutes -
                 b_calendar->minutes) * 60) +
               a_calendar->seconds -
               b_calendar->seconds);

    if (a_calendar->subseconds < b_calendar->subseconds)
    {
        subseconds = (32768 + a_calendar->subseconds) - b_calendar->subseconds;

        seconds--;
    }
    else
    {
        subseconds = a_calendar->subseconds - b_calendar->subseconds;
    }

    *p_seconds = seconds;
    *p_subseconds = subseconds;
}

void stm32l0_rtc_calendar_offset(const stm32l0_rtc_calendar_t *a_calendar, int32_t b_seconds, uint16_t b_subseconds, stm32l0_rtc_calendar_t *p_calendar)
{
    uint32_t offset, minutes, hours, days, months, years, seconds;
    uint16_t subseconds;

    if (b_seconds >= 0)
    {
        p_calendar->subseconds = a_calendar->subseconds + b_subseconds;
        p_calendar->seconds = a_calendar->seconds;
        p_calendar->minutes = a_calendar->minutes;
        p_calendar->hours = a_calendar->hours;
        p_calendar->day = a_calendar->day;
        p_calendar->month = a_calendar->month;
        p_calendar->year = a_calendar->year;
        
        if (p_calendar->subseconds >= 32768) 
        {
            p_calendar->subseconds -= 32768;
            
            b_seconds++;
        }
        
        if (b_seconds)
        {
            offset = a_calendar->seconds + b_seconds;
            
            if (offset < 60)
            {
                p_calendar->seconds = offset;
            }
            else
            {
                offset += (a_calendar->minutes * 60);
                
                if (offset < 3600)
                {
                    p_calendar->seconds = offset % 60; minutes = offset / 60;
                    p_calendar->minutes = minutes % 60;
                }
                else
                {
                    offset += (a_calendar->hours * 3600);
                    
                    p_calendar->seconds = offset % 60; minutes = offset / 60;
                    p_calendar->minutes = minutes % 60; hours = minutes / 60;
                    p_calendar->hours = hours % 24; days = hours / 24;
                    
                    if (days)
                    {
                        days += ((a_calendar->year * 365 + ((a_calendar->year +3) / 4)) +
                                 stm32l0_rtc_days_since_month[(a_calendar->year & 3) == 0][a_calendar->month -1] +
                                 (a_calendar->day -1));
                    
                        years = (days / 365);
                    
                        if ((years * 365 + ((years +3) / 4)) > days)
                        {
                            years--;
                        }
                    
                        days -= (years * 365 + ((years +3) / 4));
                    
                        months = (days / 29);
                    
                        if ((months >= 12) || (stm32l0_rtc_days_since_month[(years & 3) == 0][months] > days))
                        {
                            months--;
                        }
                    
                        days -= stm32l0_rtc_days_since_month[(years & 3) == 0][months];
                    
                        p_calendar->day = days +1;
                        p_calendar->month = months +1;
                        p_calendar->year = years;
                    }
                }
            }
        }
    }
    else
    {
        stm32l0_rtc_calendar_to_time(a_calendar, &seconds, &subseconds);
        stm32l0_rtc_time_offset(seconds, subseconds, b_seconds, b_subseconds, &seconds, &subseconds);
        stm32l0_rtc_time_to_calendar(seconds, subseconds, p_calendar);
    }
}

int stm32l0_rtc_time_compare(uint32_t a_seconds, uint16_t a_subseconds, uint32_t b_seconds, uint16_t b_subseconds)
{
    int delta;

    delta = a_seconds - b_seconds;
    
    if (!delta)
    {
        delta = a_subseconds - b_subseconds;
    }

    if (delta < 0) delta = -1;
    if (delta > 0) delta =  1;

    return delta;
}

void stm32l0_rtc_time_delta(uint32_t a_seconds, uint16_t a_subseconds, uint32_t b_seconds, uint16_t b_subseconds, int32_t *p_seconds, uint16_t *p_subseconds)
{
    int32_t seconds;
    uint16_t subseconds;

    subseconds = 32768 + a_subseconds - b_subseconds;
    seconds = a_seconds - b_seconds -1;

    if (subseconds >= 32768)
    {
        subseconds -= 32768;
        seconds++;
    }

    *p_seconds = seconds;
    *p_subseconds = subseconds;
}

void stm32l0_rtc_time_offset(uint32_t a_seconds, uint16_t a_subseconds, int32_t b_seconds, uint16_t b_subseconds, uint32_t *p_seconds, uint16_t *p_subseconds)
{
    uint32_t seconds;
    uint16_t subseconds;

    seconds = a_seconds + b_seconds;
    subseconds = a_subseconds + b_subseconds;

    if (subseconds >= 32678)
    {
        subseconds -= 32768;
        seconds++;
    }

    *p_seconds = seconds;
    *p_subseconds = subseconds;
}

void stm32l0_rtc_time_add(int32_t a_seconds, uint16_t a_subseconds, int32_t b_seconds, uint16_t b_subseconds, int32_t *p_seconds, uint16_t *p_subseconds)
{
    int32_t seconds;
    uint16_t subseconds;

    seconds = a_seconds + b_seconds;
    subseconds = a_subseconds + b_subseconds;

    if (subseconds >= 32678)
    {
        subseconds -= 32768;
        seconds++;
    }

    *p_seconds = seconds;
    *p_subseconds = subseconds;
}

void stm32l0_rtc_time_subtract(int32_t a_seconds, uint16_t a_subseconds, int32_t b_seconds, uint16_t b_subseconds, int32_t *p_seconds, uint16_t *p_subseconds)
{
    int32_t seconds;
    uint16_t subseconds;

    subseconds = 32768 + a_subseconds - b_subseconds;
    seconds = a_seconds - b_seconds -1;

    if (subseconds >= 32768)
    {
        subseconds -= 32768;
        seconds++;
    }

    *p_seconds = seconds;
    *p_subseconds = subseconds;
}

void stm32l0_rtc_time_negate(int32_t seconds, uint16_t subseconds, int32_t *p_seconds, uint16_t *p_subseconds)
{
    subseconds = 32768 - subseconds;
    seconds = -seconds - 1;

    *p_seconds = seconds;
    *p_subseconds = subseconds;
}

void RTC_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PIF17)
    {
        EXTI->PR = EXTI_PR_PIF17;

        if (RTC->ISR & RTC_ISR_ALRAF)
        {
            armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_ALRAF | RTC_ISR_INIT));
            
            if (stm32l0_rtc_device.alarm_callback)
            {
                (*stm32l0_rtc_device.alarm_callback)(stm32l0_rtc_device.alarm_context);
            }
        }

        if (RTC->ISR & RTC_ISR_ALRBF)
        {
            armv6m_atomic_and(&RTC->CR, ~(RTC_CR_ALRBIE | RTC_CR_ALRBE));
            armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_ALRBF | RTC_ISR_INIT));
            
            if (stm32l0_rtc_device.timer_busy)
            {
                armv6m_atomic_add(&stm32l0_rtc_device.timer_events, 1);

                armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)stm32l0_rtc_timer_routine, NULL, 0);
            }
        }
    }

    if (EXTI->PR & EXTI_PR_PIF19)
    {
        EXTI->PR = EXTI_PR_PIF19;
    }

    if (EXTI->PR & EXTI_PR_PIF20)
    {
        EXTI->PR = EXTI_PR_PIF20;

        if (RTC->ISR & RTC_ISR_WUTF)
        {
            armv6m_atomic_and(&RTC->CR, ~(RTC_CR_WUTIE | RTC_CR_WUTE | RTC_CR_WUCKSEL));
            armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_WUTF | RTC_ISR_INIT));

            if (stm32l0_rtc_device.wakeup_callback)
            {
                (*stm32l0_rtc_device.wakeup_callback)(stm32l0_rtc_device.wakeup_context);
            }
        }
    }
}
