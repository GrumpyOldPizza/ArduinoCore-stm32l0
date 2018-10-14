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
#include "stm32l0_gpio.h"
#include "stm32l0_system.h"

extern void RTC_IRQHandler(void);

typedef struct _stm32l0_rtc_device_t {
    stm32l0_rtc_callback_t  alarm_callback;
    void                    *alarm_context;
    stm32l0_rtc_callback_t  wakeup_callback;
    void                    *wakeup_context;
    stm32l0_rtc_callback_t  tamp1_callback;
    void                    *tamp1_context;
    stm32l0_rtc_callback_t  tamp2_callback;
    void                    *tamp2_context;
    int32_t                 reference_seconds;
    uint16_t                reference_subseconds;
    int16_t                 adjust_subseconds;
    uint32_t                offset_seconds;
    stm32l0_rtc_calendar_t  alarm_stop;
    volatile uint8_t        alarm_busy;
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


static const uint16_t stm32l0_rtc_bcd_year_to_days[160] = {
    0,     366,   731,   1096,  1461,  1827,  2192,  2557,  2922,  3288,  3653,  3653,  3653,  3653,  3653,  3653, 
    3653,  4018,  4383,  4749,  5114,  5479,  5844,  6210,  6575,  6940,  7305,  7305,  7305,  7305,  7305,  7305, 
    7305,  7671,  8036,  8401,  8766,  9132,  9497,  9862,  10227, 10593, 10958, 10958, 10958, 10958, 10958, 10958, 
    10958, 11323, 11688, 12054, 12419, 12784, 13149, 13515, 13880, 14245, 14610, 14610, 14610, 14610, 14610, 14610, 
    14610, 14976, 15341, 15706, 16071, 16437, 16802, 17167, 17532, 17898, 18263, 18263, 18263, 18263, 18263, 18263, 
    18263, 18628, 18993, 19359, 19724, 20089, 20454, 20820, 21185, 21550, 21915, 21915, 21915, 21915, 21915, 21915, 
    21915, 22281, 22646, 23011, 23376, 23742, 24107, 24472, 24837, 25203, 25568, 25568, 25568, 25568, 25568, 25568, 
    25568, 25933, 26298, 26664, 27029, 27394, 27759, 28125, 28490, 28855, 29220, 29220, 29220, 29220, 29220, 29220, 
    29220, 29586, 29951, 30316, 30681, 31047, 31412, 31777, 32142, 32508, 32873, 32873, 32873, 32873, 32873, 32873, 
    32873, 33238, 33603, 33969, 34334, 34699, 35064, 35430, 35795, 36160, 36525, 36525, 36525, 36525, 36525, 36525, 
};

static const uint16_t stm32l0_rtc_bcd_month_to_days[2][32] = {
    {
        0,   0,   31,  59,  90,  120, 151, 181, 212, 243, 243, 243, 243, 243, 243, 243, 
        273, 304, 334, 334, 334, 334, 334, 334, 334, 334, 334, 334, 334, 334, 334, 334, 
    },
    {
        0,   0,   31,  60,  91,  121, 152, 182, 213, 244, 244, 244, 244, 244, 244, 244, 
        274, 305, 335, 335, 335, 335, 335, 335, 335, 335, 335, 335, 335, 335, 335, 335, 
    },
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
static void stm32l0_rtc_timer_alarm(const stm32l0_rtc_calendar_t *start);

void __stm32l0_rtc_initialize(void)
{
    uint32_t r_seconds;
    uint16_t r_subseconds;

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
	RTC->TAMPCR = RTC_TAMPCR_TAMP2NOERASE | RTC_TAMPCR_TAMP1NOERASE | RTC_TAMPCR_TAMPPUDIS;
        
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

    RTC->CR &= ~(RTC_CR_TSIE | RTC_CR_WUTIE | RTC_CR_ALRBIE | RTC_CR_ALRAIE | RTC_CR_TSE | RTC_CR_WUTE | RTC_CR_ALRBE | RTC_CR_ALRAE);
    RTC->TAMPCR &= ~(RTC_TAMPCR_TAMP2IE | RTC_TAMPCR_TAMP2E | RTC_TAMPCR_TAMP1IE | RTC_TAMPCR_TAMP1E);
    RTC->ISR = 0;

    EXTI->PR = EXTI_PR_PIF17 | EXTI_PR_PIF19 | EXTI_PR_PIF20;

    stm32l0_rtc_clock_time(&r_seconds, &r_subseconds);
    stm32l0_rtc_time_delta(0, 0, r_seconds, r_subseconds, &stm32l0_rtc_device.reference_seconds, &stm32l0_rtc_device.reference_subseconds);

    stm32l0_rtc_device.adjust_subseconds = ((int32_t)(RTC->BKP4R & 0xfff00000) >> 20) * STM32L0_RTC_PREDIV_A;
    stm32l0_rtc_device.offset_seconds = (uint32_t)RTC->BKP3R;

    stm32l0_rtc_device.timer_queue = NULL;
}

void stm32l0_rtc_configure(unsigned int priority)
{
    NVIC_SetPriority(RTC_IRQn, priority);
    NVIC_EnableIRQ(RTC_IRQn);

    armv6m_atomic_and(&EXTI->IMR, ~(EXTI_IMR_IM17 | EXTI_IMR_IM19 | EXTI_IMR_IM20));

    EXTI->PR = (EXTI_PR_PIF17 | EXTI_PR_PIF19 | EXTI_PR_PIF20);

    armv6m_atomic_or(&EXTI->RTSR, (EXTI_RTSR_RT17 | EXTI_RTSR_RT19 | EXTI_RTSR_RT20));
    armv6m_atomic_or(&EXTI->IMR, (EXTI_IMR_IM17 | EXTI_IMR_IM19 | EXTI_IMR_IM20));
}

__attribute__((optimize("O3"))) void stm32l0_rtc_clock_capture(uint32_t *p_data)
{
    uint32_t rtc_ssr, rtc_tr, rtc_dr;

    do
    {
        rtc_ssr = RTC->SSR;
        rtc_tr = RTC->TR;
        rtc_dr = RTC->DR;
    }
    while (rtc_ssr != RTC->SSR);

    p_data[0] = rtc_ssr;
    p_data[1] = rtc_tr;
    p_data[2] = rtc_dr;
}

__attribute__((optimize("O3"))) void stm32l0_rtc_clock_convert(uint32_t *data, uint32_t *p_seconds, uint16_t *p_subseconds)
{
    uint32_t rtc_ssr, rtc_tr, rtc_dr;

    rtc_ssr = data[0];
    rtc_tr  = data[1];
    rtc_dr  = data[2];

    *p_seconds = (((stm32l0_rtc_bcd_year_to_days[(rtc_dr & (RTC_DR_YU_Msk | RTC_DR_YT_Msk)) >> RTC_DR_YU_Pos] +
                    stm32l0_rtc_bcd_month_to_days[(((rtc_dr & (RTC_DR_YU_Msk | RTC_DR_YT_Msk)) >> RTC_DR_YU_Pos) & 3) ? 0 : 1][(rtc_dr & (RTC_DR_MU_Msk | RTC_DR_MT_Msk)) >> RTC_DR_MU_Pos] +
                    stm32l0_rtc_bcd_to_int[(rtc_dr & (RTC_DR_DU_Msk | RTC_DR_DT_Msk)) >> RTC_DR_DU_Pos] - 1) * 86400) +
                  (stm32l0_rtc_bcd_to_int[(rtc_tr & (RTC_TR_HU_Msk | RTC_TR_HT_Msk)) >> RTC_TR_HU_Pos] * 3600) +
                  (stm32l0_rtc_bcd_to_int[(rtc_tr & (RTC_TR_MNU_Msk | RTC_TR_MNT_Msk)) >> RTC_TR_MNU_Pos] * 60) +
                  (stm32l0_rtc_bcd_to_int[(rtc_tr & (RTC_TR_SU_Msk | RTC_TR_ST_Msk)) >> RTC_TR_SU_Pos]));
    *p_subseconds = ((STM32L0_RTC_PREDIV_S - 1) - (rtc_ssr & (STM32L0_RTC_PREDIV_S - 1))) * STM32L0_RTC_PREDIV_A;
}

__attribute__((optimize("O3"))) void stm32l0_rtc_clock_time(uint32_t *p_seconds, uint16_t *p_subseconds)
{
    uint32_t rtc_ssr, rtc_tr, rtc_dr;

    do
    {
        rtc_ssr = RTC->SSR;
        rtc_tr = RTC->TR;
        rtc_dr = RTC->DR;
    }
    while (rtc_ssr != RTC->SSR);
    
    *p_seconds = (((stm32l0_rtc_bcd_year_to_days[(rtc_dr & (RTC_DR_YU_Msk | RTC_DR_YT_Msk)) >> RTC_DR_YU_Pos] +
                    stm32l0_rtc_bcd_month_to_days[(((rtc_dr & (RTC_DR_YU_Msk | RTC_DR_YT_Msk)) >> RTC_DR_YU_Pos) & 3) ? 0 : 1][(rtc_dr & (RTC_DR_MU_Msk | RTC_DR_MT_Msk)) >> RTC_DR_MU_Pos] +
                    stm32l0_rtc_bcd_to_int[(rtc_dr & (RTC_DR_DU_Msk | RTC_DR_DT_Msk)) >> RTC_DR_DU_Pos] - 1) * 86400) +
                  (stm32l0_rtc_bcd_to_int[(rtc_tr & (RTC_TR_HU_Msk | RTC_TR_HT_Msk)) >> RTC_TR_HU_Pos] * 3600) +
                  (stm32l0_rtc_bcd_to_int[(rtc_tr & (RTC_TR_MNU_Msk | RTC_TR_MNT_Msk)) >> RTC_TR_MNU_Pos] * 60) +
                  (stm32l0_rtc_bcd_to_int[(rtc_tr & (RTC_TR_SU_Msk | RTC_TR_ST_Msk)) >> RTC_TR_SU_Pos]));
    *p_subseconds = ((STM32L0_RTC_PREDIV_S - 1) - (rtc_ssr & (STM32L0_RTC_PREDIV_S - 1))) * STM32L0_RTC_PREDIV_A;
}

__attribute__((optimize("O3"))) void stm32l0_rtc_clock_calendar(stm32l0_rtc_calendar_t *p_calendar)
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

void stm32l0_rtc_get_time(uint32_t *p_seconds, uint16_t *p_subseconds)
{
    uint32_t r_seconds;
    uint16_t r_subseconds;

    stm32l0_rtc_clock_time(&r_seconds, &r_subseconds);
    stm32l0_rtc_time_offset(r_seconds, r_subseconds, stm32l0_rtc_device.reference_seconds, stm32l0_rtc_device.reference_subseconds, &r_seconds, &r_subseconds);

    *p_seconds = r_seconds;
    *p_subseconds = r_subseconds;
}

static void stm32l0_rtc_modify_calendar(uint32_t mask, const stm32l0_rtc_calendar_t *calendar)
{
    stm32l0_rtc_calendar_t before, after;
    uint32_t seconds, b_seconds, a_seconds;
    uint16_t subseconds, b_subseconds, a_subseconds;

    stm32l0_rtc_clock_time(&seconds, &subseconds);

    if (stm32l0_rtc_device.adjust_subseconds < 0)
    {
        if ((int)subseconds < -stm32l0_rtc_device.adjust_subseconds)
        {
            seconds -= 1;
            subseconds += 32768;
        }

        subseconds += stm32l0_rtc_device.adjust_subseconds;
    }
    else
    {
        subseconds += stm32l0_rtc_device.adjust_subseconds;

        if (subseconds >= 32768)
        {
            seconds += 1;
            subseconds -= 32768;
        }
    }

    b_seconds = seconds + stm32l0_rtc_device.offset_seconds;
    b_subseconds = subseconds;

    stm32l0_rtc_time_to_calendar(b_seconds, b_subseconds, &before);
    
    after.subseconds = before.subseconds;
    after.seconds    = (mask & STM32L0_RTC_CALENDAR_MASK_SECONDS) ? calendar->seconds : before.seconds;
    after.minutes    = (mask & STM32L0_RTC_CALENDAR_MASK_MINUTES) ? calendar->minutes : before.minutes;
    after.hours      = (mask & STM32L0_RTC_CALENDAR_MASK_HOURS)   ? calendar->hours   : before.hours;
    after.day        = (mask & STM32L0_RTC_CALENDAR_MASK_DAY)     ? calendar->day     : before.day;
    after.month      = (mask & STM32L0_RTC_CALENDAR_MASK_MONTH)   ? calendar->month   : before.month;
    after.year       = (mask & STM32L0_RTC_CALENDAR_MASK_YEAR)    ? calendar->year    : before.year;

    stm32l0_rtc_calendar_to_time(&after, &a_seconds, &a_subseconds);

    stm32l0_rtc_device.offset_seconds = a_seconds - seconds;

    RTC->BKP3R = stm32l0_rtc_device.offset_seconds;
}

void stm32l0_rtc_get_calendar(stm32l0_rtc_calendar_t *p_calendar)
{
    uint32_t seconds;
    uint16_t subseconds;

    stm32l0_rtc_clock_time(&seconds, &subseconds);

    if (stm32l0_rtc_device.adjust_subseconds < 0)
    {
        if ((int)subseconds < -stm32l0_rtc_device.adjust_subseconds)
        {
            seconds -= 1;
            subseconds += 32768;
        }

        subseconds += stm32l0_rtc_device.adjust_subseconds;
    }
    else
    {
        subseconds += stm32l0_rtc_device.adjust_subseconds;

        if (subseconds >= 32768)
        {
            seconds += 1;
            subseconds -= 32768;
        }
    }

    seconds = seconds + stm32l0_rtc_device.offset_seconds;

    stm32l0_rtc_time_to_calendar(seconds, subseconds, p_calendar);
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

void stm32l0_rtc_set_adjust(int32_t adjust)
{
    stm32l0_rtc_device.adjust_subseconds = adjust;

    RTC->BKP4R = (RTC->BKP4R & ~0xfff00000) | ((stm32l0_rtc_device.adjust_subseconds / STM32L0_RTC_PREDIV_A) << 20);
}

int32_t stm32l0_rtc_get_adjust(void)
{
    return stm32l0_rtc_device.adjust_subseconds;
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

bool stm32l0_rtc_alarm_start(const stm32l0_rtc_calendar_t *alarm, stm32l0_rtc_callback_t callback, void *context)
{
    uint32_t seconds;
    uint16_t subseconds;
    uint32_t primask;
    stm32l0_rtc_calendar_t calendar;

    armv6m_atomic_and(&RTC->CR, ~(RTC_CR_ALRAIE | RTC_CR_ALRAE));
    armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_ALRAF | RTC_ISR_INIT));

    stm32l0_rtc_device.alarm_callback = callback;
    stm32l0_rtc_device.alarm_context = context;

    stm32l0_rtc_calendar_to_time(alarm, &seconds, &subseconds);

    if (stm32l0_rtc_device.adjust_subseconds < 0)
    {
        if ((int)subseconds < -stm32l0_rtc_device.adjust_subseconds)
        {
            seconds -= 1;
            subseconds += 32768;
        }

        subseconds += stm32l0_rtc_device.adjust_subseconds;
    }
    else
    {
        subseconds += stm32l0_rtc_device.adjust_subseconds;

        if (subseconds >= 32768)
        {
            seconds += 1;
            subseconds -= 32768;
        }
    }

    seconds -= stm32l0_rtc_device.offset_seconds;

    stm32l0_rtc_time_to_calendar(seconds, subseconds, &stm32l0_rtc_device.alarm_stop);

    RTC->ALRMAR = ((stm32l0_rtc_int_to_bcd[stm32l0_rtc_device.alarm_stop.seconds] << RTC_ALRMAR_SU_Pos) |
                   (stm32l0_rtc_int_to_bcd[stm32l0_rtc_device.alarm_stop.minutes] << RTC_ALRMAR_MNU_Pos) |
                   (stm32l0_rtc_int_to_bcd[stm32l0_rtc_device.alarm_stop.hours] << RTC_ALRMAR_HU_Pos) |
                   (stm32l0_rtc_int_to_bcd[stm32l0_rtc_device.alarm_stop.day] << RTC_ALRMAR_DU_Pos));
    
    RTC->ALRMASSR = ((STM32L0_RTC_PREDIV_S - 1) - (stm32l0_rtc_device.alarm_stop.subseconds / STM32L0_RTC_PREDIV_A)) | STM32L0_RTC_ALRMSSR_MASKSS;

    primask = __get_PRIMASK();

    __disable_irq();

    stm32l0_rtc_device.alarm_busy = 1;

    RTC->CR |= (RTC_CR_ALRAIE | RTC_CR_ALRAE);

    __set_PRIMASK(primask);
    
    stm32l0_rtc_clock_calendar(&calendar);
        
    if (stm32l0_rtc_calendar_compare(&calendar, &stm32l0_rtc_device.alarm_stop) >= 0)
    {
        if (stm32l0_rtc_device.alarm_busy)
        {
            armv6m_atomic_and(&RTC->CR, ~(RTC_CR_ALRAIE | RTC_CR_ALRAE));
            armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_ALRAF | RTC_ISR_INIT));
            
            return false;
        }
    }

    return true;
}

void stm32l0_rtc_alarm_stop(void)
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
        stm32l0_rtc_clock_calendar(&calendar);
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
                if (armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)stm32l0_rtc_timer_routine, NULL, 0)) 
                {
                    armv6m_atomic_add(&stm32l0_rtc_device.timer_events, 1);
                }
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
                    stm32l0_rtc_clock_calendar(&calendar);
                }
                while (!stm32l0_rtc_device.timer_events && (stm32l0_rtc_calendar_compare(&calendar, &stm32l0_rtc_device.timer_stop) < 0));

                armv6m_atomic_and(&RTC->CR, ~(RTC_CR_ALRBIE | RTC_CR_ALRBE));
                armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_ALRBF | RTC_ISR_INIT));

                if (!stm32l0_rtc_device.timer_events)
                {
                    if (armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)stm32l0_rtc_timer_routine, NULL, 0)) 
                    {
                        armv6m_atomic_add(&stm32l0_rtc_device.timer_events, 1);
                    }
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
        timer->previous = (void*)~0; // leave as not done
        timer->seconds = 0;
        timer->subseconds = 0;
    }

    stm32l0_rtc_clock_calendar(&calendar);

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

            if (success)
            {
                if (timer->next == NULL)
                {
                    timer->previous = (void*)~0;
                }
            }
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

bool stm32l0_rtc_timer_done(stm32l0_rtc_timer_t *timer)
{
    return (timer->previous == NULL) || !((uint32_t)timer->callback & 1);
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

    armv6m_atomic_modify(&RTC->CR, RTC_CR_WUCKSEL, rtc_cr);
    armv6m_atomic_or(&RTC->CR, RTC_CR_WUTIE | RTC_CR_WUTE);
}

void stm32l0_rtc_wakeup_stop(void)
{
    armv6m_atomic_and(&RTC->CR, ~(RTC_CR_WUTIE | RTC_CR_WUTE));
    armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_WUTF | RTC_ISR_INIT));

    EXTI->PR = EXTI_PR_PIF20;
}

bool stm32l0_rtc_wakeup_done(void)
{
    return !(RTC->CR & RTC_CR_WUTE);
}

bool stm32l0_rtc_tamp_attach(uint16_t pin, uint32_t control, stm32l0_rtc_callback_t callback, void *context)
{
    pin &= STM32L0_GPIO_PIN_IO_MASK;

    if (pin == STM32L0_GPIO_PIN_PC13)
    {
	armv6m_atomic_and(&RTC->TAMPCR, ~(RTC_TAMPCR_TAMP1IE | RTC_TAMPCR_TAMP1TRG | RTC_TAMPCR_TAMP1E));
	armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_TAMP1F | RTC_ISR_INIT));

	stm32l0_rtc_device.tamp1_callback = callback;
	stm32l0_rtc_device.tamp1_context = context;

	if (control & (STM32L0_RTC_TAMP_CONTROL_EDGE_FALLING | STM32L0_RTC_TAMP_CONTROL_EDGE_RISING))
	{
	    if (control & STM32L0_RTC_TAMP_CONTROL_EDGE_FALLING)
	    {
		armv6m_atomic_or(&RTC->TAMPCR, (RTC_TAMPCR_TAMP1IE | RTC_TAMPCR_TAMP1TRG | RTC_TAMPCR_TAMP1E));
	    }
	    else
	    {
		armv6m_atomic_or(&RTC->TAMPCR, (RTC_TAMPCR_TAMP1IE | RTC_TAMPCR_TAMP1E));
	    }
	}

	return true;
    }

    if (pin == STM32L0_GPIO_PIN_PA0)
    {
	armv6m_atomic_and(&RTC->TAMPCR, ~(RTC_TAMPCR_TAMP2IE | RTC_TAMPCR_TAMP2TRG | RTC_TAMPCR_TAMP2E));
	armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_TAMP2F | RTC_ISR_INIT));

	stm32l0_rtc_device.tamp2_callback = callback;
	stm32l0_rtc_device.tamp2_context = context;

	if (control & (STM32L0_RTC_TAMP_CONTROL_EDGE_FALLING | STM32L0_RTC_TAMP_CONTROL_EDGE_RISING))
	{
	    if (control & STM32L0_RTC_TAMP_CONTROL_EDGE_FALLING)
	    {
		armv6m_atomic_or(&RTC->TAMPCR, (RTC_TAMPCR_TAMP2IE | RTC_TAMPCR_TAMP2TRG | RTC_TAMPCR_TAMP2E));
	    }
	    else
	    {
		armv6m_atomic_or(&RTC->TAMPCR, (RTC_TAMPCR_TAMP2IE | RTC_TAMPCR_TAMP2E));
	    }
	}

	return true;
    }

    return false;
}

void stm32l0_rtc_tamp_detach(uint16_t pin)
{
    pin &= STM32L0_GPIO_PIN_IO_MASK;

    if (pin == STM32L0_GPIO_PIN_PC13)
    {
	armv6m_atomic_and(&RTC->TAMPCR, ~(RTC_TAMPCR_TAMP1IE | RTC_TAMPCR_TAMP1E));
	armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_TAMP1F | RTC_ISR_INIT));
    }

    if (pin == STM32L0_GPIO_PIN_PA0)
    {
	armv6m_atomic_and(&RTC->TAMPCR, ~(RTC_TAMPCR_TAMP2IE | RTC_TAMPCR_TAMP2E));
	armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_TAMP2F | RTC_ISR_INIT));
    }
}

void stm32l0_rtc_standby(uint32_t config)
{
    /* Called with IRQs disabled */

    RTC->CR &= ~(RTC_CR_TSIE | RTC_CR_WUTIE | RTC_CR_ALRBIE | RTC_CR_TSE | RTC_CR_WUTE | RTC_CR_ALRBE);

    if (config & STM32L0_SYSTEM_CONFIG_WKUP1)
    {
	RTC->TAMPCR &= ~(RTC_TAMPCR_TAMP2IE | RTC_TAMPCR_TAMP2E);
    }
    
    if (config & STM32L0_SYSTEM_CONFIG_WKUP2)
    {
	RTC->TAMPCR &= ~(RTC_TAMPCR_TAMP1IE | RTC_TAMPCR_TAMP1E);
    }

    RTC->ISR = 0;

    EXTI->PR = EXTI_PR_PIF20 | EXTI_PR_PIF19 | EXTI_PR_PIF17;
}

void stm32l0_rtc_reset(void)
{
    /* Called with IRQs disabled */

    RTC->CR &= ~(RTC_CR_TSIE | RTC_CR_WUTIE | RTC_CR_ALRBIE | RTC_CR_ALRAIE | RTC_CR_TSE | RTC_CR_WUTE | RTC_CR_ALRBE | RTC_CR_ALRAE);
    RTC->TAMPCR &= ~(RTC_TAMPCR_TAMP2IE | RTC_TAMPCR_TAMP2E | RTC_TAMPCR_TAMP1IE | RTC_TAMPCR_TAMP1E);
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

    if (subseconds >= 32768)
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

    if (subseconds >= 32768)
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
    stm32l0_rtc_calendar_t calendar;

    if (EXTI->PR & EXTI_PR_PIF17)
    {
        EXTI->PR = EXTI_PR_PIF17;

        if (RTC->ISR & RTC_ISR_ALRAF)
        {
            stm32l0_rtc_clock_calendar(&calendar);

            if (stm32l0_rtc_calendar_compare(&calendar, &stm32l0_rtc_device.alarm_stop) >= 0)
            {
                armv6m_atomic_and(&RTC->CR, ~(RTC_CR_ALRAIE | RTC_CR_ALRAE));
                armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_ALRAF | RTC_ISR_INIT));

                stm32l0_rtc_device.alarm_busy = 0;

                if (stm32l0_rtc_device.alarm_callback)
                {
                    (*stm32l0_rtc_device.alarm_callback)(stm32l0_rtc_device.alarm_context);
                }
            }
            else
            {
                armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_ALRAF | RTC_ISR_INIT));
            }
        }

        if (RTC->ISR & RTC_ISR_ALRBF)
        {
            armv6m_atomic_and(&RTC->CR, ~(RTC_CR_ALRBIE | RTC_CR_ALRBE));
            armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_ALRBF | RTC_ISR_INIT));
            
            if (stm32l0_rtc_device.timer_busy)
            {
                if (armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)stm32l0_rtc_timer_routine, NULL, 0))
                {
                    armv6m_atomic_add(&stm32l0_rtc_device.timer_events, 1);
                }
            }
        }
    }

    if (EXTI->PR & EXTI_PR_PIF19)
    {
        EXTI->PR = EXTI_PR_PIF19;

        if (RTC->ISR & RTC_ISR_TAMP1F)
        {
	    armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_TAMP1F | RTC_ISR_INIT));
            
	    if (stm32l0_rtc_device.tamp1_callback)
	    {
		(*stm32l0_rtc_device.tamp1_callback)(stm32l0_rtc_device.tamp1_context);
	    }
        }

        if (RTC->ISR & RTC_ISR_TAMP2F)
        {
	    armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_TAMP2F | RTC_ISR_INIT));
            
	    if (stm32l0_rtc_device.tamp2_callback)
	    {
		(*stm32l0_rtc_device.tamp2_callback)(stm32l0_rtc_device.tamp2_context);
	    }
        }
    }

    if (EXTI->PR & EXTI_PR_PIF20)
    {
        EXTI->PR = EXTI_PR_PIF20;

        if (RTC->ISR & RTC_ISR_WUTF)
        {
            armv6m_atomic_and(&RTC->CR, ~(RTC_CR_WUTIE | RTC_CR_WUTE));
            armv6m_atomic_modify(&RTC->ISR, ~RTC_ISR_INIT, ~(RTC_ISR_WUTF | RTC_ISR_INIT));

            if (stm32l0_rtc_device.wakeup_callback)
            {
                (*stm32l0_rtc_device.wakeup_callback)(stm32l0_rtc_device.wakeup_context);
            }
        }
    }
}
