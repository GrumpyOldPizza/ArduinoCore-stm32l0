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

#if !defined(_STM32L0_RTC_H)
#define _STM32L0_RTC_H

#include "armv6m.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STM32L0_RTC_CALENDAR_MASK_SECONDS       0x00000001
#define STM32L0_RTC_CALENDAR_MASK_MINUTES       0x00000002
#define STM32L0_RTC_CALENDAR_MASK_HOURS         0x00000004
#define STM32L0_RTC_CALENDAR_MASK_DAY           0x00000008
#define STM32L0_RTC_CALENDAR_MASK_MONTH         0x00000010
#define STM32L0_RTC_CALENDAR_MASK_YEAR          0x00000020
#define STM32L0_RTC_CALENDAR_MASK_TIME          0x00000007
#define STM32L0_RTC_CALENDAR_MASK_DATE          0x00000038
#define STM32L0_RTC_CALENDAR_MASK_ALL           0x0000007f

#define STM32L0_RTC_TAMP_CONTROL_EDGE_FALLING   0x00000001
#define STM32L0_RTC_TAMP_CONTROL_EDGE_RISING    0x00000002

typedef struct __attribute__((aligned(4))) _stm32l0_rtc_calendar_t {
    uint16_t                       subseconds;
    uint8_t                        seconds;
    uint8_t                        minutes;
    uint8_t                        hours;
    uint8_t                        day;
    uint8_t                        month;
    uint8_t                        year;
} stm32l0_rtc_calendar_t;

typedef struct _stm32l0_rtc_timer_t stm32l0_rtc_timer_t;

typedef void (*stm32l0_rtc_callback_t)(void *context);
typedef void (*stm32l0_rtc_timer_callback_t)(void *context, stm32l0_rtc_timer_t *timer);

struct _stm32l0_rtc_timer_t {
    struct _stm32l0_rtc_timer_t           *next;
    struct _stm32l0_rtc_timer_t           *previous;
    volatile stm32l0_rtc_timer_callback_t callback;
    void                                  *context;
    int32_t                               seconds;
    uint16_t                              subseconds;
    volatile uint16_t                     adjust;
};

extern void __stm32l0_rtc_initialize(void);

extern void stm32l0_rtc_configure(unsigned int priority);
extern void stm32l0_rtc_clock_capture(uint32_t *p_data);
extern void stm32l0_rtc_clock_convert(uint32_t *data, uint32_t *p_seconds, uint16_t *p_subseconds);
extern void stm32l0_rtc_clock_time(uint32_t *p_seconds, uint16_t *p_subseconds);
extern void stm32l0_rtc_clock_calendar(stm32l0_rtc_calendar_t *p_calendar);
extern void stm32l0_rtc_get_time(uint32_t *p_seconds, uint16_t *p_subseconds);
extern void stm32l0_rtc_get_calendar(stm32l0_rtc_calendar_t *p_calendar);
extern void stm32l0_rtc_set_calendar(unsigned int mask, const stm32l0_rtc_calendar_t *calendar);
extern uint32_t stm32l0_rtc_get_subseconds(void);
extern int32_t stm32l0_rtc_get_adjust(void);
extern void stm32l0_rtc_set_adjust(int32_t adjust);
extern int32_t stm32l0_rtc_get_calibration(void);
extern void stm32l0_rtc_set_calibration(int32_t calibration);

extern bool stm32l0_rtc_alarm_start(const stm32l0_rtc_calendar_t *alarm, stm32l0_rtc_callback_t callback, void *context);
extern void stm32l0_rtc_alarm_stop(void);

extern void stm32l0_rtc_timer_create(stm32l0_rtc_timer_t *timer, stm32l0_rtc_timer_callback_t callback, void *context);
extern bool stm32l0_rtc_timer_destroy(stm32l0_rtc_timer_t *timer);
extern bool stm32l0_rtc_timer_start(stm32l0_rtc_timer_t *timer, uint32_t seconds, uint16_t subseconds, bool absolute);
extern bool stm32l0_rtc_timer_stop(stm32l0_rtc_timer_t *timer);
extern bool stm32l0_rtc_timer_done(stm32l0_rtc_timer_t *timer);

extern void stm32l0_rtc_wakeup_start(uint32_t timeout, stm32l0_rtc_callback_t callback, void *context);
extern void stm32l0_rtc_wakeup_stop(void);
extern bool stm32l0_rtc_wakeup_done(void);

extern bool stm32l0_rtc_tamp_attach(uint16_t pin, uint32_t control, stm32l0_rtc_callback_t callback, void *context);
extern void stm32l0_rtc_tamp_detach(uint16_t pin);

extern void stm32l0_rtc_standby(uint32_t config);
extern void stm32l0_rtc_reset(void);

#define STM32L0_RTC_PREDIV_S               2048
#define STM32L0_RTC_PREDIV_A               16
#define STM32L0_RTC_ALRMSSR_MASKSS         (RTC_ALRMBSSR_MASKSS_3 | RTC_ALRMBSSR_MASKSS_1 | RTC_ALRMBSSR_MASKSS_0)

#define STM32L0_RTC_TIMER_MAX_SECONDS      (28 * 24 * 3600)

static inline void stm32l0_rtc_micros_to_time(uint32_t micros, uint32_t *p_seconds, uint16_t *p_subseconds)
{
    uint32_t seconds = (micros / 1000000);

    *p_seconds = seconds;
    *p_subseconds = ((micros - seconds * 1000000) * 32768) / 1000000;
}
    
static inline void stm32l0_rtc_millis_to_time(uint32_t millis, uint32_t *p_seconds, uint16_t *p_subseconds)
{
    uint32_t seconds = (millis / 1000);

    *p_seconds = seconds;
    *p_subseconds = ((millis - seconds * 1000) * 32768) / 1000;
}

static inline void stm32l0_rtc_seconds_to_time(uint32_t seconds, uint32_t *p_seconds, uint16_t *p_subseconds)
{
    *p_seconds = seconds;
    *p_subseconds = 0;
}

extern void stm32l0_rtc_calendar_to_time(const stm32l0_rtc_calendar_t *calendar, uint32_t *p_seconds, uint16_t *p_subseconds);
extern void stm32l0_rtc_time_to_calendar(uint32_t seconds, uint16_t subseconds, stm32l0_rtc_calendar_t *p_calendar);

extern int stm32l0_rtc_calendar_compare(const stm32l0_rtc_calendar_t *a_calendar, const stm32l0_rtc_calendar_t *b_calendar);
extern void stm32l0_rtc_calendar_delta(const stm32l0_rtc_calendar_t *a_calendar, const stm32l0_rtc_calendar_t *b_calendar, int32_t *p_seconds, uint16_t *p_subseconds);
extern void stm32l0_rtc_calendar_offset(const stm32l0_rtc_calendar_t *a_calendar, int32_t b_seconds, uint16_t b_subseconds, stm32l0_rtc_calendar_t *p_calendar);

extern int stm32l0_rtc_time_compare(uint32_t a_seconds, uint16_t a_subseconds, uint32_t b_seconds, uint16_t b_subseconds);
extern void stm32l0_rtc_time_delta(uint32_t a_seconds, uint16_t a_subseconds, uint32_t b_seconds, uint16_t b_subseconds, int32_t *p_seconds, uint16_t *p_subseconds);
extern void stm32l0_rtc_time_offset(uint32_t a_seconds, uint16_t a_subseconds, int32_t b_seconds, uint16_t b_subseconds, uint32_t *p_second, uint16_t *p_subseconds);
extern void stm32l0_rtc_time_add(int32_t a_seconds, uint16_t a_subseconds, int32_t b_seconds, uint16_t b_subseconds, int32_t *p_seconds, uint16_t *p_subseconds);
extern void stm32l0_rtc_time_subtract(int32_t a_seconds, uint16_t a_subseconds, int32_t b_seconds, uint16_t b_subseconds, int32_t *p_seconds, uint16_t *p_subseconds);
extern void stm32l0_rtc_time_negate(int32_t seconds, uint16_t subseconds, int32_t *p_seconds, uint16_t *p_subseconds);


#ifdef __cplusplus
}
#endif

#endif /* _STM32L0_RTC_H */

