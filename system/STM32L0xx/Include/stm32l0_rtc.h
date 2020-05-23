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

#if !defined(_STM32L0_RTC_H)
#define _STM32L0_RTC_H

#include "armv6m.h"

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************************************************/

typedef struct _stm32l0_rtc_capture_t stm32l0_rtc_capture_t;
typedef struct _stm32l0_rtc_timer_t stm32l0_rtc_timer_t;
typedef struct _stm32l0_rtc_tod_t stm32l0_rtc_tod_t;

struct _stm32l0_rtc_capture_t {
    uint32_t dr;
    uint32_t tr;
    uint32_t ssr;
};

struct _stm32l0_rtc_tod_t {
    uint8_t                        year;
    uint8_t                        month;
    uint8_t                        day;
    uint8_t                        hours;
    uint8_t                        minutes;
    uint8_t                        seconds;
    uint16_t                       ticks;
};
  
typedef void (*stm32l0_rtc_callback_t)(void *context);

typedef void (*stm32l0_rtc_alarm_callback_t)(void *context);
typedef void (*stm32l0_rtc_wakeup_callback_t)(void *context);
typedef void (*stm32l0_rtc_timer_callback_t)(void *context);

typedef struct _stm32l0_rtc_timer_t {
    stm32l0_rtc_timer_t * volatile        next;
    stm32l0_rtc_timer_callback_t          callback;
    void                                  *context;
    volatile uint32_t                     clock[2];
} stm32l0_rtc_timer_t;

#define STM32L0_RTC_IRQ_PRIORITY                2

#define STM32L0_RTC_STATUS_TIME_INTERNAL        0x00000001
#define STM32L0_RTC_STATUS_TIME_EXTERNAL        0x00000002
#define STM32L0_RTC_STATUS_UTC_OFFSET_INTERNAL  0x00000004
#define STM32L0_RTC_STATUS_UTC_OFFSET_EXTERNAL  0x00000008

#define STM32L0_RTC_TAMP_CONTROL_EDGE_FALLING   0x00000001
#define STM32L0_RTC_TAMP_CONTROL_EDGE_RISING    0x00000002

#define STM32L0_RTC_STAMP_CONTROL_EDGE_FALLING  0x00000001
#define STM32L0_RTC_STAMP_CONTROL_EDGE_RISING   0x00000002

#define STM32L0_RTC_ALARM_MODE_TIME             0x00000000
#define STM32L0_RTC_ALARM_MODE_UTC_OFFSET       0x00000001

#define STM32L0_RTC_TIMER_MODE_ABSOLUTE         0x00000000
#define STM32L0_RTC_TIMER_MODE_RELATIVE         0x00000001
  
#define STM32L0_RTC_TIMER_INIT(_callback, _context) { NULL, (stm32l0_rtc_timer_callback_t)(_callback), (void*)(_context), { 0, 0 } }

#define STM32L0_RTC_PREDIV_S         2048
#define STM32L0_RTC_PREDIV_A         16

#define STM32L0_RTC_ALRMSSR_MASKSS   (RTC_ALRMBSSR_MASKSS_3 | RTC_ALRMBSSR_MASKSS_1 | RTC_ALRMBSSR_MASKSS_0)

#define STM32L0_RTC_CLOCK_TICKS_PER_SECOND STM32L0_RTC_PREDIV_S

#define STM32L0_RTC_BKP2R_UPLINK_COUNTER_SHIFT      0
#define STM32L0_RTC_BKP2R_UPLINK_COUNTER_MASK       0x000000ff
#define STM32L0_RTC_BKP2R_DOWNLINK_COUNTER_SHIFT    8
#define STM32L0_RTC_BKP2R_DOWNLINK_COUNTER_MASK     0x0000ff00
#define STM32L0_RTC_BKP2R_DATARATE_SHIFT            16
#define STM32L0_RTC_BKP2R_DATARATE_MASK             0x000f0000
#define STM32L0_RTC_BKP2R_TX_POWER_SHIFT            20
#define STM32L0_RTC_BKP2R_TX_POWER_MASK             0x00f00000
#define STM32L0_RTC_BKP2R_REPEAT_SHIFT              24
#define STM32L0_RTC_BKP2R_REPEAT_MASK               0x0f000000
#define STM32L0_RTC_BKP2R_ADR_ENABLE_SHIFT          28
#define STM32L0_RTC_BKP2R_ADR_ENABLE_MASK           0x10000000
#define STM32L0_RTC_BKP2R_UPLINK_COUNTER_PRESENT    0x20000000
#define STM32L0_RTC_BKP2R_DOWNLINK_COUNTER_PRESENT  0x40000000
#define STM32L0_RTC_BKP2R_ADR_PRESENT               0x80000000
  
#define STM32L0_RTC_BKP3R_SECONDS_OFFSET_MASK       0xffffffff
#define STM32L0_RTC_BKP3R_SECONDS_OFFSET_SHIFT      0
#define STM32L0_RTC_BKP4R_LSICLK_MASK               0x000001ff
#define STM32L0_RTC_BKP4R_LSICLK_SHIFT              0
#define STM32L0_RTC_BKP4R_LSICLK_SCALE              128
#define STM32L0_RTC_BKP4R_HSEON_MASK                0x00000200
#define STM32L0_RTC_BKP4R_HSEON_SHIFT               9
#define STM32L0_RTC_BKP4R_HSEON                     STM32L0_RTC_BKP4R_HSEON_MASK 
#define STM32L0_RTC_BKP4R_HSITRIM_MASK              0x00007c00
#define STM32L0_RTC_BKP4R_HSITRIM_SHIFT             10
#define STM32L0_RTC_BKP4R_TIME_WRITTEN_MASK         0x00008000
#define STM32L0_RTC_BKP4R_TIME_WRITTEN_SHIFT        15
#define STM32L0_RTC_BKP4R_TIME_WRITTEN              STM32L0_RTC_BKP4R_TIME_WRITTEN_MASK
#define STM32L0_RTC_BKP4R_UTC_OFFSET_WRITTEN_MASK   0x00010000
#define STM32L0_RTC_BKP4R_UTC_OFFSET_WRITTEN_SHIFT  16
#define STM32L0_RTC_BKP4R_UTC_OFFSET_WRITTEN        STM32L0_RTC_BKP4R_UTC_OFFSET_WRITTEN_MASK
#define STM32L0_RTC_BKP4R_TICKS_OFFSET_MASK         0x0ffe0000
#define STM32L0_RTC_BKP4R_TICKS_OFFSET_SHIFT        17
#define STM32L0_RTC_BKP4R_UTC_OFFSET_MASK           0xf0000000
#define STM32L0_RTC_BKP4R_UTC_OFFSET_SHIFT          28
#define STM32L0_RTC_BKP4R_UTC_OFFSET_BIAS           16

extern void __stm32l0_rtc_initialize(void);
extern void stm32l0_rtc_configure(unsigned int priority);
extern uint32_t stm32l0_rtc_status(void);

extern int32_t stm32l0_rtc_get_calibration(void);
extern void stm32l0_rtc_set_calibration(int32_t calibration);
extern int32_t stm32l0_rtc_get_utc_offset(void);
extern void stm32l0_rtc_set_utc_offset(int32_t utc_offset, bool external);

extern void stm32l0_rtc_clock_capture(stm32l0_rtc_capture_t *data);
extern uint64_t stm32l0_rtc_clock_convert(const stm32l0_rtc_capture_t *data);
extern uint64_t stm32l0_rtc_clock_read();
extern void stm32l0_rtc_clock_to_time(uint64_t clock, uint32_t *p_seconds, uint32_t *p_ticks);

extern uint64_t stm32l0_rtc_time_to_clock(uint32_t seconds, uint32_t ticks);
extern void stm32l0_rtc_time_read(uint32_t *p_seconds, uint32_t *p_ticks);
extern void stm32l0_rtc_time_write(uint64_t clock, uint32_t seconds, uint32_t ticks, bool external);
extern int32_t stm32l0_rtc_time_to_utc_offset(uint32_t seconds);
extern int32_t stm32l0_rtc_utc_to_utc_offset(uint32_t seconds);

extern void stm32l0_rtc_alarm_start(uint32_t seconds, uint32_t ticks, uint32_t period, uint32_t mode, stm32l0_rtc_alarm_callback_t callback, void *context);
extern void stm32l0_rtc_alarm_stop();
extern bool stm32l0_rtc_alarm_done();

extern void stm32l0_rtc_timer_create(stm32l0_rtc_timer_t *timer, stm32l0_rtc_timer_callback_t callback, void *context);
extern bool stm32l0_rtc_timer_destroy(stm32l0_rtc_timer_t *timer);
extern void stm32l0_rtc_timer_start(stm32l0_rtc_timer_t *timer, uint64_t clock, uint32_t mode);
extern void stm32l0_rtc_timer_stop(stm32l0_rtc_timer_t *timer);
extern bool stm32l0_rtc_timer_done(stm32l0_rtc_timer_t *timer);

extern bool stm32l0_rtc_wakeup_start(uint32_t seconds, stm32l0_rtc_wakeup_callback_t callback, void *context);
extern void stm32l0_rtc_wakeup_stop();
extern bool stm32l0_rtc_wakeup_done();

extern bool stm32l0_rtc_tamp_attach(uint16_t pin, uint32_t control, stm32l0_rtc_callback_t callback, void *context);
extern bool stm32l0_rtc_tamp_detach(uint16_t pin);

extern void stm32l0_rtc_stamp_attach(uint32_t control, stm32l0_rtc_callback_t callback, void *context);
extern void stm32l0_rtc_stamp_detach();
extern uint64_t stm32l0_rtc_stamp_read();

extern void stm32l0_rtc_standby(uint32_t control, uint32_t timeout);
extern void stm32l0_rtc_reset(void);

/***********************************************************************************************/

static inline uint32_t stm32l0_rtc_micros_to_ticks(uint32_t micros)
{
    if (micros < 1000000)
    {
        return ((micros * STM32L0_RTC_CLOCK_TICKS_PER_SECOND + 999999) / 1000000);
    }
    else
    {   
        uint32_t seconds;

        seconds = (micros / 1000000);
        micros = micros - seconds * 1000000;

        return (seconds * STM32L0_RTC_CLOCK_TICKS_PER_SECOND) + ((micros * STM32L0_RTC_CLOCK_TICKS_PER_SECOND + 999999) / 1000000);
    }
}

static inline uint32_t stm32l0_rtc_millis_to_ticks(uint32_t millis)
{
    if (millis < 60000)
    {
        return ((millis * STM32L0_RTC_CLOCK_TICKS_PER_SECOND + 999) / 1000);
    }
    else
    {
        uint32_t seconds;

        seconds = (millis / 1000);
        millis = millis - seconds * 1000;

        return (seconds * STM32L0_RTC_CLOCK_TICKS_PER_SECOND) + ((millis * STM32L0_RTC_CLOCK_TICKS_PER_SECOND + 999) / 1000);
    }
}

static inline uint32_t stm32l0_rtc_seconds_to_ticks(uint32_t seconds)
{
    return (seconds * STM32L0_RTC_CLOCK_TICKS_PER_SECOND);
}

static inline uint32_t stm32l0_rtc_ticks_to_micros(uint32_t ticks)
{
    uint32_t seconds;

    seconds = ticks / STM32L0_RTC_CLOCK_TICKS_PER_SECOND;
    ticks = ticks & (STM32L0_RTC_CLOCK_TICKS_PER_SECOND -1);

    return (seconds * 1000000) + ((ticks * 1000000) / STM32L0_RTC_CLOCK_TICKS_PER_SECOND);
}

static inline uint32_t stm32l0_rtc_ticks_to_millis(uint32_t ticks)
{
    uint32_t seconds;

    seconds = ticks / STM32L0_RTC_CLOCK_TICKS_PER_SECOND;
    ticks = ticks & (STM32L0_RTC_CLOCK_TICKS_PER_SECOND -1);

    return (seconds * 1000) + ((ticks * 1000) / STM32L0_RTC_CLOCK_TICKS_PER_SECOND);
}

static inline uint32_t stm32l0_rtc_ticks_to_seconds(uint32_t ticks)
{
    return ticks / STM32L0_RTC_CLOCK_TICKS_PER_SECOND;
}

static inline uint64_t stm32l0_rtc_micros_to_clock(uint32_t micros)
{
    uint32_t seconds;

    seconds = (micros / 1000000);
    micros = micros - seconds * 1000000;

    return ((uint64_t)seconds * STM32L0_RTC_CLOCK_TICKS_PER_SECOND) + ((micros * STM32L0_RTC_CLOCK_TICKS_PER_SECOND + 999999) / 1000000);
}

static inline uint64_t stm32l0_rtc_millis_to_clock(uint32_t millis)
{
    uint32_t seconds;

    seconds = (millis / 1000);
    millis = millis - seconds * 1000;

    return ((uint64_t)seconds * STM32L0_RTC_CLOCK_TICKS_PER_SECOND) + ((millis * STM32L0_RTC_CLOCK_TICKS_PER_SECOND + 999) / 1000);
}

static inline uint64_t stm32l0_rtc_seconds_to_clock(uint32_t seconds)
{
    return ((uint64_t)seconds * STM32L0_RTC_CLOCK_TICKS_PER_SECOND);
}

static inline uint32_t stm32l0_rtc_clock_to_micros(uint64_t clock)
{
    uint32_t seconds, ticks;

    seconds = clock / STM32L0_RTC_CLOCK_TICKS_PER_SECOND;
    ticks = clock & (STM32L0_RTC_CLOCK_TICKS_PER_SECOND -1);

    return (seconds * 1000000) + ((ticks * 1000000) / STM32L0_RTC_CLOCK_TICKS_PER_SECOND);
}

static inline uint32_t stm32l0_rtc_clock_to_millis(uint64_t clock)
{
    uint32_t seconds, ticks;

    seconds = clock / STM32L0_RTC_CLOCK_TICKS_PER_SECOND;
    ticks = clock & (STM32L0_RTC_CLOCK_TICKS_PER_SECOND -1);

    return (seconds * 1000) + ((ticks * 1000) / STM32L0_RTC_CLOCK_TICKS_PER_SECOND);
}

static inline uint32_t stm32l0_rtc_clock_to_seconds(uint64_t clock)
{
    return clock / STM32L0_RTC_CLOCK_TICKS_PER_SECOND;
}

extern void stm32l0_rtc_time_to_tod(uint32_t seconds, uint32_t ticks, stm32l0_rtc_tod_t *p_tod);
extern void stm32l0_rtc_tod_to_time(const stm32l0_rtc_tod_t *tod, uint32_t *p_seconds, uint32_t *p_ticks);

/***********************************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _STM32L0_RTC_H */

