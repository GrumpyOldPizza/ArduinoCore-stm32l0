 /*
 * Copyright (c) 2016-2020 Thomas Roell.  All rights reserved.
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

#include "Arduino.h"
#include "RTC.h"
#include "wiring_private.h"

#define Y2K_TO_GPS_OFFSET    630720000
#define Y2K_UTC_OFFSET       13
#define Y2K_UNIX_TIME        946684800
#define UNIX_TO_GPS_OFFSET   -315964800

// no "inline static" in C++11 ...

static int32_t _tz_offset = 0;
static uint8_t _alarm_match = RTC.MATCH_OFF;
static uint8_t _alarm_seconds = 0;
static uint8_t _alarm_minutes = 0;
static uint8_t _alarm_hours = 0;
static uint8_t _alarm_day = 1;
static uint8_t _alarm_month = 1;
static uint8_t _alarm_year = 0;
static uint32_t _alarm_epoch = 0;
static void (*_alarm_callback)(void) = NULL;

RTCClass::RTCClass()
{
}

void RTCClass::begin(bool resetTime)
{
    if (resetTime && !(stm32l0_rtc_status() & (STM32L0_RTC_STATUS_TIME_INTERNAL | STM32L0_RTC_STATUS_TIME_EXTERNAL)))
    {
        stm32l0_rtc_time_write(0, Y2K_TO_GPS_OFFSET + Y2K_UTC_OFFSET, 0, false);
    }
}

uint16_t RTCClass::getMilliSeconds()
{
    uint32_t seconds, ticks;
    
    stm32l0_rtc_time_read(&seconds, &ticks);

    return (1000 * ticks) / STM32L0_RTC_CLOCK_TICKS_PER_SECOND;
}

uint8_t RTCClass::getSeconds()
{
    stm32l0_rtc_tod_t tod;
    
    GetTod(&tod);

    return tod.seconds;
}

uint8_t RTCClass::getMinutes()
{
    stm32l0_rtc_tod_t tod;

    GetTod(&tod);

    return tod.minutes;
}

uint8_t RTCClass::getHours()
{
    stm32l0_rtc_tod_t tod;

    GetTod(&tod);

    return tod.hours;
}

uint8_t RTCClass::getDay()
{
    stm32l0_rtc_tod_t tod;

    GetTod(&tod);

    return tod.day;
}

uint8_t RTCClass::getMonth()
{
    stm32l0_rtc_tod_t tod;

    GetTod(&tod);

    return tod.month;
}

uint8_t RTCClass::getYear()
{
    stm32l0_rtc_tod_t tod;

    GetTod(&tod);

    return tod.year;
}

void RTCClass::getTime(uint8_t &hours, uint8_t &minutes, uint8_t &seconds)
{
    stm32l0_rtc_tod_t tod;

    GetTod(&tod);

    hours = tod.hours;
    minutes = tod.minutes;
    seconds = tod.seconds;
}

void RTCClass::getTime(uint8_t &hours, uint8_t &minutes, uint8_t &seconds, uint16_t &milliSeconds)
{
    stm32l0_rtc_tod_t tod;

    GetTod(&tod);

    hours = tod.hours;
    minutes = tod.minutes;
    seconds = tod.seconds;
    milliSeconds = (1000 * tod.ticks) / STM32L0_RTC_CLOCK_TICKS_PER_SECOND;
}

void RTCClass::getDate(uint8_t &day, uint8_t &month, uint8_t &year)
{
    stm32l0_rtc_tod_t tod;

    GetTod(&tod);

    day = tod.day;
    month = tod.month;
    year = tod.year;
}

void RTCClass::getDateTime(uint8_t &day, uint8_t &month, uint8_t &year, uint8_t &hours, uint8_t &minutes, uint8_t &seconds)
{
    stm32l0_rtc_tod_t tod;

    GetTod(&tod);

    day = tod.day;
    month = tod.month;
    year = tod.year;
    hours = tod.hours;
    minutes = tod.minutes;
    seconds = tod.seconds;
}

void RTCClass::getDateTime(uint8_t &day, uint8_t &month, uint8_t &year, uint8_t &hours, uint8_t &minutes, uint8_t &seconds, uint16_t &milliSeconds)
{
    stm32l0_rtc_tod_t tod;

    GetTod(&tod);

    day = tod.day;
    month = tod.month;
    year = tod.year;
    hours = tod.hours;
    minutes = tod.minutes;
    seconds = tod.seconds;
    milliSeconds = (1000 * tod.ticks) / STM32L0_RTC_CLOCK_TICKS_PER_SECOND;
}

void RTCClass::setSeconds(uint8_t seconds)
{
    stm32l0_rtc_tod_t tod;
    
    GetTod(&tod);

    tod.seconds = seconds;

    SetTod(&tod);
}

void RTCClass::setMinutes(uint8_t minutes)
{
    stm32l0_rtc_tod_t tod;
    
    GetTod(&tod);

    tod.minutes = minutes;

    SetTod(&tod);
}

void RTCClass::setHours(uint8_t hours)
{
    stm32l0_rtc_tod_t tod;
    
    GetTod(&tod);

    tod.hours = hours;

    SetTod(&tod);
}

void RTCClass::setDay(uint8_t day)
{
    stm32l0_rtc_tod_t tod;
    
    GetTod(&tod);

    tod.day = day;

    SetTod(&tod);
}

void RTCClass::setMonth(uint8_t month)
{
    stm32l0_rtc_tod_t tod;
    
    GetTod(&tod);

    tod.month = month;

    SetTod(&tod);
}

void RTCClass::setYear(uint8_t year)
{
    stm32l0_rtc_tod_t tod;
    
    GetTod(&tod);

    tod.year = year;

    SetTod(&tod);
}

void RTCClass::setTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    stm32l0_rtc_tod_t tod;
    
    GetTod(&tod);

    tod.hours = hours;
    tod.minutes = minutes;
    tod.seconds = seconds;

    SetTod(&tod);
}

void RTCClass::setDate(uint8_t day, uint8_t month, uint8_t year)
{
    stm32l0_rtc_tod_t tod;
    
    GetTod(&tod);

    tod.day = day;
    tod.month = month;
    tod.year = year;

    SetTod(&tod);
}

void RTCClass::setDateTime(uint8_t day, uint8_t month, uint8_t year, uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    stm32l0_rtc_tod_t tod;
    
    GetTod(&tod);

    tod.day = day;
    tod.month = month;
    tod.year = year;
    tod.hours = hours;
    tod.minutes = minutes;
    tod.seconds = seconds;

    SetTod(&tod);
}

uint32_t RTCClass::getEpoch()
{
    uint32_t tseconds, tticks;

    stm32l0_rtc_time_read(&tseconds, &tticks);

    return tseconds - UNIX_TO_GPS_OFFSET - stm32l0_rtc_time_to_utc_offset(tseconds);
}

void RTCClass::getEpoch(uint32_t &seconds, uint16_t &milliSeconds)
{
    uint32_t tseconds, tticks;
    
    stm32l0_rtc_time_read(&tseconds, &tticks);

    seconds = tseconds - UNIX_TO_GPS_OFFSET - stm32l0_rtc_time_to_utc_offset(tseconds);
    milliSeconds = (1000 * tticks) / STM32L0_RTC_CLOCK_TICKS_PER_SECOND;
}

void RTCClass::setEpoch(uint32_t seconds)
{
    if (seconds < Y2K_UNIX_TIME) {
        return;
    }

    seconds += UNIX_TO_GPS_OFFSET;

    stm32l0_rtc_time_write(0, seconds + stm32l0_rtc_utc_to_utc_offset(seconds), 0, false);
}

uint32_t RTCClass::getY2kEpoch()
{
    uint32_t tseconds, tticks;

    stm32l0_rtc_time_read(&tseconds, &tticks);

    return tseconds - Y2K_TO_GPS_OFFSET - stm32l0_rtc_time_to_utc_offset(tseconds);
}

void RTCClass::getY2kEpoch(uint32_t &seconds, uint16_t &milliSeconds)
{
    uint32_t tseconds, tticks;
    
    stm32l0_rtc_time_read(&tseconds, &tticks);

    seconds = tseconds - Y2K_TO_GPS_OFFSET - stm32l0_rtc_time_to_utc_offset(tseconds);
    milliSeconds = (1000 * tticks) / STM32L0_RTC_CLOCK_TICKS_PER_SECOND;
}

void RTCClass::setY2kEpoch(uint32_t seconds)
{
    seconds += Y2K_TO_GPS_OFFSET;

    stm32l0_rtc_time_write(0, seconds + stm32l0_rtc_utc_to_utc_offset(seconds), 0, false);
}

uint32_t RTCClass::getGpsEpoch()
{
    uint32_t tseconds, tticks;

    stm32l0_rtc_time_read(&tseconds, &tticks);

    return tseconds;
}

void RTCClass::getGpsEpoch(uint32_t &seconds, uint16_t &milliSeconds)
{
    uint32_t tseconds, tticks;
    
    stm32l0_rtc_time_read(&tseconds, &tticks);

    seconds = tseconds;
    milliSeconds = (1000 * tticks) / STM32L0_RTC_CLOCK_TICKS_PER_SECOND;
}

void RTCClass::setGpsEpoch(uint32_t seconds)
{
    stm32l0_rtc_time_write(0, seconds, 0, false);
}

uint8_t RTCClass::getAlarmSeconds()
{
    return _alarm_seconds;
}

uint8_t RTCClass::getAlarmMinutes()
{
    return _alarm_minutes;
}

uint8_t RTCClass::getAlarmHours()
{
    return _alarm_hours;
}

uint8_t RTCClass::getAlarmDay()
{
    return _alarm_day;
}

uint8_t RTCClass::getAlarmMonth()
{
    return _alarm_month;
}

uint8_t RTCClass::getAlarmYear()
{
    return _alarm_year;
}

void RTCClass::getAlarmTime(uint8_t &hours, uint8_t &minutes, uint8_t &seconds)
{
    hours = _alarm_hours;
    minutes = _alarm_minutes;
    seconds = _alarm_seconds;
}

void RTCClass::getAlarmDate(uint8_t &day, uint8_t &month, uint8_t &year)
{
    day = _alarm_day;
    month = _alarm_month;
    year = _alarm_year;
}

void RTCClass::getAlarmDateTime(uint8_t &day, uint8_t &month, uint8_t &year, uint8_t &hours, uint8_t &minutes, uint8_t &seconds)
{
    day = _alarm_day;
    month = _alarm_month;
    year = _alarm_year;
    hours = _alarm_hours;
    minutes = _alarm_minutes;
    seconds = _alarm_seconds;
}

void RTCClass::setAlarmSeconds(uint8_t seconds)
{
    _alarm_seconds = seconds;
    _alarm_epoch = 0;

    SyncAlarm();
}

void RTCClass::setAlarmMinutes(uint8_t minutes)
{
    _alarm_minutes = minutes;
    _alarm_epoch = 0;

    SyncAlarm();
}

void RTCClass::setAlarmHours(uint8_t hours)
{
    _alarm_hours = hours;
    _alarm_epoch = 0;

    SyncAlarm();
}

void RTCClass::setAlarmDay(uint8_t day)
{
    _alarm_day = day;
    _alarm_epoch = 0;

    SyncAlarm();
}

void RTCClass::setAlarmMonth(uint8_t month)
{
    _alarm_month = month;
    _alarm_epoch = 0;

    SyncAlarm();
}

void RTCClass::setAlarmYear(uint8_t year)
{
    _alarm_year = year;
    _alarm_epoch = 0;

    SyncAlarm();
}

void RTCClass::setAlarmTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    _alarm_hours = hours;
    _alarm_minutes = minutes;
    _alarm_seconds = seconds;
    _alarm_epoch = 0;

    SyncAlarm();
}

void RTCClass::setAlarmDate(uint8_t day, uint8_t month, uint8_t year)
{
    _alarm_day = day;
    _alarm_month = month;
    _alarm_year = year;
    _alarm_epoch = 0;

    SyncAlarm();
}

void RTCClass::setAlarmDateTime(uint8_t day, uint8_t month, uint8_t year, uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    _alarm_day = day;
    _alarm_month = month;
    _alarm_year = year;
    _alarm_hours = hours;
    _alarm_minutes = minutes;
    _alarm_seconds = seconds;
    _alarm_epoch = 0;

    SyncAlarm();
}

void RTCClass::setAlarmEpoch(uint32_t seconds)
{
    if (seconds < Y2K_UNIX_TIME) {
        return;
    }

    _alarm_epoch = seconds + UNIX_TO_GPS_OFFSET;
        
    SyncAlarm();
}

void RTCClass::enableAlarm(AlarmMatch match)
{
    _alarm_match = match;

    SyncAlarm();
}

void RTCClass::disableAlarm()
{
    if (_alarm_match != MATCH_OFF) {
        _alarm_match = MATCH_OFF;

        stm32l0_rtc_alarm_stop();
    }
}

void RTCClass::attachInterrupt(void(*callback)(void))
{
    _alarm_callback = callback;

    SyncAlarm();
}

void RTCClass::attachInterruptWakeup(void(*callback)(void))
{
    armv6m_atomic_or(&g_standbyControl, STM32L0_SYSTEM_STANDBY_ALARM);

    _alarm_callback = callback;

    SyncAlarm();
}

void RTCClass::detachInterrupt()
{
    armv6m_atomic_and(&g_standbyControl, ~STM32L0_SYSTEM_STANDBY_ALARM);
  
    _alarm_callback = NULL; 

    SyncAlarm();
}

int32_t RTCClass::getTzOffset()
{
    return _tz_offset;
}

void RTCClass::setTzOffset(int32_t seconds)
{
    _tz_offset = seconds;
}

int32_t RTCClass::getUtcOffset()
{
    uint32_t seconds, ticks;
    
    stm32l0_rtc_time_read(&seconds, &ticks);

    return stm32l0_rtc_time_to_utc_offset(seconds);
}

void RTCClass::setUtcOffset(int32_t seconds)
{
    stm32l0_rtc_set_utc_offset(seconds, false);
}

uint32_t RTCClass::status()
{
    return stm32l0_rtc_status();
}

void RTCClass::GetTod(stm32l0_rtc_tod_t *tod)
{
    uint32_t seconds, ticks;
    int32_t utc_offset;
    
    stm32l0_rtc_time_read(&seconds, &ticks);

    utc_offset = stm32l0_rtc_time_to_utc_offset(seconds);

    seconds -= Y2K_TO_GPS_OFFSET;

    stm32l0_rtc_time_to_tod(seconds - utc_offset + _tz_offset, ticks, tod);
}

void RTCClass::SetTod(const stm32l0_rtc_tod_t *tod)
{
    uint32_t seconds, ticks;
    int32_t utc_offset;

    stm32l0_rtc_tod_to_time(tod, &seconds, &ticks);

    seconds += Y2K_TO_GPS_OFFSET;

    utc_offset = stm32l0_rtc_time_to_utc_offset(seconds);

    stm32l0_rtc_time_write(0, seconds + utc_offset - _tz_offset, 0, false);
}

void RTCClass::SyncAlarm()
{
    stm32l0_rtc_tod_t tod;
    uint32_t seconds, ticks, period;

    if (_alarm_match != MATCH_OFF) {
        if (_alarm_epoch) {
            seconds = _alarm_epoch;
            ticks = 0;
            
            switch (_alarm_match) {
            default:
            case MATCH_ANY: // Every Second
                period = 1;
                break;
            case MATCH_SS:  // Every Minute
                period = 60;
                break;
            case MATCH_MMSS: // Every Hour
                period = 3600;
                break;
            case MATCH_HHMMSS: // Every Day
                period = 86400;
                break;
            case MATCH_YYMMDDHHMMSS: // Once, on a specific date and a specific time
                period = 0;
                break;
            }
        } else {
            if (_alarm_match != MATCH_YYMMDDHHMMSS) {
                seconds = Y2K_TO_GPS_OFFSET - _tz_offset; // offset by a day to avoid underflow
                ticks = 0;
            
                switch (_alarm_match) {
                default:
                case MATCH_ANY: // Every Second
                    period = 1;
                    break;
                case MATCH_SS:  // Every Minute
                    period = 60;
                    seconds += _alarm_seconds;
                    break;
                case MATCH_MMSS: // Every Hour
                    period = 3600;
                    seconds += (_alarm_seconds + 60 * _alarm_minutes);
                    break;
                case MATCH_HHMMSS: // Every Day
                    period = 86400;
                    seconds += (_alarm_seconds + 60 * _alarm_minutes + 3600 * _alarm_hours);
                    break;
                }
            } else {
                tod.year = _alarm_year;
                tod.month = _alarm_month;
                tod.day = _alarm_day;
                tod.hours = _alarm_hours;
                tod.minutes = _alarm_minutes;
                tod.seconds = _alarm_seconds;
                tod.ticks = 0;
            
                stm32l0_rtc_tod_to_time(&tod, &seconds, &ticks);

                seconds = seconds + Y2K_TO_GPS_OFFSET - _tz_offset;
                period = 0;
            }
        }

        stm32l0_rtc_alarm_start(seconds, ticks, period, STM32L0_RTC_ALARM_MODE_UTC_OFFSET, (stm32l0_rtc_alarm_callback_t)_alarmCallback, (void*)NULL);
    }
}

void RTCClass::_alarmCallback()
{
    if (_alarm_match == MATCH_YYMMDDHHMMSS) {
        // Once, on a specific date and a specific time
        _alarm_match = MATCH_OFF;
    }

    if (_alarm_callback) {
        (*_alarm_callback)();
    }
}

RTCClass RTC;
