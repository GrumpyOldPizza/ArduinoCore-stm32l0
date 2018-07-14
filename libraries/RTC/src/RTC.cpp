/*
 * Copyright (c) 2016-2018 Thomas Roell.  All rights reserved.
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


#define EPOCH2K_OFFSET 946684800  // This is 1st January 2000, 00:00:00 in epoch time

RTCClass::RTCClass()
{
    _alarm_enable = false;
    _alarm_match = 0;
    _alarm_year = 0;
    _alarm_month = 1;
    _alarm_day = 1;
    _alarm_hours = 0;
    _alarm_minutes = 0;
    _alarm_seconds = 0;

    _alarm_callback = NULL;
}

void RTCClass::enableAlarm(AlarmMatch match)
{
    _alarm_enable = true;
    _alarm_match = match;

    SyncAlarm();
}

void RTCClass::disableAlarm()
{
    if (_alarm_enable) {
        _alarm_enable = false;

        stm32l0_rtc_alarm_stop();
    }
}

void RTCClass::attachInterrupt(void(*callback)(void))
{
    _alarm_callback = callback;

    SyncAlarm();
}

void RTCClass::detachInterrupt()
{
    _alarm_callback = NULL; 

    SyncAlarm();
}

/*
 * Get Functions
 */

uint32_t RTCClass::getSubSeconds()
{
    stm32l0_rtc_calendar_t calendar;

    stm32l0_rtc_get_calendar(&calendar);

    return calendar.subseconds << 17;
}

uint8_t RTCClass::getSeconds()
{
    stm32l0_rtc_calendar_t calendar;

    stm32l0_rtc_get_calendar(&calendar);

    return calendar.seconds;
}

uint8_t RTCClass::getMinutes()
{
    stm32l0_rtc_calendar_t calendar;

    stm32l0_rtc_get_calendar(&calendar);

    return calendar.minutes;
}

uint8_t RTCClass::getHours()
{
    stm32l0_rtc_calendar_t calendar;

    stm32l0_rtc_get_calendar(&calendar);

    return calendar.hours;
}

void RTCClass::getTime(uint8_t &hours, uint8_t &minutes, uint8_t &seconds)
{
    stm32l0_rtc_calendar_t calendar;

    stm32l0_rtc_get_calendar(&calendar);

    hours = calendar.hours;
    minutes = calendar.minutes;
    seconds = calendar.seconds;
}

void RTCClass::getTime(uint8_t &hours, uint8_t &minutes, uint8_t &seconds, uint32_t &subSeconds)
{
    stm32l0_rtc_calendar_t calendar;

    stm32l0_rtc_get_calendar(&calendar);

    hours = calendar.hours;
    minutes = calendar.minutes;
    seconds = calendar.seconds;
    subSeconds = calendar.subseconds << 17;
}

uint8_t RTCClass::getDay()
{
    stm32l0_rtc_calendar_t calendar;

    stm32l0_rtc_get_calendar(&calendar);

    return calendar.day;
}

uint8_t RTCClass::getMonth()
{
    stm32l0_rtc_calendar_t calendar;

    stm32l0_rtc_get_calendar(&calendar);

    return calendar.month;
}

uint8_t RTCClass::getYear()
{
    stm32l0_rtc_calendar_t calendar;

    stm32l0_rtc_get_calendar(&calendar);

    return calendar.year;
}

void RTCClass::getDate(uint8_t &day, uint8_t &month, uint8_t &year)
{
    stm32l0_rtc_calendar_t calendar;

    stm32l0_rtc_get_calendar(&calendar);

    day = calendar.day;
    month = calendar.month;
    year = calendar.year;
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

void RTCClass::getAlarmTime(uint8_t &hours, uint8_t &minutes, uint8_t &seconds)
{
    hours = _alarm_hours;
    minutes = _alarm_minutes;
    seconds = _alarm_seconds;
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

void RTCClass::getAlarmDate(uint8_t &day, uint8_t &month, uint8_t &year)
{
    day = _alarm_day;
    month = _alarm_month;
    year = _alarm_year;
}

/*
 * Set Functions
 */

void RTCClass::setSeconds(uint8_t seconds)
{
    stm32l0_rtc_calendar_t calendar;

    calendar.seconds = seconds;

    stm32l0_rtc_set_calendar(STM32L0_RTC_CALENDAR_MASK_SECONDS, &calendar);
}

void RTCClass::setMinutes(uint8_t minutes)
{
    stm32l0_rtc_calendar_t calendar;

    calendar.minutes = minutes;

    stm32l0_rtc_set_calendar(STM32L0_RTC_CALENDAR_MASK_MINUTES, &calendar);
}

void RTCClass::setHours(uint8_t hours)
{
    stm32l0_rtc_calendar_t calendar;

    calendar.hours = hours;

    stm32l0_rtc_set_calendar(STM32L0_RTC_CALENDAR_MASK_HOURS, &calendar);
}

void RTCClass::setTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    stm32l0_rtc_calendar_t calendar;

    calendar.hours = hours;
    calendar.minutes = minutes;
    calendar.seconds = seconds;

    stm32l0_rtc_set_calendar((STM32L0_RTC_CALENDAR_MASK_SECONDS | STM32L0_RTC_CALENDAR_MASK_MINUTES | STM32L0_RTC_CALENDAR_MASK_HOURS), &calendar);
}

void RTCClass::setDay(uint8_t day)
{
    stm32l0_rtc_calendar_t calendar;

    calendar.day = day;

    stm32l0_rtc_set_calendar(STM32L0_RTC_CALENDAR_MASK_DAY, &calendar);
}

void RTCClass::setMonth(uint8_t month)
{
    stm32l0_rtc_calendar_t calendar;

    calendar.month = month;

    stm32l0_rtc_set_calendar(STM32L0_RTC_CALENDAR_MASK_MONTH, &calendar);
}

void RTCClass::setYear(uint8_t year)
{
    stm32l0_rtc_calendar_t calendar;

    calendar.year = year;

    stm32l0_rtc_set_calendar(STM32L0_RTC_CALENDAR_MASK_YEAR, &calendar);
}

void RTCClass::setDate(uint8_t day, uint8_t month, uint8_t year)
{
    stm32l0_rtc_calendar_t calendar;

    calendar.day = day;
    calendar.month = month;
    calendar.year = year;

    stm32l0_rtc_set_calendar((STM32L0_RTC_CALENDAR_MASK_DAY | STM32L0_RTC_CALENDAR_MASK_MONTH | STM32L0_RTC_CALENDAR_MASK_YEAR), &calendar);
}

void RTCClass::setAlarmSeconds(uint8_t seconds)
{
    _alarm_seconds = seconds;

    SyncAlarm();
}

void RTCClass::setAlarmMinutes(uint8_t minutes)
{
    _alarm_minutes = minutes;

    SyncAlarm();
}

void RTCClass::setAlarmHours(uint8_t hours)
{
    _alarm_hours = hours;

    SyncAlarm();
}

void RTCClass::setAlarmTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    _alarm_hours = hours;
    _alarm_minutes = minutes;
    _alarm_seconds = seconds;

    SyncAlarm();
}

void RTCClass::setAlarmDay(uint8_t day)
{
    _alarm_day = day;

    SyncAlarm();
}

void RTCClass::setAlarmMonth(uint8_t month)
{
    _alarm_month = month;

    SyncAlarm();
}

void RTCClass::setAlarmYear(uint8_t year)
{
    _alarm_year = year;

    SyncAlarm();
}

void RTCClass::setAlarmDate(uint8_t day, uint8_t month, uint8_t year)
{
    _alarm_day = day;
    _alarm_month = month;
    _alarm_year = year;

    SyncAlarm();
}

uint32_t RTCClass::getEpoch()
{
    stm32l0_rtc_calendar_t calendar;
    uint32_t seconds;
    uint16_t subseconds;

    stm32l0_rtc_get_calendar(&calendar);

    stm32l0_rtc_calendar_to_time(&calendar, &seconds, &subseconds);

    return seconds + EPOCH2K_OFFSET;
}

uint32_t RTCClass::getY2kEpoch()
{
    stm32l0_rtc_calendar_t calendar;
    uint32_t seconds;
    uint16_t subseconds;

    stm32l0_rtc_get_calendar(&calendar);

    stm32l0_rtc_calendar_to_time(&calendar, &seconds, &subseconds);

    return seconds;
}

void RTCClass::setEpoch(uint32_t ts)
{
    stm32l0_rtc_calendar_t calendar;

    if (ts < EPOCH2K_OFFSET) {
        return;
    }

    stm32l0_rtc_time_to_calendar(ts - EPOCH2K_OFFSET, 0, &calendar);
    
    stm32l0_rtc_set_calendar((STM32L0_RTC_CALENDAR_MASK_SECONDS | STM32L0_RTC_CALENDAR_MASK_MINUTES | STM32L0_RTC_CALENDAR_MASK_HOURS | STM32L0_RTC_CALENDAR_MASK_DAY | STM32L0_RTC_CALENDAR_MASK_MONTH | STM32L0_RTC_CALENDAR_MASK_YEAR), &calendar);
}

void RTCClass::setY2kEpoch(uint32_t ts)
{
    stm32l0_rtc_calendar_t calendar;

    stm32l0_rtc_time_to_calendar(ts, 0, &calendar);
    
    stm32l0_rtc_set_calendar((STM32L0_RTC_CALENDAR_MASK_SECONDS | STM32L0_RTC_CALENDAR_MASK_MINUTES | STM32L0_RTC_CALENDAR_MASK_HOURS | STM32L0_RTC_CALENDAR_MASK_DAY | STM32L0_RTC_CALENDAR_MASK_MONTH | STM32L0_RTC_CALENDAR_MASK_YEAR), &calendar);
}
    
void  RTCClass::setAlarmEpoch(uint32_t ts)
{
    stm32l0_rtc_calendar_t calendar;

    if (ts < EPOCH2K_OFFSET) {
        return;
    }

    stm32l0_rtc_time_to_calendar(ts - EPOCH2K_OFFSET, 0, &calendar);

    _alarm_year = calendar.year;
    _alarm_month = calendar.month;
    _alarm_day = calendar.day;
    _alarm_hours = calendar.hours;
    _alarm_minutes = calendar.minutes;
    _alarm_seconds = calendar.seconds;

    SyncAlarm();
}

void RTCClass::SyncAlarm()
{
    stm32l0_rtc_calendar_t calendar;

    if (_alarm_enable)
    {
        _alarm_calendar.year = _alarm_year;
        _alarm_calendar.month = _alarm_month;
        _alarm_calendar.day = _alarm_day;
        _alarm_calendar.hours = _alarm_hours;
        _alarm_calendar.minutes = _alarm_minutes;
        _alarm_calendar.seconds = _alarm_seconds;
        _alarm_calendar.subseconds = 0;

        if (_alarm_match != MATCH_YYMMDDHHMMSS)
        {
            stm32l0_rtc_get_calendar(&calendar);

            switch (_alarm_match) {
            case MATCH_ANY: // Every Second
                _alarm_calendar.year = calendar.year;
                _alarm_calendar.month = calendar.month;
                _alarm_calendar.day = calendar.day;
                _alarm_calendar.hours = calendar.hours;
                _alarm_calendar.minutes = calendar.minutes;
                _alarm_calendar.seconds = calendar.seconds;
                break;
            case MATCH_SS:  // Every Minute
                _alarm_calendar.year = calendar.year;
                _alarm_calendar.month = calendar.month;
                _alarm_calendar.day = calendar.day;
                _alarm_calendar.hours = calendar.hours;
                _alarm_calendar.minutes = calendar.minutes;
                break;
            case MATCH_MMSS: // Every Hour
                _alarm_calendar.year = calendar.year;
                _alarm_calendar.month = calendar.month;
                _alarm_calendar.day = calendar.day;
                _alarm_calendar.hours = calendar.hours;
                break;
            case MATCH_HHMMSS: // Every Day
                _alarm_calendar.year = calendar.year;
                _alarm_calendar.month = calendar.month;
                _alarm_calendar.day = calendar.day;
                break;
            case MATCH_DHHMMSS: // Every Month
                _alarm_calendar.year = calendar.year;
                _alarm_calendar.month = calendar.month;
                break;
            case MATCH_MMDDHHMMSS: // Every Year
                _alarm_calendar.year = calendar.year;
                break;
            default:
                break;
            }

            while (!stm32l0_rtc_alarm_start(&_alarm_calendar, (stm32l0_rtc_callback_t)_alarmCallback, (void*)this))
            {
                AdvanceAlarm();
            }
        }
        else
        {
            stm32l0_rtc_alarm_start(&_alarm_calendar, (stm32l0_rtc_callback_t)_alarmCallback, (void*)this);
        }
    }
}

void RTCClass::AdvanceAlarm()
{
    stm32l0_rtc_calendar_t calendar;

    static const uint16_t _alarm_days_after_month[2][12] = {
        {   31,  59,  90, 120, 151, 181, 212, 243, 273, 304, 334, 365 },
        {   31,  60,  91, 121, 152, 182, 213, 244, 274, 305, 335, 366  },
    };

    stm32l0_rtc_get_calendar(&calendar);

    switch (_alarm_match) {
    case MATCH_ANY: // Every Second
	_alarm_calendar.year = calendar.year;
	_alarm_calendar.month = calendar.month;
	_alarm_calendar.day = calendar.day;
	_alarm_calendar.hours = calendar.hours;
	_alarm_calendar.minutes = calendar.minutes;
	_alarm_calendar.seconds = calendar.seconds;
        stm32l0_rtc_calendar_offset(&_alarm_calendar, 1, 0, &_alarm_calendar);
        break;
    case MATCH_SS:  // Every Minute
	_alarm_calendar.year = calendar.year;
	_alarm_calendar.month = calendar.month;
	_alarm_calendar.day = calendar.day;
	_alarm_calendar.hours = calendar.hours;
	_alarm_calendar.minutes = calendar.minutes;
        stm32l0_rtc_calendar_offset(&_alarm_calendar, 60, 0, &_alarm_calendar);
        break;
    case MATCH_MMSS: // Every Hour
	_alarm_calendar.year = calendar.year;
	_alarm_calendar.month = calendar.month;
	_alarm_calendar.day = calendar.day;
	_alarm_calendar.hours = calendar.hours;
	stm32l0_rtc_calendar_offset(&_alarm_calendar, 3600, 0, &_alarm_calendar);
        break;
    case MATCH_HHMMSS: // Every Day
	_alarm_calendar.year = calendar.year;
	_alarm_calendar.month = calendar.month;
	_alarm_calendar.day = calendar.day;
        stm32l0_rtc_calendar_offset(&_alarm_calendar, 86400, 0, &_alarm_calendar);
        break;
    case MATCH_DHHMMSS: // Every Month
	_alarm_calendar.year = calendar.year;
	_alarm_calendar.month = calendar.month;
        stm32l0_rtc_calendar_offset(&_alarm_calendar, _alarm_days_after_month[(_alarm_calendar.year & 3) ? 0 : 1][_alarm_calendar.month -1] * 86400, 0, &_alarm_calendar);
        break;
    case MATCH_MMDDHHMMSS: // Every Year
	_alarm_calendar.year = calendar.year;
        stm32l0_rtc_calendar_offset(&_alarm_calendar, ((_alarm_calendar.year & 3) ? 31536000 : 31633400), 0, &_alarm_calendar);
        break;
    default:
        break;
    }
}

void RTCClass::_alarmCallback(class RTCClass *self)
{

    if (self->_alarm_match == MATCH_YYMMDDHHMMSS)
    {
        // Once, on a specific date and a specific time
        stm32l0_rtc_alarm_stop();
    }
    else
    {
        do 
        {
            self->AdvanceAlarm();
        } 
        while (!stm32l0_rtc_alarm_start(&self->_alarm_calendar, (stm32l0_rtc_callback_t)_alarmCallback, (void*)self));
    }

    if (self->_alarm_callback) {
        (*self->_alarm_callback)();
    }
}

RTCClass RTC;
