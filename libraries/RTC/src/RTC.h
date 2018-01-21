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

#ifndef _RTC_H
#define _RTC_H

#include "Arduino.h"

class RTCClass {
public:
    enum AlarmMatch: uint32_t {
	MATCH_ANY          = 0,      // Every Second
 	MATCH_SS           = 1,      // Every Minute
	MATCH_MMSS         = 3,      // Every Hour
	MATCH_HHMMSS       = 7,      // Every Day
    };

    RTCClass();

    void enableAlarm(AlarmMatch match);
    void disableAlarm();

    void attachInterrupt(void(*callback)(void));
    void attachInterruptWakeup(void(*callback)(void));
    void detachInterrupt();
  
    // Get Functions
    uint8_t getSeconds();
    uint8_t getMinutes();
    uint8_t getHours();
  
    uint8_t getDay();
    uint8_t getMonth();
    uint8_t getYear();
  
    uint8_t getAlarmSeconds();
    uint8_t getAlarmMinutes();
    uint8_t getAlarmHours();

    uint8_t getAlarmDay();

    // Set Functions
    void setSeconds(uint8_t seconds);
    void setMinutes(uint8_t minutes);
    void setHours(uint8_t hours);
    void setTime(uint8_t hours, uint8_t minutes, uint8_t seconds);

    void setDay(uint8_t day);
    void setMonth(uint8_t month);
    void setYear(uint8_t year);
    void setDate(uint8_t day, uint8_t month, uint8_t year);

    void setTimeDate(uint8_t hours, uint8_t minutes, uint8_t seconds, uint8_t day, uint8_t month, uint8_t year);

    void setAlarmSeconds(uint8_t seconds);
    void setAlarmMinutes(uint8_t minutes);
    void setAlarmHours(uint8_t hours);
    void setAlarmTime(uint8_t hours, uint8_t minutes, uint8_t seconds);

    void setAlarmDay(uint8_t day);

    // Epoch Functions
    uint32_t getEpoch();
    uint32_t getY2kEpoch();
    void setEpoch(uint32_t time);
    void setY2kEpoch(uint32_t time);

    // SubSecond Functions [-32767..32768]
    uint16_t getSubSeconds();
    void adjustSubSeconds(int32_t delta);

    // Calibration Functions [-511..512]
    int32_t getCalibration();
    void setCalibration(int32_t calibration);

private:
    uint8_t _alarm_enable;
    uint8_t _alarm_wakeup;
    uint8_t _alarm_match;
    uint8_t _alarm_day;
    uint8_t _alarm_hours;
    uint8_t _alarm_minutes;
    uint8_t _alarm_seconds;

    void (*_alarm_callback)(void);

    void SyncAlarm();

    static void _alarmCallback(class RTCClass *self);
};

extern RTCClass RTC;

#endif // _RTC_H
