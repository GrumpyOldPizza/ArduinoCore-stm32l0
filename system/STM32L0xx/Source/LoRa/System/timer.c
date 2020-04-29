/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Timer objects and scheduling management

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/

#include "armv6m.h"
#include "stm32l0xx.h"

#include "stm32l0_rtc.h"

#include "timer.h"

static void TimerCallback( TimerEvent_t *obj )
{
    if (obj->IsRunning)
    {
        obj->IsRunning = false;

        if (obj->Callback)
        {
            (*obj->Callback)();
        }
    }
}

void TimerInit( TimerEvent_t *obj, void ( *callback )( void ) )
{
    stm32l0_rtc_timer_create(&obj->Timer, (stm32l0_rtc_timer_callback_t)TimerCallback, (void*)obj);

    obj->Ticks = 0;
    obj->IsRunning = false;
    obj->Callback = callback;
}

void TimerStart( TimerEvent_t *obj )
{
    if (obj->Ticks)
    {
        obj->IsRunning = true;

        stm32l0_rtc_timer_start(&obj->Timer, obj->Ticks, STM32L0_RTC_TIMER_MODE_RELATIVE);
    }
}

void TimerStop( TimerEvent_t *obj )
{
    stm32l0_rtc_timer_stop(&obj->Timer);

    obj->IsRunning = false;
}

void TimerReset( TimerEvent_t *obj )
{
    TimerStop( obj );
    TimerStart( obj );
}

bool TimerIsRunning( TimerEvent_t *obj )
{
    return obj->IsRunning;
}

void TimerSetValue( TimerEvent_t *obj, uint32_t value )
{
    TimerStop( obj );

    obj->Ticks = stm32l0_rtc_millis_to_ticks(value);
}

TimerTime_t TimerGetCurrentTime( void )
{
    TimerTime_t currentTime;

    currentTime = stm32l0_rtc_clock_to_millis(stm32l0_rtc_clock_read());

    return currentTime;
}

TimerTime_t TimerGetElapsedTime( TimerTime_t savedTime )
{
    TimerTime_t currentTime;

    currentTime = stm32l0_rtc_clock_to_millis(stm32l0_rtc_clock_read());

    return currentTime - savedTime;
}
