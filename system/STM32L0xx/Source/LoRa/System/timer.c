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

void TimerInit( TimerEvent_t *obj, void ( *callback )( void ) )
{
    stm32l0_rtc_timer_create(&obj->rtc_timer, (stm32l0_rtc_timer_callback_t)callback, NULL);

    obj->ReloadValue = 0;
}

void TimerStart( TimerEvent_t *obj )
{
    uint32_t seconds;
    uint16_t subseconds;

    if (obj->ReloadValue)
    {
	stm32l0_rtc_millis_to_time(obj->ReloadValue, &seconds, &subseconds);
	stm32l0_rtc_timer_start(&obj->rtc_timer, seconds, subseconds, false);
    }
}

void TimerStop( TimerEvent_t *obj )
{
    stm32l0_rtc_timer_stop(&obj->rtc_timer);
}

void TimerReset( TimerEvent_t *obj )
{
    TimerStop( obj );
    TimerStart( obj );
}

bool TimerIsRunning( TimerEvent_t *obj )
{
    return !stm32l0_rtc_timer_done(&obj->rtc_timer);
}

void TimerSetValue( TimerEvent_t *obj, uint32_t value )
{
    TimerStop( obj );

    obj->ReloadValue = value;
}

TimerTime_t TimerGetCurrentTime( void )
{
    uint32_t seconds;
    uint16_t subseconds;
    TimerTime_t currentTime;

    stm32l0_rtc_get_time(&seconds, &subseconds);

    currentTime = (seconds * 1000) + ((subseconds * 1000) / 32768);

    return currentTime;
}

TimerTime_t TimerGetElapsedTime( TimerTime_t savedTime )
{
    uint32_t seconds;
    uint16_t subseconds;
    TimerTime_t currentTime;

    stm32l0_rtc_get_time(&seconds, &subseconds);

    currentTime = (seconds * 1000) + ((subseconds * 1000) / 32768);

    return currentTime - savedTime;
}
