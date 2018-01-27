/* LED Blink via TimerMillis
 *  
 *  In setup, start the timerOn instance of the millisecond timer with no delay 
 *  and repeat every 2000 milliseconds.  CallbackOn turns on the led then calls 
 *  the second instance of the millisecond timer with delay of 250 milliseconds 
 *  to execute once per call.
 *   
 *  Net effect is that the led is on for 250 millisconds every 2000 milliseconds
 *    
 * This example code is in the public domain.
 */

#include "TimerMillis.h"

TimerMillis timerOff;
TimerMillis timerOn;

void callbackOff(void)
{
    digitalWrite(LED_BUILTIN, 0);
}

void callbackOn(void)
{
    digitalWrite(LED_BUILTIN, 1);

    timerOff.start(callbackOff, 250);
}

void setup( void )
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 0);

    timerOn.start(callbackOn, 0, 2000);
}

void loop( void )
{
}
