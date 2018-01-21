/*
  Simple RTC Alarm for Arduino Zero and MKR1000

  Demonstrates how to set an RTC alarm for the Arduino Zero and MKR1000

  This example code is in the public domain

  http://arduino.cc/en/Tutorial/SimpleRTCAlarm

  created by Arturo Guadalupi <a.guadalupi@arduino.cc>
  25 Sept 2015
  
  modified
  21 Oct 2015
*/

#include <RTC.h>

/* Change these values to set the current initial time */
const byte seconds = 0;
const byte minutes = 0;
const byte hours = 16;

/* Change these values to set the current initial date */
const byte day = 25;
const byte month = 9;
const byte year = 15;

void setup()
{
  Serial.begin(9600);

  RTC.setTime(hours, minutes, seconds);
  RTC.setDate(day, month, year);

  RTC.setAlarmTime(16, 0, 10);
  RTC.enableAlarm(RTC.MATCH_HHMMSS);
  
  RTC.attachInterrupt(alarmMatch);
}

void loop()
{

}

void alarmMatch()
{
  Serial.println("Alarm Match!");
}
