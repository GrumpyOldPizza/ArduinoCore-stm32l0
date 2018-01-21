/*
  Simple RTC for Arduino Zero and MKR1000

  Demonstrates the use of the RTC library for the Arduino Zero and MKR1000

  This example code is in the public domain

  http://arduino.cc/en/Tutorial/SimpleRTC

  created by Arturo Guadalupi <a.guadalupi@arduino.cc>
  15 Jun 2015
  modified 
  18 Feb 2016
  modified by Andrea Richetta <a.richetta@arduino.cc>
  24 Aug 2016
*/

#include <RTC.h>

/* Change these values to set the current initial time */
const byte seconds = 0;
const byte minutes = 0;
const byte hours = 16;

/* Change these values to set the current initial date */
const byte day = 15;
const byte month = 6;
const byte year = 15;

void setup()
{
  Serial.begin(9600);

  // Set the time
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);

  // Set the date
  RTC.setDay(day);
  RTC.setMonth(month);
  RTC.setYear(year);

  // you can use also
  //RTC.setTime(hours, minutes, seconds);
  //RTC.setDate(day, month, year);
}

void loop()
{
  // Print date...
  print2digits(RTC.getDay());
  Serial.print("/");
  print2digits(RTC.getMonth());
  Serial.print("/");
  print2digits(RTC.getYear());
  Serial.print(" ");

  // ...and time
  print2digits(RTC.getHours());
  Serial.print(":");
  print2digits(RTC.getMinutes());
  Serial.print(":");
  print2digits(RTC.getSeconds());

  Serial.println();

  delay(1000);
}



void print2digits(int number) {
  if (number < 10) {
    Serial.print("0"); // print a 0 before if the number is < than 10
  }
  Serial.print(number);
}