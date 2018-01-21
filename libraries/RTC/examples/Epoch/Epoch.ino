/*
  Epoch time example for Arduino Zero and MKR1000

  Demonstrates how to set time using epoch for the Arduino Zero and MKR1000

  This example code is in the public domain

  created by Sandeep Mistry <s.mistry@arduino.cc>
  31 Dec 2015
  modified
  18 Feb 2016
*/

#include <RTC.h>

void setup() {
  Serial.begin(9600);

  RTC.setEpoch(1451606400); // Jan 1, 2016
}

void loop() {
  Serial.print("Unix time = ");
  Serial.println(RTC.getEpoch());

  Serial.print("Seconds since Jan 1 2000 = ");
  Serial.println(RTC.getY2kEpoch());

  // Print date...
  Serial.print(RTC.getDay());
  Serial.print("/");
  Serial.print(RTC.getMonth());
  Serial.print("/");
  Serial.print(RTC.getYear());
  Serial.print("\t");

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
    Serial.print("0");
  }
  Serial.print(number);
}

