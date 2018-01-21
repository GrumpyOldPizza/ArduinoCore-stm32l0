#include "GNSS.h"

void setup()
{
    Serial.begin(9600);

    while (!Serial) { }

    pinMode(5, OUTPUT);
    digitalWrite(5, HIGH);

    delay(500);

    GNSS.begin(Serial1, GNSS.MODE_UBLOX);

    // GNSS.setPeriodic(10, 60, true);
}

void loop()
{
    if (GNSS.available())
    {
	GNSSLocation myLocation = GNSS.read();

	if (myLocation)
	{
	    Serial.print("latitude = ");
	    Serial.print(myLocation.latitude(), 7);
	    Serial.print(", longitude = ");
	    Serial.print(myLocation.longitude(), 7);
	    Serial.print(", time = ");
	    Serial.print(myLocation.hour());
	    Serial.print("/");
	    Serial.print(myLocation.minute());
	    Serial.print("/");
	    Serial.print(myLocation.second());
	    Serial.println();
	}
    }
}
