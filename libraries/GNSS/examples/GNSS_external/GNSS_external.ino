#include "GNSS.h"

void setup( void )
{

    Serial.begin(9600);

    pinMode(5, OUTPUT);
    digitalWrite(5, HIGH);

    delay(1000);

    GNSS.begin(GNSS.MODE_UBLOX, GNSS.RATE_1HZ);

    while (!GNSS.ready()) { }

    GNSS.setConstellation(CONSTELLATION_GPS_AND_GLONASS);

    while (!GNSS.ready()) { }

    GNSS.setExternal(true);
}

void loop( void )
{
    if (GNSS.available())
    {
	GNSSLocation myLocation = GNSS.location();

	static const char *fixTypeString[] = {
	    "NONE",
	    "TIME",
	    "2D",
	    "3D",
	};

	static const char *fixQualityString[] = {
	    "",
	    "",
	    "/DIFFERENTIAL",
	    "/PRECISE",
	    "/RTK_FIXED",
	    "/RTK_FLOAT",
	    "/ESTIMATED",
	    "/MANUAL",
	    "/SIMULATION",
	};

	Serial.print("LOCATION: ");
	Serial.print(fixTypeString[myLocation.fixType()]);

	if (myLocation.fixType() != GNSSLocation::TYPE_NONE)
	{
	    Serial.print(fixQualityString[myLocation.fixQuality()]);
	    Serial.print(" ");
	    Serial.print(myLocation.year());
	    Serial.print("/");
	    Serial.print(myLocation.month());
	    Serial.print("/");
	    Serial.print(myLocation.day());
	    Serial.print(" ");
	    Serial.print(myLocation.hour());
	    Serial.print(":");
	    Serial.print(myLocation.minute());
	    Serial.print(":");
	    Serial.print(myLocation.second());

	    if (myLocation.fixType() != GNSSLocation::TYPE_TIME)
	    {
		Serial.print(" LLA=");
		Serial.print(myLocation.latitude(), 7);
		Serial.print(",");
		Serial.print(myLocation.longitude(), 7);
		Serial.print(",");
		Serial.print(myLocation.altitude(), 3);
		Serial.print(" EPE=");
		Serial.print(myLocation.ehpe(), 3);
		Serial.print(",");
		Serial.print(myLocation.evpe(), 3);
		Serial.print(" SATELLITES=");
		Serial.print(myLocation.satellites());
		Serial.print(" DOP=");
		Serial.print(myLocation.hdop(), 2);
		Serial.print(",");
		Serial.print(myLocation.vdop(), 2);
	    }
	}

	Serial.println();

	GNSSSatellites mySatellites = GNSS.satellites();

	Serial.print("SATELLITES: ");
	Serial.print(mySatellites.count());
	
	Serial.println();

	for (unsigned int index = 0; index < mySatellites.count(); index++)
	{
	    unsigned int prn = mySatellites.prn(index);

	    static const char *stateString[] = {
		"NONE",
		"SEARCHING",
		"TRACKING",
		"NAVIGATING",
	    };

	    if ((prn >= 1) && (prn <= 32))
	    {
		Serial.print("    ");

		if (prn <= 9)
		{
		    Serial.print("  G");
		    Serial.print(prn);
		}
		else
		{
		    Serial.print(" G");
		    Serial.print(prn);
		}
	    }
	    else if ((prn >= 33) && (prn <= 64))
	    {
		Serial.print("    ");
		Serial.print("S");
		Serial.print(prn +87);
	    }
	    else if ((prn >= 65) && (prn <= 96))
	    {
		Serial.print("    ");

		if ((prn - 64) <= 9)
		{
		    Serial.print("  R");
		    Serial.print(prn -64);
		}
		else
		{
		    Serial.print(" R");
		    Serial.print(prn -64);
		}
	    }
	    else if ((prn >= 193) && (prn <= 197))
	    {
		Serial.print("    ");
		Serial.print("  Q");
		Serial.print(prn -192);
	    }
	    else if (prn == 255)
	    {
		Serial.print("    ");
		Serial.print("R???");
	    }
	    else
	    {
		continue;
	    }

	    Serial.print(": SNR=");
	    Serial.print(mySatellites.snr(index));
	    Serial.print(", ELEVATION=");
	    Serial.print(mySatellites.elevation(index));
	    Serial.print(", AZIMUTH=");
	    Serial.print(mySatellites.azimuth(index));
	    Serial.print(", ");
	    Serial.print(stateString[mySatellites.state(index)]);
	    Serial.println();
	}
    }
}
