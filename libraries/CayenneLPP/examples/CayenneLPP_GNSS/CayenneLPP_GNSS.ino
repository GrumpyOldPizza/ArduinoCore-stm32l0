/*
 * This example code is in the public domain.
 */

#include "LoRaWAN.h"
#include "GNSS.h"
#include "CayenneLPP.h"

const char *appEui = "0101010101010101";
const char *appKey = "2B7E151628AED2A6ABF7158809CF4F3C";
const char *devEui = "0101010101010101";

GNSSLocation myLocation;

CayenneLPP myLPP(64);

void transmitCallback(void)
{
}


void setup( void )
{
    GNSS.begin(Serial1, GNSS.MODE_UBLOX, GNSS.RATE_1HZ);

    while (GNSS.busy()) { }

    LoRaWAN.begin(US915);
    LoRaWAN.joinOTAA(appEui, appKey, devEui);
}

void loop( void )
{
    delay(10000);

    if (!LoRaWAN.busy() && LoRaWAN.joined())
    {
        if (GNSS.location(myLocation) && (myLocation.fixType() == GNSSLocation::TYPE_3D))
        {
            myLPP.reset();
            myLPP.addGPS(0, myLocation.latitude(), myLocation.longitude(), myLocation.altitude());

            LoRaWAN.sendPacket(myLPP.getBuffer(), myLPP.getSize());
        }
    }

}
