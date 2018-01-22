/* Send a confirmed packet every 10 seconds and print out the ACK/NACK status.
 *
 *  The code assumes a US915 setup. For other regions please edit "setup()"
 *  with the appropriate settings (see LoRaWAN_OTAA_<band> examples).
 *
 *  Per default LoRaWAN uses Unconfirmed packets, which of course are subject
 *  to packet loss. However one can request a confirmation from the gateway.
 *  The downside is that network bandwidth is used up for the confirmation,
 *  as well as additional transmissions in case there was no confirmation
 *  received.
 *  
 * This example code is in the public domain.
 */

#include "LoRaWAN.h"

const char *appEui = "0101010101010101";
const char *appKey = "2B7E151628AED2A6ABF7158809CF4F3C";
const char *devEui = "0101010101010101";

void setup( void )
{
    Serial.begin(9600);

    LoRaWAN.begin(US915);
    LoRaWAN.setSubBand(2);
    LoRaWAN.joinOTAA(appEui, appKey, devEui);
}

void loop( void )
{
    delay(10000);

    if (!LoRaWAN.busy() && LoRaWAN.joined())
    {
        LoRaWAN.beginPacket();
        LoRaWAN.write(0xef);
        LoRaWAN.write(0xbe);
        LoRaWAN.write(0xad);
        LoRaWAN.write(0xde);
        LoRaWAN.endPacket(true);

        while (LoRaWAN.busy())
        {
        }

        if (LoRaWAN.confirmed())
        {
            Serial.println("ACK ");
        }
        else
        {
            Serial.println("NACK");
        }
    }
}
