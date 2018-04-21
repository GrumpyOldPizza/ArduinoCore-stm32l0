/* Simple OTAA join for a US915 LoRaWAN network
 *  
 *  In setup() below please replace the argument to LoRaWAN.begin()
 *  with your appropriate region specific band:
 *
 *  AS923
 *  AU915
 *  EU868
 *  IN865
 *  KR920
 *  US915
 *
 *  Also please check with the LoRaWAN_OTAA_<band>.ino examples for 
 *  typical region specific initialization.
 *
 *  AU915/US915 networks have 64+8 channels. Typical gateways support only
 *  8 (9) channels. Hence one has to select via LoRaWAN.setSubBand(),
 *  which set of 8 channels out of the 64 are in use.
 *    
 * This example code is in the public domain.
 */

#include "LoRaWAN.h"

const char *appEui = "0101010101010101";
const char *appKey = "2B7E151628AED2A6ABF7158809CF4F3C";
const char *devEui = "0101010101010101";

void setup( void )
{
    LoRaWAN.begin(US915);
    LoRaWAN.setSubBand(2);
}

void loop( void )
{
    if (!LoRaWAN.busy())
    {
        if (!LoRaWAN.joined())
        {
            LoRaWAN.joinOTAA(appEui, appKey, devEui);
        }
        else
        {
            LoRaWAN.beginPacket();
            LoRaWAN.write(0xef);
            LoRaWAN.write(0xbe);
            LoRaWAN.write(0xad);
            LoRaWAN.write(0xde);
            LoRaWAN.endPacket();
        }
    }

    delay(10000);
}
