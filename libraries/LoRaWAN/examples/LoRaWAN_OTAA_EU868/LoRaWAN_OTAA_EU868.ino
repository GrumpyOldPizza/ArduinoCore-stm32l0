/* Simple OTAA join for a EU868 LoRaWAN network
 *
 *  EU868 has the problem that only 3 channels are predefined, and hence
 *  the network join is problematic on those crowded channels. Thus 
 *  many gateways (like TTN) do use a set of predefined additional
 *  channels. A typical setup would look like this:
 *
 *    LoRaWAN.begin(EU868);
 *    LoRaWAN.addChannel(3, 867100000, 0, 5);
 *    LoRaWAN.addChannel(4, 867300000, 0, 5);
 *    LoRaWAN.addChannel(5, 867500000, 0, 5);
 *    LoRaWAN.addChannel(6, 867700000, 0, 5);
 *    LoRaWAN.addChannel(7, 867900000, 0, 5);
 *    
 * This example code is in the public domain.
 */

#include "LoRaWAN.h"

const char *appEui = "0101010101010101";
const char *appKey = "2B7E151628AED2A6ABF7158809CF4F3C";
const char *devEui = "0101010101010101";

void setup( void )
{
    LoRaWAN.begin(EU868);
    LoRaWAN.addChannel(3, 867100000, 0, 5);
    LoRaWAN.addChannel(4, 867300000, 0, 5);
    LoRaWAN.addChannel(5, 867500000, 0, 5);
    LoRaWAN.addChannel(6, 867700000, 0, 5);
    LoRaWAN.addChannel(7, 867900000, 0, 5);
    LoRaWAN.setDutyCycle(false);
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
