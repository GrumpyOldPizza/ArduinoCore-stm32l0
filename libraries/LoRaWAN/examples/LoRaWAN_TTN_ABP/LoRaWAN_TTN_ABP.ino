/* Simple ABB join for TheThingNetwork LoRaWAN network
 *
 *  Uncomment one of the region defined below to select the
 *  proper radio setup.
 *    
 *  EU868/IN865 have duty cycle restrictions. For debugging it makes sense
 *  to disable those via setDutyCycle(false);
 *    
 *  For an external antenna one should set the proper antenna gain
 *  (default is 2.0) via setAntennaGain().
 *    
 *  Please edit the keys below as they are just debugging samples.
 *    
 *    
 * This example code is in the public domain.
 */

#include "LoRaWAN.h"

// #define REGION_AS923_920_923 /* Japan, Malaysia, Singapore */
// #define REGION_AS923_923_925 /* Brunei, Cambodia, Hong Kong, Indonesia, Laos, Taiwan, Thailand, Vietnam */
// #define REGION_AU915
// #define REGION_EU868
// #define REGION_IN865
// #define REGION_KR920
// #define REGION_US915

const char *devAddr = "0100000A";
const char *nwkSKey = "2B7E151628AED2A6ABF7158809CF4F3C";
const char *appSKey = "2B7E151628AED2A6ABF7158809CF4F3C";

void setup( void )
{
#if defined(REGION_AS923_920_923)
    LoRaWAN.begin(AS923);
    LoRaWAN.addChannel(2, 922200000, 0, 5);
    LoRaWAN.addChannel(3, 922400000, 0, 5);
    LoRaWAN.addChannel(4, 922600000, 0, 5);
    LoRaWAN.addChannel(5, 922800000, 0, 5);
    LoRaWAN.addChannel(6, 923000000, 0, 5);
    LoRaWAN.addChannel(7, 922000000, 0, 5);
    LoRaWAN.addChannel(8, 922100000, 6, 6);
    LoRaWAN.addChannel(9, 921800000, 7, 7);
#endif

#if defined(REGION_AS923_920_923)
    LoRaWAN.begin(AS923);
    LoRaWAN.addChannel(2, 923600000, 0, 5);
    LoRaWAN.addChannel(3, 923800000, 0, 5);
    LoRaWAN.addChannel(4, 924000000, 0, 5);
    LoRaWAN.addChannel(5, 924200000, 0, 5);
    LoRaWAN.addChannel(6, 924400000, 0, 5);
    LoRaWAN.addChannel(7, 924600000, 0, 5);
    LoRaWAN.addChannel(8, 924500000, 6, 6);
    LoRaWAN.addChannel(9, 924800000, 7, 7);
#endif

#if defined(REGION_AU915)
    LoRaWAN.begin(AU915);
    LoRaWAN.setSubBand(2);
#endif

#if defined(REGION_EU868)
    LoRaWAN.begin(EU868);
    LoRaWAN.addChannel(1, 868300000, 0, 6);
    LoRaWAN.addChannel(3, 867100000, 0, 5);
    LoRaWAN.addChannel(4, 867300000, 0, 5);
    LoRaWAN.addChannel(5, 867500000, 0, 5);
    LoRaWAN.addChannel(6, 867700000, 0, 5);
    LoRaWAN.addChannel(7, 867900000, 0, 5);
    LoRaWAN.addChannel(8, 868800000, 7, 7);
    LoRaWAN.setRX2Channel(869525000, 3);
#endif

#if defined(REGION_IN865)
    LoRaWAN.begin(IN865);
#endif

#if defined(REGION_KR920)
    LoRaWAN.begin(KR920);
    LoRaWAN.addChannel(3, 922700000, 0, 5);
    LoRaWAN.addChannel(4, 922900000, 0, 5);
    LoRaWAN.addChannel(5, 923100000, 0, 5);
    LoRaWAN.addChannel(6, 923300000, 0, 5);
#endif

#if defined(REGION_US915)
    LoRaWAN.begin(US915);
    LoRaWAN.setSubBand(2);
#endif

    // LoRaWAN.setDutyCycle(false);     
    // LoRaWAN.setAntennaGain(2.0);
    LoRaWAN.joinABP(devAddr, nwkSKey, appSKey);

    Serial.println("JOIN( )");
}

void loop( void )
{
    if (LoRaWAN.joined() && !LoRaWAN.busy())
    {
        Serial.print("TRANSMIT( ");
        Serial.print("TimeOnAir: ");
        Serial.print(LoRaWAN.getTimeOnAir());
        Serial.print(", NextTxTime: ");
        Serial.print(LoRaWAN.getNextTxTime());
        Serial.print(", MaxPayloadSize: ");
        Serial.print(LoRaWAN.getMaxPayloadSize());
        Serial.print(", DR: ");
        Serial.print(LoRaWAN.getDataRate());
        Serial.print(", TxPower: ");
        Serial.print(LoRaWAN.getTxPower(), 1);
        Serial.print("dbm, UpLinkCounter: ");
        Serial.print(LoRaWAN.getUpLinkCounter());
        Serial.print(", DownLinkCounter: ");
        Serial.print(LoRaWAN.getDownLinkCounter());
        Serial.println(" )");

        LoRaWAN.beginPacket();
        LoRaWAN.write(0xef);
        LoRaWAN.write(0xbe);
        LoRaWAN.write(0xad);
        LoRaWAN.write(0xde);
        LoRaWAN.endPacket();
    }

    delay(10000);
}
