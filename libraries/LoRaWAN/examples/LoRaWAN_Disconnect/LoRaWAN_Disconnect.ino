/* Detect a lost link and rejoin a session.
 *  
 *  If ADR is enabled the disconenct logic tracks a fallback
 *  to the lowest DataRate and highest TxPower. If device
 *  fell all the way back and there is no DownLink for 32
 *  UpLinks, the link is assumed to be lost.
 *
 *  The alternative is to use the LINK_CHECK logic.
 *  Every setLinkCheckLimit() UpLinks a LINK_CHECK is send.
 *  Then the state machine wait for up to setLinkCheckDelay()
 *  UpLinks without receiving an answer to mark the lINK_CHECK
 *  as failed. If setLinkCheckThreshold back to back filed
 *  LINK_CHECK commands are send, the link is assumed to be
 *  dropped.
 *
 *  The code uses rejoinOTAA() to force a JOIN_REQUEST. Using
 *  a normal joinOTAA() would not work in case one would
 *  uwant to preserve a session across RESET/STANDBY.
 *    
 *    
 *  A nice test case is to let a node connect to a gateway, wait
 *  till ADR kicks in (it lowers either the DataRate or the TxPower,
 *  or both), then turn off the gateway for a bit till "DISCONNECTED"
 *  shows up on the serial monitor, and then turn on the gateway 
 *  again.
 *
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
 *  AU915/US915 networks have 64+8 channels. Typical gateways support only
 *  8 (9) channels. Hence it's a good idea to pick the proper channel
 *  subset via select via LoRaWAN.setSubBand(),
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

const char *appEui = "0101010101010101";
const char *appKey = "2B7E151628AED2A6ABF7158809CF4F3C";
const char *devEui = "0101010101010101";

void setup( void )
{
    Serial.begin(9600);
    
    while (!Serial) { }

    LoRaWAN.begin(US915);
    // LoRaWAN.setSubBand(2);
    // LoRaWAN.setDutyCycle(false);
    // LoRaWAN.setAntennaGain(2.0);
    // LoRaWAN.setLinkCheckLimit(32);
    LoRaWAN.joinOTAA(appEui, appKey, devEui);

    Serial.println("JOIN( )");
}

void loop( void )
{
    if (!LoRaWAN.busy())
    {
        if (!LoRaWAN.linkGateways())
        {
            Serial.println("REJOIN( )");
            
            LoRaWAN.rejoinOTAA();
        }
        
        if (LoRaWAN.joined())
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
    }

    delay(10000);
}
