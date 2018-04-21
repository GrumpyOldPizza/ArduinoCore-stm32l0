/* ADR disable to conserve power.
 *  
 *  The code assumes a US915 setup. For other regions please edit "setup()"
 *  with the appropriate settings (see LoRaWAN_OTAA_<band> examples).
 *
 *  Per default adaptive rate control (ADR) is enabled, which means
 *  the gateway and the node try to optimize DataRate and TxPower to
 *  minimize node power consumption. In practice that often does not
 *  work too well.
 *
 *  This example disabled ADR and controls DataRate and TxPower
 *  explicitly. The ranges are region specific (FSK excluded):
 *
 *           DataRate      TxPower
 *
 *  AS923    DR_0 - DR_6     2dbm - 16dbm
 *  AU915    DR_0 - DR_4    10dbm - 20dbm
 *  EU868    DR_0 - DR_6     2dbm - 20dbm
 *  IN865    DR_0 - DR_5    10dbm - 20dbm
 *  KR920    DR_0 - DR_5     2dbm - 16dbm
 *  US915    DR_0 - DR_4    10dbm - 20dbm
 *    
 *  In the code below, DR_1 and 14dbm are used.    
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
    LoRaWAN.setADR(false);
    LoRaWAN.setDataRate(1);
    LoRaWAN.setTxPower(14);
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
