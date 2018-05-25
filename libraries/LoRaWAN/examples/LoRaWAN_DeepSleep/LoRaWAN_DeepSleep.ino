/* Put the system into STOP mode after kicking off a JOIN/SEND.
 *  
 *  The code joins the network and sends 50 packets. Then it
 *  shuts down the USB connection (which blocks STOP mode),
 *  and then uses STM32L0.stop() instead of delay() to
 *  wait. STOP mode consumes about 2.1uA for the MCU and
 *  SX1272/SX176. SLEEP mode (which is used during delay())
 *  consumes about 2mA. 
 *
 *  It should be noted that the MCU is powered down into
 *  STOP mode, while SX1272/SX1276 processes the TX/RX1/RX2
 *  sequence, hence limiting peak consumption to about 36mA
 *  (US915, 10dbm) for bursts of 30mS per transmit (DR_4).
 *
 *
 *  When this core is running, STM32L0 will not respond to
 *  the Arduino IDE. You have to toggle RESET while holding
 *  BOOT manually to get into the STM32 BOOTLOADER to be able
 *  to flash another sketch.
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

#include "STM32L0.h"
#include "LoRaWAN.h"

const char *appEui = "0101010101010101";
const char *appKey = "2B7E151628AED2A6ABF7158809CF4F3C";
const char *devEui = "0101010101010101";

unsigned int CountDown = 50;

void setup( void )
{
    Serial.begin(9600);
    
    while (!Serial) { }

    LoRaWAN.begin(US915);
    // LoRaWAN.setSubBand(2);
    // LoRaWAN.setDutyCycle(false);
    // LoRaWAN.setAntennaGain(2.0);
    LoRaWAN.joinOTAA(appEui, appKey, devEui);

    Serial.println("JOIN( )");
}

void loop( void )
{
    if (!LoRaWAN.busy())
    {
        if (CountDown)
        {
            CountDown--;
            
            if (!CountDown)
            {
                Serial.end();
                
                USBDevice.detach();
            }
        }

        if (!LoRaWAN.linkGateways())
        {
            if (CountDown)
            {
                Serial.println("REJOIN( )");
            }

            LoRaWAN.rejoinOTAA();
        }
        
        if (LoRaWAN.joined())
        {
            if (CountDown)
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
            }

            LoRaWAN.beginPacket();
            LoRaWAN.write(0xef);
            LoRaWAN.write(0xbe);
            LoRaWAN.write(0xad);
            LoRaWAN.write(0xde);
            LoRaWAN.endPacket();
        }
    }

    if (CountDown)
    {
        delay(30000);
    }
    else
    {
        STM32L0.stop(30000);
    }
}
