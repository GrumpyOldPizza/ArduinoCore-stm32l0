/* Use a TimerMillis object to schedule transissions in the background.
 *  
 *  The code assumes a US915 setup. For other regions please edit "setup()"
 *  with the appropriate settings (see LoRaWAN_OTAA_<band> examples).
 *  
 *  Idea is to start a periodic timer, where every 10 seconds a packet
 *  is send to the gateway in the background. The main "loop()" could
 *  be reading sensors in the forground, or use STM32L0.stop() to enter
 *  STOP mode.
 *    
 * This example code is in the public domain.
 */

#include "LoRaWAN.h"
#include "TimerMillis.h"

const char *appEui = "0101010101010101";
const char *appKey = "2B7E151628AED2A6ABF7158809CF4F3C";
const char *devEui = "0101010101010101";

TimerMillis transmitTimer;

void transmitCallback(void)
{
    if (!LoRaWAN.busy() && LoRaWAN.joined())
    {
        LoRaWAN.beginPacket();
        LoRaWAN.write(0xef);
        LoRaWAN.write(0xbe);
        LoRaWAN.write(0xad);
        LoRaWAN.write(0xde);
        LoRaWAN.endPacket();
    }
}


void setup( void )
{
    LoRaWAN.begin(US915);
    LoRaWAN.setSubBand(2);
    LoRaWAN.joinOTAA(appEui, appKey, devEui);

    transmitTimer.start(transmitCallback, 0, 10000);
}

void loop( void )
{
}
