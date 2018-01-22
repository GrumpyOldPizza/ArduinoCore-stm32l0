/* Send a packet every 5 seconds and piggyback a link check along.
 *  
 *  The code assumes a US915 setup. For other regions please edit "setup()"
 *  with the appropriate settings (see LoRaWAN_OTAA_<band> examples).
 *  
 *  A simple timer triggers a transmission every 5 seconds. The LED gets
 *  turned on when the transmission start. When complete the LED gets turned
 *  off again in the doneCallback(). Also in the doneCallback() some network
 *  statistics are printed out to track the ADR settings. A receiveCallback()
 *  is installed to track link check replies. Finally a joinCallback() is
 *  installed to start the timer that tiggers the transmission when the actual
 *  network join is done.
 *    
 * This example code is in the public domain.
 */

#include "LoRaWAN.h"
#include "TimerMillis.h"

const char *appEui = "0101010101010101";
const char *appKey = "2B7E151628AED2A6ABF7158809CF4F3C";
const char *devEui = "0101010101010101";

TimerMillis transmitTimer;

unsigned int linkCheck = 0;

void transmitCallback(void)
{
    digitalWrite(LED_BUILTIN, 1);

    linkCheck++;
        
    if (linkCheck == 8) 
    {
        linkCheck = 0;
        
        LoRaWAN.linkCheck();
    }
    
    LoRaWAN.beginPacket();
    LoRaWAN.write(0xef);
    LoRaWAN.write(0xbe);
    LoRaWAN.write(0xad);
    LoRaWAN.write(0xde);
    LoRaWAN.endPacket();
}

void doneCallback(void)
{
    digitalWrite(LED_BUILTIN, 0);

    Serial.print("DR: ");
    Serial.print(LoRaWAN.getDataRate());
    Serial.print(", TxPower: ");
    Serial.print(LoRaWAN.getTxPower());
    Serial.print("dbm, UpLinkCounter: ");
    Serial.print(LoRaWAN.getUpLinkCounter());
    Serial.print(", DownLinkCounter: ");
    Serial.print(LoRaWAN.getDownLinkCounter());
    Serial.println();
}

void receiveCallback(void)
{
    Serial.print("RSSI: ");
    Serial.print(LoRaWAN.packetRSSI());
    Serial.print(", SNR: ");
    Serial.print(LoRaWAN.packetSNR());

    if (LoRaWAN.checked())
    {
        Serial.print(", MARGIN: ");
        Serial.print(LoRaWAN.linkMargin());
        Serial.print(", GATEWAYS: ");
        Serial.print(LoRaWAN.linkGateways());
    }

    Serial.println();
}

void joinCallback(void)
{
    if (LoRaWAN.joined())
    {
        Serial.println("JOINED");

        transmitTimer.start(transmitCallback, 0, 5000);
    }
}

void setup( void )
{
    Serial.begin(9600);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 0);

    LoRaWAN.begin(US915);
    LoRaWAN.setSubBand(2);
    LoRaWAN.joinOTAA(appEui, appKey, devEui);

    LoRaWAN.onJoin(joinCallback);
    LoRaWAN.onReceive(receiveCallback);
    LoRaWAN.onTransmit(doneCallback);
}

void loop( void )
{
}
