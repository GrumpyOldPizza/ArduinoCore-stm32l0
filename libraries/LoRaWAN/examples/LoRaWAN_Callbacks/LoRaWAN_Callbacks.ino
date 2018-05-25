/* A rather complex example using callback and automatic link checks to 
 * detect a lost link.
 *  
 *  A simple timer triggers a transmission every 5 seconds. The LED gets
 *  turned on when the transmission start. When complete the LED gets turned
 *  off again in the doneCallback(). Also in the doneCallback() some network
 *  statistics are printed out to track the ADR settings. A receiveCallback()
 *  is installed to track link check replies. Finally a joinCallback() is
 *  installed to start the timer that tiggers the transmission when the actual
 *  network join is done. Well, more or less ...
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
#include "TimerMillis.h"

const char *appEui = "0101010101010101";
const char *appKey = "2B7E151628AED2A6ABF7158809CF4F3C";
const char *devEui = "0101010101010101";

TimerMillis transmitTimer;

void transmitCallback(void)
{
    if (!LoRaWAN.busy())
    {
        if (!LoRaWAN.linkGateways())
        {
            transmitTimer.stop();

            Serial.println("REJOIN( )");
            
            LoRaWAN.rejoinOTAA();
        }
        
        if (LoRaWAN.joined())
        {
            digitalWrite(LED_BUILTIN, 1);

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
}

void joinCallback(void)
{
    if (LoRaWAN.joined())
    {
        Serial.println("JOINED");

        transmitTimer.start(transmitCallback, 0, 5000);
    }
    else
    {
        Serial.println("REJOIN( )");

        LoRaWAN.rejoinOTAA();
    }
}

void checkCallback(void)
{
    Serial.print("CHECK( ");
    Serial.print("RSSI: ");
    Serial.print(LoRaWAN.lastRSSI());
    Serial.print(", SNR: ");
    Serial.print(LoRaWAN.lastSNR());
    Serial.print(", Margin: ");
    Serial.print(LoRaWAN.linkMargin());
    Serial.print(", Gateways: ");
    Serial.print(LoRaWAN.linkGateways());
    Serial.println(" )");
}

void receiveCallback(void)
{
    Serial.print("RECEIVE( ");
    Serial.print("RSSI: ");
    Serial.print(LoRaWAN.lastRSSI());
    Serial.print(", SNR: ");
    Serial.print(LoRaWAN.lastSNR());

    if (LoRaWAN.parsePacket())
    {
        uint32_t size;
        uint8_t data[256];

        size = LoRaWAN.read(&data[0], sizeof(data));

        if (size)
        {
            data[size] = '\0';

            Serial.print(", PORT: ");
            Serial.print(LoRaWAN.remotePort());
            Serial.print(", DATA: \"");
            Serial.print((const char*)&data[0]);
            Serial.println("\"");
        }
    }

    Serial.println(" )");
}

void doneCallback(void)
{
    digitalWrite(LED_BUILTIN, 0);

    if (!LoRaWAN.linkGateways())
    {
        Serial.println("DISCONNECTED");
    }
}

void setup( void )
{
    Serial.begin(9600);
    
    while (!Serial) { }

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 0);

    LoRaWAN.begin(US915);
    // LoRaWAN.setSubBand(2);
    // LoRaWAN.setDutyCycle(false);
    // LoRaWAN.setAntennaGain(2.0);
    LoRaWAN.setLinkCheckLimit(16);
    LoRaWAN.setLinkCheckDelay(8);
    LoRaWAN.setLinkCheckThreshold(4);

    LoRaWAN.onJoin(joinCallback);
    LoRaWAN.onLinkCheck(checkCallback);
    LoRaWAN.onTransmit(doneCallback);
    LoRaWAN.onReceive(receiveCallback);

    LoRaWAN.joinOTAA(appEui, appKey, devEui);

    Serial.println("JOIN( )");
}

void loop( void )
{
}
