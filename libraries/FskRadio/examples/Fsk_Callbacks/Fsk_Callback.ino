/* Simple Ping-Pong using callbacks for a FSK Radio/Modem
 *
 * In setup() below please adjust your country specific frequency ranges,
 * as well as the Bandwidth/SpreadingFactor/CodingRate settings.
 *
 * They way this example works is that the device first listens for 5000ms.
 * If it received a "PING" message, it considers itself a SLAVE. If not
 * it considers itself a MASTER. A SLAVE waits for an incoming "PING" message,
 * which it answers with a "PONG" message. A MASTER simply sends periodically
 * every 1000ms) a "PING" message, and collects "PONG" replies while waiting.
 *
 * This variant moves the state machine into callbacks.
 *    
 *    
 * This example code is in the public domain.
 */
 
#include "FskRadio.h"

static void myScanReceiveCallback(void);
static void myMasterTransmitCallback(void);
static void myMasterReceiveCallback(void);
static void mySlaveTransmitCallback(void);
static void mySlaveReceiveCallback(void);


static void myScanReceiveCallback(void)
{
    if (FskRadio.parsePacket() == 4)
    {
        // Got a PING from a master, so we are slave ...
        
        if ((FskRadio.read() == 'P') &&
            (FskRadio.read() == 'I') &&
            (FskRadio.read() == 'N') &&
            (FskRadio.read() == 'G'))
        {
            Serial.println("= SLAVE");
            Serial.print("< PING (RSSI: ");
            Serial.print(FskRadio.packetRssi());
            Serial.println(")");
            Serial.println("> PONG");
            
            FskRadio.onTransmit(mySlaveTransmitCallback);
            FskRadio.onReceive(mySlaveReceiveCallback);
            
            FskRadio.beginPacket();
            FskRadio.write('P');
            FskRadio.write('O');
            FskRadio.write('N');
            FskRadio.write('G');
            FskRadio.endPacket();
        }
    }
    else
    {
        if (!FskRadio.busy())
        {
            // Didn't hear anything, so we are master ...
            
            Serial.println("= MASTER");
            Serial.println("> PING");
            
            FskRadio.onTransmit(myMasterTransmitCallback);
            FskRadio.onReceive(myMasterReceiveCallback);
            
            FskRadio.beginPacket();
            FskRadio.write('P');
            FskRadio.write('I');
            FskRadio.write('N');
            FskRadio.write('G');
            FskRadio.endPacket();
        }
    }
}

static void myMasterTransmitCallback(void)
{
    FskRadio.receive(1000);
}

static void myMasterReceiveCallback(void)
{
    if (FskRadio.parsePacket() == 4)
    {
        // Got a PONG from a slave ,,,
            
        if ((FskRadio.read() == 'P') &&
            (FskRadio.read() == 'O') &&
            (FskRadio.read() == 'N') &&
            (FskRadio.read() == 'G'))
        {
            Serial.print("< PONG (RSSI: ");
            Serial.print(FskRadio.packetRssi());
            Serial.println(")");
        }
    }

    if (!FskRadio.busy())
    {
        // Receive timed out, so send a PING

        Serial.println("> PING");
    
        FskRadio.beginPacket();
        FskRadio.write('P');
        FskRadio.write('I');
        FskRadio.write('N');
        FskRadio.write('G');
        FskRadio.endPacket();
    }
}

static void mySlaveTransmitCallback(void)
{
    FskRadio.receive();
}

static void mySlaveReceiveCallback(void)
{
    if (FskRadio.parsePacket() == 4)
    {
        // Got a PING from a master, so send a PONG as reply
        
        if ((FskRadio.read() == 'P') &&
            (FskRadio.read() == 'I') &&
            (FskRadio.read() == 'N') &&
            (FskRadio.read() == 'G'))
        {
            Serial.print("< PING (RSSI: ");
            Serial.print(FskRadio.packetRssi());
            Serial.println(")");
            Serial.println("> PONG");
            
            FskRadio.beginPacket();
            FskRadio.write('P');
            FskRadio.write('O');
            FskRadio.write('N');
            FskRadio.write('G');
            FskRadio.endPacket();
        }
    }
}

void setup( void )
{
    Serial.begin(9600);
    
    while (!Serial) { }

    FskRadio.begin(915000000);

    FskRadio.setFrequency(915000000);
    FskRadio.setTxPower(14);
    FskRadio.setDeviation(25000);
    FskRadio.setBandwidth(100000);
    FskRadio.setBandwidthAfc(100000);
    FskRadio.setBitRate(50000);
    FskRadio.setModulation(FskRadio.FSK);
    FskRadio.setLnaBoost(true);

    FskRadio.onReceive(myScanReceiveCallback);
    FskRadio.receive(5000);
}

void loop( void )
{
}
