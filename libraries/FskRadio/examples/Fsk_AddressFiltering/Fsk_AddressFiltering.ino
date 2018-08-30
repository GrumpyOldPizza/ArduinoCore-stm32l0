/* Simple Ping-Pong using address filtering for a FSK Radio/Modem
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
 * This variant makes use of address filter to comminicate between nodes.
 *
 *    
 * This example code is in the public domain.
 */
 
#include "FskRadio.h"

#define STATE_NONE        0
#define STATE_SCANNING    1
#define STATE_TX_MASTER   2
#define STATE_RX_MASTER   3
#define STATE_TX_SLAVE    4
#define STATE_RX_SLAVE    5

int state = STATE_NONE;
int address = 0x00;

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
    FskRadio.setAddressFiltering(FskRadio.FILTERING_NODE_AND_BROADCAST);
    FskRadio.setNodeAddress(0x00);
    FskRadio.setBroadcastAddress(0xff);
    FskRadio.setLnaBoost(true);

    state = STATE_SCANNING;

    FskRadio.receive(5000);
}

void loop( void )
{
    switch (state) {
    case STATE_NONE:
        break;

    case STATE_SCANNING:
        if ((FskRadio.parsePacket() == 4) &&
            (FskRadio.read() == 'P') &&
            (FskRadio.read() == 'I') &&
            (FskRadio.read() == 'N') &&
            (FskRadio.read() == 'G'))
        {
            // Got a PING from a master, so we are slave ...
            
            Serial.println("= SLAVE");
            Serial.print("< PING (RSSI: ");
            Serial.print(FskRadio.packetRssi());
            Serial.println(")");
            Serial.println("> PONG");
            
            state = STATE_TX_SLAVE;
            
            FskRadio.beginPacket(0xff);
            FskRadio.write('P');
            FskRadio.write('O');
            FskRadio.write('N');
            FskRadio.write('G');
            FskRadio.endPacket();
        }
        else
        {
            if (!FskRadio.busy())
            {
                // Didn't hear anything, so we are master ...
                
                Serial.println("= MASTER");
                Serial.print("> PING ");
                Serial.println(address, HEX);
                
                state = STATE_TX_MASTER;
                
                FskRadio.beginPacket(address);
                FskRadio.write('P');
                FskRadio.write('I');
                FskRadio.write('N');
                FskRadio.write('G');
                FskRadio.endPacket();

                address ^= 1;
            }
        }
        break;

    case STATE_TX_MASTER:
        if (!FskRadio.busy())
        {
            state = STATE_RX_MASTER;

            FskRadio.receive(1000);
        }
        break;

    case STATE_RX_MASTER:
        if ((FskRadio.parsePacket() == 4) &&
            (FskRadio.read() == 'P') &&
            (FskRadio.read() == 'O') &&
            (FskRadio.read() == 'N') &&
            (FskRadio.read() == 'G'))
        {
            // Got a PING from a slave

            Serial.print("< PONG (RSSI: ");
            Serial.print(FskRadio.packetRssi());
            Serial.println(")");
        }

        if (!FskRadio.busy())
        {
            // Receive timed out, so send a PING

            Serial.print("> PING ");
            Serial.println(address, HEX);
            
            state = STATE_TX_MASTER;

            FskRadio.beginPacket(address);
            FskRadio.write('P');
            FskRadio.write('I');
            FskRadio.write('N');
            FskRadio.write('G');
            FskRadio.endPacket();

            address ^= 1;
        }
        break;

    case STATE_TX_SLAVE:
        if (!FskRadio.busy())
        {
            state = STATE_RX_SLAVE;

            FskRadio.receive();
        }
        break;

    case STATE_RX_SLAVE:
        if ((FskRadio.parsePacket() == 4) &&
            (FskRadio.read() == 'P') &&
            (FskRadio.read() == 'I') &&
            (FskRadio.read() == 'N') &&
            (FskRadio.read() == 'G'))
        {
            // Got a PING from a master, so send a PONG as reply

            Serial.print("< PING (RSSI: ");
            Serial.print(FskRadio.packetRssi());
            Serial.println(")");
            Serial.println("> PONG");
            
            state = STATE_TX_SLAVE;

            FskRadio.beginPacket(0xff);
            FskRadio.write('P');
            FskRadio.write('O');
            FskRadio.write('N');
            FskRadio.write('G');
            FskRadio.endPacket();
        }
        break;
    }
}
