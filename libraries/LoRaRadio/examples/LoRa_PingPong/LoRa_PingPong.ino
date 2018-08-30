/* Simple Ping-Pong for a LoRa Radio/Modem
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
 *    
 * This example code is in the public domain.
 */
 
#include "LoRaRadio.h"

#define STATE_NONE        0
#define STATE_SCANNING    1
#define STATE_TX_MASTER   2
#define STATE_RX_MASTER   3
#define STATE_TX_SLAVE    4
#define STATE_RX_SLAVE    5

int state = STATE_NONE;

void setup( void )
{
    Serial.begin(9600);
    
    while (!Serial) { }

    LoRaRadio.begin(915000000);

    LoRaRadio.setFrequency(915000000);
    LoRaRadio.setTxPower(14);
    LoRaRadio.setBandwidth(LoRaRadio.BW_125);
    LoRaRadio.setSpreadingFactor(LoRaRadio.SF_7);
    LoRaRadio.setCodingRate(LoRaRadio.CR_4_5);
    LoRaRadio.setLnaBoost(true);

    state = STATE_SCANNING;

    LoRaRadio.receive(5000);
}

void loop( void )
{
    switch (state) {
    case STATE_NONE:
        break;

    case STATE_SCANNING:
        if ((LoRaRadio.parsePacket() == 4) &&
            (LoRaRadio.read() == 'P') &&
            (LoRaRadio.read() == 'I') &&
            (LoRaRadio.read() == 'N') &&
            (LoRaRadio.read() == 'G'))
        {
            // Got a PING from a master, so we are slave ...
            
            Serial.println("= SLAVE");
            Serial.print("< PING (RSSI: ");
            Serial.print(LoRaRadio.packetRssi());
            Serial.print(", SNR: ");
            Serial.print(LoRaRadio.packetSnr());
            Serial.println(")");
            Serial.println("> PONG");
            
            state = STATE_TX_SLAVE;
            
            LoRaRadio.beginPacket();
            LoRaRadio.write('P');
            LoRaRadio.write('O');
            LoRaRadio.write('N');
            LoRaRadio.write('G');
            LoRaRadio.endPacket();
        }
        else
        {
            if (!LoRaRadio.busy())
            {
                // Didn't hear anything, so we are master ...
                
                Serial.println("= MASTER");
                Serial.println("> PING");
                
                state = STATE_TX_MASTER;
                
                LoRaRadio.beginPacket();
                LoRaRadio.write('P');
                LoRaRadio.write('I');
                LoRaRadio.write('N');
                LoRaRadio.write('G');
                LoRaRadio.endPacket();
            }
        }
        break;

    case STATE_TX_MASTER:
        if (!LoRaRadio.busy())
        {
            state = STATE_RX_MASTER;

            LoRaRadio.receive(1000);
        }
        break;

    case STATE_RX_MASTER:
        if ((LoRaRadio.parsePacket() == 4) &&
            (LoRaRadio.read() == 'P') &&
            (LoRaRadio.read() == 'O') &&
            (LoRaRadio.read() == 'N') &&
            (LoRaRadio.read() == 'G'))
        {
            // Got a PING from a slave

            Serial.print("< PONG (RSSI: ");
            Serial.print(LoRaRadio.packetRssi());
            Serial.print(", SNR: ");
            Serial.print(LoRaRadio.packetSnr());
            Serial.println(")");
        }

        if (!LoRaRadio.busy())
        {
            // Receive timed out, so send a PING

            Serial.println("> PING");
            
            state = STATE_TX_MASTER;

            LoRaRadio.beginPacket();
            LoRaRadio.write('P');
            LoRaRadio.write('I');
            LoRaRadio.write('N');
            LoRaRadio.write('G');
            LoRaRadio.endPacket();
        }
        break;

    case STATE_TX_SLAVE:
        if (!LoRaRadio.busy())
        {
            state = STATE_RX_SLAVE;

            LoRaRadio.receive();
        }
        break;

    case STATE_RX_SLAVE:
        if ((LoRaRadio.parsePacket() == 4) &&
            (LoRaRadio.read() == 'P') &&
            (LoRaRadio.read() == 'I') &&
            (LoRaRadio.read() == 'N') &&
            (LoRaRadio.read() == 'G'))
        {
            // Got a PING from a master, so send a PONG as reply

            Serial.print("< PING (RSSI: ");
            Serial.print(LoRaRadio.packetRssi());
            Serial.print(", SNR: ");
            Serial.print(LoRaRadio.packetSnr());
            Serial.println(")");
            Serial.println("> PONG");
            
            state = STATE_TX_SLAVE;

            LoRaRadio.beginPacket();
            LoRaRadio.write('P');
            LoRaRadio.write('O');
            LoRaRadio.write('N');
            LoRaRadio.write('G');
            LoRaRadio.endPacket();
        }
        break;
    }
}
