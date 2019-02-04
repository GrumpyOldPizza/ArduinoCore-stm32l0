/* Wire Slave example (see Wire_Master.ino for the master)
 *    
 * The myReceiveCallback tracking the received data sets
 * tx_index into tx_data[], if only one byte had been
 * transferred. The myRequestCallback puts 32 bytes from
 * tx_data[] starting at tx_index into the transmit buffer.
 * Finally the myTransmitCallback adjusts tx_index with
 * the number of transferred bytes.
 *
 * The code roughly simluates a slave device with a FIFO.
 * Sunce the myRequestCallback cannot know how many bytes
 * need to be send, it fills up the buffer to the max. 
 * Only at the myTransmitCallback the number  of bytes
 * transmitted is known.
 *
 *    
 * This example code is in the public domain.
 */

#include "Wire.h"

int tx_index = 7;

uint8_t tx_data[] = "The quick brown fox jumps over the lazy dog\r\n";


void myReceiveCallback(int count)
{
    if (count == 1)
    {
        tx_index = Wire.read();
        
        while (tx_index >= sizeof(tx_data))
        {
            tx_index -= sizeof(tx_data);
        }
    }
}

void myRequestCallback(void)
{
    for (int i = 0, n = tx_index; i < BUFFER_LENGTH; i++)
    {
        Wire.write(tx_data[n]);
        
        n++;
        
        if (n >= sizeof(tx_data)) { n = 0; }
    }
}

void myTransmitCallback(int count)
{
    tx_index += count;

    while (tx_index >= sizeof(tx_data))
    {
        tx_index -= sizeof(tx_data);
    }
}

void setup()
{
    Wire.begin(0x7c);

    Wire.onReceive(myReceiveCallback);
    Wire.onRequest(myRequestCallback);
    Wire.onTransmit(myTransmitCallback);
}

void loop()
{
}
