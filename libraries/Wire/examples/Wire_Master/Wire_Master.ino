/* Wire Master example (see Wire_Slave.ino for the master)
 *    
 * The code in Wire_Slave.ino implements a simple slave
 * device that transmits a recurring data stream. Hence
 * first initialize the index into the stream, and then
 * read it in 8 byte chunks, and then print it.
 *
 *    
 * This example code is in the public domain.
 */

#include "Wire.h"

void setup()
{
    Serial.begin(9600);
    
    while (!Serial) { }

    Wire.begin();

    Wire.beginTransmission(0x7c);
    Wire.write(0x00);
    Wire.endTransmission();
}

void loop()
{
    int size;
    uint8_t data[8];

    size = Wire.requestFrom(0x7c, 8);

    if (size)
    {
        Wire.read(data, size);
        Serial.write(data, size);
    }

    delay(500);
}
