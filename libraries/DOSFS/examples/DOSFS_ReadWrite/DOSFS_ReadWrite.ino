/* DOSFS read/write
 *
 * This example shows how to read and write data to and from a DOSFS file
 *
 * Code adopted from the SD Library
 *    
 * This example code is in the public domain.
 */

#include <DOSFS.h>

File myFile;

void setup ( void )
{
    Serial.begin(9600);

    while (!Serial) { }


    DOSFS.begin();

    myFile = DOSFS.open("test.txt", "w");

    if (myFile)
    {
        Serial.println("Writing to test.txt...");
        myFile.println("The Quick Brown Fox Jumps Over The Lazy Dog.");
        myFile.close();
    }
    else
    {
        Serial.println("Error opening test.txt");
    }

    myFile = DOSFS.open("test.txt", "r");
    if (myFile)
    {
        Serial.println("Reading from test.txt... ");

        while (myFile.available())
        {
            Serial.write(myFile.read());
        }

        myFile.close();
    }
    else
    {
        Serial.println("Error opening test.txt");
    }

    Serial.println("Done");
}

void loop( void )
{
}


