/* DOSFS basic file example
 *
 * This example shows how to create and destroy a DOSFS file
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

    if (DOSFS.exists("example.txt"))
    {
        Serial.println("Oops ... example.txt already exists.");
    }

    Serial.println("Creating example.txt...");
    myFile = DOSFS.open("example.txt", "w");
    myFile.close();

    if (!DOSFS.exists("example.txt"))
    {
        Serial.println("example.txt doesn't exist.");
    }

    Serial.println("Removing example.txt...");
    DOSFS.remove("example.txt");

    if (DOSFS.exists("example.txt")) {
        Serial.println("Oops ... example.txt still exists.");
    }

    Serial.println("Done.");
}

void loop( void )
{
}
