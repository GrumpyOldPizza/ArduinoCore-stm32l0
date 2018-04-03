/* DOSFS datalogger
 *
 * This example shows how to log data to a DOSFS file
 *
 * Please select in the "Tools->DOSFS" menu either
 * SDCARD or SFLASH. Make sure that "Tools->USB Type"
 * does not contain "Mass Storage". 
 *
 * Kind of simple and a tad boring. The temperature sensor
 * is read and then the data appended to a logfile. There
 * are however a few interesting details internally. The
 * log file is closed after each dataset. That has 2 effects.
 * One is that the if the crashes or gets reset there is no
 * file system corruption. The other important detail is 
 * that if no wok is pending, the SDCARD/SFLASH can go
 * to sleep, hence consuming less power.
 *
 * The loop is run every 5 seconds (STM32L0.stop() to
 * put STM32L0 into STOP mode). Every 60 seconds the current
 * log file is printed out.
 *
 * This example code is in the public domain.
 */

#include "DOSFS.h"
#include "STM32L0.h"

int mySamples;
float myTemperature;
File myFile;

void setup ( void )
{
    Serial.begin(9600);

    while (!Serial) { }

    DOSFS.begin();

    DOSFS.remove("data.txt");

    mySamples = 0;

}

void loop( void )
{
    myTemperature = STM32L0.getTemperature();

    myFile = DOSFS.open("data.txt", "a+");

    if (myFile)
    {
        myFile.println(myTemperature);
        myFile.close();
    }

    mySamples++;

    if ((mySamples % 12) == 0)
    {
        myFile = DOSFS.open("data.txt", "r");

        if (myFile)
        {
            Serial.println();
            Serial.println();
            Serial.println(mySamples);
            while (myFile.available())
            {
                Serial.write(myFile.read());
            }

            myFile.close();
        }
    }

    STM32L0.stop(5000);
}


