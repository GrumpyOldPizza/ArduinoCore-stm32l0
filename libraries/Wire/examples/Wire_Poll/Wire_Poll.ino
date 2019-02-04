/* Extended Wire Sensor example
 *    
 * The code is using the proprietary Wire transfer interface
 * to communicate with a LPS22HB sensor and print temperature,
 * pressure samples every 10 seconds.
 *
 * Wire.transfer() to avoid intermediate buffer handling
 * within the Wire library.
 *
 * Since we official don't know the conversion time for
 * LPS22HB, the code simply polls the status register.
 *
 * This here is not meant to provide a useful library
 * or anything generic, it's just here to show how
 * to use the Wire library to communicate with LPS22HB.
 *
 *    
 * This example code is in the public domain.
 */

#include "Wire.h"
#include "TimerMillis.h"

#define LPS22HB_I2C_ADDRESS 0x5C

uint8_t lps22hb_data[5];

void setup()
{
    Serial.begin(9600);
    
    while (!Serial) { }

    Wire.begin();

    lps22hb_write_config();
}

void loop()
{
    float temperature, pressure;

    Wire.transfer(LPS22HB_I2C_ADDRESS, (const uint8_t[]){ 0x11, 0x11 }, 2, NULL, 0);

    do
    {
        Wire.transfer(LPS22HB_I2C_ADDRESS, (const uint8_t[]){ 0x27 }, 1, &lps22hb_data[0], 1);
    }
    while ((lps22hb_data[0] & 0x03) != 0x03);

    Wire.transfer(LPS22HB_I2C_ADDRESS, (const uint8_t[]){ 0x28 }, 1, &lps22hb_data[0], 5);

    temperature = (float)((int16_t)(((uint16_t)lps22hb_data[3] << 0) | ((uint16_t)lps22hb_data[4] << 8))) / 100.0;
    pressure = (float)((uint32_t)(((uint32_t)lps22hb_data[0] << 0) | ((uint32_t)lps22hb_data[1] << 8) | ((uint32_t)lps22hb_data[2] << 16))) / 4096.0;

    Serial.print("Temperature = ");
    Serial.print(temperature);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.println(" hPa");
    
    Serial.println();

    delay(10);
}

void lps22hb_write_config()
{
    Wire.transfer(LPS22HB_I2C_ADDRESS, (const uint8_t[]){ 0x10, 0x02 }, 2, NULL, 0);
    Wire.transfer(LPS22HB_I2C_ADDRESS, (const uint8_t[]){ 0x11, 0x10 }, 2, NULL, 0);
    Wire.transfer(LPS22HB_I2C_ADDRESS, (const uint8_t[]){ 0x12, 0x04 }, 2, NULL, 0);
}
