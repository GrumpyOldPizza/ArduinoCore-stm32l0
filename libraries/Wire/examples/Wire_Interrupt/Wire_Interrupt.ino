/* Complicated Wire Sensor example
 *    
 * The code is using the proprietary Wire transaction interface
 * to communicate with a LPS22HB sensor and print temperature,
 * pressure samples every 10 seconds.
 *
 * It is assumed that INT_DRDY is connected to D2.
 *
 * Wire.transfer() to avoid intermediate buffer handling
 * withint the Wire library. Also use asynchronous
 * TwoWireTransaction objects to convert/read the 
 * samples in the background.
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

volatile bool lps22hb_done = false;

uint8_t lps22hb_data[5];

TwoWireTransaction lps22hb_transaction;

TimerMillis lps22hb_sample;

void sampleCallback(void)
{
    static uint8_t tx_data[] = { 0x11, 0x11 };

    lps22hb_transaction.submit(Wire, LPS22HB_I2C_ADDRESS, &tx_data[0], 2, NULL, 0, NULL);
}

void readyCallback(void)
{
    static uint8_t tx_data[] = { 0x28 };

    lps22hb_transaction.submit(Wire, LPS22HB_I2C_ADDRESS, &tx_data[0], 1, &lps22hb_data[0], 5, doneCallback);
}

void doneCallback(void)
{
    lps22hb_done = true;
}

void setup()
{
    Serial.begin(9600);
    
    while (!Serial) { }

    Wire.begin();

    lps22hb_write_config();

    lps22hb_sample.start(sampleCallback, 1000, 1000);

    pinMode(2, INPUT_PULLUP);
    attachInterrupt(2, readyCallback, RISING);
}

void loop()
{
    float temperature, pressure;

    if (lps22hb_done)
    {
        lps22hb_done = false;

        temperature = (float)((int16_t)(((uint16_t)lps22hb_data[3] << 0) | ((uint16_t)lps22hb_data[4] << 8))) / 100.0;
        pressure = (float)((uint32_t)(((uint32_t)lps22hb_data[0] << 0) | ((uint32_t)lps22hb_data[1] << 8) | ((uint32_t)lps22hb_data[2] << 16))) / 4096.0;

        Serial.print("Temperature = ");
        Serial.print(temperature);
        Serial.println(" *C");

        Serial.print("Pressure = ");
        Serial.print(pressure);
        Serial.println(" hPa");

        Serial.println();
    }
}

void lps22hb_write_config()
{
    Wire.transfer(LPS22HB_I2C_ADDRESS, (const uint8_t[]){ 0x10, 0x02 }, 2, NULL, 0);
    Wire.transfer(LPS22HB_I2C_ADDRESS, (const uint8_t[]){ 0x11, 0x10 }, 2, NULL, 0);
    Wire.transfer(LPS22HB_I2C_ADDRESS, (const uint8_t[]){ 0x12, 0x04 }, 2, NULL, 0);
}
