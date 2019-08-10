/* Antenna tuning solution to sweep the radio output in the specified range
 *
 * This sweeps the radio output from start to stop frequency.
 * Analog reading is taken if an external circuit is in place to measure VSWR.
 *
 * WARNING: This is for development use in controlled environment only.
 *    
 *    
 * This example code is in the public domain.
 */

#include "LoRaRadio.h"
#include "STM32L0.h"
#include "TimerMillis.h"

TimerMillis sampler;

// Pin to enable VSWR meter power - otherwise set to -1
#define VSWR_EN 1
// Pin to measure the analog voltage from VSWR meter
#define VSWR_ADC A0

// Configure scanning parameters
uint32_t freq_start  = 860000000; // start MHz
uint32_t freq_stop   = 880000000; // stop MHz
uint32_t freq_step     = 1000000; // step Mhz
int8_t power         = -4;        // TX power in dBm
uint16_t time        = 100;       // TX per frequency in ms



uint32_t freq        = 0;

void sampler_callback(void)
{
    uint16_t sensorValue = analogRead(VSWR_ADC);
    // print out the value you read:
    Serial.print(freq);
    Serial.print(",");
    Serial.println(sensorValue);
}

void setup( void )
{
    pinMode(VSWR_EN,OUTPUT);
    digitalWrite(VSWR_EN,HIGH);
    analogReadResolution(12);

    LoRaRadio.begin(868000000);
    LoRaRadio.setFrequency(868000000);
    LoRaRadio.setTxPower(-4); 

    Serial.begin(115200);
    freq=freq_start;
}

void loop( void )
{
    // set-up the continuous mode
    LoRaRadio.setTxContinuousWave(freq,power,time);
    // enable the ADC sampling to happen in background
    // 10ms delay for stabilization
    // 10ms period
    sampler.start(sampler_callback, 10, 10);
    // wait for time to pass
    delay(time);
    // increment frequency and repeat or go to sleep
    freq+=freq_step;
    sampler.stop();
    if(freq>freq_stop){
        freq=freq_start;
        STM32L0.stop();
    }  
}