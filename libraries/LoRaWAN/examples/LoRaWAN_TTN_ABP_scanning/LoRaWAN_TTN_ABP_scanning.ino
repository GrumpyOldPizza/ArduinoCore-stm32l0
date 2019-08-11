
/* Antenna tuning solution to sweep the LoRaWAN radio output in the specified range
 *
 * This sweeps the radio output from start to stop frequency.
 * Analog reading is taken if an external circuit is in place to measure VSWR
 * Results are sent via LoRaWAN.
 *
 * WARNING: This is for development use in controlled environment only.
 *    
 *    
 * This example code is in the public domain.
 */

#include "LoRaWAN.h"
#include <math.h>

//const char *devAddr = "0100000A";
//const char *nwkSKey = "2B7E151628AED2A6ABF7158809CF4F3C";
//const char *appSKey = "2B7E151628AED2A6ABF7158809CF4F3C";

// Pin to enable VSWR meter power - otherwise set to -1
#define VSWR_EN 3
// Pin to measure the analog voltage from VSWR meter
#define VSWR_ADC PA3

// Configure scanning parameters
uint32_t freq_start  = 860000000;   // start MHz
uint32_t freq_stop   = 880000000;   // stop MHz
uint32_t samples     = 200;         // number of samples to take
int8_t power         = -4;          // TX power in dBm
uint16_t time        = 100;         // TX per frequency in ms
float vcc            = 2.5;         //voltage to calculate the measurement

// 51 to 222 bytes long max packet
// take at most 256 measurements
static uint8_t message[256];

void setup( void )
{
    LoRaWAN.begin(EU868);
    LoRaWAN.setDutyCycle(false);
    LoRaWAN.setTxPower(20);    
    LoRaWAN.setDataRate(5);
    LoRaWAN.joinABP(devAddr, nwkSKey, appSKey);

    Serial.begin(115200);
    Serial.println("JOIN( )");

    pinMode(VSWR_EN,OUTPUT);
    digitalWrite(VSWR_EN,HIGH);
    analogReadResolution(12);

    delay(3000);
}

boolean scan_vswr(uint32_t start, uint32_t stop, int8_t power, uint16_t samples, uint32_t time, uint8_t *output)
{
    uint32_t freq = start;
    uint32_t step = (stop-start)/samples;

    uint32_t measurement_sum = 0;
    uint16_t measurement_count = 0;

    if(step==0 | start>=stop){
        return false;
    }

    for(uint16_t i;i<samples;i++){
        Serial.print("Scan( "); Serial.print(freq); Serial.print(" Hz ");
        // limit time to maximal value
        if(time>4000){
            time=4000;
        }
        // set-up the continuous mode
        LoRaWAN.setTxContinuousWave(freq,power,time);
        // acquire measurements for the specified duration
        unsigned long start_time = millis();
        //wait for stabilization
        delay(10);
        while((millis()-start_time)<time){
            measurement_sum=+analogRead(VSWR_ADC);
            measurement_count++;
            delay(10);
        }

        double value = measurement_sum/measurement_count*vcc/4096; // calculate average and convert to volts
        double dbm=-30; // covnert to dBm
        // secure against division by zero
        if(value!=0){
            dbm = log10(value)-27;
        }
        Serial.print(dbm); Serial.println(" dBm) ");
        // create an 8 bit datapoint use dBm=result/10-30 on the server side
        uint8_t result = (dbm+30)*10;
        *output=result;
        output++;
        //increment frequency
        freq+=step;
        if(freq>stop){
            break;
        }
    }
    return true;
}

void loop( void )
{

    // guard against overflow
    if(samples>sizeof(message)){
       samples=sizeof(message);
    }
    scan_vswr(freq_start, freq_stop, power, samples, 100, &message[0]);

    delay(5000); // required

    if (LoRaWAN.joined() && !LoRaWAN.busy())
    {
        Serial.print("TRANSMIT( ");
        Serial.print("TimeOnAir: ");
        Serial.print(LoRaWAN.getTimeOnAir());
        Serial.print(", NextTxTime: ");
        Serial.print(LoRaWAN.getNextTxTime());
        Serial.print(", MaxPayloadSize: ");
        Serial.print(LoRaWAN.getMaxPayloadSize());
        Serial.print(", DR: ");
        Serial.print(LoRaWAN.getDataRate());
        Serial.print(", TxPower: ");
        Serial.print(LoRaWAN.getTxPower(), 1);
        Serial.print("dbm, UpLinkCounter: ");
        Serial.print(LoRaWAN.getUpLinkCounter());
        Serial.print(", DownLinkCounter: ");
        Serial.print(LoRaWAN.getDownLinkCounter());
        Serial.println(" )");

        // truncate if payload is longer
        uint16_t length=samples;
        if(length>LoRaWAN.getMaxPayloadSize()){
            length=LoRaWAN.getMaxPayloadSize();
        }
        LoRaWAN.sendPacket(30, &message[0], length, false);
    }

    delay(10000);
}
