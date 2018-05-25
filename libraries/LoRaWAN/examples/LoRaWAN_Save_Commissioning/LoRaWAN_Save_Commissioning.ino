/* Save commissioning data to EEPROM for later reuse.
 *
 *  Please edit the keys below as they are just debugging samples.
 *    
 *    
 * This example code is in the public domain.
 */

#include "LoRaWAN.h"

const char *appEui  = "0101010101010101";
const char *appKey  = "2B7E151628AED2A6ABF7158809CF4F3C";
const char *devEui  = "0101010101010101";
const char *devAddr = "0100000A";
const char *nwkSKey = "2B7E151628AED2A6ABF7158809CF4F3C";
const char *appSKey = "2B7E151628AED2A6ABF7158809CF4F3C";

void setup( void )
{
    LoRaWAN.setAppEui(appEui);
    LoRaWAN.setAppKey(appKey);
    LoRaWAN.setDevEui(devEui);
    LoRaWAN.setDevAddr(devAddr);
    LoRaWAN.setNwkSKey(nwkSKey);
    LoRaWAN.setAppSKey(appSKey);
}

void loop( void )
{
}
