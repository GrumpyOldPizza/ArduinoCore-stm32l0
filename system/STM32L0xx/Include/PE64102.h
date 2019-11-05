#ifndef PE64102_H
#define PE64102_H
#include <stdint.h>

uint8_t DTC_set( uint8_t serial_enable, uint8_t cap_value );
void DTC_deactivate( uint8_t serial_enable );

#endif
