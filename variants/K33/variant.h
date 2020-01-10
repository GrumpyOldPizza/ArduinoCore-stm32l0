/*
 * Copyright (c) 2017-2018 Thomas Roell.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#ifndef _VARIANT_K33_
#define _VARIANT_K33_

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

#define STM32L0_CONFIG_LSECLK             32768
#define STM32L0_CONFIG_HSECLK             0
#define STM32L0_CONFIG_SYSOPT             0

/** Master clock frequency */
#define VARIANT_MCK                       F_CPU

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT           (28u)
#define NUM_DIGITAL_PINS     (27u)
#define NUM_ANALOG_INPUTS    (1u)
#define NUM_ANALOG_OUTPUTS   (0u)

/*
 * LEDs
 */

#define PIN_LED_GR           (4ul)
#define PIN_LED_BL           (5ul)
#define PIN_LED_RD           (8ul)

#define LED_BUILTIN          PIN_LED_BL

/*
 * Serial interfaces
 */

#define SERIAL_INTERFACES_COUNT 1

#define PIN_SERIAL_RX        (0ul)
#define PIN_SERIAL_TX        (1ul)

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (14u)
#define PIN_WIRE_SCL         (15u)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 2

#define PIN_SPI_MOSI         (11u)
#define PIN_SPI_MISO         (12u)
#define PIN_SPI_SCK          (13u)

static const uint8_t SS   = 10;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

#define PIN_RADIO_NRST        (16u)
#define PIN_RADIO_NSS         (17u)
#define PIN_RADIO_DIO0        (18u)
#define PIN_RADIO_DIO1        (19u)
#define PIN_RADIO_DIO2        (20u)
#define PIN_RADIO_DIO3        (21u)

static const uint8_t RADIO_RESET = PIN_RADIO_NRST;
static const uint8_t RADIO_NSS   = PIN_RADIO_NSS;
static const uint8_t RADIO_INT   = PIN_RADIO_DIO0;
static const uint8_t RADIO_DIO1  = PIN_RADIO_DIO1;
static const uint8_t RADIO_DIO2  = PIN_RADIO_DIO2;
static const uint8_t RADIO_DIO3  = PIN_RADIO_DIO3;

/*
 * ADC pin
 */
#define PIN_A0                  (23ul)

static const uint8_t A0  = PIN_A0;

#define ADC_RESOLUTION          12

#define PWM_INSTANCE_COUNT      1

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
extern Uart Serial;
#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_HARDWARE1       Serial
#define SERIAL_PORT_HARDWARE_OPEN1  Serial

#endif /*_VARIANT_K33_ */

