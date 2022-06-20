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

#ifndef _VARIANT_K48_S76G_SWD_
#define _VARIANT_K48_S76G_SWD_

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

#define STM32L0_CONFIG_LSECLK             32768
#define STM32L0_CONFIG_HSECLK             0
#define STM32L0_CONFIG_SYSOPT             0

#define STM32L0_CONFIG_PIN_VBUS           STM32L0_GPIO_PIN_PA8

// #define STM32L0_CONFIG_PIN_VBAT           STM32L0_GPIO_PIN_PA2
// #define STM32L0_CONFIG_CHANNEL_VBAT       STM32L0_ADC_CHANNEL_2
// #define STM32L0_CONFIG_VBAT_PERIOD        40
// #define STM32L0_CONFIG_VBAT_SCALE         ((float)2.08)

// #define STM32L0_CONFIG_PIN_GNSS_ENABLE    STM32L0_GPIO_PIN_NONE
// #define STM32L0_CONFIG_PIN_GNSS_RX        STM32L0_GPIO_PIN_PC11_USART4_RX
// #define STM32L0_CONFIG_PIN_GNSS_TX        STM32L0_GPIO_PIN_PC10_USART4_TX
// #define GPS_BAUD_RATE                     115200

#define USBCON

/** Master clock frequency */
#define VARIANT_MCK                       F_CPU

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
#include "USBAPI.h"
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
#define PINS_COUNT          (22u)
#define NUM_DIGITAL_PINS    (19u)
#define NUM_ANALOG_INPUTS   (3u)
#define NUM_ANALOG_OUTPUTS  (0u)
#define PWM_INSTANCE_COUNT   0
#define SPI_INTERFACES_COUNT 0

/*
 * Button pin
 */
#define PIN_BUTTON          (0ul)
#define PIN_SENSOR_INT      (0ul)
static const uint8_t BUTTON = PIN_BUTTON;
static const uint8_t SENSOR = PIN_SENSOR_INT;

/*
 * LEDs
 */
#define PIN_LED_RED         (1ul)
#define PIN_LED_GREEN       (2ul)
#define PIN_LOAD_TEST       (3ul)
#define LED_BUILTIN         PIN_LED_GREEN
#define LED_ON              HIGH
#define LED_OFF             LOW

/*
 * ADC open-drain enablers Control pins
 */
#define PIN_VBAT_TEST        (4ul)

/*
 * Genarator control pins
 */
#define PIN_GEN_POWER        (6ul)
#define PIN_GEN_ENABLE       (7ul)

/*
 * GPS receiver pins
 */
#define GPS_RST              (5ul)
#define GPS_1PPS             (8ul)
#define GPS_LEVEL_SHIFTER_EN (9ul)

/*
 * Serial interface
 */
#define SERIAL_INTERFACES_COUNT 3

#define PIN_SERIAL1_RX      (10ul)
#define PIN_SERIAL1_TX      (11ul)

#define PIN_SERIAL2_RX      (12ul)
#define PIN_SERIAL2_TX      (13ul)

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA        (14u)
#define PIN_WIRE_SCL        (15u)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * Analog pins
 */
#define PIN_LOAD_ADC        (16ul)
#define PIN_VBAT_ADC        (17ul)
#define PIN_TEMP_ADC        (18ul)

static const uint8_t A0  = PIN_LOAD_ADC;
static const uint8_t A1  = PIN_VBAT_ADC;
static const uint8_t A2  = PIN_TEMP_ADC;

#define ADC_RESOLUTION          12

#define POUT_MAX            (23ul)

/*
 * USB Interface
 */
#define PIN_USB_VBUS         (19ul)
#define PIN_USB_DM           (20ul)
#define PIN_USB_DP           (21ul) 


#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
extern CDC  SerialUSB;
extern Uart Serial1;
extern Uart Serial2;
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
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_MONITOR         Serial2
#define SERIAL_PORT_GNSS            Serial1
#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE2       Serial2

// Alias Serial to SerialUSB
#define Serial                      Serial2

#endif /*_VARIANT_K48_S76G_SWD */

