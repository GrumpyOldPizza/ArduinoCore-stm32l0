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

#ifndef _VARIANT_RHF76_052_
#define _VARIANT_RHF76_052_

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

#define STM32L0_CONFIG_LSECLK             32768
#define STM32L0_CONFIG_HSECLK             12000000
#define STM32L0_CONFIG_SYSOPT             0

#define STM32L0_CONFIG_PIN_VBAT           STM32L0_GPIO_PIN_PA3
#define STM32L0_CONFIG_CHANNEL_VBAT       STM32L0_ADC_CHANNEL_3
#define STM32L0_CONFIG_VBAT_PERIOD        40
#define STM32L0_CONFIG_VBAT_SCALE         ((float)1.27)

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
#define PINS_COUNT           (30u)
#define NUM_DIGITAL_PINS     (28u)
#define NUM_ANALOG_INPUTS    (2u)
#define NUM_ANALOG_OUTPUTS   (0u)

// LEDs

#define PIN_LED1             (3ul)
#define PIN_LED2             (4ul)
#define LED_BUILTIN          PIN_LED1
#define LED_TRX              PIN_LED2
/*
 * Analog pins
 */
#define PIN_A0               (26ul)
#define PIN_A1               (27ul)

static const uint8_t A0   =  PIN_A0;
static const uint8_t A1   =  PIN_A1;

#define ADC_RESOLUTION       12


/*
 * Serial interface
 */
#if (RHF76-L052 >= 1)
#define SERIAL_INTERFACES_COUNT 2
#else  /* (RHF76-L052 >= 1) */
#define SERIAL_INTERFACES_COUNT 1
#endif /* (RHF76-L052 >= 1) */

#define PIN_SERIAL_RX        (6ul)
#define PIN_SERIAL_TX        (5ul)
#define PIN_SERIAL_CTS       (22ul)
#define PIN_SERIAL_RTS       (23ul)

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
#define PIN_FEM_CTX           (28u)
#define PIN_FEM_CPS           (29u)

static const uint8_t RADIO_RESET = PIN_RADIO_NRST;
static const uint8_t RADIO_NSS   = PIN_RADIO_NSS;
static const uint8_t RADIO_INT   = PIN_RADIO_DIO0;
static const uint8_t RADIO_DIO1  = PIN_RADIO_DIO1;
static const uint8_t RADIO_DIO2  = PIN_RADIO_DIO2;
static const uint8_t RADIO_DIO3  = PIN_RADIO_DIO3;
static const uint8_t RADIO_FEM_CTX  = PIN_FEM_CTX;
static const uint8_t RADIO_FEM_CPS  = PIN_FEM_CPS;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (14u)
#define PIN_WIRE_SCL         (15u)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;


/*
 * USB Interface
 */
#if (RHF76-L052 >= 1)
#define STM32L0_CONFIG_PIN_VBUS           STM32L0_GPIO_PIN_PA8
#define PIN_USB_DM           (22ul)
#define PIN_USB_DP           (23ul)
#define PIN_USB_VBUS         (7ul)
#endif (RHF76-L052 >= 1)

#define PWM_INSTANCE_COUNT    2

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
#if (RHF76-L052 >= 1)
#include "USBAPI.h"
#define USBCON
extern CDC  SerialUSB;
extern Uart Serial1;
#else  /* (RHF76-L052 >= 1) */
extern Uart Serial;
#endif /* (RHF76-L052 >= 1) */
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
#if (RHF76-L052 >= 1)
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_MONITOR         Serial1
#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1
// Alias Serial to SerialUSB
// You can replace it with this after you unjail RHF76 and it has USB (if you have STM32L052 inside)
#define Serial                      SerialUSB
#else  /* (RHF76-L052 >= 1) */
#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_HARDWARE1       Serial
#define SERIAL_PORT_HARDWARE_OPEN   Serial
#endif /* (RHF76-L052 >= 1) */

#endif /*_VARIANT_RHF76_052_ */

