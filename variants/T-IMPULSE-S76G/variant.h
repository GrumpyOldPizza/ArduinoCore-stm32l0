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

#ifndef _VARIANT_T_IMPULSE_S76G_
#define _VARIANT_T_IMPULSE_S76G_

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

#define STM32L0_CONFIG_LSECLK             32768
#define STM32L0_CONFIG_HSECLK             0
#define STM32L0_CONFIG_SYSOPT             0

#define STM32L0_CONFIG_PIN_VBUS           STM32L0_GPIO_PIN_PA9

#define STM32L0_CONFIG_PIN_VBAT           STM32L0_GPIO_PIN_PC4
#define STM32L0_CONFIG_CHANNEL_VBAT       STM32L0_ADC_CHANNEL_14
#define STM32L0_CONFIG_VBAT_PERIOD        40
#define STM32L0_CONFIG_VBAT_SCALE         ((float)2.08)

#define STM32L0_CONFIG_PIN_GNSS_ENABLE    STM32L0_GPIO_PIN_PA3
#define STM32L0_CONFIG_PIN_LSHIFTER_EN    STM32L0_GPIO_PIN_PC6
#define STM32L0_CONFIG_PIN_GNSS_PPS       STM32L0_GPIO_PIN_PB5
#define STM32L0_CONFIG_PIN_GNSS_RX        STM32L0_GPIO_PIN_PC11_USART4_RX
#define STM32L0_CONFIG_PIN_GNSS_TX        STM32L0_GPIO_PIN_PC10_USART4_TX
#define GPS_PWR_SWITCH                    (2ul)
#define GPS_LEVEL_SHIFTER_EN              (3ul)
#define GPS_RST                           (4ul)
#define GPS_RX                            (5ul)
#define GPS_TX                            (6ul)
#define GPS_PPS                           (7ul)
#define GPS_BAUD_RATE                     115200

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
#define PINS_COUNT           (20u)
#define NUM_DIGITAL_PINS     (19u)
#define NUM_ANALOG_INPUTS    (1u)
#define NUM_ANALOG_OUTPUTS   (0u)

/*
 * Analog pins
 */
#define PIN_A0               (16ul)
#define PIN_VBAT             (16ul)

static const uint8_t A0  = PIN_A0;

#define ADC_RESOLUTION          12

/*
 * Other pins
 */
#define PIN_BUTTON          (0ul)
static const uint8_t BUTTON = PIN_BUTTON;
#define TOUCH_PAD           (0ul)
#define TTP223_VDD_PIN      (1ul)

/*
 * Serial interfaces
 */
#define SERIAL_INTERFACES_COUNT 2

#define PIN_SERIAL1_RX       (5ul)
#define PIN_SERIAL1_TX       (6ul)

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define OLED_RESET           (13u)
#define PIN_WIRE_SDA         (14u)
#define PIN_WIRE_SCL         (15u)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#define SPI_INTERFACES_COUNT 0

#define PWM_INSTANCE_COUNT   0

#define POUT_MAX             (23ul)

/*
 * USB Interface
 */
#define PIN_USB_DM           (17ul)
#define PIN_USB_DP           (18ul) 
#define PIN_USB_VBUS         (19ul)

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
extern CDC  SerialUSB;
extern Uart Serial1;
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
#define SERIAL_PORT_MONITOR         SerialUSB
#define SERIAL_PORT_GNSS            Serial1
#define SERIAL_PORT_HARDWARE1       Serial1

// Alias Serial to SerialUSB
#define Serial                      SerialUSB

#endif /*_VARIANT_T_IMPULSE_S76G_ */

