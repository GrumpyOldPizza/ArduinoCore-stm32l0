/*
 * Copyright (c) 2016-2018 Thomas Roell.  All rights reserved.
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

#include "Arduino.h"
#include "wiring_private.h"

extern void serialEvent() __attribute__((weak));
extern bool Serial_available() __attribute__((weak));

#if defined(USBCON)

bool Serial_available() { return SerialUSB.available(); }

extern stm32l0_usbd_cdc_t g_CDC;

CDC SerialUSB(&g_CDC);

#else

bool Serial_available() { return Serial.available(); }

static stm32l0_uart_t g_Serial;
extern const stm32l0_uart_params_t g_SerialParams;

Uart Serial(&g_Serial, &g_SerialParams);

#endif

#if SERIAL_INTERFACES_COUNT > 1

extern void serialEvent1() __attribute__((weak));
extern bool Serial1_available() __attribute__((weak));

bool Serial1_available() { return Serial1.available(); }

static stm32l0_uart_t g_Serial1;
extern const stm32l0_uart_params_t g_Serial1Params;

Uart Serial1(&g_Serial1, &g_Serial1Params);

#endif

#if SERIAL_INTERFACES_COUNT > 2

extern void serialEvent2() __attribute__((weak));
extern bool Serial2_available() __attribute__((weak));

bool Serial2_available() { return Serial2.available(); }

static stm32l0_uart_t g_Serial2;
extern const stm32l0_uart_params_t g_Serial2Params;

Uart Serial2(&g_Serial2, &g_Serial2Params);

#endif

#if SERIAL_INTERFACES_COUNT > 3

extern void serialEvent3() __attribute__((weak));
extern bool Serial3_available() __attribute__((weak));

bool Serial3_available() { return Serial3.available(); }

static stm32l0_uart_t g_Serial3;
extern const stm32l0_uart_params_t g_Serial3Params;

Uart Serial3(&g_Serial3, &g_Serial3Params);

#endif

#if SERIAL_INTERFACES_COUNT > 4

extern void serialEvent4() __attribute__((weak));
extern bool Serial4_available() __attribute__((weak));

bool Serial4_available() { return Serial4.available(); }

static stm32l0_uart_t g_Serial4;
extern const stm32l0_uart_params_t g_Serial4Params;

Uart Serial4(&g_Serial4, &g_Serial4Params);

#endif

#if SERIAL_INTERFACES_COUNT > 5

extern void serialEvent5() __attribute__((weak));
extern bool Serial5_available() __attribute__((weak));

bool Serial5_available() { return Serial5.available(); }

static stm32l0_uart_t g_Serial5;
extern const stm32l0_uart_params_t g_Serial5Params;

Uart Serial5(&g_Serial5, &g_Serial5Params, (serialEvent5 != NULL));

#endif

void serialEventRun()
{
    if (Serial_available && serialEvent && Serial_available()) serialEvent();
#if SERIAL_INTERFACES_COUNT > 1
    if (Serial1_available && serialEvent1 && Serial1_available()) serialEvent1();
#endif
#if SERIAL_INTERFACES_COUNT > 2
    if (Serial2_available && serialEvent2 && Serial2_available()) serialEvent2();
#endif
#if SERIAL_INTERFACES_COUNT > 3
    if (Serial3_available && serialEvent3 && Serial3_available()) serialEvent3();
#endif
#if SERIAL_INTERFACES_COUNT > 4
    if (Serial4_available && serialEvent4 && Serial4_available()) serialEvent4();
#endif
#if SERIAL_INTERFACES_COUNT > 5
    if (Serial5_available && serialEvent5 && Serial5_available()) serialEvent5();
#endif
}
