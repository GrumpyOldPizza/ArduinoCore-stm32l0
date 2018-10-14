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

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PIN_ATTR_DAC1          (1UL<<0)
#define PIN_ATTR_DAC2          (1UL<<1)
#define PIN_ATTR_EXTI          (1UL<<2)
#define PIN_ATTR_WKUP1         (1UL<<3)   /* PA0  */
#define PIN_ATTR_WKUP2         (1UL<<4)   /* PC13 */
#define PIN_ATTR_TAMP          (1UL<<5)   /* PA0  */
#define PIN_ATTR_SWD           (1UL<<6)   /* PA13/PA14 */

#define PWM_INSTANCE_NONE      255

#define PWM_CHANNEL_1          0
#define PWM_CHANNEL_2          1
#define PWM_CHANNEL_3          2
#define PWM_CHANNEL_4          3
#define PWM_CHANNEL_NONE       255

#define ADC_CHANNEL_0          0
#define ADC_CHANNEL_1          1
#define ADC_CHANNEL_2          2
#define ADC_CHANNEL_3          3
#define ADC_CHANNEL_4          4
#define ADC_CHANNEL_5          5
#define ADC_CHANNEL_6          6
#define ADC_CHANNEL_7          7
#define ADC_CHANNEL_8          8
#define ADC_CHANNEL_9          9
#define ADC_CHANNEL_10         10
#define ADC_CHANNEL_11         11
#define ADC_CHANNEL_12         12
#define ADC_CHANNEL_13         13
#define ADC_CHANNEL_14         14
#define ADC_CHANNEL_15         15
#define ADC_CHANNEL_NONE       255

/* Types used for the table below */
typedef struct _PinDescription
{
  void                    *GPIO;
  uint16_t                bit;
  uint16_t                pin;
  uint8_t                 attr;
  uint8_t                 pwm_instance;
  uint8_t                 pwm_channel;
  uint8_t                 adc_channel;
} PinDescription ;

/* Pins table to be instantiated into variant.cpp */
extern const PinDescription g_APinDescription[] ;

// Low-level pin register query macros
#define digitalPinToPort(P)        ( (void*)g_APinDescription[P].GPIO )
#define digitalPinToBitMask(P)     ( (uint16_t)g_APinDescription[P].bit )
#define portInputRegister(port)    ( &((GPIO_TypeDef*)((void*)(port)))->IDR )
#define portOutputRegister(port)   ( &((GPIO_TypeDef*)((void*)(port)))->ODR )
#define portSetRegister(port)      ( &((GPIO_TypeDef*)((void*)(port)))->BSRR )
#define portClearRegister(port)    ( &((GPIO_TypeDef*)((void*)(port)))->BRR )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].pwm_instance != PWM_INSTANCE_NONE )

#define digitalPinToInterrupt(P)   ( P )
#define analogInputToDigitalPin(P) ( (P < NUM_ANALOG_INPUTS) ? (P) + A0 : -1 )

#ifdef __cplusplus
} // extern "C"
#endif
