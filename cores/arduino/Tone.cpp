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
#include "Tone.h"
#include "wiring_private.h"


static GPIO_TypeDef *toneGPIO = NULL;
static uint16_t     toneBit = 0x0000;
static uint32_t     toneCount = 0;

static stm32l0_timer_t stm32l0_tone;

static void tone_event_callback(void *context, uint32_t events)
{
    (void)context;
    (void)events;

    if (toneCount) {
      if (toneGPIO->ODR & toneBit) {
	    toneGPIO->BRR = toneBit;
	} else {
	    toneGPIO->BSRR = toneBit;
	}

	if (toneCount <= 0xfffffffe) {
	    toneCount--;
	}
    } else {
	stm32l0_timer_stop(&stm32l0_tone);
	stm32l0_timer_disable(&stm32l0_tone);

	toneGPIO->BRR = toneBit;
	toneGPIO = NULL;
    }
}

void tone(uint32_t pin, uint32_t frequency, uint32_t duration)
{
    uint32_t modulus;

    if (frequency == 0) {
	return;
    }

    if ( (pin >= PINS_COUNT) || (g_APinDescription[pin].GPIO == NULL) ) {
	return ;
    }

    if (stm32l0_tone.state == STM32L0_TIMER_STATE_NONE) {
	stm32l0_timer_create(&stm32l0_tone, STM32L0_TIMER_INSTANCE_TIM6, STM32L0_TONE_IRQ_PRIORITY, 0);
    }

    GPIO_TypeDef *GPIO = (GPIO_TypeDef *)g_APinDescription[pin].GPIO;
    uint32_t bit = g_APinDescription[pin].bit;

    if ((toneGPIO != GPIO) || (toneBit != bit)) {
	if (toneGPIO) {
	    noTone(pin);
	}

	digitalWrite(pin, LOW);
	pinMode(pin, OUTPUT);
    }

    toneGPIO = GPIO;
    toneBit  = bit;

    /* Use 4MHz as a carrier frequency. The Arduino UNO spec says we need to be able
     * to hit 31.5Hz at the bottom, which means a 63Hz period to toggle the GPIO.
     * Hence 4MHz is upper boundary if the timer counter should still fit into 16 bits.
     */
    modulus = (4000000 / frequency) / 2;

    if (modulus < 1) {
	modulus = 1;
    }

    if (modulus > 65536) {
	modulus = 63356;
    }

    frequency = 4000000 / (modulus * 2);

    if (duration) {
	toneCount = (frequency * duration * 2) / 1000;
    } else {
	toneCount = 0xffffffff;
    }

    if (stm32l0_tone.state == STM32L0_TIMER_STATE_INIT) {
	stm32l0_timer_enable(&stm32l0_tone, (stm32l0_timer_clock(&stm32l0_tone) / 4000000) -1, STM32L0_TIMER_OPTION_COUNT_PRELOAD, tone_event_callback, NULL, STM32L0_TIMER_EVENT_PERIOD);
	stm32l0_timer_start(&stm32l0_tone, modulus -1, false);
    } else {
	stm32l0_timer_period(&stm32l0_tone, modulus -1, false);
    }
}

void noTone(uint32_t pin)
{
    stm32l0_timer_stop(&stm32l0_tone);
    stm32l0_timer_disable(&stm32l0_tone);

    digitalWrite(pin, LOW);

    toneGPIO = NULL;
}
