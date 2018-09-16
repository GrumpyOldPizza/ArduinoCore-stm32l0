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

#include "Arduino.h"
#include "STM32L0.h"
#include "wiring_private.h"

uint64_t STM32L0Class::getSerial()
{
    uint32_t serial0, serial1, serial2;

    /* This crummy value is what the USB/DFU bootloader uses.
     */

    serial0 = *((uint32_t*)0x1ff80050);
    serial1 = *((uint32_t*)0x1ff80054);
    serial2 = *((uint32_t*)0x1ff80058);

    serial0 += serial2;

    return (((uint64_t)serial0 << 16) | ((uint64_t)serial1 >> 16));
}

void STM32L0Class::getUID(uint32_t uid[3])
{
    stm32l0_system_uid(uid);
}

bool STM32L0Class::getVBUS()
{
#if defined(STM32L0_CONFIG_PIN_VBUS)
    if (STM32L0_CONFIG_PIN_VBUS == STM32L0_GPIO_PIN_NONE) {
	return false;
    }

    return !!stm32l0_gpio_pin_read(STM32L0_CONFIG_PIN_VBUS);

#else /* defined(STM32L0_CONFIG_PIN_VBUS */

    return false;

#endif /* defined(STM32L0_CONFIG_PIN_VBUS */
}

float STM32L0Class::getVBAT()
{
#if defined(STM32L0_CONFIG_PIN_VBAT)
    int32_t vrefint_data, vbat_data;
    float vdda;

    vrefint_data = __analogReadInternal(STM32L0_ADC_CHANNEL_VREFINT, STM32L0_ADC_VREFINT_PERIOD);
    vbat_data = __analogReadInternal(STM32L0_CONFIG_CHANNEL_VBAT, STM32L0_CONFIG_VBAT_PERIOD);

    vdda = (3.0 * STM32L0_ADC_VREFINT_CAL) / vrefint_data;

    return (STM32L0_CONFIG_VBAT_SCALE * vdda * vbat_data) / 4095.0;

#else /* defined(STM32L0_CONFIG_PIN_VBAT) */
    
    return -1;

#endif /* defined(STM32L0_CONFIG_PIN_VBAT) */
}

float STM32L0Class::getVDDA()
{
    int32_t vrefint_data;

    vrefint_data = __analogReadInternal(STM32L0_ADC_CHANNEL_VREFINT, STM32L0_ADC_VREFINT_PERIOD);

    return (3.0 * STM32L0_ADC_VREFINT_CAL) / vrefint_data;
}

float STM32L0Class::getTemperature()
{
    int32_t vrefint_data, tsense_data;

    vrefint_data = __analogReadInternal(STM32L0_ADC_CHANNEL_VREFINT, STM32L0_ADC_VREFINT_PERIOD);
    tsense_data = __analogReadInternal(STM32L0_ADC_CHANNEL_TSENSE, STM32L0_ADC_TSENSE_PERIOD);

    /* Compensate TSENSE_DATA for VDDA vs. 3.0 */
    tsense_data = (tsense_data * STM32L0_ADC_VREFINT_CAL) / vrefint_data;

    return (30.0 + (100.0 * (float)(tsense_data - STM32L0_ADC_TSENSE_CAL1)) / (float)(STM32L0_ADC_TSENSE_CAL2 - STM32L0_ADC_TSENSE_CAL1));
}

uint32_t STM32L0Class::resetCause()
{
    return stm32l0_system_reset_cause();
}

void STM32L0Class::wakeup()
{
    stm32l0_system_wakeup();
}

void STM32L0Class::sleep(uint32_t timeout)
{
    stm32l0_system_sleep(STM32L0_SYSTEM_POLICY_SLEEP, timeout);
}

void STM32L0Class::stop(uint32_t timeout)
{
    if (g_swdStatus == 0) {
	stm32l0_system_swd_disable();

	g_swdStatus = 2;
    }

    stm32l0_system_sleep(STM32L0_SYSTEM_POLICY_STOP, timeout);
}

void STM32L0Class::standby()
{
    stm32l0_system_standby(0);
}

void STM32L0Class::standby(uint32_t pin)
{
    uint32_t config;

    if ( (pin >= PINS_COUNT) || !(g_APinDescription[pin].attr & (PIN_ATTR_WKUP1 | PIN_ATTR_WKUP2)))  {
	return;
    }
    
    if (g_APinDescription[pin].attr & PIN_ATTR_WKUP1) {
	config = STM32L0_SYSTEM_CONFIG_WKUP1;
    }

    if (g_APinDescription[pin].attr & PIN_ATTR_WKUP2) {
	config = STM32L0_SYSTEM_CONFIG_WKUP2;
    }

    stm32l0_system_standby(config);
}

void STM32L0Class::reset()
{
    stm32l0_system_reset();
}

void STM32L0Class::swdEnable()
{
    if (g_swdStatus != 3) {
	stm32l0_system_swd_enable();

	g_swdStatus = 1;
    }
}

void STM32L0Class::swdDisable()
{
    if (g_swdStatus != 3) {
	stm32l0_system_swd_disable();

	g_swdStatus = 2;
    }
}

void STM32L0Class::wdtEnable(uint32_t timeout)
{
    stm32l0_iwdg_enable(timeout);
}

void STM32L0Class::wdtReset()
{
    stm32l0_iwdg_reset();
}

bool STM32L0Class::flashErase(uint32_t address, uint32_t count)
{
    if (address & 127) {
	return false;
    }

    count = (count + 127) & ~127;

    if ((address < FLASHSTART) || ((address + count) > FLASHEND)) {
	return false;
    }

    stm32l0_flash_unlock();
    stm32l0_flash_erase(address, count);
    stm32l0_flash_lock();
    
    return true;
}

bool STM32L0Class::flashProgram(uint32_t address, const void *data, uint32_t count)
{
    if ((address & 3) || (count & 3)) {
	return false;
    }

    if ((address < FLASHSTART) || ((address + count) > FLASHEND)) {
	return false;
    }

    if (count) {
	stm32l0_flash_unlock();
	stm32l0_flash_program(address, (const uint8_t*)data, count);
	stm32l0_flash_lock();
    }

    return true;
}

STM32L0Class STM32L0;
