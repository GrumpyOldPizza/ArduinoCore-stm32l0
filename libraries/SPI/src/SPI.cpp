/*
 * Copyright (c) 2016-2020 Thomas Roell.  All rights reserved.
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
#include "SPI.h"
#include "wiring_private.h"

static SPIClass *_spi_class[STM32L0_SPI_INSTANCE_COUNT];

/* The code below deserves some explanation. The SPIClass has really 2 modes of operation.
 * One is the beginTransaction()/endTransaction() which locally braces every atomic transaction,
 * and one is the old model, where you'd configure the SPI PORT directly. For STM32L4 we
 * really, really only want to deal with the transaction model, as this allows to clock gate
 * the peripheral. Hence the transfer*() function are wrapped via indirect functions calls.
 * If a transfer*() function is called outside a transaction then a virtual beginTransaction()
 * is inserted, and if API are called the reconfigure the SPI PORT, then a virtual endTransaction()
 * is inserted.
 */

SPIClass::SPIClass(struct _stm32l0_spi_t *spi, const struct _stm32l0_spi_params_t *params)
{
    _spi = spi;

    stm32l0_spi_create(spi, params);

    _active = false;

    _clock = 4000000;
    _option = STM32L0_SPI_OPTION_MODE_0 | STM32L0_SPI_OPTION_MSB_FIRST;

    _transfer8Routine = SPIClass::_transfer8Select;
    _transfer16Routine = SPIClass::_transfer16Select;

    _spi_class[spi->instance] = this;
}

void SPIClass::begin()
{
    stm32l0_spi_enable(_spi);
}

void SPIClass::end()
{
    if (_active) {
	stm32l0_spi_release(_spi);
    
	_transfer8Routine = SPIClass::_transfer8Select;
	_transfer16Routine = SPIClass::_transfer16Select;
    
	_active = false;
    }

    stm32l0_spi_disable(_spi);
}

void SPIClass::usingInterrupt(uint32_t pin)
{
    if ((pin >= PINS_COUNT) || !(g_APinDescription[pin].attr & PIN_ATTR_EXTI)) {
	return;
    }

    stm32l0_spi_block(_spi, pin);
}

void SPIClass::notUsingInterrupt(uint32_t pin)
{
    if ((pin >= PINS_COUNT) || !(g_APinDescription[pin].attr & PIN_ATTR_EXTI)) {
	return;
    }

    stm32l0_spi_unblock(_spi, pin);
}

void SPIClass::beginTransaction(SPISettings settings)
{
    if (_active) {
	stm32l0_spi_release(_spi);
    }

    stm32l0_spi_acquire(_spi, settings._clock, settings._option);

    _active = true;

    _transfer8Routine = stm32l0_spi_data8;
    _transfer16Routine = stm32l0_spi_data16;
}

void SPIClass::endTransaction(void)
{
    stm32l0_spi_release(_spi);

    _transfer8Routine = SPIClass::_transfer8Select;
    _transfer16Routine = SPIClass::_transfer16Select;
    
    _active = false;
}

void SPIClass::setBitOrder(BitOrder bitOrder)
{
    if (_active) {
	stm32l0_spi_release(_spi);
    
	_transfer8Routine = SPIClass::_transfer8Select;
	_transfer16Routine = SPIClass::_transfer16Select;
    
	_active = false;
    }

    _option = (_option & ~(STM32L0_SPI_OPTION_LSB_FIRST | STM32L0_SPI_OPTION_MSB_FIRST)) | ((bitOrder == MSBFIRST) ? STM32L0_SPI_OPTION_MSB_FIRST : STM32L0_SPI_OPTION_LSB_FIRST);
}

void SPIClass::setDataMode(uint8_t dataMode)
{
    if (_active) {
	stm32l0_spi_release(_spi);
    
	_transfer8Routine = SPIClass::_transfer8Select;
	_transfer16Routine = SPIClass::_transfer16Select;
    
	_active = false;
    }

    _option = (_option & ~STM32L0_SPI_OPTION_MODE_MASK) | (dataMode & STM32L0_SPI_OPTION_MODE_MASK);
}

void SPIClass::setClockDivider(uint8_t divider)
{
    if (_active) {
	stm32l0_spi_release(_spi);
    
	_transfer8Routine = SPIClass::_transfer8Select;
	_transfer16Routine = SPIClass::_transfer16Select;
    
	_active = false;
    }

    if (divider != 0) {
	_clock = SystemCoreClock / divider;
    }
}

void SPIClass::attachInterrupt()
{
  // Should be enableInterrupt()
}

void SPIClass::detachInterrupt()
{
  // Should be disableInterrupt()
}

uint8_t SPIClass::_transfer8Select(struct _stm32l0_spi_t *spi, uint8_t data)
{
    SPIClass *spi_class = _spi_class[spi->instance];

    stm32l0_spi_acquire(spi, spi_class->_clock, spi_class->_option);

    spi_class->_active = true;

    spi_class->_transfer8Routine = stm32l0_spi_data8;
    spi_class->_transfer16Routine = stm32l0_spi_data16;
  
    return (*spi_class->_transfer8Routine)(spi, data);
}

uint16_t SPIClass::_transfer16Select(struct _stm32l0_spi_t *spi, uint16_t data)
{
    SPIClass *spi_class = _spi_class[spi->instance];

    stm32l0_spi_acquire(spi, spi_class->_clock, spi_class->_option);

    spi_class->_active = true;

    spi_class->_transfer8Routine = stm32l0_spi_data8;
    spi_class->_transfer16Routine = stm32l0_spi_data16;
  
    return (*spi_class->_transfer16Routine)(spi, data);
}

void SPIClass::transfer(const void *txBuffer, void *rxBuffer, size_t count) 
{
    if (!_active)
    {
	stm32l0_spi_acquire(_spi, _clock, _option);
	
	_active = true;
	
	_transfer8Routine = stm32l0_spi_data8;
	_transfer16Routine = stm32l0_spi_data16;
    }

    stm32l0_spi_data(_spi, (const uint8_t*)txBuffer, (uint8_t*)rxBuffer, count);
}

bool SPIClass::transfer(const void *txBuffer, void *rxBuffer, size_t count, void(*callback)(void))
{
    return transfer(txBuffer, rxBuffer, count, Callback(callback));
}

bool SPIClass::transfer(const void *txBuffer, void *rxBuffer, size_t count, Callback callback)
{
    if (!stm32l0_spi_done(_spi))
    {
	return false;
    }

    if (!_active)
    {
	stm32l0_spi_acquire(_spi, _clock, _option);
	
	_active = true;
	
	_transfer8Routine = stm32l0_spi_data8;
	_transfer16Routine = stm32l0_spi_data16;
    }

    _callback = callback;

    if (txBuffer) {
	if (rxBuffer) {
	    if (!stm32l0_spi_transfer(_spi, static_cast<const uint8_t*>(txBuffer), static_cast<uint8_t*>(rxBuffer), count, (stm32l0_spi_done_callback_t)SPIClass::_doneCallback, (void*)this)) {
		return false;
	    }
	} else {
	    if (!stm32l0_spi_transmit(_spi, static_cast<const uint8_t*>(txBuffer), count, (stm32l0_spi_done_callback_t)SPIClass::_doneCallback, (void*)this)) {
		return false;
	    }
	}
    }
    else
    {
	if (!stm32l0_spi_receive(_spi, static_cast<uint8_t*>(rxBuffer), count, (stm32l0_spi_done_callback_t)SPIClass::_doneCallback, (void*)this)) {	
	    return false;
	}
    }

    return true;
}

size_t SPIClass::cancel(void)
{
    return stm32l0_spi_cancel(_spi);
}

bool SPIClass::done(void)
{
    return stm32l0_spi_done(_spi);
}

void SPIClass::_doneCallback(class SPIClass *self)
{
    self->_callback.queue(false);;
}

#if SPI_INTERFACES_COUNT > 0

extern stm32l0_spi_t g_SPI;
extern const stm32l0_spi_params_t g_SPIParams;

SPIClass SPI(&g_SPI, &g_SPIParams);

#endif

#if SPI_INTERFACES_COUNT > 1

static stm32l0_spi_t g_SPI1;
extern const stm32l0_spi_params_t g_SPI1Params;

SPIClass SPI1(&g_SPI1, &g_SPI1Params);

#endif
