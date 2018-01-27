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
#include "Wire.h"
#include "wiring_private.h"

TwoWire::TwoWire(struct _stm32l0_i2c_t *i2c, const struct _stm32l0_i2c_params_t *params)
{
    _i2c = i2c;

    stm32l0_i2c_create(i2c, params);

    _clock = TWI_CLOCK;
    _option = 0;
    _ev_address = 0;
    _xf_address = 0;

    _rx_read = 0;
    _rx_write = 0;
    _rx_active = false;

    _tx_write = 0;
    _tx_active = false;

    _requestCallback = NULL;
    _receiveCallback = NULL;
}

void TwoWire::begin()
{
    _ev_address = 0;

    if      (_clock > 400000) { _option = STM32L0_I2C_OPTION_MODE_1000K; }
    else if (_clock > 100000) { _option = STM32L0_I2C_OPTION_MODE_400K;  }
    else                      { _option = STM32L0_I2C_OPTION_MODE_100K;  }

    stm32l0_i2c_enable(_i2c, _option, NULL, NULL);
}

void TwoWire::begin(uint8_t address, bool generalCall) 
{
    _ev_address = address;

    _option |= (STM32L0_I2C_OPTION_WAKEUP | (address << STM32L0_I2C_OPTION_ADDRESS_SHIFT));

    if (generalCall) {
	_option |= STM32L0_I2C_OPTION_GENERAL_CALL;
    }

    if      (_clock > 400000) { _option |= STM32L0_I2C_OPTION_MODE_1000K; }
    else if (_clock > 100000) { _option |= STM32L0_I2C_OPTION_MODE_400K;  }
    else                      { _option |= STM32L0_I2C_OPTION_MODE_100K;  }

    stm32l0_i2c_enable(_i2c, _option, (stm32l0_i2c_event_callback_t)TwoWire::_eventCallback, (void*)this);
}

void TwoWire::end() 
{
    stm32l0_i2c_disable(_i2c);
}

void TwoWire::setClock(uint32_t clock) 
{
    _clock = clock;

    if      (_clock > 400000) { _option |= STM32L0_I2C_OPTION_MODE_1000K; }
    else if (_clock > 100000) { _option |= STM32L0_I2C_OPTION_MODE_400K;  }
    else                      { _option |= STM32L0_I2C_OPTION_MODE_100K;  }
  
    stm32l0_i2c_configure(_i2c, _option);
}

void TwoWire::beginTransmission(uint8_t address)
{
    if (__get_IPSR() != 0) {
	return;
    }

    if (_ev_address) {
	return;
    }

    if (_tx_active) {
	return;
    }

    _tx_write = 0;
    _tx_address = address;
    _tx_active = true;
}

// Errors:
//  0 : Success
//  1 : Data too long
//  2 : NACK on transmit of address
//  3 : NACK on transmit of data
//  4 : Other error
uint8_t TwoWire::endTransmission(bool stopBit)
{
    if (!_tx_active) {
	return 4;
    }

    _tx_active = false;

    return sendTransmission(_tx_address, &_tx_data[0], _tx_write, stopBit);
}

uint8_t TwoWire::sendTransmission(uint8_t address, const uint8_t *buffer, size_t size, bool stopBit)
{
    stm32l0_i2c_transaction_t transaction;

    if (__get_IPSR() != 0) {
	return 4;
    }

    if (_ev_address) {
	return 4;
    }

    if (_xf_address && (_xf_address != address)) {
	return 4;
    }

    transaction.control = (_xf_address ? STM32L0_I2C_CONTROL_CONTINUE : 0) | (!stopBit ? STM32L0_I2C_CONTROL_RESTART : 0) | STM32L0_I2C_CONTROL_TX;
    transaction.data = (uint8_t*)&buffer[0];
    transaction.data2 = NULL;
    transaction.count = size;
    transaction.count2 = 0;

    transaction.address = address;
    transaction.callback = NULL;
    transaction.context = NULL;

    if (!stm32l0_i2c_enqueue(_i2c, &transaction)) {
	return 4;
    }

    _xf_address = 0;

    while (transaction.status == STM32L0_I2C_STATUS_BUSY) {
	armv6m_core_wait();
    }

    if (transaction.status != STM32L0_I2C_STATUS_SUCCESS) {
	if (transaction.status == STM32L0_I2C_STATUS_ADDRESS_NACK) {
	    return 2;
	}

	if (transaction.status == STM32L0_I2C_STATUS_DATA_NACK) {
	    return 3;
	}

	return 4;
    } 

    if (!stopBit) {
	_xf_address = address;
    }

    return 0;
}

uint8_t TwoWire::sendTransmission(uint8_t address, const uint8_t *buffer, size_t size, uint32_t iaddress, uint8_t isize, bool stopBit)
{
    stm32l0_i2c_transaction_t transaction;

    if (__get_IPSR() != 0) {
	return 4;
    }

    if (_ev_address) {
	return 4;
    }

    if (_xf_address && (_xf_address != address)) {
	return 4;
    }

    if ((isize == 0) || (isize > 4)) {
	return 4;
    }

    if (size == 0)
    {
	transaction.control = (_xf_address ? STM32L0_I2C_CONTROL_CONTINUE : 0) | (!stopBit ? STM32L0_I2C_CONTROL_RESTART : 0) | STM32L0_I2C_CONTROL_TX;
	transaction.data = (uint8_t*)&iaddress;
	transaction.data2 = NULL;
	transaction.count = isize;
	transaction.count2 = 0;
    }
    else
    {
	transaction.control = (_xf_address ? STM32L0_I2C_CONTROL_CONTINUE : 0) | (!stopBit ? STM32L0_I2C_CONTROL_RESTART : 0) | STM32L0_I2C_CONTROL_TX | STM32L0_I2C_CONTROL_TX_SECONDARY;
	transaction.data = (uint8_t*)&iaddress;
	transaction.data2 = (uint8_t*)&buffer[0];
	transaction.count = isize;
	transaction.count2 = size;
    }

    transaction.address = address;
    transaction.callback = NULL;
    transaction.context = NULL;

    if (!stm32l0_i2c_enqueue(_i2c, &transaction)) {
	return 4;
    }

    _xf_address = 0;

    while (transaction.status == STM32L0_I2C_STATUS_BUSY) {
	armv6m_core_wait();
    }

    if (transaction.status != STM32L0_I2C_STATUS_SUCCESS) {
	if (transaction.status == STM32L0_I2C_STATUS_ADDRESS_NACK) {
	    return 2;
	}

	if (transaction.status == STM32L0_I2C_STATUS_DATA_NACK) {
	    return 3;
	}

	return 4;
    }

    if (!stopBit) {
	_xf_address = address;
    }

    return 0;
}

size_t TwoWire::requestFrom(uint8_t address, size_t size, bool stopBit)
{
    if (size == 0) {
	return 0;
    }

    if (size > BUFFER_LENGTH) {
	size = BUFFER_LENGTH;
    }

    size = requestFrom(address, &_rx_data[0], size, stopBit);

    if (size)
    {
	_rx_read = 0;
	_rx_write = size;
    }

    return size;
}

size_t TwoWire::requestFrom(uint8_t address, uint8_t *buffer, size_t size, bool stopBit)
{
    stm32l0_i2c_transaction_t transaction;

    if (__get_IPSR() != 0) {
	return 0;
    }

    if (_ev_address) {
	return 0;
    }

    if (_xf_address && (_xf_address != address)) {
	return 0;
    }

    if (size == 0) {
	return 0;
    }

    transaction.control = (_xf_address ? STM32L0_I2C_CONTROL_CONTINUE : 0) | (!stopBit ? STM32L0_I2C_CONTROL_RESTART : 0) | STM32L0_I2C_CONTROL_RX;
    transaction.data = buffer;
    transaction.data2 = NULL;
    transaction.count = size;
    transaction.count2 = 0;

    transaction.address = address;
    transaction.callback = NULL;
    transaction.context = NULL;

    if (!stm32l0_i2c_enqueue(_i2c, &transaction)) {
	return 0;
    }

    _xf_address = 0;

    while (transaction.status == STM32L0_I2C_STATUS_BUSY) {
	armv6m_core_wait();
    }

    if (transaction.status != STM32L0_I2C_STATUS_SUCCESS) {
	return 0;
    }

    if (!stopBit) {
	_xf_address = address;
    }

    return size;
}

size_t TwoWire::requestFrom(uint8_t address, size_t size, uint32_t iaddress, uint8_t isize, bool stopBit)
{
    if (size > BUFFER_LENGTH) {
	size = BUFFER_LENGTH;
    }

    size = requestFrom(address, &_rx_data[0], size, iaddress, isize, stopBit);

    if (size)
    {
	_rx_read = 0;
	_rx_write = size;
    }

    return size;
}

size_t TwoWire::requestFrom(uint8_t address, uint8_t *buffer, size_t size, uint32_t iaddress, uint8_t isize, bool stopBit)
{
    stm32l0_i2c_transaction_t transaction;

    if (__get_IPSR() != 0) {
	return 0;
    }

    if (_ev_address) {
	return 0;
    }

    if (_xf_address && (_xf_address != address)) {
	return 0;
    }

    if (size == 0) {
	return 0;
    }

    if ((isize == 0) || (isize > 4)) {
	return 0;
    }

    transaction.control = (_xf_address ? STM32L0_I2C_CONTROL_CONTINUE : 0) | (!stopBit ? STM32L0_I2C_CONTROL_RESTART : 0) | STM32L0_I2C_CONTROL_TX | STM32L0_I2C_CONTROL_RX;
    transaction.data = (uint8_t*)&iaddress;
    transaction.data2 = buffer;
    transaction.count = isize;
    transaction.count2 = size;

    transaction.address = address;
    transaction.callback = NULL;
    transaction.context = NULL;

    if (!stm32l0_i2c_enqueue(_i2c, &transaction)) {
	return 0;
    }

    _xf_address = 0;

    while (transaction.status == STM32L0_I2C_STATUS_BUSY) {
	armv6m_core_wait();
    }

    if (transaction.status != STM32L0_I2C_STATUS_SUCCESS) {
	return 0;
    }

    if (!stopBit) {
	_xf_address = address;
    }

    return size;
}

size_t TwoWire::write(uint8_t data)
{
    if (!_tx_active) {
	return 0;
    }

    if (_tx_write >= BUFFER_LENGTH) {
	return 0;
    }

    _tx_data[_tx_write++] = data;

    return 1;
}

size_t TwoWire::write(const uint8_t *data, size_t quantity)
{
    if (!_tx_active) {
	return 0;
    }

    if (quantity > (unsigned int)(BUFFER_LENGTH - _tx_write)) {
	quantity = BUFFER_LENGTH - _tx_write;
    }

    memcpy(&_tx_data[_tx_write], data, quantity);

    _tx_write += quantity;

    return quantity;
}

int TwoWire::available(void)
{
    return (_rx_write - _rx_read);
}

int TwoWire::read(void)
{
    if (_rx_read >= _rx_write) {
	return -1;
    }

    return _rx_data[_rx_read++];
}

size_t TwoWire::read(uint8_t *buffer, size_t size)
{
    if (size > (unsigned int)(_rx_write - _rx_read))
    {
	size = (_rx_write - _rx_read);
    }

    memcpy(buffer, &_rx_data[_rx_read], size);

    _rx_read += size;

    return size;
}

int TwoWire::peek(void)
{
    if (_rx_read >= _rx_write) {
	return -1;
    }

    return _rx_data[_rx_read];
}

void TwoWire::flush(void)
{
}

void TwoWire::reset(void)
{
    stm32l0_i2c_reset(_i2c);
}

void TwoWire::onReceive(void(*callback)(int))
{
    _receiveCallback = callback;
}

void TwoWire::onRequest(void(*callback)(void))
{
    _requestCallback = callback;
}

void TwoWire::_eventCallback(class TwoWire *self, uint32_t events)
{
    if (events & STM32L0_I2C_EVENT_RECEIVE_REQUEST) {
	if (self->_rx_active) {
	    self->_rx_write = BUFFER_LENGTH;

	    if (self->_receiveCallback) {
		(*self->_receiveCallback)(self->_rx_write - self->_rx_read);
	    }

	    if (self->_rx_read != self->_rx_write) {
		memmove(&self->_rx_data[0], &self->_rx_data[self->_rx_read], (self->_rx_write - self->_rx_read));
		
		self->_rx_write = self->_rx_read;
		self->_rx_read = 0;
	    }

	    self->_rx_active = false;
	}

	if (self->_rx_write < BUFFER_LENGTH) {
	    stm32l0_i2c_receive(self->_i2c, &self->_rx_data[self->_rx_write], BUFFER_LENGTH - self->_rx_write, false);

	    self->_rx_active = true;
	}
    }

    if (events & STM32L0_I2C_EVENT_RECEIVE_DONE) {
	self->_rx_write += ((events & STM32L0_I2C_EVENT_COUNT_MASK) >> STM32L0_I2C_EVENT_COUNT_SHIFT);

	if (self->_receiveCallback) {
	    (*self->_receiveCallback)(self->_rx_write - self->_rx_read);
	}

	self->_rx_active = false;
    }
    
    if (events & STM32L0_I2C_EVENT_TRANSMIT_REQUEST) {
	self->_tx_active = true;
	self->_tx_write = 0;

	if(self->_requestCallback) {
	    (*self->_requestCallback)();
	}
      
	stm32l0_i2c_transmit(self->_i2c, &self->_tx_data[0], self->_tx_write);
    }

    if (events & STM32L0_I2C_EVENT_TRANSMIT_DONE) {
	self->_tx_active = false;
	self->_tx_write = 0;
    }
}

TwoWireTransaction::TwoWireTransaction()
{
    _transaction.status = STM32L0_I2C_STATUS_SUCCESS;

    _xf_address = 0;
    _xf_status = 0;
}

TwoWireTransaction::~TwoWireTransaction()
{
}

bool TwoWireTransaction::sendTransmission(class TwoWire &twowire, uint8_t address, const uint8_t *buffer, size_t size, bool stopBit, void(*callback)(void))
{
    if (twowire._ev_address) {
	return false;
    }

    if (_xf_address && (_xf_address != address)) {
	return false;
    }

    _transaction.control = (_xf_address ? STM32L0_I2C_CONTROL_CONTINUE : 0) | (!stopBit ? STM32L0_I2C_CONTROL_RESTART : 0) | STM32L0_I2C_CONTROL_TX;
    _transaction.data = (uint8_t*)&buffer[0];
    _transaction.data2 = NULL;
    _transaction.count = size;
    _transaction.count2 = 0;

    _transaction.address = address;
    _transaction.callback = (stm32l0_i2c_done_callback_t)doneCallback;
    _transaction.context = (void*)this;

    _callback = Callback(callback);

    if (!stm32l0_i2c_enqueue(twowire._i2c, &_transaction)) {
	return false;
    }

    _xf_address = 0;

    return true;
}

bool TwoWireTransaction::sendTransmission(class TwoWire &twowire, uint8_t address, const uint8_t *buffer, size_t size, uint32_t iaddress, uint8_t isize, bool stopBit, void(*callback)(void))
{
    if (twowire._ev_address) {
	return false;
    }

    if (_xf_address && (_xf_address != address)) {
	return false;
    }

    if ((isize == 0) || (isize > 4)) {
	return false;
    }

    if (size == 0)
    {
	_transaction.control = (_xf_address ? STM32L0_I2C_CONTROL_CONTINUE : 0) | (!stopBit ? STM32L0_I2C_CONTROL_RESTART : 0) | STM32L0_I2C_CONTROL_TX;
	_transaction.data = (uint8_t*)&iaddress;
	_transaction.data2 = NULL;
	_transaction.count = isize;
	_transaction.count2 = 0;
    }
    else
    {
	_transaction.control = (_xf_address ? STM32L0_I2C_CONTROL_CONTINUE : 0) | (!stopBit ? STM32L0_I2C_CONTROL_RESTART : 0) | STM32L0_I2C_CONTROL_TX | STM32L0_I2C_CONTROL_TX_SECONDARY;
	_transaction.data = (uint8_t*)&iaddress;
	_transaction.data2 = (uint8_t*)&buffer[0];
	_transaction.count = isize;
	_transaction.count2 = size;
    }

    _transaction.address = address;
    _transaction.callback = (stm32l0_i2c_done_callback_t)doneCallback;
    _transaction.context = (void*)this;

    _callback = Callback(callback);

    if (!stm32l0_i2c_enqueue(twowire._i2c, &_transaction)) {
	return false;
    }

    _xf_address = 0;

    return true;
}

bool TwoWireTransaction::requestFrom(class TwoWire &twowire, uint8_t address, uint8_t *buffer, size_t size, bool stopBit, void(*callback)(void))
{
    if (twowire._ev_address) {
	return false;
    }

    if (_xf_address && (_xf_address != address)) {
	return false;
    }

    if (size == 0) {
	return false;
    }

    _transaction.control = (_xf_address ? STM32L0_I2C_CONTROL_CONTINUE : 0) | (!stopBit ? STM32L0_I2C_CONTROL_RESTART : 0) | STM32L0_I2C_CONTROL_RX;
    _transaction.data = buffer;
    _transaction.data2 = NULL;
    _transaction.count = size;
    _transaction.count2 = 0;

    _transaction.address = address;
    _transaction.callback = (stm32l0_i2c_done_callback_t)doneCallback;
    _transaction.context = (void*)this;

    _callback = Callback(callback);

    if (!stm32l0_i2c_enqueue(twowire._i2c, &_transaction)) {
	return false;
    }

    _xf_address = 0;

    return true;
}

bool TwoWireTransaction::requestFrom(class TwoWire &twowire, uint8_t address, uint8_t *buffer, size_t size, uint32_t iaddress, uint8_t isize, bool stopBit, void(*callback)(void))
{
    if (twowire._ev_address) {
	return false;
    }

    if (_xf_address && (_xf_address != address)) {
	return false;
    }

    if (size == 0) {
	return false;
    }

    if ((isize == 0) || (isize > 4)) {
	return false;
    }

    _transaction.control = (_xf_address ? STM32L0_I2C_CONTROL_CONTINUE : 0) | (!stopBit ? STM32L0_I2C_CONTROL_RESTART : 0) | STM32L0_I2C_CONTROL_TX | STM32L0_I2C_CONTROL_RX;
    _transaction.data = (uint8_t*)iaddress;
    _transaction.data2 = buffer;
    _transaction.count = isize;
    _transaction.count2 = size;

    _transaction.address = address;
    _transaction.callback = (stm32l0_i2c_done_callback_t)doneCallback;
    _transaction.context = (void*)this;

    _callback = Callback(callback);

    if (!stm32l0_i2c_enqueue(twowire._i2c, &_transaction)) {
	return false;
    }

    _xf_address = 0;

    return true;
}

int TwoWireTransaction::status()
{
    return _xf_status;
}

bool TwoWireTransaction::busy()
{
    return (_transaction.status == STM32L0_I2C_STATUS_BUSY);
}

void TwoWireTransaction::doneCallback(class TwoWireTransaction *self)
{
    if (self->_transaction.status != STM32L0_I2C_STATUS_SUCCESS) {
	if (self->_transaction.status == STM32L0_I2C_STATUS_ADDRESS_NACK) {
	    self->_xf_status = 2;
	}

	if (self->_transaction.status == STM32L0_I2C_STATUS_DATA_NACK) {
	    self->_xf_status = 3;
	}

	self->_xf_status = 4;
    	self->_xf_address = 0;
    } else {
	self->_xf_status = 0;

	if (self->_transaction.control & STM32L0_I2C_CONTROL_RESTART) {
	    self->_xf_address = self->_transaction.address;
	}
    }

    self->_callback.queue();
}

#if WIRE_INTERFACES_COUNT > 0

extern stm32l0_i2c_t g_Wire;
extern const stm32l0_i2c_params_t g_WireParams;

TwoWire Wire(&g_Wire, &g_WireParams);

#endif

#if WIRE_INTERFACES_COUNT > 1

static stm32l0_i2c_t f_Wire1;
extern const stm32l0_i2c_params_t g_Wire1Params;

TwoWire Wire1(&g_Wire1, &g_Wire1Params);

#endif

#if WIRE_INTERFACES_COUNT > 2

static stm32l0_i2c_t g_Wire2;
extern const stm32l0_i2c_params_t g_Wire2Params;

TwoWire Wire2(&g_Wire2, &g_Wire2Params);

#endif
