/*
 * Copyright (c) 2016-2019 Thomas Roell.  All rights reserved.
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

    _option = STM32L0_I2C_OPTION_MODE_100K;
    _timeout = 0;
    _ev_address = 0;
    _xf_address = 0;

    _rx_read = 0;
    _rx_write = 0;

    _tx_write = 0;
    _tx_active = false;

    _receiveCallback = NULL;
    _requestCallback = NULL;
    _transmitCallback = NULL;
}

void TwoWire::begin()
{
    _ev_address = 0;

    _option &= ~(STM32L0_I2C_OPTION_GENERAL_CALL | STM32L0_I2C_OPTION_ADDRESS_MASK);

    stm32l0_i2c_enable(_i2c, _option, _timeout, NULL, NULL);
}

void TwoWire::begin(uint8_t address, bool generalCall) 
{
    _ev_address = address;

    _option &= ~(STM32L0_I2C_OPTION_GENERAL_CALL | STM32L0_I2C_OPTION_ADDRESS_MASK);

    _option |= (address << STM32L0_I2C_OPTION_ADDRESS_SHIFT);

    if (generalCall) {
        _option |= STM32L0_I2C_OPTION_GENERAL_CALL;
    }

    stm32l0_i2c_enable(_i2c, _option, _timeout, (stm32l0_i2c_event_callback_t)TwoWire::_eventCallback, (void*)this);
}

void TwoWire::end() 
{
    stm32l0_i2c_disable(_i2c);
}

void TwoWire::setClock(uint32_t clock) 
{
    _option &= ~STM32L0_I2C_OPTION_MODE_MASK;

    if      (clock > 400000) { _option |= STM32L0_I2C_OPTION_MODE_1000K; }
    else if (clock > 100000) { _option |= STM32L0_I2C_OPTION_MODE_400K;  }
    else                     { _option |= STM32L0_I2C_OPTION_MODE_100K;  }

    stm32l0_i2c_configure(_i2c, _option, _timeout);
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
//  1 : Parameter error
//  2 : NACK on transmit of address
//  3 : NACK on transmit of data
//  4 : Arbitration lost
//  5 : SCL timeout

uint8_t TwoWire::endTransmission(bool stopBit)
{
    stm32l0_i2c_transaction_t transaction;

    if (!_tx_active) {
        return 1;
    }

    _tx_active = false;

    if (_xf_address && (_xf_address != _tx_address)) {
        return 1;
    }

    transaction.status = STM32L0_I2C_STATUS_SUCCESS;
    transaction.control = stopBit ? 0 : STM32L0_I2C_CONTROL_RESTART;
    transaction.address = _tx_address;
    transaction.tx_data = &_tx_data[0];
    transaction.rx_data = NULL;
    transaction.tx_count = _tx_write;
    transaction.rx_count = 0;
    transaction.callback = NULL;
    transaction.context = NULL;

    if (!stm32l0_i2c_submit(_i2c, &transaction)) {
        return 1;
    }

    _xf_address = 0;

    while (transaction.status == STM32L0_I2C_STATUS_BUSY) {
        armv6m_core_wait();
    }

    if (transaction.status == STM32L0_I2C_STATUS_SUCCESS) {
        if (!stopBit) {
            _xf_address = _tx_address;
        }
    } 

    return transaction.status;
}

size_t TwoWire::requestFrom(uint8_t address, size_t size, bool stopBit)
{
    return requestFrom(address, size, 0, 0, stopBit);
}

size_t TwoWire::requestFrom(uint8_t address, size_t size, uint32_t iaddress, uint8_t isize, bool stopBit)
{
    stm32l0_i2c_transaction_t transaction;
    uint8_t tx_data[3];

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

    if (size > BUFFER_LENGTH) {
        size = BUFFER_LENGTH;
    }

    transaction.status = STM32L0_I2C_STATUS_SUCCESS;
    transaction.control = stopBit ? 0 : STM32L0_I2C_CONTROL_RESTART;
    transaction.address = address;
    transaction.tx_data = NULL;
    transaction.rx_data = &_rx_data[0];
    transaction.tx_count = 0;
    transaction.rx_count = size;
    transaction.callback = NULL;
    transaction.context = NULL;

    if (isize)
    {
        if (isize == 1) 
        {
            tx_data[0] = iaddress >> 0;
        }
        else if (isize == 2) 
        {
            tx_data[0] = iaddress >> 8;
            tx_data[1] = iaddress >> 0;
        }
        else
        {
            tx_data[0] = iaddress >> 16;
            tx_data[1] = iaddress >> 8;
            tx_data[2] = iaddress >> 0;

            isize = 3;
        }

        transaction.tx_data = &tx_data[0];
        transaction.tx_count = isize;
    }

    if (!stm32l0_i2c_submit(_i2c, &transaction)) {
        return 0;
    }

    _xf_address = 0;

    _rx_read = 0;
    _rx_write = 0;

    while (transaction.status == STM32L0_I2C_STATUS_BUSY) {
        armv6m_core_wait();
    }


    if (transaction.status == STM32L0_I2C_STATUS_SUCCESS) {
        if (!stopBit) {
            _xf_address = address;
        }

        _rx_write = size;
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

size_t TwoWire::write(const uint8_t *data, size_t size)
{
    if (!_tx_active) {
        return 0;
    }

    if (size > (unsigned int)(BUFFER_LENGTH - _tx_write)) {
        size = BUFFER_LENGTH - _tx_write;
    }

    memcpy(&_tx_data[_tx_write], data, size);

    _tx_write += size;

    return size;
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

void TwoWire::setClockLowTimeout(unsigned long timeout)
{
    _timeout = timeout;

    stm32l0_i2c_configure(_i2c, _option, _timeout);
}

bool TwoWire::isGeneralCall()
{
    return (_rx_address == 0);
}

// Errors:
//  0 : Success
//  1 : Parameter error
//  2 : (TRANSMIT) NACK on transmit of address
//  3 : (TRANSMIT) NACK on transmit of data
//  4 : (TRANSMIT) Arbitration lost
//  5 : (TRANSMIT) SCL timeout
//  6 : (RECEIVE) NACK on transmit of address
//  7 : (RECEIVE) Arbitration lost
//  8 : (RECEIVE) SCL timeout

uint8_t TwoWire::transfer(uint8_t address, const uint8_t *txBuffer, size_t txSize, uint8_t *rxBuffer, size_t rxSize, bool stopBit)
{
    stm32l0_i2c_transaction_t transaction;

    if (__get_IPSR() != 0) {
        return 1;
    }

    if (_ev_address) {
        return 1;
    }

    if (_xf_address && (_xf_address != address)) {
        return 1;
    }

    if (!txBuffer && txSize)  {
        return 1;
    }

    if (!rxBuffer && rxSize)  {
        return 1;
    }

    if (rxBuffer && !rxSize)  {
        return 1;
    }

    if ((txSize > 65535) || (rxSize > 65536))  {
        return 1;
    }

    transaction.status = STM32L0_I2C_STATUS_SUCCESS;
    transaction.control = stopBit ? 0 : STM32L0_I2C_CONTROL_RESTART;
    transaction.address = address;
    transaction.tx_data = txBuffer;
    transaction.rx_data = rxBuffer;
    transaction.tx_count = txSize;
    transaction.rx_count = rxSize;
    transaction.callback = NULL;
    transaction.context = NULL;

    if (!stm32l0_i2c_submit(_i2c, &transaction)) {
        return 1;
    }

    while (transaction.status == STM32L0_I2C_STATUS_BUSY) {
        armv6m_core_wait();
    }

    if (transaction.status == STM32L0_I2C_STATUS_SUCCESS) {
        if (!stopBit) {
            _xf_address = address;
        }
    } 

    return transaction.status;
}

void TwoWire::reset()
{
    if (__get_IPSR() != 0) {
        return;
    }

    if (stm32l0_i2c_suspend(_i2c, NULL, NULL)) {
        while (_i2c->state != STM32L0_I2C_STATE_SUSPENDED) {
            armv6m_core_wait();
        }
    }

    stm32l0_i2c_reset(_i2c);

    stm32l0_i2c_resume(_i2c);
}

uint8_t TwoWire::scan(uint8_t address)
{
    stm32l0_i2c_transaction_t transaction;

    if (__get_IPSR() != 0) {
        return 0x00;
    }

    if (_ev_address) {
        return 0x00;
    }

    if (_xf_address) {
        return 0x00;
    }

    while (address < 0x77)
    {
        address++;

        if (address < 0x08) {
            address = 0x08;
        }

        transaction.status = STM32L0_I2C_STATUS_SUCCESS;
        transaction.control = 0;
        transaction.address = address;
        transaction.tx_data = &_tx_data[0];
        transaction.rx_data = NULL;
        transaction.tx_count = 0;
        transaction.rx_count = 0;
        transaction.callback = NULL;
        transaction.context = NULL;

        if (!stm32l0_i2c_submit(_i2c, &transaction)) {
            return 1;
        }
        
        while (transaction.status == STM32L0_I2C_STATUS_BUSY) {
            armv6m_core_wait();
        }
        
        if (transaction.status == STM32L0_I2C_STATUS_SUCCESS) {
            return address;
        }
    }

    return 0x00;
}

bool TwoWire::suspend()
{
    if (__get_IPSR() != 0) {
        return false;
    }

    if (stm32l0_i2c_suspend(_i2c, NULL, NULL)) {
        while (_i2c->state != STM32L0_I2C_STATE_SUSPENDED) {
            armv6m_core_wait();
        }
    }

    return true;
}

void TwoWire::resume()
{
    if (__get_IPSR() != 0) {
        return;
    }

    stm32l0_i2c_resume(_i2c);
}

void TwoWire::enableWakeup()
{
    _option |= STM32L0_I2C_OPTION_WAKEUP;

    stm32l0_i2c_configure(_i2c, _option, _timeout);
}

void TwoWire::disableWakeup()
{
    _option &= ~STM32L0_I2C_OPTION_WAKEUP;

    stm32l0_i2c_configure(_i2c, _option, _timeout);
}

void TwoWire::onReceive(void(*callback)(int))
{
    _receiveCallback = callback;
}

void TwoWire::onRequest(void(*callback)(void))
{
    _requestCallback = callback;
}

void TwoWire::onTransmit(void(*callback)(int))
{
    _transmitCallback = callback;
}

void TwoWire::_eventCallback(class TwoWire *self, uint32_t events)
{
    if (events & STM32L0_I2C_EVENT_RECEIVE_REQUEST) {
        self->_rx_address = (events & STM32L0_I2C_EVENT_ADDRESS_MASK) >> STM32L0_I2C_EVENT_ADDRESS_SHIFT;

        stm32l0_i2c_receive(self->_i2c, &self->_rx_data[0], BUFFER_LENGTH);
    }

    if (events & STM32L0_I2C_EVENT_RECEIVE_DONE) {
        self->_rx_read = 0;
        self->_rx_write = (events & STM32L0_I2C_EVENT_COUNT_MASK) >> STM32L0_I2C_EVENT_COUNT_SHIFT;

        if (self->_receiveCallback) {
            (*self->_receiveCallback)(self->_rx_write);
        }
    }
    
    if (events & STM32L0_I2C_EVENT_TRANSMIT_REQUEST) {
        self->_tx_active = true;
        self->_tx_write = 0;

        if (self->_requestCallback) {
            (*self->_requestCallback)();
        }

        self->_tx_active = false;

        stm32l0_i2c_transmit(self->_i2c, &self->_tx_data[0], self->_tx_write);
    }

    if (events & STM32L0_I2C_EVENT_TRANSMIT_DONE) {
        if (self->_transmitCallback) {
            (*self->_transmitCallback)((events & STM32L0_I2C_EVENT_COUNT_MASK) >> STM32L0_I2C_EVENT_COUNT_SHIFT);
        }
    }
}

TwoWireTransaction::TwoWireTransaction()
{
    _transaction.status = STM32L0_I2C_STATUS_SUCCESS;
}

TwoWireTransaction::~TwoWireTransaction()
{
    if (_transaction.status == STM32L0_I2C_STATUS_BUSY) {
        __BKPT();
    }
}

bool TwoWireTransaction::submit(class TwoWire &wire, uint8_t address, const uint8_t *txBuffer, size_t txSize, uint8_t *rxBuffer, size_t rxSize, void(*callback)(void), bool stopBit)
{
    return submit(wire, address, txBuffer, txSize, rxBuffer, rxSize, Callback(callback), stopBit);
}

bool TwoWireTransaction::submit(class TwoWire &wire, uint8_t address, const uint8_t *txBuffer, size_t txSize, uint8_t *rxBuffer, size_t rxSize, Callback callback, bool stopBit)
{
    if (!txBuffer && txSize)  {
        return false;
    }

    if (!rxBuffer && rxSize)  {
        return false;
    }

    if (rxBuffer && !rxSize)  {
        return false;
    }

    if ((txSize > 65535) || (rxSize > 65536))  {
        return false;
    }

    if (wire._ev_address) {
        return false;
    }

    if (_transaction.status == STM32L0_I2C_STATUS_BUSY) {
        return false;
    }

    _transaction.status = STM32L0_I2C_STATUS_SUCCESS;
    _transaction.control = stopBit ? 0 : STM32L0_I2C_CONTROL_RESTART;
    _transaction.address = address;
    _transaction.tx_data = txBuffer;
    _transaction.rx_data = rxBuffer;
    _transaction.tx_count = txSize;
    _transaction.rx_count = rxSize;
    _transaction.callback = (stm32l0_i2c_done_callback_t)TwoWireTransaction::_doneCallback;
    _transaction.context = (void*)this;

    _callback = callback;

    if (!stm32l0_i2c_submit(wire._i2c, &_transaction)) {
        return 1;
    }

    return 0;
}

bool TwoWireTransaction::done()
{
    return (_transaction.status != STM32L0_I2C_STATUS_BUSY);
}

// Status:
//  0 : Success
//  1 : Busy
//  2 : (TRANSMIT) NACK on transmit of address
//  3 : (TRANSMIT) NACK on transmit of data
//  4 : (TRANSMIT) Arbitration lost
//  5 : (TRANSMIT) SCL timeout
//  6 : (RECEIVE) NACK on transmit of address
//  7 : (RECEIVE) Arbitration lost
//  8 : (RECEIVE) SCL timeout

uint8_t TwoWireTransaction::status()
{
    return _transaction.status;
}

void TwoWireTransaction::_doneCallback(class TwoWireTransaction *self)
{
    self->_callback.queue();
}

#if WIRE_INTERFACES_COUNT > 0

extern stm32l0_i2c_t g_Wire;
extern const stm32l0_i2c_params_t g_WireParams;

TwoWire Wire(&g_Wire, &g_WireParams);

#endif

#if WIRE_INTERFACES_COUNT > 1

static stm32l0_i2c_t g_Wire1;
extern const stm32l0_i2c_params_t g_Wire1Params;

TwoWire Wire1(&g_Wire1, &g_Wire1Params);

#endif

#if WIRE_INTERFACES_COUNT > 2

static stm32l0_i2c_t g_Wire2;
extern const stm32l0_i2c_params_t g_Wire2Params;

TwoWire Wire2(&g_Wire2, &g_Wire2Params);

#endif
