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
#include "Uart.h"
#include "wiring_private.h"

#define UART_TX_PACKET_SIZE 16

Uart::Uart(struct _stm32l0_uart_t *uart, const struct _stm32l0_uart_params_t *params, void (*serialEventRun)(void))
{
    _uart = uart;

    _enabled = false;
    _nonblocking = false;

    _tx_busy = false;

    _tx_read = 0;
    _tx_write = 0;
    _tx_count = 0;
    _tx_size = 0;

    _tx_data2 = NULL;
    _tx_size2 = 0;

    if (serialEventRun) {
	g_serialEventRun = serialEventRun;
    }

    stm32l0_uart_create(uart, params);
}

void Uart::begin(unsigned long baudrate)
{
    begin(baudrate, SERIAL_8N1, &_rx_data[0], sizeof(_rx_data));
}

void Uart::begin(unsigned long baudrate, uint32_t config)
{
    begin(baudrate, config, &_rx_data[0], sizeof(_rx_data));
}

void Uart::begin(unsigned long baudrate, uint8_t *buffer, size_t size)
{
    begin(baudrate, SERIAL_8N1, buffer, size);
}

void Uart::begin(unsigned long baudrate, uint32_t config, uint8_t *buffer, size_t size)
{
    uint32_t option;

    if (((_uart->pins.rx & STM32L0_GPIO_PIN_IO_MASK) == STM32L0_GPIO_PIN_PA13) || ((_uart->pins.tx & STM32L0_GPIO_PIN_IO_MASK) == STM32L0_GPIO_PIN_PA14)) {
	if (g_swdStatus != 3) {
	    stm32l0_system_swd_disable();

	    g_swdStatus = 3;
	}
    }
	
    if (_enabled) {
	flush();
	stm32l0_uart_disable(_uart);
    }

    option = (_option & (STM32L0_UART_OPTION_RTS | STM32L0_UART_OPTION_CTS | STM32L0_UART_OPTION_XONOFF | STM32L0_UART_OPTION_WAKEUP)) | config;

    _enabled = stm32l0_uart_enable(_uart, buffer, size, baudrate, option, (stm32l0_uart_event_callback_t)Uart::_eventCallback, (void*)this);

    if (_enabled)
    {
	_baudrate = baudrate;
	_option = option;
    }
}

void Uart::end()
{
    if (_enabled) {
	flush();

	stm32l0_uart_disable(_uart);

	_enabled = false;
    }
}

int Uart::available()
{
    return stm32l0_uart_count(_uart);
}

int Uart::availableForWrite()
{
    if (!_enabled) {
	return 0;
    }

    if (_tx_size2 != 0) {
	return 0;
    }

    return UART_TX_BUFFER_SIZE - _tx_count;
}

int Uart::peek()
{
    uint8_t data;

    if (!stm32l0_uart_receive(_uart, &data, 1, true)) {
	return -1;
    }

    return data;
}

int Uart::read()
{
    uint8_t data;

    if (!stm32l0_uart_receive(_uart, &data, 1, false)) {
	return -1;
    }

    return data;
}

size_t Uart::read(uint8_t *buffer, size_t size)
{
    return stm32l0_uart_receive(_uart, (uint8_t*)buffer, size, false);
}

void Uart::flush()
{
    if (__get_IPSR() == 0) {
	while (_tx_busy) {
	    armv6m_core_wait();
	}
    }
}

size_t Uart::write(const uint8_t data)
{
    return write(&data, 1);
}

size_t Uart::write(const uint8_t *buffer, size_t size)
{
    unsigned int tx_read, tx_write, tx_count, tx_size;
    size_t count;

    if (!_enabled) {
	return 0;
    }

    if (size == 0) {
	return 0;
    }

    if (_tx_size2 != 0) {
	if (_nonblocking || (__get_IPSR() != 0)) {
	    return 0;
	}
	
	while (_tx_size2 != 0) {
	    armv6m_core_wait();
	}
    }
      
    count = 0;

    while (count < size) {

	tx_count = UART_TX_BUFFER_SIZE - _tx_count;

	if (tx_count == 0) {

	    if (_nonblocking || (__get_IPSR() != 0)) {
		break;
	    }

	    if (!_tx_busy) {
		tx_size = _tx_count;
		tx_read = _tx_read;

		if (tx_size > (UART_TX_BUFFER_SIZE - tx_read)) {
		    tx_size = (UART_TX_BUFFER_SIZE - tx_read);
		}
		
		if (tx_size > UART_TX_PACKET_SIZE) {
		    tx_size = UART_TX_PACKET_SIZE;
		}
		
		_tx_size = tx_size;
		_tx_busy = true;

		if (!stm32l0_uart_transmit(_uart, &_tx_data[tx_read], tx_size, (stm32l0_uart_done_callback_t)Uart::_doneCallback, (void*)this)) {
		    _tx_busy = false;

		    _tx_size = 0;
		    _tx_count = 0;
		    _tx_read = _tx_write;
		}
	    }

	    while (UART_TX_BUFFER_SIZE == _tx_count) {
		armv6m_core_wait();
	    }

	    tx_count = UART_TX_BUFFER_SIZE - _tx_count;
	}

	tx_write = _tx_write;

	if (tx_count > (UART_TX_BUFFER_SIZE - tx_write)) {
	    tx_count = (UART_TX_BUFFER_SIZE - tx_write);
	}

	if (tx_count > (size - count)) {
	    tx_count = (size - count);
	}

	memcpy(&_tx_data[tx_write], &buffer[count], tx_count);
	count += tx_count;
      
	_tx_write = (unsigned int)(tx_write + tx_count) & (UART_TX_BUFFER_SIZE -1);

	armv6m_atomic_add(&_tx_count, tx_count);
    }

    if (!_tx_busy) {
	tx_size = _tx_count;
	tx_read = _tx_read;
	
	if (tx_size) {
	    if (tx_size > (UART_TX_BUFFER_SIZE - tx_read)) {
		tx_size = (UART_TX_BUFFER_SIZE - tx_read);
	    }
	    
	    if (tx_size > UART_TX_PACKET_SIZE) {
		tx_size = UART_TX_PACKET_SIZE;
	    }
	    
	    _tx_size = tx_size;
	    _tx_busy = true;
	    
	    if (!stm32l0_uart_transmit(_uart, &_tx_data[tx_read], tx_size, (stm32l0_uart_done_callback_t)Uart::_doneCallback, (void*)this)) {
		_tx_busy = false;

		_tx_size = 0;
		_tx_count = 0;
		_tx_read = _tx_write;
	    }
	}
    }

    return count;
}

bool Uart::write(const uint8_t *buffer, size_t size, void(*callback)(void))
{
    return write(buffer, size, Callback(callback));
}

bool Uart::write(const uint8_t *buffer, size_t size, Callback callback)
{
    if (!_enabled) {
	return false;
    }

    if (size == 0) {
	return false;
    }

    if (_tx_size2 != 0) {
	return false;
    }

    _completionCallback = callback;

    _tx_data2 = buffer;
    _tx_size2 = size;

    if (!_tx_busy) {
	_tx_busy = true;

	if (!stm32l0_uart_transmit(_uart, _tx_data2, _tx_size2, (stm32l0_uart_done_callback_t)Uart::_doneCallback, (void*)this)) {
	    _tx_busy = false;

	    _completionCallback = Callback();

	    _tx_data2 = NULL;
	    _tx_size2 = 0;

	    return false;
	}
    }

    return true;
}

void Uart::rts(bool enable)
{
    stm32l0_uart_rts_enable(_uart, enable);
}

bool Uart::cts()
{
    return stm32l0_uart_cts_holding(_uart);
}

void Uart::setNonBlocking(bool enable)
{
    _nonblocking = enable;
}

void Uart::setWakeup(bool enable)
{
    _option = (_option & ~STM32L0_UART_OPTION_WAKEUP) | (enable ? STM32L0_UART_OPTION_WAKEUP : 0);

    stm32l0_uart_configure(_uart, _baudrate, _option);
}

void Uart::setFlowControl(enum FlowControl mode)
{
    _option = ((_option & ~(STM32L0_UART_OPTION_RTS | STM32L0_UART_OPTION_CTS | STM32L0_UART_OPTION_XONOFF)) |
	       ((mode == 3) ? (STM32L0_UART_OPTION_RTS | STM32L0_UART_OPTION_CTS)
		: ((mode == 4) ? (STM32L0_UART_OPTION_XONOFF) : 0)));

    stm32l0_uart_configure(_uart, _baudrate, _option);
}

void Uart::onReceive(void(*callback)(void))
{
    _receiveCallback = Callback(callback);
}

void Uart::onReceive(Callback callback)
{
    _receiveCallback = callback;
}

void Uart::_eventCallback(class Uart *self, uint32_t events)
{
    if (events & STM32L0_UART_EVENT_RECEIVE) {
	self->_receiveCallback.queue();
    }
}

void Uart::_doneCallback(class Uart *self)
{
    unsigned int tx_read, tx_size;

    self->_tx_busy = false;

    tx_size = self->_tx_size;

    if (tx_size != 0) {
	self->_tx_read = (self->_tx_read + tx_size) & (UART_TX_BUFFER_SIZE -1);
      
	armv6m_atomic_sub(&self->_tx_count, tx_size);
      
	self->_tx_size = 0;

	if (self->_tx_count != 0) {
	    tx_size = self->_tx_count;
	    tx_read = self->_tx_read;

	    if (tx_size > (UART_TX_BUFFER_SIZE - tx_read)) {
		tx_size = (UART_TX_BUFFER_SIZE - tx_read);
	    }
	  
	    if (tx_size > UART_TX_PACKET_SIZE) {
		tx_size = UART_TX_PACKET_SIZE;
	    }
	    
	    self->_tx_size = tx_size;
	    self->_tx_busy = true;
	    
	    if (!stm32l0_uart_transmit(self->_uart, &self->_tx_data[tx_read], tx_size, (stm32l0_uart_done_callback_t)Uart::_doneCallback, (void*)self)) {
		self->_tx_busy = false;

		self->_tx_size = 0;
		self->_tx_count = 0;
		self->_tx_read = self->_tx_write;

		if (self->_tx_size2 != 0) {
		    self->_tx_size2 = 0;
		    self->_tx_data2 = NULL;

                    self->_completionCallback.queue();
		    self->_completionCallback = Callback();
		}
	    }
	} else {
	    if (self->_tx_size2 != 0) {
		self->_tx_busy = true;

		if (!stm32l0_uart_transmit(self->_uart, self->_tx_data2, self->_tx_size2, (stm32l0_uart_done_callback_t)Uart::_doneCallback, (void*)self)) {
		    self->_tx_busy = false;

		    self->_tx_size2 = 0;
		    self->_tx_data2 = NULL;

                    self->_completionCallback.queue();
		    self->_completionCallback = Callback();
		}
	    }
	}
    } else {
	self->_tx_size2 = 0;
	self->_tx_data2 = NULL;

	self->_completionCallback.queue();
	self->_completionCallback = Callback();
    }
}
