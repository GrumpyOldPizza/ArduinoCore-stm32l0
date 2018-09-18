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
#include "USBAPI.h"
#include "wiring_private.h"

#if defined(USBCON)

#define CDC_TX_PACKET_SIZE 64

stm32l0_usbd_cdc_t stm32l0_usbd_cdc;

extern int (*stm32l0_stdio_put)(char, FILE*);

static int serialusb_stdio_put(char data, FILE *fp)
{
    (void)fp;

    return SerialUSB.write(&data, 1);
}

CDC::CDC(struct _stm32l0_usbd_cdc_t *usbd_cdc, void (*serialEventRun)(void))
{
    _usbd_cdc = usbd_cdc;

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

    stm32l0_usbd_cdc_create(usbd_cdc);
}

void CDC::begin(unsigned long baudrate)
{
    begin(baudrate, (uint8_t)SERIAL_8N1);
}

void CDC::begin(unsigned long baudrate, uint32_t config)
{
    (void)baudrate;
    (void)config;

    /* If USBD_CDC has already been enabled/initialized by STDIO, just add the notify.
     */

    if (_enabled) {
	flush();
    }

    _enabled = stm32l0_usbd_cdc_enable(_usbd_cdc, &_rx_data[0], sizeof(_rx_data), (stm32l0_usbd_cdc_event_callback_t)CDC::_eventCallback, (void*)this);

    if (_enabled) {
	if (stm32l0_stdio_put == NULL) {
	    stm32l0_stdio_put = serialusb_stdio_put;
	}
    }
}

void CDC::end()
{
    if (_enabled) {
	flush();

	if (stm32l0_stdio_put == serialusb_stdio_put) {
	    stm32l0_stdio_put = NULL;
	}
	
	stm32l0_usbd_cdc_disable(_usbd_cdc);
	
	_enabled = false;
    }
}

int CDC::available()
{
    return stm32l0_usbd_cdc_count(_usbd_cdc);
}

int CDC::availableForWrite(void)
{
    if (!_enabled) {
	return 0;
    }

    if (_tx_size2 != 0) {
	return 0;
    }

    return CDC_TX_BUFFER_SIZE - _tx_count;
}

int CDC::peek()
{
    uint8_t data;

    if (!stm32l0_usbd_cdc_receive(_usbd_cdc, &data, 1, true)) {
	return -1;
    }

    return data;
}

int CDC::read()
{
    uint8_t data;

    if (!stm32l0_usbd_cdc_receive(_usbd_cdc, &data, 1, false)) {
	return -1;
    }

    return data;
}

size_t CDC::read(uint8_t *buffer, size_t size)
{
    return stm32l0_usbd_cdc_receive(_usbd_cdc, (uint8_t*)buffer, size, false);
}

void CDC::flush()
{
    if (__get_IPSR() == 0) {
	while (_tx_busy) {
	    armv6m_core_wait();
	}
    }
}

size_t CDC::write(const uint8_t data)
{
    return write(&data, 1);
}

size_t CDC::write(const uint8_t *buffer, size_t size)
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

	tx_count = CDC_TX_BUFFER_SIZE - _tx_count;

	if (tx_count == 0) {

	    if (_nonblocking || (__get_IPSR() != 0)) {
		break;
	    }

	    if (!_tx_busy) {
		tx_size = _tx_count;
		tx_read = _tx_read;

		if (tx_size > (CDC_TX_BUFFER_SIZE - tx_read)) {
		    tx_size = (CDC_TX_BUFFER_SIZE - tx_read);
		}
		
		if (tx_size > CDC_TX_PACKET_SIZE) {
		    tx_size = CDC_TX_PACKET_SIZE;
		}
		
		_tx_size = tx_size;
		_tx_busy = true;

		if (!stm32l0_usbd_cdc_transmit(_usbd_cdc, &_tx_data[tx_read], tx_size, (stm32l0_usbd_cdc_done_callback_t)CDC::_doneCallback, (void*)this)) {
		    _tx_busy = false;

		    _tx_size = 0;
		    _tx_count = 0;
		    _tx_read = _tx_write;
		}
	    }

	    while (CDC_TX_BUFFER_SIZE == _tx_count) {
		armv6m_core_wait();
	    }

	    tx_count = CDC_TX_BUFFER_SIZE - _tx_count;
	}

	tx_write = _tx_write;

	if (tx_count > (CDC_TX_BUFFER_SIZE - tx_write)) {
	    tx_count = (CDC_TX_BUFFER_SIZE - tx_write);
	}

	if (tx_count > (size - count)) {
	    tx_count = (size - count);
	}

	memcpy(&_tx_data[tx_write], &buffer[count], tx_count);
	count += tx_count;
      
	_tx_write = (unsigned int)(tx_write + tx_count) & (CDC_TX_BUFFER_SIZE -1);

	armv6m_atomic_add(&_tx_count, tx_count);
    }

    if (!_tx_busy) {
	tx_size = _tx_count;
	tx_read = _tx_read;
	
	if (tx_size > (CDC_TX_BUFFER_SIZE - tx_read)) {
	    tx_size = (CDC_TX_BUFFER_SIZE - tx_read);
	}
	
	if (tx_size > CDC_TX_PACKET_SIZE) {
	    tx_size = CDC_TX_PACKET_SIZE;
	}
	
	_tx_size = tx_size;
	_tx_busy = true;
	
	if (!stm32l0_usbd_cdc_transmit(_usbd_cdc, &_tx_data[tx_read], tx_size, (stm32l0_usbd_cdc_done_callback_t)CDC::_doneCallback, (void*)this)) {
	    _tx_busy = false;

	    _tx_size = 0;
	    _tx_count = 0;
	    _tx_read = _tx_write;
	}
    }

    return count;
}

bool CDC::write(const uint8_t *buffer, size_t size, void(*callback)(void))
{
    return write(buffer, size, Callback(callback));
}

bool CDC::write(const uint8_t *buffer, size_t size, Callback callback)
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

	if (!stm32l0_usbd_cdc_transmit(_usbd_cdc, _tx_data2, _tx_size2, (stm32l0_usbd_cdc_done_callback_t)CDC::_doneCallback, (void*)this)) {
	    _tx_busy = false;

	    _completionCallback = Callback();

	    _tx_data2 = NULL;
	    _tx_size2 = 0;

	    return false;
	}
    }

    return true;
}

void CDC::setNonBlocking(bool enabled)
{
    _nonblocking = enabled;
}

void CDC::onReceive(void(*callback)(void))
{
    _receiveCallback = Callback(callback);
}

void CDC::onReceive(Callback callback)
{
    _receiveCallback = callback;
}

CDC::operator bool()
{
    return (_enabled && (stm32l0_usbd_cdc_info.lineState & 1));
}

unsigned long CDC::baud()
{
    return stm32l0_usbd_cdc_info.dwDTERate;
}

uint8_t CDC::stopbits()
{
    return stm32l0_usbd_cdc_info.bCharFormat;
}

uint8_t CDC::paritytype()
{
    return stm32l0_usbd_cdc_info.bParityType;
}

uint8_t CDC::numbits()
{
    return stm32l0_usbd_cdc_info.bDataBits;
}

bool CDC::dtr()
{
    return stm32l0_usbd_cdc_info.lineState & 1;
}

bool CDC::rts() 
{
    return stm32l0_usbd_cdc_info.lineState & 2;
}

void CDC::_eventCallback(class CDC *self, uint32_t events)
{
    if (events & STM32L0_USBD_CDC_EVENT_RECEIVE) {
	self->_receiveCallback.queue();
    }
}

void CDC::_doneCallback(class CDC *self)
{
    unsigned int tx_read, tx_size;

    self->_tx_busy = false;

    tx_size = self->_tx_size;
    
    if (tx_size != 0) {
	self->_tx_read = (self->_tx_read + tx_size) & (CDC_TX_BUFFER_SIZE -1);
      
	armv6m_atomic_sub(&self->_tx_count, tx_size);
      
	self->_tx_size = 0;

	if (self->_tx_count != 0) {
	    tx_size = self->_tx_count;
	    tx_read = self->_tx_read;
		    
	    if (tx_size > (CDC_TX_BUFFER_SIZE - tx_read)) {
		tx_size = (CDC_TX_BUFFER_SIZE - tx_read);
	    }
		    
	    if (tx_size > CDC_TX_PACKET_SIZE) {
		tx_size = CDC_TX_PACKET_SIZE;
	    }
		    
	    self->_tx_size = tx_size;
	    self->_tx_busy = true;
		    
	    if (!stm32l0_usbd_cdc_transmit(self->_usbd_cdc, &self->_tx_data[tx_read], tx_size, (stm32l0_usbd_cdc_done_callback_t)CDC::_doneCallback, (void*)self)) {
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

		if (!stm32l0_usbd_cdc_transmit(self->_usbd_cdc, self->_tx_data2, self->_tx_size2, (stm32l0_usbd_cdc_done_callback_t)CDC::_doneCallback, (void*)self)) {
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

#endif
