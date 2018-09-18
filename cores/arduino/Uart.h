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

#pragma once

#include "HardwareSerial.h"

#define UART_RX_BUFFER_SIZE 64
#define UART_TX_BUFFER_SIZE 64

class Uart : public HardwareSerial
{
public:
    enum FlowControl: uint32_t {
	DISABLED           = 0,
 	RTC_CTS            = 3,
	XON_XOFF           = 4,
    };
  
    Uart(struct _stm32l0_uart_t *uart, const struct _stm32l0_uart_params_t *params, void (*serialEventRun)(void));
    void begin(unsigned long baudRate);
    void begin(unsigned long baudrate, uint32_t config);
    void begin(unsigned long baudRate, uint8_t *buffer, size_t size);
    void begin(unsigned long baudrate, uint32_t config, uint8_t *buffer, size_t size);
    void end();
    virtual int available();
    virtual int availableForWrite(void);
    virtual int peek();
    virtual int read();
    virtual size_t read(uint8_t *buffer, size_t size);
    virtual void flush();
    virtual size_t write(const uint8_t data);
    virtual size_t write(const uint8_t *buffer, size_t size);
    using Print::write; // pull in write(str) and write(buf, size) from Print
    operator bool() { return true; }


    // STM32L0 EXTENSION: RTS/CTS state/control
    void rts(bool enable);
    bool cts();

    // STM32L0 EXTENSION: asynchronous write with callback
    bool write(const uint8_t *buffer, size_t size, void(*callback)(void));
    bool write(const uint8_t *buffer, size_t size, Callback callback);

    // STM32L0 EXTENSION: enable/disable non-blocking writes
    void setNonBlocking(bool enable);

    // STM32L0 EXTENSION: enable/disable wakeup from STOP
    void setWakeup(bool enable);

    // STM32L0 EXTENSION: configure flow control
    void setFlowControl(enum FlowControl mode);

    // STM32L0 EXTENSION: asynchronous receive
    void onReceive(void(*callback)(void));
    void onReceive(Callback callback);

private:
    struct _stm32l0_uart_t *_uart;
    bool _enabled;
    bool _nonblocking;
    uint32_t _baudrate;
    uint32_t _option;
    uint8_t _rx_data[UART_RX_BUFFER_SIZE];
    uint8_t _tx_data[UART_TX_BUFFER_SIZE];
    volatile bool _tx_busy;
    volatile uint16_t _tx_write;
    volatile uint16_t _tx_read;
    volatile uint32_t _tx_count;
    volatile uint32_t _tx_size;

    const uint8_t *_tx_data2;
    volatile uint32_t _tx_size2;

    Callback _completionCallback;
    Callback _receiveCallback;

    static void _eventCallback(class Uart *self, uint32_t events);
    static void _doneCallback(class Uart *self);
};
