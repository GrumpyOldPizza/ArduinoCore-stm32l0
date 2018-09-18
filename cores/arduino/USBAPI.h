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

class USBDeviceClass
{
public:
    USBDeviceClass() {};

    // USB Device API
    void init();
    bool attach();
    bool detach();
    void poll();
    
    bool configured();
    bool connected();
    bool suspended();

private:
    bool _initialized;
};

extern USBDeviceClass USBDevice;

#define CDC_RX_BUFFER_SIZE 256
#define CDC_TX_BUFFER_SIZE 256

class CDC : public HardwareSerial
{
public:
    CDC(struct _stm32l0_usbd_cdc_t *usbd_cdc, void (*serialEventRun)(void));
    void begin(unsigned long baudRate);
    void begin(unsigned long baudrate, uint32_t config);
    void end(void);

    virtual int available(void);
    virtual int availableForWrite(void);
    virtual int peek(void);
    virtual int read(void);
    virtual size_t read(uint8_t *buffer, size_t size);
    virtual void flush(void);
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *buffer, size_t size);
    using Print::write; // pull in write(str) from Print
    operator bool();

    // These return the settings specified by the USB host for the
    // serial port. These aren't really used, but are offered here
    // in case a sketch wants to act on these settings.
    uint32_t baud();
    uint8_t stopbits();
    uint8_t paritytype();
    uint8_t numbits();
    bool dtr();
    bool rts();

    enum {
	ONE_STOP_BIT = 0,
	ONE_AND_HALF_STOP_BIT = 1,
	TWO_STOP_BITS = 2,
    };
    enum {
	NO_PARITY = 0,
	ODD_PARITY = 1,
	EVEN_PARITY = 2,
	MARK_PARITY = 3,
	SPACE_PARITY = 4,
    };

    // STM32L0 EXTENSTION: asynchronous write with callback
    bool write(const uint8_t *buffer, size_t size, void(*callback)(void));
    bool write(const uint8_t *buffer, size_t size, Callback callback);

    // STM32L0 EXTENSTION: enable/disable non-blocking writes
    void setNonBlocking(bool enable);

    // STM32L0 EXTENSTION: asynchronous receive
    void onReceive(void(*callback)(void));
    void onReceive(Callback callback);

private:
    struct _stm32l0_usbd_cdc_t *_usbd_cdc;
    bool _enabled;
    bool _nonblocking;
    uint8_t _rx_data[CDC_RX_BUFFER_SIZE];
    uint8_t _tx_data[CDC_TX_BUFFER_SIZE];
    volatile bool _tx_busy;
    volatile uint16_t _tx_write;
    volatile uint16_t _tx_read;
    volatile uint32_t _tx_count;
    volatile uint32_t _tx_size;

    const uint8_t *_tx_data2;
    volatile uint32_t _tx_size2;

    Callback _completionCallback;
    Callback _receiveCallback;

    static void _eventCallback(class CDC *self, uint32_t events);
    static void _doneCallback(class CDC *self);
};


#define MOUSE_LEFT 1
#define MOUSE_RIGHT 2
#define MOUSE_MIDDLE 4
#define MOUSE_ALL (MOUSE_LEFT | MOUSE_RIGHT | MOUSE_MIDDLE)

class MouseClass
{
public:
    MouseClass(void);
    void begin(void);
    void end(void);
    void click(uint8_t b = MOUSE_LEFT);
    void move(signed char x, signed char y, signed char wheel = 0);	
    void press(uint8_t b = MOUSE_LEFT);		// press LEFT by default
    void release(uint8_t b = MOUSE_LEFT);	// release LEFT by default
    bool isPressed(uint8_t b = MOUSE_LEFT);	// check LEFT by default

private:
    uint8_t _buttons;
    void buttons(uint8_t b);
};

extern MouseClass Mouse;

#define KEY_LEFT_CTRL		0x80
#define KEY_LEFT_SHIFT		0x81
#define KEY_LEFT_ALT		0x82
#define KEY_LEFT_GUI		0x83
#define KEY_RIGHT_CTRL		0x84
#define KEY_RIGHT_SHIFT		0x85
#define KEY_RIGHT_ALT		0x86
#define KEY_RIGHT_GUI		0x87

#define KEY_UP_ARROW		0xDA
#define KEY_DOWN_ARROW		0xD9
#define KEY_LEFT_ARROW		0xD8
#define KEY_RIGHT_ARROW		0xD7
#define KEY_BACKSPACE		0xB2
#define KEY_TAB			0xB3
#define KEY_RETURN		0xB0
#define KEY_ESC			0xB1
#define KEY_INSERT		0xD1
#define KEY_DELETE		0xD4
#define KEY_PAGE_UP		0xD3
#define KEY_PAGE_DOWN		0xD6
#define KEY_HOME		0xD2
#define KEY_END			0xD5
#define KEY_CAPS_LOCK		0xC1
#define KEY_F1			0xC2
#define KEY_F2			0xC3
#define KEY_F3			0xC4
#define KEY_F4			0xC5
#define KEY_F5			0xC6
#define KEY_F6			0xC7
#define KEY_F7			0xC8
#define KEY_F8			0xC9
#define KEY_F9			xCA
#define KEY_F10			0xCB
#define KEY_F11			0xCC
#define KEY_F12			0xCD

//	Low level key report: up to 6 keys and shift, ctrl etc at once
typedef struct
{
    uint8_t modifiers;
    uint8_t reserved;
    uint8_t keys[6];
} KeyReport;

class KeyboardClass: public Print
{
public:
    KeyboardClass(void);
    void begin(void);
    void end(void);
    virtual size_t write(uint8_t k);
    virtual size_t press(uint8_t k);
    virtual size_t release(uint8_t k);
    virtual void releaseAll(void);
private:
    KeyReport _keyReport;
    void sendReport(KeyReport* keys);
};

extern KeyboardClass Keyboard;
