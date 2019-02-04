/*
 * TWI/I2C library for Arduino Zero
 * Copyright (c) 2015 Arduino LLC. All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef TwoWire_h
#define TwoWire_h

#include "Callback.h"
#include "Stream.h"
#include "variant.h"
#include "stm32l0_i2c.h"

#define BUFFER_LENGTH 32

 // WIRE_HAS_END means Wire has end()
#define WIRE_HAS_END 1

class TwoWire : public Stream
{
public:
    TwoWire(struct _stm32l0_i2c_t *i2c, const struct _stm32l0_i2c_params_t *params);
    TwoWire(const TwoWire&) = delete;
    TwoWire& operator=(const TwoWire&) = delete;

    void begin();
    void begin(uint8_t, bool enableGeneralCall = false);
    void end();
    void setClock(uint32_t);

    void beginTransmission(uint8_t address);
    uint8_t endTransmission(bool stopBit = true);

    size_t requestFrom(uint8_t address, size_t size, bool stopBit = true);
    size_t requestFrom(uint8_t address, size_t size, uint32_t iaddress, uint8_t isize, bool stopBit = true);

    size_t write(uint8_t data);
    size_t write(const uint8_t *buffer, size_t size);

    virtual int available(void);
    virtual int read(void);
    virtual size_t read(uint8_t *buffer, size_t size);
    virtual int peek(void);
    virtual void flush(void);
    void onReceive(void(*)(int));
    void onRequest(void(*)(void));
    void onTransmit(void(*)(int));

    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    using Print::write;

    // STM32L0 EXTENSTION: clock low timeout in milliseconds
    void setClockLowTimeout(unsigned long timeout);

    // STM32L0 EXTENSTION: general call status
    bool isGeneralCall();

    // STM32L0 EXTENSTION: synchronous composite transmit/receive 
    uint8_t transfer(uint8_t address, const uint8_t *txBuffer, size_t txSize, uint8_t *rxBuffer, size_t rxSize, bool stopBit = true);

    // STM32L0 EXTENSTION: reset I2C bus
    void reset();

    // STM32L0 EXTENSTION: scan I2C bus
    uint8_t scan(uint8_t address);

    // STM32L0 EXTENSTION: suspend/resume the I2C bus
    bool suspend();
    void resume();

    // STM32L0 EXTENSTION: enable/disable wakeup from STOP
    void enableWakeup();
    void disableWakeup();

  private:
    struct _stm32l0_i2c_t *_i2c;
    uint32_t _option;
    uint32_t _timeout;
    uint8_t _ev_address;
    uint8_t _xf_address;

    uint8_t _rx_data[BUFFER_LENGTH];
    uint8_t _rx_read;
    uint8_t _rx_write;
    uint8_t _rx_address;

    uint8_t _tx_data[BUFFER_LENGTH];
    uint8_t _tx_write;
    uint8_t _tx_address;
    bool _tx_active;

    void (*_receiveCallback)(int);
    void (*_requestCallback)(void);
    void (*_transmitCallback)(int);

    static void _eventCallback(class TwoWire *self, uint32_t events);

    static const uint32_t TWI_CLOCK = 100000;

    friend class TwoWireTransaction;
};

class TwoWireTransaction {
public:
    TwoWireTransaction();
    ~TwoWireTransaction();
    TwoWireTransaction(const TwoWireTransaction&) = delete;
    TwoWireTransaction& operator=(const TwoWireTransaction&) = delete;

    bool submit(class TwoWire &wire, uint8_t address, const uint8_t *txBuffer, size_t txSize, uint8_t *rxBuffer, size_t rxSize, void(*callback)(void), bool stopBit = true);
    bool submit(class TwoWire &wire, uint8_t address, const uint8_t *txBuffer, size_t txSize, uint8_t *rxBuffer, size_t rxSize, Callback callback, bool stopBit = true);
    bool done();
    uint8_t status();

private:
    stm32l0_i2c_transaction_t _transaction;
    Callback _callback;
    static void _doneCallback(class TwoWireTransaction *self);
};

#if WIRE_INTERFACES_COUNT > 0
  extern TwoWire Wire;
#endif
#if WIRE_INTERFACES_COUNT > 1
  extern TwoWire Wire1;
#endif
#if WIRE_INTERFACES_COUNT > 2
  extern TwoWire Wire2;
#endif

#endif
