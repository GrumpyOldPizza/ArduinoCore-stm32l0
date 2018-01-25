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

#include "Stream.h"
#include "variant.h"

#define BUFFER_LENGTH 32

 // WIRE_HAS_END means Wire has end()
#define WIRE_HAS_END 1

class TwoWire : public Stream
{
  public:
    TwoWire(struct _stm32l0_i2c_t *i2c, const struct _stm32l0_i2c_params_t *params);
    void begin();
    void begin(uint8_t, bool enableGeneralCall = false);
    void end();
    void setClock(uint32_t);

    void beginTransmission(uint8_t address);
    uint8_t endTransmission(bool stopBit = true);
    uint8_t sendTransmission(uint8_t address, const uint8_t *buffer, size_t size, bool stopBit = true);
    uint8_t sendTransmission(uint8_t address, const uint8_t *buffer, size_t size, uint32_t iaddress, uint8_t isize, bool stopBit = true);

    size_t requestFrom(uint8_t address, size_t size, bool stopBit = true);
    size_t requestFrom(uint8_t address, uint8_t *buffer, size_t size, bool stopBit = true);
    size_t requestFrom(uint8_t address, size_t size, uint32_t iaddress, uint8_t isize, bool stopBit = true);
    size_t requestFrom(uint8_t address, uint8_t *buffer, size_t size, uint32_t iaddress, uint8_t isize, bool stopBit = true);

    size_t write(uint8_t data);
    size_t write(const uint8_t *buffer, size_t size);

    virtual int available(void);
    virtual int read(void);
    virtual size_t read(uint8_t *buffer, size_t size);
    virtual int peek(void);
    virtual void flush(void);
    void onReceive(void(*)(int));
    void onRequest(void(*)(void));

    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    using Print::write;

    // STM32L0 EXTENSTION: reset I2C bus
    void reset(void);

  private:
    struct _stm32l0_i2c_t *_i2c;
    uint32_t _clock;
    uint32_t _option;
    uint8_t _ev_address;
    uint8_t _xf_address;

    uint8_t _rx_data[BUFFER_LENGTH];
    uint8_t _rx_read;
    uint8_t _rx_write;
    bool _rx_active;

    uint8_t _tx_data[BUFFER_LENGTH];
    uint8_t _tx_write;
    uint8_t _tx_address;
    bool _tx_active;

    void (*_requestCallback)(void);
    void (*_receiveCallback)(int);

    static void _eventCallback(class TwoWire *self, uint32_t events);

    static const uint32_t TWI_CLOCK = 100000;

    friend class TwoWireTransaction;
};

class TwoWireTransaction {
public:
    TwoWireTransaction(class TwoWire &twowire);
    ~TwoWireTransaction();

    bool sendTransmission(uint8_t address, const uint8_t *buffer, size_t size, bool stopBit, void(*callback)(void) = NULL);
    bool sendTransmission(uint8_t address, const uint8_t *buffer, size_t size, uint32_t iaddress, uint8_t isize, bool stopBit, void(*callback)(void) = NULL);
    bool requestFrom(uint8_t address, uint8_t *buffer, size_t size, bool stopBit, void(*callback)(void) = NULL);
    bool requestFrom(uint8_t address, uint8_t *buffer, size_t size, uint32_t iaddress, uint8_t isize, bool stopBit, void(*callback)(void) = NULL);
    bool busy();
    int status();

private:
    class TwoWire *_twowire;
    struct _stm32l0_i2c_transaction_t *_transaction;
    void (*_callback)(void);
    uint8_t _xf_address;
    uint8_t _xf_status;
    static void doneCallback(class TwoWireTransaction *self);
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
