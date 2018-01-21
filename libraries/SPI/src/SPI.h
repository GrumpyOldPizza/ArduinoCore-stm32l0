/*
 * SPI Master library for Arduino Zero.
 * Copyright (c) 2015 Arduino LLC
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

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <Arduino.h>

// SPI_HAS_TRANSACTION means SPI has
//   - beginTransaction()
//   - endTransaction()
//   - usingInterrupt()
//   - SPISetting(clock, bitOrder, dataMode)
#define SPI_HAS_TRANSACTION 1

// SPI_HAS_NOTUSINGINTERRUPT means that SPI has notUsingInterrupt() method
#define SPI_HAS_NOTUSINGINTERRUPT 1

#define SPI_MODE0 0x00
#define SPI_MODE1 0x01
#define SPI_MODE2 0x02
#define SPI_MODE3 0x03

#define SPI_MIN_CLOCK_DIVIDER 4

class SPISettings {
public:
  SPISettings() : _clock(4000000), _option(0) { }
  SPISettings(uint32_t clock, BitOrder bitOrder, uint8_t dataMode, bool halfduplex = false) :
    _clock(clock), _option((dataMode & (SPI_CR1_CPHA | SPI_CR1_CPOL)) | ((bitOrder != MSBFIRST) ? SPI_CR1_LSBFIRST : 0) | (halfduplex ? 0x80000000 : 0x00000000)) { }
  
private:
  uint32_t _clock;
  uint32_t _option;

  friend class SPIClass;
};

class SPIClass {
public:
  SPIClass(struct _stm32l0_spi_t *spi, const struct _stm32l0_spi_params_t *params);

  inline uint8_t transfer(uint8_t data) { return _transfer(data); }
  inline uint16_t transfer16(uint16_t data) { return _transfer16(data); }
  inline void transfer(void *buffer, size_t count) { return transfer(buffer, buffer, count); }

  // Transaction Functions
  void usingInterrupt(uint32_t pin);
  void notUsingInterrupt(uint32_t pin);
  void beginTransaction(SPISettings settings);
  void endTransaction(void);

  // SPI Configuration methods
  void attachInterrupt();
  void detachInterrupt();

  void begin();
  void end();

  void setBitOrder(BitOrder bitOrder);
  void setDataMode(uint8_t dataMode);
  void setClockDivider(uint8_t divider);
  void setHalfDuplex(bool enable);

  // STM32L0 EXTENSION: transfer of a 32 bit chunk
  inline uint32_t transfer32(uint32_t data) { return _transfer32(data); }

  // STM32L0 EXTENSION: transfer with separate read/write buffer
  void transfer(const void *txBuffer, void *rxBuffer, size_t count);

private:
    struct _stm32l0_spi_t *_spi;
    bool _active;
    uint32_t _clock;
    uint32_t _option;

    uint8_t (*_transferRoutine)(struct _stm32l0_spi_t*, uint8_t);
    uint16_t (*_transfer16Routine)(struct _stm32l0_spi_t*, uint16_t);
    uint32_t (*_transfer32Routine)(struct _stm32l0_spi_t*, uint32_t);

    uint8_t _transfer(uint8_t data)  __attribute__((__always_inline__)) {
	return (*_transferRoutine)(_spi, data);
    }

    uint16_t _transfer16(uint16_t data) __attribute__((__always_inline__)) {
	return (*_transfer16Routine)(_spi, data);
    }

    uint32_t _transfer32(uint32_t data) __attribute__((__always_inline__)) {
	return (*_transfer32Routine)(_spi, data);
    }
    
    static uint8_t _transferSelect(struct _stm32l0_spi_t *spi, uint8_t data);
    static uint16_t _transfer16Select(struct _stm32l0_spi_t *spi, uint16_t data);
    static uint32_t _transfer32Select(struct _stm32l0_spi_t *spi, uint32_t data);
};

#if SPI_INTERFACES_COUNT > 0
  extern SPIClass SPI;
#endif
#if SPI_INTERFACES_COUNT > 1
  extern SPIClass SPI1;
#endif

#endif
