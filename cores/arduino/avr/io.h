/*
  io.h - Definitions for compatibility with AVR io macros

  Copyright (c) 2016 Arduino LLC

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE
*/

#ifndef _IO_H_
#define _IO_H_

#include <Arduino.h>
#include <assert.h>

#define RAMSTART (SRAM_BASE)
#define RAMSIZE  (SRAM_SIZE_MAX)
#define RAMEND   (RAMSTART + RAMSIZE - 1)

#if defined(DATA_EEPROM_BASE) && defined(DATA_EEPROM_END)
  #define REAL_E2END (DATA_EEPROM_END - DATA_EEPROM_BASE)
#elif defined(DATA_EEPROM_BASE) && defined(DATA_EEPROM_BANK2_BASE) && defined(DATA_EEPROM_BANK1_END) && defined(DATA_EEPROM_BANK2_END)
  #define REAL_E2END (DATA_EEPROM_BANK1_END - DATA_EEPROM_BASE + 1 + DATA_EEPROM_BANK2_END - DATA_EEPROM_BANK2_BASE)
#else
  #error "Cannot determine EEPROM size"
#endif

#if defined(STM32L0_CONFIG_EEPROM_RESERVED)
  static_assert(STM32L0_CONFIG_EEPROM_RESERVED <= REAL_E2END + 1, "STM32L0_CONFIG_EEPROM_RESERVED bigger than EEPROM");
  #define E2END (REAL_E2END - STM32L0_CONFIG_EEPROM_RESERVED)
#else
  #define E2END REAL_E2END
#endif


#endif
