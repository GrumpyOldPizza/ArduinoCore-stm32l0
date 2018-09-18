/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef HardwareSerial_h
#define HardwareSerial_h

#include <inttypes.h>

#include "Stream.h"

#define SERIAL_7N1	(STM32L0_UART_OPTION_DATA_SIZE_7 | STM32L0_UART_OPTION_PARITY_NONE | STM32L0_UART_OPTION_STOP_1)
#define SERIAL_8N1	(STM32L0_UART_OPTION_DATA_SIZE_8 | STM32L0_UART_OPTION_PARITY_NONE | STM32L0_UART_OPTION_STOP_1)
#define SERIAL_7N2	(STM32L0_UART_OPTION_DATA_SIZE_7 | STM32L0_UART_OPTION_PARITY_NONE | STM32L0_UART_OPTION_STOP_2)
#define SERIAL_8N2	(STM32L0_UART_OPTION_DATA_SIZE_8 | STM32L0_UART_OPTION_PARITY_NONE | STM32L0_UART_OPTION_STOP_2)
#define SERIAL_7E1	(STM32L0_UART_OPTION_DATA_SIZE_7 | STM32L0_UART_OPTION_PARITY_EVEN | STM32L0_UART_OPTION_STOP_1)
#define SERIAL_8E1	(STM32L0_UART_OPTION_DATA_SIZE_8 | STM32L0_UART_OPTION_PARITY_EVEN | STM32L0_UART_OPTION_STOP_1)
#define SERIAL_7E2	(STM32L0_UART_OPTION_DATA_SIZE_7 | STM32L0_UART_OPTION_PARITY_EVEN | STM32L0_UART_OPTION_STOP_2)
#define SERIAL_8E2	(STM32L0_UART_OPTION_DATA_SIZE_8 | STM32L0_UART_OPTION_PARITY_EVEN | STM32L0_UART_OPTION_STOP_2)
#define SERIAL_7O1	(STM32L0_UART_OPTION_DATA_SIZE_7 | STM32L0_UART_OPTION_PARITY_ODD | STM32L0_UART_OPTION_STOP_1)
#define SERIAL_8O1	(STM32L0_UART_OPTION_DATA_SIZE_8 | STM32L0_UART_OPTION_PARITY_ODD | STM32L0_UART_OPTION_STOP_1)
#define SERIAL_7O2	(STM32L0_UART_OPTION_DATA_SIZE_7 | STM32L0_UART_OPTION_PARITY_ODD | STM32L0_UART_OPTION_STOP_2)
#define SERIAL_8O2	(STM32L0_UART_OPTION_DATA_SIZE_8 | STM32L0_UART_OPTION_PARITY_ODD | STM32L0_UART_OPTION_STOP_2)

#define SERIAL_SBUS	(STM32L0_UART_OPTION_DATA_SIZE_8 | STM32L0_UART_OPTION_PARITY_EVEN | STM32L0_UART_OPTION_STOP_2 | STM32L0_UART_OPTION_RX_INVERT | STM32L0_UART_OPTION_TX_INVERT)

class HardwareSerial : public Stream
{
  public:
    virtual void begin(unsigned long);
    virtual void begin(unsigned long baudrate, uint32_t config);
    virtual void end();
    virtual int available(void) = 0;
    virtual int peek(void) = 0;
    virtual int read(void) = 0;
    virtual size_t read(uint8_t *buffer, size_t size) = 0;
    virtual void flush(void) = 0;
    virtual size_t write(uint8_t) = 0;
    using Print::write; // pull in write(str) and write(buf, size) from Print
    virtual operator bool() = 0;
};

extern void serialEventRun(void);

#endif
