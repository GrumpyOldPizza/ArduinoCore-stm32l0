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

#define ARDUINO_MAIN
#include "Arduino.h"
#include "wiring_private.h"

#if defined(ARDUINO_MAKEFILE)

static uint32_t table[1024];

void setup(void) {
    int i;

    for (i = 0; i < 1024; i++)
    {
	stm32l0_random((uint8_t*)&table[i], 4);
    }
}

#if 0

volatile uint32_t __t0 = 0;
volatile uint32_t __t1 = 0;

void setup(void) {
    stm32l0_eeprom_transaction_t transaction;
    uint8_t data[256];
    unsigned int i, l, n;

    for (i = 0; i < 256; i++)
    {
	data[i] = i;
    }

    transaction.status = STM32L0_EEPROM_STATUS_BUSY;
    transaction.control = STM32L0_EEPROM_CONTROL_ERASE;
    transaction.count = 256;
    transaction.address = 1024;
    transaction.data = &data[0];
    transaction.callback = NULL;
    transaction.context = NULL;

    stm32l0_eeprom_enqueue(&transaction);

    while (transaction.status == STM32L0_EEPROM_STATUS_BUSY)
    {
    }

    for (l = 0; l < 0xffffffff; l++)
    {
	data[0] = l >> 0;
	data[1] = l >> 8;
	data[2] = l >> 16;
	data[3] = l >> 24;

	transaction.status = STM32L0_EEPROM_STATUS_BUSY;
	transaction.control = STM32L0_EEPROM_CONTROL_PROGRAM;
	transaction.count = 4;
	transaction.address = 1024;
	transaction.data = &data[0];
	transaction.callback = NULL;
	transaction.context = NULL;
	
	stm32l0_eeprom_enqueue(&transaction);

	while (transaction.status == STM32L0_EEPROM_STATUS_BUSY)
	{
	}

	transaction.status = STM32L0_EEPROM_STATUS_BUSY;
	transaction.control = STM32L0_EEPROM_CONTROL_READ;
	transaction.count = 4;
	transaction.address = 1024;
	transaction.data = &data[0];
	transaction.callback = NULL;
	transaction.context = NULL;
	
	stm32l0_eeprom_enqueue(&transaction);

	while (transaction.status == STM32L0_EEPROM_STATUS_BUSY)
	{
	}

	n = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | (data[0] << 0);

	if (n != l)
	{
	    __BKPT();
	}
    }

    __t1 = millis();

    __BKPT();
}

#endif

void loop(void) { }
#if 0
void setup(void) { }
void loop(void) { }
#endif
#endif

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

// Initialize C library
extern "C" void __libc_init_array(void);

void (*g_serialEventRun)(void) = NULL;

/*
 * \brief Main entry point of Arduino application
 */
int main( void )
{
  init();
  initVariant();

  __libc_init_array();

  delay(1);
#if defined(USBCON)
  USBDevice.init();
  USBDevice.attach();
#endif

  setup();

  for (;;)
  {
    loop();
    if (g_serialEventRun) (*g_serialEventRun)();
  }

  return 0;
}
