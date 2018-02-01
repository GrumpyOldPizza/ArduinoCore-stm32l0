/*
 * Copyright (c) 2017-2018 Thomas Roell.  All rights reserved.
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

#include "armv6m.h"
#include "stm32l0xx.h"

#include "stm32l0_eeprom.h"

#define DATA_EEPROM_SIZE 6144

bool stm32l0_eeprom_write(uint32_t address, const uint8_t *data, uint32_t count)
{
    uint32_t primask;

    if ((address + count) > DATA_EEPROM_SIZE)
    {
        return false;
    }

    primask = __get_PRIMASK();

    __disable_irq();

    FLASH->PEKEYR  = 0x89abcdef;
    FLASH->PEKEYR  = 0x02030405;

    __set_PRIMASK(primask);

    if (address & 1)
    {
        if (count >= 1)
        {
            *((volatile uint8_t*)(DATA_EEPROM_BASE + address)) = data[0];
            
            data    += 1;
            address += 1;
            count   -= 1;
        }
    }

    if (address & 2)
    {
        if (count >= 2)
        {
            *((volatile uint16_t*)(DATA_EEPROM_BASE + address)) = ((data[1] << 8) | data[0]);

            data    += 2;
            address += 2;
            count   -= 2;
        }
    }

    while (count >= 4)
    {
        *((volatile uint32_t*)(DATA_EEPROM_BASE + address)) = ((data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]);

        data    += 4;
        address += 4;
        count   -= 4;
    }

    if (count >= 2)
    {
        *((volatile uint16_t*)(DATA_EEPROM_BASE + address)) = ((data[1] << 8) | data[0]);
        
        data    += 2;
        address += 2;
        count   -= 2;
    }

    if (count >= 1)
    {
        *((volatile uint8_t*)(DATA_EEPROM_BASE + address)) = data[0];
        
        data    += 1;
        address += 1;
        count   -= 1;
    }

    while (FLASH->SR & FLASH_SR_BSY)
    {
    }

    __disable_irq();

    FLASH->PECR |= FLASH_PECR_PELOCK;

    __set_PRIMASK(primask);

    FLASH->SR = (FLASH_SR_EOP | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_RDERR | FLASH_SR_NOTZEROERR | FLASH_SR_FWWERR);

    return true;
}

bool stm32l0_eeprom_read(uint32_t address, uint8_t *data, uint32_t count)
{
    if ((address + count) > DATA_EEPROM_SIZE)
    {
        return false;
    }

    memcpy(data, (const void*)(DATA_EEPROM_BASE + address), count);

    return true;
}
