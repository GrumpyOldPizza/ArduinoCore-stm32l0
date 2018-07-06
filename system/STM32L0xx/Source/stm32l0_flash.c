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

#include "stm32l0_flash.h"

static void __empty() { }

void stm32l0_eeprom_acquire(void) __attribute__ ((weak, alias("__empty")));
void stm32l0_eeprom_release(void) __attribute__ ((weak, alias("__empty")));

static __attribute__((optimize("O3"), section(".ramfunc.stm32l0_flash_do_erase"), long_call)) void stm32l0_flash_do_erase(uint32_t address)
{
    *((volatile uint32_t*)address) = 0;

    __DMB();

    while (FLASH->SR & FLASH_SR_BSY)
    {
    }
}

static __attribute__((optimize("O3"), section(".ramfunc.stm32l0_flash_do_program"), long_call)) void stm32l0_flash_do_program(uint32_t address, const uint8_t *data, const uint8_t *data_e)
{
    do
    {
        *((volatile uint32_t*)address) = ((data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]);

        address += 4;
        data += 4;
    }
    while (data != data_e);

    __DMB();

    while (FLASH->SR & FLASH_SR_BSY)
    {
    }
}

uint32_t stm32l0_flash_size(void)
{
    return *((const uint16_t*)0x1ff8007c) * 1024;
}

void stm32l0_flash_lock(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();

    __disable_irq();

    FLASH->PECR |= FLASH_PECR_PELOCK;
    FLASH->PECR |= FLASH_PECR_PRGLOCK;

    __set_PRIMASK(primask);

    stm32l0_eeprom_release();
}

bool stm32l0_flash_unlock(void)
{
    uint32_t primask;

    stm32l0_eeprom_acquire();

    primask = __get_PRIMASK();

    __disable_irq();

    if (FLASH->PECR & FLASH_PECR_PELOCK)
    {
        FLASH->PEKEYR  = 0x89abcdef;
        FLASH->PEKEYR  = 0x02030405;
    }

    if (!(FLASH->PECR & FLASH_PECR_PELOCK))
    {
        if (FLASH->PECR & FLASH_PECR_PRGLOCK)
        {
            FLASH->PRGKEYR = 0x8c9daebf;
            FLASH->PRGKEYR = 0x13141516;
        }
    }

    __set_PRIMASK(primask);

    if (FLASH->PECR & FLASH_PECR_PRGLOCK)
    {
        stm32l0_eeprom_release();

        return false;
    }

    return true;
}

bool stm32l0_flash_erase(uint32_t address, uint32_t count)
{
    bool success = true;
    uint32_t primask;

    if ((address & 127) || (count & 127) || (address < FLASH_BASE) || ((address + count) > (FLASH_BASE + stm32l0_flash_size())))
    {
        return false;
    }

    if (FLASH->PECR & FLASH_PECR_PRGLOCK)
    {
        return false;
    }
    
    do
    {
        primask = __get_PRIMASK();

        __disable_irq();

        FLASH->PECR |= (FLASH_PECR_PROG | FLASH_PECR_ERASE);

        stm32l0_flash_do_erase(address);

        if (FLASH->SR & FLASH_SR_EOP)
        {
            FLASH->SR = FLASH_SR_EOP;
        }
        else
        {
            FLASH->SR = (FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_RDERR | FLASH_SR_NOTZEROERR | FLASH_SR_FWWERR);

            success = false;
        }

        FLASH->PECR &= ~(FLASH_PECR_PROG | FLASH_PECR_ERASE);

        __set_PRIMASK(primask);

        address += 128;
        count   -= 128;
    }
    while (success && count);

    return success;
}

bool stm32l0_flash_program(uint32_t address, const uint8_t *data, uint32_t count)
{
    bool success = true;
    const uint8_t *data_e;
    uint32_t primask, size;

    if ((address & 3) || (count & 3) || (address < FLASH_BASE) || ((address + count) > (FLASH_BASE + stm32l0_flash_size())))
    {
        return false;
    }

    if (FLASH->PECR & FLASH_PECR_PRGLOCK)
    {
        return false;
    }

    do
    {
        size = count;

        if (size > 64)
        {
            size = 64;
        }

        if (size > (((address + 64) & ~63) - address))
        {
            size = ((address + 64) & ~63) - address;
        }

        primask = __get_PRIMASK();

        __disable_irq();

        if (size == 64)
        {
            FLASH->PECR |= (FLASH_PECR_PROG | FLASH_PECR_FPRG);

            stm32l0_flash_do_program(address, data, data + 64);

            if (FLASH->SR & FLASH_SR_EOP)
            {
                FLASH->SR = FLASH_SR_EOP;
            }
            else
            {
                FLASH->SR = (FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_RDERR | FLASH_SR_NOTZEROERR | FLASH_SR_FWWERR);

                success = false;
            }

            data    += 64;
            address += 64;
        
            FLASH->PECR &= ~(FLASH_PECR_PROG | FLASH_PECR_FPRG);
        }
        else
        {
            FLASH->PECR |= FLASH_PECR_PROG;

            data_e = data + size;

            do 
            {
                stm32l0_flash_do_program(address, data, data + 4);

                if (FLASH->SR & FLASH_SR_EOP)
                {
                    FLASH->SR = FLASH_SR_EOP;
                }
                else
                {
                    FLASH->SR = (FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_RDERR | FLASH_SR_NOTZEROERR | FLASH_SR_FWWERR);
                    
                    success = false;
                }

                address += 4;
                data += 4;
            }
            while (success && (data != data_e));

            FLASH->PECR &= ~FLASH_PECR_PROG;
        }
    
        __set_PRIMASK(primask);

        count -= size;
    }
    while (success && count);

    return success;
}
