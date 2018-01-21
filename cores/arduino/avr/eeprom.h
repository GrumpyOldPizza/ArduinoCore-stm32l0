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

#ifndef _AVR_EEPROM_H_
#define _AVR_EEPROM_H_ 1

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t eeprom_read_byte(const uint8_t *address) __attribute__ ((pure));
uint16_t eeprom_read_word(const uint16_t *address) __attribute__ ((pure));
uint32_t eeprom_read_dword(const uint32_t *address) __attribute__ ((pure));
float eeprom_read_float(const float *address) __attribute__ ((pure));
void eeprom_read_block(void *data, const void *address, uint32_t count);
void eeprom_write_byte(uint8_t *address, uint8_t data);
void eeprom_write_word(uint16_t *address, uint16_t data);
void eeprom_write_dword(uint32_t *address, uint32_t data);
void eeprom_write_float(float *address, float data);
void eeprom_write_block(const void *data, void *address, uint32_t count);
int eeprom_is_ready(void);
#define eeprom_busy_wait() do {} while (!eeprom_is_ready())

static inline void eeprom_update_byte(uint8_t *address, uint8_t data)
{
    eeprom_write_byte(address, data);
}

static inline void eeprom_update_word(uint16_t *address, uint16_t data)
{
    eeprom_write_word(address, data);
}

static inline void eeprom_update_dword(uint32_t *address, uint32_t data)
{
    eeprom_write_dword(address, data);
}

static inline void eeprom_update_float(float *address, float data)
{
    eeprom_write_float(address, data);
}

static inline void eeprom_update_block(const void *data, void *address, uint32_t count)
{
    eeprom_write_block(data, address, count);
}

#ifdef __cplusplus
}
#endif

#endif	/* !_AVR_EEPROM_H_ */
