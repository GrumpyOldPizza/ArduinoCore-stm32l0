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

#if !defined(_STM32L0_EEPROM_H)
#define _STM32L0_EEPROM_H

#include "armv6m.h"
#include "stm32l0xx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STM32L0_EEPROM_STATUS_NONE               0
#define STM32L0_EEPROM_STATUS_BUSY               1
#define STM32L0_EEPROM_STATUS_SUCCESS            2
#define STM32L0_EEPROM_STATUS_FAIL               3

#define STM32L0_EEPROM_CONTROL_ERASE             1
#define STM32L0_EEPROM_CONTROL_PROGRAM           2
#define STM32L0_EEPROM_CONTROL_READ              3

typedef void (*stm32l0_eeprom_done_callback_t)(void *context);

typedef struct _stm32l0_eeprom_transaction_t {
    volatile uint8_t                     status;
    uint8_t                              control;
    uint16_t                             count;
    uint32_t                             address;
    uint8_t                              *data;
    stm32l0_eeprom_done_callback_t       callback;
    void                                 *context;
    struct _stm32l0_eeprom_transaction_t *next;
} stm32l0_eeprom_transaction_t;

extern void __stm32l0_eeprom_initialize(void);

extern bool stm32l0_eeprom_enqueue(stm32l0_eeprom_transaction_t *transaction);
extern void stm32l0_eeprom_acquire(void);
extern void stm32l0_eeprom_release(void);

#ifdef __cplusplus
}
#endif

#endif /* _STM32L0_EEPROM_H */
