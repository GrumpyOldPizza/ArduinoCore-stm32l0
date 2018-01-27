/*
 * Copyright (c) 2014-2018 Thomas Roell.  All rights reserved.
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

#if !defined(_STM32L0_SFSPI_H)
#define _STM32L0_SFSPI_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "dosfs_device.h"
#include "dosfs_sflash.h"

#include "stm32l0_system.h"
#include "stm32l0_gpio.h"
#include "stm32l0_spi.h"

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct _stm32l0_sfspi_pins_t {
    uint16_t                     mosi;
    uint16_t                     miso;
    uint16_t                     sck;
    uint16_t                     cs;
} stm32l0_sfspi_pins_t;

typedef struct _stm32l0_sfspi_params_t {
    uint16_t                     pin_cs;
} stm32l0_sfspi_params_t;

#define STM32L0_SFSPI_STATE_NONE                   0
#define STM32L0_SFSPI_STATE_INIT                   1
#define STM32L0_SFSPI_STATE_READY                  2
#define STM32L0_SFSPI_STATE_LOCKED                 3
#define STM32L0_SFSPI_STATE_SLEEP                  4

typedef struct _stm32l0_sfspi_t {
    volatile uint8_t             state;
    uint8_t                      ID[3];
    stm32l0_spi_t                *spi;
    uint16_t                     pin_cs;
    stm32l0_system_notify_t      notify;
} stm32l0_sfspi_t;

extern bool stm32l0_sfspi_initialize(stm32l0_spi_t *spi, const stm32l0_sfspi_params_t *params);

#ifdef __cplusplus
}
#endif

#endif /*_STM32L0_SFSPI_H */
