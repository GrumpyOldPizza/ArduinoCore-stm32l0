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

#if !defined(_STM32L0_SPI_H)
#define _STM32L0_SPI_H

#include "armv6m.h"
#include "stm32l0xx.h"

#include "stm32l0_dma.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    STM32L0_SPI_INSTANCE_SPI1 = 0,
    STM32L0_SPI_INSTANCE_SPI2,
    STM32L0_SPI_INSTANCE_COUNT
};

#define STM32L0_SPI_OPTION_MODE_MASK           0x00000003
#define STM32L0_SPI_OPTION_MODE_SHIFT          0
#define STM32L0_SPI_OPTION_MODE_0              0x00000000
#define STM32L0_SPI_OPTION_MODE_1              0x00000001
#define STM32L0_SPI_OPTION_MODE_2              0x00000002
#define STM32L0_SPI_OPTION_MODE_3              0x00000003
#define STM32L0_SPI_OPTION_MSB_FIRST           0x00000000
#define STM32L0_SPI_OPTION_LSB_FIRST           0x00000080
#define STM32L0_SPI_OPTION_HALFDUPLEX          0x80000000

#define STM32L0_SPI_STATE_NONE                 0
#define STM32L0_SPI_STATE_INIT                 1
#define STM32L0_SPI_STATE_READY                2
#define STM32L0_SPI_STATE_DATA                 3

typedef struct _stm32l0_spi_pins_t {
    uint16_t                    mosi;
    uint16_t                    miso;
    uint16_t                    sck;
    uint16_t                    nss;
} stm32l0_spi_pins_t;

typedef struct _stm32l0_spi_params_t {
    uint8_t                     instance;
    uint8_t                     priority;
    uint8_t                     rx_dma;
    uint8_t                     tx_dma;
    stm32l0_spi_pins_t          pins;
} stm32l0_spi_params_t;

typedef void (*stm32l0_spi_notify_callback_t)(void *context, int acquire);

typedef struct _stm32l0_spi_t {
    SPI_TypeDef                   *SPI;
    volatile uint8_t              state;
    uint8_t                       instance;
    uint8_t                       priority;
    uint8_t                       rx_dma;
    uint8_t                       tx_dma;
    uint8_t                       lock;
    stm32l0_spi_pins_t            pins;
    stm32l0_spi_notify_callback_t callback;
    void                          *context;
    uint32_t                      pclk;
    uint32_t                      clock;
    uint32_t                      option;
    uint32_t                      mask;
} stm32l0_spi_t;

extern bool stm32l0_spi_create(stm32l0_spi_t *spi, const stm32l0_spi_params_t *params);
extern bool stm32l0_spi_destroy(stm32l0_spi_t *spi);
extern bool stm32l0_spi_enable(stm32l0_spi_t *spi);
extern bool stm32l0_spi_disable(stm32l0_spi_t *spi);
extern bool stm32l0_spi_notify(stm32l0_spi_t *spi, stm32l0_spi_notify_callback_t callback, void *context);
extern bool stm32l0_spi_block(stm32l0_spi_t *spi, uint16_t pin);
extern bool stm32l0_spi_unblock(stm32l0_spi_t *spi, uint16_t pin);
extern bool stm32l0_spi_acquire(stm32l0_spi_t *spi, uint32_t clock, uint32_t option);
extern bool stm32l0_spi_release(stm32l0_spi_t *spi);
extern uint8_t stm32l0_spi_data(stm32l0_spi_t *spi, uint8_t data);
extern uint16_t stm32l0_spi_data16(stm32l0_spi_t *spi, uint16_t data);
extern uint32_t stm32l0_spi_data32(stm32l0_spi_t *spi, uint32_t data);
extern bool stm32l0_spi_receive(stm32l0_spi_t *spi, uint8_t *rx_data, uint32_t count);
extern bool stm32l0_spi_transmit(stm32l0_spi_t *spi, const uint8_t *tx_data, uint32_t count);
extern bool stm32l0_spi_transfer(stm32l0_spi_t *spi, const uint8_t *tx_data, uint8_t *rx_data, uint32_t count);

#ifdef __cplusplus
}
#endif

#endif /* _STM32L0_SPI_H */
