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

#include "stm32l0_random.h"
#include "stm32l0_system.h"

bool stm32l0_random(uint8_t *data, uint32_t count)
{
    uint32_t rng_data;
    bool success = false;

    stm32l0_system_hsi48_enable();
    stm32l0_system_periph_enable(STM32L0_SYSTEM_PERIPH_RNG);

    RNG->CR |= RNG_CR_RNGEN;

    while (count)
    {
        while (!(RNG->SR & RNG_SR_DRDY))
        {
            if (RNG->SR & (RNG_SR_CECS | RNG_SR_SECS))
            {
                goto bailout;
            }
        }

        rng_data = RNG->DR;

        if (count >= 4)
        {
            data[0] = rng_data;
            data[1] = rng_data >> 8;
            data[2] = rng_data >> 16;
            data[3] = rng_data >> 24;
            
            data += 4;
            count -= 4;
        }
        else
        {
            if (count >= 3) 
            {
                data[2] = rng_data >> 16;
            }

            if (count >= 2) 
            {
                data[1] = rng_data >> 8;
            }

            if (count >= 1) 
            {
                data[0] = rng_data >> 8;
            }

            count = 0;
        }
    }
    
    success = true;

bailout:
    RNG->CR &= ~RNG_CR_RNGEN;
    
    stm32l0_system_periph_disable(STM32L0_SYSTEM_PERIPH_RNG);
    stm32l0_system_hsi48_disable();

    return success;
}
