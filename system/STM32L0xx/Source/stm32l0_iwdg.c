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

#include "stm32l0_iwdg.h"
#include "stm32l0_system.h"


void stm32l0_iwdg_enable(uint32_t timeout)
{
    uint32_t iwdg_pr, iwdg_rl;

    /* LSI is nominal @ 37kHz, but can vary between 26kHz and 56kHz.
     * Hence be conservative on a maximum timeout.
     */
    if (timeout > 18000)
    {
        timeout = 18000;
    }

    iwdg_pr = 0;
    iwdg_rl = timeout * (stm32l0_system_lsiclk() / (4 * 1000));
    
    while (iwdg_rl > 4096)
    {
        iwdg_pr++;
        iwdg_rl >>= 1;
    }
    
    IWDG->KR = 0xcccc;
    IWDG->KR = 0x5555;

    while (IWDG->SR & (IWDG_SR_WVU | IWDG_SR_RVU | IWDG_SR_PVU))
    {
    }

    IWDG->PR = iwdg_pr;
    IWDG->RLR = iwdg_rl -1;
    IWDG->KR = 0xaaaa;
}
