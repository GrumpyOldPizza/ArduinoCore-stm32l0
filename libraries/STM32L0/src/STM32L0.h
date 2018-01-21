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

#ifndef STM32L0_H
#define STM32L0_H

#include <Arduino.h>

#define RESET_POWERON        0
#define RESET_EXTERNAL       1
#define RESET_SOFTWARE       2
#define RESET_WATCHDOG       3
#define RESET_FIREWALL       4
#define RESET_OTHER          5
#define RESET_STANDBY        6

#define FLASHSTART           ((uint32_t)(&__FlashBase))
#define FLASHEND             ((uint32_t)(&__FlashLimit))

class STM32L0Class {
public:
    uint64_t getSerial();
    void getUID(uint32_t uid[3]);

    bool  getVBUS();
    float getVBAT();
    float getVDDA();
    float getTemperature();

    uint32_t resetCause();

    void  wakeup();
    void  sleep(uint32_t timeout = 0);
    void  stop(uint32_t timeout = 0);
    void  standby();
    void  standby(uint32_t pin);
    void  reset();

    void  swdEnable();
    void  swdDisable();

    void  wdtEnable(uint32_t timeout);
    void  wdtReset();

    bool  flashErase(uint32_t address, uint32_t count);
    bool  flashProgram(uint32_t address, const void *data, uint32_t count);
};

extern STM32L0Class STM32L0;

#endif // STM32L0_H
