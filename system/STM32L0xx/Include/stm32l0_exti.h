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

#if !defined(_STM32L0_EXTI_H)
#define _STM32L0_EXTI_H

#include "armv6m.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define STM32L0_EXTI_CHANNEL_COUNT              16

#define STM32L0_EXTI_CONTROL_EDGE_FALLING       0x00000001
#define STM32L0_EXTI_CONTROL_EDGE_RISING        0x00000002

typedef void (*stm32l0_exti_callback_t)(void *context);

extern void stm32l0_exti_configure(unsigned int priority);
extern bool stm32l0_exti_attach(uint16_t pin, uint32_t control, stm32l0_exti_callback_t callback, void *context);
extern void stm32l0_exti_detach(uint16_t pin);
extern bool stm32l0_exti_control(uint16_t pin, uint32_t control);
extern void stm32l0_exti_block(uint32_t mask);
extern void stm32l0_exti_unblock(uint32_t mask);

#ifdef __cplusplus
}
#endif

#endif /* _STM32L0_EXTI_H */
