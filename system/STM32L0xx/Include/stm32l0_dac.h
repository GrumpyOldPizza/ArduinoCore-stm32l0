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

#if !defined(_STM32L0_DAC_H)
#define _STM32L0_DAC_H

#include "armv6m.h"
#include "stm32l0xx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STM32L0_DAC_CHANNEL_1                            0x00000001
#if defined(STM32L072xx) || defined(STM32L082xx)
#define STM32L0_DAC_CHANNEL_2                            0x00000002
#endif /* STM32L072xx || STM32L082xx */

#define STM32L0_DAC_CONTROL_RIGHT_ALIGNED                0x00000000 /* 12 bit LSB */
#define STM32L0_DAC_CONTROL_LEFT_ALIGNED                 0x00000001 /* 12 bit MSB */
#define STM32L0_DAC_CONTROL_BYTE_PACKED                  0x00000002 /* 8 bit LSB */
#define STM32L0_DAC_CONTROL_MONO                         0x00000000
#if defined(STM32L072xx) || defined(STM32L082xx)
#define STM32L0_DAC_CONTROL_STEREO                       0x00000004
#endif /* STM32L072xx || STM32L082xx */
#define STM32L0_DAC_CONTROL_TRIG_MASK                    0x00000038
#define STM32L0_DAC_CONTROL_TRIG_SHIFT                   3
#define STM32L0_DAC_CONTROL_TRIG_TIM6                    0x00000000
#define STM32L0_DAC_CONTROL_TRIG_TIM3                    0x00000008
#define STM32L0_DAC_CONTROL_TRIG_TIM3_CH3                0x00000010
#define STM32L0_DAC_CONTROL_TRIG_TIM21                   0x00000018
#define STM32L0_DAC_CONTROL_TRIG_TIM2                    0x00000020
#define STM32L0_DAC_CONTROL_TRIG_TIM7                    0x00000028
#define STM32L0_DAC_CONTROL_TRIG_EXTI_9_RISING           0x00000030 /* EXTI_9 */

typedef void (*stm32l0_dac_done_callback_t)(void *context, uint32_t count);

extern bool stm32l0_dac_enable(uint32_t channels);
extern bool stm32l0_dac_disable(uint32_t channels);
extern void stm32l0_dac_write(uint32_t channels, uint32_t output);
extern bool stm32l0_dac_convert(const void *data, uint32_t count, uint32_t control, stm32l0_dac_done_callback_t callback, void *context);
extern void stm32l0_dac_cancel(void);

#ifdef __cplusplus
}
#endif

#endif /* _STM32L0_DAC_H */
