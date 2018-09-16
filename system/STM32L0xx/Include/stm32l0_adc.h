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

#if !defined(_STM32L0_ADC_H)
#define _STM32L0_ADC_H

#include "armv6m.h"
#include "stm32l0xx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STM32L0_ADC_CHANNEL_0                       0   /* PA0 */
#define STM32L0_ADC_CHANNEL_1                       1   /* PA1 */
#define STM32L0_ADC_CHANNEL_2                       2   /* PA2 */
#define STM32L0_ADC_CHANNEL_3                       3   /* PA3 */
#define STM32L0_ADC_CHANNEL_4                       4   /* PA4 */
#define STM32L0_ADC_CHANNEL_5                       5   /* PA5 */
#define STM32L0_ADC_CHANNEL_6                       6   /* PA6 */
#define STM32L0_ADC_CHANNEL_7                       7   /* PA7 */
#define STM32L0_ADC_CHANNEL_8                       8   /* PB0 */
#define STM32L0_ADC_CHANNEL_9                       9   /* PB1 */
#define STM32L0_ADC_CHANNEL_10                      10  /* PC0 */
#define STM32L0_ADC_CHANNEL_11                      11  /* PC1 */
#define STM32L0_ADC_CHANNEL_12                      12  /* PC2 */
#define STM32L0_ADC_CHANNEL_13                      13  /* PC3 */
#define STM32L0_ADC_CHANNEL_14                      14  /* PC4 */
#define STM32L0_ADC_CHANNEL_15                      15  /* PC5 */
#define STM32L0_ADC_CHANNEL_VREFINT                 17
#define STM32L0_ADC_CHANNEL_TSENSE                  18  /* TEMPERATURE SENSOR */

#define STM32L0_ADC_CHANNEL_NONE                    255

#define STM32L0_ADC_CONTROL_RIGHT_ALIGNED           0x00000000 /* 12 bit LSB */
#define STM32L0_ADC_CONTROL_LEFT_ALIGNED            0x00000001 /* 12 bit MSB */
#define STM32L0_ADC_CONTROL_BYTE_PACKED             0x00000002 /* 8 bit LSB */
#define STM32L0_ADC_CONTROL_MODE_MASK               0x0000000c
#define STM32L0_ADC_CONTROL_MODE_SHIFT              2
#define STM32L0_ADC_CONTROL_MODE_SINGLE             0x00000000 /* hardware triggered */
#define STM32L0_ADC_CONTROL_MODE_ONESHOT            0x00000004 /* software triggered */
#define STM32L0_ADC_CONTROL_MODE_CONTINUOUS_1000000 0x00000008 /* 12.5 +  3.5 @ 16MHz */
#define STM32L0_ADC_CONTROL_MODE_CONTINUOUS_500000  0x0000000c /* 12.5 + 19.5 @ 16MHz */
#define STM32L0_ADC_CONTROL_TRIG_MASK               0x00000070
#define STM32L0_ADC_CONTROL_TRIG_SHIFT              4
#define STM32L0_ADC_CONTROL_TRIG_TIM6               0x00000000
#define STM32L0_ADC_CONTROL_TRIG_TIM21_CH2          0x00000010
#define STM32L0_ADC_CONTROL_TRIG_TIM2               0x00000020
#define STM32L0_ADC_CONTROL_TRIG_TIM2_CH4           0x00000030
#define STM32L0_ADC_CONTROL_TRIG_TIM22              0x00000040
#define STM32L0_ADC_CONTROL_TRIG_TIM2_CH3           0x00000050
#define STM32L0_ADC_CONTROL_TRIG_TIM3               0x00000060
#define STM32L0_ADC_CONTROL_TRIG_EXTERNAL           0x00000070 /* EXTI_11 */
#define STM32L0_ADC_CONTROL_EDGE_MASK               0x00000180
#define STM32L0_ADC_CONTROL_EDGE_SHIFT              7
#define STM32L0_ADC_CONTROL_EDGE_NONE               0x00000000
#define STM32L0_ADC_CONTROL_EDGE_RISING             0x00000080
#define STM32L0_ADC_CONTROL_EDGE_FALLING            0x00000100
#define STM32L0_ADC_CONTROL_EDGE_BOTH               0x00000180
#define STM32L0_ADC_CONTROL_DISCONTINUOUS           0x00000200
#define STM32L0_ADC_CONTROL_NOSLEEP                 0x00000400
#define STM32L0_ADC_CONTROL_SHIFT_MASK              0x0000f000
#define STM32L0_ADC_CONTROL_SHIFT_SHIFT             12
#define STM32L0_ADC_CONTROL_SHIFT_NONE              0x00000000
#define STM32L0_ADC_CONTROL_SHIFT_1                 0x00001000
#define STM32L0_ADC_CONTROL_SHIFT_2                 0x00002000
#define STM32L0_ADC_CONTROL_SHIFT_3                 0x00003000
#define STM32L0_ADC_CONTROL_SHIFT_4                 0x00004000
#define STM32L0_ADC_CONTROL_SHIFT_5                 0x00005000
#define STM32L0_ADC_CONTROL_SHIFT_6                 0x00006000
#define STM32L0_ADC_CONTROL_SHIFT_7                 0x00007000
#define STM32L0_ADC_CONTROL_SHIFT_8                 0x00008000
#define STM32L0_ADC_CONTROL_RATIO_MASK              0x000f0000
#define STM32L0_ADC_CONTROL_RATIO_SHIFT             16
#define STM32L0_ADC_CONTROL_RATIO_1                 0x00000000
#define STM32L0_ADC_CONTROL_RATIO_2                 0x00010000
#define STM32L0_ADC_CONTROL_RATIO_4                 0x00020000
#define STM32L0_ADC_CONTROL_RATIO_8                 0x00030000
#define STM32L0_ADC_CONTROL_RATIO_16                0x00040000
#define STM32L0_ADC_CONTROL_RATIO_32                0x00050000
#define STM32L0_ADC_CONTROL_RATIO_64                0x00060000
#define STM32L0_ADC_CONTROL_RATIO_128               0x00070000
#define STM32L0_ADC_CONTROL_RATIO_256               0x00080000

typedef void (*stm32l0_adc_done_callback_t)(void *context, uint32_t count);

#define STM32L0_ADC_VREFINT_CAL                     (*((const uint16_t*)0x1ff80078))
#define STM32L0_ADC_TSENSE_CAL1                     (*((const uint16_t*)0x1ff8007a))
#define STM32L0_ADC_TSENSE_CAL2                     (*((const uint16_t*)0x1ff8007e))

#define STM32L0_ADC_VREFINT_PERIOD                  5
#define STM32L0_ADC_TSENSE_PERIOD                   10

extern bool stm32l0_adc_enable(void);
extern bool stm32l0_adc_disable(void);
extern uint32_t stm32l0_adc_read(unsigned int channel, uint16_t period);
extern bool stm32l0_adc_convert(void *data, uint32_t count, uint16_t mask, uint16_t period, uint32_t control, stm32l0_adc_done_callback_t callback, void *context);
extern void stm32l0_adc_cancel(void);
extern bool stm32l0_adc_done(void);

#ifdef __cplusplus
}
#endif

#endif /* _STM32L0_ADC_H */
