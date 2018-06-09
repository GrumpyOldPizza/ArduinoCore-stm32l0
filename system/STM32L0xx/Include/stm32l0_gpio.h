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

#if !defined(_STM32L0_GPIO_H)
#define _STM32L0_GPIO_H

#include "armv6m.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define STM32L0_GPIO_MODE_MASK                  0x0003
#define STM32L0_GPIO_MODE_SHIFT                 0
#define STM32L0_GPIO_MODE_INPUT                 0x0000
#define STM32L0_GPIO_MODE_OUTPUT                0x0001
#define STM32L0_GPIO_MODE_ALTERNATE             0x0002
#define STM32L0_GPIO_MODE_ANALOG                0x0003
#define STM32L0_GPIO_OTYPE_MASK                 0x0004
#define STM32L0_GPIO_OTYPE_SHIFT                2
#define STM32L0_GPIO_OTYPE_PUSHPULL             0x0000
#define STM32L0_GPIO_OTYPE_OPENDRAIN            0x0004
#define STM32L0_GPIO_OSPEED_MASK                0x0018
#define STM32L0_GPIO_OSPEED_SHIFT               3
#define STM32L0_GPIO_OSPEED_LOW                 0x0000
#define STM32L0_GPIO_OSPEED_MEDIUM              0x0008
#define STM32L0_GPIO_OSPEED_HIGH                0x0010
#define STM32L0_GPIO_OSPEED_VERY_HIGH           0x0018
#define STM32L0_GPIO_PUPD_MASK                  0x0060
#define STM32L0_GPIO_PUPD_SHIFT                 5
#define STM32L0_GPIO_PUPD_NONE                  0x0000
#define STM32L0_GPIO_PUPD_PULLUP                0x0020
#define STM32L0_GPIO_PUPD_PULLDOWN              0x0040
#define STM32L0_GPIO_PARK_SHIFT                 7
#define STM32L0_GPIO_PARK_MASK                  0x0180
#define STM32L0_GPIO_PARK_NONE                  0x0000
#define STM32L0_GPIO_PARK_PULLUP                0x0080
#define STM32L0_GPIO_PARK_PULLDOWN              0x0100
#define STM32L0_GPIO_PARK_HIZ                   0x0180

#if defined(STM32L052xx)

#define STM32L0_GPIO_PIN_PA0                    0x00
#define STM32L0_GPIO_PIN_PA1                    0x01
#define STM32L0_GPIO_PIN_PA2                    0x02
#define STM32L0_GPIO_PIN_PA3                    0x03
#define STM32L0_GPIO_PIN_PA4                    0x04
#define STM32L0_GPIO_PIN_PA5                    0x05
#define STM32L0_GPIO_PIN_PA6                    0x06
#define STM32L0_GPIO_PIN_PA7                    0x07
#define STM32L0_GPIO_PIN_PA8                    0x08
#define STM32L0_GPIO_PIN_PA9                    0x09
#define STM32L0_GPIO_PIN_PA10                   0x0a
#define STM32L0_GPIO_PIN_PA11                   0x0b
#define STM32L0_GPIO_PIN_PA12                   0x0c
#define STM32L0_GPIO_PIN_PA13                   0x0d
#define STM32L0_GPIO_PIN_PA14                   0x0e
#define STM32L0_GPIO_PIN_PA15                   0x0f
#define STM32L0_GPIO_PIN_PB0                    0x10
#define STM32L0_GPIO_PIN_PB1                    0x11
#define STM32L0_GPIO_PIN_PB2                    0x12
#define STM32L0_GPIO_PIN_PB3                    0x13
#define STM32L0_GPIO_PIN_PB4                    0x14
#define STM32L0_GPIO_PIN_PB5                    0x15
#define STM32L0_GPIO_PIN_PB6                    0x16
#define STM32L0_GPIO_PIN_PB7                    0x17
#define STM32L0_GPIO_PIN_PB8                    0x18
#define STM32L0_GPIO_PIN_PB9                    0x19
#define STM32L0_GPIO_PIN_PB10                   0x1a
#define STM32L0_GPIO_PIN_PB11                   0x1b
#define STM32L0_GPIO_PIN_PB12                   0x1c
#define STM32L0_GPIO_PIN_PB13                   0x1d
#define STM32L0_GPIO_PIN_PB14                   0x1e
#define STM32L0_GPIO_PIN_PB15                   0x1f
#define STM32L0_GPIO_PIN_PC0                    0x20
#define STM32L0_GPIO_PIN_PC1                    0x21
#define STM32L0_GPIO_PIN_PC2                    0x22
#define STM32L0_GPIO_PIN_PC3                    0x23
#define STM32L0_GPIO_PIN_PC4                    0x24
#define STM32L0_GPIO_PIN_PC5                    0x25
#define STM32L0_GPIO_PIN_PC6                    0x26
#define STM32L0_GPIO_PIN_PC7                    0x27
#define STM32L0_GPIO_PIN_PC8                    0x28
#define STM32L0_GPIO_PIN_PC9                    0x29
#define STM32L0_GPIO_PIN_PC10                   0x2a
#define STM32L0_GPIO_PIN_PC11                   0x2b
#define STM32L0_GPIO_PIN_PC12                   0x2c
#define STM32L0_GPIO_PIN_PC13                   0x2d
#define STM32L0_GPIO_PIN_PC14                   0x2e
#define STM32L0_GPIO_PIN_PC15                   0x2f
#define STM32L0_GPIO_PIN_PD2                    0x32
#define STM32L0_GPIO_PIN_PH0                    0x70
#define STM32L0_GPIO_PIN_PH1                    0x71
#define STM32L0_GPIO_PIN_NONE                   0xff

#endif /* STM32L052xx */

#if defined(STM32L072xx)

#define STM32L0_GPIO_PIN_PA0                    0x00
#define STM32L0_GPIO_PIN_PA1                    0x01
#define STM32L0_GPIO_PIN_PA2                    0x02
#define STM32L0_GPIO_PIN_PA3                    0x03
#define STM32L0_GPIO_PIN_PA4                    0x04
#define STM32L0_GPIO_PIN_PA5                    0x05
#define STM32L0_GPIO_PIN_PA6                    0x06
#define STM32L0_GPIO_PIN_PA7                    0x07
#define STM32L0_GPIO_PIN_PA8                    0x08
#define STM32L0_GPIO_PIN_PA9                    0x09
#define STM32L0_GPIO_PIN_PA10                   0x0a
#define STM32L0_GPIO_PIN_PA11                   0x0b
#define STM32L0_GPIO_PIN_PA12                   0x0c
#define STM32L0_GPIO_PIN_PA13                   0x0d
#define STM32L0_GPIO_PIN_PA14                   0x0e
#define STM32L0_GPIO_PIN_PA15                   0x0f
#define STM32L0_GPIO_PIN_PB0                    0x10
#define STM32L0_GPIO_PIN_PB1                    0x11
#define STM32L0_GPIO_PIN_PB2                    0x12
#define STM32L0_GPIO_PIN_PB3                    0x13
#define STM32L0_GPIO_PIN_PB4                    0x14
#define STM32L0_GPIO_PIN_PB5                    0x15
#define STM32L0_GPIO_PIN_PB6                    0x16
#define STM32L0_GPIO_PIN_PB7                    0x17
#define STM32L0_GPIO_PIN_PB8                    0x18
#define STM32L0_GPIO_PIN_PB9                    0x19
#define STM32L0_GPIO_PIN_PB10                   0x1a
#define STM32L0_GPIO_PIN_PB11                   0x1b
#define STM32L0_GPIO_PIN_PB12                   0x1c
#define STM32L0_GPIO_PIN_PB13                   0x1d
#define STM32L0_GPIO_PIN_PB14                   0x1e
#define STM32L0_GPIO_PIN_PB15                   0x1f
#define STM32L0_GPIO_PIN_PC0                    0x20
#define STM32L0_GPIO_PIN_PC1                    0x21
#define STM32L0_GPIO_PIN_PC2                    0x22
#define STM32L0_GPIO_PIN_PC3                    0x23
#define STM32L0_GPIO_PIN_PC4                    0x24
#define STM32L0_GPIO_PIN_PC5                    0x25
#define STM32L0_GPIO_PIN_PC6                    0x26
#define STM32L0_GPIO_PIN_PC7                    0x27
#define STM32L0_GPIO_PIN_PC8                    0x28
#define STM32L0_GPIO_PIN_PC9                    0x29
#define STM32L0_GPIO_PIN_PC10                   0x2a
#define STM32L0_GPIO_PIN_PC11                   0x2b
#define STM32L0_GPIO_PIN_PC12                   0x2c
#define STM32L0_GPIO_PIN_PC13                   0x2d
#define STM32L0_GPIO_PIN_PC14                   0x2e
#define STM32L0_GPIO_PIN_PC15                   0x2f
#define STM32L0_GPIO_PIN_PD0                    0x30
#define STM32L0_GPIO_PIN_PD1                    0x31
#define STM32L0_GPIO_PIN_PD2                    0x32
#define STM32L0_GPIO_PIN_PD3                    0x33
#define STM32L0_GPIO_PIN_PD4                    0x34
#define STM32L0_GPIO_PIN_PD5                    0x35
#define STM32L0_GPIO_PIN_PD6                    0x36
#define STM32L0_GPIO_PIN_PD7                    0x37
#define STM32L0_GPIO_PIN_PD8                    0x38
#define STM32L0_GPIO_PIN_PD9                    0x39
#define STM32L0_GPIO_PIN_PD10                   0x3a
#define STM32L0_GPIO_PIN_PD11                   0x3b
#define STM32L0_GPIO_PIN_PD12                   0x3c
#define STM32L0_GPIO_PIN_PD13                   0x3d
#define STM32L0_GPIO_PIN_PD14                   0x3e
#define STM32L0_GPIO_PIN_PD15                   0x3f
#define STM32L0_GPIO_PIN_PE0                    0x40
#define STM32L0_GPIO_PIN_PE1                    0x41
#define STM32L0_GPIO_PIN_PE2                    0x42
#define STM32L0_GPIO_PIN_PE3                    0x43
#define STM32L0_GPIO_PIN_PE4                    0x44
#define STM32L0_GPIO_PIN_PE5                    0x45
#define STM32L0_GPIO_PIN_PE6                    0x46
#define STM32L0_GPIO_PIN_PE7                    0x47
#define STM32L0_GPIO_PIN_PE8                    0x48
#define STM32L0_GPIO_PIN_PE9                    0x49
#define STM32L0_GPIO_PIN_PE10                   0x4a
#define STM32L0_GPIO_PIN_PE11                   0x4b
#define STM32L0_GPIO_PIN_PE12                   0x4c
#define STM32L0_GPIO_PIN_PE13                   0x4d
#define STM32L0_GPIO_PIN_PE14                   0x4e
#define STM32L0_GPIO_PIN_PE15                   0x4f
#define STM32L0_GPIO_PIN_PH0                    0x70
#define STM32L0_GPIO_PIN_PH1                    0x71
#define STM32L0_GPIO_PIN_NONE                   0xff

#endif /* STM32L072xx */

#if defined(STM32L082xx)

#define STM32L0_GPIO_PIN_PA0                    0x00
#define STM32L0_GPIO_PIN_PA1                    0x01
#define STM32L0_GPIO_PIN_PA2                    0x02
#define STM32L0_GPIO_PIN_PA3                    0x03
#define STM32L0_GPIO_PIN_PA4                    0x04
#define STM32L0_GPIO_PIN_PA5                    0x05
#define STM32L0_GPIO_PIN_PA6                    0x06
#define STM32L0_GPIO_PIN_PA7                    0x07
#define STM32L0_GPIO_PIN_PA8                    0x08
#define STM32L0_GPIO_PIN_PA9                    0x09
#define STM32L0_GPIO_PIN_PA10                   0x0a
#define STM32L0_GPIO_PIN_PA11                   0x0b
#define STM32L0_GPIO_PIN_PA12                   0x0c
#define STM32L0_GPIO_PIN_PA13                   0x0d
#define STM32L0_GPIO_PIN_PA14                   0x0e
#define STM32L0_GPIO_PIN_PA15                   0x0f
#define STM32L0_GPIO_PIN_PB0                    0x10
#define STM32L0_GPIO_PIN_PB1                    0x11
#define STM32L0_GPIO_PIN_PB2                    0x12
#define STM32L0_GPIO_PIN_PB3                    0x13
#define STM32L0_GPIO_PIN_PB4                    0x14
#define STM32L0_GPIO_PIN_PB5                    0x15
#define STM32L0_GPIO_PIN_PB6                    0x16
#define STM32L0_GPIO_PIN_PB7                    0x17
#define STM32L0_GPIO_PIN_PB8                    0x18
#define STM32L0_GPIO_PIN_PB9                    0x19
#define STM32L0_GPIO_PIN_PB10                   0x1a
#define STM32L0_GPIO_PIN_PB11                   0x1b
#define STM32L0_GPIO_PIN_PB12                   0x1c
#define STM32L0_GPIO_PIN_PB13                   0x1d
#define STM32L0_GPIO_PIN_PB14                   0x1e
#define STM32L0_GPIO_PIN_PB15                   0x1f
#define STM32L0_GPIO_PIN_PC0                    0x20
#define STM32L0_GPIO_PIN_PC1                    0x21
#define STM32L0_GPIO_PIN_PC2                    0x22
#define STM32L0_GPIO_PIN_PC13                   0x2d
#define STM32L0_GPIO_PIN_PC14                   0x2e
#define STM32L0_GPIO_PIN_PC15                   0x2f
#define STM32L0_GPIO_PIN_PH0                    0x70
#define STM32L0_GPIO_PIN_PH1                    0x71
#define STM32L0_GPIO_PIN_NONE                   0xff

#endif /* STM32L082xx */

#define STM32L0_GPIO_PIN_INDEX_MASK             0x000f
#define STM32L0_GPIO_PIN_INDEX_SHIFT            0
#define STM32L0_GPIO_PIN_GROUP_MASK             0x00f0
#define STM32L0_GPIO_PIN_GROUP_SHIFT            4
#define STM32L0_GPIO_PIN_AFSEL_MASK             0x0700
#define STM32L0_GPIO_PIN_AFSEL_SHIFT            8

#define STM32L0_GPIO_PIN_IO_MASK                0x00ff
#define STM32L0_GPIO_PIN_IO_SHIFT               0

#if defined(STM32L052xx)

#define STM32L0_GPIO_PIN_PA8_MCO                0x0008
#define STM32L0_GPIO_PIN_PA9_MCO                0x0009
#define STM32L0_GPIO_PIN_PA13_SWDIO             0x000d
#define STM32L0_GPIO_PIN_PA14_SWCLK             0x000e
#define STM32L0_GPIO_PIN_PB14_RTC_OUT           0x021e
#define STM32L0_GPIO_PIN_PB15_RTC_REFIN         0x021f

#define STM32L0_GPIO_PIN_PA8_USART1_CK          0x0408
#define STM32L0_GPIO_PIN_PA9_USART1_TX          0x0409
#define STM32L0_GPIO_PIN_PA10_USART1_RX         0x040a
#define STM32L0_GPIO_PIN_PA11_USART1_CTS        0x040b
#define STM32L0_GPIO_PIN_PA12_USART1_RTS_DE     0x040c
#define STM32L0_GPIO_PIN_PB3_USART1_RTS_DE      0x0513
#define STM32L0_GPIO_PIN_PB4_USART1_CTS         0x0514
#define STM32L0_GPIO_PIN_PB6_USART1_TX          0x0016
#define STM32L0_GPIO_PIN_PB7_USART1_RX          0x0017

#define STM32L0_GPIO_PIN_PA0_USART2_CTS         0x0400
#define STM32L0_GPIO_PIN_PA1_USART2_RTS_DE      0x0401
#define STM32L0_GPIO_PIN_PA2_USART2_TX          0x0402
#define STM32L0_GPIO_PIN_PA3_USART2_RX          0x0403
#define STM32L0_GPIO_PIN_PA4_USART2_CK          0x0404
#define STM32L0_GPIO_PIN_PA14_USART2_TX         0x040e
#define STM32L0_GPIO_PIN_PA15_USART2_RX         0x040f

#define STM32L0_GPIO_PIN_PA6_LPUART1_CTS        0x0406
#define STM32L0_GPIO_PIN_PB1_LPUART1_RTS_DE     0x0411
#define STM32L0_GPIO_PIN_PB10_LPUART1_TX        0x041a
#define STM32L0_GPIO_PIN_PB11_LPUART1_RX        0x041b
#define STM32L0_GPIO_PIN_PB12_LPUART1_RTS_DE    0x021c
#define STM32L0_GPIO_PIN_PB13_LPUART1_CTS       0x041d
#define STM32L0_GPIO_PIN_PB14_LPUART1_RTS_DE    0x041e
#define STM32L0_GPIO_PIN_PC4_LPUART1_TX         0x0224
#define STM32L0_GPIO_PIN_PC5_LPUART1_RX         0x0225
#define STM32L0_GPIO_PIN_PC10_LPUART1_TX        0x002a
#define STM32L0_GPIO_PIN_PC11_LPUART1_RX        0x002b
#define STM32L0_GPIO_PIN_PD2_LPUART1_RTS_DE     0x0032

#define STM32L0_GPIO_PIN_PB5_I2C1_SMBA          0x0315
#define STM32L0_GPIO_PIN_PB6_I2C1_SCL           0x0116
#define STM32L0_GPIO_PIN_PB7_I2C1_SDA           0x0117
#define STM32L0_GPIO_PIN_PB8_I2C1_SCL           0x0418
#define STM32L0_GPIO_PIN_PB9_I2C1_SDA           0x0419

#define STM32L0_GPIO_PIN_PB10_I2C2_SCL          0x061a
#define STM32L0_GPIO_PIN_PB11_I2C2_SDA          0x061b
#define STM32L0_GPIO_PIN_PB12_I2C2_SMBA         0x051c
#define STM32L0_GPIO_PIN_PB13_I2C2_SCL          0x051d
#define STM32L0_GPIO_PIN_PB14_I2C2_SDA          0x051e

#define STM32L0_GPIO_PIN_PA4_SPI1_NSS           0x0004
#define STM32L0_GPIO_PIN_PA5_SPI1_SCK           0x0005
#define STM32L0_GPIO_PIN_PA6_SPI1_MISO          0x0006
#define STM32L0_GPIO_PIN_PA7_SPI1_MOSI          0x0007
#define STM32L0_GPIO_PIN_PA11_SPI1_MISO         0x000b
#define STM32L0_GPIO_PIN_PA12_SPI1_MOSI         0x000c
#define STM32L0_GPIO_PIN_PA15_SPI1_NSS          0x000f
#define STM32L0_GPIO_PIN_PB3_SPI1_SCK           0x0013
#define STM32L0_GPIO_PIN_PB4_SPI1_MISO          0x0014
#define STM32L0_GPIO_PIN_PB5_SPI1_MOSI          0x0015

#define STM32L0_GPIO_PIN_PB9_SPI2_NSS           0x0519
#define STM32L0_GPIO_PIN_PB10_SPI2_SCK          0x051a
#define STM32L0_GPIO_PIN_PB12_SPI2_NSS          0x001c
#define STM32L0_GPIO_PIN_PB13_SPI2_SCK          0x001d
#define STM32L0_GPIO_PIN_PB14_SPI2_MISO         0x001e
#define STM32L0_GPIO_PIN_PB15_SPI2_MOSI         0x001f
#define STM32L0_GPIO_PIN_PC2_SPI2_MISO          0x0222
#define STM32L0_GPIO_PIN_PC3_SPI2_MOSI          0x0223

#define STM32L0_GPIO_PIN_PB9_I2S2_WS            0x0519
#define STM32L0_GPIO_PIN_PB12_I2S2_WS           0x001c
#define STM32L0_GPIO_PIN_PB13_I2S2_CK           0x001d
#define STM32L0_GPIO_PIN_PB14_I2S2_MCK          0x001e
#define STM32L0_GPIO_PIN_PB15_I2S2_SD           0x001f
#define STM32L0_GPIO_PIN_PC2_I2S2_MCK           0x0222
#define STM32L0_GPIO_PIN_PC3_I2S2_SD            0x0223

#define STM32L0_GPIO_PIN_PA8_USB_CRS_STBC       0x0208
#define STM32L0_GPIO_PIN_PA13_USB_OE            0x020d
#define STM32L0_GPIO_PIN_PH0_USB_CRS_SYNC       0x0070
#define STM32L0_GPIO_PIN_PC9_USB_NOE            0x0229

#define STM32L0_GPIO_PIN_PA0_TIM2_CH1           0x0200
#define STM32L0_GPIO_PIN_PA0_TIM2_ETR           0x0500
#define STM32L0_GPIO_PIN_PA1_TIM2_CH2           0x0201
#define STM32L0_GPIO_PIN_PA2_TIM2_CH3           0x0202
#define STM32L0_GPIO_PIN_PA3_TIM2_CH4           0x0203
#define STM32L0_GPIO_PIN_PA5_TIM2_CH1           0x0505
#define STM32L0_GPIO_PIN_PA5_TIM2_ETR           0x0205
#define STM32L0_GPIO_PIN_PA15_TIM2_CH1          0x050f
#define STM32L0_GPIO_PIN_PA15_TIM2_ETR          0x020f
#define STM32L0_GPIO_PIN_PB3_TIM2_CH2           0x0213
#define STM32L0_GPIO_PIN_PB10_TIM2_CH3          0x021a
#define STM32L0_GPIO_PIN_PB11_TIM2_CH4          0x021b

#define STM32L0_GPIO_PIN_PA1_TIM21_ETR          0x0501
#define STM32L0_GPIO_PIN_PA2_TIM21_CH1          0x0002
#define STM32L0_GPIO_PIN_PA3_TIM21_CH2          0x0003
#define STM32L0_GPIO_PIN_PB13_TIM21_CH1         0x061d
#define STM32L0_GPIO_PIN_PB14_TIM21_CH2         0x061e
#define STM32L0_GPIO_PIN_PC9_TIM21_ETR          0x0029

#define STM32L0_GPIO_PIN_PA4_TIM22_ETR          0x0504
#define STM32L0_GPIO_PIN_PA6_TIM22_CH1          0x0506
#define STM32L0_GPIO_PIN_PA7_TIM22_CH2          0x0507
#define STM32L0_GPIO_PIN_PB4_TIM22_CH1          0x0414
#define STM32L0_GPIO_PIN_PB5_TIM22_CH2          0x0415
#define STM32L0_GPIO_PIN_PC6_TIM22_CH1          0x0026
#define STM32L0_GPIO_PIN_PC7_TIM22_CH2          0x0027
#define STM32L0_GPIO_PIN_PC8_TIM22_ETR          0x0028

#define STM32L0_GPIO_PIN_PB2_LPTIM1_OUT         0x0212
#define STM32L0_GPIO_PIN_PB5_LPTIM1_IN1         0x0215
#define STM32L0_GPIO_PIN_PB6_LPTIM1_ETR         0x0216
#define STM32L0_GPIO_PIN_PB7_LPTIM1_IN2         0x0217
#define STM32L0_GPIO_PIN_PC0_LPTIM1_IN1         0x0020
#define STM32L0_GPIO_PIN_PC1_LPTIM1_OUT         0x0021
#define STM32L0_GPIO_PIN_PC2_LPTIM1_IN2         0x0022
#define STM32L0_GPIO_PIN_PC3_LPTIM1_ETR         0x0023

#define STM32L0_GPIO_PIN_PA0_COMP1_OUT          0x0700
#define STM32L0_GPIO_PIN_PA6_COMP1_OUT          0x0706
#define STM32L0_GPIO_PIN_PA11_COMP1_OUT         0x070b

#define STM32L0_GPIO_PIN_PA2_COMP2_OUT          0x0702
#define STM32L0_GPIO_PIN_PA7_COMP2_OUT          0x0707
#define STM32L0_GPIO_PIN_PA12_COMP2_OUT         0x070c

#define STM32L0_GPIO_PIN_PA0_TSC_G1_IO1         0x0300
#define STM32L0_GPIO_PIN_PA1_TSC_G1_IO2         0x0301
#define STM32L0_GPIO_PIN_PA2_TSC_G1_IO3         0x0302
#define STM32L0_GPIO_PIN_PA3_TSC_G1_IO4         0x0303
#define STM32L0_GPIO_PIN_PA4_TSC_G2_IO1         0x0304
#define STM32L0_GPIO_PIN_PA5_TSC_G2_IO2         0x0305
#define STM32L0_GPIO_PIN_PA6_TSC_G2_IO3         0x0306
#define STM32L0_GPIO_PIN_PA7_TSC_G2_IO4         0x0307
#define STM32L0_GPIO_PIN_PA9_TSC_G4_IO1         0x0309
#define STM32L0_GPIO_PIN_PA10_TSC_G4_IO2        0x030a
#define STM32L0_GPIO_PIN_PA11_TSC_G4_IO3        0x030b
#define STM32L0_GPIO_PIN_PA12_TSC_G4_IO4        0x030c
#define STM32L0_GPIO_PIN_PB0_TSC_G3_IO2         0x0310
#define STM32L0_GPIO_PIN_PB1_TSC_G3_IO3         0x0311
#define STM32L0_GPIO_PIN_PB2_TSC_G3_IO4         0x0312
#define STM32L0_GPIO_PIN_PB3_TSC_G5_IO1         0x0313
#define STM32L0_GPIO_PIN_PB4_TSC_G5_IO2         0x0314
#define STM32L0_GPIO_PIN_PB6_TSC_G5_IO3         0x0316
#define STM32L0_GPIO_PIN_PB7_TSC_G5_IO4         0x0317
#define STM32L0_GPIO_PIN_PB8_TSC_SYNC           0x0318
#define STM32L0_GPIO_PIN_PB10_TSC_SYNC          0x031a
#define STM32L0_GPIO_PIN_PB11_TSC_G6_IO1        0x031b
#define STM32L0_GPIO_PIN_PB12_TSC_G6_IO2        0x031c
#define STM32L0_GPIO_PIN_PB13_TSC_G6_IO3        0x031d
#define STM32L0_GPIO_PIN_PB14_TSC_G6_IO4        0x031e
#define STM32L0_GPIO_PIN_PC0_TSC_G7_IO1         0x0320
#define STM32L0_GPIO_PIN_PC1_TSC_G7_IO2         0x0321
#define STM32L0_GPIO_PIN_PC2_TSC_G7_IO3         0x0322
#define STM32L0_GPIO_PIN_PC3_TSC_G7_IO4         0x0323
#define STM32L0_GPIO_PIN_PC5_TSC_G3_IO1         0x0325
#define STM32L0_GPIO_PIN_PC6_TSC_G8_IO1         0x0326
#define STM32L0_GPIO_PIN_PC7_TSC_G8_IO2         0x0327
#define STM32L0_GPIO_PIN_PC8_TSC_G8_IO3         0x0328
#define STM32L0_GPIO_PIN_PC9_TSC_G8_IO4         0x0329

#endif /* STM32L052xx */

#if defined(STM32L072xx)

#define STM32L0_GPIO_PIN_PA8_MCO                0x0008
#define STM32L0_GPIO_PIN_PA9_MCO                0x0009
#define STM32L0_GPIO_PIN_PA13_SWDIO             0x000d
#define STM32L0_GPIO_PIN_PA14_SWCLK             0x000e
#define STM32L0_GPIO_PIN_PB13_MCO               0x021d
#define STM32L0_GPIO_PIN_PB14_RTC_OUT           0x021e
#define STM32L0_GPIO_PIN_PB15_RTC_REFIN         0x021f

#define STM32L0_GPIO_PIN_PA8_USART1_CK          0x0408
#define STM32L0_GPIO_PIN_PA9_USART1_TX          0x0409
#define STM32L0_GPIO_PIN_PA10_USART1_RX         0x040a
#define STM32L0_GPIO_PIN_PA11_USART1_CTS        0x040b
#define STM32L0_GPIO_PIN_PA12_USART1_RTS_DE     0x040c
#define STM32L0_GPIO_PIN_PB3_USART1_RTS_DE      0x0513
#define STM32L0_GPIO_PIN_PB4_USART1_CTS         0x0514
#define STM32L0_GPIO_PIN_PB5_USART1_CK          0x0515
#define STM32L0_GPIO_PIN_PB6_USART1_TX          0x0016
#define STM32L0_GPIO_PIN_PB7_USART1_RX          0x0017

#define STM32L0_GPIO_PIN_PA0_USART2_CTS         0x0400
#define STM32L0_GPIO_PIN_PA1_USART2_RTS_DE      0x0401
#define STM32L0_GPIO_PIN_PA2_USART2_TX          0x0402
#define STM32L0_GPIO_PIN_PA3_USART2_RX          0x0403
#define STM32L0_GPIO_PIN_PA4_USART2_CK          0x0404
#define STM32L0_GPIO_PIN_PA14_USART2_TX         0x040e
#define STM32L0_GPIO_PIN_PA15_USART2_RX         0x040f
#define STM32L0_GPIO_PIN_PD3_USART2_CTS         0x0033
#define STM32L0_GPIO_PIN_PD4_USART2_RTS_DE      0x0034
#define STM32L0_GPIO_PIN_PD5_USART2_TX          0x0035
#define STM32L0_GPIO_PIN_PD6_USART2_RX          0x0036
#define STM32L0_GPIO_PIN_PD7_USART2_CK          0x0037

#define STM32L0_GPIO_PIN_PA0_USART4_TX          0x0600
#define STM32L0_GPIO_PIN_PA1_USART4_RX          0x0601
#define STM32L0_GPIO_PIN_PA15_USART4_RTS_DE     0x060f
#define STM32L0_GPIO_PIN_PB7_USART4_CTS         0x0617
#define STM32L0_GPIO_PIN_PC10_USART4_TX         0x062a
#define STM32L0_GPIO_PIN_PC11_USART4_RX         0x062b
#define STM32L0_GPIO_PIN_PC12_USART4_CK         0x062c
#define STM32L0_GPIO_PIN_PE8_USART4_TX          0x0648
#define STM32L0_GPIO_PIN_PE9_USART4_RX          0x0649

#define STM32L0_GPIO_PIN_PB3_USART5_TX          0x0613
#define STM32L0_GPIO_PIN_PB4_USART5_RX          0x0614
#define STM32L0_GPIO_PIN_PB5_USART5_CK          0x0615
#define STM32L0_GPIO_PIN_PB5_USART5_RTS_DE      0x0615
#define STM32L0_GPIO_PIN_PC12_USART5_TX         0x022c
#define STM32L0_GPIO_PIN_PD2_USART5_RX          0x0632
#define STM32L0_GPIO_PIN_PE7_USART5_CK          0x0647
#define STM32L0_GPIO_PIN_PE7_USART5_RTS_DE      0x0647
#define STM32L0_GPIO_PIN_PE10_USART5_TX         0x064a
#define STM32L0_GPIO_PIN_PE11_USART5_RX         0x064b

#define STM32L0_GPIO_PIN_PA2_LPUART1_TX         0x0602
#define STM32L0_GPIO_PIN_PA3_LPUART1_RX         0x0603
#define STM32L0_GPIO_PIN_PA6_LPUART1_CTS        0x0406
#define STM32L0_GPIO_PIN_PA13_LPUART1_RX        0x060d
#define STM32L0_GPIO_PIN_PA14_LPUART1_TX        0x060e
#define STM32L0_GPIO_PIN_PB1_LPUART1_RTS_DE     0x0411
#define STM32L0_GPIO_PIN_PB10_LPUART1_TX        0x041a
#define STM32L0_GPIO_PIN_PB11_LPUART1_RX        0x041b
#define STM32L0_GPIO_PIN_PB12_LPUART1_RTS_DE    0x021c
#define STM32L0_GPIO_PIN_PB13_LPUART1_CTS       0x041d
#define STM32L0_GPIO_PIN_PB14_LPUART1_RTS_DE    0x041e
#define STM32L0_GPIO_PIN_PC0_LPUART1_RX         0x0620
#define STM32L0_GPIO_PIN_PC1_LPUART1_TX         0x0621
#define STM32L0_GPIO_PIN_PC4_LPUART1_TX         0x0224
#define STM32L0_GPIO_PIN_PC5_LPUART1_RX         0x0225
#define STM32L0_GPIO_PIN_PC10_LPUART1_TX        0x002a
#define STM32L0_GPIO_PIN_PC11_LPUART1_RX        0x002b
#define STM32L0_GPIO_PIN_PD2_LPUART1_RTS_DE     0x0032
#define STM32L0_GPIO_PIN_PD8_LPUART1_TX         0x0038
#define STM32L0_GPIO_PIN_PD9_LPUART1_RX         0x0039
#define STM32L0_GPIO_PIN_PD11_LPUART1_CTS       0x003b
#define STM32L0_GPIO_PIN_PD12_LPUART1_RTS_DE    0x003c

#define STM32L0_GPIO_PIN_PA9_I2C1_SCL           0x0609
#define STM32L0_GPIO_PIN_PA10_I2C1_SDA          0x060a
#define STM32L0_GPIO_PIN_PB5_I2C1_SMBA          0x0315
#define STM32L0_GPIO_PIN_PB6_I2C1_SCL           0x0116
#define STM32L0_GPIO_PIN_PB7_I2C1_SDA           0x0117
#define STM32L0_GPIO_PIN_PB8_I2C1_SCL           0x0418
#define STM32L0_GPIO_PIN_PB9_I2C1_SDA           0x0419

#define STM32L0_GPIO_PIN_PB10_I2C2_SCL          0x061a
#define STM32L0_GPIO_PIN_PB11_I2C2_SDA          0x061b
#define STM32L0_GPIO_PIN_PB12_I2C2_SMBA         0x051c
#define STM32L0_GPIO_PIN_PB13_I2C2_SCL          0x051d
#define STM32L0_GPIO_PIN_PB14_I2C2_SDA          0x051e

#define STM32L0_GPIO_PIN_PA8_I2C3_SCL           0x0708
#define STM32L0_GPIO_PIN_PA9_I2C3_SMBA          0x0709
#define STM32L0_GPIO_PIN_PB2_I2C3_SMBA          0x0712
#define STM32L0_GPIO_PIN_PB4_I2C3_SDA           0x0714
#define STM32L0_GPIO_PIN_PC0_I2C3_SCL           0x0720
#define STM32L0_GPIO_PIN_PC1_I2C3_SDA           0x0721
#define STM32L0_GPIO_PIN_PC9_I2C3_SDA           0x0729

#define STM32L0_GPIO_PIN_PA4_SPI1_NSS           0x0004
#define STM32L0_GPIO_PIN_PA5_SPI1_SCK           0x0005
#define STM32L0_GPIO_PIN_PA6_SPI1_MISO          0x0006
#define STM32L0_GPIO_PIN_PA7_SPI1_MOSI          0x0007
#define STM32L0_GPIO_PIN_PA11_SPI1_MISO         0x000b
#define STM32L0_GPIO_PIN_PA12_SPI1_MOSI         0x000c
#define STM32L0_GPIO_PIN_PA15_SPI1_NSS          0x000f
#define STM32L0_GPIO_PIN_PB3_SPI1_SCK           0x0013
#define STM32L0_GPIO_PIN_PB4_SPI1_MISO          0x0014
#define STM32L0_GPIO_PIN_PB5_SPI1_MOSI          0x0015
#define STM32L0_GPIO_PIN_PE12_SPI1_NSS          0x024c
#define STM32L0_GPIO_PIN_PE13_SPI1_SCK          0x024d
#define STM32L0_GPIO_PIN_PE14_SPI1_MISO         0x024e
#define STM32L0_GPIO_PIN_PE15_SPI1_MOSI         0x024f

#define STM32L0_GPIO_PIN_PB9_SPI2_NSS           0x0519
#define STM32L0_GPIO_PIN_PB10_SPI2_SCK          0x051a
#define STM32L0_GPIO_PIN_PB12_SPI2_NSS          0x001c
#define STM32L0_GPIO_PIN_PB13_SPI2_SCK          0x001d
#define STM32L0_GPIO_PIN_PB14_SPI2_MISO         0x001e
#define STM32L0_GPIO_PIN_PB15_SPI2_MOSI         0x001f
#define STM32L0_GPIO_PIN_PC2_SPI2_MISO          0x0222
#define STM32L0_GPIO_PIN_PC3_SPI2_MOSI          0x0223
#define STM32L0_GPIO_PIN_PD0_SPI2_NSS           0x0130
#define STM32L0_GPIO_PIN_PD1_SPI2_SCK           0x0131
#define STM32L0_GPIO_PIN_PD3_SPI2_MISO          0x0233
#define STM32L0_GPIO_PIN_PD4_SPI2_MOSI          0x0134

#define STM32L0_GPIO_PIN_PB9_I2S2_WS            0x0519
#define STM32L0_GPIO_PIN_PB12_I2S2_WS           0x001c
#define STM32L0_GPIO_PIN_PB13_I2S2_CK           0x001d
#define STM32L0_GPIO_PIN_PB14_I2S2_MCK          0x001e
#define STM32L0_GPIO_PIN_PB15_I2S2_SD           0x001f
#define STM32L0_GPIO_PIN_PC2_I2S2_MCK           0x0222
#define STM32L0_GPIO_PIN_PC3_I2S2_SD            0x0223
#define STM32L0_GPIO_PIN_PD0_I2S2_WS            0x0130
#define STM32L0_GPIO_PIN_PD1_I2S2_CK            0x0131
#define STM32L0_GPIO_PIN_PD3_I2S2_MCK           0x0233
#define STM32L0_GPIO_PIN_PD4_I2S2_SD            0x0134

#define STM32L0_GPIO_PIN_PA8_USB_CRS_STBC       0x0208
#define STM32L0_GPIO_PIN_PA13_USB_OE            0x020d
#define STM32L0_GPIO_PIN_PH0_USB_CRS_SYNC       0x0070
#define STM32L0_GPIO_PIN_PC9_USB_NOE            0x0229
#define STM32L0_GPIO_PIN_PD15_USB_CRS_SYNC      0x003f

#define STM32L0_GPIO_PIN_PA0_TIM2_CH1           0x0200
#define STM32L0_GPIO_PIN_PA0_TIM2_ETR           0x0500
#define STM32L0_GPIO_PIN_PA1_TIM2_CH2           0x0201
#define STM32L0_GPIO_PIN_PA2_TIM2_CH3           0x0202
#define STM32L0_GPIO_PIN_PA3_TIM2_CH4           0x0203
#define STM32L0_GPIO_PIN_PA5_TIM2_CH1           0x0505
#define STM32L0_GPIO_PIN_PA5_TIM2_ETR           0x0205
#define STM32L0_GPIO_PIN_PA15_TIM2_CH1          0x050f
#define STM32L0_GPIO_PIN_PA15_TIM2_ETR          0x020f
#define STM32L0_GPIO_PIN_PB3_TIM2_CH2           0x0213
#define STM32L0_GPIO_PIN_PB10_TIM2_CH3          0x021a
#define STM32L0_GPIO_PIN_PB11_TIM2_CH4          0x021b
#define STM32L0_GPIO_PIN_PE9_TIM2_CH1           0x0049
#define STM32L0_GPIO_PIN_PE9_TIM2_ETR           0x0249
#define STM32L0_GPIO_PIN_PE10_TIM2_CH2          0x004a
#define STM32L0_GPIO_PIN_PE11_TIM2_CH3          0x004b
#define STM32L0_GPIO_PIN_PE12_TIM2_CH4          0x004c

#define STM32L0_GPIO_PIN_PA6_TIM3_CH1           0x0206
#define STM32L0_GPIO_PIN_PA7_TIM3_CH2           0x0207
#define STM32L0_GPIO_PIN_PB0_TIM3_CH3           0x0210
#define STM32L0_GPIO_PIN_PB1_TIM3_CH4           0x0211
#define STM32L0_GPIO_PIN_PB4_TIM3_CH1           0x0214
#define STM32L0_GPIO_PIN_PB5_TIM3_CH2           0x0415
#define STM32L0_GPIO_PIN_PC6_TIM3_CH1           0x0226
#define STM32L0_GPIO_PIN_PC7_TIM3_CH2           0x0227
#define STM32L0_GPIO_PIN_PC8_TIM3_CH3           0x0228
#define STM32L0_GPIO_PIN_PC9_TIM3_CH4           0x0229
#define STM32L0_GPIO_PIN_PD2_TIM3_ETR           0x0232
#define STM32L0_GPIO_PIN_PE2_TIM3_ETR           0x0242
#define STM32L0_GPIO_PIN_PE3_TIM3_CH1           0x0243
#define STM32L0_GPIO_PIN_PE4_TIM3_CH2           0x0244
#define STM32L0_GPIO_PIN_PE5_TIM3_CH3           0x0245
#define STM32L0_GPIO_PIN_PE6_TIM3_CH4           0x0246

#define STM32L0_GPIO_PIN_PA1_TIM21_ETR          0x0501
#define STM32L0_GPIO_PIN_PA2_TIM21_CH1          0x0002
#define STM32L0_GPIO_PIN_PA3_TIM21_CH2          0x0003
#define STM32L0_GPIO_PIN_PB13_TIM21_CH1         0x061d
#define STM32L0_GPIO_PIN_PB14_TIM21_CH2         0x061e
#define STM32L0_GPIO_PIN_PC9_TIM21_ETR          0x0029
#define STM32L0_GPIO_PIN_PD0_TIM21_CH1          0x0030
#define STM32L0_GPIO_PIN_PD7_TIM21_CH2          0x0137
#define STM32L0_GPIO_PIN_PE5_TIM21_CH1          0x0045
#define STM32L0_GPIO_PIN_PE6_TIM21_CH2          0x0046

#define STM32L0_GPIO_PIN_PA4_TIM22_ETR          0x0504
#define STM32L0_GPIO_PIN_PA6_TIM22_CH1          0x0506
#define STM32L0_GPIO_PIN_PA7_TIM22_CH2          0x0507
#define STM32L0_GPIO_PIN_PB4_TIM22_CH1          0x0414
#define STM32L0_GPIO_PIN_PB5_TIM22_CH2          0x0415
#define STM32L0_GPIO_PIN_PC6_TIM22_CH1          0x0026
#define STM32L0_GPIO_PIN_PC7_TIM22_CH2          0x0027
#define STM32L0_GPIO_PIN_PC8_TIM22_ETR          0x0028
#define STM32L0_GPIO_PIN_PE3_TIM22_CH1          0x0043
#define STM32L0_GPIO_PIN_PE4_TIM22_CH2          0x0044

#define STM32L0_GPIO_PIN_PB2_LPTIM1_OUT         0x0212
#define STM32L0_GPIO_PIN_PB5_LPTIM1_IN1         0x0215
#define STM32L0_GPIO_PIN_PB6_LPTIM1_ETR         0x0216
#define STM32L0_GPIO_PIN_PB7_LPTIM1_IN2         0x0217
#define STM32L0_GPIO_PIN_PC0_LPTIM1_IN1         0x0020
#define STM32L0_GPIO_PIN_PC1_LPTIM1_OUT         0x0021
#define STM32L0_GPIO_PIN_PC2_LPTIM1_IN2         0x0022
#define STM32L0_GPIO_PIN_PC3_LPTIM1_ETR         0x0023

#define STM32L0_GPIO_PIN_PA0_COMP1_OUT          0x0700
#define STM32L0_GPIO_PIN_PA6_COMP1_OUT          0x0706
#define STM32L0_GPIO_PIN_PA11_COMP1_OUT         0x070b

#define STM32L0_GPIO_PIN_PA2_COMP2_OUT          0x0702
#define STM32L0_GPIO_PIN_PA7_COMP2_OUT          0x0707
#define STM32L0_GPIO_PIN_PA12_COMP2_OUT         0x070c

#define STM32L0_GPIO_PIN_PA0_TSC_G1_IO1         0x0300
#define STM32L0_GPIO_PIN_PA1_TSC_G1_IO2         0x0301
#define STM32L0_GPIO_PIN_PA2_TSC_G1_IO3         0x0302
#define STM32L0_GPIO_PIN_PA3_TSC_G1_IO4         0x0303
#define STM32L0_GPIO_PIN_PA4_TSC_G2_IO1         0x0304
#define STM32L0_GPIO_PIN_PA5_TSC_G2_IO2         0x0305
#define STM32L0_GPIO_PIN_PA6_TSC_G2_IO3         0x0306
#define STM32L0_GPIO_PIN_PA7_TSC_G2_IO4         0x0307
#define STM32L0_GPIO_PIN_PA9_TSC_G4_IO1         0x0309
#define STM32L0_GPIO_PIN_PA10_TSC_G4_IO2        0x030a
#define STM32L0_GPIO_PIN_PA11_TSC_G4_IO3        0x030b
#define STM32L0_GPIO_PIN_PA12_TSC_G4_IO4        0x030c
#define STM32L0_GPIO_PIN_PB0_TSC_G3_IO2         0x0310
#define STM32L0_GPIO_PIN_PB1_TSC_G3_IO3         0x0311
#define STM32L0_GPIO_PIN_PB2_TSC_G3_IO4         0x0312
#define STM32L0_GPIO_PIN_PB3_TSC_G5_IO1         0x0313
#define STM32L0_GPIO_PIN_PB4_TSC_G5_IO2         0x0314
#define STM32L0_GPIO_PIN_PB6_TSC_G5_IO3         0x0316
#define STM32L0_GPIO_PIN_PB7_TSC_G5_IO4         0x0317
#define STM32L0_GPIO_PIN_PB8_TSC_SYNC           0x0318
#define STM32L0_GPIO_PIN_PB10_TSC_SYNC          0x031a
#define STM32L0_GPIO_PIN_PB11_TSC_G6_IO1        0x031b
#define STM32L0_GPIO_PIN_PB12_TSC_G6_IO2        0x031c
#define STM32L0_GPIO_PIN_PB13_TSC_G6_IO3        0x031d
#define STM32L0_GPIO_PIN_PB14_TSC_G6_IO4        0x031e
#define STM32L0_GPIO_PIN_PC0_TSC_G7_IO1         0x0320
#define STM32L0_GPIO_PIN_PC1_TSC_G7_IO2         0x0321
#define STM32L0_GPIO_PIN_PC2_TSC_G7_IO3         0x0322
#define STM32L0_GPIO_PIN_PC3_TSC_G7_IO4         0x0323
#define STM32L0_GPIO_PIN_PC5_TSC_G3_IO1         0x0325
#define STM32L0_GPIO_PIN_PC6_TSC_G8_IO1         0x0326
#define STM32L0_GPIO_PIN_PC7_TSC_G8_IO2         0x0327
#define STM32L0_GPIO_PIN_PC8_TSC_G8_IO3         0x0328
#define STM32L0_GPIO_PIN_PC9_TSC_G8_IO4         0x0329

#endif /* STM32L072xx */

#if defined(STM32L082xx)

#define STM32L0_GPIO_PIN_PA8_MCO                0x0008
#define STM32L0_GPIO_PIN_PA9_MCO                0x0009
#define STM32L0_GPIO_PIN_PA13_SWDIO             0x000d
#define STM32L0_GPIO_PIN_PA14_SWCLK             0x000e
#define STM32L0_GPIO_PIN_PB13_MCO               0x021d
#define STM32L0_GPIO_PIN_PB14_RTC_OUT           0x021e
#define STM32L0_GPIO_PIN_PB15_RTC_REFIN         0x021f

#define STM32L0_GPIO_PIN_PA8_USART1_CK          0x0408
#define STM32L0_GPIO_PIN_PA9_USART1_TX          0x0409
#define STM32L0_GPIO_PIN_PA10_USART1_RX         0x040a
#define STM32L0_GPIO_PIN_PA11_USART1_CTS        0x040b
#define STM32L0_GPIO_PIN_PA12_USART1_RTS_DE     0x040c
#define STM32L0_GPIO_PIN_PB3_USART1_RTS_DE      0x0513
#define STM32L0_GPIO_PIN_PB4_USART1_CTS         0x0514
#define STM32L0_GPIO_PIN_PB5_USART1_CK          0x0515
#define STM32L0_GPIO_PIN_PB6_USART1_TX          0x0016
#define STM32L0_GPIO_PIN_PB7_USART1_RX          0x0017

#define STM32L0_GPIO_PIN_PA0_USART2_CTS         0x0400
#define STM32L0_GPIO_PIN_PA1_USART2_RTS_DE      0x0401
#define STM32L0_GPIO_PIN_PA2_USART2_TX          0x0402
#define STM32L0_GPIO_PIN_PA3_USART2_RX          0x0403
#define STM32L0_GPIO_PIN_PA4_USART2_CK          0x0404
#define STM32L0_GPIO_PIN_PA14_USART2_TX         0x040e
#define STM32L0_GPIO_PIN_PA15_USART2_RX         0x040f

#define STM32L0_GPIO_PIN_PA0_USART4_TX          0x0600
#define STM32L0_GPIO_PIN_PA1_USART4_RX          0x0601
#define STM32L0_GPIO_PIN_PA15_USART4_RTS_DE     0x060f
#define STM32L0_GPIO_PIN_PB7_USART4_CTS         0x0617

#define STM32L0_GPIO_PIN_PB3_USART5_TX          0x0613
#define STM32L0_GPIO_PIN_PB4_USART5_RX          0x0614
#define STM32L0_GPIO_PIN_PB5_USART5_CK          0x0615
#define STM32L0_GPIO_PIN_PB5_USART5_RTS_DE      0x0615

#define STM32L0_GPIO_PIN_PA2_LPUART1_TX         0x0602
#define STM32L0_GPIO_PIN_PA3_LPUART1_RX         0x0603
#define STM32L0_GPIO_PIN_PA6_LPUART1_CTS        0x0406
#define STM32L0_GPIO_PIN_PA13_LPUART1_RX        0x060d
#define STM32L0_GPIO_PIN_PA14_LPUART1_TX        0x060e
#define STM32L0_GPIO_PIN_PB1_LPUART1_RTS_DE     0x0411
#define STM32L0_GPIO_PIN_PB10_LPUART1_TX        0x041a
#define STM32L0_GPIO_PIN_PB11_LPUART1_RX        0x041b
#define STM32L0_GPIO_PIN_PB12_LPUART1_RTS_DE    0x021c
#define STM32L0_GPIO_PIN_PB13_LPUART1_CTS       0x041d
#define STM32L0_GPIO_PIN_PB14_LPUART1_RTS_DE    0x041e
#define STM32L0_GPIO_PIN_PC0_LPUART1_RX         0x0620
#define STM32L0_GPIO_PIN_PC1_LPUART1_TX         0x0621

#define STM32L0_GPIO_PIN_PA9_I2C1_SCL           0x0609
#define STM32L0_GPIO_PIN_PA10_I2C1_SDA          0x060a
#define STM32L0_GPIO_PIN_PB5_I2C1_SMBA          0x0315
#define STM32L0_GPIO_PIN_PB6_I2C1_SCL           0x0116
#define STM32L0_GPIO_PIN_PB7_I2C1_SDA           0x0117
#define STM32L0_GPIO_PIN_PB8_I2C1_SCL           0x0418
#define STM32L0_GPIO_PIN_PB9_I2C1_SDA           0x0419

#define STM32L0_GPIO_PIN_PB10_I2C2_SCL          0x061a
#define STM32L0_GPIO_PIN_PB11_I2C2_SDA          0x061b
#define STM32L0_GPIO_PIN_PB12_I2C2_SMBA         0x051c
#define STM32L0_GPIO_PIN_PB13_I2C2_SCL          0x051d
#define STM32L0_GPIO_PIN_PB14_I2C2_SDA          0x051e

#define STM32L0_GPIO_PIN_PA8_I2C3_SCL           0x0708
#define STM32L0_GPIO_PIN_PA9_I2C3_SMBA          0x0709
#define STM32L0_GPIO_PIN_PB2_I2C3_SMBA          0x0712
#define STM32L0_GPIO_PIN_PB4_I2C3_SDA           0x0714
#define STM32L0_GPIO_PIN_PC0_I2C3_SCL           0x0720
#define STM32L0_GPIO_PIN_PC1_I2C3_SDA           0x0721

#define STM32L0_GPIO_PIN_PA4_SPI1_NSS           0x0004
#define STM32L0_GPIO_PIN_PA5_SPI1_SCK           0x0005
#define STM32L0_GPIO_PIN_PA6_SPI1_MISO          0x0006
#define STM32L0_GPIO_PIN_PA7_SPI1_MOSI          0x0007
#define STM32L0_GPIO_PIN_PA11_SPI1_MISO         0x000b
#define STM32L0_GPIO_PIN_PA12_SPI1_MOSI         0x000c
#define STM32L0_GPIO_PIN_PA15_SPI1_NSS          0x000f
#define STM32L0_GPIO_PIN_PB3_SPI1_SCK           0x0013
#define STM32L0_GPIO_PIN_PB4_SPI1_MISO          0x0014
#define STM32L0_GPIO_PIN_PB5_SPI1_MOSI          0x0015

#define STM32L0_GPIO_PIN_PB9_SPI2_NSS           0x0519
#define STM32L0_GPIO_PIN_PB10_SPI2_SCK          0x051a
#define STM32L0_GPIO_PIN_PB12_SPI2_NSS          0x001c
#define STM32L0_GPIO_PIN_PB13_SPI2_SCK          0x001d
#define STM32L0_GPIO_PIN_PB14_SPI2_MISO         0x001e
#define STM32L0_GPIO_PIN_PB15_SPI2_MOSI         0x001f
#define STM32L0_GPIO_PIN_PC2_SPI2_MISO          0x0222

#define STM32L0_GPIO_PIN_PB9_I2S2_WS            0x0519
#define STM32L0_GPIO_PIN_PB12_I2S2_WS           0x001c
#define STM32L0_GPIO_PIN_PB13_I2S2_CK           0x001d
#define STM32L0_GPIO_PIN_PB14_I2S2_MCK          0x001e
#define STM32L0_GPIO_PIN_PB15_I2S2_SD           0x001f
#define STM32L0_GPIO_PIN_PC2_I2S2_MCK           0x0222

#define STM32L0_GPIO_PIN_PA8_USB_CRS_STBC       0x0208
#define STM32L0_GPIO_PIN_PA13_USB_OE            0x020d
#define STM32L0_GPIO_PIN_PH0_USB_CRS_SYNC       0x0070

#define STM32L0_GPIO_PIN_PA0_TIM2_CH1           0x0200
#define STM32L0_GPIO_PIN_PA0_TIM2_ETR           0x0500
#define STM32L0_GPIO_PIN_PA1_TIM2_CH2           0x0201
#define STM32L0_GPIO_PIN_PA2_TIM2_CH3           0x0202
#define STM32L0_GPIO_PIN_PA3_TIM2_CH4           0x0203
#define STM32L0_GPIO_PIN_PA5_TIM2_CH1           0x0505
#define STM32L0_GPIO_PIN_PA5_TIM2_ETR           0x0205
#define STM32L0_GPIO_PIN_PA15_TIM2_CH1          0x050f
#define STM32L0_GPIO_PIN_PA15_TIM2_ETR          0x020f
#define STM32L0_GPIO_PIN_PB3_TIM2_CH2           0x0213
#define STM32L0_GPIO_PIN_PB10_TIM2_CH3          0x021a
#define STM32L0_GPIO_PIN_PB11_TIM2_CH4          0x021b

#define STM32L0_GPIO_PIN_PA6_TIM3_CH1           0x0206
#define STM32L0_GPIO_PIN_PA7_TIM3_CH2           0x0207
#define STM32L0_GPIO_PIN_PB0_TIM3_CH3           0x0210
#define STM32L0_GPIO_PIN_PB1_TIM3_CH4           0x0211
#define STM32L0_GPIO_PIN_PB4_TIM3_CH1           0x0214
#define STM32L0_GPIO_PIN_PB5_TIM3_CH2           0x0415

#define STM32L0_GPIO_PIN_PA1_TIM21_ETR          0x0501
#define STM32L0_GPIO_PIN_PA2_TIM21_CH1          0x0002
#define STM32L0_GPIO_PIN_PA3_TIM21_CH2          0x0003
#define STM32L0_GPIO_PIN_PB13_TIM21_CH1         0x061d
#define STM32L0_GPIO_PIN_PB14_TIM21_CH2         0x061e

#define STM32L0_GPIO_PIN_PA4_TIM22_ETR          0x0504
#define STM32L0_GPIO_PIN_PA6_TIM22_CH1          0x0506
#define STM32L0_GPIO_PIN_PA7_TIM22_CH2          0x0507
#define STM32L0_GPIO_PIN_PB4_TIM22_CH1          0x0414
#define STM32L0_GPIO_PIN_PB5_TIM22_CH2          0x0415

#define STM32L0_GPIO_PIN_PB2_LPTIM1_OUT         0x0212
#define STM32L0_GPIO_PIN_PB5_LPTIM1_IN1         0x0215
#define STM32L0_GPIO_PIN_PB6_LPTIM1_ETR         0x0216
#define STM32L0_GPIO_PIN_PB7_LPTIM1_IN2         0x0217
#define STM32L0_GPIO_PIN_PC0_LPTIM1_IN1         0x0020
#define STM32L0_GPIO_PIN_PC1_LPTIM1_OUT         0x0021
#define STM32L0_GPIO_PIN_PC2_LPTIM1_IN2         0x0022

#define STM32L0_GPIO_PIN_PA0_COMP1_OUT          0x0700
#define STM32L0_GPIO_PIN_PA6_COMP1_OUT          0x0706
#define STM32L0_GPIO_PIN_PA11_COMP1_OUT         0x070b

#define STM32L0_GPIO_PIN_PA2_COMP2_OUT          0x0702
#define STM32L0_GPIO_PIN_PA7_COMP2_OUT          0x0707
#define STM32L0_GPIO_PIN_PA12_COMP2_OUT         0x070c

#define STM32L0_GPIO_PIN_PA0_TSC_G1_IO1         0x0300
#define STM32L0_GPIO_PIN_PA1_TSC_G1_IO2         0x0301
#define STM32L0_GPIO_PIN_PA2_TSC_G1_IO3         0x0302
#define STM32L0_GPIO_PIN_PA3_TSC_G1_IO4         0x0303
#define STM32L0_GPIO_PIN_PA4_TSC_G2_IO1         0x0304
#define STM32L0_GPIO_PIN_PA5_TSC_G2_IO2         0x0305
#define STM32L0_GPIO_PIN_PA6_TSC_G2_IO3         0x0306
#define STM32L0_GPIO_PIN_PA7_TSC_G2_IO4         0x0307
#define STM32L0_GPIO_PIN_PA9_TSC_G4_IO1         0x0309
#define STM32L0_GPIO_PIN_PA10_TSC_G4_IO2        0x030a
#define STM32L0_GPIO_PIN_PA11_TSC_G4_IO3        0x030b
#define STM32L0_GPIO_PIN_PA12_TSC_G4_IO4        0x030c
#define STM32L0_GPIO_PIN_PB0_TSC_G3_IO2         0x0310
#define STM32L0_GPIO_PIN_PB1_TSC_G3_IO3         0x0311
#define STM32L0_GPIO_PIN_PB2_TSC_G3_IO4         0x0312
#define STM32L0_GPIO_PIN_PB3_TSC_G5_IO1         0x0313
#define STM32L0_GPIO_PIN_PB4_TSC_G5_IO2         0x0314
#define STM32L0_GPIO_PIN_PB6_TSC_G5_IO3         0x0316
#define STM32L0_GPIO_PIN_PB7_TSC_G5_IO4         0x0317
#define STM32L0_GPIO_PIN_PB8_TSC_SYNC           0x0318
#define STM32L0_GPIO_PIN_PB10_TSC_SYNC          0x031a
#define STM32L0_GPIO_PIN_PB11_TSC_G6_IO1        0x031b
#define STM32L0_GPIO_PIN_PB12_TSC_G6_IO2        0x031c
#define STM32L0_GPIO_PIN_PB13_TSC_G6_IO3        0x031d
#define STM32L0_GPIO_PIN_PB14_TSC_G6_IO4        0x031e
#define STM32L0_GPIO_PIN_PC0_TSC_G7_IO1         0x0320
#define STM32L0_GPIO_PIN_PC1_TSC_G7_IO2         0x0321
#define STM32L0_GPIO_PIN_PC2_TSC_G7_IO3         0x0322

#endif /* STM32L082xx */

#if defined(STM32L052xx)

#define STM32L0_GPIO_PORT_COUNT                 6 /* A, B, C, D, H */
#define STM32L0_GPIO_GROUP_COUNT                8 /* A, B, C, D, E, F, G, H */

#endif /* STM32L052xx */

#if defined(STM32L072xx)

#define STM32L0_GPIO_PORT_COUNT                 6 /* A, B, C, D, E, H */
#define STM32L0_GPIO_GROUP_COUNT                8 /* A, B, C, D, E, F, G, H */

#endif /* STM32L072xx */

#if defined(STM32L082xx)

#define STM32L0_GPIO_PORT_COUNT                 4 /* A, B, C, H */
#define STM32L0_GPIO_GROUP_COUNT                8 /* A, B, C, D, E, F, G, H */

#endif /* STM32L082xx */

typedef struct _stm32l0_gpio_state_t {
    uint32_t                mode[STM32L0_GPIO_PORT_COUNT];
    uint32_t                pupd[STM32L0_GPIO_PORT_COUNT];
} stm32l0_gpio_state_t;

#define STM32L0_GPIO_PIN_MASK(_pin)             (1ul << ((_pin) & 15))

extern void __stm32l0_gpio_initialize(void);

extern void stm32l0_gpio_swd_enable(void);
extern void stm32l0_gpio_swd_disable(void);
extern void stm32l0_gpio_pin_configure(unsigned int pin, unsigned int mode);
extern void stm32l0_gpio_pin_input(unsigned int pin);
extern void stm32l0_gpio_pin_output(unsigned int pin);
extern void stm32l0_gpio_pin_alternate(unsigned int pin);
extern void stm32l0_gpio_pin_analog(unsigned int pin);
extern void stm32l0_gpio_save(stm32l0_gpio_state_t *state);
extern void stm32l0_gpio_restore(stm32l0_gpio_state_t *state);

extern unsigned int __stm32l0_gpio_pin_read(unsigned int pin);
extern void __stm32l0_gpio_pin_write(unsigned int pin, unsigned int data);

static inline unsigned int stm32l0_gpio_pin_read(unsigned int pin)
{
    if (__builtin_constant_p(pin))
    {
        GPIO_TypeDef *GPIO;
        uint32_t group, index;
        
        group = (pin & STM32L0_GPIO_PIN_GROUP_MASK) >> STM32L0_GPIO_PIN_GROUP_SHIFT;
        index = (pin & STM32L0_GPIO_PIN_INDEX_MASK) >> STM32L0_GPIO_PIN_INDEX_SHIFT;
        
        GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);
        
        return ((GPIO->IDR >> index) & 1);
    }
    else
    {
        return __stm32l0_gpio_pin_read(pin);
    }
}

static inline void stm32l0_gpio_pin_write(unsigned int pin, unsigned int data)
{
    if (__builtin_constant_p(pin))
    {
        GPIO_TypeDef *GPIO;
        uint32_t group, index;
        
        group = (pin & STM32L0_GPIO_PIN_GROUP_MASK) >> STM32L0_GPIO_PIN_GROUP_SHIFT;
        index = (pin & STM32L0_GPIO_PIN_INDEX_MASK) >> STM32L0_GPIO_PIN_INDEX_SHIFT;
        
        GPIO = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE - GPIOA_BASE) * group);

        if (data) {
            GPIO->BSRR = (1ul << index);
        } else {
            GPIO->BRR = (1ul << index);
        }
    }
    else
    {
        __stm32l0_gpio_pin_write(pin, data);
    }
}

#ifdef __cplusplus
}
#endif

#endif /* _STM32L0_GPIO_H */
