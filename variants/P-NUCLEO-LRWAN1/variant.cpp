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

#include "Arduino.h"
#include "wiring_private.h"

#define PWM_INSTANCE_TIM2     0
#define PWM_INSTANCE_TIM3     1

/*
 * Pins descriptions
 */
extern const PinDescription g_APinDescription[PINS_COUNT] =
{
    // 0..13 - Digital pins 
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA3),  STM32L0_GPIO_PIN_PA3,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA2),  STM32L0_GPIO_PIN_PA2,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA10), STM32L0_GPIO_PIN_PA10,           0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB3),  STM32L0_GPIO_PIN_PB3,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB5),  STM32L0_GPIO_PIN_PB5,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB4),  STM32L0_GPIO_PIN_PB4,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB10), STM32L0_GPIO_PIN_PB10_TIM2_CH3,  0,                                             PWM_INSTANCE_TIM2,  PWM_CHANNEL_3,    ADC_CHANNEL_NONE },
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA8),  STM32L0_GPIO_PIN_PA8,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA9),  STM32L0_GPIO_PIN_PA9,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC7),  STM32L0_GPIO_PIN_PC7_TIM3_CH2,   (PIN_ATTR_EXTI),                               PWM_INSTANCE_TIM3,  PWM_CHANNEL_2,    ADC_CHANNEL_NONE },
    { NULL,  STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB6),  STM32L0_GPIO_PIN_PB6,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA7),  STM32L0_GPIO_PIN_PA7,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA6),  STM32L0_GPIO_PIN_PA6,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA5),  STM32L0_GPIO_PIN_PA5,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 14..15 - I2C pins (SDA,SCL)
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB9),  STM32L0_GPIO_PIN_PB9,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB8),  STM32L0_GPIO_PIN_PB8,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 16..21 - Analog pins
    { NULL,  STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA0),  STM32L0_GPIO_PIN_PA0,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA1),  STM32L0_GPIO_PIN_PA1,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_1    },
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA4),  STM32L0_GPIO_PIN_PA4,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_4    },
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB0),  STM32L0_GPIO_PIN_PB0,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_8    },
    { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC1),  STM32L0_GPIO_PIN_PC1,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_11   },
    { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC0),  STM32L0_GPIO_PIN_PC0,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_10   },

    // 22 - Button 
    { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC13), STM32L0_GPIO_PIN_PC13,           (PIN_ATTR_EXTI | PIN_ATTR_WKUP1),              PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
};

extern const unsigned int g_PWMInstances[PWM_INSTANCE_COUNT] = {
    STM32L0_TIMER_INSTANCE_TIM2,
    STM32L0_TIMER_INSTANCE_TIM3,
};


static uint8_t stm32l0_usart2_rx_fifo[32];

extern const stm32l0_uart_params_t g_SerialParams = {
    STM32L0_UART_INSTANCE_USART2,
    STM32L0_UART_IRQ_PRIORITY,
    STM32L0_DMA_CHANNEL_DMA1_CH6_USART2_RX,
    STM32L0_DMA_CHANNEL_NONE,
    &stm32l0_usart2_rx_fifo[0],
    sizeof(stm32l0_usart2_rx_fifo),
    {
        STM32L0_GPIO_PIN_PA3_USART2_RX,
        STM32L0_GPIO_PIN_PA2_USART2_TX,
        STM32L0_GPIO_PIN_NONE,
        STM32L0_GPIO_PIN_NONE,
    },
};

extern const stm32l0_i2c_params_t g_WireParams = {
    STM32L0_I2C_INSTANCE_I2C1,
    STM32L0_I2C_IRQ_PRIORITY,
    STM32L0_DMA_CHANNEL_DMA1_CH7_I2C1_RX,
    STM32L0_DMA_CHANNEL_NONE,
    {
        STM32L0_GPIO_PIN_PB8_I2C1_SCL,
        STM32L0_GPIO_PIN_PB9_I2C1_SDA,
    },
};

void initVariant()
{
    SX1272MB2DAS_Initialize();
}
