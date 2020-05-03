/*
 * Copyright (c) 2017-2019 Thomas Roell.  All rights reserved.
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

#define PWM_INSTANCE_TIM2      0

/*
 * Pins descriptions
 */
extern const PinDescription g_APinDescription[PINS_COUNT] =
{
    // 0..13 - Digital pins
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA10), STM32L0_GPIO_PIN_PA10,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA9),  STM32L0_GPIO_PIN_PA9,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB5),  STM32L0_GPIO_PIN_PB5,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB6),  STM32L0_GPIO_PIN_PB6,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB7),  STM32L0_GPIO_PIN_PB7,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB2),  STM32L0_GPIO_PIN_PB2,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  0,                                            STM32L0_GPIO_PIN_NONE,           0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  0,                                            STM32L0_GPIO_PIN_NONE,           0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA13), STM32L0_GPIO_PIN_PA13,           (PIN_ATTR_SWD | PIN_ATTR_EXTI),                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA14), STM32L0_GPIO_PIN_PA14,           (PIN_ATTR_SWD | PIN_ATTR_EXTI),                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB12), STM32L0_GPIO_PIN_PB12,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  0,                                            STM32L0_GPIO_PIN_NONE,           0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  0,                                            STM32L0_GPIO_PIN_NONE,           0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  0,                                            STM32L0_GPIO_PIN_NONE,           0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 14..15 - I2C pins (SDA,SCL)
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB9),  STM32L0_GPIO_PIN_PB9,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB8),  STM32L0_GPIO_PIN_PB8,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 16..21 - Analog pins
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA4),  STM32L0_GPIO_PIN_PA4,            (PIN_ATTR_DAC1),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_4    },
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA5),  STM32L0_GPIO_PIN_PA5,            (PIN_ATTR_DAC2 | PIN_ATTR_EXTI),               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_5    },
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA2),  STM32L0_GPIO_PIN_PA2_TIM2_CH3,   (PIN_ATTR_EXTI),                               PWM_INSTANCE_TIM2,  PWM_CHANNEL_3,    ADC_CHANNEL_2    },
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA3),  STM32L0_GPIO_PIN_PA3_TIM2_CH4,   (PIN_ATTR_EXTI),                               PWM_INSTANCE_TIM2,  PWM_CHANNEL_4,    ADC_CHANNEL_3    },
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA0),  STM32L0_GPIO_PIN_PA0_TIM2_CH1,   (PIN_ATTR_WKUP1 | PIN_ATTR_TAMP),              PWM_INSTANCE_TIM2,  PWM_CHANNEL_1,    ADC_CHANNEL_0    },
    { NULL,  0,                                            STM32L0_GPIO_PIN_NONE,           0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 22..25 - Special pins (USB_DM, USB_DP, USB_VBUS)
    { NULL,  STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA11), STM32L0_GPIO_PIN_PA11,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA12), STM32L0_GPIO_PIN_PA12,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA8),  STM32L0_GPIO_PIN_PA8,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
};

extern const unsigned int g_PWMInstances[PWM_INSTANCE_COUNT] = {
    STM32L0_TIMER_INSTANCE_TIM2,
};


static uint8_t stm32l0_usart1_rx_fifo[32];

extern const stm32l0_uart_params_t g_Serial1Params = {
    STM32L0_UART_INSTANCE_USART1,
    STM32L0_UART_IRQ_PRIORITY,
    STM32L0_DMA_CHANNEL_DMA1_CH3_USART1_RX,
    STM32L0_DMA_CHANNEL_NONE,
    &stm32l0_usart1_rx_fifo[0],
    sizeof(stm32l0_usart1_rx_fifo),
    {
        STM32L0_GPIO_PIN_PA10_USART1_RX,
        STM32L0_GPIO_PIN_PA9_USART1_TX,
        STM32L0_GPIO_PIN_NONE,
        STM32L0_GPIO_PIN_NONE,
    },
};


static uint8_t stm32l0_usart2_rx_fifo[32];

extern const stm32l0_uart_params_t g_Serial2Params = {
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


extern const stm32l0_uart_params_t g_Serial3Params = {
    STM32L0_UART_INSTANCE_LPUART1,
    STM32L0_UART_IRQ_PRIORITY,
    STM32L0_DMA_CHANNEL_NONE,
    STM32L0_DMA_CHANNEL_NONE,
    NULL,
    0,
    {
        STM32L0_GPIO_PIN_PA13_LPUART1_RX,
        STM32L0_GPIO_PIN_PA14_LPUART1_TX,
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

extern stm32l0_i2c_t g_Wire;


void initVariant()
{
    stm32l0_i2c_transaction_t transaction;
    uint8_t tx_data[2];

    stm32l0_i2c_create(&g_Wire, &g_WireParams);
    stm32l0_i2c_enable(&g_Wire, STM32L0_I2C_OPTION_MODE_100K, 0, NULL, NULL);

    tx_data[0] = 0x11;
    tx_data[1] = 0x20;

    transaction.status = STM32L0_I2C_STATUS_SUCCESS;
    transaction.control = 0;
    transaction.address = 0x18;
    transaction.tx_data = (uint8_t*)&tx_data[0];
    transaction.rx_data = NULL;
    transaction.tx_count = 2;
    transaction.rx_count = 0;
    transaction.callback = NULL;
    transaction.context = NULL;

    if (stm32l0_i2c_submit(&g_Wire, &transaction)) {
        while (transaction.status == STM32L0_I2C_STATUS_BUSY) {
            armv6m_core_wait();
        }
    }

    stm32l0_i2c_disable(&g_Wire);

    if (transaction.status == STM32L0_I2C_STATUS_SUCCESS) {
        CMWX1ZZABZ_Initialize(STM32L0_GPIO_PIN_PH1, STM32L0_GPIO_PIN_NONE);
    } else {
        CMWX1ZZABZ_Initialize(STM32L0_GPIO_PIN_PB14, STM32L0_GPIO_PIN_NONE);
    }
}
