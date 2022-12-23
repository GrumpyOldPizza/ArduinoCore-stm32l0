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
#include "../Source/LoRa/Radio/radio.h"

/*
 * Pins descriptions
 */
extern const PinDescription g_APinDescription[PINS_COUNT] =
{
    // 0..13 - Digital pins 
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA10), STM32L0_GPIO_PIN_PA10,   (PIN_ATTR_EXTI),                    PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //RX
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA9),  STM32L0_GPIO_PIN_PA9,    (PIN_ATTR_EXTI),                    PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //TX
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA8),  STM32L0_GPIO_PIN_PA8,    (PIN_ATTR_EXTI),                    PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA7),  STM32L0_GPIO_PIN_PA7,    (PIN_ATTR_EXTI),                    PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA6),  STM32L0_GPIO_PIN_PA6,    (PIN_ATTR_EXTI),                    PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA5),  STM32L0_GPIO_PIN_PA5,    (PIN_ATTR_EXTI),                    PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA4),  STM32L0_GPIO_PIN_PA4,    (PIN_ATTR_EXTI),                    PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB2),  STM32L0_GPIO_PIN_PB2,    0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //GPS_RST
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB5),  STM32L0_GPIO_PIN_PB5,    (PIN_ATTR_EXTI),                    PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //GPS_PPS
    { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC6),  STM32L0_GPIO_PIN_PC6,    0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //GPS_LEVEL_SHIFTER_EN
    { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC11), STM32L0_GPIO_PIN_PC11,   0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //GPS_RX
    { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC10), STM32L0_GPIO_PIN_PC10,   0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //GPS_TX
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA14), STM32L0_GPIO_PIN_PA14,   (PIN_ATTR_SWD),                     PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //SWCLK
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA13), STM32L0_GPIO_PIN_PA13,   (PIN_ATTR_SWD),                     PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //SWDIO
    // 14..15 - I2C pins (SDA,SCL)
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB7),  STM32L0_GPIO_PIN_PB7,    0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //SDA
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB6),  STM32L0_GPIO_PIN_PB6,    0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //SCL
    // 16 - Analog pin
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA3),  STM32L0_GPIO_PIN_PA3,    (PIN_ATTR_EXTI),                    PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_3    },    //
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA2),  STM32L0_GPIO_PIN_PA2,    (PIN_ATTR_EXTI),                    PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_2    },    //
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA0),  STM32L0_GPIO_PIN_PA0,    (PIN_ATTR_WKUP1 | PIN_ATTR_TAMP),   PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_0    },    //
    // 19..21 - Special pins (VBUS, USB_DM, USB_DP)
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB1),  STM32L0_GPIO_PIN_PB1,    (PIN_ATTR_EXTI),                    PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_1    },    //VBUS
    { NULL,  STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA11), STM32L0_GPIO_PIN_PA11,   (PIN_ATTR_EXTI),                    PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //DM
    { NULL,  STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA12), STM32L0_GPIO_PIN_PA12,   (PIN_ATTR_EXTI),                    PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //DP
};


static uint8_t stm32l0_usart4_rx_fifo[32];

extern const stm32l0_uart_params_t g_Serial1Params = {
    STM32L0_UART_INSTANCE_USART4,
    STM32L0_UART_IRQ_PRIORITY,
    STM32L0_DMA_CHANNEL_DMA1_CH6_USART4_RX,
    STM32L0_DMA_CHANNEL_NONE,
    &stm32l0_usart4_rx_fifo[0],
    sizeof(stm32l0_usart4_rx_fifo),
    {
        STM32L0_GPIO_PIN_PC11_USART4_RX,
        STM32L0_GPIO_PIN_PC10_USART4_TX,
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
        STM32L0_GPIO_PIN_PB6_I2C1_SCL,
        STM32L0_GPIO_PIN_PB7_I2C1_SDA,
    },
};

extern stm32l0_i2c_t g_Wire;

void RadioInit( const RadioEvents_t *events, uint32_t freq )
{
    SX1276Init(events, freq);
}

void initVariant()
{
    // Set A1, A2
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA2, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA3, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
    // Set GPS RESET pin to 0
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PB2, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PB2, 0);
    // Enable GPS Level Shifter
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PC6, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PC6, 1);
    armv6m_core_udelay(250000); //250ms
    // Set GPS RESET pin to 1
    stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PB2, 1);
    armv6m_core_udelay(125000); //125ms
    // Set 1PPS input
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PB5, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));

	S76x_Initialize(STM32L0_GPIO_PIN_PD7);
}