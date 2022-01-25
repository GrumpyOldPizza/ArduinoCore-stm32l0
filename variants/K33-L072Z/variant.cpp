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
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA3),  STM32L0_GPIO_PIN_PA3,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //T_VCP_RX
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA2),  STM32L0_GPIO_PIN_PA2,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //T_VCP_TX
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA10), STM32L0_GPIO_PIN_PA10,           0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //LED_GRN
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA0),  STM32L0_GPIO_PIN_PA0,            (PIN_ATTR_WKUP1 | PIN_ATTR_TAMP),              PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //IMU_INT
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB5),  STM32L0_GPIO_PIN_PB5,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //EXT_PWR
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB7),  STM32L0_GPIO_PIN_PB7,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //LOAD_TEST
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB2),  STM32L0_GPIO_PIN_PB2,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //EXT_EN
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA5),  STM32L0_GPIO_PIN_PA5,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //BGG_INT
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA9),  STM32L0_GPIO_PIN_PA9,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //LED_RED
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB6),  STM32L0_GPIO_PIN_PB6,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //EXTINT
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB12), STM32L0_GPIO_PIN_PB12,           0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //SPI1_SS
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB15), STM32L0_GPIO_PIN_PB15,           0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //SPI1_MOSI
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB14), STM32L0_GPIO_PIN_PB14,           0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //SPI1_MISO
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB13), STM32L0_GPIO_PIN_PB13,           0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //SPI_SCK

    // 14..15 - I2C pins (SDA,SCL)
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB9),  STM32L0_GPIO_PIN_PB9,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB8),  STM32L0_GPIO_PIN_PB8,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 16 - ADC pin
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA4),  STM32L0_GPIO_PIN_PA4,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_4    },
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


void RadioInit( const RadioEvents_t *events, uint32_t freq )
{
    SX1276Init(events, freq);
}

void initVariant()
{
    CMWX1ZZABZ_Initialize(STM32L0_GPIO_PIN_PH1, STM32L0_GPIO_PIN_NONE);
}