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
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA0),  STM32L0_GPIO_PIN_PA0,    (PIN_ATTR_WKUP1 | PIN_ATTR_TAMP),   PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //IMU_INT/BTN
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA10), STM32L0_GPIO_PIN_PA10,   0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //LED_RED
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA9),  STM32L0_GPIO_PIN_PA9,    0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //LED_GREEN
    { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC0),  STM32L0_GPIO_PIN_PC0,    0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //LOAD_TEST
    { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC3),  STM32L0_GPIO_PIN_PC3,    0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //VBAT_TEST
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA7),  STM32L0_GPIO_PIN_PA7,    0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //TEMP_TEST
    { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC1),  STM32L0_GPIO_PIN_PC1,    0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //GEN_POWER
    { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC2),  STM32L0_GPIO_PIN_PC2,    0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //GEN_ENABLE
    { NULL,  0,                                            STM32L0_GPIO_PIN_NONE,   0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    // NC
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB0),  STM32L0_GPIO_PIN_PB0,    0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //PERIPH_EN
    { NULL,  0,                                            STM32L0_GPIO_PIN_NONE,   0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    // NC
    { NULL,  0,                                            STM32L0_GPIO_PIN_NONE,   0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    // NC
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA2),  STM32L0_GPIO_PIN_PA2,    0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //T_VCP_RX
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA3),  STM32L0_GPIO_PIN_PA3,    0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //T_VCP_TX
    // 14..15 - I2C pins (SDA,SCL)
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB7),  STM32L0_GPIO_PIN_PB7,    0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //SDA
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB6),  STM32L0_GPIO_PIN_PB6,    0,                                  PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //SCL
    // 16 - Analog pin
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA5),  STM32L0_GPIO_PIN_PA5,    (PIN_ATTR_EXTI),                    PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_5    },    //LOAD_ADC
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA4),  STM32L0_GPIO_PIN_PA4,    (PIN_ATTR_EXTI),                    PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_4    },    //VBAT_ADC 
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA6),  STM32L0_GPIO_PIN_PA6,    (PIN_ATTR_EXTI),                    PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_6    },    //TEMP_ADC
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
    STM32L0_DMA_CHANNEL_DMA1_CH3_I2C1_RX,
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
    // Disable generator
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PC1,  (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PC2,  (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
    // Disable LOAD_TEST and VBAT_TEST, TEMP_TEST
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PC0,  (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PC0, 1);
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PC3,  (STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_MEDIUM | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PC3, 1);
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA7,  (STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_MEDIUM | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PA7, 1);
    // Set LOAD_ADC, VBAT_ADC, LOAD_ADC
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA5,  (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA4,  (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA6,  (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
    // Turn off LEDs
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA9,  (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PA9, 0);
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA10, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PA10, 0);
    // Set IMU_INT
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA0,  (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));
    // S76x_Initialize(STM32L0_GPIO_PIN_NONE);
    // Disable NSS
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PB12, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PB12, 1);
    // Set Ant Rx/Tx switch to Rx Mode
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA1, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PA1, 1);
    // Set RESET pin to 0
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PB10, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PB10, 0);
    // Wait 1 ms
    armv6m_core_udelay(1000);
    // Configure RESET as input
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PB10, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
    armv6m_core_udelay(6000);   // Wait 6 ms
}
