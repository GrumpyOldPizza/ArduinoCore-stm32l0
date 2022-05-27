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
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB7),  STM32L0_GPIO_PIN_PB7,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //T_VCP_TX
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB6),  STM32L0_GPIO_PIN_PB6,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //T_VCP_RX
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA8),  STM32L0_GPIO_PIN_PA8,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //BTN_A
    { GPIOC, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PC13), STM32L0_GPIO_PIN_PC13,           (PIN_ATTR_EXTI | PIN_ATTR_WKUP2),              PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //BTN_B
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB3),  STM32L0_GPIO_PIN_PB3,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //BTN_BL
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA15), STM32L0_GPIO_PIN_PA15,           0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //LED_BLU 
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB4),  STM32L0_GPIO_PIN_PB4,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //TFT_BL
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA11), STM32L0_GPIO_PIN_PA11,           0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //TFT_DC
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA9),  STM32L0_GPIO_PIN_PA9,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //BTN_C
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA12), STM32L0_GPIO_PIN_PA12,           0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //TFT_RST
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB12), STM32L0_GPIO_PIN_PB12,           0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //SPI1_SS
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB15), STM32L0_GPIO_PIN_PB15,           0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //SPI1_MOSI
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB14), STM32L0_GPIO_PIN_PB14,           0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //SPI1_MISO
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB13), STM32L0_GPIO_PIN_PB13,           0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //SPI1_SCK
    // 14..15 - I2C pins (SDA,SCL)
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB9),  STM32L0_GPIO_PIN_PB9,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //SDA
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB8),  STM32L0_GPIO_PIN_PB8,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //SCL
    // 16..17 - Analog pins
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA3),  STM32L0_GPIO_PIN_PA3,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_3    },    //TEMP_ADC
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA0),  STM32L0_GPIO_PIN_PA0,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_0    },    //VBAT_ADC
    // 18..19
    { GPIOA, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PA10), STM32L0_GPIO_PIN_PA10,           0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //TEMP_EN
    { GPIOB, STM32L0_GPIO_PIN_MASK(STM32L0_GPIO_PIN_PB5),  STM32L0_GPIO_PIN_PB5,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },    //VBAT_EN
};

static uint8_t stm32l0_usart1_rx_fifo[32];

extern const stm32l0_uart_params_t g_SerialParams = {
    STM32L0_UART_INSTANCE_USART1,
    STM32L0_UART_IRQ_PRIORITY,
    STM32L0_DMA_CHANNEL_DMA1_CH3_USART1_RX,
    STM32L0_DMA_CHANNEL_NONE,
    &stm32l0_usart1_rx_fifo[0],
    sizeof(stm32l0_usart1_rx_fifo),
    {
        STM32L0_GPIO_PIN_PB7_USART1_RX,
        STM32L0_GPIO_PIN_PB6_USART1_TX,
        STM32L0_GPIO_PIN_NONE,
        STM32L0_GPIO_PIN_NONE,
    },
};

extern const stm32l0_spi_params_t g_SPIParams = {
    STM32L0_SPI_INSTANCE_SPI2,
    STM32L0_SPI_IRQ_PRIORITY,
    STM32L0_DMA_CHANNEL_DMA1_CH4_SPI2_RX,
    STM32L0_DMA_CHANNEL_DMA1_CH5_SPI2_TX,
    {
        STM32L0_GPIO_PIN_PB15_SPI2_MOSI,
        STM32L0_GPIO_PIN_PB14_SPI2_MISO,
        STM32L0_GPIO_PIN_PB13_SPI2_SCK,
        STM32L0_GPIO_PIN_PB12_SPI2_NSS,
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
    // Set VBAT_ADC, TEMP_ADC
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA0, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA3, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
    // Turn off LEDs
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA8, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PA8, 0);
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA9, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PA9, 0);;
    // Disable VBAT_EN (OUTPUT_OPENDRAIN)
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PB5, (STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_MEDIUM | STM32L0_GPIO_OTYPE_OPENDRAIN | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PB5, 1);
    // Disable NSS
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA4, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PA4, 1);
    // Set Ant Rx/Tx switch to Rx Mode
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA2, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PA1, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
	stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PA2, 1);
	stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PA1, 0);
    // Set RESET pin to 0
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PB11, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(STM32L0_GPIO_PIN_PB11, 0);
    // Wait 1 ms
    armv6m_core_udelay(1000);
    // Configure RESET as input
    stm32l0_gpio_pin_configure(STM32L0_GPIO_PIN_PB11, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
    // Wait 6 ms
    armv6m_core_udelay(6000);

    stm32l0_system_swd_enable();
}
