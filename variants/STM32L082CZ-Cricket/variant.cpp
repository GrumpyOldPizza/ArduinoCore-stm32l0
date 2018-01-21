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

#define PWM_INSTANCE_TIM2      0
#define PWM_INSTANCE_TIM3      1

/*
 * Pins descriptions
 */
extern const PinDescription g_APinDescription[PINS_COUNT] =
{
    // 0..13 - Digital pins
    { NULL,  GPIO_PIN_MASK(GPIO_PIN_PA10), GPIO_PIN_PA10,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  GPIO_PIN_MASK(GPIO_PIN_PA9),  GPIO_PIN_PA9,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB5),  GPIO_PIN_PB5_TIM3_CH2,   (PIN_ATTR_EXTI),                               PWM_INSTANCE_TIM3,  PWM_CHANNEL_2,    ADC_CHANNEL_NONE },
    { GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB6),  GPIO_PIN_PB6,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB7),  GPIO_PIN_PB7,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB2),  GPIO_PIN_PB2,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  0,                            GPIO_PIN_NONE,           0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  0,                            GPIO_PIN_NONE,           0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOA, GPIO_PIN_MASK(GPIO_PIN_PA13), GPIO_PIN_PA13,           (PIN_ATTR_SWD | PIN_ATTR_EXTI),                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOA, GPIO_PIN_MASK(GPIO_PIN_PA14), GPIO_PIN_PA14,           (PIN_ATTR_SWD | PIN_ATTR_EXTI),                PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
#if (DOSFS_SDCARD >= 1)
    { NULL,  GPIO_PIN_MASK(GPIO_PIN_PB12), GPIO_PIN_PB12,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
#else /* (DOSFS_SDCARD >= 1) */
    { GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB12), GPIO_PIN_PB12,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
#endif /* (DOSFS_SDCARD >= 1) */
#if (DOSFS_SDCARD >= 1) || (DOSFS_SFLASH >= 1)
    { NULL,  GPIO_PIN_MASK(GPIO_PIN_PB15), GPIO_PIN_PB15,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  GPIO_PIN_MASK(GPIO_PIN_PB14), GPIO_PIN_PB14,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  GPIO_PIN_MASK(GPIO_PIN_PB13), GPIO_PIN_PB13,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
#else /* (DOSFS_SDCARD >= 1) || (DOSFS_SFLASH >= 1) */
    { GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB15), GPIO_PIN_PB15,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB14), GPIO_PIN_PB14,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB13), GPIO_PIN_PB13,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
#endif /* (DOSFS_SDCARD >= 1) || (DOSFS_SFLASH >= 1) */

    // 14..15 - I2C pins (SDA,SCL)
    { GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB9),  GPIO_PIN_PB9,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { GPIOB, GPIO_PIN_MASK(GPIO_PIN_PB8),  GPIO_PIN_PB8,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 16..21 - Analog pins
    { GPIOA, GPIO_PIN_MASK(GPIO_PIN_PA4),  GPIO_PIN_PA4,            (PIN_ATTR_DAC1),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_4    },
    { GPIOA, GPIO_PIN_MASK(GPIO_PIN_PA5),  GPIO_PIN_PA5,            (PIN_ATTR_DAC2 | PIN_ATTR_EXTI),               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_5    },
    { GPIOA, GPIO_PIN_MASK(GPIO_PIN_PA2),  GPIO_PIN_PA2_TIM2_CH3,   (PIN_ATTR_EXTI),                               PWM_INSTANCE_TIM2,  PWM_CHANNEL_3,    ADC_CHANNEL_2    },
    { GPIOA, GPIO_PIN_MASK(GPIO_PIN_PA3),  GPIO_PIN_PA3_TIM2_CH4,   (PIN_ATTR_EXTI),                               PWM_INSTANCE_TIM2,  PWM_CHANNEL_4,    ADC_CHANNEL_3    },
    { GPIOA, GPIO_PIN_MASK(GPIO_PIN_PA0),  GPIO_PIN_PA0_TIM2_CH1,   (PIN_ATTR_WKUP1),                              PWM_INSTANCE_TIM2,  PWM_CHANNEL_1,    ADC_CHANNEL_0    },
    { NULL,  0,                            GPIO_PIN_NONE,           0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },

    // 22..25 - Special pins (USB_DM, USB_DP, USB_VBUS, SFLASH_CS)
    { NULL,  GPIO_PIN_MASK(GPIO_PIN_PA11), GPIO_PIN_PA11,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  GPIO_PIN_MASK(GPIO_PIN_PA12), GPIO_PIN_PA12,           (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
    { NULL,  GPIO_PIN_MASK(GPIO_PIN_PA8),  GPIO_PIN_PA8,            (PIN_ATTR_EXTI),                               PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
#if (DOSFS_SFLASH >= 1)
    { NULL,  GPIO_PIN_MASK(GPIO_PIN_PH0),  GPIO_PIN_PH0,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
#else /* (DOSFS_SFLASH >= 1) */
    { GPIOH, GPIO_PIN_MASK(GPIO_PIN_PH0),  GPIO_PIN_PH0,            0,                                             PWM_INSTANCE_NONE,  PWM_CHANNEL_NONE, ADC_CHANNEL_NONE },
#endif /* (DOSFS_SFLASH >= 1) */
};

extern const unsigned int g_PWMInstances[PWM_INSTANCE_COUNT] = {
    TIMER_INSTANCE_TIM2,
    TIMER_INSTANCE_TIM3,
};


static uint8_t stm32l0_usart1_rx_fifo[32];

extern const stm32l0_uart_params_t g_SerialGNSSParams = {
    UART_INSTANCE_USART1,
    UART_IRQ_PRIORITY,
    DMA_CHANNEL_DMA1_CH3_USART1_RX,
    DMA_CHANNEL_NONE,
    &stm32l0_usart1_rx_fifo[0],
    sizeof(stm32l0_usart1_rx_fifo),
    {
	GPIO_PIN_PA10_USART1_RX,
	GPIO_PIN_PA9_USART1_TX,
	GPIO_PIN_NONE,
	GPIO_PIN_NONE,
    },
};


static uint8_t stm32l0_usart2_rx_fifo[32];

extern const stm32l0_uart_params_t g_Serial1Params = {
    UART_INSTANCE_USART2,
    UART_IRQ_PRIORITY,
    DMA_CHANNEL_DMA1_CH5_USART2_RX,
    DMA_CHANNEL_DMA1_CH4_USART2_TX,
    &stm32l0_usart2_rx_fifo[0],
    sizeof(stm32l0_usart2_rx_fifo),
    {
	GPIO_PIN_PA3_USART2_RX,
	GPIO_PIN_PA2_USART2_TX,
	GPIO_PIN_PA4_USART2_CK,
	GPIO_PIN_PA0_USART2_CTS,
    },
};


extern const stm32l0_uart_params_t g_Serial2Params = {
    UART_INSTANCE_LPUART1,
    UART_IRQ_PRIORITY,
    DMA_CHANNEL_NONE,
    DMA_CHANNEL_NONE,
    NULL,
    0,
    {
	GPIO_PIN_PA13_LPUART1_RX,
	GPIO_PIN_PA14_LPUART1_TX,
	GPIO_PIN_NONE,
	GPIO_PIN_NONE,
    },
};


extern const stm32l0_spi_params_t g_SPIParams = {
    SPI_INSTANCE_SPI2,
    SPI_IRQ_PRIORITY,
    DMA_CHANNEL_DMA1_CH6_SPI2_RX,
    DMA_CHANNEL_NONE,
    {
        GPIO_PIN_PB15_SPI2_MOSI,
        GPIO_PIN_PB14_SPI2_MISO,
        GPIO_PIN_PB13_SPI2_SCK,
        GPIO_PIN_PB12_SPI2_NSS,
    },
};


extern const stm32l0_i2c_params_t g_WireParams = {
    I2C_INSTANCE_I2C1,
    I2C_IRQ_PRIORITY,
    DMA_CHANNEL_DMA1_CH7_I2C1_RX,
    DMA_CHANNEL_NONE,
    {
        GPIO_PIN_PB8_I2C1_SCL,
        GPIO_PIN_PB9_I2C1_SDA,
    },
};

extern stm32l0_spi_t g_SPI;

extern stm32l0_i2c_t g_Wire;

extern const stm32l0_sfspi_params_t g_SFSPIParams = {
    GPIO_PIN_PH0,
};

extern const stm32l0_sdspi_params_t g_SDSPIParams = {
    GPIO_PIN_PB12,
};

void initVariant()
{
    stm32l0_i2c_transaction_t transaction;
    uint8_t tx_data[2];

    CMWX1ZZABZ_Initialize();

    stm32l0_i2c_create(&g_Wire, &g_WireParams);
    stm32l0_i2c_enable(&g_Wire, I2C_OPTION_MODE_100K, NULL, NULL);

    tx_data[0] = 0x11;
    tx_data[1] = 0x20;

    transaction.control = I2C_CONTROL_TX;
    transaction.data = (uint8_t*)&tx_data[0];
    transaction.data2 = NULL;
    transaction.count = 2;
    transaction.count2 = 0;
    transaction.address = 0x18;
    transaction.callback = NULL;
    transaction.context = NULL;

    if (stm32l0_i2c_enqueue(&g_Wire, &transaction)) {
	while (transaction.status == I2C_STATUS_BUSY) {
	    armv6m_core_wait();
	}
    }

    tx_data[0] = 0xf4;
    tx_data[1] = 0x00;
    
    transaction.control = I2C_CONTROL_TX;
    transaction.data = (uint8_t*)&tx_data[0];
    transaction.data2 = NULL;
    transaction.count = 2;
    transaction.count2 = 0;
    transaction.address = 0x77;
    transaction.callback = NULL;
    transaction.context = NULL;

    if (stm32l0_i2c_enqueue(&g_Wire, &transaction)) {
	while (transaction.status == I2C_STATUS_BUSY) {
	    armv6m_core_wait();
	}
    }

    stm32l0_i2c_disable(&g_Wire);

#if (DOSFS_SFLASH == 0)
    stm32l0_gpio_pin_configure(GPIO_PIN_PH0, (GPIO_PARK_PULLUP | GPIO_PUPD_NONE | GPIO_OSPEED_VERY_HIGH | GPIO_OTYPE_PUSHPULL | GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(GPIO_PIN_PH0, 1);

    stm32l0_spi_create(&g_SPI, &g_SPIParams);
    stm32l0_spi_enable(&g_SPI);

    stm32l0_spi_acquire(&g_SPI, 32000000, 0);
    stm32l0_gpio_pin_write(GPIO_PIN_PH0, 0);

    armv6m_core_udelay(20);

    stm32l0_spi_data(&g_SPI, 0xb9);

    armv6m_core_udelay(20);

    stm32l0_gpio_pin_write(GPIO_PIN_PH0, 1);
    stm32l0_spi_release(&g_SPI);
        
    stm32l0_spi_disable(&g_SPI);
    stm32l0_gpio_pin_configure(GPIO_PIN_PH0, (GPIO_PARK_NONE | GPIO_MODE_ANALOG));
#endif /* (DOSFS_SFLASH == 0) */
}
