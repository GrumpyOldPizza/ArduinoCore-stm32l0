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

#include "Arduino.h"
#include "wiring_private.h"
#include "dosfs_api.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(USBCON)
void USBD_Poll(void) __attribute__((weak));
void USBD_Poll(void) { };
#endif

stm32l0_spi_t g_SPI;
extern const stm32l0_spi_params_t g_SPIParams;

stm32l0_i2c_t g_Wire;
extern const stm32l0_i2c_params_t g_WireParams;

extern const stm32l0_sfspi_params_t g_SFSPIParams;

extern const stm32l0_sdspi_params_t g_SDSPIParams;

void HardFault_Handler(void)
{
    while (1)
    {
#if defined(USBCON)
	USBD_Poll();
#endif
    }
}

int g_swdStatus = 0;

void init( void )
{
    armv6m_core_initialize();
    stm32l0_system_initialize(_SYSTEM_CORE_CLOCK_, 0, 0, STM32L0_CONFIG_LSECLK, STM32L0_CONFIG_HSECLK, STM32L0_CONFIG_SYSOPT);

    stm32l0_exti_configure(STM32L0_EXTI_IRQ_PRIORITY);
    stm32l0_rtc_configure(STM32L0_RTC_IRQ_PRIORITY);
    stm32l0_dma_configure(STM32L0_ADC_IRQ_PRIORITY, STM32L0_UART_IRQ_PRIORITY, STM32L0_UART_IRQ_PRIORITY);
    stm32l0_lptim_configure(STM32L0_LPTIM_IRQ_PRIORITY);

#if defined(STM32L0_CONFIG_PIN_VBUS)
    if (STM32L0_CONFIG_PIN_VBUS != STM32L0_GPIO_PIN_NONE) {
	stm32l0_gpio_pin_configure(STM32L0_CONFIG_PIN_VBUS, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));
    }
#endif

#if defined(STM32L0_CONFIG_PIN_VBAT)
    stm32l0_gpio_pin_configure(STM32L0_CONFIG_PIN_VBAT, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
#endif

#if (DOSFS_SDCARD >= 1)
    if (g_SPI.state == STM32L0_SPI_STATE_NONE) {
	stm32l0_spi_create(&g_SPI, &g_SPIParams);
    }

    stm32l0_sdspi_initialize(&g_SPI, &g_SDSPIParams);
#endif
#if (DOSFS_SFLASH >= 1)
    if (g_SPI.state == STM32L0_SPI_STATE_NONE) {
	stm32l0_spi_create(&g_SPI, &g_SPIParams);
    }

    stm32l0_sfspi_initialize(&g_SPI, &g_SFSPIParams);

    dosfs_sflash_init(STM32L0_CONFIG_SFLASH_DATA_START);
#endif

    /* This is here to work around a linker issue in avr/fdevopen.c */
    asm(".global stm32l0_stdio_put");
    asm(".global stm32l0_stdio_get");
}

#ifdef __cplusplus
}
#endif
