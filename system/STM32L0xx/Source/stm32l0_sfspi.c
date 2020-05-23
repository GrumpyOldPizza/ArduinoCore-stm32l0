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

#include "armv6m.h"
#include "stm32l0_sfspi.h"
#include "stm32l0_system.h"
#include "dosfs_sflash.h"

#define SFLASH_CMD_WRSR         0x01
#define SFLASH_CMD_PAGE_PROGRAM 0x02
#define SFLASH_CMD_READ         0x03 /* up to 33MHz SPI clock */
#define SFLASH_CMD_RDSR         0x05
#define SFLASH_CMD_WREN         0x06
#define SFLASH_CMD_FAST_READ    0x0B
#define SFLASH_CMD_SECTOR_ERASE 0x20 /* 4K ERASE */
#define SFLASH_CMD_RDSCUR       0x2B /* MACRONIX */
#define SFLASH_CMD_WRSCUR       0x2F /* MACRONIX */
#define SFLASH_CMD_CLSR         0x30 /* SPANSION */
#define SFLASH_CMD_CLFS         0x50 /* MICRON */
#define SFLASH_CMD_RDFS         0x70 /* MICRON */
#define SFLASH_CMD_RDID         0x9F
#define SFLASH_CMD_RDPD         0xAB /* release from deep power down */
#define SFLASH_CMD_DPD          0xB9 /* deep power down */
#define SFLASH_CMD_CHIP_ERASE   0xC7
#define SFLASH_CMD_BLOCK_ERASE  0xD8 /* 64K ERASE */

#define SFLASH_SR_WIP           0x01 /* write in progress */
#define SFLASH_SR_WEL           0x02 /* write enable latch */
#define SFLASH_SR_E_ERR         0x20 /* SPANSION */
#define SFLASH_SR_P_ERR         0x40 /* SPANSION */

#define SFLASH_FS_P_FAIL        0x10 /* MICRON */
#define SFLASH_FS_E_FAIL        0x20 /* MICRON */

#define SFLASH_SCUR_P_FAIL      0x20 /* MACRONIX */
#define SFLASH_SCUR_E_FAIL      0x40 /* MACRONIX */


#define SFLASH_MID_SPANSION     0x01 /* RDSR / E_ERR / P_ERR */
#define SFLASH_MID_MICRON       0x20 /* RDFS / P_FAIL / E_FAIL */
#define SFLASH_MID_MACRONIX     0xC2 /* RDSCUR / P_FAIL / E_FAIL */

static stm32l0_sfspi_t stm32l0_sfspi;

static void stm32l0_sfspi_select(stm32l0_sfspi_t *sfspi)
{
    stm32l0_gpio_pin_write(sfspi->pin_cs, 0);

    /* 3ns delay */
}

static void stm32l0_sfspi_unselect(stm32l0_sfspi_t *sfspi)
{
    stm32l0_gpio_pin_write(sfspi->pin_cs, 1);

    /* 100ns delay */
    __NOP();
    __NOP();
    __NOP();
    __NOP();
}

static uint8_t stm32l0_sfspi_wait(stm32l0_sfspi_t *sfspi)
{
    uint8_t status;

    stm32l0_sfspi_select(sfspi);
    stm32l0_spi_data8(sfspi->spi, SFLASH_CMD_RDSR);

    do
    {
        status = stm32l0_spi_data8(sfspi->spi, 0xff);
    }
    while (status & SFLASH_SR_WIP);

    stm32l0_sfspi_unselect(sfspi);

    return status;
}

static bool stm32l0_sfspi_info(void *context, uint32_t *p_capacity, uint32_t *p_serial)
{
    stm32l0_sfspi_t *sfspi = (stm32l0_sfspi_t*)context;
    uint32_t uid[3];
    
    if ((sfspi->state != STM32L0_SFSPI_STATE_READY) && (sfspi->state != STM32L0_SFSPI_STATE_LOCKED))
    {
        return false;
    }

    stm32l0_system_uid(&uid[0]);
    
    *p_capacity = (1u << sfspi->ID[2]);
    *p_serial = (uid[0] + uid[2]) ^ uid[1];
    
    return true;
}

static void stm32l0_sfspi_hook(void *context, dosfs_device_lock_callback_t callback, void *cookie)
{
    stm32l0_sfspi_t *sfspi = (stm32l0_sfspi_t*)context;

    stm32l0_spi_hook(sfspi->spi, callback, cookie);
}

static void stm32l0_sfspi_lock(void *context)
{
    stm32l0_sfspi_t *sfspi = (stm32l0_sfspi_t*)context;

    stm32l0_spi_acquire(sfspi->spi, 32000000, 0);

    if (sfspi->state == STM32L0_SFSPI_STATE_SLEEP)
    {
        stm32l0_sfspi_select(sfspi);
        stm32l0_spi_data8(sfspi->spi, SFLASH_CMD_RDPD);
        stm32l0_sfspi_unselect(sfspi);
        
        armv6m_core_udelay(50);
    }

    sfspi->state = STM32L0_SFSPI_STATE_LOCKED;
}

static void stm32l0_sfspi_unlock(void *context)
{
    stm32l0_sfspi_t *sfspi = (stm32l0_sfspi_t*)context;

    stm32l0_spi_release(sfspi->spi);

    sfspi->state = STM32L0_SFSPI_STATE_READY;
}

static bool stm32l0_sfspi_erase(void *context, uint32_t address)
{
    stm32l0_sfspi_t *sfspi = (stm32l0_sfspi_t*)context;
    uint8_t status;

    DOSFS_SFLASH_STATISTICS_COUNT(sflash_command_erase);
    DOSFS_SFLASH_STATISTICS_COUNT_N(sflash_nor_erase, DOSFS_SFLASH_ERASE_SIZE);

    do
    {
        stm32l0_sfspi_select(sfspi);
        stm32l0_spi_data8(sfspi->spi, SFLASH_CMD_WREN);
        stm32l0_sfspi_unselect(sfspi);
        
        stm32l0_sfspi_select(sfspi);
        stm32l0_spi_data8(sfspi->spi, SFLASH_CMD_RDSR);
        status = stm32l0_spi_data8(sfspi->spi, 0xff);
        stm32l0_sfspi_unselect(sfspi);
    }
    while (!(status & SFLASH_SR_WEL));

    stm32l0_sfspi_select(sfspi);
    stm32l0_spi_data8(sfspi->spi, SFLASH_CMD_BLOCK_ERASE);
    stm32l0_spi_data8(sfspi->spi, (address >> 16));
    stm32l0_spi_data8(sfspi->spi, (address >> 8));
    stm32l0_spi_data8(sfspi->spi, (address >> 0));
    stm32l0_sfspi_unselect(sfspi);

    status = stm32l0_sfspi_wait(sfspi);

    if (sfspi->ID[0] == SFLASH_MID_MACRONIX)
    {
        stm32l0_sfspi_select(sfspi);
        stm32l0_spi_data8(sfspi->spi, SFLASH_CMD_RDSCUR);
        status = stm32l0_spi_data8(sfspi->spi, 0xff);
        stm32l0_sfspi_unselect(sfspi);
        
        if (status & SFLASH_SCUR_E_FAIL)
        {
            DOSFS_SFLASH_STATISTICS_COUNT(sflash_nor_efail);
            
            return false;
        }
    }

#if 0
    if (sfspi->ID[0] == SFLASH_MID_MICRON)
    {
        stm32l0_sfspi_select(sfspi);
        stm32l0_spi_data8(sfspi->spi, SFLASH_CMD_RDFS);
        status = stm32l0_spi_data8(sfspi->spi, 0xff);
        stm32l0_sfspi_unselect(sfspi);
        
        if (status & SFLASH_FS_E_FAIL)
        {
            DOSFS_SFLASH_STATISTICS_COUNT(sflash_nor_efail);

            stm32l0_sfspi_select(sfspi);
            stm32l0_spi_data8(sfspi->spi, SFLASH_CMD_CLFS);
            stm32l0_sfspi_unselect(sfspi);
            
            return false;
        }
    }

    if (sfspi->ID[0] == SFLASH_MID_SPANSION)
    {
        if (status & SFLASH_SR_E_ERR)
        {
            DOSFS_SFLASH_STATISTICS_COUNT(sflash_nor_efail);

            stm32l0_sfspi_select(sfspi);
            stm32l0_spi_data8(sfspi->spi, SFLASH_CMD_CLSR);
            stm32l0_sfspi_unselect(sfspi);

            return false;
        }
    }
#endif

    return true;
}

static bool stm32l0_sfspi_program(void *context, uint32_t address, const uint8_t *data, uint32_t count)
{
    stm32l0_sfspi_t *sfspi = (stm32l0_sfspi_t*)context;
    uint8_t status;

    DOSFS_SFLASH_STATISTICS_COUNT(sflash_command_program);
    DOSFS_SFLASH_STATISTICS_COUNT_N(sflash_nor_program, count);

    do
    {
        stm32l0_sfspi_select(sfspi);
        stm32l0_spi_data8(sfspi->spi, SFLASH_CMD_WREN);
        stm32l0_sfspi_unselect(sfspi);
        
        stm32l0_sfspi_select(sfspi);
        stm32l0_spi_data8(sfspi->spi, SFLASH_CMD_RDSR);
        status = stm32l0_spi_data8(sfspi->spi, 0xff);
        stm32l0_sfspi_unselect(sfspi);
    }
    while (!(status & SFLASH_SR_WEL));

    stm32l0_sfspi_select(sfspi);
    stm32l0_spi_data8(sfspi->spi, SFLASH_CMD_PAGE_PROGRAM);
    stm32l0_spi_data8(sfspi->spi, (address >> 16));
    stm32l0_spi_data8(sfspi->spi, (address >> 8));
    stm32l0_spi_data8(sfspi->spi, (address >> 0));
    stm32l0_spi_data(sfspi->spi, data, NULL, count);
    stm32l0_sfspi_unselect(sfspi);

    status = stm32l0_sfspi_wait(sfspi);

    if (sfspi->ID[0] == SFLASH_MID_MACRONIX)
    {
        stm32l0_sfspi_select(sfspi);
        stm32l0_spi_data8(sfspi->spi, SFLASH_CMD_RDSCUR);
        status = stm32l0_spi_data8(sfspi->spi, 0xff);
        stm32l0_sfspi_unselect(sfspi);
        
        if (status & SFLASH_SCUR_P_FAIL)
        {
            DOSFS_SFLASH_STATISTICS_COUNT(sflash_nor_pfail);
            
            return false;
        }
    }

#if 0
    if (sfspi->ID[0] == SFLASH_MID_MICRON)
    {
        stm32l0_sfspi_select(sfspi);
        stm32l0_spi_data8(sfspi->spi, SFLASH_CMD_RDFS);
        status = stm32l0_spi_data8(sfspi->spi, 0xff);
        stm32l0_sfspi_unselect(sfspi);
        
        if (status & SFLASH_FS_P_FAIL)
        {
            DOSFS_SFLASH_STATISTICS_COUNT(sflash_nor_pfail);

            stm32l0_sfspi_select(sfspi);
            stm32l0_spi_data8(sfspi->spi, SFLASH_CMD_CLFS);
            stm32l0_sfspi_unselect(sfspi);
            
            return false;
        }
    }

    if (sfspi->ID[0] == SFLASH_MID_SPANSION)
    {
        if (status & SFLASH_SR_P_ERR)
        {
            DOSFS_SFLASH_STATISTICS_COUNT(sflash_nor_pfail);

            stm32l0_sfspi_select(sfspi);
            stm32l0_spi_data8(sfspi->spi, SFLASH_CMD_CLSR);
            stm32l0_sfspi_unselect(sfspi);

            return false;
        }
    }
#endif

    return true;
}

static void stm32l0_sfspi_read(void *context, uint32_t address, uint8_t *data, uint32_t count)
{
    stm32l0_sfspi_t *sfspi = (stm32l0_sfspi_t*)context;

    DOSFS_SFLASH_STATISTICS_COUNT(sflash_command_read);
    DOSFS_SFLASH_STATISTICS_COUNT_N(sflash_nor_read, count);

    stm32l0_sfspi_select(sfspi);
    stm32l0_spi_data8(sfspi->spi, SFLASH_CMD_READ);
    stm32l0_spi_data8(sfspi->spi, (address >> 16));
    stm32l0_spi_data8(sfspi->spi, (address >> 8));
    stm32l0_spi_data8(sfspi->spi, (address >> 0));
    stm32l0_spi_data(sfspi->spi, NULL, data, count);
    stm32l0_sfspi_unselect(sfspi);
}

static void stm32l0_sfspi_callback(void *context, uint32_t notify)
{
    stm32l0_sfspi_t *sfspi = (stm32l0_sfspi_t*)context;
    
    if (notify & STM32L0_SYSTEM_NOTIFY_SLEEP)
    {
        if (sfspi->ID[0] == SFLASH_MID_MACRONIX)
        {
            /* acquire/release will block shared interrupts and USB/MSC.
             */

            if (sfspi->state == STM32L0_SFSPI_STATE_READY)
            {
                stm32l0_spi_acquire(sfspi->spi, 32000000, 0);

                stm32l0_sfspi_select(sfspi);
                stm32l0_spi_data8(sfspi->spi, SFLASH_CMD_DPD);
                stm32l0_sfspi_unselect(sfspi);
                
                stm32l0_spi_release(sfspi->spi);

                sfspi->state = STM32L0_SFSPI_STATE_SLEEP;
            }
        }
    }
}

static const  dosfs_sflash_interface_t stm32l0_sfspi_interface = {
    stm32l0_sfspi_info,
    stm32l0_sfspi_hook,
    stm32l0_sfspi_lock,
    stm32l0_sfspi_unlock,
    stm32l0_sfspi_erase,
    stm32l0_sfspi_program,
    stm32l0_sfspi_read,
};

bool stm32l0_sfspi_initialize(stm32l0_spi_t *spi, const stm32l0_sfspi_params_t *params)
{
    stm32l0_sfspi_t *sfspi = &stm32l0_sfspi;

    if (sfspi->state == STM32L0_SFSPI_STATE_NONE)
    {
        if (spi->state == STM32L0_SPI_STATE_NONE)
        {
            return false;
        }

        sfspi->spi = spi;
        sfspi->pin_cs = params->pin_cs;

        stm32l0_spi_enable(spi); // bump up refcount

        stm32l0_gpio_pin_configure(sfspi->pin_cs, (STM32L0_GPIO_PARK_PULLUP | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
        stm32l0_gpio_pin_write(sfspi->pin_cs, 1);

        stm32l0_sfspi_lock(sfspi);

        stm32l0_sfspi_select(sfspi);
        stm32l0_spi_data8(sfspi->spi, SFLASH_CMD_RDPD);
        stm32l0_sfspi_unselect(sfspi);

        armv6m_core_udelay(50);

        stm32l0_sfspi_select(sfspi);
        stm32l0_spi_data8(sfspi->spi, SFLASH_CMD_RDID);
        sfspi->ID[0] = stm32l0_spi_data8(sfspi->spi, 0xff);
        sfspi->ID[1] = stm32l0_spi_data8(sfspi->spi, 0xff);
        sfspi->ID[2] = stm32l0_spi_data8(sfspi->spi, 0xff);
        stm32l0_sfspi_unselect(sfspi);

        stm32l0_sfspi_unlock(sfspi);

        if (((sfspi->ID[0] == 0x00) && (sfspi->ID[1] == 0x00) && (sfspi->ID[2] == 0x00)) ||
            ((sfspi->ID[0] == 0xff) && (sfspi->ID[1] == 0xff) && (sfspi->ID[2] == 0xff)))
        {
            sfspi->ID[0] = 0x00;
            sfspi->ID[1] = 0x00;
            sfspi->ID[2] = 0x00;

            sfspi->state = STM32L0_SFSPI_STATE_INIT;
        }
        else
        {
            sfspi->state = STM32L0_SFSPI_STATE_READY;
            
            stm32l0_system_register(&stm32l0_sfspi.notify, stm32l0_sfspi_callback, (void*)&stm32l0_sfspi, STM32L0_SYSTEM_NOTIFY_SLEEP);

            dosfs_sflash_device.interface = &stm32l0_sfspi_interface;
            dosfs_sflash_device.context = sfspi;
        }
    }

    return (sfspi->state == STM32L0_SFSPI_STATE_READY);
}
