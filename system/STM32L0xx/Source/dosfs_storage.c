/*
 * Copyright (c) 2014-2020 Thomas Roell.  All rights reserved.
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

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "armv6m.h"
#include "stm32l0_rtc.h"
#include "stm32l0_usbd_msc.h"
#include "dosfs_core.h"
#include "dosfs_storage.h"

#define STANDARD_INQUIRY_DATA_LEN 0x24

static const uint8_t dosfs_storage_inquiry_data[] = {
  /* LUN 0 */
  0x00,         
  0x80,         
  0x02,         
  0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00,
  0x00, 
  0x00,
  'T', 'l', 'e', 'r', 'a', ' ', ' ', ' ', /* Manufacturer : 8 bytes */
  'D', 'O', 'S', 'F', 'S', ' ', ' ', ' ', /* Product      : 16 Bytes */
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  '0', '.', '0' ,'1',                     /* Version      : 4 Bytes */
}; 

static bool dosfs_storage_delay(void)
{
    uint64_t clock;

    static bool delay = true;

    if (delay)
    {
        clock = stm32l0_rtc_clock_read();

        if (clock >= (2 * STM32L0_RTC_CLOCK_TICKS_PER_SECOND))
        {
            delay = false;
        }
    }

    return delay;
}


static bool dosfs_storage_init(uint8_t **p_cache_data, const uint8_t **p_inquiry_data)
{
    int status;

    if (!dosfs_device.interface)
    {
        return false;
    }

    status = (*dosfs_device.interface->hook)(dosfs_device.context, (dosfs_device_lock_callback_t)USBD_MSC_Notify, NULL);

    if (status != F_NO_ERROR)
    {
        return false;
    }

    *p_cache_data = dosfs_device.cache;
    *p_inquiry_data = &dosfs_storage_inquiry_data[0];
    
    return true;
}

static bool dosfs_storage_deinit(void)
{
    int status;

    if (!dosfs_device.interface)
    {
        return false;
    }

    dosfs_device.lock &= ~(DOSFS_DEVICE_LOCK_ACCESSED | DOSFS_DEVICE_LOCK_SCSI | DOSFS_DEVICE_LOCK_MEDIUM);

    status = (*dosfs_device.interface->hook)(dosfs_device.context, NULL, NULL);

    if (status != F_NO_ERROR)
    {
        return false;
    }

    return true;
}

static bool dosfs_storage_is_ready(void)
{
    if (!dosfs_device.interface)
    {
        return false;
    }

    if ((dosfs_device.lock & (DOSFS_DEVICE_LOCK_INIT | DOSFS_DEVICE_LOCK_SFLASH | DOSFS_DEVICE_LOCK_VOLUME | DOSFS_DEVICE_LOCK_EJECTED)) || dosfs_storage_delay())
    {
        return false;
    }
    
    return true;
}

static bool dosfs_storage_get_capacity(uint32_t *p_block_count, uint32_t *p_block_size)
{
    int status;
    uint8_t media, write_protected;
    uint32_t block_count, au_size, serial;

    if (!dosfs_device.interface)
    {
        return false;
    }

    if ((dosfs_device.lock & (DOSFS_DEVICE_LOCK_INIT | DOSFS_DEVICE_LOCK_SFLASH | DOSFS_DEVICE_LOCK_VOLUME | DOSFS_DEVICE_LOCK_EJECTED)) || dosfs_storage_delay())
    {
        return false;
    }
    
    status = (*dosfs_device.interface->info)(dosfs_device.context, &media, &write_protected, &block_count, &au_size, &serial);

    if (status != F_NO_ERROR)
    {
        return false;
    }

    *p_block_count = block_count;
    *p_block_size = 512;

    return true;
}

static bool dosfs_storage_get_write_protected(bool *p_write_protected)
{
    int status;
    uint8_t media, write_protected;
    uint32_t block_count, au_size, serial;

    if (!dosfs_device.interface)
    {
        return false;
    }

    if ((dosfs_device.lock & (DOSFS_DEVICE_LOCK_INIT | DOSFS_DEVICE_LOCK_SFLASH | DOSFS_DEVICE_LOCK_VOLUME | DOSFS_DEVICE_LOCK_EJECTED)) || dosfs_storage_delay())
    {
        return false;
    }
    
    status = (*dosfs_device.interface->info)(dosfs_device.context, &media, &write_protected, &block_count, &au_size, &serial);

    if (status != F_NO_ERROR)
    {
        return false;
    }

    *p_write_protected = write_protected;
    
    return true;
}

static bool dosfs_storage_get_changed(bool *p_changed)
{
    bool changed;
    
    if (!dosfs_device.interface)
    {
        return false;
    }

    if ((dosfs_device.lock & (DOSFS_DEVICE_LOCK_INIT | DOSFS_DEVICE_LOCK_SFLASH | DOSFS_DEVICE_LOCK_VOLUME | DOSFS_DEVICE_LOCK_EJECTED)) || dosfs_storage_delay())
    {
        return false;
    }

    changed = false;
    
    if (dosfs_device.lock & DOSFS_DEVICE_LOCK_MODIFIED)
    {
        dosfs_device.lock &= ~DOSFS_DEVICE_LOCK_MODIFIED;

        if (dosfs_device.lock & DOSFS_DEVICE_LOCK_ACCESSED)
        {
            changed = true;
        }
    }

    *p_changed = changed;
    
    return true;
}

static bool dosfs_storage_start_stop_unit(bool start, bool loej)
{
    int status;

    if (!dosfs_device.interface)
    {
        return false;
    }

    if ((dosfs_device.lock & (DOSFS_DEVICE_LOCK_INIT | DOSFS_DEVICE_LOCK_SFLASH | DOSFS_DEVICE_LOCK_VOLUME | DOSFS_DEVICE_LOCK_EJECTED)) || dosfs_storage_delay())
    {
        return false;
    }

    if (!start)
    {
        status = (*dosfs_device.interface->sync)(dosfs_device.context);

        if (status != F_NO_ERROR)
        {
            return false;
        }
        
        if (loej)
        {
            dosfs_device.lock &= ~(DOSFS_DEVICE_LOCK_ACCESSED | DOSFS_DEVICE_LOCK_SCSI | DOSFS_DEVICE_LOCK_MEDIUM);
            dosfs_device.lock |= DOSFS_DEVICE_LOCK_EJECTED;
        }
    }

    return true;
}

static bool dosfs_storage_prevent_allow_medium_removal(bool prevent)
{
    if (!dosfs_device.interface)
    {
        return false;
    }

    if ((dosfs_device.lock & (DOSFS_DEVICE_LOCK_INIT | DOSFS_DEVICE_LOCK_SFLASH | DOSFS_DEVICE_LOCK_VOLUME | DOSFS_DEVICE_LOCK_EJECTED)) || dosfs_storage_delay())
    {
        return false;
    }

    if (prevent)
    {
        dosfs_device.lock |= DOSFS_DEVICE_LOCK_MEDIUM;
    }
    else
    {
        dosfs_device.lock &= ~DOSFS_DEVICE_LOCK_MEDIUM;
    }

    return true;
    
}

static bool dosfs_storage_acquire(void)
{
    if ((dosfs_device.lock & (DOSFS_DEVICE_LOCK_INIT | DOSFS_DEVICE_LOCK_SFLASH | DOSFS_DEVICE_LOCK_VOLUME | DOSFS_DEVICE_LOCK_EJECTED)) || dosfs_storage_delay())
    {
        return false;
    }
    
    dosfs_device.lock |= DOSFS_DEVICE_LOCK_SCSI;
    
    return true;
}

static void dosfs_storage_release(void)
{
    if ((dosfs_device.lock & (DOSFS_DEVICE_LOCK_INIT | DOSFS_DEVICE_LOCK_SFLASH | DOSFS_DEVICE_LOCK_VOLUME | DOSFS_DEVICE_LOCK_EJECTED)) || dosfs_storage_delay())
    {
        return;
    }
    
    dosfs_device.lock |= DOSFS_DEVICE_LOCK_ACCESSED;

    dosfs_device.lock &= ~DOSFS_DEVICE_LOCK_SCSI;
}

static bool dosfs_storage_read(uint8_t *data, uint32_t blk_address, uint32_t blk_length, bool release)
{
    int status;

    status = (*dosfs_device.interface->read)(dosfs_device.context, blk_address, data, blk_length, true);

    if (status != F_NO_ERROR)
    {
        dosfs_device.lock &= ~DOSFS_DEVICE_LOCK_SCSI;

        return false;
    }

    dosfs_device.lock |= DOSFS_DEVICE_LOCK_ACCESSED;

    if (release)
    {
        dosfs_device.lock &= ~DOSFS_DEVICE_LOCK_SCSI;
    }

    return true;
}

static bool dosfs_storage_write(const uint8_t *data, uint32_t blk_address, uint32_t blk_length, bool release)
{
    int status;

    status = (*dosfs_device.interface->write)(dosfs_device.context, blk_address, data, blk_length, false);

    if (status != F_NO_ERROR)
    {
        dosfs_device.lock &= ~DOSFS_DEVICE_LOCK_SCSI;

        return false;
    }

    dosfs_device.lock |= DOSFS_DEVICE_LOCK_ACCESSED;

    if (release)
    {
        dosfs_device.lock &= ~DOSFS_DEVICE_LOCK_SCSI;
    }

    return true;
}

const dosfs_storage_interface_t dosfs_storage_interface = {
    dosfs_storage_init,
    dosfs_storage_deinit,
    dosfs_storage_is_ready,
    dosfs_storage_get_capacity,
    dosfs_storage_get_write_protected,
    dosfs_storage_get_changed,
    dosfs_storage_start_stop_unit,
    dosfs_storage_prevent_allow_medium_removal,
    dosfs_storage_acquire,
    dosfs_storage_release,
    dosfs_storage_read,
    dosfs_storage_write,
};
