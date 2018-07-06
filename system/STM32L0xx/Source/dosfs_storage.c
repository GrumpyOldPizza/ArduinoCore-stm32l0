/*
 * Copyright (c) 2014-2018 Thomas Roell.  All rights reserved.
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
#include "dosfs_core.h"
#include "usbd_msc.h"
#include "stm32l0_rtc.h"

static bool dosfs_storage_delay(void)
{
    uint32_t seconds;
    uint16_t subseconds;

    static bool delay = true;

    if (delay)
    {
        stm32l0_rtc_get_time(&seconds, &subseconds);

        if (seconds >= 2)
        {
            delay = false;
        }
    }

    return delay;
}


static int8_t dosfs_storage_init(uint8_t lun)
{
    int status;

    if (!dosfs_device.interface)
    {
        return -1;
    }

    status = (*dosfs_device.interface->notify)(dosfs_device.context, (dosfs_device_notify_callback_t)USBD_MSC_Notify, (void*)lun);

    if (status != F_NO_ERROR)
    {
        return -1;
    }

    return 0;
}

static int8_t dosfs_storage_deinit(uint8_t lun)
{
    int status;

    if (!dosfs_device.interface)
    {
        return -1;
    }

    dosfs_device.lock &= ~(DOSFS_DEVICE_LOCK_ACCESSED | DOSFS_DEVICE_LOCK_SCSI | DOSFS_DEVICE_LOCK_MEDIUM);

    status = (*dosfs_device.interface->notify)(dosfs_device.context, NULL, NULL);

    if (status != F_NO_ERROR)
    {
        return -1;
    }

    return 0;
}

static int8_t dosfs_storage_get_capacity(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
    int status;
    uint8_t media, write_protected;
    uint32_t block_count, au_size, serial;

    if (!dosfs_device.interface)
    {
        return -1;
    }

    if ((dosfs_device.lock & (DOSFS_DEVICE_LOCK_INIT | DOSFS_DEVICE_LOCK_SFLASH | DOSFS_DEVICE_LOCK_VOLUME | DOSFS_DEVICE_LOCK_EJECTED)) || dosfs_storage_delay())
    {
        return -1;
    }
    
    status = (*dosfs_device.interface->info)(dosfs_device.context, &media, &write_protected, &block_count, &au_size, &serial);

    if (status != F_NO_ERROR)
    {
        return -1;
    }

    *block_num  = block_count;
    *block_size = 512;

    return 0;
}

static int8_t dosfs_storage_is_ready(uint8_t lun)
{
    if (!dosfs_device.interface)
    {
        return -1;
    }

    if ((dosfs_device.lock & (DOSFS_DEVICE_LOCK_INIT | DOSFS_DEVICE_LOCK_SFLASH | DOSFS_DEVICE_LOCK_VOLUME | DOSFS_DEVICE_LOCK_EJECTED)) || dosfs_storage_delay())
    {
        return -1;
    }
    
    return 0;
}

static int8_t dosfs_storage_is_write_protected(uint8_t lun)
{
    return 0;
}

static int8_t dosfs_storage_is_changed(uint8_t lun)
{
    if (dosfs_device.lock & DOSFS_DEVICE_LOCK_MODIFIED)
    {
        dosfs_device.lock &= ~DOSFS_DEVICE_LOCK_MODIFIED;

        if (dosfs_device.lock & DOSFS_DEVICE_LOCK_ACCESSED)
        {
            return -1;
        }
    }
    
    return 0;
}

static int8_t dosfs_storage_start_stop_unit(uint8_t lun, uint8_t start, uint8_t loej)
{
    if (!dosfs_device.interface)
    {
        return -1;
    }

    if ((dosfs_device.lock & (DOSFS_DEVICE_LOCK_INIT | DOSFS_DEVICE_LOCK_SFLASH | DOSFS_DEVICE_LOCK_VOLUME | DOSFS_DEVICE_LOCK_EJECTED)) || dosfs_storage_delay())
    {
        return -1;
    }

    if (!start && loej)
    {
        dosfs_device.lock &= ~(DOSFS_DEVICE_LOCK_ACCESSED | DOSFS_DEVICE_LOCK_SCSI | DOSFS_DEVICE_LOCK_MEDIUM);
        dosfs_device.lock |= DOSFS_DEVICE_LOCK_EJECTED;
    }

    return 0;
}

static int8_t dosfs_storage_prevent_allow_medium_removal(uint8_t lun, uint8_t param)
{
    if (!dosfs_device.interface)
    {
        return -1;
    }

    if ((dosfs_device.lock & (DOSFS_DEVICE_LOCK_INIT | DOSFS_DEVICE_LOCK_SFLASH | DOSFS_DEVICE_LOCK_VOLUME | DOSFS_DEVICE_LOCK_EJECTED)) || dosfs_storage_delay())
    {
        return -1;
    }

    if (param & 1)
    {
        dosfs_device.lock |= DOSFS_DEVICE_LOCK_MEDIUM;
    }
    else
    {
        dosfs_device.lock &= ~DOSFS_DEVICE_LOCK_MEDIUM;
    }

    return 0;
    
}

static int8_t dosfs_storage_acquire(uint8_t lun)
{
    if ((dosfs_device.lock & (DOSFS_DEVICE_LOCK_INIT | DOSFS_DEVICE_LOCK_SFLASH | DOSFS_DEVICE_LOCK_VOLUME | DOSFS_DEVICE_LOCK_EJECTED)) || dosfs_storage_delay())
    {
        return -1;
    }
    
    dosfs_device.lock |= DOSFS_DEVICE_LOCK_SCSI;
    
    return 0;
}

static int8_t dosfs_storage_read(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len, uint8_t last)
{
    int status;

    status = (*dosfs_device.interface->read)(dosfs_device.context, blk_addr, buf, blk_len, false);

    if (status != F_NO_ERROR)
    {
        dosfs_device.lock &= ~DOSFS_DEVICE_LOCK_SCSI;

        return -1;
    }

    if (last)
    {
        dosfs_device.lock &= ~DOSFS_DEVICE_LOCK_SCSI;
    }

    dosfs_device.lock |= DOSFS_DEVICE_LOCK_ACCESSED;

    return 0;
}

static int8_t dosfs_storage_write(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len, uint8_t last)
{
    int status;

    status = (*dosfs_device.interface->write)(dosfs_device.context, blk_addr, buf, blk_len, true);

    if (status != F_NO_ERROR)
    {
        dosfs_device.lock &= ~DOSFS_DEVICE_LOCK_SCSI;

        return -1;
    }

    if (last)
    {
        dosfs_device.lock &= ~DOSFS_DEVICE_LOCK_SCSI;
    }

    dosfs_device.lock |= DOSFS_DEVICE_LOCK_ACCESSED;

    return 0;
}


static int8_t dosfs_storage_get_maxlun(void)
{
    return 0;
}

static const int8_t dosfs_storage_inquiry_data[] = {
  
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

const USBD_StorageTypeDef dosfs_storage_interface =
{
    dosfs_storage_init,
    dosfs_storage_deinit,
    dosfs_storage_get_capacity,
    dosfs_storage_is_ready,
    dosfs_storage_is_write_protected,
    dosfs_storage_is_changed,
    dosfs_storage_start_stop_unit,
    dosfs_storage_prevent_allow_medium_removal,
    dosfs_storage_acquire,
    dosfs_storage_read,
    dosfs_storage_write,
    dosfs_storage_get_maxlun,
    dosfs_storage_inquiry_data,
};
