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

#include "dosfs_sflash.h"

#include <stdio.h>

#if (DOSFS_CONFIG_SFLASH_DEBUG == 1)
#include <assert.h>
static void dosfs_sflash_ftl_check();
#endif /* DOSFS_CONFIG_SFLASH_DEBUG == 1 */

dosfs_sflash_t dosfs_sflash;

static __attribute__((section(".noinit"))) uint32_t dosfs_sflash_cache[2 * (DOSFS_SFLASH_BLOCK_SIZE / sizeof(uint32_t))];

#if (DOSFS_CONFIG_SFLASH_DEBUG == 1)
static uint8_t sflash_data_shadow[DOSFS_SFLASH_DATA_SIZE]; /* 16 MB */
static uint16_t sflash_xlate_shadow[DOSFS_SFLASH_DATA_SIZE / DOSFS_SFLASH_BLOCK_SIZE];
#endif /* DOSFS_CONFIG_SFLASH_DEBUG == 1 */

dosfs_sflash_device_t dosfs_sflash_device;

static inline uint32_t dosfs_sflash_ftl_sector_lookup(dosfs_sflash_t *sflash, uint32_t index)
{
    return sflash->sector_table[index];
}

static inline void dosfs_sflash_ftl_sector_assign(dosfs_sflash_t *sflash, uint32_t index, uint32_t sector)
{
    sflash->sector_table[index] = sector;
}

static uint32_t dosfs_sflash_ftl_translate(dosfs_sflash_t *sflash, uint32_t logical)
{
    return ((dosfs_sflash_ftl_sector_lookup(sflash, (logical >> DOSFS_SFLASH_LOGICAL_SECTOR_SHIFT)) << DOSFS_SFLASH_LOGICAL_SECTOR_SHIFT) + (logical & DOSFS_SFLASH_LOGICAL_BLOCK_MASK)) * DOSFS_SFLASH_BLOCK_SIZE;
}

static uint32_t dosfs_sflash_ftl_extract_info_entry(const void *cache, uint32_t index)
{
    uint32_t offset = (DOSFS_SFLASH_INFO_ENTRY_OFFSET + (index * DOSFS_SFLASH_INFO_ENTRY_SIZE));

    return ((((const uint8_t*)cache)[offset+0] << 0) |
            (((const uint8_t*)cache)[offset+1] << 8) |
            (((const uint8_t*)cache)[offset+2] << 16));
}

static void dosfs_sflash_ftl_merge_info_type(void *cache, uint32_t index, uint32_t info)
{
    uint32_t offset = (DOSFS_SFLASH_INFO_ENTRY_OFFSET + (index * DOSFS_SFLASH_INFO_ENTRY_SIZE));

    ((uint8_t*)cache)[offset+2] = (info >> 16);
}

static void dosfs_sflash_ftl_merge_info_entry(void *cache, uint32_t index, uint32_t info)
{
    uint32_t offset = (DOSFS_SFLASH_INFO_ENTRY_OFFSET + (index * DOSFS_SFLASH_INFO_ENTRY_SIZE));

    ((uint8_t*)cache)[offset+0] = (info >> 0);
    ((uint8_t*)cache)[offset+1] = (info >> 8);
    ((uint8_t*)cache)[offset+2] = (info >> 16);
}

static void dosfs_sflash_ftl_set_info_type(dosfs_sflash_t *sflash, uint32_t logical, uint32_t info)
{
    (*sflash->interface->program)(sflash->context,
                                  (dosfs_sflash_ftl_translate(sflash, logical) & ~(DOSFS_SFLASH_ERASE_SIZE-1)) +
                                  DOSFS_SFLASH_INFO_ENTRY_OFFSET +
                                  ((logical & DOSFS_SFLASH_LOGICAL_BLOCK_MASK) * DOSFS_SFLASH_INFO_ENTRY_SIZE) +2,
                                  (const uint8_t*)&info + 2,
                                  1);
}

static void dosfs_sflash_ftl_set_info_entry(dosfs_sflash_t *sflash, uint32_t logical, uint32_t info)
{
    (*sflash->interface->program)(sflash->context,
                                  (dosfs_sflash_ftl_translate(sflash, logical) & ~(DOSFS_SFLASH_ERASE_SIZE-1)) +
                                  DOSFS_SFLASH_INFO_ENTRY_OFFSET +
                                  ((logical & DOSFS_SFLASH_LOGICAL_BLOCK_MASK) * DOSFS_SFLASH_INFO_ENTRY_SIZE),
                                  (const uint8_t*)&info,
                                  DOSFS_SFLASH_INFO_ENTRY_SIZE);
}
    
static void dosfs_sflash_ftl_swap(dosfs_sflash_t *sflash, uint32_t victim_offset, uint32_t victim_sector, uint32_t victim_erase_count, uint32_t reclaim_offset)
{
    uint32_t info_entry, head_info[DOSFS_SFLASH_INFO_SLOT_COUNT], tail_info[1];

    (*sflash->interface->erase)(sflash->context, victim_offset);
    
    info_entry = DOSFS_SFLASH_INFO_TYPE_RESERVED | DOSFS_SFLASH_INFO_DATA_MASK;

    (*sflash->interface->program)(sflash->context, victim_offset + DOSFS_SFLASH_INFO_ENTRY_OFFSET, (const uint8_t*)&info_entry, DOSFS_SFLASH_INFO_ENTRY_SIZE);

    head_info[0] = DOSFS_SFLASH_SECTOR_IDENT_0;
    head_info[1] = DOSFS_SFLASH_SECTOR_IDENT_1;
    head_info[2] = sflash->data_start;
    head_info[3] = sflash->data_limit;
    head_info[4] = victim_erase_count;
    head_info[5] = 0xffffffff;
    head_info[6] = 0xffffffff;
    head_info[7] = 0xffffffff;

    (*sflash->interface->program)(sflash->context, victim_offset + DOSFS_SFLASH_INFO_HEAD_OFFSET, (const uint8_t*)&head_info[0], DOSFS_SFLASH_INFO_HEAD_SIZE);

    tail_info[0] = DOSFS_SFLASH_SECTOR_IDENT_TAIL;

    (*sflash->interface->program)(sflash->context, victim_offset + DOSFS_SFLASH_INFO_TAIL_OFFSET, (const uint8_t*)&tail_info[0], DOSFS_SFLASH_INFO_TAIL_SIZE);

    /* Now the old VICTIM has been erased, so just flip the RECLAIM sector into an ERASE sector (RECLAIM -> ERASE).
     */

    dosfs_sflash_ftl_sector_assign(sflash, victim_sector, (reclaim_offset / DOSFS_SFLASH_ERASE_SIZE));

    dosfs_sflash_ftl_set_info_entry(sflash, (victim_sector << DOSFS_SFLASH_LOGICAL_SECTOR_SHIFT), (DOSFS_SFLASH_INFO_TYPE_ERASE | victim_sector));

    sflash->reclaim_offset = victim_offset;
    sflash->reclaim_erase_count = victim_erase_count;
}

static void dosfs_sflash_ftl_reclaim(dosfs_sflash_t *sflash, uint32_t victim_offset)
{
    uint32_t index, victim_sector, victim_erase_count, info_entry;
    uint32_t *cache, *data, *head_info, *tail_info;

    // printf("RECLAIM %08x\n", victim_offset);

    /* Mark the reclaim erase unit as reclaim type so that upon
     * a crash the data can be recovered.
     *
     * N.b. that during the copy operation the reclaim_victim
     * and reclaim_erase_count will be 0xffffffff.
     */
    
    cache = sflash->cache[0];
    data = sflash->cache[1];

    head_info = &cache[DOSFS_SFLASH_INFO_HEAD_OFFSET / sizeof(uint32_t)];
    tail_info = &cache[DOSFS_SFLASH_INFO_TAIL_OFFSET / sizeof(uint32_t)];

    sflash->xlate_logical = DOSFS_SFLASH_BLOCK_RESERVED;
    sflash->xlate2_logical = DOSFS_SFLASH_BLOCK_RESERVED;
    
    (*sflash->interface->read)(sflash->context, victim_offset, (uint8_t*)cache, DOSFS_SFLASH_BLOCK_SIZE);

    info_entry = dosfs_sflash_ftl_extract_info_entry(cache, 0);

    victim_sector = info_entry & DOSFS_SFLASH_INFO_DATA_MASK;
    victim_erase_count = head_info[DOSFS_SFLASH_INFO_SLOT_ERASE_COUNT];

    sflash->alloc_sector  = victim_sector;
    sflash->alloc_count   = 0;
    sflash->alloc_index   = 0;
    sflash->alloc_mask[0] = 0;
    sflash->alloc_mask[1] = 0;
    sflash->alloc_mask[2] = 0;
    sflash->alloc_mask[3] = 0;

    for (index = 1; index < DOSFS_SFLASH_BLOCK_INFO_ENTRIES; index++)
    {
        info_entry = dosfs_sflash_ftl_extract_info_entry(cache, index);

        if ((info_entry & DOSFS_SFLASH_INFO_NOT_WRITTEN_TO) || ((info_entry & DOSFS_SFLASH_INFO_TYPE_MASK) == DOSFS_SFLASH_INFO_TYPE_DELETED))
        {
            if (sflash->alloc_count == 0)
            {
                sflash->alloc_index = index;
            }
            
            sflash->alloc_mask[index / 32] |= (1ul << (index & 31));
            sflash->alloc_count++;

            if ((info_entry & DOSFS_SFLASH_INFO_TYPE_MASK) == DOSFS_SFLASH_INFO_TYPE_DELETED)
            {
                dosfs_sflash_ftl_merge_info_entry(cache, index, DOSFS_SFLASH_INFO_ERASED);
            }
        }
    }

    if (sflash->alloc_count || ((sflash->erase_count_max - victim_erase_count) > DOSFS_SFLASH_ERASE_COUNT_THRESHOLD))
    {
        /* Mark the victim sector as VICTIM and record the "reclaim_offset" (ERASE -> VICTIM).
         */

        info_entry = dosfs_sflash_ftl_extract_info_entry(cache, 0);
        
        info_entry &= ~DOSFS_SFLASH_INFO_TYPE_VICTIM;
        
        dosfs_sflash_ftl_merge_info_type(cache, 0, info_entry);
        
        (*sflash->interface->program)(sflash->context, victim_offset + DOSFS_SFLASH_INFO_ENTRY_OFFSET, (const uint8_t*)&info_entry, DOSFS_SFLASH_INFO_ENTRY_SIZE);

        head_info[0] = DOSFS_SFLASH_SECTOR_IDENT_0;
        head_info[1] = DOSFS_SFLASH_SECTOR_IDENT_1;
        head_info[2] = sflash->data_start;
        head_info[3] = sflash->data_limit;
        head_info[4] = victim_erase_count;
        head_info[5] = sflash->reclaim_erase_count + 1;
        head_info[6] = 0xffffffff;
        head_info[7] = 0xffffffff;
        
        (*sflash->interface->program)(sflash->context, victim_offset + DOSFS_SFLASH_INFO_HEAD_OFFSET, (const uint8_t*)&head_info[0], DOSFS_SFLASH_INFO_HEAD_SIZE);
        
        /* Here the reclaim sector stays in uncommitted state.
         */


        for (index = 1; index < DOSFS_SFLASH_BLOCK_INFO_ENTRIES; index++)
        {
            info_entry = dosfs_sflash_ftl_extract_info_entry(cache, index);

            if (!((info_entry & DOSFS_SFLASH_INFO_NOT_WRITTEN_TO) || ((info_entry & DOSFS_SFLASH_INFO_TYPE_MASK) == DOSFS_SFLASH_INFO_TYPE_DELETED)))
            {
                (*sflash->interface->read)(sflash->context, victim_offset + (index * DOSFS_SFLASH_BLOCK_SIZE), (uint8_t*)data, DOSFS_SFLASH_BLOCK_SIZE);

                (*sflash->interface->program)(sflash->context, sflash->reclaim_offset + (index * DOSFS_SFLASH_BLOCK_SIZE), (const uint8_t*)data, DOSFS_SFLASH_PAGE_SIZE);
                (*sflash->interface->program)(sflash->context, sflash->reclaim_offset + (index * DOSFS_SFLASH_BLOCK_SIZE) + DOSFS_SFLASH_PAGE_SIZE, (const uint8_t*)data + DOSFS_SFLASH_PAGE_SIZE, DOSFS_SFLASH_PAGE_SIZE);
            }
        }

        victim_erase_count++;

        /* Write the update info header to the reclaim sector (2nd page first). This commits the reclaim operation (UNCOMITTED -> RECLAIM).
         */
        
        info_entry = DOSFS_SFLASH_INFO_TYPE_RECLAIM | DOSFS_SFLASH_INFO_DATA_MASK;

        dosfs_sflash_ftl_merge_info_entry(cache, 0, info_entry);

        head_info[0] = DOSFS_SFLASH_SECTOR_IDENT_0;
        head_info[1] = DOSFS_SFLASH_SECTOR_IDENT_1;
        head_info[2] = sflash->data_start;
        head_info[3] = sflash->data_limit;
        head_info[4] = sflash->reclaim_erase_count;
        head_info[5] = 0xffffffff;
        head_info[6] = 0xffffffff;
        head_info[7] = 0xffffffff;

        tail_info[0] = DOSFS_SFLASH_SECTOR_IDENT_TAIL;

        (*sflash->interface->program)(sflash->context, sflash->reclaim_offset + DOSFS_SFLASH_PAGE_SIZE, (const uint8_t*)cache + DOSFS_SFLASH_PAGE_SIZE, DOSFS_SFLASH_PAGE_SIZE);
        (*sflash->interface->program)(sflash->context, sflash->reclaim_offset, (const uint8_t*)cache, DOSFS_SFLASH_PAGE_SIZE);

        /* Here a valid VICTIM and a valid RECLAIM header exist. Erase the victim and set it up as new reclaim.
         */
        
        dosfs_sflash_ftl_swap(sflash, victim_offset, victim_sector, victim_erase_count, sflash->reclaim_offset);

        if (sflash->erase_count_max < victim_erase_count)
        {
            sflash->erase_count_max = victim_erase_count;
        }
    }

    DOSFS_SFLASH_STATISTICS_COUNT(sflash_ftl_reclaim);

#if (DOSFS_CONFIG_SFLASH_DEBUG == 1)
    dosfs_sflash_ftl_check();
#endif
}


static void dosfs_sflash_ftl_format(dosfs_sflash_t *sflash)
{
    uint32_t offset, erase_count, info_entry;
    uint32_t *cache, *head_info, *tail_info;

    cache = sflash->cache[0];
    head_info = &cache[DOSFS_SFLASH_INFO_HEAD_OFFSET / sizeof(uint32_t)];
    tail_info = &cache[DOSFS_SFLASH_INFO_TAIL_OFFSET / sizeof(uint32_t)];

    sflash->xlate_logical = DOSFS_SFLASH_BLOCK_RESERVED;

    for (offset = sflash->data_start; offset < sflash->data_limit; offset += DOSFS_SFLASH_ERASE_SIZE)
    {
        erase_count = 1;

        (*sflash->interface->read)(sflash->context, offset, (uint8_t*)cache, DOSFS_SFLASH_BLOCK_SIZE);

        info_entry = dosfs_sflash_ftl_extract_info_entry(cache, 0);

        if (!(info_entry & DOSFS_SFLASH_INFO_NOT_WRITTEN_TO))
        {
            if ((head_info[0] == DOSFS_SFLASH_SECTOR_IDENT_0) &&
                (head_info[1] == DOSFS_SFLASH_SECTOR_IDENT_1) &&
                (tail_info[0] == DOSFS_SFLASH_SECTOR_IDENT_TAIL))
            {
                erase_count = head_info[DOSFS_SFLASH_INFO_SLOT_ERASE_COUNT] + 1;
            }
        }

        (*sflash->interface->erase)(sflash->context, offset);

        if (offset == (sflash->data_limit - DOSFS_SFLASH_ERASE_SIZE))
        {
            info_entry = DOSFS_SFLASH_INFO_TYPE_RESERVED | DOSFS_SFLASH_INFO_DATA_MASK;
        }
        else
        {
            info_entry = DOSFS_SFLASH_INFO_TYPE_ERASE | (offset / DOSFS_SFLASH_ERASE_SIZE);
        }

        (*sflash->interface->program)(sflash->context, offset + DOSFS_SFLASH_INFO_ENTRY_OFFSET, (const uint8_t*)&info_entry, DOSFS_SFLASH_INFO_ENTRY_SIZE);

        head_info[0] = DOSFS_SFLASH_SECTOR_IDENT_0;
        head_info[1] = DOSFS_SFLASH_SECTOR_IDENT_1;
        head_info[2] = sflash->data_start;
        head_info[3] = sflash->data_limit;
        head_info[4] = erase_count;
        head_info[5] = 0xffffffff;
        head_info[6] = 0xffffffff;
        head_info[7] = 0xffffffff;

        (*sflash->interface->program)(sflash->context, offset + DOSFS_SFLASH_INFO_HEAD_OFFSET, (const uint8_t*)&head_info[0], DOSFS_SFLASH_INFO_HEAD_SIZE);

        tail_info[0] = DOSFS_SFLASH_SECTOR_IDENT_TAIL;

        (*sflash->interface->program)(sflash->context, offset + DOSFS_SFLASH_INFO_TAIL_OFFSET, (const uint8_t*)&tail_info[0], DOSFS_SFLASH_INFO_TAIL_SIZE);
    }

#if (DOSFS_CONFIG_SFLASH_DEBUG == 1)
    memset(&sflash_data_shadow, 0xff, sizeof(sflash_data_shadow));
    memset(&sflash_xlate_shadow, 0xff, sizeof(sflash_xlate_shadow));
#endif /* DOSFS_CONFIG_SFLASH_DEBUG == 1 */
}

/* Modify XLATE/XLATE_SECONDARY mappings. Assumption is that XLATE/XLATE_SECONDARY already
 * have been setup to have a slot that can be written. Also there is a DATA_WRITTEN block
 * that contains valid data.
 */

static void dosfs_sflash_ftl_modify(dosfs_sflash_t *sflash, uint32_t address, uint32_t write_logical)
{
    uint32_t xlate_segment, xlate_index, read_logical;
    uint16_t *xlate_cache, *xlate2_cache;

    xlate_cache   = (uint16_t*)sflash->cache[0];
    xlate2_cache  = (uint16_t*)sflash->cache[1];

    xlate_segment = (address - DOSFS_SFLASH_XLATE_OFFSET) >> DOSFS_SFLASH_XLATE_SEGMENT_SHIFT;
    xlate_index   = (address - DOSFS_SFLASH_XLATE_OFFSET) & DOSFS_SFLASH_XLATE_INDEX_MASK;

    if (sflash->xlate_logical != sflash->xlate_table[xlate_segment])
    {
        DOSFS_SFLASH_STATISTICS_COUNT(sflash_ftl_xlate_miss);

        sflash->xlate_logical = sflash->xlate_table[xlate_segment];
        
        (*sflash->interface->read)(sflash->context, dosfs_sflash_ftl_translate(sflash, sflash->xlate_logical), (uint8_t*)xlate_cache, DOSFS_SFLASH_BLOCK_SIZE);
    }
    else
    {
        DOSFS_SFLASH_STATISTICS_COUNT(sflash_ftl_xlate_hit);
    }

    read_logical = xlate_cache[xlate_index];
    
    if (read_logical == DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
    {
#if (DOSFS_CONFIG_SFLASH_DEBUG == 1)
        assert(read_logical == sflash_xlate_shadow[address]);
        sflash_xlate_shadow[address] = write_logical;
#endif /* DOSFS_CONFIG_SFLASH_DEBUG == 1 */
        
        xlate_cache[xlate_index] = write_logical;
        
        (*sflash->interface->program)(sflash->context, dosfs_sflash_ftl_translate(sflash, sflash->xlate_logical) + (xlate_index * 2), (const uint8_t*)&xlate_cache[xlate_index], 2);
        
        dosfs_sflash_ftl_set_info_type(sflash, write_logical, (DOSFS_SFLASH_INFO_TYPE_DATA_COMMITTED | address));
    }
    else
    {
        if (sflash->xlate2_logical != sflash->xlate2_table[xlate_segment])
        {
            DOSFS_SFLASH_STATISTICS_COUNT(sflash_ftl_xlate2_miss);

            sflash->xlate2_logical = sflash->xlate2_table[xlate_segment];
            
            (*sflash->interface->read)(sflash->context, dosfs_sflash_ftl_translate(sflash, sflash->xlate2_logical), (uint8_t*)xlate2_cache, DOSFS_SFLASH_BLOCK_SIZE);
        }
        else
        {
            DOSFS_SFLASH_STATISTICS_COUNT(sflash_ftl_xlate2_hit);
        }

#if (DOSFS_CONFIG_SFLASH_DEBUG == 1)
        assert(read_logical == sflash_xlate_shadow[address]);
        sflash_xlate_shadow[address] = write_logical;
#endif /* DOSFS_CONFIG_SFLASH_DEBUG == 1 */
        
        xlate2_cache[xlate_index] = write_logical;
        
        (*sflash->interface->program)(sflash->context, dosfs_sflash_ftl_translate(sflash, sflash->xlate2_logical) + (xlate_index * 2), (const uint8_t*)&xlate2_cache[xlate_index], 2);
        
        dosfs_sflash_ftl_set_info_type(sflash, write_logical, (DOSFS_SFLASH_INFO_TYPE_DATA_COMMITTED | address));
        
        if (read_logical != DOSFS_SFLASH_BLOCK_DELETED)
        {
            xlate_cache[xlate_index] = DOSFS_SFLASH_BLOCK_DELETED;
            
            (*sflash->interface->program)(sflash->context, dosfs_sflash_ftl_translate(sflash, sflash->xlate_logical) + (xlate_index * 2), (const uint8_t*)&xlate_cache[xlate_index], 2);
            
            dosfs_sflash_ftl_set_info_entry(sflash, read_logical, DOSFS_SFLASH_INFO_TYPE_DELETED);
        }
    }
}

static void dosfs_sflash_ftl_trim(dosfs_sflash_t *sflash, uint32_t address, uint32_t delete_logical)
{
    uint32_t xlate_segment, xlate_index;
    uint16_t *xlate_cache, *xlate2_cache;

    xlate_cache   = (uint16_t*)sflash->cache[0];
    xlate2_cache  = (uint16_t*)sflash->cache[1];

    xlate_segment = (address - DOSFS_SFLASH_XLATE_OFFSET) >> DOSFS_SFLASH_XLATE_SEGMENT_SHIFT;
    xlate_index   = (address - DOSFS_SFLASH_XLATE_OFFSET) & DOSFS_SFLASH_XLATE_INDEX_MASK;

    if (sflash->xlate_logical != sflash->xlate_table[xlate_segment])
    {
        DOSFS_SFLASH_STATISTICS_COUNT(sflash_ftl_xlate_miss);

        sflash->xlate_logical = sflash->xlate_table[xlate_segment];
        
        (*sflash->interface->read)(sflash->context, dosfs_sflash_ftl_translate(sflash, sflash->xlate_logical), (uint8_t*)xlate_cache, DOSFS_SFLASH_BLOCK_SIZE);
    }
    else
    {
        DOSFS_SFLASH_STATISTICS_COUNT(sflash_ftl_xlate_hit);;
    }

    if (xlate_cache[xlate_index] != DOSFS_SFLASH_BLOCK_DELETED)
    {
#if (DOSFS_CONFIG_SFLASH_DEBUG == 1)
        sflash_xlate_shadow[address] = DOSFS_SFLASH_BLOCK_DELETED;
#endif /* DOSFS_CONFIG_SFLASH_DEBUG == 1 */

        xlate_cache[xlate_index] = DOSFS_SFLASH_BLOCK_DELETED;
                                
        (*sflash->interface->program)(sflash->context, dosfs_sflash_ftl_translate(sflash, sflash->xlate_logical) + (xlate_index * 2), (const uint8_t*)&xlate_cache[xlate_index], 2);
    }
    else
    {
        if (sflash->xlate2_table[xlate_segment] != DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
        {
            if (sflash->xlate2_logical != sflash->xlate2_table[xlate_segment])
            {
                DOSFS_SFLASH_STATISTICS_COUNT(sflash_ftl_xlate2_miss);

                sflash->xlate2_logical = sflash->xlate2_table[xlate_segment];
                
                (*sflash->interface->read)(sflash->context, dosfs_sflash_ftl_translate(sflash, sflash->xlate2_logical), (uint8_t*)xlate2_cache, DOSFS_SFLASH_BLOCK_SIZE);
            }
            else
            {
                DOSFS_SFLASH_STATISTICS_COUNT(sflash_ftl_xlate2_hit);
            }

#if (DOSFS_CONFIG_SFLASH_DEBUG == 1)
            sflash_xlate_shadow[address] = DOSFS_SFLASH_BLOCK_DELETED;
#endif /* DOSFS_CONFIG_SFLASH_DEBUG == 1 */

            xlate2_cache[xlate_index] = DOSFS_SFLASH_BLOCK_DELETED;
            
            (*sflash->interface->program)(sflash->context, dosfs_sflash_ftl_translate(sflash, sflash->xlate2_logical) + (xlate_index * 2), (const uint8_t*)&xlate2_cache[xlate_index], 2);
        }
    }

    dosfs_sflash_ftl_set_info_entry(sflash, delete_logical, DOSFS_SFLASH_INFO_TYPE_DELETED);
}

#if (DOSFS_CONFIG_SFLASH_DEBUG == 1)

//#define DOSFS_SFLASH_FTL_CHECK() while (1) { }
#define DOSFS_SFLASH_FTL_CHECK() assert(0);

static void dosfs_sflash_ftl_check()
{
    dosfs_sflash_t *sflash = &dosfs_sflash;
    uint32_t offset, index, sector, address, info_logical, cache_logical, info_entry;
    uint32_t cache[512 / 4], *head_info, *tail_info;
    uint32_t read_logical, xlate_segment, xlate_index;
    uint16_t *xlate_cache, *xlate2_cache;

    head_info = &cache[DOSFS_SFLASH_INFO_HEAD_OFFSET / sizeof(uint32_t)];
    tail_info = &cache[DOSFS_SFLASH_INFO_TAIL_OFFSET / sizeof(uint32_t)];

    for (offset = sflash->data_start; offset < sflash->data_limit; offset += DOSFS_SFLASH_ERASE_SIZE)
    {
        (*sflash->interface->read)(sflash->context, offset, (uint8_t*)cache, DOSFS_SFLASH_BLOCK_SIZE);

        info_entry = dosfs_sflash_ftl_extract_info_entry(cache, 0);
        
        if (info_entry & DOSFS_SFLASH_INFO_NOT_WRITTEN_TO)
        {
            DOSFS_SFLASH_FTL_CHECK();
        }
        else
        {
            if ((head_info[0] != DOSFS_SFLASH_SECTOR_IDENT_0) ||
                (head_info[1] != DOSFS_SFLASH_SECTOR_IDENT_1) ||
                (head_info[2] != sflash->data_start) ||
                (head_info[3] != sflash->data_limit) ||
                (tail_info[0] != DOSFS_SFLASH_SECTOR_IDENT_TAIL))
            {
                DOSFS_SFLASH_FTL_CHECK();
            }

            switch (info_entry & DOSFS_SFLASH_INFO_TYPE_MASK) {
            case DOSFS_SFLASH_INFO_TYPE_DELETED:
            case DOSFS_SFLASH_INFO_TYPE_XLATE:
            case DOSFS_SFLASH_INFO_TYPE_XLATE_SECONDARY:
            case DOSFS_SFLASH_INFO_TYPE_DATA_DELETED:
            case DOSFS_SFLASH_INFO_TYPE_DATA_COMMITTED:
            case DOSFS_SFLASH_INFO_TYPE_DATA_WRITTEN:
                DOSFS_SFLASH_FTL_CHECK();
                break;

            case DOSFS_SFLASH_INFO_TYPE_VICTIM:
                DOSFS_SFLASH_FTL_CHECK();
                break;

            case DOSFS_SFLASH_INFO_TYPE_ERASE:
                sector = info_entry & DOSFS_SFLASH_INFO_DATA_MASK;

                if (dosfs_sflash_ftl_sector_lookup(sflash, sector) != (offset / DOSFS_SFLASH_ERASE_SIZE))
                {
                    DOSFS_SFLASH_FTL_CHECK();
                }

                for (index = 1; index < DOSFS_SFLASH_BLOCK_INFO_ENTRIES; index++)
                {
                    info_entry = dosfs_sflash_ftl_extract_info_entry(cache, index);

                    if (info_entry & DOSFS_SFLASH_INFO_NOT_WRITTEN_TO)
                    {
                        if ((info_entry & DOSFS_SFLASH_INFO_TYPE_MASK) != DOSFS_SFLASH_INFO_TYPE_RESERVED)
                        {
                            DOSFS_SFLASH_FTL_CHECK();
                        }
                    }
                    else
                    {
                        info_logical = (sector << DOSFS_SFLASH_LOGICAL_SECTOR_SHIFT) + index;

                        switch (info_entry & DOSFS_SFLASH_INFO_TYPE_MASK) {
                        case DOSFS_SFLASH_INFO_TYPE_VICTIM:
                        case DOSFS_SFLASH_INFO_TYPE_ERASE:
                        case DOSFS_SFLASH_INFO_TYPE_RECLAIM:
                            DOSFS_SFLASH_FTL_CHECK();
                            break;

                        case DOSFS_SFLASH_INFO_TYPE_DELETED:
                            break;

                        case DOSFS_SFLASH_INFO_TYPE_DATA_DELETED:
                            DOSFS_SFLASH_FTL_CHECK();
                            break;
                
                        case DOSFS_SFLASH_INFO_TYPE_DATA_COMMITTED:
                            address = info_entry & DOSFS_SFLASH_INFO_DATA_MASK;

                            if (address < DOSFS_SFLASH_XLATE_OFFSET)
                            {
                                read_logical = sflash->block_table[address];
                            }
                            else
                            {
                                xlate_cache   = (uint16_t*)sflash->cache[0];
                                xlate2_cache  = (uint16_t*)sflash->cache[1];
                                
                                xlate_segment = (address - DOSFS_SFLASH_XLATE_OFFSET) >> DOSFS_SFLASH_XLATE_SEGMENT_SHIFT;
                                xlate_index   = (address - DOSFS_SFLASH_XLATE_OFFSET) & DOSFS_SFLASH_XLATE_INDEX_MASK;
                                
                                if (sflash->xlate_table[xlate_segment] == DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
                                {
                                    read_logical = DOSFS_SFLASH_BLOCK_NOT_ALLOCATED;
                                }
                                else
                                {
                                    if (sflash->xlate_logical != sflash->xlate_table[xlate_segment])
                                    {
                                        sflash->xlate_logical = sflash->xlate_table[xlate_segment];

                                        (*sflash->interface->read)(sflash->context, dosfs_sflash_ftl_translate(sflash, sflash->xlate_logical), (uint8_t*)xlate_cache, DOSFS_SFLASH_BLOCK_SIZE);
                                    }

                                    read_logical = xlate_cache[xlate_index];
                                    
                                    if (read_logical == DOSFS_SFLASH_BLOCK_DELETED)
                                    {
                                        if (sflash->xlate2_table[xlate_segment] != DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
                                        {
                                            if (sflash->xlate2_logical != sflash->xlate2_table[xlate_segment])
                                            {
                                                sflash->xlate2_logical = sflash->xlate2_table[xlate_segment];
                        
                                                (*sflash->interface->read)(sflash->context, dosfs_sflash_ftl_translate(sflash, sflash->xlate2_logical), (uint8_t*)xlate2_cache, DOSFS_SFLASH_BLOCK_SIZE);
                                            }

                                            if (xlate2_cache[xlate_index] != DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
                                            {
                                                read_logical = xlate2_cache[xlate_index];
                                            }
                                        }
                                    }
                                }
                            }
                        
                            if (read_logical != info_logical)
                            {
                                DOSFS_SFLASH_FTL_CHECK();
                            }
                            break;

                        case DOSFS_SFLASH_INFO_TYPE_DATA_WRITTEN:
                            DOSFS_SFLASH_FTL_CHECK();
                            break;

                        case DOSFS_SFLASH_INFO_TYPE_XLATE:
                            if (sflash->xlate_table[info_entry & DOSFS_SFLASH_INFO_DATA_MASK] != info_logical)
                            {
                                DOSFS_SFLASH_FTL_CHECK();
                            }
                            break;

                        case DOSFS_SFLASH_INFO_TYPE_XLATE_SECONDARY:
                            if (sflash->xlate2_table[info_entry & DOSFS_SFLASH_INFO_DATA_MASK] != info_logical)
                            {
                                DOSFS_SFLASH_FTL_CHECK();
                            }
                            break;

                        case DOSFS_SFLASH_INFO_TYPE_RESERVED:
                            break;
                        }
                    }
                }
                break;

            case DOSFS_SFLASH_INFO_TYPE_RECLAIM:
                DOSFS_SFLASH_FTL_CHECK();
                break;
                    
            case DOSFS_SFLASH_INFO_TYPE_RESERVED:
                if (sflash->reclaim_offset != offset)
                {
                    DOSFS_SFLASH_FTL_CHECK();
                }
                break;
            }
        }
    }

    xlate_cache   = (uint16_t*)sflash->cache[0];
    xlate2_cache  = (uint16_t*)sflash->cache[1];
    
    cache_logical = DOSFS_SFLASH_BLOCK_NOT_ALLOCATED;

    for (xlate_segment = 0; xlate_segment < sflash->xlate_count; xlate_segment++)
    {
        if (sflash->xlate_table[xlate_segment] == DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
        {
            if (sflash->xlate2_table[xlate_segment] != DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
            {
                DOSFS_SFLASH_FTL_CHECK();
            }
        }
        else
        {
            for (xlate_index = 0; xlate_index < DOSFS_SFLASH_XLATE_ENTRIES; xlate_index++)
            {
                address = ((xlate_segment << DOSFS_SFLASH_XLATE_SEGMENT_SHIFT) | xlate_index) + DOSFS_SFLASH_XLATE_OFFSET;

                if (sflash->xlate_table[xlate_segment] == DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
                {
                    read_logical = DOSFS_SFLASH_BLOCK_NOT_ALLOCATED;
                }
                else
                {
                    if (sflash->xlate_logical != sflash->xlate_table[xlate_segment])
                    {
                        sflash->xlate_logical = sflash->xlate_table[xlate_segment];

                        (*sflash->interface->read)(sflash->context, dosfs_sflash_ftl_translate(sflash, sflash->xlate_logical), (uint8_t*)xlate_cache, DOSFS_SFLASH_BLOCK_SIZE);
                    }

                    read_logical = xlate_cache[xlate_index];

                    if (read_logical == DOSFS_SFLASH_BLOCK_DELETED)
                    {
                        if (sflash->xlate2_table[xlate_segment] != DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
                        {
                            if (sflash->xlate2_logical != sflash->xlate2_table[xlate_segment])
                            {
                                sflash->xlate2_logical = sflash->xlate2_table[xlate_segment];
                        
                                (*sflash->interface->read)(sflash->context, dosfs_sflash_ftl_translate(sflash, sflash->xlate2_logical), (uint8_t*)xlate2_cache, DOSFS_SFLASH_BLOCK_SIZE);
                            }
                            
                            if (xlate2_cache[xlate_index] != DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
                            {
                                read_logical = xlate2_cache[xlate_index];
                            }
                        }
                    }
                }

                if ((read_logical != DOSFS_SFLASH_BLOCK_DELETED) && (read_logical != DOSFS_SFLASH_BLOCK_NOT_ALLOCATED))
                {
                    /* Here we have a "address" and it's "read_logical" translation. Check whether the corresponding info header says the same.
                     */

                    if (cache_logical != (read_logical & ~DOSFS_SFLASH_LOGICAL_BLOCK_MASK))
                    {
                        cache_logical = (read_logical & ~DOSFS_SFLASH_LOGICAL_BLOCK_MASK);
                        
                        (*sflash->interface->read)(sflash->context, dosfs_sflash_ftl_translate(sflash, cache_logical), (uint8_t*)cache, DOSFS_SFLASH_BLOCK_SIZE);
                    }

                    info_entry = dosfs_sflash_ftl_extract_info_entry(cache, (read_logical & DOSFS_SFLASH_LOGICAL_BLOCK_MASK));

                    if (info_entry != (DOSFS_SFLASH_INFO_TYPE_DATA_COMMITTED | address))
                    {
                        DOSFS_SFLASH_FTL_CHECK();
                    }
                }
            }
        }
    }
}

#endif

/* A earse unit header has been read into the cache. Merge
 * the entries into the tables ...
 */

static bool dosfs_sflash_ftl_collect(dosfs_sflash_t *sflash, const uint32_t *cache, uint32_t offset, uint32_t sector, uint32_t *p_data_written, uint32_t *p_data_deleted)
{
    unsigned int index;
    uint32_t info_logical, info_entry;

    dosfs_sflash_ftl_sector_assign(sflash, sector, (offset / DOSFS_SFLASH_ERASE_SIZE));

    for (index = 1; index < DOSFS_SFLASH_BLOCK_INFO_ENTRIES; index++)
    {
        info_entry = dosfs_sflash_ftl_extract_info_entry(cache, index);

        if (!(info_entry & DOSFS_SFLASH_INFO_NOT_WRITTEN_TO))
        {
            info_logical = (sector << DOSFS_SFLASH_LOGICAL_SECTOR_SHIFT) + index;

            switch (info_entry & DOSFS_SFLASH_INFO_TYPE_MASK) {
            case DOSFS_SFLASH_INFO_TYPE_VICTIM:
            case DOSFS_SFLASH_INFO_TYPE_ERASE:
            case DOSFS_SFLASH_INFO_TYPE_RECLAIM:
                return false;

            case DOSFS_SFLASH_INFO_TYPE_DELETED:
                break;

            case DOSFS_SFLASH_INFO_TYPE_DATA_DELETED:
                p_data_deleted[0] = (info_entry & DOSFS_SFLASH_INFO_DATA_MASK);
                p_data_deleted[1] = info_logical;
                break;
                
            case DOSFS_SFLASH_INFO_TYPE_DATA_COMMITTED:
                if ((info_entry & DOSFS_SFLASH_INFO_DATA_MASK) < DOSFS_SFLASH_XLATE_OFFSET)
                {
                    if (sflash->block_table[info_entry & DOSFS_SFLASH_INFO_DATA_MASK] == DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
                    {
                        sflash->block_table[info_entry & DOSFS_SFLASH_INFO_DATA_MASK] = info_logical;
                    }
                    else
                    {
                        dosfs_sflash_ftl_set_info_entry(sflash, info_logical, DOSFS_SFLASH_INFO_TYPE_DELETED);
                    }
                }
                break;

            case DOSFS_SFLASH_INFO_TYPE_DATA_WRITTEN:
                p_data_written[0] = (info_entry & DOSFS_SFLASH_INFO_DATA_MASK);
                p_data_written[1] = info_logical;
                break;

            case DOSFS_SFLASH_INFO_TYPE_XLATE:
                if ((info_entry & DOSFS_SFLASH_INFO_DATA_MASK) >= DOSFS_SFLASH_XLATE_COUNT)
                {
                    return false;
                }

                sflash->xlate_table[info_entry & DOSFS_SFLASH_INFO_DATA_MASK] = info_logical;
                break;

            case DOSFS_SFLASH_INFO_TYPE_XLATE_SECONDARY:
                if ((info_entry & DOSFS_SFLASH_INFO_DATA_MASK) >= DOSFS_SFLASH_XLATE_COUNT)
                {
                    return false;
                }

                sflash->xlate2_table[info_entry & DOSFS_SFLASH_INFO_DATA_MASK] = info_logical;
                break;

            case DOSFS_SFLASH_INFO_TYPE_RESERVED:
                dosfs_sflash_ftl_set_info_entry(sflash, info_logical, DOSFS_SFLASH_INFO_TYPE_DELETED);
                break;
            }
        }
    }

    return true;
}

static bool dosfs_sflash_ftl_mount(dosfs_sflash_t *sflash)
{
    uint32_t offset, erase_offset, reclaim_offset, victim_offset, victim_sector, victim_erase_count, erase_count, erase_count_max;
    uint32_t index, info_entry, xlate_segment, xlate2_logical, data_written[2], data_deleted[2];
    uint32_t *cache, *head_info, *tail_info;

    // printf("==== MOUNT %d ====\n", sizeof(dosfs_sflash_t));

    memset(&sflash->sector_table[0], 0xff, sizeof(sflash->sector_table));
#if (DOSFS_SFLASH_DATA_SIZE == 0x02000000)
    memset(&sflash->sector_mask[0], 0xff, sizeof(sflash->sector_mask));
#endif /* DOSFS_SFLASH_DATA_SIZE */
    memset(&sflash->block_table[0], 0xff, sizeof(sflash->block_table));
    memset(&sflash->xlate_table[0], 0xff, sizeof(sflash->xlate_table));
    memset(&sflash->xlate2_table[0], 0xff, sizeof(sflash->xlate2_table));

    sflash->xlate_logical = DOSFS_SFLASH_BLOCK_RESERVED;
    sflash->xlate2_logical = DOSFS_SFLASH_BLOCK_RESERVED;

    sflash->victim_offset = DOSFS_SFLASH_PHYSICAL_ILLEGAL;

    sflash->reclaim_offset = DOSFS_SFLASH_PHYSICAL_ILLEGAL;
    sflash->reclaim_erase_count = 0xffffffff;


    data_written[0] = DOSFS_SFLASH_BLOCK_NOT_ALLOCATED;
    data_written[1] = 0;

    data_deleted[0] = DOSFS_SFLASH_BLOCK_NOT_ALLOCATED;
    data_deleted[1] = 0;

    erase_count_max = 0x00000000;

    erase_offset = DOSFS_SFLASH_PHYSICAL_ILLEGAL;
    reclaim_offset = DOSFS_SFLASH_PHYSICAL_ILLEGAL;
    victim_offset = DOSFS_SFLASH_PHYSICAL_ILLEGAL;

    cache = sflash->cache[0];
    head_info = &cache[DOSFS_SFLASH_INFO_HEAD_OFFSET / sizeof(uint32_t)];
    tail_info = &cache[DOSFS_SFLASH_INFO_TAIL_OFFSET / sizeof(uint32_t)];

    for (offset = sflash->data_start; offset < sflash->data_limit; offset += DOSFS_SFLASH_ERASE_SIZE)
    {
        (*sflash->interface->read)(sflash->context, offset, (uint8_t*)cache, DOSFS_SFLASH_BLOCK_SIZE);

        info_entry = dosfs_sflash_ftl_extract_info_entry(cache, 0);
        
        if (info_entry & DOSFS_SFLASH_INFO_NOT_WRITTEN_TO)
        {
            /* A crash happened during the reclaim process. 
             */

            if (erase_offset != DOSFS_SFLASH_PHYSICAL_ILLEGAL)
            {
                return false;
            }

            erase_offset = offset;
        }
        else
        {
            if ((head_info[0] != DOSFS_SFLASH_SECTOR_IDENT_0) ||
                (head_info[1] != DOSFS_SFLASH_SECTOR_IDENT_1) ||
                (head_info[2] != sflash->data_start) ||
                (head_info[3] != sflash->data_limit) ||
                (tail_info[0] != DOSFS_SFLASH_SECTOR_IDENT_TAIL))
            {
                return false;
            }

            erase_count = head_info[DOSFS_SFLASH_INFO_SLOT_ERASE_COUNT];

            switch (info_entry & DOSFS_SFLASH_INFO_TYPE_MASK) {
            case DOSFS_SFLASH_INFO_TYPE_DELETED:
            case DOSFS_SFLASH_INFO_TYPE_XLATE:
            case DOSFS_SFLASH_INFO_TYPE_XLATE_SECONDARY:
            case DOSFS_SFLASH_INFO_TYPE_DATA_DELETED:
            case DOSFS_SFLASH_INFO_TYPE_DATA_COMMITTED:
            case DOSFS_SFLASH_INFO_TYPE_DATA_WRITTEN:
                break;

            case DOSFS_SFLASH_INFO_TYPE_VICTIM:
                /* A crash happened during the reclaim process. 
                 */

                if (victim_offset != DOSFS_SFLASH_PHYSICAL_ILLEGAL)
                {
                    return false;
                }

                victim_offset = offset;
                /* FALLTHROU */

            case DOSFS_SFLASH_INFO_TYPE_ERASE:
                if (!dosfs_sflash_ftl_collect(sflash, cache, offset, (info_entry & DOSFS_SFLASH_INFO_DATA_MASK), &data_written[0], &data_deleted[0]))
                {
                    return false;
                }
                break;

            case DOSFS_SFLASH_INFO_TYPE_RECLAIM:
                /* A crash happened during the reclaim process. 
                 */

                if (reclaim_offset != DOSFS_SFLASH_PHYSICAL_ILLEGAL)
                {
                    return false;
                }

                reclaim_offset = offset;
                break;

            case DOSFS_SFLASH_INFO_TYPE_RESERVED:
                /* This is really the case where there is a unallocated free
                 * free erase unit ... So pick the one with the lowest erase
                 * count as the reclaim erase unit.
                 *
                 * There should be only one, and that is reclaim sector.
                 */

                if (sflash->reclaim_offset != DOSFS_SFLASH_PHYSICAL_ILLEGAL)
                {
                    return false;
                }

                sflash->reclaim_offset = offset;
                sflash->reclaim_erase_count = erase_count;
                break;
            }

            if (erase_count_max < erase_count)
            {
                erase_count_max = erase_count;
            }
        }
    }

    /* Check for the case where a crash happened during the reclaim process.
     * In that case, erase the victim and update the wear level info
     */

    if (victim_offset != DOSFS_SFLASH_PHYSICAL_ILLEGAL)
    {
        (*sflash->interface->read)(sflash->context, victim_offset, (uint8_t*)cache, DOSFS_SFLASH_BLOCK_SIZE);

        info_entry = dosfs_sflash_ftl_extract_info_entry(cache, 0);
    
        victim_sector = info_entry & DOSFS_SFLASH_INFO_DATA_MASK;
        victim_erase_count = head_info[DOSFS_SFLASH_INFO_SLOT_ERASE_COUNT];

        if (reclaim_offset != DOSFS_SFLASH_PHYSICAL_ILLEGAL)
        {
            /* If there is a victim and a reclaim sector, That means everything got
             * copied, and just the victim sector needs to be erased and made
             * the new reclaim sector.
             */

            victim_erase_count++;
            
            dosfs_sflash_ftl_swap(sflash, victim_offset, victim_sector, victim_erase_count, reclaim_offset);

            if (erase_count_max < victim_erase_count)
            {
                erase_count_max = victim_erase_count;
            }
        }
        else
        {
            /* Here a crash happened while copying data from the victim sector to the reclaim sector.
             * Hence erase the reclaim sector and try again with the reclaim.
             *
             * If there is a valid erase_offset, there was a double crash before, so assume that
             * the newly earase sector was the reclaim sector, and the victim sector contains
             * the proper erase_count.
             */

            if (erase_offset != DOSFS_SFLASH_PHYSICAL_ILLEGAL)
            {
                sflash->reclaim_erase_count = head_info[DOSFS_SFLASH_INFO_SLOT_RECLAIM_ERASE_COUNT];
                sflash->reclaim_offset = erase_offset;

                erase_offset = DOSFS_SFLASH_PHYSICAL_ILLEGAL;
            }
            else
            {
                sflash->reclaim_erase_count++;

                (*sflash->interface->erase)(sflash->context, sflash->reclaim_offset);

                if (erase_count_max < sflash->reclaim_erase_count)
                {
                    erase_count_max = sflash->reclaim_erase_count;
                }
            }

            info_entry = DOSFS_SFLASH_INFO_ERASED & ~DOSFS_SFLASH_INFO_NOT_WRITTEN_TO;
            
            (*sflash->interface->program)(sflash->context, sflash->reclaim_offset + DOSFS_SFLASH_INFO_ENTRY_OFFSET, (const uint8_t*)&info_entry, DOSFS_SFLASH_INFO_ENTRY_SIZE);

            head_info[0] = DOSFS_SFLASH_SECTOR_IDENT_0;
            head_info[1] = DOSFS_SFLASH_SECTOR_IDENT_1;
            head_info[2] = sflash->data_start;
            head_info[3] = sflash->data_limit;
            head_info[4] = sflash->reclaim_erase_count;
            head_info[5] = 0xffffffff;
            head_info[6] = 0xffffffff;
            head_info[7] = 0xffffffff;

            (*sflash->interface->program)(sflash->context, sflash->reclaim_offset + DOSFS_SFLASH_INFO_HEAD_OFFSET, (const uint8_t*)&head_info[0], DOSFS_SFLASH_INFO_HEAD_SIZE);

            tail_info[0] = DOSFS_SFLASH_SECTOR_IDENT_TAIL;

            (*sflash->interface->program)(sflash->context, sflash->reclaim_offset + DOSFS_SFLASH_INFO_TAIL_OFFSET, (const uint8_t*)&tail_info[0], DOSFS_SFLASH_INFO_TAIL_SIZE);
        }
    }
    else
    {
        if (reclaim_offset != DOSFS_SFLASH_PHYSICAL_ILLEGAL)
        {
            /* Here we crashed before marking the reclaim sector as normal erase sector.
             */
            (*sflash->interface->read)(sflash->context, reclaim_offset, (uint8_t*)cache, DOSFS_SFLASH_BLOCK_SIZE);

            info_entry = dosfs_sflash_ftl_extract_info_entry(cache, 0);

            info_entry &= ~DOSFS_SFLASH_INFO_TYPE_ERASE;

            (*sflash->interface->program)(sflash->context, reclaim_offset + DOSFS_SFLASH_INFO_ENTRY_OFFSET, (const uint8_t*)&info_entry, DOSFS_SFLASH_INFO_ENTRY_SIZE); /* RECLAIM -> ERASE */

            if (!dosfs_sflash_ftl_collect(sflash, cache, reclaim_offset, (cache[0] & DOSFS_SFLASH_INFO_DATA_MASK), &data_written[0], &data_deleted[0]))
            {
                return false;
            }
        }
    }

    /* If there is still a fully erased sector around, it means
     * something happened during the format process, hence fail
     * and force a reformat.
     */

    if (erase_offset != DOSFS_SFLASH_PHYSICAL_ILLEGAL)
    {
        return false;
    }

    /* Scan xlate_table and xlate2_table for the case where there is a xlate2 entry but
     * no xlate entry. This can occure while xlate2 is converted to an xlate entry,
     * which means the xlate2 entry is really an xlate entry.
     *
     * Also scan xlate/xlate2 for entries where UNCOMITTED is set, but BLOCK_MASK is not all 1s.
     * If the block pointed to is DELETED, then also delete it in xlate/xlate2, otherwise
     * clear the UNCOMITTED bit. This recovers from a crash during a write operation.
     */

    for (xlate_segment = 0; xlate_segment < sflash->xlate_count; xlate_segment++)
    {
        if ((sflash->xlate_table[xlate_segment] == DOSFS_SFLASH_BLOCK_NOT_ALLOCATED) && (sflash->xlate2_table[xlate_segment] != DOSFS_SFLASH_BLOCK_NOT_ALLOCATED))
        {
            xlate2_logical = sflash->xlate2_table[xlate_segment];
            
            dosfs_sflash_ftl_set_info_type(sflash, xlate2_logical, (DOSFS_SFLASH_INFO_TYPE_XLATE | xlate_segment));
            
            sflash->xlate_table[xlate_segment] = xlate2_logical;
            sflash->xlate2_table[xlate_segment] = DOSFS_SFLASH_BLOCK_NOT_ALLOCATED;
        }
    }

    if (data_written[0] != DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
    {
        dosfs_sflash_ftl_modify(sflash, data_written[0], data_written[1]);
    }

    if (data_deleted[0] != DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
    {
        dosfs_sflash_ftl_trim(sflash, data_deleted[0], data_deleted[1]);
    }

    sflash->erase_count_max = erase_count_max;

    sflash->victim_offset = sflash->reclaim_offset + DOSFS_SFLASH_ERASE_SIZE;

    if (sflash->victim_offset == sflash->data_limit)
    {
        sflash->victim_offset = sflash->data_start;
    }

    (*sflash->interface->read)(sflash->context, sflash->victim_offset, (uint8_t*)cache, DOSFS_SFLASH_BLOCK_SIZE);

    info_entry = dosfs_sflash_ftl_extract_info_entry(cache, 0);

    sflash->alloc_sector  = info_entry & DOSFS_SFLASH_INFO_DATA_MASK;
    sflash->alloc_count   = 0;
    sflash->alloc_index   = 0;
    sflash->alloc_mask[0] = 0;
    sflash->alloc_mask[1] = 0;
    sflash->alloc_mask[2] = 0;
    sflash->alloc_mask[3] = 0;

    for (index = 1; index < DOSFS_SFLASH_BLOCK_INFO_ENTRIES; index++)
    {
        info_entry = dosfs_sflash_ftl_extract_info_entry(cache, index);

        if ((info_entry & DOSFS_SFLASH_INFO_NOT_WRITTEN_TO) || ((info_entry & DOSFS_SFLASH_INFO_TYPE_MASK) == DOSFS_SFLASH_INFO_TYPE_DELETED))
        {
            if (sflash->alloc_count == 0)
            {
                sflash->alloc_index = index;
            }
            
            sflash->alloc_mask[index / 32] |= (1ul << (index & 31));
            sflash->alloc_count++;
        }
    }

#if (DOSFS_CONFIG_SFLASH_DEBUG == 1)
    dosfs_sflash_ftl_check();
#endif

    return true;
}

static uint32_t dosfs_sflash_ftl_allocate(dosfs_sflash_t *sflash)
{
    uint32_t logical;

    while (!sflash->alloc_count)
    {
        sflash->victim_offset = sflash->victim_offset + DOSFS_SFLASH_ERASE_SIZE;

        if (sflash->victim_offset == sflash->data_limit)
        {
            sflash->victim_offset = sflash->data_start;
        }

        if (sflash->victim_offset == sflash->reclaim_offset)
        {
            sflash->victim_offset = sflash->victim_offset + DOSFS_SFLASH_ERASE_SIZE;

            if (sflash->victim_offset == sflash->data_limit)
            {
                sflash->victim_offset = sflash->data_start;
            }
        }

        dosfs_sflash_ftl_reclaim(sflash, sflash->victim_offset);
    }

    while (!(sflash->alloc_mask[sflash->alloc_index / 32] & (1ul << (sflash->alloc_index & 31))))
    {
        sflash->alloc_index++;
    }

    logical = (sflash->alloc_sector << DOSFS_SFLASH_LOGICAL_SECTOR_SHIFT) + sflash->alloc_index;
            
    sflash->alloc_mask[sflash->alloc_index / 32] &= ~(1ul << (sflash->alloc_index & 31));
    sflash->alloc_count--;
    sflash->alloc_index++;

    // printf("ALLOCATE @%08x\n", logical);

    return logical;
}

static void dosfs_sflash_ftl_write(dosfs_sflash_t *sflash, uint32_t address, const uint8_t *data)
{
    uint32_t read_logical, write_logical, write_offset, xlate_segment, xlate_index, index;
    uint16_t *xlate_cache, *xlate2_cache;

    DOSFS_SFLASH_STATISTICS_COUNT_N(sflash_ftl_write, 512);

#if (DOSFS_CONFIG_SFLASH_DEBUG == 1)
    memcpy(&sflash_data_shadow[address * 512], data, 512);
#endif /* DOSFS_CONFIG_SFLASH_DEBUG == 1 */

    /* Prepare xlate/xlate2 so that the logical mapping can be updated.
     */

    xlate_cache   = (uint16_t*)sflash->cache[0];
    xlate2_cache  = (uint16_t*)sflash->cache[1];

    xlate_segment = (address - DOSFS_SFLASH_XLATE_OFFSET) >> DOSFS_SFLASH_XLATE_SEGMENT_SHIFT;
    xlate_index   = (address - DOSFS_SFLASH_XLATE_OFFSET) & DOSFS_SFLASH_XLATE_INDEX_MASK;

    if (address >= DOSFS_SFLASH_XLATE_OFFSET)
    {
        if (sflash->xlate_table[xlate_segment] == DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
        {
            sflash->xlate_logical = dosfs_sflash_ftl_allocate(sflash);
            sflash->xlate_table[xlate_segment] = sflash->xlate_logical;

            dosfs_sflash_ftl_set_info_entry(sflash, sflash->xlate_logical, (DOSFS_SFLASH_INFO_TYPE_XLATE | xlate_segment));

            memset(xlate_cache, 0xff, DOSFS_SFLASH_BLOCK_SIZE);
        }
        else 
        {
            if (sflash->xlate_logical != sflash->xlate_table[xlate_segment])
            {
                DOSFS_SFLASH_STATISTICS_COUNT(sflash_ftl_xlate_miss);

                sflash->xlate_logical = sflash->xlate_table[xlate_segment];

                (*sflash->interface->read)(sflash->context, dosfs_sflash_ftl_translate(sflash, sflash->xlate_logical), (uint8_t*)xlate_cache, DOSFS_SFLASH_BLOCK_SIZE);
            }
            else
            {
                DOSFS_SFLASH_STATISTICS_COUNT(sflash_ftl_xlate_hit);;
            }

            if (xlate_cache[xlate_index] != DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
            {
                if (sflash->xlate2_table[xlate_segment] == DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
                {
                    sflash->xlate2_logical = dosfs_sflash_ftl_allocate(sflash);
                    sflash->xlate2_table[xlate_segment] = sflash->xlate2_logical;
                    
                    dosfs_sflash_ftl_set_info_entry(sflash, sflash->xlate2_logical, (DOSFS_SFLASH_INFO_TYPE_XLATE_SECONDARY | xlate_segment));
                    
                    memset(xlate2_cache, 0xff, DOSFS_SFLASH_BLOCK_SIZE);
                }
                else
                {
                    if (sflash->xlate2_logical != sflash->xlate2_table[xlate_segment])
                    {
                        DOSFS_SFLASH_STATISTICS_COUNT(sflash_ftl_xlate2_miss);

                        sflash->xlate2_logical = sflash->xlate2_table[xlate_segment];
                        
                        (*sflash->interface->read)(sflash->context, dosfs_sflash_ftl_translate(sflash, sflash->xlate2_logical), (uint8_t*)xlate2_cache, DOSFS_SFLASH_BLOCK_SIZE);
                    }
                    else
                    {
                        DOSFS_SFLASH_STATISTICS_COUNT(sflash_ftl_xlate2_hit);
                    }

                    if (xlate2_cache[xlate_index] != DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
                    {
                        /* Swap xlate & xlate2. Backprop all changes into xlate_cache, and then write
                         * xlate2 with the merged data back. Next allocate a new xlate2 and properly
                         * add the next logical mapping.
                         *
                         * Make sure the upper allocate did not blow away the xlate_cache ...
                         */

                        for (index = 0; index < DOSFS_SFLASH_XLATE_ENTRIES; index++)
                        {
                            if (xlate2_cache[index] != DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
                            {
                                xlate_cache[index] = xlate2_cache[index];
                            }
                        }

                        (*sflash->interface->program)(sflash->context, dosfs_sflash_ftl_translate(sflash, sflash->xlate2_logical), (const uint8_t*)xlate_cache, DOSFS_SFLASH_PAGE_SIZE);
                        (*sflash->interface->program)(sflash->context, dosfs_sflash_ftl_translate(sflash, sflash->xlate2_logical) + DOSFS_SFLASH_PAGE_SIZE, (const uint8_t*)xlate_cache + DOSFS_SFLASH_PAGE_SIZE, DOSFS_SFLASH_PAGE_SIZE);

                        /* Change xlate2 to xlate, and delete the old xlate entry.
                         */
                        
                        dosfs_sflash_ftl_set_info_entry(sflash, sflash->xlate_logical, DOSFS_SFLASH_INFO_TYPE_DELETED);

                        dosfs_sflash_ftl_set_info_type(sflash, sflash->xlate2_logical, (DOSFS_SFLASH_INFO_TYPE_XLATE | xlate_segment));

                        sflash->xlate_logical = sflash->xlate2_logical;
                        sflash->xlate_table[xlate_segment] = sflash->xlate_logical;

                        /* Allocate a new xlate2 and modify the target entry.
                         */

                        sflash->xlate2_logical = dosfs_sflash_ftl_allocate(sflash);
                        sflash->xlate2_table[xlate_segment] = sflash->xlate2_logical;

                        dosfs_sflash_ftl_set_info_entry(sflash, sflash->xlate2_logical, (DOSFS_SFLASH_INFO_TYPE_XLATE_SECONDARY | xlate_segment));

                        memset(xlate2_cache, 0xff, DOSFS_SFLASH_BLOCK_SIZE);
                    }
                }
            }
        }
    }

    /* Allocate a new block and write the data to it.
     */

    write_logical = dosfs_sflash_ftl_allocate(sflash);

    // printf("WRITE %08x @%08x =%08x\n", address, write_logical, dosfs_sflash_ftl_translate(sflash, write_logical));

    // ###
    dosfs_sflash_ftl_set_info_entry(sflash, write_logical, (DOSFS_SFLASH_INFO_TYPE_RESERVED | address));

    write_offset = dosfs_sflash_ftl_translate(sflash, write_logical);
    
    (*sflash->interface->program)(sflash->context, write_offset, (const uint8_t*)data, DOSFS_SFLASH_PAGE_SIZE);
    (*sflash->interface->program)(sflash->context, write_offset + DOSFS_SFLASH_PAGE_SIZE, (const uint8_t*)data + DOSFS_SFLASH_PAGE_SIZE, DOSFS_SFLASH_PAGE_SIZE);



    /* Now we have a valid new block, and allocated xlate/xlate2 entries. So simply plug in the new
     * mapping.
     */

    if (address < DOSFS_SFLASH_XLATE_OFFSET)
    {
        dosfs_sflash_ftl_set_info_type(sflash, write_logical, (DOSFS_SFLASH_INFO_TYPE_DATA_COMMITTED | address));

        read_logical = sflash->block_table[address];

#if (DOSFS_CONFIG_SFLASH_DEBUG == 1)
        assert(read_logical == sflash_xlate_shadow[address]);
        sflash_xlate_shadow[address] = write_logical;
#endif /* DOSFS_CONFIG_SFLASH_DEBUG == 1 */

        sflash->block_table[address] = write_logical;

        if (read_logical != DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
        {
            dosfs_sflash_ftl_set_info_entry(sflash, read_logical, DOSFS_SFLASH_INFO_TYPE_DELETED);
        }
    }
    else
    {
        dosfs_sflash_ftl_set_info_type(sflash, write_logical, (DOSFS_SFLASH_INFO_TYPE_DATA_WRITTEN | address));

        dosfs_sflash_ftl_modify(sflash, address, write_logical);
    }
}

static void dosfs_sflash_ftl_read(dosfs_sflash_t *sflash, uint32_t address, uint8_t *data)
{
    uint32_t read_logical, read_offset, xlate_segment, xlate_index;
    uint16_t *xlate_cache, *xlate2_cache;

    DOSFS_SFLASH_STATISTICS_COUNT_N(sflash_ftl_read, 512);

    if (address < DOSFS_SFLASH_XLATE_OFFSET)
    {
        read_logical = sflash->block_table[address];
    }
    else
    {
        xlate_cache   = (uint16_t*)sflash->cache[0];
        xlate2_cache  = (uint16_t*)sflash->cache[1];

        xlate_segment = (address - DOSFS_SFLASH_XLATE_OFFSET) >> DOSFS_SFLASH_XLATE_SEGMENT_SHIFT;
        xlate_index   = (address - DOSFS_SFLASH_XLATE_OFFSET) & DOSFS_SFLASH_XLATE_INDEX_MASK;

        if (sflash->xlate_table[xlate_segment] == DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
        {
            read_logical = DOSFS_SFLASH_BLOCK_NOT_ALLOCATED;
        }
        else
        {
            if (sflash->xlate_logical != sflash->xlate_table[xlate_segment])
            {
                DOSFS_SFLASH_STATISTICS_COUNT(sflash_ftl_xlate_miss);

                sflash->xlate_logical = sflash->xlate_table[xlate_segment];

                (*sflash->interface->read)(sflash->context, dosfs_sflash_ftl_translate(sflash, sflash->xlate_logical), (uint8_t*)xlate_cache, DOSFS_SFLASH_BLOCK_SIZE);
            }
            else
            {
                DOSFS_SFLASH_STATISTICS_COUNT(sflash_ftl_xlate_hit);;
            }

            read_logical = xlate_cache[xlate_index];

            if (read_logical == DOSFS_SFLASH_BLOCK_DELETED)
            {
                if (sflash->xlate2_table[xlate_segment] != DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
                {
                    if (sflash->xlate2_logical != sflash->xlate2_table[xlate_segment])
                    {
                        DOSFS_SFLASH_STATISTICS_COUNT(sflash_ftl_xlate2_miss);

                        sflash->xlate2_logical = sflash->xlate2_table[xlate_segment];
                        
                        (*sflash->interface->read)(sflash->context, dosfs_sflash_ftl_translate(sflash, sflash->xlate2_logical), (uint8_t*)xlate2_cache, DOSFS_SFLASH_BLOCK_SIZE);
                    }
                    else
                    {
                        DOSFS_SFLASH_STATISTICS_COUNT(sflash_ftl_xlate2_hit);
                    }

                    if (xlate2_cache[xlate_index] != DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
                    {
                        read_logical = xlate2_cache[xlate_index];
                    }
                }
            }
        }
    }

#if (DOSFS_CONFIG_SFLASH_DEBUG == 1)
    assert(read_logical == sflash_xlate_shadow[address]);
#endif /* DOSFS_CONFIG_SFLASH_DEBUG == 1 */

    if ((read_logical != DOSFS_SFLASH_BLOCK_NOT_ALLOCATED) && (read_logical != DOSFS_SFLASH_BLOCK_DELETED))
    {
        read_offset = dosfs_sflash_ftl_translate(sflash, read_logical);

        // printf("READ %08x @%08x =%08x\n", address, read_logical, dosfs_sflash_ftl_translate(sflash, read_logical));
    
        (*sflash->interface->read)(sflash->context, read_offset, (uint8_t*)data, DOSFS_SFLASH_BLOCK_SIZE);
    }
    else
    {
        memset(data, 0xff, DOSFS_SFLASH_BLOCK_SIZE);
    }

#if (DOSFS_CONFIG_SFLASH_DEBUG == 1)
    assert (!memcmp(&sflash_data_shadow[address * 512], data, 512));
#endif /* DOSFS_CONFIG_SFLASH_DEBUG == 1 */
}

static void dosfs_sflash_ftl_discard(dosfs_sflash_t *sflash, uint32_t address)
{
    uint32_t delete_logical, xlate_segment, xlate_index;
    uint16_t *xlate_cache, *xlate2_cache;

    DOSFS_SFLASH_STATISTICS_COUNT_N(sflash_ftl_delete, 512);

#if (DOSFS_CONFIG_SFLASH_DEBUG == 1)
    memset(&sflash_data_shadow[address * 512], 0xff, 512);
#endif /* DOSFS_CONFIG_SFLASH_DEBUG == 1 */

    if (address < DOSFS_SFLASH_XLATE_OFFSET)
    {
        delete_logical = sflash->block_table[address];

#if (DOSFS_CONFIG_SFLASH_DEBUG == 1)
        assert(delete_logical == sflash_xlate_shadow[address]);
#endif /* DOSFS_CONFIG_SFLASH_DEBUG == 1 */

        if ((delete_logical != DOSFS_SFLASH_BLOCK_NOT_ALLOCATED) && (delete_logical != DOSFS_SFLASH_BLOCK_DELETED))
        {
#if (DOSFS_CONFIG_SFLASH_DEBUG == 1)
            sflash_xlate_shadow[address] = DOSFS_SFLASH_BLOCK_DELETED;
#endif /* DOSFS_CONFIG_SFLASH_DEBUG == 1 */

            sflash->block_table[address] = DOSFS_SFLASH_BLOCK_DELETED;

            dosfs_sflash_ftl_set_info_entry(sflash, delete_logical, DOSFS_SFLASH_INFO_TYPE_DELETED);
        }
    }
    else
    {
        xlate_cache   = (uint16_t*)sflash->cache[0];
        xlate2_cache  = (uint16_t*)sflash->cache[1];

        xlate_segment = (address - DOSFS_SFLASH_XLATE_OFFSET) >> DOSFS_SFLASH_XLATE_SEGMENT_SHIFT;
        xlate_index   = (address - DOSFS_SFLASH_XLATE_OFFSET) & DOSFS_SFLASH_XLATE_INDEX_MASK;

        if (sflash->xlate_table[xlate_segment] != DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
        {
            if (sflash->xlate_logical != sflash->xlate_table[xlate_segment])
            {
                DOSFS_SFLASH_STATISTICS_COUNT(sflash_ftl_xlate_miss);

                sflash->xlate_logical = sflash->xlate_table[xlate_segment];

                (*sflash->interface->read)(sflash->context, dosfs_sflash_ftl_translate(sflash, sflash->xlate_logical), (uint8_t*)xlate_cache, DOSFS_SFLASH_BLOCK_SIZE);
            }
            else
            {
                DOSFS_SFLASH_STATISTICS_COUNT(sflash_ftl_xlate_hit);;
            }

            delete_logical = xlate_cache[xlate_index];

            if (delete_logical != DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
            {
                if (delete_logical == DOSFS_SFLASH_BLOCK_DELETED)
                {
                    if (sflash->xlate2_table[xlate_segment] != DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
                    {
                        if (sflash->xlate2_logical != sflash->xlate2_table[xlate_segment])
                        {
                            DOSFS_SFLASH_STATISTICS_COUNT(sflash_ftl_xlate2_miss);
                            
                            sflash->xlate2_logical = sflash->xlate2_table[xlate_segment];
                        
                            (*sflash->interface->read)(sflash->context, dosfs_sflash_ftl_translate(sflash, sflash->xlate2_logical), (uint8_t*)xlate2_cache, DOSFS_SFLASH_BLOCK_SIZE);
                        }
                        else
                        {
                            DOSFS_SFLASH_STATISTICS_COUNT(sflash_ftl_xlate2_hit);
                        }

                        if (xlate2_cache[xlate_index] != DOSFS_SFLASH_BLOCK_NOT_ALLOCATED)
                        {
                            delete_logical = xlate2_cache[xlate_index];
                        }
                    }
                }
            }

#if (DOSFS_CONFIG_SFLASH_DEBUG == 1)
            assert(delete_logical == sflash_xlate_shadow[address]);
#endif /* DOSFS_CONFIG_SFLASH_DEBUG == 1 */

            if ((delete_logical != DOSFS_SFLASH_BLOCK_NOT_ALLOCATED) && (delete_logical != DOSFS_SFLASH_BLOCK_DELETED))
            {
                dosfs_sflash_ftl_set_info_type(sflash, delete_logical, (DOSFS_SFLASH_INFO_TYPE_DATA_DELETED | address));
                
                dosfs_sflash_ftl_trim(sflash, address, delete_logical);
            }
        }
    }
}

static int dosfs_sflash_release(void *context)
{
    dosfs_sflash_t *sflash = (dosfs_sflash_t*)context;
    int status = F_NO_ERROR;

#if (DOSFS_CONFIG_SFLASH_SIMULATE_TRACE == 1)
    printf("SFLASH_RELEASE\n");
#endif /* (DOSFS_CONFIG_SFLASH_SIMULATE_TRACE == 1) */

    (*sflash->interface->notify)(sflash->context, NULL, NULL);

    return status;
}

static int dosfs_sflash_info(void *context, uint8_t *p_media, uint8_t *p_write_protected, uint32_t *p_block_count, uint32_t *p_au_size, uint32_t *p_serial)
{
    dosfs_sflash_t *sflash = (dosfs_sflash_t*)context;
    int status = F_NO_ERROR;
    uint32_t blkcnt;

#if (DOSFS_CONFIG_SFLASH_SIMULATE_TRACE == 1)
    printf("SFLASH_INFO\n");
#endif /* (DOSFS_CONFIG_SFLASH_SIMULATE_TRACE == 1) */

    /* One sector needs to be put away as reclaim sector. There is the starting block of each erase block.
     * There are XLATE/XLATE2 entries for each set of 128 blocks.
     */

    blkcnt = (((sflash->data_limit - sflash->data_start) / DOSFS_SFLASH_ERASE_SIZE) -1) * ((DOSFS_SFLASH_ERASE_SIZE / DOSFS_SFLASH_BLOCK_SIZE) -1) -2 - (2 * sflash->xlate_count);

    *p_media = DOSFS_MEDIA_SFLASH;
    *p_write_protected = false;
    *p_block_count = blkcnt;
    *p_au_size = 8;
    *p_serial = 0;

    return status;
}

static int dosfs_sflash_notify(void *context, dosfs_device_notify_callback_t callback, void *cookie)
{
    dosfs_sflash_t *sflash = (dosfs_sflash_t*)context;
    int status = F_NO_ERROR;

    (*sflash->interface->notify)(sflash->context, callback, cookie);

    return status;
}

static int dosfs_sflash_format(void *context)
{
    dosfs_sflash_t *sflash = (dosfs_sflash_t*)context;
    int status = F_NO_ERROR;

#if (DOSFS_CONFIG_SFLASH_SIMULATE_TRACE == 1)
    printf("SFLASH_FORMAT\n");
#endif /* (DOSFS_CONFIG_SFLASH_SIMULATE_TRACE == 1) */

    (*sflash->interface->lock)(sflash->context);

    dosfs_sflash_ftl_format(sflash);

    if (!dosfs_sflash_ftl_mount(sflash))
    {
        sflash->state = DOSFS_SFLASH_STATE_NOT_FORMATTED;

        status = F_ERR_ONDRIVE;
    }
    else
    {
        sflash->state = DOSFS_SFLASH_STATE_READY;
    }

    (*sflash->interface->unlock)(sflash->context);

    return status;
}

static int dosfs_sflash_erase(void *context, uint32_t address, uint32_t length)
{
    return F_NO_ERROR;
}

static int dosfs_sflash_discard(void *context, uint32_t address, uint32_t length)
{
    dosfs_sflash_t *sflash = (dosfs_sflash_t*)context;
    int status = F_NO_ERROR;

#if (DOSFS_CONFIG_SFLASH_SIMULATE_TRACE == 1)
    printf("SFLASH_DELETE %08x, %d\n", address, length);
#endif /* (DOSFS_CONFIG_SFLASH_SIMULATE_TRACE == 1) */

    if (sflash->state != DOSFS_SFLASH_STATE_READY)
    {
        status = F_ERR_ONDRIVE;
    }
    else
    {
        (*sflash->interface->lock)(sflash->context);

        while (length--)
        {
            dosfs_sflash_ftl_discard(sflash, address);
            
            address++;
        }

        (*sflash->interface->unlock)(sflash->context);
    }

#if (DOSFS_CONFIG_SFLASH_DEBUG == 1)
    dosfs_sflash_ftl_check();
#endif

    return status;
}

static int dosfs_sflash_read(void *context, uint32_t address, uint8_t *data, uint32_t length, bool prefetch)
{
    dosfs_sflash_t *sflash = (dosfs_sflash_t*)context;
    int status = F_NO_ERROR;

#if (DOSFS_CONFIG_SFLASH_SIMULATE_TRACE == 1)
    printf("SFLASH_READ %08x, %d\n", address, length);
#endif /* (DOSFS_CONFIG_SFLASH_SIMULATE_TRACE == 1) */

    if (sflash->state != DOSFS_SFLASH_STATE_READY)
    {
        status = F_ERR_ONDRIVE;
    }
    else
    {
        (*sflash->interface->lock)(sflash->context);

        while (length--)
        {
            dosfs_sflash_ftl_read(sflash, address, data);
            
            address++;
        }

        (*sflash->interface->unlock)(sflash->context);
    }

    return status;
}

static int dosfs_sflash_write(void *context, uint32_t address, const uint8_t *data, uint32_t length, bool wait)
{
    dosfs_sflash_t *sflash = (dosfs_sflash_t*)context;
    int status = F_NO_ERROR;

#if (DOSFS_CONFIG_SFLASH_SIMULATE_TRACE == 1)
    printf("SFLASH_WRITE %08x, %d\n", address, length);
#endif /* (DOSFS_CONFIG_SFLASH_SIMULATE_TRACE == 1) */

    if (sflash->state != DOSFS_SFLASH_STATE_READY)
    {
        status = F_ERR_ONDRIVE;
    }
    else
    {
        (*sflash->interface->lock)(sflash->context);

        while (length--)
        {
            dosfs_sflash_ftl_write(sflash, address, data);
            
            address++;
        }

        (*sflash->interface->unlock)(sflash->context);
    }

#if (DOSFS_CONFIG_SFLASH_DEBUG == 1)
    dosfs_sflash_ftl_check();
#endif

    return status;
}

static int dosfs_sflash_sync(void *context)
{
    dosfs_sflash_t *sflash = (dosfs_sflash_t*)context;
    int status = F_NO_ERROR;

    if (sflash->state != DOSFS_SFLASH_STATE_READY)
    {
        status = F_ERR_ONDRIVE;
    }

    return status;
}

static const dosfs_device_interface_t dosfs_sflash_interface = {
    dosfs_sflash_release,
    dosfs_sflash_info,
    dosfs_sflash_notify,
    dosfs_sflash_format,
    dosfs_sflash_erase,
    dosfs_sflash_discard,
    dosfs_sflash_read,
    dosfs_sflash_write,
    dosfs_sflash_sync,
};

int dosfs_sflash_init(uint32_t data_start)
{
    dosfs_sflash_t *sflash = (dosfs_sflash_t*)&dosfs_sflash;
    int status = F_NO_ERROR;
    uint32_t data[DOSFS_BLK_SIZE / sizeof(uint32_t)];

    if (!dosfs_sflash_device.interface)
    {
        status = F_ERR_INITFUNC;
    }
    else
    {
        sflash->interface = dosfs_sflash_device.interface;
        sflash->context = dosfs_sflash_device.context;

        dosfs_device.lock = DOSFS_DEVICE_LOCK_INIT;
        dosfs_device.context = (void*)sflash;
        dosfs_device.interface = &dosfs_sflash_interface;
    
        if (sflash->state == DOSFS_SFLASH_STATE_NONE)
        {
            sflash->data_start = data_start;
            sflash->data_limit = (*sflash->interface->capacity)(sflash->context);

            if (sflash->data_limit <= sflash->data_start)
            {
                status = F_ERR_INVALIDMEDIA;
            }
            else
            {
                sflash->xlate_count = (((((sflash->data_limit - sflash->data_start) / DOSFS_SFLASH_ERASE_SIZE) * ((DOSFS_SFLASH_ERASE_SIZE / DOSFS_SFLASH_BLOCK_SIZE) -1)) -2) + (DOSFS_SFLASH_XLATE_ENTRIES -1)) / DOSFS_SFLASH_XLATE_ENTRIES;
            
                sflash->cache[0] = &dosfs_sflash_cache[0];
                sflash->cache[1] = &dosfs_sflash_cache[DOSFS_SFLASH_BLOCK_SIZE / sizeof(uint32_t)];

                (*sflash->interface->lock)(sflash->context);

                if (!dosfs_sflash_ftl_mount(sflash))
                {
                    (*sflash->interface->unlock)(sflash->context);

                    status = dosfs_device_format(&dosfs_device, (uint8_t*)data);

                    if (status == F_NO_ERROR)
                    {
                        sflash->state = DOSFS_SFLASH_STATE_READY;
                    }
                    else
                    {
                        sflash->state = DOSFS_SFLASH_STATE_NOT_FORMATTED;

                        status = F_ERR_NOTFORMATTED;
                    }
                }
                else
                {
                    (*sflash->interface->unlock)(sflash->context);

                    sflash->state = DOSFS_SFLASH_STATE_READY;
                }
            }
        }

        dosfs_device.lock = 0;
    }

    return status;
}
