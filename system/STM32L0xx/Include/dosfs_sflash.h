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

#if !defined(_DOSFS_SFLASH_H)
#define _DOSFS_SFLASH_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "dosfs_device.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DOSFS_SFLASH_INFO_SLOT_IDENT_0             0
#define DOSFS_SFLASH_INFO_SLOT_IDENT_1             1
#define DOSFS_SFLASH_INFO_SLOT_DATA_START          2
#define DOSFS_SFLASH_INFO_SLOT_DATA_LIMIT          3
#define DOSFS_SFLASH_INFO_SLOT_ERASE_COUNT         4
#define DOSFS_SFLASH_INFO_SLOT_RECLAIM_ERASE_COUNT 5
#define DOSFS_SFLASH_INFO_SLOT_RESERVED_6          6
#define DOSFS_SFLASH_INFO_SLOT_RESERVED_7          7
#define DOSFS_SFLASH_INFO_SLOT_COUNT               8

#define DOSFS_SFLASH_INFO_HEAD_OFFSET              0
#define DOSFS_SFLASH_INFO_HEAD_SIZE                32

#define DOSFS_SFLASH_INFO_TAIL_OFFSET              508
#define DOSFS_SFLASH_INFO_TAIL_SIZE                4

#define DOSFS_SFLASH_SECTOR_IDENT_0                0x52444154     /* "RFAT" */
#define DOSFS_SFLASH_SECTOR_IDENT_1                0x4e4f3032     /* "NO02" */

#define DOSFS_SFLASH_SECTOR_IDENT_TAIL             0xaa550000


/* RESERVED -> XLATE_SECONDARY -> XLATE -> DELETED
 * RESERVED -> DATA_WRITTEN -> DATA_COMMITTED -> DATA_DELETED -> DELETED
 * RESERVED -> RECLAIM -> ERASE -> VICTIM -> DELETED
 */
#define DOSFS_SFLASH_INFO_TYPE_MASK                0xf00000
#define DOSFS_SFLASH_INFO_TYPE_DELETED             0x000000     /* 0000 sector is deleted                      */
#define DOSFS_SFLASH_INFO_TYPE_UNUSED_0001         0x100000     /* 0001                                        */
#define DOSFS_SFLASH_INFO_TYPE_UNUSED_0010         0x200000     /* 0010                                        */
#define DOSFS_SFLASH_INFO_TYPE_UNUSED_0011         0x300000     /* 0011                                        */
#define DOSFS_SFLASH_INFO_TYPE_UNUSED_0101         0x500000     /* 0101                                        */
#define DOSFS_SFLASH_INFO_TYPE_UNUSED_0111         0x700000     /* 0111                                        */
#define DOSFS_SFLASH_INFO_TYPE_UNUSED_1001         0x900000     /* 1001                                        */
#define DOSFS_SFLASH_INFO_TYPE_DATA_DELETED        0x400000     /* 0100 sector is user data (to be deleted)    */
#define DOSFS_SFLASH_INFO_TYPE_DATA_COMMITTED      0x600000     /* 0110 sector is user data (valid)            */
#define DOSFS_SFLASH_INFO_TYPE_VICTIM              0x800000     /* 1000 sector is victim header                */
#define DOSFS_SFLASH_INFO_TYPE_ERASE               0xa00000     /* 1010 sector is erase header (one bit off)   */
#define DOSFS_SFLASH_INFO_TYPE_RECLAIM             0xb00000     /* 1011 sector is reclaim header               */
#define DOSFS_SFLASH_INFO_TYPE_XLATE               0xc00000     /* 1100 sector is logical to physical xlate    */
#define DOSFS_SFLASH_INFO_TYPE_XLATE_SECONDARY     0xd00000     /* 1101 sector is patch to xlate               */
#define DOSFS_SFLASH_INFO_TYPE_DATA_WRITTEN        0xe00000     /* 1110 sector is user data (not comitted)     */
#define DOSFS_SFLASH_INFO_TYPE_RESERVED            0xf00000     /* 1111 sector is reserved                     */
#define DOSFS_SFLASH_INFO_NOT_WRITTEN_TO           0x080000     /* sector is erased (i.e. not written to)      */
#define DOSFS_SFLASH_INFO_DATA_MASK                0x07ffff 
#define DOSFS_SFLASH_INFO_ERASED                   0xffffff
#define DOSFS_SFLASH_INFO_ENTRY_OFFSET             124
#define DOSFS_SFLASH_INFO_ENTRY_SIZE               3
#define DOSFS_SFLASH_INFO_ENTRY_COUNT              128

#define DOSFS_SFLASH_BLOCK_NOT_ALLOCATED           0xffff
#define DOSFS_SFLASH_BLOCK_RESERVED                0xfffe
#define DOSFS_SFLASH_BLOCK_DELETED                 0x0000

#define DOSFS_SFLASH_PAGE_SIZE                     0x00000100
#define DOSFS_SFLASH_BLOCK_SIZE                    0x00000200
#define DOSFS_SFLASH_ERASE_SIZE                    0x00010000
#define DOSFS_SFLASH_DATA_SIZE                     0x01000000

#define DOSFS_SFLASH_XLATE_ENTRIES                 256
#define DOSFS_SFLASH_XLATE_SEGMENT_SHIFT           8
#define DOSFS_SFLASH_XLATE_INDEX_MASK              0x000000ff
#define DOSFS_SFLASH_XLATE_OFFSET                  64
#define DOSFS_SFLASH_XLATE_COUNT                   ((((DOSFS_SFLASH_DATA_SIZE / DOSFS_SFLASH_ERASE_SIZE) * ((DOSFS_SFLASH_ERASE_SIZE / DOSFS_SFLASH_BLOCK_SIZE) -1) -2) + (DOSFS_SFLASH_XLATE_ENTRIES -1)) / DOSFS_SFLASH_XLATE_ENTRIES)

#define DOSFS_SFLASH_XLATE_ENTRY_NOT_ALLOCATED     0xffff

#define DOSFS_SFLASH_BLOCK_INFO_ENTRIES            128

#define DOSFS_SFLASH_LOGICAL_BLOCK_MASK            0x0000007f
#define DOSFS_SFLASH_LOGICAL_SECTOR_MASK           0x00007f80
#define DOSFS_SFLASH_LOGICAL_SECTOR_SHIFT          7

#define DOSFS_SFLASH_LOGICAL_BLOCK_NOT_ALLOCATED   0x0000

#define DOSFS_SFLASH_PHYSICAL_ILLEGAL              0xffffffff

#define DOSFS_SFLASH_STATE_NONE                    0
#define DOSFS_SFLASH_STATE_READY                   1
#define DOSFS_SFLASH_STATE_NOT_FORMATTED           2

#define DOSFS_SFLASH_ERASE_COUNT_THRESHOLD         64

typedef struct _dosfs_sflash_t dosfs_sflash_t;
typedef struct _dosfs_sflash_interface_t dosfs_sflash_interface_t;

typedef void (*dosfs_sflash_notify_callback_t)(void *cookie, int acquire);


struct _dosfs_sflash_interface_t {
    uint32_t                (*capacity)(void *context);
    void                    (*notify)(void *context, dosfs_sflash_notify_callback_t callback, void *cookie);
    void                    (*lock)(void *context);
    void                    (*unlock)(void *context);
    bool                    (*erase)(void *context, uint32_t address);
    bool                    (*program)(void *context, uint32_t address, const uint8_t *data, uint32_t size);
    void                    (*read)(void *context, uint32_t address, uint8_t *data, uint32_t size);
};

struct _dosfs_sflash_t {
    uint8_t                 state;
    uint8_t                 address;
    uint16_t                xlate_count;
    uint32_t                data_start;
    uint32_t                data_limit;

    uint8_t                 sector_table[DOSFS_SFLASH_DATA_SIZE / DOSFS_SFLASH_ERASE_SIZE];
    uint16_t                block_table[DOSFS_SFLASH_XLATE_OFFSET];
    uint16_t                xlate_logical; 
    uint16_t                xlate_table[DOSFS_SFLASH_XLATE_COUNT];
    uint16_t                xlate2_logical; 
    uint16_t                xlate2_table[DOSFS_SFLASH_XLATE_COUNT];

    uint16_t                alloc_sector;
    uint8_t                 alloc_index;
    uint8_t                 alloc_count;
    uint32_t                alloc_mask[DOSFS_SFLASH_BLOCK_INFO_ENTRIES / 32];

    uint32_t                victim_offset;

    uint32_t                reclaim_offset;
    uint32_t                reclaim_erase_count;

    uint32_t                erase_count_max;

    uint32_t                *cache[2];

    const dosfs_sflash_interface_t *interface;
    void                    *context;

#if (DOSFS_CONFIG_STATISTICS == 1)
    struct {
        uint32_t                sflash_command_erase;
        uint32_t                sflash_command_program;
        uint32_t                sflash_command_read;
        uint32_t                sflash_nor_erase;
        uint32_t                sflash_nor_program;
        uint32_t                sflash_nor_read;
        uint32_t                sflash_nor_pfail;
        uint32_t                sflash_nor_efail;
        uint32_t                sflash_ftl_read;
        uint32_t                sflash_ftl_write;
        uint32_t                sflash_ftl_delete;
        uint32_t                sflash_ftl_xlate_miss;
        uint32_t                sflash_ftl_xlate_hit;
        uint32_t                sflash_ftl_xlate2_miss;
        uint32_t                sflash_ftl_xlate2_hit;
        uint32_t                sflash_ftl_reclaim;
    }                       statistics;
#endif /* (DOSFS_CONFIG_STATISTICS == 1) */
};

/*
 * victim_erase_count
 * reclaim_erase_count
 * ident_0
 * ident_1
 */

#if (DOSFS_CONFIG_STATISTICS == 1)

extern dosfs_sflash_t dosfs_sflash;

#define DOSFS_SFLASH_STATISTICS_COUNT(_name)         { dosfs_sflash.statistics._name += 1; }
#define DOSFS_SFLASH_STATISTICS_COUNT_N(_name,_n)    { dosfs_sflash.statistics._name += (_n); }

#else /* (DOSFS_CONFIG_STATISTICS == 1) */

#define DOSFS_SFLASH_STATISTICS_COUNT(_name)         /**/
#define DOSFS_SFLASH_STATISTICS_COUNT_N(_name,_n)    /**/

#endif /* (DOSFS_CONFIG_STATISTICS == 1) */

typedef struct _dosfs_sflash_device_t {
    const dosfs_sflash_interface_t *interface;
    void                           *context;
} dosfs_sflash_device_t;

extern dosfs_sflash_device_t dosfs_sflash_device;

extern int dosfs_sflash_init(uint32_t data_start);

#ifdef __cplusplus
}
#endif

#endif /*_DOSFS_SFLASH_H */
