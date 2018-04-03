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

#if !defined(_DOSFS_CONFIG_h)
#define _DOSFS_CONFIG_h

#ifdef __cplusplus
 extern "C" {
#endif

#define DOSFS_CONFIG_FAT12_SUPPORTED            1
#define DOSFS_CONFIG_VFAT_SUPPORTED             1
#define DOSFS_CONFIG_UTF8_SUPPORTED             1
#define DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED 0
#define DOSFS_CONFIG_CONTIGUOUS_SUPPORTED       1
#define DOSFS_CONFIG_SEQUENTIAL_SUPPORTED       1
#define DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED    0
#define DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED     0
#define DOSFS_CONFIG_FSINFO_SUPPORTED           0
#define DOSFS_CONFIG_2NDFAT_SUPPORTED           1

#define DOSFS_CONFIG_FAT_CACHE_ENTRIES          1
#define DOSFS_CONFIG_DATA_CACHE_ENTRIES         0
#define DOSFS_CONFIG_FILE_DATA_CACHE            0
#define DOSFS_CONFIG_CLUSTER_CACHE_ENTRIES      0
#define DOSFS_CONFIG_META_DATA_RETRIES          3
#define DOSFS_CONFIG_STATISTICS                 0

#define DOSFS_CONFIG_SDCARD_HIGH_SPEED          0
#define DOSFS_CONFIG_SDCARD_CRC                 1
#define DOSFS_CONFIG_SDCARD_COMMAND_RETRIES     4
#define DOSFS_CONFIG_SDCARD_DATA_RETRIES        4
#define DOSFS_CONFIG_SDCARD_SIMULATE            0
#define DOSFS_CONFIG_SDCARD_SIMULATE_BLKCNT     (unsigned long)(65536 * 64)
#define DOSFS_CONFIG_SDCARD_SIMULATE_TRACE      0

#define DOSFS_CONFIG_SFLASH_SIMULATE            0
#define DOSFS_CONFIG_SFLASH_SIMULATE_DATA_SIZE  0x01000000
#define DOSFS_CONFIG_SFLASH_SIMULATE_TRACE      0
#define DOSFS_CONFIG_SFLASH_DEBUG               0

#ifdef __cplusplus
}
#endif

#endif /* _DOSFS_CONFIG_h */

