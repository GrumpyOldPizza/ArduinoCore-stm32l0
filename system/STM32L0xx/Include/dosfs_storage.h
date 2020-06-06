/*
 * Copyright (c) 2019-2020 Thomas Roell.  All rights reserved.
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

#if !defined(_DOSFS_STORAGE_H)
#define _DOSFS_STORAGE_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct _dosfs_storage_interface_t {
    bool (*Init)(uint8_t **p_cache_data, const uint8_t **p_inquiry_data);
    bool (*DeInit)(void);
    bool (*IsReady)(void);
    bool (*GetCapacity)(uint32_t *pblock_count, uint32_t *p_block_size);
    bool (*GetWriteProtected)(bool *p_write_protected);
    bool (*GetChanged)(bool *p_changed);
    bool (*StartStopUnit)(bool start, bool loej);
    bool (*PreventAllowMediumRemoval)(bool prevent);
    bool (*Acquire)(void);
    void (*Release)(void);
    bool (*Read)(uint8_t *data, uint32_t blk_addr, uint32_t blk_len, bool release);
    bool (*Write)(const uint8_t *data, uint32_t blk_addr, uint32_t blk_len, bool release);
} dosfs_storage_interface_t;

extern const dosfs_storage_interface_t dosfs_storage_interface;
     
#ifdef __cplusplus
}
#endif

#endif /*_DOSFS_STORAGE_H */
