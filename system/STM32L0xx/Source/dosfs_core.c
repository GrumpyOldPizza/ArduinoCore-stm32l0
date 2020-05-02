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

#include "dosfs_core.h"

#include "armv6m.h"


static int dosfs_volume_init(dosfs_volume_t *volume, dosfs_device_t *device);
static int dosfs_volume_mount(dosfs_volume_t *volume);
static int dosfs_volume_unmount(dosfs_volume_t *volume);
static int dosfs_volume_lock(dosfs_volume_t *volume);
static int dosfs_volume_lock_noinit(dosfs_volume_t *volume);
static int dosfs_volume_lock_nomount(dosfs_volume_t *volume);
static int dosfs_volume_unlock(dosfs_volume_t *volume, int status);
static int dosfs_volume_read(dosfs_volume_t *volume, uint32_t address, uint8_t *data);
static int dosfs_volume_write(dosfs_volume_t *volume, uint32_t address, const uint8_t *data);
static int dosfs_volume_zero(dosfs_volume_t *volume, uint32_t address, uint32_t length, bool wait);
#if (DOSFS_CONFIG_FSINFO_SUPPORTED == 1)
static int dosfs_volume_fsinfo(dosfs_volume_t *volume, uint32_t free_clscnt, uint32_t next_clsno);
#endif /* (DOSFS_CONFIG_FSINFO_SUPPORTED == 1) */
#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0)
static int dosfs_volume_dirty(dosfs_volume_t *volume);
static int dosfs_volume_clean(dosfs_volume_t *volume, int status_o);
#else /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */
static int dosfs_volume_record(dosfs_volume_t *volume);
static int dosfs_volume_commit(dosfs_volume_t *volume);
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */
static int dosfs_volume_format(dosfs_volume_t *volume);

static int dosfs_dir_cache_write(dosfs_volume_t *volume);
static int dosfs_dir_cache_fill(dosfs_volume_t *volume, uint32_t blkno, int zero);
static int dosfs_dir_cache_read(dosfs_volume_t *volume, uint32_t blkno, dosfs_cache_entry_t **p_entry);
static int dosfs_dir_cache_zero(dosfs_volume_t *volume, uint32_t blkno, dosfs_cache_entry_t **p_entry);
static int dosfs_dir_cache_flush(dosfs_volume_t *volume);

#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)
static int dosfs_map_cache_fill(dosfs_volume_t *volume, uint32_t page);
static int dosfs_map_cache_write(dosfs_volume_t *volume, uint32_t blkno, const uint8_t *data);
static int dosfs_map_cache_read(dosfs_volume_t *volume, uint32_t blkno, uint8_t *data);
static int dosfs_map_cache_flush(dosfs_volume_t *volume);
static int dosfs_map_cache_resolve(dosfs_volume_t *volume);
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */

static int dosfs_fat_cache_write(dosfs_volume_t *volume, dosfs_cache_entry_t *entry);
static int dosfs_fat_cache_fill(dosfs_volume_t *volume, uint32_t blkno, dosfs_cache_entry_t **p_entry);
static int dosfs_fat_cache_read(dosfs_volume_t *volume, uint32_t blkno, dosfs_cache_entry_t **p_entry);
static void dosfs_fat_cache_modify(dosfs_volume_t *volume, dosfs_cache_entry_t *entry);
static int dosfs_fat_cache_flush(dosfs_volume_t *volume);

static int dosfs_data_cache_write(dosfs_volume_t *volume, dosfs_file_t *file);
static int dosfs_data_cache_fill(dosfs_volume_t *volume, dosfs_file_t *file, uint32_t blkno, int zero);
static int dosfs_data_cache_read(dosfs_volume_t *volume, dosfs_file_t *file, uint32_t blkno, dosfs_cache_entry_t ** p_entry);
static int dosfs_data_cache_zero(dosfs_volume_t *volume, dosfs_file_t *file, uint32_t blkno, dosfs_cache_entry_t ** p_entry);
static void dosfs_data_cache_modify(dosfs_volume_t *volume, dosfs_file_t *file);
static int dosfs_data_cache_flush(dosfs_volume_t *volume, dosfs_file_t *file);

static int dosfs_cluster_read_uncached(dosfs_volume_t *volume, uint32_t clsno, uint32_t *p_clsdata);
static int dosfs_cluster_read(dosfs_volume_t *volume, uint32_t clsno, uint32_t *p_clsdata);
static int dosfs_cluster_write(dosfs_volume_t *volume, uint32_t clsno, uint32_t clsdata, int allocate);
static int dosfs_cluster_chain_seek(dosfs_volume_t *volume, uint32_t clsno, uint32_t clscnt, uint32_t *p_clsno);
static int dosfs_cluster_chain_create(dosfs_volume_t *volume, uint32_t clsno, uint32_t clscnt, uint32_t *p_clsno_a, uint32_t *p_clsno_l);
#if (DOSFS_CONFIG_SEQUENTIAL_SUPPORTED == 1)
static int dosfs_cluster_chain_create_sequential(dosfs_volume_t *volume, uint32_t clsno, uint32_t clscnt, uint32_t *p_clsno_a, uint32_t *p_clsno_l);
#endif /* (DOSFS_CONFIG_SEQUENTIAL_SUPPORTED == 1) */
#if (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1)
static int dosfs_cluster_chain_create_contiguous(dosfs_volume_t *volume, uint32_t clscnt, uint32_t *p_clsno_a);
#endif /* (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1) */
static int dosfs_cluster_chain_destroy(dosfs_volume_t *volume, uint32_t clsno, uint32_t clsdata);

static unsigned int dosfs_name_ascii_upcase(unsigned int cc);
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 0)
static const char *dosfs_name_cstring_to_dosname(const char *cstring, uint8_t *dosname);
static const char *dosfs_name_cstring_to_pattern(const char *cstring, char *pattern);
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 0) */
static const char *dosfs_name_cstring_to_label(const char *cstring, uint8_t *label);
static char *dosfs_name_dosname_to_cstring(const uint8_t *dosname, unsigned int doscase, char *cstring, char *cstring_e);
static char *dosfs_name_label_to_cstring(const uint8_t *label, char *cstring, char *cstring_e);
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
#if (DOSFS_CONFIG_UTF8_SUPPORTED == 1)
static unsigned int dosfs_name_cstring_to_unicode(const char *cstring, const char **p_cstring);
static char * dosfs_name_unicode_to_cstring(unsigned int cc, char *cstring, char *cstring_e);
static unsigned int dosfs_name_unicode_upcase(unsigned int cc);
#endif /* (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */
static const char *dosfs_name_cstring_to_uniname(const char *cstring, dosfs_unicode_t *uniname, uint8_t *p_unicount, uint8_t *dosname, uint8_t *p_doscase);
static char *dosfs_name_uniname_to_cstring(const dosfs_unicode_t *uniname, unsigned int unicount, char *cstring, char *cstring_e);
static void dosfs_name_uniname_to_dosname(const dosfs_unicode_t *uniname, unsigned int unicount, uint8_t *dosname, unsigned int *p_dosprefix);
static uint8_t dosfs_name_checksum_dosname(const uint8_t *dosname);
static uint16_t dosfs_name_checksum_uniname(const dosfs_unicode_t *uniname, unsigned int unicount);
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */

static int dosfs_path_convert_filename(dosfs_volume_t *volume, const char *filename, const char **p_filename);
static int dosfs_path_find_entry(dosfs_volume_t *volume, uint32_t clsno, uint32_t index, uint32_t count, dosfs_find_callback_t callback, void *private, uint32_t *p_clsno, uint32_t *p_index, dosfs_dir_t **p_dir);
static int dosfs_path_find_directory(dosfs_volume_t *volume, const char *filename, const char **p_filename, uint32_t *p_clsno);
static int dosfs_path_find_file(dosfs_volume_t *volume, const char *filename, uint32_t *p_clsno, uint32_t *p_index, dosfs_dir_t **p_dir);
static int dosfs_path_find_pattern(dosfs_volume_t *volume, F_DIR *parent, char *pattern, char *filename, dosfs_dir_t **p_dir);

static void dosfs_path_setup_entry(dosfs_volume_t *volume, const char *dosname, uint8_t attr, uint32_t first_clsno, uint16_t ctime, uint16_t cdate, dosfs_dir_t *dir);
static int dosfs_path_create_entry(dosfs_volume_t *volume, uint32_t clsno_d, uint32_t clsno, uint32_t index, const char *dosname, uint8_t attr, uint32_t first_clsno, uint16_t ctime, uint16_t cdate);
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
static int dosfs_path_destroy_entry(dosfs_volume_t *volume, uint32_t clsno, uint32_t index, uint32_t entries, uint32_t first_clsno);
#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
static int dosfs_path_destroy_entry(dosfs_volume_t *volume, uint32_t clsno, uint32_t index, uint32_t first_clsno);
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */

static dosfs_file_t *dosfs_file_enumerate(dosfs_volume_t *volume, dosfs_file_t *file, uint32_t clsno, uint32_t index);
static int dosfs_file_sync(dosfs_volume_t *volume, dosfs_file_t *file, int access, int modify, uint32_t first_clsno, uint32_t length);
static int dosfs_file_flush(dosfs_volume_t *volume, dosfs_file_t *file, int close);
static int dosfs_file_seek(dosfs_volume_t *volume, dosfs_file_t *file, uint32_t position);
static int dosfs_file_shrink(dosfs_volume_t *volume, dosfs_file_t *file);
static int dosfs_file_extend(dosfs_volume_t *volume, dosfs_file_t *file, uint32_t length);
#if (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1)
static int dosfs_file_reserve(dosfs_volume_t *volume, dosfs_file_t *file, uint32_t size);
#endif /* (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1) */
static int dosfs_file_open(dosfs_volume_t *volume, dosfs_file_t *file, const char *filename, uint32_t mode, uint32_t size);
static int dosfs_file_close(dosfs_volume_t *volume, dosfs_file_t *file);
static int dosfs_file_read(dosfs_volume_t *volume, dosfs_file_t *file, uint8_t *data, uint32_t count, uint32_t *p_count);
static int dosfs_file_write(dosfs_volume_t *volume, dosfs_file_t *file, const uint8_t *data, uint32_t count, uint32_t *p_count);


dosfs_volume_t dosfs_volume;

__attribute__((section(".noinit"))) static uint32_t dosfs_cache[(1 +
                                                                 DOSFS_CONFIG_FAT_CACHE_ENTRIES +
                                                                 ((DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) ? 1 : 0) +
                                                                 DOSFS_CONFIG_DATA_CACHE_ENTRIES)
                                                                * (DOSFS_BLK_SIZE / sizeof(uint32_t))];

static const char dosfs_dirname_dot[11]    = ".          ";
static const char dosfs_dirname_dotdot[11] = "..         ";

#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)

static const uint8_t dosfs_path_ldir_name_table[13] = {
    offsetof(dosfs_ldir_t, ldir_name_1[0]),
    offsetof(dosfs_ldir_t, ldir_name_1[2]),
    offsetof(dosfs_ldir_t, ldir_name_1[4]),
    offsetof(dosfs_ldir_t, ldir_name_1[6]),
    offsetof(dosfs_ldir_t, ldir_name_1[8]),
    offsetof(dosfs_ldir_t, ldir_name_2[0]),
    offsetof(dosfs_ldir_t, ldir_name_2[2]),
    offsetof(dosfs_ldir_t, ldir_name_2[4]),
    offsetof(dosfs_ldir_t, ldir_name_2[6]),
    offsetof(dosfs_ldir_t, ldir_name_2[8]),
    offsetof(dosfs_ldir_t, ldir_name_2[10]),
    offsetof(dosfs_ldir_t, ldir_name_3[0]),
    offsetof(dosfs_ldir_t, ldir_name_3[2]),
};

#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */


static int dosfs_volume_init(dosfs_volume_t *volume, dosfs_device_t *device)
{
    int status = F_NO_ERROR;
    uint8_t *cache;

#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)
    /* In order not to mess up dosfs_volume_t too much, the dosfs_boot_t.bpblog struct is aliased to
     * dosfs_volume_t. The asserts below ensure that the struct elements don't move and mirror
     * the location relative to dosfs_boot_t.
     */
    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.log_map_flags) >= offsetof(dosfs_boot_t, bpb40.bs_reserved_1));
    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.log_map_flags) >= offsetof(dosfs_boot_t, bpb71.bs_reserved_2));
    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.bs_trail_sig) == 510);

    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.log_map_flags) == (offsetof(dosfs_volume_t, map_flags) - offsetof(dosfs_volume_t, bs_data)));
    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.log_map_entries) == (offsetof(dosfs_volume_t, map_entries) - offsetof(dosfs_volume_t, bs_data)));
    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.log_map_table) == (offsetof(dosfs_volume_t, map_table) - offsetof(dosfs_volume_t, bs_data)));
    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.log_map_blkno) == (offsetof(dosfs_volume_t, map_blkno) - offsetof(dosfs_volume_t, bs_data)));
    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.log_free_clscnt) == (offsetof(dosfs_volume_t, free_clscnt) - offsetof(dosfs_volume_t, bs_data)));
    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.log_next_clsno) == (offsetof(dosfs_volume_t, next_clsno) - offsetof(dosfs_volume_t, bs_data)));
    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.log_dir) == (offsetof(dosfs_volume_t, dir) - offsetof(dosfs_volume_t, bs_data)));
    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.log_dir_flags) == (offsetof(dosfs_volume_t, dir_flags) - offsetof(dosfs_volume_t, bs_data)));
    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.log_dir_entries) == (offsetof(dosfs_volume_t, dir_entries) - offsetof(dosfs_volume_t, bs_data)));
    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.log_dir_index) == (offsetof(dosfs_volume_t, dir_index) - offsetof(dosfs_volume_t, bs_data)));
    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.log_dir_clsno) == (offsetof(dosfs_volume_t, dir_clsno) - offsetof(dosfs_volume_t, bs_data)));
    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.log_dot_clsno) == (offsetof(dosfs_volume_t, dot_clsno) - offsetof(dosfs_volume_t, bs_data)));
    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.log_del_clsno) == (offsetof(dosfs_volume_t, del_clsno) - offsetof(dosfs_volume_t, bs_data)));
    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.log_del_index) == (offsetof(dosfs_volume_t, del_index) - offsetof(dosfs_volume_t, bs_data)));
    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.log_del_entries) == (offsetof(dosfs_volume_t, del_entries) - offsetof(dosfs_volume_t, bs_data)));

#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.log_lfn_count) == (offsetof(dosfs_volume_t, lfn_count) - offsetof(dosfs_volume_t, bs_data)));
    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.log_lfn_blkno) == (offsetof(dosfs_volume_t, lfn_blkno) - offsetof(dosfs_volume_t, bs_data)));
#if (DOSFS_CONFIG_UTF8_SUPPORTED == 0)
    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.log_lfn_name) == (offsetof(dosfs_volume_t, lfn_name) - offsetof(dosfs_volume_t, bs_data)));
    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.bs_trail_sig) == (offsetof(dosfs_volume_t, bs_trail_sig) - offsetof(dosfs_volume_t, bs_data)));
#else /* (DOSFS_CONFIG_UTF8_SUPPORTED == 0) */
    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.bs_trail_sig) == (offsetof(dosfs_volume_t, bs_trail_sig) - offsetof(dosfs_volume_t, lfn_name)));
#endif /* (DOSFS_CONFIG_UTF8_SUPPORTED == 0) */
#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
    DOSFS_CT_ASSERT(offsetof(dosfs_boot_t, bpblog.bs_trail_sig) == (offsetof(dosfs_volume_t, bs_trail_sig) - offsetof(dosfs_volume_t, bs_data)));
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */

    cache = (uint8_t*)&dosfs_cache[0];
    volume->dir_cache.data = cache;
    cache += DOSFS_BLK_SIZE;
#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)
    volume->map_cache.data = cache;
    cache += DOSFS_BLK_SIZE;
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */

#if (DOSFS_CONFIG_FAT_CACHE_ENTRIES != 0)
#if (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 1)
    volume->fat_cache.data = cache;
    cache += DOSFS_BLK_SIZE;
#else /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 1) */
    volume->fat_cache[0].data = cache;
    cache += DOSFS_BLK_SIZE;
    volume->fat_cache[1].data = cache;
    cache += DOSFS_BLK_SIZE;
#endif /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 1) */
#endif /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES != 0) */

#if (DOSFS_CONFIG_DATA_CACHE_ENTRIES != 0)
#if (DOSFS_CONFIG_FILE_DATA_CACHE == 0)
    volume->data_cache.data = cache;
    cache += DOSFS_BLK_SIZE;
#endif /* (DOSFS_CONFIG_FILE_DATA_CACHE == 0) */
#endif /* (DOSFS_CONFIG_DATA_CACHE_ENTRIES != 0) */

    if (device->interface)
    {
        volume->state = DOSFS_VOLUME_STATE_INITIALIZED;
        volume->flags = 0;
        volume->media = DOSFS_MEDIA_NONE;
    }
    else
    {
        status = F_ERR_INITFUNC;
    }

    return status;
}

static int dosfs_volume_mount(dosfs_volume_t * volume)
{
    int status = F_NO_ERROR;
    dosfs_device_t *device;
    uint8_t media, write_protected;
    uint32_t boot_blkno, tot_sec, cls_mask, cls_shift, blkcnt, au_size, product;
#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)
    uint32_t boot_blkcnt;
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */
#if (DOSFS_CONFIG_FSINFO_SUPPORTED == 1)
    uint32_t next_clsno;
#endif /* (DOSFS_CONFIG_FSINFO_SUPPORTED == 1) */
    dosfs_boot_t *boot;
#if (DOSFS_CONFIG_CLUSTER_CACHE_ENTRIES != 0)
    unsigned int index;
#endif /* (DOSFS_CONFIG_CLUSTER_CACHE_ENTRIES != 0) */

    device = DOSFS_VOLUME_DEVICE(volume);

#if (DOSFS_CONFIG_STATISTICS == 1)
    memset(&volume->statistics, 0, sizeof(volume->statistics));
#endif /* (DOSFS_CONFIG_STATISTICS == 1) */

    status = (*device->interface->info)(device->context, &media, &write_protected, &blkcnt, &au_size, &product);

    if (status == F_NO_ERROR)
    {
        if (volume->state == DOSFS_VOLUME_STATE_CARDREMOVED)
        {
            if (volume->product != product)
            {
                volume->state = DOSFS_VOLUME_STATE_UNUSABLE;
                
                status = F_ERR_UNUSABLE;
            }
#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)
            else
            {
                /* Check whether the log needs to be replayed after a SDCARD power failure.
                 */
                if ((volume->log_lead_sig == DOSFS_HTOFL(DOSFS_LOG_LEAD_SIG)) &&
                    (volume->log_struct_sig == DOSFS_HTOFL(DOSFS_LOG_STRUCT_SIG)))
                {
                    status = dosfs_volume_commit(volume);
                }
            }
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */
        }
        else
        {
            volume->flags = 0;
        
            if (write_protected)
            {
                volume->flags |= DOSFS_VOLUME_FLAG_WRITE_PROTECTED;
            }

            volume->media = media;
            volume->product = product;
            volume->serial = 0;

            boot_blkno = 0;

#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)
            /* For a FAT32 file system TRANSACTION_SAFE needs 3 blocks,
             * which are #3, #4 & #5, which are conveniantly reserved
             * by the FAT32 spec. FAT16 is somewhat more tricky. There
             * are 2 ways to lay out the BPB relative to FAT/DIR and still
             * respect the erase boundary. One is to used a fixed value
             * of 1 for bpb_rsvd_sec_cnt. That implies that there is at
             * least 1 unused block before the BPB (since FAT/DIR always
             * have an even count). The other variant is to anchor the BPB
             * at an erase boundary, which then results in a bpb_rsvd_sec_cnt
             * of at least 2, again 1 reserved block.
             */
            boot_blkcnt = 0;

            boot = (dosfs_boot_t*)((void*)&volume->bs_data[0]);

#else /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */

            boot = (dosfs_boot_t*)((void*)volume->dir_cache.data);

#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */

            status = dosfs_volume_read(volume, boot_blkno, (uint8_t*)boot);

            if (status == F_NO_ERROR)
            {
                /* This is somewhat iffy. MMC/SDSC do support a FILE_FORMAT setting in CSD
                 * that points to a FDC descriptor, without partition table. However the
                 * default is to use a partition table (with BPB or extended BPB). Thus
                 * there are checks in place for the presence of a FDC descriptor, and
                 * a validation check for the partition table to filter out exFAT for SDXC.
                 */
               
                if (boot->bs.bs_trail_sig != DOSFS_HTOFS(0xaa55))
                {
                    status = F_ERR_NOTFORMATTED;
                }
                else
                {
                    if (!((boot->bpb.bs_jmp_boot[0] == 0xe9) ||
                          ((boot->bpb.bs_jmp_boot[0] == 0xeb) && (boot->bpb.bs_jmp_boot[2] == 0x90))))
                    {
                        if ((boot->mbr.mbr_par_table[0].mbr_sys_id == 0x01) ||
                            (boot->mbr.mbr_par_table[0].mbr_sys_id == 0x04) ||
                            (boot->mbr.mbr_par_table[0].mbr_sys_id == 0x06) ||
                            (boot->mbr.mbr_par_table[0].mbr_sys_id == 0x0b) ||
                            (boot->mbr.mbr_par_table[0].mbr_sys_id == 0x0c))
                        {
                            boot_blkno = DOSFS_FTOHL(boot->mbr.mbr_par_table[0].mbr_rel_sec);

#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)
                            if ((boot->mbr.mbr_par_table[1].mbr_sys_id == 0x00) &&
                                (boot->mbr.mbr_par_table[2].mbr_sys_id == 0x00) &&
                                (boot->mbr.mbr_par_table[3].mbr_sys_id == 0x00))
                            {
                                boot_blkcnt = boot_blkno -1;
                            }
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */

                            status = dosfs_volume_read(volume, boot_blkno, (uint8_t*)boot);
                        
                            if (status == F_NO_ERROR)
                            {
                                if ((boot->bs.bs_trail_sig != DOSFS_HTOFS(0xaa55)) ||
                                    !((boot->bpb.bs_jmp_boot[0] == 0xe9) ||
                                      ((boot->bpb.bs_jmp_boot[0] == 0xeb) && (boot->bpb.bs_jmp_boot[2] == 0x90))))
                                {
                                    status = F_ERR_NOTFORMATTED;
                                }
                            }
                        }
                        else
                        {
                            status = F_ERR_INVALIDMEDIA;
                        }
                    }
                }

                if (status == F_NO_ERROR)
                {
                    /* Here we have a valid PBR signature, check whether there is a valid BPB.
                     */

                    /* Legal values for bpb_byts_per_sec are 512, 1024, 2048 and 4096.
                     * Hence if 0x8000 is ored in, it will be illegal. This 0x8000 mask is
                     * used to figure out whether TRANSACTION_SAFE was in dosfs_volume_commit
                     * or not.
                     */

                    if ((boot->bpb.bpb_byts_per_sec != DOSFS_HTOFS(DOSFS_BLK_SIZE))
#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)
                        && ((boot->bpb.bpb_byts_per_sec != (0x8000 | DOSFS_HTOFS(DOSFS_BLK_SIZE))) ||
                            (boot->bpblog.log_lead_sig != DOSFS_HTOFL(DOSFS_LOG_LEAD_SIG)) ||
                            (boot->bpblog.log_struct_sig != DOSFS_HTOFL(DOSFS_LOG_STRUCT_SIG)))
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */
                        )
                    {
                        status = F_ERR_NOTSUPPSECTORSIZE;
                    }
                    else if (!boot->bpb.bpb_sec_per_clus || (boot->bpb.bpb_sec_per_clus & (boot->bpb.bpb_sec_per_clus -1)))
                    {
                        status = F_ERR_INVALIDMEDIA;
                    }
                    else if ((boot->bpb.bpb_num_fats == 0) || (boot->bpb.bpb_num_fats > 2))
                    {
                        status = F_ERR_INVALIDMEDIA;
                    }
                    else if (!((boot->bpb.bpb_media == 0xf0) || (boot->bpb.bpb_media >= 0xf8)))
                    {
                        status = F_ERR_INVALIDMEDIA;
                    }
                    else
                    {
#if (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1) && (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0)
                        memcpy(&volume->bs_data[0], boot, sizeof(volume->bs_data));
#endif /* (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1) && (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */

                        volume->cls_size = (boot->bpb.bpb_sec_per_clus * DOSFS_BLK_SIZE);
                        volume->cls_mask = volume->cls_size -1;

                        for (cls_mask = 0x8000, cls_shift = 16; !(volume->cls_mask & cls_mask); cls_mask >>= 1, cls_shift--) { }

                        volume->cls_shift = cls_shift;
                        volume->cls_blk_shift = cls_shift - DOSFS_BLK_SHIFT;
                        volume->cls_blk_size = (1 << volume->cls_blk_shift);
                        volume->cls_blk_mask = volume->cls_blk_size -1;

                        volume->boot_blkno = boot_blkno;

                        /* FAT32 differs from FAT12/FAT16 by having bpb_fat_sz_16 forced to 0. 
                         * The bpb_root_ent_cnt is ignored for the decision.
                         */
                        if (boot->bpb.bpb_fat_sz_16 != 0)
                        {
                            /* FAT12/FAT16 */

                            volume->fsinfo_blkofs = 0;
                            volume->bkboot_blkofs = 0;

                            if (boot->bpb.bpb_tot_sec_16 != 0)
                            {
                                tot_sec = DOSFS_FTOHS(boot->bpb.bpb_tot_sec_16);
                            }
                            else
                            {
                                tot_sec = DOSFS_FTOHL(boot->bpb40.bpb_tot_sec_32);
                            }

                            volume->fat_blkcnt = DOSFS_FTOHS(boot->bpb.bpb_fat_sz_16);
                            volume->fat1_blkno = volume->boot_blkno + DOSFS_FTOHS(boot->bpb40.bpb_rsvd_sec_cnt);

                            if (boot->bpb.bpb_num_fats == 1)
                            {
                                volume->fat2_blkno = 0;
                                volume->root_blkno = volume->fat1_blkno + volume->fat_blkcnt;

                            }
                            else
                            {
                                volume->fat2_blkno = volume->fat1_blkno + volume->fat_blkcnt;
                                volume->root_blkno = volume->fat2_blkno + volume->fat_blkcnt;
                            }

                            volume->root_clsno = DOSFS_CLSNO_NONE;
                            volume->root_blkcnt = ((DOSFS_FTOHS(boot->bpb40.bpb_root_ent_cnt) * 32) + DOSFS_BLK_MASK) >> DOSFS_BLK_SHIFT;

                            volume->cls_blk_offset = (volume->root_blkno + volume->root_blkcnt) - (2 << volume->cls_blk_shift);

                            if (boot->bpb40.bs_boot_sig == 0x29)
                            {
                                volume->serial = DOSFS_FTOHL(boot->bpb40.bs_vol_id);

#if (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1)
                                if (boot->bpb40.bs_nt_reserved & 0x01)
                                {
                                    volume->flags |= DOSFS_VOLUME_FLAG_MOUNTED_DIRTY;
                                }
#endif /* (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1) */
                            }
                        }
                        else
                        {
                            /* FAT32 */

                            volume->fsinfo_blkofs = DOSFS_FTOHS(boot->bpb71.bpb_fsinfo);
                            volume->bkboot_blkofs = DOSFS_FTOHS(boot->bpb71.bpb_bkboot);

                            tot_sec = DOSFS_FTOHL(boot->bpb71.bpb_tot_sec_32);

                            volume->fat_blkcnt = DOSFS_FTOHL(boot->bpb71.bpb_fat_sz_32);
                            volume->fat1_blkno = volume->boot_blkno + DOSFS_FTOHS(boot->bpb71.bpb_rsvd_sec_cnt);

                            /* It's tempting to use bpb71.bpb_ext_flags here to modify mirroring, 
                             * it seems to be problematic. For one most of the OSes do not support
                             * or respect this flag. The other minor detail is that Win95/Win98
                             * do use different settings than Win2k/WinXP/Win7.
                             */
                            if (boot->bpb.bpb_num_fats == 1)
                            {
                                volume->fat2_blkno = 0;

                                volume->cls_blk_offset = (volume->fat1_blkno + volume->fat_blkcnt) - (2 << volume->cls_blk_shift);
                            }
                            else
                            {
                                volume->fat2_blkno = volume->fat1_blkno + volume->fat_blkcnt;

                                volume->cls_blk_offset = (volume->fat2_blkno + volume->fat_blkcnt) - (2 << volume->cls_blk_shift);
                            }

                            volume->root_clsno = DOSFS_FTOHL(boot->bpb71.bpb_root_clus);
                            volume->root_blkno = volume->cls_blk_offset + (volume->root_clsno << volume->cls_blk_shift);
                            volume->root_blkcnt = volume->cls_blk_size;

                            if (boot->bpb71.bs_boot_sig == 0x29)
                            {
                                volume->serial = DOSFS_FTOHL(boot->bpb71.bs_vol_id);

#if (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1)
                                if (boot->bpb71.bs_nt_reserved & 0x01)
                                {
                                    volume->flags |= DOSFS_VOLUME_FLAG_MOUNTED_DIRTY;
                                }
#endif /* (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1) */
                            }
                        }

                        if (status == F_NO_ERROR)
                        {
                            volume->last_clsno = (((boot_blkno + tot_sec) - volume->cls_blk_offset) >> volume->cls_blk_shift) -1;

#if (DOSFS_CONFIG_SEQUENTIAL_SUPPORTED == 1) || (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1)
                            if (au_size == 0)
                            {
                                if (media == DOSFS_MEDIA_SFLASH)
                                {
                                    /* NOR based sflash has a virtual AU size of 64k ...
                                     */
                                    au_size = 128;
                                }
                                else
                                {
                                    /* Estimate the upper limit of the AU Size according to 4.13.1.8.1 from
                                     * the blkcnt to compute volume->au_size, used to align contiguous
                                     * cluster allocations.
                                     */
                                    
                                    if (blkcnt < 4209984)
                                    {
                                        if (blkcnt <= 131072)        /* <= 64MB  -> 512k */
                                        {
                                            au_size = 1024;
                                        }
                                        else if (blkcnt <= 524288)   /* <= 256MB -> 1MB  */
                                        {
                                            au_size = 2048;
                                        }
                                        else                         /* <= 1GB   -> 2MB  */
                                        {
                                            au_size = 4096;
                                        }
                                    }
                                    else
                                    {
                                        /* SDXC may have a larger AU size, but the spec calls out for speed class recording
                                         * to use 4MB as a basis.
                                         */
                                        au_size = 8192;
                                    }
                                }
                            }

                            volume->au_size = au_size;
                            
                            volume->start_clsno = ((((volume->cls_blk_offset + (2 << volume->cls_blk_shift) + au_size -1)
                                                     / au_size) * au_size)
                                                   - volume->cls_blk_offset) >> volume->cls_blk_shift;
                            volume->end_clsno = ((((((volume->last_clsno +1) << volume->cls_blk_shift) + volume->cls_blk_offset)
                                                   / au_size) * au_size)
                                                 - volume->cls_blk_offset) >> volume->cls_blk_shift;
#endif /* (DOSFS_CONFIG_SEQUENTIAL_SUPPORTED == 1) || (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1) */

                            if (status == F_NO_ERROR)
                            {
                                if ((volume->last_clsno -1) < 4085)
                                {
#if (DOSFS_CONFIG_FAT12_SUPPORTED == 1)
                                    volume->type = DOSFS_VOLUME_TYPE_FAT12;
#else /* (DOSFS_CONFIG_FAT12_SUPPORTED == 1) */
                                    status = F_ERR_INVALIDMEDIA;
#endif /* (DOSFS_CONFIG_FAT12_SUPPORTED == 1) */
                                }
                                else
                                {
                                    if ((volume->last_clsno -1) < 65525)
                                    {
                                        volume->type = DOSFS_VOLUME_TYPE_FAT16;
                                    }
                                    else
                                    {
                                        volume->type = DOSFS_VOLUME_TYPE_FAT32;
                                    }
                                }

                                if (status == F_NO_ERROR)
                                {
                                    /* Assign the caches.
                                     */
                                    volume->dir_cache.blkno = DOSFS_BLKNO_INVALID;

#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)
                                    volume->map_cache.blkno = DOSFS_BLKNO_INVALID;
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */

#if (DOSFS_CONFIG_FAT_CACHE_ENTRIES != 0)
#if (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 1)
                                    volume->fat_cache.blkno = DOSFS_BLKNO_INVALID;
#else /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 1) */
                                    volume->fat_cache[0].blkno = DOSFS_BLKNO_INVALID;
                                    volume->fat_cache[1].blkno = DOSFS_BLKNO_INVALID;
#endif /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 1) */
#endif /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES != 0) */
                            
#if (DOSFS_CONFIG_FILE_DATA_CACHE == 0)
                                    volume->data_file = NULL;
#if (DOSFS_CONFIG_DATA_CACHE_ENTRIES != 0)
                                    volume->data_cache.blkno = DOSFS_BLKNO_INVALID;
#endif /* (DOSFS_CONFIG_DATA_CACHE_ENTRIES != 0) */
#endif /* (DOSFS_CONFIG_FILE_DATA_CACHE == 0) */
                            
#if (DOSFS_CONFIG_CLUSTER_CACHE_ENTRIES != 0)
                                    for (index = 0; index < DOSFS_CONFIG_CLUSTER_CACHE_ENTRIES; index++)
                                    {
                                        volume->cluster_cache[index].clsno   = DOSFS_CLSNO_NONE;
                                        volume->cluster_cache[index].clsdata = DOSFS_CLSNO_NONE;
                                    }
#endif /* (DOSFS_CONFIG_CLUSTER_CACHE_ENTRIES != 0) */
                            
                                    volume->cwd_clsno = DOSFS_CLSNO_NONE;

                                    volume->files = NULL;

#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)
                                    if (!write_protected)
                                    {
                                        /* Check whether the log needs to be replayed, or whether
                                         * the BPB has to be set up in memory.
                                         */
                                        if ((boot->bpblog.log_lead_sig == DOSFS_HTOFL(DOSFS_LOG_LEAD_SIG)) &&
                                            (boot->bpblog.log_struct_sig == DOSFS_HTOFL(DOSFS_LOG_STRUCT_SIG)))
                                        {
                                            status = dosfs_volume_commit(volume);
                                        }
                                        else
                                        {
                                            if ((boot->bpb.bpb_num_fats == 2) && (volume->fat_blkcnt <= 8192))
                                            {
                                                if (volume->type != DOSFS_VOLUME_TYPE_FAT32)
                                                {
                                                    if (DOSFS_FTOHS(boot->bpb40.bpb_rsvd_sec_cnt) != 1)
                                                    {
                                                        boot->bpblog.log_lfn_blkno = volume->boot_blkno +1;
                                                    }
                                                    else
                                                    {
                                                        if (boot_blkcnt == 0)
                                                        {
                                                            status = F_ERR_UNUSABLE;
                                                        }
                                                        else
                                                        {
                                                            boot->bpblog.log_lfn_blkno = volume->boot_blkno -1;
                                                        }
                                                    }
                                            
                                                    boot->bpblog.log_map_blkno = DOSFS_BLKNO_RESERVED;
                                                }
                                                else
                                                {
                                                    if ((volume->fsinfo_blkofs != 1) || (volume->bkboot_blkofs != 6))
                                                    {
                                                        status = F_ERR_UNUSABLE;
                                                    }
                                                    else
                                                    {
                                                        boot->bpblog.log_lfn_blkno = volume->boot_blkno +3;
                                                        boot->bpblog.log_map_blkno = volume->boot_blkno +4;
                                                    }
                                                }
                                        
                                                if (status == F_NO_ERROR)
                                                {
                                                    boot->bpblog.log_map_flags = 0;
                                                    boot->bpblog.log_map_entries = 0;
                                                    boot->bpblog.log_dir_flags = 0;
                                                    boot->bpblog.log_lead_sig = DOSFS_HTOFL(0x00000000);
                                                    boot->bpblog.log_struct_sig = DOSFS_HTOFL(0x00000000);
                                                }
                                            }
                                            else
                                            {
                                                status = F_ERR_UNUSABLE;
                                            }
                                        }
                                    }
                                    else
                                    {
                                        status = F_ERR_WRITEPROTECT;
                                    }
                                
#if (DOSFS_CONFIG_FSINFO_SUPPORTED == 1)
                                    /* For TRANSACTION_SAFE the FSINFO might have been already read
                                     * throu dosfs_volume_commit(). In this case DOSFS_VOLUME_FLAG_FSINFO_VALID
                                     * would be already set ...
                                     */
                                    if (!(volume->flags & DOSFS_VOLUME_FLAG_FSINFO_VALID))
#endif /* (DOSFS_CONFIG_FSINFO_SUPPORTED == 1) */
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */
                                    {
                                        volume->next_clsno = 2;
                                
#if (DOSFS_CONFIG_FSINFO_SUPPORTED == 1)
                                        volume->free_clscnt = 0;

                                        if (status == F_NO_ERROR)
                                        {
                                            if (volume->fsinfo_blkofs != 0)
                                            {
                                                dosfs_fsinfo_t *fsinfo = (dosfs_fsinfo_t*)((void*)volume->dir_cache.data);

                                                status = dosfs_volume_read(volume, volume->boot_blkno + volume->fsinfo_blkofs, (uint8_t*)fsinfo);
                                    
                                                if (status == F_NO_ERROR)
                                                {
                                                    if ((fsinfo->fsi_lead_sig  == DOSFS_HTOFL(0x41615252)) &&
                                                        (fsinfo->fsi_struc_sig == DOSFS_HTOFL(0x61417272)) &&
                                                        (fsinfo->fsi_trail_sig == DOSFS_HTOFL(0xaa550000)))
                                                    {
#if (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1)
                                                        if (!(volume->flags & DOSFS_VOLUME_FLAG_MOUNTED_DIRTY))
#endif /* (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1) */
                                                        {
                                                            if (fsinfo->fsi_free_count != 0xffffffff)
                                                            {
                                                                volume->free_clscnt = DOSFS_FTOHL(fsinfo->fsi_free_count);
                                                        
                                                                if (volume->free_clscnt <= (volume->last_clsno -1))
                                                                {
                                                                    volume->flags |= DOSFS_VOLUME_FLAG_FSINFO_VALID;
                                                                }
                                                            }
                                                        }

                                                        if (fsinfo->fsi_nxt_free != 0xffffffff)
                                                        {
                                                            next_clsno = DOSFS_FTOHL(fsinfo->fsi_nxt_free);

                                                            if ((next_clsno >= 2) && (next_clsno <= volume->last_clsno))
                                                            {
                                                                volume->next_clsno = next_clsno;
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
#endif /* (DOSFS_CONFIG_FSINFO_SUPPORTED == 1) */
                                    }

                                    if (status == F_NO_ERROR)
                                    {
#if (DOSFS_CONFIG_SEQUENTIAL_SUPPORTED == 1) 
                                        volume->free_clsno = volume->start_clsno;
                                        volume->base_clsno = volume->start_clsno;
                                        volume->limit_clsno = volume->start_clsno;
#endif /* (DOSFS_CONFIG_SEQUENTIAL_SUPPORTED == 1) */
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        
        if (status == F_NO_ERROR)
        {
            volume->state = DOSFS_VOLUME_STATE_MOUNTED;
        }
    }

    return status;
}


static int dosfs_volume_unmount(dosfs_volume_t * volume)
{
    int status = F_NO_ERROR;

    if (volume->state >= DOSFS_VOLUME_STATE_MOUNTED)
    {
        while ((status == F_NO_ERROR) && volume->files)
        {
            status = dosfs_file_close(volume, volume->files);
        }

        if (status == F_NO_ERROR)
        {
            /* Revert the state to be DOSFS_VOLUME_STATE_INITIALIZED, so that can be remounted.
             */
            volume->state = DOSFS_VOLUME_STATE_INITIALIZED;
        }
    }

    return status;
}

static int dosfs_volume_lock(dosfs_volume_t *volume)
{
    int status = F_NO_ERROR;

    if (volume->state == DOSFS_VOLUME_STATE_NONE)
    {
        status = F_ERR_INITFUNC;
    }
    else
    {
        if (volume->state == DOSFS_VOLUME_STATE_UNUSABLE)
        {
            status = F_ERR_UNUSABLE;
        }
        else
        {
            /*
             * Check here whether the volume is mounted. If not mount it. If it cannot
             * be mounted, throw an error.
             */
            
            if (volume->state != DOSFS_VOLUME_STATE_MOUNTED)
            {
                if (volume->state == DOSFS_VOLUME_STATE_NONE)
                {
                    status = F_ERR_INITFUNC;
                }
                else
                {
                    status = dosfs_volume_mount(volume);
                }
            }
        }
    }

    return status;
}

static int dosfs_volume_lock_noinit(dosfs_volume_t *volume)
{
    int status = F_NO_ERROR;

    return status;
}

static int  dosfs_volume_lock_nomount(dosfs_volume_t *volume)
{
    int status = F_NO_ERROR;

    if (volume->state == DOSFS_VOLUME_STATE_NONE)
    {
        status = F_ERR_INITFUNC;
    }

    return status;
}

static int dosfs_volume_unlock(dosfs_volume_t *volume, int status)
{
    if (status != F_NO_ERROR)
    {
#if (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1)
        if (status == F_ERR_INVALIDSECTOR)
        {
            volume->flags |= DOSFS_VOLUME_FLAG_MEDIA_FAILURE;
        }
        else
#endif /* (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1) */
        {
            if (status == F_ERR_CARDREMOVED)
            {
                volume->state = DOSFS_VOLUME_STATE_CARDREMOVED;
            }

            if (status == F_ERR_UNUSABLE)
            {
                volume->state = DOSFS_VOLUME_STATE_UNUSABLE;
            }
        }
    }

    return status;
}

static int dosfs_volume_read(dosfs_volume_t *volume, uint32_t address, uint8_t *data)
{
    int status = F_NO_ERROR;
    dosfs_device_t *device;
#if (DOSFS_CONFIG_META_DATA_RETRIES != 0)
    unsigned int retries = DOSFS_CONFIG_META_DATA_RETRIES +1;
#else /* (DOSFS_CONFIG_META_DATA_RETRIES != 0) */
    unsigned int retries = 0;
#endif /* (DOSFS_CONFIG_META_DATA_RETRIES != 0) */

    device = DOSFS_VOLUME_DEVICE(volume);

    do
    {
        status = (*device->interface->read)(device->context, address, data, 1, false);
                
        if ((status == F_ERR_ONDRIVE) && (retries >= 1))
        {
            status = F_NO_ERROR;
            retries--;
        }
        else
        {
            retries = 0;
        }
    }
    while ((status == F_NO_ERROR) && retries);

#if (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1)
    if (status == F_ERR_INVALIDSECTOR)
    {
        volume->flags |= DOSFS_VOLUME_FLAG_MEDIA_FAILURE;
    }
#endif /* (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1) */

    if (status != F_NO_ERROR)
    {
        if (status != F_ERR_CARDREMOVED)
        {
            status = F_ERR_UNUSABLE;
        }
    }

    return status;
}

static int dosfs_volume_write(dosfs_volume_t *volume, uint32_t address, const uint8_t *data)
{
    int status = F_NO_ERROR;
    dosfs_device_t *device;
#if (DOSFS_CONFIG_META_DATA_RETRIES != 0)
    unsigned int retries = DOSFS_CONFIG_META_DATA_RETRIES +1;
#else /* (DOSFS_CONFIG_META_DATA_RETRIES != 0) */
    unsigned int retries = 0;
#endif /* (DOSFS_CONFIG_META_DATA_RETRIES != 0) */

    device = DOSFS_VOLUME_DEVICE(volume);

    do
    {
        status = (*device->interface->write)(device->context, address, data, 1, true);
                
        if ((status == F_ERR_ONDRIVE) && (retries >= 1))
        {
            status = F_NO_ERROR;
            retries--;
        }
        else
        {
            retries = 0;
        }
    }
    while ((status == F_NO_ERROR) && retries);

#if (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1)
    if (status == F_ERR_INVALIDSECTOR)
    {
        volume->flags |= DOSFS_VOLUME_FLAG_MEDIA_FAILURE;
    }
#endif /* (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1) */

    if (status != F_NO_ERROR)
    {
        if (status != F_ERR_CARDREMOVED)
        {
            status = F_ERR_UNUSABLE;
        }
    }

    if (status == F_NO_ERROR)
    {
        device->lock |= DOSFS_DEVICE_LOCK_MODIFIED;
    }

    return status;
}

static int  dosfs_volume_zero(dosfs_volume_t *volume, uint32_t address, uint32_t length, bool wait)
{
    int status = F_NO_ERROR;
    dosfs_device_t *device;
    uint8_t *data;

    device = DOSFS_VOLUME_DEVICE(volume);

    status = dosfs_dir_cache_flush(volume);

    if (status == F_NO_ERROR)
    {
        data = volume->dir_cache.data;
        volume->dir_cache.blkno = DOSFS_BLKNO_INVALID;
        
        memset(data, 0, DOSFS_BLK_SIZE);

        do
        {
            status = (*device->interface->write)(device->context, address, data, 1, ((length == 1) ? wait : false));

            if (status == F_NO_ERROR)
            {
                device->lock |= DOSFS_DEVICE_LOCK_MODIFIED;
            }

            address++;
            length--;
        }
        while ((status == F_NO_ERROR) && length);
    }

#if (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1)
    if (status == F_ERR_INVALIDSECTOR)
    {
        volume->flags |= DOSFS_VOLUME_FLAG_MEDIA_FAILURE;
    }
#endif /* (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1) */

    if (wait)
    {
        if (status != F_NO_ERROR)
        {
            if (status != F_ERR_CARDREMOVED)
            {
                status = F_ERR_UNUSABLE;
            }
        }
    }

    return status;
}

#if (DOSFS_CONFIG_FSINFO_SUPPORTED == 1)

static int dosfs_volume_fsinfo(dosfs_volume_t *volume, uint32_t free_clscnt, uint32_t next_clsno)
{
    int status = F_NO_ERROR;
    uint32_t blkno;
    dosfs_fsinfo_t *fsinfo;
    
    status = dosfs_dir_cache_flush(volume);
            
    if (status == F_NO_ERROR)
    {
        blkno = (volume->boot_blkno + volume->fsinfo_blkofs);
        fsinfo = (dosfs_fsinfo_t*)((void*)volume->dir_cache.data);
        
        if (volume->dir_cache.blkno != blkno)
        {
            memset((uint8_t*)fsinfo, 0, DOSFS_BLK_SIZE);
            
            fsinfo->fsi_lead_sig   = DOSFS_HTOFL(0x41615252);
            fsinfo->fsi_struc_sig  = DOSFS_HTOFL(0x61417272);
            fsinfo->fsi_trail_sig  = DOSFS_HTOFL(0xaa550000);
            
            volume->dir_cache.blkno = blkno;
        }               
        
        fsinfo->fsi_free_count = DOSFS_HTOFL(free_clscnt);
        fsinfo->fsi_nxt_free   = DOSFS_HTOFL(next_clsno);
        
        status = dosfs_volume_write(volume, blkno, (uint8_t*)fsinfo);
    }

    return status;
}

#endif /* (DOSFS_CONFIG_FSINFO_SUPPORTED == 1) */

#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0)

static int dosfs_volume_dirty(dosfs_volume_t *volume)
{
    int status = F_NO_ERROR;
#if (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1)
    dosfs_boot_t *boot;
#endif /* (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1) */

    if (volume->state == DOSFS_VOLUME_STATE_MOUNTED)
    {
#if (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1)
        if (!(volume->flags & DOSFS_VOLUME_FLAG_MOUNTED_DIRTY) &&
            !(volume->flags & DOSFS_VOLUME_FLAG_VOLUME_DIRTY))
        {
            status = dosfs_dir_cache_flush(volume);

            if (status == F_NO_ERROR)
            {
                boot = (dosfs_boot_t*)((void*)volume->dir_cache.data);

                if (volume->dir_cache.blkno != volume->boot_blkno)
                {
                    memcpy((uint8_t*)boot, volume->bs_data, sizeof(volume->bs_data));
                    memset((uint8_t*)boot + sizeof(volume->bs_data), 0, 510 - sizeof(volume->bs_data));

                    boot->bpb.bs_trail_sig = DOSFS_HTOFS(0xaa55);

                    volume->dir_cache.blkno = volume->boot_blkno;
                }

                if (status == F_NO_ERROR)
                {
                    if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
                    {
                        boot->bpb71.bs_nt_reserved |= 0x01;
                    }
                    else
                    {
                        boot->bpb40.bs_nt_reserved |= 0x01;
                    }

#if (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1)
                    if (volume->flags & DOSFS_VOLUME_FLAG_MEDIA_FAILURE)
                    {
                        if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
                        {
                            boot->bpb71.bs_nt_reserved |= 0x02;
                        }
                        else
                        {
                            boot->bpb40.bs_nt_reserved |= 0x02;
                        }
                    }
#endif /* (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1) */
                    
                    status = dosfs_volume_write(volume, volume->boot_blkno, (uint8_t*)boot);

                    if (status == F_NO_ERROR)
                    {
                        volume->flags |= DOSFS_VOLUME_FLAG_VOLUME_DIRTY;
                    }
                }
            }
        }
#endif /* (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1) */
    }

    return status;
}

static int dosfs_volume_clean(dosfs_volume_t *volume, int status_o)
{
    int status = F_NO_ERROR;
#if (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1)
    dosfs_boot_t *boot;
#endif /* (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1) */

    if (volume->state == DOSFS_VOLUME_STATE_MOUNTED)
    {
#if (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1)
#if (DOSFS_CONFIG_FSINFO_SUPPORTED == 1)
        if ((volume->fsinfo_blkofs != 0) &&
            (volume->flags & DOSFS_VOLUME_FLAG_FSINFO_DIRTY))
        {
            if (!(volume->flags & DOSFS_VOLUME_FLAG_MOUNTED_DIRTY) &&
                (volume->flags & DOSFS_VOLUME_FLAG_FSINFO_VALID))
            {
                status = dosfs_volume_fsinfo(volume, volume->free_clscnt, volume->next_clsno);
            }
            else
            {
                status = dosfs_volume_fsinfo(volume, 0xffffffff, volume->next_clsno);
            }
            
            if (status == F_NO_ERROR)
            {
                volume->flags &= ~DOSFS_VOLUME_FLAG_FSINFO_DIRTY;
            }
        }
#endif /* (DOSFS_CONFIG_FSINFO_SUPPORTED == 1) */

        if (!(volume->flags & DOSFS_VOLUME_FLAG_MOUNTED_DIRTY) &&
            (volume->flags & DOSFS_VOLUME_FLAG_VOLUME_DIRTY))
        {
            status = dosfs_dir_cache_flush(volume);

            if (status == F_NO_ERROR)
            {
                boot = (dosfs_boot_t*)((void*)volume->dir_cache.data);

                if (volume->dir_cache.blkno != volume->boot_blkno)
                {
                    memcpy((uint8_t*)boot, volume->bs_data, 90);
                    memset((uint8_t*)boot + 90, 0, 510 - 90);

                    boot->bpb.bs_trail_sig = DOSFS_HTOFS(0xaa55);

                    volume->dir_cache.blkno = volume->boot_blkno;
                }

                if (status == F_NO_ERROR)
                {
                    if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
                    {
                        boot->bpb71.bs_nt_reserved &= ~0x01;
                    }
                    else
                    {
                        boot->bpb40.bs_nt_reserved &= ~0x01;
                    }

#if (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1)
                    if (volume->flags & DOSFS_VOLUME_FLAG_MEDIA_FAILURE)
                    {
                        if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
                        {
                            boot->bpb71.bs_nt_reserved |= 0x02;
                        }
                        else
                        {
                            boot->bpb40.bs_nt_reserved |= 0x02;
                        }
                    }
#endif /* (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1) */
                    
                    status = dosfs_volume_write(volume, volume->boot_blkno, (uint8_t*)boot);

                    if (status == F_NO_ERROR)
                    {
                        volume->flags &= ~DOSFS_VOLUME_FLAG_VOLUME_DIRTY;
                    }
                }
            }
        }

#else /* (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1) */
#if (DOSFS_CONFIG_FSINFO_SUPPORTED == 1)
        if ((volume->fsinfo_blkofs != 0) &&
            (volume->flags & DOSFS_VOLUME_FLAG_FSINFO_DIRTY))
        {
            status = dosfs_volume_fsinfo(volume, 0xffffffff, volume->next_clsno);
            
            if (status == F_NO_ERROR)
            {
                volume->flags &= ~DOSFS_VOLUME_FLAG_FSINFO_DIRTY;
            }
        }
#endif /* (DOSFS_CONFIG_FSINFO_SUPPORTED == 1) */
#endif /* (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1) */
    }

    if (status_o != F_NO_ERROR)
    {
        status = status_o;
    }

    return status;
}

#else /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */

static int dosfs_volume_record(dosfs_volume_t *volume)
{
    int status = F_NO_ERROR;
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1) && (DOSFS_CONFIG_UTF8_SUPPORTED == 1)
    uint16_t bs_trail_sig;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) && (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */
    dosfs_boot_t *boot;

    status = dosfs_map_cache_flush(volume);

    if (status == F_NO_ERROR)
    {
        if (volume->map_flags
            || ((volume->dir_flags & (DOSFS_DIR_FLAG_DESTROY_ENTRY | DOSFS_DIR_FLAG_CREATE_ENTRY)) == (DOSFS_DIR_FLAG_DESTROY_ENTRY | DOSFS_DIR_FLAG_CREATE_ENTRY))
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
            || ((volume->dir_flags & DOSFS_DIR_FLAG_DESTROY_ENTRY) && volume->del_entries)
            || ((volume->dir_flags & DOSFS_DIR_FLAG_CREATE_ENTRY) && volume->dir_entries)
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
            )
        {
            boot = (dosfs_boot_t*)((void*)&volume->bs_data[0]);

            /* Make sure the file system is marked unusable other
             * than for TRANSACTION_SAFE recovery.
             */
            boot->bpb.bpb_byts_per_sec |= 0x8000;

            /* Fill in the heads to signal an uncomitted dosfs_volume_record().
             */
            boot->bpblog.log_lead_sig = DOSFS_HTOFL(DOSFS_LOG_LEAD_SIG);
            boot->bpblog.log_struct_sig = DOSFS_HTOFL(DOSFS_LOG_STRUCT_SIG);

#if (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1)
            if (volume->flags & DOSFS_VOLUME_FLAG_MEDIA_FAILURE)
            {
                if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
                {
                    boot->bpb71.bs_nt_reserved |= 0x02;
                }
                else
                {
                    boot->bpb40.bs_nt_reserved |= 0x02;
                }
            }
#endif /* (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1) */
            
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1) && (DOSFS_CONFIG_UTF8_SUPPORTED == 1)
            /* If VFAT and UTF8 are enabled, VFAT uses uint16_t per lfn_name character. That will
             * overflow the boot sector with the maximum length of 255. Hence detect the overflow
             * case, and write the lfn_name to a separate block, patch in the bs_trail_sig, 
             * write the boot block, and then undo the patching of the bs_trail_sig.
             */
            if ((volume->dir_flags & DOSFS_DIR_FLAG_CREATE_ENTRY) &&
                (volume->dir_entries != 0) &&
                (volume->lfn_count > 128))
            {
                status = dosfs_volume_write(volume, volume->lfn_blkno, (uint8_t*)volume->lfn_name);
            }
            
            if (status == F_NO_ERROR)
            {
                bs_trail_sig = boot->bpb.bs_trail_sig;
                
                boot->bpb.bs_trail_sig = DOSFS_HTOFS(0xaa55);
                
                status = dosfs_volume_write(volume, volume->boot_blkno, (uint8_t*)boot);
                
                boot->bpb.bs_trail_sig = bs_trail_sig;
            }
#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) && (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */
            status = dosfs_volume_write(volume, volume->boot_blkno, (uint8_t*)boot);
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) && (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */
        }

        if (status == F_NO_ERROR)
        {
            status = dosfs_volume_commit(volume);
        }
    }

    return status;
}

static int dosfs_volume_commit(dosfs_volume_t *volume)
{
    int status = F_NO_ERROR;
    uint32_t clsno, blkno, index, clsno_s;
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
    unsigned int sequence, chksum, s, i, cc;
    uint32_t blkno_e, offset;
#if (DOSFS_CONFIG_UTF8_SUPPORTED == 1)
    uint16_t bs_trail_sig;
#endif /* (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */
    dosfs_dir_t *dir_e;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
    dosfs_dir_t *dir;
    dosfs_cache_entry_t *entry;
    dosfs_boot_t *boot;

    boot = (dosfs_boot_t*)((void*)&volume->bs_data[0]);
        
    if (boot->bpblog.log_lead_sig == DOSFS_HTOFL(DOSFS_LOG_LEAD_SIG))
    {
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1) && (DOSFS_CONFIG_UTF8_SUPPORTED == 1)
        /* At this point, the boot block had been loaded. For VFAT and UTF8
         * the remaining lfn_name[] characters have to be loaded as there is
         * not enough space for 255 unicode characters.
         */
        if ((volume->dir_flags & DOSFS_DIR_FLAG_CREATE_ENTRY) &&
            (volume->dir_entries != 0) &&
            (volume->lfn_count > 128))
        {
            status = dosfs_volume_read(volume, volume->lfn_blkno, (uint8_t*)volume->lfn_name);
        }
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) && (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */
    }

    if (status == F_NO_ERROR)
    {
        if (volume->map_flags & DOSFS_MAP_FLAG_MAP_CHANGED)
        {
            status = dosfs_map_cache_resolve(volume);
        }
    }

#if (DOSFS_CONFIG_FSINFO_SUPPORTED == 1)
    /* Update the FSINFO before dealing with DOSFS_DIR_FLAG_CREATE_ENTRY, so that
     * volume->dir_cache is valid and not nuked by dosfs_volume_fsinfo.
     * N.b. DOSFS_VOLUME_FLAG_FSINFO_VALID & DOSFS_VOLUME_FLAG_FSINFO_DIRTY are 
     * set if DOSFS_MAP_FLAG_FSINFO was set (which only gets set if there was
     * a map/fat update pending).
     */
    if (status == F_NO_ERROR)
    {
        if ((volume->fsinfo_blkofs != 0) &&
            (volume->flags & DOSFS_VOLUME_FLAG_FSINFO_VALID) &&
            (volume->flags & DOSFS_VOLUME_FLAG_FSINFO_DIRTY))
        {
            status = dosfs_volume_fsinfo(volume, volume->free_clscnt, volume->next_clsno);

            if (status == F_NO_ERROR)
            {
                volume->flags &= ~DOSFS_VOLUME_FLAG_FSINFO_DIRTY;
            }
        }
    }
#endif /* (DOSFS_CONFIG_FSINFO_SUPPORTED == 1) */

    if (status == F_NO_ERROR)
    {
        if (volume->dir_flags & (DOSFS_DIR_FLAG_SYNC_ENTRY | DOSFS_DIR_FLAG_ACCESS_ENTRY | DOSFS_DIR_FLAG_MODIFY_ENTRY))
        {
            clsno = volume->dir_clsno;
            index = volume->dir_index;

            if (clsno == DOSFS_CLSNO_NONE)
            {
                blkno = volume->root_blkno + DOSFS_INDEX_TO_BLKCNT_ROOT(index);
            }
            else
            {
                blkno = DOSFS_CLSNO_TO_BLKNO(clsno) + DOSFS_INDEX_TO_BLKCNT(index);
            }
    
            status = dosfs_dir_cache_read(volume, blkno, &entry);

            if (status == F_NO_ERROR)
            {
                dir = (dosfs_dir_t*)((void*)(entry->data + DOSFS_INDEX_TO_BLKOFS(index)));

                dir->dir_clsno_lo = volume->dir.dir_clsno_lo;
                dir->dir_file_size = volume->dir.dir_file_size;
                
                if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
                {
                    dir->dir_clsno_hi = volume->dir.dir_clsno_hi;
                }
                
                if (volume->dir_flags & DOSFS_DIR_FLAG_ACCESS_ENTRY)
                {
                    dir->dir_acc_date = volume->dir.dir_acc_date;
                }
                
                if (volume->dir_flags & DOSFS_DIR_FLAG_MODIFY_ENTRY)
                {
                    dir->dir_attr |= DOSFS_DIR_ATTR_ARCHIVE;
                    dir->dir_wrt_time = volume->dir.dir_wrt_time;
                    dir->dir_wrt_date = volume->dir.dir_wrt_date;
                }

                status = dosfs_dir_cache_write(volume);

                if (status == F_NO_ERROR)
                {
                    volume->dir_flags &= ~(DOSFS_DIR_FLAG_SYNC_ENTRY | DOSFS_DIR_FLAG_ACCESS_ENTRY | DOSFS_DIR_FLAG_MODIFY_ENTRY);
                }
            }
        }
    }

    if (status == F_NO_ERROR)
    {
        if (volume->dir_flags & DOSFS_DIR_FLAG_DESTROY_ENTRY)
        {
            clsno = volume->del_clsno;
            index = volume->del_index;

            if (clsno == DOSFS_CLSNO_NONE)
            {
                clsno = volume->root_clsno;
                blkno = volume->root_blkno + DOSFS_INDEX_TO_BLKCNT_ROOT(index);
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                blkno_e = volume->root_blkno + volume->root_blkcnt;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
            }
            else
            {
                blkno = DOSFS_CLSNO_TO_BLKNO(clsno) + DOSFS_INDEX_TO_BLKCNT(index);
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                blkno_e = DOSFS_CLSNO_TO_BLKNO(clsno) + volume->cls_blk_size;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
            }
    
            status = dosfs_dir_cache_read(volume, blkno, &entry);

            if (status == F_NO_ERROR)
            {
                dir = (dosfs_dir_t*)((void*)(entry->data + DOSFS_INDEX_TO_BLKOFS(index)));

#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                if (volume->del_entries != 0)
                {
                    sequence = volume->del_entries;
                    
                    dir_e = (dosfs_dir_t*)((void*)(entry->data + DOSFS_BLK_SIZE));
                    
                    do
                    {
                        dir->dir_name[0] = 0xe5;
                        
                        dir++;
                        
                        if (dir == dir_e)
                        {
                            status = dosfs_dir_cache_write(volume);
                            
                            blkno++;
                            
                            if (blkno == blkno_e)
                            {
                                status = dosfs_cluster_read(volume, clsno, &clsno);
                                
                                if (status == F_NO_ERROR)
                                {
                                    blkno = DOSFS_CLSNO_TO_BLKNO(clsno);
                                    blkno_e = blkno + volume->cls_blk_size;
                                }
                            }
                            
                            if (status == F_NO_ERROR)
                            {
                                status = dosfs_dir_cache_read(volume, blkno, &entry);

                                if (status == F_NO_ERROR)
                                {
                                    dir = (dosfs_dir_t*)((void*)(entry->data));
                                }
                            }
                        }
                        
                        sequence--;
                    }
                    while ((status == F_NO_ERROR) && (sequence != 0));
                }

                if (status == F_NO_ERROR)
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                {
                    dir->dir_name[0] = 0xe5;
                    
                    status = dosfs_dir_cache_write(volume);

                    if (status == F_NO_ERROR)
                    {
                        volume->dir_flags &= ~DOSFS_DIR_FLAG_DESTROY_ENTRY;
                    }
                }
            }
        }
    }

    if (status == F_NO_ERROR)
    {
        if (volume->dir_flags & DOSFS_DIR_FLAG_CREATE_ENTRY)
        {
            clsno = volume->dir_clsno;
            index = volume->dir_index;

            if (clsno == DOSFS_CLSNO_NONE)
            {
                clsno = volume->root_clsno;
                blkno = volume->root_blkno + DOSFS_INDEX_TO_BLKCNT_ROOT(index);
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                blkno_e = volume->root_blkno + volume->root_blkcnt;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
            }
            else
            {
                blkno = DOSFS_CLSNO_TO_BLKNO(clsno) + DOSFS_INDEX_TO_BLKCNT(index);
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                blkno_e = DOSFS_CLSNO_TO_BLKNO(clsno) + volume->cls_blk_size;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
            }
    
            status = dosfs_dir_cache_read(volume, blkno, &entry);

            if (status == F_NO_ERROR)
            {
                dir = (dosfs_dir_t*)((void*)(entry->data + DOSFS_INDEX_TO_BLKOFS(index)));
                
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                if (volume->dir_entries)
                {
                    sequence = 0x40 | volume->dir_entries;
                    chksum = dosfs_name_checksum_dosname(volume->dir.dir_name);

                    dir_e = (dosfs_dir_t*)((void*)(entry->data + DOSFS_BLK_SIZE));

                    do
                    {
                        s = 0;
                        i = ((sequence & 0x1f) - 1) * 13;
                
                        dir->dir_name[0] = sequence;
                        dir->dir_attr = DOSFS_DIR_ATTR_LONG_NAME;
                        dir->dir_nt_reserved = 0x00;
                        dir->dir_crt_time_tenth = chksum;
                        dir->dir_clsno_lo = 0x0000;
                
                        do
                        {
                            offset = dosfs_path_ldir_name_table[s++];
                    
                            if (i < volume->lfn_count)
                            {
                                cc = volume->lfn_name[i];
                        
                                ((uint8_t*)dir)[offset +0] = cc;
                                ((uint8_t*)dir)[offset +1] = cc >> 8;
                            }
                            else
                            {
                                if (i == volume->lfn_count)
                                {
                                    ((uint8_t*)dir)[offset +0] = 0x00;
                                    ((uint8_t*)dir)[offset +1] = 0x00;
                                }
                                else
                                {
                                    ((uint8_t*)dir)[offset +0] = 0xff;
                                    ((uint8_t*)dir)[offset +1] = 0xff;
                                }
                            }

                            i++;
                        }
                        while (s < 13);
                
                        dir++;

                        if (dir == dir_e)
                        {
                            status = dosfs_dir_cache_write(volume);

                            blkno++;

                            if (blkno == blkno_e)
                            {
                                status = dosfs_cluster_read(volume, clsno, &clsno);
                
                                if (status == F_NO_ERROR)
                                {
                                    blkno = DOSFS_CLSNO_TO_BLKNO(clsno);
                                    blkno_e = blkno + volume->cls_blk_size;
                                }
                            }

                            if (status == F_NO_ERROR)
                            {
                                status = dosfs_dir_cache_read(volume, blkno, &entry);

                                if (status == F_NO_ERROR)
                                {
                                    dir = (dosfs_dir_t*)((void*)(entry->data));
                                }
                            }
                        }

                        sequence = (sequence & 0x1f) -1;
                    }
                    while ((status == F_NO_ERROR) && (sequence != 0));
                }

                if (status == F_NO_ERROR)
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                {
                    memcpy(dir, &volume->dir, sizeof(dosfs_dir_t));

                    status = dosfs_dir_cache_write(volume);

                    if (status == F_NO_ERROR)
                    {
                        if (volume->dir.dir_attr & DOSFS_DIR_ATTR_DIRECTORY)
                        {
                            if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
                            {
                                clsno_s = ((uint32_t)DOSFS_FTOHS(volume->dir.dir_clsno_hi) << 16) | (uint32_t)DOSFS_FTOHS(volume->dir.dir_clsno_lo);
                            }
                            else
                            {
                                clsno_s = (uint32_t)DOSFS_FTOHS(volume->dir.dir_clsno_lo);
                            }
                            
                            blkno = DOSFS_CLSNO_TO_BLKNO(clsno_s);
                            
                            status = dosfs_dir_cache_read(volume, blkno, &entry);
                                                                    
                            if (status == F_NO_ERROR)
                            {
                                dir = (dosfs_dir_t*)((void*)(entry->data + DOSFS_INDEX_TO_BLKOFS(1)));
                                
                                dir->dir_clsno_lo = volume->dot_clsno;
                                
                                if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
                                {
                                    dir->dir_clsno_hi = volume->dot_clsno >> 16;
                                }
                                
                                status = dosfs_dir_cache_write(volume);
                            }
                        }
                    }

                    if (status == F_NO_ERROR)
                    {
                        volume->dir_flags &= ~DOSFS_DIR_FLAG_CREATE_ENTRY;
                    }
                }
            }
        }
    }

    if (status == F_NO_ERROR)
    {
        if (boot->bpblog.log_lead_sig == DOSFS_HTOFL(DOSFS_LOG_LEAD_SIG))
        {
            boot->bpb.bpb_byts_per_sec &= ~0x8000;

            boot->bpblog.log_lead_sig = DOSFS_HTOFL(0x00000000);
            boot->bpblog.log_struct_sig = DOSFS_HTOFL(0x00000000);

#if (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1)
            if (volume->flags & DOSFS_VOLUME_FLAG_MEDIA_FAILURE)
            {
                if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
                {
                    boot->bpb71.bs_nt_reserved |= 0x02;
                }
                else
                {
                    boot->bpb40.bs_nt_reserved |= 0x02;
                }
            }
#endif /* (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1) */

#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1) && (DOSFS_CONFIG_UTF8_SUPPORTED == 1)
            /* For VFAT and UTF8 the lfn_name[] needs to patched to contain
             * the bs_trail_sig before writing the boot block, and the unpatched
             * afterwards.
             */
            bs_trail_sig = boot->bpb.bs_trail_sig;

            boot->bpb.bs_trail_sig = DOSFS_HTOFS(0xaa55);
        
            status = dosfs_volume_write(volume, volume->boot_blkno, (uint8_t*)boot);
        
            boot->bpb.bs_trail_sig = bs_trail_sig;

#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) && (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */

            status = dosfs_volume_write(volume, volume->boot_blkno, (uint8_t*)boot);

#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) && (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */
        }
    }

    return status;
}

#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */

static int dosfs_volume_format(dosfs_volume_t *volume)
{
    int status = F_NO_ERROR;
    dosfs_device_t *device;
    uint32_t clscnt;
    uint32_t boot_blkno, fat1_blkno, fat2_blkno, root_blkno, fat_blkcnt, root_blkcnt, user_blkno, user_blkcnt;
    uint32_t bkboot_blkofs, fsinfo_blkofs;
    dosfs_boot_t *boot;
    dosfs_fsinfo_t *fsinfo;
    void *data;

    device = DOSFS_VOLUME_DEVICE(volume);

    boot = (dosfs_boot_t*)((void*)volume->dir_cache.data);
    volume->dir_cache.blkno = DOSFS_BLKNO_INVALID;

    boot_blkno = volume->boot_blkno;    
    bkboot_blkofs = volume->bkboot_blkofs;
    fsinfo_blkofs = volume->fsinfo_blkofs;
    
    fat_blkcnt = volume->fat_blkcnt;
    fat1_blkno = volume->fat1_blkno;
    fat2_blkno = volume->fat2_blkno;

    clscnt = volume->last_clsno -1;

    if (volume->type != DOSFS_VOLUME_TYPE_FAT32)
    {
        root_blkno = volume->root_blkno;
        root_blkcnt = volume->root_blkcnt;
    }
    else
    {
        root_blkno = ((fat2_blkno == 0) ? (fat1_blkno + fat_blkcnt) : (fat2_blkno + fat_blkcnt));
        root_blkcnt = volume->cls_blk_size;
        
        status = dosfs_volume_read(volume, boot_blkno, (uint8_t*)boot);
        
        if (status == F_NO_ERROR)
        {
            boot->bpb71.bpb_ext_flags = DOSFS_HTOFS(0x0000);
            boot->bpb71.bpb_root_clus = DOSFS_HTOFL(2);
            
            if (bkboot_blkofs)
            {
                status = dosfs_volume_write(volume, boot_blkno + bkboot_blkofs, (uint8_t*)boot);
            }
            
            if (status == F_NO_ERROR)
            {
                status = dosfs_volume_write(volume, boot_blkno, (uint8_t*)boot);
            }
        }
    }

    if (status == F_NO_ERROR)
    {
        /* For FAT32 above root_clsno got forced to be 2, so that fat1/fat2/root are contiguous.
         */
        status = dosfs_volume_zero(volume, fat1_blkno, ((root_blkno - fat1_blkno) + root_blkcnt), true);
        
        if (status == F_NO_ERROR)
        {
            /* Write first FAT1/FAT2 entry */
            
            data = (void*)volume->dir_cache.data;
            memset(data, 0, DOSFS_BLK_SIZE);

#if (DOSFS_CONFIG_FAT12_SUPPORTED == 1)
            if (volume->type == DOSFS_VOLUME_TYPE_FAT12)
            {
                ((uint8_t*)data)[0] = 0xf8;
                ((uint8_t*)data)[1] = 0xff;
                ((uint8_t*)data)[2] = 0xff;
            }
            else
#endif /* (DOSFS_CONFIG_FAT12_SUPPORTED == 1) */
            {
                if (volume->type == DOSFS_VOLUME_TYPE_FAT16)
                {
                    ((uint16_t*)data)[0] = DOSFS_HTOFS(0xfff8);
                    ((uint16_t*)data)[1] = DOSFS_HTOFS(0xffff);
                }
                else
                {
                    ((uint32_t*)data)[0] = DOSFS_HTOFS(0x0ffffff8);
                    ((uint32_t*)data)[1] = DOSFS_HTOFS(0x0fffffff);
                    ((uint32_t*)data)[2] = DOSFS_HTOFS(0x0fffffff);
                }
            }

            status = dosfs_volume_write(volume, fat1_blkno, data);
                
            if (status == F_NO_ERROR)
            {
                if (fat2_blkno)
                {
                    status = dosfs_volume_write(volume, fat2_blkno, data);
                }
            }

            if (fsinfo_blkofs)
            {
                if (status == F_NO_ERROR)
                {
                    fsinfo = (dosfs_fsinfo_t*)((void*)volume->dir_cache.data);
                    memset(fsinfo, 0, DOSFS_BLK_SIZE);

                    fsinfo->fsi_lead_sig   = DOSFS_HTOFL(0x41615252);
                    fsinfo->fsi_struc_sig  = DOSFS_HTOFL(0x61417272);
                    fsinfo->fsi_free_count = DOSFS_HTOFL(clscnt);
                    fsinfo->fsi_nxt_free   = DOSFS_HTOFL(3);
                    fsinfo->fsi_trail_sig  = DOSFS_HTOFL(0xaa550000);
                        
                    status = dosfs_volume_write(volume, boot_blkno + fsinfo_blkofs, (uint8_t*)fsinfo);
                        
                    if (status == F_NO_ERROR)
                    {
                        if (bkboot_blkofs)
                        {
                            status = dosfs_volume_write(volume, boot_blkno + bkboot_blkofs + fsinfo_blkofs, (uint8_t*)fsinfo);
                        }
                    }
                }
            }
        }
    }

    if (status == F_NO_ERROR)
    {
        if (volume->type != DOSFS_VOLUME_TYPE_FAT32)
        {
            user_blkno = (root_blkno + root_blkcnt);
            user_blkcnt = ((volume->last_clsno -1) << volume->cls_blk_shift);
        }
        else
        {
            user_blkno = (root_blkno + root_blkcnt);
            user_blkcnt = ((volume->last_clsno -2) << volume->cls_blk_shift);
        }

        status = (*device->interface->discard)(device->context, user_blkno, user_blkcnt);
    }

    return status;
}

/***********************************************************************************************************************/

static int dosfs_dir_cache_write(dosfs_volume_t *volume)
{
    int status = F_NO_ERROR;
    dosfs_device_t *device;

    device = DOSFS_VOLUME_DEVICE(volume);

#if (DOSFS_CONFIG_DATA_CACHE_ENTRIES == 0)
    if (volume->data_file)
    {
        dosfs_file_t *file = volume->data_file;
        
        status = (*device->interface->write)(device->context, volume->dir_cache.blkno, volume->dir_cache.data, 1, !!(file->mode & DOSFS_FILE_MODE_RANDOM));

#if (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1)
        if (status == F_ERR_INVALIDSECTOR)
        {
            volume->flags |= DOSFS_VOLUME_FLAG_MEDIA_FAILURE;
        }
#endif /* (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1) */

        if (status == F_NO_ERROR)
        {
            DOSFS_VOLUME_STATISTICS_COUNT(data_cache_write);

            device->lock |= DOSFS_DEVICE_LOCK_MODIFIED;
        }

        /* Unconditionally clean the dirty condition, or
         * otherwise the while system would get stuck.
         */
        volume->data_file = NULL;
    }
    else
#endif /* (DOSFS_CONFIG_DATA_CACHE_ENTRIES == 0) */
    {
#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) && (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 0)
        if (volume->flags & DOSFS_VOLUME_FLAG_FAT_DIRTY)
        {
            status = dosfs_map_cache_write(volume, volume->dir_cache.blkno, volume->dir_cache.data);

            if (status == F_NO_ERROR)
            {
                DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_write);

                volume->flags &= ~DOSFS_VOLUME_FLAG_FAT_DIRTY;
            }
        }
        else
        {
            status = dosfs_volume_write(volume, volume->dir_cache.blkno, volume->dir_cache.data);

            if (status == F_NO_ERROR)
            {
                DOSFS_VOLUME_STATISTICS_COUNT(dir_cache_write);
            }
        }
#else /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) && (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 0) */
        status = dosfs_volume_write(volume, volume->dir_cache.blkno, volume->dir_cache.data);
        
        if (status == F_NO_ERROR)
        {
#if (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 0)
            if (volume->flags & DOSFS_VOLUME_FLAG_FAT_DIRTY)
            {
                DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_write);
                
#if (DOSFS_CONFIG_2NDFAT_SUPPORTED == 1)
                if (volume->fat2_blkno)
                {
                    status = dosfs_volume_write(volume, volume->dir_cache.blkno + volume->fat_blkcnt, volume->dir_cache.data);
                }
                
                if (status == F_NO_ERROR)
#endif /* (DOSFS_CONFIG_2NDFAT_SUPPORTED == 1) */
                {
                    volume->flags &= ~DOSFS_VOLUME_FLAG_FAT_DIRTY;
                }
            }
            else
#endif /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 0) */
            {
                DOSFS_VOLUME_STATISTICS_COUNT(dir_cache_write);
            }
        }
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) && (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 0) */
    }

    return status;
}

static int dosfs_dir_cache_fill(dosfs_volume_t *volume, uint32_t blkno, int zero)
{
    int status = F_NO_ERROR;

#if (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 0) || (DOSFS_CONFIG_DATA_CACHE_ENTRIES == 0)
    if (0
#if (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 0)
        || (volume->flags & DOSFS_VOLUME_FLAG_FAT_DIRTY)
#endif /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 0) */
#if (DOSFS_CONFIG_DATA_CACHE_ENTRIES == 0)
        || volume->data_file
#endif /* (DOSFS_CONFIG_DATA_CACHE_ENTRIES == 0) */
        )
    {
        status = dosfs_dir_cache_write(volume);
    }

    if (status == F_NO_ERROR)
#endif /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 0) || (DOSFS_CONFIG_DATA_CACHE_ENTRIES == 0) */
    {
        if (zero)
        {
            memset(volume->dir_cache.data, 0, DOSFS_BLK_SIZE);

            volume->dir_cache.blkno = blkno;
        }
        else
        {
            status = dosfs_volume_read(volume, blkno, volume->dir_cache.data);
            
            if (status == F_NO_ERROR)
            {
                volume->dir_cache.blkno = blkno;
            }
        }
    }

    return status;
}

static int dosfs_dir_cache_read(dosfs_volume_t *volume, uint32_t blkno, dosfs_cache_entry_t **p_entry)
{
    int status = F_NO_ERROR;

    if (volume->dir_cache.blkno != blkno)
    {
        DOSFS_VOLUME_STATISTICS_COUNT(dir_cache_miss);
        DOSFS_VOLUME_STATISTICS_COUNT(dir_cache_read);

        status = dosfs_dir_cache_fill(volume, blkno, FALSE);
    }
    else
    {
        DOSFS_VOLUME_STATISTICS_COUNT(dir_cache_hit);
    }

    *p_entry = &volume->dir_cache;

    return status;
}

static int dosfs_dir_cache_zero(dosfs_volume_t *volume, uint32_t blkno, dosfs_cache_entry_t **p_entry)
{
    int status = F_NO_ERROR;

    if (volume->dir_cache.blkno != blkno)
    {
        DOSFS_VOLUME_STATISTICS_COUNT(dir_cache_miss);
        DOSFS_VOLUME_STATISTICS_COUNT(dir_cache_zero);

        status = dosfs_dir_cache_fill(volume, blkno, TRUE);
    }
    else
    {
        DOSFS_VOLUME_STATISTICS_COUNT(dir_cache_hit);
    }

    *p_entry = &volume->dir_cache;

    return status;
}

static int dosfs_dir_cache_flush(dosfs_volume_t *volume)
{
    int status = F_NO_ERROR;

#if (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 0) || (DOSFS_CONFIG_DATA_CACHE_ENTRIES == 0)
    if (0
#if (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 0)
        || (volume->flags & DOSFS_VOLUME_FLAG_FAT_DIRTY)
#endif /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 0) */
#if (DOSFS_CONFIG_DATA_CACHE_ENTRIES == 0)
        || volume->data_file
#endif /* (DOSFS_CONFIG_DATA_CACHE_ENTRIES == 0) */
        )
    {
#if (DOSFS_CONFIG_DATA_CACHE_ENTRIES == 0)
        if (volume->data_file)
        {
            DOSFS_VOLUME_STATISTICS_COUNT(data_cache_flush);
        }
#endif /* (DOSFS_CONFIG_DATA_CACHE_ENTRIES == 0) */

#if (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 0)
        if (volume->flags & DOSFS_VOLUME_FLAG_FAT_DIRTY)
        {
            DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_flush);
        }
#endif /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 0) */

        status = dosfs_dir_cache_write(volume);
    }
#endif /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 0) || (DOSFS_CONFIG_DATA_CACHE_ENTRIES == 0) */

    return status;
}

/***********************************************************************************************************************/

#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)

static int dosfs_map_cache_fill(dosfs_volume_t *volume, uint32_t page)
{
    int status = F_NO_ERROR;

    if (volume->map_flags & DOSFS_MAP_FLAG_MAP_DIRTY)
    {
        status = dosfs_volume_write(volume, volume->map_cache.blkno, volume->map_cache.data);
    }
            
    if (status == F_NO_ERROR)
    {
        if (!(volume->map_flags & (DOSFS_MAP_FLAG_MAP_0_CHANGED << page)))
        {
            memset(volume->map_cache.data, 0, DOSFS_BLK_SIZE);
            
            volume->map_cache.blkno = volume->map_blkno + page;
        }
        else
        {
            status = dosfs_volume_read(volume, (volume->map_blkno + page), volume->map_cache.data);

            if (status == F_NO_ERROR)
            {
                volume->map_cache.blkno = volume->map_blkno + page;
            }
        }
    }

    return status;
}

static int dosfs_map_cache_write(dosfs_volume_t *volume, uint32_t blkno, const uint8_t *data)
{
    int status = F_NO_ERROR;
    uint32_t page, index, mask, offset;
    uint32_t *map;

    page   = (blkno - volume->fat1_blkno) >> (DOSFS_BLK_SHIFT + 8);
    index  = ((blkno - volume->fat1_blkno) >> 5) & DOSFS_BLK_MASK;
    mask   = 1ul << ((blkno - volume->fat1_blkno) & 31);
    offset = volume->fat_blkcnt;

    if (volume->map_cache.blkno != (volume->map_blkno + page))
    {
        status = dosfs_map_cache_fill(volume, page);
    }

    if (status == F_NO_ERROR)
    {
        map = (uint32_t*)((void*)volume->map_cache.data);
        
        if (!(map[index] & mask))
        {
            map[index] |= mask;
            
            volume->map_flags |= (DOSFS_MAP_FLAG_MAP_DIRTY | (DOSFS_MAP_FLAG_MAP_0_CHANGED << page));
            
            if (volume->map_entries >= DOSFS_MAP_TABLE_ENTRIES)
            {
                volume->map_entries = DOSFS_MAP_TABLE_OVERFLOW;
            }
            else
            {
                volume->map_table[volume->map_entries++] = (blkno - volume->fat1_blkno);
            }
        }

        status = dosfs_volume_write(volume, (blkno + offset), data);
    }

    return status;
}

static int dosfs_map_cache_read(dosfs_volume_t *volume, uint32_t blkno, uint8_t *data)
{
    int status = F_NO_ERROR;
    uint32_t page, index, mask, offset;
    uint32_t *map;

    page   = (blkno - volume->fat1_blkno) >> (DOSFS_BLK_SHIFT + 8);
    index  = ((blkno - volume->fat1_blkno) >> 5) & DOSFS_BLK_MASK;
    mask   = 1ul << ((blkno - volume->fat1_blkno) & 31);
    offset = 0;

    if (volume->map_cache.blkno != (volume->map_blkno + page))
    {
        status = dosfs_map_cache_fill(volume, page);
    }

    if (status == F_NO_ERROR)
    {
        map = (uint32_t*)((void*)volume->map_cache.data);
        
        if (map[index] & mask)
        {
            offset = volume->fat_blkcnt;
        }

        status = dosfs_volume_read(volume, (blkno + offset), data);
    }

    return status;
}

static int dosfs_map_cache_flush(dosfs_volume_t *volume)
{
    int status = F_NO_ERROR;

    status = dosfs_fat_cache_flush(volume);

    if (status == F_NO_ERROR)
    {
        if (volume->map_entries == DOSFS_MAP_TABLE_OVERFLOW)
        {
            if (volume->map_flags & DOSFS_MAP_FLAG_MAP_DIRTY)
            {
                if (volume->type != DOSFS_VOLUME_TYPE_FAT32)
                {
                    /* For FAT12/FAT16 the map is stored in volume->map_table[]. There
                     * can be at most 256 FAT blocks for FAT32, which means 256 bits,
                     * which is 64 bytes. Hence it fits into volume->map_table[].
                     */
                    memcpy(volume->map_table, volume->map_cache.data, 64);
                }
                else
                {
                    status = dosfs_volume_write(volume, volume->map_cache.blkno, volume->map_cache.data);
                }
            }
        }

        if (status == F_NO_ERROR)
        {
            volume->map_flags &= ~DOSFS_MAP_FLAG_MAP_DIRTY;

#if (DOSFS_CONFIG_FSINFO_SUPPORTED == 1)
            if ((volume->map_flags & DOSFS_MAP_FLAG_MAP_CHANGED) &&
                (volume->fsinfo_blkofs != 0) &&
                (volume->flags & DOSFS_VOLUME_FLAG_FSINFO_VALID) &&
                (volume->flags & DOSFS_VOLUME_FLAG_FSINFO_DIRTY))
            {
                volume->map_flags |= DOSFS_MAP_FLAG_MAP_FSINFO;
            }
#endif /* (DOSFS_CONFIG_FSINFO_SUPPORTED == 1) */
        }
    }

    return status;
}

static int dosfs_map_cache_resolve(dosfs_volume_t *volume)
{
    int status = F_NO_ERROR;
    uint16_t *map_table, *map_table_e;
    uint32_t blkno, blkno_n, mask, page;
    uint32_t *map, *map_e;
    uint8_t *data;
    dosfs_cache_entry_t *entry;

    status = dosfs_dir_cache_flush(volume);

    if (status == F_NO_ERROR)
    {
        if (volume->map_entries != DOSFS_MAP_TABLE_OVERFLOW)
        {
            map_table = &volume->map_table[0];
            map_table_e = &volume->map_table[volume->map_entries];

            do
            {
                blkno = volume->fat1_blkno + *map_table++;

#if (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 0)
                if (blkno == volume->dir_cache.blkno)
                {
                    data = volume->dir_cache.data;
                }
#else /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 0) */
#if (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 1)
                if (blkno == volume->fat_cache.blkno)
                {
                    data = volume->fat_cache.data;
                }
#else /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 1) */
                if (blkno == volume->fat_cache[0].blkno)
                {
                    data = volume->fat_cache[0].data;
                }
                else if (blkno == volume->fat_cache[1].blkno)
                {
                    data = volume->fat_cache[1].data;
                }
#endif /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 1) */
#endif /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 0) */
                else
                {
                    status = dosfs_dir_cache_read(volume, blkno + volume->fat_blkcnt, &entry);

                    if (status == F_NO_ERROR)
                    {
                        entry->blkno = blkno;

                        data = entry->data;
                    }
                }

                if (status == F_NO_ERROR)
                {
                    status = dosfs_volume_write(volume, blkno, data);
                }
            }
            while ((status == F_NO_ERROR) && (map_table < map_table_e));
        }
        else
        {
            do
            {
                blkno = volume->fat1_blkno;

                if (volume->type != DOSFS_VOLUME_TYPE_FAT32)
                {
                    /* For FAT12/FAT16 the map is stored in volume->map_table[]. There
                     * can be at most 256 FAT blocks for FAT32, which means 256 bits,
                     * which is 64 bytes. Hence it fits into volume->map_table[].
                     */
                    page = 0;

                    map = (uint32_t*)((void*)volume->map_table);
                    map_e = (uint32_t*)((void*)((uint8_t*)volume->map_table + 64));
                }
                else
                {
                    if (volume->map_flags & (DOSFS_MAP_FLAG_MAP_0_CHANGED << (volume->map_cache.blkno - volume->map_blkno)))
                    {
                        page = (volume->map_cache.blkno - volume->map_blkno);
                    }
                    else
                    {
                        if (volume->map_flags & DOSFS_MAP_FLAG_MAP_0_CHANGED)
                        {
                            page = 0;
                        }
                        else
                        {
                            page = 1;
                        }

                        if (volume->map_cache.blkno != (volume->map_blkno + page))
                        {
                            status = dosfs_map_cache_fill(volume, page);
                        }
                    }

                    if (page == 1)
                    {
                        blkno += (DOSFS_BLK_SIZE * 8);
                    }
                    
                    map = (uint32_t*)((void*)volume->map_cache.data);
                    map_e = (uint32_t*)((void*)(volume->map_cache.data + DOSFS_BLK_SIZE));
                }

                while ((status == F_NO_ERROR) && (map < map_e))
                {
                    mask = *map++;

                    blkno_n  = blkno + 32;
                    
                    while (mask)
                    {
                        if (mask & 1)
                        {
#if (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 0)
                            if (blkno == volume->dir_cache.blkno)
                            {
                                data = volume->dir_cache.data;
                            }
#else /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 0) */
#if (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 1)
                            if (blkno == volume->fat_cache.blkno)
                            {
                                data = volume->fat_cache.data;
                            }
#else /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 1) */
                            if (blkno == volume->fat_cache[0].blkno)
                            {
                                data = volume->fat_cache[0].data;
                            }
                            else if (blkno == volume->fat_cache[1].blkno)
                            {
                                data = volume->fat_cache[1].data;
                            }
#endif /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 1) */
#endif /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 0) */
                            else
                            {
                                status = dosfs_dir_cache_read(volume, blkno + volume->fat_blkcnt, &entry);

                                if (status == F_NO_ERROR)
                                {
                                    entry->blkno = blkno;

                                    data = entry->data;
                                }
                            }
                            
                            if (status == F_NO_ERROR)
                            {
                                status = dosfs_volume_write(volume, blkno, data);
                            }
                        }

                        mask >>= 1;
                        blkno++;
                    }

                    blkno = blkno_n;
                }

                if (status == F_NO_ERROR)
                {
                    volume->map_flags &= ~(DOSFS_MAP_FLAG_MAP_0_CHANGED << page);
                }
            }
            while ((status == F_NO_ERROR) && (volume->map_flags & DOSFS_MAP_FLAG_MAP_CHANGED));
        }

        if (status == F_NO_ERROR)
        {
#if (DOSFS_CONFIG_FSINFO_SUPPORTED == 1)
            if (volume->map_flags & DOSFS_MAP_FLAG_MAP_FSINFO)
            {
                volume->flags |= (DOSFS_VOLUME_FLAG_FSINFO_VALID | DOSFS_VOLUME_FLAG_FSINFO_DIRTY);
            }
#endif /* (DOSFS_CONFIG_FSINFO_SUPPORTED == 1) */

            volume->map_flags = 0;
            volume->map_entries = 0;

            volume->map_cache.blkno = DOSFS_BLKNO_INVALID;

            if (volume->type != DOSFS_VOLUME_TYPE_FAT32)
            {
                /* For FAT12/FAT16 the map is stored in volume->map_table[]. There
                 * can be at most 256 FAT blocks for FAT32, which means 256 bits,
                 * which is 64 bytes. Hence it fits into volume->map_table[].
                 */
                memset(volume->map_cache.data, 0, 64);
            }

        }
    }

    return status;
}

#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */

/***********************************************************************************************************************/

#if (DOSFS_CONFIG_FAT_CACHE_ENTRIES != 0)
#if (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 1)

static int dosfs_fat_cache_write(dosfs_volume_t *volume, dosfs_cache_entry_t *entry)
{
    int status = F_NO_ERROR;

#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)
    status = dosfs_map_cache_write(volume, volume->fat_cache.blkno, volume->fat_cache.data);
#else /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */
    status = dosfs_volume_write(volume, volume->fat_cache.blkno, volume->fat_cache.data);
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */
                
    if (status == F_NO_ERROR)
    {
#if (DOSFS_CONFIG_2NDFAT_SUPPORTED == 1) && (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) 
        if (volume->fat2_blkno)
        {
            status = dosfs_volume_write(volume, volume->fat_cache.blkno + volume->fat_blkcnt, volume->fat_cache.data);
        }

        if (status == F_NO_ERROR)
#endif /* (DOSFS_CONFIG_2NDFAT_SUPPORTED == 1) && (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */
        {
            DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_write);

            volume->flags &= ~DOSFS_VOLUME_FLAG_FAT_DIRTY;
        }
    }

    return status;
}

static int dosfs_fat_cache_fill(dosfs_volume_t *volume, uint32_t blkno, dosfs_cache_entry_t **p_entry)
{
    int status = F_NO_ERROR;

    if (volume->flags & DOSFS_VOLUME_FLAG_FAT_DIRTY)
    {
        status = dosfs_fat_cache_write(volume, &volume->fat_cache);
    }

    if (status == F_NO_ERROR)
    {
        DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_miss);

#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)
        status = dosfs_map_cache_read(volume, blkno, volume->fat_cache.data);
#else /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */
        status = dosfs_volume_read(volume, blkno, volume->fat_cache.data);
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */
        
        if (status == F_NO_ERROR)
        {
            DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_read);

            volume->fat_cache.blkno = blkno;
        }
    }

    *p_entry = &volume->fat_cache;

    return status;
}

static int dosfs_fat_cache_read(dosfs_volume_t *volume, uint32_t blkno, dosfs_cache_entry_t **p_entry)
{
    int status = F_NO_ERROR;

    if (volume->fat_cache.blkno != blkno)
    {
        status = dosfs_fat_cache_fill(volume, blkno, p_entry);
    }
    else
    {
        DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_hit);

        *p_entry = &volume->fat_cache;
    }

    return status;
}

static inline void dosfs_fat_cache_modify(dosfs_volume_t *volume, dosfs_cache_entry_t *entry)
{
    volume->flags |= DOSFS_VOLUME_FLAG_FAT_DIRTY;
}

static int dosfs_fat_cache_flush(dosfs_volume_t *volume)
{
    int status = F_NO_ERROR;

    if (volume->flags & DOSFS_VOLUME_FLAG_FAT_DIRTY)
    {
        DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_flush);

        status = dosfs_fat_cache_write(volume, &volume->fat_cache);
    }

    return status;
}

#else /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 1) */

static int dosfs_fat_cache_write(dosfs_volume_t *volume, dosfs_cache_entry_t *entry)
{
    int status = F_NO_ERROR;

    DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_write);

#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)
    status = dosfs_map_cache_write(volume, entry->blkno, entry->data);
#else /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */
    status = dosfs_volume_write(volume, entry->blkno, entry->data);
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */

    if (status == F_NO_ERROR)
    {
#if (DOSFS_CONFIG_2NDFAT_SUPPORTED == 1) && (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0)
        if (volume->fat2_blkno)
        {
            status = dosfs_volume_write(volume, entry->blkno + volume->fat_blkcnt, entry->data);
        }

        if (status == F_NO_ERROR)
#endif /* (DOSFS_CONFIG_2NDFAT_SUPPORTED == 1) && (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */
        {
            volume->flags &= ((entry == &volume->fat_cache[0]) ? ~DOSFS_VOLUME_FLAG_FAT_0_DIRTY : ~DOSFS_VOLUME_FLAG_FAT_1_DIRTY);
        }
    }

    return status;
}

static int dosfs_fat_cache_fill(dosfs_volume_t *volume, uint32_t blkno, dosfs_cache_entry_t **p_entry)
{
    int status = F_NO_ERROR;
    unsigned int index;

    index = (volume->flags & DOSFS_VOLUME_FLAG_FAT_INDEX_CURRENT) ^ DOSFS_VOLUME_FLAG_FAT_INDEX_CURRENT;

#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0)
    /* For a non-TRANSACTION_SAFE setup, the write backs from the cache
     * have to be in sequence. That means, if there is any cache entry
     * dirty it have to be written back, before either a new one gets 
     * read or the non least recently used one is used.
     */
    if (volume->flags & (DOSFS_VOLUME_FLAG_FAT_0_DIRTY | DOSFS_VOLUME_FLAG_FAT_1_DIRTY))
    {
        status = dosfs_fat_cache_write(volume, &volume->fat_cache[index ^ DOSFS_VOLUME_FLAG_FAT_INDEX_CURRENT]);
    }

    if (status == F_NO_ERROR)
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */
    {
        if (volume->fat_cache[index].blkno != blkno)
        {
            DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_miss);
            DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_read);
            
#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)
            if (volume->flags & (DOSFS_VOLUME_FLAG_FAT_0_DIRTY << index))
            {
                status = dosfs_fat_cache_write(volume, &volume->fat_cache[index]);
            }
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */

            if (status == F_NO_ERROR)
            {
#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)
                status = dosfs_map_cache_read(volume, blkno, volume->fat_cache[index].data);
#else /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */
                status = dosfs_volume_read(volume, blkno, volume->fat_cache[index].data);
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */
            
                if (status == F_NO_ERROR)
                {
                    volume->flags ^= DOSFS_VOLUME_FLAG_FAT_INDEX_CURRENT;
                
                    volume->fat_cache[index].blkno = blkno;
                }
            }
        }
        else
        {
            DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_hit);

            volume->flags ^= DOSFS_VOLUME_FLAG_FAT_INDEX_CURRENT;
        }
    }

    *p_entry = &volume->fat_cache[index];

    return status;
}

static int dosfs_fat_cache_read(dosfs_volume_t *volume, uint32_t blkno, dosfs_cache_entry_t **p_entry)
{
    int status = F_NO_ERROR;
    unsigned int index;

    index = volume->flags & DOSFS_VOLUME_FLAG_FAT_INDEX_CURRENT;

    if (volume->fat_cache[index].blkno != blkno)
    {
        status = dosfs_fat_cache_fill(volume, blkno, p_entry);
    }
    else
    {
        DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_hit);

        *p_entry = &volume->fat_cache[index];
    }

    return status;
}

static inline void dosfs_fat_cache_modify(dosfs_volume_t *volume, dosfs_cache_entry_t *entry)
{
    volume->flags |= ((entry == &volume->fat_cache[0]) ? DOSFS_VOLUME_FLAG_FAT_0_DIRTY : DOSFS_VOLUME_FLAG_FAT_1_DIRTY);
}

static int dosfs_fat_cache_flush(dosfs_volume_t *volume)
{
    int status = F_NO_ERROR;

    if (volume->flags & DOSFS_VOLUME_FLAG_FAT_0_DIRTY)
    {
        DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_flush);

        status = dosfs_fat_cache_write(volume, &volume->fat_cache[0]);
    }

    if ((status == F_NO_ERROR) && (volume->flags & DOSFS_VOLUME_FLAG_FAT_1_DIRTY))
    {
        DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_flush);

        status = dosfs_fat_cache_write(volume, &volume->fat_cache[1]);
    }

    return status;
}

#endif /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 1) */

#else /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES != 0) */

static inline int dosfs_fat_cache_write(dosfs_volume_t *volume, dosfs_cache_entry_t *entry)
{
    return dosfs_dir_cache_write(volume);
}

#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)

static int dosfs_fat_cache_fill(dosfs_volume_t *volume, uint32_t blkno, dosfs_cache_entry_t **p_entry)
{
    int status = F_NO_ERROR;

    if ((volume->flags & DOSFS_VOLUME_FLAG_FAT_DIRTY) 
#if (DOSFS_CONFIG_DATA_CACHE_ENTRIES == 0)
        || volume->data_file
#endif /* (DOSFS_CONFIG_DATA_CACHE_ENTRIES == 0) */
        )
    {
        status = dosfs_dir_cache_write(volume);
    }

    if (status == F_NO_ERROR)
    {
        DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_miss);

        status = dosfs_map_cache_read(volume, blkno, volume->dir_cache.data);
        
        if (status == F_NO_ERROR)
        {
            DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_read);

            volume->dir_cache.blkno = blkno;
        }
    }

    *p_entry = &volume->dir_cache;

    return status;
}

static int dosfs_fat_cache_read(dosfs_volume_t *volume, uint32_t blkno, dosfs_cache_entry_t **p_entry)
{
    int status = F_NO_ERROR;

    if (volume->dir_cache.blkno != blkno)
    {
        DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_miss);
        DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_read);

        status = dosfs_fat_cache_fill(volume, blkno, p_entry);
    }
    else
    {
        DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_hit);

        *p_entry = &volume->dir_cache;
    }

    return status;
}

#else /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */

static int dosfs_fat_cache_fill(dosfs_volume_t *volume, uint32_t blkno, dosfs_cache_entry_t **p_entry)
{
    int status = F_NO_ERROR;

    status = dosfs_dir_cache_fill(volume, blkno, FALSE);

    *p_entry = &volume->dir_cache;

    return status;
}

#if (DOSFS_CONFIG_STATISTICS == 1)

static int dosfs_fat_cache_read(dosfs_volume_t *volume, uint32_t blkno, dosfs_cache_entry_t **p_entry)
{
    int status = F_NO_ERROR;

    if (volume->dir_cache.blkno != blkno)
    {
        DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_miss);
        DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_read);

        status = dosfs_dir_cache_fill(volume, blkno, FALSE);
    }
    else
    {
        DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_hit);
    }

    *p_entry = &volume->dir_cache;

    return status;
}

#else /* (DOSFS_CONFIG_STATISTICS == 1) */

static inline int dosfs_fat_cache_read(dosfs_volume_t *volume, uint32_t blkno, dosfs_cache_entry_t **p_entry)
{
    return dosfs_dir_cache_read(volume, blkno, p_entry);
}

#endif /* (DOSFS_CONFIG_STATISTICS == 1) */

#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */

static inline void dosfs_fat_cache_modify(dosfs_volume_t *volume, dosfs_cache_entry_t *entry)
{
    volume->flags |= DOSFS_VOLUME_FLAG_FAT_DIRTY;
}

static int dosfs_fat_cache_flush(dosfs_volume_t *volume)
{
    int status = F_NO_ERROR;

    if (volume->flags & DOSFS_VOLUME_FLAG_FAT_DIRTY)
    {
        DOSFS_VOLUME_STATISTICS_COUNT(fat_cache_flush);

        status = dosfs_dir_cache_write(volume);
    }

    return status;
}

#endif /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES != 0) */

/***********************************************************************************************************************/

#if (DOSFS_CONFIG_DATA_CACHE_ENTRIES != 0)
#if (DOSFS_CONFIG_FILE_DATA_CACHE == 1)

static int dosfs_data_cache_write(dosfs_volume_t *volume, dosfs_file_t *file)
{
    int status = F_NO_ERROR;
    dosfs_device_t *device;

    device = DOSFS_VOLUME_DEVICE(volume);

    status = (*device->interface->write)(device->context, file->data_cache.blkno, file->data_cache.data, 1, !!(file->mode & DOSFS_FILE_MODE_RANDOM));

#if (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1)
    if (status == F_ERR_INVALIDSECTOR)
    {
        volume->flags |= DOSFS_VOLUME_FLAG_MEDIA_FAILURE;
    }
#endif /* (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1) */

    if (status == F_NO_ERROR)
    {
        DOSFS_VOLUME_STATISTICS_COUNT(data_cache_write);

        device->lock |= DOSFS_DEVICE_LOCK_MODIFIED;
    }

    /* Unconditionally clean the dirty condition, or
     * otherwise the while system would get stuck.
     */
    file->flags &= ~DOSFS_FILE_FLAG_DATA_DIRTY;

    return status;
}

static int dosfs_data_cache_fill(dosfs_volume_t *volume, dosfs_file_t *file, uint32_t blkno, int zero)
{
    int status = F_NO_ERROR;
    dosfs_device_t *device;

    device = DOSFS_VOLUME_DEVICE(volume);

    if (file->flags & DOSFS_FILE_FLAG_DATA_DIRTY)
    {
        status = dosfs_data_cache_write(volume, file);
    }

    if (status == F_NO_ERROR)
    {
        DOSFS_VOLUME_STATISTICS_COUNT(data_cache_miss);

        if (zero)
        {
            memset(file->data_cache.data, 0, DOSFS_BLK_SIZE);

            DOSFS_VOLUME_STATISTICS_COUNT(data_cache_zero);

            file->data_cache.blkno = blkno;
        }
        else
        {
            status = (*device->interface->read)(device->context, blkno, file->data_cache.data, 1, !!(file->mode & DOSFS_FILE_MODE_SEQUENTIAL));

#if (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1)
            if (status == F_ERR_INVALIDSECTOR)
            {
                volume->flags |= DOSFS_VOLUME_FLAG_MEDIA_FAILURE;
            }
#endif /* (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1) */
            
            if (status == F_NO_ERROR)
            {
                DOSFS_VOLUME_STATISTICS_COUNT(data_cache_read);

                file->data_cache.blkno = blkno;
            }
        }
    }

    return status;
}

static int dosfs_data_cache_read(dosfs_volume_t *volume, dosfs_file_t *file, uint32_t blkno, dosfs_cache_entry_t ** p_entry)
{
    int status = F_NO_ERROR;

    if (file->data_cache.blkno != blkno)
    {
        status = dosfs_data_cache_fill(volume, file, blkno, FALSE);
    }
    else
    {
        DOSFS_VOLUME_STATISTICS_COUNT(data_cache_hit);
    }

    *p_entry = &file->data_cache;
    
    return status;
}

static int dosfs_data_cache_zero(dosfs_volume_t *volume, dosfs_file_t *file, uint32_t blkno, dosfs_cache_entry_t ** p_entry)
{
    int status = F_NO_ERROR;

    if (file->data_cache.blkno != blkno)
    {
        status = dosfs_data_cache_fill(volume, file, blkno, TRUE);
    }
    else
    {
        DOSFS_VOLUME_STATISTICS_COUNT(data_cache_hit);
    }

    *p_entry = &file->data_cache;
    
    return status;
}

static inline void dosfs_data_cache_modify(dosfs_volume_t *volume, dosfs_file_t *file)
{
    file->flags |= DOSFS_FILE_FLAG_DATA_DIRTY;
}

static int dosfs_data_cache_flush(dosfs_volume_t *volume, dosfs_file_t *file)
{
    int status = F_NO_ERROR;

    if (file->flags & DOSFS_FILE_FLAG_DATA_DIRTY)
    {
        DOSFS_VOLUME_STATISTICS_COUNT(data_cache_flush);

        status = dosfs_data_cache_write(volume, file);
    }

    return status;
}

static int dosfs_data_cache_invalidate(dosfs_volume_t *volume, dosfs_file_t *file, uint32_t blkno, uint32_t blkcnt)
{
    int status = F_NO_ERROR;

    if ((blkno <= file->data_cache.blkno) && (file->data_cache.blkno < (blkno + blkcnt)))
    {
        DOSFS_VOLUME_STATISTICS_COUNT(data_cache_invalidate);

        file->data_cache.blkno = DOSFS_BLKNO_INVALID;

        file->flags &= ~DOSFS_FILE_FLAG_DATA_DIRTY;
    }

    return status;
}

#else /* (DOSFS_CONFIG_FILE_DATA_CACHE == 1) */

static int dosfs_data_cache_write(dosfs_volume_t *volume, dosfs_file_t *file)
{
    int status = F_NO_ERROR;
    dosfs_device_t *device;

    device = DOSFS_VOLUME_DEVICE(volume);

    status = (*device->interface->write)(device->context, volume->data_cache.blkno, volume->data_cache.data, 1, !!(file->mode & DOSFS_FILE_MODE_RANDOM));

#if (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1)
    if (status == F_ERR_INVALIDSECTOR)
    {
        volume->flags |= DOSFS_VOLUME_FLAG_MEDIA_FAILURE;
    }
#endif /* (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1) */

    if (status == F_NO_ERROR)
    {
        DOSFS_VOLUME_STATISTICS_COUNT(data_cache_write);

        device->lock |= DOSFS_DEVICE_LOCK_MODIFIED;
    }

    /* Unconditionally clean the dirty condition, or
     * otherwise the while system would get stuck.
     */
    volume->data_file = NULL;

    return status;
}

static int dosfs_data_cache_fill(dosfs_volume_t *volume, dosfs_file_t *file, uint32_t blkno, int zero)
{
    int status = F_NO_ERROR;
    dosfs_device_t *device;

    device = DOSFS_VOLUME_DEVICE(volume);

    if (volume->data_file)
    {
        status = dosfs_data_cache_write(volume, file);
    }

    if (status == F_NO_ERROR)
    {
        DOSFS_VOLUME_STATISTICS_COUNT(data_cache_miss);

        if (zero)
        {
            memset(volume->data_cache.data, 0, DOSFS_BLK_SIZE);

            DOSFS_VOLUME_STATISTICS_COUNT(data_cache_zero);

            volume->data_cache.blkno = blkno;
        }
        else
        {
            status = (*device->interface->read)(device->context, blkno, volume->data_cache.data, 1, !!(file->mode & DOSFS_FILE_MODE_SEQUENTIAL));

#if (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1)
            if (status == F_ERR_INVALIDSECTOR)
            {
                volume->flags |= DOSFS_VOLUME_FLAG_MEDIA_FAILURE;
            }
#endif /* (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1) */
            
            if (status == F_NO_ERROR)
            {
                DOSFS_VOLUME_STATISTICS_COUNT(data_cache_read);

                volume->data_cache.blkno = blkno;
            }
        }
    }

    return status;
}

static int dosfs_data_cache_read(dosfs_volume_t *volume, dosfs_file_t *file, uint32_t blkno, dosfs_cache_entry_t ** p_entry)
{
    int status = F_NO_ERROR;

    if (volume->data_cache.blkno != blkno)
    {
        status = dosfs_data_cache_fill(volume, file, blkno, FALSE);
    }
    else
    {
        DOSFS_VOLUME_STATISTICS_COUNT(data_cache_hit);
    }

    *p_entry = &volume->data_cache;
    
    return status;
}

static int dosfs_data_cache_zero(dosfs_volume_t *volume, dosfs_file_t *file, uint32_t blkno, dosfs_cache_entry_t ** p_entry)
{
    int status = F_NO_ERROR;

    if (volume->data_cache.blkno != blkno)
    {
        status = dosfs_data_cache_fill(volume, file, blkno, TRUE);
    }
    else
    {
        DOSFS_VOLUME_STATISTICS_COUNT(data_cache_hit);
    }

    *p_entry = &volume->data_cache;
    
    return status;
}

static inline void dosfs_data_cache_modify(dosfs_volume_t *volume, dosfs_file_t *file)
{
    volume->data_file = file;
}

static int dosfs_data_cache_flush(dosfs_volume_t *volume, dosfs_file_t *file)
{
    int status = F_NO_ERROR;

    if (volume->data_file == file)
    {
        DOSFS_VOLUME_STATISTICS_COUNT(data_cache_flush);

        status = dosfs_data_cache_write(volume, file);
    }

    return status;
}

static int dosfs_data_cache_invalidate(dosfs_volume_t *volume, dosfs_file_t *file, uint32_t blkno, uint32_t blkcnt)
{
    int status = F_NO_ERROR;

    if ((blkno <= volume->data_cache.blkno) && (volume->data_cache.blkno < (blkno + blkcnt)))
    {
        DOSFS_VOLUME_STATISTICS_COUNT(data_cache_invalidate);

        volume->data_cache.blkno = DOSFS_BLKNO_INVALID;

        volume->data_file = NULL;
    }

    return status;
}

#endif /* (DOSFS_CONFIG_FILE_DATA_CACHE == 1) */

#else /* (DOSFS_CONFIG_DATA_CACHE_ENTRIES != 0) */

static int dosfs_data_cache_write(dosfs_volume_t *volume, dosfs_file_t *file)
{
    int status = F_NO_ERROR;

    status = dosfs_dir_cache_write(volume);

    return status;
}

static int dosfs_data_cache_fill(dosfs_volume_t *volume, dosfs_file_t *file, uint32_t blkno, int zero)
{
    int status = F_NO_ERROR;
    dosfs_device_t *device;

    device = DOSFS_VOLUME_DEVICE(volume);

    if (volume->data_file
#if (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 0)
        || (volume->flags & DOSFS_VOLUME_FLAG_FAT_DIRTY)
#endif /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 0) */
        )
    {
        status = dosfs_dir_cache_write(volume);
    }

    if (status == F_NO_ERROR)
    {
        if (zero)
        {
            memset(volume->dir_cache.data, 0, DOSFS_BLK_SIZE);

            volume->dir_cache.blkno = blkno;
        }
        else
        {
            status = (*device->interface->read)(device->context, blkno, volume->dir_cache.data, 1, !!(file->mode & DOSFS_FILE_MODE_SEQUENTIAL));

#if (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1)
            if (status == F_ERR_INVALIDSECTOR)
            {
                volume->flags |= DOSFS_VOLUME_FLAG_MEDIA_FAILURE;
            }
#endif /* (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1) */
            
            if (status == F_NO_ERROR)
            {
                volume->dir_cache.blkno = blkno;
            }
        }
    }

    return status;
}

static int dosfs_data_cache_read(dosfs_volume_t *volume, dosfs_file_t *file, uint32_t blkno, dosfs_cache_entry_t ** p_entry)
{
    int status = F_NO_ERROR;

    if (volume->dir_cache.blkno != blkno)
    {
        DOSFS_VOLUME_STATISTICS_COUNT(data_cache_miss);
        DOSFS_VOLUME_STATISTICS_COUNT(data_cache_read);

        status = dosfs_data_cache_fill(volume, file, blkno, FALSE);
    }
    else
    {
        DOSFS_VOLUME_STATISTICS_COUNT(data_cache_hit);
    }

    *p_entry = &volume->dir_cache;
    
    return status;
}

static int dosfs_data_cache_zero(dosfs_volume_t *volume, dosfs_file_t *file, uint32_t blkno, dosfs_cache_entry_t ** p_entry)
{
    int status = F_NO_ERROR;

    if (volume->dir_cache.blkno != blkno)
    {
        DOSFS_VOLUME_STATISTICS_COUNT(data_cache_miss);
        DOSFS_VOLUME_STATISTICS_COUNT(data_cache_zero);

        status = dosfs_data_cache_fill(volume, file, blkno, TRUE);
    }

    *p_entry = &volume->dir_cache;
    
    return status;
}

static inline void dosfs_data_cache_modify(dosfs_volume_t *volume, dosfs_file_t *file)
{
    volume->data_file = file;
}

static int dosfs_data_cache_flush(dosfs_volume_t *volume, dosfs_file_t *file)
{
    int status = F_NO_ERROR;

    if (volume->data_file == file)
    {
        DOSFS_VOLUME_STATISTICS_COUNT(data_cache_flush);

        status = dosfs_data_cache_write(volume, file);
    }

    return status;
}

static int dosfs_data_cache_invalidate(dosfs_volume_t *volume, dosfs_file_t *file, uint32_t blkno, uint32_t blkcnt)
{
    int status = F_NO_ERROR;


    if ((blkno <= volume->dir_cache.blkno) && (volume->dir_cache.blkno < (blkno + blkcnt)))
    {
        DOSFS_VOLUME_STATISTICS_COUNT(data_cache_invalidate);

        volume->dir_cache.blkno = DOSFS_BLKNO_INVALID;

        volume->data_file = NULL;
    }

    return status;
}

#endif /* (DOSFS_CONFIG_DATA_CACHE_ENTRIES != 0) */

/***********************************************************************************************************************/

static int dosfs_cluster_read_uncached(dosfs_volume_t *volume, uint32_t clsno, uint32_t *p_clsdata)
{
    int status = F_NO_ERROR;
    uint32_t offset, blkno, clsdata;
    dosfs_cache_entry_t *entry;

#if (DOSFS_CONFIG_FAT12_SUPPORTED == 1)
    if (volume->type == DOSFS_VOLUME_TYPE_FAT12)
    {
        uint8_t *fat_data;

        offset = clsno + (clsno >> 1);
        blkno = volume->fat1_blkno + (offset >> DOSFS_BLK_SHIFT);

        status = dosfs_fat_cache_read(volume, blkno, &entry);
        
        if (status == F_NO_ERROR)
        {
            fat_data = (uint8_t*)(entry->data + (offset & DOSFS_BLK_MASK));
            
            if (clsno & 1)
            {
                clsdata = ((uint32_t)(*fat_data) >> 4);
            }
            else
            {
                clsdata = ((uint32_t)(*fat_data));
            }
            
            if ((offset & DOSFS_BLK_MASK) == DOSFS_BLK_MASK)
            {
                status = dosfs_fat_cache_read(volume, (blkno+1), &entry);
                
                if (status == F_NO_ERROR)
                {
                    fat_data = (uint8_t*)(entry->data + 0);
                }
            }
            else
            {
                fat_data++;
            }
            
            if (status == F_NO_ERROR)
            {
                if (clsno & 1)
                {
                    clsdata |= ((uint32_t)(*fat_data) << 4);
                }
                else
                {
                    clsdata |= (((uint32_t)(*fat_data) & 0x0000000fu) << 8);
                }
            
                if (clsdata >= DOSFS_CLSNO_RESERVED12)
                {
                    clsdata += (DOSFS_CLSNO_RESERVED32 - DOSFS_CLSNO_RESERVED12);
                }

                *p_clsdata = clsdata;
            }
        }
    }
    else
#endif /* DOSFS_CONFIG_FAT12_SUPPORTED == 1 */
    {
        offset = clsno << volume->type;
        blkno = volume->fat1_blkno + (offset >> DOSFS_BLK_SHIFT);

        status = dosfs_fat_cache_read(volume, blkno, &entry);
            
        if (status == F_NO_ERROR)
        {
            if (volume->type == DOSFS_VOLUME_TYPE_FAT16)
            {
                uint16_t *fat_data;

                fat_data = (uint16_t*)((void*)(entry->data + (offset & DOSFS_BLK_MASK)));
                
                clsdata = DOSFS_FTOHS(*fat_data);
                
                if (clsdata >= DOSFS_CLSNO_RESERVED16)
                {
                    clsdata += (DOSFS_CLSNO_RESERVED32 - DOSFS_CLSNO_RESERVED16);
                }
            }
            else
            {
                uint32_t *fat_data;
            
                fat_data = (uint32_t*)((void*)(entry->data + (offset & DOSFS_BLK_MASK)));
                
                clsdata = DOSFS_FTOHL(*fat_data) & 0x0fffffff;
            }

            *p_clsdata = clsdata;
        }
    }

    return status;
}

#if (DOSFS_CONFIG_CLUSTER_CACHE_ENTRIES != 0)

static int dosfs_cluster_read(dosfs_volume_t *volume, uint32_t clsno, uint32_t *p_clsdata)
{
    int status = F_NO_ERROR;
    uint32_t clsdata;
    unsigned int index;

    index = clsno % DOSFS_CONFIG_CLUSTER_CACHE_ENTRIES;

    if (volume->cluster_cache[index].clsno != clsno)
    {
        DOSFS_VOLUME_STATISTICS_COUNT(cluster_cache_miss);

        status = dosfs_cluster_read_uncached(volume, clsno, &clsdata);

        volume->cluster_cache[index].clsno = clsno;
        volume->cluster_cache[index].clsdata = clsdata;
    }
    else
    {
        DOSFS_VOLUME_STATISTICS_COUNT(cluster_cache_hit);
    }

    *p_clsdata = volume->cluster_cache[index].clsdata;

    return status;
}

#else /* (DOSFS_CONFIG_CLUSTER_CACHE_ENTRIES != 0) */

static inline int dosfs_cluster_read(dosfs_volume_t *volume, uint32_t clsno, uint32_t *p_clsdata)
{
    return dosfs_cluster_read_uncached(volume, clsno, p_clsdata);
}

#endif /* (DOSFS_CONFIG_CLUSTER_CACHE_ENTRIES != 0) */

static int dosfs_cluster_write(dosfs_volume_t *volume, uint32_t clsno, uint32_t clsdata, int allocate)
{
    int status = F_NO_ERROR;
    uint32_t offset, blkno;
    dosfs_cache_entry_t *entry;

#if (DOSFS_CONFIG_FAT12_SUPPORTED == 1)
    if (volume->type == DOSFS_VOLUME_TYPE_FAT12)
    {
        uint8_t *fat_data;

        offset = clsno + (clsno >> 1);
        blkno = volume->fat1_blkno + (offset >> DOSFS_BLK_SHIFT);
        
        status = dosfs_fat_cache_read(volume, blkno, &entry);

        if (status == F_NO_ERROR)
        {
            fat_data = (uint8_t*)(entry->data + (offset & DOSFS_BLK_MASK));
            
            if (clsno & 1)
            {
                *fat_data = (*fat_data & 0x0f) | (clsdata << 4);
            }
            else
            {
                *fat_data = clsdata;
            }
            
            if ((offset & DOSFS_BLK_MASK) == DOSFS_BLK_MASK)
            {
                dosfs_fat_cache_modify(volume, entry);
                
                status = dosfs_fat_cache_read(volume, (blkno+1), &entry);
                
                if (status == F_NO_ERROR)
                {
                    fat_data = (uint8_t*)(entry->data + 0);
                }
            }
            else
            {
                fat_data++;
            }
            
            if (status == F_NO_ERROR)
            {
                if (clsno & 1)
                {
                    *fat_data = clsdata >> 4;
                }
                else
                {
                    *fat_data = (*fat_data & 0xf0) | ((clsdata >> 8) & 0x0f);
                }
                
                dosfs_fat_cache_modify(volume, entry);
            }
        }
    }
    else
#endif /* (DOSFS_CONFIG_FAT12_SUPPORTED == 1) */
    {
        offset = clsno << volume->type;
        blkno = volume->fat1_blkno + (offset >> DOSFS_BLK_SHIFT);

        status = dosfs_fat_cache_read(volume, blkno, &entry);
            
        if (status == F_NO_ERROR)
        {
            if (volume->type == DOSFS_VOLUME_TYPE_FAT16)
            {
                uint16_t *fat_data;

                fat_data = (uint16_t*)((void*)(entry->data + (offset & DOSFS_BLK_MASK)));

                *fat_data = DOSFS_HTOFS(clsdata);
            }
            else
            {
                uint32_t *fat_data;

                fat_data = (uint32_t*)((void*)(entry->data + (offset & DOSFS_BLK_MASK)));

                *fat_data = (*fat_data & 0xf0000000) | (DOSFS_HTOFL(clsdata) & 0x0fffffff);
            }

            dosfs_fat_cache_modify(volume, entry);
        }
    }

#if (DOSFS_CONFIG_CLUSTER_CACHE_ENTRIES != 0)
    if (status == F_NO_ERROR)
    {
        unsigned int index;

        index = clsno % DOSFS_CONFIG_CLUSTER_CACHE_ENTRIES;
        
        if (allocate || (volume->cluster_cache[index].clsno == clsno))
        {
            volume->cluster_cache[index].clsno = clsno;
            volume->cluster_cache[index].clsdata = clsdata;
        }
    }

#endif /* (DOSFS_CONFIG_CLUSTER_CACHE_ENTRIES != 0) */

    return status;
}


static int dosfs_cluster_chain_seek(dosfs_volume_t *volume, uint32_t clsno, uint32_t clscnt, uint32_t *p_clsno)
{
    int status = F_NO_ERROR;
    uint32_t clsdata;

    do
    {
        status = dosfs_cluster_read(volume, clsno, &clsdata);
        
        if (status == F_NO_ERROR)
        {
            if ((clsdata >= 2) && (clsdata <= volume->last_clsno))
            {
                clsno = clsdata;
                clscnt--;
            }
            else
            {
                status = F_ERR_EOF;
            }
        }
    }
    while ((status == F_NO_ERROR) && (clscnt != 0));
    
    if (status == F_NO_ERROR)
    {
        *p_clsno = clsno;
    }

    return status;
}


/* In order to guarantee file system fault tolerance the chain allocation is done iteratively.
 * free entry is found, it's marked as END_OF_CHAIN, and then the previous entry in the chain
 * is linked to it. This ensures that there is not broken chain anywhere. The is only the
 * possibility that there is a lost END_OF_CHAIN somewhere. Given that the fat cache writes
 * in sequence, the link process happens AFTER the END_OF_CHAIN has been written. 
 * 
 * If the allocation fails, then the chain is first split at the original create point,
 * and then freed. Hence again no inconsistent state is possible.
 */

static int dosfs_cluster_chain_create(dosfs_volume_t *volume, uint32_t clsno, uint32_t clscnt, uint32_t *p_clsno_a, uint32_t *p_clsno_l)
{
    int status = F_NO_ERROR;
    uint32_t clsno_a, clsno_f, clsno_n, clsno_l, clscnt_a, clsdata;

    clsno_f = volume->next_clsno;
    clsno_a = DOSFS_CLSNO_NONE;
    clsno_l = DOSFS_CLSNO_NONE;
    clsno_n = clsno_f;

    clscnt_a = 0;

    do
    {
        /* Bypass cluster cache on read while searching.
         */
        status = dosfs_cluster_read_uncached(volume, clsno_n, &clsdata);
        
        if (status == F_NO_ERROR)
        {
            if (clsdata == DOSFS_CLSNO_FREE)
            {
                status = dosfs_cluster_write(volume, clsno_n, DOSFS_CLSNO_END_OF_CHAIN, TRUE);
                
                if (status == F_NO_ERROR)
                {
                    if (clsno_a == DOSFS_CLSNO_NONE)
                    {
                        clsno_a = clsno_n;
                    }
                    
                    if (clsno_l != DOSFS_CLSNO_NONE)
                    {
                        status = dosfs_cluster_write(volume, clsno_l, clsno_n, TRUE);
                    }
                    
                    clsno_l = clsno_n;
                    clscnt_a++;
                }
            }
            
            clsno_n++;

            if (clsno_n > volume->last_clsno)
            {
                clsno_n = 2; 
            }

            if (clsno_n == clsno_f)
            {
                status = F_ERR_NOMOREENTRY;
            }
        }
    }
    while ((status == F_NO_ERROR) && (clscnt != clscnt_a));

    if (status == F_NO_ERROR)
    {
        if (clsno != DOSFS_CLSNO_NONE)
        {
            status = dosfs_cluster_write(volume, clsno, clsno_a, TRUE);
        }
        
        if (status == F_NO_ERROR)
        {
#if (DOSFS_CONFIG_FSINFO_SUPPORTED == 1)
            volume->flags |= DOSFS_VOLUME_FLAG_FSINFO_DIRTY;
            
            volume->free_clscnt -= clscnt_a;
#endif /* (DOSFS_CONFIG_FSINFO_SUPPORTED == 1) */
            
            volume->next_clsno = clsno_n;

            *p_clsno_a = clsno_a;
            
            if (p_clsno_l)
            {
                *p_clsno_l = clsno_l;
            }
        }
    }
    else
    {
        if (status == F_ERR_NOMOREENTRY)
        {
            if (clsno_a != DOSFS_CLSNO_NONE)
            {
                status = dosfs_cluster_chain_destroy(volume, clsno_a, DOSFS_CLSNO_FREE);
            }
            
            if (status == F_NO_ERROR)
            {
                status = F_ERR_NOMOREENTRY;
            }
        }
#if (DOSFS_CONFIG_FSINFO_SUPPORTED == 1)
        else
        {
            volume->flags &= ~DOSFS_VOLUME_FLAG_FSINFO_VALID;
        }
#endif /* (DOSFS_CONFIG_FSINFO_SUPPORTED == 1) */
    }

    return status;
}

#if (DOSFS_CONFIG_SEQUENTIAL_SUPPORTED == 1)

static int dosfs_cluster_chain_create_sequential(dosfs_volume_t *volume, uint32_t clsno, uint32_t clscnt, uint32_t *p_clsno_a, uint32_t *p_clsno_l)
{
    int status = F_NO_ERROR;
    uint32_t clsno_a, clsno_b, clsno_t, clsno_n, clsno_l, clsno_s, clscnt_a, clsdata;

    clsno_b = volume->base_clsno;
    clsno_t = volume->limit_clsno;
    clsno_n = volume->free_clsno;

    clsno_a = DOSFS_CLSNO_NONE;
    clsno_l = DOSFS_CLSNO_NONE;

    clscnt_a = 0;

    do
    {
        if (clsno_n == clsno_t)
        {
            do
            {
                if (clsno_b == volume->start_clsno)
                {
                    clsno_t = volume->end_clsno;
                    clsno_b = clsno_t - (volume->au_size >> volume->cls_blk_shift);
                }
                else
                {
                    clsno_t = clsno_b;
                    clsno_b = clsno_t - (volume->au_size >> volume->cls_blk_shift);
                }

                if (clsno_b == volume->base_clsno)
                {
                    status = F_ERR_NOMOREENTRY;
                }
                else
                {
                    clsno_n = clsno_t;

                    do 
                    {
                        clsno_s = clsno_n -1;

                        status = dosfs_cluster_read_uncached(volume, clsno_s, &clsdata);

                        if (status == F_NO_ERROR)
                        {
                            if (clsdata == DOSFS_CLSNO_FREE)
                            {
                                clsno_n = clsno_s;
                            }
                        }
                    }
                    while ((status == F_NO_ERROR) && (clsdata == DOSFS_CLSNO_FREE) && (clsno_n != clsno_b));
                }
            }
            while ((status == F_NO_ERROR) && (clsno_n == clsno_t));
        }

        if (status == F_NO_ERROR)
        {
            status = dosfs_cluster_write(volume, clsno_n, DOSFS_CLSNO_END_OF_CHAIN, TRUE);
                
            if (status == F_NO_ERROR)
            {
                if (clsno_a == DOSFS_CLSNO_NONE)
                {
                    clsno_a = clsno_n;
                }
                    
                if (clsno_l != DOSFS_CLSNO_NONE)
                {
                    status = dosfs_cluster_write(volume, clsno_l, clsno_n, TRUE);
                }
                
                clsno_l = clsno_n;
                clscnt_a++;

                clsno_n++;
            }
        }
    }
    while ((status == F_NO_ERROR) && (clscnt != clscnt_a));

    if (status == F_NO_ERROR)
    {
        if (clsno != DOSFS_CLSNO_NONE)
        {
            status = dosfs_cluster_write(volume, clsno, clsno_a, TRUE);
        }
        
        if (status == F_NO_ERROR)
        {
#if (DOSFS_CONFIG_FSINFO_SUPPORTED == 1)
            volume->flags |= DOSFS_VOLUME_FLAG_FSINFO_DIRTY;
            
            volume->free_clscnt -= clscnt_a;
#endif /* (DOSFS_CONFIG_FSINFO_SUPPORTED == 1) */
            
            volume->base_clsno = clsno_b;
            volume->limit_clsno = clsno_t;
            volume->free_clsno = clsno_n;

            *p_clsno_a = clsno_a;
            
            if (p_clsno_l)
            {
                *p_clsno_l = clsno_l;
            }
        }
    }
    else
    {
        if (status == F_ERR_NOMOREENTRY)
        {
            if (clsno_a != DOSFS_CLSNO_NONE)
            {
                status = dosfs_cluster_chain_destroy(volume, clsno_a, DOSFS_CLSNO_FREE);
            }

            if ((status == F_NO_ERROR) || (status == F_ERR_NOMOREENTRY))
            {
                status = dosfs_cluster_chain_create(volume, clsno, clscnt, p_clsno_a, p_clsno_l);
            }
        }
#if (DOSFS_CONFIG_FSINFO_SUPPORTED == 1)
        else
        {
            volume->flags &= ~DOSFS_VOLUME_FLAG_FSINFO_VALID;
        }
#endif /* (DOSFS_CONFIG_FSINFO_SUPPORTED == 1) */
    }

    return status;
}

#endif /* (DOSFS_CONFIG_SEQUENTIAL_SUPPORTED == 1) */


#if (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1)

/* A sequential cluster chain is created by first finding a free sequentual, and then hooking it up back to front. 
 * Hence there is no inconsistent file system state.
 */

static int dosfs_cluster_chain_create_contiguous(dosfs_volume_t *volume, uint32_t clscnt, uint32_t *p_clsno_a)
{
    int status = F_NO_ERROR;
    uint32_t clsno_a, clsno_n, clscnt_a, clsdata;

    clsno_a = volume->end_clsno;
    clscnt_a = 0;

    do
    {
        clsno_a--;

        /* Bypass cluster cache on read while searching.
         */
        status = dosfs_cluster_read_uncached(volume, clsno_a, &clsdata);
        
        if (status == F_NO_ERROR)
        {
            if (clsdata == DOSFS_CLSNO_FREE)
            {
                clscnt_a++;
            }
            else
            {
                clsno_a = (((((clsno_a << volume->cls_blk_shift) + volume->cls_blk_offset) / volume->au_size) * volume->au_size) - volume->cls_blk_offset) >> volume->cls_blk_shift;
            }
        }
    }
    while ((status == F_NO_ERROR) && (clscnt != clscnt_a) && (clsno_a != volume->start_clsno));

    if (status == F_NO_ERROR)
    {
        if (clscnt == clscnt_a)
        {
            clsno_n = clsno_a + clscnt_a;
            clsdata = DOSFS_CLSNO_END_OF_CHAIN;

            do
            {
                clsno_n--;

                status = dosfs_cluster_write(volume, clsno_n, clsdata, FALSE);

                clsdata = clsno_n;
            }
            while ((status == F_NO_ERROR) && (clsno_n != clsno_a));

#if (DOSFS_CONFIG_FSINFO_SUPPORTED == 1)
            if (status == F_NO_ERROR)
            {
                volume->flags |= DOSFS_VOLUME_FLAG_FSINFO_DIRTY;

                volume->free_clscnt -= clscnt_a;
            }
            else
            {
                volume->flags &= ~DOSFS_VOLUME_FLAG_FSINFO_VALID;
            }
#endif /* (DOSFS_CONFIG_FSINFO_SUPPORTED == 1) */
        }
        else
        {
            status = F_ERR_NOMOREENTRY;
        }
    }

    *p_clsno_a = clsno_a;

    return status;
}

#endif /* (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1) */


/* The chain destroy process walks along the chain and frees each entry front to back.
 * Hence it's possible to have a lost chain, but no file system corruption.
 */

static int dosfs_cluster_chain_destroy(dosfs_volume_t *volume, uint32_t clsno, uint32_t clsdata)
{
    int status = F_NO_ERROR;
    dosfs_device_t *device;
    uint32_t clsno_n;
    uint32_t clsno_s, clsno_e;

    device = DOSFS_VOLUME_DEVICE(volume);

    clsno_s = DOSFS_CLSNO_NONE;
    clsno_e = DOSFS_CLSNO_NONE;

    do
    {
        /* Bypass cluster cache on read while destroying.
         */
        status = dosfs_cluster_read_uncached(volume, clsno, &clsno_n);

        if (status == F_NO_ERROR)
        {
            status = dosfs_cluster_write(volume, clsno, clsdata, FALSE);

            if (status == F_NO_ERROR)
            {
                if (clsdata == DOSFS_CLSNO_FREE)
                {
                    if (clsno_s != DOSFS_CLSNO_NONE)
                    {
                        if ((clsno_e + 1) == clsno)
                        {
                            clsno_e = clsno;
                        }
                        else
                        {
                            (*device->interface->discard)(device->context, (volume->cls_blk_offset + (clsno_s << volume->cls_blk_shift)), ((clsno_e - clsno_s + 1) << volume->cls_blk_shift));

                            clsno_s = clsno;
                            clsno_e = clsno;
                        }
                    }
                    else
                    {
                        clsno_s = clsno;
                        clsno_e = clsno;
                    }

#if (DOSFS_CONFIG_FSINFO_SUPPORTED == 1)
                    volume->free_clscnt++;
#endif /* (DOSFS_CONFIG_FSINFO_SUPPORTED == 1) */
                }

                if ((clsno_n >= 2) && (clsno_n <= volume->last_clsno))
                {
                    clsno = clsno_n;
                    clsdata = DOSFS_CLSNO_FREE;
                }
                else
                {
                    if (clsno_n < DOSFS_CLSNO_LAST)
                    {
                        status = F_ERR_EOF;
                    }
                }
            }
        }
    }
    while ((status == F_NO_ERROR) && (clsno_n < DOSFS_CLSNO_LAST));

    if (clsno_s != DOSFS_CLSNO_NONE)
    {
        (*device->interface->discard)(device->context, (volume->cls_blk_offset + (clsno_s << volume->cls_blk_shift)), ((clsno_e - clsno_s + 1) << volume->cls_blk_shift));
    }

#if (DOSFS_CONFIG_FSINFO_SUPPORTED == 1)
    if (status == F_NO_ERROR)
    {
        volume->flags |= DOSFS_VOLUME_FLAG_FSINFO_DIRTY;
    }
    else
    {
        volume->flags &= ~DOSFS_VOLUME_FLAG_FSINFO_VALID;
    }
#endif /* (DOSFS_CONFIG_FSINFO_SUPPORTED == 1) */

    return status;
}

/***********************************************************************************************************************/

static inline unsigned int dosfs_name_ascii_upcase(unsigned int cc)
{
    unsigned int uc;

    if ((cc <= 0x1f) || (cc > 0x7f))
    {
        uc = 0x0000;
    }
    else
    {
        if ((cc >= 'a') && (cc <= 'z'))
        {
            uc = cc - ('a' - 'A');
        }
        else
        {
            uc = cc;
        }
    }

    return uc;
}

#if (DOSFS_CONFIG_VFAT_SUPPORTED == 0)

static const char *dosfs_name_cstring_to_dosname(const char *cstring, uint8_t *dosname)
{
    unsigned int cc, n, n_e;

    memset(dosname, ' ', 11);

    n = 0;
    n_e = 8;

    cc = *cstring++;

    if (cc == ' ')
    {
        cstring = NULL;
    }
    else
    {
        /* A leading sequence of "." or ".." is handled varbatim.
         */
    
        if (cc == '.')
        {
            dosname[n++] = cc;

            cc = *cstring++;

            if (cc == '.')
            {
                dosname[n++] = cc;

                cc = *cstring++;
            }

            if ((cc != '/') && (cc != '\\') && (cc != '\0'))
            {
                cstring = NULL;
            }
        }

        while ((cstring != NULL) && (cc != '/') && (cc != '\\') && (cc != '\0'))
        {
            if (cc == '.')
            {
                if (n_e == 11)
                {
                    cstring = NULL;
                }
                else
                {
                    n = 8;
                    n_e = 11;
                        
                    cc = *cstring++;
                }
            }
            else
            {
                if (n == n_e)
                {
                    cstring = NULL;
                }
                else
                {
                    if ((cc <= 0x001f) ||
                        (cc >= 0x0080) ||
                        (cc == '"') ||
                        (cc == '*') ||
                        (cc == '+') ||
                        (cc == ',') ||
                        (cc == '/') ||
                        (cc == ':') ||
                        (cc == ';') ||
                        (cc == '<') ||
                        (cc == '=') ||
                        (cc == '>') ||
                        (cc == '?') ||
                        (cc == '[') ||
                        (cc == '\\') ||
                        (cc == ']') ||
                        (cc == '|'))
                    {
                        cstring = NULL;
                    }
                    else
                    {
                        dosname[n++] = dosfs_name_ascii_upcase(cc);
                            
                        cc = *cstring++;
                    }
                }
            }
        }

        if (n == 0)
        {
            cstring = NULL;
        }
    }

    return cstring;
}

static const char *dosfs_name_cstring_to_pattern(const char *cstring, char *pattern)
{
    unsigned int cc, n, n_e;

    memset(pattern, ' ', 11);

    n = 0;
    n_e = 8;

    cc = *cstring++;

    if (cc == ' ')
    {
        cstring = NULL;
    }
    else
    {
        /* A leading sequence of "." or ".." is handled varbatim.
         */
    
        if (cc == '.')
        {
            pattern[n++] = cc;

            cc = *cstring++;

            if (cc == '.')
            {
                pattern[n++] = cc;

                cc = *cstring++;
            }
        }

        while ((cstring != NULL) && (cc != '\0'))
        {
            if (cc == '.')
            {
                if (n_e == 11)
                {
                    cstring = NULL;
                }
                else
                {
                    n = 8;
                    n_e = 11;
                    
                    cc = *cstring++;
                }
            }
            else
            {
                if (n == n_e)
                {
                    cstring = NULL;
                }
                else
                {
                    if (cc == '*')
                    {
                        while (n != n_e)
                        {
                            pattern[n++] = '?';
                        }

                        cc = *cstring++;
                    }
                    else
                    {
                        if ((cc <= 0x001f) ||
                            (cc >= 0x0080) ||
                            (cc == '"') ||
                            (cc == '+') ||
                            (cc == ',') ||
                            (cc == '/') ||
                            (cc == ':') ||
                            (cc == ';') ||
                            (cc == '<') ||
                            (cc == '=') ||
                            (cc == '>') ||
                            (cc == '[') ||
                            (cc == '\\') ||
                            (cc == ']') ||
                            (cc == '|'))
                        {
                            cstring = NULL;
                        }
                        else
                        {
                            pattern[n++] = dosfs_name_ascii_upcase(cc);
                            
                            cc = *cstring++;
                        }
                    }
                }
            }
        }
    }

    return cstring;
}

#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */

static const char *dosfs_name_cstring_to_label(const char *cstring, uint8_t *label)
{
    unsigned int cc, n, n_e;

    memset(label, ' ', 11);

    n = 0;
    n_e = 11;

    cc = *cstring++;

    if (cc == ' ')
    {
        cstring = NULL;
    }
    else
    {
        do
        {
            if (n == n_e)
            {
                cstring = NULL;
            }
            else
            {
                if ((cc <= 0x001f) ||
                    (cc >= 0x0080) ||
                    (cc == '"') ||
                    (cc == '*') ||
                    (cc == '+') ||
                    (cc == ',') ||
                    (cc == '.') ||
                    (cc == '/') ||
                    (cc == ':') ||
                    (cc == ';') ||
                    (cc == '<') ||
                    (cc == '=') ||
                    (cc == '>') ||
                    (cc == '?') ||
                    (cc == '[') ||
                    (cc == '\\') ||
                    (cc == ']') ||
                    (cc == '|'))
                {
                    cstring = NULL;
                }
                else
                {
                    label[n++] = dosfs_name_ascii_upcase(cc);
                            
                    cc = *cstring++;
                }
            }
        }
        while ((cstring != NULL) && (cc != '\0'));
    }
    
    return cstring;
}

static char *dosfs_name_dosname_to_cstring(const uint8_t *dosname, unsigned int doscase, char *cstring, char *cstring_e)
{
    unsigned int i, n;

    for (n = 7; n != 0; n--)
    {
        if (dosname[n] != ' ')
        {
            break;
        }
    }

    if (cstring + (n +1) <= cstring_e)
    {
        for (i = 0; i < (n +1); i++)
        {
            if ((dosname[i] <= 0x1f) || (dosname[i] >= 0x80))
            {
                *cstring++ = '_';
            }
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
            else if ((doscase & DOSFS_DIR_TYPE_LCASE_NAME) && (dosname[i] >= 'A') && (dosname[i] <= 'Z'))
            {
                *cstring++ = (dosname[i] + ('a' - 'A'));
            }
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
            else
            {
                *cstring++ = dosname[i];
            }
        }

        for (n = 10; n != 7; n--)
        {
            if (dosname[n] != ' ')
            {
                break;
            }
        }

        if (n != 7)
        {
            if (cstring < cstring_e)
            {
                *cstring++ = '.';
                
                if (cstring + ((n -8) +1) <= cstring_e)
                {
                    for (i = 8; i < (n +1); i++)
                    {
                        if ((dosname[i] <= 0x1f) || (dosname[i] >= 0x80))
                        {
                            *cstring++ = '_';
                        }
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                        else if ((doscase & DOSFS_DIR_TYPE_LCASE_EXT) && (dosname[i] >= 'A') && (dosname[i] <= 'Z'))
                        {
                            *cstring++ = (dosname[i] + ('a' - 'A'));
                        }
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                        else
                        {
                            *cstring++ = dosname[i];
                        }
                    }
                }
            }
        }

        if (cstring != NULL)
        {
            if (cstring < cstring_e)
            {
                *cstring++ = '\0';
            }
            else
            {
                cstring = NULL;
            }
        }
    }
    else
    {
        cstring = NULL;
    }

    return cstring;
}

static char *dosfs_name_label_to_cstring(const uint8_t *label, char *cstring, char *cstring_e)
{
    unsigned int i, n;

    for (n = 10; n != 0; n--)
    {
        if (label[n] != ' ')
        {
            break;
        }
    }

    if (cstring + (n +1) <= cstring_e)
    {
        for (i = 0; i < (n +1); i++)
        {
            if ((label[i] <= 0x1f) || (label[i] >= 0x80))
            {
                *cstring++ = '_';
            }
            else
            {
                *cstring++ = label[i];
            }
        }

        if (cstring < cstring_e)
        {
            *cstring++ = '\0';
        }
        else
        {
            cstring = NULL;
        }
    }
    else
    {
        cstring = NULL;
    }

    return cstring;
}

#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)

#if (DOSFS_CONFIG_UTF8_SUPPORTED == 1)

static inline unsigned int dosfs_name_cstring_to_unicode(const char *cstring, const char **p_cstring)
{
    unsigned int cc, c;
    const uint8_t *utf8;

    utf8 = (const uint8_t*)cstring;

    c = *utf8++;

    if (c <= 0x7f)                /* 0XXX XXXX one byte    */
    {
        cc = c;
    }
    else if ((c & 0xe0) == 0xc0)  /* 110X XXXX two bytes   */
    {
        cc = c & 31;

        c = *utf8++;

        if ((c & 0xc0) == 0x80)
        {
            cc = (cc << 6) | (c & 31);
        }
        else
        {
            cc = 0x001f; /* error */
        }
    }
    else if ((c & 0xf0) == 0xe0)  /* 1110 XXXX three bytes */
    {
        cc = c & 15;

        c = *utf8++;

        if ((c & 0xc0) == 0x80)
        {
            cc = (cc << 6) | (c & 31);

            c = *utf8++;

            if ((c & 0xc0) == 0x80)
            {
                cc = (cc << 6) | (c & 31);
            }
            else
            {
                cc = 0x001f; /* error */
            }
        }
        else
        {
            cc = 0x001f; /* error */
        }
    }
    else
    {
        cc = 0x001f;
    }

    if (p_cstring)
    {
        *p_cstring = (const char*)utf8;
    }

    return cc;
}

static inline char * dosfs_name_unicode_to_cstring(unsigned int cc, char *cstring, char *cstring_e)
{
    uint8_t *utf8;

    utf8 = (uint8_t*)cstring;

    if (cc <= 0x007f)      /* 0XXX XXXX one byte */
    {
        if ((utf8 + 1) >= (uint8_t*)cstring_e)
        {
            utf8 = NULL;
        }
        else
        {
            *utf8++ = cc;
        }
    }
    else if (cc <= 0x07ff)  /* 110X XXXX two bytes */
    {
        if ((utf8 + 2) >= (uint8_t*)cstring_e)
        {
            utf8 = NULL;
        }
        else
        {
            *utf8++ = 0xc0 | (cc >> 6);
            *utf8++ = 0x80 | (cc & 31);
        }
    }
    else                   /* 1110 XXXX three bytes */
    {
        if ((utf8 + 3) >= (uint8_t*)cstring_e)
        {
            utf8 = NULL;
        }
        else
        {
            *utf8++ = 0xe0 | (cc >> 12);
            *utf8++ = 0x80 | ((cc >> 6) & 31);
            *utf8++ = 0x80 | (cc & 31);
        }
    }

    return (char*)utf8;
}

static const uint8_t dosfs_name_unicode_upcase_index_table[2048] = {
    0, 0, 0, 1, 0, 0, 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 
    3, 11, 12, 13, 14, 0, 0, 0, 0, 0, 0, 15, 0, 16, 17, 18, 
    0, 19, 20, 3, 21, 3, 22, 3, 23, 0, 0, 24, 25, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 26, 0, 0, 0, 0, 
    3, 3, 3, 3, 27, 3, 3, 28, 29, 30, 31, 32, 30, 33, 34, 35, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 36, 37, 38, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 39, 40, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 41, 42, 43, 3, 3, 3, 44, 45, 46, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 
};

static const int16_t dosfs_name_unicode_upcase_delta_table[1504] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, 
    -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, 0, 0, 0, 0, 0, 
    -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, 
    -32, -32, -32, -32, -32, -32, -32, 0, -32, -32, -32, -32, -32, -32, -32, 121, 
    0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 
    0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 
    0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 
    0, 0, 0, -1, 0, -1, 0, -1, 0, 0, -1, 0, -1, 0, -1, 0, 
    -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, 0, -1, 0, -1, 0, -1, 
    0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 
    0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 
    0, -1, 0, -1, 0, -1, 0, -1, 0, 0, -1, 0, -1, 0, -1, 0, 
    195, 0, 0, -1, 0, -1, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 
    0, 0, -1, 0, 0, 97, 0, 0, 0, -1, 163, 0, 0, 0, 130, 0, 
    0, -1, 0, -1, 0, -1, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 
    -1, 0, 0, 0, -1, 0, -1, 0, 0, -1, 0, 0, 0, -1, 0, 56, 
    0, 0, 0, 0, 0, 0, -2, 0, 0, -2, 0, 0, -2, 0, -1, 0, 
    -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, -79, 0, -1, 
    0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 
    0, 0, 0, -2, 0, -1, 0, 0, 0, -1, 0, -1, 0, -1, 0, -1, 
    0, 0, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 
    0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 10795, 0, -1, 0, 10792, 0, 
    0, 0, -1, 0, 0, 0, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 
    0, 0, 0, -210, -206, 0, -205, -205, 0, -202, 0, -203, 0, 0, 0, 0, 
    -205, 0, 0, -207, 0, 0, 0, 0, -209, -211, 0, 10743, 0, 0, 0, -211, 
    0, 0, -213, 0, 0, -214, 0, 0, 0, 0, 0, 0, 0, 10727, 0, 0, 
    -218, 0, 0, -218, 0, 0, 0, 0, -218, -69, -217, -217, -71, 0, 0, 0, 
    0, 0, -219, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 130, 130, 130, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -38, -37, -37, -37, 
    0, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, 
    -32, -32, -31, -32, -32, -32, -32, -32, -32, -32, -32, -32, -64, -63, -63, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, -1, 0, -1, 0, -1, 
    0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 
    0, 0, 7, 0, 0, 0, 0, 0, -1, 0, 0, -1, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, 
    -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, -32, 
    -80, -80, -80, -80, -80, -80, -80, -80, -80, -80, -80, -80, -80, -80, -80, -80, 
    0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, -1, 0, -1, 
    0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 
    0, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, -15, 
    0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 
    0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 
    0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, 
    -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, 
    -48, -48, -48, -48, -48, -48, -48, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3814, 0, 0, 
    0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 
    0, -1, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 
    0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 
    8, 8, 8, 8, 8, 8, 8, 8, 0, 0, 0, 0, 0, 0, 0, 0, 
    8, 8, 8, 8, 8, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    8, 8, 8, 8, 8, 8, 8, 8, 0, 0, 0, 0, 0, 0, 0, 0, 
    8, 8, 8, 8, 8, 8, 8, 8, 0, 0, 0, 0, 0, 0, 0, 0, 
    8, 8, 8, 8, 8, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 8, 0, 8, 0, 8, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 
    8, 8, 8, 8, 8, 8, 8, 8, 0, 0, 0, 0, 0, 0, 0, 0, 
    74, 74, 86, 86, 86, 86, 100, 100, 128, 128, 112, 112, 126, 126, 0, 0, 
    8, 8, 8, 8, 8, 8, 8, 8, 0, 0, 0, 0, 0, 0, 0, 0, 
    8, 8, 0, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -9, 0, 0, 0, 
    8, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    8, 8, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -9, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -28, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, 
    0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    -26, -26, -26, -26, -26, -26, -26, -26, -26, -26, -26, -26, -26, -26, -26, -26, 
    -26, -26, -26, -26, -26, -26, -26, -26, -26, -26, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, 
    -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, 
    -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, -48, 0, 
    0, -1, 0, 0, 0, 0, 0, 0, -1, 0, -1, 0, -1, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    -7264, -7264, -7264, -7264, -7264, -7264, -7264, -7264, -7264, -7264, -7264, -7264, -7264, -7264, -7264, -7264, 
    -7264, -7264, -7264, -7264, -7264, -7264, -7264, -7264, -7264, -7264, -7264, -7264, -7264, -7264, -7264, -7264, 
    -7264, -7264, -7264, -7264, -7264, -7264, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
};

static inline unsigned int dosfs_name_unicode_upcase(unsigned int cc)
{
    return ((cc & 0xffff) + dosfs_name_unicode_upcase_delta_table[(dosfs_name_unicode_upcase_index_table[(cc & 0xffff) >> 5] << 5) + (cc & 31)]);
}

#endif /* (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */

static const uint8_t dosfs_name_nibble_to_char_table[16] = "0123456789ABCDEF";

static const char *dosfs_name_cstring_to_uniname(const char *cstring, dosfs_unicode_t *uniname, uint8_t *p_unicount, uint8_t *dosname, uint8_t *p_doscase)
{
    unsigned int cc, i, n, n_e, doscase;

    doscase = 0;

    memset(dosname, ' ', 11);

    n = 0;
    n_e = 8;
    i = 0;

#if (DOSFS_CONFIG_UTF8_SUPPORTED == 1)
    cc = dosfs_name_cstring_to_unicode(cstring, &cstring);
#else /* (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */
    cc = *cstring++;
#endif /* (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */

    if (cc == ' ')
    {
        cstring = NULL;
    }
    else
    {
        /* A leading sequence of "." and ".." is handled varbatim.
         */
    
        if (cc == '.')
        {
            dosname[n++] = cc;
            uniname[i++] = cc;

#if (DOSFS_CONFIG_UTF8_SUPPORTED == 1)
            cc = dosfs_name_cstring_to_unicode(cstring, &cstring);
#else /* (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */
            cc = *cstring++;
#endif /* (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */

            if (cc == '.')
            {
                dosname[n++] = cc;
                uniname[i++] = cc;
            
#if (DOSFS_CONFIG_UTF8_SUPPORTED == 1)
                cc = dosfs_name_cstring_to_unicode(cstring, &cstring);
#else /* (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */
                cc = *cstring++;
#endif /* (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */
            }

            if ((cstring != NULL) && (cc != '/') && (cc != '\\') && (cc != '\0'))
            {
                dosname[0] = '\0';
            }
        }

        while ((cstring != NULL) && (cc != '/') && (cc != '\\') && (cc != '\0'))
        {
            if (dosname[0] != '\0')
            {
                if (cc == '.')
                {
                    if (n_e == 11)
                    {
                        dosname[0] = '\0';
                    }
                    else
                    {
                        n = 8;
                        n_e = 11;
                    }
                }
                else
                {
                    if (n == n_e)
                    {
                        dosname[0] = '\0';
                    }
                    else
                    {
                        if ((cc <= 0x001f) ||
                            (cc >= 0x0080) || 
                            (cc == '"') ||
                            (cc == '*') ||
                            (cc == '+') ||
                            (cc == ',') ||
                            (cc == '.') ||
                            (cc == '/') ||
                            (cc == ':') ||
                            (cc == ';') ||
                            (cc == '<') ||
                            (cc == '=') ||
                            (cc == '>') ||
                            (cc == '?') ||
                            (cc == '[') ||
                            (cc == '\\') ||
                            (cc == ']') ||
                            (cc == '|'))
                        {
                            dosname[0] = '\0';
                        }
                        else
                        {
                            dosname[n++] = dosfs_name_ascii_upcase(cc);

                            if ((cc >= 'a') && (cc <= 'z'))
                            {
                                doscase |= ((n_e == 8) ? DOSFS_DIR_TYPE_LCASE_NAME : DOSFS_DIR_TYPE_LCASE_EXT);
                            }

                            if ((cc >= 'A') && (cc <= 'Z'))
                            {
                                doscase |= ((n_e == 8) ? DOSFS_DIR_TYPE_UCASE_NAME : DOSFS_DIR_TYPE_UCASE_EXT);
                            }
                        }
                    }
                }
            }

            if (i == 255)
            {
                cstring = NULL;
            }
            else
            {
                if ((cc <= 0x001f) ||
#if (DOSFS_CONFIG_UTF8_SUPPORTED == 0)
                    (cc >= 0x0080) || 
#endif /* (DOSFS_CONFIG_UTF8_SUPPORTED == 0) */
                    (cc == '"') ||
                    (cc == '*') ||
                    (cc == '/') ||
                    (cc == ':') ||
                    (cc == '<') ||
                    (cc == '>') ||
                    (cc == '?') ||
                    (cc == '\\') ||
                    (cc == '|'))
                {
                    cstring = NULL;
                }
                else
                {
                    uniname[i++] = cc;

#if (DOSFS_CONFIG_UTF8_SUPPORTED == 1)
                    cc = dosfs_name_cstring_to_unicode(cstring, &cstring);
#else /* (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */
                    cc = *cstring++;
#endif /* (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */
                }
            }
        }

        if (cstring != NULL)
        {
            /* Strip trailing ' ' and '.'. This will force "." and ".."
             * to be a dosname only.
             */
            while (i != 0)
            {
                if ((uniname[i -1] != ' ') && (uniname[i -1] != '.'))
                {
                    break;
                }

                i--;
            }

            if (dosname[0] != '\0')
            {
                if (((doscase & (DOSFS_DIR_TYPE_LCASE_NAME | DOSFS_DIR_TYPE_UCASE_NAME)) == (DOSFS_DIR_TYPE_LCASE_NAME | DOSFS_DIR_TYPE_UCASE_NAME)) ||
                    ((doscase & (DOSFS_DIR_TYPE_LCASE_EXT  | DOSFS_DIR_TYPE_UCASE_EXT )) == (DOSFS_DIR_TYPE_LCASE_EXT  | DOSFS_DIR_TYPE_UCASE_EXT)))
                {
                    /* Have seen both lower case and upper case characters, hence a lossy dosname mapping.
                     */

                    doscase = DOSFS_DIR_TYPE_LOSSY;
                }
                else
                {
                    /* A fully legel dosname, hence no uniname mapping.
                     */
                    doscase &= (DOSFS_DIR_TYPE_LCASE_NAME | DOSFS_DIR_TYPE_LCASE_EXT);
                }
            }
            else
            {
                /* If there is no valid dosname, there has to be a valid uniname.
                 */
                doscase = 0x00;

                if (i == 0)
                {
                    cstring = NULL;
                }
            }
        }
    }

    *p_unicount = i;
    *p_doscase = doscase;

    return cstring;
}

static char *dosfs_name_uniname_to_cstring(const dosfs_unicode_t *uniname, unsigned int unicount, char *cstring, char *cstring_e)
{
    unsigned int cc, n;

    for (n = 0; n < unicount; n++)
    {
        cc = *uniname++;

#if (DOSFS_CONFIG_UTF8_SUPPORTED == 1)
        cstring = dosfs_name_unicode_to_cstring(cc, cstring, cstring_e);

        if (cstring == NULL)
        {
            break;
        }
#else /* (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */

        if (cstring < cstring_e)
        {
            if ((cc <= 0x1f) || (cc >= 0x80))
            {
                *cstring++ = '_';
            }
            else
            {
                *cstring++ = cc;
            }
        }
        else
        {
            cstring = NULL;

            break;
        }
#endif /* (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */
    }

    if (cstring != NULL)
    {
        if (cstring < cstring_e)
        {
            *cstring++ = '\0';
        }
        else
        {
            cstring = NULL;
        }
    }

    return cstring;
}

static void dosfs_name_uniname_to_dosname(const dosfs_unicode_t *uniname, unsigned int unicount, uint8_t *dosname, unsigned int *p_dosprefix)
{
    unsigned int offset, prefix, i, n, n_e, cc;

    memset(dosname, ' ', 11);

    /* Compute the offset of the last '.' in uniname.
     */
    
    for (offset = unicount, i = 0; i < unicount; i++)
    {
        if (uniname[i] == '.')
        {
            offset = i;
        }
    }

    prefix = 0;
    n = 0;
    n_e = 8;
    i = 0;

    do
    {
        cc = uniname[i];

        if (cc == '.')
        {
            if (i <= offset)
            {
                if ((n_e == 11) || (i != offset))
                {
                    if (n != n_e)
                    {
                        dosname[n++] = '_';
                    }
                }
                else
                {
                    prefix = n;
                    n = 8;
                    n_e = 11;
                }
            }
            else
            {
                /* ignore */
            }
        }
        else
        {
            if (cc == ' ')
            {
                /* ignore */
            }
            else
            {
                if (n != n_e)
                {
                    if ((cc <= 0x001f) ||
                        (cc >= 0x0080) ||
                        (cc == '+') ||
                        (cc == ',') ||
                        (cc == ';') ||
                        (cc == '=') ||
                        (cc == '[') ||
                        (cc == ']'))
                    {
                        dosname[n++] = '_';
                    }
                    else
                    {
                        dosname[n++] = dosfs_name_ascii_upcase(cc);
                    }
                }
                else
                {
                    /* ignore */
                }
            }
        }

        i++;
    }
    while ((n != 11) && (i < unicount));

    if (n_e == 8)
    {
        prefix = n;
    }

    *p_dosprefix = prefix;
}

static uint8_t dosfs_name_checksum_dosname(const uint8_t *dosname)
{
    unsigned int n, chksum;

    for (chksum = 0, n = 0; n < 11; n++)
    {
        chksum = (((chksum >> 1) & 0x7f) | (chksum << 7)) + *dosname++;
    }

    return chksum & 0xff;
}

static uint16_t dosfs_name_checksum_uniname(const dosfs_unicode_t *uniname, unsigned int unicount)
{
    unsigned int n, chksum;

    for (chksum = 0, n = 0; n < unicount; n++)
    {
        chksum = (((chksum >> 1) & 0x7fff) | (chksum << 15)) + *uniname++;
    }

    return chksum & 0xffff;
}

#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */

/***********************************************************************************************************************/

static int dosfs_path_convert_filename(dosfs_volume_t *volume, const char *filename, const char **p_filename)
{
    int status = F_NO_ERROR;

#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1) 

    filename = dosfs_name_cstring_to_uniname(filename, volume->lfn_name, &volume->lfn_count, volume->dir.dir_name, &volume->dir.dir_nt_reserved);

    volume->dir_entries = volume->lfn_count ? ((volume->lfn_count +12) / 13) : 0;

#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */

    filename = dosfs_name_cstring_to_dosname(filename, volume->dir.dir_name);

    volume->dir.dir_nt_reserved = 0;

#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */

    if (filename != NULL)
    {
        if (p_filename)
        {
            *p_filename = filename;
        }
    }
    else
    {
        status = F_ERR_INVALIDNAME;
    }

    return status;
}

#if (DOSFS_CONFIG_VFAT_SUPPORTED == 0)

static int dosfs_path_find_callback_empty(dosfs_volume_t *volume, void *private, dosfs_dir_t *dir)
{
    return TRUE;
}

static int dosfs_path_find_callback_volume(dosfs_volume_t *volume, void *private, dosfs_dir_t *dir)
{
    return (dir->dir_attr & DOSFS_DIR_ATTR_VOLUME_ID);
}

static int dosfs_path_find_callback_directory(dosfs_volume_t *volume, void *private, dosfs_dir_t *dir)
{
    if (!(dir->dir_attr & DOSFS_DIR_ATTR_DIRECTORY))
    {
        return FALSE;
    }
    else
    {
        uint32_t clsno, clsno_d;

        clsno = (uint32_t)private;

        if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
        {
            clsno_d = ((uint32_t)DOSFS_FTOHS(dir->dir_clsno_hi) << 16) | (uint32_t)DOSFS_FTOHS(dir->dir_clsno_lo);
        }
        else
        {
            clsno_d = (uint32_t)DOSFS_FTOHS(dir->dir_clsno_lo);
        }

        return (clsno == clsno_d);
    }
}

static int dosfs_path_find_callback_name(dosfs_volume_t *volume, void *private, dosfs_dir_t *dir)
{
    return (!(dir->dir_attr & DOSFS_DIR_ATTR_VOLUME_ID) && !memcmp(dir->dir_name, volume->dir.dir_name, sizeof(dir->dir_name)));
}

static int dosfs_path_find_callback_pattern(dosfs_volume_t *volume, void *private, dosfs_dir_t *dir)
{
    const uint8_t *pattern = (const uint8_t*)private;
    int match;
    unsigned int n;

    if (!(dir->dir_attr & DOSFS_DIR_ATTR_VOLUME_ID))
    {
        match = TRUE;

        if (pattern != NULL)
        {
            for (n = 0; n < 11; n++)
            {
                if (!((pattern[n] == '?') || (dir->dir_name[n] == pattern[n])))
                {
                    match = FALSE;
                    
                    break;
                }
            }
        }
    }
    else
    {
        match = FALSE;
    }

    return match;
}

#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 0) */

static void dosfs_path_convert_ldir_entry(dosfs_volume_t *volume, dosfs_dir_t *dir, unsigned int sequence)
{
    unsigned int s, i, offset, cc;

    if (sequence & DOSFS_LDIR_SEQUENCE_FIRST)
    {
        volume->dir.dir_name[0] = '\0';
        volume->dir_entries = (sequence & DOSFS_LDIR_SEQUENCE_INDEX);
        volume->lfn_count = (sequence & DOSFS_LDIR_SEQUENCE_INDEX) * 13;
    }

    s = 0;
    i = ((sequence & DOSFS_LDIR_SEQUENCE_INDEX) -1) * 13;

    do
    {
        offset = dosfs_path_ldir_name_table[s++];
        
        cc = (((uint8_t*)dir)[offset +1] << 8) | ((uint8_t*)dir)[offset +0];
        
        if (cc == 0x0000)
        {
            volume->lfn_count = i;
        }
        else if (i < 255)
        {
#if (DOSFS_CONFIG_UTF8_SUPPORTED == 0)
            /* If UTF8 is not supported, then the name space is ASCII. In
             * that case remap illegal values to 0x001f.
             */
            if (cc >= 0x0080)
            {
                volume->lfn_name[i++] = 0x001f;
            }
            else
#endif /* (DOSFS_CONFIG_UTF8_SUPPORTED == 0) */
            {
                volume->lfn_name[i++] = cc;
            }
        }
    }
    while ((s < 13) && (cc != 0x0000));
}


static int dosfs_path_find_callback_empty(dosfs_volume_t *volume, void *private, dosfs_dir_t *dir, unsigned int sequence)
{
    return TRUE;
}

static int dosfs_path_find_callback_volume(dosfs_volume_t *volume, void *private, dosfs_dir_t *dir, unsigned int sequence)
{
    int match;

    if (sequence & DOSFS_LDIR_SEQUENCE_INDEX)
    {
        match = FALSE;
    }
    else
    {
        if (dir->dir_attr & DOSFS_DIR_ATTR_VOLUME_ID)
        {
            match = (sequence != DOSFS_LDIR_SEQUENCE_LAST);
        }
        else
        {
            match = FALSE;
        }
    }

    return match;
}

static int dosfs_path_find_callback_directory(dosfs_volume_t *volume, void *private, dosfs_dir_t *dir, unsigned int sequence)
{
    uint32_t clsno, clsno_d;
    int match;

    if (sequence & DOSFS_LDIR_SEQUENCE_INDEX)
    {
        dosfs_path_convert_ldir_entry(volume, dir, sequence);

        match = TRUE;
    }
    else
    {
        if (dir->dir_attr & DOSFS_DIR_ATTR_DIRECTORY)
        {
            clsno = (uint32_t)private;

            if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
            {
                clsno_d = ((uint32_t)DOSFS_FTOHS(dir->dir_clsno_hi) << 16) | (uint32_t)DOSFS_FTOHS(dir->dir_clsno_lo);
            }
            else
            {
                clsno_d = (uint32_t)DOSFS_FTOHS(dir->dir_clsno_lo);
            }

            if (clsno_d == clsno)
            {
                if (sequence != DOSFS_LDIR_SEQUENCE_LAST)
                {
                    volume->dir_entries = 0;
                    volume->lfn_count = 0;
                }

                match = TRUE;
            }
            else
            {
                match = FALSE;
            }
        }
        else
        {
            match = FALSE;
        }
    }

    return match;
}

static int dosfs_path_find_callback_name(dosfs_volume_t *volume, void *private, dosfs_dir_t *dir, unsigned int sequence)
{
    unsigned int offset, i, s, cc, lc;
    int match;

    if (sequence & DOSFS_LDIR_SEQUENCE_INDEX)
    {
        match = TRUE;

        if ((sequence & DOSFS_LDIR_SEQUENCE_FIRST) && ((sequence & DOSFS_LDIR_SEQUENCE_INDEX) != volume->dir_entries))
        {
            match = FALSE;
        }
        else
        {
            s = 0;
            i = ((sequence & DOSFS_LDIR_SEQUENCE_INDEX) -1) * 13;
                
            do
            {
                offset = dosfs_path_ldir_name_table[s++];
                    
                cc = (((uint8_t*)dir)[offset +1] << 8) | ((uint8_t*)dir)[offset +0];
                    
                if (i < volume->lfn_count)
                {
                    lc = volume->lfn_name[i];

#if (DOSFS_CONFIG_UTF8_SUPPORTED == 1)
                    cc = dosfs_name_unicode_upcase(cc);
                    lc = dosfs_name_unicode_upcase(lc);
#else /* (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */
                    /* That's a tad non-intuitive here. "volume->lfn_name[]" does contain
                     * only ASCII characters. Hence only the ASCII portion of the UNICODE
                     * name space has to be upcased. Anything outside needs properly mismatch.
                     */
                    if (cc < 0x80)
                    {
                        cc = dosfs_name_ascii_upcase(cc);
                    }
                    lc = dosfs_name_ascii_upcase(lc);
#endif /* (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */
                  
                    if (lc != cc)
                    {
                        match = FALSE;
                    }
                }
                else 
                {
                    if (i == volume->lfn_count)
                    {
                        if (cc != 0x0000)
                        {
                            match = FALSE;
                        }
                    }
                    else
                    {
                        if (cc != 0xffff)
                        {
                            match = FALSE;
                        }
                    }
                }

                i++;
            }
            while (match && (s < 13));
        }
    }
    else
    {
        if (!(dir->dir_attr & DOSFS_DIR_ATTR_VOLUME_ID))
        {
            if (sequence != DOSFS_LDIR_SEQUENCE_LAST)
            {
                if ((volume->dir.dir_name[0] != '\0') && !memcmp(dir->dir_name, volume->dir.dir_name, sizeof(dir->dir_name)))
                {
                    /* Set volume->dir_entries to 0 on a SFN match.
                     */
                    volume->dir_entries = 0;
                    
                    match = TRUE;
                }
                else
                {
                    match = FALSE;
                }
            }
            else
            {
                /* A LFN name had been checked already while looking at the LDIR entries.
                 * If we get here it's a match, otherwise the code would not be called.
                 */
                match = TRUE;
            }
        }
        else
        {
            match = FALSE;
        }
    }

    return match;
}

static int dosfs_path_find_callback_unique(dosfs_volume_t *volume, void *private, dosfs_dir_t *dir, unsigned int sequence)
{
    unsigned int cc, index;
    unsigned int *p_mask;
    int match;
    
    if (sequence & DOSFS_LDIR_SEQUENCE_INDEX)
    {
        match = TRUE;
    }
    else
    {
        if (!(dir->dir_attr & DOSFS_DIR_ATTR_VOLUME_ID))
        {
            p_mask = (unsigned int*)private;

            if (p_mask != NULL)
            {
                /* The pivot index is stored in the upper 4 bits, while the
                 * pivot result is store in the lower 9 bit. After the 
                 * unique search, the lower 9 bit contain which similar
                 * names with '1' to '9' at the privot index were encountered.
                 */

                index = (*p_mask) >> 9;

                cc = dir->dir_name[index];

                if ((cc >= '1') && (cc <= '9'))
                {
                    volume->dir.dir_name[index] = cc;
                    
                    if (!memcmp(dir->dir_name, volume->dir.dir_name, sizeof(dir->dir_name)))
                    {
                        *p_mask |= (1 << (cc - '1'));
                    }
                }

                match = FALSE;
            }
            else
            {
                match = !memcmp(dir->dir_name, volume->dir.dir_name, sizeof(dir->dir_name));
            }
        }

        match = FALSE;
    }

    return match;
}

static int dosfs_path_find_callback_pattern(dosfs_volume_t *volume, void *private, dosfs_dir_t *dir, unsigned int sequence)
{
    const char *pattern = (const char*)private;
    unsigned int offset, i, n, n_e, cc, lc;
    int match, skip;

    if (sequence & DOSFS_LDIR_SEQUENCE_INDEX)
    {
        dosfs_path_convert_ldir_entry(volume, dir, sequence);

        match = TRUE;
    }
    else
    {
        if (!(dir->dir_attr & DOSFS_DIR_ATTR_VOLUME_ID))
        {
            if (sequence != DOSFS_LDIR_SEQUENCE_LAST)
            {
                /* A SFN is converted into a LFN before pattern matching.
                 * Otherwise, the matching would be inconsistent.
                 */

                i = 0;

                for (n_e = 7; n_e != 0; n_e--)
                {
                    if (dir->dir_name[n_e] != ' ')
                    {
                        break;
                    }
                }
                
                for (n = 0; n <= n_e; n++)
                {
                    /* Matching is always done case insensitive. However when the name is converted
                     * back into a cstring, it needs to be case sensitive.
                     */
                    if ((dir->dir_nt_reserved & DOSFS_DIR_TYPE_LCASE_NAME) && (dir->dir_name[n] >= 'A') && (dir->dir_name[n] <= 'Z'))
                    {
                        volume->lfn_name[i++] = (dir->dir_name[n] + ('a' - 'A'));
                    }
                    else
                    {
                        volume->lfn_name[i++] = dir->dir_name[n];
                    }
                }

                offset = i;

                for (n_e = 10; n_e != 7; n_e--)
                {
                    if (dir->dir_name[n_e] != ' ')
                    {
                        break;
                    }
                }
                
                if (n_e != 7)
                {
                    volume->lfn_name[i++] = '.';

                    for (n = 8; n <= n_e; n++)
                    {
                        /* Matching is always done case insensitive. However when the name is converted
                         * back into a cstring, it needs to be case sensitive.
                         */
                        if ((dir->dir_nt_reserved & DOSFS_DIR_TYPE_LCASE_EXT) && (dir->dir_name[n] >= 'A') && (dir->dir_name[n] <= 'Z'))
                        {
                            volume->lfn_name[i++] = (dir->dir_name[n] + ('a' - 'A'));
                        }
                        else
                        {
                            volume->lfn_name[i++] = dir->dir_name[n];
                        }
                    }
                }

                volume->dir_entries = 0;
                volume->lfn_count = i;
            }
            else
            {
                /* Compute the offset of the last '.' in the LFN.
                 */

                for (offset = volume->lfn_count, i = 0; i < volume->lfn_count; i++)
                {
                    if (volume->lfn_name[i] == '.')
                    {
                        offset = i;
                    }
                }
            }

            match = true;

            if (pattern != NULL)
            {
                /* The matcher below uses WINNT semantics:
                 *
                 * '.' (DOS_DOT)          matches either a '.' or zero characters beyond the name string
                 * '?' (DOS_QM)           matches a single character or, upon encountering a '.' or end of name string, advances the expression to the end of the set of contiguous DOS_QMs
                 * '*' (DOS_STAR)         matches zero or more characters until encountering and matching the final '.' in the name
                 *
                 * Derived sequences:
                 *
                 * '*' EOF                matches to the end of the name
                 * '*' '.'                matches either to the next '.' or zero characters beyond the name string
                 * '*' '?'                same as '?' (zero length '*' match)
                 * '*' '*'                same as '*' (zero length '*' match)
                 */

                skip = FALSE;

                i = 0;
                cc = 0;

                do
                {
                    if (!skip)
                    {
#if (DOSFS_CONFIG_UTF8_SUPPORTED == 1)
                        cc = dosfs_name_cstring_to_unicode(pattern, &pattern);
#else /* (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */
                        cc = *pattern++;
#endif /* (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */
                    }

                    skip = FALSE;

                    if (i < volume->lfn_count)
                    {
                        if (cc == '\0')
                        {
                            match = FALSE;
                        }
                        else if (cc == '*')                         /* asterisk      */
                        {
                            if (i <= offset)
                            {
                                i = offset;
                            }
                            else
                            {
                                i = volume->lfn_count;
                            }
                        }
                        else if (cc == '?')                         /* question mask  */
                        {
                            if (i == offset)
                            {
                                do
                                {
#if (DOSFS_CONFIG_UTF8_SUPPORTED == 1)
                                    cc = dosfs_name_cstring_to_unicode(pattern, &pattern);
#else /* (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */
                                    cc = *pattern++;
#endif /* (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */
                                }
                                while (cc == '?');

                                skip = TRUE;
                            }

                            i++;
                        }
                        else                                        /* <character>   */
                        {
                            lc = volume->lfn_name[i++];

#if (DOSFS_CONFIG_UTF8_SUPPORTED == 1)
                            cc = dosfs_name_unicode_upcase(cc);
                            lc = dosfs_name_unicode_upcase(lc);
#else /* (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */
                            /* That's a tad non-intuitive here. "volume->lfn_name[]" does contain
                             * only ASCII characters. Hence only the ASCII portion of the UNICODE
                             * name space has to be upcased. Anything outside needs properly mismatch.
                             */
                            cc = dosfs_name_ascii_upcase(cc);
                            lc = dosfs_name_ascii_upcase(lc);
#endif /* (DOSFS_CONFIG_UTF8_SUPPORTED == 1) */

                            if (lc != cc)
                            {
                                match = FALSE;
                            }
                        }
                    }
                    else
                    {
                        if ((cc == '\0') ||                         /* EOF           */
                            (cc == '.') ||                          /* dot           */
                            (cc == '?') ||                          /* question mark */
                            (cc == '*'))                            /* start         */
                        {
                        }
                        else                                        /* <character>   */
                        {
                            match = FALSE;
                        }
                    }
                }
                while (match && (cc != '\0'));
            }

            /* Set volume->dir_entries to 0 on a SFN match.
             */
            if (match && (sequence != DOSFS_LDIR_SEQUENCE_LAST))
            {
                volume->dir_entries = 0;
            }
        }
        else
        {
            match = FALSE;
        }
    }

    return match;
}

#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 0) */


/*
 * p_dir != NULL:
 *
 *   p_clsno    clsno of first primary/secondary dir entry
 *   p_index    index within dirctory of first primary/secondary dir entry
 *   p_dir      dir cache entry containg the primary dir entry

 * p_dir == NULL:
 *
 *   p_clsno    clsno of first free dir entry
 *   p_index    index within directory of first free dir entry,
 *              or (index | 0x00020000) if a new cluster is needed,
 *              or 0x00010000 if there is no free entry anymore.
 *
 * The number of secondary entries is contained within name->lfn_entries. 
 */

static int dosfs_path_find_entry(dosfs_volume_t *volume, uint32_t clsno, uint32_t index, uint32_t count, dosfs_find_callback_t callback, void *private, uint32_t *p_clsno, uint32_t *p_index, dosfs_dir_t **p_dir)
{
    int status = F_NO_ERROR;
    int done;
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
    unsigned int sequence, chksum, ordinal;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
    uint32_t clsno_f, clsno_m, clsdata, blkno, blkno_e, index_f, index_m, count_f;
    dosfs_cache_entry_t *entry;
    dosfs_dir_t *dir, *dir_e;

    done = FALSE;
    dir = NULL;

    if (clsno == DOSFS_CLSNO_NONE)
    {
        clsno = volume->root_clsno;
        blkno = volume->root_blkno + DOSFS_INDEX_TO_BLKCNT_ROOT(index);
        blkno_e = volume->root_blkno + volume->root_blkcnt;
    }
    else
    {
        /* dosfs_path_find_entry reports back clsno/index for the current set of
         * directory entries. Hence dosfs_path_find_pattern has to set to the next
         * entry by adding the composite size to "index", which of course
         * can cross then a cluster boundary.
         */

        if (index && !((index << DOSFS_DIR_SHIFT) & volume->cls_mask))
        {
            blkno = DOSFS_CLSNO_TO_BLKNO(clsno) + volume->cls_blk_size;
            blkno_e = DOSFS_CLSNO_TO_BLKNO(clsno) + volume->cls_blk_size;
        }
        else
        {
            blkno = DOSFS_CLSNO_TO_BLKNO(clsno) + DOSFS_INDEX_TO_BLKCNT(index);
            blkno_e = DOSFS_CLSNO_TO_BLKNO(clsno) + volume->cls_blk_size;
        }
    }

    clsno_f = DOSFS_CLSNO_NONE;
    index_f = 0;
    count_f = 0;

    clsno_m = DOSFS_CLSNO_NONE;
    index_m = 0;

#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
    sequence = 0;
    chksum = 0;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
    
    while ((status == F_NO_ERROR) && !done && (index < 0x10000))
    {
        if (blkno == blkno_e)
        {
            if (clsno == DOSFS_CLSNO_NONE)
            {
                done = TRUE;
                dir = NULL;
            }
            else
            {
                status = dosfs_cluster_read(volume, clsno, &clsdata);
                
                if (status == F_NO_ERROR)
                {
                    if (clsdata >= DOSFS_CLSNO_LAST)
                    {
                        done = TRUE;
                        dir = NULL;
                    }
                    else
                    {
                        if ((clsdata >= 2) && (clsdata <= volume->last_clsno))
                        {
                            clsno = clsdata;
                            blkno = DOSFS_CLSNO_TO_BLKNO(clsno);
                            blkno_e = blkno + volume->cls_blk_size;
                        }
                        else
                        {
                            status = F_ERR_EOF;
                        }
                    }
                }
            }
        }

        if (status == F_NO_ERROR)
        {
            if (!done)
            {
                status = dosfs_dir_cache_read(volume, blkno, &entry);
                
                if (status == F_NO_ERROR)
                {
                    dir = (dosfs_dir_t*)((void*)(entry->data + DOSFS_INDEX_TO_BLKOFS(index)));
                    dir_e = (dosfs_dir_t*)((void*)(entry->data + DOSFS_BLK_SIZE));
                
                    do
                    {
                        if (dir->dir_name[0] == 0x00)
                        {
                            /* A 0x00 in dir_name[0] says it's a free entry, and all subsequent entries
                             * are also free. So we can stop searching.
                             */

                            done = TRUE;
                            dir = NULL;
                        }
                        else if (dir->dir_name[0] == 0xe5)
                        {
                            /* A 0xe5 in dir_name[0] says it's a free entry.
                             */

#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                            if (count_f != count)
                            {
                                if ((count_f == 0) || !((clsno_f == clsno) && ((index_f + count_f) == index)))
                                {
                                    clsno_f = clsno;
                                    index_f = index;
                                    count_f = 1;
                                }
                                else
                                {
                                    count_f++;
                                }
                            }
#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                            if (clsno_f == DOSFS_CLSNO_NONE)
                            {
                                clsno_f = clsno;
                                index_f = index;
                            }
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */

                            dir++;
                            index++;
                        }
                        else
                        { 
                            if ((dir->dir_attr & DOSFS_DIR_ATTR_LONG_NAME_MASK) == DOSFS_DIR_ATTR_LONG_NAME)
                            {
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                                ordinal = (dir->dir_name[0] & 0x1f); /* LDIR_Ord    */
                                
                                if (ordinal == 0)
                                {
                                    sequence = 0; /* mark as bad sequence */
                                }
                                else
                                {
                                    if (dir->dir_name[0] & 0x40)
                                    {
                                        clsno_m = clsno;
                                        index_m = index;

                                        sequence = ordinal | DOSFS_LDIR_SEQUENCE_FIRST;
                                        chksum   = dir->dir_crt_time_tenth;     /* LDIR_ChkSum */
                                    }
                                    else
                                    {
                                        if (((sequence & DOSFS_LDIR_SEQUENCE_INDEX) == (ordinal +1)) && (chksum == dir->dir_crt_time_tenth))
                                        {
                                            sequence = (sequence & DOSFS_LDIR_SEQUENCE_MISMATCH) | ordinal;
                                        }
                                        else
                                        {
                                            sequence = 0; /* mark as bad sequence */
                                        }
                                    }
                                }

                                if ((sequence != 0) && !(sequence & DOSFS_LDIR_SEQUENCE_MISMATCH))
                                {
                                    if (!(*callback)(volume, private, dir, sequence))
                                    {
                                        sequence |= DOSFS_LDIR_SEQUENCE_MISMATCH;
                                    }
                                }
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */

                                dir++;
                                index++;
                            }
                            else
                            {
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 0)
                                if ((*callback)(volume, private, dir))
                                {
                                    clsno_m = clsno;
                                    index_m = index;

                                    done = TRUE;
                                }
#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 0) */

                                if ((sequence & DOSFS_LDIR_SEQUENCE_INDEX) == 1)
                                {
                                    if (chksum == dosfs_name_checksum_dosname(dir->dir_name))
                                    {
                                        sequence = (sequence & DOSFS_LDIR_SEQUENCE_MISMATCH) | DOSFS_LDIR_SEQUENCE_LAST;
                                    }
                                    else
                                    {
                                        sequence = 0;
                                    }
                                }

                                if (!(sequence & DOSFS_LDIR_SEQUENCE_MISMATCH))
                                {
                                    if ((*callback)(volume, private, dir, sequence))
                                    {
                                        if (sequence == 0)
                                        {
                                            clsno_m = clsno;
                                            index_m = index;
                                        }

                                        done = TRUE;
                                    }
                                }

                                sequence = 0;

#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 0) */

                                if (!done)
                                {
                                    dir++;
                                    index++;
                                }
                            }
                        }
                    }
                    while ((status == F_NO_ERROR) && !done && (dir != dir_e));

                    if ((status == F_NO_ERROR) && !done)
                    {
                        blkno++;
                    }
                }
            }
        }
    }

    if (status == F_NO_ERROR)
    {
        if (p_clsno)
        {
            if (dir != NULL)
            {
                *p_clsno = clsno_m;
                *p_index = index_m;
            }
            else
            {
                if (count != 0)
                {
                    if (count_f == count)
                    {
                        /* Found a matching set of entries ...
                         */
                        *p_clsno = clsno_f;
                        *p_index = index_f;
                    }
                    else
                    {
                        /* Need to allocate possibly a new cluster.
                         */
                            
                        if ((clsno == DOSFS_CLSNO_NONE) && (volume->type != DOSFS_VOLUME_TYPE_FAT32))
                        {
                            if (((index + count) << DOSFS_DIR_SHIFT) <= (volume->root_blkcnt * DOSFS_BLK_SIZE))
                            {
                                *p_clsno = DOSFS_CLSNO_NONE;
                                *p_index = index;
                            }
                            else
                            {
                                *p_clsno = DOSFS_CLSNO_NONE;
                                *p_index = 0x00010000;
                            }
                        }
                        else
                        {
                            if (((index << DOSFS_DIR_SHIFT) & ~volume->cls_mask) == (((index + count) << DOSFS_DIR_SHIFT) & ~volume->cls_mask))
                            {
                                *p_clsno = clsno;
                                *p_index = index;
                            }
                            else
                            {
                                if ((index + count) <= 0x00010000)
                                {
                                    *p_clsno = clsno;
                                    *p_index = index | 0x00020000;
                                }
                                else
                                {
                                    *p_clsno = DOSFS_CLSNO_NONE;
                                    *p_index = 0x00010000;
                                }
                            }
                        }
                    }
                }
            }
        }

        if (p_dir)
        {
            *p_dir = dir;
        }
    }

    return status;
}

/*
 * filename   incoming full path
 * p_filename last path element 
 * p_clsno    clsno of parent directory of last path element
 */

static int dosfs_path_find_directory(dosfs_volume_t *volume, const char *filename, const char **p_filename, uint32_t *p_clsno)
{
    int status = F_NO_ERROR;
    unsigned int cc;
    const char *filename_e;
    uint32_t clsno;
    dosfs_dir_t *dir;

    if (*filename == '\0')
    {
        status = F_ERR_INVALIDNAME;
    }
    else if ((*filename == '/') || (*filename == '\\'))
    {
        clsno = DOSFS_CLSNO_NONE;

        filename++;
    }
    else
    {
        if (volume->cwd_clsno != DOSFS_CLSNO_END_OF_CHAIN)
        {
            clsno = volume->cwd_clsno;
        }
        else
        {
            status = F_ERR_INVALIDDIR;
        }
    }

    if (status == F_NO_ERROR)
    {
        do
        {
            /* Scan ahead to see whether there is a directory left.
             */
            filename_e = filename;
            
            do
            {
                cc = *filename_e++;
            }
            while ((cc != '/') && (cc != '\\') && (cc != '\0'));
            
            if ((cc == '/') || (cc == '\\'))
            {
                status = dosfs_path_convert_filename(volume, filename, &filename);

                if (status == F_NO_ERROR)
                {
                    if ((volume->dir.dir_name[0] == '.') && (volume->dir.dir_name[1] == ' '))
                    {
                        /* This is a special case here, as there is no "." or ".." entry
                         * in the root directory.
                         */
                    }
                    else
                    {
                        status = dosfs_path_find_entry(volume, clsno, 0, 0, dosfs_path_find_callback_name, NULL, NULL, NULL, &dir);

                        if (status == F_NO_ERROR)
                        {
                            if (dir != NULL)
                            {
                                if (dir->dir_attr & DOSFS_DIR_ATTR_DIRECTORY)
                                {
                                    if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
                                    {
                                        clsno = ((uint32_t)DOSFS_FTOHS(dir->dir_clsno_hi) << 16) | (uint32_t)DOSFS_FTOHS(dir->dir_clsno_lo);
                                    }
                                    else
                                    { 
                                        clsno = (uint32_t)DOSFS_FTOHS(dir->dir_clsno_lo);
                                    }
                                }
                                else
                                {
                                    status = F_ERR_INVALIDDIR;
                                }
                            }
                            else
                            {
                                status = F_ERR_INVALIDDIR;
                            }
                        }
                    }
                }
            }
        }
        while ((status == F_NO_ERROR) && ((cc == '/') || (cc == '\\')));
    }

    if (status == F_NO_ERROR)
    {
        *p_filename = filename;
        *p_clsno = clsno;
    }

    return status;
}

static int dosfs_path_find_file(dosfs_volume_t *volume, const char *filename, uint32_t *p_clsno, uint32_t *p_index, dosfs_dir_t **p_dir)
{
    int status = F_NO_ERROR;
    uint32_t clsno_d;

    status = dosfs_path_find_directory(volume, filename, &filename, &clsno_d);

    if (status == F_NO_ERROR)
    {
        status = dosfs_path_convert_filename(volume, filename, NULL);

        if (status == F_NO_ERROR)
        {
            status = dosfs_path_find_entry(volume, clsno_d, 0, 0, dosfs_path_find_callback_name, NULL, p_clsno, p_index, p_dir);
            
            if (status == F_NO_ERROR)
            {
                if (*p_dir == NULL)
                {
                    status = F_ERR_NOTFOUND;
                }
            }
        }
    }

    return status;
}

static int dosfs_path_find_pattern(dosfs_volume_t *volume, F_DIR *parent, char *pattern, char *filename, dosfs_dir_t **p_dir)
{
    int status = F_NO_ERROR;
    uint32_t clsno, index;
    dosfs_dir_t *dir;

    status = dosfs_path_find_entry(volume, parent->dir_clsno, parent->dir_index, 0, dosfs_path_find_callback_pattern, pattern, &clsno, &index, &dir);

    if (status == F_NO_ERROR)
    {
        if (dir != NULL)
        {
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
            /* It's possible to have a sfn_name only ("." and ".." for example).
             */
            if (volume->lfn_count)
            {
                filename = dosfs_name_uniname_to_cstring(volume->lfn_name, volume->lfn_count, filename, filename + F_MAXPATH);
            }
            else
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
            {
                filename = dosfs_name_dosname_to_cstring(dir->dir_name, dir->dir_nt_reserved, filename, filename + F_MAXPATH);
            }

            if (filename != NULL)
            {
                parent->dir_clsno = clsno;
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                parent->dir_index = index + volume->dir_entries +1;
#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                parent->dir_dir.index = index +1;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */

                *p_dir = dir;
            }
            else
            {
                status = F_ERR_TOOLONGNAME;
            }
        }
        else
        {
            status = F_ERR_NOTFOUND;
        }
    }

    return status;
}

static void dosfs_path_setup_entry(dosfs_volume_t *volume, const char *dosname, uint8_t attr, uint32_t first_clsno, uint16_t ctime, uint16_t cdate, dosfs_dir_t *dir)
{
    if (dosname)
    {
        memcpy(dir->dir_name, dosname, 11);
        
        dir->dir_nt_reserved = 0x00;
    }
    else
    {
        if (dir != &volume->dir)
        {
            memcpy(dir->dir_name, volume->dir.dir_name, 11);
        
            dir->dir_nt_reserved = volume->dir.dir_nt_reserved;
        }
    }

    dir->dir_attr = attr;
    
    if (volume->type != DOSFS_VOLUME_TYPE_FAT32)
    {
        dir->dir_crt_time_tenth = 0;
        dir->dir_crt_time = 0;
        dir->dir_crt_date = 0;
        dir->dir_acc_date = 0;
        dir->dir_clsno_hi = 0;
    }
    else
    {
        dir->dir_crt_time_tenth = 0;
        dir->dir_crt_time = ctime;
        dir->dir_crt_date = cdate;
        dir->dir_acc_date = 0;
        dir->dir_clsno_hi = DOSFS_HTOFS(first_clsno >> 16);
    }
    
    dir->dir_wrt_time = ctime;
    dir->dir_wrt_date = cdate;
    dir->dir_clsno_lo = DOSFS_HTOFS(first_clsno & 0xffff);
    dir->dir_file_size = DOSFS_HTOFL(0);
}

static int dosfs_path_create_entry(dosfs_volume_t *volume, uint32_t clsno_d, uint32_t clsno, uint32_t index, const char *dosname, uint8_t attr, uint32_t first_clsno, uint16_t ctime, uint16_t cdate)
{
    int status = F_NO_ERROR;
    uint32_t blkno, blkno_s, clsno_s;
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
    unsigned int chksum, mask, prefix;
    dosfs_dir_t *dir;
#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0)
    uint32_t blkno_e;
    unsigned int sequence, offset, i, s, cc;
    dosfs_dir_t *dir_e;
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */
#else  /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0)
    dosfs_dir_t *dir;
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
    dosfs_cache_entry_t *entry;

    if (index == 0x00010000)
    {
        status = F_ERR_NOMOREENTRY;
    }
    else
    {
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
        if (!dosname && ((volume->dir.dir_name[0] == '\0') || (volume->dir.dir_nt_reserved & DOSFS_DIR_TYPE_LOSSY)))
        {
            dosfs_name_uniname_to_dosname(volume->lfn_name, volume->lfn_count, volume->dir.dir_name, &prefix);

            volume->dir.dir_nt_reserved = 0;

            if (prefix)
            {
                if (prefix > 6)
                {
                    prefix = 6;
                }
                
                /* First check unique entries of the <name>~[1-9].<ext> form.
                 */
                
                volume->dir.dir_name[prefix +0] = '~';
                
                mask = ((prefix +1) << 9);
                
                status = dosfs_path_find_entry(volume, clsno_d, 0, 0, dosfs_path_find_callback_unique, &mask, NULL, NULL, NULL);

                if (status == F_NO_ERROR)
                {
                    mask &= 0x000001ff;
                
                    if (mask != 0x000001ff)
                    {
                        volume->dir.dir_name[prefix +1] = '1';
                        
                        while (mask & 1)
                        {
                            volume->dir.dir_name[prefix +1] += 1;
                            mask >>= 1;
                        }
                    }
                }
            }
            else
            {
                mask = 0x000001ff;
            }

            if (status == F_NO_ERROR)
            {
                if (mask & 1)
                {
                    /* Then check unique entries of the <name><checksum>~1.<ext> form.
                     * Loop till a vaild unique name was found by stepping linear over
                     * "chksum".
                     */

                    if (prefix > 2)
                    {
                        prefix = 2;
                    }

                    chksum = dosfs_name_checksum_uniname(volume->lfn_name, volume->lfn_count);

                    do
                    {
                        volume->dir.dir_name[prefix +0] = dosfs_name_nibble_to_char_table[(chksum >> 12) & 15];
                        volume->dir.dir_name[prefix +1] = dosfs_name_nibble_to_char_table[(chksum >>  8) & 15];
                        volume->dir.dir_name[prefix +2] = dosfs_name_nibble_to_char_table[(chksum >>  4) & 15];
                        volume->dir.dir_name[prefix +3] = dosfs_name_nibble_to_char_table[(chksum >>  0) & 15];
                        volume->dir.dir_name[prefix +4] = '~';
                        volume->dir.dir_name[prefix +5] = '1';
                        
                        status = dosfs_path_find_entry(volume, clsno_d, 0, 0, dosfs_path_find_callback_unique, NULL, NULL, NULL, &dir);
                        
                        if (status == F_NO_ERROR) 
                        {
                            if (dir != NULL)
                            {
                                chksum += 1;
                            }
                        }
                    }
                    while ((status == F_NO_ERROR) && (dir != NULL));
                }
            }
        }
        else
        {
            volume->dir_entries = 0;
        }

        if (status == F_NO_ERROR)
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
        {
            if (index & 0x00020000)
            {
                /* The appending of a new cluster to a directory is a tad tricky. One has to allocate
                 * a cluster first, zero out it's contents, link the cluster to the end of the chain.
                 * Also the fat cache has to be flushed as it's the final operation of a sequence (see f_mkdir).
                 * The issue at hand is that if a new cluster is linked in that is not zeroed out, the
                 * directory is invalid. One the other hand, we cannot write a directory entry to a directory
                 * that has uncommited clusters in the fat cache.
                 */
                
                status = dosfs_cluster_chain_create(volume, DOSFS_CLSNO_NONE, 1, &clsno_s, NULL);
                
                if (status == F_NO_ERROR)
                {
                    blkno_s = DOSFS_CLSNO_TO_BLKNO(clsno_s);

                    status = dosfs_volume_zero(volume, blkno_s, volume->cls_blk_size, true);

                    if (status == F_NO_ERROR)
                    {
                        status = dosfs_cluster_write(volume, clsno, clsno_s, TRUE);

                        if (status == F_NO_ERROR)
                        {
                            index &= ~0x00020000;

                            if (!((index << DOSFS_DIR_SHIFT) & volume->cls_mask))
                            {
                                clsno = clsno_s;

                                /* If "index" is the first one of a new cluster,
                                 * one might as well avoid having to read the 
                                 * dir_cache entry.
                                 */
                                status = dosfs_dir_cache_zero(volume, blkno_s, &entry);
                            }
                        }
                    }
                }
            }

#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0)
            /* For TRANSACTION_SAFE all of this happens throu dosfs_volume_record().
             */
        
            /* The fat cache needs to be flushed before any dir entry update.
             */
            status = dosfs_fat_cache_flush(volume);

            if (status == F_NO_ERROR)
            {
                if (clsno == DOSFS_CLSNO_NONE)
                {
                    clsno = volume->root_clsno;
                    blkno = volume->root_blkno + DOSFS_INDEX_TO_BLKCNT_ROOT(index);
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                    blkno_e = volume->root_blkno + volume->root_blkcnt;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                }
                else
                {
                    blkno = DOSFS_CLSNO_TO_BLKNO(clsno) + DOSFS_INDEX_TO_BLKCNT(index);
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                    blkno_e = blkno + volume->cls_blk_size;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                }
                
                status = dosfs_dir_cache_read(volume, blkno, &entry);

                if (status == F_NO_ERROR)
                {
                    dir = (dosfs_dir_t*)((void*)(entry->data + DOSFS_INDEX_TO_BLKOFS(index)));

#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                    if (volume->dir_entries != 0)
                    {
                        dir_e = (dosfs_dir_t*)((void*)(entry->data + DOSFS_BLK_SIZE));

                        sequence = 0x40 | volume->dir_entries;
                        chksum = dosfs_name_checksum_dosname(volume->dir.dir_name);

                        do
                        {
                            s = 0;
                            i = ((sequence & 0x1f) - 1) * 13;
                
                            dir->dir_name[0] = sequence;
                            dir->dir_attr = DOSFS_DIR_ATTR_LONG_NAME;
                            dir->dir_nt_reserved = 0x00;
                            dir->dir_crt_time_tenth = chksum;
                            dir->dir_clsno_lo = 0x0000;
                
                            do
                            {
                                offset = dosfs_path_ldir_name_table[s++];
                    
                                if (i < volume->lfn_count)
                                {
                                    cc = volume->lfn_name[i];
                        
                                    ((uint8_t*)dir)[offset +0] = cc;
                                    ((uint8_t*)dir)[offset +1] = cc >> 8;
                                }
                                else
                                {
                                    if (i == volume->lfn_count)
                                    {
                                        ((uint8_t*)dir)[offset +0] = 0x00;
                                        ((uint8_t*)dir)[offset +1] = 0x00;
                                    }
                                    else
                                    {
                                        ((uint8_t*)dir)[offset +0] = 0xff;
                                        ((uint8_t*)dir)[offset +1] = 0xff;
                                    }
                                }

                                i++;
                            }
                            while (s < 13);
                
                            dir++;

                            if (dir == dir_e)
                            {
                                status = dosfs_dir_cache_write(volume);

                                blkno++;

                                if (blkno == blkno_e)
                                {
                                    status = dosfs_cluster_read(volume, clsno, &clsno);
                
                                    if (status == F_NO_ERROR)
                                    {
                                        blkno = DOSFS_CLSNO_TO_BLKNO(clsno);
                                        blkno_e = blkno + volume->cls_blk_size;
                                    }
                                }

                                if (status == F_NO_ERROR)
                                {
                                    status = dosfs_dir_cache_read(volume, blkno, &entry);

                                    if (status == F_NO_ERROR)
                                    {
                                        dir = (dosfs_dir_t*)((void*)(entry->data));
                                    }
                                }
                            }

                            sequence = (sequence & 0x1f) -1;
                        }
                        while ((status == F_NO_ERROR) && (sequence != 0));
                    }

                    if (status == F_NO_ERROR)
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                    {
                        /* Special semantics if "first_clsno" == DOSFS_CLUSTER_END_OF_CHAIN. In that
                         * case volume->dir contains a valid template from f_rename, and
                         * "attr" gets merged with volume->dir.dir_nt_reserved (as this
                         * gets blown away by the name conversion).
                         */
                        if (first_clsno == DOSFS_CLSNO_END_OF_CHAIN)
                        {
                            memcpy(dir, &volume->dir, sizeof(dosfs_dir_t));

                            dir->dir_nt_reserved = ((volume->dir.dir_nt_reserved & (DOSFS_DIR_TYPE_LCASE_NAME | DOSFS_DIR_TYPE_LCASE_EXT)) |
                                                    (attr & ~(DOSFS_DIR_TYPE_LCASE_NAME | DOSFS_DIR_TYPE_LCASE_EXT)));
                        }
                        else
                        {
                            dosfs_path_setup_entry(volume, dosname, attr, first_clsno, ctime, cdate, dir);
                        }

                        status = dosfs_dir_cache_write(volume);
                    }
                }
            }

#else /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */
        
            if (status == F_NO_ERROR)
            {
                /* Special semantics if "first_clsno" == DOSFS_CLUSTER_END_OF_CHAIN. In that
                 * case volume->dir contains a valid template from f_rename, and
                 * "attr" gets merged with volume->dir.dir_nt_reserved (as this
                 * gets blown away by the name conversion).
                 */
                if (first_clsno == DOSFS_CLSNO_END_OF_CHAIN)
                {
                    volume->dir.dir_nt_reserved = ((volume->dir.dir_nt_reserved & (DOSFS_DIR_TYPE_LCASE_NAME | DOSFS_DIR_TYPE_LCASE_EXT)) |
                                                   (attr & ~(DOSFS_DIR_TYPE_LCASE_NAME | DOSFS_DIR_TYPE_LCASE_EXT)));
                }
                else
                {
                    dosfs_path_setup_entry(volume, dosname, attr, first_clsno, ctime, cdate, &volume->dir);
                }

                volume->dir_flags |= DOSFS_DIR_FLAG_CREATE_ENTRY;
                volume->dir_clsno = clsno;
                volume->dir_index = index;
                volume->dot_clsno = clsno_d;

                status = dosfs_volume_record(volume);
            }
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */
        }
    }

    return status;
}

/* To destroy a directory/file, first the directory entry is deleted. Then the assocociated
 * cluster chain is freed.
 */

#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
static int dosfs_path_destroy_entry(dosfs_volume_t *volume, uint32_t clsno, uint32_t index, uint32_t entries, uint32_t first_clsno)
#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
static int dosfs_path_destroy_entry(dosfs_volume_t *volume, uint32_t clsno, uint32_t index, uint32_t first_clsno)
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
{
    int status = F_NO_ERROR;
#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0)
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
    unsigned int sequence;
    uint32_t blkno, blkno_e;
    dosfs_dir_t *dir, *dir_e;
#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
    uint32_t blkno;
    dosfs_dir_t *dir;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
    dosfs_cache_entry_t *entry;

    if (clsno == DOSFS_CLSNO_NONE)
    {
        clsno = volume->root_clsno;
        blkno = volume->root_blkno + DOSFS_INDEX_TO_BLKCNT_ROOT(index);
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
        blkno_e = volume->root_blkno + volume->root_blkcnt;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
    }
    else
    {
        blkno = DOSFS_CLSNO_TO_BLKNO(clsno) + DOSFS_INDEX_TO_BLKCNT(index);
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
        blkno_e = DOSFS_CLSNO_TO_BLKNO(clsno) + volume->cls_blk_size;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
    }
    
    status = dosfs_dir_cache_read(volume, blkno, &entry);

    if (status == F_NO_ERROR)
    {
        dir = (dosfs_dir_t*)((void*)(entry->data + DOSFS_INDEX_TO_BLKOFS(index)));

#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
        if (entries != 0)
        {
            sequence = entries;

            dir_e = (dosfs_dir_t*)((void*)(entry->data + DOSFS_BLK_SIZE));

            do
            {
                dir->dir_name[0] = 0xe5;
                
                dir++;
                
                if (dir == dir_e)
                {
                    status = dosfs_dir_cache_write(volume);

                    blkno++;

                    if (blkno == blkno_e)
                    {
                        status = dosfs_cluster_read(volume, clsno, &clsno);
                
                        if (status == F_NO_ERROR)
                        {
                            blkno = DOSFS_CLSNO_TO_BLKNO(clsno);
                            blkno_e = blkno + volume->cls_blk_size;
                        }
                    }

                    if (status == F_NO_ERROR)
                    {
                        status = dosfs_dir_cache_read(volume, blkno, &entry);

                        if (status == F_NO_ERROR)
                        {
                            dir = (dosfs_dir_t*)((void*)(entry->data));
                        }
                    }
                }
                
                sequence--;
            }
            while ((status == F_NO_ERROR) && (sequence != 0));
        }

        if (status == F_NO_ERROR)
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
        {
            dir->dir_name[0] = 0xe5;
        
            status = dosfs_dir_cache_write(volume);
        
            if (status == F_NO_ERROR)
            {
                if (first_clsno != DOSFS_CLSNO_NONE)
                {
                    status = dosfs_cluster_chain_destroy(volume, first_clsno, DOSFS_CLSNO_FREE);
                }
            }
        }


        if (status == F_NO_ERROR)
        {
            /* The post condition for dosfs_path_destroy_entry is that the fat cache
             * is flushed, so unconditionally do it here.
             */
            status = dosfs_fat_cache_flush(volume);
        }
    }

#else /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */

    if (first_clsno != DOSFS_CLSNO_NONE)
    {
        status = dosfs_cluster_chain_destroy(volume, first_clsno, DOSFS_CLSNO_FREE);
    }

    if (status == F_NO_ERROR)
    {
        volume->dir_flags |= DOSFS_DIR_FLAG_DESTROY_ENTRY;
        volume->del_clsno = clsno;
        volume->del_index = index;
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
        volume->del_entries = entries;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */

        status = dosfs_volume_record(volume);
    }

#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */


    return status;
}


/*****************************************************************************************************************************************/


static dosfs_file_t * dosfs_file_enumerate(dosfs_volume_t *volume, dosfs_file_t *file, uint32_t clsno, uint32_t index)
{
    if (file)
    {
        file = file->next;
    }
    else
    {
        file = volume->files;
    }

    for (; file; file = file->next)
    {
        if ((file->dir_clsno == clsno) && (file->dir_index == index))
        {
            break;
        }
    }

    return file;
}

static int dosfs_file_sync(dosfs_volume_t *volume, dosfs_file_t *file, int access, int modify, uint32_t first_clsno, uint32_t length)
{
    int status = F_NO_ERROR;
    uint16_t ctime, cdate;
#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0)
    uint32_t clsno, blkno, index;
    dosfs_cache_entry_t *entry;
    dosfs_dir_t *dir;
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */

#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)

    volume->dir.dir_clsno_lo = DOSFS_HTOFS(first_clsno & 0xffff);
    volume->dir.dir_file_size = DOSFS_HTOFL(length);
    
    if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
    {
        volume->dir.dir_clsno_hi = DOSFS_HTOFS(first_clsno >> 16);
    }
    
    volume->dir_flags |= DOSFS_DIR_FLAG_SYNC_ENTRY;
    
    if (access || modify)
    {
#if defined(DOSFS_PORT_CORE_TIMEDATE)
        DOSFS_PORT_CORE_TIMEDATE(&ctime, &cdate);
#else /* DOSFS_PORT_CORE_TIMEDATE */
        ctime = 0;
        cdate = 0;
#endif /* DOSFS_PORT_CORE_TIMEDATE */

        if (access)
        {
            if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
            {
                volume->dir.dir_acc_date = DOSFS_HTOFS(cdate);
                    
                volume->dir_flags |= DOSFS_DIR_FLAG_ACCESS_ENTRY;
            }
        }

        if (modify)
        {
            volume->dir.dir_wrt_time = DOSFS_HTOFS(ctime);
            volume->dir.dir_wrt_date = DOSFS_HTOFS(cdate);

            volume->dir_flags |= DOSFS_DIR_FLAG_MODIFY_ENTRY;
        }
    }
        
    volume->dir_clsno = file->dir_clsno;
    volume->dir_index = file->dir_index;
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
    volume->dir_entries = 0;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */

    status = dosfs_volume_record(volume);

#else /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */

    status = dosfs_fat_cache_flush(volume);

    if (status == F_NO_ERROR)
    {
        clsno = file->dir_clsno;
        index = file->dir_index;

        if (clsno == DOSFS_CLSNO_NONE)
        {
            blkno = volume->root_blkno + DOSFS_INDEX_TO_BLKCNT_ROOT(index);
        }
        else
        {
            blkno = DOSFS_CLSNO_TO_BLKNO(clsno) + DOSFS_INDEX_TO_BLKCNT(index);
        }
    
        status = dosfs_dir_cache_read(volume, blkno, &entry);
    
        if (status == F_NO_ERROR)
        {
            dir = (dosfs_dir_t*)((void*)(entry->data + DOSFS_INDEX_TO_BLKOFS(index)));

            dir->dir_clsno_lo = DOSFS_HTOFS(first_clsno & 0xffff);
            dir->dir_file_size = DOSFS_HTOFL(length);
        
            if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
            {
                dir->dir_clsno_hi = DOSFS_HTOFS(first_clsno >> 16);
            }
        
            if (access || modify)
            {
#if defined(DOSFS_PORT_CORE_TIMEDATE)
                DOSFS_PORT_CORE_TIMEDATE(&ctime, &cdate);
#else /* DOSFS_PORT_CORE_TIMEDATE */
                ctime = 0;
                cdate = 0;
#endif /* DOSFS_PORT_CORE_TIMEDATE */

                if (access)
                {
                    if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
                    {
                        dir->dir_acc_date = DOSFS_HTOFS(cdate);
                    }
                }

                if (modify)
                {
                    dir->dir_attr |= DOSFS_DIR_ATTR_ARCHIVE;
                    dir->dir_wrt_time = DOSFS_HTOFS(ctime);
                    dir->dir_wrt_date = DOSFS_HTOFS(cdate);
                }
            }

            status = dosfs_dir_cache_write(volume);

            if (status == F_NO_ERROR)
            {
                status = dosfs_volume_clean(volume, status);
            }
        }
    }
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */

    return status;
}


static int dosfs_file_flush(dosfs_volume_t *volume, dosfs_file_t *file, int close)
{
    int status = F_NO_ERROR;
    dosfs_device_t *device;

    device = DOSFS_VOLUME_DEVICE(volume);

    if (volume->state == DOSFS_VOLUME_STATE_MOUNTED)
    {
        /* The data cache is first flushed and outstanding disk operations
         * are finished. Then however the fat cache flush and  the directory updates
         * are only performed uncoditionally irregardless of whether there were
         * outstanding errors.
         */

        status = dosfs_data_cache_flush(volume, file);

        if (status == F_NO_ERROR)
        {
            status = (*device->interface->sync)(device->context);
        }

        if (file->status == F_NO_ERROR)
        {
            file->status = status;
        }

        if (close || (file->flags & (DOSFS_FILE_FLAG_DIR_MODIFIED | DOSFS_FILE_FLAG_DATA_MODIFIED)))
        {
            status = dosfs_file_sync(volume, file, close, (file->flags & (DOSFS_FILE_FLAG_DIR_MODIFIED | DOSFS_FILE_FLAG_DATA_MODIFIED)), file->first_clsno, file->length);
        }
    }

    if (status == F_NO_ERROR)
    {
        file->flags &= ~(DOSFS_FILE_FLAG_DATA_MODIFIED | DOSFS_FILE_FLAG_DIR_MODIFIED);
    }

    return status;
}

static int dosfs_file_seek(dosfs_volume_t *volume, dosfs_file_t *file, uint32_t position)
{
    int status = F_NO_ERROR;
    uint32_t clsno, clscnt, offset;

    if ((file->mode & DOSFS_FILE_MODE_WRITE) && ((file->position & ~DOSFS_BLK_MASK) != (position & ~DOSFS_BLK_MASK)))
    {
        status = dosfs_data_cache_flush(volume, file);
    }

    if (status == F_NO_ERROR)
    {
        if (position == 0)
        {
            /* A seek to position 0 (i.e. rewind) always succeeds.
             */
            if (file->first_clsno != DOSFS_CLSNO_NONE)
            {
                file->clsno = file->first_clsno;
                file->blkno = DOSFS_CLSNO_TO_BLKNO(file->clsno);
                file->blkno_e = file->blkno + volume->cls_blk_size;
            }
        
            file->position = 0;
        }
        else
        {
            if (file->position != position)
            {
                if (file->length == 0)
                {
                    /* Any seek for file->length == 0 always succeeds.
                     */
                    file->position = position;
                }
                else
                {
                    if (file->first_clsno == DOSFS_CLSNO_NONE)
                    {
                        /* A seek to any non-zero position with no
                         * cluster in the chain is an error.
                         */
                        status = F_ERR_EOF;
                    }
                    else
                    {
                        if (position < file->length)
                        {
                            offset = position;
                        }
                        else
                        {
                            offset = file->length;
                        }

                        if (status == F_NO_ERROR)
                        {
#if (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1)
                            if (file->flags & DOSFS_FILE_FLAG_CONTIGUOUS)
                            {
                                clsno = file->first_clsno + DOSFS_OFFSET_TO_CLSCNT(offset -1);
                            }
                            else
#endif /* (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1) */
                            {
                                if ((offset == file->length) && (file->last_clsno != DOSFS_CLSNO_NONE))
                                {
                                    clsno = file->last_clsno;
                                }
                                else
                                {
                                    if ((file->position == 0) || (file->position > offset))
                                    {
                                        clsno = file->first_clsno;
                                        clscnt = DOSFS_OFFSET_TO_CLSCNT(offset -1);
                                    }
                                    else
                                    {
                                        clsno = file->clsno;
                                        clscnt = DOSFS_OFFSET_TO_CLSCNT(offset -1) - DOSFS_OFFSET_TO_CLSCNT(file->position -1);
                                    }

                                    if (clscnt != 0)
                                    {
                                        status = dosfs_cluster_chain_seek(volume, clsno, clscnt, &clsno);
                                    }
                                }
                            }
                        
                            if (status == F_NO_ERROR)
                            {
                                file->position = position;
                                file->clsno = clsno;

                                if (!(offset & volume->cls_mask))
                                {
                                    file->blkno = DOSFS_CLSNO_TO_BLKNO(clsno) + volume->cls_blk_size;
                                    file->blkno_e = file->blkno;
                                }
                                else
                                {
                                    file->blkno = DOSFS_CLSNO_TO_BLKNO(clsno) + DOSFS_OFFSET_TO_BLKCNT(offset);
                                    file->blkno_e = DOSFS_CLSNO_TO_BLKNO(clsno) + volume->cls_blk_size;
                                }

                                if (offset == file->length)
                                {
                                    file->last_clsno = clsno;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return status;
}

static int dosfs_file_shrink(dosfs_volume_t *volume, dosfs_file_t *file)
{
    int status = F_NO_ERROR;

    if (file->position == 0)
    {
        if (file->first_clsno != DOSFS_CLSNO_NONE)
        {
#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)

            /* Defer the deletion of file->first_clsno to the next dosfs_file_flush(),
             * so that file->first_clsno ends up on the disk before the cluster is made
             * available again.
             */
                   
            status = dosfs_cluster_chain_destroy(volume, file->first_clsno, DOSFS_CLSNO_FREE);

            if (status == F_NO_ERROR)
            {
                status = dosfs_file_sync(volume, file, FALSE, FALSE, DOSFS_CLSNO_NONE, file->length);
            }

#else /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */

            /* To be on the safe side, the directory entry on the disk is updated first,
             * so that "file->length" is always correct, and in case the whole chain gets
             * deleted "file->first_clsno" on disk is properly zeroed out.
             */

            status = dosfs_file_sync(volume, file, FALSE, FALSE, DOSFS_CLSNO_NONE, file->length);

            if (status == F_NO_ERROR)
            {
                status = dosfs_volume_dirty(volume);

                if (status == F_NO_ERROR)
                {
                    status = dosfs_cluster_chain_destroy(volume, file->first_clsno, DOSFS_CLSNO_FREE);
                }
            }
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */
        }

        if (status == F_NO_ERROR)
        {
            file->first_clsno = DOSFS_CLSNO_NONE;
            file->last_clsno = DOSFS_CLSNO_NONE;
                
            /* file->position is 0 here, but clsno/blkno/blkno_e
             * point to the first cluster, which just got deleted.
             */
            file->clsno = DOSFS_CLSNO_NONE;
            file->blkno = DOSFS_BLKNO_INVALID;
            file->blkno_e = DOSFS_BLKNO_INVALID;
        }
    }
    else
    {
        if (file->clsno != DOSFS_CLSNO_NONE)
        {
#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0)
            status = dosfs_volume_dirty(volume);

            if (status == F_NO_ERROR)
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */
            {
                status = dosfs_cluster_chain_destroy(volume, file->clsno, DOSFS_CLSNO_END_OF_CHAIN);
                
                if (status == F_NO_ERROR)
                {
                    file->last_clsno = file->clsno;
                }
            }
        }
        else
        {
            status = F_ERR_EOF;
        }
    }

    if (status == F_NO_ERROR)
    {
        if (file->length != file->position)
        {
            file->flags |= DOSFS_FILE_FLAG_DIR_MODIFIED;
            file->length = file->position;
        }

        file->flags |= DOSFS_FILE_FLAG_END_OF_CHAIN;

#if (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1)
        if (file->flags & DOSFS_FILE_FLAG_CONTIGUOUS)
        {
            if (file->length >= (DOSFS_FILE_SIZE_MAX & ~volume->cls_mask))
            {
                file->size = DOSFS_FILE_SIZE_MAX;
            }
            else
            {
                file->size = (file->length + volume->cls_mask) & ~volume->cls_mask;
            }
        }
#endif /* (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1) */
    }

    if (file->status == F_NO_ERROR)
    {
        file->status = status;
    }

    return status;
}

static int dosfs_file_extend(dosfs_volume_t *volume, dosfs_file_t *file, uint32_t length)
{
    int status = F_NO_ERROR;
    uint32_t clsno, clscnt, clsno_a, clsno_l, clsno_n, clsdata, blkno, blkno_e, blkcnt, count, size, position, offset, length_o;
    dosfs_cache_entry_t *entry;

    /* Compute below:
     *
     * clsno      file->last_clsno or DOSFS_CLSNO_NONE (corresponding to file->length)
     * clsno_l    file->last_clsno (corresponding to length)
     * clsno_n    next cluster after clsno (or first allocated)
     */

    length_o = file->length;

#if (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1)
    if (file->flags & DOSFS_FILE_FLAG_CONTIGUOUS)
    {
        if (length_o == 0)
        {
            clsno = file->first_clsno;
            clsno_n = clsno;
        }
        else
        {
            clsno = file->first_clsno + DOSFS_OFFSET_TO_CLSCNT(length_o -1);
            clsno_n = clsno +1;
        }

        clsno_l = file->first_clsno + DOSFS_OFFSET_TO_CLSCNT(length -1);
    }
    else
#endif /* (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1) */
    {
        if (length_o == 0)
        {
            clsno = file->first_clsno;
        }
        else
        {
            if (file->first_clsno == DOSFS_CLSNO_NONE)
            {
                clsno = DOSFS_CLSNO_NONE;
                length_o = 0;
            }
            else
            {
                if (file->last_clsno != DOSFS_CLSNO_NONE)
                {
                    clsno = file->last_clsno;
                }
                else
                {
                    if (!file->position || (file->position > length_o))
                    {
                        clsno = file->first_clsno;
                        clscnt = DOSFS_OFFSET_TO_CLSCNT(length_o -1);
                    }
                    else
                    {
                        clsno = file->clsno;
                        clscnt = DOSFS_OFFSET_TO_CLSCNT(length_o -1) - DOSFS_OFFSET_TO_CLSCNT(file->position -1);
                    }
                    
                    if (clscnt != 0)
                    {
                        status = dosfs_cluster_chain_seek(volume, clsno, clscnt, &clsno);
                    }
                }

                if (status == F_NO_ERROR)
                {
                    file->last_clsno = clsno;
                }
            }
        }

        if (status == F_NO_ERROR)
        {
            if ((length_o == 0) || (clsno == DOSFS_CLSNO_NONE))
            {
                clscnt = DOSFS_SIZE_TO_CLSCNT(length);
            }
            else
            {
                clscnt = DOSFS_SIZE_TO_CLSCNT(length) - DOSFS_SIZE_TO_CLSCNT(length_o);
            }
            
            clsno_l = clsno;
            clsno_n = DOSFS_CLSNO_NONE;

            if (clscnt)
            {
                /* While the last cluster had not been seen yet, step throu the trailing 
                 * list and make use of what had been there already.
                 *
                 * Normally this should only set DOSFS_FILE_FLAG_END_OF_CHAIN, but there might
                 * be dangling, orphaned entries which one might make use of to begin with.
                 *
                 * N.b. file_first_clsno == DOSFS_CLSNO_NONE implies DOSFS_FILE_FLAG_END_OF_CHAIN 
                 * set.
                 */

                if (!(file->flags & DOSFS_FILE_FLAG_END_OF_CHAIN))
                {
                    do
                    {
                        status = dosfs_cluster_read(volume, clsno_l, &clsdata);
                        
                        if (status == F_NO_ERROR)
                        {
                            if (clsdata >= DOSFS_CLSNO_LAST)
                            {
                                file->flags |= DOSFS_FILE_FLAG_END_OF_CHAIN;
                            }
                            else
                            {
                                if ((clsdata >= 2) && (clsdata <= volume->last_clsno))
                                {
                                    if (clsno_n == DOSFS_CLSNO_NONE)
                                    {
                                        clsno_n = clsdata;
                                    }

                                    clsno_l = clsdata;
                                    clscnt--;
                                }
                                else
                                {
                                    status = F_ERR_EOF;
                                }
                            }
                        }
                    }
                    while ((status == F_NO_ERROR) && !(file->flags & DOSFS_FILE_FLAG_END_OF_CHAIN) && (clscnt != 0));
                }

                if (status == F_NO_ERROR)
                {
                    if (clscnt)
                    {
#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0)
                        status = dosfs_volume_dirty(volume);
                        
                        if (status == F_NO_ERROR)
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */
                        {
#if (DOSFS_CONFIG_SEQUENTIAL_SUPPORTED == 1)
                            if (file->mode & DOSFS_FILE_MODE_SEQUENTIAL)
                            {
                                status = dosfs_cluster_chain_create_sequential(volume, clsno_l, clscnt, &clsno_a, &clsno_l);
                            }
                            else
#endif /* (DOSFS_CONFIG_SEQUENTIAL_SUPPORTED == 1) */
                            {
                                status = dosfs_cluster_chain_create(volume, clsno_l, clscnt, &clsno_a, &clsno_l);
                            }

                            if (clsno_n == DOSFS_CLSNO_NONE)
                            {
                                clsno_n = clsno_a;
                            }
                        }
                    }
                }

                if (status == F_NO_ERROR)
                {
                    if (file->first_clsno == DOSFS_CLSNO_NONE)
                    {
                        file->first_clsno = clsno_n;
                        
                        file->clsno = clsno_n;
                        file->blkno = DOSFS_CLSNO_TO_BLKNO(clsno_n);
                        file->blkno_e = file->blkno + volume->cls_blk_size;

                        /* If a new cluster chain gets created, the dir entry needs to be updated to
                         * not lose the cluster chain.
                         */
                        
                        status = dosfs_file_sync(volume, file, FALSE, FALSE, file->first_clsno, file->length);
                    }
                }
            }
        }
    }

    if (status == F_NO_ERROR)
    {
        file->flags |= DOSFS_FILE_FLAG_DIR_MODIFIED;
        file->last_clsno = clsno_l;
        file->length = length;

        if (file->position > length_o)
        {
            position = length_o;
            offset = (length_o + DOSFS_BLK_MASK) & ~DOSFS_BLK_MASK;
            count  = file->position - length_o;

            if (!(position & volume->cls_mask))
            {
                /* If the old file->length was on a cluster boundary, then the 
                 * old cached file data should be flushed to get a nice sequential
                 * write sequence.
                 */
                status = dosfs_data_cache_flush(volume, file);

                if (status == F_NO_ERROR)
                {
                    clsno = clsno_n;
                }
            }

            if (status == F_NO_ERROR)
            {
                blkno = DOSFS_CLSNO_TO_BLKNO(clsno) + DOSFS_OFFSET_TO_BLKCNT(position);
                blkno_e = DOSFS_CLSNO_TO_BLKNO(clsno) + volume->cls_blk_size;

                if (((position & DOSFS_BLK_MASK) + count) < DOSFS_BLK_SIZE)
                {
                    /* All data is within one block, simply go throu the cache.
                     * If the block is beyond the old file->length, is has to 
                     * be all zeros. Otherwise got throu the normal cache.
                     */
                
                    if (position >= offset)
                    {
                        status = dosfs_data_cache_zero(volume, file, blkno, &entry);
                    }
                    else
                    {
                        status = dosfs_data_cache_read(volume, file, blkno, &entry);
                    
                        if (status == F_NO_ERROR)
                        {
                            memset(entry->data + (position & DOSFS_BLK_MASK), 0, count);
                        }
                    }
                
                    if (status == F_NO_ERROR)
                    {
                        dosfs_data_cache_modify(volume, file);
                    
                        position += count;
                        count = 0;
                    
                        if (!(position & DOSFS_BLK_MASK))
                        {
                            blkno++;
                        }
                    }
                }
                else
                {
                    if (position & DOSFS_BLK_MASK)
                    {
                        size = (DOSFS_BLK_SIZE - (position & DOSFS_BLK_MASK));

                        if (position >= offset)
                        {
                            status = dosfs_data_cache_zero(volume, file, blkno, &entry);
                        }
                        else
                        { 
                            status = dosfs_data_cache_read(volume, file, blkno, &entry);
                        
                            if (status == F_NO_ERROR)
                            {
                                memset(entry->data + (DOSFS_BLK_SIZE - size), 0, size);
                            }
                        }
                    
                        if (status == F_NO_ERROR)
                        {
                            dosfs_data_cache_modify(volume, file);

                            status = dosfs_data_cache_write(volume, file);
                        
                            if (status == F_NO_ERROR)
                            {
                                position += size;
                                count -= size;
                            
                                blkno++;
                            }
                        }
                    }

                    while ((status == F_NO_ERROR) && (count != 0))
                    {
                        if (blkno == blkno_e)
                        {
                            status = dosfs_data_cache_flush(volume, file);

                            if (status == F_NO_ERROR)
                            {
#if (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1)
                                if (file->flags & DOSFS_FILE_FLAG_CONTIGUOUS)
                                {
                                    clsno++;
                                    blkno_e += volume->cls_blk_size;
                                }
                                else
#endif /* (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1) */
                                {
                                    status = dosfs_cluster_chain_seek(volume, clsno, 1, &clsno);
                                        
                                    if (status == F_NO_ERROR)
                                    {
                                        blkno = DOSFS_CLSNO_TO_BLKNO(clsno);
                                        blkno_e = blkno + volume->cls_blk_size;
                                    }
                                }
                            }
                        }
                        
                        if (status == F_NO_ERROR)
                        {
                            if (count < DOSFS_BLK_SIZE)
                            {
                                status = dosfs_data_cache_zero(volume, file, blkno, &entry);
                            
                                if (status == F_NO_ERROR)
                                {
                                    dosfs_data_cache_modify(volume, file);
                                
                                    position += count;
                                    count = 0;
                                }
                            }
                            else
                            {
                                size = volume->cls_size - (position & volume->cls_mask);
                            
                                if (size > count)
                                {
                                    size = count & ~DOSFS_BLK_MASK;
                                }
                            
                                blkcnt = size >> DOSFS_BLK_SHIFT;
                            
                                status = dosfs_data_cache_invalidate(volume, file, blkno, blkcnt);
                            
                                if (status == F_NO_ERROR)
                                {
                                    status = dosfs_volume_zero(volume, blkno, blkcnt, false);

                                    if (status == F_NO_ERROR)
                                    {
                                        position += size;
                                        count -= size;
                                    
                                        blkno += blkcnt;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            if (status == F_NO_ERROR)
            {
                file->clsno = clsno;
                file->blkno = blkno;
                file->blkno_e = blkno_e;
            }
        }
    }

    if (file->status == F_NO_ERROR)
    {
        file->status = status;
    }

    return status;
}

#if (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1)
static int dosfs_file_reserve(dosfs_volume_t *volume, dosfs_file_t *file, uint32_t size)
{
    int status = F_NO_ERROR;
    dosfs_device_t *device;
    uint32_t clsno_a, clscnt;

    device = DOSFS_VOLUME_DEVICE(volume);

    clscnt = DOSFS_SIZE_TO_CLSCNT(size);
    clscnt = ((((clscnt << volume->cls_blk_shift) + (volume->au_size -1)) / volume->au_size) * volume->au_size) >> volume->cls_blk_shift;

#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0)
    status = dosfs_volume_dirty(volume);

    if (status == F_NO_ERROR)
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */
    {
        status = dosfs_cluster_chain_create_contiguous(volume, clscnt, &clsno_a);

        if (status == F_NO_ERROR)
        {
            file->flags |= DOSFS_FILE_FLAG_CONTIGUOUS;
            file->size = size;
            file->first_clsno = clsno_a;
        
            file->clsno = clsno_a;
            file->blkno = DOSFS_CLSNO_TO_BLKNO(clsno_a);
            file->blkno_e = file->blkno + volume->cls_blk_size;

            /* If a new cluster chain gets created, the dir entry needs to be updated to
             * not lose the cluster chain.
             */
            
            status = dosfs_file_sync(volume, file, FALSE, FALSE, file->first_clsno, file->length);

            if (status == F_NO_ERROR)
            {
                status = (device->interface->erase)(device->context, file->blkno, (clscnt << volume->cls_blk_shift));
            }
        }
    }

    return status;
}

#endif /* (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1) */

static int dosfs_file_open(dosfs_volume_t *volume, dosfs_file_t *file, const char *filename, uint32_t mode, uint32_t size)
{
    int status = F_NO_ERROR;
    uint32_t clsno, clsno_d, index, count;
    uint16_t ctime, cdate;
#if (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1)
    uint32_t clscnt, clsdata;
#endif /* (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1) */
    dosfs_file_t *file_o;
    dosfs_dir_t *dir;

    file->mode = 0;
    file->flags = 0;

    if ((mode & DOSFS_FILE_MODE_WRITE) && (volume->flags & DOSFS_VOLUME_FLAG_WRITE_PROTECTED))
    {
        status = F_ERR_WRITEPROTECT;
    }
    else
    {
        status = dosfs_path_find_directory(volume, filename, &filename, &clsno_d);
        
        if (status == F_NO_ERROR)
        {
            status = dosfs_path_convert_filename(volume, filename, NULL);
            
            if (status == F_NO_ERROR)
            {
                if (mode & DOSFS_FILE_MODE_CREATE)
                {
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                    if ((volume->dir.dir_name[0] == '\0') || (volume->dir.dir_nt_reserved & DOSFS_DIR_TYPE_LOSSY))
                    {
                        count = 1 + volume->dir_entries;
                    }
                    else
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                    {
                        count = 1;
                    }
                }
                else
                {
                    count = 0;
                }
                
                status = dosfs_path_find_entry(volume, clsno_d, 0, count, dosfs_path_find_callback_name, NULL, &clsno, &index, &dir);
                
                if (status == F_NO_ERROR)
                {
                    if (dir != NULL)
                    {
                        if (dir->dir_attr & DOSFS_DIR_ATTR_DIRECTORY)
                        {
                            status = F_ERR_INVALIDDIR;
                        }
                        else
                        {
                            if ((mode & DOSFS_FILE_MODE_WRITE) && (dir->dir_attr & DOSFS_DIR_ATTR_READ_ONLY))
                            {
                                status = F_ERR_ACCESSDENIED;
                            }
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                            else
                            {
                                /* Advance to primary dir entry. */
                                index += volume->dir_entries;
                            }
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                            
                            file->dir_clsno = clsno;
                            file->dir_index = index;
                            file->length = DOSFS_FTOHL(dir->dir_file_size);
                            
                            if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
                            {
                                file->first_clsno = ((uint32_t)DOSFS_FTOHS(dir->dir_clsno_hi) << 16) | (uint32_t)DOSFS_FTOHS(dir->dir_clsno_lo);
                            }
                            else
                            {
                                file->first_clsno = (uint32_t)DOSFS_FTOHS(dir->dir_clsno_lo);
                            }
                        }
                    }
                    else
                    {
                        if (mode & DOSFS_FILE_MODE_CREATE)
                        {
#if defined(DOSFS_PORT_CORE_TIMEDATE)
                            DOSFS_PORT_CORE_TIMEDATE(&ctime, &cdate);
#else /* DOSFS_PORT_CORE_TIMEDATE */
                            ctime = 0;
                            cdate = 0;
#endif /* DOSFS_PORT_CORE_TIMEDATE */
                            
                            status = dosfs_path_create_entry(volume, clsno_d, clsno, index, NULL, 0, DOSFS_CLSNO_NONE, ctime, cdate);
                            
                            if (status == F_NO_ERROR)
                            {
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                                /* Stip out allocations bits, and advance to primary dir entry. */
                                index = (index & 0x0000ffff) + (count -1);
#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                                index = (index & 0x0000ffff);
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                                
                                file->dir_clsno = clsno;
                                file->dir_index = index;
                                file->length = 0;
                                file->first_clsno = DOSFS_CLSNO_NONE;
                            }
                        }
                        else
                        {
                            status = F_ERR_NOTFOUND;
                        }
                    }
                    
                    if (status == F_NO_ERROR)
                    {
                        file_o = NULL;
                        
                        do
                        {
                            file_o = dosfs_file_enumerate(volume, file_o, clsno, index);
                            
                            if (file_o != NULL)
                            {
                                if (mode & DOSFS_FILE_MODE_WRITE)
                                {
                                    status = F_ERR_LOCKED;
                                }
                                else
                                {
                                    if (file_o->mode & DOSFS_FILE_MODE_WRITE)
                                    {
                                        status = F_ERR_LOCKED;
                                    }
                                }
                            }
                        }
                        while ((status == F_NO_ERROR) && (file_o != NULL));
                        
                        if (status == F_NO_ERROR)
                        {
                            file->status = F_NO_ERROR;
                            file->position = 0;
                            file->last_clsno = DOSFS_CLSNO_NONE;
                            
                            if (file->first_clsno == DOSFS_CLSNO_NONE)
                            {
                                file->flags |= DOSFS_FILE_FLAG_END_OF_CHAIN;
                                file->clsno = DOSFS_CLSNO_NONE;
                                file->blkno = DOSFS_BLKNO_INVALID;
                                file->blkno_e = DOSFS_BLKNO_INVALID;
                            }
                            else
                            {
                                file->clsno = file->first_clsno;
                                file->blkno = DOSFS_CLSNO_TO_BLKNO(file->clsno);
                                file->blkno_e = file->blkno + volume->cls_blk_size;
                            }
                            
                            if (mode & DOSFS_FILE_MODE_TRUNCATE)
                            {
                                file->length = 0;
                                
                                status = dosfs_file_shrink(volume, file);
                            }
                            
#if (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1)
                            if (size)
                            {
                                if (file->first_clsno == DOSFS_CLSNO_NONE)
                                {
                                    status = dosfs_file_reserve(volume, file, size);
                                }
                                else
                                {
                                    clsno = file->first_clsno;
                                    clscnt = 0;
                                    
                                    do
                                    {
                                        status = dosfs_cluster_read(volume, clsno, &clsdata);
                                        
                                        if (status == F_NO_ERROR)
                                        {
                                            if ((clsdata >= 2) && (clsdata <= volume->last_clsno))
                                            {
                                                if ((clsno +1) == clsdata)
                                                {
                                                    clsno++;
                                                    clscnt++;
                                                }
                                                else
                                                {
                                                    status = F_ERR_EOF;
                                                }
                                                
                                            }
                                            else
                                            {
                                                if (clsdata < DOSFS_CLSNO_LAST)
                                                {
                                                    status = F_ERR_EOF;
                                                }
                                            }
                                        }
                                    }
                                    while ((status == F_NO_ERROR) && (clsdata < DOSFS_CLSNO_LAST));
                                    
                                    if (status == F_NO_ERROR)
                                    {
                                        if (clscnt == DOSFS_SIZE_TO_CLSCNT(size))
                                        {
                                            file->flags |= (DOSFS_FILE_FLAG_CONTIGUOUS | DOSFS_FILE_FLAG_END_OF_CHAIN);
                                            file->last_clsno = file->first_clsno + clscnt -1;

                                            if (size >= (DOSFS_FILE_SIZE_MAX & ~volume->cls_mask))
                                            {
                                                file->size = DOSFS_FILE_SIZE_MAX;
                                            }
                                            else
                                            {
                                                file->size = (size + volume->cls_mask) & ~volume->cls_mask;
                                            }
                                        }
                                        else
                                        {
                                            status = F_ERR_EOF;
                                        }
                                    }
                                }
                            }
#endif /* (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1) */
                            
                            if (status == F_NO_ERROR)
                            {
                                if (mode & DOSFS_FILE_MODE_APPEND)
                                {
                                    status = dosfs_file_seek(volume, file, file->length);
                                }
                                
                                if (status == F_NO_ERROR)
                                {
#if (DOSFS_CONFIG_FILE_DATA_CACHE == 1)
                                    file->data_cache.blkno = DOSFS_BLKNO_INVALID;
#endif /* (DOSFS_CONFIG_FILE_DATA_CACHE == 1) */

                                    file->mode = mode;

                                    file->next = volume->files;

                                    volume->files = file;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return status;
}

static int dosfs_file_close(dosfs_volume_t *volume, dosfs_file_t *file)
{
    int status = F_NO_ERROR;
    dosfs_file_t *file_s, **file_p;


    if (file->mode & DOSFS_FILE_MODE_WRITE)
    {
        status = dosfs_file_flush(volume, file, TRUE);
    }

    for (file_p = &volume->files, file_s = *file_p; file_s; file_p = &file_s->next, file_s = *file_p) 
    {
        if (file == file_s)
        {
            *file_p = file->next;
        }
    }

    file->mode = 0;
    file->dir_clsno = DOSFS_CLSNO_NONE;
    file->dir_index = 0;
    file->next = NULL;

    return status;
}

static int dosfs_file_read(dosfs_volume_t *volume, dosfs_file_t *file, uint8_t *data, uint32_t count, uint32_t *p_count)
{
    int status = F_NO_ERROR;
    dosfs_device_t *device;
    uint32_t blkno, blkno_e, blkcnt, clsno, position, total, size;
    dosfs_cache_entry_t *entry;

    device = DOSFS_VOLUME_DEVICE(volume);

    *p_count = 0;

    if (file->position >= file->length)
    {
        count = 0;
    }
    else
    {
        if (count > (file->length - file->position))
        {
            count = (file->length - file->position);
        }
    }

    if (count != 0)
    {
        total = count;

        position = file->position;
        clsno = file->clsno;
        blkno = file->blkno;
        blkno_e = file->blkno_e;

        /* Take care of the case where there is an empty cluster chain,
         * but file->length is not 0.
         */
        if (clsno == DOSFS_CLSNO_NONE)
        {
            status = F_ERR_EOF;
        }
        else
        {
            if (blkno == blkno_e)
            {
#if (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1)
                if (file->flags & DOSFS_FILE_FLAG_CONTIGUOUS)
                {
                    clsno++;
                    blkno_e += volume->cls_blk_size;
                }
                else
#endif /* (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1) */
                {
                    status = dosfs_cluster_chain_seek(volume, clsno, 1, &clsno);

                    if (status == F_NO_ERROR)
                    {
                        blkno = DOSFS_CLSNO_TO_BLKNO(clsno);
                        blkno_e = blkno + volume->cls_blk_size;
                    }
                }
            }
        }

        if (status == F_NO_ERROR)
        {
            if (((position & DOSFS_BLK_MASK) + count) < DOSFS_BLK_SIZE)
            {
                status = dosfs_data_cache_read(volume, file, blkno, &entry);
            
                if (status == F_NO_ERROR)
                {
                    memcpy(data, entry->data + (position & DOSFS_BLK_MASK), count);

                    position += count;
                    count = 0;

                    if (!(position & DOSFS_BLK_MASK))
                    {
                        blkno++;
                    }
                }
            }
            else
            {
                if (position & DOSFS_BLK_MASK)
                {
                    status = dosfs_data_cache_read(volume, file, blkno, &entry);
            
                    if (status == F_NO_ERROR)
                    {
                        size = (DOSFS_BLK_SIZE - (position & DOSFS_BLK_MASK));

                        memcpy(data, entry->data + (DOSFS_BLK_SIZE - size), size);

                        position += size;
                        data += size;
                        count -= size;

                        blkno++;
                    }
                }

                while ((status == F_NO_ERROR) && (count != 0))
                {
                    if (blkno == blkno_e)
                    {
#if (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1)
                        if (file->flags & DOSFS_FILE_FLAG_CONTIGUOUS)
                        {
                            clsno++;
                            blkno_e += volume->cls_blk_size;
                        }
                        else
#endif /* (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1) */
                        {
                            status = dosfs_cluster_chain_seek(volume, clsno, 1, &clsno);
                            
                            if (status == F_NO_ERROR)
                            {
                                blkno = DOSFS_CLSNO_TO_BLKNO(clsno);
                                blkno_e = blkno + volume->cls_blk_size;
                            }
                        }
                    }
                
                    if (status == F_NO_ERROR)
                    {
                        if (count < DOSFS_BLK_SIZE)
                        {
                            status = dosfs_data_cache_read(volume, file, blkno, &entry);
                    
                            if (status == F_NO_ERROR)
                            {
                                memcpy(data, entry->data, count);

                                position += count;
                                data += count;
                                count = 0;
                            }
                        }
                        else
                        {
                            size = volume->cls_size - (position & volume->cls_mask);
                            
                            if (size > count)
                            {
                                size = count & ~DOSFS_BLK_MASK;
                            }

                            blkcnt = size >> DOSFS_BLK_SHIFT;

                            status = dosfs_data_cache_flush(volume, file);

                            if (status == F_NO_ERROR)
                            {
                                status = (*device->interface->read)(device->context, blkno, data, blkcnt, !!(file->mode & DOSFS_FILE_MODE_SEQUENTIAL));

#if (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1)
                                if (status == F_ERR_INVALIDSECTOR)
                                {
                                    volume->flags |= DOSFS_VOLUME_FLAG_MEDIA_FAILURE;
                                }
#endif /* (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1) */

                                if (status == F_NO_ERROR)
                                {
                                    position += size;
                                    data += size;
                                    count -= size;

                                    blkno += blkcnt;
                                }
                            }
                        }
                    }
                }
            }
        
            if (status == F_NO_ERROR)
            {
                file->position = position;
                file->clsno = clsno;
                file->blkno = blkno;
                file->blkno_e = blkno_e;
            }
        }

        *p_count = total - count;
    }

    if (file->status == F_NO_ERROR)
    {
        file->status = status;
    }

    return status;
}

static int dosfs_file_write(dosfs_volume_t *volume, dosfs_file_t *file, const uint8_t *data, uint32_t count, uint32_t *p_count)
{
    int status = F_NO_ERROR;
    dosfs_device_t *device;
    uint32_t blkno, blkno_e, blkcnt, clsno, offset, position, length, total, size;
    dosfs_cache_entry_t *entry;

    device = DOSFS_VOLUME_DEVICE(volume);

    *p_count = 0;

    if ((file->mode & DOSFS_FILE_MODE_APPEND) && (file->position != file->length))
    {
        status = dosfs_file_seek(volume, file, file->length);
    }

    if (status == F_NO_ERROR)
    {
#if (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1)
        if (file->flags & DOSFS_FILE_FLAG_CONTIGUOUS)
        {
            if (file->size < file->position)
            {
                status = F_ERR_EOF;
            }
            else
            {
                if (count > (file->size - file->position))
                {
                    count = (file->size - file->position);
                }
            }
        }
        else
#endif /* (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1) */
        {
            if (count > (DOSFS_FILE_SIZE_MAX - file->position))
            {
                count = (DOSFS_FILE_SIZE_MAX - file->position);
            }
        }

        if (status == F_NO_ERROR)
        {
            if (count != 0)
            {
                file->flags |= DOSFS_FILE_FLAG_DATA_MODIFIED;

                total  = count;
                offset = (file->length + DOSFS_BLK_MASK) & ~DOSFS_BLK_MASK;
                length = file->position + count;

                if (length > file->length)
                {
                    if ((file->length == 0) || (file->length != file->position) || (((file->length -1) & ~volume->cls_mask) != ((length -1) & ~volume->cls_mask)))
                    {
                        status = dosfs_file_extend(volume, file, length);
                    }
                    else
                    {
                        file->length = length;
                        file->flags |= DOSFS_FILE_FLAG_DIR_MODIFIED;
                    }
                }

                if (status == F_NO_ERROR)
                {
                    position = file->position;
                    clsno = file->clsno;
                    blkno = file->blkno;
                    blkno_e = file->blkno_e;

                    /* Take care of the case where the cluster chain of a file was truncted,
                     * but the length was not adjusted on disk.
                     */
                    if (clsno == DOSFS_CLSNO_NONE)
                    {
                        status = F_ERR_EOF;
                    }
                    else
                    {
                        if (blkno == blkno_e)
                        {
                            status = dosfs_data_cache_flush(volume, file);

                            if (status == F_NO_ERROR)
                            {
#if (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1)
                                if (file->flags & DOSFS_FILE_FLAG_CONTIGUOUS)
                                {
                                    clsno++;
                                    blkno_e += volume->cls_blk_size;
                                }
                                else
#endif /* (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1) */
                                {
                                    status = dosfs_cluster_chain_seek(volume, clsno, 1, &clsno);
                        
                                    if (status == F_NO_ERROR)
                                    {
                                        blkno = DOSFS_CLSNO_TO_BLKNO(clsno);
                                        blkno_e = blkno + volume->cls_blk_size;
                                    }
                                }
                            }
                        }
                    }

                    if (status == F_NO_ERROR)
                    {
                        if (((position & DOSFS_BLK_MASK) + count) < DOSFS_BLK_SIZE)
                        {
                            /* All data is within one block, simply go throu the cache.
                             * If the block is beyond the old file->length, is has to 
                             * be all zeros. Otherwise got throu the normal cache.
                             */

                            if (position >= offset)
                            {
                                status = dosfs_data_cache_zero(volume, file, blkno, &entry);
                            }
                            else
                            {
                                status = dosfs_data_cache_read(volume, file, blkno, &entry);
                            }
            
                            if (status == F_NO_ERROR)
                            {
                                memcpy(entry->data + (position & DOSFS_BLK_MASK), data, count);

                                dosfs_data_cache_modify(volume, file);

                                position += count;
                                count = 0;

                                if (!(position & DOSFS_BLK_MASK))
                                {
                                    blkno++;
                                }
                            }
                        }
                        else
                        {
                            if (position & DOSFS_BLK_MASK)
                            {
                                size = (DOSFS_BLK_SIZE - (position & DOSFS_BLK_MASK));

                                if (position >= offset)
                                {
                                    status = dosfs_data_cache_zero(volume, file, blkno, &entry);
                                }
                                else
                                { 
                                    status = dosfs_data_cache_read(volume, file, blkno, &entry);
                                }

                                if (status == F_NO_ERROR)
                                {
                                    memcpy(entry->data + (DOSFS_BLK_SIZE - size), data, size);
                                
                                    dosfs_data_cache_modify(volume, file);

                                    if (size < count)
                                    {
                                        status = dosfs_data_cache_write(volume, file);
                                    }

                                    if (status == F_NO_ERROR)
                                    {
                                        position += size;
                                        data += size;
                                        count -= size;
                                    
                                        blkno++;
                                    }
                                }
                            }

                            while ((status == F_NO_ERROR) && (count != 0))
                            {
                                if (blkno == blkno_e)
                                {
                                    status = dosfs_data_cache_flush(volume, file);

                                    if (status == F_NO_ERROR)
                                    {
#if (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1)
                                        if (file->flags & DOSFS_FILE_FLAG_CONTIGUOUS)
                                        {
                                            clsno++;
                                            blkno_e += volume->cls_blk_size;
                                        }
                                        else
#endif /* (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1) */
                                        {
                                            status = dosfs_cluster_chain_seek(volume, clsno, 1, &clsno);
                                        
                                            if (status == F_NO_ERROR)
                                            {
                                                blkno = DOSFS_CLSNO_TO_BLKNO(clsno);
                                                blkno_e = blkno + volume->cls_blk_size;
                                            }
                                        }
                                    }
                                }

                                if (status == F_NO_ERROR)
                                {
                                    if (count < DOSFS_BLK_SIZE)
                                    {
                                        if (position >= offset)
                                        {
                                            status = dosfs_data_cache_zero(volume, file, blkno, &entry);
                                        }
                                        else
                                        {
                                            status = dosfs_data_cache_read(volume, file, blkno, &entry);
                                        }

                                        if (status == F_NO_ERROR)
                                        {
                                            memcpy(entry->data, data, count);

                                            dosfs_data_cache_modify(volume, file);

                                            if (status == F_NO_ERROR)
                                            {
                                                position += count;
                                                data += count;
                                                count = 0;
                                            }
                                        }
                                    }
                                    else
                                    {
                                        size = volume->cls_size - (position & volume->cls_mask);
                            
                                        if (size > count)
                                        {
                                            size = count & ~DOSFS_BLK_MASK;
                                        }

                                        blkcnt = size >> DOSFS_BLK_SHIFT;

                                        status = dosfs_data_cache_invalidate(volume, file, blkno, blkcnt);

                                        if (status == F_NO_ERROR)
                                        {
                                            status = (*device->interface->write)(device->context, blkno, data, blkcnt, false);

#if (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1)
                                            if (status == F_ERR_INVALIDSECTOR)
                                            {
                                                volume->flags |= DOSFS_VOLUME_FLAG_MEDIA_FAILURE;
                                            }
#endif /* (DOSFS_CONFIG_MEDIA_FAILURE_SUPPORTED == 1) */

                                            if (status == F_NO_ERROR)
                                            {
                                                position += size;
                                                data += size;
                                                count -= size;
                                            
                                                blkno += blkcnt;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }

                    if (status == F_NO_ERROR)
                    {
                        if (file->mode & DOSFS_FILE_MODE_RANDOM)
                        {
                            status = (*device->interface->sync)(device->context);
                        }

                        if (status == F_NO_ERROR)
                        {
                            file->position = position;
                            file->clsno = clsno;
                            file->blkno = blkno;
                            file->blkno_e = blkno_e;
                        }
                    }
                }

                if (total != count)
                {
                    device->lock |= DOSFS_DEVICE_LOCK_MODIFIED;
                }

                *p_count = total - count;
            }
        }
    }

    if (file->status == F_NO_ERROR)
    {
        file->status = status;
    }

    return status;
}

/***********************************************************************************************************************/

int f_initvolume(void)
{
    int status = F_NO_ERROR;
    dosfs_volume_t *volume;
    dosfs_device_t *device; 
    uint32_t o_lock, n_lock;

    volume = DOSFS_DEFAULT_VOLUME();
    device = DOSFS_VOLUME_DEVICE(volume);

    while (1)
    {
        o_lock = device->lock;
        
        if (!(o_lock & (DOSFS_DEVICE_LOCK_SCSI | DOSFS_DEVICE_LOCK_MEDIUM | DOSFS_DEVICE_LOCK_INIT)))
        {
            n_lock = o_lock | DOSFS_DEVICE_LOCK_VOLUME;
            
            if (armv6m_atomic_compare_and_swap(&device->lock, o_lock, n_lock) == o_lock)
            {
                break;
            }
        }
        
        __WFE();
    }

    status = dosfs_volume_lock_noinit(volume);
    
    if (status == F_NO_ERROR)
    {
        status = dosfs_volume_init(volume, device);
        
        status = dosfs_volume_unlock(volume, status);
    }

    if (status != F_NO_ERROR)
    {
        armv6m_atomic_and(&device->lock, ~DOSFS_DEVICE_LOCK_VOLUME);
    }

    return status;
}

int f_delvolume(void)
{
    int status = F_NO_ERROR;
    dosfs_volume_t *volume;
    dosfs_device_t *device;

    volume = DOSFS_DEFAULT_VOLUME();

    status = dosfs_volume_lock_nomount(volume);
    device = DOSFS_VOLUME_DEVICE(volume);
    
    if (status == F_NO_ERROR)
    {
        status = dosfs_volume_unmount(volume);

        if (status == F_NO_ERROR)
        {
            status = (*device->interface->release)(device->context);

            if (status == F_NO_ERROR)
            {
                volume->state = DOSFS_VOLUME_STATE_INITIALIZED;
            }
        }
        
        status = dosfs_volume_unlock(volume, status);
    }

    if (status == F_NO_ERROR)
    {
        armv6m_atomic_and(&device->lock, ~DOSFS_DEVICE_LOCK_VOLUME);
    }

    return status;
}

int f_checkvolume(void)
{
    int status = F_NO_ERROR;
    dosfs_volume_t *volume;
    dosfs_device_t *device;

    volume = DOSFS_DEFAULT_VOLUME();
    device = DOSFS_VOLUME_DEVICE(volume);

    status = dosfs_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        status = (*device->interface->sync)(device->context);

        status = dosfs_volume_unlock(volume, status);
    }

    return status;
}

int f_format(int fattype)
{
    int status = F_NO_ERROR;
    dosfs_volume_t *volume;

    volume = DOSFS_DEFAULT_VOLUME();

    status = dosfs_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        if (volume->flags & DOSFS_VOLUME_FLAG_WRITE_PROTECTED)
        {
            status = F_ERR_WRITEPROTECT;
        }
        else
        {
            status = dosfs_volume_unmount(volume);

            if (status == F_NO_ERROR)
            {
                status = dosfs_volume_format(volume);
            }
        }

        status = dosfs_volume_unlock(volume, status);
    }

    return status;
}


int f_hardformat(int fattype)
{
    int status = F_NO_ERROR;
    dosfs_volume_t *volume;
    dosfs_device_t *device;

    volume = DOSFS_DEFAULT_VOLUME();
    device = DOSFS_VOLUME_DEVICE(volume);

    status = dosfs_volume_lock_nomount(volume);
    
    if (status == F_NO_ERROR)
    {
        status = dosfs_volume_unmount(volume);

        if (status == F_NO_ERROR)
        {
            volume->dir_cache.blkno = DOSFS_BLKNO_INVALID;

            status = dosfs_device_format(device, volume->dir_cache.data);
        }
        
        status = dosfs_volume_unlock(volume, status);
    }

    return status;
}


int f_getfreespace(F_SPACE *pspace)
{
    int status = F_NO_ERROR;
    uint32_t clsno, clsno_e, clscnt_total, clscnt_free, clsdata;
    dosfs_volume_t *volume;

    volume = DOSFS_DEFAULT_VOLUME();

    status = dosfs_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
#if (DOSFS_CONFIG_FSINFO_SUPPORTED == 1)
        if (volume->flags & DOSFS_VOLUME_FLAG_FSINFO_VALID)
        {
            clscnt_total = volume->last_clsno - 1;
            clscnt_free = volume->free_clscnt;
        }
        else
#endif /* (DOSFS_CONFIG_FSINFO_SUPPORTED == 1) */
        {
            clscnt_total = volume->last_clsno - 1;
            clscnt_free = 0;

            for (clsno = 2, clsno_e = volume->last_clsno; ((status == F_NO_ERROR) && (clsno <= clsno_e)); clsno++)
            {
                /* Bypass cluster cache on read while scanning.
                 */
                status = dosfs_cluster_read_uncached(volume, clsno, &clsdata);
                
                if (status == F_NO_ERROR)
                {
                    if (clsdata == DOSFS_CLSNO_FREE)
                    {
                        clscnt_free++;
                    }
                }
            }

#if (DOSFS_CONFIG_FSINFO_SUPPORTED == 1)
            if (status == F_NO_ERROR)
            {
                volume->flags |= (DOSFS_VOLUME_FLAG_FSINFO_VALID | DOSFS_VOLUME_FLAG_FSINFO_DIRTY);
                volume->free_clscnt = clscnt_free;

                if ((volume->state == DOSFS_VOLUME_STATE_MOUNTED) &&
                    (volume->fsinfo_blkofs != 0))
                {
#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)
                    /* If a map/fat entry has changed, this triggers a record/commit sequence.
                     * Otherwise the DOSFS_VOLUME_FLAG_FSINFO_DIRTY setting will simply update
                     * the FSINFO on disk.
                     */
                    status = dosfs_volume_record(volume);

#else /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */

#if (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1)
                    /* volume->free_clscnt is only maintained in FSINFO if DOSFS_VOLUME_FLAG_MOUNTED_DIRTY
                     * is not set. If DOSFS_VOLUME_FLAG_VOLUME_DIRTY is set, then the volume is in a dirty
                     * state, where volume->free_clscnt cannot be updated. In this case wait for the update
                     * till the next dosfs_volume_clean() gets called.
                     */
                    if (!(volume->flags & (DOSFS_VOLUME_FLAG_VOLUME_DIRTY | DOSFS_VOLUME_FLAG_MOUNTED_DIRTY)))
                    {
                        status = dosfs_fat_cache_flush(volume);
                    
                        if (status == F_NO_ERROR)
                        {
                            status = dosfs_volume_fsinfo(volume, volume->free_clscnt, volume->next_clsno);
                            
                            if (status == F_NO_ERROR)
                            {
                                volume->flags &= ~DOSFS_VOLUME_FLAG_FSINFO_DIRTY;
                            }
                        }
                    }
#endif /* (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1) */
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */
                }
            }
#endif /* (DOSFS_CONFIG_FSINFO_SUPPORTED == 1) */
        }

        if (status == F_NO_ERROR)
        {
            pspace->total      = clscnt_total << volume->cls_shift;
            pspace->free       = clscnt_free  << volume->cls_shift;
            pspace->used       = (clscnt_total - clscnt_free) << volume->cls_shift;
            pspace->bad        = 0;
            pspace->total_high = clscnt_total >> (32 - volume->cls_shift);
            pspace->free_high  = clscnt_free  >> (32 - volume->cls_shift);
            pspace->used_high  = (clscnt_total - clscnt_free) >> (32 - volume->cls_shift);
            pspace->bad_high   = 0;
        }

        status = dosfs_volume_unlock(volume, status);
    }

    return status;
}

int f_getserial(unsigned long *p_serial)
{
    int status = F_NO_ERROR;
    dosfs_volume_t *volume;

    volume = DOSFS_DEFAULT_VOLUME();
    
    status = dosfs_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        *p_serial = volume->serial;

        status = dosfs_volume_unlock(volume, status);
    }

    return status;
}


int f_setlabel(const char *volname)
{
    int status = F_NO_ERROR;
    uint32_t clsno, index;
    uint16_t ctime, cdate;
    dosfs_dir_t *dir;
    dosfs_volume_t *volume;

    volume = DOSFS_DEFAULT_VOLUME();
    
    status = dosfs_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        if (volume->flags & DOSFS_VOLUME_FLAG_WRITE_PROTECTED)
        {
            status = F_ERR_WRITEPROTECT;
        }
        else
        {
            volname = dosfs_name_cstring_to_label(volname, volume->dir.dir_name);

            if (volname != NULL)
            {
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                volume->dir.dir_nt_reserved = 0x00;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */

                status = dosfs_path_find_entry(volume, DOSFS_CLSNO_NONE, 0, 1, dosfs_path_find_callback_volume, NULL, &clsno, &index, &dir);

                if (status == F_NO_ERROR)
                {
#if defined(DOSFS_PORT_CORE_TIMEDATE)
                    DOSFS_PORT_CORE_TIMEDATE(&ctime, &cdate);
#else /* DOSFS_PORT_CORE_TIMEDATE */
                    ctime = 0;
                    cdate = 0;
#endif /* DOSFS_PORT_CORE_TIMEDATE */

                    if (dir == NULL)
                    {
#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0)
                        status = dosfs_volume_dirty(volume);

                        if (status == F_NO_ERROR)
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */
                        {
                            status = dosfs_path_create_entry(volume, DOSFS_CLSNO_NONE, clsno, index, (const char*)volume->dir.dir_name, DOSFS_DIR_ATTR_VOLUME_ID, DOSFS_CLSNO_NONE, ctime, cdate);
                        
#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0)
                            status = dosfs_volume_clean(volume, status);
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */
                        }
                    }
                    else
                    {
                        memcpy(dir->dir_name, volume->dir.dir_name, sizeof(dir->dir_name));

                        dir->dir_wrt_time = DOSFS_HTOFS(ctime);
                        dir->dir_wrt_date = DOSFS_HTOFS(cdate);

                        status = dosfs_dir_cache_write(volume);
                    }
                }
            }
            else
            {
                status = F_ERR_INVALIDNAME;
            }
        }

        status = dosfs_volume_unlock(volume, status);
    }

    return status;
}

int f_getlabel(char *volname, int length)
{
    int status = F_NO_ERROR;
    const uint8_t *dosname;
    char *volname_e;
    dosfs_dir_t *dir;
    dosfs_boot_t *boot;
    dosfs_volume_t *volume;

    volume = DOSFS_DEFAULT_VOLUME();
    
    status = dosfs_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        dosname = NULL;

        status = dosfs_path_find_entry(volume, DOSFS_CLSNO_NONE, 0, 0, dosfs_path_find_callback_volume, NULL, NULL, NULL, &dir);

        if (status == F_NO_ERROR)
        {
            volname_e = volname + length;

            if (dir == NULL)
            {
#if (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1) || (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)
                boot = (dosfs_boot_t*)((void*)&volume->bs_data[0]);
#else /* (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1) || (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */

                boot = (dosfs_boot_t*)((void*)volume->dir_cache.data);

                status = dosfs_dir_cache_flush(volume);
                
                if (status == F_NO_ERROR)
                {
                    status = dosfs_volume_read(volume, volume->boot_blkno, (uint8_t*)boot);
                }

                if (status == F_NO_ERROR)
#endif /* (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1) || (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */
                {
                    if (volume->type != DOSFS_VOLUME_TYPE_FAT32)
                    {
                        if (boot->bpb40.bs_boot_sig == 0x29)
                        {
                            dosname = (const uint8_t*)boot->bpb40.bs_vol_lab;
                        }
                    }
                    else
                    {
                        if (boot->bpb71.bs_boot_sig == 0x29)
                        {
                            dosname = (const uint8_t*)boot->bpb71.bs_vol_lab;
                        }
                    }
                }
            }
            else
            {
                dosname = (const uint8_t*)dir->dir_name;
            }
        }

        if (status == F_NO_ERROR)
        {
            if (dosname != NULL)
            {
                volname = dosfs_name_label_to_cstring(dosname, volname, volname_e);

                if (volname == NULL)
                {
                    status = F_ERR_TOOLONGNAME;
                }
            }
            else
            {
                status = F_ERR_NOTFOUND;
            }
        }

        status = dosfs_volume_unlock(volume, status);
    }

    return status;
}


int f_mkdir(const char *dirname)
{
    int status = F_NO_ERROR;
    uint16_t ctime, cdate;
    uint32_t clsno, clsno_d, clsno_s, blkno_s, index, count;
    dosfs_dir_t *dir;
    dosfs_cache_entry_t *entry;
    dosfs_volume_t *volume;

    volume = DOSFS_PATH_VOLUME(dirname);
    
    status = dosfs_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        if (volume->flags & DOSFS_VOLUME_FLAG_WRITE_PROTECTED)
        {
            status = F_ERR_WRITEPROTECT;
        }
        else
        {
            if (((dirname[0] == '/') || (dirname[0] == '\\')) && (dirname[1] == '\0'))
            {
                status = F_ERR_INVALIDDIR;
            }
            else
            {
                status = dosfs_path_find_directory(volume, dirname, &dirname, &clsno_d);
            
                if (status == F_NO_ERROR)
                {
                    status = dosfs_path_convert_filename(volume, dirname, NULL);

                    if (status == F_NO_ERROR)
                    {
                        if (volume->dir.dir_name[0] == '.')
                        {
                            status = F_ERR_NOTFOUND;
                        }
                        else
                        {
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                            if ((volume->dir.dir_name[0] == '\0') || (volume->dir.dir_nt_reserved & DOSFS_DIR_TYPE_LOSSY))
                            {
                                count = 1 + volume->dir_entries;
                            }
                            else
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                            {
                                count = 1;
                            }

                            status = dosfs_path_find_entry(volume, clsno_d, 0, count, dosfs_path_find_callback_name, NULL, &clsno, &index, &dir);
                        
                            if (status == F_NO_ERROR)
                            {
                                if (dir == NULL)
                                {
                                    /* No conflicting entry found, hence start out creating the subdirectory.
                                     *
                                     * The strategy is to first allocate the cluster for the subdirectory, populate it
                                     * and then create the new directory entry in the parent. This way there is no
                                     * inconsistent file system state along the way.
                                     */

#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0)
                                    status = dosfs_volume_dirty(volume);

                                    if (status == F_NO_ERROR)
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */
                                    {
                                        status = dosfs_cluster_chain_create(volume, DOSFS_CLSNO_NONE, 1, &clsno_s, NULL);
                                
                                        if (status == F_NO_ERROR)
                                        {
                                            blkno_s = DOSFS_CLSNO_TO_BLKNO(clsno_s);
                                    
                                            status = dosfs_volume_zero(volume, blkno_s, volume->cls_blk_size, true);

                                            if (status == F_NO_ERROR)
                                            {
                                                status = dosfs_dir_cache_zero(volume, blkno_s, &entry);

                                                if (status == F_NO_ERROR)
                                                {
#if defined(DOSFS_PORT_CORE_TIMEDATE)
                                                    DOSFS_PORT_CORE_TIMEDATE(&ctime, &cdate);
#else /* DOSFS_PORT_CORE_TIMEDATE */
                                                    ctime = 0;
                                                    cdate = 0;
#endif /* DOSFS_PORT_CORE_TIMEDATE */
                                                
                                                    dosfs_path_setup_entry(volume, dosfs_dirname_dot, DOSFS_DIR_ATTR_DIRECTORY, clsno_s, ctime, cdate, ((dosfs_dir_t*)((void*)entry->data)));
                                                    dosfs_path_setup_entry(volume, dosfs_dirname_dotdot, DOSFS_DIR_ATTR_DIRECTORY, clsno_d, ctime, cdate, ((dosfs_dir_t*)((void*)(entry->data + sizeof(dosfs_dir_t)))));
                                                
                                                    status = dosfs_dir_cache_write(volume);
                                                
                                                    if (status == F_NO_ERROR)
                                                    {
                                                        /* Subdirectory has been created, so back to hooking up the directory entry.
                                                         */
                                                    
                                                        status = dosfs_path_create_entry(volume, clsno_d, clsno, index, NULL, DOSFS_DIR_ATTR_DIRECTORY, clsno_s, ctime, cdate);

                                                        if (status == F_ERR_NOMOREENTRY)
                                                        {
                                                            dosfs_cluster_chain_destroy(volume, clsno_s, DOSFS_CLSNO_FREE);
                                                        }
                                                    }
                                                }
                                            }
                                        }

#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0)
                                        status = dosfs_volume_clean(volume, status);
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */
                                    }
                                }
                                else
                                {
                                    status = F_ERR_DUPLICATED;
                                }
                            }
                        }
                    }
                }
            }
        }

        status = dosfs_volume_unlock(volume, status);
    }

    return status;
}

int f_rmdir(const char *dirname)
{
    int status = F_NO_ERROR;
    uint32_t clsno, clsno_d, index, first_clsno;
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
    unsigned int entries;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
    dosfs_dir_t *dir;
    dosfs_volume_t *volume;

    volume = DOSFS_PATH_VOLUME(dirname);
    
    status = dosfs_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        if (volume->flags & DOSFS_VOLUME_FLAG_WRITE_PROTECTED)
        {
            status = F_ERR_WRITEPROTECT;
        }
        else
        {
            if (((dirname[0] == '/') || (dirname[0] == '\\')) && (dirname[1] == '\0'))
            {
                status = F_ERR_INVALIDDIR;
            }
            else
            {
                status = dosfs_path_find_directory(volume, dirname, &dirname, &clsno_d);
            
                if (status == F_NO_ERROR)
                {
                    status = dosfs_path_convert_filename(volume, dirname, NULL);

                    if (status == F_NO_ERROR)
                    {
                        if (volume->dir.dir_name[0] == '.')
                        {
                            status = F_ERR_NOTFOUND;
                        }
                        else
                        {
                            // ### FIXME ... filter out "/"
                            status = dosfs_path_find_entry(volume, clsno_d, 0, 0, dosfs_path_find_callback_name, NULL, &clsno, &index, &dir);
                        
                            if (status == F_NO_ERROR)
                            {
                                if (dir != NULL)
                                {
                                    if (dir->dir_attr & DOSFS_DIR_ATTR_DIRECTORY)
                                    {
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                                        /* dosfs_path_find_callback_name() fills in the proper volume->dir_entries on a match.
                                         */
                                        entries = volume->dir_entries;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */

                                        if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
                                        {
                                            first_clsno = ((uint32_t)DOSFS_FTOHS(dir->dir_clsno_hi) << 16) | (uint32_t)DOSFS_FTOHS(dir->dir_clsno_lo);
                                        }
                                        else
                                        { 
                                            first_clsno = (uint32_t)DOSFS_FTOHS(dir->dir_clsno_lo);
                                        }

                                        /* Skip "." and ".."
                                         */
                                        status = dosfs_path_find_entry(volume, first_clsno, 2, 0, dosfs_path_find_callback_empty, NULL, NULL, NULL, &dir);

                                        if (status == F_NO_ERROR)
                                        {
                                            if (dir == NULL)
                                            {
#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0)
                                                status = dosfs_volume_dirty(volume);

                                                if (status == F_NO_ERROR)
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */
                                                {
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                                                    status = dosfs_path_destroy_entry(volume, clsno, index, entries, first_clsno);
#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                                                    status = dosfs_path_destroy_entry(volume, clsno, index, first_clsno);
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                                            
                                                    if (status == F_NO_ERROR)
                                                    {
                                                        if (volume->cwd_clsno == first_clsno)
                                                        {
                                                            volume->cwd_clsno = DOSFS_CLSNO_END_OF_CHAIN;
                                                        }
                                                    }

                                            
#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0)
                                                    status = dosfs_volume_clean(volume, status);
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */
                                                }
                                            }
                                            else
                                            {
                                                status = F_ERR_NOTEMPTY;
                                            }
                                        }
                                    }
                                    else
                                    {
                                        status = F_ERR_INVALIDDIR;
                                    }
                                }
                                else
                                {
                                    status = F_ERR_NOTFOUND;
                                }
                            }
                        }
                    }
                }
            }
        }

        status = dosfs_volume_unlock(volume, status);
    }

    return status;
}

int f_chdir(const char *dirname)
{
    int status = F_NO_ERROR;
    uint32_t clsno_d, blkno_d;
    dosfs_cache_entry_t *entry;
    dosfs_dir_t *dir;
    dosfs_volume_t *volume;

    volume = DOSFS_PATH_VOLUME(dirname);

    status = dosfs_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        if (((dirname[0] == '/') || (dirname[0] == '\\')) && (dirname[1] == '\0'))
        {
            /* dosfs_path_find_directory cannot return a vaild "dir" for the "/",
             * so special code this here.
             */
            volume->cwd_clsno = DOSFS_CLSNO_NONE;
        }
        else
        {
            status = dosfs_path_find_directory(volume, dirname, &dirname, &clsno_d);
            
            if (status == F_NO_ERROR)
            {
                status = dosfs_path_convert_filename(volume, dirname, NULL);
                
                if (status == F_NO_ERROR)
                {
                    if (volume->dir.dir_name[0] == '.')
                    {
                        if (volume->dir.dir_name[1] == ' ')
                        {
                            /* "." */
                            volume->cwd_clsno = clsno_d;
                        }
                        else
                        {
                            /* ".." */

                            if (clsno_d != DOSFS_CLSNO_NONE)
                            {
                                blkno_d = DOSFS_CLSNO_TO_BLKNO(clsno_d);

                                status = dosfs_dir_cache_read(volume, blkno_d, &entry);
                                                
                                if (status == F_NO_ERROR)
                                {
                                    dir = (dosfs_dir_t*)((void*)(entry->data + sizeof(dosfs_dir_t)));
                                
                                    if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
                                    {
                                        volume->cwd_clsno = ((uint32_t)DOSFS_FTOHS(dir->dir_clsno_hi) << 16) | (uint32_t)DOSFS_FTOHS(dir->dir_clsno_lo);
                                    }
                                    else
                                    {
                                        volume->cwd_clsno = (uint32_t)DOSFS_FTOHS(dir->dir_clsno_lo);
                                    }
                                }
                            }
                            else
                            {
                                status = F_ERR_NOTFOUND;
                            }
                        }
                    }
                    else
                    {
                        status = dosfs_path_find_entry(volume, clsno_d, 0, 0, dosfs_path_find_callback_name, NULL, NULL, NULL, &dir);

                        if (status == F_NO_ERROR)
                        {
                            if (dir != NULL)
                            {
                                if (dir->dir_attr & DOSFS_DIR_ATTR_DIRECTORY)
                                {
                                    if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
                                    {
                                        volume->cwd_clsno = ((uint32_t)DOSFS_FTOHS(dir->dir_clsno_hi) << 16) | (uint32_t)DOSFS_FTOHS(dir->dir_clsno_lo);
                                    }
                                    else
                                    {
                                        volume->cwd_clsno = (uint32_t)DOSFS_FTOHS(dir->dir_clsno_lo);
                                    }
                                }
                                else
                                {
                                    status = F_ERR_INVALIDDIR;
                                }
                            }
                            else
                            {
                                status = F_ERR_NOTFOUND;
                            }
                        }
                    }
                }
            }
        }

        status = dosfs_volume_unlock(volume, status);
    }

    return status;
}

int f_getcwd(char *dirname, int length)
{
    int status = F_NO_ERROR;
    uint32_t clsno_s, clsno_p, clsno_g, blkno, index;
    char *dirname_e;
    dosfs_cache_entry_t *entry;
    dosfs_dir_t *dir;
    dosfs_volume_t *volume;
    
    volume = DOSFS_DEFAULT_VOLUME();

    status = dosfs_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        if (length == 1)
        {
            status = F_ERR_TOOLONGNAME;
        }
        else
        {
            clsno_s = volume->cwd_clsno;
        
            index = length -1;
            
            if (clsno_s != DOSFS_CLSNO_NONE)
            {
                blkno = volume->cls_blk_offset + (clsno_s << volume->cls_blk_shift);
                    
                status = dosfs_dir_cache_read(volume, blkno, &entry);

                if (status == F_NO_ERROR)
                {
                    dir = (dosfs_dir_t*)((void*)(entry->data + sizeof(dosfs_dir_t)));
                    
                    if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
                    {
                        clsno_p = ((uint32_t)DOSFS_FTOHS(dir->dir_clsno_hi) << 16) | (uint32_t)DOSFS_FTOHS(dir->dir_clsno_lo);
                    }
                    else
                    {
                        clsno_p = (uint32_t)DOSFS_FTOHS(dir->dir_clsno_lo);
                    }
                    
                    do
                    {
                        if (clsno_p != DOSFS_CLSNO_NONE)
                        {
                            blkno = volume->cls_blk_offset + (clsno_p << volume->cls_blk_shift);
                            
                            status = dosfs_dir_cache_read(volume, blkno, &entry);
                            
                            if (status == F_NO_ERROR)
                            {
                                dir = (dosfs_dir_t*)((void*)(entry->data + sizeof(dosfs_dir_t)));
                                
                                if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
                                {
                                    clsno_g = ((uint32_t)DOSFS_FTOHS(dir->dir_clsno_hi) << 16) | (uint32_t)DOSFS_FTOHS(dir->dir_clsno_lo);
                                }
                                else
                                {
                                    clsno_g = (uint32_t)DOSFS_FTOHS(dir->dir_clsno_lo);
                                }
                            }
                        }
                        else
                        {
                            clsno_g = DOSFS_CLSNO_NONE;
                        }

                        if (status == F_NO_ERROR)
                        {
                            status = dosfs_path_find_entry(volume, clsno_p, 0, 0, dosfs_path_find_callback_directory, (void*)clsno_s, NULL, NULL, &dir);

                            if (status == F_NO_ERROR)
                            {
                                if (dir != NULL)
                                {
                                    if (clsno_s != volume->cwd_clsno)
                                    {
                                        if (index != 0)
                                        {
                                            dirname[index--] = F_SEPARATORCHAR;
                                        }
                                        else
                                        {
                                            status = F_ERR_TOOLONGNAME;
                                        }
                                    }
                                    
                                    if (status == F_NO_ERROR)
                                    {
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                                        /* It's possible to have a sfn_name only ("." and ".." for example).
                                         */
                                        if (volume->lfn_count)
                                        {
                                            dirname_e = dosfs_name_uniname_to_cstring(volume->lfn_name, volume->lfn_count, dirname, dirname + index);
                                        }
                                        else
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                                        {
                                            dirname_e = dosfs_name_dosname_to_cstring(dir->dir_name, dir->dir_nt_reserved, dirname, dirname + index);
                                        }
                                        
                                        if (dirname_e != NULL)
                                        {
                                            index -= (dirname_e - dirname -1);
                                            
                                            memmove(&dirname[index+1], &dirname[0], (dirname_e - dirname -1));
                                            
                                            clsno_s = clsno_p;
                                            clsno_p = clsno_g;
                                        }
                                        else
                                        {
                                            status = F_ERR_TOOLONGNAME;
                                        }
                                    }
                                }
                                else
                                {
                                    status = F_ERR_NOTFOUND;
                                }
                            }
                        }
                    }
                    while ((status == F_NO_ERROR) && (clsno_s != DOSFS_CLSNO_NONE));
                }
            }
                
            if (status == F_NO_ERROR)
            {
                if (index >= 2)
                {
                    dirname[index] = F_SEPARATORCHAR;
                    
                    memmove(&dirname[0], &dirname[index], (length - index));
                    
                    dirname[length - index] = '\0';
                }
                else
                {
                    status = F_ERR_TOOLONGNAME;
                }
            }
        }

        status = dosfs_volume_unlock(volume, status);
    }
        
    return status;
}

int f_opendir(F_DIR *dir, const char *dirname)
{
    int status = F_NO_ERROR;
    uint32_t clsno_d;
    dosfs_volume_t *volume;
    dosfs_dir_t *dir_e;

    dir->dot_clsno = DOSFS_CLSNO_END_OF_CHAIN;
    dir->dir_clsno = DOSFS_CLSNO_END_OF_CHAIN;
    dir->dir_index = 0;

    volume = DOSFS_PATH_VOLUME(dirname);
    
    status = dosfs_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        if (((dirname[0] == '/') || (dirname[0] == '\\')) && (dirname[1] == '\0'))
        {
            clsno_d = DOSFS_CLSNO_NONE;
        }
        else
        {
            status = dosfs_path_find_file(volume, dirname, NULL, NULL, &dir_e);

            if (status == F_NO_ERROR)
            {
                if (dir_e->dir_attr & DOSFS_DIR_ATTR_DIRECTORY)
                {
                    if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
                    {
                        clsno_d = ((uint32_t)DOSFS_FTOHS(dir_e->dir_clsno_hi) << 16) | (uint32_t)DOSFS_FTOHS(dir_e->dir_clsno_lo);
                    }
                    else
                    {
                        clsno_d = (uint32_t)DOSFS_FTOHS(dir_e->dir_clsno_lo);
                    }
                }
                else
                {
                    status = F_ERR_INVALIDDIR;
                }
            }
        }

        if (status == F_NO_ERROR)
        {
            dir->dir_clsno = dir->dot_clsno = clsno_d;
        }
    }

    return status;
}

int f_readdir(F_DIR *dir, F_DIRENT *dirent)
{
    int status = F_NO_ERROR;
    dosfs_volume_t *volume;
    dosfs_dir_t *dir_e;

    volume = DOSFS_FIND_VOLUME(find);

    status = dosfs_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        if (dir->dir_clsno == DOSFS_CLSNO_END_OF_CHAIN)
        {
            status = F_ERR_NOTFOUND;
        }
        else
        {
            status = dosfs_path_find_pattern(volume, dir, NULL, dirent->filename, &dir_e);

            if (status == F_NO_ERROR)
            {
                dirent->attr = dir_e->dir_attr;
                dirent->filesize = DOSFS_FTOHL(dir_e->dir_file_size);
            }
        }

        status = dosfs_volume_unlock(volume, status);
    }

    return status;
}

int f_rewinddir(F_DIR *dir)
{
    int status = F_NO_ERROR;

    if (dir->dot_clsno == DOSFS_CLSNO_END_OF_CHAIN)
    {
        status = F_ERR_INVALIDDIR;
    }
    else
    {
        dir->dir_clsno = dir->dot_clsno;
        dir->dir_index = 0;
    }

    return status;
}

int f_closedir(F_DIR *dir)
{
    int status = F_NO_ERROR;

    dir->dot_clsno = DOSFS_CLSNO_END_OF_CHAIN;
    dir->dir_clsno = DOSFS_CLSNO_END_OF_CHAIN;
    dir->dir_index = 0;

    return status;
}


int f_rename(const char *filename, const char *newname)
{
    int status = F_NO_ERROR;
    uint32_t blkno, clsno_d, clsno_o, index_o;
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
    uint32_t clsno, index, entries_o, count;
    uint8_t nt_reserved_o;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
    dosfs_dir_t *dir;
    dosfs_cache_entry_t *entry;
    dosfs_volume_t *volume;

    volume = DOSFS_PATH_VOLUME(filename);

    status = dosfs_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        if (volume->flags & DOSFS_VOLUME_FLAG_WRITE_PROTECTED)
        {
            status = F_ERR_WRITEPROTECT;
        }
        else
        {
            status = dosfs_path_find_directory(volume, filename, &filename, &clsno_d);
            
            if (status == F_NO_ERROR)
            {
                status = dosfs_path_convert_filename(volume, filename, NULL);
            
                if (status == F_NO_ERROR)
                {
                    status = dosfs_path_find_entry(volume, clsno_d, 0, 0, dosfs_path_find_callback_name, NULL, &clsno_o, &index_o, &dir);

                    if (status == F_NO_ERROR)
                    {
                        if (dir != NULL)
                        {
                            if (dir->dir_attr & DOSFS_DIR_ATTR_READ_ONLY)
                            {
                                status = F_ERR_ACCESSDENIED;
                            }
                            else
                            {
                                if (dosfs_file_enumerate(volume, NULL, clsno_o, index_o))
                                {
                                    status = F_ERR_LOCKED;
                                }
                                else
                                {
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                                    memcpy(&volume->dir, dir, sizeof(dosfs_dir_t));

                                    nt_reserved_o = dir->dir_nt_reserved;

                                    /* dosfs_path_find_callback_name() fills in the proper volume->dir_entries on a match.
                                     */
                                    entries_o = volume->dir_entries;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */

                                    status = dosfs_path_convert_filename(volume, newname, NULL);
                                
                                    if (status == F_NO_ERROR)
                                    {
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                                        if ((volume->dir.dir_name[0] == '\0') || (volume->dir.dir_nt_reserved & DOSFS_DIR_TYPE_LOSSY))
                                        {
                                            count = 1 + volume->dir_entries;
                                        }
                                        else
                                        {
                                            count = 1;
                                        }

                                        status = dosfs_path_find_entry(volume, clsno_d, 0, count, dosfs_path_find_callback_name, NULL, &clsno, &index, &dir);
#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                                        status = dosfs_path_find_entry(volume, clsno_d, 0, 0, dosfs_path_find_callback_name, NULL, NULL, NULL, &dir);
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                                    
                                        if (status == F_NO_ERROR)
                                        {
                                            if (dir != NULL)
                                            {
                                                status = F_ERR_DUPLICATED;
                                            }
                                            else
                                            {
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                                                if ((entries_o != 0) || (count != 1))
                                                {
#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)
                                                    /* For TRANSACTION_SAFE, the DOSFS_DIR_FLAG_DESTROY_ENTRY operation is recorded first, and
                                                     * the dosfs_path_create_entry() gets called to add the DOSFS_DIR_FLAG_CREATE_ENTRY part.
                                                     * If that fails somehow, then the pending operations are canceled.
                                                     */
                                                    volume->dir_flags |= DOSFS_DIR_FLAG_DESTROY_ENTRY;
                                                    volume->del_clsno = clsno_o;
                                                    volume->del_index = index_o;
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                                                    volume->del_entries = entries_o;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */

                                                    /* The call to dosfs_path_create_entry() is special. The DOSFS_CLUSTER_END_OF_CHAIN tells it
                                                     * use the existing volume->dir as a template, and to overlay "attr" onto dir_nt_reserved.
                                                     */
                                                    status = dosfs_path_create_entry(volume, clsno_d, clsno, index, NULL, nt_reserved_o, DOSFS_CLSNO_END_OF_CHAIN, 0, 0);
                                                
                                                    if (status != F_NO_ERROR)
                                                    {
                                                        volume->dir_flags &= ~(DOSFS_DIR_FLAG_DESTROY_ENTRY | DOSFS_DIR_FLAG_CREATE_ENTRY);
                                                    }

#else /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */
                                                    status = dosfs_volume_dirty(volume);

                                                    if (status == F_NO_ERROR)
                                                    {
                                                        /* The call to dosfs_path_create_entry() is special. The DOSFS_CLUSTER_END_OF_CHAIN tells it
                                                         * use the existing volume->dir as a template, and to overlay "attr" onto dir_nt_reserved.
                                                         */

                                                        status = dosfs_path_create_entry(volume, clsno_d, clsno, index, NULL, nt_reserved_o, DOSFS_CLSNO_END_OF_CHAIN, 0, 0);
                                                    
                                                        if (status == F_NO_ERROR)
                                                        {
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                                                            status = dosfs_path_destroy_entry(volume, clsno_o, index_o, entries_o, DOSFS_CLSNO_NONE);
#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                                                            status = dosfs_path_destroy_entry(volume, clsno_o, index_o, DOSFS_CLUSTER_NONE);
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                                                        }

                                                        status = dosfs_volume_clean(volume, status);
                                                    }
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */
                                                }
                                                else
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                                                {
                                                    /* Nothing special to do here for TRANSACTION_SAFE/VOLUME_DIRTY/FSINFO. The dir entry
                                                     * update is atomic and not dependent upon the FAT.
                                                     */

                                                    if (clsno_o == DOSFS_CLSNO_NONE)
                                                    {
                                                        blkno = volume->root_blkno + DOSFS_INDEX_TO_BLKCNT_ROOT(index_o);
                                                    }
                                                    else
                                                    {
                                                        blkno = DOSFS_CLSNO_TO_BLKNO(clsno_o) + DOSFS_INDEX_TO_BLKCNT(index_o);
                                                    }
                                                
                                                    status = dosfs_dir_cache_read(volume, blkno, &entry);
                                                
                                                    if (status == F_NO_ERROR)
                                                    {
                                                        dir = (dosfs_dir_t*)((void*)(entry->data + DOSFS_INDEX_TO_BLKOFS(index_o)));
                                                    
                                                        memcpy(dir->dir_name, volume->dir.dir_name, 11);
                                                    
                                                        dir->dir_nt_reserved = volume->dir.dir_nt_reserved;
                                                    
                                                        status = dosfs_dir_cache_write(volume);
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        else
                        {
                            status = F_ERR_NOTFOUND;
                        }
                    }
                }
            }
        }

        status = dosfs_volume_unlock(volume, status);
    }

    return status;
}


int f_move(const char *filename, const char *newname)
{
    int status = F_NO_ERROR;
    uint32_t blkno, clsno_d, clsno_o, index_o, clsno_s, clsno, index;
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
    uint32_t entries_o, count;
    uint8_t nt_reserved_o;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
    dosfs_dir_t *dir;
    dosfs_cache_entry_t *entry;
    dosfs_volume_t *volume;

    volume = DOSFS_PATH_VOLUME(filename);

    status = dosfs_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        if (volume->flags & DOSFS_VOLUME_FLAG_WRITE_PROTECTED)
        {
            status = F_ERR_WRITEPROTECT;
        }
        else
        {
            status = dosfs_path_find_directory(volume, filename, &filename, &clsno_d);
            
            if (status == F_NO_ERROR)
            {
                status = dosfs_path_convert_filename(volume, filename, NULL);
            
                if (status == F_NO_ERROR)
                {
                    status = dosfs_path_find_entry(volume, clsno_d, 0, 0, dosfs_path_find_callback_name, NULL, &clsno_o, &index_o, &dir);

                    if (status == F_NO_ERROR)
                    {
                        if (dir != NULL)
                        {
                            if (dir->dir_attr & DOSFS_DIR_ATTR_READ_ONLY)
                            {
                                status = F_ERR_ACCESSDENIED;
                            }
                            else
                            {
                                if (dosfs_file_enumerate(volume, NULL, clsno_o, index_o))
                                {
                                    status = F_ERR_LOCKED;
                                }
                                else
                                {
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                                    memcpy(&volume->dir, dir, sizeof(dosfs_dir_t));

                                    nt_reserved_o = dir->dir_nt_reserved;

                                    /* dosfs_path_find_callback_name() fills in the proper volume->dir_entries on a match.
                                     */
                                    entries_o = volume->dir_entries;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */

                                    status = dosfs_path_find_directory(volume, newname, &newname, &clsno_d);
            
                                    if (status == F_NO_ERROR)
                                    {
                                        status = dosfs_path_convert_filename(volume, newname, NULL);
                                        
                                        if (status == F_NO_ERROR)
                                        {
                                            status = dosfs_path_convert_filename(volume, newname, NULL);
                                
                                            if (status == F_NO_ERROR)
                                            {
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                                                if ((volume->dir.dir_name[0] == '\0') || (volume->dir.dir_nt_reserved & DOSFS_DIR_TYPE_LOSSY))
                                                {
                                                    count = 1 + volume->dir_entries;
                                                }
                                                else
                                                {
                                                    count = 1;
                                                }

                                                status = dosfs_path_find_entry(volume, clsno_d, 0, count, dosfs_path_find_callback_name, NULL, &clsno, &index, &dir);
#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                                                status = dosfs_path_find_entry(volume, clsno_d, 0, 0, dosfs_path_find_callback_name, NULL, &clsno, &index, &dir);
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                                    
                                                if (status == F_NO_ERROR)
                                                {
                                                    if (dir != NULL)
                                                    {
                                                        status = F_ERR_DUPLICATED;
                                                    }
                                                    else
                                                    {
#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)
                                                        /* For TRANSACTION_SAFE, the DOSFS_DIR_FLAG_DESTROY_ENTRY operation is recorded first, and
                                                         * the dosfs_path_create_entry() gets called to add the DOSFS_DIR_FLAG_CREATE_ENTRY part.
                                                         * If that fails somehow, then the pending operations are canceled.
                                                         */
                                                        volume->dir_flags |= DOSFS_DIR_FLAG_DESTROY_ENTRY;
                                                        volume->del_clsno = clsno_o;
                                                        volume->del_index = index_o;
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                                                        volume->del_entries = entries_o;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */

                                                        /* The call to dosfs_path_create_entry() is special. The DOSFS_CLUSTER_END_OF_CHAIN tells it
                                                         * use the existing volume->dir as a template, and to overlay "attr" onto dir_nt_reserved.
                                                         */
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                                                        status = dosfs_path_create_entry(volume, clsno_d, clsno, index, NULL, nt_reserved_o, DOSFS_CLSNO_END_OF_CHAIN, 0, 0);
#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                                                        status = dosfs_path_create_entry(volume, clsno_d, clsno, index, NULL, 0, DOSFS_CLSNO_END_OF_CHAIN, 0, 0);
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                                                        
                                                        if (status != F_NO_ERROR)
                                                        {
                                                            volume->dir_flags &= ~(DOSFS_DIR_FLAG_DESTROY_ENTRY | DOSFS_DIR_FLAG_CREATE_ENTRY);
                                                        }

#else /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */
                                                        status = dosfs_volume_dirty(volume);

                                                        if (status == F_NO_ERROR)
                                                        {
                                                            /* The call to dosfs_path_create_entry() is special. The DOSFS_CLUSTER_END_OF_CHAIN tells it
                                                             * use the existing volume->dir as a template, and to overlay "attr" onto dir_nt_reserved.
                                                             */
                                                            
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                                                            status = dosfs_path_create_entry(volume, clsno_d, clsno, index, NULL, nt_reserved_o, DOSFS_CLSNO_END_OF_CHAIN, 0, 0);
#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                                                            status = dosfs_path_create_entry(volume, clsno_d, clsno, index, NULL, 0, DOSFS_CLSNO_END_OF_CHAIN, 0, 0);
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */

                                                            if (status == F_NO_ERROR)
                                                            {
                                                                if (volume->dir.dir_attr & DOSFS_DIR_ATTR_DIRECTORY)
                                                                {
                                                                    if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
                                                                    {
                                                                        clsno_s = ((uint32_t)DOSFS_FTOHS(volume->dir.dir_clsno_hi) << 16) | (uint32_t)DOSFS_FTOHS(volume->dir.dir_clsno_lo);
                                                                    }
                                                                    else
                                                                    {
                                                                        clsno_s = (uint32_t)DOSFS_FTOHS(volume->dir.dir_clsno_lo);
                                                                    }

                                                                    blkno = DOSFS_CLSNO_TO_BLKNO(clsno_s);
                                                                        
                                                                    status = dosfs_dir_cache_read(volume, blkno, &entry);
                                                                    
                                                                    if (status == F_NO_ERROR)
                                                                    {
                                                                        dir = (dosfs_dir_t*)((void*)(entry->data + DOSFS_INDEX_TO_BLKOFS(1)));

                                                                        dir->dir_clsno_lo = clsno_d;

                                                                        if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
                                                                        {
                                                                            dir->dir_clsno_hi = clsno_d >> 16;
                                                                        }

                                                                        status = dosfs_dir_cache_write(volume);
                                                                    }
                                                                }
                                                            }

                                                            if (status == F_NO_ERROR)
                                                            {
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                                                                status = dosfs_path_destroy_entry(volume, clsno_o, index_o, entries_o, DOSFS_CLSNO_NONE);
#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                                                                status = dosfs_path_destroy_entry(volume, clsno_o, index_o, DOSFS_CLSNO_NONE);
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                                                            }
                                                            
                                                            status = dosfs_volume_clean(volume, status);
                                                        }
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        else
                        {
                            status = F_ERR_NOTFOUND;
                        }
                    }
                }
            }
        }

        status = dosfs_volume_unlock(volume, status);
    }

    return status;
}


int f_delete(const char *filename)
{
    int status = F_NO_ERROR;
    uint32_t clsno, index, first_clsno;
    dosfs_dir_t *dir;
    dosfs_volume_t *volume;

    volume = DOSFS_PATH_VOLUME(filename);

    status = dosfs_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        if (volume->flags & DOSFS_VOLUME_FLAG_WRITE_PROTECTED)
        {
            status = F_ERR_WRITEPROTECT;
        }
        else
        {
            status = dosfs_path_find_file(volume, filename, &clsno, &index, &dir);
            
            if (status == F_NO_ERROR)
            {
                if (dir->dir_attr & DOSFS_DIR_ATTR_DIRECTORY)
                {
                    status = F_ERR_NOTFOUND;
                }
                else if (dir->dir_attr & DOSFS_DIR_ATTR_READ_ONLY)
                {
                    status = F_ERR_ACCESSDENIED;
                }
                else
                {
                    if (dosfs_file_enumerate(volume, NULL, clsno, index))
                    {
                        status = F_ERR_LOCKED;
                    }
                    else
                    {
                        if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
                        {
                            first_clsno = ((uint32_t)DOSFS_FTOHS(dir->dir_clsno_hi) << 16) | (uint32_t)DOSFS_FTOHS(dir->dir_clsno_lo);
                        }
                        else
                        { 
                            first_clsno = (uint32_t)DOSFS_FTOHS(dir->dir_clsno_lo);
                        }

#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0)
                        status = dosfs_volume_dirty(volume);
                    
                        if (status == F_NO_ERROR)
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */
                        {
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
                            /* dosfs_path_find_file() fills in the proper volume->dir_entries on a match.
                             */
                            status = dosfs_path_destroy_entry(volume, clsno, index, volume->dir_entries, first_clsno);
#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
                            status = dosfs_path_destroy_entry(volume, clsno, index, first_clsno);
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */

#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0)
                            status = dosfs_volume_clean(volume, status);
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 0) */
                        }
                    }
                }
            }
        }

        status = dosfs_volume_unlock(volume, status);
    }

    return status;
}

long f_filelength(const char *filename)
{
    int status = F_NO_ERROR;
    uint32_t length;
    dosfs_dir_t *dir;
    dosfs_volume_t *volume;

    length = 0;

    volume = DOSFS_PATH_VOLUME(filename);

    status = dosfs_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
      status = dosfs_path_find_file(volume, filename, NULL, NULL, &dir);
            
        if (status == F_NO_ERROR)
        {
            length = dir->dir_file_size;
        }

        status = dosfs_volume_unlock(volume, status);
    }

    return (long)length;
}

int f_findfirst(const char *filename, F_FIND *find)
{
    int status = F_NO_ERROR;
    dosfs_volume_t *volume;

    find->find_dir.dot_clsno = DOSFS_CLSNO_END_OF_CHAIN;
    find->find_dir.dir_clsno = DOSFS_CLSNO_END_OF_CHAIN;
    find->find_dir.dir_index = 0;

    volume = DOSFS_PATH_VOLUME(filename);

    status = dosfs_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        status = dosfs_path_find_directory(volume, filename, &filename, &find->find_dir.dot_clsno);
            
        if (status == F_NO_ERROR)
        {
            find->find_dir.dir_clsno = find->find_dir.dot_clsno;

#if (DOSFS_CONFIG_VFAT_SUPPORTED == 0)

            /* For non-VFAT, build a MSDOS style template where '*' is expanded into
             * a sequence of '?'.
             */

            filename = dosfs_name_cstring_to_pattern(filename, find->find_pattern);

            if (filename == NULL)
            {
                status = F_ERR_INVALIDNAME;
            }

#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 0) */

            /* Copy the remaining filename into find_pattern, and do a WINNT compatible
             * pattern match based upon the incoming lfn_name, or the converted sfn_name.
             */

            char c, *pattern, *pattern_e;
            
            pattern = &find->find_pattern[0];
            pattern_e = &find->find_pattern[F_MAXPATH];
            
            do
            {
                if (pattern == pattern_e)
                {
                    status = F_ERR_TOOLONGNAME;
                }
                else
                {
                    c = *filename++;
                    
                    *pattern++ = c;
                }
            }
            while ((status == F_NO_ERROR) && (c != '\0'));
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 0) */
        }

        if (status != F_NO_ERROR)
        {
            find->find_dir.dir_clsno = DOSFS_CLSNO_END_OF_CHAIN;
        }

        status = dosfs_volume_unlock(volume, status);
    }

    if (status == F_NO_ERROR)
    {
        status = f_findnext(find);
    }

    return status;
}

int f_findnext(F_FIND *find)
{
    int status = F_NO_ERROR;
    dosfs_volume_t *volume;
    dosfs_dir_t *dir_e;

    volume = DOSFS_FIND_VOLUME(find);

    status = dosfs_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        if (find->find_dir.dir_clsno == DOSFS_CLSNO_END_OF_CHAIN)
        {
            status = F_ERR_NOTFOUND;
        }
        else
        {
            status = dosfs_path_find_pattern(volume, &find->find_dir, find->find_pattern, find->filename, &dir_e);

            if (status == F_NO_ERROR)
            {
                memcpy(&find->name[0], &dir_e->dir_name[0], (F_MAXNAME+F_MAXEXT));
                
                find->attr = dir_e->dir_attr;
                find->ctime = DOSFS_FTOHS(dir_e->dir_wrt_time);
                find->cdate = DOSFS_FTOHS(dir_e->dir_wrt_date);
                find->filesize = DOSFS_FTOHL(dir_e->dir_file_size);

                if (volume->type == DOSFS_VOLUME_TYPE_FAT32)
                {
                    find->cluster = ((uint32_t)DOSFS_FTOHS(dir_e->dir_clsno_hi) << 16) | (uint32_t)DOSFS_FTOHS(dir_e->dir_clsno_lo);
                }
                else
                {
                    find->cluster = (uint32_t)DOSFS_FTOHS(dir_e->dir_clsno_lo);
                }
            }
        }

        status = dosfs_volume_unlock(volume, status);
    }

    return status;
}


int f_settimedate(const char *filename, unsigned short ctime, unsigned short cdate)
{
    int status = F_NO_ERROR;
    dosfs_dir_t *dir;
    dosfs_volume_t *volume;

    volume = DOSFS_PATH_VOLUME(filename);

    status = dosfs_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        if (volume->flags & DOSFS_VOLUME_FLAG_WRITE_PROTECTED)
        {
            status = F_ERR_WRITEPROTECT;
        }
        else
        {
            status = dosfs_path_find_file(volume, filename, NULL, NULL, &dir);
        
            if (status == F_NO_ERROR)
            {
                /* Nothing special to do here for TRANSACTION_SAFE/VOLUME_DIRTY/FSINFO. The dir entry
                 * update is atomic and not dependent upon the FAT.
                 */
                dir->dir_wrt_time = DOSFS_HTOFS(ctime);
                dir->dir_wrt_date = DOSFS_HTOFS(cdate);
            
                status = dosfs_dir_cache_write(volume);
            }
        }

        status = dosfs_volume_unlock(volume, status);
    }

    return status;
}

int f_gettimedate(const char *filename, unsigned short *p_ctime, unsigned short *p_cdate)
{
    int status = F_NO_ERROR;
    dosfs_dir_t *dir;
    dosfs_volume_t *volume;

    volume = DOSFS_PATH_VOLUME(filename);

    status = dosfs_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        status = dosfs_path_find_file(volume, filename, NULL, NULL, &dir);
        
        if (status == F_NO_ERROR)
        {
            *p_ctime = DOSFS_FTOHS(dir->dir_wrt_time);
            *p_cdate = DOSFS_FTOHS(dir->dir_wrt_date);
        }

        status = dosfs_volume_unlock(volume, status);
    }

    return status;
}

int f_setattr(const char *filename, unsigned char attr)
{
    int status = F_NO_ERROR;
    dosfs_dir_t *dir;
    dosfs_volume_t *volume;

    volume = DOSFS_PATH_VOLUME(filename);

    status = dosfs_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        if (volume->flags & DOSFS_VOLUME_FLAG_WRITE_PROTECTED)
        {
            status = F_ERR_WRITEPROTECT;
        }
        else
        {
            status = dosfs_path_find_file(volume, filename, NULL, NULL, &dir);
        
            if (status == F_NO_ERROR)
            {
                /* Nothing special to do here for TRANSACTION_SAFE/VOLUME_DIRT/FSINFO. The dir entry
                 * update is atomic and not dependent upon the FAT.
                 */
                dir->dir_attr = ((dir->dir_attr & ~(F_ATTR_ARC | F_ATTR_SYSTEM | F_ATTR_HIDDEN | F_ATTR_READONLY)) |
                                 (attr & (F_ATTR_ARC | F_ATTR_SYSTEM | F_ATTR_HIDDEN | F_ATTR_READONLY)));
            
                status = dosfs_dir_cache_write(volume);
            }
        }

        status = dosfs_volume_unlock(volume, status);
    }

    return status;
}

int f_getattr(const char *filename, unsigned char *p_attr)
{
    int status = F_NO_ERROR;
    dosfs_dir_t *dir;
    dosfs_volume_t *volume;

    volume = DOSFS_PATH_VOLUME(filename);

    status = dosfs_volume_lock(volume);
    
    if (status == F_NO_ERROR)
    {
        status = dosfs_path_find_file(volume, filename, NULL, NULL, &dir);
        
        if (status == F_NO_ERROR)
        {
            *p_attr = dir->dir_attr;
        }

        status = dosfs_volume_unlock(volume, status);
    }

    return status;
}


int f_open(F_FILE *file, const char *filename, const char *type)
{
    int status = F_NO_ERROR;
    int c;
    uint32_t mode, size;
    dosfs_volume_t *volume;

    mode = 0;
    size = 0;

    while ((status == F_NO_ERROR) && (c = *type++))
    {
        if ((c == 'r') || (c == 'w') || (c == 'a'))
        {
            if (!(mode & (DOSFS_FILE_MODE_READ | DOSFS_FILE_MODE_WRITE)))
            {
                if (c == 'r')
                {
                    mode = DOSFS_FILE_MODE_READ;
                }
                else if (c == 'w')
                {
                    mode = DOSFS_FILE_MODE_WRITE | DOSFS_FILE_MODE_CREATE | DOSFS_FILE_MODE_TRUNCATE;
                }
                else
                {
                    mode = DOSFS_FILE_MODE_WRITE | DOSFS_FILE_MODE_CREATE | DOSFS_FILE_MODE_APPEND;
                }
                
                if (*type == '+')
                {
                    mode |= (DOSFS_FILE_MODE_READ | DOSFS_FILE_MODE_WRITE);
                    type++;
                }
            }
            else
            {
                status = F_ERR_NOTUSEABLE;
            }
        }
        else if (c == 'S')
        {
            mode |= DOSFS_FILE_MODE_SEQUENTIAL;
        }
        else if (c == 'R')
        {
            mode |= DOSFS_FILE_MODE_RANDOM;
        }
#if (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1)
        else if ((c == ',') && (*type != '\0'))
        {
            /* ",<size>". Make sure that <size> does not overflow
             * DOSFS_FILE_SIZE_MAX.
             */
            while ((status == F_NO_ERROR) && (*type != '\0'))
            {
                c = *type++;

                if ((c >= '0') && (c <= '9'))
                {
                    if (size >= (DOSFS_FILE_SIZE_MAX / 10u))
                    {
                        size = DOSFS_FILE_SIZE_MAX;
                    }
                    else
                    {
                        size = size * 10u;

                        if (size >= (DOSFS_FILE_SIZE_MAX - (c - '0')))
                        {
                            size = DOSFS_FILE_SIZE_MAX;
                        }
                        else
                        {
                            size += (c - '0');
                        }
                    }
                }
                else if (*type == '\0')
                {
                    if (size >= DOSFS_FILE_SIZE_MAX)
                    {
                        size = DOSFS_FILE_SIZE_MAX;
                    }
                }
                else
                {
                    status = F_ERR_NOTUSEABLE;
                }
            }
        }
#endif /* (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1) */
        else
        {
            status = F_ERR_NOTUSEABLE;
        }
    }

    if ((status == F_NO_ERROR) && (mode & (DOSFS_FILE_MODE_READ | DOSFS_FILE_MODE_WRITE)))
    {
        volume = DOSFS_PATH_VOLUME(filename);
      
        status = dosfs_volume_lock(volume);
    
        if (status == F_NO_ERROR)
        {
            status = dosfs_file_open(volume, file, filename, mode, size);

            status = dosfs_volume_unlock(volume, status);
        }
    }
    else
    {
        status = F_ERR_NOTUSEABLE;
    }

    return status;
}

int f_close(F_FILE *file)
{
    int status = F_NO_ERROR;
    dosfs_volume_t *volume;

    if (!file || !file->mode)
    {
        status = F_ERR_NOTOPEN;
    }
    else
    {
        /* There is no check for "file->status" here, as this
         * is handled in dosfs_file_close(), so that the meta
         * data gets updated, while the file data is treated
         * as optional.
         */

        volume = DOSFS_FILE_VOLUME(file);
    
        status = dosfs_volume_lock(volume);
    
        if (status == F_NO_ERROR)
        {
            status = dosfs_file_close(volume, file);

            status = dosfs_volume_unlock(volume, status);
        }
    }

    return status;
}

int f_flush(F_FILE *file)
{
    int status = F_NO_ERROR;
    dosfs_volume_t *volume;

    if (!file || !file->mode)
    {
        status = F_ERR_NOTOPEN;
    }
    else
    {
        status = file->status;
        
        if (status == F_NO_ERROR)
        {
            if (file->mode & DOSFS_FILE_MODE_WRITE)
            {
                volume = DOSFS_FILE_VOLUME(file);
            
                status = dosfs_volume_lock(volume);
                
                if (status == F_NO_ERROR)
                {
                    status = dosfs_file_flush(volume, file, FALSE);
                    
                    status = dosfs_volume_unlock(volume, status);
                }
            }
        }
    }

    return status;
}

long f_write(const void *buffer, long size, long count, F_FILE *file)
{
    int status = F_NO_ERROR;
    long result = 0;
    uint32_t total;
    dosfs_volume_t *volume;

    if (!file || !file->mode)
    {
        status = F_ERR_NOTOPEN;
    }
    else
    {
        if (!(file->mode & DOSFS_FILE_MODE_WRITE))
        {
            status = F_ERR_ACCESSDENIED;
        }
        else
        {
            status = file->status;
            
            if (status == F_NO_ERROR)
            {
                if ((size > 0) && (count > 0))
                {
                    volume = DOSFS_FILE_VOLUME(file);
                    
                    status = dosfs_volume_lock(volume);
                    
                    if (status == F_NO_ERROR)
                    {
                        status = dosfs_file_write(volume, file, (const uint8_t*)buffer, (unsigned long)count * (unsigned long)size, &total);

                        result = total / (unsigned long)size;
                        
                        status = dosfs_volume_unlock(volume, status);
                    }
                }
            }
        }
    }

    return result;
}

long f_read(void *buffer, long size, long count, F_FILE *file)
{
    int status = F_NO_ERROR;
    long result = 0;
    uint32_t total;
    dosfs_volume_t *volume;

    if (!file || !file->mode)
    {
        status = F_ERR_NOTOPEN;
    }
    else
    {
        if (!(file->mode & DOSFS_FILE_MODE_READ))
        {
            status = F_ERR_ACCESSDENIED;
        }
        else
        {
            status = file->status;
                
            if (status == F_NO_ERROR)
            {
                if ((size > 0) && (count > 0))
                {
                    volume = DOSFS_FILE_VOLUME(file);
                    
                    status = dosfs_volume_lock(volume);
                    
                    if (status == F_NO_ERROR)
                    {
                        status = dosfs_file_read(volume, file, (uint8_t*)buffer, (unsigned long)count * (unsigned long)size, &total);

                        result = total / (unsigned long)size;

                        status = dosfs_volume_unlock(volume, status);
                    }
                }
            }
        }
    }

    return result;
}

int f_seek(F_FILE *file, long offset, int whence)
{
    int status = F_NO_ERROR;
    uint32_t position;
    dosfs_volume_t *volume;

    if (!file || !file->mode)
    {
        status = F_ERR_NOTOPEN;
    }
    else
    {
        status = file->status;

        if (status == F_NO_ERROR)
        {
            switch (whence) {
            case F_SEEK_CUR:
                if ((offset >= 0)
                    ? ((uint32_t)offset > (DOSFS_FILE_SIZE_MAX - file->position))
                    : ((uint32_t)(0 - offset) > file->position))
                {
                    status = F_ERR_NOTUSEABLE;
                }
                else
                {
                    position = file->position + offset;
                }
                break;

            case F_SEEK_END:
                if ((offset >= 0)
                    ? ((uint32_t)offset > (DOSFS_FILE_SIZE_MAX - file->length))
                    : ((uint32_t)(0 - offset) > file->length))
                {
                    status = F_ERR_NOTUSEABLE;
                }
                else
                {
                    position = file->length + offset;
                }
                break;

            case F_SEEK_SET:
                if (offset < 0)
                {
                    status = F_ERR_NOTUSEABLE;
                }
                else
                {
                    position = offset;
                }
                break;

            default:
                status = F_ERR_NOTUSEABLE;
                break;
            }
            
            if (status == F_NO_ERROR)
            {
                volume = DOSFS_FILE_VOLUME(file);

                status = dosfs_volume_lock(volume);
    
                if (status == F_NO_ERROR)
                {
                    status = dosfs_file_seek(volume, file, position);

                    status = dosfs_volume_unlock(volume, status);
                }
            }
        }
    }

    return status;
}

long f_tell(F_FILE *file)
{
    int status = F_NO_ERROR;
    uint32_t position;
    
    if (!file || !file->mode)
    {
        status = F_ERR_NOTOPEN;
    }
    else
    {
        status = file->status;
        position = file->position;
    }

    return (status == F_NO_ERROR) ? (long)position : -1;
}

long f_length(F_FILE *file)
{
    int status = F_NO_ERROR;
    uint32_t length;
    
    if (!file || !file->mode)
    {
        status = F_ERR_NOTOPEN;
    }
    else
    {
        status = file->status;
        length = file->length;
    }

    return (status == F_NO_ERROR) ? (long)length : -1;
}

int f_eof(F_FILE *file)
{
    return (!file || !file->mode || (file->position >= file->length));
}

int f_error(F_FILE *file)
{
    int status = F_NO_ERROR;

    if (!file || !file->mode)
    {
        status = F_ERR_NOTOPEN;
    }
    else
    {
        status = file->status;
    }

    return status;
}

int f_rewind(F_FILE *file)
{
    int status = F_NO_ERROR;
    dosfs_volume_t *volume;
    dosfs_device_t *device;

    if (!file || !file->mode)
    {
        status = F_ERR_NOTOPEN;
    }
    else
    {
        volume = DOSFS_FILE_VOLUME(file);
        device = DOSFS_VOLUME_DEVICE(volume);

        status = dosfs_volume_lock(volume);
    
        if (status == F_NO_ERROR)
        {
            /* Call (*device->interface->sync)() to collect all outstanding asynchronous
             * errors. If that succeeds, clear the error status for the file
             * and seek to the beginning of the time.
             */

            status = dosfs_data_cache_flush(volume, file);

            if (status == F_NO_ERROR)
            {
                status = (*device->interface->sync)(device->context);

                if (status == F_NO_ERROR)
                {
                    file->status = F_NO_ERROR;
                    
                    status = dosfs_file_seek(volume, file, 0);
                }
            }

            status = dosfs_volume_unlock(volume, status);
        }
    }

    return status;
}

int f_putc(int c, F_FILE *file)
{
    int status = F_NO_ERROR;
    int result = -1;
    uint8_t data;
    uint32_t total;
    dosfs_cache_entry_t *entry;
    dosfs_volume_t *volume;

    if (!file || !file->mode)
    {
        status = F_ERR_NOTOPEN;
    }
    else
    {
        if (!(file->mode & DOSFS_FILE_MODE_WRITE))
        {
            status = F_ERR_ACCESSDENIED;
        }
        else
        {
            status = file->status;

            if (status == F_NO_ERROR)
            {
                volume = DOSFS_FILE_VOLUME(file);

#if (DOSFS_CONFIG_FILE_DATA_CACHE == 1)
                entry = &file->data_cache;
#else /* (DOSFS_CONFIG_FILE_DATA_CACHE) */
                entry = &volume->dir_cache;
#endif /* (DOSFS_CONFIG_FILE_DATA_CACHE) */

                if ((entry->blkno == file->blkno) && ((file->mode & DOSFS_FILE_MODE_APPEND) ? (file->position == file->length) : (file->position <= file->length)))
                {
                    file->flags |= DOSFS_FILE_FLAG_DATA_MODIFIED;

                    *(entry->data + (file->position & DOSFS_BLK_MASK)) = c;

                    dosfs_data_cache_modify(volume, file);
                
                    file->position++;
                
                    if (!(file->position & DOSFS_BLK_MASK))
                    {
                        file->blkno++;
                    }
                
                    if (file->position >= file->length)
                    {
                        file->length = file->position;
                    }

                    result = c;
                }
                else
                {
                    status = dosfs_volume_lock(volume);
                
                    if (status == F_NO_ERROR)
                    {
                        data = c;
                
                        status = dosfs_file_write(volume, file, &data, 1, &total);

                        if (status == F_NO_ERROR)
                        {
                            if (total == 1)
                            {
                                result = c;
                            }
                        }

                        status = dosfs_volume_unlock(volume, status);
                    }
                }
            }
        }
    }

    return result;
}

int f_getc(F_FILE *file)
{
    int status = F_NO_ERROR;
    int result = -1;
    uint8_t data;
    uint32_t total;
    dosfs_cache_entry_t *entry;
    dosfs_volume_t *volume;

    if (!file || !file->mode)
    {
        status = F_ERR_NOTOPEN;
    }
    else
    {
        if (!(file->mode & DOSFS_FILE_MODE_READ))
        {
            status = F_ERR_ACCESSDENIED;
        }
        else
        {
            status = file->status;

            if (status == F_NO_ERROR)
            {
                volume = DOSFS_FILE_VOLUME(file);

#if (DOSFS_CONFIG_FILE_DATA_CACHE == 1)
                entry = &file->data_cache;
#else /* (DOSFS_CONFIG_FILE_DATA_CACHE) */
                entry = &volume->dir_cache;
#endif /* (DOSFS_CONFIG_FILE_DATA_CACHE) */
                    
                if ((entry->blkno == file->blkno) && (file->position < file->length))
                {
                    result = *(entry->data + (file->position & DOSFS_BLK_MASK));
                        
                    file->position++;
                        
                    if (!(file->position & DOSFS_BLK_MASK))
                    {
                        file->blkno++;
                    }
                }
                else
                {
                    status = dosfs_volume_lock(volume);
                        
                    if (status == F_NO_ERROR)
                    {
                        status = dosfs_file_read(volume, file, &data, 1, &total);
                            
                        if (total == 1)
                        {
                            result = data;
                        }
                            
                        status = dosfs_volume_unlock(volume, status);
                    }
                }
            }
        }
    }

    return result;
}

int f_seteof(F_FILE *file)
{
    int status = F_NO_ERROR;
    dosfs_volume_t *volume;

    if (!file || !file->mode)
    {
        status = F_ERR_NOTOPEN;
    }
    else
    {
        status = file->status;
        
        if (status == F_NO_ERROR)
        {
            volume = DOSFS_FILE_VOLUME(file);

            status = dosfs_volume_lock(volume);
        
            if (status == F_NO_ERROR)
            {
                /* Allow always a truncation, to deal with the case where there
                 * was a file->length but no cluster allocated.
                 */
                if ((file->position == 0) || (file->position < file->length))
                {
                    status = dosfs_file_shrink(volume, file);
                }
                else
                {
                    if ((file->first_clsno == DOSFS_CLSNO_NONE) || (file->length < file->position))
                    {
#if (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1)
                        if (file->flags & DOSFS_FILE_FLAG_CONTIGUOUS)
                        {
                            if (file->size < file->position)
                            {
                                status = F_ERR_EOF;
                            }
                        }

                        if (status == F_NO_ERROR)
#endif /* (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1) */
                        {
                            status = dosfs_file_extend(volume, file, file->position);
                        }
                    }
                }
                
                status = dosfs_volume_unlock(volume, status);
            }
        }
    }

    return status;
}

int f_truncate(F_FILE *file, const char *filename, long length)
{
    int status = F_NO_ERROR;

    status = f_open(file, filename, "r+");

    if (status == F_NO_ERROR)
    {
        status = f_seek(file, length, F_SEEK_SET);

        if (status == F_NO_ERROR)
        {
            status = f_seteof(file);
        }
        
        if (status != F_NO_ERROR)
        {
            f_close(file);
        }
    }

    return status;
}
