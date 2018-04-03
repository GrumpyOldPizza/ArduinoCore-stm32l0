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

#if !defined(_DOSFS_CORE_H)
#define _DOSFS_CORE_H

#include "dosfs_device.h"
//#include "dosfs_port.h"

#ifdef __cplusplus
 extern "C" {
#endif

typedef union  _dosfs_boot_t          dosfs_boot_t;
typedef struct _dosfs_fsinfo_t        dosfs_fsinfo_t;
typedef struct _dosfs_dir_t           dosfs_dir_t;
typedef struct _dosfs_ldir_t          dosfs_ldir_t;
typedef struct _dosfs_file_t          dosfs_file_t;
typedef struct _dosfs_cache_entry_t   dosfs_cache_entry_t;
typedef struct _dosfs_cluster_entry_t dosfs_cluster_entry_t;
typedef struct _dosfs_volume_t        dosfs_volume_t;

#if (DOSFS_CONFIG_VFAT_SUPPORTED == 0)

typedef int (*dosfs_find_callback_t)(dosfs_volume_t *volume, void *private, dosfs_dir_t *dir);

#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 0) */

#define DOSFS_LDIR_SEQUENCE_MISMATCH         0x80
#define DOSFS_LDIR_SEQUENCE_FIRST            0x40
#define DOSFS_LDIR_SEQUENCE_LAST             0x20
#define DOSFS_LDIR_SEQUENCE_INDEX            0x1f

typedef int (*dosfs_find_callback_t)(dosfs_volume_t *volume, void *private, dosfs_dir_t *dir, unsigned sequence);

#if (DOSFS_CONFIG_UTF8_SUPPORTED == 0)
typedef uint8_t dosfs_unicode_t;
#else /* (DOSFS_CONFIG_UTF8_SUPPORTED == 0) */
typedef uint16_t dosfs_unicode_t;
#endif /* (DOSFS_CONFIG_UTF8_SUPPORTED == 0) */
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 0) */

#define FALSE  0
#define TRUE   1

#if !defined(NULL)
#define NULL   ((void*)0)
#endif

#define DOSFS_CT_ASSERT(_c)                  ((void)sizeof(char[1-(2*!(_c))]))


#define DOSFS_CLSNO_NONE                     0x00000000u
#define DOSFS_CLSNO_FREE                     0x00000000u
#define DOSFS_CLSNO_FREE_RESERVED            0x00000001u
#define DOSFS_CLSNO_LAST                     0x0ffffff8u
#define DOSFS_CLSNO_END_OF_CHAIN             0x0fffffffu

#define DOSFS_CLSNO_RESERVED12               0x00000ff6u
#define DOSFS_CLSNO_RESERVED16               0x0000fff6u
#define DOSFS_CLSNO_RESERVED32               0x0ffffff6u

#define DOSFS_FILE_SIZE_MAX                  0xffffffffu

#define DOSFS_DIR_SIZE                       32
#define DOSFS_DIR_MASK                       31
#define DOSFS_DIR_SHIFT                      5

#define DOSFS_BLKNO_RESERVED                 0xfffffffe /* internal only, will fail to read/write */
#define DOSFS_BLKNO_INVALID                  0xffffffff /* internal only, will fail to read/write */

#define DOSFS_HTOFS(_data)                   (_data)
#define DOSFS_HTOFL(_data)                   (_data)
#define DOSFS_FTOHS(_data)                   (_data)
#define DOSFS_FTOHL(_data)                   (_data)

#define DOSFS_DIR_ATTR_READ_ONLY             0x01
#define DOSFS_DIR_ATTR_HIDDEN                0x02
#define DOSFS_DIR_ATTR_SYSTEM                0x04
#define DOSFS_DIR_ATTR_VOLUME_ID             0x08
#define DOSFS_DIR_ATTR_DIRECTORY             0x10
#define DOSFS_DIR_ATTR_ARCHIVE               0x20

#define DOSFS_DIR_ATTR_LONG_NAME             0x0f
#define DOSFS_DIR_ATTR_LONG_NAME_MASK        0x3f

#define DOSFS_DIR_TYPE_LOSSY                 0x01      /* internal only */
#define DOSFS_DIR_TYPE_UCASE_NAME            0x02      /* internal only */
#define DOSFS_DIR_TYPE_UCASE_EXT             0x04      /* internal only */
#define DOSFS_DIR_TYPE_LCASE_NAME            0x08
#define DOSFS_DIR_TYPE_LCASE_EXT             0x10

struct _dosfs_dir_t {
    uint8_t                 dir_name[11];
    uint8_t                 dir_attr;
    uint8_t                 dir_nt_reserved;
    uint8_t                 dir_crt_time_tenth;
    uint16_t                dir_crt_time;
    uint16_t                dir_crt_date;
    uint16_t                dir_acc_date;
    uint16_t                dir_clsno_hi;
    uint16_t                dir_wrt_time;
    uint16_t                dir_wrt_date;
    uint16_t                dir_clsno_lo;
    uint32_t                dir_file_size;
};

/* The spec says that a valid entry need to have ldir_ord valid,
 * and unused bits set to 0x00 ((ldir_ord & 0xa0) == 0x00).
 * It states furthermore that ldor_type needs to be 0x00, and
 * that other values are reserved for future use. Also it
 * states that for compatibility reasons ldir_clsno_lo needs
 * to be 0x0000. The ldir entries need to be in reverse sequences,
 * from 31 to 1, whereby the first entry has 0x40 set to indicate
 * the start of a sequence. ldir_chksum needs to be identical for
 * all entries, and need to match the computed value for the 
 * dir->dir_name.
 */

#define DOSFS_LAST_LONG_ENTRY                0x40
#define DOSFS_LONG_ENTRY_MASK                0x1f
#define DOSFS_LONG_ENTRY_TYPE                0x00

struct _dosfs_ldir_t {
    uint8_t                 ldir_ord;
    uint8_t                 ldir_name_1[10];
    uint8_t                 ldir_attr;
    uint8_t                 ldir_type;
    uint8_t                 ldir_chksum;
    uint8_t                 ldir_name_2[12];
    uint16_t                ldir_clsno_lo;
    uint8_t                 ldir_name_3[4];
};


union _dosfs_boot_t {
    struct __attribute__((packed)) {
        uint8_t             bs_reserved_1[510];
        uint16_t            bs_trail_sig;            /* 0xaa55 */
    } bs;

    struct __attribute__((packed)) {
        uint8_t             bs_reserved_1[446];
        struct __attribute__((packed)) {
            uint8_t             mbr_boot_ind;
            uint8_t             mbr_start_chs[3];
            uint8_t             mbr_sys_id;
            uint8_t             mbr_end_chs[3];
            uint32_t            mbr_rel_sec;
            uint32_t            mbr_tot_sec;
        }                   mbr_par_table[4];
        uint16_t            bs_trail_sig;            /* 0xaa55 */
    } mbr;

    struct __attribute__((packed)) {
        uint8_t             bs_jmp_boot[3];
        uint8_t             bs_oem_name[8];
        uint16_t            bpb_byts_per_sec;
        uint8_t             bpb_sec_per_clus;
        uint16_t            bpb_rsvd_sec_cnt;
        uint8_t             bpb_num_fats;
        uint16_t            bpb_root_ent_cnt;
        uint16_t            bpb_tot_sec_16;
        uint8_t             bpb_media;
        uint16_t            bpb_fat_sz_16;
        uint16_t            bpb_sec_per_trk;
        uint16_t            bpb_num_heads;
        uint32_t            bpb_hidd_sec_16;
        uint8_t             bs_reserved_1[478];
        uint16_t            bs_trail_sig;            /* 0xaa55 */
    } bpb;

    struct __attribute__((packed)) {
        uint8_t             bs_jmp_boot[3];
        uint8_t             bs_oem_name[8];
        uint16_t            bpb_byts_per_sec;
        uint8_t             bpb_sec_per_clus;
        uint16_t            bpb_rsvd_sec_cnt;
        uint8_t             bpb_num_fats;
        uint16_t            bpb_root_ent_cnt;
        uint16_t            bpb_tot_sec_16;
        uint8_t             bpb_media;
        uint16_t            bpb_fat_sz_16;
        uint16_t            bpb_sec_per_trk;
        uint16_t            bpb_num_heads;
        uint32_t            bpb_hidd_sec_32;
        uint32_t            bpb_tot_sec_32;
        uint8_t             bs_drv_num;
        uint8_t             bs_nt_reserved;
        uint8_t             bs_boot_sig;
        uint32_t            bs_vol_id;
        uint8_t             bs_vol_lab[11];
        uint8_t             bs_fil_sys_type[8];
        uint8_t             bs_reserved_1[448];
        uint16_t            bs_trail_sig;            /* 0xaa55 */
    } bpb40;

    struct __attribute__((packed)) {
        uint8_t             bs_jmp_boot[3];
        uint8_t             bs_oem_name[8];
        uint16_t            bpb_byts_per_sec;
        uint8_t             bpb_sec_per_clus;
        uint16_t            bpb_rsvd_sec_cnt;
        uint8_t             bpb_num_fats;
        uint16_t            bpb_root_ent_cnt;
        uint16_t            bpb_tot_sec_16;
        uint8_t             bpb_media;
        uint16_t            bpb_fat_sz_16;
        uint16_t            bpb_sec_per_trk;
        uint16_t            bpb_num_heads;
        uint32_t            bpb_hidd_sec_32;
        uint32_t            bpb_tot_sec_32;
        uint32_t            bpb_fat_sz_32;
        uint16_t            bpb_ext_flags;
        uint16_t            bpb_fs_ver;
        uint32_t            bpb_root_clus;
        uint16_t            bpb_fsinfo;
        uint16_t            bpb_bkboot;
        uint8_t             bs_reserved_1[12];
        uint8_t             bs_drv_num;
        uint8_t             bs_nt_reserved;
        uint8_t             bs_boot_sig;
        uint32_t            bs_vol_id;
        uint8_t             bs_vol_lab[11];
        uint8_t             bs_fil_sys_type[8];
        uint8_t             bs_reserved_2[420];
        uint16_t            bs_trail_sig;            /* 0xaa55 */
    } bpb71;

    struct __attribute__((packed)) {
        uint8_t             bs_jmp_boot[3];
        uint8_t             bs_oem_name[8];
        uint8_t             bs_reserved_1[79];
        uint8_t             log_map_flags;
        uint8_t             log_map_entries;
        uint32_t            log_map_table[16];        /* hardcoded, uint32_t for FAT12/FAT16 map */
        uint32_t            log_map_blkno;
        uint32_t            log_lead_sig;             /* 0x52444154 ("DOSFS") */
        uint32_t            log_free_clscnt;
        uint32_t            log_next_clsno;
        dosfs_dir_t          log_dir;
        uint8_t             log_dir_flags;
        uint8_t             log_dir_entries;
        uint16_t            log_dir_index;
        uint32_t            log_dir_clsno;
        uint32_t            log_dot_clsno;
        uint32_t            log_struct_sig;           /* 0x45563031 ("EV01") */
        uint32_t            log_del_clsno;
        uint16_t            log_del_index;
        uint8_t             log_del_entries;
        uint8_t             log_lfn_count;
        uint32_t            log_lfn_blkno;
        uint8_t             log_lfn_name[256];
        uint8_t             bs_reserved_2[22];
        uint16_t            bs_trail_sig;            /* 0xaa55 */
    } bpblog;

    uint32_t                __align__[128];
};

struct _dosfs_fsinfo_t {
    uint32_t                fsi_lead_sig;            /* 0x41615252 */
    uint8_t                 fsi_reserved_1[480];
    uint32_t                fsi_struc_sig;           /* 0x61417272 */
    uint32_t                fsi_free_count;
    uint32_t                fsi_nxt_free;
    uint8_t                 fsi_reserved_2[12];
    uint32_t                fsi_trail_sig;           /* 0xaa550000 */
};

struct _dosfs_cache_entry_t {
    uint32_t                blkno;
    uint8_t                 *data;
};

#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)

#define DOSFS_MAP_FLAG_MAP_0_CHANGED         0x01
#define DOSFS_MAP_FLAG_MAP_1_CHANGED         0x02
#define DOSFS_MAP_FLAG_MAP_CHANGED           0x03
#define DOSFS_MAP_FLAG_MAP_DIRTY             0x04
#define DOSFS_MAP_FLAG_MAP_FSINFO            0x08

#define DOSFS_MAP_TABLE_ENTRIES              32
#define DOSFS_MAP_TABLE_OVERFLOW             255

#define DOSFS_DIR_FLAG_SYNC_ENTRY            0x01
#define DOSFS_DIR_FLAG_ACCESS_ENTRY          0x02
#define DOSFS_DIR_FLAG_MODIFY_ENTRY          0x04
#define DOSFS_DIR_FLAG_DESTROY_ENTRY         0x08
#define DOSFS_DIR_FLAG_CREATE_ENTRY          0x10

#define DOSFS_LOG_LEAD_SIG                   0x52444154     /* "RFAT" */
#define DOSFS_LOG_STRUCT_SIG                 0x45563031     /* "EV01" */

#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */

#define DOSFS_FILE_MODE_READ                 0x01
#define DOSFS_FILE_MODE_WRITE                0x02
#define DOSFS_FILE_MODE_APPEND               0x04
#define DOSFS_FILE_MODE_CREATE               0x08
#define DOSFS_FILE_MODE_TRUNCATE             0x10
#define DOSFS_FILE_MODE_SEQUENTIAL           0x40
#define DOSFS_FILE_MODE_RANDOM               0x80

#if (DOSFS_CONFIG_FILE_DATA_CACHE == 1)
#if (DOSFS_CONFIG_DATA_CACHE_ENTRIES != 0)
#define DOSFS_FILE_FLAG_DATA_DIRTY           0x01
#endif /* (DOSFS_CONFIG_DATA_CACHE_ENTRIES != 0) */
#endif /* (DOSFS_CONFIG_FILE_DATA_CACHE == 1) */
#define DOSFS_FILE_FLAG_DATA_MODIFIED        0x02
#define DOSFS_FILE_FLAG_DIR_MODIFIED         0x04
#if (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1)
#define DOSFS_FILE_FLAG_CONTIGUOUS           0x40   /* contiguous cluster range to file->total_clscnt */
#endif /* (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1) */
#define DOSFS_FILE_FLAG_END_OF_CHAIN         0x80   /* END_OF_CHAIN seen */

#if (DOSFS_CONFIG_CLUSTER_CACHE_ENTRIES != 0)

struct _dosfs_cluster_entry_t {
    uint32_t                clsno;
    uint32_t                clsdata;
};

#endif /* (DOSFS_CONFIG_CLUSTER_CACHE_ENTRIES != 0) */

#define DOSFS_VOLUME_STATE_NONE              0
#define DOSFS_VOLUME_STATE_INITIALIZED       1
#define DOSFS_VOLUME_STATE_CARDREMOVED       2
#define DOSFS_VOLUME_STATE_MOUNTED           3
#define DOSFS_VOLUME_STATE_UNUSABLE          4

#define DOSFS_VOLUME_TYPE_FAT12              0
#define DOSFS_VOLUME_TYPE_FAT16              1
#define DOSFS_VOLUME_TYPE_FAT32              2

#if (DOSFS_CONFIG_FAT_CACHE_ENTRIES > 1)
#define DOSFS_VOLUME_FLAG_FAT_INDEX_CURRENT  0x0001
#define DOSFS_VOLUME_FLAG_FAT_0_CURRENT      0x0000
#define DOSFS_VOLUME_FLAG_FAT_1_CURRENT      0x0001
#define DOSFS_VOLUME_FLAG_FAT_0_DIRTY        0x0002
#define DOSFS_VOLUME_FLAG_FAT_1_DIRTY        0x0004
#else /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES > 1) */
#define DOSFS_VOLUME_FLAG_FAT_DIRTY          0x0002
#endif /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES > 1) */
#if (DOSFS_CONFIG_FSINFO_SUPPORTED == 1)
#define DOSFS_VOLUME_FLAG_FSINFO_DIRTY       0x0008
#define DOSFS_VOLUME_FLAG_FSINFO_VALID       0x0010
#endif /* (DOSFS_CONFIG_FSINFO_SUPPORTED == 1) */
#define DOSFS_VOLUME_FLAG_VOLUME_DIRTY       0x0020
#define DOSFS_VOLUME_FLAG_MEDIA_FAILURE      0x0040
#if (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1)
#define DOSFS_VOLUME_FLAG_MOUNTED_DIRTY      0x0080
#endif /* (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1) */
#define DOSFS_VOLUME_FLAG_WRITE_PROTECTED    0x0100

struct _dosfs_volume_t {
    uint8_t                 state;
    uint8_t                 type;
    uint16_t                flags;
    uint32_t                product;                      /* product serial number */
    uint32_t                serial;                       /* volume serial number */
    uint32_t                boot_blkno;
    uint16_t                bkboot_blkofs;
    uint16_t                fsinfo_blkofs;
    uint32_t                fat_blkcnt;
    uint32_t                fat1_blkno;                   /* FAT to read/write from/to */
    uint32_t                fat2_blkno;
    uint32_t                root_clsno;                   /* FAT12/FAT16: 0, FAT32: root_clus */
    uint32_t                root_blkno;                   /* FAT32: root_clus mapped to blkno */
    uint32_t                root_blkcnt;                  /* FAT32: cluster size mapped to blkcnt */
    uint32_t                last_clsno;                   /* last valid cluster */
    uint32_t                cls_size;                     /* (1 << cls_shift) */
    uint32_t                cls_mask;                     /* (1 << cls_shift) -1 */
    uint8_t                 cls_shift;                    /* shift to get the byte offset for a clsno (0x10 max) */
    uint8_t                 cls_blk_shift;                /* shift to get the blkno for a clsno (0x07 max)*/
    uint8_t                 cls_blk_mask;                 /* (1 << cls_blk_shift) -1 (0x7f max) */
    uint8_t                 cls_blk_size;                 /* (1 << cls_blk_shift) (0x80 max) */
    int32_t                 cls_blk_offset;               /* offset to get the blkno for a clsno */
    dosfs_cache_entry_t     dir_cache;
#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)
    dosfs_cache_entry_t     map_cache;
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */
#if (DOSFS_CONFIG_FAT_CACHE_ENTRIES != 0)
#if (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 1)
    dosfs_cache_entry_t     fat_cache;
#else /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 1) */
    dosfs_cache_entry_t     fat_cache[DOSFS_CONFIG_FAT_CACHE_ENTRIES];
#endif /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES == 1) */
#endif /* (DOSFS_CONFIG_FAT_CACHE_ENTRIES != 0) */
#if (DOSFS_CONFIG_FILE_DATA_CACHE == 0)
    dosfs_file_t            *data_file;
#if (DOSFS_CONFIG_DATA_CACHE_ENTRIES != 0)
    dosfs_cache_entry_t     data_cache;
#endif /* (DOSFS_CONFIG_DATA_CACHE_ENTRIES != 0) */
#endif /* (DOSFS_CONFIG_FILE_DATA_CACHE == 0) */
#if (DOSFS_CONFIG_CLUSTER_CACHE_ENTRIES != 0)
    dosfs_cluster_entry_t   cluster_cache[DOSFS_CONFIG_CLUSTER_CACHE_ENTRIES];
#endif /* (DOSFS_CONFIG_CLUSTER_CACHE_ENTRIES != 0) */

    /* WORK AREA BELOW */

    uint32_t                cwd_clsno;                /* put this first of the work area to align the rest */
#if (DOSFS_CONFIG_SEQUENTIAL_SUPPORTED == 1) || (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1)
    uint32_t                au_size;
    uint32_t                erase_size;
    uint32_t                start_clsno;              /* first valid cluster for sequential/contiguous allocation */
    uint32_t                end_clsno;                /* last valid cluster for sequential/contiguous allocation */
#if (DOSFS_CONFIG_SEQUENTIAL_SUPPORTED == 1)
    uint32_t                base_clsno;               /* bottom clsno for  next_clsno allocation unit (inclusive) */
    uint32_t                limit_clsno;              /* top clsno for  next_clsno allocation unit (exclusive) */
    uint32_t                free_clsno;               /* first free clsno */
#endif /* (DOSFS_CONFIG_SEQUENTIAL_SUPPORTED == 1) */
#endif /* (DOSFS_CONFIG_SEQUENTIAL_SUPPORTED == 1) || (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1) */

#if (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1)
    uint8_t                 bs_data[90];
    uint8_t                 map_flags;
    uint8_t                 map_entries;
    uint16_t                map_table[DOSFS_MAP_TABLE_ENTRIES];
    uint32_t                map_blkno;
    uint32_t                log_lead_sig;             /* 0x52444154 ("RFAT") */
    uint32_t                free_clscnt;
    uint32_t                next_clsno;
    dosfs_dir_t             dir;
    uint8_t                 dir_flags;
    uint8_t                 dir_entries;
    uint16_t                dir_index;
    uint32_t                dir_clsno;
    uint32_t                dot_clsno;                /* reserved for f_move() */
    uint32_t                log_struct_sig;           /* 0x45563031 ("EV01") */
    uint32_t                del_clsno;
    uint16_t                del_index;
    uint8_t                 del_entries;
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
    uint8_t                 lfn_count;
    uint32_t                lfn_blkno;
#if (DOSFS_CONFIG_UTF8_SUPPORTED == 0)
    dosfs_unicode_t         lfn_name[256];
    uint8_t                 bs_reserved_2[22];
    uint16_t                bs_trail_sig;
#else /* (DOSFS_CONFIG_UTF8_SUPPORTED == 0) */
    dosfs_unicode_t         lfn_name[255];
    uint16_t                bs_trail_sig;
#endif /* (DOSFS_CONFIG_UTF8_SUPPORTED == 0) */
#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
    uint8_t                 bs_reserved_2[283];
    uint16_t                bs_trail_sig;
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */

#else /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */

#if (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1)
    uint8_t                 bs_data[90];
#endif /* (DOSFS_CONFIG_VOLUME_DIRTY_SUPPORTED == 1) */
#if (DOSFS_CONFIG_FSINFO_SUPPORTED == 1)
    uint32_t                free_clscnt;
#endif /* (DOSFS_CONFIG_FSINFO_SUPPORTED == 1) */
    uint32_t                next_clsno;
    dosfs_dir_t              dir;
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
    uint8_t                 dir_entries;
    uint8_t                 lfn_count;
#if (DOSFS_CONFIG_UTF8_SUPPORTED == 0)
    dosfs_unicode_t         lfn_name[255];
#else /* (DOSFS_CONFIG_UTF8_SUPPORTED == 0) */
    dosfs_unicode_t         lfn_name[255];
#endif /* (DOSFS_CONFIG_UTF8_SUPPORTED == 0) */
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
#endif /* (DOSFS_CONFIG_TRANSACTION_SAFE_SUPPORTED == 1) */

    /* WORK AREA ABOVE */

    uint8_t                 media;
    dosfs_file_t            *files;

#if (DOSFS_CONFIG_STATISTICS == 1)
    struct {
        uint32_t                fat_cache_hit;
        uint32_t                fat_cache_miss;
        uint32_t                fat_cache_read;
        uint32_t                fat_cache_write;
        uint32_t                fat_cache_flush;
        uint32_t                dir_cache_hit;
        uint32_t                dir_cache_miss;
        uint32_t                dir_cache_zero;
        uint32_t                dir_cache_read;
        uint32_t                dir_cache_write;
        uint32_t                data_cache_hit;
        uint32_t                data_cache_miss;
        uint32_t                data_cache_zero;
        uint32_t                data_cache_read;
        uint32_t                data_cache_write;
        uint32_t                data_cache_flush;
        uint32_t                data_cache_invalidate;
        uint32_t                cluster_cache_hit;
        uint32_t                cluster_cache_miss;
    }                       statistics;
#endif /* (DOSFS_CONFIG_STATISTICS == 1) */
};

extern dosfs_volume_t dosfs_volume;

/* The conversion from a linear offset/size to a clscnt is somewhat tricky.
 * An offset is rounded down, while a size is rounded up. But rouding up
 * a size close to DOSFS_FILE_SIZE_MAX will overflow 32bit arithmatic. Hence
 * the macro splits up away the bytes within a cluster and rounds them up
 * separately.
 */

#define DOSFS_OFFSET_TO_CLSCNT(_offset)    ((_offset) >> volume->cls_shift)
#define DOSFS_SIZE_TO_CLSCNT(_size)        (((_size) >> volume->cls_shift) + ((((_size) & volume->cls_mask) + volume->cls_mask) >> volume->cls_shift))

/* Convert a clsno to a blkno
 */
#define DOSFS_CLSNO_TO_BLKNO(_clsno)       (volume->cls_blk_offset + ((_clsno) << volume->cls_blk_shift))
#define DOSFS_CLSNO_TO_BLKNO_NEXT(_clsno)  (DOSFS_CLSNO_TO_BLKNO((_clsno)) + volume->cls_blk_size)

/* Convert a file offset to a blkcnt realtive to the start of a cluster.
 */
#define DOSFS_OFFSET_TO_BLKCNT(_offset)    (((_offset) >> DOSFS_BLK_SHIFT) & volume->cls_blk_mask)

/* Convert a directory index to a blkcnt realtive to the start of a cluster.
 */
#define DOSFS_INDEX_TO_BLKCNT(_index)      (((_index) >> (DOSFS_BLK_SHIFT - DOSFS_DIR_SHIFT)) & volume->cls_blk_mask)
#define DOSFS_INDEX_TO_BLKCNT_ROOT(_index) ((_index) >> (DOSFS_BLK_SHIFT - DOSFS_DIR_SHIFT))

/* Convert a directory index to a offset realtive to the start of a block.
 */

#define DOSFS_INDEX_TO_BLKOFS(_index)      (((_index) << DOSFS_DIR_SHIFT) & DOSFS_BLK_MASK)


#define DOSFS_DEFAULT_VOLUME()       (&dosfs_volume)
#define DOSFS_PATH_VOLUME(_path)     (&dosfs_volume)
#define DOSFS_FIND_VOLUME(_find)     (&dosfs_volume)
#define DOSFS_FILE_VOLUME(_file)     (&dosfs_volume)
#define DOSFS_VOLUME_DEVICE(_volume) (&dosfs_device)

#if (DOSFS_CONFIG_STATISTICS == 1)

#define DOSFS_VOLUME_STATISTICS_COUNT(_name)       { volume->statistics._name += 1; }
#define DOSFS_VOLUME_STATISTICS_COUNT_N(_name,_n)  { volume->statistics._name += (_n); }

#else /* (DOSFS_CONFIG_STATISTICS == 1) */

#define DOSFS_VOLUME_STATISTICS_COUNT(_name)       /**/
#define DOSFS_VOLUME_STATISTICS_COUNT_N(_name,_n)  /**/

#endif /* (DOSFS_CONFIG_STATISTICS == 1) */

#ifdef __cplusplus
}
#endif

#endif /* _DOSFS_CORE_H */
