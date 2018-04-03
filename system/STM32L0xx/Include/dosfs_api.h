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

#if !defined(_DOSFS_API_H)
#define _DOSFS_API_H

#include <stdint.h>

#include "dosfs_config.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define F_NO_ERROR                   0
#define F_ERR_RESERVED_1             1
#define F_ERR_NOTFORMATTED           2
#define F_ERR_INVALIDDIR             3
#define F_ERR_INVALIDNAME            4
#define F_ERR_NOTFOUND               5
#define F_ERR_DUPLICATED             6
#define F_ERR_NOMOREENTRY            7
#define F_ERR_NOTOPEN                8
#define F_ERR_EOF                    9
#define F_ERR_RESERVED_2             10
#define F_ERR_NOTUSEABLE             11
#define F_ERR_LOCKED                 12
#define F_ERR_ACCESSDENIED           13
#define F_ERR_NOTEMPTY               14
#define F_ERR_INITFUNC               15
#define F_ERR_CARDREMOVED            16
#define F_ERR_ONDRIVE                17
#define F_ERR_INVALIDSECTOR          18
#define F_ERR_READ                   19
#define F_ERR_WRITE                  20
#define F_ERR_INVALIDMEDIA           21
#define F_ERR_BUSY                   22
#define F_ERR_WRITEPROTECT           23
#define F_ERR_INVFATTYPE             24
#define F_ERR_MEDIATOOSMALL          25
#define F_ERR_MEDIATOOLARGE          26
#define F_ERR_NOTSUPPSECTORSIZE      27
#define F_ERR_ALLOCATION             28
#define F_ERR_OS                     29
#define F_ERR_TOOLONGNAME            30
#define F_ERR_UNUSABLE               31

#define F_SEEK_SET                   0
#define F_SEEK_CUR                   1
#define F_SEEK_END                   2

#define F_SEPARATORCHAR              '/'
#define F_SECTOR_SIZE                512

#define F_MAXLNAME                   255 /* long file name */
#define F_MAXNAME                    8   /* 8 byte name */
#define F_MAXEXT                     3   /* 3 byte extension */

#if !defined(F_MAXPATH)
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 1)
#define F_MAXPATH                    (F_MAXLNAME+1)
#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
#define F_MAXPATH                    (F_MAXNAME+F_MAXEXT+2)
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 1) */
#endif /* !defined(F_MAXPATH) */

#define F_ATTR_READONLY              0x01
#define F_ATTR_HIDDEN                0x02
#define F_ATTR_SYSTEM                0x04
#define F_ATTR_VOLUME                0x08
#define F_ATTR_DIR                   0x10
#define F_ATTR_ARC                   0x20

#define F_UNKNOWN_MEDIA              0
#define F_FAT12_MEDIA                1
#define F_FAT16_MEDIA                2
#define F_FAT32_MEDIA                3

#define F_CTIME_SEC_SHIFT            0
#define F_CTIME_SEC_MASK             0x001f   /* 0-30 in 2 seconds */
#define F_CTIME_MIN_SHIFT            5
#define F_CTIME_MIN_MASK             0x07e0   /* 0-59 */
#define F_CTIME_HOUR_SHIFT           11
#define F_CTIME_HOUR_MASK            0xf800   /* 0-23 */

#define F_CDATE_DAY_SHIFT            0
#define F_CDATE_DAY_MASK             0x001f   /* 1-31 */
#define F_CDATE_MONTH_SHIFT          5
#define F_CDATE_MONTH_MASK           0x01e0   /* 1-12 */
#define F_CDATE_YEAR_SHIFT           9
#define F_CDATE_YEAR_MASK            0xfe00   /* 0-119 (1980+value) */

#define F_CLUSTER_FREE               ((unsigned long)0x00000000)
#define F_CLUSTER_RESERVED           ((unsigned long)0x0ffffff0)
#define F_CLUSTER_BAD                ((unsigned long)0x0ffffff7)
#define F_CLUSTER_LAST               ((unsigned long)0x0ffffff8)
#define F_CLUSTER_LASTF32R           ((unsigned long)0x0fffffff)

typedef struct _dosfs_file_t         F_FILE;

typedef struct {
    uint32_t       dot_clsno;
    uint32_t       dir_clsno;
    uint32_t       dir_index;
} F_DIR;

typedef struct {
    unsigned char  attr;                            /* file attribute     */
    unsigned int   filesize;                        /* file length        */
    char           filename[F_MAXPATH];             /* name.ext           */
} F_DIRENT;

typedef struct {
    char           filename[F_MAXPATH];             /* name.ext           */
    char           name[F_MAXNAME];                 /* file name          */
    char           ext[F_MAXEXT];                   /* file extension     */
    unsigned char  attr;                            /* file attribute     */
    unsigned short ctime;                           /* file creation time */
    unsigned short cdate;                           /* file creation date */
    unsigned int   cluster;                         /* file start cluster */
    unsigned int   filesize;                        /* file length        */

    /* IMPLEMENTATION SPECIFIC BELOW */
#if (DOSFS_CONFIG_VFAT_SUPPORTED == 0)
    char           find_pattern[F_MAXNAME+F_MAXEXT];
#else /* (DOSFS_CONFIG_VFAT_SUPPORTED == 0) */
    char           find_pattern[F_MAXLNAME];
#endif /* (DOSFS_CONFIG_VFAT_SUPPORTED == 0) */
    F_DIR          find_dir;
    /* IMPLEMENTATION SPECIFIC ABOVE */
} F_FIND;

typedef struct {
    unsigned int   filesize;
    unsigned short createdate;
    unsigned short createtime;
    unsigned short modifieddate;
    unsigned short modifiedtime;
    unsigned short lastaccessdate;
    unsigned char  attr;
} F_STAT;

typedef struct {
    unsigned int   total;
    unsigned int   free;
    unsigned int   used;
    unsigned int   bad;
    unsigned int   total_high;
    unsigned int   free_high;
    unsigned int   used_high;
    unsigned int   bad_high;
} F_SPACE;

struct _dosfs_file_t {
    uint8_t                 mode;
    uint8_t                 flags;
    uint8_t                 status;
    uint8_t                 reserved[3];    /* unused for now */
    uint16_t                dir_index;      /* index within directory where primary dir entry resides */
    uint32_t                dir_clsno;      /* clsno where primary dir entry resides */ 
    uint32_t                first_clsno;    /* dir_clsno_hi/dir_clsno_lo from dir entry */
    uint32_t                last_clsno;     /* clsno corresponding to file->length */
#if (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1)
    uint32_t                size;           /* size of reserved area */
#endif /* (DOSFS_CONFIG_CONTIGUOUS_SUPPORTED == 1) */
    uint32_t                length;         /* dir_file_size from dir entry */
    uint32_t                position;
    uint32_t                clsno;
    uint32_t                blkno;
    uint32_t                blkno_e;        /* exclusive */
    F_FILE                  *next;
#if (DOSFS_CONFIG_FILE_DATA_CACHE == 1)
#if (DOSFS_CONFIG_DATA_CACHE_ENTRIES != 0)
    dosfs_cache_entry_t      data_cache;
#endif /* (DOSFS_CONFIG_DATA_CACHE_ENTRIES != 0) */
#endif /* (DOSFS_CONFIG_FILE_DATA_CACHE == 1) */
};

extern int     f_initvolume(void);
extern int     f_delvolume(void);
extern int     f_checkvolume(void);
extern int     f_format(int fattype);
extern int     f_hardformat(int fattype);
extern int     f_getfreespace(F_SPACE *pspace);
extern int     f_getserial(unsigned long *p_serial);
extern int     f_setlabel(const char *volname);
extern int     f_getlabel(char *volname, int length);

extern int     f_mkdir(const char *dirname);
extern int     f_rmdir(const char *dirname);
extern int     f_chdir(const char *dirname);
extern int     f_getcwd(char *dirname, int length);

extern int     f_opendir(F_DIR *dir, const char *dirname);
extern int     f_readdir(F_DIR *dir, F_DIRENT *dirent);
extern int     f_rewinddir(F_DIR *dir);
extern int     f_closedir(F_DIR *dir);

extern int     f_rename(const char *filename, const char *newname);
extern int     f_move(const char *filename, const char *newname);
extern int     f_delete(const char *filename);
extern long    f_filelength(const char *filename);
extern int     f_findfirst(const char *filename, F_FIND *find);
extern int     f_findnext(F_FIND *find);
extern int     f_settimedate(const char *filename, unsigned short ctime, unsigned short cdate);
extern int     f_gettimedate(const char *filename, unsigned short *p_ctime, unsigned short *p_cdate);
extern int     f_setattr(const char *filename, unsigned char attr);
extern int     f_getattr(const char *filename, unsigned char *p_attr);
extern int     f_stat(const char *filename, F_STAT *stat);

extern int     f_open(F_FILE *file, const char *filename, const char *type);
extern int     f_close(F_FILE *file);
extern int     f_flush(F_FILE *file);
extern long    f_write(const void *buffer, long size, long count, F_FILE *file);
extern long    f_read(void *buffer, long size, long count, F_FILE *file);
extern int     f_seek(F_FILE *file, long offset, int whence);
extern long    f_tell(F_FILE *file);
extern long    f_length(F_FILE *file);
extern int     f_eof(F_FILE *file);
extern int     f_error(F_FILE *file);
extern int     f_rewind(F_FILE *file);
extern int     f_putc(int c, F_FILE *file);
extern int     f_getc(F_FILE *file);
extern int     f_seteof(F_FILE *file);
extern int     f_truncate(F_FILE *file, const char *filename, long length);

#ifdef __cplusplus
}
#endif

#endif /* _DOSFS_API_H */
