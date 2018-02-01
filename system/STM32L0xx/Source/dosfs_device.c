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

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "dosfs_core.h"

dosfs_device_t dosfs_device;

int dosfs_device_format(dosfs_device_t *device, uint8_t *data)
{
    int status = F_NO_ERROR;
    uint8_t media, write_protected;
    unsigned int fattype, sys_id;
    uint32_t blkno, blkcnt, blk_unit_size, cls_blk_shift, clscnt, clscnt_e, tot_sec, num_fats;
    uint32_t boot_blkno, fat1_blkno, fat2_blkno, root_blkno, clus_blkno, fat_blkcnt, root_blkcnt, user_blkno, user_blkcnt;
    uint32_t hpc, spt, start_h, start_s, start_c, end_h, end_s, end_c;
    uint32_t au_size, product;
    dosfs_boot_t *boot;
    dosfs_fsinfo_t *fsinfo;

    status = (*device->interface->info)(device->context, &media, &write_protected, &blkcnt, &au_size, &product);

    if (status == F_NO_ERROR)
    {
        if (write_protected)
        {
            status = F_ERR_WRITEPROTECT;
        }
        else
        {
#if (DOSFS_CONFIG_FAT12_SUPPORTED == 0)
            /* SDSC with 64MB or less end up being FAT12. So let's reject them
             * early.
             */
            if (blkcnt <= 131072)
            {
                status = F_ERR_MEDIATOOSMALL;
            }
            else
#endif /* (DOSFS_CONFIG_FAT12_SUPPORTED == 0) */
            {
                /* SDSC/SDHC are limited to 32GB. Above it's a SDXC,
                 * which would exFAT ... but a 128GB SDXC still works
                 * fine with FAT32.
                 */
                if (blkcnt > 268435456)
                {
                    status = F_ERR_MEDIATOOLARGE;
                }
            }
        }

        if (status == F_NO_ERROR)
        {
            device->lock |= DOSFS_DEVICE_LOCK_MODIFIED;

            /*
             * There is a upper limit of 4153344 blocks for SDSC cards, and
             * a lower limit of 4211712 for SDHC cards as per SD spec (CSD description).
             *
             * Hower the real lower limit for SHDC is:
             *
             *    16384 + (65525 * 64) = 4209984
             *    4211712 = 8192 + 8192 + (65552 * 64)
             *
             *    (16384 is the required padding upfront for SDHC at this size)
             *
             * The upper legal limit for SDSC is also slightly different from the
             * spec if one thinks about it:
             *
             *    768 + (65524 * 64) = 4194304
             *    4153344 = 768 + (64884 * 64)
             *
             *    (768 is the required padding upfront for SDSC at this size)
             *
             * Thus use those corrected limits to figure out SDSC/SDHC.
             */
    
            if ((media == DOSFS_MEDIA_SFLASH) || (blkcnt < 4209984))
            {
                /* SFLASH, SDSC, FAT12/FAT16 */

                /* Number of Heads and Sectors Per Track ...
                 *
                 * See Microsoft's MBR specs and the term "translation mode".
                 * 
                 * We want to use powers of 2, which means the upper limit for
                 * that representation is 1024 * 128 * 32  = 4194304. That happens
                 * to be also the upper limit for SDSC here. If the size is smaller
                 * than 2 * 16, use "hpc" 2 and "spt" 16, otherwise "spt" is 32, and 
                 * "hpc" is the smallest power of 2.
                 */

                if (blkcnt <= (1024 * 2 * 16))
                {
                    hpc = 2;
                    spt = 16;
                }
                else
                {
                    hpc = 2;
                    spt = 32;
                    
                    while (hpc < 128)
                    {
                        if (blkcnt <= (1024 * hpc * spt))
                        {
                            break;
                        }

                        hpc <<= 1;
                    }
                }

                if (media == DOSFS_MEDIA_SFLASH)
                {
                    /* For SFLASH we follow the default FAT settings with a cluster size
                     * of 4k. Also the erase unit is set that way for now. This means that
                     * a 16MB device will need 64 blocks for PAR/MBR/FAT1/FAT2/DIR.
                     */

                    num_fats = 2;

                    if (blkcnt <= 4906)            /* <= 2MB   -> 512/4k  */
                    {
                        cls_blk_shift = 0;
                    }
                    if (blkcnt <= 8192)            /* <= 4MB   -> 1k/4k   */
                    {
                        cls_blk_shift = 1;
                    }
                    else if (blkcnt <= 16384)      /* <= 8MB   -> 2k/4k   */
                    {
                        cls_blk_shift = 2;
                    }
                    else                           /* <= 16MB  -> 4k/4k   */
                    {
                        cls_blk_shift = 3;
                    }

                    blk_unit_size = 8;
                }
                else
                {
                    /* This table is derived from a ScanDisk SDCARD Product Manual.
                     *
                     * "au_blk_size", is derived by assuming that "Total LBAs" - "User Data Sectors"
                     * needs to be a multiple of a power of 2. It was assumed that the maximum value
                     * would map to the proper "au_blk_size". Next "Total LBAs" - "Total Paration Sectors"
                     * allows to compute the number of blocks per FAT. Assuming that the FAT does not
                     * contain more than one partial populated blocks, this allows to compute the
                     * cluster size picked.
                     * 
                     * The 16MB threshold was derived from a ScanDisk MMC Card Product Manual.
                     * Cards with 4MB and 8MB capacity have an erase size of 8k, hence it's assumed
                     * here that this would map to SDCARDs as well. In reality it probably does not
                     * matter as there are not such cards out there.
                     */

                    num_fats = 2;

#if (DOSFS_CONFIG_FAT12_SUPPORTED == 1)
                    if (blkcnt <= 16384)            /* <= 8MB   -> 8k/8k   */
                    {
                        cls_blk_shift = 4;
                        blk_unit_size = 16;
                    }
                    else if (blkcnt <= 131072)      /* <= 64MB  -> 16k/16k */
                    {
                        cls_blk_shift = 5;
                        blk_unit_size = 32;
                    }
                    else
#endif /* (DOSFS_CONFIG_FAT12_SUPPORTED == 1) */
                    {
                        if (blkcnt <= 524288)       /* <= 256MB -> 16k/32k */
                        {
                            cls_blk_shift = 5;
                            blk_unit_size = 64;
                        }
                        else if (blkcnt <= 2097152) /* <= 1GB   -> 16k/64k */
                        {
                            cls_blk_shift = 5;
                            blk_unit_size = 128;
                        }
                        else                        /* <= 2GB   -> 32k/64k */
                        {
                            cls_blk_shift = 6;
                            blk_unit_size = 128;
                        }
                    }
                }

                /*
                 * FAT12/FAT16 layout requires that the MBR takes up "blk_unit_size" blocks for an SDCARD. There is
                 * also 32 blocks for the root directory. Then there is one boot sector
                 *
                 * To compute fat_blkcnt, first estimate clscnt_e based upon the minimum system area, then compute
                 * the required system area based upon this clscnt_e. With the required system area size recompute clscnt.
                 * If clscnt is less than clscnt_e, use clscnt as clscnt_e and restart the process.
                 *
                 * The will incrementally adjust the clscnt estimate downwards, minimizing the required system area.
                 */

                root_blkcnt = 32;

                if (media == DOSFS_MEDIA_SFLASH)
                {
                    clscnt = ((blkcnt - blk_unit_size) & ~(blk_unit_size -1)) >> cls_blk_shift;
                }
                else
                {
                    clscnt = ((blkcnt - 2 * blk_unit_size) & ~(blk_unit_size -1)) >> cls_blk_shift;
                }

#if (DOSFS_CONFIG_FAT12_SUPPORTED == 1)
                if (clscnt < 4085)
                {
                    fattype = DOSFS_VOLUME_TYPE_FAT12;
                }
                else
#endif /* (DOSFS_CONFIG_FAT12_SUPPORTED == 1) */
                {
                    fattype = DOSFS_VOLUME_TYPE_FAT16;

                    if (clscnt > 65524)
                    {
                        clscnt = 65524;
                    }
                }


                do 
                {
                    clscnt_e = clscnt;
                            
#if (DOSFS_CONFIG_FAT12_SUPPORTED == 1)
                    if (fattype == DOSFS_VOLUME_TYPE_FAT12)
                    {
                        fat_blkcnt = (((((clscnt + 2) * 3) + 1) / 2) + (DOSFS_BLK_SIZE -1)) >> DOSFS_BLK_SHIFT;
                    }
                    else
#endif /* (DOSFS_CONFIG_FAT12_SUPPORTED == 1) */
                    {
                        fat_blkcnt = ((clscnt + 2) * 2 + (DOSFS_BLK_SIZE -1)) >> DOSFS_BLK_SHIFT;
                    }
                    
                    if (media == DOSFS_MEDIA_SFLASH)
                    {
                        /* ### For a TRANSACTION SAFE, FAT12/FAT16 needs on extra block, so there need to be 3 extra blocks in total.
                         */
                        clus_blkno = ((2 + num_fats * fat_blkcnt + root_blkcnt + (blk_unit_size -1)) & ~(blk_unit_size -1));
                    }
                    else
                    {
                        clus_blkno = blk_unit_size + ((1 + num_fats * fat_blkcnt + root_blkcnt + (blk_unit_size -1)) & ~(blk_unit_size -1));
                    }
                                
                    clscnt = ((blkcnt - clus_blkno) & ~(blk_unit_size -1)) >> cls_blk_shift;

                    if (clscnt > clscnt_e)
                    {
                        clscnt = clscnt_e;
                    }
                }
                while (clscnt != clscnt_e);

                root_blkno = (clus_blkno - 32);

                if (num_fats == 1)
                {
                    fat2_blkno = 0;
                    fat1_blkno = root_blkno - fat_blkcnt;
                }
                else
                {
                    fat2_blkno = root_blkno - fat_blkcnt;
                    fat1_blkno = fat2_blkno - fat_blkcnt;
                }

                if (media == DOSFS_MEDIA_SFLASH)
                {
                    boot_blkno = 1;
                }
                else
                {
                    boot_blkno = fat1_blkno - 1;
                }
            }
            else
            {
                /* SDHC, FAT32 */

                /* Number of Heads and Sectors Per Track ...
                 *
                 * See Microsoft's MBR specs and the term "translation mode".
                 */

                if (blkcnt <= 8388608)
                {
                    hpc = 128;
                    spt = 63;
                }
                else
                {
                    hpc = 255;
                    spt = 63;
                }

                num_fats = 2;

                if (blkcnt <= 67108864) /* <= 32GB  -> 4MB  */
                {
                    cls_blk_shift = 6;
                    blk_unit_size = 8192;
                            
                    if (blkcnt <= 33554432)
                    {
                        /* For a 16GB SDHC one has to make sure that the FAT1/FAT2 do not
                         * overflow the 1 allocation unit that is available for the 
                         * "System Area".
                         *
                         * (8192 - 9) / 2     = 4091     blocks are available per FAT
                         * (4091 * 128) - 2   = 523646   data clusters
                         * 523646 * 64        = 33513344 data blocks
                         * 33513344 + 8192    = 33521536 total blocks
                         * 33521536 & ~8191   = 33513472 usable blocks
                         */
                                
                        if (blkcnt > 33513472)
                        {
                            blkcnt = 33513472;
                        }

                        clus_blkno = 16384;
                    }
                    else
                    {
                        /* For a 32GB SDHC one has to make sure that the FAT1/FAT2 do not
                         * overflow the a 2 allocation units that are available for the 
                         * "System Area".
                         *
                         * (16384 - 9) / 2    = 8187     blocks are available per FAT
                         * (8187 * 128) - 2   = 1047934  data clusters
                         * 1047934 * 64       = 67067776 data blocks
                         * 67067776 + 8192    = 67075968 total blocks
                         * 67075968 & ~8191   = 67067904 usable blocks
                         */
                                
                        if (blkcnt > 67067904)
                        {
                            blkcnt = 67067904;
                        }

                        clus_blkno = 24576;
                    }
                }
                else                   /* <= 128GB -> 16MB */
                {
                    /* The guess "blk_unit_size" for a 64GB card has been confirmed
                     * by looking at various available SDXC cards which all were formatted
                     * the same way.
                     */

                    cls_blk_shift = 7;
                    blk_unit_size = 32768;

                    /* For a 128GB SDXC one has to make sure that the FAT1/FAT2 do not
                     * overflow the single allocation unit that is available for the 
                     * "System Area".
                     *
                     * (32768 - 9) / 2    = 16379     blocks are available per FAT
                     * (16379 * 128) - 2  = 2096510   data clusters
                     * 2096510 * 128      = 268353280 data blocks
                     * 268353280 + 65536  = 268418816 total blocks
                     * 268418816 & ~32767 = 268402688 usable blocks
                     */

                    if (blkcnt > 268402688)
                    {
                        blkcnt = 268402688;
                    }

                    clus_blkno = 65536;
                }

                /* FAT32 is layed out different than FAT12/FAT16. There is no root_blkno, and we know
                 * that there are at least 65525 clusters. Also there are at least 9 reserved blocks
                 * minimum.
                 */

                root_blkcnt = (1 << cls_blk_shift);

                fattype = DOSFS_VOLUME_TYPE_FAT32;

                clscnt = ((blkcnt - clus_blkno) & ~(blk_unit_size -1)) >> cls_blk_shift;
                fat_blkcnt = ((clscnt + 2) * 4 + (DOSFS_BLK_SIZE -1)) >> DOSFS_BLK_SHIFT;

                root_blkno = clus_blkno;
                fat2_blkno = clus_blkno - fat_blkcnt;
                fat1_blkno = fat2_blkno - fat_blkcnt;
                boot_blkno = blk_unit_size;
            }

            if (status == F_NO_ERROR)
            {
                status = (*device->interface->format)(device->context);
            }

            if (status == F_NO_ERROR)
            {
                blkcnt = clus_blkno + (clscnt << cls_blk_shift);

                tot_sec = (blkcnt - boot_blkno);

                if (fattype != DOSFS_VOLUME_TYPE_FAT32)
                {
                    if (tot_sec < 32680)
                    {
                        sys_id = 0x01;
                    }
                    else if (tot_sec < 65536)
                    {
                        sys_id = 0x04;
                    }
                    else
                    {
                        sys_id = 0x06;
                    }
                }
                else
                {
                    /* Select sys_id to get either CHS/LBA or LBA.
                     */
                    if (blkcnt <= 16450560)
                    {
                        sys_id = 0x0b;
                    }
                    else
                    {
                        sys_id = 0x0c;
                    }
                }

                /* CHS has max legal values (0..1023) * (0..254) * (1..63). If the LBA is outside
                 * this CHS range, then use the maximum value. This can only happen with FAT32,
                 * in which case the partition type signals to use LBA anyway.
                 */

                start_c = boot_blkno / (hpc * spt);
                start_h = (boot_blkno - (start_c * hpc * spt)) / spt;
                start_s = boot_blkno - (start_c * hpc * spt) - (start_h * spt) + 1;
                        
                if (blkcnt <= 16450560)
                {
                    end_c = (blkcnt-1) / (hpc * spt);
                    end_h = ((blkcnt-1) - (end_c * hpc * spt)) / spt;
                    end_s = (blkcnt-1) - (end_c * hpc * spt) - (end_h * spt) + 1;
                }
                else
                {
                    end_c   = 1023;
                    end_h   = 254;
                    end_s   = 63;
                }

                /* Write the MBR */

                boot = (dosfs_boot_t*)data;
                memset(boot, 0, DOSFS_BLK_SIZE);

                boot->mbr.mbr_par_table[0].mbr_boot_ind     = 0x00;
                boot->mbr.mbr_par_table[0].mbr_start_chs[0] = start_h;
                boot->mbr.mbr_par_table[0].mbr_start_chs[1] = ((start_c << 6) | start_s);
                boot->mbr.mbr_par_table[0].mbr_start_chs[2] = (start_c >> 2);
                boot->mbr.mbr_par_table[0].mbr_sys_id       = sys_id;
                boot->mbr.mbr_par_table[0].mbr_end_chs[0]   = end_h; 
                boot->mbr.mbr_par_table[0].mbr_end_chs[1]   = ((end_c << 6) | end_s);
                boot->mbr.mbr_par_table[0].mbr_end_chs[2]   = (end_c >> 2);
                boot->mbr.mbr_par_table[0].mbr_rel_sec      = DOSFS_HTOFL(boot_blkno);
                boot->mbr.mbr_par_table[0].mbr_tot_sec      = DOSFS_HTOFL(tot_sec);
                boot->bs.bs_trail_sig                       = DOSFS_HTOFS(0xaa55);

                status = (*device->interface->write)(device->context, 0, (uint8_t*)boot, 1, NULL);

                if (status == F_NO_ERROR)
                {
                    /* Write the PBR */
                
                    memset(boot, 0, DOSFS_BLK_SIZE);

                    boot->bpb.bs_jmp_boot[0]   = 0xeb;
                    boot->bpb.bs_jmp_boot[1]   = 0x00;
                    boot->bpb.bs_jmp_boot[2]   = 0x90;
                    boot->bpb.bs_oem_name[0]   = 'M';
                    boot->bpb.bs_oem_name[1]   = 'S';
                    boot->bpb.bs_oem_name[2]   = 'D';
                    boot->bpb.bs_oem_name[3]   = 'O';
                    boot->bpb.bs_oem_name[4]   = 'S';
                    boot->bpb.bs_oem_name[5]   = '5';
                    boot->bpb.bs_oem_name[6]   = '.';
                    boot->bpb.bs_oem_name[7]   = '0';
                    boot->bpb.bpb_byts_per_sec = DOSFS_HTOFS(DOSFS_BLK_SIZE);
                    boot->bpb.bpb_sec_per_clus = (1 << cls_blk_shift);
                    boot->bpb.bpb_rsvd_sec_cnt = DOSFS_HTOFS(fat1_blkno - boot_blkno);
                    boot->bpb.bpb_num_fats     = num_fats;
                    boot->bpb.bpb_media        = 0xf8;
                    boot->bpb.bpb_sec_per_trk  = spt;
                    boot->bpb.bpb_num_heads    = hpc;
                    boot->bs.bs_trail_sig      = DOSFS_HTOFS(0xaa55);

                    if (fattype != DOSFS_VOLUME_TYPE_FAT32)
                    {
                        boot->bpb.bpb_root_ent_cnt   = DOSFS_HTOFS(512);
                        boot->bpb.bpb_tot_sec_16     = DOSFS_HTOFS((tot_sec >= 65536) ? 0 : tot_sec);
                        boot->bpb.bpb_fat_sz_16      = DOSFS_HTOFS(fat_blkcnt);
                        boot->bpb40.bpb_hidd_sec_32  = DOSFS_HTOFL(boot_blkno);
                        boot->bpb40.bpb_tot_sec_32   = DOSFS_HTOFS((tot_sec >= 65536) ? tot_sec : 0);
                        boot->bpb40.bs_drv_num       = 0x80;
                        boot->bpb40.bs_nt_reserved   = 0x00;
                        boot->bpb40.bs_boot_sig      = 0x29;
                        boot->bpb40.bs_vol_id        = DOSFS_HTOFL(product);

                        memcpy(boot->bpb40.bs_vol_lab, "NO NAME    ", sizeof(boot->bpb40.bs_vol_lab));

#if (DOSFS_CONFIG_FAT12_SUPPORTED == 1)
                        if (fattype == DOSFS_VOLUME_TYPE_FAT12)
                        {
                            memcpy(boot->bpb40.bs_fil_sys_type, "FAT12   ", sizeof(boot->bpb40.bs_fil_sys_type));
                        }
                        else
#endif /* (DOSFS_CONFIG_FAT12_SUPPORTED == 1) */
                        {
                            memcpy(boot->bpb40.bs_fil_sys_type, "FAT16   ", sizeof(boot->bpb40.bs_fil_sys_type));
                        }

                        status = (*device->interface->write)(device->context, boot_blkno, (uint8_t*)boot, 1, NULL);
                    }
                    else
                    {
                        boot->bpb.bpb_root_ent_cnt   = DOSFS_HTOFS(0);
                        boot->bpb.bpb_tot_sec_16     = DOSFS_HTOFS(0);
                        boot->bpb.bpb_fat_sz_16      = DOSFS_HTOFS(0);
                        boot->bpb71.bpb_hidd_sec_32  = DOSFS_HTOFL(boot_blkno);
                        boot->bpb71.bpb_tot_sec_32   = DOSFS_HTOFL(tot_sec);
                        boot->bpb71.bpb_fat_sz_32    = DOSFS_HTOFL(fat_blkcnt);
                        boot->bpb71.bpb_ext_flags    = DOSFS_HTOFS(0x0000);
                        boot->bpb71.bpb_fs_ver       = DOSFS_HTOFS(0x0000);
                        boot->bpb71.bpb_root_clus    = DOSFS_HTOFL(2);
                        boot->bpb71.bpb_fsinfo       = DOSFS_HTOFS(1);
                        boot->bpb71.bpb_bkboot       = DOSFS_HTOFS(6);
                        boot->bpb71.bs_drv_num       = 0x80;
                        boot->bpb71.bs_nt_reserved   = 0x00;
                        boot->bpb71.bs_boot_sig      = 0x29;
                        boot->bpb71.bs_vol_id        = DOSFS_HTOFL(product);

                        memcpy(boot->bpb71.bs_vol_lab, "NO NAME    ", sizeof(boot->bpb71.bs_vol_lab));
                        memcpy(boot->bpb71.bs_fil_sys_type, "FAT32   ", sizeof(boot->bpb71.bs_fil_sys_type));

                        status = (*device->interface->write)(device->context, boot_blkno, (uint8_t*)boot, 1, NULL);

                        if (status == F_NO_ERROR)
                        {
                            status = (*device->interface->write)(device->context, boot_blkno +6, (uint8_t*)boot, 1, NULL);

                            if (status == F_NO_ERROR)
                            {
                                fsinfo = (dosfs_fsinfo_t*)data;
                                memset(fsinfo, 0, DOSFS_BLK_SIZE);

                                fsinfo->fsi_lead_sig   = DOSFS_HTOFL(0x41615252);
                                fsinfo->fsi_struc_sig  = DOSFS_HTOFL(0x61417272);
                                fsinfo->fsi_free_count = DOSFS_HTOFL(clscnt);
                                fsinfo->fsi_nxt_free   = DOSFS_HTOFL(3);
                                fsinfo->fsi_trail_sig  = DOSFS_HTOFL(0xaa550000);
                                        
                                status = (*device->interface->write)(device->context, boot_blkno +1, (uint8_t*)fsinfo, 1, NULL);
                                        
                                if (status == F_NO_ERROR)
                                {
                                    status = (*device->interface->write)(device->context, boot_blkno +7, (uint8_t*)fsinfo, 1, NULL);

                                    if (status == F_NO_ERROR)
                                    {
                                        memset(boot, 0, DOSFS_BLK_SIZE);
                                    
                                        boot->bs.bs_trail_sig = DOSFS_HTOFS(0xaa55);
                                                
                                        status = (*device->interface->write)(device->context, boot_blkno +2, (uint8_t*)boot, 1, NULL);
                                    
                                        if (status == F_NO_ERROR)
                                        {
                                            status = (*device->interface->write)(device->context, boot_blkno +8, (uint8_t*)boot, 1, NULL);
                                        }
                                    }
                                }
                            }
                        }
                    }

                    if (status == F_NO_ERROR)
                    {
                        /* For FAT32 above root_clsno got forced to be 2, so that fat1/fat2/root are contiguous.
                         */

                        memset(data, 0, DOSFS_BLK_SIZE);
                        
                        blkno = fat1_blkno;
                        blkcnt = (root_blkno - fat1_blkno) + root_blkcnt;
                        
                        do
                        {
                            status = (*device->interface->write)(device->context, blkno, data, 1, (blkcnt == 1) ? true : false);
                            
                            blkno++;
                            blkcnt--;
                        }
                        while ((status == F_NO_ERROR) && blkcnt);
                        
                        if (status == F_NO_ERROR)
                        {
                            /* Write first FAT1/FAT2 entry */

#if (DOSFS_CONFIG_FAT12_SUPPORTED == 1)
                            if (fattype == DOSFS_VOLUME_TYPE_FAT12)
                            {
                                ((uint8_t*)data)[0] = 0xf8;
                                ((uint8_t*)data)[1] = 0xff;
                                ((uint8_t*)data)[2] = 0xff;
                            }
                            else
#endif /* (DOSFS_CONFIG_FAT12_SUPPORTED == 1) */
                            {
                                if (fattype == DOSFS_VOLUME_TYPE_FAT16)
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
                                
                            status = (*device->interface->write)(device->context, fat1_blkno, data, 1, NULL);
                
                            if (status == F_NO_ERROR)
                            {
                                if (fat2_blkno)
                                {
                                    status = (*device->interface->write)(device->context, fat2_blkno, data, 1, NULL);
                                }
                            }
                        }

                        if (status == F_NO_ERROR)
                        {
                            if (fattype != DOSFS_VOLUME_TYPE_FAT32)
                            {
                                user_blkno = (root_blkno + root_blkcnt);
                                user_blkcnt = (clscnt << cls_blk_shift);
                            }
                            else
                            {
                                user_blkno = (root_blkno + root_blkcnt);
                                user_blkcnt = ((clscnt -1) << cls_blk_shift);
                            }

                            status = (*device->interface->discard)(device->context, user_blkno, user_blkcnt);
                        }
                    }
                }
            }
        }
    }

    return status;
}
