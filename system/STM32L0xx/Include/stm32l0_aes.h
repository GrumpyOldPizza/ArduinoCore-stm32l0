/*
 * Copyright (c) 2017-2018 Thomas Roell.  All rights reserved.
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

#if !defined(_STM32L0_AES_H)
#define _STM32L0_AES_H

#if defined(STM32L082xx)

#include "armv6m.h"
#include "stm32l0xx.h"

#ifdef __cplusplus
extern "C" {
#endif

extern void stm32l0_aes_set_key(const uint8_t *key);
extern void stm32l0_aes_ecb_encrypt(const uint8_t *in, uint8_t *out, uint32_t n);
extern void stm32l0_aes_ecb_decrypt(const uint8_t *in, uint8_t *out, uint32_t n);
extern void stm32l0_aes_cbc_encrypt(const uint8_t *iv, const uint8_t *in, uint8_t *out, uint32_t n);
extern void stm32l0_aes_cbc_decrypt(const uint8_t *iv, const uint8_t *in, uint8_t *out, uint32_t n);
extern void stm32l0_aes_ctr_xcrypt(const uint8_t *ctr, const uint8_t *in, uint8_t *out, uint32_t n);
extern void stm32l0_aes_ccm_encrypt(uint32_t msize, const uint8_t *nonce, const uint8_t *adata, uint32_t asize, const uint8_t *in, uint8_t *out, uint32_t n);
extern bool stm32l0_aes_ccm_decrypt(uint32_t msize, const uint8_t *nonce, const uint8_t *adata, uint32_t asize, const uint8_t *in, uint8_t *out, uint32_t n);
extern void stm32l0_aes_cmac_init(void);
extern void stm32l0_aes_cmac_update(const uint8_t *in, uint32_t n);
extern void stm32l0_aes_cmac_final(uint8_t *out);

#ifdef __cplusplus
}
#endif

#endif /* defined(STM32L082xx) */

#endif /* _STM32L0_AES_H */
