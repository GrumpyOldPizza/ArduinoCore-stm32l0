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

#if defined(STM32L082xx)

#include "armv6m.h"
#include "stm32l0xx.h"

#include "stm32l0_aes.h"

typedef struct _stm32l0_aes_device_t {
    uint32_t  K[4];
    uint8_t   K1[16];      /* CCM compyted mac,  CMAC K1 key */
    uint8_t   K2[16];      /* CCM decrypted mac, CMAC K2 key */
    uint8_t   IV[16];      /* CMAC state */
    uint8_t   M[16];       /* partial message chunk for CMAC, scratch pad otherwise */
    uint32_t  index;
} stm32l0_aes_device_t;

static stm32l0_aes_device_t stm32l0_aes_device;

/* AVOID EXTERNAL REFERENCES */
static void stm32l0_aes_memzero(uint8_t *d, uint32_t n)
{
    uint8_t *d_e;

    d_e = d + n;

    while (d != d_e)
    {
        *d++ = 0x00;
    }
}

/* AVOID EXTERNAL REFERENCES */
static void stm32l0_aes_memcpy(uint8_t *d, const uint8_t *s, uint32_t n)
{
    uint8_t *d_e;

    d_e = d + n;

    while (d != d_e)
    {
        *d++ = *s++;
    }
}

static void stm32l0_aes_engine(const uint8_t *iv, const uint8_t *in, uint8_t *out, uint32_t n, uint32_t mode, bool discard)
{
    unsigned int i, size, tail;
    const uint8_t *in_e;
    uint32_t data[4];

    /* AVOID EXTERNAL REFERENCES */
    RCC->AHBENR |= RCC_AHBENR_CRYPEN;
    RCC->AHBENR;

    AES->KEYR0 = stm32l0_aes_device.K[0];
    AES->KEYR1 = stm32l0_aes_device.K[1];
    AES->KEYR2 = stm32l0_aes_device.K[2];
    AES->KEYR3 = stm32l0_aes_device.K[3];

    if (mode & AES_CR_MODE_1)
    {
        AES->CR = AES_CR_DATATYPE_1 | AES_CR_MODE_0;

        AES->CR |= AES_CR_EN;

        while (!(AES->SR & AES_SR_CCF))
        {
        }

        AES->CR |= AES_CR_CCFC;
    }

    AES->CR = AES_CR_DATATYPE_1 | mode;

    if (iv)
    {
        AES->IVR0 = ((iv[12] << 24) | (iv[13] << 16) |(iv[14] <<  8) |(iv[15] <<  0));
        AES->IVR1 = ((iv[ 8] << 24) | (iv[ 9] << 16) |(iv[10] <<  8) |(iv[11] <<  0));
        AES->IVR2 = ((iv[ 4] << 24) | (iv[ 5] << 16) |(iv[ 6] <<  8) |(iv[ 7] <<  0));
        AES->IVR3 = ((iv[ 0] << 24) | (iv[ 1] << 16) |(iv[ 2] <<  8) |(iv[ 3] <<  0));
    }
    else
    {
        AES->IVR0 = 0;
        AES->IVR1 = 0;
        AES->IVR2 = 0;
        AES->IVR3 = 0;
    }

    AES->CR |= AES_CR_EN;

    size = n & ~15;

    in_e = in + size;

    while (in != in_e)
    {
        data[0] = ((in[ 3] << 24) | (in[ 2] << 16) |(in[ 1] <<  8) |(in[ 0] <<  0));
        data[1] = ((in[ 7] << 24) | (in[ 6] << 16) |(in[ 5] <<  8) |(in[ 4] <<  0));
        data[2] = ((in[11] << 24) | (in[10] << 16) |(in[ 9] <<  8) |(in[ 8] <<  0));
        data[3] = ((in[15] << 24) | (in[14] << 16) |(in[13] <<  8) |(in[12] <<  0));

        in += 16;

        AES->DINR = data[0];
        AES->DINR = data[1];
        AES->DINR = data[2];
        AES->DINR = data[3];

        while (!(AES->SR & AES_SR_CCF))
        {
        }

        AES->CR |= AES_CR_CCFC;
        
        data[0] = AES->DOUTR;
        data[1] = AES->DOUTR;
        data[2] = AES->DOUTR;
        data[3] = AES->DOUTR;
        
        if (!discard)
        {
            out[ 0] = data[0] >>  0;
            out[ 1] = data[0] >>  8;
            out[ 2] = data[0] >> 16;
            out[ 3] = data[0] >> 24;
            
            out[ 4] = data[1] >>  0;
            out[ 5] = data[1] >>  8;
            out[ 6] = data[1] >> 16;
            out[ 7] = data[1] >> 24;
            
            out[ 8] = data[2] >>  0;
            out[ 9] = data[2] >>  8;
            out[10] = data[2] >> 16;
            out[11] = data[2] >> 24;
            
            out[12] = data[3] >>  0;
            out[13] = data[3] >>  8;
            out[14] = data[3] >> 16;
            out[15] = data[3] >> 24;

            out += 16;
        }
    }

    tail = n - size;

    if (tail)
    {
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;

        for (i = 0; i < tail; i++)
        {
            ((uint8_t*)(&data[0]))[i] = in[i];
        }

        AES->DINR = data[0];
        AES->DINR = data[1];
        AES->DINR = data[2];
        AES->DINR = data[3];

        while (!(AES->SR & AES_SR_CCF))
        {
        }

        AES->CR |= AES_CR_CCFC;
        
        data[0] = AES->DOUTR;
        data[1] = AES->DOUTR;
        data[2] = AES->DOUTR;
        data[3] = AES->DOUTR;
        
        if (!discard)
        {
            for (i = 0; i < tail; i++)
            {
                out[i] = ((uint8_t*)(&data[0]))[i];
            }
        }
    }

    if (discard)
    {
        out[ 0] = data[0] >>  0;
        out[ 1] = data[0] >>  8;
        out[ 2] = data[0] >> 16;
        out[ 3] = data[0] >> 24;
            
        out[ 4] = data[1] >>  0;
        out[ 5] = data[1] >>  8;
        out[ 6] = data[1] >> 16;
        out[ 7] = data[1] >> 24;
        
        out[ 8] = data[2] >>  0;
        out[ 9] = data[2] >>  8;
        out[10] = data[2] >> 16;
        out[11] = data[2] >> 24;
        
        out[12] = data[3] >>  0;
        out[13] = data[3] >>  8;
        out[14] = data[3] >> 16;
        out[15] = data[3] >> 24;
    }

    AES->CR &= ~AES_CR_EN;

    AES->KEYR0 = 0;
    AES->KEYR1 = 0;
    AES->KEYR2 = 0;
    AES->KEYR3 = 0;

    AES->IVR0 = 0;
    AES->IVR1 = 0;
    AES->IVR2 = 0;
    AES->IVR3 = 0;

    /* AVOID EXTERNAL REFERENCES */
    RCC->AHBENR &= ~RCC_AHBENR_CRYPEN;
}

void stm32l0_aes_set_key(const uint8_t *key)
{
    stm32l0_aes_device.K[0] = ((key[12] << 24) | (key[13] << 16) |(key[14] <<  8) |(key[15] <<  0));
    stm32l0_aes_device.K[1] = ((key[ 8] << 24) | (key[ 9] << 16) |(key[10] <<  8) |(key[11] <<  0));
    stm32l0_aes_device.K[2] = ((key[ 4] << 24) | (key[ 5] << 16) |(key[ 6] <<  8) |(key[ 7] <<  0));
    stm32l0_aes_device.K[3] = ((key[ 0] << 24) | (key[ 1] << 16) |(key[ 2] <<  8) |(key[ 3] <<  0));
}

void stm32l0_aes_ecb_encrypt(const uint8_t *in, uint8_t *out, uint32_t n)
{
    stm32l0_aes_engine(NULL, in, out, n, 0, false);
}

void stm32l0_aes_ecb_decrypt(const uint8_t *in, uint8_t *out, uint32_t n)
{
    stm32l0_aes_engine(NULL, in, out, n, AES_CR_MODE_1, false);
}

void stm32l0_aes_cbc_encrypt(const uint8_t *iv, const uint8_t *in, uint8_t *out, uint32_t n)
{
    stm32l0_aes_engine(iv, in, out, n, AES_CR_CHMOD_0, false);
}

void stm32l0_aes_cbc_decrypt(const uint8_t *iv, const uint8_t *in, uint8_t *out, uint32_t n)
{
    stm32l0_aes_engine(iv, in, out, n, AES_CR_MODE_1 | AES_CR_CHMOD_0, false);
}

void stm32l0_aes_ctr_xcrypt(const uint8_t *ctr, const uint8_t *in, uint8_t *out, uint32_t n)
{
    stm32l0_aes_engine(ctr, in, out, n, AES_CR_CHMOD_1, false);
}

static void stm32l0_aes_ccm_authenticate(uint32_t msize, const uint8_t *nonce, const uint8_t *adata, uint32_t asize, const uint8_t *in, uint32_t n)
{
    unsigned int size;

    /* AUTHENTICATION */

    stm32l0_aes_device.M[0] = (adata ? 0x41 : 0x01) |  (((msize - 2) / 2) << 3);
    stm32l0_aes_device.M[14] = (n >> 8);
    stm32l0_aes_device.M[15] = (n >> 0);
    stm32l0_aes_memcpy(&stm32l0_aes_device.M[1], nonce, 13);

    stm32l0_aes_engine(NULL, &stm32l0_aes_device.M[0], &stm32l0_aes_device.K1[0], 16, AES_CR_CHMOD_0, true);

    if (adata)
    {
        stm32l0_aes_memzero(&stm32l0_aes_device.M[0], 16);

        stm32l0_aes_device.M[0] = asize >> 8;
        stm32l0_aes_device.M[1] = asize >> 0;
        
        size = 14;
        
        if (size > asize)
        {
            size = asize;
        }
        
        stm32l0_aes_memcpy(&stm32l0_aes_device.M[2], adata, size);
        adata += size;
        asize -= size;

        stm32l0_aes_engine(&stm32l0_aes_device.K1[0], &stm32l0_aes_device.M[0], &stm32l0_aes_device.K1[0], 16, AES_CR_CHMOD_0, true);

        if (asize)
        {
            stm32l0_aes_engine(&stm32l0_aes_device.K1[0], adata, &stm32l0_aes_device.K1[0], asize, AES_CR_CHMOD_0, true);
        }
    }

    if (n)
    {
        stm32l0_aes_engine(&stm32l0_aes_device.K1[0], in, &stm32l0_aes_device.K1[0], n, AES_CR_CHMOD_0, true);
    }
}

/* "out" contains encrypted "in" plus the MAC appended. "in" is only the unecrypted data.
 */
void stm32l0_aes_ccm_encrypt(uint32_t msize, const uint8_t *nonce, const uint8_t *adata, uint32_t asize, const uint8_t *in, uint8_t *out, uint32_t n)
{
    stm32l0_aes_ccm_authenticate(msize, nonce, adata, asize, in, n);

    stm32l0_aes_device.M[0] = 0x01;
    stm32l0_aes_device.M[14] = 0x00;
    stm32l0_aes_device.M[15] = 0x01;
    stm32l0_aes_memcpy(&stm32l0_aes_device.M[1], nonce, 13);

    if (n)
    {
        stm32l0_aes_engine(&stm32l0_aes_device.M[0], in, out, n, AES_CR_CHMOD_1, false);
    }

    stm32l0_aes_device.M[14] = 0x00;
    stm32l0_aes_device.M[15] = 0x00;

    stm32l0_aes_engine(&stm32l0_aes_device.M[0], &stm32l0_aes_device.K1[0], out + n, msize, AES_CR_CHMOD_1, false);

#if 0
    stm32l0_aes_engine(&stm32l0_aes_device.M[0], &stm32l0_aes_device.K1[0], &stm32l0_aes_device.M[0], 16, AES_CR_CHMOD_1, false);
    stm32l0_aes_memcpy(out + n, &stm32l0_aes_device.M[0], msize);
#endif
}

/* "in" contains encrypted data plus the MAC appended. "out" is only the decrypted data.
 */
bool stm32l0_aes_ccm_decrypt(uint32_t msize, const uint8_t *nonce, const uint8_t *adata, uint32_t asize, const uint8_t *in, uint8_t *out, uint32_t n)
{
    unsigned int i;
    bool check = true;

    stm32l0_aes_device.M[0] = 0x01;
    stm32l0_aes_device.M[14] = 0x00;
    stm32l0_aes_device.M[15] = 0x01;
    stm32l0_aes_memcpy(&stm32l0_aes_device.M[1], nonce, 13);

    if (n)
    {
        stm32l0_aes_engine(&stm32l0_aes_device.M[0], in, out, n, AES_CR_CHMOD_1, false);
    }

    stm32l0_aes_device.M[14] = 0x00;
    stm32l0_aes_device.M[15] = 0x00;

    stm32l0_aes_engine(&stm32l0_aes_device.M[0], in + n, &stm32l0_aes_device.K2[0], msize, AES_CR_CHMOD_1, false);

    stm32l0_aes_ccm_authenticate(msize, nonce, adata, asize, out, n);

    for (i = 0; i < msize; i++)
    {
        if (stm32l0_aes_device.K1[i] != stm32l0_aes_device.K2[i])
        {
            check = false;
            break;
        }
    }

    return check;
}

void stm32l0_aes_cmac_init(void)
{
    stm32l0_aes_memzero(&stm32l0_aes_device.IV[0], 16);
    stm32l0_aes_memzero(&stm32l0_aes_device.M[0], 16);
    stm32l0_aes_device.index = 0;
}

void stm32l0_aes_cmac_update(const uint8_t *in, uint32_t n)
{
    uint32_t size;

    if (n)
    {
        if (stm32l0_aes_device.index == 16)
        {
            stm32l0_aes_engine(&stm32l0_aes_device.IV[0], &stm32l0_aes_device.M[0], &stm32l0_aes_device.IV[0], 16, AES_CR_CHMOD_0, true);
            stm32l0_aes_device.index = 0;
        }

        if (stm32l0_aes_device.index != 0)
        {
            size = 16 - stm32l0_aes_device.index;
            
            if (size > n)
            {
                size =  n;
            }
            
            stm32l0_aes_memcpy(&stm32l0_aes_device.M[stm32l0_aes_device.index], in, size);
            stm32l0_aes_device.index += size;
            
            in += size;
            n -= size;
        }

        if (n)
        {
            if (stm32l0_aes_device.index == 16)
            {
                stm32l0_aes_engine(&stm32l0_aes_device.IV[0], &stm32l0_aes_device.M[0], &stm32l0_aes_device.IV[0], 16, AES_CR_CHMOD_0, true);
                stm32l0_aes_device.index = 0;
            }

            if (n > 16)
            {
                size = n & ~15;

                if (size == n)
                {
                    size -= 16;
                }

                stm32l0_aes_engine(&stm32l0_aes_device.IV[0], in, &stm32l0_aes_device.IV[0], size, AES_CR_CHMOD_0, true);
                
                in += size;
                n -= size;
            }

            stm32l0_aes_memcpy(&stm32l0_aes_device.M[0], in, n);
            stm32l0_aes_device.index = n;
        }
    }
}

void stm32l0_aes_cmac_final(uint8_t *out)
{
    unsigned int i;
    uint8_t *K;
    
    stm32l0_aes_memzero(&stm32l0_aes_device.K2[0], 16);
    stm32l0_aes_engine(NULL, &stm32l0_aes_device.K2[0], &stm32l0_aes_device.K2[0], 16, 0, false);
        
    for (i = 0; i < 15; i++)
    {
        stm32l0_aes_device.K1[i] = (stm32l0_aes_device.K2[i] << 1) | (stm32l0_aes_device.K2[i+1] >> 7);
    }

    stm32l0_aes_device.K1[15] = (stm32l0_aes_device.K2[15] << 1);
        
    if (stm32l0_aes_device.K2[0] & 0x80) 
    {
        stm32l0_aes_device.K1[15] ^= 0x87;
    }

    if (stm32l0_aes_device.index == 16)
    {
        K = &stm32l0_aes_device.K1[0];
    }
    else
    {
        for (i = 0; i < 15; i++)
        {
            stm32l0_aes_device.K2[i] = (stm32l0_aes_device.K1[i] << 1) | (stm32l0_aes_device.K1[i+1] >> 7);
        }
        
        stm32l0_aes_device.K2[15] = (stm32l0_aes_device.K1[15] << 1);

        if (stm32l0_aes_device.K1[0] & 0x80) 
        {
            stm32l0_aes_device.K2[15] ^= 0x87;
        }

        K = &stm32l0_aes_device.K2[0];

        stm32l0_aes_memzero(&stm32l0_aes_device.M[stm32l0_aes_device.index], 16 - stm32l0_aes_device.index);
        stm32l0_aes_device.M[stm32l0_aes_device.index] = 0x80;
    }

    for (i = 0; i < 16; i++)
    {
        stm32l0_aes_device.M[i] ^= K[i];
    }

    stm32l0_aes_engine(&stm32l0_aes_device.IV[0], &stm32l0_aes_device.M[0], out, 16, AES_CR_CHMOD_0, true);
}

#endif /* STM32L082xx */
