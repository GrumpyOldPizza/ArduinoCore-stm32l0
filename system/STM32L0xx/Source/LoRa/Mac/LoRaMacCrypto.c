/*!
 * \file      LoRaMacCrypto.c
 *
 * \brief     LoRa MAC layer cryptography implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Daniel Jaeckle ( STACKFORCE )
 */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "utilities.h"

#include "LoRaMacCrypto.h"

#if defined(STM32L082xx)

#include "stm32l0_aes.h"

void LoRaMacComputeMic( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint32_t *mic )
{
    uint8_t B0[16], T[16];

    stm32l0_aes_set_key(key);

    B0[ 0] = 0x49;
    B0[ 1] = 0x00;
    B0[ 2] = 0x00;
    B0[ 3] = 0x00;
    B0[ 4] = 0x00;
    B0[ 5] = dir;
    B0[ 6] = address >> 0;
    B0[ 7] = address >> 8;
    B0[ 8] = address >> 16;
    B0[ 9] = address >> 24;
    B0[10] = sequenceCounter >> 0;
    B0[11] = sequenceCounter >> 8;
    B0[12] = sequenceCounter >> 16;
    B0[13] = sequenceCounter >> 24;
    B0[14] = 0x00;
    B0[15] = size;

    stm32l0_aes_cmac_init();
    stm32l0_aes_cmac_update(&B0[0], 16);
    stm32l0_aes_cmac_update(buffer, size);
    stm32l0_aes_cmac_final(&T[0]);

    *mic = (T[3] << 24) | (T[2] << 16) | (T[1] << 8) | (T[0] << 0);
}

void LoRaMacPayloadEncrypt( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint8_t *encBuffer )
{
    uint8_t CTR[16];

    stm32l0_aes_set_key(key);

    CTR[ 0] = 0x01;
    CTR[ 1] = 0x00;
    CTR[ 2] = 0x00;
    CTR[ 3] = 0x00;
    CTR[ 4] = 0x00;
    CTR[ 5] = dir;
    CTR[ 6] = address >> 0;
    CTR[ 7] = address >> 8;
    CTR[ 8] = address >> 16;
    CTR[ 9] = address >> 24;
    CTR[10] = sequenceCounter >> 0;
    CTR[11] = sequenceCounter >> 8;
    CTR[12] = sequenceCounter >> 16;
    CTR[13] = sequenceCounter >> 24;
    CTR[14] = 0x00;
    CTR[15] = 0x01;

    stm32l0_aes_ctr_xcrypt(&CTR[0], buffer, encBuffer, size);
}

void LoRaMacPayloadDecrypt( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint8_t *decBuffer )
{
    LoRaMacPayloadEncrypt( buffer, size, key, address, dir, sequenceCounter, decBuffer );
}

void LoRaMacJoinComputeMic( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t *mic )
{
    uint8_t T[16];

    stm32l0_aes_set_key(key);
    stm32l0_aes_cmac_init();
    stm32l0_aes_cmac_update(buffer, size);
    stm32l0_aes_cmac_final(&T[0]);

    *mic = (T[3] << 24) | (T[2] << 16) | (T[1] << 8) | (T[0] << 0);
}

void LoRaMacJoinDecrypt( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint8_t *decBuffer )
{
    stm32l0_aes_set_key(key);
    stm32l0_aes_ecb_encrypt(buffer, decBuffer, size);
}

void LoRaMacJoinComputeSKeys( const uint8_t *key, const uint8_t *appNonce, uint16_t devNonce, uint8_t *nwkSKey, uint8_t *appSKey )
{
    uint8_t X[16];

    stm32l0_aes_set_key(key);

    X[ 0] = 0x01;
    X[ 1] = appNonce[0];
    X[ 2] = appNonce[1];
    X[ 3] = appNonce[2];
    X[ 4] = appNonce[3];
    X[ 5] = appNonce[4];
    X[ 6] = appNonce[5];
    X[ 7] = devNonce >> 0;
    X[ 8] = devNonce >> 8;
    X[ 9] = 0x00;
    X[10] = 0x00;
    X[11] = 0x00;
    X[12] = 0x00;
    X[13] = 0x00;
    X[14] = 0x00;
    X[15] = 0x00;

    stm32l0_aes_ecb_encrypt(&X[0], nwkSKey, 16);

    X[0] = 0x02;

    stm32l0_aes_ecb_encrypt(&X[0], appSKey, 16);
}

#else /* defined(STM32L082xx) */

#include "aes.h"
#include "cmac.h"

/*!
 * CMAC/AES Message Integrity Code (MIC) Block B0 size
 */
#define LORAMAC_MIC_BLOCK_B0_SIZE                   16

/*!
 * MIC field computation initial data
 */
static uint8_t MicBlockB0[] = { 0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                              };

/*!
 * Contains the computed MIC field.
 *
 * \remark Only the 4 first bytes are used
 */
static uint8_t Mic[16];

/*!
 * Encryption aBlock and sBlock
 */
static uint8_t aBlock[] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                          };
static uint8_t sBlock[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                          };

/*!
 * AES computation context variable
 */
static aes_context AesContext;

/*!
 * CMAC computation context variable
 */
static AES_CMAC_CTX AesCmacCtx[1];

/*!
 * \brief Computes the LoRaMAC frame MIC field  
 *
 * \param [IN]  buffer          Data buffer
 * \param [IN]  size            Data buffer size
 * \param [IN]  key             AES key to be used
 * \param [IN]  address         Frame address
 * \param [IN]  dir             Frame direction [0: uplink, 1: downlink]
 * \param [IN]  sequenceCounter Frame sequence counter
 * \param [OUT] mic Computed MIC field
 */
void LoRaMacComputeMic( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint32_t *mic )
{
    MicBlockB0[5] = dir;
    
    MicBlockB0[6] = ( address ) & 0xFF;
    MicBlockB0[7] = ( address >> 8 ) & 0xFF;
    MicBlockB0[8] = ( address >> 16 ) & 0xFF;
    MicBlockB0[9] = ( address >> 24 ) & 0xFF;

    MicBlockB0[10] = ( sequenceCounter ) & 0xFF;
    MicBlockB0[11] = ( sequenceCounter >> 8 ) & 0xFF;
    MicBlockB0[12] = ( sequenceCounter >> 16 ) & 0xFF;
    MicBlockB0[13] = ( sequenceCounter >> 24 ) & 0xFF;

    MicBlockB0[15] = size & 0xFF;

    AES_CMAC_Init( AesCmacCtx );

    AES_CMAC_SetKey( AesCmacCtx, key );

    AES_CMAC_Update( AesCmacCtx, MicBlockB0, LORAMAC_MIC_BLOCK_B0_SIZE );
    
    AES_CMAC_Update( AesCmacCtx, buffer, size & 0xFF );
    
    AES_CMAC_Final( Mic, AesCmacCtx );
    
    *mic = ( uint32_t )( ( uint32_t )Mic[3] << 24 | ( uint32_t )Mic[2] << 16 | ( uint32_t )Mic[1] << 8 | ( uint32_t )Mic[0] );
}

void LoRaMacPayloadEncrypt( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint8_t *encBuffer )
{
    uint16_t i;
    uint8_t bufferIndex = 0;
    uint16_t ctr = 1;

    memset( AesContext.ksch, '\0', 240 );
    aes_set_key( key, 16, &AesContext );

    aBlock[5] = dir;

    aBlock[6] = ( address ) & 0xFF;
    aBlock[7] = ( address >> 8 ) & 0xFF;
    aBlock[8] = ( address >> 16 ) & 0xFF;
    aBlock[9] = ( address >> 24 ) & 0xFF;

    aBlock[10] = ( sequenceCounter ) & 0xFF;
    aBlock[11] = ( sequenceCounter >> 8 ) & 0xFF;
    aBlock[12] = ( sequenceCounter >> 16 ) & 0xFF;
    aBlock[13] = ( sequenceCounter >> 24 ) & 0xFF;

    while( size >= 16 )
    {
        aBlock[15] = ( ( ctr ) & 0xFF );
        ctr++;
        aes_encrypt( aBlock, sBlock, &AesContext );
        for( i = 0; i < 16; i++ )
        {
            encBuffer[bufferIndex + i] = buffer[bufferIndex + i] ^ sBlock[i];
        }
        size -= 16;
        bufferIndex += 16;
    }

    if( size > 0 )
    {
        aBlock[15] = ( ( ctr ) & 0xFF );
        aes_encrypt( aBlock, sBlock, &AesContext );
        for( i = 0; i < size; i++ )
        {
            encBuffer[bufferIndex + i] = buffer[bufferIndex + i] ^ sBlock[i];
        }
    }
}

void LoRaMacPayloadDecrypt( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint8_t *decBuffer )
{
    LoRaMacPayloadEncrypt( buffer, size, key, address, dir, sequenceCounter, decBuffer );
}

void LoRaMacJoinComputeMic( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t *mic )
{
    AES_CMAC_Init( AesCmacCtx );

    AES_CMAC_SetKey( AesCmacCtx, key );

    AES_CMAC_Update( AesCmacCtx, buffer, size & 0xFF );

    AES_CMAC_Final( Mic, AesCmacCtx );

    *mic = ( uint32_t )( ( uint32_t )Mic[3] << 24 | ( uint32_t )Mic[2] << 16 | ( uint32_t )Mic[1] << 8 | ( uint32_t )Mic[0] );
}

void LoRaMacJoinDecrypt( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint8_t *decBuffer )
{
    memset( AesContext.ksch, '\0', 240 );
    aes_set_key( key, 16, &AesContext );
    aes_encrypt( buffer, decBuffer, &AesContext );
    // Check if optional CFList is included
    if( size >= 16 )
    {
        aes_encrypt( buffer + 16, decBuffer + 16, &AesContext );
    }
}

void LoRaMacJoinComputeSKeys( const uint8_t *key, const uint8_t *appNonce, uint16_t devNonce, uint8_t *nwkSKey, uint8_t *appSKey )
{
    uint8_t nonce[16];
    uint8_t *pDevNonce = ( uint8_t * )&devNonce;
    
    memset( AesContext.ksch, '\0', 240 );
    aes_set_key( key, 16, &AesContext );

    memset( nonce, 0, sizeof( nonce ) );
    nonce[0] = 0x01;
    memcpy( nonce + 1, appNonce, 6 );
    memcpy( nonce + 7, pDevNonce, 2 );
    aes_encrypt( nonce, nwkSKey, &AesContext );

    memset( nonce, 0, sizeof( nonce ) );
    nonce[0] = 0x02;
    memcpy( nonce + 1, appNonce, 6 );
    memcpy( nonce + 7, pDevNonce, 2 );
    aes_encrypt( nonce, appSKey, &AesContext );
}

#endif /* defined(STM32L082xx) */

