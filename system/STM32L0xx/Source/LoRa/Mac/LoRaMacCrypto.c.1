/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech
 ___ _____ _   ___ _  _____ ___  ___  ___ ___
/ __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
\__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
|___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
embedded.connectivity.solutions===============

Description: LoRa MAC layer implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis ( Semtech ), Gregory Cristian ( Semtech ) and Daniel Jaeckle ( STACKFORCE )
*/

#include <stdlib.h>
#include <stdint.h>

#include "LoRaMacCrypto.h"
#include "stm32l0_aes.h"

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
