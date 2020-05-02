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

#include "stm32l0_sdspi.h"
#include "stm32l0_system.h"

#include "armv6m_systick.h"

static stm32l0_sdspi_t stm32l0_sdspi;

#define SD_CMD_GO_IDLE_STATE           (0)
#define SD_CMD_SEND_OP_COND            (1)
#define SD_CMD_SWITCH_FUNC             (6)
#define SD_CMD_SEND_IF_COND            (8)
#define SD_CMD_SEND_CSD                (9)
#define SD_CMD_SEND_CID                (10)
#define SD_CMD_STOP_TRANSMISSION       (12)
#define SD_CMD_SEND_STATUS             (13)
#define SD_CMD_SET_BLOCKLEN            (16)
#define SD_CMD_READ_SINGLE_BLOCK       (17)
#define SD_CMD_READ_MULTIPLE_BLOCK     (18)
#define SD_CMD_WRITE_SINGLE_BLOCK      (24)
#define SD_CMD_WRITE_MULTIPLE_BLOCK    (25)
#define SD_CMD_PROGRAM_CSD             (27)
#define SD_CMD_ERASE_WR_BLK_START_ADDR (32)
#define SD_CMD_ERASE_WR_BLK_END_ADDR   (33)
#define SD_CMD_ERASE                   (38)
#define SD_CMD_APP_CMD                 (55)
#define SD_CMD_READ_OCR                (58)
#define SD_CMD_CRC_ON_OFF              (59)

#define SD_ACMD_SD_STATUS              (13)
#define SD_ACMD_SEND_NUM_WR_BLOCKS     (22)
#define SD_ACMD_SET_WR_BLK_ERASE_COUNT (23)
#define SD_ACMD_SD_SEND_OP_COND        (41)
#define SD_ACMD_SET_CLR_CARD_DETECT    (42)
#define SD_ACMD_SEND_SCR               (51)

#define SD_R1_VALID_MASK               0x80
#define SD_R1_VALID_DATA               0x00
#define SD_R1_IN_IDLE_STATE            0x01
#define SD_R1_ERASE_RESET              0x02
#define SD_R1_ILLEGAL_COMMAND          0x04
#define SD_R1_COM_CRC_ERROR            0x08
#define SD_R1_ERASE_SEQUENCE_ERROR     0x10
#define SD_R1_ADDRESS_ERROR            0x20
#define SD_R1_PARAMETER_ERROR          0x40
#define SD_R1_ERROR_MASK               0x76

#define SD_R2_CARD_IS_LOCKED           0x01
#define SD_R2_WP_ERASE_SKIP            0x02
#define SD_R2_LOCK_UNLOCK_FAILED       0x02
#define SD_R2_ERROR                    0x04
#define SD_R2_CC_ERROR                 0x08
#define SD_R2_CARD_ECC_FAILED          0x10
#define SD_R2_WP_VIOLATION             0x20
#define SD_R2_ERASE_PARAM              0x40
#define SD_R2_OUT_OF_RANGE             0x80
#define SD_R2_CSD_OVERWRITE            0x80
#define SD_R2_ERROR_MASK               0xfe

#define SD_READY_TOKEN                 0xff    /* host -> card, card -> host */
#define SD_START_READ_TOKEN            0xfe    /* card -> host */
#define SD_START_WRITE_SINGLE_TOKEN    0xfe    /* host -> card */
#define SD_START_WRITE_MULTIPLE_TOKEN  0xfc    /* host -> card */
#define SD_STOP_TRANSMISSION_TOKEN     0xfd    /* host -> card */

/* data response token for WRITE */
#define SD_DATA_RESPONSE_MASK          0x1f
#define SD_DATA_RESPONSE_ACCEPTED      0x05
#define SD_DATA_RESPONSE_CRC_ERROR     0x0b
#define SD_DATA_RESPONSE_WRITE_ERROR   0x0d

/* data error token for READ */
#define SD_DATA_ERROR_TOKEN_VALID_MASK 0xf0
#define SD_DATA_ERROR_TOKEN_VALID_DATA 0x00
#define SD_DATA_ERROR_EXECUTION_ERROR  0x01
#define SD_DATA_ERROR_CC_ERROR         0x02
#define SD_DATA_ERROR_CARD_ECC_FAILED  0x04
#define SD_DATA_ERROR_OUT_OF_RANGE     0x08

#define SD_READ_TIMEOUT       100
#define SD_WRITE_TIMEOUT      250
#define SD_WRITE_STOP_TIMEOUT 250

static inline __attribute__((optimize("O3"),always_inline)) uint32_t stm32l0_sdspi_slice(const uint8_t *data, uint32_t size, uint32_t start, uint32_t width)
{
    uint32_t mask, shift;

    data += ((size >> 3) -1);
    data -= (start >> 3);

    shift = start & 7;
    mask = (data[0] >> shift);

    if ((width + shift) > 8)
    {
        mask |= (data[-1] << (8 - shift));

        if ((width + shift) > 16)
        {
            mask |= (data[-2] << (16 - shift));

            if ((width + shift) > 24)
            {
                mask |= (data[-3] << (24 - shift));

                if ((width + shift) > 32)
                {
                    mask |= (data[-4] << (32 - shift));
                }
            }
        }
    }

    return mask & (0xffffffff >> (32 - width));
}

static const uint8_t stm32l0_sdspi_crc7_table[256]= {
    0x00, 0x09, 0x12, 0x1b, 0x24, 0x2d, 0x36, 0x3f,
    0x48, 0x41, 0x5a, 0x53, 0x6c, 0x65, 0x7e, 0x77,
    0x19, 0x10, 0x0b, 0x02, 0x3d, 0x34, 0x2f, 0x26,
    0x51, 0x58, 0x43, 0x4a, 0x75, 0x7c, 0x67, 0x6e,
    0x32, 0x3b, 0x20, 0x29, 0x16, 0x1f, 0x04, 0x0d,
    0x7a, 0x73, 0x68, 0x61, 0x5e, 0x57, 0x4c, 0x45,
    0x2b, 0x22, 0x39, 0x30, 0x0f, 0x06, 0x1d, 0x14,
    0x63, 0x6a, 0x71, 0x78, 0x47, 0x4e, 0x55, 0x5c,
    0x64, 0x6d, 0x76, 0x7f, 0x40, 0x49, 0x52, 0x5b,
    0x2c, 0x25, 0x3e, 0x37, 0x08, 0x01, 0x1a, 0x13,
    0x7d, 0x74, 0x6f, 0x66, 0x59, 0x50, 0x4b, 0x42,
    0x35, 0x3c, 0x27, 0x2e, 0x11, 0x18, 0x03, 0x0a,
    0x56, 0x5f, 0x44, 0x4d, 0x72, 0x7b, 0x60, 0x69,
    0x1e, 0x17, 0x0c, 0x05, 0x3a, 0x33, 0x28, 0x21,
    0x4f, 0x46, 0x5d, 0x54, 0x6b, 0x62, 0x79, 0x70,
    0x07, 0x0e, 0x15, 0x1c, 0x23, 0x2a, 0x31, 0x38,
    0x41, 0x48, 0x53, 0x5a, 0x65, 0x6c, 0x77, 0x7e,
    0x09, 0x00, 0x1b, 0x12, 0x2d, 0x24, 0x3f, 0x36,
    0x58, 0x51, 0x4a, 0x43, 0x7c, 0x75, 0x6e, 0x67,
    0x10, 0x19, 0x02, 0x0b, 0x34, 0x3d, 0x26, 0x2f,
    0x73, 0x7a, 0x61, 0x68, 0x57, 0x5e, 0x45, 0x4c,
    0x3b, 0x32, 0x29, 0x20, 0x1f, 0x16, 0x0d, 0x04,
    0x6a, 0x63, 0x78, 0x71, 0x4e, 0x47, 0x5c, 0x55,
    0x22, 0x2b, 0x30, 0x39, 0x06, 0x0f, 0x14, 0x1d,
    0x25, 0x2c, 0x37, 0x3e, 0x01, 0x08, 0x13, 0x1a,
    0x6d, 0x64, 0x7f, 0x76, 0x49, 0x40, 0x5b, 0x52,
    0x3c, 0x35, 0x2e, 0x27, 0x18, 0x11, 0x0a, 0x03,
    0x74, 0x7d, 0x66, 0x6f, 0x50, 0x59, 0x42, 0x4b,
    0x17, 0x1e, 0x05, 0x0c, 0x33, 0x3a, 0x21, 0x28,
    0x5f, 0x56, 0x4d, 0x44, 0x7b, 0x72, 0x69, 0x60,
    0x0e, 0x07, 0x1c, 0x15, 0x2a, 0x23, 0x38, 0x31,
    0x46, 0x4f, 0x54, 0x5d, 0x62, 0x6b, 0x70, 0x79
};

static inline uint8_t stm32l0_sdspi_update_crc7(uint8_t crc7, uint8_t data)
{
    return stm32l0_sdspi_crc7_table[(crc7 << 1) ^ data];
}

static __attribute__((optimize("O3"))) uint8_t stm32l0_sdspi_compute_crc7(const uint8_t *data, uint32_t count)
{
    unsigned int n;
    uint8_t crc7 = 0;

    for (n = 0; n < count; n++)
    {
        crc7 = stm32l0_sdspi_update_crc7(crc7, data[n]);
    }
    
    return crc7;
}

#if 0

static const uint16_t stm32l0_sdspi_crc16_table[256]= {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067, 
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};


static inline uint16_t stm32l0_sdspi_update_crc16(uint16_t crc16, uint8_t data)
{
    return (uint16_t)(crc16 << 8) ^ stm32l0_sdspi_crc16_table[(crc16 >> 8) ^ data];
}


static __attribute__((optimize("O3"))) uint16_t stm32l0_sdspi_compute_crc16(const uint8_t *data, uint32_t count)
{
    unsigned int n;
    uint16_t crc16 = 0;

    for (n = 0; n < count; n++)
    {
        crc16 = stm32l0_sdspi_update_crc16(crc16, data[n]);
    }
    
    return crc16;
}

#endif

static bool stm32l0_sdspi_detect(stm32l0_sdspi_t *sdspi);
static void stm32l0_sdspi_select(stm32l0_sdspi_t *sdspi);
static void stm32l0_sdspi_unselect(stm32l0_sdspi_t *sdspi);
static void stm32l0_sdspi_mode(stm32l0_sdspi_t *sdspi, uint32_t mode);
static int stm32l0_sdspi_command(stm32l0_sdspi_t *sdspi, uint8_t index, uint32_t argument, uint32_t wait);
static int stm32l0_sdspi_wait_ready(stm32l0_sdspi_t *sdspi, uint32_t timeout);
static int stm32l0_sdspi_receive(stm32l0_sdspi_t *sdspi, uint8_t *data, uint32_t count, uint32_t *p_count);
static int stm32l0_sdspi_transmit(stm32l0_sdspi_t *sdspi, uint8_t start, const uint8_t *data, uint32_t count, bool *p_check);
static int stm32l0_sdspi_read_stop(stm32l0_sdspi_t *sdspi);
static int stm32l0_sdspi_write_stop(stm32l0_sdspi_t *sdspi);
static int stm32l0_sdspi_write_sync(stm32l0_sdspi_t *sdspi);

static int stm32l0_sdspi_idle(stm32l0_sdspi_t *sdspi, uint32_t *p_media);
static int stm32l0_sdspi_reset(stm32l0_sdspi_t *sdspi, uint32_t media);
static int stm32l0_sdspi_lock(stm32l0_sdspi_t *sdspi, int state, uint32_t address);
static int stm32l0_sdspi_unlock(stm32l0_sdspi_t *sdspi, int status);

#if 0

static uint16_t stm32l0_sdspi_receive_crc16(stm32l0_sdspi_t *sdspi, uint8_t *data, uint32_t count)
{
    stm32l0_spi_t *spi = sdspi->spi;
    uint16_t crc16;

    stm32l0_spi_receive(spi, data, count);

    crc16 = stm32l0_spi_data(spi, 0xff) << 8;
    crc16 |= stm32l0_spi_data(spi, 0xff);

    crc16 ^= stm32l0_sdspi_compute_crc16(data, count);

    return crc16;
}

#endif

#if 0

static void stm32l0_sdspi_transmit_crc16(stm32l0_sdspi_t *sdspi, const uint8_t *data, uint32_t count)
{
    stm32l0_spi_t *spi = sdspi->spi;
    uint16_t crc16;

    stm32l0_spi_transmit(spi, data, count);

    crc16 = stm32l0_sdspi_compute_crc16(data, count);

    stm32l0_spi_data(spi, crc16 >> 8);
    stm32l0_spi_data(spi, crc16 >> 0);
}

#endif

#if 1

static inline __attribute__((optimize("O3"))) uint16_t stm32l0_sdspi_receive_crc16(stm32l0_sdspi_t *sdspi, uint8_t *data, uint32_t count)
{
    stm32l0_spi_t *spi = sdspi->spi;
    SPI_TypeDef *SPI = spi->SPI;
    uint16_t crc16;
    uint8_t *data_e;
    uint16_t rx_data, tx_data;

    data_e = data + count - 2;

    tx_data = 0xffff;
    
    SPI->CR1 &= ~SPI_CR1_SPE;
    SPI->CR1 |= (SPI_CR1_DFF | SPI_CR1_CRCEN);
    SPI->CR1 |= SPI_CR1_SPE;
    
    SPI->DR = tx_data;
    
    do
    {
        __asm__ volatile("": : : "memory");

        while (!(SPI->SR & SPI_SR_RXNE))
        {
        }
    
        rx_data = SPI->DR;
        SPI->DR = tx_data;

        __asm__ volatile("": : : "memory");

        *data++ = rx_data >> 8;
        *data++ = rx_data;
    } 
    while (data != data_e);

    while (!(SPI->SR & SPI_SR_RXNE))
    {
    }
        
    rx_data = SPI->DR;

    *data++ = rx_data >> 8;
    *data++ = rx_data;

    while (!(SPI->SR & SPI_SR_TXE))
    {
    }

    while (SPI->SR & SPI_SR_BSY)
    {
    }

    SPI->CR1 &= ~SPI_CR1_SPE;
    SPI->CR1 &= ~SPI_CR1_CRCEN;
    SPI->CR1 |= SPI_CR1_SPE;

    crc16 = SPI->RXCRCR;
    
    SPI->DR = tx_data;

    while (!(SPI->SR & SPI_SR_RXNE))
    {
    }

    rx_data = SPI->DR;

    while (!(SPI->SR & SPI_SR_TXE))
    {
    }

    while (SPI->SR & SPI_SR_BSY)
    {
    }

    SPI->CR1 &= ~SPI_CR1_SPE;
    SPI->CR1 &= ~SPI_CR1_DFF;
    SPI->CR1 |= SPI_CR1_SPE;
    
    crc16 ^= rx_data;

    return crc16;
}

#endif

#if 1

static inline __attribute__((optimize("O3"))) void stm32l0_sdspi_transmit_crc16(stm32l0_sdspi_t *sdspi, const uint8_t *data, uint32_t count)
{
    stm32l0_spi_t *spi = sdspi->spi;
    SPI_TypeDef *SPI = spi->SPI;
    const uint8_t *data_e;
    uint16_t tx_data;

    data_e = data + count;

    SPI->CR1 &= ~SPI_CR1_SPE;
    SPI->CR1 |= (SPI_CR1_DFF | SPI_CR1_CRCEN);
    SPI->CR1 |= SPI_CR1_SPE;

    do
    {
        tx_data = *data++ << 8;
        tx_data |= *data++;

        __asm__ volatile("": : : "memory");
        
        while (!(SPI->SR & SPI_SR_TXE))
        {
        }

        SPI->DR = tx_data;

        __asm__ volatile("": : : "memory");
    } 
    while (data != data_e);

    while (!(SPI->SR & SPI_SR_TXE))
    {
    }

    while (SPI->SR & SPI_SR_BSY)
    {
    }

    SPI->CR1 &= ~SPI_CR1_SPE;
    SPI->CR1 &= ~SPI_CR1_CRCEN;
    SPI->CR1 |= SPI_CR1_SPE;

    tx_data = SPI->TXCRCR;

    SPI->DR = tx_data;

    while (!(SPI->SR & SPI_SR_TXE))
    {
    }

    while (SPI->SR & SPI_SR_BSY)
    {
    }

    SPI->CR1 &= ~SPI_CR1_SPE;
    SPI->CR1 &= ~SPI_CR1_DFF;
    SPI->CR1 |= SPI_CR1_SPE;

    /* Reset the overflow condition.
     */
    SPI->DR;
    SPI->SR;
}

#endif

static bool stm32l0_sdspi_detect(stm32l0_sdspi_t *sdspi)
{
    bool detect;

    /* This below is kind of fragile. The idea is to first set CS to 0, wait a tad
     * till the input settles to a 0. The switch the mode to input, while will make
     * the external pullup on CS take effect. If internal PULLDOWN does not work,
     * it overpowers the external pullupon CS. The delays are required to let
     * the signal on CS settle.
     */

    stm32l0_gpio_pin_write(sdspi->pin_cs, 0);
    armv6m_core_udelay(2);
    stm32l0_gpio_pin_input(sdspi->pin_cs);
    armv6m_core_udelay(2);
    detect = !!stm32l0_gpio_pin_read(sdspi->pin_cs);
    stm32l0_gpio_pin_output(sdspi->pin_cs);
    stm32l0_gpio_pin_write(sdspi->pin_cs, 1);

    return detect;
}

static void stm32l0_sdspi_select(stm32l0_sdspi_t *sdspi)
{
    stm32l0_spi_t *spi = sdspi->spi;

    SDSPI_STATISTICS_COUNT(sdcard_select);

    stm32l0_spi_acquire(spi, sdspi->clock, STM32L0_SPI_OPTION_MODE_0);

    /* CS output, drive CS to L */
    stm32l0_gpio_pin_write(sdspi->pin_cs, 0);

    /* The card will not drive DO for one more clock
     * after CS goes L, but will accept data right away.
     * The first thing after a select will be always
     * either a command (send_command), or a "Stop Token".
     * In both cases there will be a byte over the
     * bus, and hence DO will be stable.
     */
}

static void stm32l0_sdspi_unselect(stm32l0_sdspi_t *sdspi)
{
    stm32l0_spi_t *spi = sdspi->spi;

    SDSPI_STATISTICS_COUNT(sdcard_unselect);

    /* CS is output, drive CS to H */
    stm32l0_gpio_pin_write(sdspi->pin_cs, 1);

    /* The card drives the DO line at least one more
     * clock cycle after CS goes H. Hence send
     * one extra byte over the bus.
     */
    stm32l0_spi_data(spi, 0xff);

    stm32l0_spi_release(spi);
}

static void stm32l0_sdspi_mode(stm32l0_sdspi_t *sdspi, uint32_t mode)
{
    stm32l0_spi_t *spi = sdspi->spi;
    unsigned int n;

    stm32l0_spi_release(spi);

    if (mode == STM32L0_SDSPI_MODE_NONE)
    {
        sdspi->clock = 0;

        stm32l0_gpio_pin_configure(sdspi->pin_cs, (STM32L0_GPIO_PARK_PULLUP | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));
    }
    else
    {
        if (mode == STM32L0_SDSPI_MODE_IDENTIFY)
        {
            sdspi->clock = 400000;
        }
        else
        {
            sdspi->clock = 25000000;
        }

        stm32l0_gpio_pin_configure(sdspi->pin_cs, (STM32L0_GPIO_PARK_PULLUP | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
        stm32l0_gpio_pin_write(sdspi->pin_cs, 1);

        stm32l0_spi_acquire(spi, sdspi->clock, STM32L0_SPI_OPTION_MODE_0);

        if (mode == STM32L0_SDSPI_MODE_IDENTIFY)
        {
            stm32l0_gpio_pin_output(spi->pins.mosi);
            stm32l0_gpio_pin_write(spi->pins.mosi, 1);

            /* Here CS/MOSI are driven both to H.
             *
             * Specs says to issue 74 clock cycles in SPI mode while CS/MOSI are H,
             * so simply send 10 bytes over the clock line.
             */
    
            for (n = 0; n < 10; n++)
            {
                stm32l0_spi_data(sdspi->spi, 0xff);
            }
    
            stm32l0_gpio_pin_alternate(spi->pins.mosi);
        }

        stm32l0_gpio_pin_write(sdspi->pin_cs, 0);
    }
}

/*
 * stm32l0_sdspi_wait_ready(stm32l0_sdspi_t *sdspi, uint32_t timeout)
 *
 * Wait till DO transitions from BUSY (0x00) to READY (0xff).
 */

static int stm32l0_sdspi_wait_ready(stm32l0_sdspi_t *sdspi, uint32_t timeout)
{
    stm32l0_spi_t *spi = sdspi->spi;
    int status = F_NO_ERROR;
    uint32_t tstart, tend;
    uint8_t token;

    /* While waiting for non busy (not 0x00) the host can
     * release the CS line to let somebody else access the
     * bus. However there needs to be an extra clock
     * cycle after driving CS to H for the card to release DO,
     * as well as one extra clock cycle after driving CS to L
     * before the data is valid again.
     */

    tstart = armv6m_systick_millis();

    do
    {
        tend = armv6m_systick_millis();

        token = stm32l0_spi_data(spi, 0xff);

        if (token == SD_READY_TOKEN)
        {
            break;
        }

        if ((tend - tstart) > timeout)
        {
            status = F_ERR_ONDRIVE;
        }
    }
    while (status == F_NO_ERROR);

    return status;
}

static int stm32l0_sdspi_command(stm32l0_sdspi_t *sdspi, uint8_t index, uint32_t argument, uint32_t wait)
{
    stm32l0_spi_t *spi = sdspi->spi;
    int status = F_NO_ERROR;
    unsigned int n;
    uint8_t data[5], response, crc7;
#if (DOSFS_CONFIG_SDCARD_CRC == 1) && (DOSFS_CONFIG_SDCARD_COMMAND_RETRIES != 0)
    unsigned int retries = DOSFS_CONFIG_SDCARD_COMMAND_RETRIES +1;
#else /* (DOSFS_CONFIG_SDCARD_CRC == 1) && (DOSFS_CONFIG_SDCARD_COMMAND_RETRIES != 0) */
    unsigned int retries = 0;
#endif /* (DOSFS_CONFIG_SDCARD_CRC == 1) && (DOSFS_CONFIG_SDCARD_COMMAND_RETRIES != 0) */

    SDSPI_STATISTICS_COUNT(sdcard_command);

    data[0] = 0x40 | index;
    data[1] = argument >> 24;
    data[2] = argument >> 16;
    data[3] = argument >> 8;
    data[4] = argument >> 0;

#if (DOSFS_CONFIG_SDCARD_CRC == 1)
    crc7 = (stm32l0_sdspi_compute_crc7(data, 5) << 1) | 0x01;
#else /* (DOSFS_CONFIG_SDCARD_CRC == 1) */
    if (index == SD_CMD_GO_IDLE_STATE)
    {
        crc7 = 0x95;
    }
    else if (index == SD_CMD_SEND_IF_COND)
    {
        crc7 = 0x87;
    }
    else
    {
        crc7 = 0x01;
    }
#endif /* (DOSFS_CONFIG_SDCARD_CRC == 1) */

    do
    {
        /* A command needs at least one back to back idle cycle.
         */
        
        stm32l0_spi_data(spi, 0xff);

        stm32l0_spi_data(spi, data[0]);
        stm32l0_spi_data(spi, data[1]);
        stm32l0_spi_data(spi, data[2]);
        stm32l0_spi_data(spi, data[3]);
        stm32l0_spi_data(spi, data[4]);
        stm32l0_spi_data(spi, crc7);

        /* NCR is 1..8 bytes, so simply always discard the first byte,
         * and then read up to 8 bytes or till a vaild response
         * was seen. N.b that STOP_TRANSMISSION specifies that
         * the first byte on DataOut after reception of the command
         * is a stuffing byte that has to be ignored. The discard
         * takes care of that here.
         */

        stm32l0_spi_data(spi, 0xff);

        for (n = 0; n < 8; n++)
        {
            response = stm32l0_spi_data(spi, 0xff);
        
            if (!(response & 0x80))
            {
                /*
                 * A STOP_TRANSMISSION can be issued before the card
                 * had send a "Data Error Token" for that last block
                 * that we are not really intrested in. This could result
                 * in a "Parameter Error" if the next block is out of bounds.
                 * Due to the way CMD_STOP_TRANSMISSION gets send there is
                 * also the chance it gets an "Illegal Command" error ...
                 */
                if (index == SD_CMD_STOP_TRANSMISSION)
                {
                    response &= ~(SD_R1_ILLEGAL_COMMAND | SD_R1_PARAMETER_ERROR);
                }
                break;
            }
        }

        sdspi->response[0] = response;

        /* Always read the rest of the data to avoid synchronization
         * issues. Worst case the data read is random.
         */
        for (n = 1; n <= wait; n++)
        {
            sdspi->response[n] = stm32l0_spi_data(spi, 0xff);
        } 

        if (response & 0x88)
        {
            if (response & 0x80)
            {
                SDSPI_STATISTICS_COUNT(sdcard_command_timeout);

#if (DOSFS_CONFIG_STATISTICS == 1)
                sdspi->statistics.sdcard_command_timeout_2[index]++;
#endif
            }
            else
            {
                SDSPI_STATISTICS_COUNT(sdcard_command_crcfail);

#if (DOSFS_CONFIG_STATISTICS == 1)
                sdspi->statistics.sdcard_command_crcfail_2[index]++;
#endif
            }

#if (DOSFS_CONFIG_SDCARD_CRC == 1) && (DOSFS_CONFIG_SDCARD_COMMAND_RETRIES != 0)
            if (retries > 1)
            {
                stm32l0_gpio_pin_write(sdspi->pin_cs, 1);
                stm32l0_gpio_pin_write(sdspi->pin_cs, 0);

                SDSPI_STATISTICS_COUNT(sdcard_command_retry);
                    
                retries--;
            }
            else
#endif /* (DOSFS_CONFIG_SDCARD_CRC == 1) && (DOSFS_CONFIG_SDCARD_COMMAND_RETRIES != 0) */
            {
                status = F_ERR_ONDRIVE;

                retries = 0;
            }
        }
        else
        {
            if ((index != SD_CMD_STOP_TRANSMISSION) && (index != SD_CMD_SEND_STATUS))
            {
                if (response & SD_R1_ERROR_MASK)
                {
                    status = F_ERR_ONDRIVE;
                }
            }

            retries = 0;
        }
    }
    while (retries != 0);

    if (status != F_NO_ERROR)
    {
        SDSPI_STATISTICS_COUNT(sdcard_command_fail);

#if (DOSFS_CONFIG_STATISTICS == 1)
        sdspi->statistics.sdcard_command_fail_2[index]++;
#endif
    }

    return status;
}

static int stm32l0_sdspi_receive(stm32l0_sdspi_t *sdspi, uint8_t *data, uint32_t count, uint32_t *p_count)
{
    stm32l0_spi_t *spi = sdspi->spi;
    int status = F_NO_ERROR;
    uint8_t token;
    uint16_t crc16;
    uint32_t total, blksz, tstart, tend;

    SDSPI_STATISTICS_COUNT(sdcard_receive);

    total = count;
    blksz = (count >= 512) ? 512 : count;

    tstart = armv6m_systick_millis();

    do
    {
        /* If a "Start Block Token" (0xfe) zips by then
         * a data block will follow. Otherwise a "Data Error Token"
         * shows up:
         *
         * 0x01 Execution Error
         * 0x02 CC Error
         * 0x04 Card ECC failed
         * 0x08 Out of Range
         *
         * The maximum documented timeout is 100ms for SDHC.
         */

        tend = armv6m_systick_millis();

        token = stm32l0_spi_data(spi, 0xff);

        if (token == SD_START_READ_TOKEN)
        {
            crc16 = stm32l0_sdspi_receive_crc16(sdspi, data, blksz);
            
#if (DOSFS_CONFIG_SDCARD_CRC == 1)
            if (crc16 != 0)
            {
                SDSPI_STATISTICS_COUNT(sdcard_receive_crcfail);

                /* On a CRC error always send a STOP_TRANSMISSION, just to make sure.
                 */
                status = stm32l0_sdspi_command(sdspi, SD_CMD_STOP_TRANSMISSION, 0, 0);
                
                if (status == F_NO_ERROR)
                {
                    sdspi->state = STM32L0_SDSPI_STATE_READY;
                }

                break;
            }
            else
#endif /* (DOSFS_CONFIG_SDCARD_CRC == 1)  */
            {
                data  += blksz;
                total -= blksz;

                tstart = armv6m_systick_millis();
            }
        }

        else if ((token & SD_DATA_ERROR_TOKEN_VALID_MASK) == SD_DATA_ERROR_TOKEN_VALID_DATA)
        {
            sdspi->state = STM32L0_SDSPI_STATE_READY;

            if (token & SD_DATA_ERROR_CARD_ECC_FAILED)
            {
                status = F_ERR_INVALIDSECTOR;
            }
            else
            {
                status = F_ERR_ONDRIVE;
            }
        }

        else
        {
            if ((tend - tstart) > SD_READ_TIMEOUT)
            {
                SDSPI_STATISTICS_COUNT(sdcard_receive_timeout);
                
                status = F_ERR_ONDRIVE;
            }
        }
    }
    while ((status == F_NO_ERROR) && total);
        
    *p_count = (count - total);

    if (status != F_NO_ERROR)
    {
        SDSPI_STATISTICS_COUNT(sdcard_receive_fail);
    }

    return status;
}

static int stm32l0_sdspi_transmit(stm32l0_sdspi_t *sdspi, uint8_t start, const uint8_t *data, uint32_t count, bool *p_check)
{
    stm32l0_spi_t *spi = sdspi->spi;
    int status = F_NO_ERROR;
    uint8_t token, response;
    uint32_t total, blksz, tstart, tend;

    SDSPI_STATISTICS_COUNT(sdcard_transmit);

    *p_check = false;

    total = count;
    blksz = (count >= 512) ? 512 : count;

    tstart = armv6m_systick_millis();

    do
    {
        tend = armv6m_systick_millis();

        token = stm32l0_spi_data(spi, 0xff);

        if (token == SD_READY_TOKEN)
        {
            stm32l0_spi_data(spi, start);
            stm32l0_sdspi_transmit_crc16(sdspi, data, blksz);

            /* At last read back the "Data Response Token":
             *
             * 0x05 No Error
             * 0x0b CRC Error
             * 0x0d Write Error
             */
            
            response = stm32l0_spi_data(spi, 0xff) & SD_DATA_RESPONSE_MASK;

            if (response != SD_DATA_RESPONSE_ACCEPTED)
            {
                *p_check = true;

                status = stm32l0_sdspi_command(sdspi, SD_CMD_STOP_TRANSMISSION, 0, 1);

                if (status != F_NO_ERROR)
                {
                    status = stm32l0_sdspi_command(sdspi, SD_CMD_STOP_TRANSMISSION, 0, 1);
                }

                if (status == F_NO_ERROR)
                {
                    status = stm32l0_sdspi_wait_ready(sdspi, SD_WRITE_STOP_TIMEOUT);
                    
                    if (status == F_NO_ERROR)
                    {
                        sdspi->state = STM32L0_SDSPI_STATE_READY;

                        if (response == SD_DATA_RESPONSE_CRC_ERROR)
                        {
                            SDSPI_STATISTICS_COUNT(sdcard_transmit_crcfail);
                        }
                        else
                        {
                            status = stm32l0_sdspi_command(sdspi, SD_CMD_SEND_STATUS, 0, 1);
                            
                            if (status == F_NO_ERROR)
                            {
                                if ((sdspi->response[0] & SD_R1_ERROR_MASK) || (sdspi->response[1] & SD_R2_ERROR_MASK))
                                {
                                    if (sdspi->response[1] & (SD_R2_WP_VIOLATION | SD_R2_WP_ERASE_SKIP))
                                    {
                                        status = F_ERR_WRITEPROTECT;
                                    }
                                    else if (sdspi->response[1] & SD_R2_CARD_ECC_FAILED)
                                    {
                                        status = F_ERR_INVALIDSECTOR;
                                    }
                                    else
                                    {
                                        status = F_ERR_ONDRIVE;
                                    }
                                }
                            }
                        }
                    }
                }

                break;
            }
            else
            {
                data  += blksz;
                total -= blksz;

                tstart = armv6m_systick_millis();
            }
        }
        else
        {
            if ((tend - tstart) > SD_WRITE_TIMEOUT)
            {
                SDSPI_STATISTICS_COUNT(sdcard_transmit_timeout);

                status = F_ERR_ONDRIVE;
            }
        }
    }
    while ((status == F_NO_ERROR) && total);

    if (status != F_NO_ERROR)
    {
        SDSPI_STATISTICS_COUNT(sdcard_transmit_fail);
    }

    return status;
}

static int stm32l0_sdspi_read_stop(stm32l0_sdspi_t *sdspi)
{
    int status = F_NO_ERROR;

    SDSPI_STATISTICS_COUNT(sdcard_read_stop);

    status = stm32l0_sdspi_command(sdspi, SD_CMD_STOP_TRANSMISSION, 0, 0);

    if (status == F_NO_ERROR)
    {
        sdspi->state = STM32L0_SDSPI_STATE_READY;

        if (sdspi->response[0] & SD_R1_ERROR_MASK)
        {
            status = stm32l0_sdspi_command(sdspi, SD_CMD_SEND_STATUS, 0, 1);
                                
            if (status == F_NO_ERROR)
            {
                if (sdspi->response[1] & (SD_R2_WP_VIOLATION | SD_R2_WP_ERASE_SKIP))
                {
                    status = F_ERR_WRITEPROTECT;
                }
                else if (sdspi->response[1] & SD_R2_CARD_ECC_FAILED)
                {
                    status = F_ERR_INVALIDSECTOR;
                }
                else
                {
                    status = F_ERR_ONDRIVE;
                }
            }
        }
    }
                                
    return status;
}

static int stm32l0_sdspi_write_stop(stm32l0_sdspi_t *sdspi)
{
    stm32l0_spi_t *spi = sdspi->spi;
    int status = F_NO_ERROR;

    SDSPI_STATISTICS_COUNT(sdcard_write_stop);

    /* There need to be 8 clocks before the "Stop Transfer Token",
     * and 8 clocks after it, before the card signals busy properly.
     * The 8 clocks before are covered by stm32l0_sdspi_wait_ready().
     */

    status = stm32l0_sdspi_wait_ready(sdspi, SD_WRITE_TIMEOUT);
    
    if (status == F_NO_ERROR)
    {
        stm32l0_spi_data(spi, SD_STOP_TRANSMISSION_TOKEN);
        stm32l0_spi_data(spi, 0xff);

        sdspi->state = STM32L0_SDSPI_STATE_WRITE_STOP;
    }

    return status;
}

static int stm32l0_sdspi_write_sync(stm32l0_sdspi_t *sdspi)
{
    int status = F_NO_ERROR;

    SDSPI_STATISTICS_COUNT(sdcard_write_sync);

    status = stm32l0_sdspi_wait_ready(sdspi, SD_WRITE_STOP_TIMEOUT);

    if (status == F_NO_ERROR)
    {
        sdspi->state = STM32L0_SDSPI_STATE_READY;

        status = stm32l0_sdspi_command(sdspi, SD_CMD_SEND_STATUS, 0, 1);
        
        if (status == F_NO_ERROR)
        {
            if ((sdspi->response[0] & SD_R1_ERROR_MASK) || (sdspi->response[1] & SD_R2_ERROR_MASK))
            {
                if (sdspi->response[1] & (SD_R2_WP_VIOLATION | SD_R2_WP_ERASE_SKIP | SD_R2_CARD_ECC_FAILED))
                {
                    if (sdspi->response[1] & (SD_R2_WP_VIOLATION | SD_R2_WP_ERASE_SKIP))
                    {
                        status = F_ERR_WRITEPROTECT;
                    }
                    else
                    {
                        status = F_ERR_INVALIDSECTOR;
                    }
                }
                else
                {
                    status = F_ERR_ONDRIVE;
                }
            }
        }
    }

    if (status != F_NO_ERROR)
    {
        SDSPI_STATISTICS_COUNT(sdcard_write_sync_fail);
    }

    return status;
}

static int stm32l0_sdspi_idle(stm32l0_sdspi_t *sdspi, uint32_t *p_media)
{
    stm32l0_spi_t *spi = sdspi->spi;
    int status = F_NO_ERROR;
    uint32_t media;
    unsigned int n;

    SDSPI_STATISTICS_COUNT(sdcard_idle);

    media = DOSFS_MEDIA_NONE;
    
    stm32l0_sdspi_mode(sdspi, STM32L0_SDSPI_MODE_IDENTIFY);

    /* Apply an initial CMD_GO_IDLE_STATE, so that the card is out of
     * data read/write mode, and can properly respond.
     */

    stm32l0_sdspi_command(sdspi, SD_CMD_GO_IDLE_STATE, 0, 0);

    if (sdspi->response[0] != 0x01)
    {
        /* There could be 2 error scenarios. One is that the
         * CMD_GO_IDLE_STATE was send while the card was waiting
         * for the next "Start Block" / "Stop Transmission" token.
         * The card will answer that normally by signaling BUSY,
         * when means there would be a 0x00 response.
         * The other case is that there was a reset request while
         * being in the middle of a write transaction. As a result
         * the card will send back a stream of non-zero values
         * (READY or a "Data Response Token"), which can be handled
         * by simple flushing the transaction (CRC will take care of
         * rejecting the data ... without CRC the last write will be
         * garbage.
         *
         * Hence first drain a possible write command, and then
         * rety CMD_GO_IDLE_STATE after a dosfs_sdcard_wait_ready().
         * If the card is still in BUSY mode, then retry the
         * CMD_GO_IDLE_STATE again.
         */

        if (sdspi->response[0] != 0x00)
        {
            for (n = 0; n < 1024; n++)
            {
                stm32l0_spi_data(spi, 0xff);
            }

            stm32l0_sdspi_command(sdspi, SD_CMD_GO_IDLE_STATE, 0, 0);
        }

        stm32l0_sdspi_wait_ready(sdspi, SD_WRITE_STOP_TIMEOUT);
            
        stm32l0_sdspi_command(sdspi, SD_CMD_GO_IDLE_STATE, 0, 0);

        if (sdspi->response[0] == 0x00)
        {
            stm32l0_sdspi_wait_ready(sdspi, SD_WRITE_STOP_TIMEOUT);
            
            stm32l0_sdspi_command(sdspi, SD_CMD_GO_IDLE_STATE, 0, 0);
        }
    }

    if (sdspi->response[0] == 0x01)
    {
        stm32l0_sdspi_command(sdspi, SD_CMD_SEND_IF_COND, 0x000001aa, 4);

        if (sdspi->response[0] == 0x01)
        {
            if ((sdspi->response[1] == 0x00) &&
                (sdspi->response[2] == 0x00) &&
                (sdspi->response[3] == 0x01) &&
                (sdspi->response[4] == 0xaa))
            {
                media = DOSFS_MEDIA_SDHC;
            }
            else
            {
                status = F_ERR_INVALIDMEDIA;
            }
        }
        else
        {
            media = DOSFS_MEDIA_SDSC;
        }
    }

    if (status == F_NO_ERROR)
    {
        status = stm32l0_sdspi_command(sdspi, SD_CMD_READ_OCR, 0x00000000, 4);

        if (status == F_NO_ERROR)
        {
            sdspi->OCR = ((sdspi->response[1] << 24) |
                          (sdspi->response[2] << 16) |
                          (sdspi->response[3] <<  8) |
                          (sdspi->response[4] <<  0));
            
            if (!(sdspi->OCR & 0x00300000))
            {
                status = F_ERR_INVALIDMEDIA;
            }
        }
    }

    if ((p_media == NULL) || (status != F_NO_ERROR))
    {
        stm32l0_sdspi_mode(sdspi, STM32L0_SDSPI_MODE_NONE);
    }
    
    if (p_media)
    {
        *p_media = media;
    }

    return status;
}

static uint32_t stm32l0_sdspi_au_size_table[16] = {
    0,
    32,      /* 16KB  */
    64,      /* 32KB  */
    128,     /* 64KB  */
    256,     /* 128KB */
    512,     /* 256KB */
    1024,    /* 512KB */
    2048,    /* 1MB   */
    4096,    /* 2MB   */
    8192,    /* 4MB   */
    16384,   /* 8MB   */
    24576,   /* 12MB  */
    32768,   /* 16MB  */
    49152,   /* 24MB  */
    65536,   /* 32MB  */
    131072,  /* 64MB  */
};

static int stm32l0_sdspi_reset(stm32l0_sdspi_t *sdspi, uint32_t media)
{
    int status = F_NO_ERROR;
    uint32_t millis, count;

    SDSPI_STATISTICS_COUNT(sdcard_reset);

    /* Send ACMD_SD_SEND_OP_COND, and wait till the initialization process is done. The SDCARD has
     * 1000ms to complete this initialization process.
     */

    millis = armv6m_systick_millis();
    
    do
    {
        status = stm32l0_sdspi_command(sdspi, SD_CMD_APP_CMD, 0, 0);
            
        if (status == F_NO_ERROR)
        {
            status = stm32l0_sdspi_command(sdspi, SD_ACMD_SD_SEND_OP_COND, ((media == DOSFS_MEDIA_SDHC) ? 0x40300000 : 0x00300000), 0);
                
            if (status == F_NO_ERROR)
            {
                if (!(sdspi->response[0] & SD_R1_IN_IDLE_STATE))
                {
                    break;
                }
                    
                if (((uint32_t)armv6m_systick_millis() - millis) > 1000)
                {
                    status = F_ERR_ONDRIVE;
                }
            }
        }
    }
    while (status == F_NO_ERROR);

    if (status == F_NO_ERROR)
    {
        if (media == DOSFS_MEDIA_SDHC)
        {
            status = stm32l0_sdspi_command(sdspi, SD_CMD_READ_OCR, 0x00000000, 4);
            
            if (status == F_NO_ERROR)
            {
                sdspi->OCR = ((sdspi->response[1] << 24) |
                              (sdspi->response[2] << 16) |
                              (sdspi->response[3] <<  8) |
                              (sdspi->response[4] <<  0));
                
                if (sdspi->OCR & 0x40000000)
                {
                    media = DOSFS_MEDIA_SDHC;
                }
                else
                {
                    media = DOSFS_MEDIA_SDSC;
                }
            }
        }
    }

    /* If we got here without a hitch then it's time to enter transfer mode
     * and setup buswidth and speed.
     */

    if (status == F_NO_ERROR)
    {
        stm32l0_sdspi_mode(sdspi, STM32L0_SDSPI_MODE_DATA_TRANSFER);

#if (DOSFS_CONFIG_SDCARD_CRC == 1)
        if (status == F_NO_ERROR)
        {
            status  = stm32l0_sdspi_command(sdspi, SD_CMD_CRC_ON_OFF, 1, 0);
        }
#endif /* (DOSFS_CONFIG_SDCARD_CRC == 1) */

        status = stm32l0_sdspi_command(sdspi, SD_CMD_SEND_CID, 0, 0);
        
        if (status == F_NO_ERROR)
        {
            status = stm32l0_sdspi_receive(sdspi, &sdspi->CID[0], 16, &count);
            
            if ((status == F_NO_ERROR) && (count != 16))
            {
                status = F_ERR_READ;
            }
        }
        
        if (status == F_NO_ERROR)
        {
            status = stm32l0_sdspi_command(sdspi, SD_CMD_SEND_CSD, 0, 0);
            
            if (status == F_NO_ERROR)
            {
                status = stm32l0_sdspi_receive(sdspi, &sdspi->CSD[0], 16, &count);
                
                if ((status == F_NO_ERROR) && (count != 16))
                {
                    status = F_ERR_READ;
                }
            }
        }

        if (status == F_NO_ERROR)
        {
            status = stm32l0_sdspi_command(sdspi, SD_CMD_APP_CMD, 0, 0);
                
            if (status == F_NO_ERROR)
            {
                status = stm32l0_sdspi_command(sdspi, SD_ACMD_SEND_SCR, 0, 0);

                if (status == F_NO_ERROR)
                {
                    status = stm32l0_sdspi_receive(sdspi, &sdspi->SCR[0], 8, &count);
                    
                    if ((status == F_NO_ERROR) && (count != 8))
                    {
                        status = F_ERR_READ;
                    }
                }
            }
        }
        
        if (status == F_NO_ERROR)
        {
            status = stm32l0_sdspi_command(sdspi, SD_CMD_APP_CMD, 0, 0);
                
            if (status == F_NO_ERROR)
            {
                status = stm32l0_sdspi_command(sdspi, SD_ACMD_SD_STATUS, 0, 1);

                if (status == F_NO_ERROR)
                {
                    status = stm32l0_sdspi_receive(sdspi, &sdspi->SSR[0], 64, &count);
                    
                    if ((status == F_NO_ERROR) && (count != 64))
                    {
                        status = F_ERR_READ;
                    }
                }
            }
        }
        
        if (status == F_NO_ERROR)
        {
            status = stm32l0_sdspi_command(sdspi, SD_CMD_SET_BLOCKLEN, 512, 0);
        }
    }

    if (status == F_NO_ERROR)
    {
        sdspi->state = STM32L0_SDSPI_STATE_READY;

        sdspi->media = media;
        sdspi->shift = 9;
        sdspi->au_size = 0;
        sdspi->erase_size = 0;
        sdspi->erase_timeout = 0;
        sdspi->erase_offset = 0;

        if (media == DOSFS_MEDIA_SDHC)
        {
            sdspi->shift = 0;
            sdspi->au_size = stm32l0_sdspi_au_size_table[stm32l0_sdspi_slice(sdspi->SSR, 512, 428, 4)];
            sdspi->erase_size = stm32l0_sdspi_slice(sdspi->SSR, 512, 408, 16);
            sdspi->erase_timeout = stm32l0_sdspi_slice(sdspi->SSR, 512, 402, 6) * 1000;
            sdspi->erase_offset = stm32l0_sdspi_slice(sdspi->SSR, 512, 400, 2) * 1000;

            if (!sdspi->au_size || !sdspi->erase_size || !sdspi->erase_timeout)
            {
                sdspi->erase_size = 0;
                sdspi->erase_timeout = 0;
                sdspi->erase_offset = 0;
            }
        }

        SDSPI_STATISTICS_COUNT(sdcard_select);
    }
    else
    {
        sdspi->state = STM32L0_SDSPI_STATE_RESET;
        
        stm32l0_sdspi_mode(sdspi, STM32L0_SDSPI_MODE_NONE);

        status = F_ERR_CARDREMOVED;
    }
    
    return status;
}

static int stm32l0_sdspi_lock(stm32l0_sdspi_t *sdspi, int state, uint32_t address)
{
    int status = F_NO_ERROR;
    uint32_t media;

#if defined(DOSFS_PORT_SDCARD_LOCK)
    status = DOSFS_PORT_SDCARD_LOCK();
    
    if (status == F_NO_ERROR)
#endif /* DOSFS_PORT_SDCARD_LOCK */
    {
        stm32l0_system_lock(STM32L0_SYSTEM_LOCK_STOP);
        stm32l0_system_lock(STM32L0_SYSTEM_LOCK_CLOCKS);

        if (sdspi->state == STM32L0_SDSPI_STATE_INIT)
        {
            if (stm32l0_sdspi_detect(sdspi))
            {
                sdspi->state = STM32L0_SDSPI_STATE_RESET;
            }
            else
            {
                status = F_ERR_CARDREMOVED;
            }
        }
        
        if (status == F_NO_ERROR)
        {
            if (sdspi->state == STM32L0_SDSPI_STATE_RESET)
            {
                status = stm32l0_sdspi_idle(sdspi, &media);

                if (status == F_NO_ERROR)
                {
                    status = stm32l0_sdspi_reset(sdspi, media);
                }
                else
                {
                    status = F_ERR_CARDREMOVED;
                }
            }
            else
            {
                stm32l0_sdspi_select(sdspi);

                if (sdspi->state == STM32L0_SDSPI_STATE_READ_MULTIPLE)
                {
                    if ((state != STM32L0_SDSPI_STATE_READ_MULTIPLE) || (sdspi->address != address))
                    {
                        status = stm32l0_sdspi_read_stop(sdspi);
                    }
                }

                if (sdspi->state == STM32L0_SDSPI_STATE_WRITE_MULTIPLE)
                {
                    if ((state != STM32L0_SDSPI_STATE_WRITE_MULTIPLE) || (sdspi->address != address))
                    {
                        status = stm32l0_sdspi_write_stop(sdspi);
                    }
                }

                if (sdspi->state == STM32L0_SDSPI_STATE_WRITE_STOP)
                {
                    if (state != STM32L0_SDSPI_STATE_WRITE_STOP)
                    {
                        status = stm32l0_sdspi_write_sync(sdspi);
                    }
                }
            }
        }
    }
    
    if (status != F_NO_ERROR)
    {
        status = stm32l0_sdspi_unlock(sdspi, status);
    }

    return status;
}

static int stm32l0_sdspi_unlock(stm32l0_sdspi_t *sdspi, int status)
{
    if ((sdspi->state != STM32L0_SDSPI_STATE_INIT) && (sdspi->state != STM32L0_SDSPI_STATE_RESET))
    {
        stm32l0_sdspi_unselect(sdspi);
    }

    if (status == F_ERR_ONDRIVE)
    {
        if (stm32l0_sdspi_idle(sdspi, NULL) == F_NO_ERROR)
        {
            sdspi->state = STM32L0_SDSPI_STATE_INIT;
        }
        else
        {
            sdspi->state = STM32L0_SDSPI_STATE_RESET;

            status = F_ERR_CARDREMOVED;
        }
    }

    stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_CLOCKS);
    stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_STOP);

#if defined(DOSFS_PORT_SDCARD_UNLOCK)
    DOSFS_PORT_SDCARD_UNLOCK();
#endif /* DOSFS_PORT_SDCARD_UNLOCK */

    return status;
}

static int stm32l0_sdspi_release(void *context)
{
    stm32l0_sdspi_t *sdspi = (stm32l0_sdspi_t*)context;
    int status = F_NO_ERROR;

    if ((sdspi->state != STM32L0_SDSPI_STATE_INIT) && (sdspi->state != STM32L0_SDSPI_STATE_RESET))
    {
        status = stm32l0_sdspi_lock(sdspi, STM32L0_SDSPI_STATE_READY, 0);
        
        if (status == F_NO_ERROR)
        {
            status = stm32l0_sdspi_unlock(sdspi, status);

            stm32l0_sdspi_mode(sdspi, STM32L0_SDSPI_MODE_NONE);

            stm32l0_spi_notify(sdspi->spi, NULL, NULL);

            sdspi->state = STM32L0_SDSPI_STATE_INIT;
        }
    }

    return status;
}

static int stm32l0_sdspi_info(void *context, uint8_t *p_media, uint8_t *p_write_protected, uint32_t *p_block_count, uint32_t *p_au_size, uint32_t *p_serial)
{
    stm32l0_sdspi_t *sdspi = (stm32l0_sdspi_t*)context;
    int status = F_NO_ERROR;
    uint32_t c_size, c_size_mult, read_bl_len;

    status = stm32l0_sdspi_lock(sdspi, STM32L0_SDSPI_STATE_READY, 0);

    if (status == F_NO_ERROR)
    {
        *p_media = sdspi->media;
        *p_write_protected = false;

        /* PERM_WRITE_PROTECT.
         * TMP_WRITE_PROTECT
         */
        if (stm32l0_sdspi_slice(sdspi->CSD, 128, 12, 2))
        {
            *p_write_protected = true;
        }
        
        if ((sdspi->CSD[0] & 0xc0) == 0)
        {
            /* SDSC */
            
            read_bl_len = stm32l0_sdspi_slice(sdspi->CSD, 128, 80, 4);
            c_size = stm32l0_sdspi_slice(sdspi->CSD, 128, 62, 12);
            c_size_mult = stm32l0_sdspi_slice(sdspi->CSD, 128, 47, 3);
            
            *p_block_count = ((c_size + 1) << (c_size_mult + 2)) << (read_bl_len - 9);
        }
        else
        {
            /* SDHC */

            c_size = stm32l0_sdspi_slice(sdspi->CSD, 128, 48, 22);

            *p_block_count = (c_size + 1) << (19 - 9);
        }

        *p_au_size = sdspi->au_size;

        *p_serial = stm32l0_sdspi_slice(sdspi->CID, 128, 24, 32);

        status = stm32l0_sdspi_unlock(sdspi, status);
    }

    return status;
}

static int stm32l0_sdspi_notify(void *context, dosfs_device_notify_callback_t callback, void *cookie)
{
    stm32l0_sdspi_t *sdspi = (stm32l0_sdspi_t*)context;
    int status = F_NO_ERROR;

    stm32l0_spi_notify(sdspi->spi, callback, cookie);

    return status;
}

static int stm32l0_sdspi_format(void *context)
{
    return F_NO_ERROR;
}

static int stm32l0_sdspi_erase(void *context, uint32_t address, uint32_t length)
{
    stm32l0_sdspi_t *sdspi = (stm32l0_sdspi_t*)context;
    int status = F_NO_ERROR;
    uint32_t au_start, au_end, timeout;

    if (sdspi->erase_size)
    {
        au_start = address / sdspi->au_size;
        au_end   = ((address + length + (sdspi->au_size -1)) / sdspi->au_size);

        timeout = ((sdspi->erase_timeout * (au_end - au_start)) / sdspi->erase_size) + sdspi->erase_offset;

        if (timeout < 1000)
        {
            timeout = 1000;
        }

        if (address != (au_start * sdspi->au_size))
        {
            timeout += 250;
        }

        if ((address + length) != (au_end * sdspi->au_size))
        {
            timeout += 250;
        }
        
        status = stm32l0_sdspi_lock(sdspi, STM32L0_SDSPI_STATE_READY, 0);

        if (status == F_NO_ERROR)
        {
            status = stm32l0_sdspi_command(sdspi, SD_CMD_ERASE_WR_BLK_START_ADDR, (address << sdspi->shift), 0);
        
            if (status == F_NO_ERROR)
            {
                status = stm32l0_sdspi_command(sdspi, SD_CMD_ERASE_WR_BLK_END_ADDR, ((address + length -1) << sdspi->shift), 0);
        
                if (status == F_NO_ERROR)
                {
                    status = stm32l0_sdspi_command(sdspi, SD_CMD_ERASE, 0, 0);

                    if (status == F_NO_ERROR)
                    {
                        status = stm32l0_sdspi_wait_ready(sdspi, timeout);

                        if (status == F_NO_ERROR)
                        {
                            status = stm32l0_sdspi_command(sdspi, SD_CMD_SEND_STATUS, 0, 1);
                        
                            if (status == F_NO_ERROR)
                            {
                                if ((sdspi->response[0] & SD_R1_ERROR_MASK) || (sdspi->response[1] & SD_R2_ERROR_MASK))
                                {
                                    if (sdspi->response[1] & (SD_R2_WP_VIOLATION | SD_R2_WP_ERASE_SKIP))
                                    {
                                        status = F_ERR_WRITEPROTECT;
                                    }
                                    else if (sdspi->response[1] & SD_R2_CARD_ECC_FAILED)
                                    {
                                        status = F_ERR_INVALIDSECTOR;
                                    }
                                    else
                                    {
                                        status = F_ERR_ONDRIVE;
                                    }
                                }
                                else
                                {
                                    SDSPI_STATISTICS_COUNT_N(sdcard_erase, length);
                                }
                            }
                        }
                        else
                        {
                            SDSPI_STATISTICS_COUNT(sdcard_erase_timeout);
                        }
                    }
                }
            }

            status = stm32l0_sdspi_unlock(sdspi, status);
        }
    }

    return status;
}

static int stm32l0_sdspi_discard(void *context, uint32_t address, uint32_t length)
{
    return F_NO_ERROR;
}

static int stm32l0_sdspi_read(void *context, uint32_t address, uint8_t *data, uint32_t length, bool prefetch)
{
    stm32l0_sdspi_t *sdspi = (stm32l0_sdspi_t*)context;
    int status = F_NO_ERROR;
    uint32_t count;
#if (DOSFS_CONFIG_SDCARD_CRC == 1) && (DOSFS_CONFIG_SDCARD_DATA_RETRIES != 0)
    unsigned int retries = DOSFS_CONFIG_SDCARD_DATA_RETRIES;
#endif /* (DOSFS_CONFIG_SDCARD_CRC == 1) && (DOSFS_CONFIG_SDCARD_DATA_RETRIES != 0) */

    if (!prefetch && (length == 1))
    {
        status = stm32l0_sdspi_lock(sdspi, STM32L0_SDSPI_STATE_READY, 0);

        if (status == F_NO_ERROR)
        {
            do
            {
                status = stm32l0_sdspi_command(sdspi, SD_CMD_READ_SINGLE_BLOCK, (address << sdspi->shift), 1);

                if (status == F_NO_ERROR)
                {
                    status = stm32l0_sdspi_receive(sdspi, data, DOSFS_BLK_SIZE, &count);
                    
                    if (status == F_NO_ERROR)
                    {
                        if (count != DOSFS_BLK_SIZE)
                        {
#if (DOSFS_CONFIG_SDCARD_CRC == 1) && (DOSFS_CONFIG_SDCARD_DATA_RETRIES != 0)
                            if (retries)
                            {
                                SDSPI_STATISTICS_COUNT(sdcard_receive_retry);
                                
                                retries--;
                                
                                continue;
                            }
#endif /* (DOSFS_CONFIG_SDCARD_CRC == 1) && (DOSFS_CONFIG_SDCARD_DATA_RETRIES != 0) */
                            
                            SDSPI_STATISTICS_COUNT(sdcard_receive_fail);
                            
                            status = F_ERR_READ;
                        }
                        else
                        {
                            SDSPI_STATISTICS_COUNT(sdcard_read_single);
                            
                            data += DOSFS_BLK_SIZE;
                            
                            address++;
                            length--;
                        }
                    }
                }
            }
            while ((status == F_NO_ERROR) && length);

            status = stm32l0_sdspi_unlock(sdspi, status);
        }
    }
    else
    {
        if (prefetch)
        {
            status = stm32l0_sdspi_lock(sdspi, STM32L0_SDSPI_STATE_READ_MULTIPLE, address);
        }
        else
        {
            status = stm32l0_sdspi_lock(sdspi, STM32L0_SDSPI_STATE_READY, 0);
        }

        if (status == F_NO_ERROR)
        {
            do
            {
                if (sdspi->state != STM32L0_SDSPI_STATE_READ_MULTIPLE)
                {
                    status = stm32l0_sdspi_command(sdspi, SD_CMD_READ_MULTIPLE_BLOCK, (address << sdspi->shift), 1);

                    if (status == F_NO_ERROR)
                    {
                        sdspi->state = STM32L0_SDSPI_STATE_READ_MULTIPLE;
                        sdspi->address = address;
                        sdspi->count = 0;
                    }
                }

                if (status == F_NO_ERROR)
                {
                    status = stm32l0_sdspi_receive(sdspi, data, length * DOSFS_BLK_SIZE, &count);
            
                    if (status == F_NO_ERROR)
                    {
                        count /= DOSFS_BLK_SIZE;
                        
                        SDSPI_STATISTICS_COUNT_N(sdcard_read_multiple, count);
                        
                        if (length != count)
                        {
#if (DOSFS_CONFIG_SDCARD_CRC == 1) && (DOSFS_CONFIG_SDCARD_DATA_RETRIES != 0)
                            data += (DOSFS_BLK_SIZE * count);
                            
                            address += count;
                            length -= count;
                            
                            if (count)
                            {
                                SDSPI_STATISTICS_COUNT(sdcard_receive_retry);
                                
                                retries = DOSFS_CONFIG_SDCARD_DATA_RETRIES;
                                
                                continue;
                            }
                            else
                            {
                                if (retries)
                                {
                                    SDSPI_STATISTICS_COUNT(sdcard_receive_retry);
                                    
                                    retries--;
                                    
                                    continue;
                                }
                            }
#endif /* (DOSFS_CONFIG_SDCARD_CRC == 1) && (DOSFS_CONFIG_SDCARD_DATA_RETRIES != 0) */
                            
                            SDSPI_STATISTICS_COUNT(sdcard_receive_fail);
                            
                            status = F_ERR_READ;
                        }
                        else
                        {
                            sdspi->address += length;
                            sdspi->count += length;

                            length = 0;

                            if (!prefetch)
                            {
                                status = stm32l0_sdspi_read_stop(sdspi);
                            }
                        }
                    }
                }
            }
            while ((status == F_NO_ERROR) && length);
            
            status = stm32l0_sdspi_unlock(sdspi, status);
        }
    }

    return status;
}

static int stm32l0_sdspi_write(void *context, uint32_t address, const uint8_t *data, uint32_t length, bool wait)
{
    stm32l0_sdspi_t *sdspi = (stm32l0_sdspi_t*)context;
    int status = F_NO_ERROR;
    bool check;
#if (DOSFS_CONFIG_SDCARD_CRC == 1) && (DOSFS_CONFIG_SDCARD_DATA_RETRIES != 0)
    unsigned int retries = DOSFS_CONFIG_SDCARD_DATA_RETRIES;
#endif /* (DOSFS_CONFIG_SDCARD_CRC == 1) && (DOSFS_CONFIG_SDCARD_DATA_RETRIES != 0) */

    if (wait && (length == 1))
    {
        status = stm32l0_sdspi_lock(sdspi, STM32L0_SDSPI_STATE_READY, 0);

        if (status == F_NO_ERROR)
        {
            do
            {
                status = stm32l0_sdspi_command(sdspi, SD_CMD_WRITE_SINGLE_BLOCK, (address << sdspi->shift), 0);
            
                if (status == F_NO_ERROR)
                {
                    status = stm32l0_sdspi_transmit(sdspi, SD_START_WRITE_SINGLE_TOKEN, data, DOSFS_BLK_SIZE, &check);
                
                    if (status == F_NO_ERROR)
                    {
                        sdspi->state = STM32L0_SDSPI_STATE_WRITE_STOP;

                        if (check)
                        {
#if (DOSFS_CONFIG_SDCARD_CRC == 1) && (DOSFS_CONFIG_SDCARD_DATA_RETRIES != 0)
                            if (retries)
                            {
                                SDSPI_STATISTICS_COUNT(sdcard_transmit_retry);
                            
                                retries--;
                            
                                continue;
                            }
#endif /* (DOSFS_CONFIG_SDCARD_CRC == 1) && (DOSFS_CONFIG_SDCARD_DATA_RETRIES != 0) */
                        
                            SDSPI_STATISTICS_COUNT(sdcard_transmit_fail);
                        
                            status = F_ERR_WRITE;
                        }
                        else
                        {
                            SDSPI_STATISTICS_COUNT(sdcard_write_single);
                        
                            address++;
                            length--;

                            status = stm32l0_sdspi_write_sync(sdspi);
                        }
                    }
                }
            }
            while ((status == F_NO_ERROR) && length);

            status = stm32l0_sdspi_unlock(sdspi, status);
        }
    }
    else
    {
        status = stm32l0_sdspi_lock(sdspi, STM32L0_SDSPI_STATE_WRITE_MULTIPLE, address);

        if (status == F_NO_ERROR)
        {
            do
            {
                if (sdspi->state != STM32L0_SDSPI_STATE_WRITE_MULTIPLE)
                {
                    status = stm32l0_sdspi_command(sdspi, SD_CMD_WRITE_MULTIPLE_BLOCK, (address << sdspi->shift), 0);
                        
                    if (status == F_NO_ERROR)
                    {
                        sdspi->state = STM32L0_SDSPI_STATE_WRITE_MULTIPLE;
                        sdspi->address = address;
                        sdspi->count = 0;
                    }
                }

                if (status == F_NO_ERROR)
                {
                    status = stm32l0_sdspi_transmit(sdspi, SD_START_WRITE_MULTIPLE_TOKEN, data, length * DOSFS_BLK_SIZE, &check);

                    if (status == F_NO_ERROR)
                    {
                        if (check)
                        {
#if (DOSFS_CONFIG_SDCARD_CRC == 1) && (DOSFS_CONFIG_SDCARD_DATA_RETRIES != 0)
                            uint32_t count, offset;
                            uint8_t temp[4];
                                
                            status = stm32l0_sdspi_command(sdspi, SD_CMD_APP_CMD, 0, 0);
                                
                            if (status == F_NO_ERROR)
                            {
                                status = stm32l0_sdspi_command(sdspi, SD_ACMD_SEND_NUM_WR_BLOCKS, 0, 0);

                                if (status == F_NO_ERROR)
                                {
                                    status = stm32l0_sdspi_receive(sdspi, &temp[0], 4, &count);
                                    
                                    if (status == F_NO_ERROR)
                                    {
                                        if (count == 4)
                                        {
                                            offset = (((uint32_t)temp[0] << 24) |
                                                      ((uint32_t)temp[1] << 16) |
                                                      ((uint32_t)temp[2] <<  8) |
                                                      ((uint32_t)temp[3] <<  0));

                                            if (offset >= sdspi->count)
                                            {
                                                offset -= sdspi->count;

                                                SDSPI_STATISTICS_COUNT_N(sdcard_write_multiple, offset);
                                    
                                                data += (offset * DOSFS_BLK_SIZE);
                                    
                                                address += offset;
                                                length -= offset;

                                                if (offset)
                                                {
                                                    SDSPI_STATISTICS_COUNT(sdcard_transmit_retry);
                                                    
                                                    retries = DOSFS_CONFIG_SDCARD_DATA_RETRIES;
                                                    
                                                    continue;
                                                }
                                                else
                                                {
                                                    if (retries)
                                                    {
                                                        SDSPI_STATISTICS_COUNT(sdcard_transmit_retry);
                                                        
                                                        retries--;
                                                        
                                                        continue;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
#endif /* (DOSFS_CONFIG_SDCARD_CRC == 1) && (DOSFS_CONFIG_SDCARD_DATA_RETRIES != 0) */
                                
                            SDSPI_STATISTICS_COUNT(sdcard_transmit_fail);
                        
                            status = F_ERR_WRITE;
                        }
                        else
                        {
                            SDSPI_STATISTICS_COUNT_N(sdcard_write_multiple, length);

                            sdspi->address += length;
                            sdspi->count += length;

                            length = 0;

                            if (wait)
                            {
                                status = stm32l0_sdspi_write_stop(sdspi);
                                
                                if (status == F_NO_ERROR)
                                {
                                    status = stm32l0_sdspi_write_sync(sdspi);
                                }
                            }
                        }
                    }
                }
            }
            while ((status == F_NO_ERROR) && length);
        
            status = stm32l0_sdspi_unlock(sdspi, status);
        }
    }

    return status;
}

static int stm32l0_sdspi_sync(void *context)
{
    stm32l0_sdspi_t *sdspi = (stm32l0_sdspi_t*)context;
    int status = F_NO_ERROR;

    if (sdspi->state >= STM32L0_SDSPI_STATE_READ_MULTIPLE)
    {
        status = stm32l0_sdspi_lock(sdspi, STM32L0_SDSPI_STATE_READY, 0);

        if (status == F_NO_ERROR)
        {
            status = stm32l0_sdspi_unlock(sdspi, status);
        }
    }

    return status;
}

static const dosfs_device_interface_t stm32l0_sdspi_interface = {
    stm32l0_sdspi_release,
    stm32l0_sdspi_info,
    stm32l0_sdspi_notify,
    stm32l0_sdspi_format,
    stm32l0_sdspi_erase,
    stm32l0_sdspi_discard,
    stm32l0_sdspi_read,
    stm32l0_sdspi_write,
    stm32l0_sdspi_sync,
};

int stm32l0_sdspi_initialize(stm32l0_spi_t *spi, const stm32l0_sdspi_params_t *params)
{
    stm32l0_sdspi_t *sdspi = (stm32l0_sdspi_t*)&stm32l0_sdspi;
    int status = F_NO_ERROR;

    dosfs_device.lock = DOSFS_DEVICE_LOCK_INIT;
    dosfs_device.context = (void*)sdspi;
    dosfs_device.interface = &stm32l0_sdspi_interface;

    sdspi->option = 0;

    if (sdspi->state == STM32L0_SDSPI_STATE_NONE)
    {
        if (spi->state == STM32L0_SPI_STATE_NONE)
        {
            status = F_ERR_INITFUNC;
        }
        else
        {
            sdspi->spi = spi;
            sdspi->pin_cs = params->pin_cs;

            stm32l0_spi_enable(spi); // bump up refcount

            /* Try GO_IDLE after a reset, as the SDCARD could be still
             * powered during the reset.
             */
            stm32l0_sdspi_idle(sdspi, NULL);
            
            sdspi->state = STM32L0_SDSPI_STATE_INIT;
        }
    }
    
    dosfs_device.lock = 0;

    return status;
}
