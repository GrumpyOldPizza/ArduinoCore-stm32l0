/*!
 * \file      sx1272.h
 *
 * \brief     SX1272 driver implementation
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
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#ifndef __SX1272_H__
#define __SX1272_H__

#include <stdint.h>
#include <stdbool.h>
#include "stm32l0_gpio.h"
#include "stm32l0_exti.h"
#include "stm32l0_lptim.h"
#include "stm32l0_rtc.h"
#include "stm32l0_spi.h"
#include "radio.h"
#include "sx1272Regs-Fsk.h"
#include "sx1272Regs-LoRa.h"

/*!
 * Radio wake-up time from sleep
 */
#define RADIO_WAKEUP_TIME                           1 // [ms]

/*!
 * Sync word for Private LoRa networks
 */
#define LORA_MAC_PRIVATE_SYNCWORD                   0x12

/*!
 * Sync word for Public LoRa networks
 */
#define LORA_MAC_PUBLIC_SYNCWORD                    0x34

/*!
 * Radio FSK modem parameters
 */
typedef struct
{
    uint32_t Fdev;
    uint32_t Bandwidth;
    uint32_t BandwidthAfc;
    uint32_t Datarate;
    uint16_t PreambleLen;
    bool     FixLen;
    uint8_t  PayloadLen;
    uint8_t  MaxPayloadLen;
    bool     CrcOn;
    bool     RxContinuous;
    uint8_t  Modulation;
    bool     PreambleInverted;
    uint8_t  SyncSize;
    uint8_t  SyncWord[8];
    bool     AfcOn;
    uint8_t  DcFree;
    uint8_t  CrcType;
    uint8_t  AddressFiltering;
    uint8_t  NodeAddress;
    uint8_t  BroadcastAddress;
    uint32_t TxTimeout;
    uint32_t TxDoneTimeout;
    uint32_t RxSingleTimeout;
}RadioFskSettings_t;

/*!
 * Radio FSK packet handler state
 */
typedef struct
{
    bool     SyncWordDetected;
    bool     PacketSizeReceived;
    int8_t   Rssi;
    uint8_t  ChunkSize;
    uint16_t Size;
    uint16_t NbBytes;
}RadioFskPacketHandler_t;

/*!
 * Radio LoRa modem parameters
 */
typedef struct
{
    uint32_t Bandwidth;
    uint32_t Datarate;
    bool     LowDatarateOptimize;
    uint8_t  Coderate;
    uint16_t PreambleLen;
    bool     FixLen;
    uint8_t  PayloadLen;
    uint8_t  MaxPayloadLen;
    bool     CrcOn;
    bool     FreqHopOn;
    uint8_t  HopPeriod;
    bool     IqInverted;
    bool     RxContinuous;
    uint8_t  SyncWord;
    uint32_t TxTimeout;
}RadioLoRaSettings_t;

/*!
 * Radio LoRa packet handler state
 */
typedef struct
{
    int16_t Rssi;
    int8_t  Snr;
    uint8_t Size;
}RadioLoRaPacketHandler_t;

/*!
 * Radio Settings
 */
typedef struct
{
    uint8_t                  IdleMode;
    int8_t                   Power;
    uint32_t                 Channel;
    uint32_t                 RxTimeout;
    RadioFskSettings_t       Fsk;
    RadioLoRaSettings_t      LoRa;
}RadioSettings_t;

/*!
 * Radio Settings
 */
typedef struct
{
    RadioFskPacketHandler_t  Fsk;
    RadioLoRaPacketHandler_t LoRa;
}RadioPacketHandler_t;

/*!
 * Radio hardware and global parameters
 */
typedef struct SX1272_s
{
    RadioState_t         State;
    RadioModems_t        Modem;
    uint8_t              OpMode;
    bool                 TcxoOn;
    bool                 OscOn;
    bool                 AntSwOn;
    bool                 DioOn;
    const RadioEvents_t  *Events;
    RadioSettings_t      Settings;
    RadioPacketHandler_t PacketHandler;
}SX1272_t;

/*!
 * SX1272 definitions
 */
#define XTAL_FREQ                                   32000000
#define FREQ_STEP                                   61.03515625
#define FREQ_STEP_8                                 15625 /* FREQ_STEP << 8 */

#define RX_BUFFER_SIZE                              256

/*!
 * ============================================================================
 * Public functions prototypes
 * ============================================================================
 */

/*!
 * \brief Initializes the radio
 *
 * \param [IN] events Structure containing the driver callback functions
 * \param [IN] freq Channel RF frequency for rx calibration
 */
void SX1272Init( const RadioEvents_t *events, uint32_t freq );

/*!
 * \brief De-Initializes the radio
 */
void SX1272DeInit( void );

/*!
 * Return current radio status
 *
 * \param status Radio status.[RF_IDLE, RF_RX_RUNNING, RF_TX_RUNNING]
 */
RadioState_t SX1272GetStatus( void );

/*!
 * \brief Configures the radio with the given modem
 *
 * \param [IN] modem Modem to be used [0: FSK, 1: LoRa]
 */
void SX1272SetModem( RadioModems_t modem );

/*!
 * \brief Sets the channel configuration
 *
 * \param [IN] freq         Channel RF frequency
 */
void SX1272SetChannel( uint32_t freq );

/*!
 * \brief Checks if the channel is free for the given time
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] freq       Channel RF frequency
 * \param [IN] rssiThresh RSSI threshold
 * \param [IN] maxCarrierSenseTime Max time while the RSSI is measured
 *
 * \retval isFree         [true: Channel is free, false: Channel is not free]
 */
bool SX1272IsChannelFree( RadioModems_t modem, uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime );

/*!
 * \brief Sets the reception parameters
 *
 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] bandwidth    Sets the bandwidth
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] bandwidthAfc Sets the AFC Bandwidth (FSK only)
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: N/A ( set to 0 )
 * \param [IN] preambleLen  Sets the Preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] symbTimeout  Sets the RxSingle timeout value
 *                          FSK : timeout number of bytes
 *                          LoRa: timeout in symbols
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] payloadLen   Sets payload length when fixed length is used
 * \param [IN] crcOn        Enables/Disables the CRC [0: OFF, 1: ON]
 * \param [IN] freqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: OFF, 1: ON]
 * \param [IN] hopPeriod    Number of symbols between each hop
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: Number of symbols
 * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: not inverted, 1: inverted]
 * \param [IN] rxContinuous Sets the reception in continuous mode
 *                          [false: single mode, true: continuous mode]
 */
void SX1272SetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous );

/*!
 * \brief Sets the transmission parameters
 *
 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] power        Sets the output power [dBm]
 * \param [IN] fdev         Sets the frequency deviation (FSK only)
 *                          FSK : [Hz]
 *                          LoRa: 0
 * \param [IN] bandwidth    Sets the bandwidth (LoRa only)
 *                          FSK : 0
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] preambleLen  Sets the preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] crcOn        Enables disables the CRC [0: OFF, 1: ON]
 * \param [IN] freqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: OFF, 1: ON]
 * \param [IN] hopPeriod    Number of symbols between each hop
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: Number of symbols
 * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: not inverted, 1: inverted]
 * \param [IN] timeout      Transmission timeout [ms]
 */
void SX1272SetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout );

/*!
 * \brief Computes the packet time on air in ms for the given payload
 *
 * \Remark Can only be called once SetRxConfig or SetTxConfig have been called
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] pktLen     Packet payload length
 *
 * \retval airTime        Computed airTime (ms) for the given packet payload length
 */
uint32_t SX1272GetTimeOnAir( RadioModems_t modem, uint8_t pktLen );

/*!
 * \brief Sends the buffer of size. Prepares the packet to be sent and sets
 *        the radio in transmission
 *
 * \param [IN]: buffer     Buffer pointer
 * \param [IN]: size       Buffer size
 */
void SX1272Send( uint8_t *buffer, uint8_t size );

/*!
 * \brief Sets the radio in sleep mode
 */
void SX1272SetSleep( void );

/*!
 * \brief Sets the radio in standby mode
 */
void SX1272SetStby( void );

/*!
 * \brief Sets the radio in reception mode for the given time
 * \param [IN] timeout Reception timeout [ms] [0: continuous, others timeout]
 */
void SX1272SetRx( uint32_t timeout );

/*!
 * \brief Start a Channel Activity Detection
 */
void SX1272StartCad( void );

/*!
 * \brief Sets the radio in continuous wave transmission mode
 *
 * \param [IN]: freq       Channel RF frequency
 * \param [IN]: power      Sets the output power [dBm]
 * \param [IN]: time       Transmission mode timeout [s]
 */
void SX1272SetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time );

/*!
 * \brief Reads the current RSSI value
 *
 * \retval rssiValue Current RSSI value in [dBm]
 */
int16_t SX1272ReadRssi( void );

/*!
 * \brief Delays execution for a timeout in millies
 *
 * \param [IN] timeout
 */
void SX1272Delay( uint32_t timeout );

/*!
 * \brief Sets the maximum payload length.
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] max        Maximum payload length in bytes
 */
void SX1272SetMaxPayloadLength( RadioModems_t modem, uint8_t max );

/*!
 * \brief Sets the network to public or private. Updates the sync byte.
 *
 * \remark Applies to LoRa modem only
 *
 * \param [IN] enable if true, it enables a public network
 */
void SX1272SetPublicNetwork( bool enable );

/*!
 * \brief Sets the modulation shaping for FSK modem.
 *
 * \param [IN] modulation [0: FSK, 1: GFSK_BT_1_0, 2: GFSK_BT_0_5, 3: GFSK_BT_0_3, 4: OOK ]
 */
void SX1272SetModulation( uint8_t modulation );

/*!
 * \brief Sets the Preamble polatity for FSK modem
 *
 * \param [IN] enable [false: 0xaa, true: 0x55]
 */
void SX1272SetPreambleInverted( bool enable );

/*!
 * \brief Sets the SyncWord for FSK modem
 *
 * \param [IN] data Buffer containing the sync word data
 * \param [IN] size Number of bytes in sync word
 */
void SX1272SetSyncWord( const uint8_t *data, uint8_t size );

/*!
 * \brief Enables/Disables AFC for FSK modem
 *
 * \param [IN] enable if true, AFC is enabled
 */
void SX1272SetAfc( bool enable );

/*!
 * \brief Sets the DcFree encoding/decoding for FSK modem.
 *
 * \param [IN] dcFree [0: none, 1: manchester, 2: whitening]
 */
void SX1272SetDcFree( uint8_t dcFree );

/*!
 * \brief Sets the CrcType (CCITT/IBM) for FSK modem.
 *
 * \param [IN] crcType [0: CCITT, 1: IBM]
 */
void SX1272SetCrcType( uint8_t crcType );

/*!
 * \brief Sets the address filtering for FSK modem.
 *
 * \param [IN] addressFiltering address filtering [0: none, 1: node address, 2: node and broadcast address]
 */
void SX1272SetAddressFiltering( uint8_t addressFiltering );

/*!
 * \brief Sets the NodeAddress for FSK modem
 *
 * \param [IN] address NodeAddress
 */
void SX1272SetNodeAddress( uint8_t address );

/*!
 * \brief Sets the BroadcastAddress for FSK modem
 *
 * \param [IN] address BroadcastAddress
 */
void SX1272SetBroadcastAddress( uint8_t address );

/*!
 * \brief Sets the LNA boost
 *
 * \param [IN] enable
 */
void SX1272SetLnaBoost( bool enable );

/*!
 * \brief Sets the LNA boost
 *
 * \param [IN] mode [0: STANDBY, 1: SLEEP]
 */
void SX1272SetIdleMode( uint8_t mode );

/*!
 * \brief Gets the time required for the board plus radio to get out of sleep.[ms]
 *
 * \retval time Radio plus board wakeup time in ms.
 */
uint32_t SX1272GetWakeupTime( void );

/*
 * SX1272 DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
void SX1272OnDio0Irq( void );

/*!
 * \brief DIO 1 IRQ callback
 */
void SX1272OnDio1Irq( void );

/*!
 * \brief DIO 2 IRQ callback
 */
void SX1272OnDio2Irq( void );

#endif // __SX1272_H__
