/*!
 * \file      sx1272.c
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
#include <math.h>
#include <string.h>
#include "utilities.h"
#include "radio.h"
#include "sx1272.h"
#include "sx1272-board.h"

/*
 * Local types definition
 */

/*!
 * Radio registers definition
 */
typedef struct
{
    RadioModems_t Modem;
    uint8_t       Addr;
    uint8_t       Value;
}RadioRegisters_t;

/*!
 * FSK bandwidth definition
 */
typedef struct
{
    uint32_t bandwidth;
    uint8_t  RegValue;
}FskBandwidth_t;


/*
 * Private functions prototypes
 */

/*!
 * Performs the Rx chain calibration for LF and HF bands
 */
static void SX1272ImageCalibration( uint32_t freq );

/*!
 * \brief Writes the buffer contents to the SX1272 FIFO
 *
 * \param [IN] buffer Buffer containing data to be put on the FIFO.
 * \param [IN] size Number of bytes to be written to the FIFO
 */
static void SX1272WriteFifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads the contents of the SX1272 FIFO
 *
 * \param [OUT] buffer Buffer where to copy the FIFO read data.
 * \param [IN] size Number of bytes to be read from the FIFO
 */
static void SX1272ReadFifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Sets the SX1272 operating mode
 *
 * \param [IN] opMode New operating mode
 */
static void SX1272SetOpMode( uint8_t opMode );

/*!
 * \brief RxSingle timeout timer callback
 */
static void SX1272LptimCallback( void *context );

/*!
 * \brief Sets the SX1272 in either STANDBY or SLEEP mode
 */
static void SX1272SetIdle( void );

/*!
 * \brief Startup sequencer
 */
static void SX1272Sequence( RadioState_t state );

/*!
 * \brief TcxoOn timeout timer callback
 */
static void SX1272OnTcxoTimeoutIrq( void );

/*!
 * \brief OscOn timeout timer callback
 */
static void SX1272OnOscTimeoutIrq( void );

/*!
 * \brief RxSingle timeout timer callback
 */
static void SX1272OnRxSingleTimeoutIrq( void );

/*!
 * \brief Rx timeout timer callback
 */
static void SX1272OnRxTimeoutIrq( void );

/*!
 * \brief TxDone timeout timer callback
 */
static void SX1272OnTxDoneTimeoutIrq( void );

/*!
 * \brief Tx timeout timer callback
 */
static void SX1272OnTxTimeoutIrq( void );

/*
 * Private global constants
 */

/*!
 * Radio hardware registers initialization
 *
 * \remark RADIO_INIT_REGISTERS_VALUE is defined in sx1272-board.h file
 */
static const RadioRegisters_t RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET                                 -139

/*!
 * Precomputed FSK bandwidth registers values
 */
static const FskBandwidth_t FskBandwidths[] =
{
    { 2600  , 0x17 },
    { 3100  , 0x0F },
    { 3900  , 0x07 },
    { 5200  , 0x16 },
    { 6300  , 0x0E },
    { 7800  , 0x06 },
    { 10400 , 0x15 },
    { 12500 , 0x0D },
    { 15600 , 0x05 },
    { 20800 , 0x14 },
    { 25000 , 0x0C },
    { 31300 , 0x04 },
    { 41700 , 0x13 },
    { 50000 , 0x0B },
    { 62500 , 0x03 },
    { 83333 , 0x12 },
    { 100000, 0x0A },
    { 125000, 0x02 },
    { 166700, 0x11 },
    { 200000, 0x09 },
    { 250000, 0x01 },
};

/*
 * Private global variables
 */

/*!
 * Reception buffer
 */
static uint8_t RxTxBuffer[RX_BUFFER_SIZE];

/*
 * Public global variables
 */

/*!
 * Radio hardware and global parameters
 */
SX1272_t SX1272;

/*
 * Radio driver functions implementation
 */

void SX1272Init( const RadioEvents_t *events, uint32_t freq )
{
    uint8_t i;

    SX1272Reset( );

    SX1272.State = RF_IDLE;
    SX1272.Modem = MODEM_FSK;
    SX1272.OpMode = RF_OPMODE_SLEEP;
    SX1272.TcxoOn = false;
    SX1272.OscOn = false;
    SX1272.AntSwOn = false;
    SX1272.DioOn = false;

    SX1272.Events = events;

    SX1272.Settings.IdleMode = IDLE_STANDBY;
    SX1272.Settings.Power = 14;
    SX1272.Settings.Channel = freq;

    SX1272.Settings.Fsk.MaxPayloadLen = 255;
    SX1272.Settings.Fsk.Modulation = 0; // FSK
    SX1272.Settings.Fsk.PreambleInverted = false;
    SX1272.Settings.Fsk.SyncSize = 3;
    SX1272.Settings.Fsk.SyncWord[0] = 0xc1;
    SX1272.Settings.Fsk.SyncWord[1] = 0x94;
    SX1272.Settings.Fsk.SyncWord[2] = 0xc1;
    SX1272.Settings.Fsk.AfcOn = true;
    SX1272.Settings.Fsk.DcFree = 2;
    SX1272.Settings.Fsk.CrcType = 0;
    SX1272.Settings.Fsk.AddressFiltering = 0;
    SX1272.Settings.Fsk.NodeAddress = 0x00;
    SX1272.Settings.Fsk.BroadcastAddress = 0xff;

    SX1272.Settings.LoRa.MaxPayloadLen = 255;
    SX1272.Settings.LoRa.SyncWord = LORA_MAC_PRIVATE_SYNCWORD;

    // Force switch to init default LoRa Modem settings 
    SX1272SetModem( MODEM_LORA );

    SX1272SetStby( );

    // Initialize radio default values
    for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
    {
        SX1272SetModem( RadioRegsInit[i].Modem );
        SX1272Write( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }

    // Force switch to init default FSK Modem settings 
    SX1272SetModem( MODEM_FSK );

    // Calibrate Rx chain
    SX1272ImageCalibration( freq );

    SX1272SetSleep( );
}

void SX1272DeInit( void )
{
}

RadioState_t SX1272GetStatus( void )
{
    return SX1272.State;
}

void SX1272SetChannel( uint32_t freq )
{
    uint32_t frf;

    SX1272.Settings.Channel = freq;

    frf  = freq / FREQ_STEP_8;
    freq = freq - ( frf * FREQ_STEP_8 );
    frf  = ( frf << 8 ) + ( ( ( freq << 8 ) + ( FREQ_STEP_8 / 2 ) ) / FREQ_STEP_8 ); 

    SX1272Write( REG_FRFMSB, ( uint8_t )( frf >> 16 ) );
    SX1272Write( REG_FRFMID, ( uint8_t )( frf >> 8  ) );
    SX1272Write( REG_FRFLSB, ( uint8_t )( frf >> 0  ) );

    SX1272Release( );
}

bool SX1272IsChannelFree( RadioModems_t modem, uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime )
{
    bool status = true;
    int16_t rssi = 0;
    uint32_t carrierSenseTime = 0;

    SX1272SetStby( );

    SX1272SetModem( modem );

    SX1272SetChannel( freq );

    SX1272SetOpMode( RF_OPMODE_RECEIVER );

    SX1272Delay( 1 );

    carrierSenseTime = armv6m_systick_millis( );

    // Perform carrier sense for maxCarrierSenseTime
    while( (uint32_t)( armv6m_systick_millis( ) - carrierSenseTime ) < maxCarrierSenseTime )
    {
        rssi = SX1272ReadRssi( );

        if( rssi > rssiThresh )
        {
            status = false;
            break;
        }
    }

    SX1272SetStby( );

    return status;
}

/*!
 * Performs the Rx chain calibration for LF and HF bands
 */
static void SX1272ImageCalibration( uint32_t freq )
{
    uint8_t paConfig;

    // Save context
    paConfig = SX1272Read( REG_PACONFIG );

    // Cut the PA just in case, RFO output, power = -1 dBm
    SX1272Write( REG_PACONFIG, 0x00 );

    // Sets a Frequency in LF/HF band
    SX1272SetChannel( freq );

    SX1272Write( REG_IMAGECAL, ( SX1272Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( SX1272Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Restore context
    SX1272Write( REG_PACONFIG, paConfig );
}

/*!
 * Returns the known FSK bandwidth registers value
 *
 * \param [IN] bandwidth Bandwidth value in Hz
 * \retval regValue Bandwidth register value.
 */
static uint8_t SX1272GetFskBandwidthRegValue( uint32_t bandwidth )
{
    uint8_t i;

    for( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) - 1 ); i++ )
    {
        if( bandwidth <= FskBandwidths[i].bandwidth )
        {
            break;
        }
    }

    return FskBandwidths[i].RegValue;
}

void SX1272SetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{
    SX1272SetModem( modem );

    if( modem == MODEM_FSK )
    {
        SX1272.Settings.Fsk.Bandwidth = bandwidth;
        SX1272.Settings.Fsk.Datarate = datarate;
        SX1272.Settings.Fsk.BandwidthAfc = bandwidthAfc;
        SX1272.Settings.Fsk.FixLen = fixLen;
        SX1272.Settings.Fsk.PayloadLen = payloadLen;
        SX1272.Settings.Fsk.CrcOn = crcOn;
        SX1272.Settings.Fsk.RxContinuous = rxContinuous;
        SX1272.Settings.Fsk.PreambleLen = preambleLen;
        SX1272.Settings.Fsk.RxSingleTimeout = (rxContinuous ? 0 : (( 8 * symbTimeout * 32768 + ( datarate - 1 ) ) / datarate));

        datarate = ( ( 16 * XTAL_FREQ ) + ( datarate / 2 ) ) / datarate;
        SX1272Write( REG_BITRATEMSB,  ( uint8_t )( datarate >> 12 ) );
        SX1272Write( REG_BITRATELSB,  ( uint8_t )( datarate >> 4  ) );
        SX1272Write( REG_BITRATEFRAC, ( uint8_t )( datarate >> 0  ) );

        SX1272Write( REG_RXBW, SX1272GetFskBandwidthRegValue( bandwidth ) );
        SX1272Write( REG_AFCBW, SX1272GetFskBandwidthRegValue( bandwidthAfc ) );

        SX1272Write( REG_PREAMBLEMSB, ( uint8_t )( preambleLen >> 8 ) );
        SX1272Write( REG_PREAMBLELSB, ( uint8_t )( preambleLen >> 0 ) );
    }
    else
    {
        SX1272.Settings.LoRa.Bandwidth = bandwidth;
        SX1272.Settings.LoRa.Datarate = datarate;
        SX1272.Settings.LoRa.Coderate = coderate;
        SX1272.Settings.LoRa.PreambleLen = preambleLen;
        SX1272.Settings.LoRa.FixLen = fixLen;
        SX1272.Settings.LoRa.PayloadLen = payloadLen;
        SX1272.Settings.LoRa.CrcOn = crcOn;
        SX1272.Settings.LoRa.FreqHopOn = freqHopOn;
        SX1272.Settings.LoRa.HopPeriod = hopPeriod;
        SX1272.Settings.LoRa.IqInverted = iqInverted;
        SX1272.Settings.LoRa.RxContinuous = rxContinuous;

        if( datarate > 12 )
        {
            datarate = 12;
        }
        else if( datarate < 6 )
        {
            datarate = 6;
        }

        if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
            ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
        {
            SX1272.Settings.LoRa.LowDatarateOptimize = 0x01;
        }
        else
        {
            SX1272.Settings.LoRa.LowDatarateOptimize = 0x00;
        }

        if( rxContinuous == true )
        {
            symbTimeout = 0;        
        }

        SX1272Write( REG_LR_MODEMCONFIG1,
                     ( ( bandwidth << 6 ) |
                       ( coderate << 3 ) |
                       ( fixLen << 2 ) |
                       ( crcOn << 1 ) |
                       ( SX1272.Settings.LoRa.LowDatarateOptimize << 0 ) ) );

        SX1272Write( REG_LR_MODEMCONFIG2,
                     ( ( datarate << 4 ) |
                       RFLR_MODEMCONFIG2_AGCAUTO_ON |
                       ( symbTimeout >> 8 ) ) );

        SX1272Write( REG_LR_SYMBTIMEOUTLSB, ( uint8_t )( symbTimeout & 0xFF ) );

        SX1272Write( REG_LR_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
        SX1272Write( REG_LR_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

        if( fixLen == 1 )
        {
            SX1272Write( REG_LR_PAYLOADLENGTH, payloadLen );
        }

        if( datarate == 6 )
        {
            SX1272Write( REG_LR_DETECTOPTIMIZE,
                         ( SX1272Read( REG_LR_DETECTOPTIMIZE ) &
                           RFLR_DETECTIONOPTIMIZE_MASK ) |
                         RFLR_DETECTIONOPTIMIZE_SF6 );
            SX1272Write( REG_LR_DETECTIONTHRESHOLD,
                         RFLR_DETECTIONTHRESH_SF6 );
        }
        else
        {
            SX1272Write( REG_LR_DETECTOPTIMIZE,
                         ( SX1272Read( REG_LR_DETECTOPTIMIZE ) &
                           RFLR_DETECTIONOPTIMIZE_MASK ) |
                         RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
            SX1272Write( REG_LR_DETECTIONTHRESHOLD,
                         RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
        }
    }

    SX1272Release( );
}

void SX1272SetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
    uint32_t txTimeout;

    txTimeout = ( timeout * 32768 + 999 ) / 1000;

    SX1272SetModem( modem );

    SX1272SetRfTxPower( power );

    SX1272.Settings.Power = power;

    if( modem == MODEM_FSK )
    {
        SX1272.Settings.Fsk.Fdev = fdev;
        SX1272.Settings.Fsk.Datarate = datarate;
        SX1272.Settings.Fsk.PreambleLen = preambleLen;
        SX1272.Settings.Fsk.FixLen = fixLen;
        SX1272.Settings.Fsk.CrcOn = crcOn;
        SX1272.Settings.Fsk.TxTimeout = txTimeout;
        SX1272.Settings.Fsk.TxDoneTimeout = ( 2 * 32768 + ( datarate - 1 ) ) / datarate;

        fdev = ( ( fdev << 8 ) + ( FREQ_STEP_8 / 2 ) ) / FREQ_STEP_8;
        SX1272Write( REG_FDEVMSB, ( uint8_t )( fdev >> 8 ) );
        SX1272Write( REG_FDEVLSB, ( uint8_t )( fdev >> 0 ) );

        datarate = ( ( 16 * XTAL_FREQ ) + ( datarate / 2 ) ) / datarate;
        SX1272Write( REG_BITRATEMSB,  ( uint8_t )( datarate >> 12 ) );
        SX1272Write( REG_BITRATELSB,  ( uint8_t )( datarate >> 4  ) );
        SX1272Write( REG_BITRATEFRAC, ( uint8_t )( datarate >> 0  ) );

        SX1272Write( REG_PREAMBLEMSB, ( uint8_t )( preambleLen >> 8 ) );
        SX1272Write( REG_PREAMBLELSB, ( uint8_t )( preambleLen >> 0 ) );
    }
    else
    {
        SX1272.Settings.LoRa.Bandwidth = bandwidth;
        SX1272.Settings.LoRa.Datarate = datarate;
        SX1272.Settings.LoRa.Coderate = coderate;
        SX1272.Settings.LoRa.PreambleLen = preambleLen;
        SX1272.Settings.LoRa.FixLen = fixLen;
        SX1272.Settings.LoRa.FreqHopOn = freqHopOn;
        SX1272.Settings.LoRa.HopPeriod = hopPeriod;
        SX1272.Settings.LoRa.CrcOn = crcOn;
        SX1272.Settings.LoRa.IqInverted = iqInverted;
        SX1272.Settings.LoRa.TxTimeout = txTimeout;

        if( datarate > 12 )
        {
            datarate = 12;
        }
        else if( datarate < 6 )
        {
            datarate = 6;
        }
        if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
            ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
        {
            SX1272.Settings.LoRa.LowDatarateOptimize = 0x01;
        }
        else
        {
            SX1272.Settings.LoRa.LowDatarateOptimize = 0x00;
        }

        SX1272Write( REG_LR_MODEMCONFIG1,
                     ( ( bandwidth << 6 ) |
                       ( coderate << 3 ) |
                       ( fixLen << 2 ) |
                       ( crcOn << 1 ) |
                       ( SX1272.Settings.LoRa.LowDatarateOptimize << 0 ) ) );

        SX1272Write( REG_LR_MODEMCONFIG2,
                     ( ( datarate << 4 ) |
                       RFLR_MODEMCONFIG2_AGCAUTO_ON ) );

        SX1272Write( REG_LR_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
        SX1272Write( REG_LR_PREAMBLELSB, preambleLen & 0xFF );

        if( datarate == 6 )
        {
            SX1272Write( REG_LR_DETECTOPTIMIZE,
                         ( SX1272Read( REG_LR_DETECTOPTIMIZE ) &
                           RFLR_DETECTIONOPTIMIZE_MASK ) |
                         RFLR_DETECTIONOPTIMIZE_SF6 );
            SX1272Write( REG_LR_DETECTIONTHRESHOLD,
                         RFLR_DETECTIONTHRESH_SF6 );
        }
        else
        {
            SX1272Write( REG_LR_DETECTOPTIMIZE,
                         ( SX1272Read( REG_LR_DETECTOPTIMIZE ) &
                           RFLR_DETECTIONOPTIMIZE_MASK ) |
                         RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
            SX1272Write( REG_LR_DETECTIONTHRESHOLD,
                         RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
        }
    }

    SX1272Release( );
}

uint32_t SX1272GetTimeOnAir( RadioModems_t modem, uint8_t pktLen )
{
    uint32_t airTime = 0;

    if( modem == MODEM_FSK )
    {
        airTime = ( 8 * ( SX1272.Settings.Fsk.PreambleLen +
                          SX1272.Settings.Fsk.SyncSize +
                          ( ( ( SX1272.Settings.Fsk.FixLen ? 0 : 1 ) +
                              pktLen +
                              ( SX1272.Settings.Fsk.CrcOn ? 2 : 0 ) ) *
                            ( ( SX1272.Settings.Fsk.DcFree == 1) ? 2 : 1 ) ) )
                    * 1000 + ( SX1272.Settings.Fsk.Datarate - 1 ) ) / SX1272.Settings.Fsk.Datarate;
    }
    else
    {
        uint32_t bw = 0;

        switch( SX1272.Settings.LoRa.Bandwidth )
        {
        case 0: // 125 kHz
            bw = 125000;
            break;
        case 1: // 250 kHz
            bw = 250000;
            break;
        case 2: // 500 kHz
            bw = 500000;
            break;
        }

        int32_t temp;
        uint32_t nPayload;

        temp = ( 8 * pktLen - ( 4 * SX1272.Settings.LoRa.Datarate ) + 28 + ( SX1272.Settings.LoRa.CrcOn ? 16 : 0 ) - ( SX1272.Settings.LoRa.FixLen ? 20 : 0 ) );

        if ( temp <= 0 )
        {
            nPayload = 8;
        }
        else
        {
            nPayload = ( 8 + ( ( ( temp + ( 4 * ( SX1272.Settings.LoRa.Datarate - ( SX1272.Settings.LoRa.LowDatarateOptimize ? 2 : 0 ) ) - 1 ) ) /
                                 ( 4 * ( SX1272.Settings.LoRa.Datarate - ( SX1272.Settings.LoRa.LowDatarateOptimize ? 2 : 0 ) ) ) ) *
                               ( SX1272.Settings.LoRa.Coderate + 4 ) ) );
        }

        airTime = ( ( 1000 * ( SX1272.Settings.LoRa.PreambleLen + nPayload ) + 4250 ) * ( 1 << SX1272.Settings.LoRa.Datarate ) + ( bw - 1 ) ) / bw;
    }
    return airTime;
}

void SX1272Send( uint8_t *buffer, uint8_t size )
{
    if( SX1272.Modem == MODEM_FSK )
    {
        SX1272.PacketHandler.Fsk.NbBytes = 0;
        SX1272.PacketHandler.Fsk.Size = size;
        SX1272.PacketHandler.Fsk.ChunkSize = 32;

        if ( ( ( SX1272.Settings.Fsk.FixLen ? 0 : 1) + SX1272.PacketHandler.Fsk.Size ) <= 64 )
        {
            // DIO0=PacketSent
            // DIO1=
            // DIO2=
            // DIO3=TxReady
            SX1272Write( REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00 | RF_DIOMAPPING1_DIO1_11 | RF_DIOMAPPING1_DIO2_01 | RF_DIOMAPPING1_DIO3_01 );
        }
        else
        {
            // DIO0=PacketSent
            // DIO1=FifoLevel
            // DIO2=
            // DIO3=TxReady
            SX1272Write( REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00 | RF_DIOMAPPING1_DIO1_00 | RF_DIOMAPPING1_DIO2_01 | RF_DIOMAPPING1_DIO3_01 );
        }

        SX1272Write( REG_SYNCCONFIG,
                     ( RF_SYNCCONFIG_AUTORESTARTRXMODE_OFF |
                       ( SX1272.Settings.Fsk.PreambleInverted ? RF_SYNCCONFIG_PREAMBLEPOLARITY_55 : RF_SYNCCONFIG_PREAMBLEPOLARITY_AA ) |
                       RF_SYNCCONFIG_SYNC_ON |
                       ( SX1272.Settings.Fsk.SyncSize - 1 ) ) );

        SX1272Write( REG_PACKETCONFIG1,
                     ( SX1272.Settings.Fsk.FixLen ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
                     ( SX1272.Settings.Fsk.DcFree << 5 ) |
                     ( SX1272.Settings.Fsk.CrcOn << 4 ) |
                     RF_PACKETCONFIG1_CRCAUTOCLEAR_OFF |
                     ( SX1272.Settings.Fsk.AddressFiltering << 1 ) |
                     ( SX1272.Settings.Fsk.CrcType << 0 ) );

        SX1272Write( REG_PACKETCONFIG2, ( RF_PACKETCONFIG2_DATAMODE_PACKET |
                                          ( ( SX1272.Settings.Fsk.FixLen ? SX1272.PacketHandler.Fsk.Size : 0 ) >> 8 ) ) );

        SX1272Write( REG_PAYLOADLENGTH, ( SX1272.Settings.Fsk.FixLen ? SX1272.PacketHandler.Fsk.Size : 0 ) );
    }
    else
    {
        if( SX1272.Settings.LoRa.IqInverted == true )
        {
            SX1272Write( REG_LR_INVERTIQ, ( ( SX1272Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON ) );
            SX1272Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
        }
        else
        {
            SX1272Write( REG_LR_INVERTIQ, ( ( SX1272Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
            SX1272Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
        }

        SX1272.PacketHandler.LoRa.Size = size;

        // Initializes the payload size
        SX1272Write( REG_LR_PAYLOADLENGTH, size );

        // Full buffer used for Tx
        SX1272Write( REG_LR_FIFOTXBASEADDR, 0 );
        SX1272Write( REG_LR_FIFOADDRPTR, 0 );

        if( SX1272.Settings.LoRa.FreqHopOn == true )
        {
            SX1272Write( REG_LR_PLLHOP, ( SX1272Read( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
            SX1272Write( REG_LR_HOPPERIOD, SX1272.Settings.LoRa.HopPeriod );

            SX1272Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                         RFLR_IRQFLAGS_RXDONE |
                         RFLR_IRQFLAGS_PAYLOADCRCERROR |
                         RFLR_IRQFLAGS_VALIDHEADER |
                         //RFLR_IRQFLAGS_TXDONE |
                         RFLR_IRQFLAGS_CADDONE |
                         //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                         RFLR_IRQFLAGS_CADDETECTED );

            // DIO0=TxDone
            // DIO1=
            // DIO2=FhssChangeChannel
            // DIO3=
            SX1272Write( REG_DIOMAPPING1, RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_11 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_11 );
        }
        else
        {
            SX1272Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                         RFLR_IRQFLAGS_RXDONE |
                         RFLR_IRQFLAGS_PAYLOADCRCERROR |
                         RFLR_IRQFLAGS_VALIDHEADER |
                         //RFLR_IRQFLAGS_TXDONE |
                         RFLR_IRQFLAGS_CADDONE |
                         RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                         RFLR_IRQFLAGS_CADDETECTED );

            // DIO0=TxDone
            // DIO1=
            // DIO2=
            // DIO3=
            SX1272Write( REG_DIOMAPPING1, RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_11 | RFLR_DIOMAPPING1_DIO2_11 | RFLR_DIOMAPPING1_DIO3_11 );
        }
    }

    memcpy( RxTxBuffer, buffer, size );

    SX1272Sequence( RF_TX_RUNNING );
}

void SX1272SetSleep( void )
{
    uint8_t regOpMode;

    stm32l0_lptim_stop();

    if( SX1272.DioOn )
    {
        SX1272DioDeInit( );
        
        SX1272.DioOn = false;
    }

    if( SX1272.AntSwOn )
    {
        SX1272AntSwDeInit( );

        SX1272.AntSwOn = false;
    }

    if( SX1272.OpMode != RF_OPMODE_SLEEP )
    {
        // On a switch from RX to STANDBY/SLEEP sometimes the switch request is ignored (RegRxDelay)
        regOpMode = SX1272Read( REG_OPMODE ) & RF_OPMODE_MASK;

        do
        {
            SX1272Write( REG_OPMODE, regOpMode | RF_OPMODE_SLEEP );
        }
        while ( ( SX1272Read( REG_OPMODE ) & ~RF_OPMODE_MASK ) != RF_OPMODE_SLEEP );

        SX1272.OpMode = RF_OPMODE_SLEEP;
    }        

    SX1272.OscOn = false;

    if( SX1272.Modem == MODEM_FSK )
    {
        // Clear Irqs
        SX1272Write( REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN );
    }
    else
    {
        // Clear Irqs
        SX1272Write( REG_LR_IRQFLAGSMASK,
                     ( RFLR_IRQFLAGS_RXTIMEOUT |
                       RFLR_IRQFLAGS_RXDONE |
                       RFLR_IRQFLAGS_PAYLOADCRCERROR |
                       RFLR_IRQFLAGS_VALIDHEADER |
                       RFLR_IRQFLAGS_TXDONE |
                       RFLR_IRQFLAGS_CADDONE |
                       RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                       RFLR_IRQFLAGS_CADDETECTED ) );

        if( SX1272.Settings.LoRa.FreqHopOn == true )
        {
            SX1272Write( REG_LR_PLLHOP, ( SX1272Read( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_OFF );
            SX1272Write( REG_LR_HOPPERIOD, 0 );
        }
    }

    if( SX1272.TcxoOn )
    {
        SX1272SetBoardTcxo( false );

        SX1272.TcxoOn = false;
    }

    SX1272Release( );

    SX1272.State = RF_IDLE;
}

void SX1272SetStby( void )
{
    uint32_t tcxoTimeout;
    uint8_t  regOpMode;

    stm32l0_lptim_stop();

    if( SX1272.DioOn )
    {
        SX1272DioDeInit( );
        
        SX1272.DioOn = false;
    }

    if( !SX1272.TcxoOn )
    {
        SX1272SetBoardTcxo( true );

        tcxoTimeout = SX1272GetBoardTcxoWakeupTime( );

        if( tcxoTimeout )
        {
            SX1272Delay( tcxoTimeout );
        }

        SX1272.TcxoOn = true;
    }

    if( SX1272.OpMode != RF_OPMODE_STANDBY )
    {
        // On a switch from RX to STANDBY/SLEEP sometimes the switch request is ignored (RegRxDelay)
        regOpMode = SX1272Read( REG_OPMODE ) & RF_OPMODE_MASK;

        do
        {
            SX1272Write( REG_OPMODE, regOpMode | RF_OPMODE_STANDBY );
        }
        while ( ( SX1272Read( REG_OPMODE ) & ~RF_OPMODE_MASK ) != RF_OPMODE_STANDBY );

        SX1272.OpMode = RF_OPMODE_STANDBY;
    }        

    if( !SX1272.OscOn )
    {
        SX1272Delay( RADIO_WAKEUP_TIME );
        
        SX1272.OscOn = true;
    }

    if( SX1272.Modem == MODEM_FSK )
    {
        // Clear Irqs
        SX1272Write( REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN );
    }
    else
    {
        // Clear Irqs
        SX1272Write( REG_LR_IRQFLAGSMASK,
                     ( RFLR_IRQFLAGS_RXTIMEOUT |
                       RFLR_IRQFLAGS_RXDONE |
                       RFLR_IRQFLAGS_PAYLOADCRCERROR |
                       RFLR_IRQFLAGS_VALIDHEADER |
                       RFLR_IRQFLAGS_TXDONE |
                       RFLR_IRQFLAGS_CADDONE |
                       RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                       RFLR_IRQFLAGS_CADDETECTED ) );

        if( SX1272.Settings.LoRa.FreqHopOn == true )
        {
            SX1272Write( REG_LR_PLLHOP, ( SX1272Read( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_OFF );
            SX1272Write( REG_LR_HOPPERIOD, 0 );
        }
    }

    if( !SX1272.AntSwOn )
    {
        SX1272AntSwInit( );

        SX1272.AntSwOn = true;
    }

    SX1272Release( );

    SX1272.State = RF_IDLE;
}

static void SX1272SetIdle( void )
{
    if( SX1272.Settings.IdleMode == IDLE_SLEEP )
    {
        SX1272SetSleep();
    }
    else
    {
        SX1272SetStby();
    }
}

void SX1272SetRx( uint32_t timeout )
{
    if( SX1272.Modem == MODEM_FSK )
    {
        SX1272.PacketHandler.Fsk.SyncWordDetected = false;

        if( SX1272.Settings.Fsk.FixLen == true )
        {
            SX1272.PacketHandler.Fsk.PacketSizeReceived = true;
            SX1272.PacketHandler.Fsk.Size = SX1272.Settings.Fsk.PayloadLen;
        }

        // ERRATA 3.1 - PayloadReady set for 31.5ns if FIFO is empty, so make sure FIFO empty is never set
        SX1272.PacketHandler.Fsk.ChunkSize = 32 - 1;

        if( ( SX1272.Settings.Fsk.FixLen ? SX1272.Settings.Fsk.PayloadLen : ( 1 + SX1272.Settings.Fsk.MaxPayloadLen ) ) <= 64 )
        {
            // DIO0=PayloadReady
            // DIO1=
            // DIO2=SyncAddr
            // DIO3=
            SX1272Write( REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00 | RF_DIOMAPPING1_DIO1_11 | RF_DIOMAPPING1_DIO2_11 | RF_DIOMAPPING1_DIO3_01 );
        }
        else
        {
            // DIO0=PayloadReady
            // DIO1=FifoLevel
            // DIO2=SyncAddr
            // DIO3=
            SX1272Write( REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00 | RF_DIOMAPPING1_DIO1_00 | RF_DIOMAPPING1_DIO2_11 | RF_DIOMAPPING1_DIO3_01 );
        }

        SX1272Write( REG_RXCONFIG, ( SX1272.Settings.Fsk.AfcOn ? RF_RXCONFIG_AFCAUTO_ON : 0 ) | RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT );

        SX1272Write( REG_SYNCCONFIG,
                     ( ( SX1272.Settings.Fsk.RxContinuous ? RF_SYNCCONFIG_AUTORESTARTRXMODE_WAITPLL_OFF : RF_SYNCCONFIG_AUTORESTARTRXMODE_OFF ) |
                       ( SX1272.Settings.Fsk.PreambleInverted ? RF_SYNCCONFIG_PREAMBLEPOLARITY_55 : RF_SYNCCONFIG_PREAMBLEPOLARITY_AA ) |
                       RF_SYNCCONFIG_SYNC_ON |
                       ( SX1272.Settings.Fsk.SyncSize - 1 ) ) );

        SX1272Write( REG_PACKETCONFIG1,
                     ( SX1272.Settings.Fsk.FixLen ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
                     ( SX1272.Settings.Fsk.DcFree << 5 ) |
                     ( SX1272.Settings.Fsk.CrcOn << 4 ) |
                     RF_PACKETCONFIG1_CRCAUTOCLEAR_OFF |
                     ( SX1272.Settings.Fsk.AddressFiltering << 1 ) |
                     ( SX1272.Settings.Fsk.CrcType << 0 ) );

        SX1272Write( REG_PACKETCONFIG2,
                     ( RF_PACKETCONFIG2_DATAMODE_PACKET |
                       ( ( SX1272.Settings.Fsk.FixLen ? SX1272.Settings.Fsk.PayloadLen : SX1272.Settings.Fsk.MaxPayloadLen ) >> 8 ) ) );

        SX1272Write( REG_PAYLOADLENGTH, ( SX1272.Settings.Fsk.FixLen ? SX1272.Settings.Fsk.PayloadLen : SX1272.Settings.Fsk.MaxPayloadLen ) );
    }
    else
    {
        if( SX1272.Settings.LoRa.IqInverted == true )
        {
            SX1272Write( REG_LR_INVERTIQ, ( ( SX1272Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
            SX1272Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
        }
        else
        {
            SX1272Write( REG_LR_INVERTIQ, ( ( SX1272Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
            SX1272Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
        }

        // ERRATA 2.2 - Receiver Spurious Reception of a LoRa Signal
        if( SX1272.Settings.LoRa.Bandwidth < 2 )
        {
            SX1272Write( REG_LR_DETECTOPTIMIZE, SX1272Read( REG_LR_DETECTOPTIMIZE ) & 0x7F );
        }
        else
        {
            SX1272Write( REG_LR_DETECTOPTIMIZE, SX1272Read( REG_LR_DETECTOPTIMIZE ) | 0x80 );
        }

        if( SX1272.Settings.LoRa.FreqHopOn == true )
        {
            SX1272Write( REG_LR_PLLHOP, ( SX1272Read( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
            SX1272Write( REG_LR_HOPPERIOD, SX1272.Settings.LoRa.HopPeriod );

            if( SX1272.Settings.LoRa.RxContinuous == true )
            {
                SX1272Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                             //RFLR_IRQFLAGS_RXDONE |
                             RFLR_IRQFLAGS_PAYLOADCRCERROR |
                             RFLR_IRQFLAGS_VALIDHEADER |
                             RFLR_IRQFLAGS_TXDONE |
                             RFLR_IRQFLAGS_CADDONE |
                             //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                             RFLR_IRQFLAGS_CADDETECTED );
                
                // DIO0=RxDone
                // DIO1=
                // DIO2=FhssChangeChannel
                // DIO3=
                SX1272Write( REG_DIOMAPPING1, RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_11 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_11 );
            }
            else
            {
                SX1272Write( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                             //RFLR_IRQFLAGS_RXDONE |
                             RFLR_IRQFLAGS_PAYLOADCRCERROR |
                             RFLR_IRQFLAGS_VALIDHEADER |
                             RFLR_IRQFLAGS_TXDONE |
                             RFLR_IRQFLAGS_CADDONE |
                             //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                             RFLR_IRQFLAGS_CADDETECTED );
                
                // DIO0=RxDone
                // DIO1=RxTimeout
                // DIO2=FhssChangeChannel
                // DIO3=
                SX1272Write( REG_DIOMAPPING1, RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_11 );
            }
        }
        else
        {
            if( SX1272.Settings.LoRa.RxContinuous == true )
            {
                SX1272Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                             //RFLR_IRQFLAGS_RXDONE |
                             RFLR_IRQFLAGS_PAYLOADCRCERROR |
                             RFLR_IRQFLAGS_VALIDHEADER |
                             RFLR_IRQFLAGS_TXDONE |
                             RFLR_IRQFLAGS_CADDONE |
                             RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                             RFLR_IRQFLAGS_CADDETECTED );
                
                // DIO0=RxDone
                // DIO1=
                // DIO2=
                // DIO3=
                SX1272Write( REG_DIOMAPPING1, RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_11 | RFLR_DIOMAPPING1_DIO2_11 | RFLR_DIOMAPPING1_DIO3_11 );
            }
            else
            {
                SX1272Write( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                             //RFLR_IRQFLAGS_RXDONE |
                             RFLR_IRQFLAGS_PAYLOADCRCERROR |
                             RFLR_IRQFLAGS_VALIDHEADER |
                             RFLR_IRQFLAGS_TXDONE |
                             RFLR_IRQFLAGS_CADDONE |
                             RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                             RFLR_IRQFLAGS_CADDETECTED );
                
                // DIO0=RxDone
                // DIO1=RxTimeout
                // DIO2=
                // DIO3=
                SX1272Write( REG_DIOMAPPING1, RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_11 | RFLR_DIOMAPPING1_DIO3_11 );
            }
        }

        SX1272Write( REG_LR_FIFORXBASEADDR, 0 );
        SX1272Write( REG_LR_FIFOADDRPTR, 0 );
    }

    SX1272.Settings.RxTimeout = ( timeout * 32768 + 999 ) / 1000;

    SX1272Sequence( RF_RX_RUNNING );
}

void SX1272StartCad( void )
{
    if( SX1272.Modem == MODEM_LORA )
    {
        if( SX1272.Settings.LoRa.IqInverted == true )
        {
            SX1272Write( REG_LR_INVERTIQ, ( ( SX1272Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
            SX1272Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
        }
        else
        {
            SX1272Write( REG_LR_INVERTIQ, ( ( SX1272Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
            SX1272Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
        }

        // ERRATA 2.2 - Receiver Spurious Reception of a LoRa Signal
        if( SX1272.Settings.LoRa.Bandwidth < 2 )
        {
            SX1272Write( REG_LR_DETECTOPTIMIZE, SX1272Read( REG_LR_DETECTOPTIMIZE ) & 0x7F );
        }
        else
        {
            SX1272Write( REG_LR_DETECTOPTIMIZE, SX1272Read( REG_LR_DETECTOPTIMIZE ) | 0x80 );
        }

        SX1272Write( REG_LR_PLLHOP, ( SX1272Read( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_OFF );
        SX1272Write( REG_LR_HOPPERIOD, 0 );

        SX1272Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                     RFLR_IRQFLAGS_RXDONE |
                     RFLR_IRQFLAGS_PAYLOADCRCERROR |
                     RFLR_IRQFLAGS_VALIDHEADER |
                     RFLR_IRQFLAGS_TXDONE |
                     //RFLR_IRQFLAGS_CADDONE |
                     RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                     RFLR_IRQFLAGS_CADDETECTED
            );
        
        // DIO0=CadDone
        // DIO1=
        // DIO2=
        // DIO3=
        SX1272Write( REG_DIOMAPPING1, RFLR_DIOMAPPING1_DIO0_10 | RFLR_DIOMAPPING1_DIO1_11 | RFLR_DIOMAPPING1_DIO2_11 | RFLR_DIOMAPPING1_DIO1_11 );
        
        SX1272Sequence( RF_CAD );
    }
}

void SX1272SetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time )
{
    uint32_t timeout = ( uint32_t )( time * 1000 );

    SX1272SetStby( );

    SX1272SetChannel( freq );

    SX1272SetTxConfig( MODEM_FSK, power, 0, 0, 4800, 0, 5, false, false, 0, 0, 0, timeout );

    SX1272Write( REG_PACKETCONFIG2, ( SX1272Read( REG_PACKETCONFIG2 ) & RF_PACKETCONFIG2_DATAMODE_MASK ) );

    SX1272.State = RF_TX_RUNNING;

    SX1272SetOpMode( RF_OPMODE_TRANSMITTER );

    if( timeout )
    {
        stm32l0_lptim_start((timeout * 32768 + 999) / 1000, SX1272LptimCallback, SX1272OnTxTimeoutIrq);
    }

    SX1272Release( );
}

int16_t SX1272ReadRssi( void )
{
    int16_t rssi = 0;

    if( SX1272.Modem == MODEM_FSK )
    {
        rssi = -( SX1272Read( REG_RSSIVALUE ) >> 1 );
    }
    else
    {
        rssi = RSSI_OFFSET + SX1272Read( REG_LR_RSSIVALUE );
    }
    return rssi;
}

static void SX1272SetOpMode( uint8_t opMode )
{
    if( SX1272.OpMode != opMode )
    {
        SX1272SetAntSw( opMode );

        SX1272Write( REG_OPMODE, ( SX1272Read( REG_OPMODE ) & RF_OPMODE_MASK ) | opMode );
        
        SX1272.OpMode = opMode;
    }
}

void SX1272SetModem( RadioModems_t modem )
{
    if( SX1272.Modem == modem )
    {
        return;
    }

    SX1272.Modem = modem;

    SX1272SetSleep( );

    if( SX1272.Modem == MODEM_FSK )
    {
        SX1272Write( REG_OPMODE, ( ( SX1272Read( REG_OPMODE ) & RF_OPMODE_LONGRANGEMODE_MASK & RF_OPMODE_MODULATIONTYPE_MASK & RF_OPMODE_MODULATIONSHAPING_MASK ) |
                                   ( RF_OPMODE_LONGRANGEMODE_OFF | ( SX1272.Settings.Fsk.Modulation << 3 ) ) ) );
        SX1272WriteBuffer( REG_SYNCVALUE1, SX1272.Settings.Fsk.SyncWord, SX1272.Settings.Fsk.SyncSize );
        SX1272Write( REG_NODEADRS, SX1272.Settings.Fsk.NodeAddress );
        SX1272Write( REG_BROADCASTADRS, SX1272.Settings.Fsk.BroadcastAddress );
        SX1272Write( REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_10 | RF_DIOMAPPING1_DIO1_11 | RF_DIOMAPPING1_DIO2_01 | RF_DIOMAPPING1_DIO3_01 ); // DIO0=, DIO1=, DIO2=, DIO3=
        SX1272Write( REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_11 | RF_DIOMAPPING2_DIO5_11 | RF_DIOMAPPING2_MAP_PREAMBLEDETECT); // DIO4=PreambleDetected, DIO5=ModeReady
    }
    else
    {
        SX1272Write( REG_OPMODE, ( SX1272Read( REG_OPMODE ) & RF_OPMODE_LONGRANGEMODE_MASK ) | RF_OPMODE_LONGRANGEMODE_ON );
        SX1272Write( REG_LR_SYNCWORD, SX1272.Settings.LoRa.SyncWord );
        SX1272Write( REG_LR_PAYLOADMAXLENGTH, SX1272.Settings.LoRa.MaxPayloadLen );
        SX1272Write( REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_11 | RF_DIOMAPPING1_DIO1_11 | RF_DIOMAPPING1_DIO2_11 | RF_DIOMAPPING1_DIO3_11 ); // DIO0=, DIO1=, DIO2=, DIO3=
        SX1272Write( REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_11 | RF_DIOMAPPING2_DIO5_11 ); // DIO4=, DIO5=
    }

    SX1272Release( );
}

void SX1272Delay( uint32_t timeout )
{
    stm32l0_lptim_start((timeout * 32768 + 999) / 1000, NULL, NULL);

    while (!stm32l0_lptim_done());
    {
        __WFE();
    }
}

static void SX1272WriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1272WriteBuffer( 0, buffer, size );
}

static void SX1272ReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1272ReadBuffer( 0, buffer, size );
}

void SX1272SetMaxPayloadLength( RadioModems_t modem, uint8_t max )
{
    if( modem == MODEM_FSK )
    {
        SX1272.Settings.Fsk.MaxPayloadLen = max;
    }
    else
    {
        SX1272.Settings.LoRa.MaxPayloadLen = max;

        if( SX1272.Modem == MODEM_LORA )
        {
            SX1272Write( REG_LR_PAYLOADMAXLENGTH, SX1272.Settings.LoRa.MaxPayloadLen );
            SX1272Release( );
        }
    }
}

void SX1272SetPublicNetwork( bool enable )
{
    SX1272.Settings.LoRa.SyncWord = enable ? LORA_MAC_PUBLIC_SYNCWORD : LORA_MAC_PRIVATE_SYNCWORD;

    if( SX1272.Modem == MODEM_LORA )
    {
        SX1272Write( REG_LR_SYNCWORD, SX1272.Settings.LoRa.SyncWord );
        SX1272Release( );
    }
}

void SX1272SetModulation( uint8_t modulation )
{
    SX1272.Settings.Fsk.Modulation = modulation;

    if( SX1272.Modem == MODEM_FSK )
    {
        SX1272Write( REG_OPMODE, ( ( SX1272Read( REG_OPMODE ) & RF_OPMODE_MODULATIONTYPE_MASK & RF_OPMODE_MODULATIONSHAPING_MASK) | ( SX1272.Settings.Fsk.Modulation << 3 ) ) );

        SX1272Release( );
    }
}

void SX1272SetPreambleInverted( bool enable )
{
    SX1272.Settings.Fsk.PreambleInverted = enable;
}

void SX1272SetSyncWord( const uint8_t *data, uint8_t size )
{
    SX1272.Settings.Fsk.SyncSize = size;
    memcpy( SX1272.Settings.Fsk.SyncWord, data, size );

    if( SX1272.Modem == MODEM_FSK )
    {
        SX1272WriteBuffer( REG_SYNCVALUE1, SX1272.Settings.Fsk.SyncWord, SX1272.Settings.Fsk.SyncSize );
        SX1272Release( );
    }
}

void SX1272SetAfc( bool enable )
{
    SX1272.Settings.Fsk.AfcOn = enable;
}

void SX1272SetDcFree( uint8_t dcFree )
{
    SX1272.Settings.Fsk.DcFree = dcFree;
}

void SX1272SetCrcType( uint8_t crcType )
{
    SX1272.Settings.Fsk.CrcType = crcType;
}

void SX1272SetAddressFiltering( uint8_t addressFiltering )
{
    SX1272.Settings.Fsk.AddressFiltering = addressFiltering;
}

void SX1272SetNodeAddress( uint8_t address )
{
    SX1272.Settings.Fsk.NodeAddress = address;

    if( SX1272.Modem == MODEM_FSK )
    {
        SX1272Write( REG_NODEADRS, SX1272.Settings.Fsk.NodeAddress );
        SX1272Release( );
    }
}

void SX1272SetBroadcastAddress( uint8_t address )
{
    SX1272.Settings.Fsk.BroadcastAddress = address;

    if( SX1272.Modem == MODEM_FSK )
    {
        SX1272Write( REG_BROADCASTADRS, SX1272.Settings.Fsk.BroadcastAddress );
        SX1272Release( );
    }
}

void SX1272SetLnaBoost( bool enable )
{
    SX1272Write( REG_LNA, ( RF_LNA_GAIN_G1 | ( enable ? RF_LNA_BOOST_ON : RF_LNA_BOOST_OFF ) ) );
    SX1272Release( );
}

void SX1272SetIdleMode( uint8_t mode )
{
    SX1272.Settings.IdleMode = mode;
}

uint32_t SX1272GetWakeupTime( void )
{
    return SX1272GetBoardTcxoWakeupTime( ) + RADIO_WAKEUP_TIME;
}

static void SX1272LptimCallback( void *context )
{
    armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)context, NULL, 0);
}

static void SX1272Sequence( RadioState_t state )
{
    uint32_t tcxoTimeout;

    stm32l0_lptim_stop();

    SX1272.State = state;

    if( !SX1272.TcxoOn )
    {
        SX1272SetBoardTcxo( true );

        tcxoTimeout = SX1272GetBoardTcxoWakeupTime( );

        if( tcxoTimeout )
        {
            stm32l0_lptim_start((tcxoTimeout * 32768 + 999) / 1000, SX1272LptimCallback, SX1272OnTcxoTimeoutIrq);

            SX1272Release( );
            
            return;
        }
    }

    SX1272OnTcxoTimeoutIrq( );
}

static void SX1272OnTcxoTimeoutIrq( void )
{
    SX1272.TcxoOn = true;

    SX1272SetOpMode( RF_OPMODE_STANDBY );

    if( !SX1272.OscOn )
    {
        stm32l0_lptim_start((RADIO_WAKEUP_TIME * 32768 + 999) / 1000, SX1272LptimCallback, SX1272OnOscTimeoutIrq);

        SX1272Release( );

        return;
    }

    SX1272OnOscTimeoutIrq( );
}

static void SX1272OnOscTimeoutIrq( void )
{
    uint32_t chunkSize;

    SX1272.OscOn = true;
    
    if( !SX1272.AntSwOn )
    {
        SX1272AntSwInit( );

        SX1272.AntSwOn = true;
    }

    if ( !SX1272.DioOn )
    {
        SX1272DioInit( );

        SX1272.DioOn = true;
    }

    if( SX1272.Modem == MODEM_FSK )
    {
        switch( SX1272.State ) {
        case RF_IDLE:
            break;
        case RF_RX_RUNNING:
            SX1272SetOpMode( RF_OPMODE_RECEIVER );

            if( SX1272.Settings.LoRa.RxContinuous == true )
            {
                if( SX1272.Settings.RxTimeout )
                {
                    stm32l0_lptim_start(SX1272.Settings.RxTimeout, SX1272LptimCallback, SX1272OnRxTimeoutIrq);
                }
            }
            else
            {
                if( SX1272.Settings.Fsk.RxSingleTimeout )
                {
                    stm32l0_lptim_start(SX1272.Settings.Fsk.RxSingleTimeout, SX1272LptimCallback, SX1272OnRxSingleTimeoutIrq);
                }
                else
                {
                    if( SX1272.Settings.RxTimeout )
                    {
                        stm32l0_lptim_start(SX1272.Settings.RxTimeout, SX1272LptimCallback, SX1272OnRxTimeoutIrq);
                    }
                }
            }
            break;
        case RF_TX_RUNNING:
            chunkSize = 64;

            if( SX1272.Settings.Fsk.FixLen == false )
            {
                SX1272WriteFifo( ( uint8_t* )&SX1272.PacketHandler.Fsk.Size, 1 );

                chunkSize--;
            }

            if( SX1272.PacketHandler.Fsk.Size != SX1272.PacketHandler.Fsk.NbBytes )
            {
                if( chunkSize > SX1272.PacketHandler.Fsk.Size )
                {
                    chunkSize = SX1272.PacketHandler.Fsk.Size;
                }

                // Write payload buffer
                SX1272WriteFifo( RxTxBuffer, chunkSize );
                SX1272.PacketHandler.Fsk.NbBytes += chunkSize;
            }

            SX1272SetOpMode( RF_OPMODE_TRANSMITTER );
            
            if( SX1272.Settings.Fsk.TxTimeout )
            {
                stm32l0_lptim_start(SX1272.Settings.Fsk.TxTimeout, SX1272LptimCallback, SX1272OnTxTimeoutIrq);
            }
            break;
        case RF_CAD:
            break;
        }
    }
    else
    {
        switch( SX1272.State ) {
        case RF_IDLE:
            break;
        case RF_RX_RUNNING:
            if( SX1272.Settings.LoRa.RxContinuous == true )
            {
                SX1272SetOpMode( RFLR_OPMODE_RECEIVER );
            }
            else
            {
                SX1272SetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
            }
            
            if( SX1272.Settings.RxTimeout )
            {
                stm32l0_lptim_start(SX1272.Settings.RxTimeout, SX1272LptimCallback, SX1272OnRxTimeoutIrq);
            }
            break;
        case RF_TX_RUNNING:
            // Write payload buffer
            SX1272WriteFifo( RxTxBuffer, SX1272.PacketHandler.LoRa.Size );
            
            SX1272SetOpMode( RFLR_OPMODE_TRANSMITTER );
            
            if( SX1272.Settings.LoRa.TxTimeout )
            {
                stm32l0_lptim_start(SX1272.Settings.LoRa.TxTimeout, SX1272LptimCallback, SX1272OnTxTimeoutIrq);
            }
            break;
        case RF_CAD:
            SX1272SetOpMode( RFLR_OPMODE_CAD );
            break;
        }
    }

    SX1272Release( );
}

static void SX1272OnRxSingleTimeoutIrq( void )
{
    if ( SX1272.PacketHandler.Fsk.SyncWordDetected == false)
    {
        SX1272SetIdle( );

        if( ( SX1272.Events != NULL ) && ( SX1272.Events->RxTimeout != NULL ) )
        {
            SX1272.Events->RxTimeout( );
        }
    }
}

static void SX1272OnRxTimeoutIrq( void )
{
    SX1272SetIdle( );
    
    if( ( SX1272.Events != NULL ) && ( SX1272.Events->RxTimeout != NULL ) )
    {
        SX1272.Events->RxTimeout( );
    }
}

static void SX1272OnTxDoneTimeoutIrq( void )
{
    SX1272SetIdle( );
            
    if( ( SX1272.Events != NULL ) && ( SX1272.Events->TxDone != NULL ) )
    {
        SX1272.Events->TxDone( );
    }
}

static void SX1272OnTxTimeoutIrq( void )
{
    SX1272SetIdle( );

    if( ( SX1272.Events != NULL ) && ( SX1272.Events->TxTimeout != NULL ) )
    {
        SX1272.Events->TxTimeout( );
    }
}

void SX1272OnDio0Irq( void )
{
    bool crcOk, cadDetected;
    int8_t snr;
    int16_t rssi;

    if( SX1272.Modem == MODEM_FSK )
    {
        switch( SX1272.State ) {
        case RF_IDLE:
            break;
        case RF_RX_RUNNING:
            // PayloadReady interrupt
            crcOk = ( SX1272Read( REG_IRQFLAGS2 ) & RF_IRQFLAGS2_CRCOK ) == RF_IRQFLAGS2_CRCOK;
            
            if( ( SX1272.Settings.Fsk.CrcOn == false ) || ( crcOk == true ) )
            {
                // Read received packet size
                if( SX1272.PacketHandler.Fsk.PacketSizeReceived == false )
                {
                    SX1272ReadFifo( ( uint8_t* )&SX1272.PacketHandler.Fsk.Size, 1 );

                    SX1272.PacketHandler.Fsk.PacketSizeReceived = true;
                }
                
                // Read the remaining packet data
                if( SX1272.PacketHandler.Fsk.Size != SX1272.PacketHandler.Fsk.NbBytes )
                {
                    SX1272ReadFifo( RxTxBuffer + SX1272.PacketHandler.Fsk.NbBytes, SX1272.PacketHandler.Fsk.Size - SX1272.PacketHandler.Fsk.NbBytes );
                    SX1272.PacketHandler.Fsk.NbBytes += ( SX1272.PacketHandler.Fsk.Size - SX1272.PacketHandler.Fsk.NbBytes );
                }
            }

            // Clear Irq
            SX1272Write( REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN );
            
            if( SX1272.Settings.Fsk.RxContinuous == false )
            {   
                SX1272SetIdle( );
            }
            else
            {
                SX1272Release( );
            }

            if( ( SX1272.Settings.Fsk.CrcOn == true ) && ( crcOk == false ) )
            {
                if( ( SX1272.Events != NULL ) && ( SX1272.Events->RxError != NULL ) )
                {
                    SX1272.Events->RxError( );
                }
            }
            else
            {
                if( ( SX1272.Events != NULL ) && ( SX1272.Events->RxDone != NULL ) )
                {
                    SX1272.Events->RxDone( RxTxBuffer, SX1272.PacketHandler.Fsk.Size, SX1272.PacketHandler.Fsk.Rssi, 0 );
                }
            }
            break;
        case RF_TX_RUNNING:
            // PayloadSent interrupt
            stm32l0_lptim_start(SX1272.Settings.Fsk.TxDoneTimeout, SX1272LptimCallback, SX1272OnTxDoneTimeoutIrq);
            break;
        case RF_CAD:
            break;
        }
    }
    else
    {
        switch( SX1272.State ) {
        case RF_IDLE:
            break;
        case RF_RX_RUNNING:
            // RxDone interrupt
            crcOk = ( SX1272Read( REG_LR_IRQFLAGS ) & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) != RFLR_IRQFLAGS_PAYLOADCRCERROR;
            
            if( ( SX1272.Settings.LoRa.CrcOn == false ) || ( crcOk == true ) )
            {
                SX1272.PacketHandler.LoRa.Size = SX1272Read( REG_LR_RXNBBYTES );
                SX1272Write( REG_LR_FIFOADDRPTR, SX1272Read( REG_LR_FIFORXCURRENTADDR ) );
                SX1272ReadFifo( RxTxBuffer, SX1272.PacketHandler.LoRa.Size );

                snr = (int8_t)SX1272Read( REG_LR_PKTSNRVALUE );
                rssi = SX1272Read( REG_LR_PKTRSSIVALUE );

                if( snr < 0 )
                {
                    SX1272.PacketHandler.LoRa.Snr = - ( ( -snr + 2 ) / 4 );
                    SX1272.PacketHandler.LoRa.Rssi = RSSI_OFFSET + rssi + ( rssi >> 4 ) + SX1272.PacketHandler.LoRa.Snr;
                }
                else 
                {
                    SX1272.PacketHandler.LoRa.Snr = ( snr + 2 ) / 4;
                    SX1272.PacketHandler.LoRa.Rssi = RSSI_OFFSET + rssi + ( rssi >> 4 );
                }
            }
            
            // Clear Irq
            SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE | RFLR_IRQFLAGS_VALIDHEADER | RFLR_IRQFLAGS_PAYLOADCRCERROR );
            
            if( SX1272.Settings.LoRa.RxContinuous == false )
            {
                SX1272SetIdle( );
            }
            else
            {
                SX1272Release( );
            }
            
            if( ( SX1272.Settings.LoRa.CrcOn == true ) && ( crcOk == false ) )
            {
                if( ( SX1272.Events != NULL ) && ( SX1272.Events->RxError != NULL ) )
                {
                    SX1272.Events->RxError( );
                }
            }
            else
            {
                if( ( SX1272.Events != NULL ) && ( SX1272.Events->RxDone != NULL ) )
                {
                    SX1272.Events->RxDone( RxTxBuffer, SX1272.PacketHandler.LoRa.Size, SX1272.PacketHandler.LoRa.Rssi, SX1272.PacketHandler.LoRa.Snr );
                }
            }
            break;
        case RF_TX_RUNNING:
            // TxDone interrupt
            SX1272SetIdle( );
            
            if( ( SX1272.Events != NULL ) && ( SX1272.Events->TxDone != NULL ) )
            {
                SX1272.Events->TxDone( );
            }
            break;
        case RF_CAD:
            // CadDone interrupt
            cadDetected = ( SX1272Read( REG_LR_IRQFLAGS ) & RFLR_IRQFLAGS_CADDETECTED ) == RFLR_IRQFLAGS_CADDETECTED;

            SX1272SetIdle( );
            
            if( ( SX1272.Events != NULL ) && ( SX1272.Events->CadDone != NULL ) )
            {
                SX1272.Events->CadDone( cadDetected );
            }
            break;
        }
    }
}

void SX1272OnDio1Irq( void )
{
    int32_t chunkSize;

    if( SX1272.Modem == MODEM_FSK )
    {
        switch( SX1272.State ) {
        case RF_IDLE:
            break;
        case RF_RX_RUNNING:
            // FifoLevel interrupt
            chunkSize = SX1272.PacketHandler.Fsk.ChunkSize;
            
            // Read received packet size
            if( SX1272.PacketHandler.Fsk.PacketSizeReceived == false )
            {
                SX1272ReadFifo( ( uint8_t* )&SX1272.PacketHandler.Fsk.Size, 1 );
                
                SX1272.PacketHandler.Fsk.PacketSizeReceived = true;
        
                chunkSize -= 1;
            }

            if( SX1272.PacketHandler.Fsk.Size != SX1272.PacketHandler.Fsk.NbBytes )
            {
                if( chunkSize > ( SX1272.PacketHandler.Fsk.Size - SX1272.PacketHandler.Fsk.NbBytes ) )
                {
                    chunkSize = ( SX1272.PacketHandler.Fsk.Size - SX1272.PacketHandler.Fsk.NbBytes );
                }

                SX1272ReadFifo( ( RxTxBuffer + SX1272.PacketHandler.Fsk.NbBytes ), chunkSize );
                SX1272.PacketHandler.Fsk.NbBytes += chunkSize;
            }
            SX1272Release( );
            break;
        case RF_TX_RUNNING:
            // FifoLevel interrupt
            if( SX1272.PacketHandler.Fsk.Size != SX1272.PacketHandler.Fsk.NbBytes )
            {
                chunkSize = SX1272.PacketHandler.Fsk.ChunkSize;

                if( chunkSize > ( SX1272.PacketHandler.Fsk.Size - SX1272.PacketHandler.Fsk.NbBytes ) )
                {
                    chunkSize = ( SX1272.PacketHandler.Fsk.Size - SX1272.PacketHandler.Fsk.NbBytes );
                }
                
                SX1272WriteFifo( ( RxTxBuffer + SX1272.PacketHandler.Fsk.NbBytes ), chunkSize );
                SX1272.PacketHandler.Fsk.NbBytes += chunkSize;
            }
            SX1272Release( );
            break;
        case RF_CAD:
            break;
        }
    }
    else
    {
        switch( SX1272.State ) {
        case RF_IDLE:
            break;
        case RF_RX_RUNNING:
            // Sync time out
            SX1272SetIdle( );
            
            if( ( SX1272.Events != NULL ) && ( SX1272.Events->RxTimeout != NULL ) )
            {
                SX1272.Events->RxTimeout( );
            }
            break;
        case RF_TX_RUNNING:
            break;
        case RF_CAD:
            break;
        }
    }
}

void SX1272OnDio2Irq( void )
{
    uint32_t rxSingleTimeout;

    if( SX1272.Modem == MODEM_FSK )
    {
        switch( SX1272.State ) {
        case RF_IDLE:
            break;
        case RF_RX_RUNNING:
            SX1272.PacketHandler.Fsk.SyncWordDetected = true;
            SX1272.PacketHandler.Fsk.Rssi = -( SX1272Read( REG_RSSIVALUE ) >> 1 );

            SX1272.PacketHandler.Fsk.NbBytes = 0;

            if( SX1272.Settings.Fsk.FixLen == false )
            {
                SX1272.PacketHandler.Fsk.PacketSizeReceived = false;
                SX1272.PacketHandler.Fsk.Size = 0;
            }

            if( SX1272.Settings.Fsk.RxContinuous == false )
            {
                if( SX1272.Settings.Fsk.RxSingleTimeout )
                {
                    rxSingleTimeout = stm32l0_lptim_stop();
                    
                    if( SX1272.Settings.RxTimeout )
                    {
                        if( SX1272.Settings.RxTimeout > rxSingleTimeout )
                        {
                            SX1272.Settings.RxTimeout -= rxSingleTimeout;
                            
                            stm32l0_lptim_start(SX1272.Settings.RxTimeout, SX1272LptimCallback, SX1272OnRxTimeoutIrq);
                        }
                        else
                        {
                            SX1272SetIdle( );
                            
                            if( ( SX1272.Events != NULL ) && ( SX1272.Events->RxTimeout != NULL ) )
                            {
                                SX1272.Events->RxTimeout( );
                            }
                        }
                    }
                }
            }
            SX1272Release( );
            break;
        case RF_TX_RUNNING:
            break;
        case RF_CAD:
            break;
        }
    }
    else
    {
        switch( SX1272.State ) {
        case RF_IDLE:
            break;
        case RF_RX_RUNNING:
        case RF_TX_RUNNING:
            if( SX1272.Settings.LoRa.FreqHopOn == true )
            {
                // Clear Irq
                SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );
                
                if( ( SX1272.Events != NULL ) && ( SX1272.Events->FhssChangeChannel != NULL ) )
                {
                    SX1272.Events->FhssChangeChannel( ( SX1272Read( REG_LR_HOPCHANNEL ) & RFLR_HOPCHANNEL_CHANNEL_MASK ) );
                }

                SX1272Release( );
            }
            break;
        case RF_CAD:
            break;
        }
    }
}
