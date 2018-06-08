/*!
 * \file      RegionUS915.c
 *
 * \brief     Region implementation for US915
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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "radio.h"
#include "timer.h"
#include "utilities.h"

#include "LoRaMac.h"
#include "Region.h"
#include "RegionCommon.h"
#include "RegionUS915.h"

// Definitions
#define CHANNELS_MASK_SIZE              5

// Global attributes
/*!
 * LoRaMAC channels
 */
static const ChannelParams_t RegionUS915Channels[US915_MAX_NB_CHANNELS] = {
    { 902300000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 902500000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 902700000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 902900000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 903100000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 903300000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 903500000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 903700000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 903900000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 904100000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 904300000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 904500000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 904700000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 904900000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 905100000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 905300000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 905500000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 905700000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 905900000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 906100000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 906300000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 906500000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 906700000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 906900000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 907100000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 907300000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 907500000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 907700000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 907900000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 908100000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 908300000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 908500000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 908700000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 908900000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 909100000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 909300000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 909500000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 909700000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 909900000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 910100000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 910300000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 910500000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 910700000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 910900000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 911100000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 911300000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 911500000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 911700000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 911900000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 912100000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 912300000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 912500000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 912700000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 912900000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 913100000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 913300000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 913500000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 913700000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 913900000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 914100000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 914300000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 914500000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 914700000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 914900000, 0, { ( DR_3 << 4 ) | DR_0 }, 0 },
    { 903000000, 0, { ( DR_4 << 4 ) | DR_4 }, 0 },
    { 904600000, 0, { ( DR_4 << 4 ) | DR_4 }, 0 },
    { 906200000, 0, { ( DR_4 << 4 ) | DR_4 }, 0 },
    { 907800000, 0, { ( DR_4 << 4 ) | DR_4 }, 0 },
    { 909400000, 0, { ( DR_4 << 4 ) | DR_4 }, 0 },
    { 911000000, 0, { ( DR_4 << 4 ) | DR_4 }, 0 },
    { 912600000, 0, { ( DR_4 << 4 ) | DR_4 }, 0 },
    { 914200000, 0, { ( DR_4 << 4 ) | DR_4 }, 0 },
};

/*!
 * LoRaMac bands
 */
extern Band_t RegionBands[US915_MAX_NB_BANDS];

/*!
 * LoRaMac channels mask
 */
extern uint16_t RegionChannelsMask[CHANNELS_MASK_SIZE];

/*!
 * LoRaMac channels remaining
 */
extern uint16_t RegionChannelsMaskRemaining[CHANNELS_MASK_SIZE];

/*!
 * LoRaMac channels default mask
 */
extern uint16_t RegionChannelsDefaultMask[CHANNELS_MASK_SIZE];

static const uint8_t RegionUS915Priorities[US915_MAX_NB_CHANNELS] = {
    0, 18, 36, 54,  9, 27, 45, 63, 
    1, 19, 37, 55, 10, 28, 46, 64, 
    2, 20, 38, 56, 11, 29, 47, 65, 
    3, 21, 39, 57, 12, 30, 48, 66, 
    4, 22, 40, 58, 13, 31, 49, 67, 
    5, 23, 41, 59, 14, 32, 50, 68, 
    6, 24, 42, 60, 15, 33, 51, 69, 
    7, 25, 43, 61, 16, 34, 52, 70, 
    8, 17, 26, 35, 44, 53, 62, 71, 
};

static int8_t lastJoinChannel = -1;

// Static functions
static int8_t GetNextLowerTxDr( int8_t dr, int8_t minDr )
{
    uint8_t nextLowerDr = 0;

    if( dr == minDr )
    {
        nextLowerDr = minDr;
    }
    else
    {
        nextLowerDr = dr - 1;
    }
    return nextLowerDr;
}

static uint32_t GetBandwidth( uint32_t drIndex )
{
    switch( BandwidthsUS915[drIndex] )
    {
        default:
        case 125000:
            return 0;
        case 250000:
            return 1;
        case 500000:
            return 2;
    }
}

static int8_t LimitTxPower( int8_t txPower, int8_t maxBandTxPower, int8_t datarate, uint16_t* channelsMask )
{
    int8_t txPowerResult = txPower;

    // Limit tx power to the band max
    txPowerResult =  MAX( txPower, maxBandTxPower );

    if( datarate == DR_4 )
    {// Limit tx power to max 26dBm
        txPowerResult = MAX( txPower, TX_POWER_2 );
    }
    else
    {
        if( RegionCommonCountChannels( channelsMask, 0, 4 ) < 50 )
        {// Limit tx power to max 21dBm
            txPowerResult = MAX( txPower, TX_POWER_5 );
        }
    }
    return txPowerResult;
}

static uint8_t CountNbOfEnabledChannels( uint8_t datarate, uint16_t* channelsMask, const ChannelParams_t* channels, Band_t* bands, uint8_t* enabledChannels, uint8_t* delayTx )
{
    uint8_t nbEnabledChannels = 0;
    uint8_t delayTransmission = 0;

    for( uint8_t i = 0, k = 0; i < US915_MAX_NB_CHANNELS; i += 16, k++ )
    {
        for( uint8_t j = 0; j < 16; j++ )
        {
            if( ( channelsMask[k] & ( 1 << j ) ) != 0 )
            {
                if( channels[i + j].Frequency == 0 )
                { // Check if the channel is enabled
                    continue;
                }
                if( RegionCommonValueInRange( datarate, channels[i + j].DrRange.Fields.Min,
                                              channels[i + j].DrRange.Fields.Max ) == false )
                { // Check if the current channel selection supports the given datarate
                    continue;
                }
                if( bands[channels[i + j].Band].TimeOff > 0 )
                { // Check if the band is available for transmission
                    delayTransmission++;
                    continue;
                }
                enabledChannels[nbEnabledChannels++] = i + j;
            }
        }
    }

    *delayTx = delayTransmission;
    return nbEnabledChannels;
}

PhyParam_t RegionUS915GetPhyParam( GetPhyParams_t* getPhy )
{
    PhyParam_t phyParam = { 0 };

    switch( getPhy->Attribute )
    {
        case PHY_MIN_RX_DR:
        {
            phyParam.Value = US915_RX_MIN_DATARATE;
            break;
        }
        case PHY_MIN_TX_DR:
        {
            phyParam.Value = US915_TX_MIN_DATARATE;
            break;
        }
        case PHY_DEF_TX_DR:
        {
            phyParam.Value = US915_DEFAULT_DATARATE;
            break;
        }
        case PHY_NEXT_LOWER_TX_DR:
        {
            phyParam.Value = GetNextLowerTxDr( getPhy->Datarate, US915_TX_MIN_DATARATE );
            break;
        }
        case PHY_DEF_TX_POWER:
        {
            phyParam.Value = US915_DEFAULT_TX_POWER;
            break;
        }
        case PHY_MAX_PAYLOAD:
        {
            phyParam.Value = MaxPayloadOfDatarateUS915[getPhy->Datarate];
            break;
        }
        case PHY_MAX_PAYLOAD_REPEATER:
        {
            phyParam.Value = MaxPayloadOfDatarateRepeaterUS915[getPhy->Datarate];
            break;
        }
        case PHY_DUTY_CYCLE:
        {
            phyParam.Value = US915_DUTY_CYCLE_ENABLED;
            break;
        }
        case PHY_MAX_RX_WINDOW:
        {
            phyParam.Value = US915_MAX_RX_WINDOW;
            break;
        }
        case PHY_RECEIVE_DELAY1:
        {
            phyParam.Value = US915_RECEIVE_DELAY1;
            break;
        }
        case PHY_RECEIVE_DELAY2:
        {
            phyParam.Value = US915_RECEIVE_DELAY2;
            break;
        }
        case PHY_JOIN_ACCEPT_DELAY1:
        {
            phyParam.Value = US915_JOIN_ACCEPT_DELAY1;
            break;
        }
        case PHY_JOIN_ACCEPT_DELAY2:
        {
            phyParam.Value = US915_JOIN_ACCEPT_DELAY2;
            break;
        }
        case PHY_MAX_FCNT_GAP:
        {
            phyParam.Value = US915_MAX_FCNT_GAP;
            break;
        }
        case PHY_ACK_TIMEOUT:
        {
            phyParam.Value = ( US915_ACKTIMEOUT + randr( -US915_ACK_TIMEOUT_RND, US915_ACK_TIMEOUT_RND ) );
            break;
        }
        case PHY_DEF_DR1_OFFSET:
        {
            phyParam.Value = US915_DEFAULT_RX1_DR_OFFSET;
            break;
        }
        case PHY_DEF_RX2_FREQUENCY:
        {
            phyParam.Value = US915_RX_WND_2_FREQ;
            break;
        }
        case PHY_DEF_RX2_DR:
        {
            phyParam.Value = US915_RX_WND_2_DR;
            break;
        }
        case PHY_CHANNELS_MASK:
        {
            phyParam.ChannelsMask = RegionChannelsMask;
            break;
        }
        case PHY_CHANNELS_DEFAULT_MASK:
        {
            phyParam.ChannelsMask = RegionChannelsDefaultMask;
            break;
        }
        case PHY_MAX_NB_CHANNELS:
        {
            phyParam.Value = US915_MAX_NB_CHANNELS;
            break;
        }
        case PHY_CHANNELS:
        {
            phyParam.Channels = RegionUS915Channels;
            break;
        }
        case PHY_DEF_UPLINK_DWELL_TIME:
        case PHY_DEF_DOWNLINK_DWELL_TIME:
        {
            phyParam.Value = 0;
            break;
        }
        case PHY_DEF_MAX_EIRP:
        case PHY_DEF_ANTENNA_GAIN:
        {
            phyParam.fValue = 0;
            break;
        }
        case PHY_RX_CALIBRATION_FREQUENCY:
        {
            phyParam.Value = US915_RX_CAL_FREQ;
            break;
        }
        default:
        {
            break;
        }
    }

    return phyParam;
}

void RegionUS915SetBandTxDone( SetBandTxDoneParams_t* txDone )
{
    RegionCommonSetBandTxDone( txDone->Joined, &RegionBands[RegionUS915Channels[txDone->Channel].Band], txDone->LastTxDoneTime );
}

void RegionUS915InitDefaults( InitType_t type )
{
    switch( type )
    {
        case INIT_TYPE_INIT:
        {
#if 0
            // Channels
            // 125 kHz channels
            for( uint8_t i = 0; i < US915_MAX_NB_CHANNELS - 8; i++ )
            {
                RegionUS915Channels[i].Frequency = 902300000 + i * 200000;
                RegionUS915Channels[i].DrRange.Value = ( DR_3 << 4 ) | DR_0;
                RegionUS915Channels[i].Band = 0;
            }
            // 500 kHz channels
            for( uint8_t i = US915_MAX_NB_CHANNELS - 8; i < US915_MAX_NB_CHANNELS; i++ )
            {
                RegionUS915Channels[i].Frequency = 903000000 + ( i - ( US915_MAX_NB_CHANNELS - 8 ) ) * 1600000;
                RegionUS915Channels[i].DrRange.Value = ( DR_4 << 4 ) | DR_4;
                RegionUS915Channels[i].Band = 0;
            }
#endif

            // Bands
            RegionBands[0] = ( Band_t )US915_BAND0;

            // ChannelsMask
            RegionChannelsDefaultMask[0] = 0xFFFF;
            RegionChannelsDefaultMask[1] = 0xFFFF;
            RegionChannelsDefaultMask[2] = 0xFFFF;
            RegionChannelsDefaultMask[3] = 0xFFFF;
            RegionChannelsDefaultMask[4] = 0x00FF;

            // Copy channels default mask
            RegionCommonChanMaskCopy( RegionChannelsMask, RegionChannelsDefaultMask, 5 );

            // Copy into channels mask remaining
            RegionCommonChanMaskCopy( RegionChannelsMaskRemaining, RegionChannelsMask, 5 );
            break;
        }
        case INIT_TYPE_RESTORE:
        {
            // Copy channels default mask
            RegionCommonChanMaskCopy( RegionChannelsMask, RegionChannelsDefaultMask, 5 );

            for( uint8_t i = 0; i < 5; i++ )
            { // Copy-And the channels mask
                RegionChannelsMaskRemaining[i] &= RegionChannelsMask[i];
            }
            break;
        }
        case INIT_TYPE_APP_DEFAULTS:
        {
            // Copy channels default mask
            RegionCommonChanMaskCopy( RegionChannelsMask, RegionChannelsDefaultMask, 5 );

            // Copy into channels mask remaining
            RegionCommonChanMaskCopy( RegionChannelsMaskRemaining, RegionChannelsMask, 5 );
            break;
        }
        default:
        {
            break;
        }
    }
}

bool RegionUS915Verify( VerifyParams_t* verify, PhyAttribute_t phyAttribute )
{
    switch( phyAttribute )
    {
        case PHY_TX_DR:
        {
            return RegionCommonValueInRange( verify->DatarateParams.Datarate, US915_TX_MIN_DATARATE, US915_TX_MAX_DATARATE );
        }
        case PHY_DEF_TX_DR:
        {
            return RegionCommonValueInRange( verify->DatarateParams.Datarate, DR_0, DR_5 );
        }
        case PHY_RX_DR:
        {
            return RegionCommonValueInRange( verify->DatarateParams.Datarate, US915_RX_MIN_DATARATE, US915_RX_MAX_DATARATE );
        }
        case PHY_DEF_TX_POWER:
        case PHY_TX_POWER:
        {
            // Remark: switched min and max!
            return RegionCommonValueInRange( verify->TxPower, US915_MAX_TX_POWER, US915_MIN_TX_POWER );
        }
        case PHY_DUTY_CYCLE:
        {
            return US915_DUTY_CYCLE_ENABLED;
        }
        default:
            return false;
    }
}

void RegionUS915ApplyCFList( ApplyCFListParams_t* applyCFList )
{
    return;
}

bool RegionUS915ChanMaskSet( ChanMaskSetParams_t* chanMaskSet )
{
    uint8_t nbChannels = RegionCommonCountChannels( chanMaskSet->ChannelsMaskIn, 0, 4 );

    // Check the number of active channels
    if( ( nbChannels < 2 ) &&
        ( nbChannels > 0 ) )
    {
        return false;
    }

    switch( chanMaskSet->ChannelsMaskType )
    {
        case CHANNELS_MASK:
        {
            RegionCommonChanMaskCopy( RegionChannelsMask, chanMaskSet->ChannelsMaskIn, 5 );

            for( uint8_t i = 0; i < 5; i++ )
            { // Copy-And the channels mask
                RegionChannelsMaskRemaining[i] &= RegionChannelsMask[i];
            }
            break;
        }
        case CHANNELS_DEFAULT_MASK:
        {
            RegionCommonChanMaskCopy( RegionChannelsDefaultMask, chanMaskSet->ChannelsMaskIn, 5 );
            break;
        }
        default:
            return false;
    }
    return true;
}

bool RegionUS915AdrNext( AdrNextParams_t* adrNext, int8_t* drOut, int8_t* txPowOut, uint32_t* adrAckCounter )
{
    bool adrAckReq = false;
    int8_t datarate = adrNext->Datarate;
    int8_t txPower = adrNext->TxPower;
    GetPhyParams_t getPhy;
    PhyParam_t phyParam;

    // Report back the adr ack counter
    *adrAckCounter = adrNext->AdrAckCounter;

    adrAckReq = false;

    if( adrNext->AdrEnabled == true )
    {
        if( adrNext->AdrAckCounter < ( US915_ADR_ACK_LIMIT + 18 * US915_ADR_ACK_DELAY ) )
        {
            if( adrNext->AdrAckCounter >= US915_ADR_ACK_LIMIT )
            {
                adrAckReq = true;
            }
            
            if( adrNext->AdrAckCounter >= ( US915_ADR_ACK_LIMIT + US915_ADR_ACK_DELAY ) )
            {
                if( ( adrNext->AdrAckCounter % US915_ADR_ACK_DELAY ) == 1 )
                {
                    if( txPower != US915_MAX_TX_POWER )
                    {
                        // Increase the txPower
                        txPower = US915_MAX_TX_POWER;
                    }
                    else if( datarate != US915_TX_MIN_DATARATE )
                    {
                        // Decrease the datarate
                        getPhy.Attribute = PHY_NEXT_LOWER_TX_DR;
                        getPhy.Datarate = datarate;
                        getPhy.UplinkDwellTime = adrNext->UplinkDwellTime;
                        phyParam = RegionUS915GetPhyParam( &getPhy );
                        datarate = phyParam.Value;
                    }
                    else
                    {
                        *adrAckCounter = ( US915_ADR_ACK_LIMIT + 18 * US915_ADR_ACK_DELAY );

                        // We must set adrAckReq to false as soon as we reach the lowest datarate
                        adrAckReq = false;
                        if( adrNext->UpdateChanMask == true )
                        {
                            // Re-enable default channels
                            RegionChannelsMask[0] = RegionChannelsDefaultMask[0];
                            RegionChannelsMask[1] = RegionChannelsDefaultMask[1];
                            RegionChannelsMask[2] = RegionChannelsDefaultMask[2];
                            RegionChannelsMask[3] = RegionChannelsDefaultMask[3];
                            RegionChannelsMask[4] = RegionChannelsDefaultMask[4];
                        }
                    }
                }
            }
        }
    }

    *drOut = datarate;
    *txPowOut = txPower;
    return adrAckReq;
}

void RegionUS915ComputeRxWindowParameters( int8_t datarate, uint8_t minRxSymbols, uint32_t rxError, RxConfigParams_t *rxConfigParams )
{
    double tSymbol = 0.0;

    // Get the datarate, perform a boundary check
    rxConfigParams->Datarate = MIN( datarate, US915_RX_MAX_DATARATE );
    rxConfigParams->Bandwidth = GetBandwidth( rxConfigParams->Datarate );

    tSymbol = RegionCommonComputeSymbolTimeLoRa( DataratesUS915[rxConfigParams->Datarate], BandwidthsUS915[rxConfigParams->Datarate] );

    RegionCommonComputeRxWindowParameters( tSymbol, minRxSymbols, rxError, Radio.GetWakeupTime( ), &rxConfigParams->WindowTimeout, &rxConfigParams->WindowOffset );
}

bool RegionUS915RxConfig( RxConfigParams_t* rxConfig, int8_t* datarate )
{
    int8_t dr = rxConfig->Datarate;
    uint8_t maxPayload = 0;
    int8_t phyDr = 0;
    uint32_t frequency = rxConfig->Frequency;

    if( Radio.GetStatus( ) != RF_IDLE )
    {
        return false;
    }

    if( rxConfig->RxSlot == RX_SLOT_WIN_1 )
    {
        // Apply window 1 frequency
        frequency = US915_FIRST_RX1_CHANNEL + ( rxConfig->Channel % 8 ) * US915_STEPWIDTH_RX1_CHANNEL;
    }

    // Read the physical datarate from the datarates table
    phyDr = DataratesUS915[dr];

    Radio.SetChannel( frequency );

    // Radio configuration
    Radio.SetRxConfig( MODEM_LORA, rxConfig->Bandwidth, phyDr, 1, 0, 8, rxConfig->WindowTimeout, false, 0, false, 0, 0, true, rxConfig->RxContinuous );

    if( rxConfig->RepeaterSupport == true )
    {
        maxPayload = MaxPayloadOfDatarateRepeaterUS915[dr];
    }
    else
    {
        maxPayload = MaxPayloadOfDatarateUS915[dr];
    }
    Radio.SetMaxPayloadLength( MODEM_LORA, maxPayload + LORA_MAC_FRMPAYLOAD_OVERHEAD );

    *datarate = (uint8_t) dr;
    return true;
}

bool RegionUS915TxConfig( TxConfigParams_t* txConfig, int8_t* txPower, TimerTime_t* txTimeOnAir )
{
    int8_t phyDr = DataratesUS915[txConfig->Datarate];
    int8_t txPowerLimited = LimitTxPower( txConfig->TxPower, RegionBands[RegionUS915Channels[txConfig->Channel].Band].TxMaxPower, txConfig->Datarate, RegionChannelsMask );
    uint32_t bandwidth = GetBandwidth( txConfig->Datarate );
    int8_t phyTxPower = 0;

    // Calculate physical TX power
    phyTxPower = RegionCommonComputeTxPower( txPowerLimited, US915_DEFAULT_MAX_ERP, 0 );

    // Setup the radio frequency
    Radio.SetChannel( RegionUS915Channels[txConfig->Channel].Frequency );

    Radio.SetTxConfig( MODEM_LORA, phyTxPower, 0, bandwidth, phyDr, 1, 8, false, true, 0, 0, false, 3000 );

    // Setup maximum payload lenght of the radio driver
    Radio.SetMaxPayloadLength( MODEM_LORA, txConfig->PktLen );
    // Get the time-on-air of the next tx frame
    *txTimeOnAir = Radio.TimeOnAir( MODEM_LORA, txConfig->PktLen );
    *txPower = txPowerLimited;

    return true;
}

uint8_t RegionUS915LinkAdrReq( LinkAdrReqParams_t* linkAdrReq, int8_t* drOut, int8_t* txPowOut, uint8_t* nbRepOut, uint8_t* nbBytesParsed )
{
    uint8_t status = 0x07;
    RegionCommonLinkAdrParams_t linkAdrParams;
    uint8_t nextIndex = 0;
    uint8_t bytesProcessed = 0;
    uint16_t channelsMask[5] = { 0, 0, 0, 0, 0 };
    GetPhyParams_t getPhy;
    PhyParam_t phyParam;
    RegionCommonLinkAdrReqVerifyParams_t linkAdrVerifyParams;

    // Initialize local copy of channels mask
    RegionCommonChanMaskCopy( channelsMask, RegionChannelsMask, 5 );

    while( bytesProcessed < linkAdrReq->PayloadSize )
    {
        nextIndex = RegionCommonParseLinkAdrReq( &( linkAdrReq->Payload[bytesProcessed] ), &linkAdrParams );

        if( nextIndex == 0 )
            break; // break loop, since no more request has been found

        // Update bytes processed
        bytesProcessed += nextIndex;

        // Revert status, as we only check the last ADR request for the channel mask KO
        status = 0x07;

        if( linkAdrParams.ChMaskCtrl == 6 )
        {
            // Enable all 125 kHz channels
            channelsMask[0] = 0xFFFF;
            channelsMask[1] = 0xFFFF;
            channelsMask[2] = 0xFFFF;
            channelsMask[3] = 0xFFFF;
            // Apply chMask to channels 64 to 71
            channelsMask[4] = linkAdrParams.ChMask;
        }
        else if( linkAdrParams.ChMaskCtrl == 7 )
        {
            // Disable all 125 kHz channels
            channelsMask[0] = 0x0000;
            channelsMask[1] = 0x0000;
            channelsMask[2] = 0x0000;
            channelsMask[3] = 0x0000;
            // Apply chMask to channels 64 to 71
            channelsMask[4] = linkAdrParams.ChMask;
        }
        else if( linkAdrParams.ChMaskCtrl == 5 )
        {
            // chMask selects [chMask * 8, chMask * 8 +7],64+chMask
            channelsMask[0] = 0x0000;
            channelsMask[1] = 0x0000;
            channelsMask[2] = 0x0000;
            channelsMask[3] = 0x0000;
            channelsMask[(linkAdrParams.ChMask & 7) >> 1] = ((linkAdrParams.ChMask & 1) ? 0xFF00 : 0x00FF);
            channelsMask[4] = (0x0001 << (linkAdrParams.ChMask & 7));
        }
        else
        {
            channelsMask[linkAdrParams.ChMaskCtrl] = linkAdrParams.ChMask;
        }
    }

    // FCC 15.247 paragraph F mandates to hop on at least 2 125 kHz channels
    if( ( linkAdrParams.Datarate < DR_4 ) && ( RegionCommonCountChannels( channelsMask, 0, 4 ) < 2 ) )
    {
        status &= 0xFE; // Channel mask KO
    }

    // Get the minimum possible datarate
    getPhy.Attribute = PHY_MIN_TX_DR;
    getPhy.UplinkDwellTime = linkAdrReq->UplinkDwellTime;
    phyParam = RegionUS915GetPhyParam( &getPhy );

    linkAdrVerifyParams.Status = status;
    linkAdrVerifyParams.AdrEnabled = linkAdrReq->AdrEnabled;
    linkAdrVerifyParams.Datarate = linkAdrParams.Datarate;
    linkAdrVerifyParams.TxPower = linkAdrParams.TxPower;
    linkAdrVerifyParams.NbRep = linkAdrParams.NbRep;
    linkAdrVerifyParams.CurrentDatarate = linkAdrReq->CurrentDatarate;
    linkAdrVerifyParams.CurrentTxPower = linkAdrReq->CurrentTxPower;
    linkAdrVerifyParams.CurrentNbRep = linkAdrReq->CurrentNbRep;
    linkAdrVerifyParams.NbChannels = US915_MAX_NB_CHANNELS;
    linkAdrVerifyParams.ChannelsMask = channelsMask;
    linkAdrVerifyParams.MinDatarate = ( int8_t )phyParam.Value;
    linkAdrVerifyParams.MaxDatarate = US915_TX_MAX_DATARATE;
    linkAdrVerifyParams.Channels = RegionUS915Channels;
    linkAdrVerifyParams.MinTxPower = US915_MIN_TX_POWER;
    linkAdrVerifyParams.MaxTxPower = US915_MAX_TX_POWER;

    // Verify the parameters and update, if necessary
    status = RegionCommonLinkAdrReqVerifyParams( &linkAdrVerifyParams, &linkAdrParams.Datarate, &linkAdrParams.TxPower, &linkAdrParams.NbRep );

    // Update channelsMask if everything is correct
    if( status == 0x07 )
    {
        // Copy Mask
        RegionCommonChanMaskCopy( RegionChannelsMask, channelsMask, 5 );

        RegionChannelsMaskRemaining[0] &= RegionChannelsMask[0];
        RegionChannelsMaskRemaining[1] &= RegionChannelsMask[1];
        RegionChannelsMaskRemaining[2] &= RegionChannelsMask[2];
        RegionChannelsMaskRemaining[3] &= RegionChannelsMask[3];
        RegionChannelsMaskRemaining[4] = RegionChannelsMask[4];
    }

    // Update status variables
    *drOut = linkAdrParams.Datarate;
    *txPowOut = linkAdrParams.TxPower;
    *nbRepOut = linkAdrParams.NbRep;
    *nbBytesParsed = bytesProcessed;

    return status;
}

uint8_t RegionUS915RxParamSetupReq( RxParamSetupReqParams_t* rxParamSetupReq )
{
    uint8_t status = 0x07;
    uint32_t freq = rxParamSetupReq->Frequency;

    // Verify radio frequency
    if( ( Radio.CheckRfFrequency( freq ) == false ) ||
        ( freq < US915_FIRST_RX1_CHANNEL ) ||
        ( freq > US915_LAST_RX1_CHANNEL ) ||
        ( ( ( freq - ( uint32_t ) US915_FIRST_RX1_CHANNEL ) % ( uint32_t ) US915_STEPWIDTH_RX1_CHANNEL ) != 0 ) )
    {
        status &= 0xFE; // Channel frequency KO
    }

    // Verify datarate
    if( RegionCommonValueInRange( rxParamSetupReq->Datarate, US915_RX_MIN_DATARATE, US915_RX_MAX_DATARATE ) == false )
    {
        status &= 0xFD; // Datarate KO
    }
    if( ( RegionCommonValueInRange( rxParamSetupReq->Datarate, DR_5, DR_7 ) == true ) ||
        ( rxParamSetupReq->Datarate > DR_13 ) )
    {
        status &= 0xFD; // Datarate KO
    }

    // Verify datarate offset
    if( RegionCommonValueInRange( rxParamSetupReq->DrOffset, US915_MIN_RX1_DR_OFFSET, US915_MAX_RX1_DR_OFFSET ) == false )
    {
        status &= 0xFB; // Rx1DrOffset range KO
    }

    return status;
}

uint8_t RegionUS915NewChannelReq( NewChannelReqParams_t* newChannelReq )
{
    // Datarate and frequency KO
    return 0;
}

int8_t RegionUS915TxParamSetupReq( TxParamSetupReqParams_t* txParamSetupReq )
{
    return -1;
}

uint8_t RegionUS915DlChannelReq( DlChannelReqParams_t* dlChannelReq )
{
    return 0;
}

int8_t RegionUS915AlternateDr( int8_t currentDr )
{
    static int16_t trialsCount = 0;

    if (lastJoinChannel < 0)
    {
        trialsCount = 0;
    }

    // Re-enable 500 kHz default channels
    RegionChannelsMask[4] = RegionChannelsDefaultMask[4];

    if( ( trialsCount % 9 ) == 8 )
    {
        currentDr = DR_4;
    }
    else
    {
        currentDr = DR_0;
    }
    trialsCount++;
    return currentDr;
}

void RegionUS915CalcBackOff( CalcBackOffParams_t* calcBackOff )
{
    RegionCommonCalcBackOffParams_t calcBackOffParams;

    calcBackOffParams.Channels = RegionUS915Channels;
    calcBackOffParams.Bands = RegionBands;
    calcBackOffParams.LastTxIsJoinRequest = calcBackOff->LastTxIsJoinRequest;
    calcBackOffParams.Joined = calcBackOff->Joined;
    calcBackOffParams.DutyCycleEnabled = calcBackOff->DutyCycleEnabled;
    calcBackOffParams.Channel = calcBackOff->Channel;
    calcBackOffParams.ElapsedTime = calcBackOff->ElapsedTime;
    calcBackOffParams.TxTimeOnAir = calcBackOff->TxTimeOnAir;

    RegionCommonCalcBackOff( &calcBackOffParams );
}

LoRaMacStatus_t RegionUS915NextChannel( NextChanParams_t* nextChanParams, uint8_t* channel, TimerTime_t* time, TimerTime_t* aggregatedTimeOff )
{
    uint8_t nbEnabledChannels = 0;
    uint8_t delayTx = 0;
    uint8_t enabledChannels[US915_MAX_NB_CHANNELS] = { 0 };
    uint8_t priority;
    TimerTime_t nextTxDelay = 0;

    // Count 125kHz channels
    if( RegionCommonCountChannels( RegionChannelsMaskRemaining, 0, 4 ) == 0 )
    { // Reactivate default channels
        RegionCommonChanMaskCopy( RegionChannelsMaskRemaining, RegionChannelsMask, 4  );
    }
    // Check other channels
    if( nextChanParams->Datarate >= DR_4 )
    {
        if( ( RegionChannelsMaskRemaining[4] & 0x00FF ) == 0 )
        {
            RegionChannelsMaskRemaining[4] = RegionChannelsMask[4];
        }
    }

    if( nextChanParams->AggrTimeOff <= TimerGetElapsedTime( nextChanParams->LastAggrTx ) )
    {
        // Reset Aggregated time off
        *aggregatedTimeOff = 0;

        // Update bands Time OFF
        nextTxDelay = RegionCommonUpdateBandTimeOff( nextChanParams->Joined, nextChanParams->DutyCycleEnabled, RegionBands, US915_MAX_NB_BANDS );

        // Search how many channels are enabled
        nbEnabledChannels = CountNbOfEnabledChannels( nextChanParams->Datarate,
                                                      RegionChannelsMaskRemaining, RegionUS915Channels,
                                                      RegionBands, enabledChannels, &delayTx );
    }
    else
    {
        delayTx++;
        nextTxDelay = nextChanParams->AggrTimeOff - TimerGetElapsedTime( nextChanParams->LastAggrTx );
    }

    if( nbEnabledChannels > 0 )
    {
        if( channel )
        {
            // We found a valid channel
            if( nextChanParams->Joined )
            {
                if (lastJoinChannel >= 0)
                {
                    *channel = lastJoinChannel;

                    lastJoinChannel = -1;
                }
                else
                {
                    *channel = enabledChannels[randr( 0, nbEnabledChannels - 1 )];
                }
            }
            else
            {
                priority = US915_MAX_NB_CHANNELS;

                if (lastJoinChannel >= 0)
                {
                    for( uint8_t i = 0; i < nbEnabledChannels; i++ )
                    {
                        if( RegionUS915Priorities[lastJoinChannel] < RegionUS915Priorities[enabledChannels[i]] )
                        {
                            if( priority > RegionUS915Priorities[enabledChannels[i]] ) 
                            {
                                priority = RegionUS915Priorities[enabledChannels[i]];
                                
                                *channel = enabledChannels[i];
                            }
                        }
                    }
                }

                if( priority == US915_MAX_NB_CHANNELS )
                {
                    for( uint8_t i = 0; i < nbEnabledChannels; i++ )
                    {
                        if( priority > RegionUS915Priorities[enabledChannels[i]] ) 
                        {
                            priority = RegionUS915Priorities[enabledChannels[i]];
                            
                            *channel = enabledChannels[i];
                        }
                    }
                }

                lastJoinChannel = *channel;
            }

            // Disable the channel in the mask
            RegionCommonChanDisable( RegionChannelsMaskRemaining, *channel, US915_MAX_NB_CHANNELS - 8 );
        }
        *time = 0;
        return LORAMAC_STATUS_OK;
    }
    else
    {
        if( delayTx > 0 )
        {
            // Delay transmission due to AggregatedTimeOff or to a band time off
            *time = nextTxDelay;
            return LORAMAC_STATUS_DUTYCYCLE_RESTRICTED;
        }
        // Datarate not supported by any channel
        *time = 0;
        return LORAMAC_STATUS_NO_CHANNEL_FOUND;
    }
}

LoRaMacStatus_t RegionUS915ChannelAdd( ChannelAddParams_t* channelAdd )
{
    return LORAMAC_STATUS_PARAMETER_INVALID;
}

bool RegionUS915ChannelRemove( ChannelRemoveParams_t* channelRemove  )
{
    return LORAMAC_STATUS_PARAMETER_INVALID;
}

void RegionUS915SetContinuousWave( ContinuousWaveParams_t* continuousWave )
{
    int8_t txPowerLimited = LimitTxPower( continuousWave->TxPower, RegionBands[RegionUS915Channels[continuousWave->Channel].Band].TxMaxPower, continuousWave->Datarate, RegionChannelsMask );
    int8_t phyTxPower = 0;
    uint32_t frequency = RegionUS915Channels[continuousWave->Channel].Frequency;

    // Calculate physical TX power
    phyTxPower = RegionCommonComputeTxPower( txPowerLimited, US915_DEFAULT_MAX_ERP, 0 );

    Radio.SetTxContinuousWave( frequency, phyTxPower, continuousWave->Timeout );
}

uint8_t RegionUS915ApplyDrOffset( uint8_t downlinkDwellTime, int8_t dr, int8_t drOffset )
{
    int8_t datarate = DatarateOffsetsUS915[dr][drOffset];

    if( datarate < 0 )
    {
        datarate = DR_0;
    }
    return datarate;
}

const LoRaMacRegion_t LoRaMacRegionUS915 = {
    RegionUS915GetPhyParam,
    RegionUS915SetBandTxDone,
    RegionUS915InitDefaults,
    RegionUS915Verify,
    RegionUS915ApplyCFList,
    RegionUS915ChanMaskSet,
    RegionUS915AdrNext,
    RegionUS915ComputeRxWindowParameters,
    RegionUS915RxConfig,
    RegionUS915TxConfig,
    RegionUS915LinkAdrReq,
    RegionUS915RxParamSetupReq,
    RegionUS915NewChannelReq,
    RegionUS915TxParamSetupReq,
    RegionUS915DlChannelReq,
    RegionUS915AlternateDr,
    RegionUS915CalcBackOff,
    RegionUS915NextChannel,
    RegionUS915ChannelAdd,
    RegionUS915ChannelRemove,
    RegionUS915SetContinuousWave,
    RegionUS915ApplyDrOffset,
};
