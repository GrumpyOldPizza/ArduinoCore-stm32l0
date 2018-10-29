/*!
 * \file      RegionCN470.c
 *
 * \brief     Region implementation for CN470
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
#include "RegionCN470.h"

// Definitions
#define CHANNELS_MASK_SIZE              6

// Global attributes
/*!
 * LoRaMAC channels
 */
static const ChannelParams_t RegionCN470Channels[CN470_MAX_NB_CHANNELS] = {
    { 470300000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 470500000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 470700000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 470900000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 471100000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 471300000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 471500000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 471700000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 471900000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 472100000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 472300000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 472500000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 472700000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 472900000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 473100000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 473300000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 473500000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 473700000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 473900000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 474100000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 474300000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 474500000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 474700000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 474900000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 475100000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 475300000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 475500000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 475700000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 475900000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 476100000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 476300000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 476500000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 476700000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 476900000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 477100000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 477300000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 477500000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 477700000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 477900000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 478100000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 478300000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 478500000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 478700000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 478900000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 479100000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 479300000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 479500000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 479700000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 479900000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 480100000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 480300000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 480500000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 480700000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 480900000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 481100000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 481300000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 481500000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 481700000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 481900000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 482100000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 482300000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 482500000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 482700000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 482900000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 483100000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 483300000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 483500000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 483700000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 483900000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 484100000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 484300000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 484500000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 484700000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 484900000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 485100000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 485300000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 485500000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 485700000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 485900000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 486100000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 486300000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 486500000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 486700000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 486900000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 487100000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 487300000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 487500000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 487700000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 487900000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 488100000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 488300000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 488500000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 488700000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 488900000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 489100000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
    { 489300000, 0, { ( DR_5 << 4 ) | DR_0 }, 0 },
};

/*!
 * LoRaMac bands
 */
extern Band_t RegionBands[CN470_MAX_NB_BANDS];

/*!
 * LoRaMac channels mask
 */
extern uint16_t RegionChannelsMask[CHANNELS_MASK_SIZE];

/*!
 * LoRaMac channels default mask
 */
extern uint16_t RegionChannelsDefaultMask[CHANNELS_MASK_SIZE];

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
    switch( BandwidthsCN470[drIndex] )
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

    return txPowerResult;
}

static uint8_t CountNbOfEnabledChannels( uint8_t datarate, uint16_t* channelsMask, const ChannelParams_t* channels, Band_t* bands, uint8_t* enabledChannels, uint8_t* delayTx )
{
    uint8_t nbEnabledChannels = 0;
    uint8_t delayTransmission = 0;

    for( uint8_t i = 0, k = 0; i < CN470_MAX_NB_CHANNELS; i += 16, k++ )
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

PhyParam_t RegionCN470GetPhyParam( GetPhyParams_t* getPhy )
{
    PhyParam_t phyParam = { 0 };

    switch( getPhy->Attribute )
    {
        case PHY_MIN_RX_DR:
        {
            phyParam.Value = CN470_RX_MIN_DATARATE;
            break;
        }
        case PHY_MIN_TX_DR:
        {
            phyParam.Value = CN470_TX_MIN_DATARATE;
            break;
        }
        case PHY_DEF_TX_DR:
        {
            phyParam.Value = CN470_DEFAULT_DATARATE;
            break;
        }
        case PHY_NEXT_LOWER_TX_DR:
        {
            phyParam.Value = GetNextLowerTxDr( getPhy->Datarate, CN470_TX_MIN_DATARATE );
            break;
        }
        case PHY_DEF_TX_POWER:
        {
            phyParam.Value = CN470_DEFAULT_TX_POWER;
            break;
        }
        case PHY_MAX_PAYLOAD:
        {
            phyParam.Value = MaxPayloadOfDatarateCN470[getPhy->Datarate];
            break;
        }
        case PHY_MAX_PAYLOAD_REPEATER:
        {
            phyParam.Value = MaxPayloadOfDatarateRepeaterCN470[getPhy->Datarate];
            break;
        }
        case PHY_DUTY_CYCLE:
        {
            phyParam.Value = CN470_DUTY_CYCLE_ENABLED;
            break;
        }
        case PHY_MAX_RX_WINDOW:
        {
            phyParam.Value = CN470_MAX_RX_WINDOW;
            break;
        }
        case PHY_RECEIVE_DELAY1:
        {
            phyParam.Value = CN470_RECEIVE_DELAY1;
            break;
        }
        case PHY_RECEIVE_DELAY2:
        {
            phyParam.Value = CN470_RECEIVE_DELAY2;
            break;
        }
        case PHY_JOIN_ACCEPT_DELAY1:
        {
            phyParam.Value = CN470_JOIN_ACCEPT_DELAY1;
            break;
        }
        case PHY_JOIN_ACCEPT_DELAY2:
        {
            phyParam.Value = CN470_JOIN_ACCEPT_DELAY2;
            break;
        }
        case PHY_MAX_FCNT_GAP:
        {
            phyParam.Value = CN470_MAX_FCNT_GAP;
            break;
        }
        case PHY_ACK_TIMEOUT:
        {
            phyParam.Value = ( CN470_ACKTIMEOUT + randr( -CN470_ACK_TIMEOUT_RND, CN470_ACK_TIMEOUT_RND ) );
            break;
        }
        case PHY_DEF_DR1_OFFSET:
        {
            phyParam.Value = CN470_DEFAULT_RX1_DR_OFFSET;
            break;
        }
        case PHY_DEF_RX2_FREQUENCY:
        {
            phyParam.Value = CN470_RX_WND_2_FREQ;
            break;
        }
        case PHY_DEF_RX2_DR:
        {
            phyParam.Value = CN470_RX_WND_2_DR;
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
            phyParam.Value = CN470_MAX_NB_CHANNELS;
            break;
        }
        case PHY_CHANNELS:
        {
            phyParam.Channels = RegionCN470Channels;
            break;
        }
        case PHY_DEF_UPLINK_DWELL_TIME:
        case PHY_DEF_DOWNLINK_DWELL_TIME:
        {
            phyParam.Value = 0;
            break;
        }
        case PHY_DEF_MAX_EIRP:
        {
            phyParam.fValue = CN470_DEFAULT_MAX_EIRP;
            break;
        }
        case PHY_DEF_ANTENNA_GAIN:
        {
            phyParam.fValue = CN470_DEFAULT_ANTENNA_GAIN;
            break;
        }
        case PHY_RX_CALIBRATION_FREQUENCY:
        {
            phyParam.Value = CN470_RX_CAL_FREQ;
            break;
        }
        default:
        {
            break;
        }
    }

    return phyParam;
}

void RegionCN470SetBandTxDone( SetBandTxDoneParams_t* txDone )
{
    RegionCommonSetBandTxDone( txDone->Joined, &RegionBands[RegionCN470Channels[txDone->Channel].Band], txDone->LastTxDoneTime );
}

void RegionCN470InitDefaults( InitType_t type )
{
    switch( type )
    {
        case INIT_TYPE_INIT:
        {
#if 0
            // Channels
            // 125 kHz channels
            for( uint8_t i = 0; i < CN470_MAX_NB_CHANNELS; i++ )
            {
                RegionCN470Channels[i].Frequency = 470300000 + i * 200000;
                RegionCN470Channels[i].DrRange.Value = ( DR_5 << 4 ) | DR_0;
                RegionCN470Channels[i].Band = 0;
            }
#endif

            // Bands
            RegionBands[0] = ( Band_t )CN470_BAND0;

            // Initialize the channels default mask
            RegionChannelsDefaultMask[0] = 0xFFFF;
            RegionChannelsDefaultMask[1] = 0xFFFF;
            RegionChannelsDefaultMask[2] = 0xFFFF;
            RegionChannelsDefaultMask[3] = 0xFFFF;
            RegionChannelsDefaultMask[4] = 0xFFFF;
            RegionChannelsDefaultMask[5] = 0xFFFF;

            // Update the channels mask
            RegionCommonChanMaskCopy( RegionChannelsMask, RegionChannelsDefaultMask, 6 );
            break;
        }
        case INIT_TYPE_RESTORE:
        {
            // Restore channels default mask
            RegionCommonChanMaskCopy( RegionChannelsMask, RegionChannelsDefaultMask, 6 );
            break;
        }
        case INIT_TYPE_APP_DEFAULTS:
        {
            // Update the channels mask defaults
            RegionCommonChanMaskCopy( RegionChannelsMask, RegionChannelsDefaultMask, 6 );
            break;
        }
        default:
        {
            break;
        }
    }
}

bool RegionCN470Verify( VerifyParams_t* verify, PhyAttribute_t phyAttribute )
{
    switch( phyAttribute )
    {
        case PHY_TX_DR:
        case PHY_DEF_TX_DR:
        {
            return RegionCommonValueInRange( verify->DatarateParams.Datarate, CN470_TX_MIN_DATARATE, CN470_TX_MAX_DATARATE );
        }
        case PHY_RX_DR:
        {
            return RegionCommonValueInRange( verify->DatarateParams.Datarate, CN470_RX_MIN_DATARATE, CN470_RX_MAX_DATARATE );
        }
        case PHY_DEF_TX_POWER:
        case PHY_TX_POWER:
        {
            // Remark: switched min and max!
            return RegionCommonValueInRange( verify->TxPower, CN470_MAX_TX_POWER, CN470_MIN_TX_POWER );
        }
        case PHY_DUTY_CYCLE:
        {
            return CN470_DUTY_CYCLE_ENABLED;
        }
        default:
            return false;
    }
}

void RegionCN470ApplyCFList( ApplyCFListParams_t* applyCFList )
{
    return;
}

bool RegionCN470ChanMaskSet( ChanMaskSetParams_t* chanMaskSet )
{
    switch( chanMaskSet->ChannelsMaskType )
    {
        case CHANNELS_MASK:
        {
            RegionCommonChanMaskCopy( RegionChannelsMask, chanMaskSet->ChannelsMaskIn, 6 );
            break;
        }
        case CHANNELS_DEFAULT_MASK:
        {
            RegionCommonChanMaskCopy( RegionChannelsDefaultMask, chanMaskSet->ChannelsMaskIn, 6 );
            break;
        }
        default:
            return false;
    }
    return true;
}

bool RegionCN470AdrNext( AdrNextParams_t* adrNext, int8_t* drOut, int8_t* txPowOut, uint32_t* adrAckCounter )
{
    bool adrAckReq = false;
    int8_t datarate = adrNext->Datarate;
    int8_t txPower = adrNext->TxPower;
    GetPhyParams_t getPhy;
    PhyParam_t phyParam;

    // Report back the adr ack counter
    *adrAckCounter = adrNext->AdrAckCounter;

    if( adrNext->AdrEnabled == true )
    {
        if( datarate == CN470_TX_MIN_DATARATE )
        {
            *adrAckCounter = 0;
            adrAckReq = false;
        }
        else
        {
            if( adrNext->AdrAckCounter >= CN470_ADR_ACK_LIMIT )
            {
                adrAckReq = true;
                txPower = CN470_MAX_TX_POWER;
            }
            else
            {
                adrAckReq = false;
            }
            if( adrNext->AdrAckCounter >= ( CN470_ADR_ACK_LIMIT + CN470_ADR_ACK_DELAY ) )
            {
                if( ( adrNext->AdrAckCounter % CN470_ADR_ACK_DELAY ) == 1 )
                {
                    // Decrease the datarate
                    getPhy.Attribute = PHY_NEXT_LOWER_TX_DR;
                    getPhy.Datarate = datarate;
                    getPhy.UplinkDwellTime = adrNext->UplinkDwellTime;
                    phyParam = RegionCN470GetPhyParam( &getPhy );
                    datarate = phyParam.Value;

                    if( datarate == CN470_TX_MIN_DATARATE )
                    {
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
                            RegionChannelsMask[5] = RegionChannelsDefaultMask[5];
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

void RegionCN470ComputeRxWindowParameters( int8_t datarate, uint8_t minRxSymbols, uint32_t rxError, RxConfigParams_t *rxConfigParams )
{
    double tSymbol = 0.0;

    // Get the datarate, perform a boundary check
    rxConfigParams->Datarate = MIN( datarate, CN470_RX_MAX_DATARATE );
    rxConfigParams->Bandwidth = GetBandwidth( rxConfigParams->Datarate );

    tSymbol = RegionCommonComputeSymbolTimeLoRa( DataratesCN470[rxConfigParams->Datarate], BandwidthsCN470[rxConfigParams->Datarate] );

    RegionCommonComputeRxWindowParameters( tSymbol, minRxSymbols, rxError, Radio.GetWakeupTime( ), &rxConfigParams->WindowTimeout, &rxConfigParams->WindowOffset );
}

bool RegionCN470RxConfig( RxConfigParams_t* rxConfig, int8_t* datarate )
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
        frequency = CN470_FIRST_RX1_CHANNEL + ( rxConfig->Channel % 48 ) * CN470_STEPWIDTH_RX1_CHANNEL;
    }

    // Read the physical datarate from the datarates table
    phyDr = DataratesCN470[dr];

    Radio.SetChannel( frequency );

    // Radio configuration
    Radio.SetRxConfig( MODEM_LORA, rxConfig->Bandwidth, phyDr, 1, 0, 8, rxConfig->WindowTimeout, false, 0, false, 0, 0, true, rxConfig->RxContinuous );

    if( rxConfig->RepeaterSupport == true )
    {
        maxPayload = MaxPayloadOfDatarateRepeaterCN470[dr];
    }
    else
    {
        maxPayload = MaxPayloadOfDatarateCN470[dr];
    }
    Radio.SetMaxPayloadLength( MODEM_LORA, maxPayload + LORA_MAC_FRMPAYLOAD_OVERHEAD );

    *datarate = (uint8_t) dr;
    return true;
}

bool RegionCN470TxConfig( TxConfigParams_t* txConfig, int8_t* txPower, TimerTime_t* txTimeOnAir )
{
    int8_t phyDr = DataratesCN470[txConfig->Datarate];
    int8_t txPowerLimited = LimitTxPower( txConfig->TxPower, RegionBands[RegionCN470Channels[txConfig->Channel].Band].TxMaxPower, txConfig->Datarate, RegionChannelsMask );
    int8_t phyTxPower = 0;

    // Calculate physical TX power
    phyTxPower = RegionCommonComputeTxPower( txPowerLimited, txConfig->MaxEirp, txConfig->AntennaGain );

    // Setup the radio frequency
    Radio.SetChannel( RegionCN470Channels[txConfig->Channel].Frequency );

    Radio.SetTxConfig( MODEM_LORA, phyTxPower, 0, 0, phyDr, 1, 8, false, true, 0, 0, false, 3000 );
    // Setup maximum payload lenght of the radio driver
    Radio.SetMaxPayloadLength( MODEM_LORA, txConfig->PktLen );
    // Get the time-on-air of the next tx frame
    *txTimeOnAir = Radio.TimeOnAir( MODEM_LORA, txConfig->PktLen );
    *txPower = txPowerLimited;

    return true;
}

uint8_t RegionCN470LinkAdrReq( LinkAdrReqParams_t* linkAdrReq, int8_t* drOut, int8_t* txPowOut, uint8_t* nbRepOut, uint8_t* nbBytesParsed )
{
    uint8_t status = 0x07;
    RegionCommonLinkAdrParams_t linkAdrParams;
    uint8_t nextIndex = 0;
    uint8_t bytesProcessed = 0;
    uint16_t channelsMask[6] = { 0, 0, 0, 0, 0, 0 };
    GetPhyParams_t getPhy;
    PhyParam_t phyParam;
    RegionCommonLinkAdrReqVerifyParams_t linkAdrVerifyParams;

    // Initialize local copy of channels mask
    RegionCommonChanMaskCopy( channelsMask, RegionChannelsMask, 6 );

    while( bytesProcessed < linkAdrReq->PayloadSize )
    {
        // Get ADR request parameters
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
            channelsMask[4] = 0xFFFF;
            channelsMask[5] = 0xFFFF;
        }
        else if( linkAdrParams.ChMaskCtrl == 7 )
        {
            status &= 0xFE; // Channel mask KO
        }
        else
        {
            for( uint8_t i = 0; i < 16; i++ )
            {
                if( ( ( linkAdrParams.ChMask & ( 1 << i ) ) != 0 ) &&
                    ( RegionCN470Channels[linkAdrParams.ChMaskCtrl * 16 + i].Frequency == 0 ) )
                {// Trying to enable an undefined channel
                    status &= 0xFE; // Channel mask KO
                }
            }
            channelsMask[linkAdrParams.ChMaskCtrl] = linkAdrParams.ChMask;
        }
    }

    // Get the minimum possible datarate
    getPhy.Attribute = PHY_MIN_TX_DR;
    getPhy.UplinkDwellTime = linkAdrReq->UplinkDwellTime;
    phyParam = RegionCN470GetPhyParam( &getPhy );

    linkAdrVerifyParams.Status = status;
    linkAdrVerifyParams.AdrEnabled = linkAdrReq->AdrEnabled;
    linkAdrVerifyParams.Datarate = linkAdrParams.Datarate;
    linkAdrVerifyParams.TxPower = linkAdrParams.TxPower;
    linkAdrVerifyParams.NbRep = linkAdrParams.NbRep;
    linkAdrVerifyParams.CurrentDatarate = linkAdrReq->CurrentDatarate;
    linkAdrVerifyParams.CurrentTxPower = linkAdrReq->CurrentTxPower;
    linkAdrVerifyParams.CurrentNbRep = linkAdrReq->CurrentNbRep;
    linkAdrVerifyParams.NbChannels = CN470_MAX_NB_CHANNELS;
    linkAdrVerifyParams.ChannelsMask = channelsMask;
    linkAdrVerifyParams.MinDatarate = ( int8_t )phyParam.Value;
    linkAdrVerifyParams.MaxDatarate = CN470_TX_MAX_DATARATE;
    linkAdrVerifyParams.Channels = RegionCN470Channels;
    linkAdrVerifyParams.MinTxPower = CN470_MIN_TX_POWER;
    linkAdrVerifyParams.MaxTxPower = CN470_MAX_TX_POWER;

    // Verify the parameters and update, if necessary
    status = RegionCommonLinkAdrReqVerifyParams( &linkAdrVerifyParams, &linkAdrParams.Datarate, &linkAdrParams.TxPower, &linkAdrParams.NbRep );

    // Update channelsMask if everything is correct
    if( status == 0x07 )
    {
        // Copy Mask
        RegionCommonChanMaskCopy( RegionChannelsMask, channelsMask, 6 );
    }

    // Update status variables
    *drOut = linkAdrParams.Datarate;
    *txPowOut = linkAdrParams.TxPower;
    *nbRepOut = linkAdrParams.NbRep;
    *nbBytesParsed = bytesProcessed;

    return status;
}

uint8_t RegionCN470RxParamSetupReq( RxParamSetupReqParams_t* rxParamSetupReq )
{
    uint8_t status = 0x07;
    uint32_t freq = rxParamSetupReq->Frequency;

    // Verify radio frequency
    if( ( Radio.CheckRfFrequency( freq ) == false ) ||
        ( freq < CN470_FIRST_RX1_CHANNEL ) ||
        ( freq > CN470_LAST_RX1_CHANNEL ) ||
        ( ( ( freq - ( uint32_t ) CN470_FIRST_RX1_CHANNEL ) % ( uint32_t ) CN470_STEPWIDTH_RX1_CHANNEL ) != 0 ) )
    {
        status &= 0xFE; // Channel frequency KO
    }

    // Verify datarate
    if( RegionCommonValueInRange( rxParamSetupReq->Datarate, CN470_RX_MIN_DATARATE, CN470_RX_MAX_DATARATE ) == false )
    {
        status &= 0xFD; // Datarate KO
    }

    // Verify datarate offset
    if( RegionCommonValueInRange( rxParamSetupReq->DrOffset, CN470_MIN_RX1_DR_OFFSET, CN470_MAX_RX1_DR_OFFSET ) == false )
    {
        status &= 0xFB; // Rx1DrOffset range KO
    }

    return status;
}

uint8_t RegionCN470NewChannelReq( NewChannelReqParams_t* newChannelReq )
{
    // Datarate and frequency KO
    return 0;
}

int8_t RegionCN470TxParamSetupReq( TxParamSetupReqParams_t* txParamSetupReq )
{
    return -1;
}

uint8_t RegionCN470DlChannelReq( DlChannelReqParams_t* dlChannelReq )
{
    return 0;
}

int8_t RegionCN470AlternateDr( int8_t currentDr )
{
    return currentDr;
}

void RegionCN470CalcBackOff( CalcBackOffParams_t* calcBackOff )
{
    RegionCommonCalcBackOffParams_t calcBackOffParams;

    calcBackOffParams.Channels = RegionCN470Channels;
    calcBackOffParams.Bands = RegionBands;
    calcBackOffParams.LastTxIsJoinRequest = calcBackOff->LastTxIsJoinRequest;
    calcBackOffParams.Joined = calcBackOff->Joined;
    calcBackOffParams.DutyCycleEnabled = calcBackOff->DutyCycleEnabled;
    calcBackOffParams.Channel = calcBackOff->Channel;
    calcBackOffParams.ElapsedTime = calcBackOff->ElapsedTime;
    calcBackOffParams.TxTimeOnAir = calcBackOff->TxTimeOnAir;

    RegionCommonCalcBackOff( &calcBackOffParams );
}

LoRaMacStatus_t RegionCN470NextChannel( NextChanParams_t* nextChanParams, uint8_t* channel, TimerTime_t* time, TimerTime_t* aggregatedTimeOff )
{
    uint8_t nbEnabledChannels = 0;
    uint8_t delayTx = 0;
    uint8_t enabledChannels[CN470_MAX_NB_CHANNELS] = { 0 };
    TimerTime_t nextTxDelay = 0;

    // Count 125kHz channels
    if( RegionCommonCountChannels( RegionChannelsMask, 0, 6 ) == 0 )
    { // Reactivate default channels
        RegionChannelsMask[0] = RegionChannelsDefaultMask[0];
        RegionChannelsMask[1] = RegionChannelsDefaultMask[1];
        RegionChannelsMask[2] = RegionChannelsDefaultMask[2];
        RegionChannelsMask[3] = RegionChannelsDefaultMask[3];
        RegionChannelsMask[4] = RegionChannelsDefaultMask[4];
        RegionChannelsMask[5] = RegionChannelsDefaultMask[5];
    }

    if( nextChanParams->AggrTimeOff <= TimerGetElapsedTime( nextChanParams->LastAggrTx ) )
    {
        // Reset Aggregated time off
        *aggregatedTimeOff = 0;

        // Update bands Time OFF
        nextTxDelay = RegionCommonUpdateBandTimeOff( nextChanParams->Joined, nextChanParams->DutyCycleEnabled, RegionBands, CN470_MAX_NB_BANDS );

        // Search how many channels are enabled
        nbEnabledChannels = CountNbOfEnabledChannels( nextChanParams->Datarate,
                                                      RegionChannelsMask, RegionCN470Channels,
                                                      RegionBands, enabledChannels, &delayTx );
    }
    else
    {
        delayTx++;
        nextTxDelay = nextChanParams->AggrTimeOff - TimerGetElapsedTime( nextChanParams->LastAggrTx );
    }

    if( nbEnabledChannels > 0 )
    {
        if ( channel )
        {
            // We found a valid channel
            *channel = enabledChannels[randr( 0, nbEnabledChannels - 1 )];
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

LoRaMacStatus_t RegionCN470ChannelAdd( ChannelAddParams_t* channelAdd )
{
    return LORAMAC_STATUS_PARAMETER_INVALID;
}

bool RegionCN470ChannelRemove( ChannelRemoveParams_t* channelRemove  )
{
    return LORAMAC_STATUS_PARAMETER_INVALID;
}

void RegionCN470SetContinuousWave( ContinuousWaveParams_t* continuousWave )
{
    int8_t txPowerLimited = LimitTxPower( continuousWave->TxPower, RegionBands[RegionCN470Channels[continuousWave->Channel].Band].TxMaxPower, continuousWave->Datarate, RegionChannelsMask );
    int8_t phyTxPower = 0;
    uint32_t frequency = RegionCN470Channels[continuousWave->Channel].Frequency;

    // Calculate physical TX power
    phyTxPower = RegionCommonComputeTxPower( txPowerLimited, continuousWave->MaxEirp, continuousWave->AntennaGain );

    Radio.SetTxContinuousWave( frequency, phyTxPower, continuousWave->Timeout );
}

uint8_t RegionCN470ApplyDrOffset( uint8_t downlinkDwellTime, int8_t dr, int8_t drOffset )
{
    int8_t datarate = dr - drOffset;

    if( datarate < 0 )
    {
        datarate = DR_0;
    }
    return datarate;
}

const LoRaMacRegion_t LoRaMacRegionCN470 = {
    RegionCN470GetPhyParam,
    RegionCN470SetBandTxDone,
    RegionCN470InitDefaults,
    RegionCN470Verify,
    RegionCN470ApplyCFList,
    RegionCN470ChanMaskSet,
    RegionCN470AdrNext,
    RegionCN470ComputeRxWindowParameters,
    RegionCN470RxConfig,
    RegionCN470TxConfig,
    RegionCN470LinkAdrReq,
    RegionCN470RxParamSetupReq,
    RegionCN470NewChannelReq,
    RegionCN470TxParamSetupReq,
    RegionCN470DlChannelReq,
    RegionCN470AlternateDr,
    RegionCN470CalcBackOff,
    RegionCN470NextChannel,
    RegionCN470ChannelAdd,
    RegionCN470ChannelRemove,
    RegionCN470SetContinuousWave,
    RegionCN470ApplyDrOffset,
};
