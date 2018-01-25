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

#include "Arduino.h"
#include "LoRaWAN.h"
#include "STM32L0.h"
#include "wiring_private.h"

extern "C" {
#include "LoRaMac.h"
#include "LoRaMacTest.h"
}

static void BoardGetUniqueId(uint8_t *DevEui)
{
    uint32_t UID[3];

    stm32l0_system_uid(&UID[0]);

    UID[0] = UID[0] + UID[2];
    
    DevEui[0] = UID[1] >> 0;
    DevEui[1] = UID[1] >> 8;
    DevEui[2] = UID[1] >> 16;
    DevEui[3] = UID[1] >> 24;
    DevEui[4] = UID[0] >> 0;
    DevEui[5] = UID[0] >> 8;
    DevEui[6] = UID[0] >> 16;
    DevEui[7] = UID[0] >> 24;
}

static uint32_t ConvertString(uint8_t *out, uint32_t size, const char *cp)
{
    uint8_t c0, c1;
    uint8_t *out_e;

    if (strlen(cp) != (size * 2)) {
	return 0;
    }

    out_e = out + size;

    do
    {
	c0 = *cp++;

	if      ((c0 >= '0') && (c0 <= '9')) { c0 = c0 - '0';      }
	else if ((c0 >= 'A') && (c0 <= 'F')) { c0 = c0 - 'A' + 10; }
	else if ((c0 >= 'a') && (c0 <= 'f')) { c0 = c0 - 'a' + 10; }
	else
	{
	    return 0;
	}

	c1 = *cp++;

	if      ((c1 >= '0') && (c1 <= '9')) { c1 = c1 - '0';      }
	else if ((c1 >= 'A') && (c1 <= 'F')) { c1 = c1 - 'A' + 10; }
	else if ((c1 >= 'a') && (c1 <= 'f')) { c1 = c1 - 'a' + 10; }
	else
	{
	    return 0;
	}

	*out++ = (c0 << 4) | c1;
    }
    while (out != out_e);

    return size;
}

static LoRaMacStatus_t LoRaWANQueryTxPossible( uint8_t size, LoRaMacTxInfo_t* txInfo )
{
    IRQn_Type irq;

    irq = (IRQn_Type)((__get_IPSR() & 0x1ff) - 16);

    if (irq == Reset_IRQn)
    {
	return (LoRaMacStatus_t)armv6m_svcall_2((uint32_t)&LoRaMacQueryTxPossible, (uint32_t)size, (uint32_t)txInfo);
    }
    else
    {
	if ((irq == SVC_IRQn) || (irq == PendSV_IRQn))
	{
	    return LoRaMacQueryTxPossible(size, txInfo);
	}
	else
	{
	    return LORAMAC_STATUS_BUSY;
	}
    }
}

static LoRaMacStatus_t LoRaWANChannelAdd( uint8_t id, ChannelParams_t *params )
{
    IRQn_Type irq;

    irq = (IRQn_Type)((__get_IPSR() & 0x1ff) - 16);

    if (irq == Reset_IRQn)
    {
	return (LoRaMacStatus_t)armv6m_svcall_2((uint32_t)&LoRaMacChannelAdd, (uint32_t)id, (uint32_t)params);
    }
    else
    {
	if ((irq == SVC_IRQn) || (irq == PendSV_IRQn))
	{
	    return LoRaMacChannelAdd(id, params);
	}
	else
	{
	    return LORAMAC_STATUS_BUSY;
	}
    }
}

static LoRaMacStatus_t LoRaWANChannelRemove( uint8_t id )
{
    IRQn_Type irq;

    irq = (IRQn_Type)((__get_IPSR() & 0x1ff) - 16);

    if (irq == Reset_IRQn)
    {
	return (LoRaMacStatus_t)armv6m_svcall_1((uint32_t)&LoRaMacChannelRemove, (uint32_t)id);
    }
    else
    {
	if ((irq == SVC_IRQn) || (irq == PendSV_IRQn))
	{
	    return LoRaMacChannelRemove(id);
	}
	else
	{
	    return LORAMAC_STATUS_BUSY;
	}
    }
}

#ifdef notyet

static LoRaMacStatus_t LoRaWANMulticastChannelLink( MulticastParams_t *channelParam )
{
    IRQn_Type irq;

    irq = (IRQn_Type)((__get_IPSR() & 0x1ff) - 16);

    if (irq == Reset_IRQn)
    {
	return (LoRaMacStatus_t)armv6m_svcall_1((uint32_t)&LoRaMacMulticastChannelLink, (uint32_t)channelParam);
    }
    else
    {
	if ((irq == SVC_IRQn) || (irq == PendSV_IRQn))
	{
	    return LoRaMacMulticastChannelLink(channelParam);
	}
	else
	{
	    return LORAMAC_STATUS_BUSY;
	}
    }
}

static LoRaMacStatus_t LoRaWANMulticastChannelUnlink( MulticastParams_t *channelParam )
{
    IRQn_Type irq;

    irq = (IRQn_Type)((__get_IPSR() & 0x1ff) - 16);

    if (irq == Reset_IRQn)
    {
	return (LoRaMacStatus_t)armv6m_svcall_1((uint32_t)&LoRaMacMulticastChannelUnlink, (uint32_t)channelParam);
    }
    else
    {
	if ((irq == SVC_IRQn) || (irq == PendSV_IRQn))
	{
	    return LoRaMacMulticastChannelUnlink(channelParam);
	}
	else
	{
	    return LORAMAC_STATUS_BUSY;
	}
    }
}

#endif

static LoRaMacStatus_t LoRaWANMibGetRequestConfirm( MibRequestConfirm_t *mibGet )
{
    IRQn_Type irq;

    irq = (IRQn_Type)((__get_IPSR() & 0x1ff) - 16);

    if (irq == Reset_IRQn)
    {
	return (LoRaMacStatus_t)armv6m_svcall_1((uint32_t)&LoRaMacMibGetRequestConfirm, (uint32_t)mibGet);
    }
    else
    {
	if ((irq == SVC_IRQn) || (irq == PendSV_IRQn))
	{
	    return LoRaMacMibGetRequestConfirm(mibGet);
	}
	else
	{
	    return LORAMAC_STATUS_BUSY;
	}
    }
}

static LoRaMacStatus_t LoRaWANMibSetRequestConfirm( MibRequestConfirm_t *mibSet )
{
    IRQn_Type irq;

    irq = (IRQn_Type)((__get_IPSR() & 0x1ff) - 16);

    if (irq == Reset_IRQn)
    {
	return (LoRaMacStatus_t)armv6m_svcall_1((uint32_t)&LoRaMacMibSetRequestConfirm, (uint32_t)mibSet);
    }
    else
    {
	if ((irq == SVC_IRQn) || (irq == PendSV_IRQn))
	{
	    return LoRaMacMibSetRequestConfirm(mibSet);
	}
	else
	{
	    return LORAMAC_STATUS_BUSY;
	}
    }
}

static LoRaMacStatus_t LoRaWANMlmeRequest( MlmeReq_t *mlmeRequest )
{
    IRQn_Type irq;

    irq = (IRQn_Type)((__get_IPSR() & 0x1ff) - 16);

    if (irq == Reset_IRQn)
    {
	return (LoRaMacStatus_t)armv6m_svcall_1((uint32_t)&LoRaMacMlmeRequest, (uint32_t)mlmeRequest);
    }
    else
    {
	if ((irq == SVC_IRQn) || (irq == PendSV_IRQn))
	{
	    return LoRaMacMlmeRequest(mlmeRequest);
	}
	else
	{
	    return LORAMAC_STATUS_BUSY;
	}
    }
}

static LoRaMacStatus_t LoRaWANMcpsRequest( McpsReq_t *mcpsRequest )
{
    IRQn_Type irq;

    irq = (IRQn_Type)((__get_IPSR() & 0x1ff) - 16);

    if (irq == Reset_IRQn)
    {
	return (LoRaMacStatus_t)armv6m_svcall_1((uint32_t)&LoRaMacMcpsRequest, (uint32_t)mcpsRequest);
    }
    else
    {
	if ((irq == SVC_IRQn) || (irq == PendSV_IRQn))
	{
	    return LoRaMacMcpsRequest(mcpsRequest);
	}
	else
	{
	    return LORAMAC_STATUS_BUSY;
	}
    }
}

static uint8_t LoRaWANGetBatteryLevel()
{
    if (STM32L0.getVBUS()) {
	return BAT_LEVEL_EXT_SRC;
    }

#if defined(STM32L0_CONFIG_VBAT_EMPTY) && defined(STM32L0_CONFIG_VBAT_FULL)
    float vbat = STM32L0.getVBAT();

    if (vbat < STM32L0_CONFIG_VBAT_EMPTY) {
	return BAT_LEVEL_EMPTY;
    }

    if (vbat > STM32L0_CONFIG_VBAT_FULL) {
	return BAT_LEVEL_FULL;
    }

    return (uint8_t)(((vbat - STM32L0_CONFIG_VBAT_EMPTY) / (STM32L0_CONFIG_VBAT_FULL - STM32L0_CONFIG_VBAT_EMPTY)) * (BAT_LEVEL_FULL - BAT_LEVEL_EMPTY) + BAT_LEVEL_EMPTY);

#else /*  defined(STM32L0_CONFIG_VBAT_EMPTY) && defined(STM32L0_CONFIG_VBAT_FULL) */

    return BAT_LEVEL_NO_MEASURE;

#endif /*  defined(STM32L0_CONFIG_VBAT_EMPTY) && defined(STM32L0_CONFIG_VBAT_FULL) */
}

LoRaWANClass::LoRaWANClass()
{
    _initialized = false;

    _tx_port = 1;
    _tx_active = false;
    _tx_ack = 0;
    _tx_busy = false;

    _rx_index = 0;
    _rx_size = 0;
    _rx_margin = 255;
    _rx_gateways = 0;
    _rx_ack = 0;

    _JoinRetries = 3;
    _ConfirmRetries = 8;

    _PublicNetwork = true;
    _AdrEnable = true;
    _DataRate = 0;
    _TxPower = 0;

    _session.Joined = LORAWAN_JOINED_NONE;
}

LoRaMacPrimitives_t LoRaWANPrimitives;
LoRaMacCallback_t LoRaWANCallbacks;

static const LoRaMacRegion_t *LoRaWANRegions[] = {
    &LoRaMacRegionAS923,
    &LoRaMacRegionAU915,
    &LoRaMacRegionEU868,
    &LoRaMacRegionIN865,
    &LoRaMacRegionKR920,
    &LoRaMacRegionUS915,
};

static const uint8_t LoRaWANTxPowerAS923[11] = { 16, 14, 12, 10,  8,  6,  4,  2,  0,  0,  0 };
static const uint8_t LoRaWANTxPowerAU915[11] = { 30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10 };
static const uint8_t LoRaWANTxPowerEU868[11] = { 16, 14, 12, 10,  8,  6,  4,  2,  0,  0,  0 };
static const uint8_t LoRaWANTxPowerIN865[11] = { 30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10 };
static const uint8_t LoRaWANTxPowerKR920[11] = { 14, 12, 10,  8,  6,  4,  2,  0,  0,  0,  0 };
static const uint8_t LoRaWANTxPowerUS915[11] = { 30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10 };

static const uint8_t *LoRaWANTxPower[] = {
    &LoRaWANTxPowerAS923[0],
    &LoRaWANTxPowerAU915[0],
    &LoRaWANTxPowerEU868[0],
    &LoRaWANTxPowerIN865[0],
    &LoRaWANTxPowerKR920[0],
    &LoRaWANTxPowerUS915[0],
};

int LoRaWANClass::begin(LoRaWANBand band)
{
    MibRequestConfirm_t mibReq;

    if (__get_IPSR() != 0) {
	return 0;
    }

    _Band = band;

    LoRaWANPrimitives.MacMcpsConfirm = __McpsConfirm;
    LoRaWANPrimitives.MacMcpsIndication = __McpsIndication;
    LoRaWANPrimitives.MacMlmeConfirm = __MlmeConfirm;
    LoRaWANCallbacks.GetBatteryLevel = LoRaWANGetBatteryLevel;
    LoRaMacInitialization(&LoRaWANPrimitives, &LoRaWANCallbacks, LoRaWANRegions[band]);

    mibReq.Type = MIB_PUBLIC_NETWORK;
    mibReq.Param.EnablePublicNetwork = _PublicNetwork;
    LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_ADR;
    mibReq.Param.AdrEnable = _AdrEnable;
    LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_CHANNELS_DEFAULT_DATARATE;
    mibReq.Param.ChannelsDefaultDatarate = _DataRate;
    LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_CHANNELS_DATARATE;
    mibReq.Param.ChannelsDatarate = _DataRate;
    LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_CHANNELS_DEFAULT_TX_POWER;
    mibReq.Param.ChannelsDefaultTxPower = _TxPower;
    LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_CHANNELS_TX_POWER;
    mibReq.Param.ChannelsTxPower = _TxPower;
    LoRaMacMibSetRequestConfirm(&mibReq);

    _initialized = true;

    return 1;
}

void LoRaWANClass::stop()
{
    MibRequestConfirm_t mibReq;

    if (__get_IPSR() != 0) {
	return;
    }

    _session.Joined = LORAWAN_JOINED_NONE;

    mibReq.Type = MIB_NETWORK_JOINED;
    mibReq.Param.IsNetworkJoined = true;
    LoRaWANMibSetRequestConfirm(&mibReq);
}

int LoRaWANClass::joinOTAA(const char *appEui, const char *appKey, const char *devEui)
{
    MlmeReq_t mlmeReq;

    if (!_initialized) {
        return 0;
    }

    if (devEui == NULL)
    {
	BoardGetUniqueId(_session.DevEui);
    }
    else
    {
	if (!ConvertString(_session.DevEui, 8, devEui)) {
	    return 0;
	}
    }

    if (!ConvertString(_session.AppEui, 8, appEui)) {
	return 0;
    }

    if (!ConvertString(_session.AppKey, 16, appKey)) {
	return 0;
    }

    mlmeReq.Type = MLME_JOIN;
    mlmeReq.Req.Join.DevEui = _session.DevEui;
    mlmeReq.Req.Join.AppEui = _session.AppEui;
    mlmeReq.Req.Join.AppKey = _session.AppKey;
    mlmeReq.Req.Join.NbTrials = 1 + _JoinRetries;

    if (LoRaWANMlmeRequest(&mlmeReq) != LORAMAC_STATUS_OK) {
	return 0;
    }

    _session.Joined = LORAWAN_JOINED_NONE;
    _session.DataRate = _DataRate;
    _session.TxPower = _TxPower;
    _session.UpLinkCounter = 0;
    _session.DownLinkCounter = 0;

    _tx_busy = true;

    return 1;
}

int LoRaWANClass::joinABP(const char *devAddr, const char *nwkSKey, const char *appSKey)
{
    MibRequestConfirm_t mibReq;
    
    if (!_initialized) {
        return 0;
    }

    if (!ConvertString((uint8_t*)&_session.DevAddr, 4, devAddr)) {
	return 0;
    }

    if (!ConvertString(_session.NwkSKey, 16, nwkSKey)) {
	return 0;
    }

    if (!ConvertString(_session.AppSKey, 16, appSKey)) {
	return 0;
    }

    _session.Joined = LORAWAN_JOINED_ABP;
    _session.DataRate = _DataRate;
    _session.TxPower = _TxPower;
    _session.UpLinkCounter = 0;
    _session.DownLinkCounter = 0;

    memset(_session.DevEui, 0, 8);
    memset(_session.AppEui, 0, 8);
    memset(_session.AppKey, 0, 16);

    mibReq.Type = MIB_UPLINK_COUNTER;
    mibReq.Param.UpLinkCounter = _session.UpLinkCounter;
    LoRaWANMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_DOWNLINK_COUNTER;
    mibReq.Param.DownLinkCounter = _session.DownLinkCounter;
    LoRaWANMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_NET_ID;
    mibReq.Param.NetID = 0;
    LoRaWANMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_DEV_ADDR;
    mibReq.Param.DevAddr = _session.DevAddr;
    LoRaWANMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_NWK_SKEY;
    mibReq.Param.NwkSKey = _session.NwkSKey;
    LoRaWANMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_APP_SKEY;
    mibReq.Param.AppSKey = _session.AppSKey;
    LoRaWANMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_NETWORK_JOINED;
    mibReq.Param.IsNetworkJoined = true;
    LoRaWANMibSetRequestConfirm(&mibReq);

    _tx_ack = 1;
    _tx_busy = false;

    return 1;
}

bool LoRaWANClass::joined()
{
    return (_session.Joined != LORAWAN_JOINED_NONE);
}

bool LoRaWANClass::confirmed()
{
    return _tx_ack;
}

bool LoRaWANClass::checked()
{
    return _rx_ack;
}

bool LoRaWANClass::busy()
{
    return _tx_busy;
}

int LoRaWANClass::beginPacket(uint8_t port)
{
    if (_session.Joined == LORAWAN_JOINED_NONE) {
        return 0;
    }

    if (_tx_busy) {
	return 0;
    }

    if ((port < 1) || (port > 223)) {
	return 0;
    }

    _tx_size = 0;
    _tx_port = port;
    _tx_active = true;

    return 1;
}

int LoRaWANClass::endPacket(bool confirm)
{
    IRQn_Type irq;
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

    irq = (IRQn_Type)((__get_IPSR() & 0x1ff) - 16);

    if (irq >= SysTick_IRQn) {	
      return 0;
    }

    if (!_tx_active) {
	return 0;
    }

    _tx_confirm = confirm;
    _tx_ack = false;
    _tx_busy = true;

    if (_tx_size == 0)
    {
	_tx_active = false;

        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = _DataRate;

	if (LoRaWANMcpsRequest(&mcpsReq) != LORAMAC_STATUS_OK)
	{
	    _tx_busy = false;

	    return 0;
	}
    }
    else
    {
	if (LoRaWANQueryTxPossible(_tx_size, &txInfo) != LORAMAC_STATUS_OK)
	{
	    if (_tx_size > txInfo.CurrentPayloadSize)
	    {
		// Payload will never fit ...
		_tx_active = false;
		_tx_busy = false;
	    }
	    else
	    {
		// Send empty frame in order to flush MAC commands
		mcpsReq.Type = MCPS_UNCONFIRMED;
		mcpsReq.Req.Unconfirmed.fBuffer = NULL;
		mcpsReq.Req.Unconfirmed.fBufferSize = 0;
		mcpsReq.Req.Unconfirmed.Datarate = _DataRate;
		
		if (LoRaWANMcpsRequest(&mcpsReq) != LORAMAC_STATUS_OK)
		{
		    _tx_active = false;
		    _tx_busy = false;
		}
	    }

	    return 0;
	}
	else
	{
	    _tx_active = false;

	    if (_tx_confirm)
	    {
		mcpsReq.Type = MCPS_CONFIRMED;
		mcpsReq.Req.Confirmed.fPort = _tx_port;
		mcpsReq.Req.Confirmed.fBuffer = &_tx_data[0];
		mcpsReq.Req.Confirmed.fBufferSize = _tx_size;
		mcpsReq.Req.Confirmed.NbTrials = 1 + _ConfirmRetries;
		mcpsReq.Req.Confirmed.Datarate = _DataRate;
	    }
	    else
	    {
		mcpsReq.Type = MCPS_UNCONFIRMED;
		mcpsReq.Req.Unconfirmed.fPort = _tx_port;
		mcpsReq.Req.Unconfirmed.fBuffer = &_tx_data[0];
		mcpsReq.Req.Unconfirmed.fBufferSize = _tx_size;
		mcpsReq.Req.Unconfirmed.Datarate = _DataRate;
	    }

	    if (LoRaWANMcpsRequest(&mcpsReq) != LORAMAC_STATUS_OK)
	    {
		_tx_busy = false;
	    
		return 0;
	    }
	}
    }

    return 1;
}

int LoRaWANClass::sendPacket(const uint8_t *buffer, size_t size, bool confirm)
{
    return sendPacket(LORAWAN_DEFAULT_PORT, buffer, size, confirm);
}

int LoRaWANClass::sendPacket(uint8_t port, const uint8_t *buffer, size_t size, bool confirm)
{
    if (_session.Joined == LORAWAN_JOINED_NONE) {
        return 0;
    }

    if (_tx_busy) {
	return 0;
    }

    if (_tx_active) {
	return 0;
    }

    if ((port < 1) || (port > 223)) {
	return 0;
    }

    if (size > LORAWAN_MAX_PAYLOAD_SIZE) {
	return 0;
    }

    _tx_port = port;
    _tx_size = size;

    memcpy(&_tx_data[0], buffer, size);

    _tx_active = true;

    return endPacket(confirm);
}

int LoRaWANClass::availableForWrite()
{
    if (!_tx_active) {
	return 0;
    }

    return LORAWAN_MAX_PAYLOAD_SIZE - _tx_size;
}

size_t LoRaWANClass::write(uint8_t data)
{
    if (!_tx_active) {
	return 0;
    }

    if (_tx_size >= LORAWAN_MAX_PAYLOAD_SIZE) {
	return 0;
    }

    _tx_data[_tx_size++] = data;

    return 1;
}

size_t LoRaWANClass::write(const uint8_t *data, size_t size)
{
    if (!_tx_active) {
	return 0;
    }

    if (size > (unsigned int)(LORAWAN_MAX_PAYLOAD_SIZE - _tx_size)) {
	size = LORAWAN_MAX_PAYLOAD_SIZE - _tx_size;
    }

    memcpy(&_tx_data[_tx_size], data, size);

    _tx_size += size;

    return size;
}

int LoRaWANClass::parsePacket()
{
    uint32_t rx_read;

    if (_rx_size) {
	_rx_index = _rx_next;
	_rx_read = _rx_next;
	_rx_size = 0;
	_rx_port = 0;
	_rx_multicast = 0;
    }

    rx_read = _rx_read;

    if (rx_read == _rx_write) {
	return 0;
    }

    _rx_size = _rx_data[rx_read];

    if (++rx_read == LORAWAN_RX_BUFFER_SIZE) {
	rx_read = 0;
    }

    _rx_port = _rx_data[rx_read];

    if (++rx_read == LORAWAN_RX_BUFFER_SIZE) {
	rx_read = 0;
    }

    _rx_multicast = _rx_data[rx_read];

    if (++rx_read == LORAWAN_RX_BUFFER_SIZE) {
	rx_read = 0;
    }

    _rx_index = rx_read;

    rx_read += _rx_size;

    if (rx_read >= LORAWAN_RX_BUFFER_SIZE) {
	rx_read -= LORAWAN_RX_BUFFER_SIZE;
    }

    _rx_next = rx_read;

    return _rx_size;
}

int LoRaWANClass::available()
{
    if (_rx_next >= _rx_index) {
	return (_rx_next - _rx_index);
    } else {
	return (_rx_next + (LORAWAN_RX_BUFFER_SIZE - _rx_index));
    }
}

int LoRaWANClass::read()
{
    int data;
    
    if (_rx_index == _rx_next) {
	return -1;
    }

    data = _rx_data[_rx_index++];

    if (_rx_index == LORAWAN_RX_BUFFER_SIZE) {
	_rx_index = 0;
    }

    return data;
}

size_t LoRaWANClass::read(uint8_t *buffer, size_t size)
{
    size_t count;

    if (_rx_index == _rx_next) {
	return 0;
    }

    if (_rx_next < _rx_index)
    {
	count = size;

	if (count > (size_t)(LORAWAN_RX_BUFFER_SIZE - _rx_index))
	{
	    count = LORAWAN_RX_BUFFER_SIZE - _rx_index;
	}

	if (count)
	{
	    memcpy(buffer, &_rx_data[_rx_index], count);
	    
	    buffer += count;
	    size -= count;
	    
	    _rx_index += count;
	    
	    if (_rx_index == LORAWAN_RX_BUFFER_SIZE) {
		_rx_index = 0;
	    }
	}
    }
    else
    {
	count = 0;
    }

    if (size)
    {
	if (size > (size_t)(_rx_next - _rx_index)) {
	    size = _rx_next - _rx_index;
	}

	if (size)
	{
	    memcpy(buffer, &_rx_data[_rx_index], size);
	    
	    buffer += size;
	    count += size;
	    
	    _rx_index += size;
	}
    }

    return count;
}

int LoRaWANClass::peek(void)
{
    if (_rx_index == _rx_next) {
	return -1;
    }

    return _rx_data[_rx_index];
}

void LoRaWANClass::flush()
{
}

uint8_t LoRaWANClass::remotePort()
{
    return _rx_port;
}

int LoRaWANClass::lastRSSI()
{
    return _rx_rssi;
}

int LoRaWANClass::lastSNR()
{
    return _rx_snr;
}

void LoRaWANClass::linkCheck()
{
    MlmeReq_t mlmeReq;

    if (_session.Joined == LORAWAN_JOINED_NONE) {
        return;
    }

    _rx_ack = false;

    mlmeReq.Type = MLME_LINK_CHECK;
    LoRaWANMlmeRequest( &mlmeReq );
}

int LoRaWANClass::linkMargin()
{
    if (!_rx_ack) {
	return -1;
    }

    return _rx_margin;
}

int LoRaWANClass::linkGateways()
{
    if (!_rx_ack) {
	return -1;
    }

    return _rx_gateways;
}

void LoRaWANClass::onJoin(void(*callback)(void))
{
    _joinNotify = Notifier(callback);
}

void LoRaWANClass::onJoin(Notifier notify)
{
    _joinNotify = notify;
}

void LoRaWANClass::onReceive(void(*callback)(void))
{
    _receiveNotify = Notifier(callback);
}

void LoRaWANClass::onReceive(Notifier notify)
{
    _receiveNotify = notify;
}

void LoRaWANClass::onTransmit(void(*callback)(void))
{
    _transmitNotify = Notifier(callback);
}

void LoRaWANClass::onTransmit(Notifier notify)
{
    _transmitNotify = notify;
}

int LoRaWANClass::getDevEui(char *buffer, size_t size)
{
    uint8_t DevEui[8];
    unsigned int i;
    
    static const char xlate[17] = "0123456789abcdef";

    if (size < 17) {
	return 0;
    }

    BoardGetUniqueId(DevEui);

    for (i = 0; i < 8; i++)
    {
	*buffer++ = xlate[DevEui[i] >> 4];
	*buffer++ = xlate[DevEui[i] & 15];
    }

    *buffer++ = '\0';

    return 1;
}

int LoRaWANClass::getMaxPayloadSize()
{
    LoRaMacTxInfo_t txInfo;

    LoRaWANQueryTxPossible(0, &txInfo);

    return txInfo.CurrentPayloadSize;
}

unsigned int LoRaWANClass::getDataRate()
{
    return _session.DataRate;
}

unsigned int LoRaWANClass::getTxPower()
{
    return LoRaWANTxPower[_Band][_session.TxPower];
}

unsigned long LoRaWANClass::getUpLinkCounter()
{
    return _session.UpLinkCounter;
}

unsigned long LoRaWANClass::getDownLinkCounter()
{
    return _session.DownLinkCounter;
}

int LoRaWANClass::setJoinDelay1(unsigned int delay)
{
    MibRequestConfirm_t mibReq;

    if (!_initialized) {
	return 0;
    }

    if (_tx_busy) {
	return 0;
    }

    if (delay > 65535) {
	return 0;
    }

    mibReq.Type = MIB_JOIN_ACCEPT_DELAY_1;
    mibReq.Param.JoinAcceptDelay1 = delay;

    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
	return 0;
    }

    return 1;
}

int LoRaWANClass::setJoinDelay2(unsigned int delay)
{
    MibRequestConfirm_t mibReq;

    if (!_initialized) {
	return 0;
    }

    if (_tx_busy) {
	return 0;
    }

    if (delay > 65535) {
	return 0;
    }

    mibReq.Type = MIB_JOIN_ACCEPT_DELAY_2;
    mibReq.Param.JoinAcceptDelay2 = delay;

    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
	return 0;
    }

    return 1;
}

int LoRaWANClass::setJoinRetries(unsigned int n)
{
    if (!_initialized) {
	return 0;
    }

    if (n > 255) {
	return 0;
    }

    _JoinRetries = n;

    return 1;
}

int LoRaWANClass::setPublicNetwork(bool enable)
{
    MibRequestConfirm_t mibReq;

    if (!_initialized) {
	return 0;
    }

    if (_tx_busy) {
	return 0;
    }
    
    mibReq.Type = MIB_PUBLIC_NETWORK;
    mibReq.Param.EnablePublicNetwork = enable;
    
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK){
	return 0;
    }

    _PublicNetwork = enable;

    return 1;
}

int LoRaWANClass::setADR(bool enable)
{
    MibRequestConfirm_t mibReq;

    if (!_initialized) {
	return 0;
    }

    if (_tx_busy) {
	return 0;
    }
    
    mibReq.Type = MIB_ADR;
    mibReq.Param.AdrEnable = enable;
    
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
	return 0;
    }

    if (!enable) {
	mibReq.Type = MIB_CHANNELS_DATARATE;
	mibReq.Param.ChannelsDatarate = _DataRate;
	LoRaWANMibSetRequestConfirm(&mibReq);

	mibReq.Type = MIB_CHANNELS_TX_POWER;
	mibReq.Param.ChannelsTxPower = _TxPower;
	LoRaWANMibSetRequestConfirm(&mibReq);
    }

    _AdrEnable = enable;

    return 1;
}

int LoRaWANClass::setDataRate(unsigned int datarate)
{
    MibRequestConfirm_t mibReq;

    if (!_initialized) {
	return 0;
    }

    if (_tx_busy) {
	return 0;
    }

    mibReq.Type = MIB_CHANNELS_DEFAULT_DATARATE;
    mibReq.Param.ChannelsDefaultDatarate = datarate;
	
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
	return 0;
    }

    if (!_AdrEnable || (_session.Joined == LORAWAN_JOINED_NONE))
    {
	mibReq.Type = MIB_CHANNELS_DATARATE;
	mibReq.Param.ChannelsDatarate = datarate;
	
	if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
	    return 0;
	}
    }

    _DataRate = datarate;

    return 1;
}

int LoRaWANClass::setTxPower(unsigned int power)
{
    unsigned int txPower;
    MibRequestConfirm_t mibReq;

    if (!_initialized) {
	return 0;
    }

    if (_tx_busy) {
	return 0;
    }
    
    for (txPower = 10; txPower > 0; txPower--) {
	if (LoRaWANTxPower[_Band][txPower] >= power) {
	    break;
	}
    }

    mibReq.Type = MIB_CHANNELS_DEFAULT_TX_POWER;
    mibReq.Param.ChannelsDefaultTxPower = txPower;
    
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
	return 0;
    }

    if (!_AdrEnable || (_session.Joined == LORAWAN_JOINED_NONE))
    {
	mibReq.Type = MIB_CHANNELS_TX_POWER;
	mibReq.Param.ChannelsTxPower = txPower;
	
	if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
	    return 0;
	}
    }

    _TxPower = txPower;

    return 1;
}

int LoRaWANClass::setConfirmRetries(unsigned int n)
{
    if (!_initialized) {
	return 0;
    }

    if (n > 255) {
	return 0;
    }

    _ConfirmRetries = n;

    return 1;
}

int LoRaWANClass::setReceiveDelay(unsigned int delay)
{
    MibRequestConfirm_t mibReq;

    if (!_initialized) {
	return 0;
    }

    if (_tx_busy) {
	return 0;
    }

    if (delay > 65535) {
	return 0;
    }

    mibReq.Type = MIB_RECEIVE_DELAY_1;
    mibReq.Param.ReceiveDelay1 = delay;

    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
	return 0;
    }

    mibReq.Type = MIB_RECEIVE_DELAY_2;
    mibReq.Param.ReceiveDelay2 = delay + 1000;

    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
	return 0;
    }

    return 1;
}

int LoRaWANClass::setRX2Channel(unsigned long frequency, unsigned int datarate)
{
    MibRequestConfirm_t mibReq;

    if (!_initialized) {
	return 0;
    }

    if (_tx_busy) {
	return 0;
    }

    mibReq.Type = MIB_RX2_CHANNEL;
    mibReq.Param.Rx2Channel.Frequency = frequency;
    mibReq.Param.Rx2Channel.Datarate = datarate;
    
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
	return 0;
    }
    
    return 1;
}

int LoRaWANClass::setSubBand(unsigned int subband)
{
    uint16_t ChannelsMask[LORA_MAX_NB_CHANNELS / 16];
    unsigned int group;
    MibRequestConfirm_t mibReq;
    
    if (!_initialized) {
	return 0;
    }

    if (_tx_busy) {
	return 0;
    }

    if (!((_Band == AU915) || (_Band == US915))) {
	return 0;
    }

    if (subband > 8) {
	return 0;
    }

    memset(&ChannelsMask[0], 0, sizeof(ChannelsMask));

    if (subband > 0)
    {
	group = subband -1;

	ChannelsMask[group >> 1] = ((group & 1) ? 0xFF00 : 0x00FF);
	ChannelsMask[4] = (0x0001 << group);
    }
    else
    {
	ChannelsMask[0] = 0xFFFF;
	ChannelsMask[1] = 0xFFFF;
	ChannelsMask[2] = 0xFFFF;
	ChannelsMask[3] = 0xFFFF;
	ChannelsMask[4] = 0x00FF;
	ChannelsMask[5] = 0x0000;
    }

    mibReq.Type = MIB_CHANNELS_DEFAULT_MASK;
    mibReq.Param.ChannelsDefaultMask = ChannelsMask;

    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
	return 0;
    }

    mibReq.Type = MIB_CHANNELS_MASK;
    mibReq.Param.ChannelsMask = ChannelsMask;

    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
	return 0;
    }

    return 1;
}

int LoRaWANClass::addChannel(unsigned int index, unsigned long frequency, unsigned int drMin, unsigned int drMax)
{
    ChannelParams_t params;

    if (!_initialized) {
	return 0;
    }

    if ((_Band == AU915) || (_Band == US915)) {
	return 0;
    }

    // AS923: index == 0..15
    // EU868: index == 0..15
    // IN868: index == 0..15
    // KR920: index == 0..15

    if (index > 15) {
	return 0;
    }

    if (_tx_busy) {
	return 0;
    }

    params.Frequency = frequency;
    params.Rx1Frequency = 0;
    params.DrRange.Fields.Min = drMin;
    params.DrRange.Fields.Max = drMax;
    params.Band = 0;

    if (LoRaWANChannelAdd(index, &params) != LORAMAC_STATUS_OK) {
	return 0;
    }

    return 1;
}

int LoRaWANClass::removeChannel(unsigned int index)
{
    if (!_initialized) {
	return 0;
    }

    if ((_Band == AU915) || (_Band == US915)) {
	return 0;
    }

    // AS923: index == 0..15
    // EU868: index == 0..15
    // IN868: index == 0..15
    // KR920: index == 0..15

    if (index > 15) {
	return 0;
    }

    if (_tx_busy) {
	return 0;
    }

    if (LoRaWANChannelRemove(index) != LORAMAC_STATUS_OK) {
	return 0;
    }

    return 1;
}

int LoRaWANClass::enableChannel(unsigned int index)
{
    uint16_t ChannelsMask[LORA_MAX_NB_CHANNELS / 16];
    MibRequestConfirm_t mibReq;

    if (!_initialized) {
	return 0;
    }

    if (_tx_busy) {
	return 0;
    }

    // AS923: index == 0..15
    // AU915: index == 0..71
    // EU868: index == 0..15
    // IN868: index == 0..15
    // KR920: index == 0..15
    // US915: index == 0..71

    if ((_Band == AU915) || (_Band == US915)) {
	if (index > 71)	{
	    return 0;
	}
    } else {
	if (index > 15)	{
	    return 0;
	}
    }

    mibReq.Type = MIB_CHANNELS_MASK;

    if (LoRaWANMibGetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
	return 0;
    }
    
    memcpy(&ChannelsMask[0], mibReq.Param.ChannelsMask, sizeof(ChannelsMask));

    ChannelsMask[index >> 4] |= (1u << (index & 15));

    mibReq.Type = MIB_CHANNELS_MASK;
    mibReq.Param.ChannelsMask = ChannelsMask;

    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
	return 0;
    }

    return 1;
}

int LoRaWANClass::disableChannel(unsigned int index)
{
    uint16_t ChannelsMask[LORA_MAX_NB_CHANNELS / 16];
    MibRequestConfirm_t mibReq;

    if (!_initialized) {
	return 0;
    }

    if (_tx_busy) {
	return 0;
    }

    // AS923: index == 0..15
    // AU915: index == 0..71
    // EU868: index == 0..15
    // IN868: index == 0..15
    // KR920: index == 0..15
    // US915: index == 0..71

    if ((_Band == AU915) || (_Band == US915)) {
	if (index > 71)	{
	    return 0;
	}
    } else {
	if (index > 15)	{
	    return 0;
	}
    }

    mibReq.Type = MIB_CHANNELS_MASK;

    if (LoRaWANMibGetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
	return 0;
    }
    
    memcpy(&ChannelsMask[0], mibReq.Param.ChannelsMask, sizeof(ChannelsMask));

    ChannelsMask[index >> 4] &= ~(1u << (index & 15));

    mibReq.Type = MIB_CHANNELS_MASK;
    mibReq.Param.ChannelsMask = ChannelsMask;

    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
	return 0;
    }

    return 1;
}

int LoRaWANClass::setAntennaGain(float gain)
{
    MibRequestConfirm_t mibReq;

    if (!_initialized) {
	return 0;
    }

    if (_tx_busy) {
	return 0;
    }

    mibReq.Type = MIB_ANTENNA_GAIN;
    mibReq.Param.AntennaGain = gain;

    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
	return 0;
    }

    return 1;
}

int LoRaWANClass::setDutyCycle(bool enable)
{
    if (!_initialized) {
	return 0;
    }

    if (_tx_busy) {
	return 0;
    }

    if (!((_Band == EU868) || (_Band == IN865))) {
	return 0;
    }

    LoRaMacTestSetDutyCycleOn(enable);

    return 1;
}

int LoRaWANClass::setReceiveWindows(bool enable)
{
    if (!_initialized) {
	return 0;
    }

    if (_tx_busy) {
	return 0;
    }

    LoRaMacTestRxWindowsOn(enable);

    return 1;
}

void LoRaWANClass::__McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    McpsReq_t mcpsReq;
    MibRequestConfirm_t mibReq;

    if (mcpsConfirm->Status != LORAMAC_EVENT_INFO_STATUS_OK) {
	return;
    }

    LoRaWAN._session.DataRate = mcpsConfirm->Datarate;
    LoRaWAN._session.TxPower = mcpsConfirm->TxPower;
    LoRaWAN._session.UpLinkCounter = mcpsConfirm->UpLinkCounter;

    switch (mcpsConfirm->McpsRequest) {
    case MCPS_UNCONFIRMED:
	if (LoRaWAN._tx_active)
	{
	    LoRaWAN._tx_active = false;

	    if (LoRaWAN._tx_confirm)
	    {
		mcpsReq.Type = MCPS_CONFIRMED;
		mcpsReq.Req.Confirmed.fPort = LoRaWAN._tx_port;
		mcpsReq.Req.Confirmed.fBuffer = &LoRaWAN._tx_data[0];
		mcpsReq.Req.Confirmed.fBufferSize = LoRaWAN._tx_size;
		mcpsReq.Req.Confirmed.NbTrials = 1 + LoRaWAN._ConfirmRetries;
		mcpsReq.Req.Confirmed.Datarate = LoRaWAN._DataRate;
	    }
	    else
	    {
		mcpsReq.Type = MCPS_UNCONFIRMED;
		mcpsReq.Req.Unconfirmed.fPort = LoRaWAN._tx_port;
		mcpsReq.Req.Unconfirmed.fBuffer = &LoRaWAN._tx_data[0];
		mcpsReq.Req.Unconfirmed.fBufferSize = LoRaWAN._tx_size;
		mcpsReq.Req.Unconfirmed.Datarate = LoRaWAN._DataRate;
	    }

	    if (LoRaWANMcpsRequest(&mcpsReq) != LORAMAC_STATUS_OK) {
		LoRaWAN._tx_busy = false;
	    }
	}
	else
	{
	    LoRaWAN._tx_busy = false;
	}
	break;

    case MCPS_CONFIRMED:
	LoRaWAN._tx_ack = mcpsConfirm->AckReceived ? mcpsConfirm->NbRetries : 0;
	LoRaWAN._tx_busy = false;
	break;

    case MCPS_PROPRIETARY:
	break;

    default:
	break;
    }

    if (!LoRaWAN._tx_busy)
    {
	// late LoRaWAN::stop()
	if (LoRaWAN._session.Joined == LORAWAN_JOINED_NONE)
	{
	    mibReq.Type = MIB_NETWORK_JOINED;
	    mibReq.Param.IsNetworkJoined = true;
	    LoRaMacMibSetRequestConfirm(&mibReq);
	}

	LoRaWAN._transmitNotify.queue();
    }
}

void LoRaWANClass::__McpsIndication( McpsIndication_t *mcpsIndication )
{
    uint32_t rx_write, rx_size;

    if (mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK) {
        return;
    }

    switch (mcpsIndication->McpsIndication) {
    case MCPS_UNCONFIRMED:
	break;

    case MCPS_CONFIRMED:
	break;

    case MCPS_PROPRIETARY:
	break;

    case MCPS_MULTICAST:
	break;

    default:
	break;
    }

    LoRaWAN._session.DownLinkCounter = mcpsIndication->DownLinkCounter;

    LoRaWAN._rx_snr = mcpsIndication->Snr;
    LoRaWAN._rx_rssi = mcpsIndication->Rssi;

    if (mcpsIndication->RxData == true)
    {
	rx_write = LoRaWAN._rx_write;

	if (rx_write < LoRaWAN._rx_read) {
	    rx_size = LoRaWAN._rx_read - rx_write -1;
	} else {
	    rx_size = (LORAWAN_RX_BUFFER_SIZE - rx_write) + LoRaWAN._rx_read -1;
	}

	if (rx_size >= (unsigned int)(3 + mcpsIndication->BufferSize))
	{
	    LoRaWAN._rx_data[rx_write] = mcpsIndication->BufferSize;
	    
	    if (++rx_write == LORAWAN_RX_BUFFER_SIZE) {
		rx_write = 0;
	    }

	    LoRaWAN._rx_data[rx_write] = mcpsIndication->Port;

	    if (++rx_write == LORAWAN_RX_BUFFER_SIZE) {
		rx_write = 0;
	    }

	    LoRaWAN._rx_data[rx_write] = mcpsIndication->Multicast;

	    if (++rx_write == LORAWAN_RX_BUFFER_SIZE) {
		rx_write = 0;
	    }

	    rx_size = mcpsIndication->BufferSize;

	    if (rx_size > (LORAWAN_RX_BUFFER_SIZE - rx_write)) {
		rx_size = LORAWAN_RX_BUFFER_SIZE - rx_write;
	    }

	    if (rx_size) {
		memcpy(&LoRaWAN._rx_data[rx_write], mcpsIndication->Buffer, rx_size);

		rx_write += rx_size;

		if (rx_write >= LORAWAN_RX_BUFFER_SIZE) {
		    rx_write -= LORAWAN_RX_BUFFER_SIZE;
		}
	    }

	    if (rx_size != mcpsIndication->BufferSize) {
		memcpy(&LoRaWAN._rx_data[rx_write], mcpsIndication->Buffer + rx_size, mcpsIndication->BufferSize - rx_size);

		rx_write += (mcpsIndication->BufferSize - rx_size);
	    }

	    LoRaWAN._rx_write = rx_write;
	}
    }

    LoRaWAN._receiveNotify.queue();
}

void LoRaWANClass::__MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    MibRequestConfirm_t mibReq;

    switch (mlmeConfirm->MlmeRequest) {
    case MLME_JOIN:
	if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)
	{
	    mibReq.Type = MIB_DEV_ADDR;
	    LoRaMacMibGetRequestConfirm(&mibReq);
	    LoRaWAN._session.DevAddr = mibReq.Param.DevAddr;

	    mibReq.Type = MIB_NWK_SKEY;
	    LoRaMacMibGetRequestConfirm(&mibReq);
	    memcpy(LoRaWAN._session.NwkSKey, mibReq.Param.NwkSKey, 16);

	    mibReq.Type = MIB_APP_SKEY;
	    LoRaMacMibGetRequestConfirm(&mibReq);
	    memcpy(LoRaWAN._session.AppSKey, mibReq.Param.AppSKey, 16);

	    LoRaWAN._session.Joined = LORAWAN_JOINED_OTAA;

	    LoRaWAN._tx_busy = false;
	}
	else
	{
	    LoRaWAN._tx_busy = false;
	}

	LoRaWAN._joinNotify.queue();
	break;

    case MLME_LINK_CHECK:
	if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)
	{
	    LoRaWAN._rx_margin = mlmeConfirm->DemodMargin;
	    LoRaWAN._rx_gateways = mlmeConfirm->NbGateways;
	    LoRaWAN._rx_ack = true;
	}
	break;

    default:
	break;
    }
}

LoRaWANClass LoRaWAN;
