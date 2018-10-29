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
#include "wiring_private.h"

extern "C" {
#include "LoRaMac.h"
#include "LoRaMacTest.h"
extern uint32_t LoRaMacState;
extern uint32_t LoRaMacTxDelay;
extern LoRaMacFlags_t LoRaMacFlags;
extern LoRaMacParams_t LoRaMacParams;
extern LoRaMacParams_t LoRaMacParamsDefaults;
}

/* In LoRaWAN 1.0.2 identical for all regions */
#define ADR_ACK_LIMIT                  64
#define ADR_ACK_DELAY                  32

#if defined(DATA_EEPROM_BANK2_END)
#define EEPROM_OFFSET_START            ((((DATA_EEPROM_BANK2_END - DATA_EEPROM_BASE) + 1023) & ~1023) - 1024)
#else
#define EEPROM_OFFSET_START            ((((DATA_EEPROM_END - DATA_EEPROM_BASE) + 1023) & ~1023) - 1024)
#endif

#define EEPROM_OFFSET_COMMISSIONING    (EEPROM_OFFSET_START + 0)
#define EEPROM_OFFSET_SESSION          (EEPROM_OFFSET_START + 128)
#define EEPROM_OFFSET_PARAMS           (EEPROM_OFFSET_START + 256)
#define EEPROM_OFFSET_DEVNONCE         (EEPROM_OFFSET_START + 512)
#define EEPROM_OFFSET_UPLINK_COUNTER   (EEPROM_OFFSET_START + 512 + 128)
#define EEPROM_OFFSET_DOWNLINK_COUNTER (EEPROM_OFFSET_START + 512 + 256)
#define EEPROM_OFFSET_RESERVED         (EEPROM_OFFSET_START + 512 + 384)

#define EEPROM_SIZE_COMMISSIONING      128
#define EEPROM_SIZE_SESSION            128
#define EEPROM_SIZE_PARAMS             256
#define EEPROM_SIZE_DEVNONCE           128
#define EEPROM_SIZE_UPLINK_COUNTER     128
#define EEPROM_SIZE_DOWNLINK_COUNTER   128
#define EEPROM_SIZE_RESERVED           128

#define EEPROM_HEADER_COMMISSIONING    ((0x1000 << 16) | sizeof(LoRaWANCommissioning))
#define EEPROM_HEADER_SESSION          ((0x2000 << 16) | sizeof(LoRaWANSession))
#define EEPROM_HEADER_PARAMS           ((0x3000 << 16) | sizeof(LoRaWANParams))

#define EEPROM_COUNTER_UPDATE_PERIOD   128 /* 128 * 32 * 100k updates */

#define RTC ((RTC_TypeDef *) RTC_BASE)

#define BKP2R_UPLINK_COUNTER_SHIFT     0
#define BKP2R_UPLINK_COUNTER_MASK      0x000000ff
#define BKP2R_DOWNLINK_COUNTER_SHIFT   8
#define BKP2R_DOWNLINK_COUNTER_MASK    0x0000ff00
#define BKP2R_DATARATE_SHIFT           16
#define BKP2R_DATARATE_MASK            0x000f0000
#define BKP2R_TX_POWER_SHIFT           20
#define BKP2R_TX_POWER_MASK            0x00f00000
#define BKP2R_REPEAT_SHIFT             24
#define BKP2R_REPEAT_MASK              0x0f000000
#define BKP2R_ADR_ENABLE_SHIFT         28
#define BKP2R_ADR_ENABLE_MASK          0x10000000
#define BKP2R_UPLINK_COUNTER_PRESENT   0x20000000
#define BKP2R_DOWNLINK_COUNTER_PRESENT 0x40000000
#define BKP2R_ADR_PRESENT              0x80000000

static stm32l0_eeprom_transaction_t EEPROMTransaction;
static uint32_t EEPROMSessionCRC32 = 0;
static uint32_t EEPROMParamsCRC32 = 0;
static uint32_t EEPROMDevNonce = 0;
static uint32_t EEPROMUpLinkCounter = 0;
static uint32_t EEPROMDownLinkCounter = 0;

static uint32_t crc32(const uint8_t *data, uint32_t size, uint32_t crc)
{
    uint8_t c;

    static const uint32_t lut[16] = {
        0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
        0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
        0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
        0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c,
    };

    while (size--)
    {
        c = *data++;

        crc = (crc >> 4) ^ lut[(crc ^ c       ) & 15];
        crc = (crc >> 4) ^ lut[(crc ^ (c >> 4)) & 15];
    }

    return crc;
}

#define LORAWAN_COMPLIANCE_TEST

#if defined(LORAWAN_COMPLIANCE_TEST)

static struct {
    uint8_t             Running;
    uint8_t             LinkCheck;
    uint8_t             DemodMargin;
    uint8_t             NbGateways;
    uint16_t            DownLinkCounter;
    uint16_t            UpLinkCounter;
    uint8_t             IsConfirmed;
    uint8_t             TxSize;
    uint8_t             *TxData;
    stm32l0_rtc_timer_t Timer;
} ComplianceTest;

static void ComplianceTestCallback(void)
{
    McpsReq_t mcpsReq;
    unsigned int txSize;
    uint8_t *txData, txBuffer[3];

    if (ComplianceTest.Running)
    {
	if (ComplianceTest.LinkCheck)
	{
	    txSize = 3;
	    txData = &txBuffer[0];
	
	    txData[0] = 5;
	    txData[1] = ComplianceTest.DemodMargin;
	    txData[2] = ComplianceTest.NbGateways;
	}
	else
	{
	    if (ComplianceTest.TxSize)
	    {
		txSize = ComplianceTest.TxSize;
		txData = &ComplianceTest.TxData[0];
	    
		ComplianceTest.TxSize = 0;
	    }
	    else
	    {
		txSize = 2;
		txData = &txBuffer[0];
	    
		txData[0] = ComplianceTest.DownLinkCounter >> 8;
		txData[1] = ComplianceTest.DownLinkCounter & 0xFF;
	    }
	}

	if (ComplianceTest.IsConfirmed)
	{
	    mcpsReq.Type = MCPS_CONFIRMED;
	    mcpsReq.Req.Confirmed.fPort = 224;
	    mcpsReq.Req.Confirmed.fBuffer = txData;
	    mcpsReq.Req.Confirmed.fBufferSize = txSize;
	    mcpsReq.Req.Confirmed.NbTrials = 8;
	    mcpsReq.Req.Confirmed.Datarate = 0;
	}
	else
	{
	    mcpsReq.Type = MCPS_UNCONFIRMED;
	    mcpsReq.Req.Unconfirmed.fPort = 224;
	    mcpsReq.Req.Unconfirmed.fBuffer = txData;
	    mcpsReq.Req.Unconfirmed.fBufferSize = txSize;
	    mcpsReq.Req.Unconfirmed.Datarate = 0;
	}

	LoRaMacMcpsRequest(&mcpsReq);

	// RESTART TIMER
	stm32l0_rtc_timer_start(&ComplianceTest.Timer, 5, 0, false);
    }
}

#endif /* LORAWAN_COMPLIANCE_TEST */

static uint32_t LoRaWANBuffer[(LORAWAN_TX_BUFFER_SIZE + 3) / 4 + (LORAWAN_RX_BUFFER_SIZE + 3) / 4];

struct LoRaWANBand {
    uint8_t Region;
    uint8_t Channels;
    uint8_t JoinRetries;
    uint8_t DutyCycle;
    const LoRaMacRegion_t *LoRaMacRegion;
};

const struct LoRaWANBand AS923 = {
    LORAWAN_REGION_AS923,
    16,
    1,
    false,
    &LoRaMacRegionAS923,
};

const struct LoRaWANBand AU915 = {
    LORAWAN_REGION_AU915,
    72,
    71,
    false,
    &LoRaMacRegionAU915,
};

const struct LoRaWANBand CN470 = {
    LORAWAN_REGION_CN470,
    96,
    95,
    false,
    &LoRaMacRegionCN470,
};

const struct LoRaWANBand CN779 = {
    LORAWAN_REGION_CN779,
    16,
    2,
    true,
    &LoRaMacRegionCN779,
};

const struct LoRaWANBand EU433 = {
    LORAWAN_REGION_EU433,
    16,
    2,
    true,
    &LoRaMacRegionEU433,
};

const struct LoRaWANBand EU868 = {
    LORAWAN_REGION_EU868,
    16,
    2,
    true,
    &LoRaMacRegionEU868,
};

const struct LoRaWANBand IN865 = {
    LORAWAN_REGION_IN865,
    16,
    2,
    true,
    &LoRaMacRegionIN865,
};

const struct LoRaWANBand KR920 = {
    LORAWAN_REGION_KR920,
    16,
    2,
    false,
    &LoRaMacRegionKR920,
};

const struct LoRaWANBand US915 = {
    LORAWAN_REGION_US915,
    72,
    71,
    false,
    &LoRaMacRegionUS915,
};


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

static LoRaMacStatus_t LoRaWANQueryTxPossible( uint8_t size, int8_t datarate, LoRaMacTxInfo_t* txInfo )
{
    IRQn_Type irq;

    irq = (IRQn_Type)((__get_IPSR() & 0x1ff) - 16);

    if (irq == Reset_IRQn)
    {
        return (LoRaMacStatus_t)armv6m_svcall_3((uint32_t)&LoRaMacQueryTxPossible, (uint32_t)size, (uint32_t)datarate, (uint32_t)txInfo);
    } else
    {
        if ((irq == SVC_IRQn) || (irq == PendSV_IRQn))
        {
            return LoRaMacQueryTxPossible(size, datarate, txInfo);
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

LoRaWANClass::LoRaWANClass()
{
    _Band = NULL;
    _Joined = false;
    _Save = false;
    _BatteryLevel = LORAWAN_BATTERY_LEVEL_UNKNOWN;

    _tx_data = (uint8_t*)&LoRaWANBuffer[0];
    _tx_active = false;
    _tx_join = false;
    _tx_pending = false;
    _tx_ack = false;
    _tx_busy = false;

    _rx_data = (uint8_t*)&LoRaWANBuffer[(LORAWAN_TX_BUFFER_SIZE + 3) / 4];
    _rx_index = 0;
    _rx_size = 0;
    _rx_pending = false;

    _JoinRetries = 0;

    _AdrEnable = true;
    _AdrWait = 0;
    _AdrLastDataRate = 0;
    _AdrLastTxPower = 0;

    _LinkCheckLimit = 0;
    _LinkCheckDelay = 8;
    _LinkCheckThreshold = 10;
    _LinkCheckCount = 0;
    _LinkCheckWait = 0;
    _LinkCheckFail = 0;
    _LinkCheckMargin = 0;
    _LinkCheckGateways = 0;

    _ComplianceTest = true;

    _PublicNetwork = 0;
    _SubBand = 0;
    _DutyCycle = true;

    _DataRate = 0;
    _Retries = 7;

    _SNR = 0;
    _RSSI = 0;
    _DevNonce = 0;
    _TimeOnAir = 0;
    _UpLinkCounter = 0;
    _DownLinkCounter = 0;

    EEPROMTransaction.status = STM32L0_EEPROM_STATUS_NONE;
    EEPROMTransaction.callback = (stm32l0_eeprom_done_callback_t)LoRaWANClass::_eepromSync;
    EEPROMTransaction.context = (void*)this;

#if defined(LORAWAN_COMPLIANCE_TEST)
    ComplianceTest.Running = false;

    stm32l0_rtc_timer_create(&ComplianceTest.Timer, (stm32l0_rtc_timer_callback_t)ComplianceTestCallback, NULL);
#endif /* LORAWAN_COMPLIANCE_TEST */
}

int LoRaWANClass::begin(const struct LoRaWANBand &band)
{
    static const LoRaMacPrimitives_t LoRaMacPrimitives = {
        LoRaWANClass::__McpsConfirm,
        LoRaWANClass::__McpsIndication,
        LoRaWANClass::__MlmeConfirm,
        LoRaWANClass::__MlmeIndication,
    };

    static const LoRaMacCallback_t LoRaMacCallbacks = {
        LoRaWANClass::__GetBatteryLevel,
    };

    if (__get_IPSR() != 0) {
        return 0;
    }

    if (_Band) {
        return 0;
    }

    _Band = &band;

    _JoinRetries = _Band->JoinRetries;
    _DutyCycle = _Band->DutyCycle;

    if (_restoreSession()) {
        if (RTC->BKP2R & BKP2R_UPLINK_COUNTER_PRESENT) {
            if ((((RTC->BKP2R & BKP2R_UPLINK_COUNTER_MASK) >> BKP2R_UPLINK_COUNTER_SHIFT) ^ _UpLinkCounter) & EEPROM_COUNTER_UPDATE_PERIOD) {
                _UpLinkCounter += EEPROM_COUNTER_UPDATE_PERIOD;
            }

            _UpLinkCounter = (_UpLinkCounter & ~(EEPROM_COUNTER_UPDATE_PERIOD-1)) | ((RTC->BKP2R >> BKP2R_UPLINK_COUNTER_SHIFT) & (EEPROM_COUNTER_UPDATE_PERIOD-1));
        } else {
            _UpLinkCounter += (EEPROM_COUNTER_UPDATE_PERIOD << 1);
        }

        if (RTC->BKP2R & BKP2R_DOWNLINK_COUNTER_PRESENT) {
            if ((((RTC->BKP2R & BKP2R_DOWNLINK_COUNTER_MASK) >> BKP2R_DOWNLINK_COUNTER_SHIFT) ^ _DownLinkCounter) & EEPROM_COUNTER_UPDATE_PERIOD) {
                _DownLinkCounter += EEPROM_COUNTER_UPDATE_PERIOD;
            }

            _DownLinkCounter = (_DownLinkCounter & ~(EEPROM_COUNTER_UPDATE_PERIOD-1)) | ((RTC->BKP2R >> BKP2R_DOWNLINK_COUNTER_SHIFT) & (EEPROM_COUNTER_UPDATE_PERIOD-1));
        }
    } else {
        _session.Activation = LORAWAN_ACTIVATION_NONE;

        RTC->BKP2R = 0;
    }

    LoRaMacInitialization(&LoRaMacPrimitives, &LoRaMacCallbacks, _Band->LoRaMacRegion);

#if defined(STM32L0_CONFIG_ANTENNA_GAIN)
    {
	MibRequestConfirm_t mibReq;

	mibReq.Type = MIB_ANTENNA_GAIN;
	mibReq.Param.AntennaGain = STM32L0_CONFIG_ANTENNA_GAIN;
	LoRaWANMibSetRequestConfirm(&mibReq);
    }
#endif

    LoRaMacTxDelay = 1;

    return 1;
}

int LoRaWANClass::joinOTAA()
{
    LoRaWANCommissioning commissioning;

    if (!_Band) {
        return 0;
    }

    if (!_loadCommissioning(&commissioning)) {
        return 0;
    }

    return _joinOTAA(&commissioning);
}

int LoRaWANClass::joinOTAA(const char *appEui, const char *appKey, const char *devEui)
{
    LoRaWANCommissioning commissioning;

    if (!_Band) {
        return 0;
    }

    if (devEui == NULL) {
        BoardGetUniqueId(commissioning.DevEui);
    } else {
        if (!ConvertString(commissioning.DevEui, 8, devEui)) {
            return 0;
        }
    }

    if (!ConvertString(commissioning.AppEui, 8, appEui)) {
        return 0;
    }

    if (!ConvertString(commissioning.AppKey, 16, appKey)) {
        return 0;
    }

    return _joinOTAA(&commissioning);
}

int LoRaWANClass::rejoinOTAA()
{
    if (!_Band) {
        return 0;
    }

    if (_session.Activation != LORAWAN_ACTIVATION_OTAA) {
        return 0;
    }

    return _rejoinOTAA();
}

int LoRaWANClass::joinABP()
{
    LoRaWANCommissioning commissioning;

    if (!_Band) {
        return 0;
    }

    if (!_loadCommissioning(&commissioning)) {
        return 0;
    }

    return _joinABP(&commissioning);
}

int LoRaWANClass::joinABP(const char *devAddr, const char *nwkSKey, const char *appSKey)
{
    LoRaWANCommissioning commissioning;
    uint8_t DevAddr[4];

    if (!_Band) {
        return 0;
    }

    if (!ConvertString((uint8_t*)&DevAddr, 4, devAddr)) {
        return 0;
    }

    commissioning.DevAddr = (DevAddr[0] << 24) | (DevAddr[1] << 16) | (DevAddr[2] << 8) | (DevAddr[3] << 0);

    if (!ConvertString(commissioning.NwkSKey, 16, nwkSKey)) {
        return 0;
    }

    if (!ConvertString(commissioning.AppSKey, 16, appSKey)) {
        return 0;
    }

    return _joinABP(&commissioning);
}

bool LoRaWANClass::joined()
{
    return _Joined;
}

bool LoRaWANClass::confirmed()
{
    return _tx_ack;
}

bool LoRaWANClass::pending()
{
    return _tx_pending || _rx_pending;
}

bool LoRaWANClass::busy()
{
    return _tx_busy;
}

int LoRaWANClass::beginPacket(uint8_t port)
{
    if (!_Joined) {
        return 0;
    }

    if (_tx_busy) {
        return 0;
    }

    if ((port == 0) || (port >= 224)) {
        return 0;
    }

    _tx_size = 0;
    _tx_port = port;
    _tx_active = true;

    return 1;
}

int LoRaWANClass::endPacket(bool confirmed)
{
    IRQn_Type irq;

    irq = (IRQn_Type)((__get_IPSR() & 0x1ff) - 16);

    if (irq >= SysTick_IRQn) {  
        return 0;
    }

    if (!_tx_active) {
        return 0;
    }

    _tx_confirmed = confirmed;
    _tx_ack = false;
    _tx_busy = true;

    if (!_send()) {
        return 0;
    }

    return 1;
}

int LoRaWANClass::sendPacket(const uint8_t *buffer, size_t size, bool confirmed)
{
    return sendPacket(LORAWAN_DEFAULT_PORT, buffer, size, confirmed);
}

int LoRaWANClass::sendPacket(uint8_t port, const uint8_t *buffer, size_t size, bool confirmed)
{
    if (!_Joined) {
        return 0;
    }

    if (_tx_busy) {
        return 0;
    }

    if (_tx_active) {
        return 0;
    }

    if ((port == 0) || (port >= 224)) {
        return 0;
    }

    if (size > LORAWAN_MAX_PAYLOAD_SIZE) {
        return 0;
    }

    _tx_port = port;
    _tx_size = size;

    memcpy(&_tx_data[0], buffer, size);

    _tx_active = true;

    return endPacket(confirmed);
}

int LoRaWANClass::linkCheck()
{
    MlmeReq_t mlmeReq;

    if (!_Joined) {
        return 0;
    }

    if (_tx_busy) {
        return 0;
    }

    mlmeReq.Type = MLME_LINK_CHECK;
    if (LoRaWANMlmeRequest(&mlmeReq) != LORAMAC_STATUS_OK) {
        return 0;
    }

    _LinkCheckCount = _LinkCheckLimit;
    _LinkCheckWait = _LinkCheckDelay;

    return 1;
}

int LoRaWANClass::ping()
{
    if (!_Joined) {
        return 0;
    }

    if (_tx_busy) {
        return 0;
    }

    if (_tx_active) {
        return 0;
    }

    _tx_port = 0;
    _tx_size = 0;
    _tx_active = true;
    _tx_confirmed = true;
    _tx_ack = false;
    _tx_busy = true;

    if (!_send()) {
        return 0;
    }
    
    return 1;
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

    if (_rx_size)
    {
        _rx_index = _rx_next;
        _rx_read = _rx_next;
        _rx_size = 0;
        _rx_port = 0;
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

void LoRaWANClass::onJoin(void(*callback)(void))
{
    _joinCallback = Callback(callback);
}

void LoRaWANClass::onJoin(Callback callback)
{
    _joinCallback = callback;
}

void LoRaWANClass::onLinkCheck(void(*callback)(void))
{
    _linkCheckCallback = Callback(callback);
}

void LoRaWANClass::onLinkCheck(Callback callback)
{
    _linkCheckCallback = callback;
}

void LoRaWANClass::onReceive(void(*callback)(void))
{
    _receiveCallback = Callback(callback);
}

void LoRaWANClass::onReceive(Callback callback)
{
    _receiveCallback = callback;
}

void LoRaWANClass::onTransmit(void(*callback)(void))
{
    _transmitCallback = Callback(callback);
}

void LoRaWANClass::onTransmit(Callback callback)
{
    _transmitCallback = callback;
}

int LoRaWANClass::setAppEui(const char *appEui)
{
    LoRaWANCommissioning commissioning;
    
    _loadCommissioning(&commissioning);

    memset(commissioning.AppEui, 0, 8);

    if (appEui != NULL) {
        if (!ConvertString(commissioning.AppEui, 8, appEui)) {
            return 0;
        }
    }

    return _storeCommissioning(&commissioning);
}

int LoRaWANClass::setAppKey(const char *appKey)
{
    LoRaWANCommissioning commissioning;
    
    _loadCommissioning(&commissioning);

    memset(commissioning.AppKey, 0, 16);

    if (appKey != NULL) {
        if (!ConvertString(commissioning.AppKey, 16, appKey)) {
            return 0;
        }
    }

    return _storeCommissioning(&commissioning);
}

int LoRaWANClass::setDevEui(const char *devEui)
{
    LoRaWANCommissioning commissioning;
    
    _loadCommissioning(&commissioning);

    memset(commissioning.DevEui, 0, 8);

    if (devEui == NULL) {
        BoardGetUniqueId(commissioning.DevEui);
    } else {
        if (!ConvertString(commissioning.DevEui, 8, devEui)) {
            return 0;
        }
    }

    return _storeCommissioning(&commissioning);
}

int LoRaWANClass::setDevAddr(const char *devAddr)
{
    LoRaWANCommissioning commissioning;
    uint8_t DevAddr[4];
    
    _loadCommissioning(&commissioning);

    memset((uint8_t*)&commissioning.DevAddr, 0, 4);

    if (devAddr != NULL) {
        if (!ConvertString((uint8_t*)&DevAddr, 4, devAddr)) {
            return 0;
        }

        commissioning.DevAddr = (DevAddr[0] << 24) | (DevAddr[1] << 16) | (DevAddr[2] << 8) | (DevAddr[3] << 0);
    }

    return _storeCommissioning(&commissioning);
}

int LoRaWANClass::setNwkSKey(const char *nwkSKey)
{
    LoRaWANCommissioning commissioning;
    
    _loadCommissioning(&commissioning);

    memset(commissioning.NwkSKey, 0, 16);

    if (nwkSKey != NULL) {
        if (!ConvertString(commissioning.NwkSKey, 16, nwkSKey)) {
            return 0;
        }
    }

    return _storeCommissioning(&commissioning);
}

int LoRaWANClass::setAppSKey(const char *appSKey)
{
    LoRaWANCommissioning commissioning;
    
    _loadCommissioning(&commissioning);

    memset(commissioning.AppSKey, 0, 16);

    if (appSKey != NULL) {
        if (!ConvertString(commissioning.AppSKey, 16, appSKey)) {
            return 0;
        }
    }

    return _storeCommissioning(&commissioning);
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

int LoRaWANClass::getNextTxTime()
{
    LoRaMacTxInfo_t txInfo;

    if (!_Band) {
        return -1;
    }

    if (_tx_busy) {
        return -1;
    }

    LoRaWANQueryTxPossible(0, _DataRate, &txInfo);

    return txInfo.TxDelay;
}

int LoRaWANClass::getMaxPayloadSize()
{
    LoRaMacTxInfo_t txInfo;

    if (!_Band) {
        return -1;
    }

    if (_tx_busy) {
        return -1;
    }

    LoRaWANQueryTxPossible(0, _DataRate, &txInfo);

    return txInfo.CurrentPayloadSize;
}

int LoRaWANClass::getDataRate()
{
    LoRaMacTxInfo_t txInfo;

    if (!_Band) {
        return -1;
    }

    if (_tx_busy) {
        return -1;
    }

    if (!_AdrEnable) {
        return _DataRate;
    }

    LoRaWANQueryTxPossible(0, _DataRate, &txInfo);

    return txInfo.Datarate;
}

float LoRaWANClass::getTxPower()
{
    LoRaMacTxInfo_t txInfo;

    if (!_Band) {
        return -1;
    }

    if (_tx_busy) {
        return -1;
    }

    LoRaWANQueryTxPossible(0, _DataRate, &txInfo);

    if (_Band->Region == LORAWAN_REGION_US915) {
        return 30.0f - 2 * txInfo.TxPower;
    } else {
        return LoRaMacParams.MaxEirp - 2 * txInfo.TxPower;
    }
}

int LoRaWANClass::getRepeat()
{
    if (!_Band) {
        return -1;
    }

    if (_tx_busy) {
        return -1;
    }

    return LoRaMacParams.ChannelsNbRep;
}

int LoRaWANClass::setSaveSession(bool enable)
{
    if (!_Band) {
        return 0;
    }

    if (_Save != enable)
    {
        _Save = enable;
        
        if (_Save) {
            if (_Joined) {
                _getParams();
                _saveParams();
            }
        } else {
            memset(&_params, 0, sizeof(_params));
            _params.Region = LORAWAN_REGION_NONE;
            _saveParams();
        }
    }

    return 1;
}

int LoRaWANClass::setJoinDelay1(unsigned int delay)
{
    MibRequestConfirm_t mibReq;

    if (!_Band) {
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

    if (!_Band) {
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
    if (!_Band) {
        return 0;
    }

    if (n > 65536) {
        return 0;
    }

    _JoinRetries = n;

    return 1;
}

int LoRaWANClass::setLinkCheckLimit(unsigned int n)
{
    if (!_Band) {
        return 0;
    }

    if (n > 255) {
        return 0;
    }

    _LinkCheckLimit = n;
    _LinkCheckCount = n;
    _LinkCheckWait = 0;
    _LinkCheckFail = 0;

    return 1;
}

int LoRaWANClass::setLinkCheckDelay(unsigned int n)
{
    if (!_Band) {
        return 0;
    }

    if (n > 255) {
        return 0;
    }

    _LinkCheckDelay = n;

    return 1;
}

int LoRaWANClass::setLinkCheckThreshold(unsigned int n)
{
    if (!_Band) {
        return 0;
    }

    if (n > 255) {
        return 0;
    }

    _LinkCheckThreshold = n;

    return 1;
}

int LoRaWANClass::setADR(bool enable)
{
    MibRequestConfirm_t mibReq;

    if (!_Band) {
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

    if (_AdrEnable && !enable) {
        _DataRate = LoRaMacParams.ChannelsDatarate;
    }

    _AdrEnable = enable;

    if (_Joined) {
        _saveADR();
    }

    return 1;
}

int LoRaWANClass::setDataRate(unsigned int datarate)
{
    MibRequestConfirm_t mibReq;

    if (!_Band) {
        return 0;
    }

    if (_tx_busy) {
        return 0;
    }

    mibReq.Type = MIB_CHANNELS_DATARATE;
    mibReq.Param.ChannelsDatarate = datarate;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return 0;
    }

    if (!_Joined) {
        mibReq.Type = MIB_CHANNELS_DEFAULT_DATARATE;
        mibReq.Param.ChannelsDefaultDatarate = datarate;
        if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
            return 0;
        }
    }

    _DataRate = datarate;

    if (_Joined) {
        _saveADR();
    }

    return 1;
}

int LoRaWANClass::setTxPower(float power)
{
    unsigned int txPower;
    MibRequestConfirm_t mibReq;

    if (!_Band) {
        return 0;
    }

    if (_tx_busy) {
        return 0;
    }

    if (_Band->Region == LORAWAN_REGION_US915) {
        txPower = (floorf(30.0f - power) + 1) / 2;
    } else {
        txPower = (floorf(LoRaMacParams.MaxEirp - power) + 1) / 2;
    }

    mibReq.Type = MIB_CHANNELS_TX_POWER;
    mibReq.Param.ChannelsTxPower = txPower;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return 0;
    }

    if (!_Joined) {
        mibReq.Type = MIB_CHANNELS_DEFAULT_TX_POWER;
        mibReq.Param.ChannelsDefaultTxPower = txPower;
        if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
            return 0;
        }
    } else {
        _saveADR();
    }

    return 1;
}

int LoRaWANClass::setRepeat(unsigned int n)
{
    MibRequestConfirm_t mibReq;

    if (!_Band) {
        return 0;
    }

    if (n > 15) {
        return 0;
    }

    mibReq.Type = MIB_CHANNELS_NB_REP;
    mibReq.Param.ChannelNbRep = n;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return 0;
    }

    if (_Joined) {
        _saveADR();
    }

    return 1;
}

int LoRaWANClass::setRetries(unsigned int n)
{
    if (!_Band) {
        return 0;
    }

    if (n > 255) {
        return 0;
    }

    _Retries = n;

    return 1;
}

int LoRaWANClass::setPublicNetwork(bool enable)
{
    MibRequestConfirm_t mibReq;

    if (!_Band) {
        return 0;
    }

    if (_Joined) {
        return 0;
    }

    if (_tx_busy) {
        return 0;
    }
    
    mibReq.Type = MIB_PUBLIC_NETWORK;
    mibReq.Param.EnablePublicNetwork = enable;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return 0;
    }

    _PublicNetwork = enable;

    return 1;
}

int LoRaWANClass::setSubBand(unsigned int subband)
{
    uint16_t ChannelsMask[(LORA_MAX_NB_CHANNELS +15) / 16];
    unsigned int group;
    MibRequestConfirm_t mibReq;
    
    if (!_Band) {
        return 0;
    }

    if (_Joined) {
        return 0;
    }

    if (_tx_busy) {
        return 0;
    }

    if ((_Band->Region != LORAWAN_REGION_AU915) && (_Band->Region != LORAWAN_REGION_US915)) {
        return 0;
    }

    if (subband > 8) {
        return 0;
    }

    memset(&ChannelsMask[0], 0, sizeof(ChannelsMask));

    if (subband == 0)
    {
        ChannelsMask[0] = 0xFFFF;
        ChannelsMask[1] = 0xFFFF;
        ChannelsMask[2] = 0xFFFF;
        ChannelsMask[3] = 0xFFFF;
        ChannelsMask[4] = 0x00FF;
    }
    else
    {
        group = subband -1;

        ChannelsMask[group >> 1] = ((group & 1) ? 0xFF00 : 0x00FF);
        ChannelsMask[4] = (0x0001 << group);
    }

    mibReq.Type = MIB_CHANNELS_MASK;
    mibReq.Param.ChannelsMask = ChannelsMask;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return 0;
    }

    mibReq.Type = MIB_CHANNELS_DEFAULT_MASK;
    mibReq.Param.ChannelsDefaultMask = ChannelsMask;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return 0;
    }

    _SubBand = subband;

    return 1;
}

int LoRaWANClass::setReceiveDelay(unsigned int delay)
{
    MibRequestConfirm_t mibReq;

    if (!_Band) {
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

    if (_Joined) {
        if (_Save) {
            _params.ReceiveDelay = delay;
            _saveParams();
        }
    }

    return 1;
}

int LoRaWANClass::setRX1DrOffset(unsigned int offset)
{
    MibRequestConfirm_t mibReq;

    if (!_Band) {
        return 0;
    }

    if (_tx_busy) {
        return 0;
    }

    mibReq.Type = MIB_RX1_DR_OFFSET;
    mibReq.Param.Rx1DrOffset = offset;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return 0;
    }

    if (!_Joined) {
        mibReq.Type = MIB_RX1_DEFAULT_DR_OFFSET;
        mibReq.Param.Rx1DefaultDrOffset = offset;
        if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
            return 0;
        }
    } else {
        if (_Save) {
            _params.RX1DrOffset = offset;
            _saveParams();
        }
    }
    
    return 1;
}

int LoRaWANClass::setRX2Channel(unsigned long frequency, unsigned int datarate)
{
    MibRequestConfirm_t mibReq;

    if (!_Band) {
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

    if (!_Joined) {
        mibReq.Type = MIB_RX2_DEFAULT_CHANNEL;
        mibReq.Param.Rx2DefaultChannel.Frequency = frequency;
        mibReq.Param.Rx2DefaultChannel.Datarate = datarate;
        if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
            return 0;
        }
    } else {
        if (_Save) {
            _params.RX2Frequency = frequency;
            _params.RX2DataRate = datarate;
            _saveParams();
        }
    }
    
    return 1;
}

int LoRaWANClass::addChannel(unsigned int index, unsigned long frequency, unsigned int drMin, unsigned int drMax)
{
    ChannelParams_t channelParams;

    if (!_Band) {
        return 0;
    }

    if ((_Band->Region == LORAWAN_REGION_AU915) || (_Band->Region == LORAWAN_REGION_CN470) || (_Band->Region == LORAWAN_REGION_US915)) {
        return 0;
    }

    if (index >= _Band->Channels) {
        return 0;
    }

    if (_tx_busy) {
        return 0;
    }

    channelParams.Frequency = frequency;
    channelParams.Rx1Frequency = 0;
    channelParams.DrRange.Fields.Min = drMin;
    channelParams.DrRange.Fields.Max = drMax;
    channelParams.Band = 0;

    if (LoRaWANChannelAdd(index, &channelParams) != LORAMAC_STATUS_OK) {
        return 0;
    }

    if (_Joined) {
        if (_Save) {
            _getParams();
            _saveParams();
        }
    }

    return 1;
}

int LoRaWANClass::removeChannel(unsigned int index)
{
    if (!_Band) {
        return 0;
    }

    if ((_Band->Region == LORAWAN_REGION_AU915) || (_Band->Region == LORAWAN_REGION_CN470) || (_Band->Region == LORAWAN_REGION_US915)) {
        return 0;
    }

    if (index >= _Band->Channels) {
        return 0;
    }

    if (_tx_busy) {
        return 0;
    }

    if (LoRaWANChannelRemove(index) != LORAMAC_STATUS_OK) {
        return 0;
    }

    if (_Joined) {
        if (_Save) {
            _getParams();
            _saveParams();
        }
    }

    return 1;
}

int LoRaWANClass::enableChannel(unsigned int index)
{
    uint16_t ChannelsMask[(LORA_MAX_NB_CHANNELS + 15) / 16];
    MibRequestConfirm_t mibReq;

    if (!_Band) {
        return 0;
    }

    if (_tx_busy) {
        return 0;
    }

    if (index >= _Band->Channels) {
        return 0;
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

    if (_Joined) {
        if (_Save) {
            _getParams();
            _saveParams();
        }
    }

    return 1;
}

int LoRaWANClass::disableChannel(unsigned int index)
{
    uint16_t ChannelsMask[(LORA_MAX_NB_CHANNELS +15) / 16];
    MibRequestConfirm_t mibReq;

    if (!_Band) {
        return 0;
    }

    if (_tx_busy) {
        return 0;
    }

    if (index >= _Band->Channels) {
        return 0;
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

    if (_Joined) {
        if (_Save) {
            _getParams();
            _saveParams();
        }
    }

    return 1;
}

int LoRaWANClass::setDownLinkChannel(unsigned int index, unsigned long frequency)
{
    MibRequestConfirm_t mibReq;
    ChannelParams_t channelParams;

    if (!_Band) {
        return 0;
    }

    if ((_Band->Region == LORAWAN_REGION_AU915) || (_Band->Region == LORAWAN_REGION_CN470) || (_Band->Region == LORAWAN_REGION_US915)) {
        return 0;
    }

    if (index >= _Band->Channels) {
        return 0;
    }

    if (_tx_busy) {
        return 0;
    }

    mibReq.Type = MIB_CHANNELS;
    if (LoRaWANMibGetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return false;
    }

    if (mibReq.Param.ChannelList[index].Frequency == 0) {
        return 0;
    }

    channelParams.Frequency = mibReq.Param.ChannelList[index].Frequency;
    channelParams.Rx1Frequency = frequency;
    channelParams.DrRange.Value = mibReq.Param.ChannelList[index].DrRange.Value;
    channelParams.Band = 0;

    if (LoRaWANChannelAdd(index, &channelParams) != LORAMAC_STATUS_OK) {
        return 0;
    }

    if (_Joined) {
        if (_Save) {
            _getParams();
            _saveParams();
        }
    }

    return 1;
}

int LoRaWANClass::setUpLinkDwellTime(bool enable)
{
    MibRequestConfirm_t mibReq;

    if (!_Band) {
        return 0;
    }

    if (_tx_busy) {
        return 0;
    }

    mibReq.Type = MIB_UPLINK_DWELL_TIME;
    mibReq.Param.DefaultUplinkDwellTime = enable;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return 0;
    }

    if (!_Joined) {
        mibReq.Type = MIB_DEFAULT_UPLINK_DWELL_TIME;
        mibReq.Param.DefaultUplinkDwellTime = enable;
        if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
            return 0;
        }
    } else {
        if (_Save) {
            _params.UpLinkDwellTime = enable;
            _saveParams();
        }
    }

    return 1;
}

int LoRaWANClass::setDownLinkDwellTime(bool enable)
{
    MibRequestConfirm_t mibReq;

    if (!_Band) {
        return 0;
    }

    if (_tx_busy) {
        return 0;
    }

    mibReq.Type = MIB_DOWNLINK_DWELL_TIME;
    mibReq.Param.DefaultDownlinkDwellTime = enable;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return 0;
    }

    if (!_Joined) {
        mibReq.Type = MIB_DEFAULT_DOWNLINK_DWELL_TIME;
        mibReq.Param.DefaultDownlinkDwellTime = enable;
        if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
            return 0;
        }
    } else {
        if (_Save) {
            _params.DownLinkDwellTime = enable;
            _saveParams();
        }
    }

    return 1;
}

int LoRaWANClass::setMaxEIRP(float eirp)
{
    MibRequestConfirm_t mibReq;

    if (!_Band) {
        return 0;
    }

    if (_tx_busy) {
        return 0;
    }

    mibReq.Type = MIB_MAX_EIRP;
    mibReq.Param.MaxEirp = eirp;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return 0;
    }

    if (!_Joined) {
        mibReq.Type = MIB_DEFAULT_MAX_EIRP;
        mibReq.Param.DefaultMaxEirp = eirp;
        if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
            return 0;
        }
    } else {
        if (_Save) {
            _params.MaxEIRP = eirp;
            _saveParams();
        }
    }

    return 1;
}

int LoRaWANClass::setAntennaGain(float gain)
{
    MibRequestConfirm_t mibReq;

    if (!_Band) {
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

    if (!_Joined) {
        mibReq.Type = MIB_DEFAULT_ANTENNA_GAIN;
        mibReq.Param.DefaultAntennaGain = gain;
        if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
            return 0;
        }
    }

    return 1;
}

int LoRaWANClass::setDutyCycle(bool enable)
{
    if (!_Band) {
        return 0;
    }

    if (_tx_busy) {
        return 0;
    }

    if (!LoRaWAN._Band->DutyCycle) {
        return 0;
    }

    LoRaMacTestSetDutyCycleOn(enable);

    _DutyCycle = enable;

    return 1;
}

int LoRaWANClass::setComplianceTest(bool enable)
{
    if (!_Band) {
        return 0;
    }

    if (_tx_busy) {
        return 0;
    }

    _ComplianceTest = enable;

    return 1;
}

int LoRaWANClass::setBatteryLevel(unsigned int level)
{
    if (level > 255) {
        return 0;
    }

    _BatteryLevel = level;

    return 1;
}

void LoRaWANClass::_saveSession()
{
    _session.Header = EEPROM_HEADER_SESSION;
    _session.Crc32 = crc32((const uint8_t*)&_session, sizeof(_session) - sizeof(_session.Crc32), 0);

    EEPROMSessionCRC32 = ~_session.Crc32;

    if (EEPROMTransaction.status != STM32L0_EEPROM_STATUS_BUSY) {
        _eepromSync(this);
    }
}

bool LoRaWANClass::_restoreSession()
{
    uint32_t *DevNonce, *UpLinkCounter, *DownLinkCounter;
    unsigned int index;

    if (!_eepromRead(EEPROM_OFFSET_SESSION, (uint8_t*)&_session, sizeof(_session))) {
        return false;
    }

    if (_session.Header != EEPROM_HEADER_SESSION) {
        return false;
    }

    if (_session.Crc32 != crc32((const uint8_t*)&_session, sizeof(_session) - sizeof(_session.Crc32), 0)) {
        return false;
    }

    _DevNonce = 0;
    _UpLinkCounter = 0;
    _DownLinkCounter = 0;
        
    if (!_eepromRead(EEPROM_OFFSET_DEVNONCE, (uint8_t*)&LoRaWANBuffer[0], EEPROM_SIZE_DEVNONCE)) {
        return false;
    }

    DevNonce = &LoRaWANBuffer[0];

    for (index = 0; index < 32; index++) {
        if (_DevNonce < DevNonce[index]) {
            _DevNonce = DevNonce[index];
        }
    }

    if (!_eepromRead(EEPROM_OFFSET_UPLINK_COUNTER, (uint8_t*)&LoRaWANBuffer[0], EEPROM_SIZE_UPLINK_COUNTER)) {
        return false;
    }

    UpLinkCounter = &LoRaWANBuffer[0];

    for (index = 0; index < 32; index++) {
        if (_UpLinkCounter < UpLinkCounter[index]) {
            _UpLinkCounter = UpLinkCounter[index];
        }
    }

    if (!_eepromRead(EEPROM_OFFSET_DOWNLINK_COUNTER, (uint8_t*)&LoRaWANBuffer[0], EEPROM_SIZE_DOWNLINK_COUNTER)) {
        return false;
    }

    DownLinkCounter = &LoRaWANBuffer[0];
    
    for (index = 0; index < 32; index++) {
        if (_DownLinkCounter < DownLinkCounter[index]) {
            _DownLinkCounter = DownLinkCounter[index];
        }
    }

    EEPROMSessionCRC32 = _session.Crc32;
    EEPROMDevNonce = _DevNonce;
    EEPROMUpLinkCounter = _UpLinkCounter;
    EEPROMDownLinkCounter = _DownLinkCounter;

    return true;
}

void LoRaWANClass::_saveADR()
{
    RTC->BKP2R = ((RTC->BKP2R & ~(BKP2R_DATARATE_MASK | BKP2R_TX_POWER_MASK | BKP2R_REPEAT_MASK | BKP2R_ADR_ENABLE_MASK)) |
                  ((_AdrEnable ? LoRaMacParams.ChannelsDatarate : _DataRate) << BKP2R_DATARATE_SHIFT) |
                  (LoRaMacParams.ChannelsTxPower << BKP2R_TX_POWER_SHIFT) |
                  (LoRaMacParams.ChannelsNbRep << BKP2R_REPEAT_SHIFT) |
                  (_AdrEnable << BKP2R_ADR_ENABLE_SHIFT) |
                  BKP2R_ADR_PRESENT);
}

bool LoRaWANClass::_restoreADR()
{
    MibRequestConfirm_t mibReq;

    if (!(RTC->BKP2R & BKP2R_ADR_PRESENT)) {
        return false;
    }

    _AdrEnable = (RTC->BKP2R & BKP2R_ADR_ENABLE_MASK) >> BKP2R_ADR_ENABLE_SHIFT;
    _DataRate = (RTC->BKP2R & BKP2R_DATARATE_MASK) >> BKP2R_DATARATE_SHIFT;

    mibReq.Type = MIB_ADR;
    mibReq.Param.AdrEnable = _AdrEnable;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return false;
    }
    
    mibReq.Type = MIB_CHANNELS_DATARATE;
    mibReq.Param.ChannelsDatarate = _DataRate;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return false;
    }
    
    mibReq.Type = MIB_CHANNELS_TX_POWER;
    mibReq.Param.ChannelsTxPower = (RTC->BKP2R & BKP2R_TX_POWER_MASK) >> BKP2R_TX_POWER_SHIFT;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return false;
    }
    
    mibReq.Type = MIB_CHANNELS_NB_REP;
    mibReq.Param.ChannelNbRep = (RTC->BKP2R & BKP2R_REPEAT_MASK) >> BKP2R_REPEAT_SHIFT;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return false;
    }

    return true;
}

void LoRaWANClass::_saveParams( )
{
    _params.Header = EEPROM_HEADER_PARAMS;
    _params.Crc32 = crc32((const uint8_t*)&_params, sizeof(_params) - sizeof(_params.Crc32), 0);

    EEPROMParamsCRC32 = ~_params.Crc32;

    if (EEPROMTransaction.status != STM32L0_EEPROM_STATUS_BUSY) {
        _eepromSync(this);
    }
}

bool LoRaWANClass::_restoreParams()
{
    if (!_eepromRead(EEPROM_OFFSET_PARAMS, (uint8_t*)&_params, sizeof(LoRaWANParams))) {
        return false;
    }

    if (_params.Header != EEPROM_HEADER_PARAMS) {
        return false;
    }

    if (_params.Crc32 != crc32((const uint8_t*)&_params, sizeof(_params) - sizeof(_params.Crc32), 0)) {
        return false;
    }
    
    if (_params.Region != _Band->Region) {
        return false;
    }
    
    if (_params.Network != ((_SubBand & 0x0F) | (_PublicNetwork << 7))) {
        return false;
    }

    EEPROMParamsCRC32 = _params.Crc32;

    return true;
}

bool LoRaWANClass::_getParams()
{
    MibRequestConfirm_t mibReq;
    unsigned int index;

    memset(&_params, 0, sizeof(_params));

    _params.Region = _Band->Region;
    _params.Network = (_SubBand & 0x0F) | (_PublicNetwork << 7);

    _params.ReceiveDelay = LoRaMacParams.ReceiveDelay1;
    _params.RX1DrOffset = LoRaMacParams.Rx1DrOffset;
    _params.RX2DataRate = LoRaMacParams.Rx2Channel.Datarate;
    _params.RX2Frequency = LoRaMacParams.Rx2Channel.Frequency;
    _params.UpLinkDwellTime = LoRaMacParams.DownlinkDwellTime;
    _params.DownLinkDwellTime = LoRaMacParams.DownlinkDwellTime;
    _params.MaxEIRP = LoRaMacParams.MaxEirp;

    mibReq.Type = MIB_CHANNELS_MASK;
    if (LoRaWANMibGetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return false;
    }

    if ((_params.Region == LORAWAN_REGION_AU915) || (_params.Region == LORAWAN_REGION_US915))
    {
        _params.ChannelsMask[0] = mibReq.Param.ChannelsMask[0];
        _params.ChannelsMask[1] = mibReq.Param.ChannelsMask[1];
        _params.ChannelsMask[2] = mibReq.Param.ChannelsMask[2];
        _params.ChannelsMask[3] = mibReq.Param.ChannelsMask[3];
        _params.ChannelsMask[4] = mibReq.Param.ChannelsMask[4];
    }
    else
    {
        _params.ChannelsMask[0] = mibReq.Param.ChannelsMask[0];

        mibReq.Type = MIB_CHANNELS;
        if (LoRaWANMibGetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
            return false;
        }

        for (index = 0; index < 16; index++) {
            _params.ChannelsFrequency[index] = mibReq.Param.ChannelList[index].Frequency;
            _params.ChannelsRX1Frequency[index] = mibReq.Param.ChannelList[index].Rx1Frequency;
            _params.ChannelsDrRange[index] = mibReq.Param.ChannelList[index].DrRange.Value;
        }
    }

    return true;
}

bool LoRaWANClass::_setParams()
{
    MibRequestConfirm_t mibReq;
    ChannelParams_t channelParams;
    unsigned int index;

    mibReq.Type = MIB_RECEIVE_DELAY_1;
    mibReq.Param.ReceiveDelay1 = _params.ReceiveDelay;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return false;
    }

    mibReq.Type = MIB_RECEIVE_DELAY_2;
    mibReq.Param.ReceiveDelay2 = _params.ReceiveDelay + 1000;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return false;
    }

    mibReq.Type = MIB_RX1_DR_OFFSET;
    mibReq.Param.Rx1DrOffset = _params.RX1DrOffset;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return false;
    }

    mibReq.Type = MIB_RX2_CHANNEL;
    mibReq.Param.Rx2Channel.Datarate = _params.RX2DataRate;
    mibReq.Param.Rx2Channel.Frequency = _params.RX2Frequency;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return false;
    }

    mibReq.Type = MIB_UPLINK_DWELL_TIME;
    mibReq.Param.DownlinkDwellTime = _params.UpLinkDwellTime;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return false;
    }

    mibReq.Type = MIB_DOWNLINK_DWELL_TIME;
    mibReq.Param.DownlinkDwellTime = _params.DownLinkDwellTime;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return false;
    }

    mibReq.Type = MIB_MAX_EIRP;
    mibReq.Param.MaxEirp = _params.MaxEIRP;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return false;
    }

    mibReq.Type = MIB_CHANNELS_MASK;
    mibReq.Param.ChannelsMask = _params.ChannelsMask;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return false;
    }

    if ((_params.Region != LORAWAN_REGION_AU915) && (_params.Region != LORAWAN_REGION_US915)) {
        for (index = 0; index < 16; index++) {
            if (_params.ChannelsFrequency[index]) {
                channelParams.Frequency = _params.ChannelsFrequency[index];
                channelParams.Rx1Frequency = _params.ChannelsRX1Frequency[index];
                channelParams.DrRange.Value = _params.ChannelsDrRange[index];
                channelParams.Band = 0;
                if (LoRaWANChannelAdd(index, &channelParams) != LORAMAC_STATUS_OK) {
                    return false;
                }
            }
        }
    }
    
    return true;
}

void LoRaWANClass::_saveDevNonce()
{
    if (EEPROMDevNonce != _DevNonce) {
        if (EEPROMTransaction.status != STM32L0_EEPROM_STATUS_BUSY) {
            _eepromSync(this);
        }
    }
}

void LoRaWANClass::_saveUpLinkCounter()
{
    RTC->BKP2R = ((RTC->BKP2R & ~BKP2R_UPLINK_COUNTER_MASK) |
                  ((_UpLinkCounter << BKP2R_UPLINK_COUNTER_SHIFT) & BKP2R_UPLINK_COUNTER_MASK) |
                  BKP2R_UPLINK_COUNTER_PRESENT);

    if ((EEPROMUpLinkCounter ^ _UpLinkCounter) & ~(EEPROM_COUNTER_UPDATE_PERIOD-1)) {
        if (EEPROMTransaction.status != STM32L0_EEPROM_STATUS_BUSY) {
            _eepromSync(this);
        }
    }
}

void LoRaWANClass::_saveDownLinkCounter()
{
    RTC->BKP2R = ((RTC->BKP2R & ~BKP2R_DOWNLINK_COUNTER_MASK) |
                  ((_DownLinkCounter << BKP2R_DOWNLINK_COUNTER_SHIFT) & BKP2R_DOWNLINK_COUNTER_MASK) |
                  BKP2R_DOWNLINK_COUNTER_PRESENT);

    if ((EEPROMDownLinkCounter ^ _DownLinkCounter) & ~(EEPROM_COUNTER_UPDATE_PERIOD-1)) {
        if (EEPROMTransaction.status != STM32L0_EEPROM_STATUS_BUSY) {
            _eepromSync(this);
        }
    }
}

bool LoRaWANClass::_loadCommissioning(LoRaWANCommissioning *commissioning)
{
    if (!_eepromRead(EEPROM_OFFSET_COMMISSIONING, (uint8_t*)commissioning, sizeof(LoRaWANCommissioning))) {
        memset((uint8_t*)commissioning, 0, sizeof(LoRaWANCommissioning));
        return false;
    }

    if (commissioning->Header != EEPROM_HEADER_COMMISSIONING) {
        memset((uint8_t*)commissioning, 0, sizeof(LoRaWANCommissioning));
        return false;
    }
    
    if (commissioning->Crc32 != crc32((const uint8_t*)commissioning, sizeof(LoRaWANCommissioning) - sizeof(commissioning->Crc32), 0)) {
        memset((uint8_t*)commissioning, 0, sizeof(LoRaWANCommissioning));
        return false;
    }

    return true;
}

bool LoRaWANClass::_storeCommissioning(LoRaWANCommissioning *commissioning)
{
    commissioning->Header = EEPROM_HEADER_COMMISSIONING;
    commissioning->Crc32 = crc32((const uint8_t*)commissioning, sizeof(LoRaWANCommissioning) - sizeof(commissioning->Crc32), 0);

    return _eepromProgram(EEPROM_OFFSET_COMMISSIONING, (const uint8_t*)commissioning, sizeof(LoRaWANCommissioning));
}

int LoRaWANClass::_joinOTAA(LoRaWANCommissioning *commissioning)
{
    if ((_session.Activation != LORAWAN_ACTIVATION_OTAA) ||
        memcmp(_session.DevEui, commissioning->DevEui, 8) ||
        memcmp(_session.AppEui, commissioning->AppEui, 8) ||
        memcmp(_session.AppKey, commissioning->AppKey, 16))
    {
        _session.Activation = LORAWAN_ACTIVATION_OTAA;
        _session.Reserved = 0;

        memcpy(_session.DevEui, commissioning->DevEui, 8);
        memcpy(_session.AppEui, commissioning->AppEui, 8);
        memcpy(_session.AppKey, commissioning->AppKey, 16);

        if (!stm32l0_random((uint8_t*)&_session.DevNonce0, sizeof(_session.DevNonce0))) {
            return 0;
        }

        _DevNonce = ~0l; // force wraparound in _rejoinOTAA();

        return _rejoinOTAA();
    }
    else
    {
        if ((_session.DevAddr == 0) || !_Save || !_restoreParams())
        {
            return _rejoinOTAA();
        }
        else
        {
            _restoreADR();

            return _rejoinABP();
        }
    }
}

int LoRaWANClass::_joinABP(LoRaWANCommissioning *commissioning)
{
    if ((_session.Activation != LORAWAN_ACTIVATION_ABP) ||
        (_session.DevAddr != commissioning->DevAddr) ||
        memcmp(_session.NwkSKey, commissioning->NwkSKey, 16) ||
        memcmp(_session.AppSKey, commissioning->AppSKey, 16))
    {
        _session.Activation = LORAWAN_ACTIVATION_ABP;
        _session.Reserved = 0;

        memset(_session.DevEui, 0, 8);
        memset(_session.AppEui, 0, 8);
        memset(_session.AppKey, 0, 16);

        if (!stm32l0_random((uint8_t*)&_session.DevNonce0, sizeof(_session.DevNonce0))) {
            return 0;
        }

        _session.NetID = 0;
        _session.DevAddr = commissioning->DevAddr;

        memcpy(_session.NwkSKey, commissioning->NwkSKey, 16);
        memcpy(_session.AppSKey, commissioning->AppSKey, 16);

        _DevNonce = 0;
        _UpLinkCounter = 0;
        _DownLinkCounter = 0;

        _saveSession();

        if (_Save) {
            _getParams();
        } else {
            memset(&_params, 0, sizeof(_params));
            _params.Region = LORAWAN_REGION_NONE;
        }
        
        _saveParams();

        return _rejoinABP();
    }
    else
    {
        if (!_Save || !_restoreParams())
        {
            if (_Save) {
                _getParams();
            } else {
                memset(&_params, 0, sizeof(_params));
                _params.Region = LORAWAN_REGION_NONE;
            }

            _saveParams();

            return _rejoinABP();
        }
        else
        {
            _restoreADR();

            return _rejoinABP();
        }
    }
}

int LoRaWANClass::_rejoinOTAA()
{
    MlmeReq_t mlmeReq;

    _session.NetID = 0;
    _session.DevAddr = 0;
        
    memset(_session.NwkSKey, 0, 16);
    memset(_session.AppSKey, 0, 16);

    _DevNonce += 1;
    _UpLinkCounter = 0;
    _DownLinkCounter = 0;

    _saveSession();
    
    if (_Save) {
        _getParams();
    } else {
        memset(&_params, 0, sizeof(_params));
        _params.Region = LORAWAN_REGION_NONE;
    }

    _saveParams();

    _saveADR();

    mlmeReq.Type = MLME_JOIN;
    mlmeReq.Req.Join.DevEui = _session.DevEui;
    mlmeReq.Req.Join.AppEui = _session.AppEui;
    mlmeReq.Req.Join.AppKey = _session.AppKey;
    mlmeReq.Req.Join.DevNonce = _session.DevNonce0 + _DevNonce;
    mlmeReq.Req.Join.Datarate = _DataRate;
    
    if (LoRaWANMlmeRequest(&mlmeReq) != LORAMAC_STATUS_OK) {
        return 0;
    }

    _Joined = false;
    _JoinTrials = 0;

    _AdrWait = 0;
    _LinkCheckFail = 0;
    _LinkCheckGateways = 0;

    _tx_busy = true;

    return 1;
}

int LoRaWANClass::_rejoinABP()
{
    MibRequestConfirm_t mibReq;

    _saveADR();
    
    mibReq.Type = MIB_NET_ID;
    mibReq.Param.NetID = _session.NetID;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return 0;
    }

    mibReq.Type = MIB_DEV_ADDR;
    mibReq.Param.DevAddr = _session.DevAddr;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return 0;
    }

    mibReq.Type = MIB_NWK_SKEY;
    mibReq.Param.NwkSKey = _session.NwkSKey;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return 0;
    }

    mibReq.Type = MIB_APP_SKEY;
    mibReq.Param.AppSKey = _session.AppSKey;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return 0;
    }

    mibReq.Type = MIB_UPLINK_COUNTER;
    mibReq.Param.UpLinkCounter = _UpLinkCounter;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return 0;
    }

    mibReq.Type = MIB_DOWNLINK_COUNTER;
    mibReq.Param.DownLinkCounter = _DownLinkCounter;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return 0;
    }

    mibReq.Type = MIB_NETWORK_JOINED;
    mibReq.Param.IsNetworkJoined = true;
    if (LoRaWANMibSetRequestConfirm(&mibReq) != LORAMAC_STATUS_OK) {
        return 0;
    }

    _Joined = true;

    _AdrWait = 0;
    _LinkCheckFail = 0;
    _LinkCheckGateways = 1;

    _tx_busy = false;

    return 1;
}

bool LoRaWANClass::_send()
{
    McpsReq_t mcpsReq;
    MlmeReq_t mlmeReq;
    LoRaMacTxInfo_t txInfo;
    unsigned int fOptLen = 0;

    if (LoRaWANQueryTxPossible(0, _DataRate, &txInfo) != LORAMAC_STATUS_OK) 
    {
        _tx_active = false;
        _tx_busy = false;

        return false;
    }

    if (_tx_size > txInfo.CurrentPayloadSize)
    {
	_tx_active = false;
	_tx_busy = false;
        
	return false;
    }
        
    if (_LinkCheckCount)
    {
	if (!(_tx_confirmed && (_tx_size <= txInfo.MaxPossiblePayload)))
	{
	    _LinkCheckCount--;
            
	    if (_LinkCheckCount == 0) {
		mlmeReq.Type = MLME_LINK_CHECK;
		if (LoRaWANMlmeRequest(&mlmeReq) == LORAMAC_STATUS_OK) {
		    fOptLen = 1;
                    
		    _LinkCheckCount = _LinkCheckLimit;
		    _LinkCheckWait = _LinkCheckDelay;
		}  else {
		    _LinkCheckCount++;
		}
	    }
	}
    }

    if (_tx_size > (txInfo.MaxPossiblePayload + fOptLen)) 
    {
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fPort = 0;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = _DataRate;
    }
    else
    {
        _tx_active = false;

        if (_tx_confirmed)
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = _tx_port;
            mcpsReq.Req.Confirmed.fBuffer = _tx_data;
            mcpsReq.Req.Confirmed.fBufferSize = _tx_size;
            mcpsReq.Req.Confirmed.NbTrials = 1 + _Retries;
            mcpsReq.Req.Confirmed.Datarate = _DataRate;
        }
        else
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = _tx_port;
            mcpsReq.Req.Unconfirmed.fBuffer = _tx_data;
            mcpsReq.Req.Unconfirmed.fBufferSize = _tx_size;
            mcpsReq.Req.Unconfirmed.Datarate = _DataRate;
        }
    }

    if (LoRaWANMcpsRequest(&mcpsReq) != LORAMAC_STATUS_OK)
    {
        _tx_active = false;
        _tx_busy = false;

        return false;
    }

    _tx_pending = false;

    return true;
}

bool LoRaWANClass::_eepromProgram(uint32_t address, const uint8_t *data, uint32_t size)
{
    stm32l0_eeprom_transaction_t transaction;

    transaction.status = STM32L0_EEPROM_STATUS_BUSY;
    transaction.control = STM32L0_EEPROM_CONTROL_PROGRAM;
    transaction.count = size;
    transaction.address = address;
    transaction.data = (uint8_t*)data;
    transaction.callback = NULL;
    transaction.context = NULL;

    if (!stm32l0_eeprom_enqueue(&transaction)) {
        return false;
    }

    while (transaction.status == STM32L0_EEPROM_STATUS_BUSY) {
        armv6m_core_wait();
    }

    return true;
}

bool LoRaWANClass::_eepromRead(uint32_t address, uint8_t *data, uint32_t size)
{
    stm32l0_eeprom_transaction_t transaction;

    transaction.status = STM32L0_EEPROM_STATUS_BUSY;
    transaction.control = STM32L0_EEPROM_CONTROL_READ;
    transaction.count = size;
    transaction.address = address;
    transaction.data = data;
    transaction.callback = NULL;
    transaction.context = NULL;

    if (!stm32l0_eeprom_enqueue(&transaction)) {
        return false;
    }

    while (transaction.status == STM32L0_EEPROM_STATUS_BUSY) {
        armv6m_core_wait();
    }

    return true;
}

void LoRaWANClass::_eepromSync(class LoRaWANClass *self)
{
    if (EEPROMSessionCRC32 ^ self->_session.Crc32)
    {
        EEPROMTransaction.control = STM32L0_EEPROM_CONTROL_PROGRAM;
        EEPROMTransaction.count = sizeof(LoRaWANSession);
        EEPROMTransaction.address = EEPROM_OFFSET_SESSION;
        EEPROMTransaction.data = (uint8_t*)&self->_session;

        EEPROMSessionCRC32 = self->_session.Crc32;

        stm32l0_eeprom_enqueue(&EEPROMTransaction);

        return;
    }

    if (EEPROMParamsCRC32 ^ self->_params.Crc32)
    {
        EEPROMTransaction.control = STM32L0_EEPROM_CONTROL_PROGRAM;
        EEPROMTransaction.count = sizeof(LoRaWANParams);
        EEPROMTransaction.address = EEPROM_OFFSET_PARAMS;
        EEPROMTransaction.data = (uint8_t*)&self->_params;

        EEPROMParamsCRC32 = self->_params.Crc32;

        stm32l0_eeprom_enqueue(&EEPROMTransaction);
        
        return;
    }

    if (EEPROMDevNonce != self->_DevNonce)
    {
        if (self->_DevNonce == 0)
        {
            EEPROMTransaction.control = STM32L0_EEPROM_CONTROL_ERASE;
            EEPROMTransaction.count = EEPROM_SIZE_DEVNONCE;
            EEPROMTransaction.address = EEPROM_OFFSET_DEVNONCE;
            EEPROMTransaction.data = NULL;
        }
        else
        {
            EEPROMTransaction.control = STM32L0_EEPROM_CONTROL_PROGRAM;
            EEPROMTransaction.count = 4;
            EEPROMTransaction.address = EEPROM_OFFSET_DEVNONCE + (((EEPROMDevNonce ^ self->_session.DevNonce0) & 31) * 4);
            EEPROMTransaction.data = (uint8_t*)&EEPROMDevNonce;
        }

        EEPROMDevNonce = self->_DevNonce;

        stm32l0_eeprom_enqueue(&EEPROMTransaction);

        return;
    }

    if ((EEPROMUpLinkCounter ^ self->_UpLinkCounter) & ~(EEPROM_COUNTER_UPDATE_PERIOD-1))
    {
        if (self->_UpLinkCounter == 0)
        {
            EEPROMTransaction.control = STM32L0_EEPROM_CONTROL_ERASE;
            EEPROMTransaction.count = EEPROM_SIZE_UPLINK_COUNTER;
            EEPROMTransaction.address = EEPROM_OFFSET_UPLINK_COUNTER;
            EEPROMTransaction.data = NULL;
        }
        else
        {
            EEPROMTransaction.control = STM32L0_EEPROM_CONTROL_PROGRAM;
            EEPROMTransaction.count = 4;
            EEPROMTransaction.address = EEPROM_OFFSET_UPLINK_COUNTER + ((((EEPROMUpLinkCounter / EEPROM_COUNTER_UPDATE_PERIOD) ^ self->_session.DevNonce0) & 31) * 4);
            EEPROMTransaction.data = (uint8_t*)&EEPROMUpLinkCounter;
        }

        EEPROMUpLinkCounter = self->_UpLinkCounter & ~(EEPROM_COUNTER_UPDATE_PERIOD-1);

        stm32l0_eeprom_enqueue(&EEPROMTransaction);

        return;
    }

    if ((EEPROMDownLinkCounter ^ self->_DownLinkCounter) & ~(EEPROM_COUNTER_UPDATE_PERIOD-1))
    {
        if (self->_DownLinkCounter == 0)
        {
            EEPROMTransaction.control = STM32L0_EEPROM_CONTROL_ERASE;
            EEPROMTransaction.count = EEPROM_SIZE_DOWNLINK_COUNTER;
            EEPROMTransaction.address = EEPROM_OFFSET_DOWNLINK_COUNTER;
            EEPROMTransaction.data = NULL;
        }
        else
        {
            EEPROMTransaction.control = STM32L0_EEPROM_CONTROL_PROGRAM;
            EEPROMTransaction.count = 4;
            EEPROMTransaction.address = EEPROM_OFFSET_DOWNLINK_COUNTER + ((((EEPROMDownLinkCounter / EEPROM_COUNTER_UPDATE_PERIOD) ^ self->_session.DevNonce0) & 31) * 4);
            EEPROMTransaction.data = (uint8_t*)&EEPROMDownLinkCounter;
        }

        EEPROMDownLinkCounter = self->_DownLinkCounter & ~(EEPROM_COUNTER_UPDATE_PERIOD-1);

        stm32l0_eeprom_enqueue(&EEPROMTransaction);
        
        return;
    }
}

void LoRaWANClass::__McpsJoin()
{
    McpsReq_t mcpsReq;

    mcpsReq.Type = MCPS_UNCONFIRMED;
    mcpsReq.Req.Unconfirmed.fPort = 0;
    mcpsReq.Req.Unconfirmed.fBuffer = NULL;
    mcpsReq.Req.Unconfirmed.fBufferSize = 0;
    mcpsReq.Req.Unconfirmed.Datarate = LoRaWAN._DataRate;

    if (LoRaMacMcpsRequest(&mcpsReq) != LORAMAC_STATUS_OK)
    {
        LoRaWAN._Joined = true;
        
        LoRaWAN._tx_join = false;
        LoRaWAN._tx_busy = false;
        
        LoRaWAN._joinCallback.queue();
    }
}

void LoRaWANClass::__McpsSend()
{
    if (!LoRaWAN._send())
    {
	LoRaWAN._transmitCallback.queue();
    }
}

void LoRaWANClass::__McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    int minDataRate;

    if ((mcpsConfirm->Status != LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT) && (mcpsConfirm->Status != LORAMAC_EVENT_INFO_STATUS_TX_DR_PAYLOAD_SIZE_ERROR))
    {
	/* LORAMAC_EVENT_INFO_STATUS_OK
	 * LORAMAC_EVENT_INFO_STATUS_RX1_TIMEOUT
	 * LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT
	 * LORAMAC_EVENT_INFO_STATUS_RX1_ERROR
	 * LORAMAC_EVENT_INFO_STATUS_RX2_ERROR
	 */

	LoRaWAN._TimeOnAir += mcpsConfirm->TxTimeOnAir;
	LoRaWAN._UpLinkCounter = mcpsConfirm->UpLinkCounter + 1;
	
	LoRaWAN._saveUpLinkCounter();
	
#if defined(LORAWAN_COMPLIANCE_TEST)
	if (ComplianceTest.Running)
	{
	    ComplianceTest.UpLinkCounter++;

	    if (ComplianceTest.UpLinkCounter > 192)
	    {
		ComplianceTest.Running = false;

		// STOP TIMER
		stm32l0_rtc_timer_stop(&ComplianceTest.Timer);
		
		LoRaWAN._tx_busy = false;
		
		if (LoRaWAN._Band->DutyCycle) {
		    LoRaMacTestSetDutyCycleOn(LoRaWAN._DutyCycle);
		}
	    }
	}
	else
#endif /* LORAWAN_COMPLIANCE_TEST */
	{
	    LoRaWAN._saveADR();

	    if (mcpsConfirm->McpsRequest == MCPS_CONFIRMED)
	    {
		if (mcpsConfirm->AckReceived) 
		{
		    LoRaWAN._LinkCheckFail = 0;

		    LoRaWAN._tx_ack = true;
		}
		else
		{
		    LoRaWAN._LinkCheckFail++;
                
		    if (LoRaWAN._LinkCheckFail > LoRaWAN._LinkCheckThreshold) {
			LoRaWAN._LinkCheckGateways = 0;
		    }
		}
	    }

	    if (!LoRaMacFlags.Bits.McpsInd || LoRaMacFlags.Bits.McpsIndSkip)
	    {
		if (LoRaWAN._AdrEnable && LoRaWAN._AdrWait)
		{
		    minDataRate = ((LoRaWAN._Band->Region == LORAWAN_REGION_AS923) && (LoRaMacParams.UplinkDwellTime != 0)) ? 2 : 0;

		    if (LoRaMacParams.ChannelsDatarate <= minDataRate)
		    {
			LoRaWAN._AdrWait--;
			
			if (LoRaWAN._AdrWait == 0) {
			    LoRaWAN._LinkCheckGateways = 0;
			}
		    }
		}

		if (LoRaWAN._LinkCheckWait) {
		    LoRaWAN._LinkCheckWait--;
            
		    if (LoRaWAN._LinkCheckWait == 0) {
			LoRaWAN._LinkCheckFail++;
                
			if (LoRaWAN._LinkCheckFail > LoRaWAN._LinkCheckThreshold) {
			    LoRaWAN._LinkCheckGateways = 0;
			}
		    }
		}
	    }
	}

	LoRaWAN._AdrLastDataRate = LoRaMacParams.ChannelsDatarate;
	LoRaWAN._AdrLastTxPower = LoRaMacParams.ChannelsTxPower;
    }

#if defined(LORAWAN_COMPLIANCE_TEST)
    if (!ComplianceTest.Running)
#endif /* LORAWAN_COMPLIANCE_TEST */
    {
	if (LoRaWAN._tx_active)
	{
	    armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)LoRaWANClass::__McpsSend, NULL, 0);
	}
	else
	{
	    if (!LoRaMacFlags.Bits.McpsInd || LoRaMacFlags.Bits.McpsIndSkip)
	    {
		if (LoRaWAN._tx_join)
		{
		    LoRaWAN._Joined = true;
		
		    LoRaWAN._tx_join = false;
		    LoRaWAN._tx_busy = false;
		    
		    LoRaWAN._joinCallback.queue();
		}
		else
		{
		    LoRaWAN._tx_busy = false;
		    
		    LoRaWAN._transmitCallback.queue();
		}
	    }
	}
    }
}

void LoRaWANClass::__McpsIndication( McpsIndication_t *mcpsIndication )
{
    MibRequestConfirm_t mibReq;
    MlmeReq_t mlmeReq;
    uint32_t rx_write, rx_size;
    unsigned int i;
    int minDataRate;

#if defined(LORAWAN_COMPLIANCE_TEST)
    bool ComplianceTestRunning = ComplianceTest.Running;
#endif /* LORAWAN_COMPLIANCE_TEST */

    if (mcpsIndication->Status == LORAMAC_EVENT_INFO_STATUS_OK)
    {
	LoRaWAN._SNR = mcpsIndication->Snr;
	LoRaWAN._RSSI = mcpsIndication->Rssi;
	LoRaWAN._DownLinkCounter = mcpsIndication->DownLinkCounter;

	LoRaWAN._saveDownLinkCounter();

	if (mcpsIndication->ParamsUpdated) 
	{
	    LoRaWAN._saveADR();

	    if (LoRaWAN._Save)
	    {
		LoRaWAN._getParams();
		LoRaWAN._saveParams();
	    }
	}

	LoRaWAN._rx_pending = mcpsIndication->FramePending;

#if defined(LORAWAN_COMPLIANCE_TEST)
	if (ComplianceTest.Running)
	{
	    ComplianceTest.DownLinkCounter++;
	    ComplianceTest.UpLinkCounter = 0;

	    if (mcpsIndication->RxData == true)
	    {
		if (mcpsIndication->Port == 224)
		{
		    switch(mcpsIndication->Buffer[0]) {
		    case 0: // Check compliance test disable command (ii)
			ComplianceTest.Running = false;

			// STOP TIMER
			stm32l0_rtc_timer_stop(&ComplianceTest.Timer);

			LoRaWAN._tx_busy = false;

			if (LoRaWAN._Band->DutyCycle) {
			    LoRaMacTestSetDutyCycleOn(LoRaWAN._DutyCycle);
			}
			break;

		    case 1: // (iii, iv)
			break;

		    case 2: // Enable confirmed messages (v)
			ComplianceTest.IsConfirmed = true;
			break;

		    case 3:  // Disable confirmed messages (vi)
			ComplianceTest.IsConfirmed = false;
			break;
			
		    case 4: // (vii)
			ComplianceTest.TxSize = mcpsIndication->BufferSize;

			if (ComplianceTest.TxSize > LORAWAN_TX_BUFFER_SIZE) {
			    ComplianceTest.TxSize = LORAWAN_TX_BUFFER_SIZE;
			}
			
			ComplianceTest.TxData[0] = 4;
			
			for (i = 1; i < ComplianceTest.TxSize; i++) { 
			    ComplianceTest.TxData[i] = mcpsIndication->Buffer[i] +1;
			}
			break;
			
		    case 5: // (viii)
			mlmeReq.Type = MLME_LINK_CHECK;
			LoRaMacMlmeRequest(&mlmeReq);
			break;
			
		    case 6: // Disable TestMode and revert back to normal operation (ix)
			ComplianceTest.Running = false;

			// STOP TIMER
			stm32l0_rtc_timer_stop(&ComplianceTest.Timer);

			if (LoRaWAN._Band->DutyCycle) {
			    LoRaMacTestSetDutyCycleOn(LoRaWAN._DutyCycle);
			}

			LoRaWAN._session.NetID = 0;
			LoRaWAN._session.DevAddr = 0;
        
			memset(LoRaWAN._session.NwkSKey, 0, 16);
			memset(LoRaWAN._session.AppSKey, 0, 16);
			
			LoRaWAN._UpLinkCounter = 0;
			LoRaWAN._DownLinkCounter = 0;
			
			LoRaWAN._saveSession();

			LoRaWAN._Joined = false;
			LoRaWAN._JoinTrials = 0;
			    
			armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)LoRaWANClass::__MlmeJoin, NULL, 0);
			break;
			
		    case 7: // (x)
			if (mcpsIndication->BufferSize == 3)
			{
			    mlmeReq.Type = MLME_TXCW;
			    mlmeReq.Req.TxCw.Timeout = (uint16_t)((mcpsIndication->Buffer[1] << 8) | mcpsIndication->Buffer[2]);
			    LoRaMacMlmeRequest(&mlmeReq);
			}
			else if(mcpsIndication->BufferSize == 7)
			{
			    mlmeReq.Type = MLME_TXCW_1;
			    mlmeReq.Req.TxCw.Timeout = (uint16_t)((mcpsIndication->Buffer[1] << 8) | mcpsIndication->Buffer[2]);
			    mlmeReq.Req.TxCw.Frequency = (uint32_t)((mcpsIndication->Buffer[3] << 16) | (mcpsIndication->Buffer[4] << 8) | mcpsIndication->Buffer[5]) * 100;
			    mlmeReq.Req.TxCw.Power = mcpsIndication->Buffer[6];
			    LoRaMacMlmeRequest(&mlmeReq);
			}
			break;
			
		    default:
			break;
		    }
		}
	    }
	}
	else
#endif /* LORAWAN_COMPLIANCE_TEST */ 
	{
	    if (LoRaWAN._LinkCheckGateways == 0) {
		LoRaWAN._LinkCheckGateways = 1;
	    }

	    if (mcpsIndication->AdrReqReceived) 
	    {
		LoRaWAN._AdrWait = ADR_ACK_DELAY;
	    }
	    else
	    {
		if (LoRaWAN._AdrEnable && LoRaWAN._AdrWait)
		{
		    minDataRate = ((LoRaWAN._Band->Region == LORAWAN_REGION_AS923) && (LoRaMacParams.UplinkDwellTime != 0)) ? 2 : 0;

		    if (LoRaMacParams.ChannelsDatarate <= minDataRate)
		    {
			LoRaWAN._AdrWait--;
			
			if (LoRaWAN._AdrWait == 0) {
			    LoRaWAN._LinkCheckGateways = 0;
			}
		    }
		}
	    }
	
	    if (mcpsIndication->RxData == true)
	    {
		if (mcpsIndication->Port == 224)
		{
#if defined(LORAWAN_COMPLIANCE_TEST)
		    if (LoRaWAN._ComplianceTest)
		    {
			if (!ComplianceTest.Running)
			{
			    // Check compliance test enable command (i)
			    if( (mcpsIndication->BufferSize == 4) &&
				(mcpsIndication->Buffer[0] == 0x01) &&
				(mcpsIndication->Buffer[1] == 0x01) &&
				(mcpsIndication->Buffer[2] == 0x01) &&
				(mcpsIndication->Buffer[3] == 0x01))
			    {
				ComplianceTestRunning = true;

				ComplianceTest.Running = true;
				ComplianceTest.LinkCheck = false;
				ComplianceTest.DownLinkCounter = 0;
				ComplianceTest.UpLinkCounter = 0;
				ComplianceTest.IsConfirmed = false;
				ComplianceTest.TxSize = 0;
				ComplianceTest.TxData = LoRaWAN._tx_data;

				// START TIMER
				stm32l0_rtc_timer_start(&ComplianceTest.Timer, 5, 0, false);
                    
				mibReq.Param.AdrEnable = true;
				LoRaMacMibSetRequestConfirm(&mibReq);
                    
				if (LoRaWAN._Band->DutyCycle) {
				    LoRaMacTestSetDutyCycleOn(false);
				}

				LoRaWAN._tx_busy = true;
				
				if (LoRaWAN._tx_active) {
				    LoRaWAN._tx_active = false;

				    LoRaWAN._transmitCallback.queue();
				}
			    }
			}
		    }
#endif /* LORAWAN_COMPLIANCE_TEST */ 
		}
		else
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

		    LoRaWAN._receiveCallback.queue();
		}
	    }
	}
    }

#if defined(LORAWAN_COMPLIANCE_TEST)
    if (!ComplianceTestRunning)
#endif /* LORAWAN_COMPLIANCE_TEST */
    {
        if (LoRaWAN._tx_busy) 
        {
	    if (LoRaWAN._tx_join)
	    {
		LoRaWAN._Joined = true;
		
		LoRaWAN._tx_join = false;
		LoRaWAN._tx_busy = false;
		
		LoRaWAN._joinCallback.queue();
	    }
	    else
	    {
		LoRaWAN._tx_busy = false;
		    
		LoRaWAN._transmitCallback.queue();
	    }
        }
    }
}

void LoRaWANClass::__MlmeJoin( )
{
    MlmeReq_t mlmeReq;

    LoRaWAN._DevNonce += 1;

    LoRaWAN._saveDevNonce();

    mlmeReq.Type = MLME_JOIN;
    mlmeReq.Req.Join.DevEui = LoRaWAN._session.DevEui;
    mlmeReq.Req.Join.AppEui = LoRaWAN._session.AppEui;
    mlmeReq.Req.Join.AppKey = LoRaWAN._session.AppKey;
    mlmeReq.Req.Join.DevNonce = LoRaWAN._session.DevNonce0 + LoRaWAN._DevNonce;
    mlmeReq.Req.Join.Datarate = LoRaWAN._DataRate;
    
    if (LoRaMacMlmeRequest(&mlmeReq) == LORAMAC_STATUS_OK)
    {
        LoRaWAN._JoinTrials++;
    }
    else
    {
        LoRaWAN._tx_busy = false;
            
        LoRaWAN._joinCallback.call();
    }
}

void LoRaWANClass::__MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    MibRequestConfirm_t mibReq;

    switch (mlmeConfirm->MlmeRequest) {
    case MLME_JOIN:
        LoRaWAN._TimeOnAir += mlmeConfirm->TxTimeOnAir;

        if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)
        {
            if (LoRaWAN._Save)
            {
                LoRaWAN._getParams();
                LoRaWAN._saveParams();
            }

            LoRaWAN._restoreADR();

            mibReq.Type = MIB_APP_NONCE;
            LoRaMacMibGetRequestConfirm(&mibReq);
            LoRaWAN._session.AppNonce = mibReq.Param.AppNonce;

            mibReq.Type = MIB_NET_ID;
            LoRaMacMibGetRequestConfirm(&mibReq);
            LoRaWAN._session.NetID = mibReq.Param.NetID;

            mibReq.Type = MIB_DEV_ADDR;
            LoRaMacMibGetRequestConfirm(&mibReq);
            LoRaWAN._session.DevAddr = mibReq.Param.DevAddr;

            mibReq.Type = MIB_NWK_SKEY;
            LoRaMacMibGetRequestConfirm(&mibReq);
            memcpy(LoRaWAN._session.NwkSKey, mibReq.Param.NwkSKey, 16);

            mibReq.Type = MIB_APP_SKEY;
            LoRaMacMibGetRequestConfirm(&mibReq);
            memcpy(LoRaWAN._session.AppSKey, mibReq.Param.AppSKey, 16);

            LoRaWAN._saveSession();

            LoRaWAN._LinkCheckGateways = 1;

	    if ((LoRaWAN._Band->Region == LORAWAN_REGION_AU915) || (LoRaWAN._Band->Region == LORAWAN_REGION_US915))
	    {
		LoRaWAN._tx_join = true;
		
		armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)LoRaWANClass::__McpsJoin, NULL, 0);
	    }
	    else
	    {
	        LoRaWAN._Joined = true;

		LoRaWAN._tx_busy = false;
        
		LoRaWAN._joinCallback.queue();
	    }
        }
        else
        {
            if (LoRaWAN._JoinTrials < LoRaWAN._JoinRetries)
            {
                armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)LoRaWANClass::__MlmeJoin, NULL, 0);
            }
            else
            {
                LoRaWAN._tx_busy = false;
            
                LoRaWAN._joinCallback.queue();
            }
        }
        break;

    case MLME_LINK_CHECK:
        if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)
        {
	    LoRaWAN._LinkCheckWait = 0;
	    LoRaWAN._LinkCheckFail = 0;
            LoRaWAN._LinkCheckMargin = mlmeConfirm->DemodMargin;
            LoRaWAN._LinkCheckGateways = mlmeConfirm->NbGateways;

#if defined(LORAWAN_COMPLIANCE_TEST)
	    if (ComplianceTest.Running)
	    {
		ComplianceTest.LinkCheck = true;
		ComplianceTest.DemodMargin = mlmeConfirm->DemodMargin;
		ComplianceTest.NbGateways = mlmeConfirm->NbGateways;
	    }
            else
#endif /* LORAWAN_COMPLIANCE_TEST */
            {
                LoRaWAN._linkCheckCallback.queue();
            }
        }
        break;

    default:
        break;
    }
}

void LoRaWANClass::__MlmeIndication( MlmeIndication_t *mlmeIndication )
{
    if (mlmeIndication->MlmeIndication == MLME_SCHEDULE_UPLINK) {
        LoRaWAN._tx_pending = true;
    }
}

uint8_t LoRaWANClass::__GetBatteryLevel()
{
    return LoRaWAN._BatteryLevel;
}

LoRaWANClass LoRaWAN;
