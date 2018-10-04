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

#ifndef LORAWAN_H
#define LORAWAN_H

#include <Arduino.h>

#define LORAWAN_DEFAULT_PORT           1
#define LORAWAN_MAX_PAYLOAD_SIZE       242
#define LORAWAN_TX_BUFFER_SIZE         LORAWAN_MAX_PAYLOAD_SIZE
#define LORAWAN_RX_BUFFER_SIZE         (2 * (2 + LORAWAN_MAX_PAYLOAD_SIZE) + 1)

#define LORAWAN_ACTIVATION_NONE        0
#define LORAWAN_ACTIVATION_OTAA        1
#define LORAWAN_ACTIVATION_ABP         2

#define LORAWAN_REGION_NONE            0
#define LORAWAN_REGION_AS923           1
#define LORAWAN_REGION_AU915           2
#define LORAWAN_REGION_CN470           3
#define LORAWAN_REGION_CN779           4
#define LORAWAN_REGION_EU433           5
#define LORAWAN_REGION_EU868           6
#define LORAWAN_REGION_IN865           7
#define LORAWAN_REGION_KR920           8
#define LORAWAN_REGION_US915           9

#define LORAWAN_BATTERY_LEVEL_EXTERNAL 0
#define LORAWAN_BATTERY_LEVEL_EMPTY    1
#define LORAWAN_BATTERY_LEVEL_FULL     254
#define LORAWAN_BATTERY_LEVEL_UNKNOWN  255

typedef struct {
    uint32_t Header;
    uint8_t  AppEui[8];
    uint8_t  AppKey[16];
    uint8_t  DevEui[8];
    uint32_t DevAddr;
    uint8_t  NwkSKey[16];
    uint8_t  AppSKey[16];
    uint32_t Crc32;
} LoRaWANCommissioning;

typedef struct {
    uint32_t Header;
    uint8_t  Activation;
    uint8_t  Reserved;
    uint8_t  DevEui[8];
    uint8_t  AppEui[8];
    uint8_t  AppKey[16];
    uint16_t DevNonce0;
    uint32_t AppNonce;
    uint32_t NetID;
    uint32_t DevAddr;
    uint8_t  NwkSKey[16];
    uint8_t  AppSKey[16];
    uint32_t Crc32;
} LoRaWANSession;

typedef struct {
    uint32_t Header;
    uint16_t ReceiveDelay;              /* JOIN_ACCEPT, RX_TIMING_SETUP */
    uint8_t  RX1DrOffset;               /* JOIN_ACCEPT, RX_PARAM_SETUP */
    uint8_t  RX2DataRate;               /* JOIN_ACCEPT, RX_PARAM_SETUP */
    uint32_t RX2Frequency;              /* RX_PARAM_SETUP */
    uint8_t  Region;
    uint8_t  Network;                   /* (_SubBand & 0x0F) | (_PublicNetwork << 7) */
    uint8_t  UpLinkDwellTime;           /* TX_PARAM_SETUP */
    uint8_t  DownLinkDwellTime;         /* TX_PARAM_SETUP */
    float    MaxEIRP;                   /* TX_PARAM_SETUP */
    uint16_t ChannelsMask[6];           /* JOIN_ACCEPT, ADR */
    uint32_t ChannelsFrequency[16];     /* NEW_CHANNEL */
    uint8_t  ChannelsDrRange[16];       /* NEW_CHANNEL, 4 MSBs is MAX, 4 LSBs is MIN */
    uint32_t ChannelsRX1Frequency[16];  /* DL_CHANNEL */
    uint32_t Crc32;
} LoRaWANParams;

extern const struct LoRaWANBand AS923;
extern const struct LoRaWANBand AU915;
extern const struct LoRaWANBand EU868;
extern const struct LoRaWANBand IN865;
extern const struct LoRaWANBand KR920;
extern const struct LoRaWANBand US915;

class LoRaWANClass : public Stream
{
public:
    LoRaWANClass();

    int begin(const struct LoRaWANBand &band);

    int joinOTAA();
    int joinOTAA(const char *appEui, const char *appKey, const char *devEui = NULL);
    int rejoinOTAA();

    int joinABP();
    int joinABP(const char *devAddr, const char *nwkSKey, const char *appSKey);

    bool joined();       // true == joined, false == non-joined
    bool confirmed();    // last message was confired
    bool pending();      // more downlink data pending
    bool busy();         // true == busy, false == ready

    operator bool() { return joined(); }

    int beginPacket(uint8_t port = LORAWAN_DEFAULT_PORT);
    int endPacket(bool confirm = false);
    int sendPacket(const uint8_t *buffer, size_t size, bool confirmed = false);
    int sendPacket(uint8_t port, const uint8_t *buffer, size_t size, bool confirmed = false);
    int linkCheck();
    int ping();

    virtual int availableForWrite();
    virtual size_t write(uint8_t data);
    virtual size_t write(const uint8_t *buffer, size_t size);
    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    using Print::write;

    int parsePacket();
    virtual int available();
    virtual int read();
    virtual size_t read(uint8_t *buffer, size_t size);
    virtual int peek();
    virtual void flush();
    uint8_t remotePort();

    int lastRSSI() { return _RSSI; }
    int lastSNR() { return _SNR; }
    unsigned int linkMargin() { return _LinkCheckMargin; }
    unsigned int linkGateways() { return _LinkCheckGateways; }

    void onJoin(void(*callback)(void));
    void onJoin(Callback callback);
    void onLinkCheck(void(*callback)(void));
    void onLinkCheck(Callback callback);
    void onReceive(void(*callback)(void));
    void onReceive(Callback callback);
    void onTransmit(void(*callback)(void));
    void onTransmit(Callback callback);

    int setAppEui(const char *appEui);
    int setAppKey(const char *appKey);
    int setDevEui(const char *devEui);
    int setDevAddr(const char *devAddr);
    int setNwkSKey(const char *nwkSKey);
    int setAppSKey(const char *appSKey);
    int getDevEui(char *buffer, size_t size);

    int getNextTxTime();
    int getMaxPayloadSize();
    int getDataRate();
    float getTxPower();
    int getRepeat();
    unsigned long getTimeOnAir() { return _TimeOnAir; }
    unsigned long getUpLinkCounter() { return _UpLinkCounter; }
    unsigned long getDownLinkCounter() { return _DownLinkCounter; }

    int setSaveSession(bool enable);

    int setJoinDelay1(unsigned int delay);
    int setJoinDelay2(unsigned int delay);
    int setJoinRetries(unsigned int n);

    int setLinkCheckLimit(unsigned int n);
    int setLinkCheckDelay(unsigned int n);
    int setLinkCheckThreshold(unsigned int n);

    int setADR(bool enable);
    int setDataRate(unsigned int datarate);
    int setTxPower(float power); // 2dm to 20dbm 
    int setRepeat(unsigned int n);
    int setRetries(unsigned int n);

    int setPublicNetwork(bool enable);
    int setSubBand(unsigned int subband);
    int setReceiveDelay(unsigned int delay);
    int setRX1DrOffset(unsigned int offset);
    int setRX2Channel(unsigned long frequency, unsigned int datarate);
    int addChannel(unsigned int index, unsigned long frequency, unsigned int drMin, unsigned int drMax);
    int removeChannel(unsigned int index);
    int enableChannel(unsigned int index);
    int disableChannel(unsigned int index);
    int setDownLinkChannel(unsigned int index, unsigned long frequency);
    int setUpLinkDwellTime(bool enable);
    int setDownLinkDwellTime(bool enable);
    int setMaxEIRP(float eirp);
    int setAntennaGain(float gain);

    int setDutyCycle(bool enable);
    int setComplianceTest(bool enable);

    int setBatteryLevel(unsigned int level);
 
private:
    uint8_t           *_rx_data;
    volatile uint16_t _rx_read;
    volatile uint16_t _rx_write;
    uint16_t          _rx_index;
    uint16_t          _rx_next;
    uint8_t           _rx_size;
    uint8_t           _rx_port;
    volatile uint8_t  _rx_pending;

    uint8_t           *_tx_data;
    uint8_t           _tx_size;
    uint8_t           _tx_port;
    uint8_t           _tx_confirmed;
    volatile uint8_t  _tx_join;
    volatile uint8_t  _tx_active;
    volatile uint8_t  _tx_pending;
    volatile uint8_t  _tx_ack;
    volatile uint8_t  _tx_busy;

    const struct LoRaWANBand *_Band;
    volatile uint8_t  _Joined;
    uint8_t           _Save;
    uint8_t           _BatteryLevel;

    uint8_t           _DutyCycle;
    uint8_t           _PublicNetwork;
    uint8_t           _SubBand;
    uint8_t           _DataRate;
    uint8_t           _Retries;

    uint8_t           _ComplianceTest;

    uint16_t          _JoinRetries;    
    volatile uint16_t _JoinTrials;    

    uint8_t           _AdrEnable;
    volatile uint8_t  _AdrWait;
    uint8_t           _AdrLastDataRate;
    uint8_t           _AdrLastTxPower;

    uint8_t           _LinkCheckLimit;
    uint8_t           _LinkCheckDelay;
    uint8_t           _LinkCheckThreshold;
    volatile uint8_t  _LinkCheckCount;
    volatile uint8_t  _LinkCheckWait;
    volatile uint8_t  _LinkCheckFail;
    volatile uint8_t  _LinkCheckGateways;
    volatile uint8_t  _LinkCheckMargin;

    volatile int16_t  _RSSI;
    volatile int8_t   _SNR;
    volatile uint16_t _DevNonce;
    volatile uint32_t _TimeOnAir;
    volatile uint32_t _UpLinkCounter;
    volatile uint32_t _DownLinkCounter;

    LoRaWANSession    _session;
    LoRaWANParams     _params;

    Callback          _joinCallback;
    Callback          _linkCheckCallback;
    Callback          _receiveCallback;
    Callback          _transmitCallback;

    void              _saveSession();
    bool              _restoreSession();
    void              _saveADR();
    bool              _restoreADR();
    void              _saveParams();
    bool              _restoreParams();
    bool              _getParams();
    bool              _setParams();
    void              _saveDevNonce();
    void              _saveUpLinkCounter();
    void              _saveDownLinkCounter();
    bool              _loadCommissioning(LoRaWANCommissioning *commissioning);
    bool              _storeCommissioning(LoRaWANCommissioning *commissioning);
    int               _joinOTAA(LoRaWANCommissioning *commissioning);
    int               _joinABP(LoRaWANCommissioning *commissioning);
    int               _rejoinOTAA();
    int               _rejoinABP();
    bool              _send();

    static bool       _eepromProgram(uint32_t address, const uint8_t *data, uint32_t size);
    static bool       _eepromRead(uint32_t address, uint8_t *data, uint32_t size);
    static void       _eepromSync(class LoRaWANClass *self);

    static uint8_t    __GetBatteryLevel();
    static void       __McpsJoin(void);
    static void       __McpsSend(void);
    static void       __McpsConfirm(struct sMcpsConfirm*);
    static void       __McpsIndication(struct sMcpsIndication*);
    static void       __MlmeJoin(void);
    static void       __MlmeConfirm(struct sMlmeConfirm*);
    static void       __MlmeIndication(struct sMlmeIndication*);
};

extern LoRaWANClass LoRaWAN;

#endif // LORAWAN_H
