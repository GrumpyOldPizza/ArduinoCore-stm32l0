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

#define LORAWAN_DEFAULT_PORT     1
#define LORAWAN_MAX_PAYLOAD_SIZE 242
#define LORAWAN_RX_BUFFER_SIZE   (2 * (3 + LORAWAN_MAX_PAYLOAD_SIZE))

#define LORAWAN_JOINED_NONE      0
#define LORAWAN_JOINED_OTAA      1
#define LORAWAN_JOINED_ABP       2

typedef struct {
    uint8_t  Joined;
    uint8_t  DataRate;
    uint8_t  TxPower;
    uint32_t UpLinkCounter;
    uint32_t DownLinkCounter;
    uint8_t  DevEui[8];
    uint8_t  AppEui[8];
    uint8_t  AppKey[16];
    uint32_t DevAddr;
    uint8_t  NwkSKey[16];
    uint8_t  AppSKey[16];
} LoRaWANSession;

typedef enum {
    AS923 = 0,
    AU915,
    EU868,
    KR920,
    IN865,
    US915,
} LoRaWANBand;

class LoRaWANClass : public Stream
{
public:
    LoRaWANClass();

    int begin(LoRaWANBand band);
    void stop();

    int joinOTAA(const char *appEui, const char *appKey, const char *devEui);
    int joinABP(const char *devAddr, const char *nwkSKey, const char *appSKey);

    bool joined();    // true == joined, false == non-joined
    bool confirmed(); // last message was confired
    bool checked();   // link check answer received
    bool busy();      // 1 busy, 0 ready

    operator bool() { return joined(); }

    int beginPacket(uint8_t port = LORAWAN_DEFAULT_PORT);
    int endPacket(bool confirm = false);
    int sendPacket(const uint8_t *buffer, size_t size, bool confirm = false);
    int sendPacket(uint8_t port, const uint8_t *buffer, size_t size, bool confirm = false);

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

    int lastRSSI();
    int lastSNR();

    void linkCheck();
    int linkMargin();
    int linkGateways();

    void onJoin(void(*callback)(void));
    void onJoin(Callback callback);
    void onReceive(void(*callback)(void));
    void onReceive(Callback callback);
    void onTransmit(void(*callback)(void));
    void onTransmit(Callback callback);

    int getDevEui(char *buffer, size_t size);
    int getMaxPayloadSize();
    unsigned int getDataRate();
    unsigned int getTxPower();
    unsigned long getUpLinkCounter();
    unsigned long getDownLinkCounter();

    int setJoinDelay1(unsigned int delay);
    int setJoinDelay2(unsigned int delay);
    int setJoinRetries(unsigned int n);

    int setPublicNetwork(bool enable);
    int setADR(bool enable);
    int setDataRate(unsigned int datarate);
    int setTxPower(unsigned int power); // 2dm to 20dbm 
    int setConfirmRetries(unsigned int n);

    int setReceiveDelay(unsigned int delay);
    int setRX2Channel(unsigned long frequency, unsigned int datarate);
    int setSubBand(unsigned int subband);
    int addChannel(unsigned int index, unsigned long frequency, unsigned int drMin, unsigned int drMax);
    int removeChannel(unsigned int index);
    int enableChannel(unsigned int index);
    int disableChannel(unsigned int index);
    int setAntennaGain(float gain);

    int setDutyCycle(bool enable);
    int setReceiveWindows(bool enable);
 
private:
    uint8_t           _initialized;

    uint8_t           _rx_data[LORAWAN_RX_BUFFER_SIZE];
    volatile uint16_t _rx_read;
    volatile uint16_t _rx_write;
    uint16_t          _rx_index;
    uint16_t          _rx_next;
    uint8_t           _rx_size;
    uint8_t           _rx_port;
    uint8_t           _rx_multicast;
    volatile int8_t   _rx_snr;
    volatile int16_t  _rx_rssi;
    volatile uint8_t  _rx_margin;
    volatile uint8_t  _rx_gateways;
    volatile uint8_t  _rx_ack;

    uint8_t           _tx_data[LORAWAN_MAX_PAYLOAD_SIZE];
    uint8_t           _tx_size;
    uint8_t           _tx_port;
    uint8_t           _tx_active;
    uint8_t           _tx_confirm;
    volatile uint8_t  _tx_ack;
    volatile uint8_t  _tx_busy;

    uint8_t           _JoinRetries;
    uint8_t           _ConfirmRetries;

    uint8_t           _Band;
    uint8_t           _PublicNetwork;
    uint8_t           _AdrEnable;
    uint8_t           _DataRate;
    uint8_t           _TxPower;

    LoRaWANSession    _session;

    Callback          _joinCallback;
    Callback          _receiveCallback;
    Callback          _transmitCallback;

    static void __McpsResend(void);
    static void __McpsConfirm(struct sMcpsConfirm*);
    static void __McpsIndication(struct sMcpsIndication*);
    static void __MlmeConfirm(struct sMlmeConfirm*);
};

extern LoRaWANClass LoRaWAN;

#endif // LORAWAN_H
