/*
 * Copyright (c) 2018 Thomas Roell.  All rights reserved.
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

#ifndef FSKRADIO_H
#define FSKRADIO_H

#include <Arduino.h>

#define FSKRADIO_MAX_PAYLOAD_LENGTH      255

class FskRadioClass : public Stream
{
public:
    enum Modulation {
        FSK = 0,
        GFSK_BT_1_0,
        GFSK_BT_0_5,
        GFSK_BT_0_3,
    };

    enum AddressFiltering {
        FILTERING_NONE = 0,
        FILTERING_NODE,
        FILTERING_NODE_AND_BROADCAST,
    };

    enum IdleMode {
        IDLE_STANDBY = 0,
        IDLE_SLEEP,
    };

    FskRadioClass();

    int begin(unsigned long frequency);
    void end();

    bool busy();         // true == busy, false == ready

    int beginPacket();
    int beginPacket(uint8_t address);
    int endPacket(bool fixedPayloadLength = false);
    int sendPacket(const uint8_t *buffer, size_t size, bool fixedPayloadLength = false);
    int sendPacket(uint8_t address, const uint8_t *buffer, size_t size, bool fixedPayloadLength = false);

    int receive(unsigned int timeout = 0); // timeout in millis ... 0 is continuous, otherwise continuous with timeout
    int sense(int rssiThreshold, unsigned int senseTime);
    int standby();
    int sleep();

    virtual int availableForWrite();
    virtual void flush();
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
    virtual void purge();

    int packetRssi();
    bool packetBroadcast();

    int setFrequency(unsigned long frequency);
    int setTxPower(int level);
    int setDeviation(unsigned int deviation);
    int setBandwidth(unsigned int bandwidth);
    int setBandwidthAfc(unsigned int bandwidth);
    int setBitRate(unsigned int bitrate);
    int setPreambleLength(unsigned int length); // bytes
    int setFixedPayloadLength(unsigned int length); // bytes
    int setMaxPayloadLength(unsigned int length); // bytes
    int setSyncTimeout(unsigned int timeout); // bytes, if non-zero receive() returns early if no sync within timeout
    int setSyncWord(const uint8_t *data, unsigned int size);
    int setAddressFiltering(AddressFiltering addressFiltering);
    int setNodeAddress(uint8_t address);
    int setBroadcastAddress(uint8_t address);
    int setModulation(Modulation modulation);
    int setLnaBoost(bool enable);
    int enableAfc();
    int disableAfc();
    int enableWhitening();
    int disableWhitening();
    int enableCrc();
    int disableCrc();

    int setIdleMode(IdleMode mode);

    void enableWakeup();
    void disableWakeup();

    void onTransmit(void(*callback)(void));
    void onTransmit(Callback callback);
    void onReceive(void(*callback)(void));
    void onReceive(Callback callback);

private:
    bool              _initialized;
    volatile uint8_t  _busy;      // 0 idle, 1 rx, 2 tx, 3 cad

    uint8_t           *_tx_data;
    uint8_t           _tx_size;
    uint8_t           _tx_active;

    uint8_t           *_rx_data;
    volatile uint16_t _rx_read;
    volatile uint16_t _rx_write;
    uint16_t          _rx_index;
    uint16_t          _rx_next;
    uint8_t           _rx_size;
    bool              _rx_address;
    uint8_t           _rx_node;
    bool              _rx_broadcast;
    int16_t           _rx_rssi;

    bool              _updateFrequency;
    bool              _updateTxConfig;
    bool              _updateRxConfig;

    uint32_t          _frequency;
    int8_t            _txPower;
    uint16_t          _fixedPayloadLength;
    uint32_t          _deviation;
    uint32_t          _bandwidth;
    uint32_t          _bandwidthAfc;
    uint32_t          _bitrate;
    uint16_t          _preambleLength;
    uint16_t          _syncTimeout;
    uint8_t           _addressFiltering;
    uint8_t           _nodeAddress;
    uint8_t           _broadcastAddress;
    uint8_t           _modulation;
    bool              _afcOn;
    bool              _crcOn;

    bool              _wakeup;
    bool              _implicitHeader;
    uint32_t          _timeout;

    Callback          _transmitCallback;
    Callback          _receiveCallback;

    static bool       __TxStart(void);
    static bool       __RxStart(void);
    static bool       __Sense(void);
    static bool       __Standby(void);
    static bool       __Sleep(void);
    static void       __TxDone(void);
    static void       __RxDone(uint8_t *data, uint16_t size, int16_t rssi, int8_t snr);
    static void       __RxTimeout(void);
};

extern FskRadioClass FskRadio;

#endif // FSKRADIO_H
