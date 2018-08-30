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

#ifndef LORARADIO_H
#define LORARADIO_H

#include <Arduino.h>

#define LORARADIO_MAX_PAYLOAD_LENGTH     255

class LoRaRadioClass : public Stream
{
public:
    enum Bandwidth {
        BW_125 = 0,
        BW_250,
        BW_500,
    };

    enum SpreadingFactor {
        SF_7 = 7,
        SF_8,
        SF_9,
        SF_10,
        SF_11,
        SF_12,
    };
    
    enum CodingRate {
        CR_4_5 = 1,
        CR_4_6,
        CR_4_7,
        CR_4_8,
    };

    enum IdleMode {
        IDLE_STANDBY = 0,
        IDLE_SLEEP,
    };

    LoRaRadioClass();

    int begin(unsigned long frequency);
    void end();

    bool busy();         // true == busy, false == ready
    bool cadDetected();

    int beginPacket();
    int endPacket(bool fixedPayloadLength = false);
    int sendPacket(const uint8_t *buffer, size_t size, bool fixedPayloadLength = false);

    int receive(unsigned int timeout = 0); // timeout in millis ... 0 is continous, otherwise continuous with timeout
    int sense(int rssiThreshold, unsigned int senseTime);
    int cad();
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
    int packetSnr();

    int setFrequency(unsigned long frequency);
    int setTxPower(int level);
    int setBandwidth(Bandwidth bw);
    int setSpreadingFactor(SpreadingFactor sf);
    int setCodingRate(CodingRate cr);
    int setPreambleLength(unsigned int length); // symbols
    int setFixedPayloadLength(unsigned int length); // 0 == variable, else fixed; bytes
    int setSymbolTimeout(unsigned int timeout); // symbols, if non-zero receive() returns early if no sync within timeout
    int setIQInverted(bool enable);
    int setPublicNetwork(bool enable);
    int setLnaBoost(bool enable);
    int enableCrc();
    int disableCrc();

    int setIdleMode(IdleMode mode);

    void enableWakeup();
    void disableWakeup();

    void onTransmit(void(*callback)(void));
    void onTransmit(Callback callback);
    void onReceive(void(*callback)(void));
    void onReceive(Callback callback);
    void onCad(void(*callback)(void));
    void onCad(Callback callback);

private:
    bool              _initialized;
    volatile uint8_t  _busy;      // 0 idle, 1 rx, 2 tx, 3 cad
    volatile bool     _cadDetected;

    uint8_t           *_tx_data;
    uint8_t           _tx_size;
    uint8_t           _tx_active;

    uint8_t           *_rx_data;
    volatile uint16_t _rx_read;
    volatile uint16_t _rx_write;
    uint16_t          _rx_index;
    uint16_t          _rx_next;
    uint8_t           _rx_size;
    int8_t            _rx_snr;
    int16_t           _rx_rssi;

    bool              _updateFrequency;
    bool              _updateTxConfig;
    bool              _updateRxConfig;

    uint32_t          _frequency;
    int8_t            _txPower;
    uint8_t           _fixedPayloadLength;
    uint8_t           _signalBandwidth;
    uint8_t           _spreadingFactor;
    uint8_t           _codingRate;
    uint16_t          _preambleLength;
    uint16_t          _symbolTimeout;
    bool              _iqInverted;
    bool              _crcOn;
    bool              _publicNetwork;
    bool              _lnaBoost;

    bool              _wakeup;
    bool              _implicitHeader;
    uint32_t          _timeout;

    Callback          _transmitCallback;
    Callback          _receiveCallback;
    Callback          _cadCallback;

    static bool       __TxStart(void);
    static bool       __RxStart(void);
    static bool       __CadStart(void);
    static bool       __Sense(void);
    static bool       __Standby(void);
    static bool       __Sleep(void);
    static void       __TxDone(void);
    static void       __RxDone(uint8_t *data, uint16_t size, int16_t rssi, int8_t snr);
    static void       __RxTimeout(void);
    static void       __CadDone(bool cadDetected);
};

extern LoRaRadioClass LoRaRadio;

#endif // LORARADIO_H
