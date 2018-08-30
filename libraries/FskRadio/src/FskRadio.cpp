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

#include "Arduino.h"
#include "FskRadio.h"
#include "wiring_private.h"
#include "../../../system/STM32L0xx/Source/LoRa/Radio/radio.h"

#define FSKRADIO_TX_BUFFER_SIZE          FSKRADIO_MAX_PAYLOAD_LENGTH
#define FSKRADIO_RX_BUFFER_SIZE          512

static uint32_t FskRadioBuffer[(FSKRADIO_TX_BUFFER_SIZE + 3) / 4 + (FSKRADIO_RX_BUFFER_SIZE + 3) / 4];

static FskRadioClass *FskRadioInstance = NULL;

static inline bool FskRadioCall(bool(*routine)(void))
{
    IRQn_Type irq;

    irq = (IRQn_Type)((__get_IPSR() & 0x1ff) - 16);

    if (irq == Reset_IRQn) {
        return (bool)armv6m_svcall_0((uint32_t)routine);
    } else {
        if ((irq == SVC_IRQn) || (irq == PendSV_IRQn)) {
            return (*routine)();
        }
    }

    return false;
}

FskRadioClass::FskRadioClass()
{
    _tx_data = (uint8_t*)&FskRadioBuffer[0];
    _rx_data = (uint8_t*)&FskRadioBuffer[(FSKRADIO_TX_BUFFER_SIZE + 3) / 4];

    _initialized = false;
}

int FskRadioClass::begin(unsigned long frequency)
{
    static const RadioEvents_t FskRadioEvents = {
        FskRadioClass::__TxDone,
        NULL,
        FskRadioClass::__RxDone,
        FskRadioClass::__RxTimeout,
        NULL,
        NULL,
        NULL,
    };

    if (FskRadioInstance) {
        return 0;
    }

    if (_initialized) {
        return 0;
    }

    _initialized = true;
    _busy = 0;

    _tx_size = 0;
    _tx_active = false;

    _rx_read = 0;
    _rx_write = 0;
    _rx_index = 0;
    _rx_next = 0;
    _rx_size = 0;
    _rx_rssi = 0;
    _rx_broadcast = false;

    _updateFrequency = true;
    _updateTxConfig = true;
    _updateRxConfig = true;

    _frequency = frequency;
    _txPower = 14;
    _fixedPayloadLength = 0;
    _deviation = 25000;
    _bandwidth = 100000;
    _bandwidthAfc = 100000;
    _bitrate = 50000;
    _preambleLength = 8;
    _syncTimeout = 0;
    _addressFiltering = FILTERING_NONE;
    _nodeAddress = 0x00;
    _broadcastAddress = 0xff;
    _modulation = FSK;
    _afcOn = true;
    _crcOn = true;

    _wakeup = false;
    _implicitHeader = false;
    _timeout = 0;

    FskRadioInstance = this;

    Radio.Init(&FskRadioEvents, frequency);

    Radio.SetModem(MODEM_FSK);

    Radio.SetMaxPayloadLength(MODEM_FSK, FSKRADIO_MAX_PAYLOAD_LENGTH);

    Radio.SetIdleMode(IDLE_STANDBY);

    return 1;
}

void FskRadioClass::end()
{
    if (_initialized) {
        Radio.Sleep();
    }

    _initialized = false;

    FskRadioInstance = NULL;
}

bool FskRadioClass::busy()
{
    return _busy != 0;
}

int FskRadioClass::beginPacket()
{
    if (!_initialized) {
        return 0;
    }

    _tx_size = 0;
    _tx_active = true;

    return 1;
}

int FskRadioClass::beginPacket(uint8_t address)
{
    if (!_initialized) {
        return 0;
    }

    _tx_size = 1;

    _tx_data[0] = address;

    _tx_active = true;

    return 1;
}

int FskRadioClass::endPacket(bool fixedPayloadLength)
{
    if (!_tx_active) {
        return 0;
    }

    if (_implicitHeader != fixedPayloadLength) {
        _implicitHeader = fixedPayloadLength;

        _updateTxConfig = true;
    }

    return FskRadioCall(__TxStart);
}

int FskRadioClass::sendPacket(const uint8_t *buffer, size_t size, bool fixedPayloadLength)
{
    if (!_initialized) {
        return 0;
    }

    if (_tx_active) {
        return 0;
    }

    if (size > FSKRADIO_MAX_PAYLOAD_LENGTH) {
        return 0;
    }

    _tx_size = size;

    memcpy(&_tx_data[0], buffer, size);

    _tx_active = true;

    return endPacket(fixedPayloadLength);
}

int FskRadioClass::sendPacket(uint8_t address, const uint8_t *buffer, size_t size, bool fixedPayloadLength)
{
    if (!_initialized) {
        return 0;
    }

    if (_tx_active) {
        return 0;
    }

    if ((1 + size) > FSKRADIO_MAX_PAYLOAD_LENGTH) {
        return 0;
    }

    _tx_size = 1 + size;

    _tx_data[0] = address;

    memcpy(&_tx_data[1], buffer, size);

    _tx_active = true;

    return endPacket(fixedPayloadLength);
}

int FskRadioClass::receive(unsigned int timeout)
{
    if (!_initialized) {
        return 0;
    }

    _timeout = timeout;

    return FskRadioCall(__RxStart);
}

int FskRadioClass::sense(int rssiThreshold, unsigned int senseTime)
{
    IRQn_Type irq;
    bool isChannelFree;

    if (!_initialized) {
        return 0;
    }

    irq = (IRQn_Type)((__get_IPSR() & 0x1ff) - 16);

    if (irq != Reset_IRQn) {
        return 0;
    }

    if (!FskRadioCall(__Sense)) {
        return 0;
    }

    isChannelFree = Radio.IsChannelFree(MODEM_FSK, _frequency, rssiThreshold, senseTime);

    _busy = 0;

    return (isChannelFree ? 1 : 0);
}

int FskRadioClass::standby()
{
    if (!_initialized) {
        return 0;
    }

    return FskRadioCall(__Standby);
}

int FskRadioClass::sleep()
{
    if (!_initialized) {
        return 0;
    }

    return FskRadioCall(__Sleep);
}

int FskRadioClass::availableForWrite()
{
    if (!_tx_active) {
        return 0;
    }

    return FSKRADIO_MAX_PAYLOAD_LENGTH - _tx_size;
}

void FskRadioClass::flush()
{
}

size_t FskRadioClass::write(uint8_t data)
{
    if (!_tx_active) {
        return 0;
    }

    if (_tx_size >= FSKRADIO_MAX_PAYLOAD_LENGTH) {
        return 0;
    }

    _tx_data[_tx_size++] = data;

    return 1;
}

size_t FskRadioClass::write(const uint8_t *data, size_t size)
{
    if (!_tx_active) {
        return 0;
    }

    if (size > (unsigned int)(FSKRADIO_MAX_PAYLOAD_LENGTH - _tx_size)) {
        size = FSKRADIO_MAX_PAYLOAD_LENGTH - _tx_size;
    }

    memcpy(&_tx_data[_tx_size], data, size);

    _tx_size += size;

    return size;
}

int FskRadioClass::parsePacket()
{
    uint32_t rx_read;
    uint8_t rx_broadcast;
    uint16_t rx_rssi;

    if (_rx_size)
    {
        _rx_index = _rx_next;
        _rx_read = _rx_next;
        _rx_size = 0;
        _rx_broadcast = false;
        _rx_rssi = 0;
    }

    rx_read = _rx_read;

    if (rx_read == _rx_write) {
        return 0;
    }

    _rx_size = _rx_data[rx_read];

    if (++rx_read == FSKRADIO_RX_BUFFER_SIZE) {
        rx_read = 0;
    }

    rx_rssi = (_rx_data[rx_read] << 0);

    if (++rx_read == FSKRADIO_RX_BUFFER_SIZE) {
        rx_read = 0;
    }

    rx_rssi |= (_rx_data[rx_read] << 8);

    if (++rx_read == FSKRADIO_RX_BUFFER_SIZE) {
        rx_read = 0;
    }

    rx_broadcast = _rx_data[rx_read];

    if (++rx_read == FSKRADIO_RX_BUFFER_SIZE) {
        rx_read = 0;
    }

    _rx_index = rx_read;
    _rx_rssi = (int16_t)rx_rssi;
    _rx_broadcast = (rx_broadcast != 0);

    rx_read += _rx_size;

    if (rx_read >= FSKRADIO_RX_BUFFER_SIZE) {
        rx_read -= FSKRADIO_RX_BUFFER_SIZE;
    }

    _rx_next = rx_read;

    return _rx_size;
}

int FskRadioClass::available()
{
    if (_rx_next >= _rx_index) {
        return (_rx_next - _rx_index);
    } else {
        return (_rx_next + (FSKRADIO_RX_BUFFER_SIZE - _rx_index));
    }
}

int FskRadioClass::read()
{
    int data;
    
    if (_rx_index == _rx_next) {
        return -1;
    }

    data = _rx_data[_rx_index++];

    if (_rx_index == FSKRADIO_RX_BUFFER_SIZE) {
        _rx_index = 0;
    }

    return data;
}

size_t FskRadioClass::read(uint8_t *buffer, size_t size)
{
    size_t count;

    if (_rx_index == _rx_next) {
        return 0;
    }

    if (_rx_next < _rx_index)
    {
        count = size;

        if (count > (size_t)(FSKRADIO_RX_BUFFER_SIZE - _rx_index))
        {
            count = FSKRADIO_RX_BUFFER_SIZE - _rx_index;
        }

        if (count)
        {
            memcpy(buffer, &_rx_data[_rx_index], count);
            
            buffer += count;
            size -= count;
            
            _rx_index += count;
            
            if (_rx_index == FSKRADIO_RX_BUFFER_SIZE) {
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

int FskRadioClass::peek(void)
{
    if (_rx_index == _rx_next) {
        return -1;
    }

    return _rx_data[_rx_index];
}

void FskRadioClass::purge(void)
{
    do
    {
        _rx_read = _rx_write;
        _rx_index = _rx_write;
        _rx_next = _rx_write;
        _rx_size = 0;
        _rx_rssi = 0;
        _rx_broadcast = false;
    }
    while (_rx_read != _rx_write);
}

int FskRadioClass::packetRssi()
{
    return _rx_rssi;
}

bool FskRadioClass::packetBroadcast()
{
    return _rx_broadcast;
}

int FskRadioClass::setFrequency(unsigned long frequency)
{
    if (!_initialized) {
        return 0;
    }

    if (_frequency != frequency) {
        _frequency = frequency;

        _updateFrequency = true;
    }

    return 1;
}

int FskRadioClass::setTxPower(int level)
{
    if (!_initialized) {
        return 0;
    }

    if (_txPower != level) {
        _txPower = level;

        _updateTxConfig = true;
    }

    return 1;
}

int FskRadioClass::setDeviation(unsigned int deviation)
{
    if (!_initialized) {
        return 0;
    }

    if (_deviation != deviation) {
        _deviation = deviation;

        _updateTxConfig = true;
    }

    return 1;
}

int FskRadioClass::setBandwidth(unsigned int bandwidth)
{
    if (!_initialized) {
        return 0;
    }

    if (_bandwidth != bandwidth) {
        _bandwidth = bandwidth;

        _updateRxConfig = true;
    }

    return 1;
}

int FskRadioClass::setBandwidthAfc(unsigned int bandwidth)
{
    if (!_initialized) {
        return 0;
    }

    if (_bandwidthAfc != bandwidth) {
        _bandwidthAfc = bandwidth;

        _updateRxConfig = true;
    }

    return 1;
}

int FskRadioClass::setBitRate(unsigned int bitrate)
{
    if (!_initialized) {
        return 0;
    }

    if (_bitrate != bitrate) {
        _bitrate = bitrate;

        _updateTxConfig = true;
        _updateRxConfig = true;
    }

    return 1;
}

int FskRadioClass::setPreambleLength(unsigned int length)
{
    if (!_initialized) {
        return 0;
    }

    if ((length < 2) || (length > 65535)) {
        return 0;
    }

    if (_preambleLength != length) {
        _preambleLength = length;

        _updateTxConfig = true;
        _updateRxConfig = true;
    }

    return 1;
}

int FskRadioClass::setFixedPayloadLength(unsigned int length)
{
    if (!_initialized) {
        return 0;
    }

    if (length > FSKRADIO_MAX_PAYLOAD_LENGTH) {
        return 0;
    }

    if (_fixedPayloadLength != length) {
        _fixedPayloadLength = length;

        _updateRxConfig = true;
    }

    return 1;
}

int FskRadioClass::setMaxPayloadLength(unsigned int length)
{
    if (!_initialized) {
        return 0;
    }

    if (_busy) {
        return 0;
    }

    if (length > FSKRADIO_MAX_PAYLOAD_LENGTH) {
        return 0;
    }

    Radio.SetMaxPayloadLength(MODEM_FSK, length);

    return 1;
}

int FskRadioClass::setSyncTimeout(unsigned int timeout)
{
    if (!_initialized) {
        return 0;
    }

    if (timeout > 65535) {
        return 0;
    }

    if (_syncTimeout != timeout) {
        _syncTimeout = timeout;

        _updateRxConfig = true;
    }

    return 1;
}

int FskRadioClass::setSyncWord(const uint8_t *data, unsigned int size)
{
    if (!_initialized) {
        return 0;
    }

    if (_busy) {
        return 0;
    }

    if (size > 1) {
        return 0;
    }

    if (size > 8) {
        return 0;
    }

    Radio.SetSyncWord(data, size);

    return 1;
}

int FskRadioClass::setAddressFiltering(AddressFiltering filtering)
{
    if (!_initialized) {
        return 0;
    }

    if (_busy) {
        return 0;
    }
    
    Radio.SetAddressFiltering(filtering);

    _addressFiltering = filtering;

    return 1;
}

int FskRadioClass::setNodeAddress(uint8_t address)
{
    if (!_initialized) {
        return 0;
    }

    if (_busy) {
        return 0;
    }
    
    Radio.SetNodeAddress(address);

    _nodeAddress = address;

    return 1;
}

int FskRadioClass::setBroadcastAddress(uint8_t address)
{
    if (!_initialized) {
        return 0;
    }

    if (_busy) {
        return 0;
    }
    
    Radio.SetBroadcastAddress(address);

    _broadcastAddress = address;

    return 1;
}

int FskRadioClass::setModulation(Modulation modulation)
{
    if (!_initialized) {
        return 0;
    }

    if (_busy) {
        return 0;
    }
    
    Radio.SetModulation(modulation);
    
    _modulation = modulation;

    return 1;
}

int FskRadioClass::setLnaBoost(bool enable)
{
    if (!_initialized) {
        return 0;
    }

    if (_busy) {
        return 0;
    }
    
    Radio.SetLnaBoost(enable);

    return 1;
}

int FskRadioClass::enableAfc()
{
    if (!_initialized) {
        return 0;
    }

    if (_busy) {
        return 0;
    }

    Radio.SetAfc(1);

    _afcOn = true;

    return 1;
}

int FskRadioClass::disableAfc()
{
    if (!_initialized) {
        return 0;
    }

    if (_busy) {
        return 0;
    }

    Radio.SetAfc(0);

    _afcOn = false;

    return 1;
}

int FskRadioClass::enableWhitening()
{
    if (!_initialized) {
        return 0;
    }

    if (_busy) {
        return 0;
    }

    Radio.SetDcFree(2);

    return 1;
}

int FskRadioClass::disableWhitening()
{
    if (!_initialized) {
        return 0;
    }

    if (_busy) {
        return 0;
    }

    Radio.SetDcFree(0);

    return 1;
}

int FskRadioClass::enableCrc()
{
    if (!_initialized) {
        return 0;
    }

    if (!_crcOn) {
        _crcOn = true;

        _updateTxConfig = true;
        _updateRxConfig = true;
    }

    return 1;
}

int FskRadioClass::disableCrc()
{
    if (!_initialized) {
        return 0;
    }

    if (_crcOn) {
        _crcOn = false;

        _updateTxConfig = true;
        _updateRxConfig = true;
    }

    return 1;
}

int FskRadioClass::setIdleMode(IdleMode mode)
{
    if (!_initialized) {
        return 0;
    }

    if (_busy) {
        return 0;
    }

    Radio.SetIdleMode(mode);

    return 1;
}

void FskRadioClass::enableWakeup()
{
    _wakeup = true;
}

void FskRadioClass::disableWakeup()
{
    _wakeup = false;
}

void FskRadioClass::onTransmit(void(*callback)(void))
{
    _transmitCallback = Callback(callback);
}

void FskRadioClass::onTransmit(Callback callback)
{
    _transmitCallback = callback;
}

void FskRadioClass::onReceive(void(*callback)(void))
{
    _receiveCallback = Callback(callback);
}

void FskRadioClass::onReceive(Callback callback)
{
    _receiveCallback = callback;
}

bool FskRadioClass::__TxStart(void)
{
    FskRadioClass *self = FskRadioInstance;

    if (self->_busy >= 2) {
        return false;
    }
    
    if (self->_busy == 1) {
        Radio.Standby();
    }

    self->_busy = 2;

    if (self->_updateFrequency) {
        self->_updateFrequency = false;
        
        Radio.SetChannel(self->_frequency);
    }

    if (self->_updateTxConfig) {
        self->_updateTxConfig = false;

        Radio.SetTxConfig(MODEM_FSK, 
                          self->_txPower,
                          self->_deviation,
                          0,
                          self->_bitrate,
                          0,
                          self->_preambleLength,
                          self->_implicitHeader,
                          self->_crcOn,
                          false,
                          0,
                          false,
                          0);
    }
        
    Radio.Send(&self->_tx_data[0], self->_tx_size);

    return true;
}

bool FskRadioClass::__RxStart(void)
{
    FskRadioClass *self = FskRadioInstance;

    if (self->_busy >= 2) {
        return false;
    }

    if (self->_busy == 1) {
        Radio.Standby();
    }

    self->_busy = 1;
    
    if (self->_updateFrequency) {
        self->_updateFrequency = false;
        
        Radio.SetChannel(self->_frequency);
    }

    if (self->_updateRxConfig) {
        self->_updateRxConfig = false;

        Radio.SetRxConfig(MODEM_FSK, 
                          self->_bandwidth,
                          self->_bitrate,
                          0,
                          (self->_afcOn ? self->_bandwidthAfc : self->_bandwidth),
                          self->_preambleLength,
                          self->_syncTimeout,
                          ((self->_fixedPayloadLength != 0) ? true : false),
                          self->_fixedPayloadLength,
                          self->_crcOn,
                          false,
                          0,
                          false,
                          ((self->_syncTimeout == 0) ? true : false));
    }

    Radio.Rx(self->_timeout);

    return true;
}

bool FskRadioClass::__Sense(void)
{
    FskRadioClass *self = FskRadioInstance;

    if (self->_busy >= 2) {
        return false;
    }

    if (self->_busy == 1) {
        Radio.Standby();
    }

    self->_busy = 4;
    
    if (self->_updateFrequency) {
        self->_updateFrequency = false;
        
        Radio.SetChannel(self->_frequency);
    }
    
    if (self->_updateRxConfig) {
        self->_updateRxConfig = false;

        Radio.SetRxConfig(MODEM_FSK, 
                          self->_bandwidth,
                          self->_bitrate,
                          0,
                          (self->_afcOn ? self->_bandwidthAfc : self->_bandwidth),
                          self->_preambleLength,
                          self->_syncTimeout,
                          ((self->_fixedPayloadLength != 0) ? true : false),
                          self->_fixedPayloadLength,
                          self->_crcOn,
                          false,
                          0,
                          false,
                          ((self->_syncTimeout == 0) ? true : false));
    }

    return true;
}

bool FskRadioClass::__Standby(void)
{
    FskRadioClass *self = FskRadioInstance;

    if (self->_busy >= 2) {
        return false;
    }

    Radio.Standby();

    self->_busy = 0;

    return true;
}

bool FskRadioClass::__Sleep(void)
{
    FskRadioClass *self = FskRadioInstance;

    if (self->_busy >= 2) {
        return false;
    }

    Radio.Sleep();

    self->_busy = 0;

    return true;
}

void FskRadioClass::__TxDone(void)
{
    FskRadioClass *self = FskRadioInstance;

    if (self->_wakeup) {
        stm32l0_system_wakeup();
    }

    self->_busy = 0;

    self->_transmitCallback.queue();
}

void FskRadioClass::__RxDone(uint8_t *data, uint16_t size, int16_t rssi, int8_t snr)
{
    FskRadioClass *self = FskRadioInstance;
    uint32_t rx_write, rx_size;
    bool broadcast;

    if (self->_wakeup) {
        stm32l0_system_wakeup();
    }

    rx_write = self->_rx_write;

    if (rx_write < self->_rx_read) {
        rx_size = self->_rx_read - rx_write -1;
    } else {
        rx_size = (FSKRADIO_RX_BUFFER_SIZE - rx_write) + self->_rx_read -1;
    }

    if (rx_size >= (unsigned int)(4 + size - (self->_rx_address ? 1 : 0)))
    {
        broadcast = false;

        if (self->_addressFiltering != FILTERING_NONE)
        {
            if (data[0] != self->_nodeAddress) {
                broadcast = true;
            }

            data++;
            size--;
        }

        self->_rx_data[rx_write] = size;
        
        if (++rx_write == FSKRADIO_RX_BUFFER_SIZE) {
            rx_write = 0;
        }

        self->_rx_data[rx_write] = rssi >> 0;
        
        if (++rx_write == FSKRADIO_RX_BUFFER_SIZE) {
            rx_write = 0;
        }

        self->_rx_data[rx_write] = rssi >> 8;
        
        if (++rx_write == FSKRADIO_RX_BUFFER_SIZE) {
            rx_write = 0;
        }

        self->_rx_data[rx_write] = broadcast;
        
        if (++rx_write == FSKRADIO_RX_BUFFER_SIZE) {
            rx_write = 0;
        }
        
        rx_size = size;
        
        if (rx_size > (FSKRADIO_RX_BUFFER_SIZE - rx_write)) {
            rx_size = FSKRADIO_RX_BUFFER_SIZE - rx_write;
        }
        
        if (rx_size) {
            memcpy(&self->_rx_data[rx_write], data, rx_size);
            
            rx_write += rx_size;
            
            if (rx_write >= FSKRADIO_RX_BUFFER_SIZE) {
                rx_write -= FSKRADIO_RX_BUFFER_SIZE;
            }
        }
        
        if (rx_size != size) {
            memcpy(&self->_rx_data[rx_write], data + rx_size, size - rx_size);
            
            rx_write += (size - rx_size);
        }
        
        self->_rx_write = rx_write;
    }

    if (self->_syncTimeout) {
        self->_busy = 0;
    }

    self->_receiveCallback.queue();
}

void FskRadioClass::__RxTimeout(void)
{
    FskRadioClass *self = FskRadioInstance;

    if (self->_wakeup) {
        stm32l0_system_wakeup();
    }

    self->_busy = 0;

    self->_receiveCallback.queue();
}

FskRadioClass FskRadio;
