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
#include "LoRaRadio.h"
#include "wiring_private.h"
#include "../../../system/STM32L0xx/Source/LoRa/Radio/radio.h"

#define LORARADIO_TX_BUFFER_SIZE         256
#define LORARADIO_RX_BUFFER_SIZE         512

static uint32_t LoRaRadioBuffer[(LORARADIO_TX_BUFFER_SIZE + 3) / 4 + (LORARADIO_RX_BUFFER_SIZE + 3) / 4];

static LoRaRadioClass *LoRaRadioInstance = NULL;

static inline bool LoRaRadioCall(bool(*routine)(void))
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

LoRaRadioClass::LoRaRadioClass()
{
    _tx_data = (uint8_t*)&LoRaRadioBuffer[0];
    _rx_data = (uint8_t*)&LoRaRadioBuffer[(LORARADIO_TX_BUFFER_SIZE + 3) / 4];

    _initialized = false;
}

int LoRaRadioClass::begin(unsigned long frequency)
{
    static const RadioEvents_t LoRaRadioEvents = {
        LoRaRadioClass::__TxDone,
        NULL,
        LoRaRadioClass::__RxDone,
        LoRaRadioClass::__RxTimeout,
        NULL,
        NULL,
        LoRaRadioClass::__CadDone,
    };

    if (LoRaRadioInstance) {
        return 0;
    }

    if (_initialized) {
        return 0;
    }

    _initialized = true;
    _busy = 0;
    _cadDetected = false;

    _tx_size = 0;
    _tx_active = false;

    _rx_read = 0;
    _rx_write = 0;
    _rx_index = 0;
    _rx_next = 0;
    _rx_size = 0;
    _rx_rssi = 0;
    _rx_snr = 0;

    _updateFrequency = true;
    _updateTxConfig = true;
    _updateRxConfig = true;

    _frequency = frequency;
    _txPower = 14;
    _fixedPayloadLength = 0;
    _signalBandwidth = BW_125;
    _spreadingFactor = SF_7;
    _codingRate = CR_4_5;
    _preambleLength = 8;
    _symbolTimeout = 0;
    _iqInverted = false;
    _crcOn = true;

    _wakeup = false;
    _implicitHeader = false;
    _timeout = 0;

    LoRaRadioInstance = this;

    Radio.Init(&LoRaRadioEvents, frequency);

    Radio.SetModem(MODEM_LORA);

    Radio.SetMaxPayloadLength(MODEM_LORA, LORARADIO_MAX_PAYLOAD_LENGTH);

    Radio.SetIdleMode(IDLE_STANDBY);

    return 1;
}

void LoRaRadioClass::end()
{
    if (_initialized) {
        Radio.Sleep();
    }

    _initialized = false;

    LoRaRadioInstance = NULL;
}

bool LoRaRadioClass::busy()
{
    return _busy != 0;
}

bool LoRaRadioClass::cadDetected()
{
    return _cadDetected;
}

int LoRaRadioClass::beginPacket()
{
    if (!_initialized) {
        return 0;
    }

    _tx_size = 0;
    _tx_active = true;

    return 1;
}

int LoRaRadioClass::endPacket(bool fixedPayloadLength)
{
    if (!_tx_active) {
        return 0;
    }

    if (_implicitHeader != fixedPayloadLength) {
        _implicitHeader = fixedPayloadLength;

        _updateTxConfig = true;
    }

    return LoRaRadioCall(__TxStart);
}

int LoRaRadioClass::sendPacket(const uint8_t *buffer, size_t size, bool fixedPayloadLength)
{
    if (!_initialized) {
        return 0;
    }

    if (_tx_active) {
        return 0;
    }

    if (size > LORARADIO_MAX_PAYLOAD_LENGTH) {
        return 0;
    }

    _tx_size = size;

    memcpy(&_tx_data[0], buffer, size);

    _tx_active = true;

    return endPacket(fixedPayloadLength);
}

int LoRaRadioClass::receive(unsigned int timeout)
{
    if (!_initialized) {
        return 0;
    }

    _timeout = timeout;

    return LoRaRadioCall(__RxStart);
}

int LoRaRadioClass::cad()
{
    if (!_initialized) {
        return 0;
    }

    _cadDetected = false;

    return LoRaRadioCall(__CadStart);
}

int LoRaRadioClass::sense(int rssiThreshold, unsigned int senseTime)
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

    if (!LoRaRadioCall(__Sense)) {
        return 0;
    }

    isChannelFree = Radio.IsChannelFree(MODEM_LORA, _frequency, rssiThreshold, senseTime);

    _busy = 0;

    return (isChannelFree ? 1 : 0);
}

int LoRaRadioClass::standby()
{
    if (!_initialized) {
        return 0;
    }

    return LoRaRadioCall(__Standby);
}

int LoRaRadioClass::sleep()
{
    if (!_initialized) {
        return 0;
    }

    return LoRaRadioCall(__Sleep);
}

int LoRaRadioClass::availableForWrite()
{
    if (!_tx_active) {
        return 0;
    }

    return LORARADIO_MAX_PAYLOAD_LENGTH - _tx_size;
}

void LoRaRadioClass::flush()
{
}

size_t LoRaRadioClass::write(uint8_t data)
{
    if (!_tx_active) {
        return 0;
    }

    if (_tx_size >= LORARADIO_MAX_PAYLOAD_LENGTH) {
        return 0;
    }

    _tx_data[_tx_size++] = data;

    return 1;
}

size_t LoRaRadioClass::write(const uint8_t *data, size_t size)
{
    if (!_tx_active) {
        return 0;
    }

    if (size > (unsigned int)(LORARADIO_MAX_PAYLOAD_LENGTH - _tx_size)) {
        size = LORARADIO_MAX_PAYLOAD_LENGTH - _tx_size;
    }

    memcpy(&_tx_data[_tx_size], data, size);

    _tx_size += size;

    return size;
}

int LoRaRadioClass::parsePacket()
{
    uint32_t rx_read;
    uint8_t rx_snr;
    uint16_t rx_rssi;

    if (_rx_size)
    {
        _rx_index = _rx_next;
        _rx_read = _rx_next;
        _rx_size = 0;
        _rx_snr = 0;
        _rx_rssi = 0;
    }

    rx_read = _rx_read;

    if (rx_read == _rx_write) {
        return 0;
    }

    _rx_size = _rx_data[rx_read];

    if (++rx_read == LORARADIO_RX_BUFFER_SIZE) {
        rx_read = 0;
    }

    rx_rssi = (_rx_data[rx_read] << 0);

    if (++rx_read == LORARADIO_RX_BUFFER_SIZE) {
        rx_read = 0;
    }

    rx_rssi |= (_rx_data[rx_read] << 8);

    if (++rx_read == LORARADIO_RX_BUFFER_SIZE) {
        rx_read = 0;
    }

    rx_snr = _rx_data[rx_read];

    if (++rx_read == LORARADIO_RX_BUFFER_SIZE) {
        rx_read = 0;
    }

    _rx_index = rx_read;
    _rx_rssi = (int16_t)rx_rssi;
    _rx_snr = (int16_t)rx_snr;

    rx_read += _rx_size;

    if (rx_read >= LORARADIO_RX_BUFFER_SIZE) {
        rx_read -= LORARADIO_RX_BUFFER_SIZE;
    }

    _rx_next = rx_read;

    return _rx_size;
}

int LoRaRadioClass::available()
{
    if (_rx_next >= _rx_index) {
        return (_rx_next - _rx_index);
    } else {
        return (_rx_next + (LORARADIO_RX_BUFFER_SIZE - _rx_index));
    }
}

int LoRaRadioClass::read()
{
    int data;
    
    if (_rx_index == _rx_next) {
        return -1;
    }

    data = _rx_data[_rx_index++];

    if (_rx_index == LORARADIO_RX_BUFFER_SIZE) {
        _rx_index = 0;
    }

    return data;
}

size_t LoRaRadioClass::read(uint8_t *buffer, size_t size)
{
    size_t count;

    if (_rx_index == _rx_next) {
        return 0;
    }

    if (_rx_next < _rx_index)
    {
        count = size;

        if (count > (size_t)(LORARADIO_RX_BUFFER_SIZE - _rx_index))
        {
            count = LORARADIO_RX_BUFFER_SIZE - _rx_index;
        }

        if (count)
        {
            memcpy(buffer, &_rx_data[_rx_index], count);
            
            buffer += count;
            size -= count;
            
            _rx_index += count;
            
            if (_rx_index == LORARADIO_RX_BUFFER_SIZE) {
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

int LoRaRadioClass::peek(void)
{
    if (_rx_index == _rx_next) {
        return -1;
    }

    return _rx_data[_rx_index];
}

void LoRaRadioClass::purge(void)
{
    do
    {
        _rx_read = _rx_write;
        _rx_index = _rx_write;
        _rx_next = _rx_write;
        _rx_size = 0;
        _rx_rssi = 0;
        _rx_snr = 0;
    }
    while (_rx_read != _rx_write);
}

int LoRaRadioClass::packetRssi()
{
    return _rx_rssi;
}

int LoRaRadioClass::packetSnr()
{
    return _rx_snr;
}

int LoRaRadioClass::setFrequency(unsigned long frequency)
{
    if (_frequency != frequency) {
        _frequency = frequency;

        _updateFrequency = true;
    }

    return 1;
}

int LoRaRadioClass::setTxPower(int level)
{
    if (_txPower != level) {
        _txPower = level;

        _updateTxConfig = true;
    }

    return 1;
}

int LoRaRadioClass::setBandwidth(Bandwidth bw)
{
    if (_signalBandwidth != bw) {
        _signalBandwidth = bw;

        _updateTxConfig = true;
        _updateRxConfig = true;
    }

    return 1;
}

int LoRaRadioClass::setSpreadingFactor(SpreadingFactor sf)
{
    if (_spreadingFactor != sf) {
        _spreadingFactor = sf;

        _updateTxConfig = true;
        _updateRxConfig = true;
    }

    return 1;
}

int LoRaRadioClass::setCodingRate(CodingRate cr)
{
    if (_codingRate != cr) {
        _codingRate = cr;

        _updateTxConfig = true;
        _updateRxConfig = true;
    }

    return 1;
}

int LoRaRadioClass::setPreambleLength(unsigned int length)
{
    if ((length < 6) || (length > 65535)) {
        return 0;
    }

    if (_preambleLength != length) {
        _preambleLength = length;

        _updateTxConfig = true;
        _updateRxConfig = true;
    }

    return 1;
}

int LoRaRadioClass::setFixedPayloadLength(unsigned int length)
{
    if (length > 255) {
        return 0;
    }

    if (_fixedPayloadLength != length) {
        _fixedPayloadLength = length;

        _updateRxConfig = true;
    }

    return 1;
}

int LoRaRadioClass::setSymbolTimeout(unsigned int timeout)
{
    if (timeout > 1023) {
        return 0;
    }

    if (_symbolTimeout != timeout) {
        _symbolTimeout = timeout;

        _updateRxConfig = true;
    }

    return 1;
}

int LoRaRadioClass::setIQInverted(bool enable)
{
    if (_iqInverted != enable) {
        _iqInverted = enable;

        _updateTxConfig = true;
        _updateRxConfig = true;
    }

    return 1;
}

int LoRaRadioClass::setPublicNetwork(bool enable)
{
    if (!_initialized) {
        return 0;
    }

    if (_busy) {
        return 0;
    }
    
    Radio.SetPublicNetwork(enable);

    return 1;
}

int LoRaRadioClass::setLnaBoost(bool enable)
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

int LoRaRadioClass::enableCrc()
{
    if (!_crcOn) {
        _crcOn = true;

        _updateTxConfig = true;
        _updateRxConfig = true;
    }

    return 1;
}

int LoRaRadioClass::disableCrc()
{
    if (_crcOn) {
        _crcOn = false;

        _updateTxConfig = true;
        _updateRxConfig = true;
    }

    return 1;
}

int LoRaRadioClass::setIdleMode(IdleMode mode)
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

void LoRaRadioClass::enableWakeup()
{
    _wakeup = true;
}

void LoRaRadioClass::disableWakeup()
{
    _wakeup = false;
}

void LoRaRadioClass::onTransmit(void(*callback)(void))
{
    _transmitCallback = Callback(callback);
}

void LoRaRadioClass::onTransmit(Callback callback)
{
    _transmitCallback = callback;
}

void LoRaRadioClass::onReceive(void(*callback)(void))
{
    _receiveCallback = Callback(callback);
}

void LoRaRadioClass::onReceive(Callback callback)
{
    _receiveCallback = callback;
}

void LoRaRadioClass::onCad(void(*callback)(void))
{
    _cadCallback = Callback(callback);
}

void LoRaRadioClass::onCad(Callback callback)
{
    _cadCallback = callback;
}

bool LoRaRadioClass::__TxStart(void)
{
    LoRaRadioClass *self = LoRaRadioInstance;

    if (self->_busy >= 2) {
        return false;
    }
    
    if (self->_busy != 0) {
        Radio.Standby();
    }

    self->_busy = 2;

    if (self->_updateFrequency) {
        self->_updateFrequency = false;
            
        Radio.SetChannel(self->_frequency);
    }

    if (self->_updateTxConfig) {
        self->_updateTxConfig = false;

        Radio.SetTxConfig(MODEM_LORA, 
                          self->_txPower,
                          0,
                          self->_signalBandwidth,
                          self->_spreadingFactor,
                          self->_codingRate,
                          self->_preambleLength,
                          self->_implicitHeader,
                          self->_crcOn,
                          false,
                          0,
                          self->_iqInverted,
                          0);
    }
        
    Radio.Send(&self->_tx_data[0], self->_tx_size);

    return true;
}

bool LoRaRadioClass::__RxStart(void)
{
    LoRaRadioClass *self = LoRaRadioInstance;

    if (self->_busy >= 2) {
        return false;
    }

    if (self->_busy != 0) {
        Radio.Standby();
    }

    self->_busy = 1;
    
    if (self->_updateFrequency) {
        self->_updateFrequency = false;
            
        Radio.SetChannel(self->_frequency);
    }

    if (self->_updateRxConfig) {
        self->_updateRxConfig = false;

        Radio.SetRxConfig(MODEM_LORA, 
                          self->_signalBandwidth,
                          self->_spreadingFactor,
                          self->_codingRate,
                          0,
                          self->_preambleLength,
                          self->_symbolTimeout,
                          ((self->_fixedPayloadLength != 0) ? true : false),
                          self->_fixedPayloadLength,
                          self->_crcOn,
                          false,
                          0,
                          self->_iqInverted,
                          ((self->_symbolTimeout == 0) ? true : false));
    }

    Radio.Rx(self->_timeout);

    return true;
}

bool LoRaRadioClass::__CadStart(void)
{
    LoRaRadioClass *self = LoRaRadioInstance;

    if (self->_busy >= 2) {
        return false;
    }

    if (self->_busy != 0) {
        Radio.Standby();
    }

    self->_busy = 3;
    
    if (self->_updateFrequency) {
        self->_updateFrequency = false;
            
        Radio.SetChannel(self->_frequency);
    }

    if (self->_updateRxConfig) {
        self->_updateRxConfig = false;

        Radio.SetRxConfig(MODEM_LORA, 
                          self->_signalBandwidth,
                          self->_spreadingFactor,
                          self->_codingRate,
                          0,
                          self->_preambleLength,
                          self->_symbolTimeout,
                          ((self->_fixedPayloadLength != 0) ? true : false),
                          self->_fixedPayloadLength,
                          self->_crcOn,
                          false,
                          0,
                          self->_iqInverted,
                          ((self->_symbolTimeout == 0) ? true : false));
    }

    Radio.StartCad();

    return true;
}

bool LoRaRadioClass::__Sense(void)
{
    LoRaRadioClass *self = LoRaRadioInstance;

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

        Radio.SetRxConfig(MODEM_LORA, 
                          self->_signalBandwidth,
                          self->_spreadingFactor,
                          self->_codingRate,
                          0,
                          self->_preambleLength,
                          self->_symbolTimeout,
                          ((self->_fixedPayloadLength != 0) ? true : false),
                          self->_fixedPayloadLength,
                          self->_crcOn,
                          false,
                          0,
                          self->_iqInverted,
                          ((self->_symbolTimeout == 0) ? true : false));
    }

    return true;
}

bool LoRaRadioClass::__Standby(void)
{
    LoRaRadioClass *self = LoRaRadioInstance;

    if (self->_busy >= 2) {
        return false;
    }

    Radio.Standby();

    self->_busy = 0;

    return true;
}

bool LoRaRadioClass::__Sleep(void)
{
    LoRaRadioClass *self = LoRaRadioInstance;

    if (self->_busy >= 2) {
        return false;
    }

    Radio.Sleep();

    self->_busy = 0;

    return true;
}

void LoRaRadioClass::__TxDone(void)
{
    LoRaRadioClass *self = LoRaRadioInstance;

    if (self->_wakeup) {
        stm32l0_system_wakeup();
    }

    self->_busy = 0;

    self->_transmitCallback.queue();
}

void LoRaRadioClass::__RxDone(uint8_t *data, uint16_t size, int16_t rssi, int8_t snr)
{
    LoRaRadioClass *self = LoRaRadioInstance;
    uint32_t rx_write, rx_size;

    if (self->_wakeup) {
        stm32l0_system_wakeup();
    }

    rx_write = self->_rx_write;

    if (rx_write < self->_rx_read) {
        rx_size = self->_rx_read - rx_write -1;
    } else {
        rx_size = (LORARADIO_RX_BUFFER_SIZE - rx_write) + self->_rx_read -1;
    }

    if (rx_size >= (unsigned int)(4 + size))
    {
        self->_rx_data[rx_write] = size;
        
        if (++rx_write == LORARADIO_RX_BUFFER_SIZE) {
            rx_write = 0;
        }

        self->_rx_data[rx_write] = rssi >> 0;
        
        if (++rx_write == LORARADIO_RX_BUFFER_SIZE) {
            rx_write = 0;
        }

        self->_rx_data[rx_write] = rssi >> 8;
        
        if (++rx_write == LORARADIO_RX_BUFFER_SIZE) {
            rx_write = 0;
        }

        self->_rx_data[rx_write] = snr;
        
        if (++rx_write == LORARADIO_RX_BUFFER_SIZE) {
            rx_write = 0;
        }
        
        rx_size = size;
        
        if (rx_size > (LORARADIO_RX_BUFFER_SIZE - rx_write)) {
            rx_size = LORARADIO_RX_BUFFER_SIZE - rx_write;
        }
        
        if (rx_size) {
            memcpy(&self->_rx_data[rx_write], data, rx_size);
            
            rx_write += rx_size;
            
            if (rx_write >= LORARADIO_RX_BUFFER_SIZE) {
                rx_write -= LORARADIO_RX_BUFFER_SIZE;
            }
        }
        
        if (rx_size != size) {
            memcpy(&self->_rx_data[rx_write], data + rx_size, size - rx_size);
            
            rx_write += (size - rx_size);
        }
        
        self->_rx_write = rx_write;
    }

    if (self->_symbolTimeout) {
        self->_busy = 0;
    }

    self->_receiveCallback.queue();
}

void LoRaRadioClass::__RxTimeout(void)
{
    LoRaRadioClass *self = LoRaRadioInstance;

    if (self->_wakeup) {
        stm32l0_system_wakeup();
    }

    self->_busy = 0;

    self->_receiveCallback.queue();
}

void LoRaRadioClass::__CadDone(bool cadDetected)
{
    LoRaRadioClass *self = LoRaRadioInstance;

    if (self->_wakeup) {
        stm32l0_system_wakeup();
    }

    self->_cadDetected = cadDetected;

    self->_busy = 0;

    self->_cadCallback.queue();
}

LoRaRadioClass LoRaRadio;
