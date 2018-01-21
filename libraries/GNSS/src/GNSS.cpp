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

#include "GNSS.h"
#include "wiring_private.h"

static stm32l0_uart_t g_SerialGNSS;

extern const stm32l0_uart_params_t g_SerialGNSSParams;

GNSSLocation::GNSSLocation(const gnss_location_t *location)
{
    _location = *location;
}

GNSSLocation::operator bool() const
{
    return (_location.type != GNSS_LOCATION_TYPE_NONE);
}

enum GNSSLocation::GNSSfixType GNSSLocation::fixType(void) const
{
    return (enum GNSSLocation::GNSSfixType)_location.type;
}

enum GNSSLocation::GNSSfixQuality GNSSLocation::fixQuality(void) const
{
    return (enum GNSSLocation::GNSSfixQuality)_location.quality;
}

unsigned int GNSSLocation::satellites(void) const
{
    return _location.numsv;
}

uint16_t GNSSLocation::year(void) const
{
    return _location.time.year + 1980;
}

uint8_t GNSSLocation::month(void) const
{
    return _location.time.month;
}

uint8_t GNSSLocation::day(void) const
{
    return _location.time.day;
}

uint8_t GNSSLocation::hour(void) const
{
    return _location.time.hour;
}

uint8_t GNSSLocation::minute(void) const
{
    return _location.time.minute;
}

uint8_t GNSSLocation::second(void) const
{
    return _location.time.second;
}

uint16_t GNSSLocation::millis(void) const
{
    return _location.time.millis;
}

uint8_t GNSSLocation::correction(void) const
{
    return _location.correction;
}

double GNSSLocation::latitude(void) const
{
    return (double)_location.latitude / (double)1e7;
}

double GNSSLocation::longitude(void) const
{
    return (double)_location.longitude / (double)1e7;
}

float GNSSLocation::altitude(void) const
{
    return (float)_location.altitude / (float)1e3;
}

float GNSSLocation::separation(void) const
{
    return (float)_location.separation / (float)1e3;
}

float GNSSLocation::speed(void) const
{
    return (float)_location.speed / (float)1e3;
}

float GNSSLocation::course(void) const
{
    return (float)_location.course / (float)1e5;
}

float GNSSLocation::climb(void) const
{
    return (float)_location.climb / (float)1e3;
}

float GNSSLocation::ehpe(void) const
{
    return (float)_location.ehpe / (float)1e3;
}

float GNSSLocation::evpe(void) const
{
    return (float)_location.evpe / (float)1e3;
}

float GNSSLocation::pdop(void) const
{
    return (float)_location.pdop / (float)1e2;
}

float GNSSLocation::hdop(void) const
{
    return (float)_location.hdop / (float)1e2;
}

float GNSSLocation::vdop(void) const
{
    return (float)_location.vdop / (float)1e2;
}

GNSSSatellites::GNSSSatellites(const gnss_satellites_t *satellites)
{
    _satellites = *satellites;
}

unsigned int GNSSSatellites::count() const
{
    return _satellites.count;
}

unsigned int GNSSSatellites::prn(unsigned int index) const
{
    if (index > _satellites.count) {
	return 0;
    }

    return _satellites.info[index].prn;
}

enum GNSSSatellites::GNSSsatelliteState GNSSSatellites::state(unsigned int index) const
{
    if (index > _satellites.count) {
	return STATE_NONE;
    }

    if (_satellites.info[index].state & GNSS_SATELLITES_STATE_NAVIGATING) {
	return STATE_NAVIGATING;
    }

    if (_satellites.info[index].state & GNSS_SATELLITES_STATE_TRACKING) {
	return STATE_TRACKING;
    }

    return STATE_SEARCHING;
}

unsigned int GNSSSatellites::snr(unsigned int index) const
{
    if (index > _satellites.count) {
	return 0;
    }

    return _satellites.info[index].snr;
}

unsigned int GNSSSatellites::elevation(unsigned int index) const
{
    if (index > _satellites.count) {
	return 0;
    }
    
    return _satellites.info[index].elevation;
}

unsigned int GNSSSatellites::azimuth(unsigned int index) const
{
    if (index > _satellites.count) {
	return 0;
    }
    
    return _satellites.info[index].azimuth;
}

GNSSClass::GNSSClass()
{
    stm32l0_uart_create(&g_SerialGNSS, &g_SerialGNSSParams);
}

void GNSSClass::begin(GNSSmode mode, GNSSrate rate)
{
    _uart = &g_SerialGNSS; 

    stm32l0_uart_enable(_uart, &_rx_data[0], sizeof(_rx_data), 9600, (UART_OPTION_DATA_SIZE_8 | UART_OPTION_PARITY_NONE | UART_OPTION_STOP_1), (stm32l0_uart_event_callback_t)GNSSClass::_eventCallback, (void*)this);

    if (mode == MODE_NMEA)
    {
	_baudrate = 9600;

	gnss_initialize(mode, rate, _baudrate, (gnss_send_routine_t)&GNSSClass::_sendRoutine, (gnss_location_callback_t)&GNSSClass::_locationCallback, (gnss_satellites_callback_t)&GNSSClass::_satellitesCallback, (void*)this);
    }
    else
    {
        _baudrate = (rate > RATE_1HZ) ? 115200 : 38400;

	gnss_initialize(mode, rate, _baudrate, (gnss_send_routine_t)&GNSSClass::_sendRoutine, (gnss_location_callback_t)&GNSSClass::_locationCallback, (gnss_satellites_callback_t)&GNSSClass::_satellitesCallback, (void*)this);
    }
}

void GNSSClass::end()
{
    if (_uart)
    {
	stm32l0_uart_disable(_uart);
	
	_uart = NULL;
    }
}

bool GNSSClass::setExternal(bool enable)
{
    return (_uart && gnss_set_external(enable));
}

bool GNSSClass::setConstellation(GNSSconstellation constellation)
{
    return (_uart && gnss_set_constellation(constellation));
}

bool GNSSClass::setSBAS(bool on)
{
    return (_uart && gnss_set_sbas(on));
}

bool GNSSClass::setQZSS(bool on)
{
    return (_uart && gnss_set_qzss(on));
}

bool GNSSClass::setPeriodic(unsigned int onTime, unsigned int period, bool force)
{
    return (_uart && gnss_set_periodic(onTime, period, force));
}

bool GNSSClass::sleep()
{
    return (_uart && gnss_sleep());
}

bool GNSSClass::wakeup()
{
    return (_uart && gnss_wakeup());
}

bool GNSSClass::ready()
{
    return gnss_done();
}


int GNSSClass::available(void)
{
    return !!_location_pending;
}

GNSSLocation GNSSClass::location(void)
{
    gnss_location_t location;

    do
    {
	_location_pending = false;
	
	location = _location_data;
    }
    while (_location_pending);
    
    return GNSSLocation(&location);
}

GNSSSatellites GNSSClass::satellites(void)
{
    gnss_satellites_t satellites;

    do
    {
	_satellites_pending = false;
	
	satellites = _satellites_data;
    }
    while (_satellites_pending);
    
    return GNSSSatellites(&satellites);
}

void GNSSClass::onReceive(void(*callback)(void))
{
    _receive_callback = callback;
}

void GNSSClass::_doneCallback(class GNSSClass *self)
{
    if (self->_send_callback) {
	(*self->_send_callback)();
    } else {
	if (self->_baudrate <= 38400) {
	    stm32l0_uart_configure(self->_uart, self->_baudrate, (UART_OPTION_DATA_SIZE_8 | UART_OPTION_PARITY_NONE | UART_OPTION_STOP_1 | UART_OPTION_WAKEUP));
	} else {
	    stm32l0_uart_configure(self->_uart, self->_baudrate, (UART_OPTION_DATA_SIZE_8 | UART_OPTION_PARITY_NONE | UART_OPTION_STOP_1));
	}
    }
}

void GNSSClass::_sendRoutine(class GNSSClass *self, const uint8_t *data, uint32_t count, gnss_send_callback_t callback)
{
    if (self->_uart)
    {
	self->_send_callback = callback;

	stm32l0_uart_transmit(self->_uart, data, count, (stm32l0_uart_done_callback_t)GNSSClass::_doneCallback, (void*)self);
    }
}

void GNSSClass::_locationCallback(class GNSSClass *self, const gnss_location_t *location)
{
    self->_location_data = *location;
    self->_location_pending = true;

    if (self->_receive_callback) {
	(*self->_receive_callback)();
    }
}

void GNSSClass::_satellitesCallback(class GNSSClass *self, const gnss_satellites_t *satellites)
{
    self->_satellites_data = *satellites;
    self->_satellites_pending = true;
}

void GNSSClass::_receiveRoutine(class GNSSClass *self)
{
    uint8_t rx_data[16];
    int rx_count;

    do
    {
	rx_count = stm32l0_uart_receive(self->_uart, &rx_data[0], sizeof(rx_data), false);
	
	if (rx_count > 0)
	{
	    gnss_receive(&rx_data[0], rx_count);
	}
    }
    while (rx_count > 0);
}

void GNSSClass::_eventCallback(class GNSSClass *self, uint32_t events)
{
    armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)_receiveRoutine, (void*)self, 0);
}

GNSSClass GNSS;
