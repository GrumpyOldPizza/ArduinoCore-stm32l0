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

GNSSLocation::GNSSLocation(const gnss_location_t *location)
{
    _location = *location;
}

GNSSLocation::GNSSLocation()
{
    _location.time.year    = 1980 - 1980;
    _location.time.month   = 1;
    _location.time.day     = 6;
    _location.time.hours   = 0;
    _location.time.minutes = 0;
    _location.time.seconds = 0;
    _location.time.millis  = 0;
    _location.mask         = 0;
    _location.correction   = -128;
    _location.type         = 0;
    _location.latitude     = 0;
    _location.longitude    = 0;
    _location.altitude     = 0;
    _location.separation   = 0;
    _location.speed        = 0;
    _location.course       = 0;
    _location.climb        = 0;
    _location.ehpe         = 0;
    _location.evpe         = 0;
    _location.quality      = 0;
    _location.numsv        = 0;
    _location.pdop         = 9999;
    _location.hdop         = 9999;
    _location.vdop         = 9999;
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

bool GNSSLocation::fullyResolved(void) const
{
    return !!(_location.mask & GNSS_LOCATION_MASK_RESOLVED);
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

uint8_t GNSSLocation::hours(void) const
{
    return _location.time.hours;
}

uint8_t GNSSLocation::minutes(void) const
{
    return _location.time.minutes;
}

uint8_t GNSSLocation::seconds(void) const
{
    return _location.time.seconds;
}

uint16_t GNSSLocation::millis(void) const
{
    return _location.time.millis;
}

int8_t GNSSLocation::leapSeconds(void) const
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

float GNSSLocation::height(void) const
{
    return (float)(_location.altitude + _location.separation) / (float)1e3;
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

GNSSSatellites::GNSSSatellites()
{
    _satellites.count = 0;
}

unsigned int GNSSSatellites::count() const
{
    return _satellites.count;
}

unsigned int GNSSSatellites::svid(unsigned int index) const
{
    if (index > _satellites.count) {
        return 0;
    }

    return _satellites.info[index].svid;
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

bool GNSSSatellites::unhealthy(unsigned int index) const
{
    if (index > _satellites.count) {
        return false;
    }
    
    return !!(_satellites.info[index].state & GNSS_SATELLITES_STATE_UNHEALTHY);
}

bool GNSSSatellites::almanac(unsigned int index) const
{
    if (index > _satellites.count) {
        return false;
    }
    
    return !!(_satellites.info[index].state & GNSS_SATELLITES_STATE_ALMANAC);
}

bool GNSSSatellites::ephemeris(unsigned int index) const
{
    if (index > _satellites.count) {
        return false;
    }
    
    return !!(_satellites.info[index].state & GNSS_SATELLITES_STATE_EPHEMERIS);
}


bool GNSSSatellites::autonomous(unsigned int index) const
{
    if (index > _satellites.count) {
        return false;
    }
    
    return !!(_satellites.info[index].state & GNSS_SATELLITES_STATE_AUTONOMOUS);
}

bool GNSSSatellites::correction(unsigned int index) const
{
    if (index > _satellites.count) {
        return false;
    }
    
    return !!(_satellites.info[index].state & GNSS_SATELLITES_STATE_CORRECTION);
}

bool GNSSSatellites::acquired(unsigned int index) const
{
    if (index > _satellites.count) {
        return false;
    }
    
    return !!(_satellites.info[index].state & GNSS_SATELLITES_STATE_ACQUIRED);
}

bool GNSSSatellites::locked(unsigned int index) const
{
    if (index > _satellites.count) {
        return false;
    }
    
    return !!(_satellites.info[index].state & GNSS_SATELLITES_STATE_LOCKED);
}

bool GNSSSatellites::navigating(unsigned int index) const
{
    if (index > _satellites.count) {
        return false;
    }
    
    return !!(_satellites.info[index].state & GNSS_SATELLITES_STATE_NAVIGATING);
}

GNSSClass::GNSSClass()
{
}

void GNSSClass::begin(Uart &uart, GNSSmode mode, GNSSrate rate)
{
    static const gnss_callbacks_t GNSSCallbacks = {
#if defined(STM32L0_CONFIG_PIN_GNSS_ENABLE)
        (gnss_enable_callback_t)&GNSSClass::enableCallback,
        (gnss_disable_callback_t)&GNSSClass::disableCallback,
#else /* defined(STM32L0_CONFIG_PIN_GNSS_ENABLE) */
        NULL,
        NULL,
#endif /* defined(STM32L0_CONFIG_PIN_GNSS_ENABLE) */
        (gnss_location_callback_t)&GNSSClass::locationCallback,
        (gnss_satellites_callback_t)&GNSSClass::satellitesCallback,
    };

#if defined(STM32L0_CONFIG_PIN_GNSS_BACKUP)
    stm32l0_gpio_pin_configure(STM32L0_CONFIG_PIN_GNSS_BACKUP, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(STM32L0_CONFIG_PIN_GNSS_BACKUP, 1);
#endif /* defined(STM32L0_CONFIG_PIN_GNSS_BACKUP) */

#if defined(STM32L0_CONFIG_PIN_GNSS_ENABLE)
    stm32l0_gpio_pin_configure(STM32L0_CONFIG_PIN_GNSS_RX, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));
    stm32l0_gpio_pin_configure(STM32L0_CONFIG_PIN_GNSS_TX, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(STM32L0_CONFIG_PIN_GNSS_TX, 1);

    stm32l0_gpio_pin_configure(STM32L0_CONFIG_PIN_GNSS_ENABLE, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(STM32L0_CONFIG_PIN_GNSS_ENABLE, 1);

    while (!stm32l0_gpio_pin_read(STM32L0_CONFIG_PIN_GNSS_RX))
    {
    }
#endif /* defined(STM32L0_CONFIG_PIN_GNSS_ENABLE) */

#if defined(STM32L0_CONFIG_PIN_GNSS_PPS)
    stm32l0_gpio_pin_configure(STM32L0_CONFIG_PIN_GNSS_PPS, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));
    stm32l0_exti_attach(STM32L0_CONFIG_PIN_GNSS_PPS, STM32L0_EXTI_CONTROL_EDGE_FALLING, (stm32l0_exti_callback_t)ppsCallback, (void*)this);
#endif

    _uart = &uart; 

    _uart->begin(9600);

    if (mode == MODE_NMEA)
    {
        _baudrate = 9600;

        gnss_initialize(mode, rate, _baudrate, (gnss_send_routine_t)&GNSSClass::sendRoutine, &GNSSCallbacks, (void*)this);
    }
    else
    {
        _baudrate = (rate > RATE_1HZ) ? 115200 : 19200;

        gnss_initialize(mode, rate, _baudrate, (gnss_send_routine_t)&GNSSClass::sendRoutine, &GNSSCallbacks, (void*)this);
    }

    _uart->onReceive(Callback(&GNSSClass::receiveCallback, this));
}

void GNSSClass::end()
{
    if (_uart)
    {
        _uart->end();
        
        _uart = NULL;

#if defined(STM32L0_CONFIG_PIN_GNSS_BACKUP)
	stm32l0_gpio_pin_configure(STM32L0_CONFIG_PIN_GNSS_BACKUP, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
#endif /* defined(STM32L0_CONFIG_PIN_GNSS_BACKUP) */
    }
}

bool GNSSClass::setAntenna(GNSSantenna antenna)
{
#if defined(STM32L0_CONFIG_GNSS_ANT_SWITCH)
    return (_uart && gnss_set_antenna(antenna));
#else /* STM32L0_CONFIG_GNSS_ANT_SWITCH */
    return false;
#endif /* STM32L0_CONFIG_GNSS_ANT_SWITCH */
}

bool GNSSClass::setPPS(unsigned int width)
{
    return (_uart && gnss_set_pps(width));
}

bool GNSSClass::setConstellation(GNSSconstellation constellation)
{
    return (_uart && gnss_set_constellation(constellation));
}

bool GNSSClass::setSBAS(bool enable)
{
    return (_uart && gnss_set_sbas(enable));
}

bool GNSSClass::setQZSS(bool enable)
{
    return (_uart && gnss_set_qzss(enable));
}

bool GNSSClass::setAutonomous(bool enable)
{
    return (_uart && gnss_set_autonomous(enable));
}

bool GNSSClass::setPlatform(GNSSplatform platform)
{
    return (_uart && gnss_set_platform(platform));
}

bool GNSSClass::setPeriodic(unsigned int acqTime, unsigned int onTime, unsigned int period)
{
    return (_uart && gnss_set_periodic(acqTime, onTime, period));
}

bool GNSSClass::suspend()
{
    if (!(_uart && gnss_suspend()))
    {
        return false;
    }

#if defined(STM32L0_CONFIG_PIN_GNSS_PPS)
    stm32l0_exti_detach(STM32L0_CONFIG_PIN_GNSS_PPS);
    stm32l0_gpio_pin_configure(STM32L0_CONFIG_PIN_GNSS_PPS, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
#endif

    return true;
}

bool GNSSClass::resume()
{
    if (!(_uart && gnss_resume()))
    {
        return false;
    }

#if defined(STM32L0_CONFIG_PIN_GNSS_PPS)
    stm32l0_gpio_pin_configure(STM32L0_CONFIG_PIN_GNSS_PPS, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));
    stm32l0_exti_attach(STM32L0_CONFIG_PIN_GNSS_PPS, STM32L0_EXTI_CONTROL_EDGE_FALLING, (stm32l0_exti_callback_t)ppsCallback, (void*)this);
#endif
    
    return true;
}

bool GNSSClass::busy()
{
    return gnss_busy();
}


bool GNSSClass::location(GNSSLocation &location)
{
    if (!_location_pending) {
        return false;
    }

    do
    {
        _location_pending = false;
        
        location = GNSSLocation(&_location_data);
    }
    while (_location_pending);
    
    return true;
}

bool GNSSClass::satellites(GNSSSatellites &satellites)
{
    if (!_satellites_pending) {
        return false;
    }

    do
    {
        _satellites_pending = false;
        
        satellites = GNSSSatellites(&_satellites_data);
    }
    while (_satellites_pending);
    
    return true;
}

void GNSSClass::enableWakeup()
{
    _wakeup = true;
}

void GNSSClass::disableWakeup()
{
    _wakeup = false;
}

void GNSSClass::onLocation(void(*callback)(void))
{
    _locationCallback = Callback(callback);
}

void GNSSClass::onLocation(Callback callback)
{
    _locationCallback = callback;
}

void GNSSClass::onSatellites(void(*callback)(void))
{
    _satellitesCallback = Callback(callback);
}

void GNSSClass::onSatellites(Callback callback)
{
    _satellitesCallback = callback;
}

void GNSSClass::attachInterrupt(void(*callback)(void))
{
    _ppsCallback = callback;
}

void GNSSClass::detachInterrupt()
{
    _ppsCallback = NULL;
}

void GNSSClass::receiveCallback(void)
{
    uint8_t rx_data[16];
    int rx_count;

    do
    {
        rx_count = _uart->read(&rx_data[0], sizeof(rx_data));
        
        if (rx_count > 0)
        {
            gnss_receive(&rx_data[0], rx_count);
        }
    }
    while (rx_count > 0);
}

void GNSSClass::completionCallback(void)
{
    if (_doneCallback) {
        (*_doneCallback)();
    } else {
        _uart->begin(_baudrate);
        _uart->setWakeup((_baudrate <= 19200));
    }
}

void GNSSClass::sendRoutine(class GNSSClass *self, const uint8_t *data, uint32_t count, gnss_send_callback_t callback)
{
    if (self->_uart)
    {
        self->_doneCallback = callback;
        self->_uart->write(data, count, Callback(&GNSSClass::completionCallback, self));
    }
}

void GNSSClass::enableCallback(class GNSSClass *self)
{
#if defined(STM32L0_CONFIG_PIN_GNSS_ENABLE)
    self->_uart->begin(self->_baudrate);
    self->_uart->setWakeup((self->_baudrate <= 19200));

    stm32l0_gpio_pin_configure(STM32L0_CONFIG_PIN_GNSS_ENABLE, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(STM32L0_CONFIG_PIN_GNSS_ENABLE, 1);
#endif /* defined(STM32L0_CONFIG_PIN_GNSS_ENABLE) */
}

void GNSSClass::disableCallback(class GNSSClass *self)
{
#if defined(STM32L0_CONFIG_PIN_GNSS_ENABLE)
    stm32l0_gpio_pin_configure(STM32L0_CONFIG_PIN_GNSS_ENABLE, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
    
    self->_uart->end();
#endif /* defined(STM32L0_CONFIG_PIN_GNSS_ENABLE) */
}

void GNSSClass::locationCallback(class GNSSClass *self, const gnss_location_t *location)
{
    if (self->_wakeup) {
        stm32l0_system_wakeup();
    }

    self->_location_data = *location;
    self->_location_pending = true;

    self->_locationCallback.queue();
}

void GNSSClass::satellitesCallback(class GNSSClass *self, const gnss_satellites_t *satellites)
{
    if (self->_wakeup) {
        stm32l0_system_wakeup();
    }

    self->_satellites_data = *satellites;
    self->_satellites_pending = true;

    self->_satellitesCallback.queue();
}

void GNSSClass::ppsCallback(class GNSSClass *self)
{
    gnss_pps_callback();

    if (self->_ppsCallback) {
        (*self->_ppsCallback)();
    }
}

GNSSClass GNSS;
