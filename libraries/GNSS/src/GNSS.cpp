/*
 * Copyright (c) 2017-2020 Thomas Roell.  All rights reserved.
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

#if defined(USBCON)

extern stm32l0_uart_t g_Serial1;
extern const stm32l0_uart_params_t g_Serial1Params;

#else

extern stm32l0_uart_t g_Serial;
extern const stm32l0_uart_params_t g_SerialParams;

#endif

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

void GNSSClass::begin(GNSSmode mode, GNSSrate rate)
{
    uartBegin(mode, rate,
#if defined(USBCON)
              &g_Serial1,
              &g_Serial1Params,
#else
              &g_Serial,
              &g_SerialParams,
#endif
#if defined(STM32L0_CONFIG_PIN_GNSS_PPS)
              STM32L0_CONFIG_PIN_GNSS_PPS,
#else
              STM32L0_GPIO_PIN_NONE,
#endif
#if defined(STM32L0_CONFIG_PIN_GNSS_ENABLE)
              STM32L0_CONFIG_PIN_GNSS_ENABLE,
#else
              STM32L0_GPIO_PIN_NONE,
#endif
#if defined(STM32L0_CONFIG_PIN_GNSS_BACKUP)
              STM32L0_CONFIG_PIN_GNSS_BACKUP,
#else
              STM32L0_GPIO_PIN_NONE,
#endif
#if defined(STM32L0_CONFIG_GNSS_ANT_SWITCH)
              true
#else
              false
#endif
              );
}

void GNSSClass::begin(GNSSmode mode, GNSSrate rate, Uart &uart, int pinPPS, int pinENABLE, int pinBACKUP)
{
    if ((pinPPS < 0) || (pinPPS >= (int)PINS_COUNT) || (g_APinDescription[pinPPS].GPIO == NULL)) {
        pinPPS = -1;
    }

    if ((pinENABLE < 0) || (pinENABLE >= (int)PINS_COUNT) || (g_APinDescription[pinENABLE].GPIO == NULL)) {
        pinENABLE = -1;
    }

    if ((pinBACKUP < 0) || (pinBACKUP >= (int)PINS_COUNT) || (g_APinDescription[pinBACKUP].GPIO == NULL)) {
        pinBACKUP = -1;
    }

    uartBegin(mode,
              rate,
              uart._uart,
              NULL,
              ((pinPPS >= 0)    ? g_APinDescription[pinPPS].pin    : STM32L0_GPIO_PIN_NONE),
              ((pinENABLE >= 0) ? g_APinDescription[pinENABLE].pin : STM32L0_GPIO_PIN_NONE),
              ((pinBACKUP >= 0) ? g_APinDescription[pinBACKUP].pin : STM32L0_GPIO_PIN_NONE),
              false);
}

void GNSSClass::end()
{
    if (_uart)
    {
        uartEnd();
    }
}

bool GNSSClass::setAntenna(GNSSantenna antenna)
{
    if (!_internal)
    {
        return false;
    }

    return (_enabled && gnss_set_antenna(antenna));
}

bool GNSSClass::setPPS(unsigned int width)
{
    return (_enabled && gnss_set_pps(width));
}

bool GNSSClass::setConstellation(GNSSconstellation constellation)
{
    return (_enabled && gnss_set_constellation(constellation));
}

bool GNSSClass::setSBAS(bool enable)
{
    return (_enabled && gnss_set_sbas(enable));
}

bool GNSSClass::setQZSS(bool enable)
{
    return (_enabled && gnss_set_qzss(enable));
}

bool GNSSClass::setAutonomous(bool enable)
{
    return (_enabled && gnss_set_autonomous(enable));
}

bool GNSSClass::setPlatform(GNSSplatform platform)
{
    return (_enabled && gnss_set_platform(platform));
}

bool GNSSClass::setPeriodic(unsigned int acqTime, unsigned int onTime, unsigned int period)
{
    return (_enabled && gnss_set_periodic(acqTime, onTime, period));
}

bool GNSSClass::suspend()
{
    if (!(_enabled && gnss_suspend()))
    {
        return false;
    }

    if (_pins.pps != STM32L0_GPIO_PIN_NONE)
    {
        stm32l0_exti_detach(_pins.pps);
        stm32l0_gpio_pin_configure(_pins.pps, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
    }

    return true;
}

bool GNSSClass::resume()
{
    if (!(_enabled && gnss_resume()))
    {
        return false;
    }

    if (_pins.pps != STM32L0_GPIO_PIN_NONE)
    {
        stm32l0_gpio_pin_configure(_pins.pps, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));
        stm32l0_exti_attach(_pins.pps, STM32L0_EXTI_CONTROL_EDGE_FALLING | STM32L0_EXTI_CONTROL_PRIORITY_CRITICAL, (stm32l0_exti_callback_t)&gnss_pps_callback, (void*)NULL);
    }
    
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

void GNSSClass::enableWakeup()
{
    _wakeup = true;
}

void GNSSClass::disableWakeup()
{
    _wakeup = false;
}

void GNSSClass::uartBegin(GNSSmode mode, GNSSrate rate, struct _stm32l0_uart_t *uart, const struct _stm32l0_uart_params_t *params, uint16_t pps, uint16_t enable, uint16_t backup, bool internal)
{
    static const gnss_callbacks_t GNSSCallbacks = {
        NULL,
        NULL,
        (gnss_location_callback_t)&GNSSClass::locationCallback,
        (gnss_satellites_callback_t)&GNSSClass::satellitesCallback,
    };

    static const gnss_callbacks_t GNSSCallbacksEnable = {
        (gnss_enable_callback_t)&GNSSClass::enableCallback,
        (gnss_disable_callback_t)&GNSSClass::disableCallback,
        (gnss_location_callback_t)&GNSSClass::locationCallback,
        (gnss_satellites_callback_t)&GNSSClass::satellitesCallback,
    };

    if (params)
    {
        if (uart->state == STM32L0_UART_STATE_NONE)
        {
            stm32l0_uart_create(uart, params);
        }
    }

    _uart = uart;

    _pins.pps = pps;
    _pins.enable = enable;
    _pins.backup = backup;

    _internal = internal;
    
    if (_pins.backup != STM32L0_GPIO_PIN_NONE)
    {
        stm32l0_gpio_pin_configure(_pins.backup, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
        stm32l0_gpio_pin_write(_pins.backup, 1);
    }

    if (_pins.enable != STM32L0_GPIO_PIN_NONE)
    {
        stm32l0_gpio_pin_configure(_uart->pins.rx, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));
        stm32l0_gpio_pin_configure(_uart->pins.tx, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
        stm32l0_gpio_pin_write(_uart->pins.tx, 1);
        
        stm32l0_gpio_pin_configure(_pins.enable, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLUP | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
        stm32l0_gpio_pin_write(_pins.enable, 1);
        
        while (!stm32l0_gpio_pin_read(_uart->pins.rx))
        {
        }
    }

    if (_pins.pps != STM32L0_GPIO_PIN_NONE)
    {
        stm32l0_gpio_pin_configure(_pins.pps, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));
        stm32l0_exti_attach(_pins.pps, STM32L0_EXTI_CONTROL_EDGE_FALLING | STM32L0_EXTI_CONTROL_PRIORITY_CRITICAL, (stm32l0_exti_callback_t)&gnss_pps_callback, (void*)NULL);
    }

    _enabled = stm32l0_uart_enable(_uart,
                                   &_rx_data[0], sizeof(_rx_data),
                                   9600,
                                   (STM32L0_UART_OPTION_DATA_SIZE_8 | STM32L0_UART_OPTION_PARITY_NONE | STM32L0_UART_OPTION_STOP_1),
                                   (stm32l0_uart_event_callback_t)&GNSSClass::uartEventCallback, (void*)this);

    if (_enabled)
    {
        if (mode == MODE_NMEA)
        {
            _baudrate = 9600;
            
            gnss_initialize(mode, rate, _baudrate, (gnss_send_routine_t)&GNSSClass::uartSendRoutine, ((_pins.enable != STM32L0_GPIO_PIN_NONE) ? &GNSSCallbacksEnable : &GNSSCallbacks), (void*)this);
        }
        else
        {
            _baudrate = (rate > RATE_1HZ) ? 115200 : 19200;
            
            gnss_initialize(mode, rate, _baudrate, (gnss_send_routine_t)&GNSSClass::uartSendRoutine, ((_pins.enable != STM32L0_GPIO_PIN_NONE) ? &GNSSCallbacksEnable : &GNSSCallbacks), (void*)this);
        }
    }
}

void GNSSClass::uartEnd()
{
    if (_enabled)
    {
        stm32l0_uart_disable(_uart);

        _enabled = false;
    }

    if (_pins.pps != STM32L0_GPIO_PIN_NONE)
    {
        stm32l0_exti_detach(_pins.pps);
        stm32l0_gpio_pin_configure(_pins.pps, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
    }

    if (_pins.enable != STM32L0_GPIO_PIN_NONE)
    {
        stm32l0_gpio_pin_configure(_pins.enable, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
    }
    
    if (_pins.backup != STM32L0_GPIO_PIN_NONE)
    {
        stm32l0_gpio_pin_configure(_pins.backup, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
    }
}

void GNSSClass::uartReceiveCallback(class GNSSClass *self)
{
    uint8_t rx_data[32];
    int rx_count;

    do
    {
        rx_count = stm32l0_uart_input(self->_uart, &rx_data[0], sizeof(rx_data), true);
        
        if (rx_count > 0)
        {
            gnss_receive(&rx_data[0], rx_count);
        }
    }
    while (rx_count > 0);
}

void GNSSClass::uartEventCallback(class GNSSClass *self, uint32_t events)
{
    if (events & STM32L0_UART_EVENT_RECEIVE) {
        armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)&GNSSClass::uartReceiveCallback, self, 0);
    }
}

void GNSSClass::uartDoneCallback(class GNSSClass *self)
{
    if (self->_doneCallback) {
        armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)self->_doneCallback, NULL, 0);
    } else {
        stm32l0_uart_configure(self->_uart,
                               self->_baudrate,
                               (STM32L0_UART_OPTION_DATA_SIZE_8 | STM32L0_UART_OPTION_PARITY_NONE | STM32L0_UART_OPTION_STOP_1) | ((self->_baudrate <= 19200) ? STM32L0_UART_OPTION_WAKEUP : 0));
    }
}

void GNSSClass::uartSendRoutine(class GNSSClass *self, const uint8_t *data, uint32_t count, gnss_send_callback_t callback)
{
    self->_doneCallback = callback;
    stm32l0_uart_transmit(self->_uart, data, count, (stm32l0_uart_done_callback_t)&GNSSClass::uartDoneCallback, (void*)self);
}

void GNSSClass::enableCallback(class GNSSClass *self)
{
    stm32l0_uart_enable(self->_uart,
                        &self->_rx_data[0], sizeof(self->_rx_data),
                        self->_baudrate,
                        (STM32L0_UART_OPTION_DATA_SIZE_8 | STM32L0_UART_OPTION_PARITY_NONE | STM32L0_UART_OPTION_STOP_1) | ((self->_baudrate <= 19200) ? STM32L0_UART_OPTION_WAKEUP : 0),
                        (stm32l0_uart_event_callback_t)&GNSSClass::uartEventCallback, (void*)self);

    stm32l0_gpio_pin_configure(self->_pins.enable, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(self->_pins.enable, 1);
}

void GNSSClass::disableCallback(class GNSSClass *self)
{
    stm32l0_gpio_pin_configure(self->_pins.enable, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
    
    stm32l0_uart_disable(self->_uart);
}

void GNSSClass::locationCallback(class GNSSClass *self, const gnss_location_t *location)
{
    self->_location_data = *location;
    self->_location_pending = true;

    self->_locationCallback.queue(self->_wakeup);
}

void GNSSClass::satellitesCallback(class GNSSClass *self, const gnss_satellites_t *satellites)
{
    self->_satellites_data = *satellites;
    self->_satellites_pending = true;

    self->_satellitesCallback.queue(self->_wakeup);
}

GNSSClass GNSS;
