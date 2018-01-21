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

#ifndef _GNSS_H
#define _GNSS_H

#include "Arduino.h"
#include "utility/gnss_api.h"

#define GNSS_RX_BUFFER_SIZE 128

class GNSSLocation {
public:
    enum GNSSfixType {
	TYPE_NONE = 0,
	TYPE_TIME,
	TYPE_2D,
	TYPE_3D,
    };

    enum GNSSfixQuality {
	QUALITY_NONE = 0,
	QUALITY_AUTONOMOUS,
	QUALITY_DIFFERENTIAL,
	QUALITY_PRECISE,
	QUALITY_RTK_FIXED,
	QUALITY_RTK_FLOAT,
	QUALITY_ESTIMATED,
	QUALITY_MANUAL,
	QUALITY_SIMULATION,
    };

    GNSSLocation(const gnss_location_t *location);

    operator bool() const;

    enum GNSSfixType fixType(void) const;
    enum GNSSfixQuality fixQuality(void) const;
    unsigned int satellites(void) const;

    uint16_t year(void) const;
    uint8_t month(void) const;
    uint8_t day(void) const;
    uint8_t hour(void) const;
    uint8_t minute(void) const;
    uint8_t second(void) const;
    uint16_t millis(void) const;
    uint8_t correction(void) const;

    double latitude(void) const;  // WGS84
    double longitude(void) const; // WGS84
    float altitude(void) const;   // MSL
    float separation(void) const; // WGS84 = MSL + separation

    float speed(void) const;
    float course(void) const;
    float climb(void) const;

    float ehpe(void) const;
    float evpe(void) const;

    float pdop(void) const;
    float hdop(void) const;
    float vdop(void) const;

protected:
    gnss_location_t _location;

private:
};

class GNSSSatellites {
public:
    enum GNSSsatelliteState {
	STATE_NONE = 0,
	STATE_SEARCHING = 1,
	STATE_TRACKING = 2,
	STATE_NAVIGATING = 3,
    };

    GNSSSatellites(const gnss_satellites_t *satellites);

    unsigned int count() const;

    unsigned int prn(unsigned int index) const;
    enum GNSSsatelliteState state(unsigned int index) const;
    unsigned int snr(unsigned int index) const;
    unsigned int elevation(unsigned int index) const;
    unsigned int azimuth(unsigned int index) const;

protected:
    gnss_satellites_t _satellites;

private:
};

class GNSSClass {
public:
    enum GNSSmode {
	MODE_NMEA = 0,
	MODE_MEDIATEK,
	MODE_UBLOX,
    };

    enum GNSSrate {
	RATE_1HZ = 1,
	RATE_5HZ = 5,
	RATE_10HZ = 10,
    };

    enum GNSSconstellation {
	CONSTELLATION_GPS = 1,
	CONSTELLATION_GPS_AND_GLONASS = 3,
    };

    GNSSClass();

    void begin(GNSSmode mode, GNSSrate rate = RATE_1HZ);
    void end();

    bool setExternal(bool enable);
    bool setConstellation(GNSSconstellation constellation);
    bool setSBAS(bool on);
    bool setQZSS(bool on);
    bool setPeriodic(unsigned int onTime, unsigned int period, bool force = false);
    bool sleep();
    bool wakeup();
    bool ready();
    
    int available(void);
    GNSSLocation location(void);
    GNSSSatellites satellites(void);

    void onReceive(void(*callback)(void));

protected:

private:
    struct _stm32l0_uart_t *_uart;
    uint32_t _baudrate;
    uint8_t _rx_data[GNSS_RX_BUFFER_SIZE];
    gnss_location_t _location_data;
    volatile uint32_t _location_pending;
    gnss_satellites_t _satellites_data;
    volatile uint32_t _satellites_pending;
    void (*_send_callback)(void);
    void (*_receive_callback)(void);

    static void _doneCallback(class GNSSClass *self);
    static void _sendRoutine(class GNSSClass *self, const uint8_t *data, uint32_t count, gnss_send_callback_t callback);
    static void _locationCallback(class GNSSClass *self, const gnss_location_t *location);
    static void _satellitesCallback(class GNSSClass *self, const gnss_satellites_t *satellites);

    static void _receiveRoutine(class GNSSClass *self);
    static void _eventCallback(class GNSSClass *self, uint32_t events);
};

extern GNSSClass GNSS;

#endif /* _GNSS_H */
