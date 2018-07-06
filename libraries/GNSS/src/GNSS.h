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

    static const int LEAP_SECONDS_UNDEFINED = -128;

    GNSSLocation(const gnss_location_t *location);
    GNSSLocation();

    operator bool() const;

    enum GNSSfixType fixType(void) const;
    enum GNSSfixQuality fixQuality(void) const;
    bool fullyResolved(void) const;
    unsigned int satellites(void) const;

    uint16_t year(void) const;
    uint8_t month(void) const;
    uint8_t day(void) const;
    uint8_t hours(void) const;
    uint8_t minutes(void) const;
    uint8_t seconds(void) const;
    uint16_t millis(void) const;
    int8_t leapSeconds(void) const;

    double latitude(void) const;  // WGS84
    double longitude(void) const; // WGS84
    float height(void) const;   // WGS84
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

    void latitude(int32_t &latitude) const { latitude = _location.latitude; };
    void longitude(int32_t &longitude) const { longitude = _location.longitude; };
    void height(int32_t &height) const { height = _location.altitude + _location.separation; };
    void altitude(int32_t &altitude) const { altitude =  _location.altitude; };

private:
    gnss_location_t _location;
};

class GNSSSatellites {
public:
    GNSSSatellites(const gnss_satellites_t *satellites);
    GNSSSatellites();

    unsigned int count() const;

    unsigned int svid(unsigned int index) const;
    unsigned int snr(unsigned int index) const;
    unsigned int elevation(unsigned int index) const;
    unsigned int azimuth(unsigned int index) const;
    bool unhealthy(unsigned int index) const;
    bool almanac(unsigned int index) const;
    bool ephemeris(unsigned int index) const;
    bool autonomous(unsigned int index) const;
    bool correction(unsigned int index) const;
    bool acquired(unsigned int index) const;
    bool locked(unsigned int index) const;
    bool navigating(unsigned int index) const;

private:
    gnss_satellites_t _satellites;
};

class GNSSClass {
public:
    enum GNSSmode {
        MODE_NMEA = 0,
        MODE_UBLOX,
    };

    enum GNSSrate {
        RATE_1HZ = 1,
        RATE_5HZ = 5,
        RATE_10HZ = 10,
    };

    enum GNSSantenna {
        ANTENNA_INTERNAL = 0,
        ANTENNA_EXTERNAL,
    };

    enum GNSSconstellation {
        CONSTELLATION_GPS = 1,
        CONSTELLATION_GPS_AND_GLONASS = 3,
    };

    enum GNSSplatform {
        PLATFORM_PORTABLE = 0,
        PLATFORM_STATIONARY,
        PLATFORM_PEDESTRIAN,
        PLATFORM_CAR,
        PLATFORM_SEA,
        PLATFORM_BALLON,
        PLATFORM_AVIATION,
    };

    GNSSClass();

    void begin(Uart &uart, GNSSmode mode, GNSSrate rate = RATE_1HZ);
    void end();

    bool setAntenna(GNSSantenna antenna);
    bool setPPS(unsigned int width);
    bool setConstellation(GNSSconstellation constellation);
    bool setSBAS(bool enable);
    bool setQZSS(bool enable);
    bool setAutonomous(bool enable);
    bool setPlatform(GNSSplatform platform);
    bool setPeriodic(unsigned int acqTime, unsigned int onTime, unsigned int period);
    bool suspend();
    bool resume();
    bool busy();
    
    bool location(GNSSLocation &location);
    bool satellites(GNSSSatellites &satellites);

    void enableWakeup();
    void disableWakeup();

    void onLocation(void(*callback)(void));
    void onLocation(Callback callback);
    void onSatellites(void(*callback)(void));
    void onSatellites(Callback callback);

    void attachInterrupt(void(*callback)(void));
    void detachInterrupt();

private:
    Uart *_uart;
    uint32_t _baudrate;
    bool _wakeup;
    gnss_location_t _location_data;
    volatile uint32_t _location_pending;
    gnss_satellites_t _satellites_data;
    volatile uint32_t _satellites_pending;

    Callback _locationCallback;
    Callback _satellitesCallback;

    void (*_ppsCallback)(void);
    void (*_doneCallback)(void);

    void receiveCallback(void);
    void completionCallback(void);

    static void sendRoutine(class GNSSClass*, const uint8_t*, uint32_t, gnss_send_callback_t);
    static void enableCallback(class GNSSClass*);
    static void disableCallback(class GNSSClass*);
    static void locationCallback(class GNSSClass*, const gnss_location_t*);
    static void satellitesCallback(class GNSSClass*, const gnss_satellites_t*);
    static void ppsCallback(class GNSSClass*);
};

extern GNSSClass GNSS;

#endif /* _GNSS_H */
