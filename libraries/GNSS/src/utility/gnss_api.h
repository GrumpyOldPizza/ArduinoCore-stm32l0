/*
 * Copyright (c) 2015-2018 Thomas Roell.  All rights reserved.
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

#if !defined(_GNSS_API_H)
#define _GNSS_API_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GNSS_MODE_NMEA                         0
#define GNSS_MODE_UBLOX                        1

#define GNSS_ANTENNA_INTERNAL                  0
#define GNSS_ANTENNA_EXTERNAL                  1

#define GNSS_CONSTELLATION_GPS                 0x00000001
#define GNSS_CONSTELLATION_GLONASS             0x00000002
#define GNSS_CONSTELLATION_BEIDOU              0x00000004
#define GNSS_CONSTELLATION_GALILEO             0x00000008

#define GNSS_PLATFORM_PORTABLE                 0
#define GNSS_PLATFORM_STATIONARY               1
#define GNSS_PLATFORM_PEDESTRIAN               2
#define GNSS_PLATFORM_CAR                      3
#define GNSS_PLATFORM_SEA                      4
#define GNSS_PLATFORM_BALLON                   5 /* AIRBORNE 1G */
#define GNSS_PLATFORM_AVIATION                 6 /* AIRBORNE 2G */
#define GNSS_PLATFORM_COUNT                    7

#define GNSS_LOCATION_TYPE_NONE                0
#define GNSS_LOCATION_TYPE_TIME                1
#define GNSS_LOCATION_TYPE_2D                  2
#define GNSS_LOCATION_TYPE_3D                  3

#define GNSS_LOCATION_QUALITY_NONE             0
#define GNSS_LOCATION_QUALITY_AUTONOMOUS       1
#define GNSS_LOCATION_QUALITY_DIFFERENTIAL     2
#define GNSS_LOCATION_QUALITY_PRECISE          3
#define GNSS_LOCATION_QUALITY_RTK_FIXED        4
#define GNSS_LOCATION_QUALITY_RTK_FLOAT        5
#define GNSS_LOCATION_QUALITY_ESTIMATED        6
#define GNSS_LOCATION_QUALITY_MANUAL           7
#define GNSS_LOCATION_QUALITY_SIMULATION       8

typedef struct _utc_time_t {
    uint8_t        year;             /* 0 .. 255 (1980 == 0)         */
    uint8_t        month;            /* 1 .. 12                      */
    uint8_t        day;              /* 1 .. 31                      */
    uint8_t        hours;            /* 0 .. 23                      */
    uint8_t        minutes;          /* 0 .. 59                      */
    uint8_t        seconds;          /* 0 .. 60                      */ 
    uint16_t       millis;           /* 0 .. 999                     */
} utc_time_t;

#define GNSS_LOCATION_MASK_TIME           0x0001
#define GNSS_LOCATION_MASK_CORRECTION     0x0002
#define GNSS_LOCATION_MASK_RESOLVED       0x0004
#define GNSS_LOCATION_MASK_POSITION       0x0008
#define GNSS_LOCATION_MASK_ALTITUDE       0x0010
#define GNSS_LOCATION_MASK_SPEED          0x0020
#define GNSS_LOCATION_MASK_COURSE         0x0040
#define GNSS_LOCATION_MASK_CLIMB          0x0080
#define GNSS_LOCATION_MASK_EHPE           0x0100
#define GNSS_LOCATION_MASK_EVPE           0x0200
#define GNSS_LOCATION_MASK_PDOP           0x0400
#define GNSS_LOCATION_MASK_HDOP           0x0800
#define GNSS_LOCATION_MASK_VDOP           0x1000

typedef struct _gnss_location_t {
    utc_time_t     time;             /* UTC date/time                */
    uint16_t       mask;             /*                              */
    int8_t         correction;       /* GPS/UTC offset               */
    uint8_t        type;             /* fix type                     */
    int32_t        latitude;         /* (WGS84) degrees, 1e7         */
    int32_t        longitude;        /* (WGS84) degrees, 1e7         */
    int32_t        altitude;         /* (MSL) m, 1e3                 */
    int32_t        separation;       /* (WGS84) = (MSL) + separation */
    uint32_t       speed;            /* m/s, 1e3                     */
    uint32_t       course;           /* degrees, 1e5                 */
    int32_t        climb;            /* m/s, 1e3                     */
    uint32_t       ehpe;             /* m, 1e3                       */
    uint32_t       evpe;             /* m, 1e3                       */
    uint8_t        quality;          /* fix quality                  */
    uint8_t        numsv;            /* fix numsv                    */
    uint16_t       pdop;             /* 1e2                          */
    uint16_t       hdop;             /* 1e2                          */
    uint16_t       vdop;             /* 1e2                          */
} gnss_location_t;

#define GNSS_SATELLITES_STATE_UNHEALTHY        0x01
#define GNSS_SATELLITES_STATE_ALMANAC          0x02
#define GNSS_SATELLITES_STATE_EPHEMERIS        0x04
#define GNSS_SATELLITES_STATE_AUTONOMOUS       0x08
#define GNSS_SATELLITES_STATE_CORRECTION       0x10
#define GNSS_SATELLITES_STATE_ACQUIRED         0x20
#define GNSS_SATELLITES_STATE_LOCKED           0x40
#define GNSS_SATELLITES_STATE_NAVIGATING       0x80

#define GNSS_SATELLITES_COUNT_MAX              32

/* PRN ranges:
 *
 * 0       none
 * 1-32    GPS
 * 33-64   SBAS (+87)
 * 65-96   GLONASS (-64)
 * 193-200 QZSS
 * 201-237 BEIDOU (-200)
 * 255     GLONASS
 */

typedef struct _gnss_satellites_t {
    uint32_t       count;
    struct {
        uint8_t    svid;
        uint8_t    state;
        uint8_t    snr;
        uint8_t    elevation;
        uint16_t   azimuth;
    }              info[GNSS_SATELLITES_COUNT_MAX];
} gnss_satellites_t;

typedef void (*gnss_send_callback_t)(void);
typedef void (*gnss_send_routine_t)(void *context, const uint8_t *data, uint32_t count, gnss_send_callback_t callback);
typedef void (*gnss_enable_callback_t)(void *context);
typedef void (*gnss_disable_callback_t)(void *context);
typedef void (*gnss_location_callback_t)(void *context, const gnss_location_t *location);
typedef void (*gnss_satellites_callback_t)(void *context, const gnss_satellites_t *satellites);

typedef struct {
    gnss_enable_callback_t enable_callback;
    gnss_disable_callback_t disable_callback;
    gnss_location_callback_t location_callback;
    gnss_satellites_callback_t satellites_callback;
} gnss_callbacks_t;

extern void gnss_initialize(unsigned int mode, unsigned int rate, unsigned int speed, gnss_send_routine_t send_routine, const gnss_callbacks_t *callbacks, void *context);
extern void gnss_receive(const uint8_t *data, uint32_t count);
extern void gnss_pps_callback(void);
extern bool gnss_set_antenna(unsigned int antenna);
extern bool gnss_set_pps(unsigned int width);
extern bool gnss_set_constellation(unsigned int mask);
extern bool gnss_set_sbas(bool enable);
extern bool gnss_set_qzss(bool enable);
extern bool gnss_set_autonomous(bool enable);
extern bool gnss_set_platform(unsigned int platform);
extern bool gnss_set_periodic(unsigned int acqTime, unsigned int onTime, unsigned int period);
extern bool gnss_suspend(void);
extern bool gnss_resume(void);
extern bool gnss_busy(void);

#ifdef __cplusplus
}
#endif

#endif /* _GNSS_API_H */
