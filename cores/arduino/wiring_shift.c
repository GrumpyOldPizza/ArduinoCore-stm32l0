/*
 * Copyright (c) 2016-2018 Thomas Roell.  All rights reserved.
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
#include "wiring_private.h"

uint32_t shiftIn( uint32_t ulDataPin, uint32_t ulClockPin, uint32_t ulBitOrder )
{
    uint8_t value = 0 ;
    uint8_t i ;

    for ( i=0 ; i < 8 ; ++i )
    {
	digitalWrite( ulClockPin, HIGH ) ;

	if ( ulBitOrder == LSBFIRST )
	{
	    value |= digitalRead( ulDataPin ) << i ;
	}
	else
	{
	    value |= digitalRead( ulDataPin ) << (7 - i) ;
	}

	digitalWrite( ulClockPin, LOW ) ;
    }

    return value ;
}

void shiftOut( uint32_t ulDataPin, uint32_t ulClockPin, uint32_t ulBitOrder, uint32_t ulVal )
{
    uint8_t i ;

    for ( i=0 ; i < 8 ; i++ )
    {
	if ( ulBitOrder == LSBFIRST )
	{
	    digitalWrite( ulDataPin, !!(ulVal & (1 << i)) ) ;
	}
	else
	{
	    digitalWrite( ulDataPin, !!(ulVal & (1 << (7 - i))) ) ;
	}

	digitalWrite( ulClockPin, HIGH ) ;
	digitalWrite( ulClockPin, LOW ) ;
    }
}
