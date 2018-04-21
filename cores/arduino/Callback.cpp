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

#include "Arduino.h"
#include "wiring_private.h"

Callback::Callback(class EventHandler *event)
{
    _callback = reinterpret_cast<void (*)(void*)>(armv6m_event_enqueue);
    _context = reinterpret_cast<void*>(event);
}


/* The format of a member pointer is defined in the C++ ABI.
 * For ARM that document is IHI0041D, which describes the differences
 * to the Itanium C++ ABI.
 */

bool Callback::queue() {
    if (_callback) {
	return armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)_callback, _context, 0);
    } else {
	return false;
    }
}

void Callback::call() {
    if (_callback) {
	(*_callback)(_context);
    }
}

void Callback::bind(const void *method, const void *object) {
    void *ptr = (void*)(((const uint32_t*)method)[0]);
    ptrdiff_t adj = ((ptrdiff_t)(((const uint32_t*)method)[1]) >> 1);
    
    if (!((const uint32_t*)method)[1] & 1) {
	/* non-virtual function */
	_callback = (void(*)(void*))ptr;
    } else {
	/* virtual function */
	void *vptr = *((void**)((uintptr_t)object + adj)); 
	_callback = (void(*)(void*))(*((void**)((uint8_t*)vptr + (ptrdiff_t)ptr)));
    }
    _context = (void*)((uintptr_t)object + adj);
}
