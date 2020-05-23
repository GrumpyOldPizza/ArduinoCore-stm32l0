/*
 * Copyright (c) 2014-2020 Thomas Roell.  All rights reserved.
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

#include "armv6m.h"

uint32_t armv6m_atomic_add(volatile uint32_t *p_data, uint32_t data)
{
    return __armv6m_atomic_add(p_data, data);
}

uint32_t armv6m_atomic_sub(volatile uint32_t *p_data, uint32_t data)
{
    return __armv6m_atomic_sub(p_data, data);
}

uint32_t armv6m_atomic_and(volatile uint32_t *p_data, uint32_t data)
{
    return __armv6m_atomic_and(p_data, data);
}

uint32_t armv6m_atomic_or(volatile uint32_t *p_data, uint32_t data)
{
    return __armv6m_atomic_or(p_data, data);
}

uint32_t armv6m_atomic_xor(volatile uint32_t *p_data, uint32_t data)
{
    return __armv6m_atomic_xor(p_data, data);
}

uint32_t armv6m_atomic_inc(volatile uint32_t *p_data, uint32_t data_limit)
{
    return __armv6m_atomic_inc(p_data, data_limit);
}

uint32_t armv6m_atomic_dec(volatile uint32_t *p_data)
{
    return __armv6m_atomic_dec(p_data);
}

uint32_t armv6m_atomic_swap(volatile uint32_t *p_data, uint32_t data)
{
    return __armv6m_atomic_swap(p_data, data);
}

uint32_t armv6m_atomic_cas(volatile uint32_t *p_data, uint32_t data_expected, uint32_t data)
{
    return __armv6m_atomic_cas(p_data, data_expected, data);
}

uint32_t armv6m_atomic_andh(volatile uint16_t *p_data, uint32_t data)
{
    return __armv6m_atomic_andh(p_data, data);
}

uint32_t armv6m_atomic_orh(volatile uint16_t *p_data, uint32_t data)
{
    return __armv6m_atomic_orh(p_data, data);
}

uint32_t armv6m_atomic_swaph(volatile uint16_t *p_data, uint32_t data)
{
    return __armv6m_atomic_swaph(p_data, data);
}

uint32_t armv6m_atomic_cash(volatile uint16_t *p_data, uint32_t data_expected, uint32_t data)
{
    return __armv6m_atomic_cash(p_data, data_expected, data);
}

uint32_t armv6m_atomic_andb(volatile uint8_t *p_data, uint32_t data)
{
    return __armv6m_atomic_andb(p_data, data);
}

uint32_t armv6m_atomic_orb(volatile uint8_t *p_data, uint32_t data)
{
    return __armv6m_atomic_orb(p_data, data);
}

uint32_t armv6m_atomic_decb(volatile uint8_t *p_data)
{
    return __armv6m_atomic_decb(p_data);
}

uint32_t armv6m_atomic_incb(volatile uint8_t *p_data)
{
    return __armv6m_atomic_incb(p_data);
}

uint32_t armv6m_atomic_swapb(volatile uint8_t *p_data, uint32_t data)
{
    return __armv6m_atomic_swapb(p_data, data);
}

uint32_t armv6m_atomic_casb(volatile uint8_t *p_data, uint32_t data_expected, uint32_t data)
{
    return __armv6m_atomic_casb(p_data, data_expected, data);
}

uint32_t armv6m_atomic_modify(volatile uint32_t *p_data, uint32_t mask, uint32_t data)
{
    return __armv6m_atomic_modify(p_data, mask, data);
}

uint32_t armv6m_atomic_modifyz(volatile uint32_t *p_data, uint32_t mask, uint32_t data, volatile uint32_t *p_zero, uint32_t bits)
{
    return __armv6m_atomic_modifyz(p_data, mask, data, p_zero, bits);
}

uint32_t armv6m_atomic_modifyzb(volatile uint32_t *p_data, uint32_t mask, uint32_t data, volatile uint8_t *p_zero)
{
    return __armv6m_atomic_modifyzb(p_data, mask, data, p_zero);
}

uint32_t armv6m_atomic_andz(volatile uint32_t *p_data, uint32_t data, volatile uint32_t *p_zero, uint32_t bits)
{
    return __armv6m_atomic_andz(p_data, data, p_zero, bits);
}

uint32_t armv6m_atomic_andzb(volatile uint32_t *p_data, uint32_t data, volatile uint8_t *p_zero)
{
    return __armv6m_atomic_andzb(p_data, data, p_zero);
}

uint32_t armv6m_atomic_orz(volatile uint32_t *p_data, uint32_t data, volatile uint32_t *p_zero, uint32_t bits)
{
    return __armv6m_atomic_orz(p_data, data, p_zero, bits);
}

uint32_t armv6m_atomic_orzb(volatile uint32_t *p_data, uint32_t data, volatile uint8_t *p_zero)
{
    return __armv6m_atomic_orzb(p_data, data, p_zero);
}
