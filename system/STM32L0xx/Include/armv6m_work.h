/*
 * Copyright (c) 2019-2020 Thomas Roell.  All rights reserved.
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

#if !defined(_ARMV6M_WORK_H)
#define _ARMV6M_WORKD_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _armv6m_work_t {
    struct _armv6m_work_t * volatile next;
    armv6m_core_callback_t           callback;
} armv6m_work_t;

#define ARMV6M_WORK_INIT(_routine, _context) {	           \
    .callback.routine = (armv6m_core_routine_t)(_routine), \
    .callback.context = (void*)(_context),	  	   \
}

extern void __armv6m_work_initialize(void);

extern void armv6m_work_create(armv6m_work_t *work, armv6m_core_routine_t routine, void *context);
extern bool armv6m_work_destory(armv6m_work_t *work);
extern bool armv6m_work_submit(armv6m_work_t *work);
extern void armv6m_work_block(void);
extern void armv6m_work_unblock(void);
  
#ifdef __cplusplus
}
#endif

#endif /* _ARMV6M_WORK_H */
