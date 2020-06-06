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

#if !defined(_ARMV6M_PENDSV_H)
#define _ARMV6M_PENDSV_H

#ifdef __cplusplus
extern "C" {
#endif
  
typedef void (*armv6m_pendsv_callback_t)(void);
typedef void (*armv6m_pendsv_routine_t)(void *context, uint32_t data);

extern void __armv6m_pendsv_initialize(void);

extern void armv6m_pendsv_hook(armv6m_pendsv_callback_t callback);
extern bool armv6m_pendsv_enqueue(armv6m_pendsv_routine_t routine, void *context, uint32_t data);
extern bool armv6m_pendsv_raise(uint32_t index);
extern void armv6m_pendsv_block(uint32_t mask);
extern void armv6m_pendsv_unblock(uint32_t mask);

extern void SWI0_IRQHandler(void);
extern void SWI1_IRQHandler(void);
extern void SWI2_IRQHandler(void);
extern void SWI3_IRQHandler(void);
extern void SWI4_IRQHandler(void);
extern void SWI5_IRQHandler(void);
extern void SWI6_IRQHandler(void);
extern void SWI7_IRQHandler(void);
extern void SWI8_IRQHandler(void);
extern void SWI9_IRQHandler(void);
extern void SWI10_IRQHandler(void);
extern void SWI11_IRQHandler(void);
extern void SWI12_IRQHandler(void);
extern void SWI13_IRQHandler(void);
extern void SWI14_IRQHandler(void);
extern void SWI15_IRQHandler(void);
extern void SWI16_IRQHandler(void);
extern void SWI17_IRQHandler(void);
extern void SWI18_IRQHandler(void);
extern void SWI19_IRQHandler(void);
extern void SWI20_IRQHandler(void);
extern void SWI21_IRQHandler(void);
extern void SWI22_IRQHandler(void);
extern void SWI23_IRQHandler(void);
extern void SWI24_IRQHandler(void);
extern void SWI25_IRQHandler(void);
extern void SWI26_IRQHandler(void);
extern void SWI27_IRQHandler(void);
extern void SWI28_IRQHandler(void);
extern void SWI29_IRQHandler(void);
extern void SWI30_IRQHandler(void);
extern void SWI31_IRQHandler(void);

  
#define ARMV6M_PENDSV_SWI_RADIO             0
#define ARMV6M_PENDSV_SWI_EXTI              1
#define ARMV6M_PENDSV_SWI_LPTIM             2
#define ARMV6M_PENDSV_SWI_RTC_MODIFY        3
#define ARMV6M_PENDSV_SWI_RTC_ALARM         4
#define ARMV6M_PENDSV_SWI_RTC_TIMER         5
#define ARMV6M_PENDSV_SWI_USBD_CDC_CONTROL  6
#define ARMV6M_PENDSV_SWI_USBD_CDC_RECEIVE  7
#define ARMV6M_PENDSV_SWI_USBD_CDC_TRANSMIT 8
#define ARMV6M_PENDSV_SWI_WORK_SCHEDULE     9

#define SWI_RADIO_IRQHandler                SWI0_IRQHandler
#define SWI_EXTI_IRQHandler                 SWI1_IRQHandler
#define SWI_LPTIM_IRQHandler                SWI2_IRQHandler
#define SWI_RTC_MODIFY_IRQHandler           SWI3_IRQHandler
#define SWI_RTC_ALARM_IRQHandler            SWI4_IRQHandler
#define SWI_RTC_TIMER_IRQHandler            SWI5_IRQHandler
#define SWI_USBD_CDC_CONTROL_IRQHandler     SWI6_IRQHandler
#define SWI_USBD_CDC_RECEIVE_IRQHandler     SWI7_IRQHandler
#define SWI_USBD_CDC_TRANSMIT_IRQHandler    SWI8_IRQHandler
#define SWI_WORK_SCHEDULE_IRQHandler        SWI9_IRQHandler
  
#ifdef __cplusplus
}
#endif

#endif /* _ARMV6M_PENDSV_H */
