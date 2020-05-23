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

#if !defined(_ARMV6M_CORE_H)
#define _ARMV6M_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#define ARMV6M_IRQ_PRIORITY_CRITICAL  0
#define ARMV6M_IRQ_PRIORITY_HIGH      1
#define ARMV6M_IRQ_PRIORITY_MEDIUM    2
#define ARMV6M_IRQ_PRIORITY_LOW       3

#define ARMV6M_IRQ_PRIORITY_SYSTICK   1
#define ARMV6M_IRQ_PRIORITY_PENDSV    3
#define ARMV6M_IRQ_PRIORITY_SVCALL    3
  
#define Reset_IRQn        ((IRQn_Type)-16)
#define NMI_IRQn          ((IRQn_Type)-14)
#define HardFault_IRQn    ((IRQn_Type)-13)
#define SVCall_IRQn       ((IRQn_Type)-5)
#define PendSV_IRQn       ((IRQn_Type)-2)
#define SysTick_IRQn      ((IRQn_Type)-1)

typedef enum
{
  ThreadMode_EXCn             = 0,
  NMI_EXCn                    = 2,      /*!< 2 Cortex-M0+ NMI Exception                              */
  HardFault_EXCn              = 3,      /*!< 3 Cortex-M0+ Hard Fault Exception                       */
  SVCall_EXCn                 = 11,     /*!< 11 Cortex-M0+ SV Call Exception                         */
  PendSV_EXCn                 = 14,     /*!< 14 Cortex-M0+ Pend SV Exception                         */
  SysTick_EXCn                = 15,     /*!< 15 Cortex-M0+ System Tick Exception                     */
  IRQ0_EXCn                   = 16,
} EXCn_Type;
  
typedef void (*armv6m_core_routine_t)(void *context);

typedef struct _armv6m_core_callback_t {
    armv6m_core_routine_t  routine;
    void                   *context;
} armv6m_core_callback_t;

static inline IRQn_Type __current_irq(void)
{
    return (IRQn_Type)(__get_IPSR() - 16);
}

extern void NMI_Handler(void);
extern void HardFault_Handler(void);
extern void SVC_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);
  
extern void __armv6m_core_initialize(void);
extern void armv6m_core_udelay(uint32_t udelay);

extern void armv6m_core_c_function(armv6m_core_callback_t *callback, void *function);
extern void armv6m_core_cxx_method(armv6m_core_callback_t *callback, const void *method, void *object);
  
static inline void armv6m_core_load_2(volatile uint32_t *p, uint32_t *p_data_0, uint32_t *p_data_1)
{
    register uint32_t _a0 __asm__("r0");
    register uint32_t _a1 __asm__("r1");
    
    __asm__ volatile ("ldm %0!, { %1, %2 }"
                      : "+&l" (p), "=r" (_a0), "=r" (_a1)
                      :
                      : "memory");

    *p_data_0 = _a0;
    *p_data_1 = _a1;
}

static inline void armv6m_core_load_3(volatile uint32_t *p, uint32_t *p_data_0, uint32_t *p_data_1, uint32_t *p_data_2)
{
    register uint32_t _a0 __asm__("r0");
    register uint32_t _a1 __asm__("r1");
    register uint32_t _a2 __asm__("r2");
    
    __asm__ volatile ("ldm %0!, { %1, %2, %3 }"
                      : "+&l" (p), "=r" (_a0), "=r" (_a1), "=r" (_a2)
                      :
                      : "memory");

    *p_data_0 = _a0;
    *p_data_1 = _a1;
    *p_data_2 = _a2;
}

static inline void armv6m_core_load_4(volatile uint32_t *p, uint32_t *p_data_0, uint32_t *p_data_1, uint32_t *p_data_2, uint32_t *p_data_3)
{
    register uint32_t _a0 __asm__("r0");
    register uint32_t _a1 __asm__("r1");
    register uint32_t _a2 __asm__("r2");
    register uint32_t _a3 __asm__("r3");
    
    __asm__ volatile ("ldm %0!, { %1, %2, %3, %4 }"
                      : "+&l" (p), "=r" (_a0), "=r" (_a1), "=r" (_a2), "=r" (_a3)
                      :
                      : "memory");

    *p_data_0 = _a0;
    *p_data_1 = _a1;
    *p_data_2 = _a2;
    *p_data_3 = _a3;
}

 static inline void armv6m_core_load_5(volatile uint32_t *p, uint32_t *p_data_0, uint32_t *p_data_1, uint32_t *p_data_2, uint32_t *p_data_3, uint32_t *p_data_4)
{
    register uint32_t _a0 __asm__("r0");
    register uint32_t _a1 __asm__("r1");
    register uint32_t _a2 __asm__("r2");
    register uint32_t _a3 __asm__("r3");
    register uint32_t _a4 __asm__("r4");
    
    __asm__ volatile ("ldm %0!, { %1, %2, %3, %4, %5 }"
                      : "+&l" (p), "=r" (_a0), "=r" (_a1), "=r" (_a2), "=r" (_a3), "=r" (_a4)
                      :
                      : "memory");

    *p_data_0 = _a0;
    *p_data_1 = _a1;
    *p_data_2 = _a2;
    *p_data_3 = _a3;
    *p_data_4 = _a4;
}

  static inline void armv6m_core_load_6(volatile uint32_t *p, uint32_t *p_data_0, uint32_t *p_data_1, uint32_t *p_data_2, uint32_t *p_data_3, uint32_t *p_data_4, uint32_t *p_data_5)
{
    register uint32_t _a0 __asm__("r0");
    register uint32_t _a1 __asm__("r1");
    register uint32_t _a2 __asm__("r2");
    register uint32_t _a3 __asm__("r3");
    register uint32_t _a4 __asm__("r4");
    register uint32_t _a5 __asm__("r5");
    
    __asm__ volatile ("ldm %0!, { %1, %2, %3, %4, %5, %6 }"
                      : "+&l" (p), "=r" (_a0), "=r" (_a1), "=r" (_a2), "=r" (_a3), "=r" (_a4), "=r" (_a5)
                      :
                      : "memory");

    *p_data_0 = _a0;
    *p_data_1 = _a1;
    *p_data_2 = _a2;
    *p_data_3 = _a3;
    *p_data_4 = _a4;
    *p_data_5 = _a5;
}
  
static inline void armv6m_core_store_2(volatile uint32_t *p, uint32_t data_0, uint32_t data_1)
{
    register uint32_t _a0 __asm__("r0") = data_0;
    register uint32_t _a1 __asm__("r1") = data_1;
    
    __asm__ volatile ("stm %0!, { %1, %2 }"
                      : "+&l" (p)
                      : "r" (_a0), "r" (_a1)
                      : "memory");
}

static inline void armv6m_core_store_3(volatile uint32_t *p, uint32_t data_0, uint32_t data_1, uint32_t data_2)
{
    register uint32_t _a0 __asm__("r0") = data_0;
    register uint32_t _a1 __asm__("r1") = data_1;
    register uint32_t _a2 __asm__("r2") = data_2;
    
    __asm__ volatile ("stm %0!, { %1, %2, %3 }"
                      : "+&l" (p)
                      : "r" (_a0), "r" (_a1), "r" (_a2)
                      : "memory");
}

static inline void armv6m_core_store_4(volatile uint32_t *p, uint32_t data_0, uint32_t data_1, uint32_t data_2, uint32_t data_3)
{
    register uint32_t _a0 __asm__("r0") = data_0;
    register uint32_t _a1 __asm__("r1") = data_1;
    register uint32_t _a2 __asm__("r2") = data_2;
    register uint32_t _a3 __asm__("r3") = data_3;
    
    __asm__ volatile ("stm %0!, { %1, %2, %3, %4 }"
                      : "+&l" (p)
                      : "r" (_a0), "r" (_a1), "r" (_a2), "r" (_a3)
                      : "memory");
}

static inline void armv6m_core_store_5(volatile uint32_t *p, uint32_t data_0, uint32_t data_1, uint32_t data_2, uint32_t data_3, uint32_t data_4)
{
    register uint32_t _a0 __asm__("r0") = data_0;
    register uint32_t _a1 __asm__("r1") = data_1;
    register uint32_t _a2 __asm__("r2") = data_2;
    register uint32_t _a3 __asm__("r3") = data_3;
    register uint32_t _a4 __asm__("r4") = data_4;
    
    __asm__ volatile ("stm %0!, { %1, %2, %3, %4, %5 }"
                      : "+&l" (p)
                      : "r" (_a0), "r" (_a1), "r" (_a2), "r" (_a3), "r" (_a4)
                      : "memory");
}

static inline void armv6m_core_store_6(volatile uint32_t *p, uint32_t data_0, uint32_t data_1, uint32_t data_2, uint32_t data_3, uint32_t data_4, uint32_t data_5)
{
    register uint32_t _a0 __asm__("r0") = data_0;
    register uint32_t _a1 __asm__("r1") = data_1;
    register uint32_t _a2 __asm__("r2") = data_2;
    register uint32_t _a3 __asm__("r3") = data_3;
    register uint32_t _a4 __asm__("r4") = data_4;
    register uint32_t _a5 __asm__("r5") = data_5;
    
    __asm__ volatile ("stm %0!, { %1, %2, %3, %4, %5, %6 }"
                      : "+&l" (p)
                      : "r" (_a0), "r" (_a1), "r" (_a2), "r" (_a3), "r" (_a4), "r" (_a5)
                      : "memory");
}
  
#ifdef __cplusplus
}
#endif

#endif /* _ARMV6M_CORE_H */
