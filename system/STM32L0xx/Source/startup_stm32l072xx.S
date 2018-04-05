/*
 * Copyright (c) 2008-2018 Thomas Roell.  All rights reserved.
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
 * THE SOFTWARE IS PROVINCED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

        .arch armv6-m
        .syntax unified

        .section .stack, "aw"
        .align  3
#ifdef __STACK_SIZE
        .space  __STACK_SIZE
#else
        .space  2048
#endif

        .section .isr_vector, "ax"
        .align  2
        .globl  __isr_vector
__isr_vector:
        .long   0xbe00be00            /* Top of Stack (needs to be 0) */
        .long   Reset_Handler         /* Reset Handler */
        .long   NMI_Handler           /* NMI Handler */
        .long   HardFault_Handler     /* Hard Fault Handler */
        .long   0                     /* Reserved */
        .long   0                     /* Reserved */
        .long   0                     /* Reserved */
        .long   0                     /* Reserved */
        .long   0                     /* Reserved */
        .long   0                     /* Reserved */
        .long   0                     /* Reserved */
        .long   SVC_Handler           /* SVCall Handler */
        .long   0                     /* Reserved */
        .long   0                     /* Reserved */
        .long   PendSV_Handler        /* PendSV Handler */
        .long   SysTick_Handler       /* SysTick Handler */

        /* External interrupts */
        .long   WWDG_IRQHandler
        .long   PVD_IRQHandler
        .long   RTC_IRQHandler
        .long   FLASH_IRQHandler
        .long   RCC_CRS_IRQHandler
        .long   EXTI0_1_IRQHandler
        .long   EXTI2_3_IRQHandler
        .long   EXTI4_15_IRQHandler
        .long   TSC_IRQHandler
        .long   DMA1_Channel1_IRQHandler
        .long   DMA1_Channel2_3_IRQHandler
        .long   DMA1_Channel4_5_6_7_IRQHandler
        .long   ADC1_COMP_IRQHandler
        .long   LPTIM1_IRQHandler
        .long   USART4_5_IRQHandler
        .long   TIM2_IRQHandler
        .long   TIM3_IRQHandler
        .long   TIM6_DAC_IRQHandler
        .long   TIM7_IRQHandler
        .long   0
        .long   TIM21_IRQHandler
        .long   I2C3_IRQHandler
        .long   TIM22_IRQHandler
        .long   I2C1_IRQHandler
        .long   I2C2_IRQHandler
        .long   SPI1_IRQHandler
        .long   SPI2_IRQHandler
        .long   USART1_IRQHandler
        .long   USART2_IRQHandler
        .long   RNG_LPUART1_IRQHandler
        .long   0
        .long   USB_IRQHandler

        .size   __isr_vector, . - __isr_vector


        .text
        .thumb
        .thumb_func
        .align  2
        .globl  Reset_Handler
        .type   Reset_Handler, %function
Reset_Handler:
        ldr     r0, =__StackTop
        mov     sp, r0
        bl      SystemInit
        
        /*
         *  The __copy_table has the following format per entry:
         *
         *    offset 0: LMA of start of a DATA section to copy from
         *    offset 4: VMA of start of a DATA section to copy to
         *    offset 8: Size of this DATA sectiony. Must be multiply of 4
         *
         *  All addresses must be aligned to 4 bytes boundary.
         */

        ldr     r1, =__copy_table_start__
        ldr     r2, =__copy_table_end__
        b       3f
1:      ldmia   r1!, { r3, r4, r5 }
        cmp     r5, #0
        beq     3f
2:      subs    r5, #4
        ldr     r0, [r3, r5]
        str     r0, [r4, r5]
        bne     2b
3:      cmp     r1, r2
        bne     1b

        /*
         *  The __zero_table has the following format per entry:
         *
         *    offset 0: VMA of start of a BSS section to zero
         *    offset 4: Size of this BSS section. Must be multiply of 4
         *
         *  All addresses must be aligned to 4 bytes boundary.
         */

        movs    r0, #0
        ldr     r1, =__zero_table_start__
        ldr     r2, =__zero_table_end__
        b       6f
4:      ldmia   r1!, { r3, r4 }
        cmp     r4, #0
        beq     6f
5:      subs    r4, #4
        str     r0, [r3, r4]
        bne     5b
6:      cmp     r1, r2
        bne     4b
        
#ifndef __START
#define __START main
#endif
        bl      __START

        .size   Reset_Handler, . - Reset_Handler

        .text
        .thumb
        .thumb_func
        .align  2
        .weak   Default_Handler
        .type   Default_Handler, %function
Default_Handler:
        b       .
        .size   Default_Handler, . - Default_Handler

        /*
         * Macro to define default handlers. Default handler
         * will be weak symbol and just dead loops. They can be
         * overwritten by other handlers
         */
        .macro  def_irq_handler handler_name
        .weak   \handler_name
        .set    \handler_name, Default_Handler
        .endm

        def_irq_handler NMI_Handler
        def_irq_handler HardFault_Handler
        def_irq_handler SVC_Handler
        def_irq_handler PendSV_Handler
        def_irq_handler SysTick_Handler
        def_irq_handler WWDG_IRQHandler
        def_irq_handler PVD_IRQHandler
        def_irq_handler RTC_IRQHandler
        def_irq_handler FLASH_IRQHandler
        def_irq_handler RCC_CRS_IRQHandler
        def_irq_handler EXTI0_1_IRQHandler
        def_irq_handler EXTI2_3_IRQHandler
        def_irq_handler EXTI4_15_IRQHandler
        def_irq_handler TSC_IRQHandler
        def_irq_handler DMA1_Channel1_IRQHandler
        def_irq_handler DMA1_Channel2_3_IRQHandler
        def_irq_handler DMA1_Channel4_5_6_7_IRQHandler
        def_irq_handler ADC1_COMP_IRQHandler
        def_irq_handler LPTIM1_IRQHandler
        def_irq_handler USART4_5_IRQHandler
        def_irq_handler TIM2_IRQHandler
        def_irq_handler TIM3_IRQHandler
        def_irq_handler TIM6_DAC_IRQHandler
        def_irq_handler TIM7_IRQHandler
        def_irq_handler TIM21_IRQHandler
        def_irq_handler I2C3_IRQHandler
        def_irq_handler TIM22_IRQHandler
        def_irq_handler I2C1_IRQHandler
        def_irq_handler I2C2_IRQHandler
        def_irq_handler SPI1_IRQHandler
        def_irq_handler SPI2_IRQHandler
        def_irq_handler USART1_IRQHandler
        def_irq_handler USART2_IRQHandler
        def_irq_handler RNG_LPUART1_IRQHandler
        def_irq_handler USB_IRQHandler
