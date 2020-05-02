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

#include "armv6m.h"
#include "stm32l0xx.h"

#include "stm32l0_system.h"
#include "stm32l0_eeprom.h"

#define DATA_EEPROM_SIZE 6144

#define STM32L0_EEPROM_STATE_NONE  0
#define STM32L0_EEPROM_STATE_READY 1
#define STM32L0_EEPROM_STATE_BUSY  2

typedef struct _stm32l0_eeprom_device_t {
    volatile uint32_t                       state;
    stm32l0_eeprom_transaction_t * volatile queue;
    stm32l0_eeprom_done_callback_t          callback;
    void                                    *context;
    volatile uint8_t                        *status;
    uint32_t                                address;
    uint8_t                                 *data;
    uint32_t                                count;
} stm32l0_eeprom_device_t;

static stm32l0_eeprom_device_t stm32l0_eeprom_device;

void __stm32l0_eeprom_initialize(void)
{
    NVIC_SetPriority(FLASH_IRQn, ((1 << __NVIC_PRIO_BITS) -1) -1);
    
    NVIC_EnableIRQ(FLASH_IRQn);
    
    stm32l0_eeprom_device.queue = NULL;
    stm32l0_eeprom_device.address = 0;
    stm32l0_eeprom_device.state = STM32L0_EEPROM_STATE_READY;
}

void stm32l0_eeprom_acquire(void)
{
    stm32l0_eeprom_device.state = STM32L0_EEPROM_STATE_BUSY;

    while (stm32l0_eeprom_device.address)
    {
    }
}

void stm32l0_eeprom_release(void)
{
    stm32l0_eeprom_device.state = STM32L0_EEPROM_STATE_READY;

    NVIC_SetPendingIRQ(FLASH_IRQn);
}

bool stm32l0_eeprom_enqueue(stm32l0_eeprom_transaction_t *transaction)
{
    stm32l0_eeprom_transaction_t *queue;

    if (stm32l0_eeprom_device.state == STM32L0_EEPROM_STATE_NONE)
    {
        return false;
    }

    if (transaction->control == STM32L0_EEPROM_CONTROL_ERASE)
    {
        if ((transaction->address & 3) ||
            (transaction->count & 3) ||
            ((transaction->address + transaction->count) > DATA_EEPROM_SIZE))
        {
            return false;
        }
    }
    else
    {
        if ((transaction->address + transaction->count) > DATA_EEPROM_SIZE)
        {
            return false;
        }
    }

    transaction->status = STM32L0_EEPROM_STATUS_BUSY;

    do
    {
        queue = stm32l0_eeprom_device.queue;
        transaction->next = queue;
    }
    while (armv6m_atomic_compare_and_swap((volatile uint32_t*)&stm32l0_eeprom_device.queue, (uint32_t)queue, (uint32_t)transaction) != (uint32_t)queue);

    NVIC_SetPendingIRQ(FLASH_IRQn);

    return true;
}

static bool stm32l0_eeprom_do_erase(void)
{
    uint32_t address;

    do
    {
        address = stm32l0_eeprom_device.address;

        stm32l0_eeprom_device.address += 4;
        stm32l0_eeprom_device.count   -= 4;

        if (*((volatile uint32_t*)(address)) != 0x00000000)
        {
            *((volatile uint32_t*)(address)) = 0x00000000;

            return true;
        }
        
    }
    while (stm32l0_eeprom_device.count);

    return false;
}

static bool stm32l0_eeprom_do_program(void)
{
    uint32_t address, offset, size, data;

    do
    {
        address = stm32l0_eeprom_device.address & ~3;
        offset  = stm32l0_eeprom_device.address & 3;
        
        data = *((volatile uint32_t*)address);
        
        size = 4 - offset;
        
        if (size > stm32l0_eeprom_device.count)
        {
            size = stm32l0_eeprom_device.count;
        }
        
        if (size >= 1)
        {
            ((uint8_t*)&data + offset)[0] = stm32l0_eeprom_device.data[0];
        }
        
        if (size >= 2)
        {
            ((uint8_t*)&data + offset)[1] = stm32l0_eeprom_device.data[1];
        }
        
        if (size >= 3)
        {
            ((uint8_t*)&data + offset)[2] = stm32l0_eeprom_device.data[2];
        }
        
        if (size >= 4)
        {
            ((uint8_t*)&data + offset)[3] = stm32l0_eeprom_device.data[3];
        }

        stm32l0_eeprom_device.address += size;
        stm32l0_eeprom_device.data    += size;
        stm32l0_eeprom_device.count   -= size;
        
        if (*((volatile uint32_t*)address) != data)
        {
            *((volatile uint32_t*)address) = data;

            return true;
        }
    }
    while (stm32l0_eeprom_device.count);

    return false;
}

void FLASH_IRQHandler(void)
{
    stm32l0_eeprom_transaction_t *queue, *transaction, **pp_transaction;;
    uint32_t primask;

    if (stm32l0_eeprom_device.address != 0x00000000)
    {
        if (FLASH->SR & (FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_RDERR | FLASH_SR_NOTZEROERR | FLASH_SR_FWWERR | FLASH_SR_EOP))
        {
            if (FLASH->SR & (FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_RDERR | FLASH_SR_NOTZEROERR | FLASH_SR_FWWERR))
            {
                FLASH->SR = (FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_RDERR | FLASH_SR_NOTZEROERR | FLASH_SR_FWWERR);
                
                FLASH->PECR &= ~(FLASH_PECR_EOPIE | FLASH_PECR_ERRIE | FLASH_PECR_DATA);

                FLASH->PECR |= FLASH_PECR_PELOCK;

                stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_CLOCKS);
                stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_STOP);

                stm32l0_eeprom_device.address = 0x00000000;
                
                *stm32l0_eeprom_device.status = STM32L0_EEPROM_STATUS_FAIL;
                
                if (stm32l0_eeprom_device.callback)
                {
                    (*stm32l0_eeprom_device.callback)(stm32l0_eeprom_device.context);
                }
            }
            else
            {
                FLASH->SR = FLASH_SR_EOP;
            
                if (stm32l0_eeprom_device.count)
                {
                    if (stm32l0_eeprom_device.data == NULL)
                    {
                        if (stm32l0_eeprom_do_erase())
                        {
                            return;
                        }
                    }
                    else
                    {
                        if (stm32l0_eeprom_do_program())
                        {
                            return;
                        }
                    }
                }

                FLASH->PECR &= ~(FLASH_PECR_EOPIE | FLASH_PECR_ERRIE | FLASH_PECR_DATA);

                FLASH->PECR |= FLASH_PECR_PELOCK;

                stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_CLOCKS);
                stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_STOP);
                
                stm32l0_eeprom_device.address = 0x00000000;
                
                *stm32l0_eeprom_device.status = STM32L0_EEPROM_STATUS_SUCCESS;
                
                if (stm32l0_eeprom_device.callback)
                {
                    (*stm32l0_eeprom_device.callback)(stm32l0_eeprom_device.context);
                }
            }
        }
    }

    if (stm32l0_eeprom_device.state == STM32L0_EEPROM_STATE_READY)
    {
        while (stm32l0_eeprom_device.queue)
        {
            transaction = NULL;
        
            do
            {
                queue = stm32l0_eeprom_device.queue;
            
                for (pp_transaction = NULL, transaction = queue; transaction->next; pp_transaction = &transaction->next, transaction = transaction->next) 
                {
                }
            
                if (pp_transaction)
                {
                    *pp_transaction = NULL;
                
                    break;
                }
            }
            while (armv6m_atomic_compare_and_swap((volatile uint32_t*)&stm32l0_eeprom_device.queue, (uint32_t)transaction, (uint32_t)NULL) != (uint32_t)queue);
        
            if ((transaction->control == STM32L0_EEPROM_CONTROL_ERASE) || (transaction->control == STM32L0_EEPROM_CONTROL_PROGRAM))
            {
                stm32l0_system_lock(STM32L0_SYSTEM_LOCK_STOP);
                stm32l0_system_lock(STM32L0_SYSTEM_LOCK_CLOCKS);

                primask = __get_PRIMASK();

                __disable_irq();

                FLASH->PEKEYR  = 0x89abcdef;
                FLASH->PEKEYR  = 0x02030405;
            
                __set_PRIMASK(primask);

                if (FLASH->PECR & FLASH_PECR_PELOCK)
                {
                    transaction->status = STM32L0_EEPROM_STATUS_FAIL;
                
                    if (transaction->callback)
                    {
                        (*transaction->callback)(transaction->context);
                    }
                }
                else
                {
                    stm32l0_eeprom_device.address = DATA_EEPROM_BASE + transaction->address;
                    stm32l0_eeprom_device.count = transaction->count;
                    stm32l0_eeprom_device.status = &transaction->status;
                    stm32l0_eeprom_device.callback = transaction->callback;
                    stm32l0_eeprom_device.context = transaction->context;

                    FLASH->PECR |= (FLASH_PECR_EOPIE | FLASH_PECR_ERRIE | FLASH_PECR_DATA);
                
                    if (transaction->control == STM32L0_EEPROM_CONTROL_ERASE)
                    {
                        stm32l0_eeprom_device.data = NULL;

                        if (stm32l0_eeprom_do_erase())
                        {
                            return;
                        }
                    }
                    else
                    {
                        stm32l0_eeprom_device.data = transaction->data;
                    
                        if (stm32l0_eeprom_do_program())
                        {
                            return;
                        }
                    }

                    FLASH->PECR &= ~(FLASH_PECR_EOPIE | FLASH_PECR_ERRIE | FLASH_PECR_DATA);
                        
                    FLASH->PECR |= FLASH_PECR_PELOCK;
                    
                    stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_CLOCKS);
                    stm32l0_system_unlock(STM32L0_SYSTEM_LOCK_STOP);
                    
                    stm32l0_eeprom_device.address = 0x00000000;
                    
                    *stm32l0_eeprom_device.status = STM32L0_EEPROM_STATUS_SUCCESS;
                    
                    if (stm32l0_eeprom_device.callback)
                    {
                        (*stm32l0_eeprom_device.callback)(stm32l0_eeprom_device.context);
                    }
                }
            }
            
            if (transaction->control == STM32L0_EEPROM_CONTROL_READ)
            {
                memcpy(transaction->data, (const void*)(DATA_EEPROM_BASE + transaction->address), transaction->count);

                transaction->status = STM32L0_EEPROM_STATUS_SUCCESS;
                
                if (transaction->callback)
                {
                    (*transaction->callback)(transaction->context);
                }
            }
        }
    }
}
