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

#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>

#include "armv6m.h"

int (*stm32l0_stdio_put)(char, FILE*) = NULL;
int (*stm32l0_stdio_get)(FILE*) = NULL;

#undef errno
extern int errno;

extern uint32_t __end__[];
extern uint32_t __HeapLimit[];

void * _sbrk (int nbytes)
{
    void *p;

    static void *__HeapCurrent = (void*)(&__end__[0]);

    if (((uint8_t*)__HeapCurrent + nbytes) <= (uint8_t*)(&__HeapLimit[0]))
    {
	p = __HeapCurrent;
	
	__HeapCurrent = (void*)((uint8_t*)__HeapCurrent + nbytes);

	return p;
    }
    else
    {
	errno = ENOMEM;

	return  (void *) -1;
    }
}

int _getpid(void)
{
    return 1;
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;

    errno = EINVAL;

    return -1;
}

int _close(int file) {
    (void)file;

    return -1;
}

int _isatty(int file) 
{
    (void)file;

    switch (file) {
    case STDOUT_FILENO:
    case STDERR_FILENO:
    case STDIN_FILENO:
        return 1;

    default:
	errno = EBADF;
	return 0;
    }
}

int _fstat(int file, struct stat *st)
{
    (void)file;

    st->st_mode = S_IFCHR;

    return 0;
}

int _lseek(int file, int offset, int whence)
{
    (void)file;
    (void)offset;
    (void)whence;

    return 0;
}

int _read(int file, char *buf, int nbytes)
{
    int c, n;

    switch (file) {
    case STDIN_FILENO:
	n = 0;

	if (nbytes != 0)
	{
	    if (stm32l0_stdio_get != NULL)
	    {
		do
		{
		    c = (*stm32l0_stdio_get)(stdin);
		    
		    if (c == -1)
		    {
			break;
		    }

		    buf[n++] = c;
		    nbytes--;
		}
		while (nbytes != 0);
	    }
	}
	return n;

    default:
        errno = EBADF;
        return -1;
    }
}

int _write(int file, const char *buf, int nbytes)
{
    int n;

    switch (file) {
    case STDOUT_FILENO:
    case STDERR_FILENO:
	n = 0;

	if (nbytes != 0)
	{
	    if (stm32l0_stdio_put != NULL)
	    {
		do
		{
		    if (!(*stm32l0_stdio_put)(buf[n], stdout))
		    {
			break;
		    }

		    n++;
		    nbytes--;
		}
		while (nbytes != 0);
	    }
	}
	return n;

    default:
	errno = EBADF;
	return -1;
    }
}

void _exit(int status) 
{
    (void)status;

    while (1) { };
}

void *malloc(size_t nbytes)
{
    if (__get_IPSR() == 0)
    {
        return (void*)armv6m_svcall_2((uint32_t)&_malloc_r, (uint32_t)_REENT, (uint32_t)nbytes);
    }
    else
    {
	return _malloc_r (_REENT, nbytes);
    }
}

void free(void *aptr)
{
    if (__get_IPSR() == 0)
    {
        armv6m_svcall_2((uint32_t)&_free_r, (uint32_t)_REENT, (uint32_t)aptr);
    }
    else
    {
	return _free_r (_REENT, aptr);
    }
}

void *realloc(void *aptr, size_t nbytes)
{
    if (__get_IPSR() == 0)
    {
	return (void*)armv6m_svcall_3((uint32_t)&_realloc_r, (uint32_t)_REENT, (uint32_t)aptr, (uint32_t)nbytes);
    }
    else
    {
	return _realloc_r (_REENT, aptr, nbytes);
    }
}

void *reallocf(void *aptr, size_t nbytes)
{
    void *nptr;

    nptr = realloc(aptr, nbytes);
    if (!nptr && aptr) {
	free(aptr);
    }
    return (nptr);
}

void *memalign(size_t align, size_t nbytes)
{
    if (__get_IPSR() == 0)
    {
        return (void*)armv6m_svcall_3((uint32_t)&_memalign_r, (uint32_t)_REENT, (uint32_t)align, (uint32_t)nbytes);
    }
    else
    {
	return _memalign_r (_REENT, align, nbytes);
    }
}

size_t malloc_usable_size(void *aptr)
{
    if (__get_IPSR() == 0)
    {
        return armv6m_svcall_2((uint32_t)&_malloc_usable_size_r, (uint32_t)_REENT, (uint32_t)aptr);
    }
    else
    {
	return _malloc_usable_size_r (_REENT, aptr);
    }
}
