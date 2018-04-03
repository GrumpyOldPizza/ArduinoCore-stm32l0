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
#include "FS.h"

File::File(const File &file) {
    _handle = file._handle;

    if (_handle) {
	_handle->reference();
    }
}

File::File(FileHandle *handle) {
    _handle = handle;
}

File::File() {
    _handle = NULL;
}

File::~File() {
    if (_handle) {
	_handle->unreference();
    }
}

File & File::operator=(const File &file) {
    FileHandle *handle = _handle;

    _handle = file._handle;

    if (_handle) {
	_handle->reference();
    }

    if (handle) {
	handle->unreference();
    }

    return *this;
}

File::operator bool() const {
    return _handle != NULL;
}

int File::availableForWrite() {
    return 0;
}

size_t File::write(uint8_t data) {
    if (_handle) {
	return _handle->write(&data, 1);
    }
    
    return 0;
}

size_t File::write(const uint8_t *data, size_t size) {
    if (_handle) {
	return _handle->write(data, size);
    }

    return 0;
}

void File::flush() {
    if (_handle) {
	_handle->flush();
    }
}

int File::available() {
    if (_handle) {
	return _handle->size() - _handle->position();
    }

    return 0;
}

int File::read() {
    size_t size;
    uint8_t data;

    if (_handle) {
	size = _handle->read(&data, 1);
	
	return (size ? data : -1);
    }

    return -1;
}

size_t File::read(uint8_t* data, size_t size) {
    if (_handle) {
	return _handle->read(data, size);
    }

    return 0;
}

int File::peek() {
    size_t position, size;
    uint8_t data;

    if (_handle) {
	position = _handle->position();
	size = _handle->read(&data, 1);
	_handle->seek(position, SeekSet);
	
	return (size ? data : -1);
    }

    return -1;
}

bool File::seek(uint32_t position, SeekMode mode) {
    if (_handle) {
	return _handle->seek(position, mode);
    }

    return false;
}

size_t File::position() {
    if (_handle) {
	return _handle->position();
    }

    return 0;
}

size_t File::size() {
    if (_handle) {
	return _handle->size();
    }

    return 0;
}

void File::close() {
    _handle->unreference();

    _handle = NULL;
}

Dir::Dir(const Dir &dir) {
    _handle = dir._handle;

    if (_handle) {
	_handle->reference();
    }
}

Dir::Dir(DirHandle *handle) {
    _handle = handle;
}

Dir::Dir() {
    _handle = NULL;
}

Dir::~Dir() {
    if (_handle) {
	_handle->unreference();
    }
}

Dir & Dir::operator=(const Dir &dir) {
    DirHandle *handle = _handle;

    _handle = dir._handle;

    if (_handle) {
	_handle->reference();
    }

    if (handle) {
	handle->unreference();
    }

    return *this;
}

Dir::operator bool() const {
    return _handle != NULL;
}

bool Dir::read(DirEntry &entry) {
    if (_handle) {
	return _handle->read(&entry._info);
    }
    
    return NULL;
}

bool Dir::rewind() {
    if (_handle) {
	return _handle->rewind();
    }
    
    return false;
}

void Dir::close() {
    if (_handle) {
	_handle->unreference();
    }

    _handle = NULL;
}

void FileHandle::reference() {
    _refcount++;
}

void FileHandle::unreference() {
    if (--_refcount == 0) {
	close();

	delete this;
    }	
}

void DirHandle::reference() {
    _refcount++;
}

void DirHandle::unreference() {
    if (--_refcount == 0) {
	close();

	delete this;
    }	
}
