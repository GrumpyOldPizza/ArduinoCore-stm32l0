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

#include <Arduino.h>
#include "DOSFS.h"
#include "dosfs_api.h"

class DOSFSFileHandle final : public FileHandle
{
public:
    DOSFSFileHandle() { };
    ~DOSFSFileHandle() override { };
    
    File open(const char *path, const char *mode);
    void close() override;

    size_t read(uint8_t *data, size_t size) override;
    size_t write(const uint8_t *data, size_t size) override;
    void flush() override;
    bool seek(uint32_t position, SeekMode mode) override;
    size_t position() override;
    size_t size() override;

private:
    F_FILE _file;
};

File DOSFSFileHandle::open(const char *path, const char *mode) {
    if (f_open(&_file, path, mode) != F_NO_ERROR) {
	delete this;
	
	return File();
    }

    return FileHandle::open();
}

void DOSFSFileHandle::close() {
    f_close(&_file);
}

size_t DOSFSFileHandle::read(uint8_t *data, size_t size) {
    return f_read(data, 1, size, &_file);
}

size_t DOSFSFileHandle::write(const uint8_t *data, size_t size) {
    return f_write(data, 1, size, &_file);
}

void DOSFSFileHandle::flush() {
    f_flush(&_file);
}

bool DOSFSFileHandle::seek(uint32_t position, SeekMode mode) {
    return f_seek(&_file, position, mode);
}

size_t DOSFSFileHandle::position() {
    return f_tell(&_file);
}

size_t DOSFSFileHandle::size() {
    return f_length(&_file);
}


class DOSFSDirHandle final : public DirHandle
{
public:
    DOSFSDirHandle() { };
    ~DOSFSDirHandle() override { };

    Dir open(const char *path);
    void close() override;

    bool read(struct DirEntry::_DirInfo *info) override;
    bool rewind() override;

private:
    F_DIR _dir;
};

Dir DOSFSDirHandle::open(const char *path) {
    if (f_opendir(&_dir, path) != F_NO_ERROR) {
	delete this;
	
	return Dir();
    }

    return DirHandle::open();
}

void DOSFSDirHandle::close() {
}

bool DOSFSDirHandle::read(struct DirEntry::_DirInfo *info) {
    if (f_readdir(&_dir, (F_DIRENT*)info) == F_NO_ERROR)
    {
	info->type = ((((F_DIRENT*)info)->attr & F_ATTR_DIR) == F_ATTR_DIR) ? DirEntry::DT_DIR : DirEntry::DT_REG;

	return true;
    }

    return false;
}

bool DOSFSDirHandle::rewind() {
    return f_rewinddir(&_dir);
}


int DOSFSFileSystem::begin() {
    return (f_initvolume() == F_NO_ERROR);
}

void DOSFSFileSystem::end() {
}

bool DOSFSFileSystem::check() {
    return (f_checkvolume() == F_NO_ERROR);
}

bool DOSFSFileSystem::format()
{
    return (f_hardformat(0) == F_NO_ERROR);
}

File DOSFSFileSystem::open(const char *path, const char *mode) {
    DOSFSFileHandle *handle;

    handle = new DOSFSFileHandle();

    if (handle) {
	return handle->open(path, mode);
    }

    return File();
}

Dir DOSFSFileSystem::openDir(const char *path) {
    DOSFSDirHandle *handle = new DOSFSDirHandle();

    if (handle) {
	return handle->open(path);
    }

    return Dir();
}

bool DOSFSFileSystem::exists(const char *path) {
    unsigned char attr;

    return (f_getattr(path, &attr) == F_NO_ERROR);
}

bool DOSFSFileSystem::remove(const char *path) {
    return (f_delete(path) == F_NO_ERROR);
}

bool DOSFSFileSystem::mkdir(const char *path) {
    return (f_mkdir(path) == F_NO_ERROR);
}

bool DOSFSFileSystem::rmdir(const char *path) {
    return (f_rmdir(path) == F_NO_ERROR);
}

DOSFSFileSystem DOSFS;
