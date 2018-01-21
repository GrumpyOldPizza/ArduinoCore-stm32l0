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

#include "DOSFS.h"
#include "dosfs_config.h"

DOSFSFile::DOSFSFile(const char* path, const char* mode) {
    _file = f_open(path, mode);
}

DOSFSFile::DOSFSFile() {
    _file = NULL;
}

size_t DOSFSFile::write(uint8_t data) {
    if (!_file) {
        return 0;
    }

    if (f_putc(data, _file) == -1) {
	return 0;
    }

    return 1;
}

size_t DOSFSFile::write(const uint8_t *buffer, size_t size) {
    if (!_file) {
        return 0;
    }

    return f_write(buffer, 1, size, _file);
}

int DOSFSFile::available() {
    if (!_file) {
        return 0;
    }

    return f_length(_file) - f_tell(_file);
}

int DOSFSFile::read() {
    if (!_file) {
        return -1;
    }

    return f_getc(_file);
}

size_t DOSFSFile::read(uint8_t* buf, size_t size) {
    if (!_file) {
        return -1;
    }
	
    return f_read(buf, 1, size, _file);
}

int DOSFSFile::peek() {
    long position;
    int c;

    if (!_file) {
        return -1;
    }

    position = f_tell(_file);
    c = f_getc(_file);
    f_seek(_file, position, F_SEEK_SET);
    return c;
}

void DOSFSFile::flush() {
    if (!_file) {
        return;
    }

    f_flush(_file);
}

bool DOSFSFile::seek(uint32_t pos, SeekMode mode) {
    if (!_file) {
        return false;
    }

    return (f_seek(_file, pos, mode) == F_NO_ERROR);
}

size_t DOSFSFile::position() {
    if (!_file) {
        return 0;
    }
	
    return f_tell(_file);
}

size_t DOSFSFile::size() {
    if (!_file) {
        return 0;
    }

    return f_length(_file);
}

void DOSFSFile::close() {
    if (!_file) {
	return;
    }

    f_close(_file);

    _file = NULL;
}

DOSFSFile::operator bool() const {
    return !!_file;
}


DOSFSDir::DOSFSDir(const char* path) {
    _dirent.cluster = 0xffffffff;

    if (f_opendir(&_dir, path) == F_NO_ERROR) {
	f_readdir(&_dir, &_dirent);
    }
}

DOSFSDir::DOSFSDir() {
    _dir.dot_clsno = 0xffffffff;
    _dir.dir_clsno = 0xffffffff;
    _dir.dir_index = 0;

    _dirent.cluster = 0xffffffff;
};

String DOSFSDir::fileName() {
    if (_dirent.cluster == 0x0fffffff) {
        return String();
    }

    return String(&_dirent.filename[0]);
}

size_t DOSFSDir::fileSize() {
    if (_dirent.cluster == 0x0fffffff) {
        return 0;
    }

    return _dirent.filesize;
}

bool DOSFSDir::isDirectory() {
    if (_dirent.cluster == 0x0fffffff) {
        return false;
    }

    return !!(_dirent.attr & F_ATTR_DIR);
}

bool DOSFSDir::next() {
    if (_dir.dir_clsno == 0x0fffffff) {
        return false;
    }

    return (f_readdir(&_dir, &_dirent) == F_NO_ERROR);
}

bool DOSFSDir::rewind() {
    if (_dir.dot_clsno == 0x0fffffff) {
        return false;
    }

    _dir.dir_clsno = _dir.dot_clsno;
    _dir.dir_index = 0;

    return (f_readdir(&_dir, &_dirent) == F_NO_ERROR);
}

int DOSFSVolume::begin()
{
    return (f_initvolume() == F_NO_ERROR);
}

void DOSFSVolume::end()
{
    f_delvolume();
}

bool DOSFSVolume::check()
{
    return (f_checkvolume() == F_NO_ERROR);
}

bool DOSFSVolume::format()
{
    return (f_hardformat(0) == F_NO_ERROR);
}

bool DOSFSVolume::info(DOSFSInfo& info)
{
    F_SPACE space;
    
    if (f_getfreespace(&space) != F_NO_ERROR) 
	return false;
    
    info.totalBytes    = (uint64_t)space.total | ((uint64_t)space.total_high << 32);
    info.usedBytes     = (uint64_t)space.used  | ((uint64_t)space.used_high << 32);
    info.blockSize     = 512;
    info.clusterSize   = 512;
    info.maxOpenFiles  = DOSFS_CONFIG_MAX_FILES;
    info.maxPathLength = F_MAXPATH;
    
    return true;
}

File DOSFSVolume::open(const char* path, const char* mode) {
    return DOSFSFile(path, mode);
}

File DOSFSVolume::open(const String& path, const char* mode) {
    return open(path.c_str(), mode);
}

bool DOSFSVolume::exists(const char* path) {
    unsigned char attr;

    return (f_getattr(path, &attr) == F_NO_ERROR);
}

bool DOSFSVolume::exists(const String& path) {
    return exists(path.c_str());
}

Dir DOSFSVolume::openDir(const char* path) {
    return DOSFSDir(path);
}

Dir DOSFSVolume::openDir(const String& path) {
    return openDir(path.c_str());
}

bool DOSFSVolume::mkdir(const char* path) {
    return (f_mkdir(path) == F_NO_ERROR);
}

bool DOSFSVolume::mkdir(const String& path) {
    return mkdir(path.c_str());
}

bool DOSFSVolume::rmdir(const char* path) {
    return (f_rmdir(path) == F_NO_ERROR);
}

bool DOSFSVolume::rmdir(const String& path) {
    return rmdir(path.c_str());
}

bool DOSFSVolume::chdir(const char* path) {
    return (f_chdir(path) == F_NO_ERROR);
}

bool DOSFSVolume::chdir(const String& path) {
    return chdir(path.c_str());
}

bool DOSFSVolume::rename(const char* pathFrom, const char* pathTo) {
    return (f_move(pathFrom, pathTo) == F_NO_ERROR);
}

bool DOSFSVolume::rename(const String& pathFrom, const String& pathTo) {
    return rename(pathFrom.c_str(), pathTo.c_str());
}

bool DOSFSVolume::remove(const char* path) {
    return (f_delete(path) == F_NO_ERROR);
}

bool DOSFSVolume::remove(const String& path) {
    return remove(path.c_str());
}

DOSFSVolume DOSFS;
