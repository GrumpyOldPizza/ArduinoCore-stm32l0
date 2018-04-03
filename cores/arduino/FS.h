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

#ifndef FS_H
#define FS_H

#include <Arduino.h>

class File;
class Dir;
class DirEntry;
class FileSystem;

class FileHandle;
class DirHandle;

enum SeekMode {
    SeekSet = 0,
    SeekCur = 1,
    SeekEnd = 2
};

class File : public Stream
{
public:
    File(const File &file);
    File();
    ~File();

    File &operator=(const File &file);

    operator bool() const;

    // Print methods:
    int availableForWrite() override;
    size_t write(uint8_t) override;
    size_t write(const uint8_t *data, size_t size) override;
    void flush() override;

    // Stream methods:
    int available() override;
    int read() override;
    size_t read(uint8_t* data, size_t size) override;
    int peek() override;

    // File methods
    bool seek(uint32_t position, SeekMode mode = SeekSet);
    size_t position();
    size_t size();
    void close();

    using Print::write;

private:
    File(FileHandle *handle);

    FileHandle *_handle;

    friend class FileHandle;
};


class Dir 
{
public:
    Dir(const Dir &dir);
    Dir();
    ~Dir();

    Dir &operator=(const Dir &dir);

    operator bool() const;

    bool read(DirEntry &entry);
    bool rewind();
    void close();

private:
    Dir(DirHandle *handle);

    DirHandle *_handle;

    friend class DirHandle;
};


class DirEntry 
{
public:
    enum {
	DT_UNKNOWN = 0,
	DT_DIR,
	DT_REG,
    };

    DirEntry() { _info.type = DT_UNKNOWN; }

    const char *name() { return (_info.type == DT_UNKNOWN) ? NULL : _info.name; }
    size_t fileSize() { return (_info.type == DT_UNKNOWN) ? 0 : _info.size; };
    bool isDirectory() { return _info.type == DT_DIR; }
    bool isFile() { return _info.type == DT_REG; }

    String fileName() { return String(name()); }

    struct _DirInfo {
	uint8_t type;
	uint32_t size;
	char name[256];
    } _info;
};

class FileSystem
{
public:
    virtual File open(const char *path, const char *mode) = 0;
    virtual Dir openDir(const char *path) = 0;
    virtual bool exists(const char* path) = 0;
    virtual bool remove(const char *path) = 0;
    virtual bool mkdir(const char *path) = 0;
    virtual bool rmdir(const char *path) = 0;

    File open(const String &path, const char *mode) { return open(path.c_str(), mode); }
    Dir openDir(const String &path) { return openDir(path.c_str()); }
    bool exists(const String& path) { return exists(path.c_str()); };
    bool remove(const String &path) { return remove(path.c_str()); }
    bool mkdir(const String &path) { return mkdir(path.c_str()); }
    bool rmdir(const String &path) { return rmdir(path.c_str()); }
};


class FileHandle
{
public:
    FileHandle() : _refcount(1) { };
    virtual ~FileHandle() { };

    void reference();
    void unreference();

    File open() { return File(this); }
    virtual void close() = 0;

    virtual size_t read(uint8_t *data, size_t size) = 0;
    virtual size_t write(const uint8_t *data, size_t size) = 0;
    virtual void flush() = 0;
    virtual bool seek(uint32_t position, SeekMode mode) = 0;
    virtual size_t position() = 0;
    virtual size_t size() = 0;

private:
    volatile uint32_t _refcount;
};

class DirHandle
{
public:
    DirHandle() : _refcount(1) { };
    virtual ~DirHandle() { };

    void reference();
    void unreference();

    Dir open() { return Dir(this); }
    virtual void close() = 0;

    virtual bool read(struct DirEntry::_DirInfo *info) = 0;
    virtual bool rewind() = 0;

private:
    volatile uint32_t _refcount;
};

#endif // FS_H
