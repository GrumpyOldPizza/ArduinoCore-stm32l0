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

enum SeekMode {
    SeekSet = 0,
    SeekCur = 1,
    SeekEnd = 2
};

class File : public Stream
{
public:
    // Print methods:
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *buf, size_t size);
    virtual void flush();

    // Stream methods:
    virtual int available();
    virtual int read();
    virtual size_t read(uint8_t* buf, size_t size);
    virtual int peek();

    // File methods
    virtual bool seek(uint32_t pos, SeekMode mode = SeekSet);
    virtual size_t position();
    virtual size_t size();
    virtual void close();

    virtual operator bool() const;
};

class Dir 
{
public:
    virtual String fileName();
    virtual size_t fileSize();
    virtual bool isDirectory();
    virtual bool next();
    virtual bool rewind();

    virtual operator bool() const;
};

class FS
{
public:
    virtual File open(const char* path, const char* mode);
    virtual File open(const String& path, const char* mode);

    virtual Dir openDir(const char* path);
    virtual Dir openDir(const String& path);

    virtual bool exists(const char* path) = 0;
    virtual bool exists(const String& path) = 0;

    virtual bool mkdir(const char* path) = 0;
    virtual bool mkdir(const String& path) = 0;

    virtual bool rmdir(const char* path) = 0;
    virtual bool rmdir(const String& path) = 0;

    virtual bool chdir(const char* path) = 0;
    virtual bool chdir(const String& path) = 0;

    virtual bool remove(const char* path) = 0;
    virtual bool remove(const String& path) = 0;

    virtual bool rename(const char* pathFrom, const char* pathTo) = 0;
    virtual bool rename(const String& pathFrom, const String& pathTo) = 0;

};

#endif // FS_H
