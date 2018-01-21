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

#ifndef DOSFS_H
#define DOSFS_H

#include <Arduino.h>
#include <FS.h>
#include <dosfs_api.h>

class DOSFSFile;
class DSFSDir;

class DOSFSFile : public File
{
public:
    DOSFSFile(const char* path, const char* mode);
    DOSFSFile();
  
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

private:
    F_FILE *_file;
};

class DOSFSDir : public Dir
{
public:
    DOSFSDir(const char* path);
    DOSFSDir();

    virtual String fileName();
    virtual size_t fileSize();
    virtual bool isDirectory();
    virtual bool next();
    virtual bool rewind();

    virtual operator bool() const;

private:
    F_DIR _dir;
    F_DIRENT _dirent;
};

typedef struct {
    size_t totalBytes;
    size_t usedBytes;
    size_t blockSize;
    size_t clusterSize;
    size_t maxOpenFiles;
    size_t maxPathLength;
} DOSFSInfo;

class DOSFSVolume : public FS
{
public:
    int begin();
    void end();

    bool check();
    bool format();
    bool info(DOSFSInfo& info);

    virtual File open(const char* path, const char* mode);
    virtual File open(const String& path, const char* mode);

    virtual Dir openDir(const char* path);
    virtual Dir openDir(const String& path);

    virtual bool exists(const char* path);
    virtual bool exists(const String& path);

    virtual bool mkdir(const char* path);
    virtual bool mkdir(const String& path);

    virtual bool rmdir(const char* path);
    virtual bool rmdir(const String& path);

    virtual bool chdir(const char* path);
    virtual bool chdir(const String& path);

    virtual bool remove(const char* path);
    virtual bool remove(const String& path);

    virtual bool rename(const char* pathFrom, const char* pathTo);
    virtual bool rename(const String& pathFrom, const String& pathTo);

};

extern DOSFSVolume DOSFS;

#endif // DOSFS_H
