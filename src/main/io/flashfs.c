/* 
 * This file is part of RaceFlight. 
 * 
 * RaceFlight is free software: you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, or 
 * (at your option) any later version. 
 * 
 * RaceFlight is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 * 
 * You should have received a copy of the GNU General Public License 
 * along with RaceFlight.  If not, see <http://www.gnu.org/licenses/>.
 * You should have received a copy of the GNU General Public License 
 * along with RaceFlight.  If not, see <http://www.gnu.org/licenses/>.
 */ 
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "drivers/flash_m25p16.h"
#include "flashfs.h"
#include "target.h"
#ifdef USE_FLASHFS
static uint8_t flashWriteBuffer[FLASHFS_WRITE_BUFFER_SIZE];
static uint8_t bufferHead = 0, bufferTail = 0;
static uint32_t tailAddress = 0;
static void flashfsClearBuffer()
{
    bufferTail = bufferHead = 0;
}
static bool flashfsBufferIsEmpty()
{
    return bufferTail == bufferHead;
}
static void flashfsSetTailAddress(uint32_t address)
{
    tailAddress = address;
}
void flashfsEraseCompletely()
{
    m25p16_eraseCompletely();
    flashfsClearBuffer();
    flashfsSetTailAddress(0);
}
void flashfsEraseRange(uint32_t start, uint32_t end)
{
    const flashGeometry_t *geometry = m25p16_getGeometry();
    if (geometry->sectorSize <= 0)
        return;
    int startSector = start / geometry->sectorSize;
    int endSector = end / geometry->sectorSize;
    int endRemainder = end % geometry->sectorSize;
    if (endRemainder > 0) {
        endSector++;
    }
    for (int i = startSector; i < endSector; i++) {
        m25p16_eraseSector(i * geometry->sectorSize);
    }
}
bool flashfsIsReady()
{
    return m25p16_isReady();
}
uint32_t flashfsGetSize()
{
    return m25p16_getGeometry()->totalSize;
}
static uint32_t flashfsTransmitBufferUsed()
{
    if (bufferHead >= bufferTail)
        return bufferHead - bufferTail;
    return FLASHFS_WRITE_BUFFER_SIZE - bufferTail + bufferHead;
}
uint32_t flashfsGetWriteBufferSize()
{
    return FLASHFS_WRITE_BUFFER_USABLE;
}
uint32_t flashfsGetWriteBufferFreeSpace()
{
    return flashfsGetWriteBufferSize() - flashfsTransmitBufferUsed();
}
const flashGeometry_t* flashfsGetGeometry()
{
    return m25p16_getGeometry();
}
static uint32_t flashfsWriteBuffers(uint8_t const **buffers, uint32_t *bufferSizes, int bufferCount, bool sync)
{
    uint32_t bytesTotal = 0;
    int i;
    for (i = 0; i < bufferCount; i++) {
        bytesTotal += bufferSizes[i];
    }
    if (!sync && !m25p16_isReady()) {
        return 0;
    }
    uint32_t bytesTotalRemaining = bytesTotal;
    while (bytesTotalRemaining > 0) {
        uint32_t bytesTotalThisIteration;
        uint32_t bytesRemainThisIteration;
        if (tailAddress % M25P16_PAGESIZE + bytesTotalRemaining > M25P16_PAGESIZE) {
            bytesTotalThisIteration = M25P16_PAGESIZE - tailAddress % M25P16_PAGESIZE;
        } else {
            bytesTotalThisIteration = bytesTotalRemaining;
        }
        if (flashfsIsEOF()) {
            flashfsClearBuffer();
            break;
        }
        m25p16_pageProgramBegin(tailAddress);
        bytesRemainThisIteration = bytesTotalThisIteration;
        for (i = 0; i < bufferCount; i++) {
            if (bufferSizes[i] > 0) {
                if (bufferSizes[i] >= bytesRemainThisIteration) {
                    m25p16_pageProgramContinue(buffers[i], bytesRemainThisIteration);
                    buffers[i] += bytesRemainThisIteration;
                    bufferSizes[i] -= bytesRemainThisIteration;
                    bytesRemainThisIteration = 0;
                    break;
                } else {
                    m25p16_pageProgramContinue(buffers[i], bufferSizes[i]);
                    bytesRemainThisIteration -= bufferSizes[i];
                    buffers[i] += bufferSizes[i];
                    bufferSizes[i] = 0;
                }
            }
        }
        m25p16_pageProgramFinish();
        bytesTotalRemaining -= bytesTotalThisIteration;
        flashfsSetTailAddress(tailAddress + bytesTotalThisIteration);
        if (!sync)
            break;
    }
    return bytesTotal - bytesTotalRemaining;
}
static void flashfsGetDirtyDataBuffers(uint8_t const *buffers[], uint32_t bufferSizes[])
{
    buffers[0] = flashWriteBuffer + bufferTail;
    buffers[1] = flashWriteBuffer + 0;
    if (bufferHead >= bufferTail) {
        bufferSizes[0] = bufferHead - bufferTail;
        bufferSizes[1] = 0;
    } else {
        bufferSizes[0] = FLASHFS_WRITE_BUFFER_SIZE - bufferTail;
        bufferSizes[1] = bufferHead;
    }
}
uint32_t flashfsGetOffset()
{
    uint8_t const * buffers[2];
    uint32_t bufferSizes[2];
    flashfsGetDirtyDataBuffers(buffers, bufferSizes);
    return tailAddress + bufferSizes[0] + bufferSizes[1];
}
static void flashfsAdvanceTailInBuffer(uint32_t delta)
{
    bufferTail += delta;
    if (bufferTail >= FLASHFS_WRITE_BUFFER_SIZE) {
        bufferTail -= FLASHFS_WRITE_BUFFER_SIZE;
    }
    if (flashfsBufferIsEmpty()) {
        flashfsClearBuffer();
    }
}
bool flashfsFlushAsync()
{
    if (flashfsBufferIsEmpty()) {
        return true;
    }
    uint8_t const * buffers[2];
    uint32_t bufferSizes[2];
    uint32_t bytesWritten;
    flashfsGetDirtyDataBuffers(buffers, bufferSizes);
    bytesWritten = flashfsWriteBuffers(buffers, bufferSizes, 2, false);
    flashfsAdvanceTailInBuffer(bytesWritten);
    return flashfsBufferIsEmpty();
}
void flashfsFlushSync()
{
    if (flashfsBufferIsEmpty()) {
        return;
    }
    uint8_t const * buffers[2];
    uint32_t bufferSizes[2];
    flashfsGetDirtyDataBuffers(buffers, bufferSizes);
    flashfsWriteBuffers(buffers, bufferSizes, 2, true);
    flashfsClearBuffer();
}
void flashfsSeekAbs(uint32_t offset)
{
    flashfsFlushSync();
    flashfsSetTailAddress(offset);
}
void flashfsSeekRel(int32_t offset)
{
    flashfsFlushSync();
    flashfsSetTailAddress(tailAddress + offset);
}
void flashfsWriteByte(uint8_t byte)
{
    flashWriteBuffer[bufferHead++] = byte;
    if (bufferHead >= FLASHFS_WRITE_BUFFER_SIZE) {
        bufferHead = 0;
    }
    if (flashfsTransmitBufferUsed() >= FLASHFS_WRITE_BUFFER_AUTO_FLUSH_LEN) {
        flashfsFlushAsync();
    }
}
void flashfsWrite(const uint8_t *data, unsigned int len, bool sync)
{
    uint8_t const * buffers[3];
    uint32_t bufferSizes[3];
    flashfsGetDirtyDataBuffers(buffers, bufferSizes);
    buffers[2] = data;
    bufferSizes[2] = len;
    if (bufferSizes[0] + bufferSizes[1] + bufferSizes[2] >= FLASHFS_WRITE_BUFFER_AUTO_FLUSH_LEN) {
        uint32_t bytesWritten;
        bytesWritten = flashfsWriteBuffers(buffers, bufferSizes, 3, false);
        if (bufferSizes[0] == 0 && bufferSizes[1] == 0) {
            flashfsClearBuffer();
            if (bufferSizes[2] == 0) {
                return;
            }
        } else {
            flashfsAdvanceTailInBuffer(bytesWritten);
        }
        if (bufferSizes[0] + bufferSizes[1] + bufferSizes[2] > FLASHFS_WRITE_BUFFER_USABLE) {
            if (sync) {
                flashfsWriteBuffers(buffers, bufferSizes, 3, true);
                flashfsClearBuffer();
            } else {
            }
            return;
        }
        data = buffers[2];
        len = bufferSizes[2];
    }
    unsigned int bufferBytesBeforeWrap = FLASHFS_WRITE_BUFFER_SIZE - bufferHead;
    unsigned int firstPortion = len < bufferBytesBeforeWrap ? len : bufferBytesBeforeWrap;
    memcpy(flashWriteBuffer + bufferHead, data, firstPortion);
    bufferHead += firstPortion;
    data += firstPortion;
    len -= firstPortion;
    if (bufferHead == FLASHFS_WRITE_BUFFER_SIZE) {
        memcpy(flashWriteBuffer + 0, data, len);
        bufferHead = len;
    }
}
int flashfsReadAbs(uint32_t address, uint8_t *buffer, unsigned int len)
{
    int bytesRead;
    if (address + len > flashfsGetSize()) {
        len = flashfsGetSize() - address;
    }
    flashfsFlushSync();
    bytesRead = m25p16_readBytes(address, buffer, len);
    return bytesRead;
}
int flashfsIdentifyStartOfFreeSpace()
{
    enum {
        FREE_BLOCK_SIZE = 2048,
        FREE_BLOCK_TEST_SIZE_INTS = 4,
        FREE_BLOCK_TEST_SIZE_BYTES = FREE_BLOCK_TEST_SIZE_INTS * sizeof(uint32_t),
    };
    union {
        uint8_t bytes[FREE_BLOCK_TEST_SIZE_BYTES];
        uint32_t ints[FREE_BLOCK_TEST_SIZE_INTS];
    } testBuffer;
    int left = 0;
    int right = flashfsGetSize() / FREE_BLOCK_SIZE;
    int mid;
    int result = right;
    int i;
    bool blockErased;
    while (left < right) {
        mid = (left + right) / 2;
        if (m25p16_readBytes(mid * FREE_BLOCK_SIZE, testBuffer.bytes, FREE_BLOCK_TEST_SIZE_BYTES) < FREE_BLOCK_TEST_SIZE_BYTES) {
            break;
        }
        blockErased = true;
        for (i = 0; i < FREE_BLOCK_TEST_SIZE_INTS; i++) {
            if (testBuffer.ints[i] != 0xFFFFFFFF) {
                blockErased = false;
                break;
            }
        }
        if (blockErased) {
            result = mid;
            right = mid;
        } else {
            left = mid + 1;
        }
    }
    return result * FREE_BLOCK_SIZE;
}
bool flashfsIsEOF() {
    return tailAddress >= flashfsGetSize();
}
void flashfsInit()
{
    if (flashfsGetSize() > 0) {
        flashfsSeekAbs(flashfsIdentifyStartOfFreeSpace());
    }
}
#endif
