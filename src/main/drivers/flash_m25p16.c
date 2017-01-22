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
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "platform.h"
#ifdef USE_FLASH_M25P16
#include "flash_m25p16.h"
#include "bus_spi.h"
#include "system.h"
#define M25P16_INSTRUCTION_RDID 0x9F
#define M25P16_INSTRUCTION_READ_BYTES 0x03
#define M25P16_INSTRUCTION_READ_STATUS_REG 0x05
#define M25P16_INSTRUCTION_WRITE_STATUS_REG 0x01
#define M25P16_INSTRUCTION_WRITE_ENABLE 0x06
#define M25P16_INSTRUCTION_WRITE_DISABLE 0x04
#define M25P16_INSTRUCTION_PAGE_PROGRAM 0x02
#define M25P16_INSTRUCTION_SECTOR_ERASE 0xD8
#define M25P16_INSTRUCTION_BULK_ERASE 0xC7
#define M25P16_STATUS_FLAG_WRITE_IN_PROGRESS 0x01
#define M25P16_STATUS_FLAG_WRITE_ENABLED 0x02
#define JEDEC_ID_MICRON_M25P16 0x202015
#define JEDEC_ID_MICRON_N25Q064 0x20BA17
#define JEDEC_ID_WINBOND_W25Q64 0xEF4017
#define JEDEC_ID_MICRON_N25Q128 0x20ba18
#define JEDEC_ID_WINBOND_W25Q128 0xEF4018
#define DISABLE_M25P16 IOHi(flashSpim25p16CsPin)
#define ENABLE_M25P16 IOLo(flashSpim25p16CsPin)
#define DEFAULT_TIMEOUT_MILLIS 6
#define SECTOR_ERASE_TIMEOUT_MILLIS 5000
#define BULK_ERASE_TIMEOUT_MILLIS 21000
static flashGeometry_t geometry = {.pageSize = M25P16_PAGESIZE};
static IO_t flashSpim25p16CsPin = IO_NONE;
static bool couldBeBusy = false;
static void m25p16_performOneByteCommand(uint8_t command)
{
    ENABLE_M25P16;
    spiTransferByte(M25P16_SPI_INSTANCE, command);
    DISABLE_M25P16;
}
static void m25p16_writeEnable()
{
    m25p16_performOneByteCommand(M25P16_INSTRUCTION_WRITE_ENABLE);
    couldBeBusy = true;
}
static uint8_t m25p16_readStatus()
{
    uint8_t command[2] = {M25P16_INSTRUCTION_READ_STATUS_REG, 0};
    uint8_t in[2];
    ENABLE_M25P16;
    spiTransfer(M25P16_SPI_INSTANCE, in, command, sizeof(command));
    DISABLE_M25P16;
    return in[1];
}
bool m25p16_isReady()
{
    couldBeBusy = couldBeBusy && ((m25p16_readStatus() & M25P16_STATUS_FLAG_WRITE_IN_PROGRESS) != 0);
    return !couldBeBusy;
}
bool m25p16_waitForReady(uint32_t timeoutMillis)
{
    uint32_t time = millis();
    while (!m25p16_isReady()) {
        if (millis() - time > timeoutMillis) {
            return false;
        }
    }
    return true;
}
static bool m25p16_readIdentification()
{
    uint8_t out[] = { M25P16_INSTRUCTION_RDID, 0, 0, 0};
    uint8_t in[4];
    uint32_t chipID;
    delay(50);
    in[1] = 0;
    ENABLE_M25P16;
    spiTransfer(M25P16_SPI_INSTANCE, in, out, sizeof(out));
    DISABLE_M25P16;
    chipID = (in[1] << 16) | (in[2] << 8) | (in[3]);
    switch (chipID) {
        case JEDEC_ID_MICRON_M25P16:
            geometry.sectors = 32;
            geometry.pagesPerSector = 256;
        break;
        case JEDEC_ID_MICRON_N25Q064:
        case JEDEC_ID_WINBOND_W25Q64:
            geometry.sectors = 128;
            geometry.pagesPerSector = 256;
        break;
        case JEDEC_ID_MICRON_N25Q128:
        case JEDEC_ID_WINBOND_W25Q128:
            geometry.sectors = 256;
            geometry.pagesPerSector = 256;
        break;
        default:
            geometry.sectors = 0;
            geometry.pagesPerSector = 0;
            geometry.sectorSize = 0;
            geometry.totalSize = 0;
            return false;
    }
    geometry.sectorSize = geometry.pagesPerSector * geometry.pageSize;
    geometry.totalSize = geometry.sectorSize * geometry.sectors;
    couldBeBusy = true;
    return true;
}
bool m25p16_init()
{
#ifdef M25P16_CS_PIN
 flashSpim25p16CsPin = IOGetByTag(IO_TAG(M25P16_CS_PIN));
#endif
    IOInit(flashSpim25p16CsPin, OWNER_SYSTEM, RESOURCE_SPI);
 IOConfigGPIO(flashSpim25p16CsPin, SPI_IO_CS_CFG);
    spiSetDivisor(M25P16_SPI_INSTANCE, SPI_ULTRAFAST_CLOCK);
    return m25p16_readIdentification();
}
void m25p16_eraseSector(uint32_t address)
{
    uint8_t out[] = { M25P16_INSTRUCTION_SECTOR_ERASE, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};
    m25p16_waitForReady(SECTOR_ERASE_TIMEOUT_MILLIS);
    m25p16_writeEnable();
    ENABLE_M25P16;
    spiTransfer(M25P16_SPI_INSTANCE, NULL, out, sizeof(out));
    DISABLE_M25P16;
}
void m25p16_eraseCompletely()
{
    m25p16_waitForReady(BULK_ERASE_TIMEOUT_MILLIS);
    m25p16_writeEnable();
    m25p16_performOneByteCommand(M25P16_INSTRUCTION_BULK_ERASE);
}
void m25p16_pageProgramBegin(uint32_t address)
{
    uint8_t command[] = { M25P16_INSTRUCTION_PAGE_PROGRAM, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};
    m25p16_waitForReady(DEFAULT_TIMEOUT_MILLIS);
    m25p16_writeEnable();
    ENABLE_M25P16;
    spiTransfer(M25P16_SPI_INSTANCE, NULL, command, sizeof(command));
}
void m25p16_pageProgramContinue(const uint8_t *data, int length)
{
    spiTransfer(M25P16_SPI_INSTANCE, NULL, data, length);
}
void m25p16_pageProgramFinish()
{
    DISABLE_M25P16;
}
void m25p16_pageProgram(uint32_t address, const uint8_t *data, int length)
{
    m25p16_pageProgramBegin(address);
    m25p16_pageProgramContinue(data, length);
    m25p16_pageProgramFinish();
}
int m25p16_readBytes(uint32_t address, uint8_t *buffer, int length)
{
    uint8_t command[] = { M25P16_INSTRUCTION_READ_BYTES, (address >> 16) & 0xFF, (address >> 8) & 0xFF, address & 0xFF};
    if (!m25p16_waitForReady(DEFAULT_TIMEOUT_MILLIS)) {
        return 0;
    }
    ENABLE_M25P16;
    spiTransfer(M25P16_SPI_INSTANCE, NULL, command, sizeof(command));
    spiTransfer(M25P16_SPI_INSTANCE, buffer, NULL, length);
    DISABLE_M25P16;
    return length;
}
const flashGeometry_t* m25p16_getGeometry()
{
    return &geometry;
}
#endif
