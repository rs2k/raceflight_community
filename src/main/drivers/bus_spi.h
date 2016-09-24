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
 */

#pragma once

#include <stdint.h>
#include "io.h"
#include "rcc.h"

#if defined(STM32F40_41xxx) || defined (STM32F411xE) || defined(STM32F446xx)
#define SPI_IO_AF_CFG IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_UP)
#define SPI_IO_CS_CFG IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_UP)
#elif defined(STM32F303xC)
#define SPI_IO_AF_CFG IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP)
#define SPI_IO_CS_CFG IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP)
#elif defined(STM32F10X)
#define SPI_IO_AF_CFG IO_CONFIG(GPIO_Mode_AF_OD, GPIO_Speed_50MHz)
#define SPI_IO_CS_CFG IO_CONFIG(GPIO_Mode_Out_OD, GPIO_Speed_50MHz)
#else
#error "Unknown processor"
#endif
#if defined(STM32F40_41xxx) || defined (STM32F411xE) || defined(STM32F446xx)

#define SPI_SLOW_CLOCK      128 
#define SPI_STANDARD_CLOCK    8 
#define SPI_FAST_CLOCK        4 
#define SPI_ULTRAFAST_CLOCK   2 

#else

#define SPI_SLOW_CLOCK       128 
#define SPI_STANDARD_CLOCK     4 
#define SPI_FAST_CLOCK         2 
#define SPI_ULTRAFAST_CLOCK    2 

#endif

typedef enum SPIDevice {
    SPIINVALID = -1,
    SPIDEV_1 = 0,
    SPIDEV_2,
    SPIDEV_3,
    SPIDEV_MAX = SPIDEV_3,
} SPIDevice;

typedef struct SPIDevice_s {
    SPI_TypeDef *dev;
    ioTag_t nss;
    ioTag_t sck;
    ioTag_t mosi;
    ioTag_t miso;
    rccPeriphTag_t rcc;
    uint8_t af;
    volatile uint16_t errorCount;
} spiDevice_t;

bool spiInit(SPIDevice device);
void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor);
uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t in);

bool spiTransfer(SPI_TypeDef *instance, uint8_t *out, const uint8_t *in, int len);

uint16_t spiGetErrorCounter(SPI_TypeDef *instance);
void spiResetErrorCounter(SPI_TypeDef *instance);
