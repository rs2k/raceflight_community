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
#pragma once 
       
#include <stdbool.h>
#include <stdint.h>
#include "resource.h"
typedef uint8_t ioTag_t;
typedef void* IO_t;
#define IO_NONE ((IO_t)0)
#define IO_TAG(pinid) DEFIO_TAG(pinid)
typedef uint8_t ioConfig_t;
#if defined(STM32F10X)
#define IO_CONFIG(mode,speed) ((mode) | (speed))
#define IOCFG_OUT_PP IO_CONFIG(GPIO_Mode_Out_PP, GPIO_Speed_2MHz)
#define IOCFG_OUT_OD IO_CONFIG(GPIO_Mode_Out_OD, GPIO_Speed_2MHz)
#define IOCFG_AF_PP IO_CONFIG(GPIO_Mode_AF_PP, GPIO_Speed_2MHz)
#define IOCFG_AF_OD IO_CONFIG(GPIO_Mode_AF_OD, GPIO_Speed_2MHz)
#define IOCFG_IPD IO_CONFIG(GPIO_Mode_IPD, GPIO_Speed_2MHz)
#define IOCFG_IPU IO_CONFIG(GPIO_Mode_IPU, GPIO_Speed_2MHz)
#define IOCFG_IN_FLOATING IO_CONFIG(GPIO_Mode_IN_FLOATING, GPIO_Speed_2MHz)
#elif defined(STM32F303xC)
#define IO_CONFIG(mode,speed,otype,pupd) ((mode) | ((speed) << 2) | ((otype) << 4) | ((pupd) << 5))
#define IOCFG_OUT_PP IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define IOCFG_OUT_OD IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL)
#define IOCFG_AF_PP IO_CONFIG(GPIO_Mode_AF, 0, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define IOCFG_AF_OD IO_CONFIG(GPIO_Mode_AF, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL)
#define IOCFG_IPD IO_CONFIG(GPIO_Mode_IN, 0, 0, GPIO_PuPd_DOWN)
#define IOCFG_IPU IO_CONFIG(GPIO_Mode_IN, 0, 0, GPIO_PuPd_UP)
#define IOCFG_IN_FLOATING IO_CONFIG(GPIO_Mode_IN, 0, 0, GPIO_PuPd_NOPULL)
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F446xx)
#define IO_CONFIG(mode,speed,otype,pupd) ((mode) | ((speed) << 2) | ((otype) << 4) | ((pupd) << 5))
#define IOCFG_OUT_PP IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define IOCFG_OUT_OD IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL)
#define IOCFG_AF_PP IO_CONFIG(GPIO_Mode_AF, 0, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define IOCFG_AF_OD IO_CONFIG(GPIO_Mode_AF, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL)
#define IOCFG_AF_OD_UP IO_CONFIG(GPIO_Mode_AF, 0, GPIO_OType_OD, GPIO_PuPd_UP)
#define IOCFG_IPD IO_CONFIG(GPIO_Mode_IN, 0, 0, GPIO_PuPd_DOWN)
#define IOCFG_IPU IO_CONFIG(GPIO_Mode_IN, 0, 0, GPIO_PuPd_UP)
#define IOCFG_IN_FLOATING IO_CONFIG(GPIO_Mode_IN, 0, 0, GPIO_PuPd_NOPULL)
#elif defined(UNIT_TEST)
#define IOCFG_OUT_PP 0
#define IOCFG_OUT_OD 0
#define IOCFG_AF_PP 0
#define IOCFG_AF_OD 0
#define IOCFG_IPD 0
#define IOCFG_IPU 0
#define IOCFG_IN_FLOATING 0
#else
# warning "Unknown TARGET"
#endif
#include "io_def.h"
bool IORead(IO_t io);
void IOWrite(IO_t io, bool value);
void IOHi(IO_t io);
void IOLo(IO_t io);
void IOToggle(IO_t io);
void IOInit(IO_t io, resourceOwner_t owner, resourceType_t resources);
void IORelease(IO_t io);
resourceOwner_t IOGetOwner(IO_t io);
resourceType_t IOGetResources(IO_t io);
IO_t IOGetByTag(ioTag_t tag);
void IOConfigGPIO(IO_t io, ioConfig_t cfg);
#if defined(STM32F303xC) || defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F446xx)
void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af);
#endif
void IOInitGlobal(void);
