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
       
#include "io.h"
#include "platform.h"
typedef struct ioDef_s {
 ioTag_t tag;
} ioDef_t;
typedef struct ioRec_s {
 GPIO_TypeDef *gpio;
 uint16_t pin;
 resourceOwner_t owner;
 resourceType_t resourcesUsed;
} ioRec_t;
extern ioRec_t ioRecs[DEFIO_IO_USED_COUNT];
int IO_GPIOPortIdx(IO_t io);
int IO_GPIOPinIdx(IO_t io);
#if defined(STM32F10X)
int IO_GPIO_PinSource(IO_t io);
int IO_GPIO_PortSource(IO_t io);
#elif defined(STM32F303xC)
int IO_GPIO_PinSource(IO_t io);
int IO_GPIO_PortSource(IO_t io);
int IO_EXTI_PortSourceGPIO(IO_t io);
int IO_EXTI_PinSource(IO_t io);
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F446xx)
int IO_GPIO_PinSource(IO_t io);
int IO_GPIO_PortSource(IO_t io);
int IO_EXTI_PortSourceGPIO(IO_t io);
int IO_EXTI_PinSource(IO_t io);
# endif
uint32_t IO_EXTI_Line(IO_t io);
ioRec_t *IO_Rec(IO_t io);
