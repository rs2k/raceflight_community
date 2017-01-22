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
       
#ifndef WATCHDOG_H
#define WATCHDOG_H 
#if defined(STM32F4)
#include "stm32f4xx.h"
#elif defined(STM32F303xC)
#include "stm32f30x.h"
#else
#include "stm32f10x.h"
#endif
typedef enum {
 Watchdog_Timeout_5ms = 0x00,
 Watchdog_Timeout_10ms = 0x01,
 Watchdog_Timeout_15ms = 0x02,
 Watchdog_Timeout_30ms = 0x03,
 Watchdog_Timeout_60ms = 0x04,
 Watchdog_Timeout_120ms = 0x05,
 Watchdog_Timeout_250ms = 0x06,
 Watchdog_Timeout_500ms = 0x07,
 Watchdog_Timeout_1s = 0x08,
 Watchdog_Timeout_2s = 0x09,
 Watchdog_Timeout_4s = 0x0A,
 Watchdog_Timeout_8s = 0x0B,
 Watchdog_Timeout_16s = 0x0C,
 Watchdog_Timeout_32s = 0x0D
} watchdog_timeout_t;
extern uint32_t failsafeState;
uint8_t rx_watchdog_init(watchdog_timeout_t timeout);
void rx_watchdog_deinit(void);
void feedTheDog(void);
extern bool inFailSafeStg2;
extern bool inFailSafeStg1;
void updateWatchdog(void) ;
void feedTheDog(void);
void checkForRxFailsafe(void);
void resetTimeSinceRxPulse(void);
#endif
