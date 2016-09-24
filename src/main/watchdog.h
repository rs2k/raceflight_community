/*
 * This file is part of Raceflight.
 *
 * Raceflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Raceflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Raceflight.  If not, see <http://www.gnu.org/licenses/>.
 */



#ifndef WATCHDOG_H
#define WATCHDOG_H

#if defined(STM32F4)
#include "stm32f4xx.h"
#elif defined(STM32F303xC)#include "stm32f30x.h"#else#include "stm32f10x.h"
#endif

typedef enum {
	Watchdog_Timeout_5ms   = 0x00,  /*!< System reset called every 5ms   */
	Watchdog_Timeout_10ms  = 0x01,  /*!< System reset called every 10ms  */
	Watchdog_Timeout_15ms  = 0x02,  /*!< System reset called every 15ms  */
	Watchdog_Timeout_30ms  = 0x03,  /*!< System reset called every 30ms  */
	Watchdog_Timeout_60ms  = 0x04,  /*!< System reset called every 60ms  */
	Watchdog_Timeout_120ms = 0x05,  /*!< System reset called every 120ms */
	Watchdog_Timeout_250ms = 0x06,  /*!< System reset called every 250ms */
	Watchdog_Timeout_500ms = 0x07,  /*!< System reset called every 500ms */
	Watchdog_Timeout_1s    = 0x08,  /*!< System reset called every 1s    */
	Watchdog_Timeout_2s    = 0x09,  /*!< System reset called every 2s    */
	Watchdog_Timeout_4s    = 0x0A,  /*!< System reset called every 4s    */
	Watchdog_Timeout_8s    = 0x0B,  /*!< System reset called every 8s    */
	Watchdog_Timeout_16s   = 0x0C,  /*!< System reset called every 16s   */
	Watchdog_Timeout_32s   = 0x0D   /*!< System reset called every 32s. This is maximum value allowed with IWDG timer */
} watchdog_timeout_t;

uint8_t rx_watchdog_init(watchdog_timeout_t timeout);
void rx_watchdog_deinit(void);
void reset_rx_watchdog(void);



#endif 

