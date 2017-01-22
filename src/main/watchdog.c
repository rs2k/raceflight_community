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
#include <stdbool.h>
#include "watchdog.h"
#include "include.h"
#define WATCHDOG_RESET_INTERVAL 50
uint32_t failsafeState = 0;
bool watchdogInitialized = false;
volatile uint32_t lastWatchdogReset = 0;
volatile uint32_t timeSinceFailsafe = 0;
volatile uint32_t lastRxPulse = 0;
volatile uint32_t rxPulseCount = 1;
uint8_t rx_watchdog_init(watchdog_timeout_t timeout) {
 uint8_t result = 0;
 uint16_t reload = 0;
#if defined(STM32F4)
 if (RCC->CSR & RCC_CSR_WDGRSTF) {
#elif defined(STM32F303xC)
 if (RCC->CSR & RCC_CSR_WWDGRSTF) {
#else
 if (RCC->CSR & RCC_CSR_WWDGRSTF) {
#endif
  result = 1;
  RCC->CSR |= RCC_CSR_RMVF;
 }
 IWDG->KR = 0x5555;
 if (timeout >= Watchdog_Timeout_8s) {
  IWDG->PR = 0x07;
 }
 else {
  IWDG->PR = 0x03;
 }
 if (timeout == Watchdog_Timeout_5ms) {
  reload = 5;
 }
 else if (timeout == Watchdog_Timeout_10ms) {
  reload = 10;
 }
 else if (timeout == Watchdog_Timeout_15ms) {
  reload = 15;
 }
 else if (timeout == Watchdog_Timeout_30ms) {
  reload = 31;
 }
 else if (timeout == Watchdog_Timeout_60ms) {
  reload = 61;
 }
 else if (timeout == Watchdog_Timeout_120ms) {
  reload = 123;
 }
 else if (timeout == Watchdog_Timeout_250ms) {
  reload = 255;
 }
 else if (timeout == Watchdog_Timeout_500ms) {
  reload = 511;
 }
 else if (timeout == Watchdog_Timeout_1s) {
  reload = 1023;
 }
 else if (timeout == Watchdog_Timeout_2s) {
  reload = 2047;
 }
 else if (timeout == Watchdog_Timeout_4s) {
  reload = 4095;
 }
 else if (timeout == Watchdog_Timeout_8s) {
  reload = 1023;
 }
 else if (timeout == Watchdog_Timeout_16s) {
  reload = 2047;
 }
 else if (timeout == Watchdog_Timeout_32s) {
  reload = 4095;
 }
 IWDG->RLR = reload;
 IWDG->KR = 0xAAAA;
 IWDG->KR = 0xCCCC;
 watchdogInitialized = true;
 return result;
}
inline void updateWatchdog(void) {
 feedTheDog();
}
inline void feedTheDog(void) {
 IWDG->KR = 0xAAAA;
}
inline void resetTimeSinceRxPulse(void) {
 rxPulseCount++;
}
void checkForRxFailsafe(void) {
 if (!ARMING_FLAG(ARMED) ) {
  updateWatchdog();
  return;
 }
 if ((rcData[THROTTLE] > 950) && (rxPulseCount > lastRxPulse))
 {
  lastRxPulse = rxPulseCount;
  updateWatchdog();
 }
 return;
}
