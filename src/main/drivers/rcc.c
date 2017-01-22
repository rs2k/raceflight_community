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
#include "platform.h"
#include "rcc.h"
void RCC_ClockCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
{
 int tag = periphTag >> 5;
 uint32_t mask = 1 << (periphTag & 0x1f);
 switch (tag) {
#if defined(STM32F303xC)
 case RCC_AHB:
  RCC_AHBPeriphClockCmd(mask, NewState);
  break;
#endif
 case RCC_APB2:
  RCC_APB2PeriphClockCmd(mask, NewState);
  break;
 case RCC_APB1:
  RCC_APB1PeriphClockCmd(mask, NewState);
  break;
#if defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F446xx)
 case RCC_AHB1:
  RCC_AHB1PeriphClockCmd(mask, NewState);
  break;
#endif
 }
}
void RCC_ResetCmd(rccPeriphTag_t periphTag, FunctionalState NewState)
{
    int tag = periphTag >> 5;
    uint32_t mask = 1 << (periphTag & 0x1f);
    switch (tag) {
#if defined(STM32F303xC)
    case RCC_AHB:
        RCC_AHBPeriphResetCmd(mask, NewState);
        break;
#endif
    case RCC_APB2:
        RCC_APB2PeriphResetCmd(mask, NewState);
        break;
    case RCC_APB1:
        RCC_APB1PeriphResetCmd(mask, NewState);
        break;
#if defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F446xx)
    case RCC_AHB1:
        RCC_AHB1PeriphResetCmd(mask, NewState);
        break;
#endif
    }
}
