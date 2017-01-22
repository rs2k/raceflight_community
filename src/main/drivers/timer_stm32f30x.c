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
#include "stm32f30x.h"
#define CCMR_OFFSET ((uint16_t)0x0018)
#define CCMR_OC13M_MASK ((uint32_t)0xFFFEFF8F)
#define CCMR_OC24M_MASK ((uint32_t)0xFEFF8FFF)
void TIM_SelectOCxM_NoDisable(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint32_t TIM_OCMode)
{
  uint32_t tmp = 0;
  assert_param(IS_TIM_LIST1_PERIPH(TIMx));
  assert_param(IS_TIM_CHANNEL(TIM_Channel));
  assert_param(IS_TIM_OCM(TIM_OCMode));
  tmp = (uint32_t) TIMx;
  tmp += CCMR_OFFSET;
  if((TIM_Channel == TIM_Channel_1) ||(TIM_Channel == TIM_Channel_3))
  {
    tmp += (TIM_Channel>>1);
    *(__IO uint32_t *) tmp &= CCMR_OC13M_MASK;
    *(__IO uint32_t *) tmp |= TIM_OCMode;
  }
  else
  {
    tmp += (uint32_t)(TIM_Channel - (uint32_t)4)>> (uint32_t)1;
    *(__IO uint32_t *) tmp &= CCMR_OC24M_MASK;
    *(__IO uint32_t *) tmp |= (uint32_t)(TIM_OCMode << 8);
  }
}
