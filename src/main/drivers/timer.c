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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"
#include "common/utils.h"
#include "common/atomic.h"

#include "nvic.h"

#include "gpio.h"
#include "system.h"
#include "debug.h"

#include "timer.h"
#include "timer_impl.h"

#define TIM_N(n) (1 << (n))

/*
    Groups that allow running different period (ex 50Hz servos + 400Hz throttle + etc):
    TIM1 2 channels
    TIM2 4 channels
    TIM3 4 channels
    TIM4 4 channels
*/

#if defined(CJMCU) || defined(EUSTM32F103RC) || defined(NAZE) || defined(OLIMEXINO) || defined(PORT103R)
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM2, GPIOA, Pin_0, TIM_Channel_1, TIM2_IRQn, 0, Mode_IPD},          
    { TIM2, GPIOA, Pin_1, TIM_Channel_2, TIM2_IRQn, 0, Mode_IPD},          
    { TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn, 0, Mode_IPD},          
    { TIM2, GPIOA, Pin_3, TIM_Channel_4, TIM2_IRQn, 0, Mode_IPD},          
    { TIM3, GPIOA, Pin_6, TIM_Channel_1, TIM3_IRQn, 0, Mode_IPD},          
    { TIM3, GPIOA, Pin_7, TIM_Channel_2, TIM3_IRQn, 0, Mode_IPD},          
    { TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn, 0, Mode_IPD},          
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 0, Mode_IPD},          
    { TIM1, GPIOA, Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 1, Mode_IPD},       
    { TIM1, GPIOA, Pin_11, TIM_Channel_4, TIM1_CC_IRQn, 1, Mode_IPD},      
    { TIM4, GPIOB, Pin_6, TIM_Channel_1, TIM4_IRQn, 0, Mode_IPD},          
    { TIM4, GPIOB, Pin_7, TIM_Channel_2, TIM4_IRQn, 0, Mode_IPD},          
    { TIM4, GPIOB, Pin_8, TIM_Channel_3, TIM4_IRQn, 0, Mode_IPD},          
    { TIM4, GPIOB, Pin_9, TIM_Channel_4, TIM4_IRQn, 0, Mode_IPD}           
};

#define USED_TIMERS         (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB)
#endif

#ifdef CC3D
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM4, GPIOB, Pin_6, TIM_Channel_1, TIM4_IRQn, 0, Mode_IPD}, 
    { TIM3, GPIOB, Pin_5, TIM_Channel_2, TIM3_IRQn, 0, Mode_IPD}, 
    { TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn, 0, Mode_IPD}, 
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 0, Mode_IPD}, 
    { TIM2, GPIOA, Pin_0, TIM_Channel_1, TIM2_IRQn, 0, Mode_IPD}, 
    { TIM2, GPIOA, Pin_1, TIM_Channel_2, TIM2_IRQn, 0, Mode_IPD}, 

    { TIM4, GPIOB, Pin_9, TIM_Channel_4, TIM4_IRQn, 1, GPIO_Mode_AF_PP},    
    { TIM4, GPIOB, Pin_8, TIM_Channel_3, TIM4_IRQn, 1, GPIO_Mode_AF_PP},    
    { TIM4, GPIOB, Pin_7, TIM_Channel_2, TIM4_IRQn, 1, GPIO_Mode_AF_PP},    
    { TIM1, GPIOA, Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 1, GPIO_Mode_AF_PP}, 
    { TIM3, GPIOB, Pin_4, TIM_Channel_1, TIM3_IRQn, 1, GPIO_Mode_AF_PP},    
    { TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn, 1, GPIO_Mode_AF_PP}     
};

#define USED_TIMERS         (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB)
#endif

#if defined(STM32F3DISCOVERY) && !(defined(CHEBUZZF3))
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1, GPIOA, Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 1, Mode_AF_PP_PD, GPIO_PinSource8, GPIO_AF_6, 0},             
    { TIM16, GPIOB, Pin_8, TIM_Channel_1, TIM1_UP_TIM16_IRQn, 0, Mode_AF_PP_PD, GPIO_PinSource8, GPIO_AF_1, 0},      
    { TIM17, GPIOB, Pin_9, TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 0, Mode_AF_PP_PD, GPIO_PinSource9, GPIO_AF_1, 0}, 
    { TIM8, GPIOC, Pin_6, TIM_Channel_1, TIM8_CC_IRQn, 1, Mode_AF_PP_PD, GPIO_PinSource6, GPIO_AF_4, 0},             
    { TIM8, GPIOC, Pin_7, TIM_Channel_2, TIM8_CC_IRQn, 1, Mode_AF_PP_PD, GPIO_PinSource7, GPIO_AF_4, 0},             
    { TIM8, GPIOC, Pin_8, TIM_Channel_3, TIM8_CC_IRQn, 1, Mode_AF_PP_PD, GPIO_PinSource8, GPIO_AF_4, 0},             
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 0, Mode_AF_PP_PD, GPIO_PinSource1, GPIO_AF_2, 0},                
    { TIM3, GPIOA, Pin_4, TIM_Channel_2, TIM3_IRQn, 0, Mode_AF_PP_PD, GPIO_PinSource4, GPIO_AF_2, 0},                
    { TIM4, GPIOD, Pin_12, TIM_Channel_1, TIM4_IRQn, 0, Mode_AF_PP, GPIO_PinSource12, GPIO_AF_2, 0},                  
    { TIM4, GPIOD, Pin_13, TIM_Channel_2, TIM4_IRQn, 0, Mode_AF_PP, GPIO_PinSource13, GPIO_AF_2, 0},                  
    { TIM4, GPIOD, Pin_14, TIM_Channel_3, TIM4_IRQn, 0, Mode_AF_PP, GPIO_PinSource14, GPIO_AF_2, 0},                  
    { TIM4, GPIOD, Pin_15, TIM_Channel_4, TIM4_IRQn, 0, Mode_AF_PP, GPIO_PinSource15, GPIO_AF_2, 0},                  
    { TIM2, GPIOA, Pin_1, TIM_Channel_2, TIM2_IRQn, 0, Mode_AF_PP, GPIO_PinSource1, GPIO_AF_1, 0},                   
    { TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn, 0, Mode_AF_PP, GPIO_PinSource2, GPIO_AF_1, 0}                    
};

#define USED_TIMERS  (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(16) | TIM_N(17))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8 | RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM17)
#define TIMER_AHB_PERIPHERALS (RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD)

#endif

#if defined(COLIBRI_RACE) || defined(LUX_RACE)
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1,  GPIOA, Pin_8,  TIM_Channel_1, TIM1_CC_IRQn,            0, Mode_AF_PP_PD,   GPIO_PinSource8,  GPIO_AF_6, 0}, 

    { TIM3,  GPIOC, Pin_6,  TIM_Channel_1, TIM3_IRQn,               1, Mode_AF_PP,      GPIO_PinSource6,  GPIO_AF_2, 0}, 
    { TIM3,  GPIOC, Pin_7,  TIM_Channel_2, TIM3_IRQn,               1, Mode_AF_PP,      GPIO_PinSource7,  GPIO_AF_2, 0}, 
    { TIM3,  GPIOC, Pin_8,  TIM_Channel_3, TIM3_IRQn,               1, Mode_AF_PP,      GPIO_PinSource8,  GPIO_AF_2, 0}, 
    { TIM3,  GPIOC, Pin_9,  TIM_Channel_4, TIM3_IRQn,               1, Mode_AF_PP,      GPIO_PinSource9,  GPIO_AF_2, 0}, 

    { TIM2,  GPIOA, Pin_0,  TIM_Channel_1, TIM2_IRQn,               1, Mode_AF_PP,      GPIO_PinSource0,  GPIO_AF_1, 0}, 
    { TIM2,  GPIOA, Pin_1,  TIM_Channel_2, TIM2_IRQn,               1, Mode_AF_PP,      GPIO_PinSource1,  GPIO_AF_1, 0}, 
    { TIM2,  GPIOA, Pin_2,  TIM_Channel_3, TIM2_IRQn,               1, Mode_AF_PP,      GPIO_PinSource2,  GPIO_AF_1, 0}, 
    { TIM2,  GPIOA, Pin_3,  TIM_Channel_4, TIM2_IRQn,               1, Mode_AF_PP,      GPIO_PinSource3,  GPIO_AF_1, 0}, 

    { TIM15, GPIOB, Pin_14, TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP_PD,   GPIO_PinSource14, GPIO_AF_1, 0}, 
    { TIM15, GPIOB, Pin_15, TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP_PD,   GPIO_PinSource15, GPIO_AF_1, 0}, 
};

#define USED_TIMERS  (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(15))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM15)
#define TIMER_AHB_PERIPHERALS (RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOC)

#endif

#ifdef CHEBUZZF3
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    
    { TIM1,  GPIOA, Pin_8,  TIM_Channel_1, TIM1_CC_IRQn,            1, Mode_AF_PP_PD, GPIO_PinSource8, GPIO_AF_6, 0}, 
    { TIM16, GPIOB, Pin_8,  TIM_Channel_1, TIM1_UP_TIM16_IRQn,      0, Mode_AF_PP_PD, GPIO_PinSource8, GPIO_AF_1, 0}, 
    { TIM17, GPIOB, Pin_9,  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 0, Mode_AF_PP_PD, GPIO_PinSource9, GPIO_AF_1, 0}, 
    { TIM8,  GPIOC, Pin_6,  TIM_Channel_1, TIM8_CC_IRQn,            1, Mode_AF_PP_PD, GPIO_PinSource6, GPIO_AF_4, 0}, 
    { TIM8,  GPIOC, Pin_7,  TIM_Channel_2, TIM8_CC_IRQn,            1, Mode_AF_PP_PD, GPIO_PinSource7, GPIO_AF_4, 0}, 
    { TIM8,  GPIOC, Pin_8,  TIM_Channel_3, TIM8_CC_IRQn,            1, Mode_AF_PP_PD, GPIO_PinSource8, GPIO_AF_4, 0}, 
    { TIM15, GPIOF, Pin_9,  TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     0, Mode_AF_PP_PD, GPIO_PinSource9, GPIO_AF_3, 0}, 
    { TIM15, GPIOF, Pin_10, TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     0, Mode_AF_PP_PD, GPIO_PinSource10, GPIO_AF_3, 0}, 

    
    { TIM4,  GPIOD, Pin_12, TIM_Channel_1, TIM4_IRQn,               0, Mode_AF_PP, GPIO_PinSource12, GPIO_AF_2, 0},    
    { TIM4,  GPIOD, Pin_13, TIM_Channel_2, TIM4_IRQn,               0, Mode_AF_PP, GPIO_PinSource13, GPIO_AF_2, 0},    
    { TIM4,  GPIOD, Pin_14, TIM_Channel_3, TIM4_IRQn,               0, Mode_AF_PP, GPIO_PinSource14, GPIO_AF_2, 0},    
    { TIM4,  GPIOD, Pin_15, TIM_Channel_4, TIM4_IRQn,               0, Mode_AF_PP, GPIO_PinSource15, GPIO_AF_2, 0},    
    { TIM2,  GPIOA, Pin_1,  TIM_Channel_2, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource1, GPIO_AF_1, 0},    
    { TIM2,  GPIOA, Pin_2,  TIM_Channel_3, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource2, GPIO_AF_1, 0},    
    { TIM2,  GPIOA, Pin_3,  TIM_Channel_4, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource3, GPIO_AF_1, 0},    
    { TIM3,  GPIOB, Pin_0,  TIM_Channel_3, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource0, GPIO_AF_2, 0},    
    { TIM3,  GPIOB, Pin_1,  TIM_Channel_4, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource1, GPIO_AF_2, 0},    
    { TIM3,  GPIOA, Pin_4,  TIM_Channel_2, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource4, GPIO_AF_2, 0}     
};

#define USED_TIMERS  (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(15) | TIM_N(16) | TIM_N(17))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8 | RCC_APB2Periph_TIM15 | RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM17)
#define TIMER_AHB_PERIPHERALS (RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOF)

#endif

#ifdef NAZE32PRO
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1,  GPIOA, Pin_8,  TIM_Channel_1, TIM1_CC_IRQn,            0, Mode_AF_PP_PD, GPIO_PinSource8,  GPIO_AF_6}, 
    { TIM1,  GPIOA, Pin_9,  TIM_Channel_2, TIM1_CC_IRQn,            0, Mode_AF_PP_PD, GPIO_PinSource9,  GPIO_AF_6}, 
    { TIM1,  GPIOA, Pin_10, TIM_Channel_3, TIM1_CC_IRQn,            0, Mode_AF_PP_PD, GPIO_PinSource10, GPIO_AF_6}, 
    { TIM3,  GPIOB, Pin_4,  TIM_Channel_1, TIM3_IRQn,               0, Mode_AF_PP_PD, GPIO_PinSource4,  GPIO_AF_2}, 
    { TIM4,  GPIOB, Pin_6,  TIM_Channel_1, TIM4_IRQn,               0, Mode_AF_PP_PD, GPIO_PinSource6,  GPIO_AF_2}, 
    { TIM4,  GPIOB, Pin_7,  TIM_Channel_2, TIM4_IRQn,               0, Mode_AF_PP_PD, GPIO_PinSource7,  GPIO_AF_2}, 
    { TIM4,  GPIOB, Pin_8,  TIM_Channel_3, TIM4_IRQn,               0, Mode_AF_PP_PD, GPIO_PinSource8,  GPIO_AF_2}, 
    { TIM4,  GPIOB, Pin_9,  TIM_Channel_4, TIM4_IRQn,               0, Mode_AF_PP_PD, GPIO_PinSource9,  GPIO_AF_2}, 

    { TIM2,  GPIOA, Pin_0,  TIM_Channel_1, TIM2_IRQn,               1, Mode_AF_PP, GPIO_PinSource0, GPIO_AF_2}, 
    { TIM2,  GPIOA, Pin_1,  TIM_Channel_2, TIM2_IRQn,               1, Mode_AF_PP, GPIO_PinSource1, GPIO_AF_2}, 
    { TIM15, GPIOA, Pin_2,  TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP, GPIO_PinSource2, GPIO_AF_9}, 
    { TIM15, GPIOA, Pin_3,  TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP, GPIO_PinSource3, GPIO_AF_9}, 
    { TIM16, GPIOA, Pin_6,  TIM_Channel_1, TIM1_UP_TIM16_IRQn,      1, Mode_AF_PP, GPIO_PinSource6, GPIO_AF_1}, 
    { TIM17, GPIOA, Pin_7,  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, Mode_AF_PP, GPIO_PinSource7, GPIO_AF_1} 
};

#define USED_TIMERS  (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(15) | TIM_N(16) | TIM_N(17))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM15 | RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM17)
#define TIMER_AHB_PERIPHERALS (RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB)

#endif

#if defined(SPARKY) || defined(ALIENWIIF3)
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    
    
    

    { TIM15, GPIOB, Pin_15, TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP, GPIO_PinSource15, GPIO_AF_1, 0}, 
    { TIM15, GPIOB, Pin_14, TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP, GPIO_PinSource14, GPIO_AF_1, 0}, 
    { TIM1,  GPIOA, Pin_8,  TIM_Channel_1, TIM1_CC_IRQn,            1, Mode_AF_PP, GPIO_PinSource8,  GPIO_AF_6, 0}, 
    { TIM3,  GPIOB, Pin_0,  TIM_Channel_3, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource0,  GPIO_AF_2, 0}, 
    { TIM3,  GPIOA, Pin_6,  TIM_Channel_1, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource6,  GPIO_AF_2, 0}, 
    { TIM2,  GPIOA, Pin_2,  TIM_Channel_3, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource2,  GPIO_AF_1, 0}, 

    
    
    

    
    { TIM3,  GPIOB, Pin_1,  TIM_Channel_4, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource1,  GPIO_AF_2, 0}, 
    { TIM17, GPIOA, Pin_7,  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, Mode_AF_PP, GPIO_PinSource7,  GPIO_AF_1, 0}, 
    { TIM3,  GPIOA, Pin_4,  TIM_Channel_2, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource4,  GPIO_AF_2, 0}, 
    { TIM2,  GPIOA, Pin_1,  TIM_Channel_2, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource1,  GPIO_AF_1, 0}, 

    
    
    

    { TIM2, GPIOA, Pin_3,  TIM_Channel_4, TIM2_IRQn,                0, Mode_AF_PP_PD, GPIO_PinSource3, GPIO_AF_1, 0} 
    

    
    
    
    

};

#define USED_TIMERS  (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(15) | TIM_N(17))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM15 | RCC_APB2Periph_TIM17)
#define TIMER_AHB_PERIPHERALS (RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB)

#endif

#if defined(DOGE)
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1,  GPIOA,  Pin_8,  TIM_Channel_1, TIM1_CC_IRQn, 0, Mode_AF_PP_PD,   GPIO_PinSource8,  GPIO_AF_6,  0 }, 

    { TIM4,  GPIOB,  Pin_8,  TIM_Channel_3, TIM4_IRQn,    1, Mode_AF_PP,      GPIO_PinSource8,  GPIO_AF_2,  0 }, 
    { TIM4,  GPIOB,  Pin_9,  TIM_Channel_4, TIM4_IRQn,    1, Mode_AF_PP,      GPIO_PinSource9,  GPIO_AF_2,  0 }, 
    
    { TIM2,  GPIOA,  Pin_10, TIM_Channel_4, TIM2_IRQn,    1, Mode_AF_PP,      GPIO_PinSource10, GPIO_AF_10, 0 }, 
    { TIM2,  GPIOA,  Pin_9,  TIM_Channel_3, TIM2_IRQn,    1, Mode_AF_PP,      GPIO_PinSource9,  GPIO_AF_10, 0 }, 
    { TIM2,  GPIOA,  Pin_0,  TIM_Channel_1, TIM2_IRQn,    1, Mode_AF_PP,      GPIO_PinSource0,  GPIO_AF_1,  0 },  
    { TIM2,  GPIOA,  Pin_1,  TIM_Channel_2, TIM2_IRQn,    1, Mode_AF_PP,      GPIO_PinSource1,  GPIO_AF_1,  0 },  

    { TIM3,  GPIOB,  Pin_0,  TIM_Channel_3, TIM3_IRQn,    1, Mode_AF_PP_PD,   GPIO_PinSource0,  GPIO_AF_2,  0 },  
    { TIM3,  GPIOB,  Pin_1,  TIM_Channel_4, TIM3_IRQn,    1, Mode_AF_PP_PD,   GPIO_PinSource1,  GPIO_AF_2,  0 },  
};

#define USED_TIMERS     (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(15))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM15 | RCC_APB2Periph_TIM17)
#define TIMER_AHB_PERIPHERALS (RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB)

#endif

#if defined(REVO)
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
	{ TIM12, GPIOB, Pin_14, TIM_Channel_1, TIM8_BRK_TIM12_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource14, GPIO_AF_TIM12}, 
    { TIM12, GPIOB, Pin_15, TIM_Channel_2, TIM8_BRK_TIM12_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource15, GPIO_AF_TIM12}, 
    { TIM8, GPIOC, Pin_6, TIM_Channel_1, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource6, GPIO_AF_TIM8}, 
    { TIM8, GPIOC, Pin_7, TIM_Channel_2, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource7, GPIO_AF_TIM8}, 
    { TIM8, GPIOC, Pin_8, TIM_Channel_3, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource8, GPIO_AF_TIM8}, 
    { TIM8, GPIOC, Pin_9, TIM_Channel_4, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource9, GPIO_AF_TIM8}, 

    { TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM3},    
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM3},    
    { TIM9, GPIOA, Pin_3, TIM_Channel_2, TIM1_BRK_TIM9_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource3, GPIO_AF_TIM9},    
    { TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource2, GPIO_AF_TIM2}, 


};

#define USED_TIMERS  ( TIM_N(2) | TIM_N(3) | TIM_N(5) | TIM_N(12) | TIM_N(8) | TIM_N(9) | TIM_N(11))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM5 | RCC_APB1Periph_TIM12 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM8 | RCC_APB2Periph_TIM9 | RCC_APB2Periph_TIM11)

#endif

#if defined(REVOLT)
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
	{ TIM12, GPIOB, Pin_14, TIM_Channel_1, TIM8_BRK_TIM12_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource14, GPIO_AF_TIM12}, 
    { TIM12, GPIOB, Pin_15, TIM_Channel_2, TIM8_BRK_TIM12_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource15, GPIO_AF_TIM12}, 
    { TIM8, GPIOC, Pin_6, TIM_Channel_1, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource6, GPIO_AF_TIM8}, 
    { TIM8, GPIOC, Pin_7, TIM_Channel_2, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource7, GPIO_AF_TIM8}, 
    { TIM8, GPIOC, Pin_8, TIM_Channel_3, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource8, GPIO_AF_TIM8}, 
    { TIM8, GPIOC, Pin_9, TIM_Channel_4, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource9, GPIO_AF_TIM8}, 

    { TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM3},    
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM3},    
    { TIM9, GPIOA, Pin_3, TIM_Channel_2, TIM1_BRK_TIM9_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource3, GPIO_AF_TIM9},    
    { TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource2, GPIO_AF_TIM2}, 


};

#define USED_TIMERS  ( TIM_N(2) | TIM_N(3) | TIM_N(5) | TIM_N(12) | TIM_N(8) | TIM_N(9) | TIM_N(11))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM5 | RCC_APB1Periph_TIM12 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM8 | RCC_APB2Periph_TIM9 | RCC_APB2Periph_TIM11)

#endif

#if defined(REVONANO)
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
	{ TIM2, GPIOB, Pin_10, TIM_Channel_3, TIM2_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource10, GPIO_AF_TIM2},   
	{ TIM3, GPIOB, Pin_1,  TIM_Channel_4, TIM3_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM3},    
	{ TIM3, GPIOB, Pin_0,  TIM_Channel_3, TIM3_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM3},    
	{ TIM3, GPIOA, Pin_7,  TIM_Channel_2, TIM3_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource7, GPIO_AF_TIM3},    
	{ TIM3, GPIOA, Pin_6,  TIM_Channel_1, TIM3_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource6, GPIO_AF_TIM3},    
	{ TIM2, GPIOA, Pin_5,  TIM_Channel_1, TIM2_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource5, GPIO_AF_TIM2},    

	{ TIM1, GPIOA, Pin_10, TIM_Channel_3, TIM1_CC_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource10, GPIO_AF_TIM1}, 
    { TIM2, GPIOB, Pin_3,  TIM_Channel_2, TIM2_IRQn,    1, GPIO_Mode_AF, GPIO_PinSource3, GPIO_AF_TIM2},  
    { TIM4, GPIOB, Pin_8,  TIM_Channel_3, TIM4_IRQn,    1, GPIO_Mode_AF, GPIO_PinSource8, GPIO_AF_TIM4},  
    { TIM4, GPIOB, Pin_9,  TIM_Channel_4, TIM4_IRQn,    1, GPIO_Mode_AF, GPIO_PinSource9, GPIO_AF_TIM4},  
    { TIM5, GPIOA, Pin_0,  TIM_Channel_1, TIM5_IRQn,    1, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM5},  
    { TIM5, GPIOA, Pin_1,  TIM_Channel_2, TIM5_IRQn,    1, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM5},  
};

#define USED_TIMERS  ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(11) )

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM5 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM11)

#endif

#if defined(SPARKY2)
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM8, GPIOC, Pin_7, TIM_Channel_2, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource7, GPIO_AF_TIM8}, 
    { TIM12, GPIOB, Pin_14, TIM_Channel_1, TIM8_BRK_TIM12_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource14, GPIO_AF_TIM12}, 
    { TIM12, GPIOB, Pin_15, TIM_Channel_2, TIM8_BRK_TIM12_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource15, GPIO_AF_TIM12}, 
    { TIM8, GPIOC, Pin_8, TIM_Channel_3, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource8, GPIO_AF_TIM8}, 
    { TIM8, GPIOC, Pin_9, TIM_Channel_4, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource9, GPIO_AF_TIM8}, 

    { TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM3},    
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM3},    
    { TIM9, GPIOA, Pin_3, TIM_Channel_2, TIM1_BRK_TIM9_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource3, GPIO_AF_TIM9},    
    { TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource2, GPIO_AF_TIM2}, 
    { TIM5, GPIOA, Pin_1, TIM_Channel_2, TIM5_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM5},    
    { TIM5, GPIOA, Pin_0, TIM_Channel_1, TIM5_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM5},    
};

#define USED_TIMERS  ( TIM_N(2) | TIM_N(3) | TIM_N(5) | TIM_N(12) | TIM_N(8) | TIM_N(9) | TIM_N(11))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM5 | RCC_APB1Periph_TIM12 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM8 | RCC_APB2Periph_TIM9 | RCC_APB2Periph_TIM11)

#endif

#if defined(BLUEJAYF4)
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM8, GPIOC, Pin_7, TIM_Channel_2, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource7, GPIO_AF_TIM8},          
    { TIM5, GPIOA, Pin_0, TIM_Channel_1, TIM5_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM5},             
    { TIM5, GPIOA, Pin_1, TIM_Channel_2, TIM5_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM5},             
    { TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource2, GPIO_AF_TIM2},             
    { TIM9, GPIOA, Pin_3, TIM_Channel_2, TIM1_BRK_TIM9_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource3, GPIO_AF_TIM9},    
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM3},             
    { TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM3},             
  };

#define USED_TIMERS  ( TIM_N(2) | TIM_N(3) | TIM_N(5) | TIM_N(8) | TIM_N(9) | TIM_N(11))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM5 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM8 | RCC_APB2Periph_TIM9 | RCC_APB2Periph_TIM11)

#endif

#if defined(BLUEJAYF4_REV3)
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM8, GPIOC, Pin_7, TIM_Channel_2, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource7, GPIO_AF_TIM8},          
    { TIM5, GPIOA, Pin_0, TIM_Channel_1, TIM5_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM5},             
    { TIM5, GPIOA, Pin_1, TIM_Channel_2, TIM5_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM5},             
    { TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource2, GPIO_AF_TIM2},             
    { TIM9, GPIOA, Pin_3, TIM_Channel_2, TIM1_BRK_TIM9_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource3, GPIO_AF_TIM9},    
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM3},             
    { TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM3},             
  };

#define USED_TIMERS  ( TIM_N(2) | TIM_N(3) | TIM_N(5) | TIM_N(8) | TIM_N(9) | TIM_N(11))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM5 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM8 | RCC_APB2Periph_TIM9 | RCC_APB2Periph_TIM11)

#endif

#if defined(KISS)
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1,  GPIOA, Pin_8,  TIM_Channel_1, TIM1_CC_IRQn,            1, Mode_AF_PP, GPIO_PinSource8,  GPIO_AF_6,  1},
    { TIM3,  GPIOB, Pin_0,  TIM_Channel_3, TIM3_IRQn,               1, Mode_AF_PP, GPIO_PinSource0,  GPIO_AF_2,  1},
    { TIM15, GPIOB, Pin_14, TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP, GPIO_PinSource14, GPIO_AF_1,  1},
    { TIM15, GPIOB, Pin_15, TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP, GPIO_PinSource15, GPIO_AF_1,  1},
    { TIM16, GPIOA, Pin_6,  TIM_Channel_1, TIM1_UP_TIM16_IRQn,      1, Mode_AF_PP, GPIO_PinSource6,  GPIO_AF_1,  1},
    { TIM17, GPIOA, Pin_7,  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, Mode_AF_PP, GPIO_PinSource7,  GPIO_AF_1,  1},

    { TIM2,  GPIOB, Pin_3,  TIM_Channel_2, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource3,  GPIO_AF_1,  0},
    { TIM2,  GPIOA, Pin_15, TIM_Channel_1, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource15, GPIO_AF_1,  0},
    { TIM2,  GPIOA, Pin_2,  TIM_Channel_3, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource2,  GPIO_AF_1,  0},
    { TIM2,  GPIOB, Pin_11, TIM_Channel_4, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource11, GPIO_AF_1,  0},
    { TIM4,  GPIOA, Pin_13, TIM_Channel_2, TIM4_IRQn,               0, Mode_AF_PP, GPIO_PinSource13, GPIO_AF_10, 0},
    { TIM8,  GPIOA, Pin_14, TIM_Channel_3, TIM8_CC_IRQn,            0, Mode_AF_PP, GPIO_PinSource14, GPIO_AF_5,  0},
};

#define USED_TIMERS  (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(15) | TIM_N(16) | TIM_N(17))
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM15 | RCC_APB2Periph_TIM17 | RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM8)

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3  | RCC_APB1Periph_TIM4)
#define TIMER_AHB_PERIPHERALS (RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB)

#endif

#if defined(ALIENFLIGHTF4)
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1, GPIOA, Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource8, GPIO_AF_TIM1},            
    { TIM1, GPIOB, Pin_0, TIM_Channel_2, TIM1_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM1},            
    { TIM1, GPIOB, Pin_1, TIM_Channel_3, TIM1_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM1},            
    { TIM8, GPIOB, Pin_14, TIM_Channel_2, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource14, GPIO_AF_TIM8},          
    { TIM8, GPIOB, Pin_15, TIM_Channel_3, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource15, GPIO_AF_TIM8},          

    { TIM4, GPIOB, Pin_8, TIM_Channel_3, TIM4_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource8, GPIO_AF_TIM4},               
    { TIM4, GPIOB, Pin_9, TIM_Channel_4, TIM4_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource9, GPIO_AF_TIM4},               
    { TIM5, GPIOA, Pin_0, TIM_Channel_1, TIM5_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM5},               
    { TIM5, GPIOA, Pin_1, TIM_Channel_2, TIM5_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM5},               
    { TIM3, GPIOC, Pin_6, TIM_Channel_1, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource6, GPIO_AF_TIM3},               
    { TIM3, GPIOC, Pin_7, TIM_Channel_2, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource7, GPIO_AF_TIM3},               
    { TIM3, GPIOC, Pin_8, TIM_Channel_3, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource8, GPIO_AF_TIM3},               
    { TIM3, GPIOC, Pin_9, TIM_Channel_4, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource9, GPIO_AF_TIM3},               
};

#define USED_TIMERS  ( TIM_N(1) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(11))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM5 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8 | RCC_APB2Periph_TIM11)

#endif

#ifdef VRCORE
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1, GPIOE, Pin_9, TIM_Channel_1, TIM1_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource9, GPIO_AF_TIM1},       
    { TIM1, GPIOE, Pin_11,TIM_Channel_2, TIM1_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource11, GPIO_AF_TIM1},      
    { TIM1, GPIOE, Pin_13,TIM_Channel_3, TIM1_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource13, GPIO_AF_TIM1},      
    { TIM1, GPIOE, Pin_14,TIM_Channel_4, TIM1_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource14, GPIO_AF_TIM1},      
    { TIM9, GPIOE, Pin_6, TIM_Channel_1, TIM1_BRK_TIM9_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource6, GPIO_AF_TIM9}, 
    { TIM9, GPIOE, Pin_5, TIM_Channel_2, TIM1_BRK_TIM9_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource5, GPIO_AF_TIM9}, 

    { TIM2, GPIOA, Pin_1, TIM_Channel_2, TIM2_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM2},    
    { TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource2, GPIO_AF_TIM2},    
    { TIM2, GPIOA, Pin_3, TIM_Channel_4, TIM2_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource3, GPIO_AF_TIM2},    
    { TIM3, GPIOB, Pin_5, TIM_Channel_2, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource5, GPIO_AF_TIM3},    
    { TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM3},    
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM3},    
};

#define USED_TIMERS  ( TIM_N(1) | TIM_N(2) | TIM_N(4) | TIM_N(9) | TIM_N(11))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOE)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM9 | RCC_APB2Periph_TIM11)

#endif

#ifdef QUANTON
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1, GPIOA, Pin_10, TIM_Channel_3, TIM1_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource10, GPIO_AF_TIM1 },
    { TIM8, GPIOC, Pin_6, TIM_Channel_1, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource6, GPIO_AF_TIM8 },
    { TIM8, GPIOC, Pin_7, TIM_Channel_2, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource7, GPIO_AF_TIM8 },
    { TIM8, GPIOC, Pin_8, TIM_Channel_3, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource8, GPIO_AF_TIM8 },
    { TIM2, GPIOA, Pin_15, TIM_Channel_1, TIM2_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource15, GPIO_AF_TIM2 },
    { TIM2, GPIOB, Pin_3, TIM_Channel_2, TIM2_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource3, GPIO_AF_TIM2 },
    { TIM5, GPIOA, Pin_0, TIM_Channel_1, TIM5_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM5 },
    { TIM5, GPIOA, Pin_1, TIM_Channel_2, TIM5_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM5 },

    { TIM3, GPIOB, Pin_4, TIM_Channel_1, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource4, GPIO_AF_TIM3 },
    { TIM3, GPIOB, Pin_5, TIM_Channel_2, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource5, GPIO_AF_TIM3 },
    { TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM3 },
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM3 },
    { TIM12, GPIOB, Pin_14, TIM_Channel_1, TIM8_BRK_TIM12_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource14, GPIO_AF_TIM12 },
    { TIM12, GPIOB, Pin_15, TIM_Channel_2, TIM8_BRK_TIM12_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource15, GPIO_AF_TIM12 },
    { TIM4, GPIOB, Pin_8, TIM_Channel_3, TIM4_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource8, GPIO_AF_TIM4 },
    { TIM4, GPIOB, Pin_9, TIM_Channel_4, TIM4_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource9, GPIO_AF_TIM4 },
};

#define USED_TIMERS  ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(5) | TIM_N(12) | TIM_N(8) | TIM_N(9) | TIM_N(11))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM5 | RCC_APB1Periph_TIM12 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8 | RCC_APB2Periph_TIM9 | RCC_APB2Periph_TIM11)

#endif

#if defined(KKNGF4)
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
	{ TIM8, GPIOC, Pin_9, TIM_Channel_4, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource9, GPIO_AF_TIM8},          

	{ TIM9, GPIOA, Pin_3, TIM_Channel_2, TIM1_BRK_TIM9_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource3, GPIO_AF_TIM9},    
	{ TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM3},             
	{ TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM3},             
	{ TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource2, GPIO_AF_TIM2},             



};

#define USED_TIMERS  ( TIM_N(2) | TIM_N(3) | TIM_N(8) | TIM_N(9) | TIM_N(11))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM8 | RCC_APB2Periph_TIM9 | RCC_APB2Periph_TIM11)

#endif

#if defined(HOOLIGAN)
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
	{ TIM12, GPIOB, Pin_15, TIM_Channel_1, TIM8_BRK_TIM12_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource15, GPIO_AF_TIM12 },    

	{ TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM3 },    
	{ TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM3 },    
	{ TIM9, GPIOA, Pin_3, TIM_Channel_2, TIM1_BRK_TIM9_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource3, GPIO_AF_TIM9 },    
	{ TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource2, GPIO_AF_TIM2 }, 
	{ TIM5, GPIOA, Pin_1, TIM_Channel_2, TIM5_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM5 },    
	{ TIM5, GPIOA, Pin_0, TIM_Channel_1, TIM5_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM5 },    
	{ TIM12, GPIOB, Pin_14, TIM_Channel_1, TIM8_BRK_TIM12_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource14, GPIO_AF_TIM12 },    
};

#define USED_TIMERS  ( TIM_N(2) | TIM_N(3) | TIM_N(5) | TIM_N(12) | TIM_N(9) | TIM_N(11))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM5 | RCC_APB1Periph_TIM12 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB )
#define TIMER_APB2_PERIPHERALS ( RCC_APB2Periph_TIM9 | RCC_APB2Periph_TIM11)

#endif

#ifdef SPRACINGF3
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM2,  GPIOA, Pin_0,  TIM_Channel_1, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource0,  GPIO_AF_1, 0}, 
    { TIM2,  GPIOA, Pin_1,  TIM_Channel_2, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource1,  GPIO_AF_1, 0}, 
    
    { TIM2,  GPIOB, Pin_11, TIM_Channel_4, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource11, GPIO_AF_1, 0}, 
    { TIM2,  GPIOB, Pin_10, TIM_Channel_3, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource10, GPIO_AF_1, 0}, 
    { TIM3,  GPIOB, Pin_4,  TIM_Channel_1, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource4,  GPIO_AF_2, 0}, 
    { TIM3,  GPIOB, Pin_5,  TIM_Channel_2, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource5,  GPIO_AF_2, 0}, 
    { TIM3,  GPIOB, Pin_0,  TIM_Channel_3, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource0,  GPIO_AF_2, 0}, 
    { TIM3,  GPIOB, Pin_1,  TIM_Channel_4, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource1,  GPIO_AF_2, 0}, 

    { TIM16, GPIOA, Pin_6,  TIM_Channel_1, TIM1_UP_TIM16_IRQn,      1, Mode_AF_PP, GPIO_PinSource6,  GPIO_AF_1, 0},  
    { TIM17, GPIOA, Pin_7,  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, Mode_AF_PP, GPIO_PinSource7,  GPIO_AF_1, 0},  
    { TIM4,  GPIOA, Pin_11, TIM_Channel_1, TIM4_IRQn,               1, Mode_AF_PP, GPIO_PinSource11, GPIO_AF_10, 0}, 
    { TIM4,  GPIOA, Pin_12, TIM_Channel_2, TIM4_IRQn,               1, Mode_AF_PP, GPIO_PinSource12, GPIO_AF_10, 0}, 
    { TIM4,  GPIOB, Pin_8,  TIM_Channel_3, TIM4_IRQn,               1, Mode_AF_PP, GPIO_PinSource8,  GPIO_AF_2, 0},  
    { TIM4,  GPIOB, Pin_9,  TIM_Channel_4, TIM4_IRQn,               1, Mode_AF_PP, GPIO_PinSource9,  GPIO_AF_2, 0},  
    { TIM15, GPIOA, Pin_2,  TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP, GPIO_PinSource2,  GPIO_AF_9, 0},  
    { TIM15, GPIOA, Pin_3,  TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP, GPIO_PinSource3,  GPIO_AF_9, 0},  

    { TIM1,  GPIOA, Pin_8,  TIM_Channel_1, TIM1_CC_IRQn,            1, Mode_AF_PP, GPIO_PinSource8,  GPIO_AF_6, 0},  
};

#define USED_TIMERS  (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(15) | TIM_N(16) |TIM_N(17))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM15 | RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM17)
#define TIMER_AHB_PERIPHERALS (RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB)

#endif

#if defined(MOTOLAB)
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM3,  GPIOA, Pin_4,  TIM_Channel_2, TIM3_IRQn,               1, Mode_AF_PP, GPIO_PinSource4,  GPIO_AF_2, 0}, 
    { TIM3,  GPIOA, Pin_6,  TIM_Channel_1, TIM3_IRQn,               1, Mode_AF_PP, GPIO_PinSource6,  GPIO_AF_2, 0}, 
    { TIM3,  GPIOB, Pin_0,  TIM_Channel_3, TIM3_IRQn,               1, Mode_AF_PP, GPIO_PinSource0,  GPIO_AF_2, 0}, 
    { TIM3,  GPIOB, Pin_1,  TIM_Channel_4, TIM3_IRQn,               1, Mode_AF_PP, GPIO_PinSource1,  GPIO_AF_2, 0}, 
    { TIM2,  GPIOA, Pin_1,  TIM_Channel_2, TIM2_IRQn,               1, Mode_AF_PP, GPIO_PinSource1,  GPIO_AF_1, 0}, 
    { TIM2,  GPIOA, Pin_2,  TIM_Channel_3, TIM2_IRQn,               1, Mode_AF_PP, GPIO_PinSource2,  GPIO_AF_1, 0}, 
    { TIM15, GPIOA, Pin_3,  TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP, GPIO_PinSource3,  GPIO_AF_9, 0}, 
    { TIM1,  GPIOA, Pin_8,  TIM_Channel_1, TIM1_CC_IRQn,            1, Mode_AF_PP, GPIO_PinSource8,  GPIO_AF_6, 0}, 

    { TIM17, GPIOA, Pin_7,  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 0, Mode_AF_PP_PD, GPIO_PinSource7,  GPIO_AF_1, 0}, 

};

#define USED_TIMERS  (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(15) | TIM_N(17))
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM15 | RCC_APB2Periph_TIM17)

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3)
#define TIMER_AHB_PERIPHERALS (RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB)

#endif

#if defined(PIKOBLX)
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM3,  GPIOA, Pin_4,  TIM_Channel_2, TIM3_IRQn,               1, Mode_AF_PP, GPIO_PinSource4,  GPIO_AF_2, 0}, 
    { TIM3,  GPIOA, Pin_6,  TIM_Channel_1, TIM3_IRQn,               1, Mode_AF_PP, GPIO_PinSource6,  GPIO_AF_2, 0}, 
    { TIM3,  GPIOB, Pin_0,  TIM_Channel_3, TIM3_IRQn,               1, Mode_AF_PP, GPIO_PinSource0,  GPIO_AF_2, 0}, 
    { TIM3,  GPIOB, Pin_1,  TIM_Channel_4, TIM3_IRQn,               1, Mode_AF_PP, GPIO_PinSource1,  GPIO_AF_2, 0}, 
    { TIM2,  GPIOA, Pin_1,  TIM_Channel_2, TIM2_IRQn,               1, Mode_AF_PP, GPIO_PinSource1,  GPIO_AF_1, 0}, 
    { TIM2,  GPIOA, Pin_2,  TIM_Channel_3, TIM2_IRQn,               1, Mode_AF_PP, GPIO_PinSource2,  GPIO_AF_1, 0}, 
    { TIM15, GPIOA, Pin_3,  TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP, GPIO_PinSource3,  GPIO_AF_9, 0}, 
    { TIM1,  GPIOA, Pin_8,  TIM_Channel_1, TIM1_CC_IRQn,            1, Mode_AF_PP, GPIO_PinSource8,  GPIO_AF_6, 0}, 

    { TIM17, GPIOA, Pin_7,  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 0, Mode_AF_PP_PD, GPIO_PinSource7,  GPIO_AF_1, 0}, 

};

#define USED_TIMERS  (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(15) | TIM_N(17))
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM15 | RCC_APB2Periph_TIM17)

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3)
#define TIMER_AHB_PERIPHERALS (RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB)

#endif

#if defined(PRACER)

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM3,  GPIOB, Pin_0,  TIM_Channel_3, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource0,  GPIO_AF_TIM3},
	{ TIM3,  GPIOE, Pin_14, TIM_Channel_4, TIM3_IRQn,               1, Mode_AF_PP, GPIO_PinSource14, GPIO_AF_TIM3},
    { TIM3,  GPIOE, Pin_13, TIM_Channel_3, TIM3_IRQn,               1, Mode_AF_PP, GPIO_PinSource13, GPIO_AF_TIM3},
    { TIM3,  GPIOE, Pin_11, TIM_Channel_2, TIM3_IRQn,               1, Mode_AF_PP, GPIO_PinSource11, GPIO_AF_TIM3},
    { TIM2,  GPIOE, Pin_9,  TIM_Channel_1, TIM2_IRQn,               1, Mode_AF_PP, GPIO_PinSource9,  GPIO_AF_TIM2},
    { TIM2,  GPIOD, Pin_13, TIM_Channel_2, TIM2_IRQn,               1, Mode_AF_PP, GPIO_PinSource13, GPIO_AF_TIM2},
    

};

#define USED_TIMERS ( TIM_N(2) | TIM_N(3) )
#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3)


#endif

#define USED_TIMER_COUNT BITCOUNT(USED_TIMERS)
#define CC_CHANNELS_PER_TIMER 4              

#define TIM_IT_CCx(ch) (TIM_IT_CC1 << ((ch) / 4))

typedef struct timerConfig_s {
    timerCCHandlerRec_t *edgeCallback[CC_CHANNELS_PER_TIMER];
    timerOvrHandlerRec_t *overflowCallback[CC_CHANNELS_PER_TIMER];
    timerOvrHandlerRec_t *overflowCallbackActive; 
	uint32_t forcedOverflowTimerValue;
} timerConfig_t;
timerConfig_t timerConfig[USED_TIMER_COUNT];

typedef struct {
    channelType_t type;
} timerChannelInfo_t;
timerChannelInfo_t timerChannelInfo[USABLE_TIMER_CHANNEL_COUNT];

typedef struct {
    uint8_t priority;
} timerInfo_t;
timerInfo_t timerInfo[USED_TIMER_COUNT];


#define TIMER_INDEX(i) BITCOUNT((TIM_N(i) - 1) & USED_TIMERS)

static uint8_t lookupTimerIndex(const TIM_TypeDef *tim)
{
#define _CASE_SHF 10           
#define _CASE_(tim, index) case ((unsigned)tim >> _CASE_SHF): return index; break
#define _CASE(i) _CASE_(TIM##i##_BASE, TIMER_INDEX(i))


    switch((unsigned)tim >> _CASE_SHF) {
#if USED_TIMERS & TIM_N(1)
        _CASE(1);
#endif
#if USED_TIMERS & TIM_N(2)
        _CASE(2);
#endif
#if USED_TIMERS & TIM_N(3)
        _CASE(3);
#endif
#if USED_TIMERS & TIM_N(4)
        _CASE(4);
#endif
#if USED_TIMERS & TIM_N(5)
        _CASE(5);
#endif
#if USED_TIMERS & TIM_N(6)
        _CASE(6);
#endif
#if USED_TIMERS & TIM_N(7)
        _CASE(7);
#endif
#if USED_TIMERS & TIM_N(8)
        _CASE(8);
#endif
#if USED_TIMERS & TIM_N(9)
        _CASE(9);
#endif
#if USED_TIMERS & TIM_N(10)
        _CASE(10);
#endif
#if USED_TIMERS & TIM_N(11)
        _CASE(11);
#endif
#if USED_TIMERS & TIM_N(12)
        _CASE(12);
#endif
#if USED_TIMERS & TIM_N(13)
        _CASE(13);
#endif
#if USED_TIMERS & TIM_N(14)
        _CASE(14);
#endif
#if USED_TIMERS & TIM_N(15)
        _CASE(15);
#endif
#if USED_TIMERS & TIM_N(16)
        _CASE(16);
#endif
#if USED_TIMERS & TIM_N(17)
        _CASE(17);
#endif
    default:  return ~1;  
    }
#undef _CASE
#undef _CASE_
}

TIM_TypeDef * const usedTimers[USED_TIMER_COUNT] = {
#define _DEF(i) TIM##i

#if USED_TIMERS & TIM_N(1)
    _DEF(1),
#endif
#if USED_TIMERS & TIM_N(2)
    _DEF(2),
#endif
#if USED_TIMERS & TIM_N(3)
    _DEF(3),
#endif
#if USED_TIMERS & TIM_N(4)
    _DEF(4),
#endif
#if USED_TIMERS & TIM_N(5)
    _DEF(5),
#endif
#if USED_TIMERS & TIM_N(6)
    _DEF(6),
#endif
#if USED_TIMERS & TIM_N(7)
    _DEF(7),
#endif
#if USED_TIMERS & TIM_N(8)
    _DEF(8),
#endif
#if USED_TIMERS & TIM_N(9)
    _DEF(9),
#endif
#if USED_TIMERS & TIM_N(10)
    _DEF(10),
#endif
#if USED_TIMERS & TIM_N(11)
    _DEF(11),
#endif
#if USED_TIMERS & TIM_N(12)
    _DEF(12),
#endif
#if USED_TIMERS & TIM_N(13)
    _DEF(13),
#endif
#if USED_TIMERS & TIM_N(14)
    _DEF(14),
#endif
#if USED_TIMERS & TIM_N(15)
    _DEF(15),
#endif
#if USED_TIMERS & TIM_N(16)
    _DEF(16),
#endif
#if USED_TIMERS & TIM_N(17)
    _DEF(17),
#endif
#undef _DEF
};

static inline uint8_t lookupChannelIndex(const uint16_t channel)
{
    return channel >> 2;
}

void timerNVICConfigure(uint8_t irq)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER);

    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void configTimeBase(TIM_TypeDef *tim, uint16_t period, uint8_t mhz)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = (period - 1) & 0xffff; 

    
    
#if defined (STM32F40_41xxx) || defined(STM32F446xx)
    if(tim == TIM1 || tim == TIM8 || tim == TIM9|| tim == TIM10|| tim == TIM11) {
    	TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / ((uint32_t)mhz * 1000000)) - 1;
    } else {
		TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 2 / ((uint32_t)mhz * 1000000)) - 1;
    }
#elif defined (STM32F411xE)
    if(tim == TIM1 || tim == TIM9 || tim == TIM10 || tim == TIM11) {
		TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / ((uint32_t)mhz * 1000000)) - 1;
    } else {
#if !defined (REVONANO_VB)
		TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / ((uint32_t)mhz * 1000000)) - 1; 
#else
	    TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 2 / ((uint32_t)mhz * 1000000)) - 1; 
#endif
    }
#else
    TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / ((uint32_t)mhz * 1000000)) - 1;
#endif

    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);
}


void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint8_t mhz)
{
    configTimeBase(timerHardwarePtr->tim, period, mhz);
    TIM_Cmd(timerHardwarePtr->tim, ENABLE);
    timerNVICConfigure(timerHardwarePtr->irq);
    
    switch(timerHardwarePtr->irq) {
#if defined(STM32F10X)
    case TIM1_CC_IRQn:
        timerNVICConfigure(TIM1_UP_IRQn);
        break;
#endif
#if defined (STM32F40_41xxx) || defined(STM32F446xx)
    case TIM1_CC_IRQn:
        timerNVICConfigure(TIM1_UP_TIM10_IRQn);
        break;
    case TIM8_CC_IRQn:
        timerNVICConfigure(TIM8_UP_TIM13_IRQn);
        break;
#endif
#if defined (STM32F411xE)
    case TIM1_CC_IRQn:
        timerNVICConfigure(TIM1_UP_TIM10_IRQn);
        break;
#endif
#ifdef STM32F303xC
    case TIM1_CC_IRQn:
        timerNVICConfigure(TIM1_UP_TIM16_IRQn);
        break;
#endif
#if defined(STM32F10X_XL)
    case TIM8_CC_IRQn:
        timerNVICConfigure(TIM8_UP_IRQn);
        break;
#endif
    }
}


void timerChInit(const timerHardware_t *timHw, channelType_t type, int irqPriority)
{
    unsigned channel = timHw - timerHardware;
    if(channel >= USABLE_TIMER_CHANNEL_COUNT)
        return;

    timerChannelInfo[channel].type = type;
    unsigned timer = lookupTimerIndex(timHw->tim);
    if(timer >= USED_TIMER_COUNT)
        return;
    if(irqPriority < timerInfo[timer].priority) {
        
        configTimeBase(usedTimers[timer], 0, 1);
        TIM_Cmd(usedTimers[timer],  ENABLE);

        NVIC_InitTypeDef NVIC_InitStructure;

        NVIC_InitStructure.NVIC_IRQChannel = timHw->irq;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(irqPriority);
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(irqPriority);
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        timerInfo[timer].priority = irqPriority;
    }
}

void timerChCCHandlerInit(timerCCHandlerRec_t *self, timerCCHandlerCallback *fn)
{
    self->fn = fn;
}

void timerChOvrHandlerInit(timerOvrHandlerRec_t *self, timerOvrHandlerCallback *fn)
{
    self->fn = fn;
    self->next = NULL;
}



static void timerChConfig_UpdateOverflow(timerConfig_t *cfg, TIM_TypeDef *tim) {
    timerOvrHandlerRec_t **chain = &cfg->overflowCallbackActive;
    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        for(int i = 0; i < CC_CHANNELS_PER_TIMER; i++)
            if(cfg->overflowCallback[i]) {
                *chain = cfg->overflowCallback[i];
                chain = &cfg->overflowCallback[i]->next;
            }
        *chain = NULL;
    }
    
    TIM_ITConfig(tim, TIM_IT_Update, cfg->overflowCallbackActive ? ENABLE : DISABLE);
}


void timerChConfigCallbacks(const timerHardware_t *timHw, timerCCHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback)
{
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }
    uint8_t channelIndex = lookupChannelIndex(timHw->channel);
    if(edgeCallback == NULL)   
        TIM_ITConfig(timHw->tim, TIM_IT_CCx(timHw->channel), DISABLE);
    
    timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallback;
    timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;
    
    if(edgeCallback)
        TIM_ITConfig(timHw->tim, TIM_IT_CCx(timHw->channel), ENABLE);

    timerChConfig_UpdateOverflow(&timerConfig[timerIndex], timHw->tim);
}




void timerChConfigCallbacksDual(const timerHardware_t *timHw, timerCCHandlerRec_t *edgeCallbackLo, timerCCHandlerRec_t *edgeCallbackHi, timerOvrHandlerRec_t *overflowCallback)
{
    uint8_t timerIndex = lookupTimerIndex(timHw->tim);
    if (timerIndex >= USED_TIMER_COUNT) {
        return;
    }
    uint16_t chLo = timHw->channel & ~TIM_Channel_2;   
    uint16_t chHi = timHw->channel | TIM_Channel_2;    
    uint8_t channelIndex = lookupChannelIndex(chLo);   

    if(edgeCallbackLo == NULL)   
        TIM_ITConfig(timHw->tim, TIM_IT_CCx(chLo), DISABLE);
    if(edgeCallbackHi == NULL)   
        TIM_ITConfig(timHw->tim, TIM_IT_CCx(chHi), DISABLE);

    
    timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallbackLo;
    timerConfig[timerIndex].edgeCallback[channelIndex + 1] = edgeCallbackHi;
    timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;
    timerConfig[timerIndex].overflowCallback[channelIndex + 1] = NULL;

    
    if(edgeCallbackLo) {
        TIM_ClearFlag(timHw->tim, TIM_IT_CCx(chLo));
        TIM_ITConfig(timHw->tim, TIM_IT_CCx(chLo), ENABLE);
    }
    if(edgeCallbackHi) {
        TIM_ClearFlag(timHw->tim, TIM_IT_CCx(chHi));
        TIM_ITConfig(timHw->tim, TIM_IT_CCx(chHi), ENABLE);
    }

    timerChConfig_UpdateOverflow(&timerConfig[timerIndex], timHw->tim);
}


void timerChITConfigDualLo(const timerHardware_t *timHw, FunctionalState newState) {
    TIM_ITConfig(timHw->tim, TIM_IT_CCx(timHw->channel&~TIM_Channel_2), newState);
}


void timerChITConfig(const timerHardware_t *timHw, FunctionalState newState)
{
    TIM_ITConfig(timHw->tim, TIM_IT_CCx(timHw->channel), newState);
}


void timerChClearCCFlag(const timerHardware_t *timHw)
{
    TIM_ClearFlag(timHw->tim, TIM_IT_CCx(timHw->channel));
}


void timerChConfigGPIO(const timerHardware_t *timHw, GPIO_Mode mode)
{
    gpio_config_t cfg;

    cfg.pin = timHw->pin;
    cfg.mode = mode;
    cfg.speed = Speed_2MHz;
    gpioInit(timHw->gpio, &cfg);
}




static unsigned getFilter(unsigned ticks)
{
    static const unsigned ftab[16] = {
        1*1,                 
        1*2, 1*4, 1*8,       
        2*6, 2*8,            
        4*6, 4*8,
        8*6, 8*8,
        16*5, 16*6, 16*8,
        32*5, 32*6, 32*8
    };
    for(unsigned i = 1; i < ARRAYLEN(ftab); i++)
        if(ftab[i] > ticks)
            return i - 1;
    return 0x0f;
}


void timerChConfigIC(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterTicks)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = timHw->channel;
    TIM_ICInitStructure.TIM_ICPolarity = polarityRising ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = getFilter(inputFilterTicks);

    TIM_ICInit(timHw->tim, &TIM_ICInitStructure);
}



void timerChConfigICDual(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterTicks)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;
    bool directRising = (timHw->channel & TIM_Channel_2) ? !polarityRising : polarityRising;
    
    TIM_ICStructInit(&TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel = timHw->channel;
    TIM_ICInitStructure.TIM_ICPolarity = directRising ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = getFilter(inputFilterTicks);
    TIM_ICInit(timHw->tim, &TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel = timHw->channel ^ TIM_Channel_2;   
    TIM_ICInitStructure.TIM_ICPolarity = directRising ? TIM_ICPolarity_Falling : TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_IndirectTI;
    TIM_ICInit(timHw->tim, &TIM_ICInitStructure);
}

void timerChICPolarity(const timerHardware_t *timHw, bool polarityRising)
{
    timCCER_t tmpccer = timHw->tim->CCER;
    tmpccer &= ~(TIM_CCER_CC1P << timHw->channel);
    tmpccer |= polarityRising ? (TIM_ICPolarity_Rising << timHw->channel) : (TIM_ICPolarity_Falling << timHw->channel);
    timHw->tim->CCER = tmpccer;
}

volatile timCCR_t* timerChCCRHi(const timerHardware_t *timHw)
{
    return (volatile timCCR_t*)((volatile char*)&timHw->tim->CCR1 + (timHw->channel | TIM_Channel_2));
}

volatile timCCR_t* timerChCCRLo(const timerHardware_t *timHw)
{
    return (volatile timCCR_t*)((volatile char*)&timHw->tim->CCR1 + (timHw->channel & ~TIM_Channel_2));
}



volatile timCCR_t* timerChCCR(const timerHardware_t *timHw)
{
    return (volatile timCCR_t*)((volatile char*)&timHw->tim->CCR1 + timHw->channel);
}

void timerChConfigOC(const timerHardware_t* timHw, bool outEnable, bool stateHigh)
{
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    if(outEnable) {
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Inactive;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
#ifdef STM32F303
        if (timHw->outputInverted) {
         	TIM_OCInitStructure.TIM_OCPolarity = stateHigh ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
        } else {
         	TIM_OCInitStructure.TIM_OCPolarity = stateHigh ? TIM_OCPolarity_High : TIM_OCPolarity_Low;
        }
#else
        TIM_OCInitStructure.TIM_OCPolarity = stateHigh ? TIM_OCPolarity_High : TIM_OCPolarity_Low;
#endif
    } else {
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
    }

    switch (timHw->channel) {
    case TIM_Channel_1:
        TIM_OC1Init(timHw->tim, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(timHw->tim, TIM_OCPreload_Disable);
        break;
    case TIM_Channel_2:
        TIM_OC2Init(timHw->tim, &TIM_OCInitStructure);
        TIM_OC2PreloadConfig(timHw->tim, TIM_OCPreload_Disable);
        break;
    case TIM_Channel_3:
        TIM_OC3Init(timHw->tim, &TIM_OCInitStructure);
        TIM_OC3PreloadConfig(timHw->tim, TIM_OCPreload_Disable);
        break;
    case TIM_Channel_4:
        TIM_OC4Init(timHw->tim, &TIM_OCInitStructure);
        TIM_OC4PreloadConfig(timHw->tim, TIM_OCPreload_Disable);
        break;
    }
}



static void timCCxHandler(TIM_TypeDef *tim, timerConfig_t *timerConfig)
{
    uint16_t capture;
    unsigned tim_status;
    tim_status = tim->SR & tim->DIER;
#if 1
    while(tim_status) {
        
        
        unsigned bit = __builtin_clz(tim_status);
        unsigned mask = ~(0x80000000 >> bit);
        tim->SR = mask;
        tim_status &= mask;
        switch(bit) {
            case __builtin_clz(TIM_IT_Update): {

                if(timerConfig->forcedOverflowTimerValue != 0){
                    capture = timerConfig->forcedOverflowTimerValue - 1;
                    timerConfig->forcedOverflowTimerValue = 0;
                } else {
                    capture = tim->ARR;
                }

                timerOvrHandlerRec_t *cb = timerConfig->overflowCallbackActive;
                while(cb) {
                    cb->fn(cb, capture);
                    cb = cb->next;
                }
                break;
            }
            case __builtin_clz(TIM_IT_CC1):
                timerConfig->edgeCallback[0]->fn(timerConfig->edgeCallback[0], tim->CCR1);
                break;
            case __builtin_clz(TIM_IT_CC2):
                timerConfig->edgeCallback[1]->fn(timerConfig->edgeCallback[1], tim->CCR2);
                break;
            case __builtin_clz(TIM_IT_CC3):
                timerConfig->edgeCallback[2]->fn(timerConfig->edgeCallback[2], tim->CCR3);
                break;
            case __builtin_clz(TIM_IT_CC4):
                timerConfig->edgeCallback[3]->fn(timerConfig->edgeCallback[3], tim->CCR4);
                break;
        }
    }
#else
    if (tim_status & (int)TIM_IT_Update) {
        tim->SR = ~TIM_IT_Update;
        capture = tim->ARR;
        timerOvrHandlerRec_t *cb = timerConfig->overflowCallbackActive;
        while(cb) {
            cb->fn(cb, capture);
            cb = cb->next;
        }
    }
    if (tim_status & (int)TIM_IT_CC1) {
        tim->SR = ~TIM_IT_CC1;
        timerConfig->edgeCallback[0]->fn(timerConfig->edgeCallback[0], tim->CCR1);
    }
    if (tim_status & (int)TIM_IT_CC2) {
        tim->SR = ~TIM_IT_CC2;
        timerConfig->edgeCallback[1]->fn(timerConfig->edgeCallback[1], tim->CCR2);
    }
    if (tim_status & (int)TIM_IT_CC3) {
        tim->SR = ~TIM_IT_CC3;
        timerConfig->edgeCallback[2]->fn(timerConfig->edgeCallback[2], tim->CCR3);
    }
    if (tim_status & (int)TIM_IT_CC4) {
        tim->SR = ~TIM_IT_CC4;
        timerConfig->edgeCallback[3]->fn(timerConfig->edgeCallback[3], tim->CCR4);
    }
#endif
}


#define _TIM_IRQ_HANDLER2(name, i, j)                                   \
    void name(void)                                                     \
    {                                                                   \
        timCCxHandler(TIM ## i, &timerConfig[TIMER_INDEX(i)]);          \
        timCCxHandler(TIM ## j, &timerConfig[TIMER_INDEX(j)]);          \
    } struct dummy

#define _TIM_IRQ_HANDLER(name, i)                                       \
    void name(void)                                                     \
    {                                                                   \
        timCCxHandler(TIM ## i, &timerConfig[TIMER_INDEX(i)]);          \
    } struct dummy

#if USED_TIMERS & TIM_N(1)
_TIM_IRQ_HANDLER(TIM1_CC_IRQHandler, 1);
# if defined(STM32F10X)
_TIM_IRQ_HANDLER(TIM1_UP_IRQHandler, 1);       
# endif
# if defined(STM32F40_41xxx) || defined (STM32F411xE) || defined(STM32F446xx)
#  if USED_TIMERS & TIM_N(10)
_TIM_IRQ_HANDLER2(TIM1_UP_TIM10_IRQHandler, 1,10);  
#  else
_TIM_IRQ_HANDLER(TIM1_UP_TIM10_IRQHandler, 1);     
#  endif
# endif
# ifdef STM32F303xC
#  if USED_TIMERS & TIM_N(16)
_TIM_IRQ_HANDLER2(TIM1_UP_TIM16_IRQHandler, 1, 16);  
#  else
_TIM_IRQ_HANDLER(TIM1_UP_TIM16_IRQHandler, 1);       
#  endif
# endif
#endif
#if USED_TIMERS & TIM_N(2)
_TIM_IRQ_HANDLER(TIM2_IRQHandler, 2);
#endif
#if USED_TIMERS & TIM_N(3)
_TIM_IRQ_HANDLER(TIM3_IRQHandler, 3);
#endif
#if USED_TIMERS & TIM_N(4)
_TIM_IRQ_HANDLER(TIM4_IRQHandler, 4);
#endif
#if USED_TIMERS & TIM_N(5)
_TIM_IRQ_HANDLER(TIM5_IRQHandler, 5);
#endif
#if USED_TIMERS & TIM_N(8)
_TIM_IRQ_HANDLER(TIM8_CC_IRQHandler, 8);
# if defined(STM32F10X_XL)
_TIM_IRQ_HANDLER(TIM8_UP_TIM13_IRQHandler, 8);
# else  
_TIM_IRQ_HANDLER(TIM8_UP_IRQHandler, 8);
# endif
# if defined(STM32F40_41xxx) || defined(STM32F446xx)
#  if USED_TIMERS & TIM_N(13)
_TIM_IRQ_HANDLER2(TIM8_UP_TIM13_IRQHandler, 8,13);  
#  else
_TIM_IRQ_HANDLER(TIM8_UP_TIM13_IRQHandler, 8);     
#  endif
# endif
# if defined (STM32F411xE)
# endif
#endif
#if USED_TIMERS & TIM_N(9)
_TIM_IRQ_HANDLER(TIM1_BRK_TIM9_IRQHandler, 9);
#endif
#  if USED_TIMERS & TIM_N(11)

#  endif
#if USED_TIMERS & TIM_N(12)
_TIM_IRQ_HANDLER(TIM8_BRK_TIM12_IRQHandler, 12);
#endif
#if USED_TIMERS & TIM_N(15)
_TIM_IRQ_HANDLER(TIM1_BRK_TIM15_IRQHandler, 15);
#endif
#if defined(STM32F303xC) && ((USED_TIMERS & (TIM_N(1)|TIM_N(16))) == (TIM_N(16)))
_TIM_IRQ_HANDLER(TIM1_UP_TIM16_IRQHandler, 16);    
#endif
#if USED_TIMERS & TIM_N(17)

#endif

void timerInit(void)
{
    memset(timerConfig, 0, sizeof (timerConfig));

#ifdef CC3D
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
#endif

#ifdef TIMER_APB1_PERIPHERALS
    RCC_APB1PeriphClockCmd(TIMER_APB1_PERIPHERALS, ENABLE);
#endif

#ifdef TIMER_APB2_PERIPHERALS
    RCC_APB2PeriphClockCmd(TIMER_APB2_PERIPHERALS, ENABLE);
#endif

#ifdef TIMER_AHB_PERIPHERALS
    RCC_AHBPeriphClockCmd(TIMER_AHB_PERIPHERALS, ENABLE);
#endif

#ifdef STM32F303xC
    for (uint8_t timerIndex = 0; timerIndex < USABLE_TIMER_CHANNEL_COUNT; timerIndex++) {
        const timerHardware_t *timerHardwarePtr = &timerHardware[timerIndex];
        GPIO_PinAFConfig(timerHardwarePtr->gpio, (uint16_t)timerHardwarePtr->gpioPinSource, timerHardwarePtr->alternateFunction);
    }
#endif

#if defined(STM32F40_41xxx) || defined (STM32F411xE) || defined(STM32F446xx)
    for (uint8_t timerIndex = 0; timerIndex < USABLE_TIMER_CHANNEL_COUNT; timerIndex++) {
        const timerHardware_t *timerHardwarePtr = &timerHardware[timerIndex];
        GPIO_PinAFConfig(timerHardwarePtr->gpio, (uint16_t)timerHardwarePtr->gpioPinSource, timerHardwarePtr->alternateFunction);
    }
#endif


    for(int i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
        timerChannelInfo[i].type = TYPE_FREE;
    }
    for(int i = 0; i < USED_TIMER_COUNT; i++) {
        timerInfo[i].priority = ~0;
    }
}




void timerStart(void)
{
#if 0
    for(unsigned timer = 0; timer < USED_TIMER_COUNT; timer++) {
        int priority = -1;
        int irq = -1;
        for(unsigned hwc = 0; hwc < USABLE_TIMER_CHANNEL_COUNT; hwc++)
            if((timerChannelInfo[hwc].type != TYPE_FREE) && (timerHardware[hwc].tim == usedTimers[timer])) {
                
                irq = timerHardware[hwc].irq;
            }
        
        configTimeBase(usedTimers[timer], 0, 1);
        TIM_Cmd(usedTimers[timer],  ENABLE);
        if(priority >= 0) {  
            NVIC_InitTypeDef NVIC_InitStructure;

            NVIC_InitStructure.NVIC_IRQChannel = irq;
            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_SPLIT_PRIORITY_BASE(priority);
            NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_SPLIT_PRIORITY_SUB(priority);
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&NVIC_InitStructure);
        }
    }
#endif
}

/**
 * Force an overflow for a given timer.
 * Saves the current value of the counter in the relevant timerConfig's forcedOverflowTimerValue variable.
 * @param TIM_Typedef *tim The timer to overflow
 * @return void
 **/
void timerForceOverflow(TIM_TypeDef *tim)
{
    uint8_t timerIndex = lookupTimerIndex((const TIM_TypeDef *)tim);

    ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
        
        timerConfig[timerIndex].forcedOverflowTimerValue = tim->CNT + 1;

        
        tim->EGR |= TIM_EGR_UG;
    }
}
