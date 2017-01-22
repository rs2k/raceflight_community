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
#include <stdlib.h>
#include <stdint.h>
#include "include.h"
uint32_t currentTime = 0;
uint16_t averageWaitingTasks100 = 0;
void taskCheckAndFlashErase(void);
void taskHandleSerial(void);
void taskHandleAnnex(void);
void taskUpdateBeeper(void);
void taskUpdateBattery(void);
bool taskUpdateRxCheck(void);
void taskUpdateRxMain(void);
void taskProcessGPS(void);
void taskUpdateCompass(void);
void taskUpdateBaro(void);
void taskUpdateSonar(void);
void taskCalculateAltitude(void);
void taskUpdateDisplay(void);
void taskTelemetry(void);
void taskLedStrip(void);
void taskSystem(void);
#ifdef USE_BST
void taskBstReadWrite(void);
void taskBstMasterProcess(void);
#endif
#if defined(FAKE_EXTI)
extern bool init_done;
void gyro_exti_failiure_init(void)
{
 uint16_t prescalerValue;
 uint32_t scheduler_timer_mhz;
 uint32_t scheduler_timer_khz;
 if (gyroSamplePeriod == 31) {
  scheduler_timer_khz = 125;
  scheduler_timer_mhz = 3;
 } else {
  scheduler_timer_khz = 125;
  scheduler_timer_mhz = 1;
 }
 NVIC_InitTypeDef NVIC_InitStructure;
 NVIC_InitStructure.NVIC_IRQChannel = FE_TIM_IRQ;
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_MPU_DATA_READY);
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_MPU_DATA_READY);
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 NVIC_Init(&NVIC_InitStructure);
 prescalerValue = (uint16_t)(SystemCoreClock / 2 / ((uint32_t)scheduler_timer_mhz * 1000000)) - 1;
 TIM_TimeBaseInitTypeDef timerInitStructure;
 timerInitStructure.TIM_Prescaler = prescalerValue;
 timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
 timerInitStructure.TIM_Period = scheduler_timer_khz - 1;
 timerInitStructure.TIM_ClockDivision = 0;
 TIM_TimeBaseInit(FE_TIM, &timerInitStructure);
 TIM_Cmd(FE_TIM, ENABLE);
 TIM_ITConfig(FE_TIM, TIM_IT_Update, ENABLE);
}
void TIM8_UP_TIM13_IRQHandler(void)
{
 if (TIM_GetITStatus(FE_TIM, TIM_IT_Update) != RESET)
 {
  static bool running = false;
  TIM_ClearITPendingBit(FE_TIM, TIM_IT_Update);
  if (!running) {
   running = true;
   if (!ARMING_FLAG(ARMED)) {
    updateWatchdog();
    if ( (init_done == 0) || (SKIP_GYRO) )
    {
     return;
    }
   }
   static uint8_t counter_gyro = 0;
   MainPidLoop();
   counter_gyro++;
   if (counter_gyro == accDenominator) {
    counter_gyro = 0;
    UpdateAccelerometer();
   }
   running = false;
  }
 }
}
void TIM1_BRK_TIM15_IRQHandler(void)
{
 if (TIM_GetITStatus(FE_TIM, TIM_IT_Update) != RESET)
 {
  TIM_ClearITPendingBit(FE_TIM, TIM_IT_Update);
  if (!ARMING_FLAG(ARMED)) {
   updateWatchdog();
   if ( (init_done == 0) || (SKIP_GYRO) )
   {
    return;
   }
  }
  static uint8_t counter_gyro = 0;
  MainPidLoop();
  counter_gyro++;
  if (counter_gyro == accDenominator) {
   counter_gyro = 0;
   UpdateAccelerometer();
  }
 }
}
#endif
uint32_t rxPollInterval = 0;
uint32_t rxPrevTime = 0;
extern uartPort_t *spektrumUart;
void scheduler(uint8_t count)
{
 switch (count)
 {
  case 1:
   taskUpdateBeeper();
   break;
  case 2:
   taskUpdateBattery();
   break;
  case 3:
   break;
#ifdef WS2812_LED
        case 4:
            taskLedStrip();
            break;
#endif
        case 5:
         break;
        case 6:
         checkForRxFailsafe();
         break;
        case 7:
         taskCheckAndFlashErase();
         break;
 }
 currentTime = micros();
 taskHandleSerial();
 rxPollInterval = currentTime - rxPrevTime;
 rxPrevTime = currentTime;
#ifndef SERIALRX_DMA
 if(taskUpdateRxCheck())
 {
  taskUpdateRxMain();
  taskHandleAnnex();
 }
#endif
}
