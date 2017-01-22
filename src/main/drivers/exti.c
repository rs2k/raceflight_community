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
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "platform.h"
#include "drivers/nvic.h"
#include "drivers/io_impl.h"
#include "exti.h"
#ifdef USE_EXTI
typedef struct {
 extiCallbackRec_t* handler;
} extiChannelRec_t;
extiChannelRec_t extiChannelRecs[16];
#define EXTI_IRQ_GROUPS 7
static const uint8_t extiGroups[16] = { 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6 };
static uint8_t extiGroupPriority[EXTI_IRQ_GROUPS];
#if defined(STM32F10X) || defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F446xx)
static const uint8_t extiGroupIRQn[EXTI_IRQ_GROUPS] = {
 EXTI0_IRQn,
 EXTI1_IRQn,
 EXTI2_IRQn,
 EXTI3_IRQn,
 EXTI4_IRQn,
 EXTI9_5_IRQn,
 EXTI15_10_IRQn
};
#elif defined(STM32F303xC)
static const uint8_t extiGroupIRQn[EXTI_IRQ_GROUPS] = {
 EXTI0_IRQn,
 EXTI1_IRQn,
 EXTI2_TS_IRQn,
 EXTI3_IRQn,
 EXTI4_IRQn,
 EXTI9_5_IRQn,
 EXTI15_10_IRQn
};
#else
# warning "Unknown CPU"
#endif
void EXTIInit(void)
{
#ifdef STM32F10X
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif
#if defined(STM32F303xC) || defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F446xx)
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif
 memset(extiChannelRecs, 0, sizeof(extiChannelRecs));
 memset(extiGroupPriority, 0xff, sizeof(extiGroupPriority));
}
void EXTIHandlerInit(extiCallbackRec_t *self, extiHandlerCallback *fn)
{
 self->fn = fn;
}
void EXTIConfig(IO_t io, extiCallbackRec_t *cb, int irqPriority, EXTITrigger_TypeDef trigger)
{
 int chIdx;
 chIdx = IO_GPIOPinIdx(io);
 if (chIdx < 0)
  return;
 extiChannelRec_t *rec = &extiChannelRecs[chIdx];
 int group = extiGroups[chIdx];
 rec->handler = cb;
#if defined(STM32F10X)
 GPIO_EXTILineConfig(IO_GPIO_PortSource(io), IO_GPIO_PinSource(io));
#elif defined(STM32F303xC)
 SYSCFG_EXTILineConfig(IO_EXTI_PortSourceGPIO(io), IO_EXTI_PinSource(io));
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F446xx)
    SYSCFG_EXTILineConfig(IO_EXTI_PortSourceGPIO(io), IO_EXTI_PinSource(io));
#else
# warning "Unknown CPU"
#endif
 uint32_t extiLine = IO_EXTI_Line(io);
 EXTI_ClearITPendingBit(extiLine);
 EXTI_InitTypeDef EXTIInit;
 EXTIInit.EXTI_Line = extiLine;
 EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
 EXTIInit.EXTI_Trigger = trigger;
 EXTIInit.EXTI_LineCmd = ENABLE;
 EXTI_Init(&EXTIInit);
 if (extiGroupPriority[group] > irqPriority) {
  extiGroupPriority[group] = irqPriority;
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = extiGroupIRQn[group];
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(irqPriority);
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(irqPriority);
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 }
}
void EXTIRelease(IO_t io)
{
 EXTIEnable(io, false);
 int chIdx;
 chIdx = IO_GPIOPinIdx(io);
 if (chIdx < 0)
  return;
 extiChannelRec_t *rec = &extiChannelRecs[chIdx];
 rec->handler = NULL;
}
void EXTIEnable(IO_t io, bool enable)
{
#if defined(STM32F10X)
 uint32_t extiLine = IO_EXTI_Line(io);
 if (!extiLine)
  return;
 if (enable)
  EXTI->IMR |= extiLine;
 else
  EXTI->IMR &= ~extiLine;
#elif defined(STM32F303xC)
    int extiLine = IO_EXTI_Line(io);
    if (extiLine < 0)
        return;
    if (enable)
        EXTI->IMR |= 1 << extiLine;
    else
        EXTI->IMR &= ~(1 << extiLine);
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F446xx)
    int extiLine = IO_EXTI_Line(io);
    if (extiLine < 0)
        return;
    if (enable)
        EXTI->IMR |= 1 << extiLine;
    else
        EXTI->IMR &= ~(1 << extiLine);
#else
# error "Unsupported target"
#endif
}
void EXTI_IRQHandler(void)
{
 uint32_t exti_active = EXTI->IMR & EXTI->PR;
 while (exti_active) {
  unsigned idx = 31 - __builtin_clz(exti_active);
  uint32_t mask = 1 << idx;
  extiChannelRecs[idx].handler->fn(extiChannelRecs[idx].handler);
  EXTI->PR = mask;
  exti_active &= ~mask;
 }
}
#define _EXTI_IRQ_HANDLER(name) \
    void name(void) { \
        EXTI_IRQHandler(); \
    } \
    struct dummy \
_EXTI_IRQ_HANDLER(EXTI0_IRQHandler);
_EXTI_IRQ_HANDLER(EXTI1_IRQHandler);
#if defined(STM32F10X) || defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F446xx)
_EXTI_IRQ_HANDLER(EXTI2_IRQHandler);
#elif defined(STM32F303xC)
_EXTI_IRQ_HANDLER(EXTI2_TS_IRQHandler);
#else
# warning "Unknown CPU"
#endif
_EXTI_IRQ_HANDLER(EXTI3_IRQHandler);
_EXTI_IRQ_HANDLER(EXTI4_IRQHandler);
_EXTI_IRQ_HANDLER(EXTI9_5_IRQHandler);
_EXTI_IRQ_HANDLER(EXTI15_10_IRQHandler);
#endif
