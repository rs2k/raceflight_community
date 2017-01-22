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
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "include.h"
#ifdef WS2812_LED
#include "ws2812_led.h"
#define WS2812_EXTRA_CYCLES 42
#define WS2812_BUFSIZE (8*3*WS2812_MAX_LEDS+WS2812_EXTRA_CYCLES)
#define MAX_LED_COLORS 7
ws2812Led_t WS2812_IO_colors[WS2812_MAX_LEDS];
uint32_t WS2812_IO_framedata[WS2812_BUFSIZE];
ws2812Led_t colorTable[MAX_LED_COLORS];
uint8_t lastLEDMode = 0;
uint8_t ledColor = 254;
ws2812Led_t red = {
 .r = 0xff,
 .g = 0x00,
 .b = 0x00,
};
ws2812Led_t yellow = {
 .r = 0xff,
 .g = 0xff,
 .b = 0x00,
};
ws2812Led_t white = {
 .r = 0xff,
 .g = 0xff,
 .b = 0xff,
};
ws2812Led_t green = {
 .r = 0x00,
 .g = 0xff,
 .b = 0x00,
};
ws2812Led_t cyan = {
 .r = 0x00,
 .g = 0xff,
 .b = 0xff,
};
ws2812Led_t blue = {
 .r = 0x00,
 .g = 0x00,
 .b = 0xff,
};
ws2812Led_t purple = {
 .r = 0xff,
 .g = 0x00,
 .b = 0xff,
};
void SetLEDColor(uint8_t newColor)
{
 uint8_t x;
 if (newColor >= MAX_LED_COLORS)
 {
  ledColor = 0;
 }
 else
 {
  ledColor = newColor;
 }
 for (x = 0; x < masterConfig.led_count; x++)
 {
  memcpy(&WS2812_IO_colors[x], &colorTable[ledColor], 3);
 }
}
void ws2812_led_init( void )
{
 memcpy(&colorTable[0], &red, 3);
 memcpy(&colorTable[1], &yellow, 3);
 memcpy(&colorTable[2], &green, 3);
 memcpy(&colorTable[3], &cyan, 3);
 memcpy(&colorTable[4], &blue, 3);
 memcpy(&colorTable[5], &purple, 3);
 memcpy(&colorTable[6], &white, 3);
 memset(WS2812_IO_colors, 0xff, sizeof(WS2812_IO_colors));
    RCC_APB2PeriphClockCmd(WS2812_LED_TIM_PERIPH, ENABLE);
#ifdef STM32F4
    RCC_AHB1PeriphClockCmd(WS2812_LED_PERIPH, ENABLE);
    RCC_AHB1PeriphClockCmd(WS2812_LED_DMA_PERIPH, ENABLE);
#else
    RCC_AHBPeriphClockCmd(WS2812_LED_PERIPH, ENABLE);
    RCC_AHBPeriphClockCmd(WS2812_LED_DMA_PERIPH, ENABLE);
#endif
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = WS2812_LED_PIN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(WS2812_LED_GPIO, &GPIO_InitStructure);
    GPIO_PinAFConfig(WS2812_LED_GPIO, WS2812_LED_PINSOURCE, WS2812_LED_TIM_AF);
    TIM_Cmd(WS2812_LED_TIM, DISABLE);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    uint16_t PrescalerValue;
#ifdef STM32F4
 if (IS_RCC_APB2_PERIPH(WS2812_LED_TIM_PERIPH))
 {
  PrescalerValue = (uint16_t)(SystemCoreClock / 24000000) - 1;
 }
 else
 {
  PrescalerValue = (uint16_t)(SystemCoreClock / 2 / 24000000) - 1;
 }
#else
    PrescalerValue = (uint16_t) ( SystemCoreClock / 24000000 ) - 1;
#endif
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
 TIM_TimeBaseStructure.TIM_Period = 29;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(WS2812_LED_TIM, &TIM_TimeBaseStructure);
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
#if defined(WS2812_LED_TIMER_CH1)
    TIM_OC1Init(WS2812_LED_TIM, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(WS2812_LED_TIM, TIM_OCPreload_Enable);
#elif defined(WS2812_LED_TIMER_CH2)
    TIM_OC2Init(WS2812_LED_TIM, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(WS2812_LED_TIM, TIM_OCPreload_Enable);
#elif defined(WS2812_LED_TIMER_CH3)
    TIM_OC3Init(WS2812_LED_TIM, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(WS2812_LED_TIM, TIM_OCPreload_Enable);
#elif defined(WS2812_LED_TIMER_CH4)
    TIM_OC4Init(WS2812_LED_TIM, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(WS2812_LED_TIM, TIM_OCPreload_Enable);
#else
#error Unknown WS2812 timer channel
#endif
    TIM_CtrlPWMOutputs(WS2812_LED_TIM, ENABLE);
    DMA_InitTypeDef DMA_InitStructure;
#ifdef STM32F4
    DMA_Cmd(WS2812_LED_DMA_ST, DISABLE);
    DMA_DeInit(WS2812_LED_DMA_ST);
#else
    DMA_Cmd(WS2812_LED_DMA_CH, DISABLE);
    DMA_DeInit(WS2812_LED_DMA_CH);
#endif
    DMA_StructInit(&DMA_InitStructure);
#ifdef STM32F4
    DMA_InitStructure.DMA_Channel = WS2812_LED_DMA_CH;
#endif
#if defined(WS2812_LED_TIMER_CH1)
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &WS2812_LED_TIM->CCR1;
#elif defined(WS2812_LED_TIMER_CH2)
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &WS2812_LED_TIM->CCR2;
#elif defined(WS2812_LED_TIMER_CH3)
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &WS2812_LED_TIM->CCR3;
#elif defined(WS2812_LED_TIMER_CH4)
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &WS2812_LED_TIM->CCR4;
#else
#error Unknown WS2812 timer channel
#endif
#ifdef STM32F4
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) WS2812_IO_framedata;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
#else
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) WS2812_IO_framedata;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
#endif
    DMA_InitStructure.DMA_BufferSize = 0;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
#ifdef STM32F4
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
#else
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
#endif
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
#ifdef STM32F4
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(WS2812_LED_DMA_ST, &DMA_InitStructure);
#else
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(WS2812_LED_DMA_CH, &DMA_InitStructure);
#endif
#if defined(WS2812_LED_TIMER_CH1)
    TIM_DMACmd(WS2812_LED_TIM, TIM_DMA_CC1, ENABLE);
#elif defined(WS2812_LED_TIMER_CH2)
    TIM_DMACmd(WS2812_LED_TIM, TIM_DMA_CC2, ENABLE);
#elif defined(WS2812_LED_TIMER_CH3)
    TIM_DMACmd(WS2812_LED_TIM, TIM_DMA_CC3, ENABLE);
#elif defined(WS2812_LED_TIMER_CH4)
    TIM_DMACmd(WS2812_LED_TIM, TIM_DMA_CC4, ENABLE);
#else
#error Unknown WS2812 timer channel
#endif
#ifdef STM32F4
    DMA_ITConfig(WS2812_LED_DMA_ST, DMA_IT_TC, ENABLE);
#else
    DMA_ITConfig(WS2812_LED_DMA_CH, DMA_IT_TC, ENABLE);
#endif
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = WS2812_LED_DMA_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );
    SetLEDColor(masterConfig.led_color);
}
void ws2812_led_update(uint8_t nLeds) {
    int8_t bitIdx;
    uint8_t ledIdx;
    uint16_t bufferIdx = 0;
    if (nLeds > WS2812_MAX_LEDS) {
        nLeds = WS2812_MAX_LEDS;
    }
    for (ledIdx = 0; ledIdx < nLeds; ledIdx++) {
        uint32_t grb = (WS2812_IO_colors[ledIdx].g << 16) | (WS2812_IO_colors[ledIdx].r << 8) | WS2812_IO_colors[ledIdx].b;
        for (bitIdx = 23; bitIdx >= 0; bitIdx--) {
            WS2812_IO_framedata[bufferIdx++] = (grb & (1 << bitIdx)) ? 17 : 8;
        }
    }
    while (bufferIdx < WS2812_BUFSIZE) {
        WS2812_IO_framedata[bufferIdx++] = 0;
    }
#ifdef STM32F4
    if (DMA_GetCurrDataCounter(WS2812_LED_DMA_ST) == 0) {
        DMA_SetCurrDataCounter(WS2812_LED_DMA_ST, 24*nLeds + WS2812_EXTRA_CYCLES);
        DMA_Cmd(WS2812_LED_DMA_ST, ENABLE);
#else
    if (DMA_GetCurrDataCounter(WS2812_LED_DMA_CH) == 0) {
        DMA_SetCurrDataCounter(WS2812_LED_DMA_CH, 24*nLeds + WS2812_EXTRA_CYCLES);
        DMA_Cmd(WS2812_LED_DMA_CH, ENABLE);
#endif
        TIM_SetCounter(WS2812_LED_TIM, 0);
        TIM_Cmd(WS2812_LED_TIM, ENABLE);
    }
}
void WS2812_LED_DMA_IRQ_HANDLER(void)
{
#ifdef STM32F4
    if (DMA_GetFlagStatus(WS2812_LED_DMA_ST, WS2812_LED_DMA_FLAG)) {
        DMA_Cmd(WS2812_LED_DMA_ST, DISABLE);
#else
    if (DMA_GetITStatus(DMA1_FLAG_TC3)) {
        DMA_Cmd(WS2812_LED_DMA_CH, DISABLE);
#endif
        TIM_Cmd(WS2812_LED_TIM, DISABLE);
#ifdef STM32F4
        DMA_ClearITPendingBit(WS2812_LED_DMA_ST, WS2812_LED_DMA_FLAG);
#else
        DMA_ClearITPendingBit(WS2812_LED_DMA_FLAG);
#endif
    }
}
#endif
