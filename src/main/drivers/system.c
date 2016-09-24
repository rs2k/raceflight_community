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

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "build_config.h"

#include "gpio.h"
#include "light_led.h"
#include "sound_beeper.h"
#include "nvic.h"

#include "system.h"
#include "debug.h"

volatile uint32_t Millis=0;

volatile uint32_t Micros=0;

volatile uint32_t last_Micros=0;


static uint32_t usTicks = 0;

static volatile uint32_t sysTickUptime = 0;

uint32_t cachedRccCsrValue;

static void cycleCounterInit(void)
{
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	usTicks = SystemCoreClock / 1000000;
}


void SysTick_Handler(void)
{
	Millis++;
}


uint32_t micros(void)
{

	Micros = Millis * 1000 + 1000 - SysTick->VAL / (SystemCoreClock / 1000000);

	    
	if (Micros > last_Micros)
	{
		last_Micros = Micros;
		return (Micros);
	}
	return last_Micros;

}


uint32_t millis(void)
{
	return Millis;
}

void systemInit(void)
{

#ifdef CC3D
    /* Accounts for OP Bootloader, set the Vector Table base address as specified in .ld file */
	extern void *isr_vector_table_base;

	NVIC_SetVectorTable((uint32_t)&isr_vector_table_base, 0x0);
#endif
	    
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

#ifdef STM32F10X
	    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif

	    
	cachedRccCsrValue = RCC->CSR;
#if defined(STM32F40_41xxx) || defined (STM32F411xE) || defined(STM32F446xx)
	    /* Accounts for OP Bootloader, set the Vector Table base address as specified in .ld file */
	extern void *isr_vector_table_base;
	NVIC_SetVectorTable((uint32_t)&isr_vector_table_base, 0x0);
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, DISABLE);
#endif
	RCC_ClearFlag();

	enableGPIOPowerUsageAndNoiseReductions();

#ifdef STM32F10X
	    
	    
	gpio_config_t gpio;

	gpio.mode = Mode_Out_PP;
	gpio.speed = Speed_2MHz;
	gpio.pin = Pin_9;
	digitalHi(GPIOA, gpio.pin);
	gpioInit(GPIOA, &gpio);

	    
#define AFIO_MAPR_SWJ_CFG_NO_JTAG_SW            (0x2 << 24)
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_NO_JTAG_SW;
#endif

	    
	cycleCounterInit();

	uint32_t oh_crap = 100000000;
	    
	if (SysTick_Config(SystemCoreClock / 1000)) { 
		LED0_ON;
		LED1_ON;
		LED2_ON;
		while (1)
		{
			oh_crap = 100000000;
			while (oh_crap--);
			LED0_TOGGLE;
			LED1_TOGGLE;
			LED2_TOGGLE;
		}
	}

	
	
	NVIC_SetPriority(SysTick_IRQn, 0);

	Millis = 0;
}


void delayMicroseconds(uint32_t nTime)
{
	int32_t curTime = micros();
	int counter = 0;

	for (counter = 0; counter < 500000; counter++) /* a break out in case system interrupt isn't set up yet */
	{
		if (((int32_t )nTime - ((int32_t )micros() - (int32_t )curTime)) < 1)
			break;
	}




}

void delay(uint32_t nTime)
{
	int32_t curTime = Millis;
	int counter=0;

	for (counter = 0; counter < 500000; counter++) /* a break out in case system interrupt isn't set up yet */
	{
		if (((int32_t )nTime - ((int32_t)Millis - (int32_t )curTime)) < 1)
			break;
	}

	/*
	while ((nTime - (Millis - curTime)) > 0)
	{
		counter = counter + 1;
		if (counter > 5000000)
			break;


	}
	;*/

}

 


#define SHORT_FLASH_DURATION 50
#define CODE_FLASH_DURATION 250

void failureMode(failureMode_e mode)
{
	int codeRepeatsRemaining = 10;
	int codeFlashesRemaining;
	int shortFlashesRemaining;

	while (codeRepeatsRemaining--) {
		LED1_ON;
		LED0_OFF;
		shortFlashesRemaining = 5;
		codeFlashesRemaining = mode + 1;
		uint8_t flashDuration = SHORT_FLASH_DURATION;

		while (shortFlashesRemaining || codeFlashesRemaining) {
			LED1_TOGGLE;
			LED0_TOGGLE;
			BEEP_ON;
			delay(flashDuration);

			LED1_TOGGLE;
			LED0_TOGGLE;
			BEEP_OFF;
			delay(flashDuration);

			if (shortFlashesRemaining) {
				shortFlashesRemaining--;
				if (shortFlashesRemaining == 0) {
					delay(500);
					flashDuration = CODE_FLASH_DURATION;
				}
			}
			else {
				codeFlashesRemaining--;
			}
		}
		delay(1000);
	}

#ifdef DEBUG
	systemReset();
#else
	systemResetToBootloader();
#endif
}
