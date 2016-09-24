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
#include <stdlib.h>
#include <stdint.h>

#include "include.h"

#include "platform.h"
#include "scheduler.h"
#include "debug.h"
#include "drivers/nvic.h"

#include "common/maths.h"

#include "drivers/system.h"

#include "drivers/ws2812_led.h"

uint32_t currentTime = 0;
uint16_t averageWaitingTasks100 = 0;

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



uint32_t rxPollInterval = 0;
uint32_t rxPrevTime = 0;

void scheduler(uint8_t count)
{

	switch (count)
	{
		case 1:
			if (!ARMING_FLAG(ARMED)) {
				reset_rx_watchdog(); 
			}
			taskUpdateBeeper();
			break;
		case 2:
			taskUpdateBattery();
			break;
		case 3:
			taskTelemetry();
			break;
#ifdef WS2812_LED
        case 4:
            taskLedStrip();
            break;
#endif
		default:
			
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
