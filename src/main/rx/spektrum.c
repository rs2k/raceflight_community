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

#include "include.h"

#include "platform.h"
#include "debug.h"

#include "drivers/gpio.h"
#include "drivers/system.h"

#include "drivers/light_led.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "io/serial.h"

#include "config/config.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "watchdog.h"

#include "scheduler.h"
#include "../drivers/system.h"



#define SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT 12
#define SPEKTRUM_2048_CHANNEL_COUNT 12
#define SPEKTRUM_1024_CHANNEL_COUNT 7

#define SPEK_FRAME_SIZE 16

#define SPEKTRUM_BAUDRATE 115200

static uint8_t initialPacket = 0;
static uint8_t frameLoss = 0;
static uint8_t spek_chan_shift;
static uint8_t spek_chan_mask;
static bool rcFrameComplete = false;
static bool spekHiRes = false;

static volatile uint8_t spekFrame[SPEK_FRAME_SIZE];

void taskHandleAnnex(void);
bool taskUpdateRxCheck(void);
void taskUpdateRxMain(void);

static void spektrumDataReceive(uint16_t c);
static uint16_t spektrumReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan);

static rxRuntimeConfig_t *rxRuntimeConfigPtr;

serialPort_t *spektrumPort;
uartPort_t *spektrumUart;

bool spektrumInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{
	rxRuntimeConfigPtr = rxRuntimeConfig;

	switch (rxConfig->serialrx_provider) {
	case SERIALRX_SPEKTRUM2048:
	    
		spek_chan_shift = 3;
		spek_chan_mask = 0x07;
		spekHiRes = true;
		rxRuntimeConfig->channelCount = SPEKTRUM_2048_CHANNEL_COUNT;
		break;
	case SERIALRX_SPEKTRUM1024:
	    
		spek_chan_shift = 2;
		spek_chan_mask = 0x03;
		spekHiRes = false;
		rxRuntimeConfig->channelCount = SPEKTRUM_1024_CHANNEL_COUNT;
		break;
	}

	if (callback)
		*callback = spektrumReadRawRC;

	serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
	if (!portConfig) {
		return false;
	}

	spektrumPort = openSerialPort(portConfig->identifier, FUNCTION_RX_SERIAL, spektrumDataReceive, SPEKTRUM_BAUDRATE, MODE_RX, SERIAL_NOT_INVERTED);
	spektrumUart = (uartPort_t *)spektrumPort;

#ifdef SERIALRX_DMA	
	USART_DMACmd(spektrumUart->USARTx, USART_DMAReq_Rx, DISABLE);
	DMA_Cmd(spektrumUart->rxDMAStream, DISABLE);
	DMA_ClearFlag(spektrumUart->rxDMAStream, spektrumUart->rxTCIF);
	DMA_ClearFlag(spektrumUart->rxDMAStream, spektrumUart->rxHTIF);
	DMA_SetCurrDataCounter(spektrumUart->rxDMAStream, SPEK_FRAME_SIZE);
	DMA_Cmd(spektrumUart->rxDMAStream, ENABLE);
	USART_DMACmd(spektrumUart->USARTx, USART_DMAReq_Rx, ENABLE);
#endif

	return spektrumPort != NULL;
}


#ifdef SERIALRX_DMA

static void spektrumDataReceive(uint16_t buff)
{
	(void)(buff);

	if (SKIP_RX) {
		reset_rx_watchdog();
		return;
	}

	if (DMA_GetCurrDataCounter(spektrumUart->rxDMAStream) == 0)
	{
		USART_DMACmd(spektrumUart->USARTx, USART_DMAReq_Rx, DISABLE);
		memcpy((void *)&spekFrame, (void *)spektrumPort->rxBuffer, SPEK_FRAME_SIZE);
		rcFrameComplete = true;
		reset_rx_watchdog();
		taskUpdateRxCheck();
		taskUpdateRxMain();
		taskHandleAnnex();
		DMA_ClearFlag(spektrumUart->rxDMAStream, spektrumUart->rxTCIF);
		DMA_ClearFlag(spektrumUart->rxDMAStream, spektrumUart->rxHTIF);
		DMA_Cmd(spektrumUart->rxDMAStream, DISABLE);
		DMA_SetCurrDataCounter(spektrumUart->rxDMAStream, SPEK_FRAME_SIZE);
		DMA_Cmd(spektrumUart->rxDMAStream, ENABLE);
		USART_DMACmd(spektrumUart->USARTx, USART_DMAReq_Rx, ENABLE);
		
	}

}

#else

static void spektrumDataReceive(uint16_t c)
{
	uint32_t spekTime;
	static uint32_t spekTimeLast, spekTimeInterval;
	static uint8_t spekFramePosition;

	if (SKIP_RX) {
		reset_rx_watchdog();
		return;
	}

	spekTime = micros();
	spekTimeInterval = spekTime - spekTimeLast;
	spekTimeLast = spekTime;
	if (spekTimeInterval > 5000) {
		spekFramePosition = 0;
	}

	if (spekFramePosition < SPEK_FRAME_SIZE) {
		spekFrame[spekFramePosition++] = (uint8_t)c;
		if (spekFramePosition < SPEK_FRAME_SIZE) {
			rcFrameComplete = false;
		}
		else {
			rcFrameComplete = true;
			reset_rx_watchdog();
		}
	}
}
#endif

static uint32_t spekChannelData[SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT];






uint8_t spektrumFrameStatus(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
	uint8_t b;

	if (!rcFrameComplete) {
		return SERIAL_RX_FRAME_PENDING;
	}

	rcFrameComplete = false;

	frameLoss = spekFrame[0];   

	for (b = 3; b < SPEK_FRAME_SIZE; b += 2) {
		uint8_t spekChannel = 0x0F & (spekFrame[b - 1] >> spek_chan_shift);
		if (spekChannel < rxRuntimeConfigPtr->channelCount && spekChannel < SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT) {
			spekChannelData[spekChannel] = ((uint32_t)(spekFrame[b - 1] & spek_chan_mask) << 8) + spekFrame[b];
		}
	}

	return SERIAL_RX_FRAME_COMPLETE;
}

static uint16_t spektrumReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
	uint16_t data;

	if (chan >= rxRuntimeConfig->channelCount) {
		return 0;
	}

#ifdef SPEKTRUM_PROPER_SCALING 
	if (spekHiRes)
		data = 903 + (spekChannelData[chan] * .583);    
	else
		data = 903 + (spekChannelData[chan] * 1.166);   
#else
	if (spekHiRes)
		data = 988 + (spekChannelData[chan] >> 1);      
	else
		data = 988 + spekChannelData[chan];             
#endif
	return data;
}

#ifdef SPEKTRUM_BIND

bool spekShouldBind(uint8_t spektrum_sat_bind)
{
#ifdef HARDWARE_BIND_PLUG
	gpio_config_t cfg = {
		BINDPLUG_PIN,
		Mode_IPU,
		Speed_2MHz
	};
	gpioInit(BINDPLUG_PORT, &cfg);

	    
	delayMicroseconds(10);  
	if (digitalIn(BINDPLUG_PORT, BINDPLUG_PIN)) {
		return false;
	}
#endif

	return !(
	    isMPUSoftReset() ||
	    spektrum_sat_bind == SPEKTRUM_SAT_BIND_DISABLED ||
	    spektrum_sat_bind > SPEKTRUM_SAT_BIND_MAX
	);
}
/* spektrumBind function ported from Baseflight. It's used to bind satellite receiver to TX.
 * Function must be called immediately after startup so that we don't miss satellite bind window.
 * Known parameters. Tested with DSMX satellite and DX8 radio. Framerate (11ms or 22ms) must be selected from TX.
 * 9 = DSMX 11ms / DSMX 22ms
 * 5 = DSM2 11ms 2048 / DSM2 22ms 1024
 */
void spektrumBind(rxConfig_t *rxConfig)
{
	int i;
	if (!spekShouldBind(rxConfig->spektrum_sat_bind)) {
		return;
	}

	LED1_ON;

#ifdef STM32F4
	gpio_config_t cfg = {
		BIND_PIN,
		Mode_Out_PP,
		Speed_50MHz
	};
#else
	gpio_config_t cfg = {
		BIND_PIN,
		Mode_Out_OD,
		Speed_2MHz
	};
#endif

	gpioInit(BIND_PORT, &cfg);

		
	digitalHi(BIND_PORT, BIND_PIN);

    #ifdef BIND_PORT2
    cfg.pin = BIND_PIN2;
    gpioInit(BIND_PORT2, &cfg);

    
    digitalHi(BIND_PORT2, BIND_PIN2);
    #endif

    

    LED1_OFF;

	for (i = 0; i < rxConfig->spektrum_sat_bind; i++) {

        LED0_OFF;
        LED2_OFF;
        
        digitalLo(BIND_PORT, BIND_PIN);

        #ifdef BIND_PORT2
        digitalLo(BIND_PORT2, BIND_PIN2);
        #endif
		delayMicroseconds(120);

        LED0_ON;
        LED2_ON;
        
		digitalHi(BIND_PORT, BIND_PIN);

        #ifdef BIND_PORT2
        digitalHi(BIND_PORT2, BIND_PIN2);
        #endif
		delayMicroseconds(120);

	}

#ifndef HARDWARE_BIND_PLUG
	    
	    
	if (!isMPUSoftReset()) {
		rxConfig->spektrum_sat_bind = 0;
		saveConfigAndNotify();
	}
#endif

}
#endif
