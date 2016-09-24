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

#include "drivers/system.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "io/serial.h"

#include "rx/rx.h"
#include "rx/xbus.h"

#include "watchdog.h"

#include "scheduler.h"





#define XBUS_CHANNEL_COUNT 12
#define XBUS_RJ01_CHANNEL_COUNT 12


#define XBUS_FRAME_SIZE 27

#define XBUS_RJ01_FRAME_SIZE 33
#define XBUS_RJ01_MESSAGE_LENGTH 30
#define XBUS_RJ01_OFFSET_BYTES 3

#define XBUS_CRC_AND_VALUE 0x8000
#define XBUS_CRC_POLY 0x1021

#define XBUS_BAUDRATE 115200
#define XBUS_RJ01_BAUDRATE 250000
#define XBUS_MAX_FRAME_TIME 8000









#define XBUS_START_OF_FRAME_BYTE (0xA1)







#define XBUS_CONVERT_TO_USEC(V)	(800 + ((V * 1400) >> 12))

static bool xBusFrameReceived = false;
static bool xBusDataIncoming = false;
static uint8_t xBusFramePosition;
static uint8_t xBusFrameLength;
static uint8_t xBusChannelCount;
static uint8_t xBusProvider;



static volatile uint8_t xBusFrame[XBUS_RJ01_FRAME_SIZE];
static uint16_t xBusChannelData[XBUS_RJ01_CHANNEL_COUNT];

static void xBusDataReceive(uint16_t c);
static uint16_t xBusReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan);

void taskHandleAnnex(void);
bool taskUpdateRxCheck(void);
void taskUpdateRxMain(void);

serialPort_t *xBusPort;
uartPort_t *xBusUart;

bool xBusInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{
    uint32_t baudRate;

    switch (rxConfig->serialrx_provider) {
        case SERIALRX_XBUS_MODE_B:
            rxRuntimeConfig->channelCount = XBUS_CHANNEL_COUNT;
            xBusFrameReceived = false;
            xBusDataIncoming = false;
            xBusFramePosition = 0;
            baudRate = XBUS_BAUDRATE;
            xBusFrameLength = XBUS_FRAME_SIZE;
            xBusChannelCount = XBUS_CHANNEL_COUNT;
            xBusProvider = SERIALRX_XBUS_MODE_B;
            break;
        case SERIALRX_XBUS_MODE_B_RJ01:
            rxRuntimeConfig->channelCount = XBUS_RJ01_CHANNEL_COUNT;
            xBusFrameReceived = false;
            xBusDataIncoming = false;
            xBusFramePosition = 0;
            baudRate = XBUS_RJ01_BAUDRATE;
            xBusFrameLength = XBUS_RJ01_FRAME_SIZE;
            xBusChannelCount = XBUS_RJ01_CHANNEL_COUNT;
            xBusProvider = SERIALRX_XBUS_MODE_B_RJ01;
            break;
        default:
            return false;
            break;
    }

    if (callback) {
        *callback = xBusReadRawRC;
    }

    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

    xBusPort = openSerialPort(portConfig->identifier, FUNCTION_RX_SERIAL, xBusDataReceive, baudRate, MODE_RX, SERIAL_NOT_INVERTED);
	xBusUart = (uartPort_t *)xBusPort;

#ifdef SERIALRX_DMA	
	USART_DMACmd(xBusUart->USARTx, USART_DMAReq_Rx, DISABLE);
	DMA_Cmd(xBusUart->rxDMAStream, DISABLE);
	DMA_ClearFlag(xBusUart->rxDMAStream, xBusUart->rxTCIF);
	DMA_ClearFlag(xBusUart->rxDMAStream, xBusUart->rxHTIF);
	DMA_SetCurrDataCounter(xBusUart->rxDMAStream, XBUS_FRAME_SIZE);
	DMA_Cmd(xBusUart->rxDMAStream, ENABLE);
	USART_DMACmd(xBusUart->USARTx, USART_DMAReq_Rx, ENABLE);
#endif

    return xBusPort != NULL;
}


static uint16_t xBusCRC16(uint16_t crc, uint8_t value)
{
    uint8_t i;
    
    crc = crc ^ (int16_t)value << 8;

    for (i = 0; i < 8; i++) {
        if (crc & XBUS_CRC_AND_VALUE) {
            crc = crc << 1 ^ XBUS_CRC_POLY;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}


uint8_t xBusRj01CRC8(uint8_t inData, uint8_t seed)
{
    uint8_t bitsLeft;
    uint8_t temp;

    for (bitsLeft = 8; bitsLeft > 0; bitsLeft--) {
        temp = ((seed ^ inData) & 0x01);

        if (temp == 0) {
            seed >>= 1;
        } else {
            seed ^= 0x18;
            seed >>= 1;
            seed |= 0x80;
        }

        inData >>= 1;
    }

    return seed;    
}


static void xBusUnpackModeBFrame(uint8_t offsetBytes)
{
    
    uint16_t crc = 0;
    uint16_t inCrc = 0;
    uint8_t i = 0;
    uint16_t value;
    uint8_t frameAddr;

    
    for (i = 0; i < XBUS_FRAME_SIZE - 2; i++) {
        inCrc = xBusCRC16(inCrc, xBusFrame[i+offsetBytes]);
    }

    
    crc = ((uint16_t)xBusFrame[offsetBytes + XBUS_FRAME_SIZE - 2]) << 8;
    crc = crc + ((uint16_t)xBusFrame[offsetBytes + XBUS_FRAME_SIZE - 1]);

    if (crc == inCrc) {
        
        for (i = 0; i < xBusChannelCount; i++) {

            frameAddr = offsetBytes + 1 + i * 2;
            value = ((uint16_t)xBusFrame[frameAddr]) << 8;
            value = value + ((uint16_t)xBusFrame[frameAddr + 1]);

            
            xBusChannelData[i] = XBUS_CONVERT_TO_USEC(value);
        }

        xBusFrameReceived = true;
    }

}

static void xBusUnpackRJ01Frame(void)
{
    
    uint8_t outerCrc = 0;
    uint8_t i = 0;

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    if (xBusFrame[1] != XBUS_RJ01_MESSAGE_LENGTH)
    {
        
        return;
    }
    
    
    
    
    for (i = 0; i < xBusFrameLength - 1; i++) {
        outerCrc = xBusRj01CRC8(outerCrc, xBusFrame[i]);
    }
    
    if (outerCrc != xBusFrame[xBusFrameLength - 1])
    {
        
        return;
    }

    
    xBusUnpackModeBFrame(XBUS_RJ01_OFFSET_BYTES);
}


#ifdef SERIALRX_DMA

static void xBusDataReceive(uint16_t buff)
{
	(void)(buff);

	if (SKIP_RX) {
		reset_rx_watchdog();
		return;
	}

	if (DMA_GetCurrDataCounter(xBusUart->rxDMAStream) == 0)
	{
		USART_DMACmd(xBusUart->USARTx, USART_DMAReq_Rx, DISABLE);
		memcpy((void *)&xBusFrame, (void *)xBusPort->rxBuffer, XBUS_FRAME_SIZE);
		switch (xBusProvider) {
		case SERIALRX_XBUS_MODE_B:
			xBusUnpackModeBFrame(0);
			reset_rx_watchdog();
		case SERIALRX_XBUS_MODE_B_RJ01:
			xBusUnpackRJ01Frame();
			reset_rx_watchdog();
		}
		taskUpdateRxCheck();
		taskUpdateRxMain();
		taskHandleAnnex();
		DMA_ClearFlag(xBusUart->rxDMAStream, xBusUart->rxTCIF);
		DMA_ClearFlag(xBusUart->rxDMAStream, xBusUart->rxHTIF);
		DMA_Cmd(xBusUart->rxDMAStream, DISABLE);
		DMA_SetCurrDataCounter(xBusUart->rxDMAStream, XBUS_FRAME_SIZE);
		DMA_Cmd(xBusUart->rxDMAStream, ENABLE);
		USART_DMACmd(xBusUart->USARTx, USART_DMAReq_Rx, ENABLE);
		
	}

}

#else

static void xBusDataReceive(uint16_t c)
{
    uint32_t now;
    static uint32_t xBusTimeLast, xBusTimeInterval;

	if (SKIP_RX) {
		reset_rx_watchdog();
		return;
	}

    
    now = micros();
    xBusTimeInterval = now - xBusTimeLast;
    xBusTimeLast = now;
    if (xBusTimeInterval > XBUS_MAX_FRAME_TIME) {
        xBusFramePosition = 0;
        xBusDataIncoming = false;
    }
    
    
    if ((xBusFramePosition == 0) && (c == XBUS_START_OF_FRAME_BYTE)) {
        xBusDataIncoming = true;
    }

    
    if (xBusDataIncoming == true) {
        
        xBusFrame[xBusFramePosition] = (uint8_t)c;
        xBusFramePosition++;
    }
    
    
    if (xBusFramePosition == xBusFrameLength) {
        switch (xBusProvider) {
            case SERIALRX_XBUS_MODE_B:
                xBusUnpackModeBFrame(0);
				reset_rx_watchdog();
            case SERIALRX_XBUS_MODE_B_RJ01:
                xBusUnpackRJ01Frame();
				reset_rx_watchdog();
        }
        xBusDataIncoming = false;
        xBusFramePosition = 0;
    }
}
#endif

uint8_t xBusFrameStatus(void)
{
    if (!xBusFrameReceived) {
        return SERIAL_RX_FRAME_PENDING;
    }

    xBusFrameReceived = false;

    return SERIAL_RX_FRAME_COMPLETE;
}

static uint16_t xBusReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    uint16_t data;

    
    if (chan >= rxRuntimeConfig->channelCount) {
        return 0;
    }

    data = xBusChannelData[chan];

    return data;
}
