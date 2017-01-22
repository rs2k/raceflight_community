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
#include "platform.h"
#include "build_config.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "io/serial.h"
#include "rx/rx.h"
#include "rx/ibus.h"
#include "watchdog.h"
#include "scheduler.h"
#define IBUS_MAX_CHANNEL 10
#define IBUS_BUFFSIZE 32
#define IBUS_SYNCBYTE 0x20
#define IBUS_BAUDRATE 115200
static bool ibusFrameDone = false;
static uint32_t ibusChannelData[IBUS_MAX_CHANNEL];
static void ibusDataReceive(uint16_t c);
static uint16_t ibusReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan);
void taskHandleAnnex(void);
bool taskUpdateRxCheck(void);
void taskUpdateRxMain(void);
serialPort_t *ibusPort;
uartPort_t *iBusUart;
bool ibusInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{
    UNUSED(rxConfig);
    if (callback)
        *callback = ibusReadRawRC;
    rxRuntimeConfig->channelCount = IBUS_MAX_CHANNEL;
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }
    ibusPort = openSerialPort(portConfig->identifier, FUNCTION_RX_SERIAL, ibusDataReceive, IBUS_BAUDRATE, MODE_RX, SERIAL_NOT_INVERTED);
 iBusUart = (uartPort_t *)ibusPort;
#ifdef SERIALRX_DMA
 USART_DMACmd(iBusUart->USARTx, USART_DMAReq_Rx, DISABLE);
 DMA_Cmd(iBusUart->rxDMAStream, DISABLE);
 DMA_ClearFlag(iBusUart->rxDMAStream, iBusUart->rxTCIF);
 DMA_ClearFlag(iBusUart->rxDMAStream, iBusUart->rxHTIF);
 DMA_SetCurrDataCounter(iBusUart->rxDMAStream, IBUS_BUFFSIZE);
 DMA_Cmd(iBusUart->rxDMAStream, ENABLE);
 USART_DMACmd(iBusUart->USARTx, USART_DMAReq_Rx, ENABLE);
#endif
    return ibusPort != NULL;
}
static uint8_t ibus[IBUS_BUFFSIZE] = { 0, };
#ifdef SERIALRX_DMA
static void ibusDataReceive(uint16_t buff)
{
 (void)(buff);
 if (SKIP_RX) {
  resetTimeSinceRxPulse();
  return;
 }
 if (DMA_GetCurrDataCounter(iBusUart->rxDMAStream) == 0)
 {
  USART_DMACmd(iBusUart->USARTx, USART_DMAReq_Rx, DISABLE);
  memcpy((void *)&ibus, (void *)ibusPort->rxBuffer, IBUS_BUFFSIZE);
  ibusFrameDone = true;
  resetTimeSinceRxPulse();
  taskUpdateRxCheck();
  taskUpdateRxMain();
  taskHandleAnnex();
  DMA_ClearFlag(iBusUart->rxDMAStream, iBusUart->rxTCIF);
  DMA_ClearFlag(iBusUart->rxDMAStream, iBusUart->rxHTIF);
  DMA_Cmd(iBusUart->rxDMAStream, DISABLE);
  DMA_SetCurrDataCounter(iBusUart->rxDMAStream, IBUS_BUFFSIZE);
  DMA_Cmd(iBusUart->rxDMAStream, ENABLE);
  USART_DMACmd(iBusUart->USARTx, USART_DMAReq_Rx, ENABLE);
 }
}
#else
static void ibusDataReceive(uint16_t c)
{
    uint32_t ibusTime;
    static uint32_t ibusTimeLast;
    static uint8_t ibusFramePosition;
 if (SKIP_RX) {
  resetTimeSinceRxPulse();
  return;
 }
    ibusTime = micros();
    if ((ibusTime - ibusTimeLast) > 3000)
        ibusFramePosition = 0;
    ibusTimeLast = ibusTime;
    if (ibusFramePosition == 0 && c != IBUS_SYNCBYTE)
        return;
    ibus[ibusFramePosition] = (uint8_t)c;
    if (ibusFramePosition == IBUS_BUFFSIZE - 1) {
        ibusFrameDone = true;
        resetTimeSinceRxPulse();
    } else {
        ibusFramePosition++;
    }
}
#endif
uint8_t ibusFrameStatus(void)
{
    uint8_t i;
    uint8_t frameStatus = SERIAL_RX_FRAME_PENDING;
    uint16_t chksum, rxsum;
    if (!ibusFrameDone) {
        return frameStatus;
    }
    ibusFrameDone = false;
    chksum = 0xFFFF;
    for (i = 0; i < 30; i++)
        chksum -= ibus[i];
    rxsum = ibus[30] + (ibus[31] << 8);
    if (chksum == rxsum) {
        ibusChannelData[0] = (ibus[ 3] << 8) + ibus[ 2];
        ibusChannelData[1] = (ibus[ 5] << 8) + ibus[ 4];
        ibusChannelData[2] = (ibus[ 7] << 8) + ibus[ 6];
        ibusChannelData[3] = (ibus[ 9] << 8) + ibus[ 8];
        ibusChannelData[4] = (ibus[11] << 8) + ibus[10];
        ibusChannelData[5] = (ibus[13] << 8) + ibus[12];
        ibusChannelData[6] = (ibus[15] << 8) + ibus[14];
        ibusChannelData[7] = (ibus[17] << 8) + ibus[16];
        ibusChannelData[8] = (ibus[19] << 8) + ibus[18];
        ibusChannelData[9] = (ibus[21] << 8) + ibus[20];
        frameStatus = SERIAL_RX_FRAME_COMPLETE;
    }
    return frameStatus;
}
static uint16_t ibusReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    UNUSED(rxRuntimeConfig);
    return ibusChannelData[chan];
}
