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
#include "rx/sumd.h"
#include "watchdog.h"
#define SUMD_SYNCBYTE 0xA8
#define SUMD_MAX_CHANNEL 16
#define SUMD_BUFFSIZE (SUMD_MAX_CHANNEL * 2 + 5)
#define SUMD_BAUDRATE 115200
static bool sumdFrameDone = false;
static uint16_t sumdChannels[SUMD_MAX_CHANNEL];
static uint16_t crc;
static void sumdDataReceive(uint16_t c);
static uint16_t sumdReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan);
void taskHandleAnnex(void);
bool taskUpdateRxCheck(void);
void taskUpdateRxMain(void);
serialPort_t *sumdPort;
uartPort_t *sumdUart;
bool sumdInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{
    UNUSED(rxConfig);
    if (callback)
        *callback = sumdReadRawRC;
    rxRuntimeConfig->channelCount = SUMD_MAX_CHANNEL;
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }
    sumdPort = openSerialPort(portConfig->identifier, FUNCTION_RX_SERIAL, sumdDataReceive, SUMD_BAUDRATE, MODE_RX, SERIAL_NOT_INVERTED);
 sumdUart = (uartPort_t *)sumdPort;
#ifdef SERIALRX_DMA
 USART_DMACmd(sumdUart->USARTx, USART_DMAReq_Rx, DISABLE);
 DMA_Cmd(sumdUart->rxDMAStream, DISABLE);
 DMA_ClearFlag(sumdUart->rxDMAStream, sumdUart->rxTCIF);
 DMA_ClearFlag(sumdUart->rxDMAStream, sumdUart->rxHTIF);
 DMA_SetCurrDataCounter(sumdUart->rxDMAStream, SUMD_BUFFSIZE);
 DMA_Cmd(sumdUart->rxDMAStream, ENABLE);
 USART_DMACmd(sumdUart->USARTx, USART_DMAReq_Rx, ENABLE);
#endif
    return sumdPort != NULL;
}
#define CRC_POLYNOME 0x1021
static void CRC16(uint8_t value)
{
    uint8_t i;
    crc = crc ^ (int16_t)value << 8;
    for (i = 0; i < 8; i++) {
    if (crc & 0x8000)
        crc = (crc << 1) ^ CRC_POLYNOME;
    else
        crc = (crc << 1);
    }
}
static uint8_t sumd[SUMD_BUFFSIZE] = { 0, };
static uint8_t sumdChannelCount;
#ifdef SERIALRX_DMA
static void sumdDataReceive(uint16_t buff)
{
 (void)(buff);
 if (SKIP_RX) {
  resetTimeSinceRxPulse();
  return;
 }
 if (DMA_GetCurrDataCounter(sumdUart->rxDMAStream) == 0)
 {
  USART_DMACmd(sumdUart->USARTx, USART_DMAReq_Rx, DISABLE);
  memcpy((void *)&sumd, (void *)sumdPort->rxBuffer, SUMD_BUFFSIZE);
  sumdFrameDone = true;
  resetTimeSinceRxPulse();
  sumdChannelCount = sumd[2];
  for (uint8_t i = 0; i < sumdChannelCount * 2 + 3; i++)
  {
   CRC16(sumd[i]);
  }
  taskUpdateRxCheck();
  taskUpdateRxMain();
  taskHandleAnnex();
  DMA_ClearFlag(sumdUart->rxDMAStream, sumdUart->rxTCIF);
  DMA_ClearFlag(sumdUart->rxDMAStream, sumdUart->rxHTIF);
  DMA_Cmd(sumdUart->rxDMAStream, DISABLE);
  DMA_SetCurrDataCounter(sumdUart->rxDMAStream, SUMD_BUFFSIZE);
  DMA_Cmd(sumdUart->rxDMAStream, ENABLE);
  USART_DMACmd(sumdUart->USARTx, USART_DMAReq_Rx, ENABLE);
  crc = 0;
 }
}
#else
static void sumdDataReceive(uint16_t c)
{
    uint32_t sumdTime;
    static uint32_t sumdTimeLast;
    static uint8_t sumdIndex;
 if (SKIP_RX) {
  resetTimeSinceRxPulse();
  return;
 }
    sumdTime = micros();
    if ((sumdTime - sumdTimeLast) > 4000)
        sumdIndex = 0;
    sumdTimeLast = sumdTime;
    if (sumdIndex == 0) {
        if (c != SUMD_SYNCBYTE)
            return;
        else
        {
            sumdFrameDone = false;
            crc = 0;
        }
    }
    if (sumdIndex == 2)
        sumdChannelCount = (uint8_t)c;
    if (sumdIndex < SUMD_BUFFSIZE)
        sumd[sumdIndex] = (uint8_t)c;
    sumdIndex++;
    if (sumdIndex < sumdChannelCount * 2 + 4)
        CRC16((uint8_t)c);
    else
        if (sumdIndex == sumdChannelCount * 2 + 5) {
            sumdIndex = 0;
            sumdFrameDone = true;
            resetTimeSinceRxPulse();
        }
}
#endif
#define SUMD_OFFSET_CHANNEL_1_HIGH 3
#define SUMD_OFFSET_CHANNEL_1_LOW 4
#define SUMD_BYTES_PER_CHANNEL 2
#define SUMD_FRAME_STATE_OK 0x01
#define SUMD_FRAME_STATE_FAILSAFE 0x81
uint8_t sumdFrameStatus(void)
{
    uint8_t channelIndex;
    uint8_t frameStatus = SERIAL_RX_FRAME_PENDING;
    if (!sumdFrameDone) {
        return frameStatus;
    }
    sumdFrameDone = false;
    if (crc != ((sumd[SUMD_BYTES_PER_CHANNEL * sumdChannelCount + SUMD_OFFSET_CHANNEL_1_HIGH] << 8) |
            (sumd[SUMD_BYTES_PER_CHANNEL * sumdChannelCount + SUMD_OFFSET_CHANNEL_1_LOW])))
        return frameStatus;
    switch (sumd[1]) {
        case SUMD_FRAME_STATE_FAILSAFE:
            frameStatus = SERIAL_RX_FRAME_COMPLETE | SERIAL_RX_FRAME_FAILSAFE;
            break;
        case SUMD_FRAME_STATE_OK:
            frameStatus = SERIAL_RX_FRAME_COMPLETE;
            break;
        default:
            return frameStatus;
    }
    if (sumdChannelCount > SUMD_MAX_CHANNEL)
        sumdChannelCount = SUMD_MAX_CHANNEL;
    for (channelIndex = 0; channelIndex < sumdChannelCount; channelIndex++) {
        sumdChannels[channelIndex] = (
            (sumd[SUMD_BYTES_PER_CHANNEL * channelIndex + SUMD_OFFSET_CHANNEL_1_HIGH] << 8) |
            sumd[SUMD_BYTES_PER_CHANNEL * channelIndex + SUMD_OFFSET_CHANNEL_1_LOW]
        );
    }
    return frameStatus;
}
static uint16_t sumdReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    UNUSED(rxRuntimeConfig);
    return sumdChannels[chan] / 8;
}
