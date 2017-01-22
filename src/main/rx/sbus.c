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
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "io/serial.h"
#include "scheduler.h"
#define SBUS_TIME_NEEDED_PER_FRAME 3000
#undef DEBUG_SBUS_PACKETS
#ifdef DEBUG_SBUS_PACKETS
static uint16_t sbusStateFlags = 0;
#define SBUS_STATE_FAILSAFE (1 << 0)
#define SBUS_STATE_SIGNALLOSS (1 << 1)
#endif
#define SBUS_MAX_CHANNEL 18
#define SBUS_FRAME_SIZE 25
#define SBUS_FRAME_BEGIN_BYTE 0x0F
#define SBUS_BAUDRATE 100000
#ifdef KISS
#define SBUS_PORT_OPTIONS (SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN | SERIAL_INVERTED | SERIAL_BIDIR)
#else
#define SBUS_PORT_OPTIONS (SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN | SERIAL_INVERTED)
#endif
#define SBUS_DIGITAL_CHANNEL_MIN 173
#define SBUS_DIGITAL_CHANNEL_MAX 1812
static bool sbusFrameDone = false;
static void sbusDataReceive(uint16_t c);
static uint16_t sbusReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan);
static uint32_t sbusChannelData[SBUS_MAX_CHANNEL];
void taskHandleAnnex(void);
bool taskUpdateRxCheck(void);
void taskUpdateRxMain(void);
serialPort_t *sBusPort;
uartPort_t *sBusUart;
bool sbusInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{
    int b;
    for (b = 0; b < SBUS_MAX_CHANNEL; b++)
        sbusChannelData[b] = (16 * rxConfig->midrc) / 10 - 1408;
    if (callback)
        *callback = sbusReadRawRC;
    rxRuntimeConfig->channelCount = SBUS_MAX_CHANNEL;
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }
    portOptions_t options = SBUS_PORT_OPTIONS;
    if (!feature(FEATURE_SBUS_INVERTER)) options = options & ~SERIAL_INVERTED;
    sBusPort = openSerialPort(portConfig->identifier, FUNCTION_RX_SERIAL, sbusDataReceive, SBUS_BAUDRATE, MODE_RX, options);
 sBusUart = (uartPort_t *)sBusPort;
#ifdef SERIALRX_DMA
 USART_DMACmd(sBusUart->USARTx, USART_DMAReq_Rx, DISABLE);
 DMA_Cmd(sBusUart->rxDMAStream, DISABLE);
 DMA_ClearFlag(sBusUart->rxDMAStream, sBusUart->rxTCIF);
 DMA_ClearFlag(sBusUart->rxDMAStream, sBusUart->rxHTIF);
 DMA_SetCurrDataCounter(sBusUart->rxDMAStream, SBUS_FRAME_SIZE);
 DMA_Cmd(sBusUart->rxDMAStream, ENABLE);
 USART_DMACmd(sBusUart->USARTx, USART_DMAReq_Rx, ENABLE);
#endif
    return sBusPort != NULL;
}
#define SBUS_FLAG_CHANNEL_17 (1 << 0)
#define SBUS_FLAG_CHANNEL_18 (1 << 1)
#define SBUS_FLAG_SIGNAL_LOSS (1 << 2)
#define SBUS_FLAG_FAILSAFE_ACTIVE (1 << 3)
struct sbusFrame_s {
    uint8_t syncByte;
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
    uint8_t flags;
    uint8_t endByte;
} __attribute__ ((__packed__));
typedef union {
    uint8_t bytes[SBUS_FRAME_SIZE];
    struct sbusFrame_s frame;
} sbusFrame_t;
static sbusFrame_t sbusFrame;
#ifdef SERIALRX_DMA
static void sbusDataReceive(uint16_t buff)
{
 if (SKIP_RX) {
  resetTimeSinceRxPulse();
  return;
 }
 (void)(buff);
 if (DMA_GetCurrDataCounter(sBusUart->rxDMAStream) == 0)
 {
  USART_DMACmd(sBusUart->USARTx, USART_DMAReq_Rx, DISABLE);
  memcpy((void *)&sbusFrame.bytes, (void *)sBusPort->rxBuffer, SBUS_FRAME_SIZE);
  sbusFrameDone = true;
  resetTimeSinceRxPulse();
  taskUpdateRxCheck();
  taskUpdateRxMain();
  taskHandleAnnex();
  DMA_ClearFlag(sBusUart->rxDMAStream, sBusUart->rxTCIF);
  DMA_ClearFlag(sBusUart->rxDMAStream, sBusUart->rxHTIF);
  DMA_Cmd(sBusUart->rxDMAStream, DISABLE);
  DMA_SetCurrDataCounter(sBusUart->rxDMAStream, SBUS_FRAME_SIZE);
  DMA_Cmd(sBusUart->rxDMAStream, ENABLE);
  USART_DMACmd(sBusUart->USARTx, USART_DMAReq_Rx, ENABLE);
 }
}
#else
static void sbusDataReceive(uint16_t c)
{
    static uint8_t sbusFramePosition = 0;
    static uint32_t sbusFrameStartAt = 0;
 if (SKIP_RX) {
  resetTimeSinceRxPulse();
  return;
 }
    uint32_t now = micros();
    int32_t sbusFrameTime = now - sbusFrameStartAt;
    if (sbusFrameTime > (long)(SBUS_TIME_NEEDED_PER_FRAME + 500)) {
        sbusFramePosition = 0;
    }
    if (sbusFramePosition == 0) {
        if (c != SBUS_FRAME_BEGIN_BYTE) {
            return;
        }
        sbusFrameStartAt = now;
    }
    if (sbusFramePosition < SBUS_FRAME_SIZE) {
        sbusFrame.bytes[sbusFramePosition++] = (uint8_t)c;
        if (sbusFramePosition < SBUS_FRAME_SIZE) {
            sbusFrameDone = false;
        } else {
            sbusFrameDone = true;
            resetTimeSinceRxPulse();
#ifdef DEBUG_SBUS_PACKETS
        debug[2] = sbusFrameTime;
#endif
        }
    }
}
#endif
uint8_t sbusFrameStatus(void)
{
    if (!sbusFrameDone) {
        return SERIAL_RX_FRAME_PENDING;
    }
    sbusFrameDone = false;
#ifdef DEBUG_SBUS_PACKETS
    sbusStateFlags = 0;
    debug[1] = sbusFrame.frame.flags;
#endif
    sbusChannelData[0] = sbusFrame.frame.chan0;
    sbusChannelData[1] = sbusFrame.frame.chan1;
    sbusChannelData[2] = sbusFrame.frame.chan2;
    sbusChannelData[3] = sbusFrame.frame.chan3;
    sbusChannelData[4] = sbusFrame.frame.chan4;
    sbusChannelData[5] = sbusFrame.frame.chan5;
    sbusChannelData[6] = sbusFrame.frame.chan6;
    sbusChannelData[7] = sbusFrame.frame.chan7;
    sbusChannelData[8] = sbusFrame.frame.chan8;
    sbusChannelData[9] = sbusFrame.frame.chan9;
    sbusChannelData[10] = sbusFrame.frame.chan10;
    sbusChannelData[11] = sbusFrame.frame.chan11;
    sbusChannelData[12] = sbusFrame.frame.chan12;
    sbusChannelData[13] = sbusFrame.frame.chan13;
    sbusChannelData[14] = sbusFrame.frame.chan14;
    sbusChannelData[15] = sbusFrame.frame.chan15;
    if (sbusFrame.frame.flags & SBUS_FLAG_CHANNEL_17) {
        sbusChannelData[16] = SBUS_DIGITAL_CHANNEL_MAX;
    } else {
        sbusChannelData[16] = SBUS_DIGITAL_CHANNEL_MIN;
    }
    if (sbusFrame.frame.flags & SBUS_FLAG_CHANNEL_18) {
        sbusChannelData[17] = SBUS_DIGITAL_CHANNEL_MAX;
    } else {
        sbusChannelData[17] = SBUS_DIGITAL_CHANNEL_MIN;
    }
    if (sbusFrame.frame.flags & SBUS_FLAG_SIGNAL_LOSS) {
#ifdef DEBUG_SBUS_PACKETS
        sbusStateFlags |= SBUS_STATE_SIGNALLOSS;
        debug[0] = sbusStateFlags;
#endif
    }
    if (sbusFrame.frame.flags & SBUS_FLAG_FAILSAFE_ACTIVE) {
#ifdef DEBUG_SBUS_PACKETS
        sbusStateFlags |= SBUS_STATE_FAILSAFE;
        debug[0] = sbusStateFlags;
#endif
        return SERIAL_RX_FRAME_COMPLETE | SERIAL_RX_FRAME_FAILSAFE;
    }
#ifdef DEBUG_SBUS_PACKETS
    debug[0] = sbusStateFlags;
#endif
    return SERIAL_RX_FRAME_COMPLETE;
}
static uint16_t sbusReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    UNUSED(rxRuntimeConfig);
    return (0.625f * sbusChannelData[chan]) + 880;
}
