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
#pragma once 
       
#include "io/serial.h"
#if defined(STM32F40_41xxx) || defined (STM32F411xE) || defined(STM32F303xC)
#define UART1_RX_BUFFER_SIZE 256
#define UART1_TX_BUFFER_SIZE 256
#define UART2_RX_BUFFER_SIZE 256
#define UART2_TX_BUFFER_SIZE 256
#define UART3_RX_BUFFER_SIZE 256
#define UART3_TX_BUFFER_SIZE 256
#define UART4_RX_BUFFER_SIZE 256
#define UART4_TX_BUFFER_SIZE 256
#define UART5_RX_BUFFER_SIZE 256
#define UART5_TX_BUFFER_SIZE 256
#define UART6_RX_BUFFER_SIZE 256
#define UART6_TX_BUFFER_SIZE 256
#else
#define UART1_RX_BUFFER_SIZE 256
#define UART1_TX_BUFFER_SIZE 256
#define UART2_RX_BUFFER_SIZE 256
#define UART2_TX_BUFFER_SIZE 256
#define UART3_RX_BUFFER_SIZE 256
#define UART3_TX_BUFFER_SIZE 256
#define UART4_RX_BUFFER_SIZE 256
#define UART4_TX_BUFFER_SIZE 256
#define UART5_RX_BUFFER_SIZE 256
#define UART5_TX_BUFFER_SIZE 256
#define UART6_RX_BUFFER_SIZE 256
#define UART6_TX_BUFFER_SIZE 256
#endif
typedef struct {
    serialPort_t port;
#if defined(STM32F40_41xxx) || defined (STM32F411xE) || defined(STM32F446xx)
    DMA_Stream_TypeDef *rxDMAStream;
    DMA_Stream_TypeDef *txDMAStream;
    uint32_t rxDMAChannel;
    uint32_t txDMAChannel;
#else
    DMA_Channel_TypeDef *rxDMAChannel;
    DMA_Channel_TypeDef *txDMAChannel;
#endif
    uint32_t rxDMAIrq;
    uint32_t txDMAIrq;
    uint32_t rxDMAPos;
    bool txDMAEmpty;
    uint32_t txDMAPeripheralBaseAddr;
    uint32_t rxDMAPeripheralBaseAddr;
    USART_TypeDef *USARTx;
 uint32_t rxHTIF;
 uint32_t rxTCIF;
 uint32_t rxTEIF;
 uint32_t rxFEIF;
 uint32_t rxDMEIF;
} uartPort_t;
serialPort_t *uartOpen(USART_TypeDef *USARTx, serialPortFunction_e function, serialReceiveCallbackPtr callback, uint32_t baudRate, portMode_t mode, portOptions_t options);
void uartWrite(serialPort_t *instance, uint8_t ch);
uint32_t uartTotalRxBytesWaiting(serialPort_t *instance);
uint8_t uartTotalTxBytesFree(serialPort_t *instance);
uint8_t uartRead(serialPort_t *instance);
void uartSetBaudRate(serialPort_t *s, uint32_t baudRate);
bool isUartTransmitBufferEmpty(serialPort_t *s);
