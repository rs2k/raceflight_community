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
#include "platform.h"
#include "system.h"
#include "io.h"
#include "rcc.h"
#include "nvic.h"
#include "serial.h"
#include "serial_uart.h"
#include "serial_uart_impl.h"
#define UART_RX_BUFFER_SIZE 4096
#define UART_TX_BUFFER_SIZE 4096
typedef enum UARTDevice {
    UARTDEV_1 = 0,
    UARTDEV_2 = 1,
    UARTDEV_3 = 2,
    UARTDEV_4 = 3,
    UARTDEV_5 = 4,
    UARTDEV_6 = 5
} UARTDevice;
typedef struct uartDevice_s {
    USART_TypeDef* dev;
    uartPort_t port;
    uint32_t DMAChannel;
    DMA_Stream_TypeDef *txDMAStream;
    DMA_Stream_TypeDef *rxDMAStream;
    ioTag_t rx;
    ioTag_t tx;
    volatile uint8_t rxBuffer[UART_RX_BUFFER_SIZE];
    volatile uint8_t txBuffer[UART_TX_BUFFER_SIZE];
    uint32_t rcc_ahb1;
    rccPeriphTag_t rcc_apb2;
    rccPeriphTag_t rcc_apb1;
    uint8_t af;
    uint8_t txIrq;
    uint8_t rxIrq;
    uint32_t txPriority;
    uint32_t rxPriority;
 uint32_t rxHTIF;
 uint32_t rxTCIF;
 uint32_t rxTEIF;
 uint32_t rxFEIF;
 uint32_t rxDMEIF;
} uartDevice_t;
#ifdef USE_USART1
static uartDevice_t uart1 =
{
    .DMAChannel = DMA_Channel_4,
    .txDMAStream = DMA2_Stream7,
#ifdef USE_USART1_RX_DMA
    .rxDMAStream = DMA2_Stream5,
#endif
    .dev = USART1,
    .rx = IO_TAG(USART1_RX_PIN),
    .tx = IO_TAG(USART1_TX_PIN),
    .af = GPIO_AF_USART1,
#ifdef USART1_AHB1_PERIPHERALS
    .rcc_ahb1 = USART1_AHB1_PERIPHERALS,
#endif
    .rcc_apb2 = RCC_APB2(USART1),
    .txIrq = DMA2_Stream7_IRQn,
    .rxIrq = USART1_IRQn,
    .txPriority = NVIC_PRIO_SERIALUART1_TXDMA,
    .rxPriority = NVIC_PRIO_SERIALUART1,
 .rxHTIF = DMA_FLAG_HTIF5,
 .rxTCIF = DMA_FLAG_TCIF5,
 .rxTEIF = DMA_FLAG_TEIF5,
 .rxFEIF = DMA_FLAG_FEIF5,
 .rxDMEIF = DMA_FLAG_DMEIF5
};
#endif
#ifdef USE_USART2
static uartDevice_t uart2 =
{
    .DMAChannel = DMA_Channel_4,
#ifdef USE_USART2_RX_DMA
    .rxDMAStream = DMA1_Stream5,
#endif
    .txDMAStream = DMA1_Stream6,
    .dev = USART2,
    .rx = IO_TAG(USART2_RX_PIN),
    .tx = IO_TAG(USART2_TX_PIN),
    .af = GPIO_AF_USART2,
#ifdef USART2_AHB1_PERIPHERALS
    .rcc_ahb1 = USART2_AHB1_PERIPHERALS,
#endif
    .rcc_apb1 = RCC_APB1(USART2),
    .txIrq = DMA1_Stream6_IRQn,
    .rxIrq = USART2_IRQn,
    .txPriority = NVIC_PRIO_SERIALUART2_TXDMA,
    .rxPriority = NVIC_PRIO_SERIALUART2,
 .rxHTIF = DMA_FLAG_HTIF5,
 .rxTCIF = DMA_FLAG_TCIF5,
 .rxTEIF = DMA_FLAG_TEIF5,
 .rxFEIF = DMA_FLAG_FEIF5,
 .rxDMEIF = DMA_FLAG_DMEIF5
};
#endif
#ifdef USE_USART3
static uartDevice_t uart3 =
{
    .DMAChannel = DMA_Channel_4,
#ifdef USE_USART3_RX_DMA
    .rxDMAStream = DMA1_Stream1,
#endif
    .txDMAStream = DMA1_Stream3,
    .dev = USART3,
    .rx = IO_TAG(USART3_RX_PIN),
    .tx = IO_TAG(USART3_TX_PIN),
    .af = GPIO_AF_USART3,
#ifdef USART3_AHB1_PERIPHERALS
    .rcc_ahb1 = USART3_AHB1_PERIPHERALS,
#endif
    .rcc_apb1 = RCC_APB1(USART3),
    .txIrq = DMA1_Stream3_IRQn,
    .rxIrq = USART3_IRQn,
    .txPriority = NVIC_PRIO_SERIALUART3_TXDMA,
    .rxPriority = NVIC_PRIO_SERIALUART3,
 .rxHTIF = DMA_FLAG_HTIF1,
 .rxTCIF = DMA_FLAG_TCIF1,
 .rxTEIF = DMA_FLAG_TEIF1,
 .rxFEIF = DMA_FLAG_FEIF1,
 .rxDMEIF = DMA_FLAG_DMEIF1
};
#endif
#ifdef USE_USART4
static uartDevice_t uart4 =
{
    .DMAChannel = DMA_Channel_4,
#ifdef USE_USART4_RX_DMA
    .rxDMAStream = DMA1_Stream2,
#endif
    .txDMAStream = DMA1_Stream4,
    .dev = UART4,
    .rx = IO_TAG(USART4_RX_PIN),
    .tx = IO_TAG(USART4_TX_PIN),
    .af = GPIO_AF_UART4,
#ifdef USART4_AHB1_PERIPHERALS
    .rcc_ahb1 = USART4_AHB1_PERIPHERALS,
#endif
    .rcc_apb1 = RCC_APB1(UART4),
    .txIrq = DMA1_Stream4_IRQn,
    .rxIrq = UART4_IRQn,
    .txPriority = NVIC_PRIO_SERIALUART4_TXDMA,
    .rxPriority = NVIC_PRIO_SERIALUART4,
 .rxHTIF = DMA_FLAG_HTIF2,
 .rxTCIF = DMA_FLAG_TCIF2,
 .rxTEIF = DMA_FLAG_TEIF2,
 .rxFEIF = DMA_FLAG_FEIF2,
 .rxDMEIF = DMA_FLAG_DMEIF2
};
#endif
#ifdef USE_USART5
static uartDevice_t uart5 =
{
    .DMAChannel = DMA_Channel_4,
#ifdef USE_USART1_RX_DMA
    .rxDMAStream = DMA1_Stream0,
#endif
    .txDMAStream = DMA1_Stream7,
    .dev = UART5,
    .rx = IO_TAG(USART5_RX_PIN),
    .tx = IO_TAG(USART5_TX_PIN),
    .af = GPIO_AF_UART5,
#ifdef USART5_AHB1_PERIPHERALS
    .rcc_ahb1 = USART5_AHB1_PERIPHERALS,
#endif
    .rcc_apb1 = RCC_APB1(UART5),
    .txIrq = DMA1_Stream7_IRQn,
    .rxIrq = UART5_IRQn,
    .txPriority = NVIC_PRIO_SERIALUART5_TXDMA,
    .rxPriority = NVIC_PRIO_SERIALUART5,
 .rxHTIF = DMA_FLAG_HTIF0,
 .rxTCIF = DMA_FLAG_TCIF0,
 .rxTEIF = DMA_FLAG_TEIF0,
 .rxFEIF = DMA_FLAG_FEIF0,
 .rxDMEIF = DMA_FLAG_DMEIF0
};
#endif
#ifdef USE_USART6
static uartDevice_t uart6 =
{
    .DMAChannel = DMA_Channel_5,
#ifdef USE_USART6_RX_DMA
    .rxDMAStream = DMA2_Stream1,
#endif
    .txDMAStream = DMA2_Stream6,
    .dev = USART6,
    .rx = IO_TAG(USART6_RX_PIN),
    .tx = IO_TAG(USART6_TX_PIN),
    .af = GPIO_AF_USART6,
#ifdef USART6_AHB1_PERIPHERALS
    .rcc_ahb1 = USART6_AHB1_PERIPHERALS,
#endif
    .rcc_apb2 = RCC_APB2(USART6),
    .txIrq = DMA2_Stream6_IRQn,
    .rxIrq = USART6_IRQn,
    .txPriority = NVIC_PRIO_SERIALUART6_TXDMA,
    .rxPriority = NVIC_PRIO_SERIALUART6,
 .rxHTIF = DMA_FLAG_HTIF1,
 .rxTCIF = DMA_FLAG_TCIF1,
 .rxTEIF = DMA_FLAG_TEIF1,
 .rxFEIF = DMA_FLAG_FEIF1,
 .rxDMEIF = DMA_FLAG_DMEIF1
};
#endif
static uartDevice_t* uartHardwareMap[] = {
#ifdef USE_USART1
    &uart1,
#else
    NULL,
#endif
#ifdef USE_USART2
    &uart2,
#else
    NULL,
#endif
#ifdef USE_USART3
    &uart3,
#else
    NULL,
#endif
#ifdef USE_USART4
    &uart4,
#else
    NULL,
#endif
#ifdef USE_USART5
    &uart5,
#else
    NULL,
#endif
#ifdef USE_USART6
    &uart6,
#else
    NULL,
#endif
    };
uint32_t interruptCount1 = 0;
uint32_t interruptCount2 = 0;
uint32_t interruptCount3 = 0;
uint32_t interruptCount4 = 0;
uint32_t interruptCount5 = 0;
uint32_t intervalTime = 0;
uint32_t prevTime = 0;
uint8_t FLAG_NE = 0;
uint8_t FLAG_FE = 0;
uint8_t FLAG_PE = 0;
uint8_t FLAG_ORE = 0;
uint8_t FLAG_IDLE = 0;
uint8_t FLAG_RXNE = 0;
uint8_t FLAG_TXE = 0;
void usartIrqHandler(uartPort_t *s)
{
 intervalTime = micros() - prevTime;
 prevTime = micros();
 FLAG_NE = USART_GetFlagStatus(s->USARTx, USART_FLAG_NE);
 FLAG_FE = USART_GetFlagStatus(s->USARTx, USART_FLAG_FE);
 FLAG_PE = USART_GetFlagStatus(s->USARTx, USART_FLAG_PE);
 FLAG_ORE = USART_GetFlagStatus(s->USARTx, USART_FLAG_ORE);
 FLAG_IDLE = USART_GetFlagStatus(s->USARTx, USART_FLAG_IDLE);
 FLAG_RXNE = USART_GetFlagStatus(s->USARTx, USART_FLAG_RXNE);
 FLAG_TXE = USART_GetFlagStatus(s->USARTx, USART_FLAG_TXE);
 if (USART_GetITStatus(s->USARTx, USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE | USART_FLAG_ORE))
 {
  if (s->rxDMAStream)
  {
   USART_DMACmd(s->USARTx, USART_DMAReq_Rx, DISABLE);
   if (DMA_GetFlagStatus(s->rxDMAStream, s->rxTCIF))
   {
    DMA_ClearFlag(s->rxDMAStream, s->rxTCIF);
    DMA_ClearFlag(s->rxDMAStream, s->rxHTIF);
    if (DMA_GetFlagStatus(s->rxDMAStream, s->rxFEIF) == SET)
    {
     DMA_ClearFlag(s->rxDMAStream, s->rxFEIF);
    }
    DMA_Cmd(s->rxDMAStream, DISABLE);
   }
   if (DMA_GetFlagStatus(s->rxDMAStream, s->rxTEIF) == SET)
   {
    DMA_ClearFlag(s->rxDMAStream, s->rxTEIF);
   }
   if (DMA_GetFlagStatus(s->rxDMAStream, s->rxDMEIF) == SET)
   {
    DMA_ClearFlag(s->rxDMAStream, s->rxDMEIF);
   }
  }
  USART_ReceiveData(s->USARTx);
  USART_ClearITPendingBit(s->USARTx, USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE | USART_FLAG_ORE);
  if (s->rxDMAStream)
  {
   DMA_SetCurrDataCounter(s->rxDMAStream, 16);
   USART_DMACmd(s->USARTx, USART_DMAReq_Rx, ENABLE);
   DMA_Cmd(s->rxDMAStream, ENABLE);
  }
  USART_ClearITPendingBit(s->USARTx, USART_IT_IDLE);
 }
 else
 {
  if ((USART_GetITStatus(s->USARTx, USART_IT_RXNE) == SET)) {
   if (s->port.callback) {
    s->port.callback(s->USARTx->DR);
   }
   else {
    s->port.rxBuffer[s->port.rxBufferHead] = s->USARTx->DR;
    s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
   }
  }
  if ((USART_GetITStatus(s->USARTx, USART_IT_TXE) == SET)) {
   if (s->port.txBufferTail != s->port.txBufferHead) {
    USART_SendData(s->USARTx, s->port.txBuffer[s->port.txBufferTail]);
    s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
   }
   else {
    USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
   }
  }
  if ((USART_GetITStatus(s->USARTx, USART_IT_IDLE) == SET)) {
   USART_GetITStatus(s->USARTx, USART_IT_IDLE);
   USART_ReceiveData(s->USARTx);
   if (s->port.callback) {
    s->port.callback((uint32_t)s->port.rxBuffer);
   }
   else {
    s->port.rxBuffer[s->port.rxBufferHead] = s->USARTx->DR;
    s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
   }
  }
 }
 if (s->USARTx == USART1)
 {
  interruptCount1++;
 }
 if (s->USARTx == USART2)
 {
  interruptCount2++;
 }
 if (s->USARTx == USART3)
 {
  interruptCount3++;
 }
 if (s->USARTx == UART4)
 {
  interruptCount4++;
 }
 if (s->USARTx == UART5)
 {
  interruptCount5++;
 }
}
static void handleUsartTxDma(uartPort_t *s)
{
    DMA_Cmd(s->txDMAStream, DISABLE);
    if (s->port.txBufferHead != s->port.txBufferTail)
        uartStartTxDMA(s);
    else
        s->txDMAEmpty = true;
}
uartPort_t *serialUSART(UARTDevice device, uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartPort_t *s;
    NVIC_InitTypeDef NVIC_InitStructure;
    uartDevice_t *uart = uartHardwareMap[device];
    if (!uart) return NULL;
    s = &(uart->port);
    s->port.vTable = uartVTable;
    s->port.baudRate = baudRate;
    s->port.rxBuffer = uart->rxBuffer;
    s->port.txBuffer = uart->txBuffer;
    s->port.rxBufferSize = sizeof(uart->rxBuffer);
    s->port.txBufferSize = sizeof(uart->txBuffer);
    s->USARTx = uart->dev;
    if (uart->rxDMAStream) {
        s->rxDMAChannel = uart->DMAChannel;
        s->rxDMAStream = uart->rxDMAStream;
     s->rxHTIF = uart->rxHTIF;
     s->rxTCIF = uart->rxTCIF;
     s->rxTEIF = uart->rxTEIF;
     s->rxFEIF = uart->rxFEIF;
     s->rxDMEIF = uart->rxDMEIF;
    }
    s->txDMAChannel = uart->DMAChannel;
    s->txDMAStream = uart->txDMAStream;
    s->txDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;
    s->rxDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;
    IO_t tx = IOGetByTag(uart->tx);
    IO_t rx = IOGetByTag(uart->rx);
    if (uart->rcc_apb2)
        RCC_ClockCmd(uart->rcc_apb2, ENABLE);
    if (uart->rcc_apb1)
        RCC_ClockCmd(uart->rcc_apb1, ENABLE);
    if (uart->rcc_ahb1)
        RCC_AHB1PeriphClockCmd(uart->rcc_ahb1, ENABLE);
    if (options & SERIAL_BIDIR) {
        IOInit(tx, OWNER_SERIAL_TX, RESOURCE_USART);
     IOConfigGPIOAF(tx, IOCFG_AF_PP, uart->af);
    }
    else {
        if (mode & MODE_TX) {
            IOInit(tx, OWNER_SERIAL_TX, RESOURCE_USART);
            IOConfigGPIOAF(tx, IOCFG_AF_PP, uart->af);
        }
        if (mode & MODE_RX) {
            IOInit(rx, OWNER_SERIAL_RX, RESOURCE_USART);
            IOConfigGPIOAF(rx, IOCFG_AF_PP, uart->af);
        }
    }
    NVIC_InitStructure.NVIC_IRQChannel = uart->txIrq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(uart->txPriority);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(uart->txPriority);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
        NVIC_InitStructure.NVIC_IRQChannel = uart->rxIrq;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(uart->rxPriority);
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(uart->rxPriority);
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    return s;
}
#ifdef USE_USART1
uartPort_t *serialUSART1(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUSART(UARTDEV_1, baudRate, mode, options);
}
void DMA2_Stream7_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_1]->port);
    if(DMA_GetITStatus(s->txDMAStream,DMA_IT_TCIF7))
    {
     DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_TCIF7);
     DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_HTIF7);
  if(DMA_GetFlagStatus(s->txDMAStream,DMA_IT_FEIF7)==SET)
  {
   DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_FEIF7);
  }
  handleUsartTxDma(s);
    }
 if(DMA_GetFlagStatus(s->txDMAStream,DMA_IT_TEIF7)==SET)
 {
  DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_TEIF7);
 }
 if(DMA_GetFlagStatus(s->txDMAStream,DMA_IT_DMEIF7)==SET)
 {
  DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_DMEIF7);
 }
}
void USART1_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_1]->port);
    usartIrqHandler(s);
}
#endif
#ifdef USE_USART2
uartPort_t *serialUSART2(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUSART(UARTDEV_2, baudRate, mode, options);
}
void DMA1_Stream6_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_2]->port);
 if(DMA_GetITStatus(s->txDMAStream,DMA_IT_TCIF6))
 {
  DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_TCIF6);
  DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_HTIF6);
  if(DMA_GetFlagStatus(s->txDMAStream,DMA_IT_FEIF6)==SET)
  {
   DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_FEIF6);
  }
  handleUsartTxDma(s);
 }
 if(DMA_GetFlagStatus(s->txDMAStream,DMA_IT_TEIF6)==SET)
 {
  DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_TEIF6);
 }
 if(DMA_GetFlagStatus(s->txDMAStream,DMA_IT_DMEIF6)==SET)
 {
  DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_DMEIF6);
 }
}
void USART2_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_2]->port);
 usartIrqHandler(s);
}
#endif
#ifdef USE_USART3
uartPort_t *serialUSART3(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUSART(UARTDEV_3, baudRate, mode, options);
}
void DMA1_Stream3_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_3]->port);
    if(DMA_GetITStatus(s->txDMAStream,DMA_IT_TCIF3))
    {
     DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_TCIF3);
     DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_HTIF3);
  if(DMA_GetFlagStatus(s->txDMAStream,DMA_IT_FEIF3)==SET)
  {
   DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_FEIF3);
  }
  handleUsartTxDma(s);
    }
 if(DMA_GetFlagStatus(s->txDMAStream,DMA_IT_TEIF3)==SET)
 {
  DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_TEIF3);
 }
 if(DMA_GetFlagStatus(s->txDMAStream,DMA_IT_DMEIF3)==SET)
 {
  DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_DMEIF3);
 }
}
void USART3_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_3]->port);
    usartIrqHandler(s);
}
#endif
#ifdef USE_USART4
uartPort_t *serialUSART4(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUSART(UARTDEV_4, baudRate, mode, options);
}
void DMA1_Stream4_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_4]->port);
    if(DMA_GetITStatus(s->txDMAStream,DMA_IT_TCIF4))
    {
     DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_TCIF4);
     DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_HTIF4);
  if(DMA_GetFlagStatus(s->txDMAStream,DMA_IT_FEIF4)==SET)
  {
   DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_FEIF4);
  }
  handleUsartTxDma(s);
    }
 if(DMA_GetFlagStatus(s->txDMAStream,DMA_IT_TEIF4)==SET)
 {
  DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_TEIF4);
 }
 if(DMA_GetFlagStatus(s->txDMAStream,DMA_IT_DMEIF4)==SET)
 {
  DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_DMEIF4);
 }
}
void UART4_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_4]->port);
    usartIrqHandler(s);
}
#endif
#ifdef USE_USART5
uartPort_t *serialUSART5(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUSART(UARTDEV_5, baudRate, mode, options);
}
void UART5_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_5]->port);
    usartIrqHandler(s);
}
#endif
#ifdef USE_USART6
uartPort_t *serialUSART6(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    return serialUSART(UARTDEV_6, baudRate, mode, options);
}
void DMA2_Stream6_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_6]->port);
    if(DMA_GetITStatus(s->txDMAStream,DMA_IT_TCIF6))
    {
     DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_TCIF6);
     DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_HTIF6);
  if(DMA_GetFlagStatus(s->txDMAStream,DMA_IT_FEIF6)==SET)
  {
   DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_FEIF6);
  }
  handleUsartTxDma(s);
    }
 if(DMA_GetFlagStatus(s->txDMAStream,DMA_IT_TEIF6)==SET)
 {
  DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_TEIF6);
 }
 if(DMA_GetFlagStatus(s->txDMAStream,DMA_IT_DMEIF6)==SET)
 {
  DMA_ClearITPendingBit(s->txDMAStream,DMA_IT_DMEIF6);
 }
}
void USART6_IRQHandler(void)
{
    uartPort_t *s = &(uartHardwareMap[UARTDEV_6]->port);
    usartIrqHandler(s);
}
#endif
