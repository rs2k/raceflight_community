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
#ifdef TELEMETRY
#include "common/maths.h"
#include "common/axis.h"
#include "drivers/system.h"
#include "drivers/rcc.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/serial.h"
#include "drivers/gyro_sync.h"
#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/altitudehold.h"
#include "flight/navigation.h"
#include "io/beeper.h"
#include "io/serial.h"
#include "io/gimbal.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"
#include "io/gps.h"
#include "rx/rx.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/altitudehold.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "io/serial.h"
#include "watchdog.h"
#include "telemetry/telemetry.h"
#include "spektrumTelemetry.h"
#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"
extern uartPort_t *spektrumUart;
STR_SRXL_TELEM telemetry;
STR_SRXL_BIND bind;
DMA_InitTypeDef DMA_InitStructure;
uint8_t dma_count;
TELEMETRY_STATE telemetryState = TELEM_START;
UN_TELEMETRY sensorData;
#define UINT16_ENDIAN(a) (((a) >> 8) | ((a) << 8) )
void sendSpektrumTelem(void)
{
 if (telemetryState >= NUM_TELEM_STATES)
 {
  telemetryState = TELEM_START;
 }
 telemetry.packet.SRXL_ID = SPEKTRUM_SRXL_ID;
 telemetry.packet.identifier = SRXL_TELEM_ID;
 telemetry.packet.length = SRXL_TELEM_LENGTH;
 switch (telemetryState)
 {
 case TELEM_FLIGHTLOG:
  {
   telemetry.packet.data.flightLog.id = FLIGHTLOG_ID;
   telemetry.packet.data.flightLog.A = 0xFFFF;
   telemetry.packet.data.flightLog.B = 0xFFFF;
   telemetry.packet.data.flightLog.L = 0xFFFF;
   telemetry.packet.data.flightLog.R = 0xFFFF;
   telemetry.packet.data.flightLog.F = 0xFFFF;
   telemetry.packet.data.flightLog.H = 0xFFFF;
   telemetry.packet.data.flightLog.rfu = 0;
   telemetry.packet.data.flightLog.rxVoltage = 0xFFFF;
   telemetryState++;
   break;
  }
 case TELEM_INTERNAL:
  {
   telemetry.packet.data.internalSensors.id = INTERNAL_ID;
   telemetry.packet.data.internalSensors.packVoltage = UINT16_ENDIAN(vbat * 10);
   telemetry.packet.data.internalSensors.rpm = 0xFFFF;
   telemetry.packet.data.internalSensors.temperature = 0x7FFF;
   telemetry.packet.data.internalSensors.rfu = 0;
   memset(telemetry.packet.data.internalSensors.spare, 0, 8);
   telemetryState++;
   break;
  }
 case TELEM_XBUS:
  {
   xbus.sensorCount = 3;
   if (xbus.sensorCount > 0)
   {
    switch (xbus.sensorPosition)
    {
    case 0:
     {
      sensorData.fpMAH.identifier = TELE_DEVICE_FP_MAH;
      sensorData.fpMAH.sID = 0;
      sensorData.fpMAH.current_A = amperage / 10;
      sensorData.fpMAH.chargeUsed_A = mAhDrawn;
      sensorData.fpMAH.temp_A = 0x7FFF;
      sensorData.fpMAH.current_B = 0x7FFF;
      sensorData.fpMAH.chargeUsed_B = 0x7FFF;
      sensorData.fpMAH.temp_B = 0x7FFF;
      break;
     }
    case 1:
     {
      sensorData.user_16SU.identifier = TELE_DEVICE_USER_16SU;
      sensorData.user_16SU.sID = 0x00;
      sensorData.user_16SU.sField1 = 1;
      sensorData.user_16SU.sField2 = 2;
      sensorData.user_16SU.sField3 = 3;
      sensorData.user_16SU.uField1 = 4;
      sensorData.user_16SU.uField2 = 5;
      sensorData.user_16SU.uField3 = 6;
      sensorData.user_16SU.uField4 = 7;
      break;
     }
    case 2:
     {
      sensorData.user_text.identifier = TELE_DEVICE_TEXTGEN;
      sensorData.user_text.sID = 0x00;
      sensorData.user_text.lineNumber = xbus.textLine;
      char *tempString[9] = { "SPEKTRUM", "MIGUEL", "WAS", "HEREEEE", "","" ,"" ,"","" };
      memcpy(sensorData.user_text.text, tempString[xbus.textLine], 13);
      xbus.textLine++;
      if (xbus.textLine > 9)
      {
       xbus.textLine = 0;
      }
      break;
     }
    case 3:
     {
      sensorData.user_16SU32S.identifier = TELE_DEVICE_USER_16SU32S;
      sensorData.user_16SU32S.sID = 0x00;
      sensorData.user_16SU32S.sField1 = 14;
      sensorData.user_16SU32S.sField2 = 15;
      sensorData.user_16SU32S.uField1 = 16;
      sensorData.user_16SU32S.uField2 = 17;
      sensorData.user_16SU32S.uField3 = 18;
      sensorData.user_16SU32S.s32Field = 19;
      break;
     }
    case 4:
     {
      sensorData.user_16U32SU.identifier = TELE_DEVICE_USER_16U32SU;
      sensorData.user_16U32SU.sID = 0x00;
      sensorData.user_16U32SU.uField1 = 20;
      sensorData.user_16U32SU.s32Field = 21;
      sensorData.user_16U32SU.u32Field1 = 22;
      sensorData.user_16U32SU.u32Field2 = 23;
      break;
     }
    }
    memcpy(&telemetry.packet.data, &sensorData, 16);
    xbus.sensorPosition++;
    xbus.sensorPosition = xbus.sensorPosition % xbus.sensorCount;
   }
   telemetryState++;
   break;
  }
 }
 uint16_t crc = 0x0000;
 for (uint8_t i = 0; i < 16; i++)
  crc = srxlCrc16(crc, telemetry.packet.data.raw[i], SRXL_POLY);
 telemetry.packet.crc = crc;
 sendSpektrumSRXL((uint32_t)&telemetry, SRXL_TELEM_LENGTH);
}
void sendSpektrumBind()
{
 bind.srxlID = SPEKTRUM_SRXL_ID;
 bind.subID = SRXL_BIND_ID;
 bind.length = SRXL_BIND_LENGTH;
 bind.data.request = SRXL_BIND_ENTER;
 bind.data.guid = 0;
 bind.data.type = 0;
 bind.data.rfID = 0;
 uint16_t crc = 0x0000;
 for (uint8_t i = 0; i < SRXLBIND_PAYLOAD_LEN; i++)
  crc = srxlCrc16(crc, bind.data.raw[i], SRXL_POLY);
 bind.crc = crc;
 sendSpektrumSRXL((uint32_t)&bind, SRXL_BIND_LENGTH);
}
void sendSpektrumSRXL(uint32_t baseAddress, uint8_t packetSize)
{
 while (DMA_GetCurrDataCounter(spektrumUart->txDMAStream) != 0)
 {
  dma_count = DMA_GetCurrDataCounter(spektrumUart->txDMAStream);
 }
 DMA_DeInit(spektrumUart->txDMAStream);
 DMA_StructInit(&DMA_InitStructure);
 DMA_InitStructure.DMA_Channel = spektrumUart->txDMAChannel;
 DMA_InitStructure.DMA_PeripheralBaseAddr = spektrumUart->txDMAPeripheralBaseAddr;
 DMA_InitStructure.DMA_Memory0BaseAddr = baseAddress;
 DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
 DMA_InitStructure.DMA_BufferSize = packetSize;
 DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
 DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
 DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
 DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
 DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
 DMA_InitStructure.DMA_Priority = DMA_Priority_High;
 DMA_Init(spektrumUart->txDMAStream, &DMA_InitStructure);
 DMA_Cmd(spektrumUart->txDMAStream, ENABLE);
}
uint16_t srxlCrc16(uint16_t crc, uint8_t data, uint16_t poly)
{
 crc = crc ^ data << 8;
 for (int i = 0; i < 8; i++)
 {
  if (crc & 0x8000)
   crc = crc << 1 ^ poly;
  else
   crc = crc << 1;
 }
 return crc;
}
#endif
ITStatus idle = 0;
ITStatus rxne = 0;
uint32_t uart5_dr = 0;
void DMA1_Stream7_IRQHandler() {
 if (DMA_GetITStatus(DMA1_Stream7, DMA_IT_TCIF7)) {
  DMA_ClearITPendingBit(DMA1_Stream7, DMA_IT_TCIF7);
  idle = USART_GetFlagStatus(spektrumUart->USARTx, USART_FLAG_IDLE);
  rxne = USART_GetITStatus(spektrumUart->USARTx, USART_FLAG_RXNE);
  uart5_dr = UART5->DR;
  idle = USART_GetFlagStatus(spektrumUart->USARTx, USART_IT_IDLE);
  rxne = USART_GetITStatus(spektrumUart->USARTx, USART_IT_RXNE);
  DMA_ITConfig(DMA1_Stream7, DMA_IT_TC, DISABLE);
  USART_DMACmd(spektrumUart->USARTx, USART_DMAReq_Rx, ENABLE);
  USART_ITConfig(spektrumUart->USARTx, USART_IT_IDLE, ENABLE);
 }
}
