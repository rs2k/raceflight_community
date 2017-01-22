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
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <platform.h>
#include <build_config.h>
#include "bus_bst.h"
#ifdef USE_BST
#define BST_SHORT_TIMEOUT ((uint32_t)0x1000)
#define BST_LONG_TIMEOUT ((uint32_t)(10 * BST_SHORT_TIMEOUT))
#if !defined(BST1_SCL_GPIO)
#define BST1_SCL_GPIO GPIOB
#define BST1_SCL_GPIO_AF GPIO_AF_4
#define BST1_SCL_PIN GPIO_Pin_6
#define BST1_SCL_PIN_SOURCE GPIO_PinSource6
#define BST1_SCL_CLK_SOURCE RCC_AHBPeriph_GPIOB
#define BST1_SDA_GPIO GPIOB
#define BST1_SDA_GPIO_AF GPIO_AF_4
#define BST1_SDA_PIN GPIO_Pin_7
#define BST1_SDA_PIN_SOURCE GPIO_PinSource7
#define BST1_SDA_CLK_SOURCE RCC_AHBPeriph_GPIOB
#endif
#if !defined(BST2_SCL_GPIO)
#define BST2_SCL_GPIO GPIOF
#define BST2_SCL_GPIO_AF GPIO_AF_4
#define BST2_SCL_PIN GPIO_Pin_6
#define BST2_SCL_PIN_SOURCE GPIO_PinSource6
#define BST2_SCL_CLK_SOURCE RCC_AHBPeriph_GPIOF
#define BST2_SDA_GPIO GPIOA
#define BST2_SDA_GPIO_AF GPIO_AF_4
#define BST2_SDA_PIN GPIO_Pin_10
#define BST2_SDA_PIN_SOURCE GPIO_PinSource10
#define BST2_SDA_CLK_SOURCE RCC_AHBPeriph_GPIOA
#endif
static uint32_t bstTimeout;
static volatile uint16_t bst1ErrorCount = 0;
static volatile uint16_t bst2ErrorCount = 0;
static I2C_TypeDef *BSTx = NULL;
volatile uint8_t CRC8 = 0;
volatile bool coreProReady = false;
uint32_t bstTimeoutUserCallback(I2C_TypeDef *BSTx)
{
    if (BSTx == I2C1) {
        bst1ErrorCount++;
    } else {
        bst2ErrorCount++;
    }
    I2C_SoftwareResetCmd(BSTx);
    return false;
}
void bstInitPort(I2C_TypeDef *BSTx )
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef BST_InitStructure;
    if (BSTx == I2C1) {
        RCC_AHBPeriphClockCmd(BST1_SCL_CLK_SOURCE | BST1_SDA_CLK_SOURCE, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
        RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);
        GPIO_PinAFConfig(BST1_SCL_GPIO, BST1_SCL_PIN_SOURCE, BST1_SCL_GPIO_AF);
        GPIO_PinAFConfig(BST1_SDA_GPIO, BST1_SDA_PIN_SOURCE, BST1_SDA_GPIO_AF);
        GPIO_StructInit(&GPIO_InitStructure);
        I2C_StructInit(&BST_InitStructure);
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_Pin = BST1_SCL_PIN;
        GPIO_Init(BST1_SCL_GPIO, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = BST1_SDA_PIN;
        GPIO_Init(BST1_SDA_GPIO, &GPIO_InitStructure);
        I2C_StructInit(&BST_InitStructure);
        BST_InitStructure.I2C_Mode = I2C_Mode_I2C;
        BST_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
        BST_InitStructure.I2C_DigitalFilter = 0x00;
        BST_InitStructure.I2C_OwnAddress1 = RaceFlight_FC;
        BST_InitStructure.I2C_Ack = I2C_Ack_Enable;
        BST_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
        BST_InitStructure.I2C_Timing = 0x30E0257A;
        I2C_Init(I2C1, &BST_InitStructure);
        I2C_Cmd(I2C1, ENABLE);
    }
    if (BSTx == I2C2) {
        RCC_AHBPeriphClockCmd(BST2_SCL_CLK_SOURCE | BST2_SDA_CLK_SOURCE, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
        RCC_I2CCLKConfig(RCC_I2C2CLK_SYSCLK);
        GPIO_PinAFConfig(BST2_SCL_GPIO, BST2_SCL_PIN_SOURCE, BST2_SCL_GPIO_AF);
        GPIO_PinAFConfig(BST2_SDA_GPIO, BST2_SDA_PIN_SOURCE, BST2_SDA_GPIO_AF);
        GPIO_StructInit(&GPIO_InitStructure);
        I2C_StructInit(&BST_InitStructure);
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_Pin = BST2_SCL_PIN;
        GPIO_Init(BST2_SCL_GPIO, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = BST2_SDA_PIN;
        GPIO_Init(BST2_SDA_GPIO, &GPIO_InitStructure);
        I2C_StructInit(&BST_InitStructure);
        BST_InitStructure.I2C_Mode = I2C_Mode_I2C;
        BST_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
        BST_InitStructure.I2C_DigitalFilter = 0x00;
        BST_InitStructure.I2C_OwnAddress1 = RaceFlight_FC;
        BST_InitStructure.I2C_Ack = I2C_Ack_Enable;
        BST_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
        BST_InitStructure.I2C_Timing = 0x30E0257A;
        I2C_Init(I2C2, &BST_InitStructure);
        I2C_Cmd(I2C2, ENABLE);
    }
}
void bstInit(BSTDevice index)
{
    if (index == BSTDEV_1) {
        BSTx = I2C1;
    } else {
        BSTx = I2C2;
    }
    bstInitPort(BSTx);
}
uint16_t bstGetErrorCounter(void)
{
    if (BSTx == I2C1) {
        return bst1ErrorCount;
    }
    return bst2ErrorCount;
}
uint8_t dataBuffer[64] = {0};
uint8_t bufferPointer = 0;
uint8_t bstWriteDataLen = 0;
bool bstWriteBusy(void)
{
 if(bstWriteDataLen)
  return true;
 else
  return false;
}
bool bstMasterWrite(uint8_t* data)
{
 if(bstWriteDataLen==0) {
  CRC8 = 0;
  bufferPointer = 0;
  dataBuffer[0] = *data;
  dataBuffer[1] = *(data+1);
  bstWriteDataLen = dataBuffer[1] + 2;
  for(uint8_t i=2; i<bstWriteDataLen; i++) {
   if(i==(bstWriteDataLen-1)) {
    crc8Cal(0);
    dataBuffer[i] = CRC8;
   } else {
    dataBuffer[i] = *(data+i);
    crc8Cal((uint8_t)dataBuffer[i]);
   }
  }
  return true;
 }
 return false;
}
bool bstSlaveRead(uint8_t* buf) {
 if(I2C_GetAddressMatched(BSTx)==RaceFlight_FC && I2C_GetTransferDirection(BSTx) == I2C_Direction_Transmitter) {
  uint8_t len = 0;
  CRC8 = 0;
  I2C_ClearFlag(BSTx, I2C_FLAG_ADDR);
  bstTimeout = BST_LONG_TIMEOUT;
  while (I2C_GetFlagStatus(BSTx, I2C_ISR_RXNE) == RESET) {
   if ((bstTimeout--) == 0) {
    return bstTimeoutUserCallback(BSTx);
   }
  }
  len = I2C_ReceiveData(BSTx);
  *buf = len;
  buf++;
  while (len) {
   bstTimeout = BST_LONG_TIMEOUT;
   while (I2C_GetFlagStatus(BSTx, I2C_ISR_RXNE) == RESET) {
    if ((bstTimeout--) == 0) {
     return bstTimeoutUserCallback(BSTx);
    }
   }
   *buf = I2C_ReceiveData(BSTx);
   if(len == 1)
    crc8Cal(0);
   else
    crc8Cal((uint8_t)*buf);
   buf++;
   len--;
  }
  return true;
 }
 return false;
}
bool bstSlaveWrite(uint8_t* data) {
    bstTimeout = BST_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(BSTx, I2C_ISR_ADDR) == RESET) {
        if ((bstTimeout--) == 0) {
         return bstTimeoutUserCallback(BSTx);
        }
    }
 if(I2C_GetAddressMatched(BSTx)==RaceFlight_FC && I2C_GetTransferDirection(BSTx) == I2C_Direction_Receiver) {
  uint8_t len = 0;
  CRC8 = 0;
  I2C_ClearFlag(BSTx, I2C_FLAG_ADDR);
      bstTimeout = BST_LONG_TIMEOUT;
      while (I2C_GetFlagStatus(BSTx, I2C_ISR_TXIS) == RESET) {
          if ((bstTimeout--) == 0) {
              return bstTimeoutUserCallback(BSTx);
          }
      }
      len = *data;
      data++;
   I2C_SendData(BSTx, (uint8_t) len);
   while(len) {
       bstTimeout = BST_LONG_TIMEOUT;
       while (I2C_GetFlagStatus(BSTx, I2C_ISR_TXIS) == RESET) {
           if ((bstTimeout--) == 0) {
               return bstTimeoutUserCallback(BSTx);
           }
       }
       if(len == 1) {
        crc8Cal(0);
        I2C_SendData(BSTx, (uint8_t) CRC8);
       } else {
        crc8Cal((uint8_t)*data);
        I2C_SendData(BSTx, (uint8_t)*data);
       }
       data++;
    len--;
   }
   bstTimeout = BST_LONG_TIMEOUT;
   while (I2C_GetFlagStatus(BSTx, I2C_ISR_TXIS) == RESET) {
    if ((bstTimeout--) == 0) {
         return bstTimeoutUserCallback(BSTx);
    }
   }
   return true;
 }
 return false;
}
uint32_t bstMasterWriteTimeout = 0;
void bstMasterWriteLoop(void)
{
 if(bstWriteDataLen != 0) {
  if(bufferPointer == 0) {
   bool scl_set = false;
   if(BSTx == I2C1)
    scl_set = BST1_SCL_GPIO->IDR&BST1_SCL_PIN;
   else
    scl_set = BST2_SCL_GPIO->IDR&BST2_SCL_PIN;
   if(I2C_GetFlagStatus(BSTx, I2C_ISR_BUSY)==RESET && scl_set) {
    I2C_TransferHandling(BSTx, dataBuffer[bufferPointer], 1, I2C_Reload_Mode, I2C_Generate_Start_Write);
    bstMasterWriteTimeout = micros();
    bufferPointer++;
   }
  } else if(bufferPointer == 1) {
   if(I2C_GetFlagStatus(BSTx, I2C_ISR_TXIS)==SET) {
    I2C_SendData(BSTx, (uint8_t) dataBuffer[bufferPointer]);
    bstMasterWriteTimeout = micros();
   } else if(I2C_GetFlagStatus(BSTx, I2C_ISR_TCR)==SET) {
    I2C_TransferHandling(BSTx, dataBuffer[bufferPointer-1], dataBuffer[bufferPointer], I2C_AutoEnd_Mode, I2C_No_StartStop);
    bstMasterWriteTimeout = micros();
    bufferPointer++;
   }
  } else if(bufferPointer == bstWriteDataLen) {
   if(I2C_GetFlagStatus(BSTx, I2C_ISR_STOPF)==SET) {
    I2C_ClearFlag(BSTx, I2C_ICR_STOPCF);
    bstWriteDataLen = 0;
    bufferPointer = 0;
   }
  } else {
   if(I2C_GetFlagStatus(BSTx, I2C_ISR_TXIS)==SET) {
    I2C_SendData(BSTx, (uint8_t) dataBuffer[bufferPointer]);
    bstMasterWriteTimeout = micros();
    bufferPointer++;
   }
  }
  uint32_t currentTime = micros();
  if(currentTime>bstMasterWriteTimeout+5000) {
   I2C_SoftwareResetCmd(BSTx);
   bstWriteDataLen = 0;
   bufferPointer = 0;
  }
 }
}
void bstMasterReadLoop(void)
{
}
void crc8Cal(uint8_t data_in)
{
 uint8_t Polynom = BST_CRC_POLYNOM;
 bool MSB_Flag;
 for (uint8_t i = 0; i < 8; i++) {
  MSB_Flag = false;
  if (CRC8 & 0x80) {
   MSB_Flag = true;
  }
  CRC8 <<= 1;
  if (data_in & 0x80) {
   CRC8++;
  }
  data_in <<= 1;
  if (MSB_Flag == true) {
   CRC8 ^= Polynom;
  }
 }
}
#endif
