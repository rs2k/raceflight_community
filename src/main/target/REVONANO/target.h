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
#pragma once 
       
#define TARGET_BOARD_IDENTIFIER "REVN"
#define CONFIG_SERIALRX_PROVIDER 2
#define CONFIG_BLACKBOX_DEVICE 1
#define CONFIG_FEATURE_RX_SERIAL 
#define CONFIG_FEATURE_ONESHOT125 
#define CONFIG_MSP_PORT 1
#define CONFIG_RX_SERIAL_PORT 2
#ifdef USE_BOOTLOADER
#define CONFIG_FLASH_SECTOR FLASH_Sector_4
#define CONFIG_START_FLASH_ADDRESS (0x08010000)
#define CONFIG_START_BACK_ADDRESS (0x08010000)
#define ADDRESS_BL_START (0x08000000)
#define ADDRESS_CONFIG_START (0x08010000)
#define ADDRESS_FLASH_START (0x08020000)
#define ADDRESS_FLASH_END (0x0807FFF0)
#else
#define CONFIG_FLASH_SECTOR FLASH_Sector_6
#define CONFIG_START_FLASH_ADDRESS (0x08040000)
#define CONFIG_START_BACK_ADDRESS (0x08040000)
#define ADDRESS_BL_START (0x08000000)
#define ADDRESS_CONFIG_START (0x08040000)
#define ADDRESS_FLASH_START (0x08000000)
#define ADDRESS_FLASH_END (0x0807FFF0)
#endif
#define USBD_PRODUCT_STRING "Revo Nano"
#define USBD_SERIALNUMBER_STRING "0x8020000"
#define ESC_HEX 
#define LED0 PC14
#define LED1 PC13
#define BEEPER PC13
#define INVERTER PC15
#define INVERTER_USART USART2
#define MPU9250_CS_PIN PB12
#define MPU9250_SPI_INSTANCE SPI2
#define SLOW_SPI_DOWN 
#define ACC 
#define USE_ACC_MPU9250 
#define USE_ACC_SPI_MPU9250 
#define ACC_MPU9250_ALIGN CW270_DEG
#define GYRO 
#define USE_GYRO_MPU9250 
#define USE_GYRO_SPI_MPU9250 
#define GYRO_MPU9250_ALIGN CW270_DEG
#define USE_MPU_DATA_READY_SIGNAL 
#define ENSURE_MPU_DATA_READY_IS_LOW 
#define EXTI_CALLBACK_HANDLER_COUNT 1
#define MPU_INT_EXTI PA15
#define USE_EXTI 
#define USABLE_TIMER_CHANNEL_COUNT 12
#define USE_VCP 
#define VBUS_SENSING_PIN PA9
#define USE_USART1 
#define USART1_RX_PIN PB7
#define USART1_TX_PIN PB6
#define USE_USART2 
#define USART2_RX_PIN PA3
#define USART2_TX_PIN PA2
#define SERIALRX_DMA 
#ifdef SERIALRX_DMA
 #define USE_USART1_RX_DMA
 #define USE_USART2_RX_DMA
#endif
#define SERIAL_PORT_COUNT 3
#define ESC_1WIRE 
#define USE_SPI 
#define USE_SPI_DEVICE_2 
#define USE_I2C 
#define I2C_DEVICE (I2CDEV_3)
#define USE_ADC 
#define CURRENT_METER_ADC_PIN PA7
#define CURRENT_METER_ADC_CHANNEL ADC_Channel_7
#define VBAT_ADC_PIN PA6
#define VBAT_ADC_CHANNEL ADC_Channel_6
#define RSSI_ADC_PIN PA5
#define RSSI_ADC_CHANNEL ADC_Channel_5
#define BLACKBOX 
#define TELEMETRY 
#define SERIAL_RX 
#define USE_SERVOS 
#define USE_CLI 
#undef USE_FLASH_TOOLS
#undef USE_FLASHFS
#undef USE_FLASH_M25P16
#define USE_QUATERNION 
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
