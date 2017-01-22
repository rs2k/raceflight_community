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
       
#define TARGET_BOARD_IDENTIFIER "CLBR"
#define BST_DEVICE_NAME "COLIBRI RACE"
#define BST_DEVICE_NAME_LENGTH 12
#define USBD_PRODUCT_STRING "TBS Colibri Race"
#define USE_EXTI 
#define MPU_INT_EXTI PA5
#define BEEPER PB13
#define LED0 PC15
#define LED1 PC14
#define LED2 PC13
#define BEEPER_INVERTED 
#define MPU6500_CS_PIN PA4
#define MPU6500_SPI_INSTANCE SPI1
#define USE_SPI 
#define USE_SPI_DEVICE_1 
#define SPI1_SCK_PIN PB3
#define SPI1_MISO_PIN PB4
#define SPI1_MOSI_PIN PB5
#define USABLE_TIMER_CHANNEL_COUNT 11
#define GYRO 
#define USE_GYRO_MPU6500 
#define USE_GYRO_SPI_MPU6500 
#define GYRO_MPU6500_ALIGN CW270_DEG
#define ACC 
#define USE_ACC_MPU6500 
#define USE_ACC_SPI_MPU6500 
#define ACC_MPU6500_ALIGN CW270_DEG
#define USE_MPU_DATA_READY_SIGNAL 
#define ENSURE_MPU_DATA_READY_IS_LOW 
#define USE_VCP 
#define USE_USART1 
#define USE_USART2 
#define USE_USART3 
#define SERIAL_PORT_COUNT 4
#define UART1_TX_PIN GPIO_Pin_4
#define UART1_RX_PIN GPIO_Pin_5
#define UART1_GPIO GPIOC
#define UART1_GPIO_AF GPIO_AF_7
#define UART1_TX_PINSOURCE GPIO_PinSource4
#define UART1_RX_PINSOURCE GPIO_PinSource5
#define UART2_TX_PIN GPIO_Pin_14
#define UART2_RX_PIN GPIO_Pin_15
#define UART2_GPIO GPIOA
#define UART2_GPIO_AF GPIO_AF_7
#define UART2_TX_PINSOURCE GPIO_PinSource14
#define UART2_RX_PINSOURCE GPIO_PinSource15
#define UART3_TX_PIN GPIO_Pin_10
#define UART3_RX_PIN GPIO_Pin_11
#define UART3_GPIO GPIOB
#define UART3_GPIO_AF GPIO_AF_7
#define UART3_TX_PINSOURCE GPIO_PinSource10
#define UART3_RX_PINSOURCE GPIO_PinSource11
#define ESC_1WIRE 
#define USE_I2C 
#define I2C_DEVICE (I2CDEV_2)
#define I2C2_SCL PA9
#define I2C2_SDA PA10
#define BLACKBOX 
#define TELEMETRY 
#define SERIAL_RX 
#define USE_SERVOS 
#define USE_CLI 
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTF 0xffff
