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
       
#define TARGET_BOARD_IDENTIFIER "RMDO"
#define LED0 PB3
#define BEEPER PC15
#define BEEPER_INVERTED 
#define USABLE_TIMER_CHANNEL_COUNT 17
#define EXTI_CALLBACK_HANDLER_COUNT 2
#define FAKE_EXTI 
#define FE_TIM_IRQ TIM1_BRK_TIM15_IRQn
#define FE_TIM TIM15
#define USE_MAG_DATA_READY_SIGNAL 
#define ENSURE_MAG_DATA_READY_IS_HIGH 
#define GYRO 
#define USE_GYRO_MPU6050 
#define GYRO_MPU6050_ALIGN CW270_DEG
#define ACC 
#define USE_ACC_MPU6050 
#define ACC_MPU6050_ALIGN CW270_DEG
#define USE_FLASHFS 
#define USE_FLASH_M25P16 
#define USE_USART1 
#define USE_USART2 
#define USE_USART3 
#define SERIAL_PORT_COUNT 5
#ifndef UART1_GPIO
#define UART1_TX_PIN GPIO_Pin_9
#define UART1_RX_PIN GPIO_Pin_10
#define UART1_GPIO GPIOA
#define UART1_GPIO_AF GPIO_AF_7
#define UART1_TX_PINSOURCE GPIO_PinSource9
#define UART1_RX_PINSOURCE GPIO_PinSource10
#endif
#define UART2_TX_PIN GPIO_Pin_14
#define UART2_RX_PIN GPIO_Pin_15
#define UART2_GPIO GPIOA
#define UART2_GPIO_AF GPIO_AF_7
#define UART2_TX_PINSOURCE GPIO_PinSource14
#define UART2_RX_PINSOURCE GPIO_PinSource15
#ifndef UART3_GPIO
#define UART3_TX_PIN GPIO_Pin_10
#define UART3_RX_PIN GPIO_Pin_11
#define UART3_GPIO_AF GPIO_AF_7
#define UART3_GPIO GPIOB
#define UART3_TX_PINSOURCE GPIO_PinSource10
#define UART3_RX_PINSOURCE GPIO_PinSource11
#endif
#define ESC_1WIRE 
#define USE_I2C 
#define I2C_DEVICE (I2CDEV_1)
#define USE_SPI 
#define USE_SPI_DEVICE_2 
#define SPI2_NSS_PIN PB12
#define SPI2_SCK_PIN PB13
#define SPI2_MISO_PIN PB14
#define SPI2_MOSI_PIN PB15
#define M25P16_CS_PIN PB12
#define M25P16_SPI_INSTANCE SPI2
#define USE_ADC 
#define BOARD_HAS_VOLTAGE_DIVIDER 
#define ADC_INSTANCE ADC2
#define ADC_DMA_CHANNEL DMA2_Channel1
#define ADC_AHB_PERIPHERAL RCC_AHBPeriph_DMA2
#define VBAT_ADC_PIN PA4
#define VBAT_ADC_CHANNEL ADC_Channel_1
#define CURRENT_METER_ADC_PIN PA5
#define CURRENT_METER_ADC_CHANNEL ADC_Channel_2
#define RSSI_ADC_PIN PB2
#define RSSI_ADC_CHANNEL ADC_Channel_12
#define BLACKBOX 
#define TELEMETRY 
#define SERIAL_RX 
#define USE_SERVOS 
#define USE_CLI 
#define SPEKTRUM_BIND 
#define BIND_PORT GPIOB
#define BIND_PIN Pin_11
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTF 0xffff
