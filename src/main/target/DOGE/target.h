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
       
#define TARGET_BOARD_IDENTIFIER "DOGE"
#define LED0 PA13
#define LED1 PA14
#define LED2 PA15
#define BEEPER PB2
#define BEEPER_INVERTED 
#define USABLE_TIMER_CHANNEL_COUNT 9
#define USE_EXTI 
#define MPU_INT_EXTI PC13
#define EXTI_CALLBACK_HANDLER_COUNT 2
#define USE_MPU_DATA_READY_SIGNAL 
#define ENSURE_MPU_DATA_READY_IS_LOW 
#define GYRO 
#define USE_GYRO_MPU9250 
#define GYRO_MPU9250_ALIGN CW270_DEG
#define ACC 
#define USE_ACC_MPU9250 
#define ACC_MPU9250_ALIGN CW270_DEG
#define MPU9250_CS_PIN PC14
#define MPU9250_SPI_INSTANCE SPI1
#define USE_FLASHFS 
#define USE_FLASH_M25P16 
#define USE_VCP 
#define USE_USART1 
#define USE_USART2 
#define USE_USART3 
#define SERIAL_PORT_COUNT 4
#ifndef UART1_GPIO
#define UART1_TX_PIN GPIO_Pin_6
#define UART1_RX_PIN GPIO_Pin_7
#define UART1_GPIO GPIOB
#define UART1_GPIO_AF GPIO_AF_7
#define UART1_TX_PINSOURCE GPIO_PinSource6
#define UART1_RX_PINSOURCE GPIO_PinSource7
#endif
#define UART2_TX_PIN GPIO_Pin_2
#define UART2_RX_PIN GPIO_Pin_3
#define UART2_GPIO GPIOA
#define UART2_GPIO_AF GPIO_AF_7
#define UART2_TX_PINSOURCE GPIO_PinSource2
#define UART2_RX_PINSOURCE GPIO_PinSource3
#ifndef UART3_GPIO
#define UART3_TX_PIN GPIO_Pin_10
#define UART3_RX_PIN GPIO_Pin_11
#define UART3_GPIO_AF GPIO_AF_7
#define UART3_GPIO GPIOB
#define UART3_TX_PINSOURCE GPIO_PinSource10
#define UART3_RX_PINSOURCE GPIO_PinSource11
#endif
#define ESC_1WIRE 
#define USE_SPI 
#define USE_SPI_DEVICE_1 
#define USE_SPI_DEVICE_2 
#define SPI1_SCK_PIN PB3
#define SPI1_MISO_PIN PB4
#define SPI1_MOSI_PIN PB5
#define SPI2_NSS_PIN PB12
#define SPI2_SCK_PIN PB13
#define SPI2_MISO_PIN PB14
#define SPI2_MOSI_PIN PB15
#define M25P16_CS_PIN PB15
#define M25P16_SPI_INSTANCE SPI2
#define USE_ADC 
#define BOARD_HAS_VOLTAGE_DIVIDER 
#define ADC_INSTANCE ADC2
#define ADC_DMA_CHANNEL DMA2_Channel1
#define ADC_AHB_PERIPHERAL RCC_AHBPeriph_DMA2
#define VBAT_ADC_PIN PA4
#define CURRENT_METER_ADC_PIN PA5
#define VBAT_ADC_CHANNEL ADC_Channel_1
#define CURRENT_METER_ADC_CHANNEL ADC_Channel_2
#define BLACKBOX 
#define SERIAL_RX 
#define TELEMETRY 
#define USE_SERVOS 
#define USE_CLI 
#define SPEKTRUM_BIND 
#define BIND_PORT GPIOB
#define BIND_PIN Pin_11
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF (BIT(0)|BIT(1)|BIT(3)|BIT(4))
#define USED_TIMERS (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(15))
