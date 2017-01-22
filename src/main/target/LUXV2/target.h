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
       
#define TARGET_BOARD_IDENTIFIER "LUXV2"
#define LED0 PC15
#define LED1 PC14
#define BEEPER PB9
#define BEEPER_INVERTED 
#define MPU6000_CS_PIN PA4
#define MPU6000_SPI_INSTANCE SPI1
#define USE_EXTI 
#define MPU_INT_EXTI PA5
#define USE_SPI 
#define USE_SPI_DEVICE_1 
#define SPI1_SCK_PIN PB3
#define SPI1_MISO_PIN PB4
#define SPI1_MOSI_PIN PB5
#define USABLE_TIMER_CHANNEL_COUNT 11
#define ACC 
#define USE_ACC_MPU6000 
#define USE_ACC_SPI_MPU6000 
#define ACC_MPU6000_ALIGN CW270_DEG
#define GYRO 
#define SLOW_SPI_DOWN 
#define USE_GYRO_MPU6000 
#define USE_GYRO_SPI_MPU6000 
#define GYRO_MPU6000_ALIGN CW270_DEG
#define MPU6000_SPI_INSTANCE SPI1
#define USE_MPU_DATA_READY_SIGNAL 
#define ENSURE_MPU_DATA_READY_IS_LOW 
#define USE_VCP 
#define USE_USART1 
#define UART1_TX_PIN GPIO_Pin_4
#define UART1_RX_PIN GPIO_Pin_5
#define UART1_GPIO GPIOC
#define UART1_GPIO_AF GPIO_AF_7
#define UART1_TX_PINSOURCE GPIO_PinSource4
#define UART1_RX_PINSOURCE GPIO_PinSource5
#define USE_USART2 
#define UART2_TX_PIN GPIO_Pin_14
#define UART2_RX_PIN GPIO_Pin_15
#define UART2_GPIO GPIOA
#define UART2_GPIO_AF GPIO_AF_7
#define UART2_TX_PINSOURCE GPIO_PinSource14
#define UART2_RX_PINSOURCE GPIO_PinSource15
#define USE_USART3 
#define UART3_TX_PIN GPIO_Pin_10
#define UART3_RX_PIN GPIO_Pin_11
#define UART3_GPIO GPIOB
#define UART3_GPIO_AF GPIO_AF_7
#define UART3_TX_PINSOURCE GPIO_PinSource10
#define UART3_RX_PINSOURCE GPIO_PinSource11
#define SERIAL_PORT_COUNT 4
#define ESC_1WIRE 
#define USE_ADC 
#define BOARD_HAS_VOLTAGE_DIVIDER 
#define ADC_INSTANCE ADC1
#define ADC_AHB_PERIPHERAL RCC_AHBPeriph_DMA1
#define ADC_DMA_CHANNEL DMA1_Channel1
#define VBAT_ADC_PIN PC0
#define VBAT_ADC_CHANNEL ADC_Channel_6
#define CURRENT_METER_ADC_PIN PC1
#define CURRENT_METER_ADC_CHANNEL ADC_Channel_7
#define RSSI_ADC_PIN PC2
#define RSSI_ADC_CHANNEL ADC_Channel_8
#define EXTERNAL1_ADC_PIN PC3
#define EXTERNAL1_ADC_CHANNEL ADC_Channel_9
#define WS2812_LED 
#define WS2812_LED_TIMER_CH1 
#define WS2812_LED_GPIO GPIOA
#define WS2812_LED_PIN GPIO_Pin_6
#define WS2812_LED_PINSOURCE GPIO_PinSource6
#define WS2812_LED_PERIPH RCC_AHBPeriph_GPIOA
#define WS2812_LED_TIM TIM16
#define WS2812_LED_TIM_AF GPIO_AF_1
#define WS2812_LED_TIM_IRQ TIM16_IRQn
#define WS2812_LED_TIM_PERIPH RCC_APB2Periph_TIM16
#define WS2812_LED_DMA_CH DMA1_Channel3
#define WS2812_LED_DMA_IRQ DMA1_Channel3_IRQn
#define WS2812_LED_DMA_FLAG DMA1_FLAG_TC3
#define WS2812_LED_DMA_IRQ_HANDLER DMA1_Channel3_IRQHandler
#define WS2812_LED_DMA_PERIPH RCC_AHBPeriph_DMA1
#define BLACKBOX 
#define TELEMETRY 
#define SERIAL_RX 
#define USE_SERVOS 
#define USE_CLI 
#define SPEKTRUM_BIND 
#define BIND_PORT GPIOC
#define BIND_PIN Pin_5
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTF 0xffff
