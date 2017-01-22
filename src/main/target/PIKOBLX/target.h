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
       
#define TARGET_BOARD_IDENTIFIER "PBLX"
#define USBD_PRODUCT_STRING "PIKOBLX"
#define CONFIG_SERIALRX_PROVIDER 2
#define CONFIG_FEATURE_RX_SERIAL 
#define CONFIG_MSP_PORT 2
#define CONFIG_RX_SERIAL_PORT 2
#define USE_CLI 
#define USE_EXTI 
#define LED0 PB9
#define LED1 PB5
#define BEEPER PA0
#define USABLE_TIMER_CHANNEL_COUNT 9
#define USE_EXTI 
#define MPU_INT_EXTI PA15
#define EXTI15_10_CALLBACK_HANDLER_COUNT 1
#define USE_MPU_DATA_READY_SIGNAL 
#define GYRO 
#define USE_GYRO_SPI_MPU6000 
#define GYRO_MPU6000_ALIGN CW180_DEG
#define ACC 
#define USE_ACC_SPI_MPU6000 
#define ACC_MPU6000_ALIGN CW180_DEG
#define MPU6000_CS_GPIO GPIOB
#define MPU6000_CS_PIN PB12
#define MPU6000_SPI_INSTANCE SPI2
#define USE_VCP 
#define USE_USART1 
#define USE_USART2 
#define USE_USART3 
#define SERIAL_PORT_COUNT 4
#define UART1_TX_PIN GPIO_Pin_6
#define UART1_RX_PIN GPIO_Pin_7
#define UART1_GPIO GPIOB
#define UART1_GPIO_AF GPIO_AF_7
#define UART1_TX_PINSOURCE GPIO_PinSource6
#define UART1_RX_PINSOURCE GPIO_PinSource7
#define UART2_TX_PIN GPIO_Pin_3
#define UART2_RX_PIN GPIO_Pin_4
#define UART2_GPIO GPIOB
#define UART2_GPIO_AF GPIO_AF_7
#define UART2_TX_PINSOURCE GPIO_PinSource3
#define UART2_RX_PINSOURCE GPIO_PinSource4
#define UART3_TX_PIN GPIO_Pin_10
#define UART3_RX_PIN GPIO_Pin_11
#define UART3_GPIO_AF GPIO_AF_7
#define UART3_GPIO GPIOB
#define UART3_TX_PINSOURCE GPIO_PinSource10
#define UART3_RX_PINSOURCE GPIO_PinSource11
#define ESC_1WIRE 
#define USE_I2C 
#define I2C_DEVICE (I2CDEV_2)
#define I2C2_SCL PA9
#define I2C2_SDA PA10
#define USE_SPI 
#define USE_SPI_DEVICE_2 
#define SPI2_NSS_PIN PB12
#define SPI2_SCK_PIN PB13
#define SPI2_MISO_PIN PB14
#define SPI2_MOSI_PIN PB15
#define M25P16_CS_PIN PB12
#define M25P16_SPI_INSTANCE SPI2
#define SENSORS_SET (SENSOR_ACC)
#define TELEMETRY 
#define BLACKBOX 
#define SERIAL_RX 
#define USE_SERVOS 
#define USE_ADC 
#define BOARD_HAS_VOLTAGE_DIVIDER 
#define ADC_INSTANCE ADC2
#define ADC_DMA_CHANNEL DMA2_Channel1
#define ADC_AHB_PERIPHERAL RCC_AHBPeriph_DMA2
#define VBAT_ADC_PIN PA5
#define VBAT_ADC_CHANNEL ADC_Channel_2
#define RSSI_ADC_PIN PB2
#define RSSI_ADC_CHANNEL ADC_Channel_12
#if 0
#define LED_STRIP_TIMER TIM17
#define USE_LED_STRIP_ON_DMA1_CHANNEL7 
#define WS2811_GPIO GPIOA
#define WS2811_GPIO_AHB_PERIPHERAL RCC_AHBPeriph_GPIOA
#define WS2811_GPIO_AF GPIO_AF_1
#define WS2811_PIN GPIO_Pin_7
#define WS2811_PIN_SOURCE GPIO_PinSource7
#define WS2811_TIMER TIM17
#define WS2811_TIMER_APB2_PERIPHERAL RCC_APB2Periph_TIM17
#define WS2811_DMA_CHANNEL DMA1_Channel7
#define WS2811_IRQ DMA1_Channel7_IRQn
#endif
#define SPEKTRUM_BIND 
#define BIND_PORT GPIOB
#define BIND_PIN Pin_11
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTF 0xffff
