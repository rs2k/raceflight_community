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
       
#define TARGET_BOARD_IDENTIFIER "SPKY"
#define USBD_PRODUCT_STRING "Sparky 1.x"
#ifdef OPBL
 #define USBD_SERIALNUMBER_STRING "0x800C000"
#endif
#define USE_EXTI 
#define LED0 PB4
#define LED1 PB5
#define BEEPER PA1
#define BEEPER_INVERTED 
#define USABLE_TIMER_CHANNEL_COUNT 11
#define EXTI15_10_CALLBACK_HANDLER_COUNT 1
#define USE_MPU_DATA_READY_SIGNAL 
#define MPU_INT_EXTI PA15
#define GYRO 
#define USE_GYRO_MPU6050 
#define GYRO_MPU6050_ALIGN CW270_DEG
#define ACC 
#define USE_ACC_MPU6050 
#define ACC_MPU6050_ALIGN CW270_DEG
#define BARO 
#define USE_BARO_MS5611 
#define USE_BARO_BMP280 
#define MAG 
#define USE_MAG_AK8975 
#define MAG_AK8975_ALIGN CW180_DEG_FLIP
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
#define UART2_TX_PIN GPIO_Pin_2
#define UART2_RX_PIN GPIO_Pin_3
#define UART2_GPIO GPIOA
#define UART2_GPIO_AF GPIO_AF_7
#define UART2_TX_PINSOURCE GPIO_PinSource2
#define UART2_RX_PINSOURCE GPIO_PinSource3
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
#define USE_ADC 
#define ADC_INSTANCE ADC2
#define ADC_DMA_CHANNEL DMA2_Channel1
#define ADC_AHB_PERIPHERAL RCC_AHBPeriph_DMA2
#define VBAT_ADC_PIN PA4
#define VBAT_ADC_CHANNEL ADC_Channel_1
#define CURRENT_METER_ADC_PIN PA7
#define CURRENT_METER_ADC_CHANNEL ADC_Channel_4
#define BLACKBOX 
#define GPS 
#define GTUNE 
#define DISPLAY 
#define SERIAL_RX 
#define TELEMETRY 
#define USE_SERVOS 
#define USE_CLI 
#define LED_STRIP 
#if 1
#define LED_STRIP_TIMER TIM16
#define USE_LED_STRIP_ON_DMA1_CHANNEL3 
#define WS2811_GPIO GPIOA
#define WS2811_GPIO_AHB_PERIPHERAL RCC_AHBPeriph_GPIOA
#define WS2811_GPIO_AF GPIO_AF_1
#define WS2811_PIN GPIO_Pin_6
#define WS2811_PIN_SOURCE GPIO_PinSource6
#define WS2811_TIMER TIM16
#define WS2811_TIMER_APB2_PERIPHERAL RCC_APB2Periph_TIM16
#define WS2811_DMA_CHANNEL DMA1_Channel3
#define WS2811_IRQ DMA1_Channel3_IRQn
#endif
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
#define BIND_PORT GPIOA
#define BIND_PIN Pin_3
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTF 0xffff
