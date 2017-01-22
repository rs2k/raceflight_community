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
       
#define TARGET_BOARD_IDENTIFIER "AWF3"
#define USBD_PRODUCT_STRING "AlienWii32 F3"
#define LED0 PB4
#define LED1 PB5
#define BEEPER PA5
#define USABLE_TIMER_CHANNEL_COUNT 11
#define GYRO 
#define USE_GYRO_MPU6050 
#define GYRO_MPU6050_ALIGN CW270_DEG
#define ACC 
#define USE_ACC_MPU6050 
#define ACC_MPU6050_ALIGN CW270_DEG
#define MAG_AK8975_ALIGN CW0_DEG_FLIP
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
#define ESC_SERIAL 
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
#define SERIAL_RX 
#define GTUNE 
#define USE_SERVOS 
#define USE_CLI 
#define SPEKTRUM_BIND 
#define BIND_PORT GPIOA
#define BIND_PIN Pin_3
#define ALIENWII32 
#define HARDWARE_BIND_PLUG 
#define BINDPLUG_PORT GPIOB
#define BINDPLUG_PIN Pin_12
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD (BIT(2))
#define TARGET_IO_PORTF (BIT(0)|BIT(1)|BIT(4))
