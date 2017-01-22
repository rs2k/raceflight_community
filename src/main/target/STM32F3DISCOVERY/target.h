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
       
#define TARGET_BOARD_IDENTIFIER "VRCR"
#define USBD_PRODUCT_STRING "STM32 F3 Discovery"
#define LED0 PE8
#define LED1 PE10
#define LED2 PE14
#define LED0_GPIO GPIOE
#define LED0_PIN Pin_8|Pin_12
#define LED0_PERIPHERAL RCC_AHBPeriph_GPIOE
#define LED0_INVERTED 
#define LED1_GPIO GPIOE
#define LED1_PIN Pin_10|Pin_14
#define LED1_PERIPHERAL RCC_AHBPeriph_GPIOE
#define LED1_INVERTED 
#define BEEPER PE9
#define BEEP_GPIO GPIOE
#define BEEP_PIN Pin_9|Pin_13
#define BEEP_PERIPHERAL RCC_AHBPeriph_GPIOE
#define BEEPER_INVERTED 
#define USE_SPI 
#define USE_SPI_DEVICE_1 
#define GYRO 
#define USE_GYRO_L3GD20 
#define L3GD20_SPI SPI1
#define L3GD20_CS_GPIO_CLK_PERIPHERAL RCC_AHBPeriph_GPIOE
#define L3GD20_CS_GPIO GPIOE
#define L3GD20_CS_PIN PE3
#define GYRO_L3GD20_ALIGN CW270_DEG
#define ACC 
#define USE_ACC_LSM303DLHC 
#define MAG 
#define USE_MAG_HMC5883 
#define USE_VCP 
#define USE_USART1 
#define USE_USART2 
#define SERIAL_PORT_COUNT 3
#define ESC_1WIRE 
#define USE_I2C 
#define I2C_DEVICE (I2CDEV_1)
#define USE_ADC 
#define ADC_INSTANCE ADC1
#define ADC_AHB_PERIPHERAL RCC_AHBPeriph_DMA1
#define ADC_DMA_CHANNEL DMA1_Channel1
#define VBAT_ADC_GPIO GPIOC
#define VBAT_ADC_GPIO_PIN GPIO_Pin_0
#define VBAT_ADC_CHANNEL ADC_Channel_6
#define CURRENT_METER_ADC_GPIO GPIOC
#define CURRENT_METER_ADC_GPIO_PIN GPIO_Pin_1
#define CURRENT_METER_ADC_CHANNEL ADC_Channel_7
#define RSSI_ADC_GPIO GPIOC
#define RSSI_ADC_GPIO_PIN GPIO_Pin_2
#define RSSI_ADC_CHANNEL ADC_Channel_8
#define EXTERNAL1_ADC_GPIO GPIOC
#define EXTERNAL1_ADC_GPIO_PIN GPIO_Pin_3
#define EXTERNAL1_ADC_CHANNEL ADC_Channel_9
#define BLACKBOX 
#define GPS 
#define GTUNE 
#define LED_STRIP 
#define LED_STRIP_TIMER TIM16
#define TELEMETRY 
#define SERIAL_RX 
#define USE_SERVOS 
#define USE_CLI 
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff
