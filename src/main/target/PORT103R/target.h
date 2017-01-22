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
       
#define TARGET_BOARD_IDENTIFIER "103R"
#define LED0 PB3
#define LED1 PB4
#define LED2 PB5
#define BEEPER PA12
#define LED0_GPIO GPIOB
#define LED0_PIN Pin_3
#define LED0_PERIPHERAL RCC_APB2Periph_GPIOB
#define LED1_GPIO GPIOB
#define LED1_PIN Pin_4
#define LED1_PERIPHERAL RCC_APB2Periph_GPIOB
#define LED2_GPIO GPIOD
#define LED2_PIN Pin_2
#define LED2_PERIPHERAL RCC_APB2Periph_GPIOD
#define BEEP_GPIO GPIOA
#define BEEP_PIN Pin_12
#define BEEP_PERIPHERAL RCC_APB2Periph_GPIOA
#define BARO_XCLR_GPIO GPIOC
#define BARO_XCLR_PIN PC13
#define BARO_EOC_GPIO GPIOC
#define BARO_EOC_PIN PC14
#define BARO_APB2_PERIPHERALS RCC_APB2Periph_GPIOC
#define USE_EXTI 
#define INVERTER PB2
#define INVERTER_PIN Pin_2
#define INVERTER_GPIO GPIOB
#define INVERTER_PERIPHERAL RCC_APB2Periph_GPIOB
#define INVERTER_USART USART2
#define USE_SPI 
#define USE_SPI_DEVICE_2 
#define PORT103R_SPI_INSTANCE SPI2
#define PORT103R_SPI_CS_GPIO GPIOB
#define PORT103R_SPI_CS_PIN PB12
#define M25P16_CS_GPIO PORT103R_SPI_CS_GPIO
#define M25P16_CS_PIN PORT103R_SPI_CS_PIN
#define M25P16_SPI_INSTANCE PORT103R_SPI_INSTANCE
#define MPU6000_CS_GPIO PORT103R_SPI_CS_GPIO
#define MPU6000_CS_PIN PORT103R_SPI_CS_PIN
#define MPU6000_SPI_INSTANCE PORT103R_SPI_INSTANCE
#define MPU6500_CS_GPIO PORT103R_SPI_CS_GPIO
#define MPU6500_CS_PIN PORT103R_SPI_CS_PIN
#define MPU6500_SPI_INSTANCE PORT103R_SPI_INSTANCE
#define MPU6500_CS_GPIO_CLK_PERIPHERAL RCC_APB2Periph_GPIOB
#define GYRO 
#define USE_FAKE_GYRO 
#define USE_GYRO_MPU6050 
#define USE_GYRO_SPI_MPU6000 
#define USE_GYRO_SPI_MPU6500 
#define ACC 
#define USE_FAKE_ACC 
#define USE_ACC_MPU6050 
#define USE_ACC_SPI_MPU6000 
#define USE_ACC_SPI_MPU6500 
#define BARO 
#define USE_BARO_MS5611 
#define USE_BARO_BMP085 
#define USE_BARO_BMP280 
#define MAG 
#define USE_MAG_HMC5883 
#define USE_MAG_AK8975 
#define USE_FLASHFS 
#define USE_FLASHTOOLS 
#define USE_FLASH_M25P16 
#define SONAR 
#define DISPLAY 
#define USE_USART1 
#define USE_USART2 
#define USE_SOFTSERIAL1 
#define USE_SOFTSERIAL2 
#define SERIAL_PORT_COUNT 4
#define SOFTSERIAL_1_TIMER TIM3
#define SOFTSERIAL_1_TIMER_RX_HARDWARE 4
#define SOFTSERIAL_1_TIMER_TX_HARDWARE 5
#define SOFTSERIAL_2_TIMER TIM3
#define SOFTSERIAL_2_TIMER_RX_HARDWARE 6
#define SOFTSERIAL_2_TIMER_TX_HARDWARE 7
#define ESC_1WIRE 
#define USE_I2C 
#define I2C_DEVICE (I2CDEV_2)
#define USE_ADC 
#define CURRENT_METER_ADC_GPIO GPIOB
#define CURRENT_METER_ADC_GPIO_PIN GPIO_Pin_1
#define CURRENT_METER_ADC_CHANNEL ADC_Channel_9
#define VBAT_ADC_GPIO GPIOA
#define VBAT_ADC_GPIO_PIN GPIO_Pin_4
#define VBAT_ADC_CHANNEL ADC_Channel_4
#define RSSI_ADC_GPIO GPIOA
#define RSSI_ADC_GPIO_PIN GPIO_Pin_1
#define RSSI_ADC_CHANNEL ADC_Channel_1
#define EXTERNAL1_ADC_GPIO GPIOA
#define EXTERNAL1_ADC_GPIO_PIN GPIO_Pin_5
#define EXTERNAL1_ADC_CHANNEL ADC_Channel_5
#define LED_STRIP 
#define LED_STRIP_TIMER TIM3
#define BLACKBOX 
#define GPS 
#define GTUNE 
#define SERIAL_RX 
#define TELEMETRY 
#define USE_SERVOS 
#define USE_CLI 
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
