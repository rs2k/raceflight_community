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

#define TARGET_BOARD_IDENTIFIER "OLI1" 




#ifdef OLIMEXINO_UNCUT_LED1_E_JUMPER
#define LED0_GPIO   GPIOA
#define LED0_PIN    Pin_5 
#define LED0_PERIPHERAL RCC_APB2Periph_GPIOA
#define LED0
#endif

#ifdef OLIMEXINO_UNCUT_LED2_E_JUMPER

#define LED1_GPIO   GPIOA
#define LED1_PIN    Pin_1 
#define LED1_PERIPHERAL RCC_APB2Periph_GPIOA
#define LED1
#endif

#define GYRO
#define USE_FAKE_GYRO



#define USE_GYRO_MPU6050



#define ACC
#define USE_FAKE_ACC




#define USE_ACC_MPU6050



#define BARO

#define USE_BARO_BMP085
#define USE_BARO_BMP280

#define MAG
#define USE_MAG_HMC5883

#define SONAR

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

#define CURRENT_METER_ADC_GPIO      GPIOB
#define CURRENT_METER_ADC_GPIO_PIN  GPIO_Pin_1
#define CURRENT_METER_ADC_CHANNEL   ADC_Channel_9

#define VBAT_ADC_GPIO               GPIOA
#define VBAT_ADC_GPIO_PIN           GPIO_Pin_4
#define VBAT_ADC_CHANNEL            ADC_Channel_4

#define RSSI_ADC_GPIO               GPIOA
#define RSSI_ADC_GPIO_PIN           GPIO_Pin_1
#define RSSI_ADC_CHANNEL            ADC_Channel_1

#define EXTERNAL1_ADC_GPIO          GPIOA
#define EXTERNAL1_ADC_GPIO_PIN      GPIO_Pin_5
#define EXTERNAL1_ADC_CHANNEL       ADC_Channel_5

#define GPS
#define LED_STRIP
#define LED_STRIP_TIMER TIM3

#define TELEMETRY
#define SERIAL_RX
#define BLACKBOX
#define USE_SERVOS
#define USE_CLI
