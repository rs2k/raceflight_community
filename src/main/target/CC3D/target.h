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
       
#define TARGET_BOARD_IDENTIFIER "CC3D"
#define USBD_PRODUCT_STRING "CC3D"
#ifdef OPBL
 #define USBD_SERIALNUMBER_STRING "0x8003000"
#endif
#define LED0 PB3
#define INVERTER PB2
#define INVERTER_USART USART1
#define USE_EXTI 
#define MPU_INT_EXTI PA3
#define BEEPER PA15
#define MPU6000_CS_PIN PA4
#define MPU6000_SPI_INSTANCE SPI1
#define M25P16_CS_PIN PB12
#define M25P16_SPI_INSTANCE SPI2
#undef USE_FLASHFS
#define USABLE_TIMER_CHANNEL_COUNT 12
#define DEBUG_MPU_DATA_READY_INTERRUPT 
#define USE_MPU_DATA_READY_SIGNAL 
#define GYRO 
#define USE_GYRO_SPI_MPU6000 
#define GYRO_MPU6000_ALIGN CW270_DEG
#define ACC 
#define USE_ACC_SPI_MPU6000 
#define ACC_MPU6000_ALIGN CW270_DEG
#define USE_MPU_DATA_READY_SIGNAL 
#define USE_VCP 
#define USE_USART1 
#define USE_USART3 
#undef USE_SOFTSERIAL1
#define SERIAL_PORT_COUNT 3
#define SOFTSERIAL_1_TIMER TIM3
#define SOFTSERIAL_1_TIMER_TX_HARDWARE 1
#define SOFTSERIAL_1_TIMER_RX_HARDWARE 2
#define USART3_RX_PIN Pin_11
#define USART3_TX_PIN Pin_10
#define USART3_GPIO GPIOB
#define USART3_APB1_PERIPHERALS RCC_APB1Periph_USART3
#define USART3_APB2_PERIPHERALS RCC_APB2Periph_GPIOB
#define USE_SPI 
#define USE_SPI_DEVICE_1 
#define USE_SPI_DEVICE_2 
#define ESC_1WIRE 
#undef USE_I2C
#define USE_ADC 
#define CURRENT_METER_ADC_PIN PB1
#define VBAT_ADC_PIN PA0
#define RSSI_ADC_PIN PA1
#define CURRENT_METER_ADC_GPIO GPIOB
#define CURRENT_METER_ADC_GPIO_PIN GPIO_Pin_1
#define CURRENT_METER_ADC_CHANNEL ADC_Channel_9
#define VBAT_ADC_GPIO GPIOA
#define VBAT_ADC_GPIO_PIN GPIO_Pin_0
#define VBAT_ADC_CHANNEL ADC_Channel_0
#define RSSI_ADC_GPIO GPIOA
#define RSSI_ADC_GPIO_PIN GPIO_Pin_1
#define RSSI_ADC_CHANNEL ADC_Channel_1
#define TELEMETRY 
#define SERIAL_RX 
#undef SONAR
#define USE_SERVOS 
#define USE_CLI 
#undef BARO
#undef GPS
#define BLACKBOX 
#undef LED_STRIP
#undef USE_ADC
#undef DISPLAY
#undef SONAR
#if defined(OPBL) && defined(USE_SERIAL_1WIRE)
#undef BARO
#undef GPS
#undef LED_STRIP
#undef USE_ADC
#endif
#define SKIP_CLI_COMMAND_HELP 
#define SPEKTRUM_BIND 
#define BIND_PORT GPIOB
#define BIND_PIN Pin_11
#undef USE_QUATERNION
#define USE_EXTI 
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC (BIT(14))
