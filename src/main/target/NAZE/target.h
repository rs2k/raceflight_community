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
       
#define TARGET_BOARD_IDENTIFIER "AFNA"
#define USE_HARDWARE_REVISION_DETECTION 
#define BOARD_HAS_VOLTAGE_DIVIDER 
#define LED0 PB3
#define LED1 PB4
#define BEEPER PA12
#define BARO_XCLR_PIN PC13
#define BARO_EOC_PIN PC14
#define INVERTER PB2
#define INVERTER_USART USART2
#define USE_SPI 
#define USE_SPI_DEVICE_2 
#define NAZE_SPI_INSTANCE SPI2
#define NAZE_SPI_CS_PIN PB12
#define M25P16_CS_PIN NAZE_SPI_CS_PIN
#define M25P16_SPI_INSTANCE NAZE_SPI_INSTANCE
#define MPU6500_CS_PIN NAZE_SPI_CS_PIN
#define MPU6500_SPI_INSTANCE NAZE_SPI_INSTANCE
#define USE_FLASHFS 
#define USE_FLASH_M25P16 
#define EXTI_CALLBACK_HANDLER_COUNT 3
#define USE_EXTI 
#define USE_MPU_DATA_READY_SIGNAL 
#define USE_MAG_DATA_READY_SIGNAL 
#define GYRO 
#define USE_GYRO_MPU3050 
#define USE_GYRO_MPU6050 
#define USE_GYRO_MPU6500 
#define USE_GYRO_SPI_MPU6500 
#define GYRO_MPU3050_ALIGN CW0_DEG
#define GYRO_MPU6050_ALIGN CW0_DEG
#define GYRO_MPU6500_ALIGN CW0_DEG
#define ACC 
#define USE_ACC_ADXL345 
#define USE_ACC_BMA280 
#define USE_ACC_MMA8452 
#define USE_ACC_MPU6050 
#define USE_ACC_MPU6500 
#define USE_ACC_SPI_MPU6500 
#define ACC_ADXL345_ALIGN CW270_DEG
#define ACC_MPU6050_ALIGN CW0_DEG
#define ACC_MMA8452_ALIGN CW90_DEG
#define ACC_BMA280_ALIGN CW0_DEG
#define ACC_MPU6500_ALIGN CW0_DEG
#define USE_USART1 
#define USE_USART2 
#define USE_USART3 
#define USE_SOFTSERIAL1 
#define USE_SOFTSERIAL2 
#define SERIAL_PORT_COUNT 5
#define SOFTSERIAL_1_TIMER TIM3
#define SOFTSERIAL_1_TIMER_RX_HARDWARE 4
#define SOFTSERIAL_1_TIMER_TX_HARDWARE 5
#define SOFTSERIAL_2_TIMER TIM3
#define SOFTSERIAL_2_TIMER_RX_HARDWARE 6
#define SOFTSERIAL_2_TIMER_TX_HARDWARE 7
#define USART3_RX_PIN Pin_11
#define USART3_TX_PIN Pin_10
#define USART3_GPIO GPIOB
#define USART3_APB1_PERIPHERALS RCC_APB1Periph_USART3
#define USART3_APB2_PERIPHERALS RCC_APB2Periph_GPIOB
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
#undef BLACKBOX
#undef TELEMETRY
#define SERIAL_RX 
#define USE_SERVOS 
#undef GTUNE
#define USE_CLI 
#define SPEKTRUM_BIND 
#define BIND_PORT GPIOA
#define BIND_PIN Pin_3
#ifdef ALIENWII32
#undef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "AWF1"
#undef BOARD_HAS_VOLTAGE_DIVIDER
#define HARDWARE_BIND_PLUG 
#define BINDPLUG_PORT GPIOB
#define BINDPLUG_PIN Pin_5
#endif
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC (BIT(13)|BIT(14)|BIT(15))
