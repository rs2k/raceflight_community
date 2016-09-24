/*
 * This file is part of Raceflight.
 *
 * Raceflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Raceflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Raceflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#define TARGET_BOARD_IDENTIFIER "PXR4"

#define CONFIG_START_FLASH_ADDRESS (0x08060000)
#define CONFIG_START_BACK_ADDRESS (0x08060000)
#define CONFIG_SERIALRX_PROVIDER SERIALRX_SPEKTRUM2048
#define CONFIG_FEATURE_RX_SERIAL
#define CONFIG_MSP_PORT 1
#define CONFIG_RX_SERIAL_PORT 4
#define USE_EXTI

#define USBD_PRODUCT_STRING "PixRacer F4"

#define LED0                    PB3 
#define LED1                    PB1 
#define LED2                    PB11 
#define BEEPER                  PA15
#define BEEPER_INVERTED
#define INVERTER                PC13
#define INVERTER_USART          USART1




#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define GYRO
#define USE_GYRO_MPU9250
#define USE_GYRO_SPI_MPU9250
#define GYRO_MPU9250_ALIGN      CW270_DEG_FLIP
#define MPU9250_CS_PIN          PC2
#define MPU9250_SPI_INSTANCE    SPI1

#define ACC
#define USE_ACC_MPU9250
#define USE_ACC_SPI_MPU9250
#define ACC_MPU9250_ALIGN       CW270_DEG_FLIP

#define MPU_INT_EXTI PA6


#define USABLE_TIMER_CHANNEL_COUNT 7


#define ESC_1WIRE


#define USE_VCP
#define VBUS_SENSING_PIN PA9
#define VBUS_SENSING_ENABLED

#define USE_UART1
#define UART1_RX_PIN            PB7
#define UART1_TX_PIN            PB6
#define USART1_AHB1_PERIPHERALS RCC_AHB1Periph_DMA2

#define USE_UART3
#define UART3_RX_PIN            PD9
#define UART3_TX_PIN            PD8
#define USART3_AHB1_PERIPHERALS RCC_AHB1Periph_DMA1


#define SERIALRX_DMA			

#ifdef SERIALRX_DMA
	#define USE_USART1_RX_DMA		
	#define USE_USART3_RX_DMA		
#endif

#define SERIAL_PORT_COUNT       3 


#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PC2
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7


#define BOARD_HAS_VOLTAGE_DIVIDER

#define USE_ADC
#define VBAT_ADC_PIN                PA2
#define VBAT_ADC_CHANNEL		    ADC_Channel_1 
#define CURRENT_METER_ADC_PIN       PA3
#define CURRENT_METER_ADC_CHANNEL   ADC_Channel_2 
#define RSSI_ADC_PIN                PC1
#define RSSI_ADC_CHANNEL		    ADC_Channel_3 


#define BLACKBOX
#define TELEMETRY
#define SERIAL_RX
#define USE_SERVOS
#define USE_CLI

#define USE_QUATERNION

#define TARGET_IO_PORTA 	0xffff
#define TARGET_IO_PORTB 	0xffff
#define TARGET_IO_PORTC 	0xffff
#define TARGET_IO_PORTD     0xffff
#define TARGET_IO_PORTE     0xffff
#define TARGET_IO_PORTF     0xffff
#define TARGET_IO_PORTG     0xffff
