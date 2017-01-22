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
       
#define TARGET_BOARD_IDENTIFIER "VIC1"
#define SPMFC_REVB 
#define CONFIG_SERIALRX_PROVIDER SERIALRX_SPEKTRUM2048
#define CONFIG_FEATURE_RX_SERIAL 
#ifdef SPMFC_REVB
#define CONFIG_BLACKBOX_DEVICE 1
#define CONFIG_MSP_PORT 1
#define CONFIG_RX_SERIAL_PORT 4
#else
#define CONFIG_MSP_PORT 1
#define CONFIG_RX_SERIAL_PORT 5
#endif
#define SPEKTRUM_PROPER_SCALING 
#define USE_EXTI 
#define DEFAULT_MIXER MIXER_QUADXL
#define USBD_PRODUCT_STRING "SPMFCF400"
#define USBD_SERIALNUMBER_STRING "0x8020000"
#ifdef USE_BOOTLOADER
#define CONFIG_FLASH_SECTOR FLASH_Sector_4
#define CONFIG_START_FLASH_ADDRESS (0x08010000)
#define CONFIG_START_BACK_ADDRESS (0x08010000)
#define ADDRESS_BL_START (0x08000000)
#define ADDRESS_CONFIG_START (0x08010000)
#define ADDRESS_FLASH_START (0x08020000)
#define ADDRESS_FLASH_END (0x0807FFF0)
#else
#define CONFIG_FLASH_SECTOR FLASH_Sector_6
#define CONFIG_START_FLASH_ADDRESS (0x08040000)
#define CONFIG_START_BACK_ADDRESS (0x08040000)
#define ADDRESS_BL_START (0x08000000)
#define ADDRESS_CONFIG_START (0x08040000)
#define ADDRESS_FLASH_START (0x08000000)
#define ADDRESS_FLASH_END (0x0807FFF0)
#endif
#define LED0 PA15
#define LED1 PC8
#define BEEPER PC2
#define BEEPER_INVERTED 
#define MPU9250_CS_PIN PB12
#define MPU9250_SPI_INSTANCE SPI2
#define ACC 
#define USE_ACC_MPU9250 
#define USE_ACC_SPI_MPU9250 
#define ACC_MPU9250_ALIGN CW270_DEG
#define GYRO 
#define USE_GYRO_MPU9250 
#define USE_GYRO_SPI_MPU9250 
#define GYRO_MPU9250_ALIGN CW270_DEG
#ifdef SPMFC_REVB
#endif
#define USABLE_TIMER_CHANNEL_COUNT 5
#define USE_MPU_DATA_READY_SIGNAL 
#define ENSURE_MPU_DATA_READY_IS_LOW 
#define MPU_INT_EXTI PC13
#define USE_VCP 
#ifdef SPMFC_REVB
#endif
#ifndef SPMFC_REVB
#define USE_USART1 
#define USART1_RX_PIN PA10
#define USART1_TX_PIN PA9
#define USART1_AHB1_PERIPHERALS RCC_AHB1Periph_DMA2
#endif
#define USE_USART2 
#define USART2_RX_PIN PA3
#define USART2_TX_PIN PA2
#define USART2_AHB1_PERIPHERALS RCC_AHB1Periph_DMA1
#define USE_USART3 
#define USART3_RX_PIN PC11
#define USART3_TX_PIN PC10
#define USART3_AHB1_PERIPHERALS RCC_AHB1Periph_DMA1
#define USE_USART4 
#define USART4_RX_PIN PA1
#define USART4_TX_PIN PA0
#define USART4_AHB1_PERIPHERALS RCC_AHB1Periph_DMA1
#define USE_USART5 
#define USART5_RX_PIN PC12
#define USART5_TX_PIN PC12
#define USART5_AHB1_PERIPHERALS RCC_AHB1Periph_DMA1
#ifndef SPMFC_REVB
#define SERIAL_PORT_COUNT 6
#else
#define SERIAL_PORT_COUNT 5
#endif
#define SERIALRX_DMA 
#ifdef SERIALRX_DMA
 #define USE_USART1_RX_DMA
 #define USE_USART3_RX_DMA
 #define USE_USART5_RX_DMA
#endif
#define ESC_1WIRE 
#define ESC_HEX 
#define USE_SPI 
#define USE_SPI_DEVICE_1 
#define SPI1_NSS_PIN PA4
#define SPI1_SCK_PIN PA5
#define SPI1_MISO_PIN PA6
#define SPI1_MOSI_PIN PA7
#define USE_SPI_DEVICE_2 
#define SPI2_NSS_PIN PB12
#define SPI2_SCK_PIN PB13
#define SPI2_MISO_PIN PB14
#define SPI2_MOSI_PIN PB15
#define USE_I2C 
#define I2C_DEVICE (I2CDEV_1)
#define I2C1_SCL PB8
#define I2C1_SDA PB9
#define USE_ADC 
#define VBAT_ADC_PIN PC0
#define VBAT_ADC_CHANNEL ADC_Channel_10
#define CURRENT_METER_ADC_PIN PC3
#define CURRENT_METER_ADC_CHANNEL ADC_Channel_13
#define WS2812_LED 
#define WS2812_LED_TIMER_CH1 
#define WS2812_LED_GPIO GPIOC
#define WS2812_LED_PIN GPIO_Pin_6
#define WS2812_LED_PINSOURCE GPIO_PinSource6
#define WS2812_LED_PERIPH RCC_AHB1Periph_GPIOC
#define WS2812_LED_TIM TIM8
#define WS2812_LED_TIM_AF GPIO_AF_TIM8
#define WS2812_LED_TIM_PERIPH RCC_APB2Periph_TIM8
#define WS2812_LED_DMA_CH DMA_Channel_7
#define WS2812_LED_DMA_ST DMA2_Stream2
#define WS2812_LED_DMA_IRQ DMA2_Stream2_IRQn
#define WS2812_LED_DMA_FLAG DMA_FLAG_TCIF2
#define WS2812_LED_DMA_IRQ_HANDLER DMA2_Stream2_IRQHandler
#define WS2812_LED_DMA_PERIPH RCC_AHB1Periph_DMA2
#define BLACKBOX 
#define TELEMETRY 
#define SERIAL_RX 
#define USE_SERVOS 
#define USE_CLI 
#define SPEKTRUM_TELEM 
#define SPEKTRUM_BIND 
#define BIND_PORT GPIOB
#define BIND_PIN Pin_6
#define BIND_PORT2 GPIOC
#define BIND_PIN2 Pin_11
#define HARDWARE_BIND_PLUG 
#define BINDPLUG_PORT GPIOA
#define BINDPLUG_PIN Pin_2
#define USE_QUATERNION 
#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD (BIT(2))
