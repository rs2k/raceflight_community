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

#define TARGET_BOARD_IDENTIFIER "CJM1" 
#define USE_HARDWARE_REVISION_DETECTION

#define LED0_GPIO GPIOC
#define LED0_PIN Pin_14 
#define LED0 PC14
#define LED0_PERIPHERAL RCC_APB2Periph_GPIOC
#define LED1_GPIO GPIOC
#define LED1_PIN Pin_13 
#define LED1 PC13
#define LED1_PERIPHERAL RCC_APB2Periph_GPIOC
#define LED2_GPIO GPIOC
#define LED2_PIN Pin_15 
#define LED2 PC15
#define LED2_PERIPHERAL RCC_APB2Periph_GPIOC


#define ACC
#define USE_ACC_MPU6050

#define GYRO
#define USE_GYRO_MPU6050




#define BRUSHED_MOTORS

#define USE_USART1
#define USE_USART2

#define SERIAL_PORT_COUNT 2

#define ESC_1WIRE

#define USE_I2C
#define I2C_DEVICE (I2CDEV_1)





#define SERIAL_RX

#define USE_CLI

#define SPEKTRUM_BIND

#define BIND_PORT  GPIOA
#define BIND_PIN   Pin_3


#define USE_QUAD_MIXER_ONLY


#if (FLASH_SIZE > 64)
#define BLACKBOX
#else
#define SKIP_TASK_STATISTICS
#define SKIP_CLI_COMMAND_HELP
#endif


#define GTUNE


#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
