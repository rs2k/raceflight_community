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
 * You should have received a copy of the GNU General Public License 
 * along with RaceFlight.  If not, see <http://www.gnu.org/licenses/>.
 */ 
#pragma once 
       
typedef enum {
 OWNER_FREE = 0,
 OWNER_PWMINPUT,
 OWNER_PPMINPUT,
 OWNER_PWMOUTPUT_MOTOR,
 OWNER_PWMOUTPUT_FAST,
 OWNER_PWMOUTPUT_ONESHOT,
 OWNER_PWMOUTPUT_SERVO,
 OWNER_SOFTSERIAL_RX,
 OWNER_SOFTSERIAL_TX,
 OWNER_SOFTSERIAL_RXTX,
 OWNER_SOFTSERIAL_AUXTIMER,
 OWNER_ADC,
 OWNER_SERIAL_RX,
 OWNER_SERIAL_TX,
 OWNER_SERIAL_RXTX,
 OWNER_PINDEBUG,
 OWNER_TIMER,
 OWNER_SONAR,
 OWNER_SYSTEM,
} resourceOwner_t;
typedef enum {
 RESOURCE_INPUT = 1 << 0,
 RESOURCE_OUTPUT = 1 << 1,
 RESOURCE_IO = RESOURCE_INPUT | RESOURCE_OUTPUT,
 RESOURCE_TIMER = 1 << 2,
 RESOURCE_TIMER_DUAL = 1 << 3,
 RESOURCE_USART = 1 << 4,
 RESOURCE_ADC = 1 << 5,
 RESOURCE_EXTI = 1 << 6,
    RESOURCE_I2C = 1 << 7,
    RESOURCE_SPI = 1 << 8,
} resourceType_t;
