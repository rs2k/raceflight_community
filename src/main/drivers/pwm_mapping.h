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
       
#include "gpio.h"
#include "timer.h"
#define MAX_PWM_MOTORS 12
#define MAX_PWM_SERVOS 8
#define MAX_MOTORS 12
#define MAX_SERVOS 8
#define MAX_PWM_OUTPUT_PORTS MAX_PWM_MOTORS
#if MAX_PWM_OUTPUT_PORTS < MAX_MOTORS || MAX_PWM_OUTPUT_PORTS < MAX_SERVOS
#error Invalid motor/servo/port configuration
#endif
#define PULSE_1MS (1000)
#define MAX_INPUTS 8
#define PWM_TIMER_MHZ 1
#define ONESHOT125_TIMER_MHZ 8
#if defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F446xx)
#define MULTISHOT_TIMER_MHZ 48
#else
#define MULTISHOT_TIMER_MHZ 24
#endif
#define PWM_BRUSHED_TIMER_MHZ 8
typedef struct sonarGPIOConfig_s {
    GPIO_TypeDef *gpio;
    uint16_t triggerPin;
    uint16_t echoPin;
} sonarGPIOConfig_t;
typedef struct drv_pwm_config_s {
    bool useParallelPWM;
    bool usePPM;
    bool useSerialRx;
    bool useRSSIADC;
    bool useCurrentMeterADC;
#ifdef STM32F10X
    bool useUART2;
#endif
#ifdef STM32F303xC
    bool useUART3;
#endif
#if defined(STM32F40_41xxx) || defined (STM32F411xE) || defined(STM32F446xx)
 bool useUART2;
 bool useUART3;
 bool useUART5;
 bool useUART6;
#endif
    bool useVbat;
 bool useOneshot;
 bool useMultiShot;
 bool usePwmRate;
    bool useFastPWM;
    bool useSoftSerial;
    bool useLEDStrip;
#ifdef SONAR
    bool useSonar;
#endif
#ifdef USE_SERVOS
    bool useServos;
    bool useChannelForwarding;
#ifdef CC3D
    bool useBuzzerP6;
#endif
    uint16_t servoPwmRate;
    uint16_t servoCenterPulse;
#endif
    bool airplane;
    uint16_t motorPwmRate;
    uint16_t idlePulse;
    sonarGPIOConfig_t *sonarGPIOConfig;
} drv_pwm_config_t;
typedef enum {
  PWM_PF_NONE = 0,
  PWM_PF_MOTOR = (1 << 0),
  PWM_PF_SERVO = (1 << 1),
  PWM_PF_MOTOR_MODE_BRUSHED = (1 << 2),
  PWM_PF_OUTPUT_PROTOCOL_PWM = (1 << 3),
  PWM_PF_OUTPUT_PROTOCOL_ONESHOT = (1 << 4),
  PWM_PF_OUTPUT_PROTOCOL_MULTISHOT = (1 << 5)
} pwmPortFlags_e;
typedef struct pwmPortConfiguration_s {
    uint8_t index;
    pwmPortFlags_e flags;
    const timerHardware_t *timerHardware;
} pwmPortConfiguration_t;
typedef struct pwmOutputConfiguration_s {
    uint8_t servoCount;
    uint8_t motorCount;
    uint8_t outputCount;
    pwmPortConfiguration_t portConfigurations[MAX_PWM_OUTPUT_PORTS];
} pwmOutputConfiguration_t;
enum {
    PWM1 = 0,
    PWM2,
    PWM3,
    PWM4,
    PWM5,
    PWM6,
    PWM7,
    PWM8,
    PWM9,
    PWM10,
    PWM11,
    PWM12,
    PWM13,
    PWM14,
    PWM15,
    PWM16
};
pwmOutputConfiguration_t *pwmGetOutputConfiguration(void);
