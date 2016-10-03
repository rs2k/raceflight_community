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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "platform.h"

#include "gpio.h"
#include "timer.h"

#include "pwm_output.h"
#include "pwm_rx.h"
#include "pwm_mapping.h"

void pwmBrushedMotorConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint16_t motorPwmRate, uint16_t idlePulse);
void pwmBrushlessMotorConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint16_t motorPwmRate, uint16_t idlePulse);
void fastPWMMotorConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint16_t motorPwmRate, uint16_t idlePulse);
void pwmOneshotPwmRateMotorConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint16_t motorPwmRate, uint16_t idlePulse);
void pwmOneshotMotorConfig(const timerHardware_t *timerHardware, uint8_t motorIndex);
void pwmMultiShotPwmRateMotorConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint16_t motorPwmRate, uint16_t idlePulse);
void pwmMultiShotMotorConfig(const timerHardware_t *timerHardware, uint8_t motorIndex);
void pwmServoConfig(const timerHardware_t *timerHardware, uint8_t servoIndex, uint16_t servoPwmRate, uint16_t servoCenterPulse);


enum {
    MAP_TO_PPM_INPUT = 1,
    MAP_TO_PWM_INPUT,
    MAP_TO_MOTOR_OUTPUT,
    MAP_TO_SERVO_OUTPUT,
};


#if defined(KISS)

const uint16_t multiPPM[] = {
    PWM12 | (MAP_TO_PPM_INPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),

    PWM7  | (MAP_TO_PWM_INPUT << 8),
    PWM8  | (MAP_TO_PWM_INPUT << 8),
    PWM9  | (MAP_TO_PWM_INPUT << 8),
    PWM10 | (MAP_TO_PWM_INPUT << 8),
    PWM11 | (MAP_TO_PWM_INPUT << 8),
    PWM12 | (MAP_TO_PWM_INPUT << 8),
    0xFFFF
};

const uint16_t airPPM[] = {
    
    0xFFFF
};

const uint16_t airPWM[] = {
    
    0xFFFF
};
#endif

#if defined(NAZE) || defined(OLIMEXINO) || defined(NAZE32PRO) || defined(STM32F3DISCOVERY) || defined(EUSTM32F103RC) || defined(PORT103R)
static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),     
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_PWM_INPUT << 8),
    PWM8  | (MAP_TO_PWM_INPUT << 8),     
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),      
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM12 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT  << 8),     
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),      
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),      
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM8  | (MAP_TO_SERVO_OUTPUT  << 8),      
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_PWM_INPUT << 8),
    PWM8  | (MAP_TO_PWM_INPUT << 8),     
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),      
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT  << 8),     
    0xFFFF
};
#endif

#ifdef CC3D
static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),      
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),      
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT  << 8),     
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM2  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM3  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),     
    0xFFFF
};
static const uint16_t multiPPM_BP6[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),      
    0xFFFF
};

static const uint16_t multiPWM_BP6[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),      
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),
    0xFFFF
};

static const uint16_t airPPM_BP6[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM2  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM3  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    0xFFFF
};

static const uint16_t airPWM_BP6[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),     
    0xFFFF
};
#endif

#ifdef CJMCU
static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8), 
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM9  | (MAP_TO_PWM_INPUT << 8),
    PWM10 | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t airPPM[] = {
        0xFFFF
};

static const uint16_t airPWM[] = {
        0xFFFF
};
#endif

#if defined(COLIBRI_RACE) || defined(LUX_RACE)
static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),			
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),			
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),			
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      	
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      	
    PWM10  | (MAP_TO_MOTOR_OUTPUT << 8),      	
    PWM11  | (MAP_TO_MOTOR_OUTPUT << 8),      	
    0xFFFF
};

static const uint16_t multiPWM[] = {
    
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),			
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),			
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      	
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      	
    PWM10  | (MAP_TO_MOTOR_OUTPUT << 8),      	
    PWM11  | (MAP_TO_MOTOR_OUTPUT << 8),      	
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),			
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),			
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),			
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      	
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      	
    PWM10  | (MAP_TO_MOTOR_OUTPUT << 8),      	
    PWM11  | (MAP_TO_MOTOR_OUTPUT << 8),      	
    0xFFFF
};

static const uint16_t airPWM[] = {
    
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),			
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),			
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      	
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      	
    PWM10  | (MAP_TO_MOTOR_OUTPUT << 8),      	
    PWM11  | (MAP_TO_MOTOR_OUTPUT << 8),      	
    0xFFFF
};
#endif

#if defined(SPARKY) || defined(ALIENWIIF3)
static const uint16_t multiPPM[] = {
    PWM11 | (MAP_TO_PPM_INPUT << 8), 

    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8), 
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8), 
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8), 
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8), 
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8), 
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8), 
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8), 
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8), 
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8), 
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8), 
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t airPPM[] = {
    
    0xFFFF
};

static const uint16_t airPWM[] = {
    
    0xFFFF
};
#endif

#ifdef DOGE

const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t multiPWM[] = {
    
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8), 
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t airPWM[] = {
    
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

#endif

#ifdef SOULF4
static const uint16_t multiPPM[] = {
    PWM6  | (MAP_TO_PPM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),      
    0xFFFF
};
static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),      
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT  << 8),     
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM6  | (MAP_TO_PPM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM2  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM3  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM1  | (MAP_TO_SERVO_OUTPUT  << 8),
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),     
    0xFFFF
};
#endif

#ifdef REVO
static const uint16_t multiPPM[] = {
	PWM6  | (MAP_TO_PPM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),      
    0xFFFF
};
static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),      
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT  << 8),     
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM6  | (MAP_TO_PPM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM2  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM3  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM1  | (MAP_TO_SERVO_OUTPUT  << 8),
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),     
    0xFFFF
};
#endif

#ifdef REVOLT
static const uint16_t multiPPM[] = {
	PWM6  | (MAP_TO_PPM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),      
    0xFFFF
};
static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),      
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT  << 8),     
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM6  | (MAP_TO_PPM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM2  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM3  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM1  | (MAP_TO_SERVO_OUTPUT  << 8),
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),     
    0xFFFF
};
#endif

#ifdef REVONANO
static const uint16_t multiPPM[] = {
	PWM1  | (MAP_TO_PPM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),     
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),     
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),     
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),     
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),      
    0xFFFF
};
static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),      
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT  << 8),     
    0xFFFF
};

static const uint16_t airPPM[] = {
	PWM1  | (MAP_TO_PPM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),      
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),     
    0xFFFF
};
#endif

#ifdef SPARKY2
static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      
    0xFFFF
};
static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT  << 8),      
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),     
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     
    PWM6  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM8  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM2  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM3  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM8  | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),     
    0xFFFF
};
#endif

#ifdef BLUEJAYF4
static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     
    PWM2  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT  << 8),
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),
    0xFFFF
};
#endif

#ifdef BLUEJAYF4_REV3
static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     
    PWM2  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT  << 8),
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),
    0xFFFF
};
#endif

#ifdef ALIENFLIGHTF4
static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),         
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      
    0xFFFF
};
static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),         
    PWM2  | (MAP_TO_PWM_INPUT << 8),         
    PWM3  | (MAP_TO_PWM_INPUT << 8),         
    PWM4  | (MAP_TO_PWM_INPUT << 8),         
    PWM5  | (MAP_TO_PWM_INPUT << 8),         
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),      
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),        
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),     
    PWM8  | (MAP_TO_SERVO_OUTPUT << 8),     
    PWM9  | (MAP_TO_SERVO_OUTPUT << 8),     
    PWM10 | (MAP_TO_SERVO_OUTPUT << 8),     
    PWM11 | (MAP_TO_SERVO_OUTPUT << 8),     
    PWM12 | (MAP_TO_SERVO_OUTPUT << 8),     
    PWM13 | (MAP_TO_SERVO_OUTPUT << 8),     
    PWM2  | (MAP_TO_SERVO_OUTPUT << 8),     
    PWM3  | (MAP_TO_SERVO_OUTPUT << 8),     
    PWM4  | (MAP_TO_SERVO_OUTPUT << 8),     
    PWM5  | (MAP_TO_SERVO_OUTPUT << 8),     
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),         
    PWM2  | (MAP_TO_PWM_INPUT << 8),         
    PWM3  | (MAP_TO_PWM_INPUT << 8),         
    PWM4  | (MAP_TO_PWM_INPUT << 8),         
    PWM5  | (MAP_TO_PWM_INPUT << 8),         
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM8  | (MAP_TO_SERVO_OUTPUT << 8),      
    PWM9  | (MAP_TO_SERVO_OUTPUT << 8),      
    PWM10 | (MAP_TO_SERVO_OUTPUT << 8),      
    PWM11 | (MAP_TO_SERVO_OUTPUT << 8),      
    PWM12 | (MAP_TO_SERVO_OUTPUT << 8),      
    PWM13 | (MAP_TO_SERVO_OUTPUT << 8),      
    0xFFFF
};
#endif

#if defined(VRCORE)
static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),      
    0xFFFF
};
static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),      
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT  << 8),     
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM2  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM3  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),     
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),     
    0xFFFF
};
#endif

#ifdef QUANTON
static const uint16_t multiPPM[] = {
	PWM1  | (MAP_TO_PPM_INPUT    << 8), 

	PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM15 | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM16 | (MAP_TO_MOTOR_OUTPUT << 8),
	0xFFFF
};

static const uint16_t multiPWM[] = {
	PWM1  | (MAP_TO_PWM_INPUT << 8),
	PWM2  | (MAP_TO_PWM_INPUT << 8),
	PWM3  | (MAP_TO_PWM_INPUT << 8),
	PWM4  | (MAP_TO_PWM_INPUT << 8),
	PWM5  | (MAP_TO_PWM_INPUT << 8),
	PWM6  | (MAP_TO_PWM_INPUT << 8),
	PWM7  | (MAP_TO_PWM_INPUT << 8),
	PWM8  | (MAP_TO_PWM_INPUT << 8),
	PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),
	PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),
	PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),
	PWM12 | (MAP_TO_MOTOR_OUTPUT  << 8),
	PWM13 | (MAP_TO_MOTOR_OUTPUT  << 8),
	PWM14 | (MAP_TO_MOTOR_OUTPUT  << 8),
	PWM15 | (MAP_TO_MOTOR_OUTPUT  << 8),
	PWM16 | (MAP_TO_MOTOR_OUTPUT  << 8),
	0xFFFF
};

static const uint16_t airPPM[] = {
	PWM1  | (MAP_TO_PPM_INPUT << 8),     
	PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8), 
	PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8), 
	PWM11 | (MAP_TO_SERVO_OUTPUT  << 8), 
	PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
	PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),
	PWM14 | (MAP_TO_SERVO_OUTPUT  << 8),
	PWM15 | (MAP_TO_SERVO_OUTPUT  << 8),
	PWM16 | (MAP_TO_SERVO_OUTPUT  << 8),
	0xFFFF
};

static const uint16_t airPWM[] = {
	PWM1  | (MAP_TO_PWM_INPUT << 8),     
	PWM2  | (MAP_TO_PWM_INPUT << 8),
	PWM3  | (MAP_TO_PWM_INPUT << 8),
	PWM4  | (MAP_TO_PWM_INPUT << 8),
	PWM5  | (MAP_TO_PWM_INPUT << 8),
	PWM6  | (MAP_TO_PWM_INPUT << 8),
	PWM7  | (MAP_TO_PWM_INPUT << 8),
	PWM8  | (MAP_TO_PWM_INPUT << 8),     
	PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8), 
	PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8), 
	PWM11 | (MAP_TO_SERVO_OUTPUT  << 8), 
	PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
	PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),
	PWM14 | (MAP_TO_SERVO_OUTPUT  << 8),
	PWM15 | (MAP_TO_SERVO_OUTPUT  << 8),
	PWM16 | (MAP_TO_SERVO_OUTPUT  << 8), 
	0xFFFF
};
#endif

#ifdef KKNGF4
static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     
    PWM2  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};
#endif

#ifdef HOOLIGAN
static const uint16_t multiPPM[] = {
	PWM1  | (MAP_TO_PPM_INPUT << 8),     
	PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
	0xFFFF
};
static const uint16_t multiPWM[] = {
	PWM1  | (MAP_TO_PPM_INPUT << 8),     
	PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
	0xFFFF
};

static const uint16_t airPPM[] = {
	PWM1  | (MAP_TO_PPM_INPUT << 8),     
	PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
	0xFFFF
};

static const uint16_t airPWM[] = {
	PWM1  | (MAP_TO_PPM_INPUT << 8),     
	PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
	0xFFFF
};
#endif

#ifdef PRACER
const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};
const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),
    0xFFFF
};
const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),      
    PWM2  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT  << 8),
    0xFFFF
};
const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),
    0xFFFF
};

#endif

#ifdef SPRACINGF3
static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT    << 8), 

    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM15 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM16 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_PWM_INPUT << 8),
    PWM8  | (MAP_TO_PWM_INPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM15 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM16 | (MAP_TO_MOTOR_OUTPUT  << 8),
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8), 
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8), 
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8), 
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM15 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM16 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM8  | (MAP_TO_SERVO_OUTPUT  << 8), 
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_PWM_INPUT << 8),
    PWM8  | (MAP_TO_PWM_INPUT << 8),     
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8), 
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8), 
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8), 
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM15 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM16 | (MAP_TO_SERVO_OUTPUT  << 8), 
    0xFFFF
};
#endif

#if defined(MOTOLAB)
static const uint16_t multiPPM[] = {
    PWM9  | (MAP_TO_PPM_INPUT << 8), 

    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t airPPM[] = {
    
    0xFFFF
};

static const uint16_t airPWM[] = {
    
    0xFFFF
};
#endif

#if defined(PIKOBLX)
static const uint16_t multiPPM[] = {
    PWM9  | (MAP_TO_PPM_INPUT << 8), 

    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t airPPM[] = {
    
    0xFFFF
};

static const uint16_t airPWM[] = {
    
    0xFFFF
};
#endif

static const uint16_t * const hardwareMaps[] = {
    multiPWM,
    multiPPM,
    airPWM,
    airPPM,
};

#ifdef CC3D
static const uint16_t * const hardwareMapsBP6[] = {
    multiPWM_BP6,
    multiPPM_BP6,
    airPWM_BP6,
    airPPM_BP6,
};
#endif

static pwmOutputConfiguration_t pwmOutputConfiguration;

pwmOutputConfiguration_t *pwmGetOutputConfiguration(void){
    return &pwmOutputConfiguration;
}

pwmOutputConfiguration_t *pwmInit(drv_pwm_config_t *init)
{
    int i = 0;
    const uint16_t *setup;

    int channelIndex = 0;


    memset(&pwmOutputConfiguration, 0, sizeof(pwmOutputConfiguration));

    
    if (init->airplane)
        i = 2; 
    if (init->usePPM || init->useSerialRx)
        i++; 

#ifdef CC3D
if (init->useBuzzerP6) {
	setup = hardwareMapsBP6[i];
} else {
	setup = hardwareMaps[i];
}
#else
    setup = hardwareMaps[i];
#endif

    for (i = 0; i < USABLE_TIMER_CHANNEL_COUNT && setup[i] != 0xFFFF; i++) {
        uint8_t timerIndex = setup[i] & 0x00FF;
        uint8_t type = (setup[i] & 0xFF00) >> 8;

        const timerHardware_t *timerHardwarePtr = &timerHardware[timerIndex];


#ifdef OLIMEXINO_UNCUT_LED2_E_JUMPER
        
        if (timerIndex == PWM2)
            continue;
#endif

#ifdef STM32F10X
        
        if (init->useUART2 && (timerIndex == PWM3 || timerIndex == PWM4))
            continue;
#endif

#if defined(STM32F303xC) && defined(USE_USART3)
        
        if (init->useUART3 && timerHardwarePtr->gpio == UART3_GPIO && (timerHardwarePtr->pin == UART3_TX_PIN || timerHardwarePtr->pin == UART3_RX_PIN))
            continue;
#endif

#ifdef SOFTSERIAL_1_TIMER
        if (init->useSoftSerial && timerHardwarePtr->tim == SOFTSERIAL_1_TIMER)
            continue;
#endif
#ifdef SOFTSERIAL_2_TIMER
        if (init->useSoftSerial && timerHardwarePtr->tim == SOFTSERIAL_2_TIMER)
            continue;
#endif

#ifdef LED_STRIP_TIMER
        
        if (init->useLEDStrip) {
            if (timerHardwarePtr->tim == LED_STRIP_TIMER)
                continue;
#if defined(STM32F303xC) && defined(WS2811_GPIO) && defined(WS2811_PIN_SOURCE)
            if (timerHardwarePtr->gpio == WS2811_GPIO && timerHardwarePtr->gpioPinSource == WS2811_PIN_SOURCE)
                continue;
#endif
        }

#endif

#ifdef VBAT_ADC_GPIO
        if (init->useVbat && timerHardwarePtr->gpio == VBAT_ADC_GPIO && timerHardwarePtr->pin == VBAT_ADC_GPIO_PIN) {
            continue;
        }
#endif

#ifdef RSSI_ADC_GPIO
        if (init->useRSSIADC && timerHardwarePtr->gpio == RSSI_ADC_GPIO && timerHardwarePtr->pin == RSSI_ADC_GPIO_PIN) {
            continue;
        }
#endif

#ifdef CURRENT_METER_ADC_GPIO
        if (init->useCurrentMeterADC && timerHardwarePtr->gpio == CURRENT_METER_ADC_GPIO && timerHardwarePtr->pin == CURRENT_METER_ADC_GPIO_PIN) {
            continue;
        }
#endif

#ifdef SONAR
        if (init->sonarGPIOConfig && timerHardwarePtr->gpio == init->sonarGPIOConfig->gpio &&
            (
                timerHardwarePtr->pin == init->sonarGPIOConfig->triggerPin ||
                timerHardwarePtr->pin == init->sonarGPIOConfig->echoPin
            )
        ) {
            continue;
        }
#endif

        
        if (type == MAP_TO_PWM_INPUT && !init->useParallelPWM)
            continue;

        if (type == MAP_TO_PPM_INPUT && !init->usePPM)
            continue;

#ifdef USE_SERVOS
        if (init->useServos && !init->airplane) {
#if defined(NAZE)
            
            if ((timerIndex == PWM9 || timerIndex == PWM10) && timerHardwarePtr->tim == TIM1)
                type = MAP_TO_SERVO_OUTPUT;
#endif

#if defined(COLIBRI_RACE) || defined(LUX_RACE)
            
            if ((timerIndex == PWM6 || timerIndex == PWM7 || timerIndex == PWM8 || timerIndex == PWM9) && timerHardwarePtr->tim == TIM2)
                type = MAP_TO_SERVO_OUTPUT;
#endif

#if defined(CC3D)
            
            if ((timerIndex == PWM9 || timerIndex == PWM10) && timerHardwarePtr->tim == TIM1)
                type = MAP_TO_SERVO_OUTPUT;
#endif

#if defined(SPARKY)
            
            if ((timerIndex == PWM1 || timerIndex == PWM2) && timerHardwarePtr->tim == TIM15)
                type = MAP_TO_SERVO_OUTPUT;
#endif

#if defined(SPRACINGF3)
            
            if ((timerIndex == PWM15 || timerIndex == PWM16) && timerHardwarePtr->tim == TIM15)
                type = MAP_TO_SERVO_OUTPUT;
#endif

#if defined(NAZE32PRO) || (defined(STM32F3DISCOVERY) && !defined(CHEBUZZF3))
            
            if (init->useSoftSerial) {
                if (timerIndex == PWM5 || timerIndex == PWM6)
                    type = MAP_TO_SERVO_OUTPUT;
            } else {
                if (timerIndex == PWM9 || timerIndex == PWM10)
                    type = MAP_TO_SERVO_OUTPUT;
            }
#endif

#if defined(MOTOLAB)
            
            if (timerIndex == PWM7 || timerIndex == PWM8)
                type = MAP_TO_SERVO_OUTPUT;
#endif
        }


        if (init->useChannelForwarding && !init->airplane) {
#if defined(NAZE) && defined(LED_STRIP_TIMER)
            
            if (init->useLEDStrip) {
                if (timerIndex >= PWM13 && timerIndex <= PWM14) {
                  type = MAP_TO_SERVO_OUTPUT;
                }
            } else
#endif
                
                if (timerIndex >= PWM5 && timerIndex <= PWM8)
                    type = MAP_TO_SERVO_OUTPUT;
        }
#endif

#ifdef CC3D
        if (init->useParallelPWM) {
            
            if ((type == MAP_TO_SERVO_OUTPUT || type == MAP_TO_MOTOR_OUTPUT) && (timerHardwarePtr->tim == TIM2 || timerHardwarePtr->tim == TIM3)) {
                continue;
            }
            if (type == MAP_TO_PWM_INPUT && timerHardwarePtr->tim == TIM4) {
                continue;
            }

        }
#endif

        if (type == MAP_TO_PPM_INPUT) {
#ifdef REVO
            if (init->useMultiShot || init->useOneshot || isMotorBrushed(init->motorPwmRate)) {
                ppmAvoidPWMTimerClash(timerHardwarePtr, TIM12);
                ppmAvoidPWMTimerClash(timerHardwarePtr, TIM8);
            }
#endif
#ifdef REVONANO
            if (init->useMultiShot || init->useOneshot || isMotorBrushed(init->motorPwmRate)) {
                ppmAvoidPWMTimerClash(timerHardwarePtr, TIM2);
            }
#endif
#ifdef SPARKY2
            if (init->useMultiShot || init->useOneshot || isMotorBrushed(init->motorPwmRate)) {
                ppmAvoidPWMTimerClash(timerHardwarePtr, TIM8);
            }
#endif
#ifdef ALIENFLIGHTF4
            if (init->useMultiShot || init->useOneshot || isMotorBrushed(init->motorPwmRate)) {
                ppmAvoidPWMTimerClash(timerHardwarePtr, TIM1);
            }
#endif
#ifdef VRCORE
            if (init->useMultiShot || init->useOneshot || isMotorBrushed(init->motorPwmRate)) {
                ppmAvoidPWMTimerClash(timerHardwarePtr, TIM1);
            }
#endif





#ifdef CC3D
            if (init->useMultiShot || init->useOneshot || isMotorBrushed(init->motorPwmRate)) {
                ppmAvoidPWMTimerClash(timerHardwarePtr, TIM4);
            }
#endif
#ifdef SPARKY
            if (init->useMultiShot || init->useOneshot || isMotorBrushed(init->motorPwmRate)) {
                ppmAvoidPWMTimerClash(timerHardwarePtr, TIM2);
            }
#endif
            ppmInConfig(timerHardwarePtr);
        } else if (type == MAP_TO_PWM_INPUT) {
            pwmInConfig(timerHardwarePtr, channelIndex);
            channelIndex++;
        } else if (type == MAP_TO_MOTOR_OUTPUT) {
            if (init->useOneshot)
            {
                if (init->useFastPWM)
                {
                    fastPWMMotorConfig(timerHardwarePtr, pwmOutputConfiguration.motorCount, init->motorPwmRate, init->idlePulse);
                }
                else
                {
                    if (init->usePwmRate)
                    {
                        pwmOneshotPwmRateMotorConfig(timerHardwarePtr, pwmOutputConfiguration.motorCount, init->motorPwmRate, init->idlePulse);
                    }
                    else
                    {
                        pwmOneshotMotorConfig(timerHardwarePtr, pwmOutputConfiguration.motorCount);
                    }
                }
                pwmOutputConfiguration.portConfigurations[pwmOutputConfiguration.outputCount].flags = PWM_PF_MOTOR | PWM_PF_OUTPUT_PROTOCOL_ONESHOT|PWM_PF_OUTPUT_PROTOCOL_PWM ;
            } 
            else if (init->useMultiShot) {
                if (init->usePwmRate)
                {
                    pwmMultiShotPwmRateMotorConfig(timerHardwarePtr, pwmOutputConfiguration.motorCount, init->motorPwmRate, init->idlePulse);
                }
                else
                {
                	pwmMultiShotMotorConfig(timerHardwarePtr, pwmOutputConfiguration.motorCount);
                }
                pwmOutputConfiguration.portConfigurations[pwmOutputConfiguration.outputCount].flags = PWM_PF_MOTOR | PWM_PF_OUTPUT_PROTOCOL_MULTISHOT|PWM_PF_OUTPUT_PROTOCOL_PWM ;
            } else if (isMotorBrushed(init->motorPwmRate)) {
                pwmBrushedMotorConfig(timerHardwarePtr, pwmOutputConfiguration.motorCount, init->motorPwmRate, init->idlePulse);
                pwmOutputConfiguration.portConfigurations[pwmOutputConfiguration.outputCount].flags = PWM_PF_MOTOR | PWM_PF_MOTOR_MODE_BRUSHED | PWM_PF_OUTPUT_PROTOCOL_PWM;
            } else {
                pwmBrushlessMotorConfig(timerHardwarePtr, pwmOutputConfiguration.motorCount, init->motorPwmRate, init->idlePulse);
                pwmOutputConfiguration.portConfigurations[pwmOutputConfiguration.outputCount].flags = PWM_PF_MOTOR | PWM_PF_OUTPUT_PROTOCOL_PWM ;
            }
            pwmOutputConfiguration.portConfigurations[pwmOutputConfiguration.outputCount].index = pwmOutputConfiguration.motorCount;
            pwmOutputConfiguration.portConfigurations[pwmOutputConfiguration.outputCount].timerHardware = timerHardwarePtr;
            pwmOutputConfiguration.motorCount++;
            pwmOutputConfiguration.outputCount++;
        } else if (type == MAP_TO_SERVO_OUTPUT) {
#ifdef USE_SERVOS
            pwmOutputConfiguration.portConfigurations[pwmOutputConfiguration.outputCount].index = pwmOutputConfiguration.servoCount;
            pwmServoConfig(timerHardwarePtr, pwmOutputConfiguration.servoCount, init->servoPwmRate, init->servoCenterPulse);
            pwmOutputConfiguration.portConfigurations[pwmOutputConfiguration.outputCount].flags = PWM_PF_SERVO | PWM_PF_OUTPUT_PROTOCOL_PWM;
            pwmOutputConfiguration.servoCount++;
            pwmOutputConfiguration.outputCount++;
#endif
        }
    }

    return &pwmOutputConfiguration;
}
