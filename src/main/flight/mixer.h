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

#define MAX_SUPPORTED_MOTORS 12
#define MAX_SUPPORTED_SERVOS 8
#define YAW_JUMP_PREVENTION_LIMIT_LOW 80
#define YAW_JUMP_PREVENTION_LIMIT_HIGH 500



typedef enum mixerMode
{
    MIXER_TRI = 1,
    MIXER_QUADP = 2,
    MIXER_QUADXL = 3,
    MIXER_BICOPTER = 4,
    MIXER_GIMBAL = 5,
    MIXER_Y6 = 6,
    MIXER_HEX6 = 7,
    MIXER_FLYING_WING = 8,
    MIXER_Y4 = 9,
    MIXER_HEX6X = 10,
    MIXER_OCTOX8 = 11,
    MIXER_OCTOFLATP = 12,
    MIXER_OCTOFLATX = 13,
    MIXER_AIRPLANE = 14,        
    MIXER_HELI_120_CCPM = 15,
    MIXER_HELI_90_DEG = 16,
    MIXER_VTAIL4 = 17,
    MIXER_HEX6H = 18,
    MIXER_PPM_TO_SERVO = 19,    
    MIXER_DUALCOPTER = 20,
    MIXER_SINGLECOPTER = 21,
    MIXER_ATAIL4 = 22,
    MIXER_CUSTOM = 23,
    MIXER_CUSTOM_AIRPLANE = 24,
    MIXER_CUSTOM_TRI = 25,
	MIXER_QUADX_1234 = 26,
    MIXER_QUADX1 = 27,
    MIXER_QUADX2 = 28,

} mixerMode_e;


typedef struct motorMixer_s {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;


typedef struct mixer_s {
    uint8_t motorCount;
    uint8_t useServo;
    const motorMixer_t *motor;
} mixer_t;

typedef struct mixerConfig_s {
	float foreAftMixerFixerStrength;
    int8_t yaw_motor_direction;
    uint16_t yaw_jump_prevention_limit;      
#ifdef USE_SERVOS
    uint8_t tri_unarmed_servo;              
    int16_t servo_lowpass_freq;             
    int8_t servo_lowpass_enable;            
#endif
} mixerConfig_t;

typedef struct flight3DConfig_s {
    uint16_t deadband3d_low;                
    uint16_t deadband3d_high;               
    uint16_t neutral3d;                     
    uint16_t deadband3d_throttle;           
} flight3DConfig_t;

typedef struct airplaneConfig_s {
    int8_t fixedwing_althold_dir;           
} airplaneConfig_t;

#define CHANNEL_FORWARDING_DISABLED (uint8_t)0xFF

#ifdef USE_SERVOS


enum {
    INPUT_STABILIZED_ROLL = 0,
    INPUT_STABILIZED_PITCH,
    INPUT_STABILIZED_YAW,
    INPUT_STABILIZED_THROTTLE,
    INPUT_RC_ROLL,
    INPUT_RC_PITCH,
    INPUT_RC_YAW,
    INPUT_RC_THROTTLE,
    INPUT_RC_AUX1,
    INPUT_RC_AUX2,
    INPUT_RC_AUX3,
    INPUT_RC_AUX4,
    INPUT_GIMBAL_PITCH,
    INPUT_GIMBAL_ROLL,

    INPUT_SOURCE_COUNT
} inputSource_e;


typedef enum {
    SERVO_GIMBAL_PITCH = 0,
    SERVO_GIMBAL_ROLL = 1,
    SERVO_FLAPS = 2,
    SERVO_FLAPPERON_1 = 3,
    SERVO_FLAPPERON_2 = 4,
    SERVO_RUDDER = 5,
    SERVO_ELEVATOR = 6,
    SERVO_THROTTLE = 7, 

    SERVO_BICOPTER_LEFT = 4,
    SERVO_BICOPTER_RIGHT = 5,

    SERVO_DUALCOPTER_LEFT = 4,
    SERVO_DUALCOPTER_RIGHT = 5,

    SERVO_SINGLECOPTER_1 = 3,
    SERVO_SINGLECOPTER_2 = 4,
    SERVO_SINGLECOPTER_3 = 5,
    SERVO_SINGLECOPTER_4 = 6,

} servoIndex_e; 

#define SERVO_PLANE_INDEX_MIN SERVO_FLAPS
#define SERVO_PLANE_INDEX_MAX SERVO_THROTTLE

#define SERVO_DUALCOPTER_INDEX_MIN SERVO_DUALCOPTER_LEFT
#define SERVO_DUALCOPTER_INDEX_MAX SERVO_DUALCOPTER_RIGHT

#define SERVO_SINGLECOPTER_INDEX_MIN SERVO_SINGLECOPTER_1
#define SERVO_SINGLECOPTER_INDEX_MAX SERVO_SINGLECOPTER_4

#define SERVO_FLAPPERONS_MIN SERVO_FLAPPERON_1
#define SERVO_FLAPPERONS_MAX SERVO_FLAPPERON_2

typedef struct servoMixer_s {
    uint8_t targetChannel;                  
    uint8_t inputSource;                    
    int8_t rate;                            
    uint8_t speed;                          
    int8_t min;                             
    int8_t max;                             
    uint8_t box;                            
} servoMixer_t;

#define MAX_SERVO_RULES (2 * MAX_SUPPORTED_SERVOS)
#define MAX_SERVO_SPEED UINT8_MAX
#define MAX_SERVO_BOXES 3


typedef struct mixerRules_s {
    uint8_t servoRuleCount;
    const servoMixer_t *rule;
} mixerRules_t;

typedef struct servoParam_s {
    int16_t min;                            
    int16_t max;                            
    int16_t middle;                         
    int8_t rate;                            
    uint8_t angleAtMin;                     
    uint8_t angleAtMax;                     
    int8_t forwardFromChannel;              
    uint32_t reversedSources;               
} __attribute__ ((__packed__)) servoParam_t;

struct gimbalConfig_s;
struct escAndServoConfig_s;
struct rxConfig_s;

extern int16_t servo[MAX_SUPPORTED_SERVOS];
bool isMixerUsingServos(void);
void writeServos(void);
void filterServos(void);
#endif

extern int16_t motor[MAX_SUPPORTED_MOTORS];
extern int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];

struct escAndServoConfig_s;
struct rxConfig_s;

void mixerUseConfigs(
#ifdef USE_SERVOS
        servoParam_t *servoConfToUse,
        struct gimbalConfig_s *gimbalConfigToUse,
#endif
        flight3DConfig_t *flight3DConfigToUse,
        struct escAndServoConfig_s *escAndServoConfigToUse,
        mixerConfig_t *mixerConfigToUse,
        airplaneConfig_t *airplaneConfigToUse,
        struct rxConfig_s *rxConfigToUse);

void writeAllMotors(int16_t mc);
void mixerLoadMix(int index, motorMixer_t *customMixers);
#ifdef USE_SERVOS
void servoMixerLoadMix(int index, servoMixer_t *customServoMixers);
void loadCustomServoMixer(void);
int servoDirection(int servoIndex, int fromChannel);
#endif
void mixerResetDisarmedMotors(void);
void mixTable(void);
void writeMotors(void);
void stopMotors(void);
void stopMotorsNoDelay(void);
void StopPwmAllMotors(void);
