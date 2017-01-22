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
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "platform.h"
#include "build_config.h"
#include "common/axis.h"
#include "common/maths.h"
#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_mapping.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/system.h"
#include "rx/rx.h"
#include "io/gimbal.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/lowpass.h"
#include "config/runtime_config.h"
#include "config/config.h"
#include "include.h"
#undef MIXER_DEBUG
uint8_t motorCount;
int16_t motor[MAX_SUPPORTED_MOTORS];
int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];
static mixerConfig_t *mixerConfig;
static flight3DConfig_t *flight3DConfig;
static escAndServoConfig_t *escAndServoConfig;
static airplaneConfig_t *airplaneConfig;
static rxConfig_t *rxConfig;
static mixerMode_e currentMixerMode;
static motorMixer_t currentMixer[MAX_SUPPORTED_MOTORS];
bool motorLimitReached = false;
#ifdef USE_SERVOS
static uint8_t servoRuleCount = 0;
static servoMixer_t currentServoMixer[MAX_SERVO_RULES];
static gimbalConfig_t *gimbalConfig;
int16_t servo[MAX_SUPPORTED_SERVOS];
static int useServo;
STATIC_UNIT_TESTED uint8_t servoCount;
static servoParam_t *servoConf;
static lowpass_t lowpassFilters[MAX_SUPPORTED_SERVOS];
#endif
static const motorMixer_t mixerQuadXL[] = {
    { 1.0f, -1.0f, 1.0f, -1.0f },
    { 1.0f, -1.0f, -1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, -1.0f, -1.0f },
};
static const motorMixer_t mixerQuadX1[] = {
    { 1.0f, -1.0f, 0.80f, -1.0f },
    { 1.0f, -1.0f, -0.80f, 1.0f },
    { 1.0f, 1.0f, 0.80f, 1.0f },
    { 1.0f, 1.0f, -0.80f, -1.0f },
};
static const motorMixer_t mixerQuadX2[] = {
    { 1.0f, -0.80f, 1.0f, -1.0f },
    { 1.0f, -0.80f, -1.0f, 1.0f },
    { 1.0f, 0.80f, 1.0f, 1.0f },
    { 1.0f, 0.80f, -1.0f, -1.0f },
};
static const motorMixer_t mixerTricopter[] = {
    { 1.0f, 0.0f, 1.333333f, 0.0f },
    { 1.0f, -1.0f, -0.666667f, 0.0f },
    { 1.0f, 1.0f, -0.666667f, 0.0f },
};
static const motorMixer_t mixerQuadP[] = {
    { 1.0f, 0.0f, 1.0f, -1.0f },
    { 1.0f, -1.0f, 0.0f, 1.0f },
    { 1.0f, 1.0f, 0.0f, 1.0f },
    { 1.0f, 0.0f, -1.0f, -1.0f },
};
static const motorMixer_t mixerBicopter[] = {
    { 1.0f, 1.0f, 0.0f, 0.0f },
    { 1.0f, -1.0f, 0.0f, 0.0f },
};
static const motorMixer_t mixerY6[] = {
    { 1.0f, 0.0f, 1.333333f, 1.0f },
    { 1.0f, -1.0f, -0.666667f, -1.0f },
    { 1.0f, 1.0f, -0.666667f, -1.0f },
    { 1.0f, 0.0f, 1.333333f, -1.0f },
    { 1.0f, -1.0f, -0.666667f, 1.0f },
    { 1.0f, 1.0f, -0.666667f, 1.0f },
};
static const motorMixer_t mixerHex6P[] = {
    { 1.0f, -0.866025f, 0.5f, 1.0f },
    { 1.0f, -0.866025f, -0.5f, -1.0f },
    { 1.0f, 0.866025f, 0.5f, 1.0f },
    { 1.0f, 0.866025f, -0.5f, -1.0f },
    { 1.0f, 0.0f, -1.0f, 1.0f },
    { 1.0f, 0.0f, 1.0f, -1.0f },
};
static const motorMixer_t mixerY4[] = {
    { 1.0f, 0.0f, 1.0f, -1.0f },
    { 1.0f, -1.0f, -1.0f, 0.0f },
    { 1.0f, 0.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, -1.0f, 0.0f },
};
static const motorMixer_t mixerHex6X[] = {
    { 1.0f, -0.5f, 0.866025f, 1.0f },
    { 1.0f, -0.5f, -0.866025f, 1.0f },
    { 1.0f, 0.5f, 0.866025f, -1.0f },
    { 1.0f, 0.5f, -0.866025f, -1.0f },
    { 1.0f, -1.0f, 0.0f, -1.0f },
    { 1.0f, 1.0f, 0.0f, 1.0f },
};
static const motorMixer_t mixerOctoX8[] = {
    { 1.0f, -1.0f, 1.0f, -1.0f },
    { 1.0f, -1.0f, -1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, -1.0f, -1.0f },
    { 1.0f, -1.0f, 1.0f, 1.0f },
    { 1.0f, -1.0f, -1.0f, -1.0f },
    { 1.0f, 1.0f, 1.0f, -1.0f },
    { 1.0f, 1.0f, -1.0f, 1.0f },
};
static const motorMixer_t mixerOctoFlatP[] = {
    { 1.0f, 0.707107f, -0.707107f, 1.0f },
    { 1.0f, -0.707107f, -0.707107f, 1.0f },
    { 1.0f, -0.707107f, 0.707107f, 1.0f },
    { 1.0f, 0.707107f, 0.707107f, 1.0f },
    { 1.0f, 0.0f, -1.0f, -1.0f },
    { 1.0f, -1.0f, 0.0f, -1.0f },
    { 1.0f, 0.0f, 1.0f, -1.0f },
    { 1.0f, 1.0f, 0.0f, -1.0f },
};
static const motorMixer_t mixerOctoFlatX[] = {
    { 1.0f, 1.0f, -0.414178f, 1.0f },
    { 1.0f, -0.414178f, -1.0f, 1.0f },
    { 1.0f, -1.0f, 0.414178f, 1.0f },
    { 1.0f, 0.414178f, 1.0f, 1.0f },
    { 1.0f, 0.414178f, -1.0f, -1.0f },
    { 1.0f, -1.0f, -0.414178f, -1.0f },
    { 1.0f, -0.414178f, 1.0f, -1.0f },
    { 1.0f, 1.0f, 0.414178f, -1.0f },
};
static const motorMixer_t mixerVtail4[] = {
    { 1.0f, -0.58f, 0.58f, 1.0f },
    { 1.0f, -0.46f, -0.39f, -0.5f },
    { 1.0f, 0.58f, 0.58f, -1.0f },
    { 1.0f, 0.46f, -0.39f, 0.5f },
};
static const motorMixer_t mixerAtail4[] = {
    { 1.0f, 0.0f, 1.0f, 1.0f },
    { 1.0f, -1.0f, -1.0f, 0.0f },
    { 1.0f, 0.0f, 1.0f, -1.0f },
    { 1.0f, 1.0f, -1.0f, -0.0f },
};
static const motorMixer_t mixerHex6H[] = {
    { 1.0f, -1.0f, 1.0f, -1.0f },
    { 1.0f, -1.0f, -1.0f, 1.0f },
    { 1.0f, 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, -1.0f, -1.0f },
    { 1.0f, 0.0f, 0.0f, 0.0f },
    { 1.0f, 0.0f, 0.0f, 0.0f },
};
static const motorMixer_t mixerDualcopter[] = {
    { 1.0f, 0.0f, 0.0f, -1.0f },
    { 1.0f, 0.0f, 0.0f, 1.0f },
};
static const motorMixer_t mixerSingleProp[] = {
    { 1.0f, 0.0f, 0.0f, 0.0f },
};
static const motorMixer_t mixerQuadX1234[] = {
 { 1.0f, 0.80f, -1.0f, -1.0f },
 { 1.0f, -0.80f, -1.0f, 1.0f },
 { 1.0f, -0.80f, 1.0f, -1.0f },
 { 1.0f, 0.80f, 1.0f, 1.0f },
};
static const motorMixer_t mixerQuadXL1234[] = {
    { 1.0f, 1.0f, -1.0f, -1.0f },
    { 1.0f, -1.0f, -1.0f, 1.0f },
    { 1.0f, -1.0f, 1.0f, -1.0f },
    { 1.0f, 1.0f, 1.0f, 1.0f },
};
static const motorMixer_t mixerQuadWideX1234[] = {
    { 1.0f, 1.0f, -0.75f, -1.0f },
    { 1.0f, -1.0f, -0.75f, 1.0f },
    { 1.0f, -1.0f, 0.75f, -1.0f },
    { 1.0f, 1.0f, 0.75f, 1.0f },
};
static const motorMixer_t mixerQuadX2143[] = {
    { 1.0f, -1.0f, -1.0f, 1.0f },
    { 1.0f, 1.0f, -1.0f, -1.0f },
    { 1.0f, 1.0f, 1.0f, 1.0f },
    { 1.0f, -1.0f, 1.0f, -1.0f },
};
const mixer_t mixers[] = {
    { 0, false, NULL },
    { 3, true, mixerTricopter },
    { 4, false, mixerQuadP },
    { 4, false, mixerQuadXL },
    { 2, true, mixerBicopter },
    { 0, true, NULL },
    { 6, false, mixerY6 },
    { 6, false, mixerHex6P },
    { 1, true, mixerSingleProp },
    { 4, false, mixerY4 },
    { 6, false, mixerHex6X },
    { 8, false, mixerOctoX8 },
    { 8, false, mixerOctoFlatP },
    { 8, false, mixerOctoFlatX },
    { 1, true, mixerSingleProp },
    { 0, true, NULL },
    { 0, true, NULL },
    { 4, false, mixerVtail4 },
    { 6, false, mixerHex6H },
    { 0, true, NULL },
    { 2, true, mixerDualcopter },
    { 1, true, NULL },
    { 4, false, mixerAtail4 },
    { 0, false, NULL },
    { 2, true, NULL },
    { 3, true, NULL },
    { 4, false, mixerQuadX1234 },
    { 4, false, mixerQuadX2143 },
    { 4, false, mixerQuadX1 },
    { 4, false, mixerQuadX2 },
    { 4, false, mixerQuadXL1234 },
    { 4, false, mixerQuadWideX1234 },
};
#ifdef USE_SERVOS
#define COUNT_SERVO_RULES(rules) (sizeof(rules) / sizeof(servoMixer_t))
static const servoMixer_t servoMixerAirplane[] = {
    { SERVO_FLAPPERON_1, INPUT_STABILIZED_ROLL, 100, 0, 0, 100, 0 },
    { SERVO_FLAPPERON_2, INPUT_STABILIZED_ROLL, 100, 0, 0, 100, 0 },
    { SERVO_RUDDER, INPUT_STABILIZED_YAW, 100, 0, 0, 100, 0 },
    { SERVO_ELEVATOR, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_THROTTLE, INPUT_STABILIZED_THROTTLE, 100, 0, 0, 100, 0 },
};
static const servoMixer_t servoMixerFlyingWing[] = {
    { SERVO_FLAPPERON_1, INPUT_STABILIZED_ROLL, 100, 0, 0, 100, 0 },
    { SERVO_FLAPPERON_1, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_FLAPPERON_2, INPUT_STABILIZED_ROLL, -100, 0, 0, 100, 0 },
    { SERVO_FLAPPERON_2, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_THROTTLE, INPUT_STABILIZED_THROTTLE, 100, 0, 0, 100, 0 },
};
static const servoMixer_t servoMixerBI[] = {
    { SERVO_BICOPTER_LEFT, INPUT_STABILIZED_YAW, 100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_LEFT, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_RIGHT, INPUT_STABILIZED_YAW, 100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_RIGHT, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
};
static const servoMixer_t servoMixerTri[] = {
    { SERVO_RUDDER, INPUT_STABILIZED_YAW, 100, 0, 0, 100, 0 },
};
static const servoMixer_t servoMixerDual[] = {
    { SERVO_DUALCOPTER_LEFT, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_DUALCOPTER_RIGHT, INPUT_STABILIZED_ROLL, 100, 0, 0, 100, 0 },
};
static const servoMixer_t servoMixerSingle[] = {
    { SERVO_SINGLECOPTER_1, INPUT_STABILIZED_YAW, 100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_1, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_2, INPUT_STABILIZED_YAW, 100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_2, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_3, INPUT_STABILIZED_YAW, 100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_3, INPUT_STABILIZED_ROLL, 100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_4, INPUT_STABILIZED_YAW, 100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_4, INPUT_STABILIZED_ROLL, 100, 0, 0, 100, 0 },
};
static const servoMixer_t servoMixerGimbal[] = {
    { SERVO_GIMBAL_PITCH, INPUT_GIMBAL_PITCH, 125, 0, 0, 100, 0 },
    { SERVO_GIMBAL_ROLL, INPUT_GIMBAL_ROLL, 125, 0, 0, 100, 0 },
};
const mixerRules_t servoMixers[] = {
    { 0, NULL },
    { COUNT_SERVO_RULES(servoMixerTri), servoMixerTri },
    { 0, NULL },
    { 0, NULL },
    { COUNT_SERVO_RULES(servoMixerBI), servoMixerBI },
    { COUNT_SERVO_RULES(servoMixerGimbal), servoMixerGimbal },
    { 0, NULL },
    { 0, NULL },
    { COUNT_SERVO_RULES(servoMixerFlyingWing), servoMixerFlyingWing },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { COUNT_SERVO_RULES(servoMixerAirplane), servoMixerAirplane },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { COUNT_SERVO_RULES(servoMixerDual), servoMixerDual },
    { COUNT_SERVO_RULES(servoMixerSingle), servoMixerSingle },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
    { 0, NULL },
};
static servoMixer_t *customServoMixers;
#endif
static motorMixer_t *customMixers;
void mixerUseConfigs(
#ifdef USE_SERVOS
        servoParam_t *servoConfToUse,
        gimbalConfig_t *gimbalConfigToUse,
#endif
        flight3DConfig_t *flight3DConfigToUse,
        escAndServoConfig_t *escAndServoConfigToUse,
        mixerConfig_t *mixerConfigToUse,
        airplaneConfig_t *airplaneConfigToUse,
        rxConfig_t *rxConfigToUse)
{
#ifdef USE_SERVOS
    servoConf = servoConfToUse;
    gimbalConfig = gimbalConfigToUse;
#endif
    flight3DConfig = flight3DConfigToUse;
    escAndServoConfig = escAndServoConfigToUse;
    mixerConfig = mixerConfigToUse;
    airplaneConfig = airplaneConfigToUse;
    rxConfig = rxConfigToUse;
}
#ifdef USE_SERVOS
int16_t determineServoMiddleOrForwardFromChannel(servoIndex_e servoIndex)
{
    uint8_t channelToForwardFrom = servoConf[servoIndex].forwardFromChannel;
    if (channelToForwardFrom != CHANNEL_FORWARDING_DISABLED && channelToForwardFrom < rxRuntimeConfig.channelCount) {
        return rcData[channelToForwardFrom];
    }
    return servoConf[servoIndex].middle;
}
int servoDirection(int servoIndex, int inputSource)
{
    if (servoConf[servoIndex].reversedSources & (1 << inputSource))
        return -1;
    else
        return 1;
}
#endif
#ifndef USE_QUAD_MIXER_ONLY
void loadCustomServoMixer(void)
{
    uint8_t i;
    servoRuleCount = 0;
    memset(currentServoMixer, 0, sizeof(currentServoMixer));
    for (i = 0; i < MAX_SERVO_RULES; i++) {
        if (customServoMixers[i].rate == 0)
            break;
        currentServoMixer[i] = customServoMixers[i];
        servoRuleCount++;
    }
}
void mixerInit(mixerMode_e mixerMode, motorMixer_t *initialCustomMotorMixers, servoMixer_t *initialCustomServoMixers)
{
    currentMixerMode = mixerMode;
    customMixers = initialCustomMotorMixers;
    customServoMixers = initialCustomServoMixers;
    useServo = mixers[currentMixerMode].useServo;
    if (feature(FEATURE_SERVO_TILT))
        useServo = 1;
    for (uint8_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = DEFAULT_SERVO_MIDDLE;
    }
}
void mixerUsePWMOutputConfiguration(pwmOutputConfiguration_t *pwmOutputConfiguration)
{
    int i;
    motorCount = 0;
    servoCount = pwmOutputConfiguration->servoCount;
    if (currentMixerMode == MIXER_CUSTOM || currentMixerMode == MIXER_CUSTOM_TRI || currentMixerMode == MIXER_CUSTOM_AIRPLANE) {
        for (i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            if (customMixers[i].throttle == 0.0f)
                break;
            currentMixer[i] = customMixers[i];
            motorCount++;
        }
    } else {
        motorCount = mixers[currentMixerMode].motorCount;
        if (mixers[currentMixerMode].motor) {
            for (i = 0; i < motorCount; i++)
                currentMixer[i] = mixers[currentMixerMode].motor[i];
        }
    }
    if (useServo) {
        servoRuleCount = servoMixers[currentMixerMode].servoRuleCount;
        if (servoMixers[currentMixerMode].rule) {
            for (i = 0; i < servoRuleCount; i++)
                currentServoMixer[i] = servoMixers[currentMixerMode].rule[i];
        }
    }
    if (feature(FEATURE_3D)) {
        if (motorCount > 1) {
            for (i = 0; i < motorCount; i++) {
                currentMixer[i].pitch *= 0.5f;
                currentMixer[i].roll *= 0.5f;
                currentMixer[i].yaw *= 0.5f;
            }
        }
    }
    if (currentMixerMode == MIXER_FLYING_WING ||
        currentMixerMode == MIXER_AIRPLANE ||
        currentMixerMode == MIXER_CUSTOM_AIRPLANE
    ) {
        ENABLE_STATE(FIXED_WING);
        if (currentMixerMode == MIXER_CUSTOM_AIRPLANE) {
            loadCustomServoMixer();
        }
    } else {
        DISABLE_STATE(FIXED_WING);
        if (currentMixerMode == MIXER_CUSTOM_TRI) {
            loadCustomServoMixer();
        }
    }
    mixerResetDisarmedMotors();
}
void servoMixerLoadMix(int index, servoMixer_t *customServoMixers)
{
    int i;
    index++;
    for (i = 0; i < MAX_SERVO_RULES; i++)
        customServoMixers[i].targetChannel = customServoMixers[i].inputSource = customServoMixers[i].rate = customServoMixers[i].box = 0;
    for (i = 0; i < servoMixers[index].servoRuleCount; i++)
        customServoMixers[i] = servoMixers[index].rule[i];
}
void mixerLoadMix(int index, motorMixer_t *customMixers)
{
    int i;
    index++;
    for (i = 0; i < MAX_SUPPORTED_MOTORS; i++)
        customMixers[i].throttle = 0.0f;
    if (mixers[index].motor != NULL) {
        for (i = 0; i < mixers[index].motorCount; i++)
            customMixers[i] = mixers[index].motor[i];
    }
}
#else
void mixerInit(mixerMode_e mixerMode, motorMixer_t *initialCustomMixers)
{
    currentMixerMode = mixerMode;
    customMixers = initialCustomMixers;
}
void mixerUsePWMOutputConfiguration(pwmOutputConfiguration_t *pwmOutputConfiguration)
{
    UNUSED(pwmOutputConfiguration);
    motorCount = 4;
#ifdef USE_SERVOS
    servoCount = 0;
#endif
    uint8_t i;
    for (i = 0; i < motorCount; i++) {
        currentMixer[i] = mixerQuadXL[i];
    }
    mixerResetDisarmedMotors();
}
#endif
void mixerResetDisarmedMotors(void)
{
    int i;
    for (i = 0; i < MAX_SUPPORTED_MOTORS; i++)
        motor_disarmed[i] = feature(FEATURE_3D) ? flight3DConfig->neutral3d : escAndServoConfig->realmincommand;
}
#ifdef USE_SERVOS
STATIC_UNIT_TESTED void forwardAuxChannelsToServos(uint8_t firstServoIndex)
{
    uint8_t channelOffset = AUX1;
    uint8_t servoOffset;
    for (servoOffset = 0; servoOffset < MAX_AUX_CHANNEL_COUNT && channelOffset < MAX_SUPPORTED_RC_CHANNEL_COUNT; servoOffset++) {
        pwmWriteServo(firstServoIndex + servoOffset, rcData[channelOffset++]);
    }
}
static void updateGimbalServos(uint8_t firstServoIndex)
{
    pwmWriteServo(firstServoIndex + 0, servo[SERVO_GIMBAL_PITCH]);
    pwmWriteServo(firstServoIndex + 1, servo[SERVO_GIMBAL_ROLL]);
}
void writeServos(void)
{
    uint8_t servoIndex = 0;
    switch (currentMixerMode) {
        case MIXER_BICOPTER:
            pwmWriteServo(servoIndex++, servo[SERVO_BICOPTER_LEFT]);
            pwmWriteServo(servoIndex++, servo[SERVO_BICOPTER_RIGHT]);
            break;
        case MIXER_TRI:
        case MIXER_CUSTOM_TRI:
            if (mixerConfig->tri_unarmed_servo) {
                pwmWriteServo(servoIndex++, servo[SERVO_RUDDER]);
            } else {
                if (ARMING_FLAG(ARMED))
                    pwmWriteServo(servoIndex++, servo[SERVO_RUDDER]);
                else
                    pwmWriteServo(servoIndex++, 0);
            }
            break;
        case MIXER_FLYING_WING:
            pwmWriteServo(servoIndex++, servo[SERVO_FLAPPERON_1]);
            pwmWriteServo(servoIndex++, servo[SERVO_FLAPPERON_2]);
            break;
        case MIXER_DUALCOPTER:
            pwmWriteServo(servoIndex++, servo[SERVO_DUALCOPTER_LEFT]);
            pwmWriteServo(servoIndex++, servo[SERVO_DUALCOPTER_RIGHT]);
            break;
        case MIXER_CUSTOM_AIRPLANE:
        case MIXER_AIRPLANE:
            for (int i = SERVO_PLANE_INDEX_MIN; i <= SERVO_PLANE_INDEX_MAX; i++) {
                pwmWriteServo(servoIndex++, servo[i]);
            }
            break;
        case MIXER_SINGLECOPTER:
            for (int i = SERVO_SINGLECOPTER_INDEX_MIN; i <= SERVO_SINGLECOPTER_INDEX_MAX; i++) {
                pwmWriteServo(servoIndex++, servo[i]);
            }
            break;
        default:
            break;
    }
    if (feature(FEATURE_SERVO_TILT) || currentMixerMode == MIXER_GIMBAL) {
        updateGimbalServos(servoIndex);
        servoIndex += 2;
    }
    if (feature(FEATURE_CHANNEL_FORWARDING)) {
        forwardAuxChannelsToServos(servoIndex);
        servoIndex += MAX_AUX_CHANNEL_COUNT;
    }
}
#endif
void writeMotors(void)
{
    uint8_t i;
    for (i = 0; i < motorCount; i++)
        pwmWriteMotor(i, motor[i]);
    if (feature(FEATURE_MULTISHOT) || (feature(FEATURE_ONESHOT125))) {
     if (!feature(FEATURE_USE_PWM_RATE)) {
      pwmCompleteOneshotMotorUpdate(motorCount);
     }
    }
}
void writeAllMotors(int16_t mc)
{
    uint8_t i;
    for (i = 0; i < motorCount; i++)
        motor[i] = mc;
    writeMotors();
}
void stopMotorsNoDelay(void)
{
    writeAllMotors(feature(FEATURE_3D) ? flight3DConfig->neutral3d : escAndServoConfig->realmincommand);
}
void stopMotors(void)
{
    stopMotorsNoDelay();
    delay(50);
}
void StopPwmAllMotors()
{
    pwmShutdownPulsesForAllMotors(motorCount);
}
#ifndef USE_QUAD_MIXER_ONLY
STATIC_UNIT_TESTED void servoMixer(void)
{
    int16_t input[INPUT_SOURCE_COUNT];
    static int16_t currentOutput[MAX_SERVO_RULES];
    uint8_t i;
    if (FLIGHT_MODE(PASSTHRU_MODE)) {
        input[INPUT_STABILIZED_ROLL] = rcCommandUsed[ROLL];
        input[INPUT_STABILIZED_PITCH] = rcCommandUsed[PITCH];
        input[INPUT_STABILIZED_YAW] = rcCommandUsed[YAW];
    } else {
        input[INPUT_STABILIZED_ROLL] = axisPID[ROLL];
        input[INPUT_STABILIZED_PITCH] = axisPID[PITCH];
        input[INPUT_STABILIZED_YAW] = axisPID[YAW];
        if (feature(FEATURE_3D) && (rcData[THROTTLE] < rxConfig->midrc)) {
            input[INPUT_STABILIZED_YAW] *= -1;
        }
    }
    input[INPUT_GIMBAL_PITCH] = scaleRange(attitude.values.pitch, -1800, 1800, -500, +500);
    input[INPUT_GIMBAL_ROLL] = scaleRange(attitude.values.roll, -1800, 1800, -500, +500);
    input[INPUT_STABILIZED_THROTTLE] = motor[0] - 1000 - 500;
    input[INPUT_RC_ROLL] = rcData[ROLL] - rxConfig->midrc;
    input[INPUT_RC_PITCH] = rcData[PITCH] - rxConfig->midrc;
    input[INPUT_RC_YAW] = rcData[YAW] - rxConfig->midrc;
    input[INPUT_RC_THROTTLE] = rcData[THROTTLE] - rxConfig->midrc;
    input[INPUT_RC_AUX1] = rcData[AUX1] - rxConfig->midrc;
    input[INPUT_RC_AUX2] = rcData[AUX2] - rxConfig->midrc;
    input[INPUT_RC_AUX3] = rcData[AUX3] - rxConfig->midrc;
    input[INPUT_RC_AUX4] = rcData[AUX4] - rxConfig->midrc;
    for (i = 0; i < MAX_SUPPORTED_SERVOS; i++)
        servo[i] = 0;
    for (i = 0; i < servoRuleCount; i++) {
        if (currentServoMixer[i].box == 0 || 0) {
            uint8_t target = currentServoMixer[i].targetChannel;
            uint8_t from = currentServoMixer[i].inputSource;
            uint16_t servo_width = servoConf[target].max - servoConf[target].min;
            int16_t min = currentServoMixer[i].min * servo_width / 100 - servo_width / 2;
            int16_t max = currentServoMixer[i].max * servo_width / 100 - servo_width / 2;
            if (currentServoMixer[i].speed == 0)
                currentOutput[i] = input[from];
            else {
                if (currentOutput[i] < input[from])
                    currentOutput[i] = constrain(currentOutput[i] + currentServoMixer[i].speed, currentOutput[i], input[from]);
                else if (currentOutput[i] > input[from])
                    currentOutput[i] = constrain(currentOutput[i] - currentServoMixer[i].speed, input[from], currentOutput[i]);
            }
            servo[target] += servoDirection(target, from) * constrain(((int32_t)currentOutput[i] * currentServoMixer[i].rate) / 100, min, max);
        } else {
            currentOutput[i] = 0;
        }
    }
    for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = ((int32_t)servoConf[i].rate * servo[i]) / 100L;
        servo[i] += determineServoMiddleOrForwardFromChannel(i);
    }
}
#endif
void mixTable(void)
{
 static int32_t throttleToKiAverage[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
 uint32_t i;
 static int16_t mixReduction;
 bool isFailsafeActive = failsafeIsActive();
 int16_t rollPitchYawMix[MAX_SUPPORTED_MOTORS];
 int16_t currentStabilizerThrottle[MAX_SUPPORTED_MOTORS];
 int16_t rollPitchYawMixMax = 0;
 int16_t rollPitchYawMixMin = 0;
 float OldRange, OldValue, NewRange, OldMax, OldMin, NewMax, NewMin;
 uint8_t mixerFixer = 0;
 switch (masterConfig.mixerMode) {
  case MIXER_QUADXL:
  case MIXER_QUADX1:
  case MIXER_QUADX2:
   mixerFixer = 1;
   break;
  case MIXER_QUADX_1234:
   mixerFixer = 2;
   break;
 }
 for (i = 0; i < motorCount; i++) {
  currentStabilizerThrottle[i] = applyTPA( (float)((motor[i] - 1000) / 2000) );
   uint8_t throttleTen = (uint8_t)(Throttle_p*10.0f);
   throttleToKiAverage[throttleTen] = (int32_t)((throttleToKiAverage[throttleTen]+axisPID_I[PITCH]) / 2);
  rollPitchYawMix[i] =
      ( ( (currentStabilizerThrottle[i] * (axisPID_P[PITCH] + axisPID_D[PITCH]) / 100 ) + axisPID_I[PITCH]) ) * currentMixer[i].pitch +
   ( ( (currentStabilizerThrottle[i] * (axisPID_P[ROLL] + axisPID_D[ROLL]) / 100 ) + axisPID_I[ROLL]) ) * currentMixer[i].roll +
      -mixerConfig->yaw_motor_direction * (axisPID_P[YAW] + axisPID_D[YAW] + axisPID_I[YAW] ) * currentMixer[i].yaw;
  if (rollPitchYawMix[i] > rollPitchYawMixMax) rollPitchYawMixMax = rollPitchYawMix[i];
  if (rollPitchYawMix[i] < rollPitchYawMixMin) rollPitchYawMixMin = rollPitchYawMix[i];
 }
 int16_t rollPitchYawMixRange = rollPitchYawMixMax - rollPitchYawMixMin;
 int16_t throttleRange, throttle;
 int16_t throttleMin, throttleMax;
 static int16_t throttlePrevious = 0;
 if (feature(FEATURE_3D)) {
  if (!ARMING_FLAG(ARMED)) throttlePrevious = rxConfig->midrc;
  if ((rcCommandUsed[THROTTLE] <= (rxConfig->midrc - flight3DConfig->deadband3d_throttle))) {
   throttleMax = flight3DConfig->deadband3d_low;
   throttleMin = escAndServoConfig->minthrottle;
   throttlePrevious = throttle = rcCommandUsed[THROTTLE];
  } else if (rcCommandUsed[THROTTLE] >= (rxConfig->midrc + flight3DConfig->deadband3d_throttle)) {
   escAndServoConfig->maxthrottle = 2000;
   throttleMax = escAndServoConfig->maxthrottle;
   throttleMin = flight3DConfig->deadband3d_high;
   throttlePrevious = throttle = rcCommandUsed[THROTTLE];
  } else if ((throttlePrevious <= (rxConfig->midrc - flight3DConfig->deadband3d_throttle))) {
   throttle = throttleMax = flight3DConfig->deadband3d_low;
   throttleMin = escAndServoConfig->minthrottle;
  } else {
   escAndServoConfig->maxthrottle = 2000;
   throttleMax = escAndServoConfig->maxthrottle;
   throttle = throttleMin = flight3DConfig->deadband3d_high;
  }
 } else {
  throttle = rcCommandUsed[THROTTLE];
  throttleMin = escAndServoConfig->minthrottle;
  escAndServoConfig->maxthrottle = 2000;
  throttleMax = escAndServoConfig->maxthrottle;
 }
 throttleRange = throttleMax - throttleMin;
 if (rollPitchYawMixRange > throttleRange) {
  motorLimitReached = true;
  mixReduction = (throttleRange << 12) / rollPitchYawMixRange;
  for (i = 0; i < motorCount; i++) {
   rollPitchYawMix[i] = ((mixReduction * rollPitchYawMix[i]) >> 12);
  }
  throttleMin = throttleMax = throttleMin + (throttleRange / 2);
 } else {
  motorLimitReached = false;
  throttleMin = throttleMin + (rollPitchYawMixRange / 2);
  throttleMax = throttleMax - (rollPitchYawMixRange / 2);
 }
 for (i = 0; i < motorCount; i++) {
  uint16_t currentMixerThrottle = throttle;
  if (mixerFixer == 1) {
   if ( (i == 1) || (i == 3) ) {
    OldMax = escAndServoConfig->maxthrottle;
    OldMin = escAndServoConfig->minthrottle;
    NewMax = (uint16_t)((float)escAndServoConfig->maxthrottle * mixerConfig->foreAftMixerFixerStrength);
    NewMin = escAndServoConfig->minthrottle;
    OldValue = throttle;
    OldRange = (OldMax - OldMin);
    NewRange = (NewMax - NewMin);
    currentMixerThrottle = (((OldValue - OldMin) * NewRange) / OldRange) + NewMin;
   }
  } else if (mixerFixer == 2) {
   if ( (i == 0) || (i == 1) ) {
    OldMax = escAndServoConfig->maxthrottle;
    OldMin = escAndServoConfig->minthrottle;
    NewMax = (uint16_t)((float)escAndServoConfig->maxthrottle * mixerConfig->foreAftMixerFixerStrength);
    NewMin = escAndServoConfig->minthrottle;
    OldValue = throttle;
    OldRange = (OldMax - OldMin);
    NewRange = (NewMax - NewMin);
    currentMixerThrottle = (((OldValue - OldMin) * NewRange) / OldRange) + NewMin;
   }
  }
  motor[i] = rollPitchYawMix[i] + constrain(currentMixerThrottle * currentMixer[i].throttle, throttleMin, throttleMax);
  if (isFailsafeActive) {
   escAndServoConfig->maxthrottle = 2000;
   motor[i] = constrain(motor[i], escAndServoConfig->mincommand, escAndServoConfig->maxthrottle);
  } else if (feature(FEATURE_3D)) {
   if (throttlePrevious <= (rxConfig->midrc - flight3DConfig->deadband3d_throttle)) {
    motor[i] = constrain(motor[i], escAndServoConfig->minthrottle, flight3DConfig->deadband3d_low);
   } else {
    escAndServoConfig->maxthrottle = 2000;
    motor[i] = constrain(motor[i], flight3DConfig->deadband3d_high, escAndServoConfig->maxthrottle);
   }
  } else {
   escAndServoConfig->maxthrottle = 2000;
   motor[i] = constrain(motor[i], escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle);
  }
  if (feature(FEATURE_MOTOR_STOP) && ARMING_FLAG(ARMED) && !feature(FEATURE_3D)) {
   if (((rcData[THROTTLE]) < rxConfig->mincheck)) {
    motor[i] = escAndServoConfig->realmincommand;
   }
  }
 }
 if (!ARMING_FLAG(ARMED)) {
  for (i = 0; i < motorCount; i++) {
   motor[i] = motor_disarmed[i];
  }
 }
#if !defined(USE_QUAD_MIXER_ONLY) || defined(USE_SERVOS)
    switch (currentMixerMode) {
        case MIXER_CUSTOM_AIRPLANE:
        case MIXER_FLYING_WING:
        case MIXER_AIRPLANE:
        case MIXER_BICOPTER:
        case MIXER_CUSTOM_TRI:
        case MIXER_TRI:
        case MIXER_DUALCOPTER:
        case MIXER_SINGLECOPTER:
        case MIXER_GIMBAL:
            servoMixer();
            break;
        default:
            break;
    }
    if (feature(FEATURE_SERVO_TILT)) {
        servo[SERVO_GIMBAL_PITCH] = determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_PITCH);
        servo[SERVO_GIMBAL_ROLL] = determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_ROLL);
        if (0) {
            if (gimbalConfig->mode == GIMBAL_MODE_MIXTILT) {
                servo[SERVO_GIMBAL_PITCH] -= (-(int32_t)servoConf[SERVO_GIMBAL_PITCH].rate) * attitude.values.pitch / 50 - (int32_t)servoConf[SERVO_GIMBAL_ROLL].rate * attitude.values.roll / 50;
                servo[SERVO_GIMBAL_ROLL] += (-(int32_t)servoConf[SERVO_GIMBAL_PITCH].rate) * attitude.values.pitch / 50 + (int32_t)servoConf[SERVO_GIMBAL_ROLL].rate * attitude.values.roll / 50;
            } else {
                servo[SERVO_GIMBAL_PITCH] += (int32_t)servoConf[SERVO_GIMBAL_PITCH].rate * attitude.values.pitch / 50;
                servo[SERVO_GIMBAL_ROLL] += (int32_t)servoConf[SERVO_GIMBAL_ROLL].rate * attitude.values.roll / 50;
            }
        }
    }
    for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = constrain(servo[i], servoConf[i].min, servoConf[i].max);
    }
#endif
}
#ifdef USE_SERVOS
bool isMixerUsingServos(void)
{
    return useServo;
}
#endif
void filterServos(void)
{
#ifdef USE_SERVOS
    int16_t servoIdx;
#if defined(MIXER_DEBUG)
    uint32_t startTime = micros();
#endif
    if (mixerConfig->servo_lowpass_enable) {
        for (servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
            servo[servoIdx] = (int16_t)lowpassFixed(&lowpassFilters[servoIdx], servo[servoIdx], mixerConfig->servo_lowpass_freq);
            servo[servoIdx] = constrain(servo[servoIdx], servoConf[servoIdx].min, servoConf[servoIdx].max);
        }
    }
#if defined(MIXER_DEBUG)
    debug[0] = (int16_t)(micros() - startTime);
#endif
#endif
}
