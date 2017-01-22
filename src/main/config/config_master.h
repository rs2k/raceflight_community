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
       
typedef struct master_t {
    uint8_t version;
    uint16_t size;
    uint8_t magic_be;
    uint8_t mixerMode;
    uint32_t enabledFeatures;
    uint8_t emf_avoidance;
    motorMixer_t customMotorMixer[MAX_SUPPORTED_MOTORS];
#ifdef USE_SERVOS
    servoMixer_t customServoMixer[MAX_SERVO_RULES];
#endif
    escAndServoConfig_t escAndServoConfig;
    flight3DConfig_t flight3DConfig;
    uint16_t motor_pwm_rate;
    uint16_t servo_pwm_rate;
    uint8_t use_fast_pwm;
#ifdef CC3D
    uint8_t use_buzzer_p6;
#endif
    sensorAlignmentConfig_t sensorAlignmentConfig;
    boardAlignment_t boardAlignment;
    int8_t yaw_control_direction;
    uint8_t acc_hardware;
    uint8_t acc_for_fast_looptime;
 uint8_t rf_loop_ctrl;
 uint8_t arm_method;
 uint8_t gyro_sampling;
 uint16_t dcm_kp;
    uint16_t dcm_ki;
 uint8_t fsStg1;
 uint8_t fsStg2;
 uint16_t fsLowThrottle;
 uint8_t fsMode;
 uint8_t fsCondition;
    gyroConfig_t gyroConfig;
    uint8_t mag_hardware;
    uint8_t baro_hardware;
    uint16_t max_angle_inclination;
    flightDynamicsTrims_t accZero;
    flightDynamicsTrims_t magZero;
    batteryConfig_t batteryConfig;
    rxConfig_t rxConfig;
    inputFilteringMode_e inputFilteringMode;
    failsafeConfig_t failsafeConfig;
    uint8_t led_color;
    uint8_t led_mode;
    uint8_t led_count;
    uint8_t retarded_arm;
    uint8_t disarm_kill_switch;
    uint8_t auto_disarm_delay;
    uint8_t small_angle;
    mixerConfig_t mixerConfig;
    airplaneConfig_t airplaneConfig;
#ifdef GPS
    gpsConfig_t gpsConfig;
#endif
    serialConfig_t serialConfig;
    telemetryConfig_t telemetryConfig;
#ifdef LED_STRIP
    ledConfig_t ledConfigs[MAX_LED_STRIP_LENGTH];
    hsvColor_t colors[CONFIGURABLE_COLOR_COUNT];
#endif
    profile_t profile[MAX_PROFILE_COUNT];
    uint8_t current_profile_index;
    controlRateConfig_t controlRateProfiles[MAX_CONTROL_RATE_PROFILE_COUNT];
#ifdef BLACKBOX
    uint8_t blackbox_rate_num;
    uint8_t blackbox_rate_denom;
    uint8_t blackbox_device;
#endif
    beeperOffConditions_t beeper_off;
    uint8_t magic_ef;
    uint8_t chk;
} master_t;
extern master_t masterConfig;
extern profile_t *currentProfile;
extern controlRateConfig_t *currentControlRateProfile;
#define ALWAYS_DOUBLE 0
#define DOUBLE_ONCE 1
#define DOUBLE_ONCE_NC 2
#define ALWAYS_SINGLE 3
#define ALWAYS_SINGLE_NC 4
#define FS_NO_RX_PULSE 0
#define FS_LOW_THROTTLE 1
#define FS_NO_RX_PULSE_AND_LOW_THROTTLE 2
#define STARVING_DOG 3
#define FS_FALL_OUT_OF_SKY 0
