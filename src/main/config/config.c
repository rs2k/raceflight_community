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

#include "platform.h"

#include "build_config.h"

#include "debug.h"

#include "common/color.h"
#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/serial.h"
#include "drivers/gyro_sync.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/gimbal.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"
#include "io/ledstrip.h"
#include "io/gps.h"

#include "rx/rx.h"

#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/altitudehold.h"
#include "flight/navigation.h"

#include "config/runtime_config.h"
#include "config/config.h"

#include "config/config_profile.h"
#include "config/config_master.h"

#define BRUSHED_MOTORS_PWM_RATE 16000
#define BRUSHLESS_MOTORS_PWM_RATE 32000

void useRcControlsConfig(modeActivationCondition_t *modeActivationConditions, escAndServoConfig_t *escAndServoConfigToUse, pidProfile_t *pidProfileToUse);

#if !defined(FLASH_SIZE)
#error "Flash size not defined for target. (specify in KB)"
#endif


#ifndef FLASH_PAGE_SIZE
    #ifdef STM32F303xC
        #define FLASH_PAGE_SIZE                 ((uint16_t)0x800)
    #endif

    #ifdef STM32F10X_MD
        #define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
    #endif

    #ifdef STM32F10X_HD
        #define FLASH_PAGE_SIZE                 ((uint16_t)0x800)
    #endif

    #if defined(STM32F40_41xxx)
        #define FLASH_PAGE_SIZE                 ((uint32_t)0x20000)
    #endif

    #if defined (STM32F411xE)
        #define FLASH_PAGE_SIZE                 ((uint32_t)0x20000)
    #endif

    #if defined (STM32F446xx)
        #define FLASH_PAGE_SIZE                 ((uint32_t)0x20000)
    #endif

#endif

#if !defined(FLASH_SIZE) && !defined(FLASH_PAGE_COUNT)
    #ifdef STM32F10X_MD
        #define FLASH_PAGE_COUNT 128
    #endif

    #ifdef STM32F10X_HD
        #define FLASH_PAGE_COUNT 128
    #endif
#endif

#if defined(FLASH_SIZE)
#if defined(STM32F40_41xxx)
    #define FLASH_PAGE_COUNT 4 
#elif defined (STM32F411xE)
    #define FLASH_PAGE_COUNT 4 
#elif defined (STM32F446xx)
    #define FLASH_PAGE_COUNT 4 
#else
    #define FLASH_PAGE_COUNT ((FLASH_SIZE * 0x400) / FLASH_PAGE_SIZE)
#endif
#endif

#if !defined(FLASH_PAGE_SIZE)
#error "Flash page size not defined for target."
#endif

#if !defined(FLASH_PAGE_COUNT)
#error "Flash page count not defined for target."
#endif

#if FLASH_SIZE <= 128
    #define FLASH_TO_RESERVE_FOR_CONFIG 0x800
#else
    #define FLASH_TO_RESERVE_FOR_CONFIG 0x1000
#endif

#if !defined(CONFIG_START_FLASH_ADDRESS)
    
#define CONFIG_START_FLASH_ADDRESS (0x08000000 + (uint32_t)((FLASH_PAGE_SIZE * FLASH_PAGE_COUNT) - FLASH_TO_RESERVE_FOR_CONFIG))
#define CONFIG_START_BACK_ADDRESS (0x08000000 + (uint32_t)((FLASH_PAGE_SIZE * FLASH_PAGE_COUNT) - FLASH_TO_RESERVE_FOR_CONFIG))
#endif

master_t masterConfig;                 
profile_t *currentProfile;
static uint32_t activeFeaturesLatch = 0;

static uint8_t currentControlRateProfileIndex = 0;
controlRateConfig_t *currentControlRateProfile;

static const uint8_t EEPROM_CONF_VERSION = 111; 

static void resetAccelerometerTrims(flightDynamicsTrims_t *accelerometerTrims)
{
    accelerometerTrims->values.pitch = 0;
    accelerometerTrims->values.roll = 0;
    accelerometerTrims->values.yaw = 0;
}

static void resetPidProfile(pidProfile_t *pidProfile)
{

    pidProfile->pidController = 1;

    pidProfile->P8[ROLL] = 55;
    pidProfile->I8[ROLL] = 80;
    pidProfile->D8[ROLL] = 18;
    pidProfile->P8[PITCH] = 65;
    pidProfile->I8[PITCH] = 100;
    pidProfile->D8[PITCH] = 22;
    pidProfile->P8[YAW] = 110;
    pidProfile->I8[YAW] = 180;
    pidProfile->D8[YAW] = 0;
    pidProfile->P8[PIDALT] = 50;
    pidProfile->I8[PIDALT] = 0;
    pidProfile->D8[PIDALT] = 0;
    pidProfile->P8[PIDPOS] = 15; 
    pidProfile->I8[PIDPOS] = 0; 
    pidProfile->D8[PIDPOS] = 0;
    pidProfile->P8[PIDPOSR] = 34; 
    pidProfile->I8[PIDPOSR] = 14; 
    pidProfile->D8[PIDPOSR] = 53; 
    pidProfile->P8[PIDNAVR] = 25; 
    pidProfile->I8[PIDNAVR] = 33; 
    pidProfile->D8[PIDNAVR] = 83; 
    pidProfile->P8[PIDLEVEL] = 55;
    pidProfile->I8[PIDLEVEL] = 55;
    pidProfile->D8[PIDLEVEL] = 100;
    pidProfile->P8[PIDMAG] = 40;
    pidProfile->P8[PIDVEL] = 120;
    pidProfile->I8[PIDVEL] = 45;
    pidProfile->D8[PIDVEL] = 1;

    pidProfile->wrgyrolpf = 70;    
    pidProfile->wpgyrolpf = 70;    
    pidProfile->wygyrolpf = 65;    
    pidProfile->wrkdlpf = 65;      
    pidProfile->wpkdlpf = 65;      
    pidProfile->wykdlpf = 60;      

    pidProfile->fcquick = 0.100;   
    pidProfile->fcrap = 250.0;   
    pidProfile->fcpress = 0.150;   

	pidProfile->yaw_pterm_cut_hz = 30;
	pidProfile->pitch_pterm_cut_hz = 0;
	pidProfile->witchcraft = 32;
	pidProfile->P_f[ROLL] = 4.612f;     
    pidProfile->I_f[ROLL] = 1.110f;
    pidProfile->D_f[ROLL] = 0.100f;
    pidProfile->P_f[PITCH] = 5.911f;
    pidProfile->I_f[PITCH] = 1.451f;
    pidProfile->D_f[PITCH] = 0.120f;
    pidProfile->P_f[YAW] = 8.510f;
    pidProfile->I_f[YAW] = 1.450f;
    pidProfile->D_f[YAW] = 0.020f;
    pidProfile->A_level = 3.000f;
    pidProfile->H_level = 3.000f;
    pidProfile->H_sensitivity = 100;

#ifdef GTUNE
    pidProfile->gtune_lolimP[ROLL] = 10;          
    pidProfile->gtune_lolimP[PITCH] = 10;         
    pidProfile->gtune_lolimP[YAW] = 10;           
    pidProfile->gtune_hilimP[ROLL] = 100;         
    pidProfile->gtune_hilimP[PITCH] = 100;        
    pidProfile->gtune_hilimP[YAW] = 100;          
    pidProfile->gtune_pwr = 0;                    
    pidProfile->gtune_settle_time = 450;          
    pidProfile->gtune_average_cycles = 16;        
#endif
}

#ifdef GPS
void resetGpsProfile(gpsProfile_t *gpsProfile)
{
    gpsProfile->gps_wp_radius = 200;
    gpsProfile->gps_lpf = 20;
    gpsProfile->nav_slew_rate = 30;
    gpsProfile->nav_controls_heading = 1;
    gpsProfile->nav_speed_min = 100;
    gpsProfile->nav_speed_max = 300;
    gpsProfile->ap_mode = 40;
}
#endif

void resetBarometerConfig(barometerConfig_t *barometerConfig)
{
    barometerConfig->baro_sample_count = 21;
    barometerConfig->baro_noise_lpf = 0.6f;
    barometerConfig->baro_cf_vel = 0.985f;
    barometerConfig->baro_cf_alt = 0.965f;
}

void resetSensorAlignment(sensorAlignmentConfig_t *sensorAlignmentConfig)
{
    sensorAlignmentConfig->gyro_align = ALIGN_DEFAULT;
    sensorAlignmentConfig->acc_align = ALIGN_DEFAULT;
    sensorAlignmentConfig->mag_align = ALIGN_DEFAULT;
}

void resetEscAndServoConfig(escAndServoConfig_t *escAndServoConfig)
{
    escAndServoConfig->minthrottle = 1065;
    escAndServoConfig->maxthrottle = 2000;
    escAndServoConfig->mincommand = 1000; 
    escAndServoConfig->realmincommand = 990; 
    escAndServoConfig->servoCenterPulse = 1500;
}

void resetFlight3DConfig(flight3DConfig_t *flight3DConfig)
{
    flight3DConfig->deadband3d_low = 1406;
    flight3DConfig->deadband3d_high = 1514;
    flight3DConfig->neutral3d = 1460;
    flight3DConfig->deadband3d_throttle = 50;
}

void resetTelemetryConfig(telemetryConfig_t *telemetryConfig)
{
    telemetryConfig->telemetry_inversion = 0;
    telemetryConfig->telemetry_switch = 0;
    telemetryConfig->gpsNoFixLatitude = 0;
    telemetryConfig->gpsNoFixLongitude = 0;
    telemetryConfig->frsky_coordinate_format = FRSKY_FORMAT_DMS;
    telemetryConfig->frsky_unit = FRSKY_UNIT_METRICS;
    telemetryConfig->frsky_vfas_precision = 0;
    telemetryConfig->hottAlarmSoundInterval = 5;
}

void resetBatteryConfig(batteryConfig_t *batteryConfig)
{
    batteryConfig->vbatscale = VBAT_SCALE_DEFAULT;
    batteryConfig->vbatdivider = VBAT_DIVIDER_DEFAULT;
    batteryConfig->vbatmaxcellvoltage = 43;
    batteryConfig->vbatwarningcellvoltage = 35;
    batteryConfig->currentMeterOffset = 0;
    batteryConfig->currentMeterScale = 400; 
    batteryConfig->batteryCapacity = 0;
    batteryConfig->currentMeterType = CURRENT_SENSOR_ADC;
}

#ifdef SWAP_SERIAL_PORT_0_AND_1_DEFAULTS
#define FIRST_PORT_INDEX 1
#define SECOND_PORT_INDEX 0
#else
#define FIRST_PORT_INDEX 0
#define SECOND_PORT_INDEX 1
#endif

void resetSerialConfig(serialConfig_t *serialConfig)
{
    uint8_t index;
    memset(serialConfig, 0, sizeof(serialConfig_t));

    for (index = 0; index < SERIAL_PORT_COUNT; index++) {
        serialConfig->portConfigs[index].identifier = serialPortIdentifiers[index];
        serialConfig->portConfigs[index].msp_baudrateIndex = BAUD_115200;
        serialConfig->portConfigs[index].gps_baudrateIndex = BAUD_57600;
        serialConfig->portConfigs[index].telemetry_baudrateIndex = BAUD_AUTO;
        serialConfig->portConfigs[index].blackbox_baudrateIndex = BAUD_115200;
    }

    serialConfig->portConfigs[0].functionMask = FUNCTION_MSP;

#ifdef CC3D
    
    serialConfig->portConfigs[1].functionMask = FUNCTION_MSP;
#endif

    serialConfig->reboot_character = 'R';
}

static void resetControlRateConfig(controlRateConfig_t *controlRateConfig) {
    controlRateConfig->rcPitchExpo8 = 40.0f;
    controlRateConfig->rcRollExpo8 = 40.0f;
    controlRateConfig->rcYawExpo8 = 40.0f;
    controlRateConfig->thrMid8 = 50;
    controlRateConfig->thrExpo8 = 0.0f;
    controlRateConfig->dynThrPID = 0;
    controlRateConfig->tpa_breakpoint = 1500;

    controlRateConfig->PitchAcroPlusFactor = 40.0f;
    controlRateConfig->RollAcroPlusFactor = 40.0f;
    controlRateConfig->YawAcroPlusFactor = 20.0f;

    for (uint8_t axis = 0; axis < FLIGHT_DYNAMICS_INDEX_COUNT; axis++) {
        if (axis == 2) {
            controlRateConfig->rates[axis] = 400.0f;
        } else {
            controlRateConfig->rates[axis] = 400.0f;
        }
    }

}

void resetRcControlsConfig(rcControlsConfig_t *rcControlsConfig) {
    rcControlsConfig->deadband = 0;
    rcControlsConfig->yaw_deadband = 0;
    rcControlsConfig->alt_hold_deadband = 40;
    rcControlsConfig->alt_hold_fast_change = 1;
}

void resetMixerConfig(mixerConfig_t *mixerConfig) {

    mixerConfig->foreAftMixerFixerStrength = .95;
    mixerConfig->yaw_motor_direction = 1;
    mixerConfig->yaw_jump_prevention_limit = 200;
#ifdef USE_SERVOS
    mixerConfig->tri_unarmed_servo = 1;
    mixerConfig->servo_lowpass_freq = 400;
    mixerConfig->servo_lowpass_enable = 0;
#endif
}

uint8_t getCurrentProfile(void)
{
    return masterConfig.current_profile_index;
}

static void setProfile(uint8_t profileIndex)
{
    currentProfile = &masterConfig.profile[profileIndex];
}

uint8_t getCurrentControlRateProfile(void)
{
    return currentControlRateProfileIndex;
}

controlRateConfig_t *getControlRateConfig(uint8_t profileIndex) {
    return &masterConfig.controlRateProfiles[profileIndex];
}

static void setControlRateProfile(uint8_t profileIndex)
{
    currentControlRateProfileIndex = profileIndex;
    currentControlRateProfile = &masterConfig.controlRateProfiles[profileIndex];
}

uint16_t getCurrentMinthrottle(void)
{
    return masterConfig.escAndServoConfig.minthrottle;
}


static void resetConf(void)
{
    int i;

    
    memset(&masterConfig, 0, sizeof(master_t));

    setProfile(0);
    setControlRateProfile(0);

    masterConfig.beeper_off.flags = BEEPER_OFF_FLAGS_MIN;
    masterConfig.version = EEPROM_CONF_VERSION;
    masterConfig.mixerMode = MIXER_QUADXL;
    featureClearAll();

#ifdef BOARD_HAS_VOLTAGE_DIVIDER
    
    
    featureSet(FEATURE_VBAT);
#endif

    featureSet(FEATURE_FAILSAFE);
    featureSet(FEATURE_MULTISHOT);
    featureSet(FEATURE_USE_PWM_RATE);
    featureSet(FEATURE_SBUS_INVERTER);

    
    masterConfig.current_profile_index = 0;     
    masterConfig.dcm_kp = 2500;                
    masterConfig.dcm_ki = 0;                    

    resetAccelerometerTrims(&masterConfig.accZero);

    resetSensorAlignment(&masterConfig.sensorAlignmentConfig);

    masterConfig.boardAlignment.rollDegrees = 0;
    masterConfig.boardAlignment.pitchDegrees = 0;
    masterConfig.boardAlignment.yawDegrees = 0;
    masterConfig.acc_hardware = 1;     
    masterConfig.max_angle_inclination = 700;    
    masterConfig.yaw_control_direction = 1;
    masterConfig.gyroConfig.gyroMovementCalibrationThreshold = 16;

    
    masterConfig.mag_hardware = 1;

    masterConfig.baro_hardware = 1;

    resetBatteryConfig(&masterConfig.batteryConfig);

    resetTelemetryConfig(&masterConfig.telemetryConfig);

#ifdef CONFIG_SERIALRX_PROVIDER
	masterConfig.rxConfig.serialrx_provider = CONFIG_SERIALRX_PROVIDER;
#else
    masterConfig.rxConfig.serialrx_provider = 1;
#endif


#if defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F446xx)
	masterConfig.rxConfig.max_aux_channels = 12;
	masterConfig.rf_loop_ctrl = DLPF_H8;
	masterConfig.arm_method = ALWAYS_DOUBLE;
	masterConfig.acc_hardware = 0;
#elif defined(STM32F303xC)
    masterConfig.rxConfig.max_aux_channels = 4;
	masterConfig.rf_loop_ctrl = DLPF_H4;
	masterConfig.acc_hardware = 1;
#else
    masterConfig.rxConfig.max_aux_channels = 4;
	masterConfig.rf_loop_ctrl = DLPF_H2;
	masterConfig.acc_hardware = 0;
#endif

    masterConfig.rxConfig.spektrum_sat_bind = 9;
    masterConfig.rxConfig.midrc = 1500;
    masterConfig.rxConfig.mincheck = 1005; 
    masterConfig.rxConfig.maxcheck = 1995;
    masterConfig.rxConfig.rx_min_usec = 885;          
    masterConfig.rxConfig.rx_max_usec = 2115;         

    for (i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rxFailsafeChannelConfiguration_t *channelFailsafeConfiguration = &masterConfig.rxConfig.failsafe_channel_configurations[i];
        channelFailsafeConfiguration->mode = (i < NON_AUX_CHANNEL_COUNT) ? RX_FAILSAFE_MODE_AUTO : RX_FAILSAFE_MODE_HOLD;
        channelFailsafeConfiguration->step = (i == THROTTLE) ? CHANNEL_VALUE_TO_RXFAIL_STEP(masterConfig.rxConfig.rx_min_usec) : CHANNEL_VALUE_TO_RXFAIL_STEP(masterConfig.rxConfig.midrc);
    }

    masterConfig.rxConfig.rssi_channel = 0;
    masterConfig.rxConfig.rssi_scale = RSSI_SCALE_DEFAULT;
    masterConfig.rxConfig.rssi_ppm_invert = 0;
    masterConfig.rxConfig.rcSmoothing = 1;
    masterConfig.rxConfig.fpvCamAngleDegrees = 0;

    resetAllRxChannelRangeConfigurations(masterConfig.rxConfig.channelRanges);

    masterConfig.inputFilteringMode = INPUT_FILTERING_DISABLED;

    masterConfig.retarded_arm = 0;  
    masterConfig.disarm_kill_switch = 1;
    masterConfig.auto_disarm_delay = 5;
    masterConfig.small_angle = 40;

    resetMixerConfig(&masterConfig.mixerConfig);

    masterConfig.airplaneConfig.fixedwing_althold_dir = 1;

    
    resetEscAndServoConfig(&masterConfig.escAndServoConfig);
    resetFlight3DConfig(&masterConfig.flight3DConfig);

#ifdef BRUSHED_MOTORS
    masterConfig.motor_pwm_rate = BRUSHED_MOTORS_PWM_RATE;
#else
    masterConfig.motor_pwm_rate = BRUSHLESS_MOTORS_PWM_RATE;
#endif
    masterConfig.servo_pwm_rate = 50;
    masterConfig.use_fast_pwm = 0;
#ifdef CC3D
    masterConfig.use_buzzer_p6 = 0;
#endif

#ifdef GPS
    
    masterConfig.gpsConfig.provider = GPS_NMEA;
    masterConfig.gpsConfig.sbasMode = SBAS_AUTO;
    masterConfig.gpsConfig.autoConfig = GPS_AUTOCONFIG_ON;
    masterConfig.gpsConfig.autoBaud = GPS_AUTOBAUD_OFF;
#endif

    resetSerialConfig(&masterConfig.serialConfig);

    masterConfig.emf_avoidance = 0;

    resetPidProfile(&currentProfile->pidProfile);

    resetControlRateConfig(&masterConfig.controlRateProfiles[0]);

    
    

    resetRollAndPitchTrims(&currentProfile->accelerometerTrims);

    currentProfile->mag_declination = 0;
    currentProfile->acc_lpf_hz = 20;
    currentProfile->accz_lpf_cutoff = 5.0f;
    currentProfile->accDeadband.xy = 40;
    currentProfile->accDeadband.z = 40;
    currentProfile->acc_unarmedcal = 1;

    resetBarometerConfig(&currentProfile->barometerConfig);

    
	parseRcChannels("TAER1234", &masterConfig.rxConfig);

    resetRcControlsConfig(&currentProfile->rcControlsConfig);

    currentProfile->throttle_correction_value = 0;      
    currentProfile->throttle_correction_angle = 800;    

    
    masterConfig.failsafeConfig.failsafe_delay = 10;              
    masterConfig.failsafeConfig.failsafe_off_delay = 10;          
    masterConfig.failsafeConfig.failsafe_throttle = 1000;         
    masterConfig.failsafeConfig.failsafe_kill_switch = 0;         
    masterConfig.failsafeConfig.failsafe_throttle_low_delay = 100; 

    
    masterConfig.led_color = 0;
    masterConfig.led_mode = 0;
    masterConfig.led_count = 12;

#ifdef USE_SERVOS
    
    for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        currentProfile->servoConf[i].min = DEFAULT_SERVO_MIN;
        currentProfile->servoConf[i].max = DEFAULT_SERVO_MAX;
        currentProfile->servoConf[i].middle = DEFAULT_SERVO_MIDDLE;
        currentProfile->servoConf[i].rate = 100;
        currentProfile->servoConf[i].angleAtMin = DEFAULT_SERVO_MIN_ANGLE;
        currentProfile->servoConf[i].angleAtMax = DEFAULT_SERVO_MAX_ANGLE;
        currentProfile->servoConf[i].forwardFromChannel = CHANNEL_FORWARDING_DISABLED;
    }

    
    currentProfile->gimbalConfig.mode = GIMBAL_MODE_NORMAL;
#endif

#ifdef GPS
    resetGpsProfile(&currentProfile->gpsProfile);
#endif

    
    for (i = 0; i < MAX_SUPPORTED_MOTORS; i++)
        masterConfig.customMotorMixer[i].throttle = 0.0f;

#ifdef LED_STRIP
    applyDefaultColors(masterConfig.colors, CONFIGURABLE_COLOR_COUNT);
    applyDefaultLedStripConfig(masterConfig.ledConfigs);
#endif

#ifdef BLACKBOX
	featureSet(FEATURE_BLACKBOX);
#ifdef CONFIG_BLACKBOX_DEVICE
	masterConfig.blackbox_device = CONFIG_BLACKBOX_DEVICE;
#else
    masterConfig.blackbox_device = 0;
#endif
    masterConfig.blackbox_rate_num = 1;
    masterConfig.blackbox_rate_denom = 1;
#endif


#ifdef CONFIG_FEATURE_RX_SERIAL
    featureSet(FEATURE_RX_SERIAL);
#endif
#ifdef CONFIG_FEATURE_ONESHOT125
    featureSet(FEATURE_MULTISHOT);
#endif
#ifdef CONFIG_MSP_PORT
    masterConfig.serialConfig.portConfigs[CONFIG_MSP_PORT].functionMask = FUNCTION_MSP;
    masterConfig.serialConfig.portConfigs[CONFIG_MSP_PORT].msp_baudrateIndex = BAUD_9600;
#endif
#ifdef CONFIG_RX_SERIAL_PORT
    masterConfig.serialConfig.portConfigs[CONFIG_RX_SERIAL_PORT].functionMask = FUNCTION_RX_SERIAL;
#endif

    
    for (i = 1; i < MAX_PROFILE_COUNT; i++) {
        memcpy(&masterConfig.profile[i], currentProfile, sizeof(profile_t));
    }

    
    for (i = 1; i < MAX_CONTROL_RATE_PROFILE_COUNT; i++) {
        memcpy(&masterConfig.controlRateProfiles[i], currentControlRateProfile, sizeof(controlRateConfig_t));
    }

    for (i = 1; i < MAX_PROFILE_COUNT; i++) {
        masterConfig.profile[i].defaultRateProfileIndex = i % MAX_CONTROL_RATE_PROFILE_COUNT;
    }
}

static uint8_t calculateChecksum(const uint8_t *data, uint32_t length)
{
    uint8_t checksum = 0;
    const uint8_t *byteOffset;

    for (byteOffset = data; byteOffset < (data + length); byteOffset++)
        checksum ^= *byteOffset;
    return checksum;
}

static bool isEEPROMContentValid(bool checkBackup)
{
	uint32_t config_address_to_use = CONFIG_START_FLASH_ADDRESS;
	if (checkBackup) {
		config_address_to_use = CONFIG_START_BACK_ADDRESS;
	}
	const master_t *temp = (const master_t *) config_address_to_use;

    uint8_t checksum = 0;
    
    if (EEPROM_CONF_VERSION != temp->version)
        return false;

    
    if (temp->size != sizeof(master_t) || temp->magic_be != 0xBE || temp->magic_ef != 0xEF)
        return false;

    
    checksum = calculateChecksum((const uint8_t *) temp, sizeof(master_t));
    if (checksum != 0)
        return false;

    
    if (checkBackup) {
    	
    }
    return true;
}

void activateControlRateConfig(void)
{
    generatePitchCurve(currentControlRateProfile);
    generateRollCurve(currentControlRateProfile);
    generateYawCurve(currentControlRateProfile);
    generateThrottleCurve(currentControlRateProfile, &masterConfig.escAndServoConfig);
}

void activateConfig(void)
{
    static imuRuntimeConfig_t imuRuntimeConfig;

    activateControlRateConfig();

    resetAdjustmentStates();

    useRcControlsConfig(
        currentProfile->modeActivationConditions,
        &masterConfig.escAndServoConfig,
        &currentProfile->pidProfile
    );

    useGyroConfig(&masterConfig.gyroConfig, 7);

#ifdef TELEMETRY
    telemetryUseConfig(&masterConfig.telemetryConfig);
#endif
    currentProfile->pidProfile.pidController = constrain(currentProfile->pidProfile.pidController, 1, 2); 
    pidSetController(currentProfile->pidProfile.pidController);

#ifdef GPS
    gpsUseProfile(&currentProfile->gpsProfile);
    gpsUsePIDs(&currentProfile->pidProfile);
#endif

    useFailsafeConfig(&masterConfig.failsafeConfig);
    setAccelerationTrims(&masterConfig.accZero);

    mixerUseConfigs(
#ifdef USE_SERVOS
        currentProfile->servoConf,
        &currentProfile->gimbalConfig,
#endif
        &masterConfig.flight3DConfig,
        &masterConfig.escAndServoConfig,
        &masterConfig.mixerConfig,
        &masterConfig.airplaneConfig,
        &masterConfig.rxConfig
    );

    imuRuntimeConfig.dcm_kp = masterConfig.dcm_kp / 10000.0f;
    imuRuntimeConfig.dcm_ki = masterConfig.dcm_ki / 10000.0f;
    imuRuntimeConfig.acc_cut_hz = currentProfile->acc_lpf_hz;
    imuRuntimeConfig.acc_unarmedcal = currentProfile->acc_unarmedcal;
    imuRuntimeConfig.small_angle = masterConfig.small_angle;

    imuConfigure(
        &imuRuntimeConfig,
        &currentProfile->pidProfile,
        &currentProfile->accDeadband,
        currentProfile->accz_lpf_cutoff,
        currentProfile->throttle_correction_angle
    );

    configureAltitudeHold(
        &currentProfile->pidProfile,
        &currentProfile->barometerConfig,
        &currentProfile->rcControlsConfig,
        &masterConfig.escAndServoConfig
    );

#ifdef BARO
    useBarometerConfig(&currentProfile->barometerConfig);
#endif
}

void validateAndFixConfig(void)
{
    if (!(featureConfigured(FEATURE_RX_PARALLEL_PWM) || featureConfigured(FEATURE_RX_PPM) || featureConfigured(FEATURE_RX_SERIAL) || featureConfigured(FEATURE_RX_MSP))) {
        featureSet(FEATURE_RX_PARALLEL_PWM); 
    }

    if (featureConfigured(FEATURE_RX_PPM)) {
        featureClear(FEATURE_RX_PARALLEL_PWM);
    }

    if (featureConfigured(FEATURE_RX_MSP)) {
        featureClear(FEATURE_RX_SERIAL);
        featureClear(FEATURE_RX_PARALLEL_PWM);
        featureClear(FEATURE_RX_PPM);
    }

    if (featureConfigured(FEATURE_RX_SERIAL)) {
        featureClear(FEATURE_RX_PARALLEL_PWM);
        featureClear(FEATURE_RX_PPM);
    }

    if (featureConfigured(FEATURE_RX_PARALLEL_PWM)) {
#if defined(STM32F10X)
        
        featureClear(FEATURE_RSSI_ADC);
        
        if (masterConfig.batteryConfig.currentMeterType == CURRENT_SENSOR_ADC) {
            featureClear(FEATURE_CURRENT_METER);
        }
#endif

#if defined(STM32F10X) || defined(CHEBUZZ) || defined(STM32F3DISCOVERY)
        
        featureClear(FEATURE_LED_STRIP);
#endif

        
        featureClear(FEATURE_SOFTSERIAL);
    }


#if defined(LED_STRIP) && (defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2))
    if (featureConfigured(FEATURE_SOFTSERIAL) && (
            0
#ifdef USE_SOFTSERIAL1
            || (LED_STRIP_TIMER == SOFTSERIAL_1_TIMER)
#endif
#ifdef USE_SOFTSERIAL2
            || (LED_STRIP_TIMER == SOFTSERIAL_2_TIMER)
#endif
    )) {
        
        featureClear(FEATURE_LED_STRIP);
    }
#endif

#if defined(NAZE) && defined(SONAR)
    if (featureConfigured(FEATURE_RX_PARALLEL_PWM) && featureConfigured(FEATURE_SONAR) && featureConfigured(FEATURE_CURRENT_METER) && masterConfig.batteryConfig.currentMeterType == CURRENT_SENSOR_ADC) {
        featureClear(FEATURE_CURRENT_METER);
    }
#endif

#if defined(OLIMEXINO) && defined(SONAR)
    if (feature(FEATURE_SONAR) && feature(FEATURE_CURRENT_METER) && masterConfig.batteryConfig.currentMeterType == CURRENT_SENSOR_ADC) {
        featureClear(FEATURE_CURRENT_METER);
    }
#endif

#if defined(CC3D) && defined(DISPLAY) && defined(USE_USART3)
    if (doesConfigurationUsePort(SERIAL_PORT_USART3) && feature(FEATURE_DISPLAY)) {
        featureClear(FEATURE_DISPLAY);
    }
#endif

#ifdef STM32F303xC
    
    masterConfig.telemetryConfig.telemetry_inversion = 1;
#endif

#if defined(CC3D) && defined(SONAR) && defined(USE_SOFTSERIAL1)
    if (feature(FEATURE_SONAR) && feature(FEATURE_SOFTSERIAL)) {
        featureClear(FEATURE_SONAR);
    }
#endif

    useRxConfig(&masterConfig.rxConfig);

    serialConfig_t *serialConfig = &masterConfig.serialConfig;

    if (!isSerialConfigValid(serialConfig)) {
        resetSerialConfig(serialConfig);
    }
}

void initEEPROM(void)
{
}

void readEEPROM(void)
{
	SKIP_GYRO=true;
    
    if (!isEEPROMContentValid(0))
        failureMode(FAILURE_INVALID_EEPROM_CONTENTS);

    suspendRxSignal();

    
    memcpy(&masterConfig, (char *) CONFIG_START_FLASH_ADDRESS, sizeof(master_t));

    if (masterConfig.current_profile_index > MAX_PROFILE_COUNT - 1) 
        masterConfig.current_profile_index = 0;

    setProfile(masterConfig.current_profile_index);

    if (currentProfile->defaultRateProfileIndex > MAX_CONTROL_RATE_PROFILE_COUNT - 1) 
        currentProfile->defaultRateProfileIndex = 0;

    setControlRateProfile(currentProfile->defaultRateProfileIndex);

    validateAndFixConfig();
    activateConfig();

    resumeRxSignal();
    SKIP_GYRO=false;
}

void readEEPROMAndNotify(void)
{
    
    readEEPROM();
    beeperConfirmationBeeps(1);
}

void writeEEPROM(void)
{
	SKIP_GYRO=true;
    
    BUILD_BUG_ON(sizeof(master_t) > FLASH_TO_RESERVE_FOR_CONFIG);

    FLASH_Status status = 0;
    uint32_t wordOffset;
    int8_t attemptsRemaining = 3;

    suspendRxSignal();

    
    masterConfig.version = EEPROM_CONF_VERSION;
    masterConfig.size = sizeof(master_t);
    masterConfig.magic_be = 0xBE;
    masterConfig.magic_ef = 0xEF;
    masterConfig.chk = 0; 
    masterConfig.chk = calculateChecksum((const uint8_t *) &masterConfig, sizeof(master_t));

    
    FLASH_Unlock();
    while (attemptsRemaining--) {
#ifdef STM32F40_41xxx
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
#endif
#ifdef STM32F411xE
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
#endif
#ifdef STM32F446xx
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
#endif
#ifdef STM32F303
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
#endif
#ifdef STM32F10X
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
#endif
        for (wordOffset = 0; wordOffset < sizeof(master_t); wordOffset += 4) {
            if (wordOffset % FLASH_PAGE_SIZE == 0) {
#if defined(STM32F40_41xxx)
                
                status = FLASH_EraseSector(FLASH_Sector_7, VoltageRange_3); 
#elif defined (STM32F411xE)
                status = FLASH_EraseSector(FLASH_Sector_7, VoltageRange_3); 
#elif defined (STM32F446xx)
                status = FLASH_EraseSector(FLASH_Sector_7, VoltageRange_3); 
#else
                status = FLASH_ErasePage(CONFIG_START_FLASH_ADDRESS + wordOffset);
#endif
                if (status != FLASH_COMPLETE) {
                    break;
                }
            }

            status = FLASH_ProgramWord(CONFIG_START_FLASH_ADDRESS + wordOffset,
                    *(uint32_t *) ((char *) &masterConfig + wordOffset));
            if (status != FLASH_COMPLETE) {
                break;
            }
        }
        if (status == FLASH_COMPLETE) {
            break;
        }
    }
    FLASH_Lock();

    
    if (status != FLASH_COMPLETE || !isEEPROMContentValid(0)) {
        failureMode(FAILURE_FLASH_WRITE_FAILED);
    }

    resumeRxSignal();
    SKIP_GYRO=false;
}

void ensureEEPROMContainsValidData(void)
{

    if (isEEPROMContentValid(0)) {
        return;
    }

    resetEEPROM();
}

void resetEEPROM(void)
{
    resetConf();
    writeEEPROM();

}

void saveConfigAndNotify(void)
{
    writeEEPROM();
    readEEPROMAndNotify();
}

void changeProfile(uint8_t profileIndex)
{
    masterConfig.current_profile_index = profileIndex;
    writeEEPROM();
    readEEPROM();
    beeperConfirmationBeeps(profileIndex + 1);
}

void changeControlRateProfile(uint8_t profileIndex)
{
    if (profileIndex > MAX_CONTROL_RATE_PROFILE_COUNT) {
        profileIndex = MAX_CONTROL_RATE_PROFILE_COUNT - 1;
    }
    setControlRateProfile(profileIndex);
    activateControlRateConfig();
}

void handleOneshotFeatureChangeOnRestart(void)
{
    
    StopPwmAllMotors();
    delay(50);
    
    if (feature(FEATURE_ONESHOT125) && !featureConfigured(FEATURE_ONESHOT125)) {
            delay(ONESHOT_FEATURE_CHANGED_DELAY_ON_BOOT_MS);
    } else if (feature(FEATURE_MULTISHOT) && !featureConfigured(FEATURE_MULTISHOT)) {
            delay(ONESHOT_FEATURE_CHANGED_DELAY_ON_BOOT_MS);
    }
}

void latchActiveFeatures()
{
    activeFeaturesLatch = masterConfig.enabledFeatures;
}

bool featureConfigured(uint32_t mask)
{
    return masterConfig.enabledFeatures & mask;
}

bool feature(uint32_t mask)
{
    return activeFeaturesLatch & mask;
}

void featureSet(uint32_t mask)
{
    masterConfig.enabledFeatures |= mask;
}

void featureClear(uint32_t mask)
{
    masterConfig.enabledFeatures &= ~(mask);
}

void featureClearAll()
{
    masterConfig.enabledFeatures = 0;
}

uint32_t featureMask(void)
{
    return masterConfig.enabledFeatures;
}
