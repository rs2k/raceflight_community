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
#include <string.h>
#include "platform.h"
#include "version.h"
#include "include.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/encoding.h"
#include "common/utils.h"
#include "drivers/gpio.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/compass.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/accgyro.h"
#include "drivers/light_led.h"
#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "io/beeper.h"
#include "io/display.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_cli.h"
#include "io/serial_msp.h"
#include "io/statusindicator.h"
#include "rx/rx.h"
#include "rx/msp.h"
#include "telemetry/telemetry.h"
#include "flight/mixer.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/navigation.h"
#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"
#include "blackbox.h"
#include "blackbox_io.h"
#include "debug.h"
#define BLACKBOX_I_INTERVAL 32
#define BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS 200
#define SLOW_FRAME_INTERVAL 4096
#define ARRAY_LENGTH(x) (sizeof((x))/sizeof((x)[0]))
#define STATIC_ASSERT(condition,name) \
    typedef char assert_failed_ ## name [(condition) ? 1 : -1 ]
#define PREDICT(x) CONCAT(FLIGHT_LOG_FIELD_PREDICTOR_, x)
#define ENCODING(x) CONCAT(FLIGHT_LOG_FIELD_ENCODING_, x)
#define CONDITION(x) CONCAT(FLIGHT_LOG_FIELD_CONDITION_, x)
#define UNSIGNED FLIGHT_LOG_FIELD_UNSIGNED
#define SIGNED FLIGHT_LOG_FIELD_SIGNED
uint32_t hidden_denom = 1;
static const char blackboxHeader[] =
    "H Product:Blackbox flight data recorder by Nicholas Sherlock\n"
    "H Data version:2\n"
    "H I interval:" STR(BLACKBOX_I_INTERVAL) "\n";
static const char* const blackboxFieldHeaderNames[] = {
    "name",
    "signed",
    "predictor",
    "encoding",
    "predictor",
    "encoding"
};
typedef struct blackboxFieldDefinition_s {
    const char *name;
    int8_t fieldNameIndex;
    uint8_t arr[1];
} blackboxFieldDefinition_t;
#define BLACKBOX_DELTA_FIELD_HEADER_COUNT ARRAY_LENGTH(blackboxFieldHeaderNames)
#define BLACKBOX_SIMPLE_FIELD_HEADER_COUNT (BLACKBOX_DELTA_FIELD_HEADER_COUNT - 2)
#define BLACKBOX_CONDITIONAL_FIELD_HEADER_COUNT (BLACKBOX_DELTA_FIELD_HEADER_COUNT - 2)
typedef struct blackboxSimpleFieldDefinition_s {
    const char *name;
    int8_t fieldNameIndex;
    uint8_t isSigned;
    uint8_t predict;
    uint8_t encode;
} blackboxSimpleFieldDefinition_t;
typedef struct blackboxConditionalFieldDefinition_s {
    const char *name;
    int8_t fieldNameIndex;
    uint8_t isSigned;
    uint8_t predict;
    uint8_t encode;
    uint8_t condition;
} blackboxConditionalFieldDefinition_t;
typedef struct blackboxDeltaFieldDefinition_s {
    const char *name;
    int8_t fieldNameIndex;
    uint8_t isSigned;
    uint8_t Ipredict;
    uint8_t Iencode;
    uint8_t Ppredict;
    uint8_t Pencode;
    uint8_t condition;
} blackboxDeltaFieldDefinition_t;
static const blackboxDeltaFieldDefinition_t blackboxMainFields[] = {
    {"loopIteration",-1, UNSIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(INC), .Pencode = FLIGHT_LOG_FIELD_ENCODING_NULL, CONDITION(ALWAYS)},
    {"time", -1, UNSIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(STRAIGHT_LINE), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"axisP", 0, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"axisP", 1, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"axisP", 2, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"axisI", 0, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(TAG2_3S32), CONDITION(ALWAYS)},
    {"axisI", 1, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(TAG2_3S32), CONDITION(ALWAYS)},
    {"axisI", 2, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(TAG2_3S32), CONDITION(ALWAYS)},
    {"axisD", 0, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_0)},
    {"axisD", 1, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_1)},
    {"axisD", 2, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_2)},
    {"rcCommand", 0, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(TAG8_4S16), CONDITION(ALWAYS)},
    {"rcCommand", 1, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(TAG8_4S16), CONDITION(ALWAYS)},
    {"rcCommand", 2, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(TAG8_4S16), CONDITION(ALWAYS)},
    {"rcCommand", 3, UNSIGNED, .Ipredict = PREDICT(MINTHROTTLE), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(TAG8_4S16), CONDITION(ALWAYS)},
    {"vbatLatest", -1, UNSIGNED, .Ipredict = PREDICT(VBATREF), .Iencode = ENCODING(NEG_14BIT), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_VBAT},
    {"amperageLatest",-1, UNSIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC},
#ifdef MAG
    {"magADC", 0, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_MAG},
    {"magADC", 1, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_MAG},
    {"magADC", 2, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_MAG},
#endif
#ifdef BARO
    {"BaroAlt", -1, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_BARO},
#endif
#ifdef SONAR
    {"sonarRaw", -1, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_SONAR},
#endif
    {"rssi", -1, UNSIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(TAG8_8SVB), FLIGHT_LOG_FIELD_CONDITION_RSSI},
    {"gyroADC", 0, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"gyroADC", 1, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"gyroADC", 2, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"accSmooth", 0, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"accSmooth", 1, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"accSmooth", 2, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"debug", 0, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"debug", 1, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"debug", 2, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"motor", 0, UNSIGNED, .Ipredict = PREDICT(MINTHROTTLE), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_1)},
    {"motor", 1, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_2)},
    {"motor", 2, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_3)},
    {"motor", 3, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_4)},
    {"motor", 4, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_5)},
    {"motor", 5, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_6)},
    {"motor", 6, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_7)},
    {"motor", 7, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_8)},
    {"servo", 5, UNSIGNED, .Ipredict = PREDICT(1500), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(SIGNED_VB), CONDITION(TRICOPTER)}
};
#ifdef GPS
static const blackboxConditionalFieldDefinition_t blackboxGpsGFields[] = {
    {"time", -1, UNSIGNED, PREDICT(LAST_MAIN_FRAME_TIME), ENCODING(UNSIGNED_VB), CONDITION(NOT_LOGGING_EVERY_FRAME)},
    {"GPS_numSat", -1, UNSIGNED, PREDICT(0), ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_coord", 0, SIGNED, PREDICT(HOME_COORD), ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_coord", 1, SIGNED, PREDICT(HOME_COORD), ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_altitude", -1, UNSIGNED, PREDICT(0), ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_speed", -1, UNSIGNED, PREDICT(0), ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_ground_course", -1, UNSIGNED, PREDICT(0), ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)}
};
static const blackboxSimpleFieldDefinition_t blackboxGpsHFields[] = {
    {"GPS_home", 0, SIGNED, PREDICT(0), ENCODING(SIGNED_VB)},
    {"GPS_home", 1, SIGNED, PREDICT(0), ENCODING(SIGNED_VB)}
};
#endif
static const blackboxSimpleFieldDefinition_t blackboxSlowFields[] = {
    {"flightModeFlags", -1, UNSIGNED, PREDICT(0), ENCODING(UNSIGNED_VB)},
    {"stateFlags", -1, UNSIGNED, PREDICT(0), ENCODING(UNSIGNED_VB)},
    {"failsafePhase", -1, UNSIGNED, PREDICT(0), ENCODING(TAG2_3S32)},
    {"rxSignalReceived", -1, UNSIGNED, PREDICT(0), ENCODING(TAG2_3S32)},
    {"rxFlightChannelsValid", -1, UNSIGNED, PREDICT(0), ENCODING(TAG2_3S32)}
};
typedef enum BlackboxState {
    BLACKBOX_STATE_DISABLED = 0,
    BLACKBOX_STATE_STOPPED,
    BLACKBOX_STATE_SEND_HEADER,
    BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER,
    BLACKBOX_STATE_SEND_GPS_H_HEADER,
    BLACKBOX_STATE_SEND_GPS_G_HEADER,
    BLACKBOX_STATE_SEND_SLOW_HEADER,
    BLACKBOX_STATE_SEND_SYSINFO,
    BLACKBOX_STATE_PAUSED,
    BLACKBOX_STATE_RUNNING,
    BLACKBOX_STATE_SHUTTING_DOWN
} BlackboxState;
#define BLACKBOX_FIRST_HEADER_SENDING_STATE BLACKBOX_STATE_SEND_HEADER
#define BLACKBOX_LAST_HEADER_SENDING_STATE BLACKBOX_STATE_SEND_SYSINFO
typedef struct blackboxMainState_s {
    uint32_t time;
    int32_t axisPID_P[XYZ_AXIS_COUNT], axisPID_I[XYZ_AXIS_COUNT], axisPID_D[XYZ_AXIS_COUNT];
    int16_t rcCommand[4];
    int16_t gyroADC[XYZ_AXIS_COUNT];
    int16_t accSmooth[XYZ_AXIS_COUNT];
    int16_t debug[3];
    int16_t motor[MAX_SUPPORTED_MOTORS];
    int16_t servo[MAX_SUPPORTED_SERVOS];
    uint16_t vbatLatest;
    uint16_t amperageLatest;
#ifdef BARO
    int32_t BaroAlt;
#endif
#ifdef MAG
    int16_t magADC[XYZ_AXIS_COUNT];
#endif
#ifdef SONAR
    int32_t sonarRaw;
#endif
    uint16_t rssi;
} blackboxMainState_t;
typedef struct blackboxGpsState_s {
    int32_t GPS_home[2], GPS_coord[2];
    uint8_t GPS_numSat;
} blackboxGpsState_t;
typedef struct blackboxSlowState_s {
    uint32_t flightModeFlags;
    uint8_t stateFlags;
    uint8_t failsafePhase;
    bool rxSignalReceived;
    bool rxFlightChannelsValid;
} __attribute__((__packed__)) blackboxSlowState_t;
extern uint8_t motorCount;
extern uint32_t currentTime;
extern uint16_t rssi;
static BlackboxState blackboxState = BLACKBOX_STATE_DISABLED;
static uint32_t blackboxLastArmingBeep = 0;
static struct {
    uint32_t headerIndex;
    union {
        int fieldIndex;
        uint32_t startTime;
    } u;
} xmitState;
static uint32_t blackboxConditionCache;
STATIC_ASSERT((sizeof(blackboxConditionCache) * 8) >= FLIGHT_LOG_FIELD_CONDITION_NEVER, too_many_flight_log_conditions);
static uint32_t blackboxIteration;
static uint16_t blackboxPFrameIndex, blackboxIFrameIndex;
static uint16_t blackboxSlowFrameIterationTimer;
static uint16_t vbatReference;
static blackboxGpsState_t gpsHistory;
static blackboxSlowState_t slowHistory;
static blackboxMainState_t blackboxHistoryRing[3];
static blackboxMainState_t* blackboxHistory[3];
static bool blackboxModeActivationConditionPresent = false;
static bool blackboxIsOnlyLoggingIntraframes() {
    return masterConfig.blackbox_rate_num == 1 && masterConfig.blackbox_rate_denom == 32;
}
static bool testBlackboxConditionUncached(FlightLogFieldCondition condition)
{
    switch (condition) {
        case FLIGHT_LOG_FIELD_CONDITION_ALWAYS:
            return true;
        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1:
        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_2:
        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_3:
        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_4:
        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_5:
        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_6:
        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_7:
        case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_8:
            return motorCount >= condition - FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1 + 1;
        case FLIGHT_LOG_FIELD_CONDITION_TRICOPTER:
            return masterConfig.mixerMode == MIXER_TRI || masterConfig.mixerMode == MIXER_CUSTOM_TRI;
        case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0:
        case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_1:
        case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_2:
            if (IS_PID_CONTROLLER_FP_BASED(currentProfile->pidProfile.pidController)) {
                return currentProfile->pidProfile.D_f[condition - FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0] != 0;
            } else {
                return currentProfile->pidProfile.D8[condition - FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0] != 0;
            }
        case FLIGHT_LOG_FIELD_CONDITION_MAG:
#ifdef MAG
            return sensors(SENSOR_MAG);
#else
            return false;
#endif
        case FLIGHT_LOG_FIELD_CONDITION_BARO:
#ifdef BARO
            return sensors(SENSOR_BARO);
#else
            return false;
#endif
        case FLIGHT_LOG_FIELD_CONDITION_VBAT:
            return feature(FEATURE_VBAT);
        case FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC:
            return feature(FEATURE_CURRENT_METER) && masterConfig.batteryConfig.currentMeterType == CURRENT_SENSOR_ADC;
        case FLIGHT_LOG_FIELD_CONDITION_SONAR:
#ifdef SONAR
            return feature(FEATURE_SONAR);
#else
            return false;
#endif
        case FLIGHT_LOG_FIELD_CONDITION_RSSI:
            return masterConfig.rxConfig.rssi_channel > 0 || feature(FEATURE_RSSI_ADC);
        case FLIGHT_LOG_FIELD_CONDITION_NOT_LOGGING_EVERY_FRAME:
            return masterConfig.blackbox_rate_num < masterConfig.blackbox_rate_denom;
        case FLIGHT_LOG_FIELD_CONDITION_NEVER:
            return false;
        default:
            return false;
    }
}
static void blackboxBuildConditionCache()
{
    FlightLogFieldCondition cond;
    blackboxConditionCache = 0;
    for (cond = FLIGHT_LOG_FIELD_CONDITION_FIRST; cond <= FLIGHT_LOG_FIELD_CONDITION_LAST; cond++) {
        if (testBlackboxConditionUncached(cond)) {
            blackboxConditionCache |= 1 << cond;
        }
    }
}
static bool testBlackboxCondition(FlightLogFieldCondition condition)
{
    return (blackboxConditionCache & (1 << condition)) != 0;
}
static void blackboxSetState(BlackboxState newState)
{
    switch (newState) {
        case BLACKBOX_STATE_SEND_HEADER:
            blackboxHeaderBudget = 0;
            xmitState.headerIndex = 0;
            xmitState.u.startTime = millis();
        break;
        case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER:
        case BLACKBOX_STATE_SEND_GPS_G_HEADER:
        case BLACKBOX_STATE_SEND_GPS_H_HEADER:
        case BLACKBOX_STATE_SEND_SLOW_HEADER:
            xmitState.headerIndex = 0;
            xmitState.u.fieldIndex = -1;
        break;
        case BLACKBOX_STATE_SEND_SYSINFO:
            xmitState.headerIndex = 0;
        break;
        case BLACKBOX_STATE_RUNNING:
            blackboxSlowFrameIterationTimer = SLOW_FRAME_INTERVAL;
        break;
        case BLACKBOX_STATE_SHUTTING_DOWN:
            xmitState.u.startTime = millis();
            blackboxDeviceFlush();
        break;
        default:
            ;
    }
    blackboxState = newState;
}
static void writeIntraframe(void)
{
    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];
    int x;
    blackboxWrite('I');
    blackboxWriteUnsignedVB(blackboxIteration);
    blackboxWriteUnsignedVB(blackboxCurrent->time);
    blackboxWriteSignedVBArray(blackboxCurrent->axisPID_P, XYZ_AXIS_COUNT);
    blackboxWriteSignedVBArray(blackboxCurrent->axisPID_I, XYZ_AXIS_COUNT);
    for (x = 0; x < XYZ_AXIS_COUNT; x++) {
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0 + x)) {
            blackboxWriteSignedVB(blackboxCurrent->axisPID_D[x]);
        }
    }
    blackboxWriteSigned16VBArray(blackboxCurrent->rcCommand, 3);
    blackboxWriteUnsignedVB(blackboxCurrent->rcCommand[THROTTLE] - masterConfig.escAndServoConfig.minthrottle);
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_VBAT)) {
        blackboxWriteUnsignedVB((vbatReference - blackboxCurrent->vbatLatest) & 0x3FFF);
    }
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC)) {
        blackboxWriteUnsignedVB(blackboxCurrent->amperageLatest);
    }
#ifdef MAG
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_MAG)) {
            blackboxWriteSigned16VBArray(blackboxCurrent->magADC, XYZ_AXIS_COUNT);
        }
#endif
#ifdef BARO
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_BARO)) {
            blackboxWriteSignedVB(blackboxCurrent->BaroAlt);
        }
#endif
#ifdef SONAR
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_SONAR)) {
            blackboxWriteSignedVB(blackboxCurrent->sonarRaw);
        }
#endif
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_RSSI)) {
        blackboxWriteUnsignedVB(blackboxCurrent->rssi);
    }
    blackboxWriteSigned16VBArray(blackboxCurrent->gyroADC, XYZ_AXIS_COUNT);
    blackboxWriteSigned16VBArray(blackboxCurrent->accSmooth, XYZ_AXIS_COUNT);
    blackboxWriteSigned16VBArray(blackboxCurrent->debug, 3);
    blackboxWriteUnsignedVB(blackboxCurrent->motor[0] - masterConfig.escAndServoConfig.minthrottle);
    for (x = 1; x < motorCount; x++) {
        blackboxWriteSignedVB(blackboxCurrent->motor[x] - blackboxCurrent->motor[0]);
    }
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_TRICOPTER)) {
        blackboxWriteSignedVB(blackboxCurrent->servo[5] - 1500);
    }
    blackboxHistory[1] = blackboxHistory[0];
    blackboxHistory[2] = blackboxHistory[0];
    blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;
}
static void blackboxWriteMainStateArrayUsingAveragePredictor(int arrOffsetInHistory, int count)
{
    int16_t *curr = (int16_t*) ((char*) (blackboxHistory[0]) + arrOffsetInHistory);
    int16_t *prev1 = (int16_t*) ((char*) (blackboxHistory[1]) + arrOffsetInHistory);
    int16_t *prev2 = (int16_t*) ((char*) (blackboxHistory[2]) + arrOffsetInHistory);
    for (int i = 0; i < count; i++) {
        int32_t predictor = (prev1[i] + prev2[i]) / 2;
        blackboxWriteSignedVB(curr[i] - predictor);
    }
}
static void writeInterframe(void)
{
    int x;
    int32_t deltas[8];
    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];
    blackboxMainState_t *blackboxLast = blackboxHistory[1];
    blackboxWrite('P');
    blackboxWriteSignedVB((int32_t) (blackboxHistory[0]->time - 2 * blackboxHistory[1]->time + blackboxHistory[2]->time));
    arraySubInt32(deltas, blackboxCurrent->axisPID_P, blackboxLast->axisPID_P, XYZ_AXIS_COUNT);
    blackboxWriteSignedVBArray(deltas, XYZ_AXIS_COUNT);
    arraySubInt32(deltas, blackboxCurrent->axisPID_I, blackboxLast->axisPID_I, XYZ_AXIS_COUNT);
    blackboxWriteTag2_3S32(deltas);
    for (x = 0; x < XYZ_AXIS_COUNT; x++) {
        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0 + x)) {
            blackboxWriteSignedVB(blackboxCurrent->axisPID_D[x] - blackboxLast->axisPID_D[x]);
        }
    }
    for (x = 0; x < 4; x++) {
        deltas[x] = blackboxCurrent->rcCommand[x] - blackboxLast->rcCommand[x];
    }
    blackboxWriteTag8_4S16(deltas);
    int optionalFieldCount = 0;
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_VBAT)) {
        deltas[optionalFieldCount++] = (int32_t) blackboxCurrent->vbatLatest - blackboxLast->vbatLatest;
    }
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC)) {
        deltas[optionalFieldCount++] = (int32_t) blackboxCurrent->amperageLatest - blackboxLast->amperageLatest;
    }
#ifdef MAG
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_MAG)) {
        for (x = 0; x < XYZ_AXIS_COUNT; x++) {
            deltas[optionalFieldCount++] = blackboxCurrent->magADC[x] - blackboxLast->magADC[x];
        }
    }
#endif
#ifdef BARO
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_BARO)) {
        deltas[optionalFieldCount++] = blackboxCurrent->BaroAlt - blackboxLast->BaroAlt;
    }
#endif
#ifdef SONAR
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_SONAR)) {
        deltas[optionalFieldCount++] = blackboxCurrent->sonarRaw - blackboxLast->sonarRaw;
    }
#endif
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_RSSI)) {
        deltas[optionalFieldCount++] = (int32_t) blackboxCurrent->rssi - blackboxLast->rssi;
    }
    blackboxWriteTag8_8SVB(deltas, optionalFieldCount);
    blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, gyroADC), XYZ_AXIS_COUNT);
    blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, accSmooth), XYZ_AXIS_COUNT);
    blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, debug), 3);
    blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, motor), motorCount);
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_TRICOPTER)) {
        blackboxWriteSignedVB(blackboxCurrent->servo[5] - blackboxLast->servo[5]);
    }
    blackboxHistory[2] = blackboxHistory[1];
    blackboxHistory[1] = blackboxHistory[0];
    blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;
}
static void writeSlowFrame(void)
{
    int32_t values[3];
    blackboxWrite('S');
    blackboxWriteUnsignedVB(slowHistory.flightModeFlags);
    blackboxWriteUnsignedVB(slowHistory.stateFlags);
    values[0] = slowHistory.failsafePhase;
    values[1] = slowHistory.rxSignalReceived ? 1 : 0;
    values[2] = slowHistory.rxFlightChannelsValid ? 1 : 0;
    blackboxWriteTag2_3S32(values);
    blackboxSlowFrameIterationTimer = 0;
}
static void loadSlowState(blackboxSlowState_t *slow)
{
    slow->flightModeFlags = flightModeFlags;
    slow->stateFlags = stateFlags;
    slow->failsafePhase = failsafePhase();
    slow->rxSignalReceived = rxIsReceivingSignal();
    slow->rxFlightChannelsValid = rxAreFlightChannelsValid();
}
static void writeSlowFrameIfNeeded(bool allowPeriodicWrite)
{
    bool shouldWrite = allowPeriodicWrite && blackboxSlowFrameIterationTimer >= SLOW_FRAME_INTERVAL;
    if (shouldWrite) {
        loadSlowState(&slowHistory);
    } else {
        blackboxSlowState_t newSlowState;
        loadSlowState(&newSlowState);
        if (memcmp(&newSlowState, &slowHistory, sizeof(slowHistory)) != 0) {
            memcpy(&slowHistory, &newSlowState, sizeof(slowHistory));
            shouldWrite = true;
        }
    }
    if (shouldWrite) {
        writeSlowFrame();
    }
}
static void validateBlackboxConfig()
{
    if (masterConfig.blackbox_rate_num == 0 || masterConfig.blackbox_rate_denom == 0
            || masterConfig.blackbox_rate_num >= masterConfig.blackbox_rate_denom) {
        masterConfig.blackbox_rate_num = 1;
        masterConfig.blackbox_rate_denom = 1;
    }
    hidden_denom = 8;
    masterConfig.blackbox_rate_num = 1;
    if (masterConfig.blackbox_device >= BLACKBOX_DEVICE_END) {
        masterConfig.blackbox_device = BLACKBOX_DEVICE_SERIAL;
    }
}
void startBlackbox(void)
{
    if (blackboxState == BLACKBOX_STATE_STOPPED) {
        validateBlackboxConfig();
        if (!blackboxDeviceOpen()) {
            blackboxSetState(BLACKBOX_STATE_DISABLED);
            return;
        }
        memset(&gpsHistory, 0, sizeof(gpsHistory));
        blackboxHistory[0] = &blackboxHistoryRing[0];
        blackboxHistory[1] = &blackboxHistoryRing[1];
        blackboxHistory[2] = &blackboxHistoryRing[2];
        vbatReference = vbatLatestADC;
        blackboxBuildConditionCache();
        blackboxModeActivationConditionPresent = isModeActivationConditionPresent(currentProfile->modeActivationConditions, BOXBLACKBOX);
        blackboxIteration = 0;
        blackboxPFrameIndex = 0;
        blackboxIFrameIndex = 0;
        blackboxLastArmingBeep = getArmingBeepTimeMicros();
        blackboxSetState(BLACKBOX_STATE_SEND_HEADER);
    }
}
void finishBlackbox(void)
{
    if (blackboxState == BLACKBOX_STATE_RUNNING || blackboxState == BLACKBOX_STATE_PAUSED) {
        blackboxLogEvent(FLIGHT_LOG_EVENT_LOG_END, NULL);
        blackboxSetState(BLACKBOX_STATE_SHUTTING_DOWN);
    } else if (blackboxState != BLACKBOX_STATE_DISABLED && blackboxState != BLACKBOX_STATE_STOPPED
            && blackboxState != BLACKBOX_STATE_SHUTTING_DOWN) {
        blackboxDeviceClose();
        blackboxSetState(BLACKBOX_STATE_STOPPED);
    }
}
#ifdef GPS
static void writeGPSHomeFrame()
{
    blackboxWrite('H');
    blackboxWriteSignedVB(GPS_home[0]);
    blackboxWriteSignedVB(GPS_home[1]);
    gpsHistory.GPS_home[0] = GPS_home[0];
    gpsHistory.GPS_home[1] = GPS_home[1];
}
static void writeGPSFrame()
{
    blackboxWrite('G');
    if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NOT_LOGGING_EVERY_FRAME)) {
        blackboxWriteUnsignedVB(currentTime - blackboxHistory[1]->time);
    }
    blackboxWriteUnsignedVB(GPS_numSat);
    blackboxWriteSignedVB(GPS_coord[0] - gpsHistory.GPS_home[0]);
    blackboxWriteSignedVB(GPS_coord[1] - gpsHistory.GPS_home[1]);
    blackboxWriteUnsignedVB(GPS_altitude);
    blackboxWriteUnsignedVB(GPS_speed);
    blackboxWriteUnsignedVB(GPS_ground_course);
    gpsHistory.GPS_numSat = GPS_numSat;
    gpsHistory.GPS_coord[0] = GPS_coord[0];
    gpsHistory.GPS_coord[1] = GPS_coord[1];
}
#endif
static void loadMainState(void)
{
    blackboxMainState_t *blackboxCurrent = blackboxHistory[0];
    int i;
    blackboxCurrent->time = currentTime;
    for (i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->axisPID_P[i] = axisPID_P[i];
    }
    for (i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->axisPID_I[i] = axisPID_I[i];
    }
    for (i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->axisPID_D[i] = axisPID_D[i];
    }
    if (masterConfig.rxConfig.rcSmoothing) {
  for (i = 0; i < 4; i++) {
   blackboxCurrent->rcCommand[i] = rcCommandUsed[i];
  }
    } else {
  for (i = 0; i < 4; i++) {
   blackboxCurrent->rcCommand[i] = rcCommand[i];
  }
    }
    for (i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->gyroADC[i] = gyroADC[i];
    }
    for (i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->accSmooth[i] = accSmooth[i];
    }
    for (i = 0; i < 3; i++) {
        blackboxCurrent->debug[i] = debug[i];
    }
    for (i = 0; i < motorCount; i++) {
        blackboxCurrent->motor[i] = motor[i];
    }
    blackboxCurrent->vbatLatest = vbatLatestADC;
    blackboxCurrent->amperageLatest = amperageLatestADC;
#ifdef MAG
    for (i = 0; i < XYZ_AXIS_COUNT; i++) {
        blackboxCurrent->magADC[i] = magADC[i];
    }
#endif
#ifdef BARO
    blackboxCurrent->BaroAlt = BaroAlt;
#endif
#ifdef SONAR
    blackboxCurrent->sonarRaw = sonarRead();
#endif
    blackboxCurrent->rssi = rssi;
#ifdef USE_SERVOS
    blackboxCurrent->servo[5] = servo[5];
#endif
}
static bool sendFieldDefinition(char mainFrameChar, char deltaFrameChar, const void *fieldDefinitions,
        const void *secondFieldDefinition, int fieldCount, const uint8_t *conditions, const uint8_t *secondCondition)
{
    const blackboxFieldDefinition_t *def;
    unsigned int headerCount;
    static bool needComma = false;
    size_t definitionStride = (char*) secondFieldDefinition - (char*) fieldDefinitions;
    size_t conditionsStride = (char*) secondCondition - (char*) conditions;
    if (deltaFrameChar) {
        headerCount = BLACKBOX_DELTA_FIELD_HEADER_COUNT;
    } else {
        headerCount = BLACKBOX_SIMPLE_FIELD_HEADER_COUNT;
    }
    if (xmitState.u.fieldIndex == -1) {
        if (xmitState.headerIndex >= headerCount) {
            return false;
        }
        uint32_t charsToBeWritten = strlen("H Field x :") + strlen(blackboxFieldHeaderNames[xmitState.headerIndex]);
        if (blackboxDeviceReserveBufferSpace(charsToBeWritten) != BLACKBOX_RESERVE_SUCCESS) {
            return true;
        }
        blackboxHeaderBudget -= blackboxPrintf("H Field %c %s:", xmitState.headerIndex >= BLACKBOX_SIMPLE_FIELD_HEADER_COUNT ? deltaFrameChar : mainFrameChar, blackboxFieldHeaderNames[xmitState.headerIndex]);
        xmitState.u.fieldIndex++;
        needComma = false;
    }
    const uint32_t LONGEST_INTEGER_STRLEN = 2;
    for (; xmitState.u.fieldIndex < fieldCount; xmitState.u.fieldIndex++) {
        def = (const blackboxFieldDefinition_t*) ((const char*)fieldDefinitions + definitionStride * xmitState.u.fieldIndex);
        if (!conditions || testBlackboxCondition(conditions[conditionsStride * xmitState.u.fieldIndex])) {
            int32_t bytesToWrite = 1;
            if (xmitState.headerIndex == 0) {
                bytesToWrite += strlen(def->name) + strlen("[]") + LONGEST_INTEGER_STRLEN;
            } else {
                bytesToWrite += LONGEST_INTEGER_STRLEN;
            }
            if (blackboxDeviceReserveBufferSpace(bytesToWrite) != BLACKBOX_RESERVE_SUCCESS) {
                return true;
            }
            blackboxHeaderBudget -= bytesToWrite;
            if (needComma) {
                blackboxWrite(',');
            } else {
                needComma = true;
            }
            if (xmitState.headerIndex == 0) {
                blackboxPrint(def->name);
                if (def->fieldNameIndex != -1) {
                    blackboxPrintf("[%d]", def->fieldNameIndex);
                }
            } else {
                blackboxPrintf("%d", def->arr[xmitState.headerIndex - 1]);
            }
        }
    }
    if (xmitState.u.fieldIndex == fieldCount && blackboxDeviceReserveBufferSpace(1) == BLACKBOX_RESERVE_SUCCESS) {
        blackboxHeaderBudget--;
        blackboxWrite('\n');
        xmitState.headerIndex++;
        xmitState.u.fieldIndex = -1;
    }
    return xmitState.headerIndex < headerCount;
}
static bool blackboxWriteSysinfo()
{
    if (blackboxDeviceReserveBufferSpace(64) != BLACKBOX_RESERVE_SUCCESS) {
        return false;
    }
    switch (xmitState.headerIndex) {
        case 0:
            blackboxPrintfHeaderLine("Firmware type:Cleanflight");
        break;
        case 1:
            blackboxPrintfHeaderLine("Firmware revision:%s", shortGitRevision);
        break;
        case 2:
            blackboxPrintfHeaderLine("Firmware date:%s %s", buildDate, buildTime);
        break;
        case 3:
            blackboxPrintfHeaderLine("P interval:%d/%d", masterConfig.blackbox_rate_num, masterConfig.blackbox_rate_denom);
        break;
        case 4:
            blackboxPrintfHeaderLine("rcRate:%d", 100);
        break;
        case 5:
            blackboxPrintfHeaderLine("minthrottle:%d", masterConfig.escAndServoConfig.minthrottle);
        break;
        case 6:
            blackboxPrintfHeaderLine("maxthrottle:%d", masterConfig.escAndServoConfig.maxthrottle);
        break;
        case 7:
            blackboxPrintfHeaderLine("gyro.scale:0x%x", castFloatBytesToInt(gyro.scale));
        break;
        case 8:
            blackboxPrintfHeaderLine("acc_1G:%u", acc_1G);
        break;
        case 9:
            if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_VBAT)) {
                blackboxPrintfHeaderLine("vbatscale:%u", masterConfig.batteryConfig.vbatscale);
            } else {
                xmitState.headerIndex += 2;
            }
        break;
        case 10:
            blackboxPrintfHeaderLine("vbatcellvoltage:%u,%u,%u", VBATMINCELLVOLTAGE,
                masterConfig.batteryConfig.vbatwarningcellvoltage, masterConfig.batteryConfig.vbatmaxcellvoltage);
        break;
        case 11:
            blackboxPrintfHeaderLine("vbatref:%u", vbatReference);
        break;
        case 12:
            if (feature(FEATURE_CURRENT_METER)) {
                blackboxPrintfHeaderLine("currentMeter:%d,%d", masterConfig.batteryConfig.currentMeterOffset, masterConfig.batteryConfig.currentMeterScale);
            }
        break;
        default:
            return true;
    }
    xmitState.headerIndex++;
    return false;
}
void blackboxLogEvent(FlightLogEvent event, flightLogEventData_t *data)
{
    if (!(blackboxState == BLACKBOX_STATE_RUNNING || blackboxState == BLACKBOX_STATE_PAUSED)) {
        return;
    }
    blackboxWrite('E');
    blackboxWrite(event);
    switch (event) {
        case FLIGHT_LOG_EVENT_SYNC_BEEP:
            blackboxWriteUnsignedVB(data->syncBeep.time);
        break;
        case FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT:
            if (data->inflightAdjustment.floatFlag) {
                blackboxWrite(data->inflightAdjustment.adjustmentFunction + FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT_FUNCTION_FLOAT_VALUE_FLAG);
                blackboxWriteFloat(data->inflightAdjustment.newFloatValue);
            } else {
                blackboxWrite(data->inflightAdjustment.adjustmentFunction);
                blackboxWriteSignedVB(data->inflightAdjustment.newValue);
            }
        case FLIGHT_LOG_EVENT_GTUNE_RESULT:
            blackboxWrite(data->gtuneCycleResult.gtuneAxis);
            blackboxWriteSignedVB(data->gtuneCycleResult.gtuneGyroAVG);
            blackboxWriteS16(data->gtuneCycleResult.gtuneNewP);
        break;
        case FLIGHT_LOG_EVENT_LOGGING_RESUME:
            blackboxWriteUnsignedVB(data->loggingResume.logIteration);
            blackboxWriteUnsignedVB(data->loggingResume.currentTime);
        break;
        case FLIGHT_LOG_EVENT_LOG_END:
            blackboxPrint("End of log");
            blackboxWrite(0);
        break;
    }
}
static void blackboxCheckAndLogArmingBeep()
{
    flightLogEvent_syncBeep_t eventData;
    if (getArmingBeepTimeMicros() != blackboxLastArmingBeep) {
        blackboxLastArmingBeep = getArmingBeepTimeMicros();
        eventData.time = blackboxLastArmingBeep;
        blackboxLogEvent(FLIGHT_LOG_EVENT_SYNC_BEEP, (flightLogEventData_t *) &eventData);
    }
}
static bool blackboxShouldLogPFrame(uint32_t pFrameIndex)
{
 return (pFrameIndex + (uint32_t)masterConfig.blackbox_rate_num - 1) % hidden_denom < (uint32_t)masterConfig.blackbox_rate_num;
}
static bool blackboxShouldLogIFrame() {
    return blackboxPFrameIndex == 0;
}
static void blackboxAdvanceIterationTimers()
{
    blackboxSlowFrameIterationTimer++;
    blackboxIteration++;
    blackboxPFrameIndex++;
    if (blackboxPFrameIndex == BLACKBOX_I_INTERVAL) {
        blackboxPFrameIndex = 0;
        blackboxIFrameIndex++;
    }
}
static void blackboxLogIteration()
{
    if (blackboxShouldLogIFrame()) {
        writeSlowFrameIfNeeded(blackboxIsOnlyLoggingIntraframes());
        loadMainState();
        writeIntraframe();
    } else {
        blackboxCheckAndLogArmingBeep();
        if (blackboxShouldLogPFrame(blackboxPFrameIndex)) {
            writeSlowFrameIfNeeded(true);
            loadMainState();
            writeInterframe();
        }
#ifdef GPS
        if (feature(FEATURE_GPS)) {
            if (GPS_home[0] != gpsHistory.GPS_home[0] || GPS_home[1] != gpsHistory.GPS_home[1]
                || (blackboxPFrameIndex == BLACKBOX_I_INTERVAL / 2 && blackboxIFrameIndex % 128 == 0)) {
                writeGPSHomeFrame();
                writeGPSFrame();
            } else if (GPS_numSat != gpsHistory.GPS_numSat || GPS_coord[0] != gpsHistory.GPS_coord[0]
                    || GPS_coord[1] != gpsHistory.GPS_coord[1]) {
                writeGPSFrame();
            }
        }
#endif
    }
    blackboxDeviceFlush();
}
void handleBlackbox(void)
{
    int i;
    if (blackboxState >= BLACKBOX_FIRST_HEADER_SENDING_STATE && blackboxState <= BLACKBOX_LAST_HEADER_SENDING_STATE) {
        blackboxReplenishHeaderBudget();
    }
    switch (blackboxState) {
        case BLACKBOX_STATE_SEND_HEADER:
            if (millis() > xmitState.u.startTime + 100) {
                if (blackboxDeviceReserveBufferSpace(BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION) == BLACKBOX_RESERVE_SUCCESS) {
                    for (i = 0; i < BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION && blackboxHeader[xmitState.headerIndex] != '\0'; i++, xmitState.headerIndex++) {
                        blackboxWrite(blackboxHeader[xmitState.headerIndex]);
                        blackboxHeaderBudget--;
                    }
                    if (blackboxHeader[xmitState.headerIndex] == '\0') {
                        blackboxSetState(BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER);
                    }
                }
            }
        break;
        case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER:
            if (!sendFieldDefinition('I', 'P', blackboxMainFields, blackboxMainFields + 1, ARRAY_LENGTH(blackboxMainFields),
                    &blackboxMainFields[0].condition, &blackboxMainFields[1].condition)) {
#ifdef GPS
                if (feature(FEATURE_GPS)) {
                    blackboxSetState(BLACKBOX_STATE_SEND_GPS_H_HEADER);
                } else
#endif
                    blackboxSetState(BLACKBOX_STATE_SEND_SLOW_HEADER);
            }
        break;
#ifdef GPS
        case BLACKBOX_STATE_SEND_GPS_H_HEADER:
            if (!sendFieldDefinition('H', 0, blackboxGpsHFields, blackboxGpsHFields + 1, ARRAY_LENGTH(blackboxGpsHFields),
                    NULL, NULL)) {
                blackboxSetState(BLACKBOX_STATE_SEND_GPS_G_HEADER);
            }
        break;
        case BLACKBOX_STATE_SEND_GPS_G_HEADER:
            if (!sendFieldDefinition('G', 0, blackboxGpsGFields, blackboxGpsGFields + 1, ARRAY_LENGTH(blackboxGpsGFields),
                    &blackboxGpsGFields[0].condition, &blackboxGpsGFields[1].condition)) {
                blackboxSetState(BLACKBOX_STATE_SEND_SLOW_HEADER);
            }
        break;
#endif
        case BLACKBOX_STATE_SEND_SLOW_HEADER:
            if (!sendFieldDefinition('S', 0, blackboxSlowFields, blackboxSlowFields + 1, ARRAY_LENGTH(blackboxSlowFields),
                    NULL, NULL)) {
                blackboxSetState(BLACKBOX_STATE_SEND_SYSINFO);
            }
        break;
        case BLACKBOX_STATE_SEND_SYSINFO:
            if (blackboxWriteSysinfo()) {
                if (blackboxDeviceFlush()) {
                    blackboxSetState(BLACKBOX_STATE_RUNNING);
                }
            }
        break;
        case BLACKBOX_STATE_PAUSED:
            if (IS_RC_MODE_ACTIVE(BOXBLACKBOX) && blackboxShouldLogIFrame()) {
                flightLogEvent_loggingResume_t resume;
                resume.logIteration = blackboxIteration;
                resume.currentTime = currentTime;
                blackboxLogEvent(FLIGHT_LOG_EVENT_LOGGING_RESUME, (flightLogEventData_t *) &resume);
                blackboxSetState(BLACKBOX_STATE_RUNNING);
                blackboxLogIteration();
            }
            blackboxAdvanceIterationTimers();
        break;
        case BLACKBOX_STATE_RUNNING:
            if (blackboxModeActivationConditionPresent && !IS_RC_MODE_ACTIVE(BOXBLACKBOX)) {
                blackboxSetState(BLACKBOX_STATE_PAUSED);
            } else {
                blackboxLogIteration();
            }
            blackboxAdvanceIterationTimers();
        break;
        case BLACKBOX_STATE_SHUTTING_DOWN:
            if (millis() > xmitState.u.startTime + BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS || blackboxDeviceFlush()) {
                blackboxDeviceClose();
                blackboxSetState(BLACKBOX_STATE_STOPPED);
            }
        break;
        default:
        break;
    }
    if (isBlackboxDeviceFull()) {
        blackboxSetState(BLACKBOX_STATE_STOPPED);
    }
}
static bool canUseBlackboxWithCurrentConfiguration(void)
{
    return feature(FEATURE_BLACKBOX);
}
void initBlackbox(void)
{
    if (canUseBlackboxWithCurrentConfiguration()) {
        blackboxSetState(BLACKBOX_STATE_STOPPED);
    } else {
        blackboxSetState(BLACKBOX_STATE_DISABLED);
    }
}
