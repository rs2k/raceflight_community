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

#include <stdint.h>

typedef enum FlightLogFieldCondition {
    FLIGHT_LOG_FIELD_CONDITION_ALWAYS = 0,
    FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1,
    FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_2,
    FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_3,
    FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_4,
    FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_5,
    FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_6,
    FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_7,
    FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_8,
    FLIGHT_LOG_FIELD_CONDITION_TRICOPTER,

    FLIGHT_LOG_FIELD_CONDITION_MAG,
    FLIGHT_LOG_FIELD_CONDITION_BARO,
    FLIGHT_LOG_FIELD_CONDITION_VBAT,
    FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC,
    FLIGHT_LOG_FIELD_CONDITION_SONAR,
    FLIGHT_LOG_FIELD_CONDITION_RSSI,

    FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0,
    FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_1,
    FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_2,

    FLIGHT_LOG_FIELD_CONDITION_NOT_LOGGING_EVERY_FRAME,

    FLIGHT_LOG_FIELD_CONDITION_NEVER,

    FLIGHT_LOG_FIELD_CONDITION_FIRST = FLIGHT_LOG_FIELD_CONDITION_ALWAYS,
    FLIGHT_LOG_FIELD_CONDITION_LAST = FLIGHT_LOG_FIELD_CONDITION_NEVER
} FlightLogFieldCondition;

typedef enum FlightLogFieldPredictor {
    
    FLIGHT_LOG_FIELD_PREDICTOR_0              = 0,

    
    FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS       = 1,

    
    FLIGHT_LOG_FIELD_PREDICTOR_STRAIGHT_LINE  = 2,

    
    FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2      = 3,

    
    FLIGHT_LOG_FIELD_PREDICTOR_MINTHROTTLE    = 4,

    
    FLIGHT_LOG_FIELD_PREDICTOR_MOTOR_0        = 5,

    
    FLIGHT_LOG_FIELD_PREDICTOR_INC            = 6,

    
    FLIGHT_LOG_FIELD_PREDICTOR_HOME_COORD     = 7,

    
    FLIGHT_LOG_FIELD_PREDICTOR_1500           = 8,

    
    FLIGHT_LOG_FIELD_PREDICTOR_VBATREF        = 9,

    
    FLIGHT_LOG_FIELD_PREDICTOR_LAST_MAIN_FRAME_TIME = 10

} FlightLogFieldPredictor;

typedef enum FlightLogFieldEncoding {
    FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB       = 0, 
    FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB     = 1, 
    FLIGHT_LOG_FIELD_ENCODING_NEG_14BIT       = 3, 
    FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB       = 6,
    FLIGHT_LOG_FIELD_ENCODING_TAG2_3S32       = 7,
    FLIGHT_LOG_FIELD_ENCODING_TAG8_4S16       = 8,
    FLIGHT_LOG_FIELD_ENCODING_NULL            = 9 
} FlightLogFieldEncoding;

typedef enum FlightLogFieldSign {
    FLIGHT_LOG_FIELD_UNSIGNED = 0,
    FLIGHT_LOG_FIELD_SIGNED   = 1
} FlightLogFieldSign;

typedef enum FlightLogEvent {
    FLIGHT_LOG_EVENT_SYNC_BEEP = 0,
    FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT = 13,
    FLIGHT_LOG_EVENT_LOGGING_RESUME = 14,
    FLIGHT_LOG_EVENT_GTUNE_RESULT = 20,
    FLIGHT_LOG_EVENT_LOG_END = 255
} FlightLogEvent;

typedef struct flightLogEvent_syncBeep_s {
    uint32_t time;
} flightLogEvent_syncBeep_t;

typedef struct flightLogEvent_inflightAdjustment_s {
    uint8_t adjustmentFunction;
    bool floatFlag;
    int32_t newValue;
    float newFloatValue;
} flightLogEvent_inflightAdjustment_t;

typedef struct flightLogEvent_loggingResume_s {
    uint32_t logIteration;
    uint32_t currentTime;
} flightLogEvent_loggingResume_t;

#define FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT_FUNCTION_FLOAT_VALUE_FLAG 128

typedef struct flightLogEvent_gtuneCycleResult_s {
    uint8_t gtuneAxis;
    int32_t gtuneGyroAVG;
    int16_t gtuneNewP;
} flightLogEvent_gtuneCycleResult_t;

typedef union flightLogEventData_u {
    flightLogEvent_syncBeep_t syncBeep;
    flightLogEvent_inflightAdjustment_t inflightAdjustment;
    flightLogEvent_loggingResume_t loggingResume;
    flightLogEvent_gtuneCycleResult_t gtuneCycleResult;
} flightLogEventData_t;

typedef struct flightLogEvent_s {
    FlightLogEvent event;
    flightLogEventData_t data;
} flightLogEvent_t;
