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
       
#include "rx/rx.h"
#ifndef TELEMETRY_COMMON_H_
#define TELEMETRY_COMMON_H_ 
typedef enum {
    FRSKY_FORMAT_DMS = 0,
    FRSKY_FORMAT_NMEA
} frskyGpsCoordFormat_e;
typedef enum {
    FRSKY_UNIT_METRICS = 0,
    FRSKY_UNIT_IMPERIALS
} frskyUnit_e;
typedef struct telemetryConfig_s {
    uint8_t telemetry_switch;
    uint8_t telemetry_inversion;
    float gpsNoFixLatitude;
    float gpsNoFixLongitude;
    frskyGpsCoordFormat_e frsky_coordinate_format;
    frskyUnit_e frsky_unit;
    uint8_t frsky_vfas_precision;
    uint8_t hottAlarmSoundInterval;
} telemetryConfig_t;
void telemetryCheckState(void);
void telemetryProcess(rxConfig_t *rxConfig, uint16_t deadband3d_throttle);
bool telemetryDetermineEnabledState(portSharing_e portSharing);
void telemetryUseConfig(telemetryConfig_t *telemetryConfig);
#endif
