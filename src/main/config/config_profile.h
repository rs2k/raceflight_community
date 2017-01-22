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
       
typedef struct profile_s {
    pidProfile_t pidProfile;
    uint8_t defaultRateProfileIndex;
    int16_t mag_declination;
    rollAndPitchTrims_t accelerometerTrims;
    uint8_t acc_lpf_hz;
    float accz_lpf_cutoff;
    accDeadband_t accDeadband;
    barometerConfig_t barometerConfig;
    uint8_t acc_unarmedcal;
    modeActivationCondition_t modeActivationConditions[MAX_MODE_ACTIVATION_CONDITION_COUNT];
    adjustmentRange_t adjustmentRanges[MAX_ADJUSTMENT_RANGE_COUNT];
    rcControlsConfig_t rcControlsConfig;
    uint16_t throttle_correction_angle;
    uint8_t throttle_correction_value;
#ifdef USE_SERVOS
    servoParam_t servoConf[MAX_SUPPORTED_SERVOS];
    gimbalConfig_t gimbalConfig;
#endif
#ifdef GPS
    gpsProfile_t gpsProfile;
#endif
} profile_t;
