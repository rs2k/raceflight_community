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

#define PITCH_LOOKUP_LENGTH 31
#define ROLL_LOOKUP_LENGTH 31
#define YAW_LOOKUP_LENGTH 31
#define THROTTLE_LOOKUP_LENGTH 12
extern float lookupPitchRC[PITCH_LOOKUP_LENGTH];   
extern float lookupRollRC[PITCH_LOOKUP_LENGTH];   
extern float lookupYawRC[YAW_LOOKUP_LENGTH];     
extern float lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];   

void generatePitchCurve(controlRateConfig_t *controlRateConfig);
void generateRollCurve(controlRateConfig_t *controlRateConfig);
void generateYawCurve(controlRateConfig_t *controlRateConfig);
void generateThrottleCurve(controlRateConfig_t *controlRateConfig, escAndServoConfig_t *escAndServoConfig);
