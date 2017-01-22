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
#include <string.h>
#include <math.h>
#include "platform.h"
#ifdef GTUNE
#include "common/axis.h"
#include "common/maths.h"
#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "config/config.h"
#include "blackbox/blackbox.h"
#include "io/rc_controls.h"
#include "config/runtime_config.h"
extern uint16_t cycleTime;
extern uint8_t motorCount;
static pidProfile_t *pidProfile;
static int16_t delay_cycles;
static int16_t time_skip[3];
static int16_t OldError[3], result_P64[3];
static int32_t AvgGyro[3];
static bool floatPID;
void updateDelayCycles(void)
{
    delay_cycles = -(((int32_t)pidProfile->gtune_settle_time * 1000) / cycleTime);
}
void init_Gtune(pidProfile_t *pidProfileToTune)
{
    uint8_t i;
    pidProfile = pidProfileToTune;
 if (pidProfile->pidController == 2) {
     floatPID = true;
 } else {
     floatPID = false;
 }
 updateDelayCycles();
 for (i = 0; i < 3; i++) {
        if ((pidProfile->gtune_hilimP[i] && pidProfile->gtune_lolimP[i] > pidProfile->gtune_hilimP[i]) || (motorCount < 4 && i == FD_YAW)) {
            pidProfile->gtune_hilimP[i] = 0;
        }
        if (floatPID) {
            if((pidProfile->P_f[i] * 10.0f) < pidProfile->gtune_lolimP[i]) {
                pidProfile->P_f[i] = (float)(pidProfile->gtune_lolimP[i] / 10.0f);
            }
            result_P64[i] = (int16_t)pidProfile->P_f[i] << 6;
        } else {
            if(pidProfile->P8[i] < pidProfile->gtune_lolimP[i]) {
                pidProfile->P8[i] = pidProfile->gtune_lolimP[i];
            }
            result_P64[i] = (int16_t)pidProfile->P8[i] << 6;
        }
        OldError[i] = 0;
        time_skip[i] = delay_cycles;
    }
}
void calculate_Gtune(uint8_t axis)
{
    int16_t error, diff_G, threshP;
    if(rcCommandUsed[axis] || (axis != FD_YAW && (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)))) {
        OldError[axis] = 0;
        time_skip[axis] = delay_cycles;
    } else {
        if (!time_skip[axis]) AvgGyro[axis] = 0;
        time_skip[axis]++;
        if (time_skip[axis] > 0) {
            if (axis == FD_YAW) {
                AvgGyro[axis] += 32 * ((int16_t)gyroADC[axis] / 32);
            } else {
                AvgGyro[axis] += 128 * ((int16_t)gyroADC[axis] / 128);
            }
        }
        if (time_skip[axis] == pidProfile->gtune_average_cycles) {
            AvgGyro[axis] /= time_skip[axis];
            time_skip[axis] = 0;
            if (axis == FD_YAW) {
                threshP = 20;
                error = -AvgGyro[axis];
            } else {
                threshP = 10;
                error = AvgGyro[axis];
            }
            if (pidProfile->gtune_hilimP[axis] && error && OldError[axis] && error != OldError[axis]) {
                diff_G = ABS(error) - ABS(OldError[axis]);
                if ((error > 0 && OldError[axis] > 0) || (error < 0 && OldError[axis] < 0)) {
                    if (diff_G > threshP) {
                        if (axis == FD_YAW) {
                            result_P64[axis] += 256 + pidProfile->gtune_pwr;
                        } else {
                            result_P64[axis] += 64 + pidProfile->gtune_pwr;
                        }
                    } else {
                        if (diff_G < -threshP) {
                            if (axis == FD_YAW) {
                                result_P64[axis] -= 64 + pidProfile->gtune_pwr;
                            } else {
                                result_P64[axis] -= 32;
                            }
                        }
                    }
                } else {
                    if (ABS(diff_G) > threshP && axis != FD_YAW) {
                        result_P64[axis] -= 32;
                    }
                }
                int16_t newP = constrain((result_P64[axis] >> 6), (int16_t)pidProfile->gtune_lolimP[axis], (int16_t)pidProfile->gtune_hilimP[axis]);
#ifdef BLACKBOX
                if (feature(FEATURE_BLACKBOX)) {
                    flightLogEvent_gtuneCycleResult_t eventData;
                    eventData.gtuneAxis = axis;
                    eventData.gtuneGyroAVG = AvgGyro[axis];
                    eventData.gtuneNewP = newP;
                    blackboxLogEvent(FLIGHT_LOG_EVENT_GTUNE_RESULT, (flightLogEventData_t*)&eventData);
                }
#endif
                if (floatPID) {
                    pidProfile->P_f[axis] = (float)newP / 10.0f;
                } else {
                    pidProfile->P8[axis] = newP;
                }
            }
            OldError[axis] = error;
        }
    }
}
#endif
