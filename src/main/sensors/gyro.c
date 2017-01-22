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
#include <math.h>
#include "debug.h"
#include "platform.h"
#include "debug.h"
#include "config/config.h"
#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"
#include "flight/pid.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/gyro_sync.h"
#include "sensors/sensors.h"
#include "io/beeper.h"
#include "io/statusindicator.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "include.h"
uint16_t calibratingG = 0;
int16_t gyroADC[XYZ_AXIS_COUNT];
int16_t gyroZero[FLIGHT_DYNAMICS_INDEX_COUNT] = { 0, 0, 0 };
static gyroConfig_t *gyroConfig;
static biquad_t gyroBiQuadState[3];
static biquad2_t gyroBiQuadState2[3];
static bool gyroFilterStateIsSet;
static uint16_t gyroLpfCutFreq;
int axis;
gyro_t gyro;
sensor_align_e gyroAlign = 0;
void useGyroConfig(gyroConfig_t *gyroConfigToUse, uint8_t gyro_lpf_hz)
{
    gyroConfig = gyroConfigToUse;
    (void)(gyro_lpf_hz);
}
void initGyroFilterCoefficients(void) {
 for (axis = 0; axis < 3; axis++) {
  if (axis == FD_ROLL) {
   gyroLpfCutFreq = currentProfile->pidProfile.wrgyrolpf;
  } else if (axis == FD_PITCH) {
   gyroLpfCutFreq = currentProfile->pidProfile.wpgyrolpf;
  } else if (axis == FD_YAW) {
   gyroLpfCutFreq = currentProfile->pidProfile.wygyrolpf;
  }
  if (gyroLpfCutFreq == 1) {
   if (gyroLpfCutFreq && targetLooptime) {
    BiQuadNewLpf2((uint16_t)(currentProfile->pidProfile.fcrap/3.0), &gyroBiQuadState2[axis], (float)targetESCwritetime * 0.000001f);
   }
  } else {
   if (gyroLpfCutFreq && targetLooptime) {
    BiQuadNewLpf(gyroLpfCutFreq, &gyroBiQuadState[axis], ((float)targetESCwritetime * 0.000001f));
   }
  }
 }
 gyroFilterStateIsSet = true;
}
void gyroSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    calibratingG = calibrationCyclesRequired;
}
bool isGyroCalibrationComplete(void)
{
    return calibratingG == 0;
}
bool isOnFinalGyroCalibrationCycle(void)
{
    return calibratingG == 1;
}
bool isOnFirstGyroCalibrationCycle(void)
{
    return calibratingG == CALIBRATING_GYRO_CYCLES;
}
static void performAcclerationCalibration(uint8_t gyroMovementCalibrationThreshold)
{
    int8_t axis;
    static int32_t g[3];
    static stdev_t var[3];
    for (axis = 0; axis < 3; axis++) {
        if (isOnFirstGyroCalibrationCycle()) {
            g[axis] = 0;
            devClear(&var[axis]);
        }
        g[axis] += gyroADC[axis];
        devPush(&var[axis], gyroADC[axis]);
        gyroADC[axis] = 0;
        gyroZero[axis] = 0;
        if (isOnFinalGyroCalibrationCycle()) {
            float dev = devStandardDeviation(&var[axis]);
            if (gyroMovementCalibrationThreshold && dev > gyroMovementCalibrationThreshold) {
                gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
                return;
            }
            gyroZero[axis] = (g[axis] + (CALIBRATING_GYRO_CYCLES / 2)) / CALIBRATING_GYRO_CYCLES;
        }
    }
    if (isOnFinalGyroCalibrationCycle()) {
        beeper(BEEPER_GYRO_CALIBRATED);
    }
    calibratingG--;
}
static void applyGyroZero(void)
{
    int8_t axis;
    for (axis = 0; axis < 3; axis++) {
        gyroADC[axis] -= gyroZero[axis];
    }
}
void gyroUpdate(void)
{
    if (!gyro.read(gyroADC)) {
        return;
    }
    if (!gyroFilterStateIsSet) {
     initGyroFilterCoefficients();
    }
    for (axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
     if (axis == FD_ROLL) {
   gyroLpfCutFreq = currentProfile->pidProfile.wrgyrolpf;
  } else if (axis == FD_PITCH) {
   gyroLpfCutFreq = currentProfile->pidProfile.wpgyrolpf;
  } else if (axis == FD_YAW) {
   gyroLpfCutFreq = currentProfile->pidProfile.wygyrolpf;
  }
  if (gyroLpfCutFreq) {
   if (IS_RC_MODE_ACTIVE(BOXPROSMOOTH)) {
    if (gyroLpfCutFreq == 1) {
     gyroADC[axis] = (int16_t)((applyBiQuadFilter2( (double)gyroShare[axis], &gyroBiQuadState2[axis])) / (double)(1.5f));
    } else {
     gyroADC[axis] = (int16_t)((applyBiQuadFilter( (float)gyroShare[axis], &gyroBiQuadState[axis])) / 1.5f);
    }
   } else {
    if (gyroLpfCutFreq == 1) {
     gyroADC[axis] = (int16_t)(applyBiQuadFilter2( (double)gyroADC[axis], &gyroBiQuadState2[axis] ) );
    } else {
     gyroADC[axis] = (int16_t)(applyBiQuadFilter( (float)gyroADC[axis], &gyroBiQuadState[axis] ) );
    }
   }
  } else if (IS_RC_MODE_ACTIVE(BOXPROSMOOTH)) {
   gyroADC[axis] = (int16_t)(gyroShare[axis]/1.5f);
  }
    }
    alignSensors(gyroADC, gyroADC, gyroAlign);
    if (!isGyroCalibrationComplete()) {
        performAcclerationCalibration(gyroConfig->gyroMovementCalibrationThreshold);
    }
    applyGyroZero();
}
