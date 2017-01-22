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
       
typedef enum {
    GYRO_NONE = 0,
    GYRO_DEFAULT,
    GYRO_MPU6050,
    GYRO_L3G4200D,
    GYRO_MPU3050,
    GYRO_L3GD20,
    GYRO_MPU6000,
    GYRO_MPU6500,
    GYRO_MPU9250,
    GYRO_FAKE
} gyroSensor_e;
extern gyro_t gyro;
extern sensor_align_e gyroAlign;
extern int16_t gyroADC[XYZ_AXIS_COUNT];
extern int16_t gyroZero[FLIGHT_DYNAMICS_INDEX_COUNT];
typedef struct gyroConfig_s {
    uint8_t gyroMovementCalibrationThreshold;
} gyroConfig_t;
void gyroSetCalibrationCycles(uint16_t calibrationCyclesRequired);
void gyroUpdate(void);
bool isGyroCalibrationComplete(void);
void useGyroConfig(gyroConfig_t *gyroConfigToUse, uint8_t gyro_lpf_hz);
