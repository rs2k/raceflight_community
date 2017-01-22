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
#include "platform.h"
#include "build_config.h"
#include "debug.h"
#include "common/axis.h"
#include "common/maths.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/gyro_sync.h"
#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "config/runtime_config.h"
#include "config/config.h"
uint32_t targetLooptime;
uint32_t targetESCwritetime;
uint8_t mpuDividerDrops;
uint8_t ESCWriteDenominator;
uint8_t accDenominator;
uint32_t gyroSamplePeriod = 0;
void gyroUpdateSampleRate(uint8_t lpf) {
    uint8_t gyroSyncDenominator;
    switch (lpf) {
        case DLPF_L1:
            gyroSamplePeriod = 1000;
            gyroSyncDenominator = 1;
            break;
        case DLPF_M1:
        case DLPF_H1:
            gyroSamplePeriod = 125;
            gyroSyncDenominator = 8;
            break;
        case DLPF_M2:
        case DLPF_H2:
            gyroSamplePeriod = 125;
            gyroSyncDenominator = 4;
            break;
        case DLPF_M4:
        case DLPF_H4:
            gyroSamplePeriod = 125;
            gyroSyncDenominator = 2;
            break;
        case DLPF_M8:
        case DLPF_H8:
            gyroSamplePeriod = 125;
            gyroSyncDenominator = 1;
            break;
#if defined(USE_GYRO_SPI_MPU9250)
        case DLPF_UH1:
            gyroSamplePeriod = 31;
            gyroSyncDenominator = 32;
            break;
        case DLPF_UH2:
            gyroSamplePeriod = 31;
            gyroSyncDenominator = 16;
            break;
        case DLPF_UH4:
            gyroSamplePeriod = 31;
            gyroSyncDenominator = 8;
            break;
        case DLPF_UH8:
            gyroSamplePeriod = 31;
            gyroSyncDenominator = 4;
            break;
        case DLPF_H16:
        case DLPF_UH16:
            gyroSamplePeriod = 31;
            gyroSyncDenominator = 2;
        case DLPF_H32:
        case DLPF_UH32:
            gyroSamplePeriod = 31;
            gyroSyncDenominator = 1;
            break;
#endif
        default:
            gyroSamplePeriod = 125;
            gyroSyncDenominator = 1;
            break;
    }
#if defined(USE_GYRO_SPI_6000) || defined(USE_GYRO_SPI_MPU9250)
    ESCWriteDenominator = gyroSyncDenominator;
    gyroSyncDenominator = 1;
#else
    ESCWriteDenominator = 1;
#endif
    mpuDividerDrops = gyroSyncDenominator - 1;
    targetLooptime = gyroSyncDenominator * gyroSamplePeriod;
    targetESCwritetime = gyroSamplePeriod * gyroSyncDenominator * ESCWriteDenominator;
    accDenominator = (4000 / targetLooptime / 2);
    if (accDenominator == 0) {
        accDenominator = 1;
    }
}
