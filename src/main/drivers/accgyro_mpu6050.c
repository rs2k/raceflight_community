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
#include "common/maths.h"
#include "nvic.h"
#include "system.h"
#include "gpio.h"
#include "exti.h"
#include "bus_i2c.h"
#include "gyro_sync.h"
#include "sensor.h"
#include "accgyro.h"
#include "accgyro_mpu.h"
#include "accgyro_mpu6050.h"
extern uint8_t mpuLowPassFilter;
#define MPU6050_ADDRESS 0x68
#define DMP_MEM_START_ADDR 0x6E
#define DMP_MEM_R_W 0x6F
#define MPU6050_SMPLRT_DIV 0
static void mpu6050AccInit(void);
static void mpu6050GyroInit(uint8_t lpf);
bool mpu6050AccDetect(acc_t *acc)
{
    if (mpuDetectionResult.sensor != MPU_60x0) {
        return false;
    }
    acc->init = mpu6050AccInit;
    acc->read = mpuAccRead;
    acc->revisionCode = (mpuDetectionResult.resolution == MPU_HALF_RESOLUTION ? 'o' : 'n');
    return true;
}
bool mpu6050GyroDetect(gyro_t *gyro)
{
    if (mpuDetectionResult.sensor != MPU_60x0) {
        return false;
    }
    gyro->init = mpu6050GyroInit;
    gyro->read = mpuGyroRead;
    gyro->scale = 1.0f / 16.4f;
    return true;
}
static void mpu6050AccInit(void)
{
    mpuIntExtiInit();
    switch (mpuDetectionResult.resolution) {
        case MPU_HALF_RESOLUTION:
            acc_1G = 256 * 8;
            break;
        case MPU_FULL_RESOLUTION:
            acc_1G = 256 * 8;
            break;
    }
}
static void mpu6050GyroInit(uint8_t lpf)
{
    bool ack;
    mpuIntExtiInit();
    ack = mpuConfiguration.write(MPU_RA_PWR_MGMT_1, 0x80);
    delay(100);
    ack = mpuConfiguration.write(MPU_RA_PWR_MGMT_1, 0x03);
    ack = mpuConfiguration.write(MPU_RA_SMPLRT_DIV, mpuDividerDrops);
    delay(15);
    if (lpf == DLPF_L1) {
        mpuConfiguration.write(MPU_RA_CONFIG, 1);
    } else if (lpf <= DLPF_M8) {
        mpuConfiguration.write(MPU_RA_CONFIG, 0);
    } else {
        mpuConfiguration.write(MPU_RA_CONFIG, 7);
    }
    delay(15);
    ack = mpuConfiguration.write(MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
    delay(15);
    ack = mpuConfiguration.write(MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);
    delay(15);
    ack = mpuConfiguration.write(MPU_RA_INT_PIN_CFG,
            0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);
    delay(15);
#ifdef USE_MPU_DATA_READY_SIGNAL
    ack = mpuConfiguration.write(MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);
#endif
    UNUSED(ack);
}
