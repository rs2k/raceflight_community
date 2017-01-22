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
       
#define mpu9250_CONFIG 0x1A
#define GYRO_SCALE_FACTOR 0.00053292f
#define MPU6500_WHO_AM_I_CONST (0x70)
#define MPU9250_WHO_AM_I_CONST (0x71)
#define MPU6500_WHO_AM_I_CONST (0x70)
#define MPU6555_WHO_AM_I_CONST (0x7C)
#define ICM20608G_WHO_AM_I_CONST (0xAF)
#define ICM20689_WHO_AM_I_CONST (0x98)
#define ICM20602_WHO_AM_I_CONST (0x12)
#define MPU9250_BIT_RESET (0x80)
#define MPU_RF_DATA_RDY_EN (1 << 0)
void resetGyro(void);
void mpu9250AccAndGyroReInit(uint8_t lpf);
bool mpu9250SpiDetect(void);
bool mpu9250SpiAccDetect(acc_t *acc);
bool mpu9250SpiGyroDetect(gyro_t *gyro);
bool mpu9250WriteRegister(uint8_t reg, uint8_t data);
bool verifympu9250WriteRegister(uint8_t reg, uint8_t data);
bool mpu9250ReadRegister(uint8_t reg, uint8_t length, uint8_t *data);
bool mpu9250SlowReadRegister(uint8_t reg, uint8_t length, uint8_t *data);
