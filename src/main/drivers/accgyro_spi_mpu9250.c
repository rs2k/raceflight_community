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
#include "light_led.h"
#include "common/axis.h"
#include "common/maths.h"
#include "io.h"
#include "system.h"
#include "exti.h"
#include "bus_spi.h"
#include "gyro_sync.h"
#include "debug.h"
#include "sensor.h"
#include "accgyro.h"
#include "accgyro_mpu.h"
#include "accgyro_spi_mpu9250.h"
static void mpu9250AccAndGyroInit(uint8_t lpf);
static bool mpuSpi9250InitDone = false;
static IO_t mpuSpi9250CsPin = IO_NONE;
#define DISABLE_MPU9250 IOHi(mpuSpi9250CsPin)
#define ENABLE_MPU9250 IOLo(mpuSpi9250CsPin)
void resetGyro (void) {
    mpu9250WriteRegister(MPU_RA_PWR_MGMT_1, MPU9250_BIT_RESET);
    delay(150);
}
bool mpu9250WriteRegister(uint8_t reg, uint8_t data)
{
 ENABLE_MPU9250;
    delayMicroseconds(1);
    spiTransferByte(MPU9250_SPI_INSTANCE, reg);
    spiTransferByte(MPU9250_SPI_INSTANCE, data);
    DISABLE_MPU9250;
    delayMicroseconds(1);
    return true;
}
bool mpu9250ReadRegister(uint8_t reg, uint8_t length, uint8_t *data)
{
 ENABLE_MPU9250;
    spiTransferByte(MPU9250_SPI_INSTANCE, reg | 0x80);
    spiTransfer(MPU9250_SPI_INSTANCE, data, NULL, length);
    DISABLE_MPU9250;
    return true;
}
bool mpu9250SlowReadRegister(uint8_t reg, uint8_t length, uint8_t *data)
{
 ENABLE_MPU9250;
    delayMicroseconds(1);
    spiTransferByte(MPU9250_SPI_INSTANCE, reg | 0x80);
    spiTransfer(MPU9250_SPI_INSTANCE, data, NULL, length);
    DISABLE_MPU9250;
    delayMicroseconds(1);
    return true;
}
void mpu9250SpiGyroInit(uint8_t lpf)
{
 (void)(lpf);
    mpuIntExtiInit();
    mpu9250AccAndGyroInit(lpf);
    spiResetErrorCounter(MPU9250_SPI_INSTANCE);
#if defined(SLOW_SPI_DOWN)
 spiSetDivisor(MPU9250_SPI_INSTANCE, SPI_FAST_CLOCK);
#else
 spiSetDivisor(MPU9250_SPI_INSTANCE, SPI_FAST_CLOCK);
#endif
    int16_t data[3];
    mpuGyroRead(data);
    if ((((int8_t)data[1]) == -1 && ((int8_t)data[0]) == -1) || spiGetErrorCounter(MPU9250_SPI_INSTANCE) != 0) {
        spiResetErrorCounter(MPU9250_SPI_INSTANCE);
        failureMode(FAILURE_GYRO_INIT_FAILED);
    }
}
void mpu9250SpiAccInit(void)
{
    mpuIntExtiInit();
    acc_1G = 256 * 8;
}
bool verifympu9250WriteRegister(uint8_t reg, uint8_t data) {
 uint8_t in;
 uint8_t attemptsRemaining = 20;
 mpu9250WriteRegister(reg, data);
 delayMicroseconds(100);
    do {
     mpu9250SlowReadRegister(reg, 1, &in);
     if (in == data) {
      return true;
     } else {
      mpu9250WriteRegister(reg, data);
      delayMicroseconds(100);
     }
    } while (attemptsRemaining--);
    return false;
}
void mpu9250AccAndGyroReInit(uint8_t lpf) {
 mpuSpi9250InitDone = false;
 mpu9250AccAndGyroInit(lpf);
}
static void mpu9250AccAndGyroInit(uint8_t lpf) {
 if (mpuSpi9250InitDone) {
  return;
 }
    spiSetDivisor(MPU9250_SPI_INSTANCE, SPI_SLOW_CLOCK);
 mpu9250WriteRegister(0x6B, 0x80);
 delay(25);
 delay(25);
 delay(25);
 delay(25);
 delay(25);
 delay(25);
 delay(25);
 delay(25);
 mpu9250WriteRegister(0x70, 0 << 7 | 1 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);
 mpu9250WriteRegister(0x6B, 0x01);
    if (lpf >= DLPF_UH1) {
     mpu9250WriteRegister(MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3 | FCB_8800_32);
    } else if (lpf >= DLPF_H16) {
     mpu9250WriteRegister(MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3 | FCB_3600_32);
    } else {
     mpu9250WriteRegister(MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3 | FCB_DISABLED);
    }
    if (lpf == DLPF_L1) {
     mpu9250WriteRegister(MPU_RA_CONFIG, 1);
    } else if (lpf <= DLPF_M8) {
     mpu9250WriteRegister(MPU_RA_CONFIG, 0);
    } else if (lpf <= DLPF_H8) {
     mpu9250WriteRegister(MPU_RA_CONFIG, 7);
    }
    mpu9250WriteRegister(MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);
    mpu9250WriteRegister(MPU_RA_FF_THR, 0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 1 << 3 | 0 << 2 | 0 << 1 | 0 << 0);
 mpu9250WriteRegister(MPU_RA_SMPLRT_DIV, 0);
 mpu9250WriteRegister(MPU_RA_FIFO_EN, 0);
 mpu9250WriteRegister(MPU_RA_INT_PIN_CFG, 24);
#ifdef FAKE_EXTI
 mpu9250WriteRegister(MPU_RA_INT_ENABLE, 0);
#else
 mpu9250WriteRegister(MPU_RA_INT_ENABLE, 1);
#endif
 mpu9250WriteRegister(MPU_RA_SIGNAL_PATH_RESET, 3);
    mpuSpi9250InitDone = true;
}
bool mpu9250SpiDetect(void)
{
    uint8_t in;
    uint8_t attemptsRemaining = 20;
 uint8_t gyroResetReg = MPU9250_BIT_RESET;
#ifdef MPU9250_CS_PIN
    mpuSpi9250CsPin = IOGetByTag(IO_TAG(MPU9250_CS_PIN));
#endif
 IOInit(mpuSpi9250CsPin, OWNER_SYSTEM, RESOURCE_SPI);
 IOConfigGPIO(mpuSpi9250CsPin, SPI_IO_CS_CFG);
    spiSetDivisor(MPU9250_SPI_INSTANCE, SPI_SLOW_CLOCK);
    mpu9250WriteRegister(MPU_RA_PWR_MGMT_1, MPU9250_BIT_RESET);
 LED0_ON;
 LED1_ON;
 LED2_ON;
 while (gyroResetReg == MPU9250_BIT_RESET) {
  mpu9250SlowReadRegister(MPU_RA_PWR_MGMT_1, 1, &gyroResetReg);
  delay(100);
  LED0_TOGGLE;
  LED1_TOGGLE;
  LED2_TOGGLE;
 }
 LED0_OFF;
 LED1_OFF;
 LED2_OFF;
    do {
        delay(150);
        mpu9250ReadRegister(MPU_RA_WHO_AM_I, 1, &in);
  if (in == MPU6500_WHO_AM_I_CONST || in == MPU9250_WHO_AM_I_CONST || in == ICM20608G_WHO_AM_I_CONST || in == MPU6555_WHO_AM_I_CONST || in == ICM20689_WHO_AM_I_CONST || in == ICM20602_WHO_AM_I_CONST) {
            break;
        }
        if (!attemptsRemaining) {
            return false;
        }
    } while (attemptsRemaining--);
    return true;
}
bool mpu9250SpiAccDetect(acc_t *acc)
{
    if (mpuDetectionResult.sensor != MPU_9250_SPI) {
        return false;
    }
    acc->init = mpu9250SpiAccInit;
    acc->read = mpuAccRead;
    return true;
}
bool mpu9250SpiGyroDetect(gyro_t *gyro)
{
    if (mpuDetectionResult.sensor != MPU_9250_SPI) {
        return false;
    }
    gyro->init = mpu9250SpiGyroInit;
    gyro->read = mpuGyroRead;
    gyro->scale = 1.0f / 16.4f;
    return true;
}
