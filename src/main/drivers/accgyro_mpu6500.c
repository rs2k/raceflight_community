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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "system.h"
#include "exti.h"
#include "gpio.h"
#include "gyro_sync.h"

#include "sensor.h"
#include "debug.h"
#include "accgyro.h"
#include "accgyro_mpu.h"
#include "accgyro_mpu6500.h"

extern uint16_t acc_1G;

#define BIT_I2C_IF_DIS              0x10

void resetGyro (void) {
    
    mpuConfiguration.write(MPU_RA_PWR_MGMT_1, MPU6500_BIT_RESET);
    delay(100);
}

bool mpu6500AccDetect(acc_t *acc)
{
    if (mpuDetectionResult.sensor != MPU_65xx_I2C) {
        return false;
    }

    acc->init = mpu6500AccInit;
    acc->read = mpuAccRead;

    return true;
}

bool mpu6500GyroDetect(gyro_t *gyro)
{
    if (mpuDetectionResult.sensor != MPU_65xx_I2C) {
        return false;
    }

    gyro->init = mpu6500GyroInit;
    gyro->read = mpuGyroRead;

    
    gyro->scale = 1.0f / 16.4f;

    return true;
}

void mpu6500AccInit(void)
{
    mpuIntExtiInit();

    acc_1G = 256 * 8;
}

void mpu6500GyroInit(uint8_t lpf)
{
    mpuIntExtiInit();

#ifdef NAZE
    

    gpio_config_t gpio;
    
    if (hse_value == 12000000) {
        gpio.pin = Pin_13;
        gpio.speed = Speed_2MHz;
        gpio.mode = Mode_IN_FLOATING;
        gpioInit(GPIOC, &gpio);
    }
#endif

    mpuIntExtiInit();

    mpuConfiguration.write(MPU_RA_PWR_MGMT_1, MPU6500_BIT_RESET);
	delay(50);

    mpuConfiguration.write(MPU_RA_PWR_MGMT_1, INV_CLK_PLL);
    delayMicroseconds(1);

    mpuConfiguration.write(MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3 | FCB_DISABLED); 
    delay(15);
    mpuConfiguration.write(MPU_RA_ACCEL_CONFIG, INV_FSR_8G << 3);
    delay(15);

    if (lpf == 4) {
    	mpuConfiguration.write(MPU_RA_CONFIG, 1); 
    } else if (lpf < 4) {
    	mpuConfiguration.write(MPU_RA_CONFIG, 7); 
    } else if (lpf > 4) {
    	mpuConfiguration.write(MPU_RA_CONFIG, 0); 
    }

    delay(15);
    mpuConfiguration.write(MPU_RA_SMPLRT_DIV, mpuDividerDrops); 
    delay(15);

    mpuConfiguration.write(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  
    delayMicroseconds(1);

#if defined(USE_MPU_DATA_READY_SIGNAL)
    mpuConfiguration.write(MPU_RA_INT_ENABLE, 0x01); 
    delayMicroseconds(1);
#endif
}
