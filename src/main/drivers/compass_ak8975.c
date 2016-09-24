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

#include <math.h>

#include "build_config.h"

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "system.h"
#include "gpio.h"
#include "bus_i2c.h"

#include "sensors/boardalignment.h"
#include "sensors/sensors.h"

#include "sensor.h"
#include "compass.h"

#include "compass_ak8975.h"


#ifndef AK8975_I2C_INSTANCE
#define AK8975_I2C_INSTANCE I2C_DEVICE
#endif


#define AK8975_MAG_I2C_ADDRESS      0x0C



#define AK8975_MAG_REG_WHO_AM_I     0x00
#define AK8975_MAG_REG_INFO         0x01
#define AK8975_MAG_REG_STATUS1      0x02
#define AK8975_MAG_REG_HXL          0x03
#define AK8975_MAG_REG_HXH          0x04
#define AK8975_MAG_REG_HYL          0x05
#define AK8975_MAG_REG_HYH          0x06
#define AK8975_MAG_REG_HZL          0x07
#define AK8975_MAG_REG_HZH          0x08
#define AK8975_MAG_REG_STATUS2      0x09
#define AK8975_MAG_REG_CNTL         0x0a
#define AK8975_MAG_REG_ASCT         0x0c 

bool ak8975detect(mag_t *mag)
{
    bool ack = false;
    uint8_t sig = 0;

    ack = i2cRead(AK8975_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_WHO_AM_I, 1, &sig);
    if (!ack || sig != 'H') 
        return false;

    mag->init = ak8975Init;
    mag->read = ak8975Read;

    return true;
}

#define AK8975A_ASAX 0x10 
#define AK8975A_ASAY 0x11 
#define AK8975A_ASAZ 0x12 

void ak8975Init()
{
    bool ack;
    uint8_t buffer[3];
    uint8_t status;

    UNUSED(ack);

    ack = i2cWrite(AK8975_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_CNTL, 0x00); 
    delay(20);

    ack = i2cWrite(AK8975_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_CNTL, 0x0F); 
    delay(10);

    ack = i2cRead(AK8975_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975A_ASAX, 3, &buffer[0]); 
    delay(10);

    ack = i2cWrite(AK8975_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_CNTL, 0x00); 
    delay(10);

    
    ack = i2cRead(AK8975_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_STATUS1, 1, &status);
    ack = i2cRead(AK8975_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_STATUS2, 1, &status);

    
    ack = i2cWrite(AK8975_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_CNTL, 0x01);
}

#define BIT_STATUS1_REG_DATA_READY              (1 << 0)

#define BIT_STATUS2_REG_DATA_ERROR              (1 << 2)
#define BIT_STATUS2_REG_MAG_SENSOR_OVERFLOW     (1 << 3)

bool ak8975Read(int16_t *magData)
{
    bool ack;
    UNUSED(ack);
    uint8_t status;
    uint8_t buf[6];

    ack = i2cRead(AK8975_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_STATUS1, 1, &status);
    if (!ack || (status & BIT_STATUS1_REG_DATA_READY) == 0) {
        return false;
    }

#if 1 
    ack = i2cRead(AK8975_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_HXL, 6, buf); 
#else
    for (uint8_t i = 0; i < 6; i++) {
        ack = i2cRead(AK8975_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_HXL + i, 1, &buf[i]); 
        if (!ack) {
            return false
        }
    }
#endif

    ack = i2cRead(AK8975_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_STATUS2, 1, &status);
    if (!ack) {
        return false;
    }

    if (status & BIT_STATUS2_REG_DATA_ERROR) {
        return false;
    }

    if (status & BIT_STATUS2_REG_MAG_SENSOR_OVERFLOW) {
        return false;
    }

    magData[X] = -(int16_t)(buf[1] << 8 | buf[0]) * 4;
    magData[Y] = -(int16_t)(buf[3] << 8 | buf[2]) * 4;
    magData[Z] = -(int16_t)(buf[5] << 8 | buf[4]) * 4;


    ack = i2cWrite(AK8975_I2C_INSTANCE, AK8975_MAG_I2C_ADDRESS, AK8975_MAG_REG_CNTL, 0x01); 
    return true;
}
