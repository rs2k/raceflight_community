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
#include "platform.h"
#include "debug.h"
#include "common/axis.h"
#include "common/maths.h"
#include "system.h"
#include "nvic.h"
#include "gpio.h"
#include "bus_i2c.h"
#include "light_led.h"
#include "drivers/exti.h"
#include "sensor.h"
#include "compass.h"
#include "sensors/sensors.h"
#include "compass_hmc5883l.h"
#ifndef HMC5883L_I2C_INSTANCE
#define HMC5883L_I2C_INSTANCE I2C_DEVICE
#endif
#define MAG_ADDRESS 0x1E
#define MAG_DATA_REGISTER 0x03
#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_X_SELF_TEST_GAUSS (+1.16f)
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16f)
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08f)
#define SELF_TEST_LOW_LIMIT (243.0f / 390.0f)
#define SELF_TEST_HIGH_LIMIT (575.0f / 390.0f)
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2
static float magGain[3] = { 1.0f, 1.0f, 1.0f };
static const hmc5883Config_t *hmc5883Config = NULL;
#ifdef USE_MAG_DATA_READY_SIGNAL
static IO_t intIO;
static extiCallbackRec_t hmc5883_extiCallbackRec;
void hmc5883_extiHandler(extiCallbackRec_t* cb)
{
    UNUSED(cb);
#undef DEBUG_MAG_DATA_READY_INTERRUPT
#ifdef DEBUG_MAG_DATA_READY_INTERRUPT
    static uint32_t lastCalledAt = 0;
    static int32_t callDelta = 0;
    uint32_t now = millis();
    callDelta = now - lastCalledAt;
    debug[0] = callDelta;
    lastCalledAt = now;
#endif
}
#endif
bool hmc5883lDetect(mag_t* mag, const hmc5883Config_t *hmc5883ConfigToUse)
{
    bool ack = false;
    uint8_t sig = 0;
    hmc5883Config = hmc5883ConfigToUse;
    ack = i2cRead(HMC5883L_I2C_INSTANCE, MAG_ADDRESS, 0x0A, 1, &sig);
    if (!ack || sig != 'H')
        return false;
    mag->init = hmc5883lInit;
    mag->read = hmc5883lRead;
    return true;
}
void hmc5883lInit(void)
{
    int16_t magADC[3];
    int i;
    int32_t xyz_total[3] = { 0, 0, 0 };
    bool bret = true;
#ifdef USE_MAG_DATA_READY_SIGNAL
    if (hmc5883Config && hmc5883Config->io) {
        intIO = IOGetByTag(hmc5883Config->io);
        IOInit(intIO, OWNER_SYSTEM, RESOURCE_INPUT | RESOURCE_EXTI);
        IOConfigGPIO(intIO, IOCFG_IN_FLOATING);
    }
#endif
    delay(50);
    i2cWrite(HMC5883L_I2C_INSTANCE, MAG_ADDRESS, HMC58X3_R_CONFA, 0x010 + HMC_POS_BIAS);
    i2cWrite(HMC5883L_I2C_INSTANCE, MAG_ADDRESS, HMC58X3_R_CONFB, 0x60);
    delay(100);
    hmc5883lRead(magADC);
    for (i = 0; i < 10; i++) {
        i2cWrite(HMC5883L_I2C_INSTANCE, MAG_ADDRESS, HMC58X3_R_MODE, 1);
        delay(50);
        hmc5883lRead(magADC);
        xyz_total[X] += magADC[X];
        xyz_total[Y] += magADC[Y];
        xyz_total[Z] += magADC[Z];
        if (-4096 >= MIN(magADC[X], MIN(magADC[Y], magADC[Z]))) {
            bret = false;
            break;
        }
        LED1_TOGGLE;
    }
    i2cWrite(HMC5883L_I2C_INSTANCE, MAG_ADDRESS, HMC58X3_R_CONFA, 0x010 + HMC_NEG_BIAS);
    for (i = 0; i < 10; i++) {
        i2cWrite(HMC5883L_I2C_INSTANCE, MAG_ADDRESS, HMC58X3_R_MODE, 1);
        delay(50);
        hmc5883lRead(magADC);
        xyz_total[X] -= magADC[X];
        xyz_total[Y] -= magADC[Y];
        xyz_total[Z] -= magADC[Z];
        if (-4096 >= MIN(magADC[X], MIN(magADC[Y], magADC[Z]))) {
            bret = false;
            break;
        }
        LED1_TOGGLE;
    }
    magGain[X] = fabsf(660.0f * HMC58X3_X_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[X]);
    magGain[Y] = fabsf(660.0f * HMC58X3_Y_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[Y]);
    magGain[Z] = fabsf(660.0f * HMC58X3_Z_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[Z]);
    i2cWrite(HMC5883L_I2C_INSTANCE, MAG_ADDRESS, HMC58X3_R_CONFA, 0x70);
    i2cWrite(HMC5883L_I2C_INSTANCE, MAG_ADDRESS, HMC58X3_R_CONFB, 0x20);
    i2cWrite(HMC5883L_I2C_INSTANCE, MAG_ADDRESS, HMC58X3_R_MODE, 0x00);
    delay(100);
    if (!bret) {
        magGain[X] = 1.0f;
        magGain[Y] = 1.0f;
        magGain[Z] = 1.0f;
    }
#ifdef USE_MAG_DATA_READY_SIGNAL
    do {
        if (!(hmc5883Config && intIO))
            break;
# ifdef ENSURE_MAG_DATA_READY_IS_HIGH
        if (!IORead(intIO))
            break;
# endif
        EXTIHandlerInit(&hmc5883_extiCallbackRec, hmc5883_extiHandler);
        EXTIConfig(intIO, &hmc5883_extiCallbackRec, NVIC_PRIO_MAG_INT_EXTI, EXTI_Trigger_Rising);
        EXTIEnable(intIO, true);
    } while (0);
#endif
}
bool hmc5883lRead(int16_t *magData)
{
    uint8_t buf[6];
    bool ack = i2cRead(HMC5883L_I2C_INSTANCE, MAG_ADDRESS, MAG_DATA_REGISTER, 6, buf);
    if (!ack) {
        return false;
    }
    magData[X] = (int16_t)(buf[0] << 8 | buf[1]) * magGain[X];
    magData[Z] = (int16_t)(buf[2] << 8 | buf[3]) * magGain[Z];
    magData[Y] = (int16_t)(buf[4] << 8 | buf[5]) * magGain[Y];
    return true;
}
