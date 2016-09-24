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
#include <string.h>

#include "include.h"

#include "drivers/gyro_sync.h"

#undef DEBUG_MPU_DATA_READY_INTERRUPT

static bool mpuReadRegisterI2C(uint8_t reg, uint8_t length, uint8_t* data);
static bool mpuWriteRegisterI2C(uint8_t reg, uint8_t data);

static void mpu6050FindRevision(void);

#ifdef USE_SPI
static bool detectSPISensorsAndUpdateDetectionResult(void);
#endif

#ifndef MPU_I2C_INSTANCE
#define MPU_I2C_INSTANCE I2C_DEVICE
#endif
mpuDetectionResult_t mpuDetectionResult;

mpuConfiguration_t mpuConfiguration;
static const extiConfig_t *mpuIntExtiConfig = NULL;

extern bool init_done;
extern bool exti_has_happened;
int32_t gyroShare[3];

#define MPU_ADDRESS             0x68


#define MPU6500_WHO_AM_I_CONST              (0x70)
#define MPU6555_WHO_AM_I_CONST              (0x7C)
#define MPUx0x0_WHO_AM_I_CONST              (0x68)

#define MPU_INQUIRY_MASK   0x7E

#if defined( USE_GYRO_MPU9250 ) || defined( USE_GYRO_SPI_MPU9250 ) || defined( USE_GYRO_MPU6500 ) || defined( USE_GYRO_SPI_MPU6500 )
#define RING_BUFFER_SIZE 1
#else
#define RING_BUFFER_SIZE 1
#endif
int16_t gyroADC0_buffer[RING_BUFFER_SIZE];
int16_t gyroADC1_buffer[RING_BUFFER_SIZE];
int16_t gyroADC2_buffer[RING_BUFFER_SIZE];
uint8_t gyroADC_buffer_position = 0;

bool MPU_ISR_RUNNING = false;

mpuDetectionResult_t *detectMpu(const extiConfig_t *configToUse)
{
    memset(&mpuDetectionResult, 0, sizeof(mpuDetectionResult));
    memset(&mpuConfiguration, 0, sizeof(mpuConfiguration));

    mpuIntExtiConfig = configToUse;

    bool ack;
    uint8_t sig;
    uint8_t inquiryResult;

    
    delay(35);

#ifndef USE_I2C
    ack = false;
    sig = 0;
#else
    ack = mpuReadRegisterI2C(MPU_RA_WHO_AM_I, 1, &sig);
#endif
    if (ack) {
        mpuConfiguration.read = mpuReadRegisterI2C;
        mpuConfiguration.write = mpuWriteRegisterI2C;
        mpuConfiguration.slowread = mpuReadRegisterI2C;
        mpuConfiguration.verifywrite = mpuWriteRegisterI2C;
    } else {
#ifdef USE_SPI
        bool detectedSpiSensor = detectSPISensorsAndUpdateDetectionResult();
        UNUSED(detectedSpiSensor);
#endif

        return &mpuDetectionResult;
    }

    mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;

    
    ack = mpuReadRegisterI2C(MPU_RA_WHO_AM_I_LEGACY, 1, &inquiryResult);
    inquiryResult &= MPU_INQUIRY_MASK;
    if (ack && inquiryResult == MPUx0x0_WHO_AM_I_CONST) {
        mpuDetectionResult.sensor = MPU_3050;
        mpuConfiguration.gyroReadXRegister = MPU3050_GYRO_OUT;
        return &mpuDetectionResult;
    }

    sig &= MPU_INQUIRY_MASK;

    if (sig == MPUx0x0_WHO_AM_I_CONST) {

        mpuDetectionResult.sensor = MPU_60x0;

        mpu6050FindRevision();
    } else if (sig == MPU6500_WHO_AM_I_CONST) {
        mpuDetectionResult.sensor = MPU_65xx_I2C;
    }

    return &mpuDetectionResult;
}

#ifdef USE_SPI
static bool detectSPISensorsAndUpdateDetectionResult(void)
{
#ifdef USE_GYRO_SPI_MPU6500
    if (mpu6500SpiDetect()) {
        mpuDetectionResult.sensor = MPU_65xx_SPI;
        mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;
        mpuConfiguration.read = mpu6500ReadRegister;
        mpuConfiguration.slowread = mpu6500SlowReadRegister;
        mpuConfiguration.verifywrite = verifympu6500WriteRegister;
        mpuConfiguration.write = mpu6500WriteRegister;
        return true;
    }
#endif

#ifdef USE_GYRO_SPI_MPU6000
    if (mpu6000SpiDetect()) {
        mpuDetectionResult.sensor = MPU_60x0_SPI;
        mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;
        mpuConfiguration.read = mpu6000ReadRegister;
        mpuConfiguration.slowread = mpu6000SlowReadRegister;
        mpuConfiguration.verifywrite = verifympu6000WriteRegister;
        mpuConfiguration.write = mpu6000WriteRegister;
        return true;
    }
#endif

#ifdef USE_GYRO_SPI_MPU9250
    if (mpu9250SpiDetect()) {
        mpuDetectionResult.sensor = MPU_9250_SPI;
        mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;
        mpuConfiguration.read = mpu9250ReadRegister;
        mpuConfiguration.slowread = mpu9250SlowReadRegister;
        mpuConfiguration.verifywrite = verifympu9250WriteRegister;
        mpuConfiguration.write = mpu9250WriteRegister;
        return true;
    }
#endif

    return false;
}
#endif

static void mpu6050FindRevision(void)
{
    bool ack;
    UNUSED(ack);

    uint8_t readBuffer[6];
    uint8_t revision;
    uint8_t productId;

    
    

    
    ack = mpuConfiguration.read(MPU_RA_XA_OFFS_H, 6, readBuffer);
    revision = ((readBuffer[5] & 0x01) << 2) | ((readBuffer[3] & 0x01) << 1) | (readBuffer[1] & 0x01);
    if (revision) {
        /* Congrats, these parts are better. */
        if (revision == 1) {
            mpuDetectionResult.resolution = MPU_HALF_RESOLUTION;
        } else if (revision == 2) {
            mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        } else {
            failureMode(FAILURE_ACC_INCOMPATIBLE);
        }
    } else {
        ack = mpuConfiguration.read(MPU_RA_PRODUCT_ID, 1, &productId);
        revision = productId & 0x0F;
        if (!revision) {
            failureMode(FAILURE_ACC_INCOMPATIBLE);
        } else if (revision == 4) {
            mpuDetectionResult.resolution = MPU_HALF_RESOLUTION;
        } else {
            mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        }
    }
}

extiCallbackRec_t mpuIntCallbackRec;

void mpuIntExtiHandler(extiCallbackRec_t *cb)
{
	exti_has_happened = true;
	if ( (init_done == 0) || (SKIP_GYRO) )
	{
		if (!ARMING_FLAG(ARMED)) {
			return;
		}
	}

	static uint8_t counter_gyro = 0;
	UNUSED(cb);
	MainPidLoop();
	counter_gyro++;
	if (counter_gyro == accDenominator) { 
		counter_gyro = 0;
		UpdateAccelerometer();
	}

}

void mpuIntExtiInit(void)
{
    static bool mpuExtiInitDone = false;

    if (mpuExtiInitDone || !mpuIntExtiConfig) {
        return;
    }

#if defined(USE_MPU_DATA_READY_SIGNAL) && defined(USE_EXTI)

	IO_t mpuIntIO = IOGetByTag(mpuIntExtiConfig->io);

#ifdef ENSURE_MPU_DATA_READY_IS_LOW
	uint8_t status = IORead(mpuIntIO);
	if (status) {
		return;
	}
#endif

    IOInit(mpuIntIO, OWNER_SYSTEM, RESOURCE_INPUT | RESOURCE_EXTI);
	IOConfigGPIO(mpuIntIO, IOCFG_IN_FLOATING);   

	EXTIHandlerInit(&mpuIntCallbackRec, mpuIntExtiHandler);
	EXTIConfig(mpuIntIO, &mpuIntCallbackRec, NVIC_PRIO_MPU_INT_EXTI, EXTI_Trigger_Rising);
	EXTIEnable(mpuIntIO, true);
#endif

    mpuExtiInitDone = true;
}

static bool mpuReadRegisterI2C(uint8_t reg, uint8_t length, uint8_t* data)
{
#ifndef USE_I2C
    UNUSED(reg);
    UNUSED(data);
    UNUSED(length);
    return false;
#else
    bool ack = i2cRead(MPU_I2C_INSTANCE, MPU_ADDRESS, reg, length, data);
    return ack;
#endif
}

static bool mpuWriteRegisterI2C(uint8_t reg, uint8_t data)
{
#ifndef USE_I2C
    UNUSED(reg);
    UNUSED(data);
    return false;
#else
    bool ack = i2cWrite(MPU_I2C_INSTANCE, MPU_ADDRESS, reg, data);
    return ack;
#endif
}

bool mpuAccRead(int16_t *accData)
{
    uint8_t data[6];

    bool ack = mpuConfiguration.read(MPU_RA_ACCEL_XOUT_H, 6, data);
    if (!ack) {
        return false;
    }

    accData[0] = (int16_t)((data[0] << 8) | data[1]);
    accData[1] = (int16_t)((data[2] << 8) | data[3]);
    accData[2] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}

int16_t averageGyroADCbuffer(int16_t *gyroADC_buffer)
{
	int x;
	float adc0 = 0;
	for (x = 0; x < RING_BUFFER_SIZE; x++)
		adc0 = adc0 + (float)gyroADC_buffer[x];

	return (int16_t)(adc0 / (float)RING_BUFFER_SIZE) ;
}

bool mpuGyroRead(int16_t *gyroADC)
{
    uint8_t data[6];
    static gyroDataStore_t gyroData;

    bool ack = mpuConfiguration.read(mpuConfiguration.gyroReadXRegister, 6, data);
    if (!ack) {
        return false;
    }

	if (gyroADC_buffer_position == RING_BUFFER_SIZE)
		gyroADC_buffer_position = 0;

	if (IS_RC_MODE_ACTIVE(BOXPROSMOOTH)) {
		gyroShare[0] = (int32_t)( ( (int16_t)((data[0] << 8) | data[1]) + gyroData.a0 + gyroData.b0) / 2);
		gyroShare[1] = (int32_t)( ( (int16_t)((data[2] << 8) | data[3]) + gyroData.a1 + gyroData.b1) / 2);
		gyroShare[2] = (int32_t)( ( (int16_t)((data[4] << 8) | data[5]) + gyroData.a2 + gyroData.b2) / 2);

		gyroData.b0 = gyroData.a0;
		gyroData.b1 = gyroData.a1;
		gyroData.b2 = gyroData.a2;

		gyroData.a0 = (int16_t)((data[0] << 8) | data[1]);
		gyroData.a1 = (int16_t)((data[2] << 8) | data[3]);
		gyroData.a2 = (int16_t)((data[4] << 8) | data[5]);
	} else {
		gyroADC0_buffer[gyroADC_buffer_position] = (int16_t)((data[0] << 8) | data[1]);
		gyroADC1_buffer[gyroADC_buffer_position] = (int16_t)((data[2] << 8) | data[3]);
		gyroADC2_buffer[gyroADC_buffer_position++] = (int16_t)((data[4] << 8) | data[5]);

		gyroADC[0] = averageGyroADCbuffer(gyroADC0_buffer);
		gyroADC[1] = averageGyroADCbuffer(gyroADC1_buffer);
		gyroADC[2] = averageGyroADCbuffer(gyroADC2_buffer);
	}

	
	
	

	/*
	if (absf((float)gyroADC[0] - averageGyroADCbuffer(gyroADC0_buffer)) > 1000.0f)
	{
		gyroADC[0] = gyroADC0_buffer[gyroADC_buffer_position]; 
	}
	else
	{
		gyroADC0_buffer[gyroADC_buffer_position] = gyroADC[0];
	}

	if (absf((float)gyroADC[1] - averageGyroADCbuffer(gyroADC1_buffer)) > 1000.0f)
	{
		gyroADC[1] = gyroADC1_buffer[gyroADC_buffer_position]; 
	}
	else
	{
		gyroADC1_buffer[gyroADC_buffer_position] = gyroADC[1];
	}

	if (absf((float)gyroADC[2] - averageGyroADCbuffer(gyroADC2_buffer)) > 1000.0f)
	{
		gyroADC[2] = gyroADC2_buffer[gyroADC_buffer_position]; 
	}
	else
	{
		gyroADC2_buffer[gyroADC_buffer_position] = gyroADC[2];
	}

	gyroADC_buffer_position++;
	if (gyroADC_buffer_position == RING_BUFFER_SIZE)
		gyroADC_buffer_position = 0;
	*/

    return true;
}
