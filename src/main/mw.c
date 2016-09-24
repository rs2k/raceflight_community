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
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "platform.h"
#include "scheduler.h"
#include "debug.h"
#include "watchdog.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"
#include "common/filter.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/light_led.h"

#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/gyro_sync.h"
#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"

#include "io/beeper.h"
#include "io/display.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_cli.h"
#include "io/serial_msp.h"
#include "io/statusindicator.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/gtune.h"
#include "flight/navigation.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"
#include "drivers/ws2812_led.h"



enum {
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};


/* VBAT monitoring interval (in microseconds) */
#define VBATINTERVAL 20000
/* IBat monitoring interval (in microseconds) - 6 default looptimes */
#define IBATINTERVAL (6 * 3500)

#define GYRO_WATCHDOG_DELAY 100 

bool FullKiLatched = false;
bool firstTimeArm = true;
uint32_t timeArmedAt = 0;


#define ERROR_RESET_DEACTIVATE_DELAY (1 * 1000)   
static bool ResetErrorActivated = true;

uint16_t cycleTime = 0;         

float dT;

int32_t reading_flash_timer;

int16_t magHold;
int16_t headFreeModeHold;

uint8_t motorControlEnable = false;

int16_t telemTemperature1;      
static uint32_t disarmAt;     

extern uint32_t currentTime;
bool calibrate_gyro = true;

static bool isRXDataNew;

typedef void (*pidControllerFuncPtr)(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, rollAndPitchTrims_t *angleTrim, rxConfig_t *rxConfig);            

extern pidControllerFuncPtr pid_controller;

void applyAndSaveAccelerometerTrimsDelta(rollAndPitchTrims_t *rollAndPitchTrimsDelta)
{
    currentProfile->accelerometerTrims.values.roll += rollAndPitchTrimsDelta->values.roll;
    currentProfile->accelerometerTrims.values.pitch += rollAndPitchTrimsDelta->values.pitch;

    saveConfigAndNotify();
}

#ifdef GTUNE

void updateGtuneState(void)
{
    static bool GTuneWasUsed = false;

    if (IS_RC_MODE_ACTIVE(BOXGTUNE)) {
        if (!FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
            ENABLE_FLIGHT_MODE(GTUNE_MODE);
            init_Gtune(&currentProfile->pidProfile);
            GTuneWasUsed = true;
        }
        if (!FLIGHT_MODE(GTUNE_MODE) && !ARMING_FLAG(ARMED) && GTuneWasUsed) {
            saveConfigAndNotify();
            GTuneWasUsed = false;
        }
    } else {
        if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
            DISABLE_FLIGHT_MODE(GTUNE_MODE);
        }
    }
}
#endif

bool isCalibrating()
{
#ifdef BARO
    if (sensors(SENSOR_BARO) && !isBaroCalibrationComplete()) {
        return true;
    }
#endif

    

    return (!isAccelerationCalibrationComplete() && sensors(SENSOR_ACC)) || (!isGyroCalibrationComplete());
}

void filterRc(void){
    static float lastCommand[4] = { 0, 0, 0, 0 };
    static float deltaRC[4] = { 0, 0, 0, 0 };
    static int16_t rcInterpolationFactor;
    static int16_t factor = 0;
    uint16_t rxRefreshRate;

    
    initRxRefreshRate(&rxRefreshRate);

    rcInterpolationFactor = (int16_t)rxRefreshRate / (int16_t)targetESCwritetime + 1; 

    if (isRXDataNew) {
        for (int channel=0; channel < 4; channel++) {
            deltaRC[channel] = rcCommand[channel] -  (lastCommand[channel] - deltaRC[channel] * (float)factor / (float)rcInterpolationFactor);
            lastCommand[channel] = rcCommand[channel];
        }
        isRXDataNew = false;
        factor = rcInterpolationFactor - 1;
    } else {
    	factor--;
    }

    
    if (factor > 0) {
    	for (int channel=0; channel < 4; channel++) {
    		rcCommandUsed[channel] = lastCommand[channel] - deltaRC[channel] * (float)factor / (float)rcInterpolationFactor;
    	}
    } else {
    	factor = 0;
    }

}

void scaleRcCommandToFpvCamAngle(void) {
    
    static uint8_t lastFpvCamAngleDegrees = 0;
    static float cosFactor = 1.0;
    static float sinFactor = 0.0;

    if (lastFpvCamAngleDegrees != masterConfig.rxConfig.fpvCamAngleDegrees){
        lastFpvCamAngleDegrees = masterConfig.rxConfig.fpvCamAngleDegrees;
        cosFactor = cos_approx(masterConfig.rxConfig.fpvCamAngleDegrees * RAD);
        sinFactor = sin_approx(masterConfig.rxConfig.fpvCamAngleDegrees * RAD);
    }

    int16_t roll = rcCommand[ROLL];
    int16_t yaw = rcCommand[YAW];
    rcCommand[ROLL] = constrain(roll * cosFactor -  yaw * sinFactor, -500, 500);
    rcCommand[YAW]  = constrain(yaw  * cosFactor + roll * sinFactor, -500, 500);
}

/**
 *  Multi-Point curve structure
 *	Used for curve fitting inputs
 */
typedef struct {
	int numberOfPoints;
	float *curveValues;
	float *curveLocations;
} MultiPointCurve;

/**
 *  Global TPA curve
 */
MultiPointCurve tpaCurve;

/**
 *  This gets the desired attenuation given an input
 *
 *  @param curve an input curve
 *  @param input 0.0 - 1.0
 *
 *  @return returns the desired attenuation given a curve, or FLT_MAX as an error
 */
float getValueFromMultiPointCurve(MultiPointCurve *curve, float input) {
	float lastCurvePoint = curve->curveValues[curve->numberOfPoints-1];
	
	float curvePointA = curve->curveLocations[0];
	float valueA = curve->curveValues[0];
	
	float curvePointB = curve->curveLocations[1];
	float valueB = curve->curveValues[1];
	
	float localizedInputRange;
	float normalizedInput;
	
	float magnitude;
	float value;
	
	if (input <= curve->curveLocations[0]) {
		return curve->curveValues[0];
	}
	
	if (input >= lastCurvePoint) {
		return lastCurvePoint;
	}
	
	for (int curveIndex = 0; curveIndex < curve->numberOfPoints - 1; curveIndex++) {
		curvePointA = curve->curveLocations[curveIndex];
		valueA = curve->curveValues[curveIndex];
		curvePointB = curve->curveLocations[curveIndex+1];
		valueB = curve->curveValues[curveIndex+1];
		
		if (input >= curvePointA && input <= curvePointB) {
			localizedInputRange = curvePointB - curvePointA;
			normalizedInput = (input - curvePointA) / localizedInputRange;
			
			magnitude = valueB - valueA;
			value = normalizedInput * magnitude + valueA;
			
			return value;
		}
	}
	
	
	return 0.0f;
}

/**
 *  Calculates the PID attenuation factor
 *
 *  @return scalar represented as 0-100
 */
int32_t applyLegacyTPA(void) {
	uint16_t range;
	uint16_t location;
	uint16_t normalizedLocation;
	
	if (rcData[THROTTLE] > currentControlRateProfile->tpa_breakpoint) {
		if (rcData[THROTTLE] < 2000) {
			range = (2000 - currentControlRateProfile->tpa_breakpoint);
			location = (rcData[THROTTLE] - currentControlRateProfile->tpa_breakpoint);
			normalizedLocation = location / range;
			return 100 - (uint16_t)currentControlRateProfile->dynThrPID * normalizedLocation;
		} else {
			return 100 - currentControlRateProfile->dynThrPID;
		}
	}
	
	return 100;
}

/**
 *  Calculates the PID attenuation factor based on a multi point curve
 *
 *  @return scalar represented as 0-100
 */
int16_t applyTPA(float throttle) {

	float attenuation;


	


	



    static float curveValuesC[9] = {0.0f, 0.0f, 0.15f, 0.20f, 0.20f, 0.20f, 0.30f, 0.25f, 0.15f};
    static float curveLocationsC[9] = {0.0f, 0.15f, 0.35f, 0.50f, 0.55f, 0.60f, 0.75f, 0.85f, 1.0f};
	









        tpaCurve.numberOfPoints = 9;
        tpaCurve.curveValues = curveValuesC;
        tpaCurve.curveLocations = curveLocationsC;






	
	attenuation = getValueFromMultiPointCurve(&tpaCurve, throttle);
	return (int16_t)((1.0f - attenuation) * 100.0f);
}

void annexCode(void)
{
    int32_t tmp, tmp2;
    float tmp3, tmp4;
    int32_t axis;
    bool allowSkitzo = false;

    float OldRange, OldValue, NewRange, NewValue, OldMax, OldMin, NewMax, NewMin, deadBand;

#if defined(REVO) || defined(REVOLT)
    allowSkitzo = true;
#endif

    if (IS_RC_MODE_ACTIVE(BOXSKITZO) && allowSkitzo) {

		for (axis = 0; axis < 3; axis++) {

			if (rcData[axis] < masterConfig.rxConfig.midrc) { 
				OldMax = masterConfig.rxConfig.midrc;
				OldMin = 1000;
				NewMax = 0;
				NewMin = -1;
			} else { 
				OldMax = 2000;
				OldMin = masterConfig.rxConfig.midrc;
				NewMax = 1;
				NewMin = 0;
			}

			OldValue = rcData[axis];
			OldRange = (OldMax - OldMin);
			NewRange = (NewMax - NewMin);

			deadBand = (float)currentProfile->rcControlsConfig.deadband * 0.002f; 

			NewValue = constrainf ( (((OldValue - OldMin) * NewRange) / OldRange) + NewMin, -1, 1);

			if (ABS(NewValue) > deadBand) {
				if (axis == YAW) {
					if (ABS(NewValue) > 0.025f) {
						NewValue *= (float)(-masterConfig.yaw_control_direction);
					} else {
						NewValue = 0;
					}
				}
			} else {
				NewValue = 0;
			}


			trueRcCommandF[axis] = NewValue; 

			rcCommand[axis] = (500.0f + 5.0f * (float)currentControlRateProfile->rcPitchExpo8 * (NewValue * NewValue - 1.0f)) * NewValue;

		}

		tmp = constrain(rcData[THROTTLE], masterConfig.rxConfig.mincheck, PWM_RANGE_MAX);
		tmp = (uint32_t)(tmp - masterConfig.rxConfig.mincheck) * PWM_RANGE_MIN / (PWM_RANGE_MAX - masterConfig.rxConfig.mincheck);       
		tmp2 = tmp / 100;
		rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;    

		
		if (((micros() - timeArmedAt) < 100000)) {
			rcCommand[THROTTLE] = (float)lookupThrottleRC[masterConfig.rxConfig.mincheck];
		}

		Throttle_p = constrainf(((float)rcCommandUsed[THROTTLE] - (float)masterConfig.rxConfig.mincheck) / ((float)masterConfig.rxConfig.maxcheck - (float)masterConfig.rxConfig.mincheck), 0.0f, 1.0f);

    } else {
		__disable_irq();
		for (axis = 0; axis < 3; axis++) {
			tmp = MIN(ABS(rcData[axis] - masterConfig.rxConfig.midrc), 500);
			if (axis == PITCH) {
				if (currentProfile->rcControlsConfig.deadband) {
					if (tmp > currentProfile->rcControlsConfig.deadband) {
						tmp -= currentProfile->rcControlsConfig.deadband;
					} else {
						tmp = 0;
					}
				}
				if (feature(FEATURE_TX_STYLE_EXPO) ) {
					tmp3 = (float)tmp / 500.0f; 
					tmp4 = (float)currentControlRateProfile->rcPitchExpo8 / 100.0f;
					rcCommand[axis] = (float)tmp*( tmp4*(tmp3*tmp3*tmp3) + tmp3*(1-tmp4) );
				} else {
					tmp2 = tmp / 20;
					rcCommand[axis] = (float)lookupPitchRC[tmp2] + (float)(tmp - tmp2 * 20) * (lookupPitchRC[tmp2 + 1] - lookupPitchRC[tmp2]) / 20;
				}
			} else if (axis == ROLL) {
				if (currentProfile->rcControlsConfig.deadband) {
					if (tmp > currentProfile->rcControlsConfig.deadband) {
						tmp -= currentProfile->rcControlsConfig.deadband;
					} else {
						tmp = 0;
					}
				}
				if (feature(FEATURE_TX_STYLE_EXPO) ) {
					tmp3 = (float)tmp / 500.0f; 
					tmp4 = (float)currentControlRateProfile->rcRollExpo8 / 100.0f;
					rcCommand[axis] = (float)tmp*( tmp4*(tmp3*tmp3*tmp3) + tmp3*(1-tmp4) );
				} else {
					tmp2 = tmp / 20;
					rcCommand[axis] = (float)lookupRollRC[tmp2] + (float)(tmp - tmp2 * 20) * (lookupRollRC[tmp2 + 1] - lookupRollRC[tmp2]) / 20;
				}
			} else if (axis == YAW) {
				if (currentProfile->rcControlsConfig.yaw_deadband) {
					if (tmp > currentProfile->rcControlsConfig.yaw_deadband) {
						tmp -= currentProfile->rcControlsConfig.yaw_deadband;
					} else {
						tmp = 0;
					}
				}
				if (feature(FEATURE_TX_STYLE_EXPO) ) {
					tmp3 = (float)tmp / 500.0f; 
					tmp4 = (float)currentControlRateProfile->rcYawExpo8 / 100.0f;
					rcCommand[axis] = (float)tmp*( tmp4*(tmp3*tmp3*tmp3) + tmp3*(1-tmp4) ) * -masterConfig.yaw_control_direction;
				} else {
					tmp2 = tmp / 20;
					rcCommand[axis] = (float)((float)lookupYawRC[tmp2] + (float)(tmp - tmp2 * 20) * (lookupYawRC[tmp2 + 1] - lookupYawRC[tmp2]) / 20) * -masterConfig.yaw_control_direction;
				}
			}

			if (rcData[axis] < masterConfig.rxConfig.midrc) {
				rcCommand[axis] = -rcCommand[axis];
			}

		}
		__enable_irq();

		tmp = constrain(rcData[THROTTLE], masterConfig.rxConfig.mincheck, PWM_RANGE_MAX);
		tmp = (uint32_t)(tmp - masterConfig.rxConfig.mincheck) * PWM_RANGE_MIN / (PWM_RANGE_MAX - masterConfig.rxConfig.mincheck);       
		tmp2 = tmp / 100;
		rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;    

		
		if (((micros() - timeArmedAt) < 100000)) {
			rcCommand[THROTTLE] = (float)lookupThrottleRC[masterConfig.rxConfig.mincheck];
		}

		Throttle_p = constrainf(((float)rcCommandUsed[THROTTLE] - (float)masterConfig.rxConfig.mincheck) / ((float)masterConfig.rxConfig.maxcheck - (float)masterConfig.rxConfig.mincheck), 0.0f, 1.0f);

    } 

    if ( ((micros() - timeArmedAt) > 100000) && (Throttle_p > 0.1f) && (ARMING_FLAG(ARMED)) ) {
    	FullKiLatched = true;
    }


    if (FLIGHT_MODE(HEADFREE_MODE)) {
        float radDiff = degreesToRadians(DECIDEGREES_TO_DEGREES(attitude.values.yaw) - headFreeModeHold);
        float cosDiff = cos_approx(radDiff);
        float sinDiff = sin_approx(radDiff);
        int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
        rcCommand[ROLL] = rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
        rcCommand[PITCH] = rcCommand_PITCH;
    }


    if (masterConfig.rxConfig.fpvCamAngleDegrees && !FLIGHT_MODE(HEADFREE_MODE)) {
        scaleRcCommandToFpvCamAngle();
    }

    if (ARMING_FLAG(ARMED)) {
        LED0_ON;
    } else {
        if (IS_RC_MODE_ACTIVE(BOXARM) == 0) {
            ENABLE_ARMING_FLAG(OK_TO_ARM);
        }

        if (!STATE(SMALL_ANGLE)) {
            DISABLE_ARMING_FLAG(OK_TO_ARM);
        }

        if (isCalibrating()) {
            warningLedFlash();
            DISABLE_ARMING_FLAG(OK_TO_ARM);
        } else {
            if (ARMING_FLAG(OK_TO_ARM)) {
                warningLedDisable();
            } else {
                warningLedFlash();
            }
        }

        warningLedUpdate();
    }

    if (gyro.temperature)
        gyro.temperature(&telemTemperature1);
}

void mwDisarm(void)
{

	calibrate_gyro = true;

    if (ARMING_FLAG(ARMED)) {

        DISABLE_ARMING_FLAG(ARMED);

        FullKiLatched = false;

#ifdef BLACKBOX
        if (feature(FEATURE_BLACKBOX)) {
            finishBlackbox();
        }
#endif

        beeper(BEEPER_DISARMING);      
    }
}

#define TELEMETRY_FUNCTION_MASK (FUNCTION_TELEMETRY_FRSKY | FUNCTION_TELEMETRY_HOTT | FUNCTION_TELEMETRY_MSP | FUNCTION_TELEMETRY_SMARTPORT)

void releaseSharedTelemetryPorts(void) {
    serialPort_t *sharedPort = findSharedSerialPort(TELEMETRY_FUNCTION_MASK, FUNCTION_MSP);
    while (sharedPort) {
        mspReleasePortIfAllocated(sharedPort);
        sharedPort = findNextSharedSerialPort(TELEMETRY_FUNCTION_MASK, FUNCTION_MSP);
    }
}

void mwArm(void)
{

	uint8_t calibrateWD = 0;
	static bool doubleOnce = true;
	static bool doubleAll = true;

    if (ARMING_FLAG(OK_TO_ARM)) {

        if (ARMING_FLAG(ARMED)) {
            return;
        }
        if (IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
            return;
        }

#if defined(STM32F4) || defined(STM32F7)

    	if (firstTimeArm) {

    		if(calibrate_gyro)
    		{
    			gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);

    	    	if ((doubleOnce) && (doubleAll)) {
    				if (masterConfig.arm_method == ALWAYS_DOUBLE) {
    					doubleOnce = false;
    					return; 
    				} else if ( (masterConfig.arm_method == DOUBLE_ONCE) || (masterConfig.arm_method == DOUBLE_ONCE_NC) ) {
    					doubleAll = false;
    					return; 
    				} else {
    					doubleAll = false;
    				}
        			calibrate_gyro = false; 
    	    	}


        		while(!isGyroCalibrationComplete()) {

        			calibrateWD++;
        			if (calibrateWD > 20) {
            			return; 
        			}
        			delay(100); 
        		}

        		
    			calibrate_gyro = false; 

    			
    			if ( (masterConfig.arm_method == DOUBLE_ONCE_NC) || (masterConfig.arm_method == ALWAYS_SINGLE_NC) ) {
        			firstTimeArm = false; 
        		}

    		}

    	}
#else

    	if (firstTimeArm) { 
    		if ( (masterConfig.arm_method == ALWAYS_DOUBLE) ) {
    			firstTimeArm = false;
    			gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
    			return;
    		}
    		if ( (masterConfig.arm_method == DOUBLE_ONCE) ) {
    			firstTimeArm = false;
    			gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
    			return;
    		}
    		if ( (masterConfig.arm_method == DOUBLE_ONCE_NC) ) {
    			firstTimeArm = false;
    			gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
    			return;
    		}
    	} else {
    		if ( (masterConfig.arm_method == ALWAYS_DOUBLE) ) {
    			firstTimeArm = true;
    		}
    	}

		if (!isGyroCalibrationComplete()) {
			return;
		}
#endif
        if (!ARMING_FLAG(PREVENT_ARMING)) {

        	doubleOnce = true;

	        rx_watchdog_init(Watchdog_Timeout_1s);

            ENABLE_ARMING_FLAG(ARMED);
            timeArmedAt = micros();

            headFreeModeHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw);

#ifdef BLACKBOX
            if (feature(FEATURE_BLACKBOX)) {
                serialPort_t *sharedBlackboxAndMspPort = findSharedSerialPort(FUNCTION_BLACKBOX, FUNCTION_MSP);
                if (sharedBlackboxAndMspPort) {
                    mspReleasePortIfAllocated(sharedBlackboxAndMspPort);
                }
                startBlackbox();
            }
#endif
            disarmAt = millis() + masterConfig.auto_disarm_delay * 1000;   

            
#ifdef GPS
            if (feature(FEATURE_GPS) && STATE(GPS_FIX) && GPS_numSat >= 5)
                beeper(BEEPER_ARMING_GPS_FIX);
            else
                beeper(BEEPER_ARMING);
#else
            beeper(BEEPER_ARMING);
#endif

            return;
        }
    }

    if (!ARMING_FLAG(ARMED)) {
        beeperConfirmationBeeps(1);
    }
}


bool AccInflightCalibrationArmed = false;
bool AccInflightCalibrationMeasurementDone = false;
bool AccInflightCalibrationSavetoEEProm = false;
bool AccInflightCalibrationActive = false;
uint16_t InflightcalibratingA = 0;

void handleInflightCalibrationStickPosition(void)
{
    if (AccInflightCalibrationMeasurementDone) {
        
        AccInflightCalibrationMeasurementDone = false;
        AccInflightCalibrationSavetoEEProm = true;
    } else {
        AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
        if (AccInflightCalibrationArmed) {
            beeper(BEEPER_ACC_CALIBRATION);
        } else {
            beeper(BEEPER_ACC_CALIBRATION_FAIL);
        }
    }
}

void updateInflightCalibrationState(void)
{
    if (AccInflightCalibrationArmed && ARMING_FLAG(ARMED) && rcData[THROTTLE] > masterConfig.rxConfig.mincheck && !IS_RC_MODE_ACTIVE(BOXARM)) {   
        InflightcalibratingA = 50;
        AccInflightCalibrationArmed = false;
    }
    if (0) {      
        if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone)
            InflightcalibratingA = 50;
        AccInflightCalibrationActive = true;
    } else if (AccInflightCalibrationMeasurementDone && !ARMING_FLAG(ARMED)) {
        AccInflightCalibrationMeasurementDone = false;
        AccInflightCalibrationSavetoEEProm = true;
    }
}

void updateMagHold(void)
{
    if (ABS(rcCommand[YAW]) < 15 && FLIGHT_MODE(MAG_MODE)) {
        int16_t dif = DECIDEGREES_TO_DEGREES(attitude.values.yaw) - magHold;
        if (dif <= -180)
            dif += 360;
        if (dif >= +180)
            dif -= 360;
        dif *= -masterConfig.yaw_control_direction;
        if (STATE(SMALL_ANGLE))
            rcCommand[YAW] -= dif * currentProfile->pidProfile.P8[PIDMAG] / 30;    
    } else
        magHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw);
}

void processRx(void)
{
    static bool armedBeeperOn = false;

    calculateRxChannelsAndUpdateFailsafe(currentTime);

    
    if (feature(FEATURE_3D)) {
        if (!IS_RC_MODE_ACTIVE(BOXARM))
            mwDisarm();
    }

    updateRSSI(currentTime);

    if (feature(FEATURE_FAILSAFE)) {

        if (currentTime > FAILSAFE_POWER_ON_DELAY_US && !failsafeIsMonitoring()) {
            failsafeStartMonitoring();
        }

        failsafeUpdateState();
    }

    throttleStatus_e throttleStatus = calculateThrottleStatus(&masterConfig.rxConfig, masterConfig.flight3DConfig.deadband3d_throttle);

	if (throttleStatus == THROTTLE_LOW)
	{
		if (!ARMING_FLAG(ARMED))
		{
			ResetErrorActivated = true;
		}
	}
	else
	{
		ResetErrorActivated = false; 
	}


    
    
    
    if (ARMING_FLAG(ARMED)
        && feature(FEATURE_MOTOR_STOP)
        && !STATE(FIXED_WING)
    ) {
        if (isUsingSticksForArming()) {
            if (throttleStatus == THROTTLE_LOW) {
                if (masterConfig.auto_disarm_delay != 0
                    && (int32_t)(disarmAt - millis()) < 0
                ) {
                    
                    mwDisarm();
                    armedBeeperOn = false;
                } else {
                    
                    beeper(BEEPER_ARMED);
                    armedBeeperOn = true;
                }
            } else {
                
                if (masterConfig.auto_disarm_delay != 0) {
                    
                    disarmAt = millis() + masterConfig.auto_disarm_delay * 1000;
                }

                if (armedBeeperOn) {
                    beeperSilence();
                    armedBeeperOn = false;
                }
            }
        } else {
            
            if (throttleStatus == THROTTLE_LOW) {
                beeper(BEEPER_ARMED);
                armedBeeperOn = true;
            } else if (armedBeeperOn) {
                beeperSilence();
                armedBeeperOn = false;
            }
        }
    }

    processRcStickPositions(&masterConfig.rxConfig, throttleStatus, masterConfig.retarded_arm, masterConfig.disarm_kill_switch);

    if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
        updateInflightCalibrationState();
    }

    updateActivatedModes(currentProfile->modeActivationConditions);

    if (!cliMode) {
        updateAdjustmentStates(currentProfile->adjustmentRanges);
        processRcAdjustments(currentControlRateProfile, &masterConfig.rxConfig);
    }

    bool canUseHorizonMode = true;

    if ((IS_RC_MODE_ACTIVE(BOXANGLE) || (feature(FEATURE_FAILSAFE) && failsafeIsActive())) && (sensors(SENSOR_ACC))) {
        
        canUseHorizonMode = false;

        if (!FLIGHT_MODE(ANGLE_MODE)) {
            ENABLE_FLIGHT_MODE(ANGLE_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(ANGLE_MODE); 
    }

    if (IS_RC_MODE_ACTIVE(BOXHORIZON) && canUseHorizonMode) {

        DISABLE_FLIGHT_MODE(ANGLE_MODE);

        if (!FLIGHT_MODE(HORIZON_MODE)) {
            ENABLE_FLIGHT_MODE(HORIZON_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(HORIZON_MODE);
    }

    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
        LED1_ON;
    } else {
        LED1_OFF;
    }

#ifdef  MAG
    if (sensors(SENSOR_ACC) || sensors(SENSOR_MAG)) {
        if (IS_RC_MODE_ACTIVE(BOXMAG)) {
            if (!FLIGHT_MODE(MAG_MODE)) {
                ENABLE_FLIGHT_MODE(MAG_MODE);
                magHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw);
            }
        } else {
            DISABLE_FLIGHT_MODE(MAG_MODE);
        }
        if (IS_RC_MODE_ACTIVE(BOXHEADFREE)) {
            if (!FLIGHT_MODE(HEADFREE_MODE)) {
                ENABLE_FLIGHT_MODE(HEADFREE_MODE);
            }
        } else {
            DISABLE_FLIGHT_MODE(HEADFREE_MODE);
        }
        if (IS_RC_MODE_ACTIVE(BOXHEADADJ)) {
            headFreeModeHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw); 
        }
    }
#endif

#ifdef GPS
    if (sensors(SENSOR_GPS)) {
        updateGpsWaypointsAndMode();
    }
#endif

    if (0) {
        ENABLE_FLIGHT_MODE(PASSTHRU_MODE);
    } else {
        DISABLE_FLIGHT_MODE(PASSTHRU_MODE);
    }

    if (masterConfig.mixerMode == MIXER_FLYING_WING || masterConfig.mixerMode == MIXER_AIRPLANE) {
        DISABLE_FLIGHT_MODE(HEADFREE_MODE);
    }

#ifdef TELEMETRY
    if (feature(FEATURE_TELEMETRY)) {
        if ((!masterConfig.telemetryConfig.telemetry_switch && ARMING_FLAG(ARMED)) ||
                (masterConfig.telemetryConfig.telemetry_switch && IS_RC_MODE_ACTIVE(BOXTELEMETRY))) {

            releaseSharedTelemetryPorts();
        } else {
            
            telemetryCheckState();
            mspAllocateSerialPorts(&masterConfig.serialConfig);
        }
    }
#endif

}

#if defined(BARO) || defined(SONAR)
static bool haveProcessedAnnexCodeOnce = false;
#endif

void MainPidLoop(void)
{
	static uint32_t cycleTimelastCalledAt = 0;
	uint32_t cycleTimenow = micros();
	static uint8_t counter = 1;
    cycleTime = cycleTimenow - cycleTimelastCalledAt;
    cycleTimelastCalledAt = cycleTimenow;

#if defined(FAKE_EXTI)
	dT = (float)cycleTime * 0.000001f; 
	debug[3] = targetESCwritetime;
#else

	if (cycleTime > (targetLooptime * 1.5 * 1.5))
	{
		dT = (float)targetESCwritetime*2 * 0.000001f;
	}
	else
	{
		dT = (float)targetESCwritetime * 0.000001f;
	}

#endif

	imuUpdateGyroAndAttitude();
    if (counter == ESCWriteDenominator) { } else {
        counter++;
    	return;
    }
	counter=1;

    filterRc();

#if defined(BARO) || defined(SONAR)
    haveProcessedAnnexCodeOnce = true;
#endif

#ifdef MAG
        if (sensors(SENSOR_MAG)) {
            updateMagHold();
        }
#endif

#if defined(BARO) || defined(SONAR)
        if (sensors(SENSOR_BARO) || sensors(SENSOR_SONAR)) {
            if (FLIGHT_MODE(BARO_MODE) || FLIGHT_MODE(SONAR_MODE)) {
                applyAltHold(&masterConfig.airplaneConfig);
            }
        }
#endif

    
    
    
    
    if (isUsingSticksForArming() && rcData[THROTTLE] <= masterConfig.rxConfig.mincheck
#ifndef USE_QUAD_MIXER_ONLY
#ifdef USE_SERVOS
                && !((masterConfig.mixerMode == MIXER_TRI || masterConfig.mixerMode == MIXER_CUSTOM_TRI) && masterConfig.mixerConfig.tri_unarmed_servo)
#endif
                && masterConfig.mixerMode != MIXER_AIRPLANE
                && masterConfig.mixerMode != MIXER_FLYING_WING
#endif
    ) {
        rcCommand[YAW] = 0;
    }

    if (currentProfile->throttle_correction_value && (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE))) {
        rcCommand[THROTTLE] += calculateThrottleAngleCorrection(currentProfile->throttle_correction_value);
    }

#ifdef GPS
    if (sensors(SENSOR_GPS)) {
        if ((FLIGHT_MODE(GPS_HOME_MODE) || FLIGHT_MODE(GPS_HOLD_MODE)) && STATE(GPS_FIX_HOME)) {
            updateGpsStateForHomeAndHoldMode();
        }
    }
#endif

	if ( (ResetErrorActivated && !FullKiLatched) || (ResetErrorActivated && isUsingSticksForArming()) ) {
		pidResetErrorGyro();
	}


    
    pid_controller(
        &currentProfile->pidProfile,
        currentControlRateProfile,
        masterConfig.max_angle_inclination,
        &currentProfile->accelerometerTrims,
        &masterConfig.rxConfig
    );

    mixTable();

#ifdef USE_SERVOS
    filterServos();
    writeServos();
#endif

    if (motorControlEnable) {
        writeMotors();
    }

#ifdef BLACKBOX
    if (!cliMode && feature(FEATURE_BLACKBOX)) {
        handleBlackbox();
    }
#endif
}

void UpdateAccelerometer(void)
{
    imuUpdateAccelerometer(&currentProfile->accelerometerTrims);
}

void taskHandleAnnex(void)
{
	annexCode();
}

void taskHandleSerial(void)
{
    handleSerial();
}

void taskUpdateBeeper(void)
{
    beeperUpdate();          
}

void taskUpdateBattery(void)
{
    static uint32_t vbatLastServiced = 0;
    static uint32_t ibatLastServiced = 0;

    if (feature(FEATURE_VBAT)) {
        if (cmp32(currentTime, vbatLastServiced) >= VBATINTERVAL) {
            vbatLastServiced = currentTime;
            updateBattery();
        }
    }

    if (feature(FEATURE_CURRENT_METER)) {
        int32_t ibatTimeSinceLastServiced = cmp32(currentTime, ibatLastServiced);

        if (ibatTimeSinceLastServiced >= IBATINTERVAL) {
            ibatLastServiced = currentTime;
            updateCurrentMeter(ibatTimeSinceLastServiced, &masterConfig.rxConfig, masterConfig.flight3DConfig.deadband3d_throttle);
        }
    }
}

bool taskUpdateRxCheck(void)
{
    updateRx(currentTime);
    return shouldProcessRx(currentTime);
}

void taskUpdateRxMain(void)
{
    processRx();
    isRXDataNew = true;

#ifdef BARO
    
    if (haveProcessedAnnexCodeOnce) {
        if (sensors(SENSOR_BARO)) {
            updateAltHoldState();
        }
    }
#endif

#ifdef SONAR
    
    if (haveProcessedAnnexCodeOnce) {
        if (sensors(SENSOR_SONAR)) {
            updateSonarAltHoldState();
        }
    }
#endif
}

#ifdef GPS
void taskProcessGPS(void)
{
    
    
    
    if (feature(FEATURE_GPS)) {
        gpsThread();
    }

    if (sensors(SENSOR_GPS)) {
        updateGpsIndicator(currentTime);
    }
}
#endif

#ifdef MAG
void taskUpdateCompass(void)
{
    if (sensors(SENSOR_MAG)) {
        updateCompass(&masterConfig.magZero);
    }
}
#endif

#ifdef BARO
void taskUpdateBaro(void)
{
    if (sensors(SENSOR_BARO)) {
        uint32_t newDeadline = baroUpdate();
        rescheduleTask(TASK_SELF, newDeadline);
    }
}
#endif

#ifdef SONAR
void taskUpdateSonar(void)
{
    if (sensors(SENSOR_SONAR)) {
        sonarUpdate();
    }
}
#endif

#if defined(BARO) || defined(SONAR)
void taskCalculateAltitude(void)
{
    if (false
#if defined(BARO)
        || (sensors(SENSOR_BARO) && isBaroReady())
#endif
#if defined(SONAR)
        || sensors(SENSOR_SONAR)
#endif
        ) {
        calculateEstimatedAltitude(currentTime);
    }}
#endif

#ifdef DISPLAY
void taskUpdateDisplay(void)
{
    if (feature(FEATURE_DISPLAY)) {
        updateDisplay();
    }
}
#endif

#ifdef TELEMETRY
void taskTelemetry(void)
{
    telemetryCheckState();

    if (!cliMode && feature(FEATURE_TELEMETRY)) {
        telemetryProcess(&masterConfig.rxConfig, masterConfig.flight3DConfig.deadband3d_throttle);
    }
}
#endif

#ifdef WS2812_LED
void taskLedStrip(void)

{


    ws2812_led_update(WS2812_MAX_LEDS);
}
#endif
