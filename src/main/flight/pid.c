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

#include <platform.h>

#include "build_config.h"
#include "debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/sensor.h"
#include "drivers/gyro_sync.h"

#include "drivers/accgyro.h"
#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "rx/rx.h"

#include "io/rc_controls.h"
#include "io/gps.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation.h"
#include "flight/gtune.h"

#include "config/runtime_config.h"

extern float dT;
float Throttle_p;
extern bool FullKiLatched;
extern bool motorLimitReached;
extern bool allowITermShrinkOnly;

int16_t axisPID[3];
float factor;
float wow_factor;

#define KD_RING_BUFFER_SIZE 256
float kd_ring_buffer_p[KD_RING_BUFFER_SIZE];
float kd_ring_buffer_p_sum = 0;
int kd_ring_buffer_p_pointer = 0;
float kd_ring_buffer_r[KD_RING_BUFFER_SIZE];
float kd_ring_buffer_r_sum = 0;
int kd_ring_buffer_r_pointer = 0;

#ifdef BLACKBOX
int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];
#endif

static int32_t errorGyroI[3] = { 0, 0, 0 };
static float errorGyroIf[3] = { 0.0f, 0.0f, 0.0f };

static float errorGyroIfLimit[3];

static void pidRewrite(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, rollAndPitchTrims_t *angleTrim, rxConfig_t *rxConfig);

typedef void (*pidControllerFuncPtr)(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, rollAndPitchTrims_t *angleTrim, rxConfig_t *rxConfig);            

pidControllerFuncPtr pid_controller = pidRewrite; 

void pidResetErrorGyro(void)
{
    errorGyroI[ROLL] = 0;
    errorGyroI[PITCH] = 0;
    errorGyroI[YAW] = 0;

    errorGyroIf[ROLL] = 0.0f;
    errorGyroIf[PITCH] = 0.0f;
    errorGyroIf[YAW] = 0.0f;
}

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

static biquad_t deltaBiQuadState[3];
static filterStatePt1_t yawPTermState;
static bool deltaStateIsSet;
static uint16_t currentLPF;
static bool onlyUseErrorMethodForKd, onlyUseMeasureMethodForKd;

static void pidRfFloat(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, rollAndPitchTrims_t *angleTrim, rxConfig_t *rxConfig)
{
	float RateError, AngleRate, gyroRate;
	float ITerm, PTerm, DTerm;
	
	static float lastRate[3] = { 0, 0, 0 }, lastError[3] = { 0, 0, 0 }, InputUsed[3] = { 0, 0, 0 }, LastInput[3] = { 0, 0, 0 }, InputDelta[3] = { 0, 0, 0 };
	float delta;
	static float lastRcCommand[3] = { 0, 0, 0 };
	int axis;
    float horizonLevelStrength = 1;
    bool kdTypeM, highSpeedStickMovement;

    if (IS_RC_MODE_ACTIVE(BOXBRAINDRAIN)) {
    	onlyUseErrorMethodForKd = true;
    	onlyUseMeasureMethodForKd = false;
    } else {
    	onlyUseErrorMethodForKd = false;
    	onlyUseMeasureMethodForKd = false;
    }

    for (axis = 0; axis < 3; axis++)  {
    	if (axis == FD_ROLL) {
        	currentLPF = pidProfile->wrkdlpf;
		} else if (axis == FD_PITCH) {
	    	currentLPF = pidProfile->wpkdlpf;
		} else if (axis == FD_YAW) {
	    	currentLPF = pidProfile->wykdlpf;
		}
		if (!deltaStateIsSet && currentLPF) {
			BiQuadNewLpf(currentLPF, &deltaBiQuadState[axis], dT);
		}
    }
    deltaStateIsSet = true;

    if (FLIGHT_MODE(HORIZON_MODE)) {
        const int32_t stickPosAil = ABS(getRcStickDeflection(FD_ROLL, rxConfig->midrc));
        const int32_t stickPosEle = ABS(getRcStickDeflection(FD_PITCH, rxConfig->midrc));
        const int32_t mostDeflectedPos = MAX(stickPosAil, stickPosEle);
        horizonLevelStrength = (float)(500 - mostDeflectedPos) / 500;  
        if(pidProfile->H_sensitivity == 0){
            horizonLevelStrength = 0;
        } else {
            horizonLevelStrength = constrainf(((horizonLevelStrength - 1) * (100 / pidProfile->H_sensitivity)) + 1, 0, 1);
        }
    }

    
    for (axis = 0; axis < 3; axis++) {
        float rate = controlRateConfig->rates[axis];

        if (axis == FD_YAW) {
	         if (controlRateConfig->YawAcroPlusFactor) {
       		 wow_factor = fabsf(rcCommandUsed[axis] / 500.0f) * (controlRateConfig->YawAcroPlusFactor / 100.0f); 
       		 factor = wow_factor * rcCommandUsed[axis] + rcCommandUsed[axis];
       	 } else {
       		 factor = rcCommandUsed[axis]; 
       	 }
   		 AngleRate = (float)((rate) * factor) / 500.0f; 

         } else {
             
        	 if (axis == FD_PITCH) {
    	         if (controlRateConfig->PitchAcroPlusFactor) {
            		 wow_factor = fabsf(rcCommandUsed[axis] / 500.0f) * ((float)controlRateConfig->PitchAcroPlusFactor / 100.0f); 
            		 factor = (int16_t)(wow_factor * (float)rcCommandUsed[axis]) + rcCommandUsed[axis];
            	 } else {
            		 factor = rcCommandUsed[axis];
            	 }
        	 } else {
    	         if (controlRateConfig->RollAcroPlusFactor) {
            		 wow_factor = fabsf(rcCommandUsed[axis] / 500.0f) * ((float)controlRateConfig->RollAcroPlusFactor / 100.0f); 
            		 factor = (int16_t)(wow_factor * (float)rcCommandUsed[axis]) + rcCommandUsed[axis];
            	 } else {
            		 factor = rcCommandUsed[axis];
            	 }
        	 }

    		 AngleRate = (float)((rate) * factor) / 500.0f; 
    		 AngleRate = constrainf(AngleRate, -1800, 1800);

             if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
                const float errorAngle = (constrain(rcCommandUsed[axis], -((int) max_angle_inclination),
                    +max_angle_inclination) - attitude.raw[axis] + angleTrim->raw[axis]) / 10.0f; 
                if (FLIGHT_MODE(ANGLE_MODE)) {
                    AngleRate = errorAngle * pidProfile->A_level;
                } else {
                    AngleRate += errorAngle * pidProfile->H_level * horizonLevelStrength;
                }
            }
        }

	    InputUsed[axis] = AngleRate;

	    InputDelta[axis] = LastInput[axis] - InputUsed[axis]; 
	    (void)(InputDelta[axis]); 
	    LastInput[axis] = InputUsed[axis]; 

	    if (ABS(lastRcCommand[axis] - rcCommandUsed[axis]) > 50) {
	    	highSpeedStickMovement = true;
	    } else {
	    	highSpeedStickMovement = false;
	    }

	    lastRcCommand[axis] = rcCommandUsed[axis];

	    if ( (AngleRate <= 0) && (LastInput[axis] <= 0) && (AngleRate < LastInput[axis]) ) { 
	    	kdTypeM = true;
	    } else if ( (AngleRate > 0) && (LastInput[axis] > 0) && (AngleRate > LastInput[axis]) ) { 
	    	kdTypeM = true;
	    } else {
	    	kdTypeM = false;
	    }

        gyroRate = gyroADC[axis] * gyro.scale; 

        
        
        
        
        RateError = AngleRate - gyroRate;

        
        PTerm = RateError * (pidProfile->P_f[axis]/4);
        if (!FullKiLatched) { PTerm = PTerm / 2; }

	    if (axis == YAW && pidProfile->yaw_pterm_cut_hz) {
		    PTerm = filterApplyPt1(PTerm, &yawPTermState, pidProfile->yaw_pterm_cut_hz, dT);
	    }


	    if (axis == FD_PITCH) {

	        
	        if ((FullKiLatched) && !(FLIGHT_MODE(BOXLANDING))) {
		        errorGyroIf[axis] = constrainf(errorGyroIf[axis] + RateError * dT * (pidProfile->I_f[axis] / 2)  * 10, -250.0f, 250.0f);
		        if (motorLimitReached) {
			        errorGyroIf[axis] = constrainf(errorGyroIf[axis], -errorGyroIfLimit[axis], errorGyroIfLimit[axis]);
		        } else {
			        errorGyroIfLimit[axis] = ABS(errorGyroIf[axis]);
		        }
	        } else {
	            errorGyroIf[axis] = constrainf(errorGyroIf[axis] + RateError * dT * (pidProfile->I_f[axis]/2)  * 10, -20.0f, 20.0f);
	        }

	        
	        

	        ITerm = errorGyroIf[axis];

	    } else {

	        
			errorGyroIf[axis] = errorGyroIf[axis] + RateError;
			if (motorLimitReached) {
				errorGyroIf[axis] = constrainf(errorGyroIf[axis], -errorGyroIfLimit[axis], errorGyroIfLimit[axis]);
			} else {
				errorGyroIfLimit[axis] = ABS(errorGyroIf[axis]);
			}

	        
	        

	        if ((FullKiLatched) && !(FLIGHT_MODE(BOXLANDING))) {
	        	ITerm = constrainf( errorGyroIf[axis] * dT * (pidProfile->I_f[axis]/2)  * 10, -250.0f, 250.0f);
	        } else {
	            ITerm = constrainf( errorGyroIf[axis] * dT * (pidProfile->I_f[axis]/2)  * 10, -20.0f, 20.0f);
	        }

	    }


        
	    if (onlyUseMeasureMethodForKd) {
    	    delta = -(gyroRate - lastRate[axis]);
    	    lastRate[axis] = gyroRate;
	    } else if (!kdTypeM && (ABS(rcCommandUsed[axis]) > 150) && (highSpeedStickMovement) && !!onlyUseErrorMethodForKd) {
    	    delta = -(gyroRate - lastRate[axis]);
    	    lastRate[axis] = gyroRate;
        } else {
    	    delta = (RateError) - lastError[axis];
	        lastError[axis] = (RateError);
        }

        
        
	    delta *= (1.0f / dT);

	    if (deltaStateIsSet) {
		    if (axis == YAW && pidProfile->wykdlpf) {
			    if (pidProfile->wykdlpf) delta = applyBiQuadFilter(delta, &deltaBiQuadState[axis]);
		    }
		    else if (axis == PITCH && pidProfile->wpkdlpf) {
			    if (pidProfile->wpkdlpf) delta = applyBiQuadFilter(delta, &deltaBiQuadState[axis]);
		    }
		    else if (axis == ROLL && pidProfile->wrkdlpf) {
			    if (pidProfile->wrkdlpf) delta = applyBiQuadFilter(delta, &deltaBiQuadState[axis]);
		    }
	    }

		
	    if ((pidProfile->witchcraft / 4)) {
			if (axis == PITCH)
			{
				kd_ring_buffer_p[kd_ring_buffer_p_pointer++] = delta;
				kd_ring_buffer_p_sum += delta;

				if (kd_ring_buffer_p_pointer == (pidProfile->witchcraft / 4))
					kd_ring_buffer_p_pointer = 0;

				kd_ring_buffer_p_sum -= kd_ring_buffer_p[kd_ring_buffer_p_pointer];
				delta = (float)(kd_ring_buffer_p_sum / (float) (pidProfile->witchcraft / 4));

			}
			else if (axis == ROLL)
			{
				kd_ring_buffer_r[kd_ring_buffer_r_pointer++] = delta;
				kd_ring_buffer_r_sum += delta;

				if (kd_ring_buffer_r_pointer == (pidProfile->witchcraft / 4))
					kd_ring_buffer_r_pointer = 0;

				kd_ring_buffer_r_sum -= kd_ring_buffer_r[kd_ring_buffer_r_pointer];
				delta = (float)(kd_ring_buffer_r_sum / (float) (pidProfile->witchcraft / 4));
			}
	    }

	    float D_f = pidProfile->D_f[axis];
	    
	    
		
		
	    

	    

	    if (targetESCwritetime == 31) {
	    	D_f *= 0.25f; 
	    }
	    DTerm = constrainf(delta * (D_f / 10), -350.0f, 350.0f);

        
	    if (!FullKiLatched) {
			axisPID[axis] = constrain(lrintf(PTerm + ITerm + DTerm), -200, 200);
	    } else {
		    axisPID[axis] = constrain(lrintf(PTerm + ITerm + DTerm), -1000, 1000);
	    }

#ifdef GTUNE
        if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
            calculate_Gtune(axis);
        }
#endif

        axisPID_P[axis] = PTerm;
        axisPID_I[axis] = ITerm;
        axisPID_D[axis] = DTerm;
    }
}

static void pidRewrite(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig, uint16_t max_angle_inclination,
        rollAndPitchTrims_t *angleTrim, rxConfig_t *rxConfig)
{
    UNUSED(pidProfile);
    UNUSED(controlRateConfig);
    UNUSED(max_angle_inclination);
    UNUSED(angleTrim);
    UNUSED(rxConfig);

    int axis;
    for (axis = 0; axis < 3; axis++) {
    	axisPID[axis] = 0;

        axisPID_P[axis] = 0;
        axisPID_I[axis] = 0;
        axisPID_D[axis] = 0;
    }
}

void pidSetController(pidControllerType_e type)
{
    switch (type) {
        case PID_CONTROLLER_MWREWRITE:
        case PID_CONTROLLER_LUX_FLOAT:
        default:
            pid_controller = pidRfFloat;
            break;
    }
}
