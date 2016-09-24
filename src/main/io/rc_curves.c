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

#include "rx/rx.h"
#include "io/rc_controls.h"
#include "io/escservo.h"

#include "io/rc_curves.h"

float lookupPitchRC[PITCH_LOOKUP_LENGTH];     
float lookupRollRC[ROLL_LOOKUP_LENGTH];     
float lookupYawRC[YAW_LOOKUP_LENGTH];     
float lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];   

void generatePitchCurve(controlRateConfig_t *controlRateConfig)
{
    uint8_t i;
	float j = 0;

	for (i = 0; i < PITCH_LOOKUP_LENGTH; i++) {
		lookupPitchRC[i] = (float)((2500.0f + controlRateConfig->rcPitchExpo8 * (j * j - 25.0f)) * j * 100.0f / 2500.0f);
		j += 0.2f;
	}
}

void generateRollCurve(controlRateConfig_t *controlRateConfig)
{
    uint8_t i;
	float j = 0;

	for (i = 0; i < ROLL_LOOKUP_LENGTH; i++) {
		lookupRollRC[i] = (float)((2500.0f + controlRateConfig->rcRollExpo8 * (j * j - 25.0f)) * j * 100.0f / 2500.0f);
		j += 0.2f;
	}
}

void generateYawCurve(controlRateConfig_t *controlRateConfig)
{
    uint8_t i;
	float j = 0;

	for (i = 0; i < YAW_LOOKUP_LENGTH; i++) {
		lookupYawRC[i] = (float)((2500.0f + controlRateConfig->rcYawExpo8 * (j * j - 25.0f)) * j * 100.0f / 2500.0f);
		j += 0.2f;
	}
}

void generateThrottleCurve(controlRateConfig_t *controlRateConfig, escAndServoConfig_t *escAndServoConfig)
{
    uint8_t i;

    for (i = 0; i < THROTTLE_LOOKUP_LENGTH; i++) {
        int16_t tmp = 10 * i - controlRateConfig->thrMid8;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - controlRateConfig->thrMid8;
        if (tmp < 0)
            y = controlRateConfig->thrMid8;
        escAndServoConfig->maxthrottle = 2000;
        lookupThrottleRC[i] = (float)(10 * controlRateConfig->thrMid8 + tmp * (100 - controlRateConfig->thrExpo8 + (int32_t) controlRateConfig->thrExpo8 * (tmp * tmp) / (y * y)) / 10);
        lookupThrottleRC[i] = (float)(escAndServoConfig->minthrottle + (int32_t) (escAndServoConfig->maxthrottle - escAndServoConfig->minthrottle) * lookupThrottleRC[i] / 1000); 
    }
}
