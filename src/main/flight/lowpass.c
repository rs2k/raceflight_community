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
#include <math.h>

#include "common/maths.h"
#include "flight/lowpass.h"



void generateLowpassCoeffs2(int16_t freq, lowpass_t *filter)
{
    float fixedScaler;
    int i;

    
    float freqf = (float)freq*0.001f;
    float omega = tan_approx((float)M_PI*freqf/2.0f);
    float scaling = 1.0f / (omega*omega + 1.4142136f*omega + 1.0f);


#if defined(UNIT_TEST) && defined(DEBUG_LOWPASS)
    printf("lowpass cutoff: %f, omega: %f\n", freqf, omega);
#endif
   
    filter->bf[0] = scaling * omega*omega;
    filter->bf[1] = 2.0f * filter->bf[0];
    filter->bf[2] = filter->bf[0];

    filter->af[0] = 1.0f;
    filter->af[1] = scaling * (2.0f * omega*omega - 2.0f);
    filter->af[2] = scaling * (omega*omega - 1.4142136f * omega + 1.0f);

    
    filter->input_bias = 1500; 
    filter->input_shift = 16;
    filter->coeff_shift = 24; 
    fixedScaler = (float)(1ULL << filter->coeff_shift);
    for (i = 0; i < LOWPASS_NUM_COEF; i++) {
        filter->a[i] = LPF_ROUND(filter->af[i] * fixedScaler);
        filter->b[i] = LPF_ROUND(filter->bf[i] * fixedScaler);
#if defined(UNIT_TEST) && defined(DEBUG_LOWPASS)
        printf("(%d) bf: %f af: %f b: %ld a: %ld\n", i, 
                filter->bf[i], filter->af[i], filter->b[i], filter->a[i]);
#endif
    }

    filter->freq = freq;
}

int32_t lowpassFixed(lowpass_t *filter, int32_t in, int16_t freq)
{
    int16_t coefIdx;
    int64_t out;
    int32_t in_s;
  
    
    if (freq != filter->freq) {
        filter->init = false;
    }

    
    if (!filter->init) {
        generateLowpassCoeffs2(freq, filter);
        for (coefIdx = 0; coefIdx < LOWPASS_NUM_COEF; coefIdx++) {
            filter->x[coefIdx] = (in - filter->input_bias) << filter->input_shift;
            filter->y[coefIdx] = (in - filter->input_bias) << filter->input_shift;
        }
        filter->init = true;
    }

    
    in_s = (in - filter->input_bias) << filter->input_shift;

    
    for (coefIdx = LOWPASS_NUM_COEF-1; coefIdx > 0; coefIdx--) {
        filter->x[coefIdx] = filter->x[coefIdx-1];
        filter->y[coefIdx] = filter->y[coefIdx-1];
    }
    filter->x[0] = in_s;

    
    out = filter->x[0] * filter->b[0];
    for (coefIdx = 1; coefIdx < LOWPASS_NUM_COEF; coefIdx++) {
        out -= filter->y[coefIdx] * filter->a[coefIdx];
        out += filter->x[coefIdx] * filter->b[coefIdx];
    }

    
    out >>= filter->coeff_shift;
    filter->y[0] = (int32_t)out;

    
    out = (out + (1 << (filter->input_shift-1))) >> filter->input_shift;
    
    
    out += filter->input_bias;

    return (int32_t)out;
}

