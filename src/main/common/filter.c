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
#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "debug.h"
#include "drivers/gyro_sync.h"
float filterApplyPt1(float input, filterStatePt1_t *filter, uint8_t f_cut, float dT) {
 if (!filter->RC) {
  filter->RC = 1.0f / ( 2.0f * (float)M_PI * f_cut );
 }
    filter->state = filter->state + dT / (filter->RC + dT) * (input - filter->state);
    return filter->state;
}
#define M_LN2_FLOAT 0.69314718055994530942f
#define M_PI_FLOAT 3.14159265358979323846f
#define BIQUAD_BANDWIDTH 1.234f
void BiQuadNewLpf(uint16_t filterCutFreq, biquad_t *newState, float refreshRate)
{
 float samplingRate;
    samplingRate = 1 / ((float)targetLooptime * (float)0.000001);
    if (!refreshRate) {
     samplingRate = 1 / ((float)targetLooptime * (float)0.000001);
    } else {
     samplingRate = (float)refreshRate;
    }
    float omega, sn, cs, alpha;
    float a0, a1, a2, b0, b1, b2;
    if (filterCutFreq == 666) {
        a0 = 0.0014553501110842863;
        a1 = 0.0029107002221685726;
        a2 = 0.0014553501110842863;
     b0 = 1;
     b1 = -1.8826089194047555;
        b2 = 0.8884303198490927;
    } else {
     omega = 2 * (float)M_PI_FLOAT * (float) filterCutFreq / samplingRate;
     sn = (float)sinf((float)omega);
     cs = (float)cosf((float)omega);
     alpha = sn * (float)sinf( (float)((float)M_LN2_FLOAT /2 * (float)BIQUAD_BANDWIDTH * (omega /sn)) );
  b0 = (1 - cs) /2;
  b1 = 1 - cs;
  b2 = (1 - cs) /2;
  a0 = 1 + alpha;
  a1 = -2 * cs;
  a2 = 1 - alpha;
 }
    newState->a0 = b0 /a0;
    newState->a1 = b1 /a0;
    newState->a2 = b2 /a0;
    newState->a3 = a1 /a0;
    newState->a4 = a2 /a0;
    newState->x1 = newState->x2 = (float)0.0;
    newState->y1 = newState->y2 = (float)0.0;
}
float applyBiQuadFilter(float sample, biquad_t *state)
{
    float result;
    result = state->a0 * (float)sample + state->a1 * state->x1 + state->a2 * state->x2 - state->a3 * state->y1 - state->a4 * state->y2;
    state->x2 = state->x1;
    state->x1 = (float)sample;
    state->y2 = state->y1;
    state->y1 = result;
    return (float)result;
}
void BiQuadNewLpf2(uint16_t filterCutFreq, biquad2_t *newState, float refreshRate)
{
 double samplingRate;
    samplingRate = 1 / ((double)targetLooptime * (double)0.000001);
    if (!refreshRate) {
     samplingRate = 1 / ((double)targetLooptime * (double)0.000001);
    } else {
     samplingRate = (double)refreshRate;
    }
    double omega, sn, cs, alpha;
    double a0, a1, a2, b0, b1, b2;
    if (filterCutFreq == 666) {
        a0 = 0.0014553501110842863;
        a1 = 0.0029107002221685726;
        a2 = 0.0014553501110842863;
     b0 = 1;
     b1 = -1.8826089194047555;
        b2 = 0.8884303198490927;
    } else {
     omega = 2 * (double)M_PI_FLOAT * (double) filterCutFreq / samplingRate;
     sn = (double)sinf((double)omega);
     cs = (double)cosf((double)omega);
     alpha = sn * (double)sinf( (double)((double)M_LN2_FLOAT /2 * (double)BIQUAD_BANDWIDTH * (omega /sn)) );
  b0 = (1 - cs) /2;
  b1 = 1 - cs;
  b2 = (1 - cs) /2;
  a0 = 1 + alpha;
  a1 = -2 * cs;
  a2 = 1 - alpha;
 }
    newState->a0 = b0 /a0;
    newState->a1 = b1 /a0;
    newState->a2 = b2 /a0;
    newState->a3 = a1 /a0;
    newState->a4 = a2 /a0;
    newState->x1 = newState->x2 = (double)0.0;
    newState->y1 = newState->y2 = (double)0.0;
}
double applyBiQuadFilter2(double sample, biquad2_t *state)
{
 double result;
    result = state->a0 * (double)sample + state->a1 * state->x1 + state->a2 * state->x2 - state->a3 * state->y1 - state->a4 * state->y2;
    state->x2 = state->x1;
    state->x1 = (double)sample;
    state->y2 = state->y1;
    state->y1 = result;
    return (double)result;
}
