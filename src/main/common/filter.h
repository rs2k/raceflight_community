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
#pragma once 
       
typedef struct {
 float q;
 float r;
 float x;
 float p;
 float k;
} kalman_state;
kalman_state kalman_init(float q, float r, float p, float intial_value);
void kalman_update(kalman_state *state, float measurement);
typedef struct biquad_s {
    float a0, a1, a2, a3, a4;
    float x1, x2, y1, y2;
} biquad_t;
typedef struct filterStatePt1_s {
 float state;
 float RC;
 float constdT;
} filterStatePt1_t;
typedef struct biquad2_s {
    double a0, a1, a2, a3, a4;
    double x1, x2, y1, y2;
} biquad2_t;
float filterApplyPt1(float input, filterStatePt1_t *filter, uint8_t f_cut, float dt);
float applyBiQuadFilter(float sample, biquad_t *state);
void BiQuadNewLpf(uint16_t filterCutFreq, biquad_t *newState, float refreshRate);
double applyBiQuadFilter2(double sample, biquad2_t *state);
void BiQuadNewLpf2(uint16_t filterCutFreq, biquad2_t *newState, float refreshRate);
