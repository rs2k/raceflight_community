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

#pragma once

extern uint16_t acc_1G; 

typedef struct gyro_s {
    sensorGyroInitFuncPtr init;                             
    sensorReadFuncPtr read;                                 
    sensorReadFuncPtr temperature;                          
    float scale;                                            
} gyro_t;

typedef struct acc_s {
    sensorInitFuncPtr init;                                 
    sensorReadFuncPtr read;                                 
    char revisionCode;                                      
} acc_t;

#define DLPF_L1 0
#define DLPF_M1 1
#define DLPF_M2 2
#define DLPF_M4 3
#define DLPF_M8 4
#define DLPF_H1 5
#define DLPF_H2 6
#define DLPF_H4 7
#define DLPF_H8 8
#define DLPF_H16 9
#define DLPF_H32 10
#define DLPF_UH1 11
#define DLPF_UH2 12
#define DLPF_UH4 13
#define DLPF_UH8 14
#define DLPF_UH16 15
#define DLPF_UH32 16
