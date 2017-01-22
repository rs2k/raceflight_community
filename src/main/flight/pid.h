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
       
#define GYRO_I_MAX 256
#define YAW_P_LIMIT_MIN 100
#define YAW_P_LIMIT_MAX 500
#define IS_POSITIVE(x) ((x > 0) ? true : false)
typedef enum {
    PIDROLL,
    PIDPITCH,
    PIDYAW,
    PIDALT,
    PIDPOS,
    PIDPOSR,
    PIDNAVR,
    PIDLEVEL,
    PIDMAG,
    PIDVEL,
    PID_ITEM_COUNT
} pidIndex_e;
typedef enum {
    PID_CONTROLLER_MWREWRITE = 1,
    PID_CONTROLLER_LUX_FLOAT,
    PID_COUNT
} pidControllerType_e;
#define IS_PID_CONTROLLER_FP_BASED(pidController) (pidController == 2)
typedef struct pidProfile_s {
    uint8_t pidController;
    uint8_t P8[PID_ITEM_COUNT];
    uint8_t I8[PID_ITEM_COUNT];
    uint8_t D8[PID_ITEM_COUNT];
    float P_f[3];
    float I_f[3];
    float D_f[3];
    float A_level;
    float H_level;
    uint8_t H_sensitivity;
    uint8_t PitchAcroPlusFactor;
    uint8_t RollAcroPlusFactor;
    uint8_t YawAcroPlusFactor;
    uint16_t wrgyrolpf;
    uint16_t wpgyrolpf;
    uint16_t wygyrolpf;
    uint16_t wrkdlpf;
    uint16_t wpkdlpf;
    uint16_t wykdlpf;
    float fcquick;
    float fcrap;
    float fcpress;
 uint16_t yaw_pterm_cut_hz;
 uint8_t pitch_pterm_cut_hz;
 uint16_t witchcraft;
#ifdef GTUNE
    uint8_t gtune_lolimP[3];
    uint8_t gtune_hilimP[3];
    uint8_t gtune_pwr;
    uint16_t gtune_settle_time;
    uint8_t gtune_average_cycles;
#endif
} pidProfile_t;
extern int16_t axisPID[XYZ_AXIS_COUNT];
extern int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];
extern float factor0;
extern float factor1;
extern float wow_factor0;
extern float wow_factor1;
extern float Throttle_p;
void pidSetController(pidControllerType_e type);
void pidResetErrorGyro(void);
