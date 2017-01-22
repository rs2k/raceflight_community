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
       
typedef enum {
    BEEPER_SILENCE = 0,
    BEEPER_GYRO_CALIBRATED,
    BEEPER_RX_LOST_LANDING,
    BEEPER_RX_LOST,
    BEEPER_DISARMING,
    BEEPER_ARMING,
    BEEPER_ARMING_GPS_FIX,
    BEEPER_BAT_CRIT_LOW,
    BEEPER_BAT_LOW,
    BEEPER_USB,
    BEEPER_RX_SET,
    BEEPER_DISARM_REPEAT,
    BEEPER_ACC_CALIBRATION,
    BEEPER_ACC_CALIBRATION_FAIL,
    BEEPER_READY_BEEP,
    BEEPER_MULTI_BEEPS,
    BEEPER_ARMED,
    BEEPER_CASE_MAX
} beeperMode_e;
#define BEEPER_OFF_FLAGS_MIN 0
#define BEEPER_OFF_FLAGS_MAX ((1 << (BEEPER_CASE_MAX - 1)) - 1)
typedef struct beeperOffConditions_t {
    uint32_t flags;
} beeperOffConditions_t;
void beeper(beeperMode_e mode);
void beeperSilence(void);
void beeperUpdate(void);
void beeperConfirmationBeeps(uint8_t beepCount);
uint32_t getArmingBeepTimeMicros(void);
beeperMode_e beeperModeForTableIndex(int idx);
const char *beeperNameForTableIndex(int idx);
int beeperTableEntryCount(void);
