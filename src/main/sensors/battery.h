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

#include "rx/rx.h"

#define VBAT_SCALE_DEFAULT 66 
#define VBAT_SCALE_MIN 10     
#define VBAT_SCALE_MAX 250

#define VBAT_DIVIDER_DEFAULT 110 
#define VBAT_DIVIDER_MIN 10 
#define VBAT_DIVIDER_MAX 255

#define VBATMINCELLVOLTAGE 33

typedef enum {
    CURRENT_SENSOR_NONE = 0,
    CURRENT_SENSOR_ADC,
    CURRENT_SENSOR_VIRTUAL,
    CURRENT_SENSOR_MAX = CURRENT_SENSOR_VIRTUAL
} currentSensor_e;

typedef struct batteryConfig_s {
    uint8_t vbatscale;                      
    uint8_t vbatdivider;                    
    uint8_t vbatmaxcellvoltage;             
    uint8_t vbatwarningcellvoltage;         

    int16_t currentMeterScale;             
    uint16_t currentMeterOffset;            
    currentSensor_e  currentMeterType;      

    
    uint8_t multiwiiCurrentMeterOutput;     
    uint16_t batteryCapacity;               
} batteryConfig_t;

typedef enum {
    BATTERY_OK = 0,
    BATTERY_WARNING,
    BATTERY_CRITICAL,
    BATTERY_NOT_PRESENT
} batteryState_e;

extern uint16_t vbat;
extern uint16_t vbatRaw;
extern uint16_t vbatLatestADC;
extern uint8_t batteryCellCount;
extern uint16_t batteryWarningVoltage;
extern uint16_t amperageLatestADC;
extern int32_t amperage;
extern int32_t mAhDrawn;

uint16_t batteryAdcToVoltage(uint16_t src);
batteryState_e getBatteryState(void);
const  char * getBatteryStateString(void);
void updateBattery(void);
void batteryInit(batteryConfig_t *initialBatteryConfig);

void updateCurrentMeter(int32_t lastUpdateAt, rxConfig_t *rxConfig, uint16_t deadband3d_throttle);
int32_t currentMeterToCentiamps(uint16_t src);

uint8_t calculateBatteryPercentage(void);
uint8_t calculateBatteryCapacityRemainingPercentage(void);
