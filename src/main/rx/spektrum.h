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
       
#define SPEKTRUM_SAT_BIND_DISABLED 0
#define SPEKTRUM_SAT_BIND_MAX 10
typedef enum
{
 MS_22 = 0,
 MS_11
} SPM_FREQ;
typedef enum
{
 FATSHARK,
 RACEBAND,
 E,
 B,
 A
} VTX_BAND;
typedef enum
{
 POWER_25MW,
 POWER_250MW,
 POWER_500MW
} VTX_POWER;
typedef enum
{
 US,
 EU
} VTX_REGION;
typedef enum
{
 ACTIVE,
 PIT
} VTX_PIT;
typedef struct
{
 uint8_t vtxChannel;
 VTX_BAND vtxBand;
 VTX_POWER vtxPower;
 VTX_REGION vtxRegion;
 VTX_PIT vtxPit;
} SPM_VTX_DATA;
uint8_t spektrumFrameStatus(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig);
