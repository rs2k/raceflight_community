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


typedef enum {
    NAV_MODE_NONE = 0,
    NAV_MODE_POSHOLD,
    NAV_MODE_WP
} navigationMode_e;



typedef struct gpsProfile_s {
    uint16_t gps_wp_radius;                 
    uint8_t gps_lpf;                        
    uint8_t nav_slew_rate;                  
    uint8_t nav_controls_heading;           
    uint16_t nav_speed_min;                 
    uint16_t nav_speed_max;                 
    uint16_t ap_mode;                       
} gpsProfile_t;

extern int16_t GPS_angle[ANGLE_INDEX_COUNT];                

extern int32_t GPS_home[2];
extern int32_t GPS_hold[2];

extern uint16_t GPS_distanceToHome;        
extern int16_t GPS_directionToHome;        

extern navigationMode_e nav_mode;          

void GPS_reset_home_position(void);
void GPS_reset_nav(void);
void GPS_set_next_wp(int32_t* lat, int32_t* lon);
void gpsUseProfile(gpsProfile_t *gpsProfileToUse);
void gpsUsePIDs(pidProfile_t *pidProfile);
void updateGpsStateForHomeAndHoldMode(void);
void updateGpsWaypointsAndMode(void);

void onGpsNewData(void);
