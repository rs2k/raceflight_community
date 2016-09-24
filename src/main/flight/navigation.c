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
#include <ctype.h>
#include <string.h>
#include <math.h>

#include "platform.h"
#include "debug.h"

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"

#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/acceleration.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/gps.h"
#include "io/rc_controls.h"

#include "flight/pid.h"
#include "flight/navigation.h"
#include "flight/gps_conversion.h"
#include "flight/imu.h"

#include "rx/rx.h"


#include "config/config.h"
#include "config/runtime_config.h"

extern int16_t magHold;

#ifdef GPS

bool areSticksInApModePosition(uint16_t ap_mode);




int16_t GPS_angle[ANGLE_INDEX_COUNT] = { 0, 0 };    
int32_t GPS_home[2];
int32_t GPS_hold[2];

uint16_t GPS_distanceToHome;        
int16_t GPS_directionToHome;        

static int16_t nav[2];
static int16_t nav_rated[2];               
navigationMode_e nav_mode = NAV_MODE_NONE;    

static gpsProfile_t *gpsProfile;

void gpsUseProfile(gpsProfile_t *gpsProfileToUse)
{
    gpsProfile = gpsProfileToUse;
}


void navigationInit(gpsProfile_t *initialGpsProfile, pidProfile_t *pidProfile)
{
    gpsUseProfile(initialGpsProfile);
    gpsUsePIDs(pidProfile);
}



/*-----------------------------------------------------------
 *
 * Multiwii GPS code - revision: 1097
 *
 *-----------------------------------------------------------*/
#define POSHOLD_IMAX           20       
#define POSHOLD_RATE_IMAX      20       
#define NAV_IMAX               20       

/* GPS navigation can control the heading */
#define NAV_TAIL_FIRST             0    
#define NAV_SET_TAKEOFF_HEADING    1    

#define GPS_FILTERING              1    
#define GPS_LOW_SPEED_D_FILTER     1    

static bool check_missed_wp(void);
static void GPS_distance_cm_bearing(int32_t * lat1, int32_t * lon1, int32_t * lat2, int32_t * lon2, uint32_t * dist, int32_t * bearing);

static void GPS_calc_longitude_scaling(int32_t lat);
static void GPS_calc_velocity(void);
static void GPS_calc_location_error(int32_t * target_lat, int32_t * target_lng, int32_t * gps_lat, int32_t * gps_lng);
static void GPS_calc_poshold(void);
static void GPS_calc_nav_rate(uint16_t max_speed);
static void GPS_update_crosstrack(void);
static uint16_t GPS_calc_desired_speed(uint16_t max_speed, bool _slow);

static int32_t wrap_18000(int32_t error);
static int32_t wrap_36000(int32_t angle);

typedef struct {
    int16_t last_velocity;
} LeadFilter_PARAM;

typedef struct {
    float kP;
    float kI;
    float kD;
    float Imax;
} PID_PARAM;

static PID_PARAM posholdPID_PARAM;
static PID_PARAM poshold_ratePID_PARAM;
static PID_PARAM navPID_PARAM;

typedef struct {
    float integrator;          
    int32_t last_input;        
    float last_derivative;     
    float output;
    float derivative;
} PID;

static PID posholdPID[2];
static PID poshold_ratePID[2];
static PID navPID[2];

static int32_t get_P(int32_t error, PID_PARAM *pid)
{
    return (float)error * pid->kP;
}

static int32_t get_I(int32_t error, float *dt, PID *pid, PID_PARAM *pid_param)
{
    pid->integrator += ((float)error * pid_param->kI) * *dt;
    pid->integrator = constrain(pid->integrator, -pid_param->Imax, pid_param->Imax);
    return pid->integrator;
}

static int32_t get_D(int32_t input, float *dt, PID *pid, PID_PARAM *pid_param)
{
    pid->derivative = (input - pid->last_input) / *dt;

    
    
    float pidFilter = (1.0f / (2.0f * M_PIf * (float)gpsProfile->gps_lpf));
    
    
    pid->derivative = pid->last_derivative + (*dt / (pidFilter + *dt)) * (pid->derivative - pid->last_derivative);
    
    pid->last_input = input;
    pid->last_derivative = pid->derivative;
    
    return pid_param->kD * pid->derivative;
}

static void reset_PID(PID *pid)
{
    pid->integrator = 0;
    pid->last_input = 0;
    pid->last_derivative = 0;
}

#define GPS_X 1
#define GPS_Y 0


#define RADX100                    0.000174532925f
#define CROSSTRACK_GAIN            1
#define NAV_SLOW_NAV               true
#define NAV_BANK_MAX               3000 

static float dTnav;             
static int16_t actual_speed[2] = { 0, 0 };
static float GPS_scaleLonDown = 1.0f;  



static int16_t rate_error[2];
static int32_t error[2];


static int32_t GPS_WP[2];





static int32_t target_bearing;





static int32_t original_target_bearing;

static int16_t crosstrack_error;






static uint32_t wp_distance;


static int16_t waypoint_speed_gov;




#define GPS_FILTER_VECTOR_LENGTH 5

static uint8_t GPS_filter_index = 0;
static int32_t GPS_filter[2][GPS_FILTER_VECTOR_LENGTH];
static int32_t GPS_filter_sum[2];
static int32_t GPS_read[2];
static int32_t GPS_filtered[2];
static int32_t GPS_degree[2];   
static uint16_t fraction3[2];



static int32_t nav_bearing;

static int16_t nav_takeoff_bearing;

void GPS_calculateDistanceAndDirectionToHome(void)
{
    if (STATE(GPS_FIX_HOME)) {      
        uint32_t dist;
        int32_t dir;
        GPS_distance_cm_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_home[LAT], &GPS_home[LON], &dist, &dir);
        GPS_distanceToHome = dist / 100;
        GPS_directionToHome = dir / 100;
    } else {
        GPS_distanceToHome = 0;
        GPS_directionToHome = 0;
    }
}

void onGpsNewData(void)
{
    int axis;
    static uint32_t nav_loopTimer;
    uint16_t speed;


    if (!(STATE(GPS_FIX) && GPS_numSat >= 5)) {
        return;
    }

    if (!ARMING_FLAG(ARMED))
        DISABLE_STATE(GPS_FIX_HOME);

    if (!STATE(GPS_FIX_HOME) && ARMING_FLAG(ARMED))
        GPS_reset_home_position();

    
#if defined(GPS_FILTERING)
    GPS_filter_index = (GPS_filter_index + 1) % GPS_FILTER_VECTOR_LENGTH;
    for (axis = 0; axis < 2; axis++) {
        GPS_read[axis] = GPS_coord[axis];               
        GPS_degree[axis] = GPS_read[axis] / 10000000;   

        
        
        fraction3[axis] = (GPS_read[axis] - GPS_degree[axis] * 10000000) / 10000;

        GPS_filter_sum[axis] -= GPS_filter[axis][GPS_filter_index];
        GPS_filter[axis][GPS_filter_index] = GPS_read[axis] - (GPS_degree[axis] * 10000000);
        GPS_filter_sum[axis] += GPS_filter[axis][GPS_filter_index];
        GPS_filtered[axis] = GPS_filter_sum[axis] / GPS_FILTER_VECTOR_LENGTH + (GPS_degree[axis] * 10000000);
        if (nav_mode == NAV_MODE_POSHOLD) {             
            if (fraction3[axis] > 1 && fraction3[axis] < 999)
                GPS_coord[axis] = GPS_filtered[axis];
        }
    }
#endif

    
    
    
    
    dTnav = (float)(millis() - nav_loopTimer) / 1000.0f;
    nav_loopTimer = millis();
    
    dTnav = MIN(dTnav, 1.0f);

    GPS_calculateDistanceAndDirectionToHome();

    
    GPS_calc_velocity();

    if (FLIGHT_MODE(GPS_HOLD_MODE) || FLIGHT_MODE(GPS_HOME_MODE)) {
        

        
        GPS_distance_cm_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_WP[LAT], &GPS_WP[LON], &wp_distance, &target_bearing);
        GPS_calc_location_error(&GPS_WP[LAT], &GPS_WP[LON], &GPS_coord[LAT], &GPS_coord[LON]);

        switch (nav_mode) {
        case NAV_MODE_POSHOLD:
            
            GPS_calc_poshold();
            break;

        case NAV_MODE_WP:
            speed = GPS_calc_desired_speed(gpsProfile->nav_speed_max, NAV_SLOW_NAV);    
            
            
            GPS_calc_nav_rate(speed);

            
            if (gpsProfile->nav_controls_heading) {
                if (NAV_TAIL_FIRST) {
                    magHold = wrap_18000(nav_bearing - 18000) / 100;
                } else {
                    magHold = nav_bearing / 100;
                }
            }
            
            if ((wp_distance <= gpsProfile->gps_wp_radius) || check_missed_wp()) {      
                nav_mode = NAV_MODE_POSHOLD;
                if (NAV_SET_TAKEOFF_HEADING) {
                    magHold = nav_takeoff_bearing;
                }
            }
            break;
        default:
            break;
        }
    }                   
}

void GPS_reset_home_position(void)
{
    if (STATE(GPS_FIX) && GPS_numSat >= 5) {
        GPS_home[LAT] = GPS_coord[LAT];
        GPS_home[LON] = GPS_coord[LON];
        GPS_calc_longitude_scaling(GPS_coord[LAT]); 
        nav_takeoff_bearing = DECIDEGREES_TO_DEGREES(attitude.values.yaw);              
        
        ENABLE_STATE(GPS_FIX_HOME);
    }
}


void GPS_reset_nav(void)
{
    int i;

    for (i = 0; i < 2; i++) {
        GPS_angle[i] = 0;
        nav_rated[i] = 0;
        nav[i] = 0;
        reset_PID(&posholdPID[i]);
        reset_PID(&poshold_ratePID[i]);
        reset_PID(&navPID[i]);
    }
}


void gpsUsePIDs(pidProfile_t *pidProfile)
{
    posholdPID_PARAM.kP = (float)pidProfile->P8[PIDPOS] / 100.0f;
    posholdPID_PARAM.kI = (float)pidProfile->I8[PIDPOS] / 100.0f;
    posholdPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;

    poshold_ratePID_PARAM.kP = (float)pidProfile->P8[PIDPOSR] / 10.0f;
    poshold_ratePID_PARAM.kI = (float)pidProfile->I8[PIDPOSR] / 100.0f;
    poshold_ratePID_PARAM.kD = (float)pidProfile->D8[PIDPOSR] / 1000.0f;
    poshold_ratePID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;

    navPID_PARAM.kP = (float)pidProfile->P8[PIDNAVR] / 10.0f;
    navPID_PARAM.kI = (float)pidProfile->I8[PIDNAVR] / 100.0f;
    navPID_PARAM.kD = (float)pidProfile->D8[PIDNAVR] / 1000.0f;
    navPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;
}













static void GPS_calc_longitude_scaling(int32_t lat)
{
    float rads = (ABS((float)lat) / 10000000.0f) * 0.0174532925f;
    GPS_scaleLonDown = cos_approx(rads);
}




void GPS_set_next_wp(int32_t *lat, int32_t *lon)
{
    GPS_WP[LAT] = *lat;
    GPS_WP[LON] = *lon;

    GPS_calc_longitude_scaling(*lat);
    GPS_distance_cm_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_WP[LAT], &GPS_WP[LON], &wp_distance, &target_bearing);

    nav_bearing = target_bearing;
    GPS_calc_location_error(&GPS_WP[LAT], &GPS_WP[LON], &GPS_coord[LAT], &GPS_coord[LON]);
    original_target_bearing = target_bearing;
    waypoint_speed_gov = gpsProfile->nav_speed_min;
}




static bool check_missed_wp(void)
{
    int32_t temp;
    temp = target_bearing - original_target_bearing;
    temp = wrap_18000(temp);
    return (ABS(temp) > 10000); 
}

#define DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS 1.113195f
#define TAN_89_99_DEGREES 5729.57795f




static void GPS_distance_cm_bearing(int32_t *currentLat1, int32_t *currentLon1, int32_t *destinationLat2, int32_t *destinationLon2, uint32_t *dist, int32_t *bearing)
{
    float dLat = *destinationLat2 - *currentLat1; 
    float dLon = (float)(*destinationLon2 - *currentLon1) * GPS_scaleLonDown;
    *dist = sqrtf(sq(dLat) + sq(dLon)) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS;

    *bearing = 9000.0f + atan2_approx(-dLat, dLon) * TAN_89_99_DEGREES;      
    if (*bearing < 0)
        *bearing += 36000;
}















static void GPS_calc_velocity(void)
{
    static int16_t speed_old[2] = { 0, 0 };
    static int32_t last_coord[2] = { 0, 0 };
    static uint8_t init = 0;
    
    

    if (init) {
        float tmp = 1.0f / dTnav;
        actual_speed[GPS_X] = (float)(GPS_coord[LON] - last_coord[LON]) * GPS_scaleLonDown * tmp;
        actual_speed[GPS_Y] = (float)(GPS_coord[LAT] - last_coord[LAT]) * tmp;

        actual_speed[GPS_X] = (actual_speed[GPS_X] + speed_old[GPS_X]) / 2;
        actual_speed[GPS_Y] = (actual_speed[GPS_Y] + speed_old[GPS_Y]) / 2;

        speed_old[GPS_X] = actual_speed[GPS_X];
        speed_old[GPS_Y] = actual_speed[GPS_Y];
    }
    init = 1;

    last_coord[LON] = GPS_coord[LON];
    last_coord[LAT] = GPS_coord[LAT];
}










static void GPS_calc_location_error(int32_t *target_lat, int32_t *target_lng, int32_t *gps_lat, int32_t *gps_lng)
{
    error[LON] = (float)(*target_lng - *gps_lng) * GPS_scaleLonDown;   
    error[LAT] = *target_lat - *gps_lat;        
}




static void GPS_calc_poshold(void)
{
    int32_t d;
    int32_t target_speed;
    int axis;

    for (axis = 0; axis < 2; axis++) {
        target_speed = get_P(error[axis], &posholdPID_PARAM);       
        rate_error[axis] = target_speed - actual_speed[axis];       

        nav[axis] = get_P(rate_error[axis], &poshold_ratePID_PARAM) +
                    get_I(rate_error[axis] + error[axis], &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);
        d = get_D(error[axis], &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);
        d = constrain(d, -2000, 2000);

        
#if defined(GPS_LOW_SPEED_D_FILTER)
        if (ABS(actual_speed[axis]) < 50)
            d = 0;
#endif

        nav[axis] += d;
        nav[axis] = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
        navPID[axis].integrator = poshold_ratePID[axis].integrator;
    }
}




static void GPS_calc_nav_rate(uint16_t max_speed)
{
    float trig[2];
    float temp;
    int axis;

    
    GPS_update_crosstrack();

    
    temp = (9000l - nav_bearing) * RADX100;
    trig[GPS_X] = cos_approx(temp);
    trig[GPS_Y] = sin_approx(temp);

    for (axis = 0; axis < 2; axis++) {
        rate_error[axis] = (trig[axis] * max_speed) - actual_speed[axis];
        rate_error[axis] = constrain(rate_error[axis], -1000, 1000);
        
        nav[axis] = get_P(rate_error[axis], &navPID_PARAM) +
                    get_I(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM) +
                    get_D(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM);

        nav[axis] = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
        poshold_ratePID[axis].integrator = navPID[axis].integrator;
    }
}





static void GPS_update_crosstrack(void)
{
    if (ABS(wrap_18000(target_bearing - original_target_bearing)) < 4500) {     
        float temp = (target_bearing - original_target_bearing) * RADX100;
        crosstrack_error = sin_approx(temp) * (wp_distance * CROSSTRACK_GAIN); 
        nav_bearing = target_bearing + constrain(crosstrack_error, -3000, 3000);
        nav_bearing = wrap_36000(nav_bearing);
    } else {
        nav_bearing = target_bearing;
    }
}












static uint16_t GPS_calc_desired_speed(uint16_t max_speed, bool _slow)
{
    
    if (_slow) {
        max_speed = MIN(max_speed, wp_distance / 2);
    } else {
        max_speed = MIN(max_speed, wp_distance);
        max_speed = MAX(max_speed, gpsProfile->nav_speed_min);      
    }

    
    
    if (max_speed > waypoint_speed_gov) {
        waypoint_speed_gov += (int)(100.0f * dTnav);    
        max_speed = waypoint_speed_gov;
    }
    return max_speed;
}




static int32_t wrap_18000(int32_t error)
{
    if (error > 18000)
        error -= 36000;
    if (error < -18000)
        error += 36000;
    return error;
}

static int32_t wrap_36000(int32_t angle)
{
    if (angle > 36000)
        angle -= 36000;
    if (angle < 0)
        angle += 36000;
    return angle;
}

void updateGpsStateForHomeAndHoldMode(void)
{
    float sin_yaw_y = sin_approx(DECIDEGREES_TO_DEGREES(attitude.values.yaw) * 0.0174532925f);
    float cos_yaw_x = cos_approx(DECIDEGREES_TO_DEGREES(attitude.values.yaw) * 0.0174532925f);
    if (gpsProfile->nav_slew_rate) {
        nav_rated[LON] += constrain(wrap_18000(nav[LON] - nav_rated[LON]), -gpsProfile->nav_slew_rate, gpsProfile->nav_slew_rate); 
        nav_rated[LAT] += constrain(wrap_18000(nav[LAT] - nav_rated[LAT]), -gpsProfile->nav_slew_rate, gpsProfile->nav_slew_rate);
        GPS_angle[AI_ROLL] = (nav_rated[LON] * cos_yaw_x - nav_rated[LAT] * sin_yaw_y) / 10;
        GPS_angle[AI_PITCH] = (nav_rated[LON] * sin_yaw_y + nav_rated[LAT] * cos_yaw_x) / 10;
    } else {
        GPS_angle[AI_ROLL] = (nav[LON] * cos_yaw_x - nav[LAT] * sin_yaw_y) / 10;
        GPS_angle[AI_PITCH] = (nav[LON] * sin_yaw_y + nav[LAT] * cos_yaw_x) / 10;
    }
}

void updateGpsWaypointsAndMode(void)
{
    bool resetNavNow = false;
    static bool gpsReadyBeepDone = false;

    if (STATE(GPS_FIX) && GPS_numSat >= 5) {

        
        
        
        

        if (IS_RC_MODE_ACTIVE(BOXGPSHOME)) {
            if (!FLIGHT_MODE(GPS_HOME_MODE)) {

                
                ENABLE_FLIGHT_MODE(GPS_HOME_MODE);
                DISABLE_FLIGHT_MODE(GPS_HOLD_MODE);
                GPS_set_next_wp(&GPS_home[LAT], &GPS_home[LON]);
                nav_mode = NAV_MODE_WP;
                resetNavNow = true;
            }
        } else {
            if (FLIGHT_MODE(GPS_HOME_MODE)) {

                
                DISABLE_FLIGHT_MODE(GPS_HOME_MODE);
                nav_mode = NAV_MODE_NONE;
                resetNavNow = true;
            }

            
            
            

            if (IS_RC_MODE_ACTIVE(BOXGPSHOLD) && areSticksInApModePosition(gpsProfile->ap_mode)) {
                if (!FLIGHT_MODE(GPS_HOLD_MODE)) {

                    
                    ENABLE_FLIGHT_MODE(GPS_HOLD_MODE);
                    GPS_hold[LAT] = GPS_coord[LAT];
                    GPS_hold[LON] = GPS_coord[LON];
                    GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
                    nav_mode = NAV_MODE_POSHOLD;
                    resetNavNow = true;
                }
            } else {
                if (FLIGHT_MODE(GPS_HOLD_MODE)) {

                    
                    DISABLE_FLIGHT_MODE(GPS_HOLD_MODE);
                    nav_mode = NAV_MODE_NONE;
                    resetNavNow = true;
                }
            }
        }
        if (!gpsReadyBeepDone) {            
            beeper(BEEPER_READY_BEEP);      
            gpsReadyBeepDone = true;        
        }
    } else {
        if (FLIGHT_MODE(GPS_HOLD_MODE | GPS_HOME_MODE)) {

            
            DISABLE_FLIGHT_MODE(GPS_HOME_MODE);
            DISABLE_FLIGHT_MODE(GPS_HOLD_MODE);
            nav_mode = NAV_MODE_NONE;
            resetNavNow = true;
        }
    }

    if (resetNavNow) {
        GPS_reset_nav();
    }
}

#endif
