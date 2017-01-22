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
       
#ifndef HOTT_TELEMETRY_H_
#define HOTT_TELEMETRY_H_ 
#define HOTTV4_RXTX 4
#define HOTTV4_TEXT_MODE_REQUEST_ID 0x7f
#define HOTTV4_BINARY_MODE_REQUEST_ID 0x80
#define HOTTV4_BUTTON_DEC 0xEB
#define HOTTV4_BUTTON_INC 0xED
#define HOTTV4_BUTTON_SET 0xE9
#define HOTTV4_BUTTON_NIL 0x0F
#define HOTTV4_BUTTON_NEXT 0xEE
#define HOTTV4_BUTTON_PREV 0xE7
#define HOTT_EAM_OFFSET_HEIGHT 500
#define HOTT_EAM_OFFSET_M2S 72
#define HOTT_EAM_OFFSET_M3S 120
#define HOTT_EAM_OFFSET_TEMPERATURE 20
#define HOTT_GPS_ALTITUDE_OFFSET 500
typedef enum {
    HOTT_EAM_ALARM1_FLAG_NONE = 0,
    HOTT_EAM_ALARM1_FLAG_MAH = (1 << 0),
    HOTT_EAM_ALARM1_FLAG_BATTERY_1 = (1 << 1),
    HOTT_EAM_ALARM1_FLAG_BATTERY_2 = (1 << 2),
    HOTT_EAM_ALARM1_FLAG_TEMPERATURE_1 = (1 << 3),
    HOTT_EAM_ALARM1_FLAG_TEMPERATURE_2 = (1 << 4),
    HOTT_EAM_ALARM1_FLAG_ALTITUDE = (1 << 5),
    HOTT_EAM_ALARM1_FLAG_CURRENT = (1 << 6),
    HOTT_EAM_ALARM1_FLAG_MAIN_VOLTAGE = (1 << 7),
} hottEamAlarm1Flag_e;
typedef enum {
    HOTT_EAM_ALARM2_FLAG_NONE = 0,
    HOTT_EAM_ALARM2_FLAG_MS = (1 << 0),
    HOTT_EAM_ALARM2_FLAG_M3S = (1 << 1),
    HOTT_EAM_ALARM2_FLAG_ALTITUDE_DUPLICATE = (1 << 2),
    HOTT_EAM_ALARM2_FLAG_MS_DUPLICATE = (1 << 3),
    HOTT_EAM_ALARM2_FLAG_M3S_DUPLICATE = (1 << 4),
    HOTT_EAM_ALARM2_FLAG_UNKNOWN_1 = (1 << 5),
    HOTT_EAM_ALARM2_FLAG_UNKNOWN_2 = (1 << 6),
    HOTT_EAM_ALARM2_FLAG_ON_SIGN_OR_TEXT_ACTIVE = (1 << 7),
} hottEamAlarm2Flag_e;
#define HOTT_TEXT_MODE_REQUEST_ID 0x7f
#define HOTT_BINARY_MODE_REQUEST_ID 0x80
#define HOTT_TELEMETRY_NO_SENSOR_ID 0x80
#define HOTT_TELEMETRY_VARIO_SENSOR_ID 0x89
#define HOTT_TELEMETRY_GPS_SENSOR_ID 0x8a
#define HOTT_TELEMETRY_AIRESC_SENSOR_ID 0x8c
#define HOTT_TELEMETRY_GAM_SENSOR_ID 0x8d
#define HOTT_TELEMETRY_EAM_SENSOR_ID 0x8e
#define HOTT_EAM_SENSOR_TEXT_ID 0xE0
#define HOTT_GPS_SENSOR_TEXT_ID 0xA0
#define HOTT_TEXTMODE_MSG_TEXT_LEN 168
struct HOTT_TEXTMODE_MSG {
    uint8_t start_byte;
    uint8_t fill1;
    uint8_t warning_beeps;
    uint8_t msg_txt[HOTT_TEXTMODE_MSG_TEXT_LEN];
    uint8_t stop_byte;
};
typedef struct HOTT_GAM_MSG_s {
    uint8_t start_byte;
    uint8_t gam_sensor_id;
    uint8_t warning_beeps;
    uint8_t sensor_id;
    uint8_t alarm_invers1;
    uint8_t alarm_invers2;
    uint8_t cell1;
    uint8_t cell2;
    uint8_t cell3;
    uint8_t cell4;
    uint8_t cell5;
    uint8_t cell6;
    uint8_t batt1_L;
    uint8_t batt1_H;
    uint8_t batt2_L;
    uint8_t batt2_H;
    uint8_t temperature1;
    uint8_t temperature2;
    uint8_t fuel_procent;
    uint8_t fuel_ml_L;
    uint8_t fuel_ml_H;
    uint8_t rpm_L;
    uint8_t rpm_H;
    uint8_t altitude_L;
    uint8_t altitude_H;
    uint8_t climbrate_L;
    uint8_t climbrate_H;
    uint8_t climbrate3s;
    uint8_t current_L;
    uint8_t current_H;
    uint8_t main_voltage_L;
    uint8_t main_voltage_H;
    uint8_t batt_cap_L;
    uint8_t batt_cap_H;
    uint8_t speed_L;
    uint8_t speed_H;
    uint8_t min_cell_volt;
    uint8_t min_cell_volt_num;
    uint8_t rpm2_L;
    uint8_t rpm2_H;
    uint8_t general_error_number;
    uint8_t pressure;
    uint8_t version;
    uint8_t stop_byte;
} OTT_GAM_MSG_t;
#define HOTT_VARIO_MSG_TEXT_LEN 21
typedef struct HOTT_VARIO_MSG_s {
    uint8_t start_byte;
    uint8_t vario_sensor_id;
    uint8_t warning_beeps;
    uint8_t sensor_id;
    uint8_t alarm_invers1;
    uint8_t altitude_L;
    uint8_t altitude_H;
    uint8_t altitude_max_L;
    uint8_t altitude_max_H;
    uint8_t altitude_min_L;
    uint8_t altitude_min_H;
    uint8_t climbrate_L;
    uint8_t climbrate_H;
    uint8_t climbrate3s_L;
    uint8_t climbrate3s_H;
    uint8_t climbrate10s_L;
    uint8_t climbrate10s_H;
    uint8_t text_msg[HOTT_VARIO_MSG_TEXT_LEN];
    uint8_t free_char1;
    uint8_t free_char2;
    uint8_t free_char3;
    uint8_t compass_direction;
    uint8_t version;
    uint8_t stop_byte;
} HOTT_VARIO_MSG_t;
typedef struct HOTT_EAM_MSG_s {
    uint8_t start_byte;
    uint8_t eam_sensor_id;
    uint8_t warning_beeps;
    uint8_t sensor_id;
    uint8_t alarm_invers1;
    uint8_t alarm_invers2;
    uint8_t cell1_L;
    uint8_t cell2_L;
    uint8_t cell3_L;
    uint8_t cell4_L;
    uint8_t cell5_L;
    uint8_t cell6_L;
    uint8_t cell7_L;
    uint8_t cell1_H;
    uint8_t cell2_H;
    uint8_t cell3_H;
    uint8_t cell4_H;
    uint8_t cell5_H;
    uint8_t cell6_H;
    uint8_t cell7_H;
    uint8_t batt1_voltage_L;
    uint8_t batt1_voltage_H;
    uint8_t batt2_voltage_L;
    uint8_t batt2_voltage_H;
    uint8_t temp1;
    uint8_t temp2;
    uint8_t altitude_L;
    uint8_t altitude_H;
    uint8_t current_L;
    uint8_t current_H;
    uint8_t main_voltage_L;
    uint8_t main_voltage_H;
    uint8_t batt_cap_L;
    uint8_t batt_cap_H;
    uint8_t climbrate_L;
    uint8_t climbrate_H;
    uint8_t climbrate3s;
    uint8_t rpm_L;
    uint8_t rpm_H;
    uint8_t electric_min;
    uint8_t electric_sec;
    uint8_t speed_L;
    uint8_t speed_H;
    uint8_t stop_byte;
} HOTT_EAM_MSG_t;
typedef struct HOTT_GPS_MSG_s {
  uint8_t start_byte;
  uint8_t gps_sensor_id;
  uint8_t warning_beeps;
  uint8_t sensor_id;
  uint8_t alarm_invers1;
  uint8_t alarm_invers2;
  uint8_t flight_direction;
  uint8_t gps_speed_L;
  uint8_t gps_speed_H;
  uint8_t pos_NS;
  uint8_t pos_NS_dm_L;
  uint8_t pos_NS_dm_H;
  uint8_t pos_NS_sec_L;
  uint8_t pos_NS_sec_H;
  uint8_t pos_EW;
  uint8_t pos_EW_dm_L;
  uint8_t pos_EW_dm_H;
  uint8_t pos_EW_sec_L;
  uint8_t pos_EW_sec_H;
  uint8_t home_distance_L;
  uint8_t home_distance_H;
  uint8_t altitude_L;
  uint8_t altitude_H;
  uint8_t climbrate_L;
  uint8_t climbrate_H;
  uint8_t climbrate3s;
  uint8_t gps_satelites;
  uint8_t gps_fix_char;
  uint8_t home_direction;
  uint8_t angle_roll;
  uint8_t angle_nick;
  uint8_t angle_compass;
  uint8_t gps_time_h;
  uint8_t gps_time_m;
  uint8_t gps_time_s;
  uint8_t gps_time_sss;
  uint8_t msl_altitude_L;
  uint8_t msl_altitude_H;
  uint8_t vibration;
  uint8_t free_char1;
  uint8_t free_char2;
  uint8_t free_char3;
  uint8_t version;
  uint8_t stop_byte;
} HOTT_GPS_MSG_t;
typedef struct HOTT_AIRESC_MSG_s {
    uint8_t start_byte;
    uint8_t gps_sensor_id;
    uint8_t warning_beeps;
    uint8_t sensor_id;
    uint8_t alarm_invers1;
    uint8_t alarm_invers2;
    uint8_t input_v_L;
    uint8_t input_v_H;
    uint8_t input_v_min_L;
    uint8_t input_v_min_H;
    uint8_t batt_cap_L;
    uint8_t batt_cap_H;
    uint8_t esc_temp;
    uint8_t esc_max_temp;
    uint8_t current_L;
    uint8_t current_H;
    uint8_t current_max_L;
    uint8_t current_max_H;
    uint8_t rpm_L;
    uint8_t rpm_H;
    uint8_t rpm_max_L;
    uint8_t rpm_max_H;
    uint8_t throttle;
    uint8_t speed_L;
    uint8_t speed_H;
    uint8_t speed_max_L;
    uint8_t speed_max_H;
    uint8_t bec_v;
    uint8_t bec_min_v;
    uint8_t bec_current;
    uint8_t bec_current_max_L;
    uint8_t bec_current_max_H;
    uint8_t pwm;
    uint8_t bec_temp;
    uint8_t bec_temp_max;
    uint8_t motor_temp;
    uint8_t motor_temp_max;
    uint8_t motor_rpm_L;
    uint8_t motor_rpm_H;
    uint8_t motor_timing;
    uint8_t motor_timing_adv;
    uint8_t motor_highest_current;
    uint8_t version;
    uint8_t stop_byte;
} HOTT_AIRESC_MSG_t;
void handleHoTTTelemetry(void);
void checkHoTTTelemetryState(void);
void initHoTTTelemetry(telemetryConfig_t *telemetryConfig);
void configureHoTTTelemetryPort(void);
void freeHoTTTelemetryPort(void);
uint32_t getHoTTTelemetryProviderBaudRate(void);
void hottPrepareGPSResponse(HOTT_GPS_MSG_t *hottGPSMessage);
#endif
